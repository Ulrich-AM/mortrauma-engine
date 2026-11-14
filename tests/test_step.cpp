#include "mortrauma/assets/ModelLoader.hpp"
#include "mortrauma/balance/BalanceController.hpp"
#include "mortrauma/balance/StandingPose.hpp"
#include "mortrauma/balance/StepController.hpp"
#include "mortrauma/core/Paths.hpp"
#include "mortrauma/motors/MotorController.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <cmath>
#include <iostream>
#include <string>

extern int gFailures;
void expect(bool condition, const std::string& message);

namespace {

struct Lab {
    mt::PhysicsWorld world;
    mt::ImportedModel model;
    mt::Ragdoll ragdoll;
    mt::MotorController motors;
    mt::BalanceController balance;
    mt::StepController stepping;
};

bool loadLab(Lab& lab) {
    mt::PhysicsSettings settings;
    settings.allowSleeping = true;
    if (!lab.world.initialize(settings)) {
        return false;
    }
    const auto config = mt::loadCharacterConfig(mt::findAsset("configurations/humanoidguy.json").string());
    lab.model = mt::ModelLoader::load(mt::findAsset("characters/humanoidguy_rigged.fbx").string());
    std::string error;
    const auto preset = mt::loadJointPreset(mt::findAsset("configurations/presets/natural.json").string(), error);
    if (!lab.model.report.ok || !config.error.empty() || !error.empty()) {
        return false;
    }
    lab.ragdoll = mt::RagdollBuilder::build(lab.world, config, lab.model, preset);
    if (!lab.ragdoll.ok || !lab.motors.bind(lab.world, lab.ragdoll) || !lab.balance.loadPreset("normal", error)) {
        return false;
    }
    lab.motors.loadDefaultProfiles();
    mt::BodyCreateInfo ground;
    ground.name = "Floor";
    ground.isStatic = true;
    ground.shape = mt::BodyCreateInfo::Shape::Box;
    ground.halfExtents = {8.f, 0.1f, 8.f};
    ground.position = {0.f, -0.1f, 0.f};
    ground.friction = 0.9f;
    return lab.world.createBody(ground) >= 0;
}

bool place(Lab& lab) {
    const auto placement = mt::placeStandingPose(lab.world, lab.ragdoll, lab.balance.config());
    if (!placement.ok) {
        return false;
    }
    lab.balance.setReference(placement.reference);
    lab.balance.resetState();
    lab.balance.setEnabled(true, lab.motors);
    return true;
}

void tick(Lab& lab) {
    const float dt = lab.world.settings().fixedTimestep;
    lab.balance.observe(lab.world, lab.ragdoll, dt);
    lab.stepping.update(dt, lab.world, lab.balance.state(), lab.ragdoll, lab.balance);
    lab.balance.command(lab.ragdoll, lab.motors);
    lab.motors.apply(lab.world, lab.ragdoll);
    lab.world.step(dt, 1);
}

void settle(Lab& lab, int steps) {
    for (int i = 0; i < steps; ++i) {
        tick(lab);
    }
}

float soleY(const Lab& lab, const char* id) {
    const auto* foot = lab.ragdoll.find(id);
    const auto state = lab.world.bodyState(foot->physics);
    const mt::Vec3 heel = state.position + mt::rotate(state.rotation, foot->heelLocal);
    const mt::Vec3 toe = state.position + mt::rotate(state.rotation, foot->forefootLocal);
    return std::min(heel.y, toe.y);
}

float lateral(const mt::BalanceState& state) {
    return mt::dot(mt::Vec3{state.com.x - state.equilibriumTarget.x, 0.f, state.com.z - state.equilibriumTarget.z}, state.right);
}

float biasOffset(const Lab& lab) {
    const auto& state = lab.balance.state();
    return mt::length(mt::Vec3{state.balanceReference.x - state.equilibriumTarget.x, 0.f, state.balanceReference.z - state.equilibriumTarget.z});
}

bool allDynamic(const Lab& lab) {
    for (const auto& body : lab.ragdoll.bodies) {
        const auto sample = lab.world.bodyState(body.physics);
        if (!sample.isDynamic || sample.isKinematic) {
            return false;
        }
    }
    return true;
}

bool withinLimits(const Lab& lab) {
    const float slack = 0.08f;
    for (const auto& joint : lab.ragdoll.joints) {
        const auto sample = lab.world.jointState(joint.physics);
        if (!sample.valid) {
            continue;
        }
        if (sample.rotationRadians.x < sample.twistMin - slack || sample.rotationRadians.x > sample.twistMax + slack ||
            sample.rotationRadians.y < sample.swing1Min - slack || sample.rotationRadians.y > sample.swing1Max + slack ||
            sample.rotationRadians.z < sample.swing2Min - slack || sample.rotationRadians.z > sample.swing2Max + slack) {
            return false;
        }
    }
    return true;
}

float torqueOver(const Lab& lab) {
    float over = 0.f;
    for (const auto& joint : lab.ragdoll.joints) {
        const auto sample = lab.world.jointState(joint.physics);
        if (!sample.valid || sample.motorTorqueLimit <= 0.f) {
            continue;
        }
        const float peak = std::max(std::fabs(sample.motorTorque.x), std::max(std::fabs(sample.motorTorque.y), std::fabs(sample.motorTorque.z)));
        over = std::max(over, peak - sample.motorTorqueLimit);
    }
    return over;
}

void test001() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-001 placed");
    settle(lab, 40);
    lab.stepping.requestTransfer(true);
    for (int i = 0; i < 30; ++i) {
        tick(lab);
    }
    const float rightCommand = lab.stepping.hipSwing1Command();
    const float rightError = lateral(lab.balance.state()) - mt::dot(lab.stepping.comTarget(), lab.balance.state().right);
    lab.stepping.cancel();
    settle(lab, 20);
    lab.stepping.requestTransfer(false);
    for (int i = 0; i < 30; ++i) {
        tick(lab);
    }
    const float leftCommand = lab.stepping.hipSwing1Command();
    std::cout << "hip_1 swing1 right-transfer " << rightCommand << " left-transfer " << leftCommand << " rightError " << rightError << '\n';
    // hip_1 keeps the correction sign. hip_2 is the negation, checked by the opposite requests.
    expect(rightCommand > 0.02f && leftCommand < -0.02f, "STEP-001 mirrored hip swing1 points toward the requested foot");
    expect(rightCommand * leftCommand < 0.f, "STEP-001 left and right requests use opposite hip_1 signs");
    expect(!lab.world.supportActive(), "STEP-001 no support fixture");
}

void test002() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-002 placed");
    settle(lab, 40);
    const float start = lateral(lab.balance.state());
    lab.stepping.setTransferDuration(4.f);
    lab.stepping.requestTransfer(false);
    float minY = 10.f;
    float toward = start;
    for (int i = 0; i < 480; ++i) {
        tick(lab);
        minY = std::min(minY, lab.balance.state().com.y);
        toward = std::min(toward, lateral(lab.balance.state()));
    }
    std::cout << "left com lateral " << start << " -> " << toward << " end " << lateral(lab.balance.state()) << " y " << minY << " phase "
              << mt::stepPhaseName(lab.stepping.phase()) << " " << lab.stepping.note() << '\n';
    expect(toward < start - 0.008f && minY > 0.90f, "STEP-002 the COM moves toward the left foot while upright");
    expect(!lab.world.supportActive() && allDynamic(lab), "STEP-002 feet stay dynamic");
}

void test003() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-003 placed");
    settle(lab, 40);
    const float start = lateral(lab.balance.state());
    lab.stepping.setTransferDuration(4.f);
    lab.stepping.requestTransfer(true);
    float minY = 10.f;
    float toward = start;
    for (int i = 0; i < 480; ++i) {
        tick(lab);
        minY = std::min(minY, lab.balance.state().com.y);
        toward = std::max(toward, lateral(lab.balance.state()));
    }
    std::cout << "right com lateral " << start << " -> " << toward << " end " << lateral(lab.balance.state()) << " y " << minY << " phase "
              << mt::stepPhaseName(lab.stepping.phase()) << " " << lab.stepping.note() << '\n';
    expect(toward > start + 0.008f && minY > 0.90f, "STEP-003 the COM moves toward the right foot while upright");
    expect(!lab.world.supportActive() && allDynamic(lab), "STEP-003 feet stay dynamic");
}

void test004() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-004 placed");
    settle(lab, 20);
    lab.stepping.requestTransfer(true);
    tick(lab);
    const bool right = lab.stepping.supportRight();
    const mt::Vec3 rightTarget = lab.stepping.comTarget();
    lab.stepping.cancel();
    lab.stepping.requestTransfer(false);
    tick(lab);
    const bool left = !lab.stepping.supportRight();
    const auto& state = lab.balance.state();
    const float rightLat = mt::dot(rightTarget, state.right);
    const float leftLat = mt::dot(lab.stepping.comTarget(), state.right);
    std::cout << "support targets lateral " << rightLat << " " << leftLat << '\n';
    expect(right && left && rightLat > leftLat, "STEP-004 the selected support foot sets the COM target side");
}

void test005() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-005 placed");
    settle(lab, 30);
    const int before = lab.balance.state().leftPoints;
    lab.stepping.requestSupport(true);
    bool lifted = false;
    int minSwing = before;
    for (int i = 0; i < 500; ++i) {
        tick(lab);
        minSwing = std::min(minSwing, lab.balance.state().leftPoints);
        if (lab.stepping.phase() == mt::StepPhase::LiftFoot) {
            lifted = true;
        }
        if (!lab.stepping.active() && i > 10) {
            break;
        }
    }
    std::cout << "swing points " << before << " -> " << minSwing << " estimate " << lab.stepping.supportEstimate() << " note " << lab.stepping.note() << '\n';
    expect(!lifted, "STEP-005 single-leg mode does not lift the foot");
    expect(lab.stepping.supportEstimate().find("not loads") != std::string::npos || lab.stepping.supportEstimate().find("contact") != std::string::npos,
           "STEP-005 support is labeled as a contact estimate");
    expect(lab.balance.state().leftLoadFraction < 0.f, "STEP-005 no invented load fraction");
}

void test006() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-006 placed");
    settle(lab, 20);
    lab.stepping.requestSupport(false);
    bool sawTransfer = false;
    bool sawLift = false;
    mt::StepPhase previous = mt::StepPhase::Idle;
    for (int i = 0; i < 500; ++i) {
        tick(lab);
        const auto phase = lab.stepping.phase();
        if (phase == mt::StepPhase::TransferWeight) {
            sawTransfer = true;
        }
        if (phase == mt::StepPhase::LiftFoot) {
            sawLift = true;
        }
        previous = phase;
        if (!lab.stepping.active() && i > 10) {
            break;
        }
    }
    std::cout << "end " << mt::stepPhaseName(previous) << " " << lab.stepping.note() << '\n';
    expect(sawTransfer && !sawLift, "STEP-006 single-leg support transfers weight and does not start a lift");
    expect(previous == mt::StepPhase::Completed || previous == mt::StepPhase::Aborted || previous == mt::StepPhase::Falling,
           "STEP-006 the attempt ends in a terminal state");
}

void test007() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-007 placed");
    settle(lab, 20);
    lab.stepping.setLiftHeight(0.04f);
    lab.stepping.requestLift(true);
    tick(lab);
    const mt::Vec3 origin = lab.stepping.swingOrigin();
    const mt::Vec3 target = lab.stepping.swingTarget();
    const float rise = target.y - origin.y;
    float jump = 0.f;
    mt::Vec3 previous = lab.world.bodyState(lab.ragdoll.find("foot_2")->physics).position;
    for (int i = 0; i < 40; ++i) {
        tick(lab);
        const mt::Vec3 sample = lab.world.bodyState(lab.ragdoll.find("foot_2")->physics).position;
        jump = std::max(jump, mt::length(sample - previous));
        previous = sample;
    }
    std::cout << "planned rise " << rise << " max foot step " << jump << '\n';
    expect(rise > 0.03f && rise < 0.06f, "STEP-007 the swing target is a few centimetres above the grounded foot");
    expect(jump < 0.04f, "STEP-007 the foot is not teleported toward that target");
}

void test008() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-008 placed");
    settle(lab, 20);
    lab.stepping.setLiftHeight(0.05f);
    lab.stepping.requestSequence(true);
    bool inside = true;
    for (int i = 0; i < 600; ++i) {
        tick(lab);
        inside = inside && withinLimits(lab);
        if (!lab.stepping.active() && i > 30) {
            break;
        }
    }
    expect(inside, "STEP-008 joint angles stay inside the anatomical limits");
}

void test009() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-009 placed");
    settle(lab, 20);
    lab.stepping.requestSequence(false);
    float over = 0.f;
    for (int i = 0; i < 600; ++i) {
        tick(lab);
        over = std::max(over, torqueOver(lab));
        if (!lab.stepping.active() && i > 30) {
            break;
        }
    }
    std::cout << "torque over limit " << over << '\n';
    expect(over < 0.75f, "STEP-009 motor torque stays at the configured limit");
}

void test010() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-010 placed");
    settle(lab, 40);
    const float before = soleY(lab, "foot_2");
    lab.stepping.setLiftHeight(0.03f);
    lab.stepping.requestSequence(true);
    float upright = before;
    float fallen = before;
    float minY = 10.f;
    for (int i = 0; i < 1400; ++i) {
        tick(lab);
        const float sole = soleY(lab, "foot_2");
        fallen = std::max(fallen, sole);
        if (lab.balance.state().com.y > 0.90f && lab.balance.state().torsoTilt < 0.45f) {
            upright = std::max(upright, sole);
        }
        minY = std::min(minY, lab.balance.state().com.y);
        if (!lab.stepping.active() && i > 40) {
            break;
        }
    }
    const float measured = upright - before;
    std::cout << "upright clearance " << measured << " controller " << lab.stepping.peakUprightClearance() << " fallen rise " << fallen - before << " minY " << minY
              << " note " << lab.stepping.note() << '\n';
    expect(std::fabs(lab.stepping.peakUprightClearance() - measured) < 0.015f, "STEP-010 clearance is the upright sole rise, not the fall");
    expect(lab.balance.state().finite && !lab.world.supportActive(), "STEP-010 the attempt stays a physical simulation");
}

void test011() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-011 placed");
    settle(lab, 30);
    lab.stepping.requestSequence(true);
    bool sawLower = false;
    bool touchedWhileLowering = false;
    for (int i = 0; i < 1200; ++i) {
        tick(lab);
        if (lab.stepping.phase() == mt::StepPhase::LowerFoot) {
            sawLower = true;
            if (lab.balance.state().leftTouching) {
                touchedWhileLowering = true;
            }
        }
        if (!lab.stepping.active() && i > 40) {
            break;
        }
    }
    for (int i = 0; i < 200; ++i) {
        tick(lab);
    }
    const bool down = lab.balance.state().leftTouching;
    std::cout << "lower " << sawLower << " touch " << touchedWhileLowering << " down " << down << " clearance " << lab.stepping.peakUprightClearance() << " "
              << lab.stepping.note() << '\n';
    if (sawLower) {
        expect(touchedWhileLowering && down, "STEP-011 lowering detects the swing foot touching the floor");
    } else {
        expect(down && lab.stepping.peakUprightClearance() < 0.02f, "STEP-011 a refused lift leaves the foot on the ground");
    }
}

void test012() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-012 placed");
    settle(lab, 30);
    lab.stepping.requestSequence(false);
    for (int i = 0; i < 1200; ++i) {
        tick(lab);
        if (!lab.stepping.active() && i > 40) {
            break;
        }
    }
    for (int i = 0; i < 240; ++i) {
        tick(lab);
    }
    const auto phase = lab.stepping.phase();
    const bool both = lab.balance.state().leftTouching && lab.balance.state().rightTouching;
    std::cout << "restore " << mt::stepPhaseName(phase) << " both " << both << " bias " << biasOffset(lab) << " " << lab.stepping.note() << '\n';
    expect(phase == mt::StepPhase::Completed || phase == mt::StepPhase::Aborted, "STEP-012 the sequence returns the supervisor to a finished state");
    if (phase == mt::StepPhase::Completed) {
        expect(both && biasOffset(lab) < 0.015f, "STEP-012 completion restores two-foot contact and clears the bias");
    } else {
        expect(biasOffset(lab) < 0.02f, "STEP-012 an aborted attempt does not leave a balance offset");
    }
}

void test013() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-013 placed");
    settle(lab, 20);
    lab.stepping.requestTransfer(true);
    settle(lab, 15);
    lab.stepping.abort();
    expect(lab.stepping.phase() == mt::StepPhase::Aborted, "STEP-013 abort leaves the transfer");
    for (int i = 0; i < 180; ++i) {
        tick(lab);
    }
    const float afterTransfer = biasOffset(lab);
    lab.stepping.requestSequence(true);
    settle(lab, 40);
    lab.stepping.abort();
    for (int i = 0; i < 180; ++i) {
        tick(lab);
    }
    lab.stepping.abort();
    std::cout << "bias after abort " << afterTransfer << " then " << biasOffset(lab) << " phase " << mt::stepPhaseName(lab.stepping.phase()) << '\n';
    expect(afterTransfer < 0.02f && biasOffset(lab) < 0.02f, "STEP-013 abort clears the motor bias");
    expect(lab.stepping.phase() == mt::StepPhase::Aborted, "STEP-013 a second abort does not start a new attempt");
}

void test014() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-014 placed");
    settle(lab, 15);
    lab.stepping.requestLift(true);
    settle(lab, 10);
    expect(lab.motors.motorTaskActive(), "STEP-014 balance still owns the motor task");
    lab.stepping.cancel();
    tick(lab);
    std::cout << "bias " << biasOffset(lab) << " phase " << mt::stepPhaseName(lab.stepping.phase()) << '\n';
    expect(lab.stepping.phase() == mt::StepPhase::Idle && biasOffset(lab) < 0.01f, "STEP-014 cancel clears the step task");
    expect(lab.motors.motorTaskActive() && !lab.world.supportActive() && allDynamic(lab), "STEP-014 motors stay with the balance controller");
}

void test015() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-015 placed");
    float maxDrift = 0.f;
    for (int i = 0; i < 3600; ++i) {
        tick(lab);
        const auto& state = lab.balance.state();
        maxDrift = std::max(maxDrift, state.comToEquilibrium);
    }
    const auto& state = lab.balance.state();
    std::cout << "stand phase " << mt::balancePhaseName(state.phase) << " y " << state.com.y << " tilt " << state.torsoTilt << " drift " << maxDrift << '\n';
    expect(state.phase == mt::BalancePhase::Standing && state.com.y > 0.95f && state.torsoTilt < 0.35f, "STEP-015 quiet standing still holds for 30 seconds");
    expect(lab.stepping.phase() == mt::StepPhase::Idle && !lab.world.supportActive(), "STEP-015 standing does not start a step or a fixture");
}

void test016() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-016 placed");
    settle(lab, 30);
    lab.stepping.requestRecovery(true, {0.f, 0.f, 1.f});
    const auto* chest = lab.ragdoll.find("chest");
    const auto chestState = lab.world.bodyState(chest->physics);
    lab.world.addImpulse(chest->physics, {0.f, 0.f, 55.f}, chestState.position);
    bool fell = false;
    bool finite = true;
    for (int i = 0; i < 360; ++i) {
        tick(lab);
        const auto& state = lab.balance.state();
        finite = finite && state.finite;
        if (state.phase == mt::BalancePhase::Falling || state.com.y < 0.7f || state.torsoTilt > 0.7f || lab.stepping.phase() == mt::StepPhase::Falling) {
            fell = true;
            break;
        }
    }
    std::cout << "fell " << fell << " comY " << lab.balance.state().com.y << '\n';
    expect(finite && fell && !lab.world.supportActive(), "STEP-016 a 55 N·s push falls without a hidden support");
}

void test017() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-017 placed");
    settle(lab, 20);
    const auto* chest = lab.ragdoll.find("chest");
    const mt::Vec3 start = lab.world.bodyState(chest->physics).position;
    const int grab = lab.world.beginGrab(chest->physics, start, start + mt::Vec3{0.12f, 0.05f, 0.f}, 8.f, 0.6f);
    for (int i = 0; i < 40; ++i) {
        lab.world.updateGrab(grab, start + mt::Vec3{0.12f, 0.05f, 0.f});
        tick(lab);
    }
    const mt::Vec3 pulled = lab.world.bodyState(chest->physics).position;
    lab.world.endGrab(grab);
    lab.world.addImpulse(chest->physics, {8.f, 0.f, 0.f}, pulled);
    tick(lab);
    const mt::Vec3 pushed = lab.world.bodyState(chest->physics).position;
    std::cout << "grab move " << mt::length(pulled - start) << " push move " << mt::length(pushed - pulled) << '\n';
    expect(lab.world.grabAlive(grab) == false && mt::length(pulled - start) > 0.01f, "STEP-017 a grab still moves the chest");
    expect(mt::length(pushed - pulled) > 0.f && !lab.world.supportActive(), "STEP-017 a push still applies and no fixture is created");
}

void test018() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-018 placed");
    settle(lab, 15);
    lab.stepping.requestSequence(true);
    bool clean = true;
    for (int i = 0; i < 700; ++i) {
        tick(lab);
        clean = clean && !lab.world.supportActive() && allDynamic(lab);
        if (!lab.stepping.active() && i > 30) {
            break;
        }
    }
    expect(clean, "STEP-018 the sequence adds no kinematic support");
}

}  // namespace

void runAllStepTests() {
    test001();
    test002();
    test003();
    test004();
    test005();
    test006();
    test007();
    test008();
    test009();
    test010();
    test011();
    test012();
    test013();
    test014();
    test015();
    test016();
    test017();
    test018();
}

void runStepTest(const std::string& which) {
    if (which == "STEP-001") test001();
    else if (which == "STEP-002") test002();
    else if (which == "STEP-003") test003();
    else if (which == "STEP-004") test004();
    else if (which == "STEP-005") test005();
    else if (which == "STEP-006") test006();
    else if (which == "STEP-007") test007();
    else if (which == "STEP-008") test008();
    else if (which == "STEP-009") test009();
    else if (which == "STEP-010") test010();
    else if (which == "STEP-011") test011();
    else if (which == "STEP-012") test012();
    else if (which == "STEP-013") test013();
    else if (which == "STEP-014") test014();
    else if (which == "STEP-015") test015();
    else if (which == "STEP-016") test016();
    else if (which == "STEP-017") test017();
    else if (which == "STEP-018") test018();
    else expect(false, which + " is not a stepping test");
}
