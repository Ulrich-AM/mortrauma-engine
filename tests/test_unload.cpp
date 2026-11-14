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

bool runUntilIdle(Lab& lab, int limit) {
    for (int i = 0; i < limit; ++i) {
        tick(lab);
        if (!lab.stepping.active() && i > 30) {
            return true;
        }
    }
    return !lab.stepping.active();
}

void test001() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-001 placed");
    settle(lab, 20);
    lab.stepping.requestUnload(true);
    tick(lab);
    const bool right = lab.stepping.supportRight();
    lab.stepping.cancel();
    lab.stepping.requestUnload(false);
    tick(lab);
    expect(right && !lab.stepping.supportRight(), "UNLOAD-001 the requested foot is the support foot");
}

void test002() {
    for (const bool supportRight : {false, true}) {
        Lab lab;
        expect(loadLab(lab) && place(lab), "UNLOAD-002 placed");
        settle(lab, 40);
        const float start = lateral(lab.balance.state());
        lab.stepping.setTransferDuration(4.f);
        lab.stepping.requestTransfer(supportRight);
        float toward = start;
        float minY = 10.f;
        for (int i = 0; i < 480; ++i) {
            tick(lab);
            minY = std::min(minY, lab.balance.state().com.y);
            const float sample = lateral(lab.balance.state());
            toward = supportRight ? std::max(toward, sample) : std::min(toward, sample);
        }
        const float moved = supportRight ? toward - start : start - toward;
        std::cout << (supportRight ? "right " : "left ") << moved << " m\n";
        expect(moved > 0.008f && minY > 0.90f, "UNLOAD-002 the COM moves toward the selected foot");
    }
}

void test003() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-003 placed");
    settle(lab, 30);
    lab.stepping.requestTransfer(true);
    settle(lab, 25);
    const float right = lab.stepping.hipSwing1Command();
    lab.stepping.cancel();
    settle(lab, 15);
    lab.stepping.requestTransfer(false);
    settle(lab, 25);
    const float left = lab.stepping.hipSwing1Command();
    std::cout << "hip swing " << right << " " << left << '\n';
    expect(right > 0.02f && left < -0.02f, "UNLOAD-003 hip 2 is mirrored against hip 1");
}

void test004() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-004 placed");
    settle(lab, 30);
    lab.stepping.requestUnload(true);
    bool sawUnload = false;
    bool stanceHeld = true;
    for (int i = 0; i < 900; ++i) {
        tick(lab);
        if (lab.stepping.phase() == mt::StepPhase::BeginUnloading) {
            sawUnload = true;
            stanceHeld = stanceHeld && lab.balance.state().rightGrounded;
        }
        if (!lab.stepping.active() && i > 40) {
            break;
        }
    }
    expect(sawUnload && stanceHeld, "UNLOAD-004 unloading starts only while the stance foot is touching");
}

void test005() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-005 placed");
    settle(lab, 30);
    lab.stepping.requestUnload(true);
    bool began = false;
    for (int i = 0; i < 900; ++i) {
        tick(lab);
        if (lab.stepping.phase() == mt::StepPhase::BeginUnloading) {
            began = true;
            break;
        }
        if (!lab.stepping.active() && i > 40) {
            break;
        }
    }
    std::cout << "note " << lab.stepping.note() << '\n';
    expect(began, "UNLOAD-005 the unloading command starts after the weight shift");
}

void test006() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-006 placed");
    settle(lab, 30);
    lab.stepping.requestLift(true);
    bool commandedWhileTouching = false;
    int points = -1;
    for (int i = 0; i < 900; ++i) {
        tick(lab);
        if (lab.stepping.phase() == mt::StepPhase::BeginUnloading) {
            points = lab.balance.state().leftPoints;
            commandedWhileTouching = points > 0;
            break;
        }
        if (!lab.stepping.active() && i > 40) {
            break;
        }
    }
    std::cout << "swing points at unload " << points << '\n';
    expect(commandedWhileTouching, "UNLOAD-006 unloading is commanded before the contact count reaches zero");
}

void test007() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-007 placed");
    settle(lab, 40);
    lab.stepping.setLiftHeight(0.01f);
    lab.stepping.requestLift(true);
    float minY = 10.f;
    for (int i = 0; i < 1400; ++i) {
        tick(lab);
        minY = std::min(minY, lab.balance.state().com.y);
        if (!lab.stepping.active() && i > 40) {
            break;
        }
    }
    const bool lifted = lab.stepping.phase() == mt::StepPhase::LiftFoot || lab.stepping.peakUprightClearance() > 0.01f;
    std::cout << "clearance " << lab.stepping.peakUprightClearance() << " minY " << minY << " " << lab.stepping.note() << '\n';
    expect(minY > 0.90f && !lifted, "UNLOAD-007 a rise that loses the stand is not counted as a lift");
    expect(lab.stepping.peakUprightClearance() < 0.01f, "UNLOAD-007 the upright sole rise stays below 1 cm in this build");
}

void test008() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-008 placed");
    settle(lab, 20);
    lab.stepping.requestSupport(true);
    settle(lab, 80);
    expect(!lab.stepping.loadMeasured(), "UNLOAD-008 foot load is not reported as a measurement");
    expect(lab.balance.state().leftLoadFraction < 0.f && lab.balance.state().rightLoadFraction < 0.f, "UNLOAD-008 load fractions stay unmeasured");
    expect(lab.stepping.supportEstimate().find("not available") != std::string::npos, "UNLOAD-008 the missing impulse is labeled");
}

void test009() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-009 placed");
    settle(lab, 20);
    lab.stepping.requestUnload(true);
    float over = 0.f;
    for (int i = 0; i < 900; ++i) {
        tick(lab);
        for (const auto& joint : lab.ragdoll.joints) {
            const auto sample = lab.world.jointState(joint.physics);
            if (!sample.valid || sample.motorTorqueLimit <= 0.f) {
                continue;
            }
            const float peak = std::max(std::fabs(sample.motorTorque.x), std::max(std::fabs(sample.motorTorque.y), std::fabs(sample.motorTorque.z)));
            over = std::max(over, peak - sample.motorTorqueLimit);
        }
        if (!lab.stepping.active() && i > 40) {
            break;
        }
    }
    expect(over < 0.75f, "UNLOAD-009 torque stays inside the configured limit");
}

void test010() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-010 placed");
    settle(lab, 20);
    lab.stepping.requestLift(false);
    bool inside = true;
    const float slack = 0.08f;
    for (int i = 0; i < 1000; ++i) {
        tick(lab);
        for (const auto& joint : lab.ragdoll.joints) {
            const auto sample = lab.world.jointState(joint.physics);
            if (!sample.valid) {
                continue;
            }
            inside = inside && sample.rotationRadians.x >= sample.twistMin - slack && sample.rotationRadians.x <= sample.twistMax + slack &&
                     sample.rotationRadians.y >= sample.swing1Min - slack && sample.rotationRadians.y <= sample.swing1Max + slack &&
                     sample.rotationRadians.z >= sample.swing2Min - slack && sample.rotationRadians.z <= sample.swing2Max + slack;
        }
        if (!lab.stepping.active() && i > 40) {
            break;
        }
    }
    expect(inside, "UNLOAD-010 joint angles stay inside the anatomical limits");
}

void test011() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-011 placed");
    settle(lab, 30);
    lab.stepping.requestUnload(true);
    bool finite = true;
    bool slippedIntoLift = false;
    for (int i = 0; i < 900; ++i) {
        tick(lab);
        finite = finite && std::isfinite(lab.stepping.stanceSlip());
        if (lab.stepping.stanceSlip() > 0.2f && lab.stepping.phase() == mt::StepPhase::LiftFoot) {
            slippedIntoLift = true;
        }
        if (!lab.stepping.active() && i > 40) {
            break;
        }
    }
    expect(finite && !slippedIntoLift, "UNLOAD-011 a slipping stance foot does not enter the lift");
}

void test012() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-012 placed");
    settle(lab, 30);
    const float before = 0.f;
    (void)before;
    lab.stepping.requestUnload(false);
    float minY = 10.f;
    for (int i = 0; i < 1000; ++i) {
        tick(lab);
        if (lab.balance.state().com.y > 0.90f) {
            minY = std::min(minY, lab.balance.state().com.y);
        }
        if (!lab.stepping.active() && i > 40) {
            break;
        }
    }
    std::cout << "peak upright " << lab.stepping.peakUprightClearance() << " " << lab.stepping.note() << '\n';
    expect(std::isfinite(lab.stepping.peakUprightClearance()) && lab.stepping.peakUprightClearance() < 0.05f, "UNLOAD-012 clearance is a measured sole rise, not a fall");
    expect(minY > 0.90f, "UNLOAD-012 the sample used for clearance was still upright");
}

void test013() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-013 placed");
    settle(lab, 20);
    lab.stepping.requestLift(true);
    bool lowered = false;
    for (int i = 0; i < 1200; ++i) {
        tick(lab);
        if (lab.stepping.phase() == mt::StepPhase::LowerFoot || lab.stepping.phase() == mt::StepPhase::RestoreSupport) {
            lowered = true;
        }
        if (!lab.stepping.active() && i > 40) {
            break;
        }
    }
    for (int i = 0; i < 200; ++i) {
        tick(lab);
    }
    const bool down = lab.balance.state().leftTouching && lab.balance.state().rightTouching;
    std::cout << "lowered " << lowered << " down " << down << " " << lab.stepping.note() << '\n';
    expect(down, "UNLOAD-013 the swing foot is on the ground at the end");
    if (lab.stepping.peakUprightClearance() > 0.008f) {
        expect(lowered, "UNLOAD-013 a foot that rose is lowered before the attempt ends");
    }
}

void test014() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-014 placed");
    settle(lab, 20);
    lab.stepping.requestUnload(true);
    runUntilIdle(lab, 1200);
    for (int i = 0; i < 200; ++i) {
        tick(lab);
    }
    const auto phase = lab.stepping.phase();
    const bool both = lab.balance.state().leftTouching && lab.balance.state().rightTouching;
    std::cout << mt::stepPhaseName(phase) << " bias " << biasOffset(lab) << " " << lab.stepping.note() << '\n';
    expect((phase == mt::StepPhase::Completed || phase == mt::StepPhase::Aborted) && both && biasOffset(lab) < 0.02f,
           "UNLOAD-014 the attempt returns to two-foot contact and clears the bias");
}

void test015() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-015 placed");
    settle(lab, 20);
    lab.stepping.requestUnload(true);
    settle(lab, 30);
    expect(lab.motors.motorTaskActive(), "UNLOAD-015 balance owns the motor task during the attempt");
    lab.stepping.abort();
    for (int i = 0; i < 180; ++i) {
        tick(lab);
    }
    expect(lab.stepping.phase() == mt::StepPhase::Aborted && biasOffset(lab) < 0.02f, "UNLOAD-015 abort clears the stepping bias");
    expect(lab.motors.motorTaskActive() && allDynamic(lab), "UNLOAD-015 motors stay with the balance controller");
}

void test016() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-016 placed");
    for (int i = 0; i < 3600; ++i) {
        tick(lab);
    }
    const auto& state = lab.balance.state();
    std::cout << "stand y " << state.com.y << " tilt " << state.torsoTilt << '\n';
    expect(state.phase == mt::BalancePhase::Standing && state.com.y > 0.95f && state.torsoTilt < 0.35f, "UNLOAD-016 quiet standing still holds for 30 seconds");
    expect(lab.stepping.phase() == mt::StepPhase::Idle && !lab.world.supportActive(), "UNLOAD-016 standing does not start a step");
}

void test017() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-017 placed");
    settle(lab, 30);
    lab.stepping.requestUnload(true);
    const auto* chest = lab.ragdoll.find("chest");
    const auto chestState = lab.world.bodyState(chest->physics);
    lab.world.addImpulse(chest->physics, {0.f, 0.f, 55.f}, chestState.position);
    bool fell = false;
    for (int i = 0; i < 360; ++i) {
        tick(lab);
        const auto& state = lab.balance.state();
        if (state.phase == mt::BalancePhase::Falling || state.com.y < 0.7f || state.torsoTilt > 0.7f || lab.stepping.phase() == mt::StepPhase::Falling) {
            fell = true;
            break;
        }
    }
    expect(fell && !lab.world.supportActive(), "UNLOAD-017 a 55 N·s push falls without a hidden support");
}

void test018() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-018 placed");
    settle(lab, 20);
    const auto* chest = lab.ragdoll.find("chest");
    const mt::Vec3 start = lab.world.bodyState(chest->physics).position;
    const int grab = lab.world.beginGrab(chest->physics, start, start + mt::Vec3{0.12f, 0.05f, 0.f}, 8.f, 0.6f);
    for (int i = 0; i < 40; ++i) {
        lab.world.updateGrab(grab, start + mt::Vec3{0.12f, 0.05f, 0.f});
        tick(lab);
    }
    const float moved = mt::length(lab.world.bodyState(chest->physics).position - start);
    lab.world.endGrab(grab);
    lab.world.addImpulse(chest->physics, {8.f, 0.f, 0.f}, lab.world.bodyState(chest->physics).position);
    tick(lab);
    expect(moved > 0.01f && !lab.world.supportActive(), "UNLOAD-018 grab and push still move the body");
}

void test019() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "UNLOAD-019 placed");
    settle(lab, 15);
    lab.stepping.requestLift(true);
    bool clean = true;
    for (int i = 0; i < 1000; ++i) {
        tick(lab);
        clean = clean && !lab.world.supportActive() && allDynamic(lab);
        if (!lab.stepping.active() && i > 30) {
            break;
        }
    }
    expect(clean, "UNLOAD-019 unloading adds no kinematic support");
}

void test020() {
    mt::StepPhase first = mt::StepPhase::Idle;
    mt::StepPhase second = mt::StepPhase::Falling;
    for (int run = 0; run < 2; ++run) {
        Lab lab;
        expect(loadLab(lab) && place(lab), "UNLOAD-020 placed");
        settle(lab, 30);
        lab.stepping.requestUnload(true);
        runUntilIdle(lab, 1200);
        if (run == 0) {
            first = lab.stepping.phase();
        } else {
            second = lab.stepping.phase();
        }
        expect(lab.balance.state().finite, "UNLOAD-020 the state stays finite");
    }
    std::cout << "repeat " << mt::stepPhaseName(first) << " " << mt::stepPhaseName(second) << '\n';
    expect(first == second && (first == mt::StepPhase::Completed || first == mt::StepPhase::Aborted), "UNLOAD-020 two identical runs end in the same terminal state");
}

}  // namespace

void runAllUnloadTests() {
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
    test019();
    test020();
}

void runUnloadTest(const std::string& which) {
    if (which == "UNLOAD-001") test001();
    else if (which == "UNLOAD-002") test002();
    else if (which == "UNLOAD-003") test003();
    else if (which == "UNLOAD-004") test004();
    else if (which == "UNLOAD-005") test005();
    else if (which == "UNLOAD-006") test006();
    else if (which == "UNLOAD-007") test007();
    else if (which == "UNLOAD-008") test008();
    else if (which == "UNLOAD-009") test009();
    else if (which == "UNLOAD-010") test010();
    else if (which == "UNLOAD-011") test011();
    else if (which == "UNLOAD-012") test012();
    else if (which == "UNLOAD-013") test013();
    else if (which == "UNLOAD-014") test014();
    else if (which == "UNLOAD-015") test015();
    else if (which == "UNLOAD-016") test016();
    else if (which == "UNLOAD-017") test017();
    else if (which == "UNLOAD-018") test018();
    else if (which == "UNLOAD-019") test019();
    else if (which == "UNLOAD-020") test020();
    else expect(false, which + " is not an unloading test");
}
