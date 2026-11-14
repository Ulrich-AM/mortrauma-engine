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

float soleY(const Lab& lab, const char* id) {
    const auto* foot = lab.ragdoll.find(id);
    const auto state = lab.world.bodyState(foot->physics);
    const mt::Vec3 heel = state.position + mt::rotate(state.rotation, foot->heelLocal);
    const mt::Vec3 toe = state.position + mt::rotate(state.rotation, foot->forefootLocal);
    return std::min(heel.y, toe.y);
}

void settle(Lab& lab, int steps) {
    for (int i = 0; i < steps; ++i) {
        tick(lab);
    }
}

void test001() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-001 placed");
    settle(lab, 30);
    lab.stepping.setPhaseDuration(0.4f);
    lab.stepping.requestTransfer(true);
    for (int i = 0; i < 50; ++i) {
        tick(lab);
    }
    const auto& state = lab.balance.state();
    const auto* stance = lab.ragdoll.find("foot_1");
    const mt::Vec3 stancePos = lab.world.bodyState(stance->physics).position;
    const mt::Vec3 toward = stancePos - state.equilibriumTarget;
    const mt::Vec3 shifted = state.balanceReference - state.equilibriumTarget;
    std::cout << "shift " << shifted.x << " " << shifted.z << " toward " << toward.x << " " << toward.z << '\n';
    expect(mt::dot(mt::Vec3{shifted.x, 0.f, shifted.z}, mt::Vec3{toward.x, 0.f, toward.z}) > 0.f, "STEP-001 the target moves toward the right foot");
    expect(!lab.world.supportActive(), "STEP-001 no support fixture");
}

void test002() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-002 placed");
    settle(lab, 20);
    lab.stepping.requestLift(true);
    bool dynamic = true;
    mt::Vec3 previous{};
    bool have = false;
    float jump = 0.f;
    for (int i = 0; i < 80; ++i) {
        const auto sample = lab.world.bodyState(lab.ragdoll.find("foot_2")->physics);
        if (have) {
            jump = std::max(jump, mt::length(sample.position - previous));
        }
        previous = sample.position;
        have = true;
        tick(lab);
        dynamic = dynamic && sample.isDynamic;
    }
    std::cout << "max foot step " << jump << " phase " << mt::stepPhaseName(lab.stepping.phase()) << '\n';
    expect(dynamic && jump < 0.04f && !lab.world.supportActive(), "STEP-002 the swing foot stays dynamic and is not teleported");
}

void test003() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-003 placed");
    settle(lab, 40);
    const float before = soleY(lab, "foot_2");
    lab.stepping.setLiftHeight(0.04f);
    lab.stepping.setPhaseDuration(0.6f);
    lab.stepping.requestLift(true);
    float peak = before;
    float uprightPeak = before;
    float minCom = 1e9f;
    bool finite = true;
    for (int i = 0; i < 360; ++i) {
        tick(lab);
        const float sole = soleY(lab, "foot_2");
        peak = std::max(peak, sole);
        if (lab.balance.state().com.y > 0.9f) {
            uprightPeak = std::max(uprightPeak, sole);
        }
        minCom = std::min(minCom, lab.balance.state().com.y);
        finite = finite && lab.balance.state().finite;
    }
    std::cout << "left sole before " << before << " uprightPeak " << uprightPeak << " uprightClearance " << uprightPeak - before << " fallenPeak "
              << peak << " minCom " << minCom << " phase " << mt::stepPhaseName(lab.stepping.phase()) << '\n';
    expect(finite && !lab.world.supportActive(), "STEP-003 the lift attempt stays finite without a fixture");
}

void test004() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-004 placed");
    settle(lab, 20);
    lab.stepping.requestLift(false);
    float over = 0.f;
    for (int i = 0; i < 200; ++i) {
        tick(lab);
        for (const auto& joint : lab.ragdoll.joints) {
            const auto sample = lab.world.jointState(joint.physics);
            if (!sample.valid || sample.motorTorqueLimit <= 0.f) {
                continue;
            }
            const float peak = std::max(std::fabs(sample.motorTorque.x), std::max(std::fabs(sample.motorTorque.y), std::fabs(sample.motorTorque.z)));
            over = std::max(over, peak - sample.motorTorqueLimit);
        }
    }
    std::cout << "over " << over << '\n';
    expect(over < 0.75f, "STEP-004 stepping still respects torque limits");
}

void test005() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-005 placed");
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
    expect(finite && fell && !lab.world.supportActive(), "STEP-005 a 55 N·s push still falls");
}

void test006() {
    for (const bool right : {true, false}) {
        Lab lab;
        expect(loadLab(lab) && place(lab), "STEP-006 placed");
        settle(lab, 30);
        const auto* stance = lab.ragdoll.find(right ? "foot_1" : "foot_2");
        const mt::Vec3 stancePos = lab.world.bodyState(stance->physics).position;
        lab.stepping.requestTransfer(right);
        bool finite = true;
        float best = -1.f;
        float bestY = 0.f;
        for (int i = 0; i < 200; ++i) {
            tick(lab);
            finite = finite && lab.balance.state().finite;
            const mt::Vec3 toward{stancePos.x - lab.balance.state().equilibriumTarget.x, 0.f, stancePos.z - lab.balance.state().equilibriumTarget.z};
            const mt::Vec3 moved{lab.balance.state().com.x - lab.balance.state().equilibriumTarget.x, 0.f,
                                 lab.balance.state().com.z - lab.balance.state().equilibriumTarget.z};
            const float along = mt::length(toward) > 1e-4f ? mt::dot(moved, mt::normalize(toward)) : 0.f;
            if (along > best) {
                best = along;
                bestY = lab.balance.state().com.y;
            }
        }
        std::cout << (right ? "right" : "left") << " peak shift toward stance " << best << " m at comY " << bestY << " endY " << lab.balance.state().com.y
                  << '\n';
        expect(finite && !lab.world.supportActive(), "STEP-006 transfer stays a physical simulation");
    }
}

void test007() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STEP-007 placed");
    lab.stepping.setPhaseDuration(0.2f);
    lab.stepping.requestTransfer(true);
    for (int i = 0; i < 80; ++i) {
        tick(lab);
    }
    expect(lab.stepping.phase() == mt::StepPhase::Idle, "STEP-007 the attempt returns to idle");
    tick(lab);
    const auto& state = lab.balance.state();
    const float leftover = mt::length(mt::Vec3{state.balanceReference.x - state.equilibriumTarget.x, 0.f, state.balanceReference.z - state.equilibriumTarget.z});
    std::cout << "leftover bias " << leftover << '\n';
    expect(leftover < 0.01f, "STEP-007 the step bias is cleared");
}

}  // namespace

void runStepTest(const std::string& which) {
    if (which == "STEP-001") test001();
    else if (which == "STEP-002") test002();
    else if (which == "STEP-003") test003();
    else if (which == "STEP-004") test004();
    else if (which == "STEP-005") test005();
    else if (which == "STEP-006") test006();
    else if (which == "STEP-007") test007();
    else {
        std::cerr << "unknown test " << which << '\n';
        ++gFailures;
    }
}

void runAllStepTests() {
    test001();
    test002();
    test003();
    test004();
    test005();
    test006();
    test007();
}
