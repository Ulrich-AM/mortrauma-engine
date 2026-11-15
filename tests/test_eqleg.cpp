#include "mortrauma/assets/ModelLoader.hpp"
#include "mortrauma/balance/BalanceController.hpp"
#include "mortrauma/balance/SingleLegPose.hpp"
#include "mortrauma/balance/StandingPose.hpp"
#include "mortrauma/core/Paths.hpp"
#include "mortrauma/motors/MotorController.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <algorithm>
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

bool placeStanding(Lab& lab) {
    const auto placement = mt::placeStandingPose(lab.world, lab.ragdoll, lab.balance.config());
    if (!placement.ok) {
        return false;
    }
    lab.balance.setStanceLeg(0);
    lab.balance.setReference(placement.reference);
    lab.balance.resetState();
    lab.balance.setEnabled(true, lab.motors);
    return true;
}

void tick(Lab& lab) {
    const float dt = lab.world.settings().fixedTimestep;
    lab.balance.observe(lab.world, lab.ragdoll, dt);
    lab.balance.command(lab.ragdoll, lab.motors);
    lab.motors.apply(lab.world, lab.ragdoll);
    lab.world.step(dt, 1);
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

void test001() {
    Lab lab;
    expect(loadLab(lab), "EQLEG-001 lab loaded");
    const auto right = mt::placeSingleLegCandidate(lab.world, lab.ragdoll, lab.balance.config(), true);
    const auto left = mt::placeSingleLegCandidate(lab.world, lab.ragdoll, lab.balance.config(), false);
    std::cout << "right feasible " << right.feasible << " com " << right.comToAnkleM << " margin " << right.soleMarginM << " clear "
              << right.swingClearanceM << " torso " << right.torsoTilt << " ankle " << right.ankleTorque << " knee " << right.kneeTorque
              << " hip " << right.hipTorque << " root " << right.hipRootTorque << " stance " << right.stanceHipSwing1Deg << " "
              << right.stanceHipSwing2Deg << " " << right.stanceKneeDeg << " swingKnee " << right.swingKneeDeg
              << " swing2 " << right.swingHipSwing2Deg << " swing1 " << right.swingHipSwing1Deg << " spine " << right.spineSwing1Deg
              << " hands " << right.handOffsetM << " posture " << right.postureCost << " mag " << right.ankleMomentMagnitude << " Mg "
              << right.gravityMoment << " ref " << right.referenceError << " placed " << right.placed << " " << right.rejection << '\n';
    std::cout << "left feasible " << left.feasible << " com " << left.comToAnkleM << " torso " << left.torsoTilt << " root " << left.hipRootTorque
              << " swingKnee " << left.swingKneeDeg << " spine " << left.spineSwing1Deg << " placed " << left.placed << " " << left.limitingJoint
              << " " << left.rejection << '\n';
    expect(right.searched && left.searched && right.supportRight && !left.supportRight, "EQLEG-001 both legs were searched");
    expect(right.totalMass > 70.f && right.totalMass < 85.f, "EQLEG-001 mass is Humanoidguy");
    expect(!left.placed && left.reference.empty() && !left.feasible, "EQLEG-001 an infeasible left pose is not written");
    expect(left.rejection.find("Not placed") != std::string::npos, "EQLEG-001 the left rejection says the pose was not placed");
    expect(right.placed == right.feasible, "EQLEG-001 the right pose is written only when it is feasible");
    if (right.feasible) {
        expect(right.comToAnkleM < 0.02f && right.soleMarginM > 0.f && right.swingClearanceM > 0.01f && right.torsoTilt < 0.45f,
               "EQLEG-001 a placed right pose stays inside the sole, the ankle disk, and the torso gate");
        expect(right.ankleTorque <= 12.f && right.kneeTorque <= 35.f && right.hipTorque <= 50.f && right.hipRootTorque <= 18.f,
               "EQLEG-001 right static moments stay inside the motor limits");
        expect(right.referenceError < 0.25f && !right.reference.empty(), "EQLEG-001 the motor reference matches the written joints");
        expect(std::fabs(right.ankleMomentMagnitude - right.gravityMoment) < 8.f,
               "EQLEG-001 the stance-ankle moment is consistent with mass times the COM arm");
    }
}

void test002() {
    Lab lab;
    expect(loadLab(lab), "EQLEG-002 lab loaded");
    const auto pose = mt::placeSingleLegCandidate(lab.world, lab.ragdoll, lab.balance.config(), true);
    expect(pose.placed == pose.feasible, "EQLEG-002 placement follows feasibility");
    if (!pose.placed) {
        expect(!pose.feasible && pose.reference.empty(), "EQLEG-002 an infeasible pose is not started");
        return;
    }
    std::cout << "pose torso " << pose.torsoTilt << " swingKnee " << pose.swingKneeDeg << " swingHip " << pose.swingHipSwing2Deg
              << " hands " << pose.handOffsetM << " posture " << pose.postureCost << " ankleMag " << pose.ankleMomentMagnitude
              << " Mgarm " << pose.gravityMoment << '\n';
    lab.balance.setReference(pose.reference);
    lab.balance.resetState();
    lab.balance.setStanceLeg(1);
    lab.balance.setEnabled(true, lab.motors);
    bool early = false;
    bool sawEarly = false;
    bool dynamic = true;
    float duration = 0.f;
    float ankleAtStart = -1.f;
    const float dt = lab.world.settings().fixedTimestep;
    for (int i = 0; i < 360; ++i) {
        tick(lab);
        duration += dt;
        dynamic = dynamic && allDynamic(lab) && !lab.world.supportActive();
        const auto& state = lab.balance.state();
        if (i == 6) {
            ankleAtStart = state.ankleEffort;
            std::cout << "t 0.05 y " << state.com.y << " tilt " << state.torsoTilt << " ankle " << state.ankleEffort << " hip " << state.hipEffort
                      << " sat " << state.saturatedMotors << " moment " << state.estimatedAnkleMoment << '\n';
        }
        if (duration >= 0.2f && !sawEarly) {
            sawEarly = true;
            early = state.rightGrounded && state.leftPoints == 0 && state.com.y > 0.9f;
        }
        if (state.com.y < 0.85f || state.torsoTilt > 0.75f) {
            break;
        }
    }
    std::cout << "hold " << duration << " y " << lab.balance.state().com.y << " tilt " << lab.balance.state().torsoTilt
              << (duration >= 1.f && lab.balance.state().com.y > 0.9f ? " sustained" : " not a sustained stand") << '\n';
    expect(early && dynamic && ankleAtStart >= 0.f, "EQLEG-002 the right foot is planted and the left foot is clear at 0.2 s, with no hidden support");
}

void test003() {
    Lab lab;
    expect(loadLab(lab) && placeStanding(lab), "EQLEG-003 standing placed");
    expect(lab.balance.stanceLeg() == 0, "EQLEG-003 standing uses both legs");
    for (int i = 0; i < 120; ++i) {
        tick(lab);
    }
    const auto standing = lab.balance.state();
    lab.balance.setStanceLeg(2);
    expect(lab.balance.stanceLeg() == 2, "EQLEG-003 the left leg can be selected");
    lab.balance.setStanceLeg(0);
    for (int i = 0; i < 120; ++i) {
        tick(lab);
    }
    expect(standing.com.y > 0.95f && lab.balance.state().com.y > 0.95f && lab.balance.stanceLeg() == 0,
           "EQLEG-003 clearing the stance leg leaves two-foot standing up");
}

void test004() {
    Lab lab;
    expect(loadLab(lab) && placeStanding(lab), "EQLEG-004 standing placed");
    lab.balance.setStanceLeg(1);
    lab.balance.setStanceLeg(0);
    expect(lab.balance.stanceLeg() == 0 && allDynamic(lab) && !lab.world.supportActive(), "EQLEG-004 aborting the stance selection leaves a dynamic body");
}

void test005() {
    Lab lab;
    expect(loadLab(lab) && placeStanding(lab), "EQLEG-005 standing placed");
    for (int i = 0; i < 3600; ++i) {
        tick(lab);
    }
    const auto& state = lab.balance.state();
    expect(state.phase == mt::BalancePhase::Standing && state.com.y > 0.95f && state.torsoTilt < 0.35f && lab.balance.stanceLeg() == 0,
           "EQLEG-005 quiet standing still holds for 30 seconds");
}

void test006() {
    Lab lab;
    expect(loadLab(lab) && placeStanding(lab), "EQLEG-006 standing placed");
    for (int i = 0; i < 30; ++i) {
        tick(lab);
    }
    const auto* chest = lab.ragdoll.find("chest");
    const auto chestState = lab.world.bodyState(chest->physics);
    lab.world.addImpulse(chest->physics, {0.f, 0.f, 55.f}, chestState.position);
    bool fell = false;
    for (int i = 0; i < 360; ++i) {
        tick(lab);
        const auto& state = lab.balance.state();
        if (state.phase == mt::BalancePhase::Falling || state.com.y < 0.7f || state.torsoTilt > 0.7f) {
            fell = true;
            break;
        }
    }
    expect(fell && !lab.world.supportActive() && allDynamic(lab), "EQLEG-006 a 55 N·s push still falls with no hidden support");
}

void test007() {
    Lab lab;
    expect(loadLab(lab) && placeStanding(lab), "EQLEG-007 standing placed");
    for (int i = 0; i < 20; ++i) {
        tick(lab);
    }
    const auto* chest = lab.ragdoll.find("chest");
    const mt::Vec3 start = lab.world.bodyState(chest->physics).position;
    const int grab = lab.world.beginGrab(chest->physics, start, start + mt::Vec3{0.12f, 0.05f, 0.f}, 8.f, 0.6f);
    for (int i = 0; i < 40; ++i) {
        lab.world.updateGrab(grab, start + mt::Vec3{0.12f, 0.05f, 0.f});
        tick(lab);
    }
    const float moved = mt::length(lab.world.bodyState(chest->physics).position - start);
    lab.world.endGrab(grab);
    expect(moved > 0.01f && allDynamic(lab), "EQLEG-007 a grab still moves the chest");
}

void test009() {
    Lab lab;
    expect(loadLab(lab), "EQLEG-009 lab loaded");
    const auto right = mt::placeSingleLegCandidate(lab.world, lab.ragdoll, lab.balance.config(), true, false);
    const auto* pelvis = lab.ragdoll.find("pelvis");
    const mt::Vec3 afterRight = lab.world.bodyState(pelvis->physics).position;
    const auto plain = mt::placeSingleLegCandidate(lab.world, lab.ragdoll, lab.balance.config(), false, false);
    const mt::Vec3 afterRejected = lab.world.bodyState(pelvis->physics).position;
    expect(right.placed && mt::length(afterRejected - afterRight) < 1e-4f, "EQLEG-009 a rejected left search leaves the placed right pose");
    const auto diagnostic = mt::placeSingleLegCandidate(lab.world, lab.ragdoll, lab.balance.config(), false, true);
    if (!plain.feasible) {
        expect(!plain.placed && plain.reference.empty(), "EQLEG-009 the default path does not write an infeasible pose");
        expect(diagnostic.placed && diagnostic.diagnostic && diagnostic.rejection.find("diagnostic only") != std::string::npos,
               "EQLEG-009 the diagnostic path is labeled and written");
    } else {
        expect(plain.placed && diagnostic.placed && !diagnostic.diagnostic, "EQLEG-009 a feasible pose is a normal placement");
    }
    expect(mt::finite(afterRight) && mt::finite(afterRejected), "EQLEG-009 pelvis position was readable");
}

void test008() {
    bool first = false;
    bool second = true;
    for (int run = 0; run < 2; ++run) {
        Lab lab;
        expect(loadLab(lab), "EQLEG-008 lab loaded");
        const auto pose = mt::placeSingleLegCandidate(lab.world, lab.ragdoll, lab.balance.config(), true);
        if (run == 0) {
            first = pose.feasible;
        } else {
            second = pose.feasible;
        }
    }
    expect(first == second, "EQLEG-008 the right-leg static result repeats");
}

}  // namespace

void runEqLegTest(const std::string& which) {
    if (which == "EQLEG-001") test001();
    else if (which == "EQLEG-002") test002();
    else if (which == "EQLEG-003") test003();
    else if (which == "EQLEG-004") test004();
    else if (which == "EQLEG-005") test005();
    else if (which == "EQLEG-006") test006();
    else if (which == "EQLEG-007") test007();
    else if (which == "EQLEG-008") test008();
    else if (which == "EQLEG-009") test009();
    else expect(false, "unknown " + which);
}

void runAllEqLegTests() {
    test001();
    test002();
    test003();
    test004();
    test005();
    test006();
    test007();
    test008();
    test009();
}
