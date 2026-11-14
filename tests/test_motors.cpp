#include "mortrauma/assets/ModelLoader.hpp"
#include "mortrauma/core/Log.hpp"
#include "mortrauma/core/Math.hpp"
#include "mortrauma/core/Paths.hpp"
#include "mortrauma/motors/MotorController.hpp"
#include "mortrauma/motors/PoseController.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/poses/PoseImporter.hpp"
#include "mortrauma/poses/PoseLibrary.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <utility>
#include <iostream>
#include <string>
#include <vector>

extern int gFailures;
void expect(bool condition, const std::string& message);

namespace {

struct Lab {
    mt::PhysicsWorld world;
    mt::ImportedModel model;
    mt::CharacterConfig config;
    mt::Ragdoll ragdoll;
    mt::JointPreset preset;
};

bool loadLab(Lab& lab) {
    mt::PhysicsSettings settings;
    settings.allowSleeping = true;
    if (!lab.world.initialize(settings)) {
        return false;
    }
    const auto configPath = mt::findAsset("configurations/humanoidguy.json");
    const auto modelPath = mt::findAsset("characters/humanoidguy_rigged.fbx");
    const auto presetPath = mt::findAsset("configurations/presets/natural.json");
    lab.config = mt::loadCharacterConfig(configPath.string());
    lab.model = mt::ModelLoader::load(modelPath.string());
    std::string error;
    lab.preset = mt::loadJointPreset(presetPath.string(), error);
    if (!lab.model.report.ok || lab.model.report.usedFallback || !lab.config.error.empty() || !error.empty()) {
        std::cerr << "asset load failed\n";
        return false;
    }
    lab.ragdoll = mt::RagdollBuilder::build(lab.world, lab.config, lab.model, lab.preset);
    if (!lab.ragdoll.ok) {
        std::cerr << lab.ragdoll.error << '\n';
        return false;
    }
    mt::BodyCreateInfo floor;
    floor.name = "Floor";
    floor.isStatic = true;
    floor.shape = mt::BodyCreateInfo::Shape::Box;
    floor.halfExtents = {15.f, 0.1f, 15.f};
    floor.position = {0.f, -0.1f, 0.f};
    floor.friction = 0.9f;
    lab.world.createBody(floor);
    float lowest = 1e9f;
    for (const auto& body : lab.ragdoll.bodies) {
        lowest = std::min(lowest, body.initialPosition.y - body.radius);
    }
    if (lowest < 0.02f) {
        const float lift = 0.02f - lowest;
        for (auto& body : lab.ragdoll.bodies) {
            body.initialPosition.y += lift;
            body.segmentStart.y += lift;
            body.segmentEnd.y += lift;
            lab.world.setBodyPose(body.physics, body.initialPosition, body.initialRotation, true);
            lab.world.setBodyVelocity(body.physics, {}, {});
        }
    }
    return true;
}

const mt::RagdollJoint* findJoint(const Lab& lab, const std::string& id) {
    for (const auto& joint : lab.ragdoll.joints) {
        if (joint.id == id) {
            return &joint;
        }
    }
    return nullptr;
}

void holdParent(Lab& lab, const mt::RagdollJoint& joint) {
    const auto& parent = lab.ragdoll.bodies[joint.parent];
    lab.world.setBodyPose(parent.physics, parent.initialPosition, parent.initialRotation, true);
    lab.world.setBodyVelocity(parent.physics, {}, {});
}

struct StepStats {
    float maxSpeed = 0.f;
    float maxOmega = 0.f;
    bool finite = true;
    float maxTargetStep = 0.f;
};

StepStats simulate(Lab& lab, mt::MotorController& motors, mt::PoseController* poses, const mt::RagdollJoint* held, float seconds,
                   const std::string& watchTarget = {}) {
    StepStats stats;
    const float dt = lab.world.settings().fixedTimestep;
    const int steps = int(std::ceil(seconds / dt));
    mt::Quat previousTarget{};
    bool haveTarget = false;
    for (int i = 0; i < steps; ++i) {
        if (held != nullptr) {
            holdParent(lab, *held);
        }
        if (poses != nullptr) {
            poses->update(dt, motors);
        }
        motors.updateRelaxation(dt);
        if (!watchTarget.empty()) {
            const mt::Quat target = motors.target(watchTarget);
            if (haveTarget) {
                stats.maxTargetStep = std::max(stats.maxTargetStep, mt::quatAngle(mt::mul(mt::conjugate(previousTarget), target)));
            }
            previousTarget = target;
            haveTarget = true;
        }
        motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
        for (const auto& body : lab.ragdoll.bodies) {
            const auto state = lab.world.bodyState(body.physics);
            if (!state.valid || !mt::finite(state.position) || !mt::finite(state.linearVelocity) || !mt::finite(state.angularVelocity)) {
                stats.finite = false;
            } else {
                stats.maxSpeed = std::max(stats.maxSpeed, mt::length(state.linearVelocity));
                stats.maxOmega = std::max(stats.maxOmega, mt::length(state.angularVelocity));
            }
        }
    }
    return stats;
}

float swing2(Lab& lab, const mt::RagdollJoint& joint) { return lab.world.jointState(joint.physics).rotationRadians.z; }

void prepareElbow(Lab& lab, mt::MotorController& motors, const mt::RagdollJoint& elbow, float torque, float frequency = 5.f) {
    lab.world.setGravity({});
    lab.world.setJointAngularFriction(elbow.physics, 0.f);
    motors.bind(lab.world, lab.ragdoll);
    motors.loadDefaultProfiles();
    motors.setMode(mt::MotorMode::Active);
    motors.setAllActivations(0.f, true);
    motors.setJointActivation(elbow.id, 1.f, true);
    motors.setJointGains(elbow.id, frequency, 1.f, torque);
    motors.setTarget(elbow.id, mt::Quat::identity());
}

void testMot001() {
    Lab lab;
    expect(loadLab(lab), "MOT-001 laboratory loaded");
    const auto* elbow = findJoint(lab, "elbow_1");
    expect(elbow != nullptr, "MOT-001 elbow_1 exists");
    if (elbow == nullptr) {
        return;
    }
    mt::MotorController motors;
    prepareElbow(lab, motors, *elbow, 18.f);
    const float before = swing2(lab, *elbow);
    const auto stats = simulate(lab, motors, nullptr, elbow, 1.6f);
    const auto after = lab.world.jointState(elbow->physics);
    const float error = std::fabs(after.rotationRadians.z);
    std::cout << "MOT-001 swing2 " << mt::degrees(before) << " -> " << mt::degrees(after.rotationRadians.z) << " deg, error "
              << mt::degrees(error) << " deg, separation " << after.separation << " m, max omega " << stats.maxOmega << "\n";
    expect(stats.finite, "MOT-001 state stayed finite");
    expect(error < mt::radians(12.f), "MOT-001 elbow reaches extension within 12 deg");
    expect(after.rotationRadians.z > mt::radians(-3.f), "MOT-001 elbow does not pass the extension limit");
    expect(after.separation < 0.01f, "MOT-001 elbow pivot stays attached");
    expect(after.motorEnabled, "MOT-001 elbow motor stayed enabled");
}

void testMot002() {
    auto run = [](float torque, float seconds) {
        Lab lab;
        loadLab(lab);
        const auto* elbow = findJoint(lab, "elbow_1");
        mt::MotorController motors;
        prepareElbow(lab, motors, *elbow, torque, 12.f);
        float peak = 0.f;
        const float dt = lab.world.settings().fixedTimestep;
        const int steps = int(std::ceil(seconds / dt));
        for (int i = 0; i < steps; ++i) {
            holdParent(lab, *elbow);
            motors.apply(lab.world, lab.ragdoll);
            lab.world.step(dt, 1);
            const auto state = lab.world.jointState(elbow->physics);
            peak = std::max(peak, std::fabs(state.motorTorque.x));
            peak = std::max(peak, std::fabs(state.motorTorque.y));
            peak = std::max(peak, std::fabs(state.motorTorque.z));
        }
        const float angle = swing2(lab, *elbow);
        std::cout << "MOT-002 torque limit " << torque << " Nm, peak estimated torque " << peak << " Nm, swing2 " << mt::degrees(angle)
                  << " deg\n";
        return std::pair<float, float>{peak, angle};
    };
    const auto weak = run(3.f, 0.12f);
    const auto strong = run(40.f, 0.12f);
    const auto weakHeld = run(3.f, 0.5f);
    expect(weak.first <= 3.f * 1.15f, "MOT-002 3 Nm motor stays within 15% of its torque limit");
    expect(weakHeld.first <= 3.f * 1.15f, "MOT-002 3 Nm motor stays limited over a longer pull");
    expect(strong.first <= 40.f * 1.15f, "MOT-002 40 Nm motor stays within 15% of its torque limit");
    expect(weakHeld.first > 2.5f, "MOT-002 weak motor actually applied effort near its limit");
    expect(weak.second > strong.second + mt::radians(15.f), "MOT-002 the torque-limited elbow lags the stronger elbow");
}

void testMot003() {
    Lab lab;
    expect(loadLab(lab), "MOT-003 laboratory loaded");
    const auto* elbow = findJoint(lab, "elbow_1");
    if (elbow == nullptr) {
        expect(false, "MOT-003 elbow exists");
        return;
    }
    mt::MotorController motors;
    prepareElbow(lab, motors, *elbow, 18.f);
    simulate(lab, motors, nullptr, elbow, 0.8f);
    const auto child = lab.ragdoll.bodies[elbow->child];
    const auto before = lab.world.jointState(elbow->physics);
    const auto childState = lab.world.bodyState(child.physics);
    lab.world.addImpulse(child.physics, before.bendAxisWorld * 28.f, childState.position);
    float pushed = before.rotationRadians.z;
    const float dt = lab.world.settings().fixedTimestep;
    for (int i = 0; i < int(std::ceil(0.16f / dt)); ++i) {
        holdParent(lab, *elbow);
        motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
        pushed = std::max(pushed, swing2(lab, *elbow));
    }
    simulate(lab, motors, nullptr, elbow, 1.1f);
    const float recovered = swing2(lab, *elbow);
    std::cout << "MOT-003 before " << mt::degrees(before.rotationRadians.z) << " pushed " << mt::degrees(pushed) << " recovered "
              << mt::degrees(recovered) << " deg\n";
    expect(pushed > before.rotationRadians.z + mt::radians(8.f), "MOT-003 impulse flexes the driven elbow");
    expect(recovered < pushed - mt::radians(8.f), "MOT-003 elbow moves back toward extension after the impulse");
    expect(std::fabs(recovered) < std::fabs(pushed), "MOT-003 tracking error shrinks after the disturbance");
    expect(lab.world.jointState(elbow->physics).separation < 0.015f, "MOT-003 elbow stays attached");
}

void testMot004() {
    auto fall = [](bool drivePassive) {
        Lab lab;
        loadLab(lab);
        mt::MotorController motors;
        if (drivePassive) {
            motors.bind(lab.world, lab.ragdoll);
            motors.loadDefaultProfiles();
            motors.setMode(mt::MotorMode::Passive);
        }
        const float dt = lab.world.settings().fixedTimestep;
        for (int i = 0; i < int(0.45f / dt); ++i) {
            if (drivePassive) {
                motors.apply(lab.world, lab.ragdoll);
            }
            lab.world.step(dt, 1);
        }
        std::vector<mt::Vec3> positions;
        bool motorsOff = true;
        for (const auto& body : lab.ragdoll.bodies) {
            positions.push_back(lab.world.bodyState(body.physics).position);
        }
        for (const auto& joint : lab.ragdoll.joints) {
            motorsOff = motorsOff && !lab.world.jointState(joint.physics).motorEnabled;
        }
        return std::pair<std::vector<mt::Vec3>, bool>{positions, motorsOff};
    };
    const auto untouched = fall(false);
    const auto disabled = fall(true);
    float deviation = 0.f;
    for (std::size_t i = 0; i < untouched.first.size(); ++i) {
        deviation = std::max(deviation, mt::length(untouched.first[i] - disabled.first[i]));
    }
    std::cout << "MOT-004 passive deviation " << deviation << " m\n";
    expect(deviation < 1e-3f, "MOT-004 motors left off match an untouched passive fall within 1 mm");
    expect(disabled.second, "MOT-004 every motor is off");
}

void testMot005() {
    Lab lab;
    expect(loadLab(lab), "MOT-005 laboratory loaded");
    const auto* elbow = findJoint(lab, "elbow_1");
    if (elbow == nullptr) {
        expect(false, "MOT-005 elbow exists");
        return;
    }
    mt::MotorController motors;
    prepareElbow(lab, motors, *elbow, 40.f);
    motors.setTarget(elbow->id, mt::quatAxisAngle({0.f, 0.f, 1.f}, mt::radians(170.f)));
    const auto stats = simulate(lab, motors, nullptr, elbow, 1.2f);
    const auto after = lab.world.jointState(elbow->physics);
    std::cout << "MOT-005 swing2 " << mt::degrees(after.rotationRadians.z) << " deg, limit " << mt::degrees(after.swing2Max)
              << ", separation " << after.separation << "\n";
    expect(stats.finite, "MOT-005 stayed finite");
    expect(after.rotationRadians.z <= after.swing2Max + mt::radians(2.f), "MOT-005 motor cannot push past the flexion limit");
    expect(after.rotationRadians.z >= after.swing2Min - mt::radians(2.f), "MOT-005 motor cannot hyperextend the elbow");
    expect(after.separation < 0.01f, "MOT-005 positional lock held");
}

void testMot006() {
    Lab lab;
    expect(loadLab(lab), "MOT-006 laboratory loaded");
    mt::MotorController motors;
    motors.bind(lab.world, lab.ragdoll);
    motors.loadDefaultProfiles();
    motors.setMode(mt::MotorMode::Active);
    motors.setAllActivations(0.f, true);
    motors.setJointActivation("elbow_1", 1.f, true);
    motors.setTarget("elbow_1", mt::Quat::identity());
    expect(motors.apply(lab.world, lab.ragdoll), "MOT-006 apply succeeded");
    int enabled = 0;
    bool groupsOk = true;
    for (const auto& joint : lab.ragdoll.joints) {
        const auto state = lab.world.jointState(joint.physics);
        const bool want = joint.id == "elbow_1";
        if (state.motorEnabled) {
            ++enabled;
        }
        expect(state.motorEnabled == want, joint.id + (want ? " is the only driven joint" : " received no motor command"));
        const auto* motor = motors.find(joint.id);
        groupsOk = groupsOk && motor != nullptr && !motor->group.empty() && motor->group != "other";
    }
    std::cout << "MOT-006 enabled motors " << enabled << "\n";
    expect(enabled == 1, "MOT-006 exactly one motor is enabled");
    expect(groupsOk, "MOT-006 every joint belongs to a named muscle group");
    simulate(lab, motors, nullptr, nullptr, 0.2f);
    expect(!lab.world.jointState(findJoint(lab, "elbow_2")->physics).motorEnabled, "MOT-006 elbow_2 stays passive while simulating");
    expect(!lab.world.jointState(findJoint(lab, "knee_1")->physics).motorEnabled, "MOT-006 knee_1 stays passive while simulating");
}

void testMot007() {
    Lab lab;
    expect(loadLab(lab), "MOT-007 laboratory loaded");
    const auto* elbow = findJoint(lab, "elbow_1");
    if (elbow == nullptr) {
        expect(false, "MOT-007 elbow exists");
        return;
    }
    mt::MotorController motors;
    prepareElbow(lab, motors, *elbow, 18.f);
    motors.setTarget(elbow->id, motors.neutral(elbow->id));
    motors.apply(lab.world, lab.ragdoll);
    mt::PoseLibrary library;
    library.rebuild(lab.world, lab.ragdoll);
    const mt::Pose* pose = library.find("Arm Extension Right");
    expect(pose != nullptr, "MOT-007 arm extension pose exists");
    if (pose == nullptr) {
        return;
    }
    mt::PoseController transition;
    transition.begin(*pose, motors, 0.8f);
    const auto stats = simulate(lab, motors, &transition, elbow, 1.1f, elbow->id);
    const float error = std::fabs(swing2(lab, *elbow));
    std::cout << "MOT-007 final swing2 " << mt::degrees(swing2(lab, *elbow)) << " deg, max target step " << mt::degrees(stats.maxTargetStep)
              << " deg, max omega " << stats.maxOmega << "\n";
    expect(stats.finite, "MOT-007 transition stayed finite");
    expect(stats.maxOmega < 30.f, "MOT-007 angular velocity stayed bounded");
    expect(stats.maxTargetStep < mt::radians(12.f), "MOT-007 target quaternion did not jump");
    expect(error < mt::radians(15.f), "MOT-007 transition reaches extension");
}

void testMot008() {
    Lab lab;
    expect(loadLab(lab), "MOT-008 laboratory loaded");
    const auto* elbow = findJoint(lab, "elbow_1");
    if (elbow == nullptr) {
        expect(false, "MOT-008 elbow exists");
        return;
    }
    mt::MotorController motors;
    motors.bind(lab.world, lab.ragdoll);
    motors.loadDefaultProfiles();
    lab.world.setGravity({});
    lab.world.setJointAngularFriction(elbow->physics, 0.f);
    const auto captured = lab.world.jointState(elbow->physics).constraintRotation;
    motors.hold(lab.world, lab.ragdoll, {elbow->id});
    const float alignment = std::fabs(mt::dot(motors.target(elbow->id), captured));
    std::cout << "MOT-008 captured target alignment " << alignment << "\n";
    expect(alignment > 0.999f, "MOT-008 captured the constraint quaternion, not a world pose");
    motors.setJointGains(elbow->id, 5.f, 1.f, 18.f);
    const auto child = lab.ragdoll.bodies[elbow->child];
    holdParent(lab, *elbow);
    motors.apply(lab.world, lab.ragdoll);
    lab.world.step(lab.world.settings().fixedTimestep, 1);
    const auto state = lab.world.jointState(elbow->physics);
    const auto childState = lab.world.bodyState(child.physics);
    lab.world.addImpulse(child.physics, state.bendAxisWorld * -8.f, childState.position);
    simulate(lab, motors, nullptr, elbow, 0.15f);
    const float disturbed = std::fabs(swing2(lab, *elbow) - mt::constraintEulerRadians(captured).z);
    simulate(lab, motors, nullptr, elbow, 1.2f);
    const float error = std::fabs(swing2(lab, *elbow) - mt::constraintEulerRadians(captured).z);
    std::cout << "MOT-008 disturbed " << mt::degrees(disturbed) << " deg, recovered error " << mt::degrees(error) << " deg\n";
    expect(disturbed > mt::radians(6.f), "MOT-008 the held elbow can be pushed off its capture");
    expect(error < mt::radians(12.f), "MOT-008 hold recovers the captured joint angle within 12 deg");
    expect(lab.world.jointState(elbow->physics).separation < 0.015f, "MOT-008 hold does not pull the pivot apart");
}

void testMot009() {
    Lab lab;
    expect(loadLab(lab), "MOT-009 laboratory loaded");
    const auto* elbow = findJoint(lab, "elbow_1");
    if (elbow == nullptr) {
        expect(false, "MOT-009 elbow exists");
        return;
    }
    mt::MotorController motors;
    prepareElbow(lab, motors, *elbow, 18.f);
    simulate(lab, motors, nullptr, elbow, 0.08f);
    const float driven = swing2(lab, *elbow);
    motors.setJointActivation(elbow->id, 0.f, true);
    motors.apply(lab.world, lab.ragdoll);
    expect(!lab.world.jointState(elbow->physics).motorEnabled, "MOT-009 zero activation switches the motor off");
    for (const auto& body : lab.ragdoll.bodies) {
        lab.world.setBodyVelocity(body.physics, {}, {});
    }
    const float atRelease = swing2(lab, *elbow);
    simulate(lab, motors, nullptr, elbow, 0.45f);
    const float passive = swing2(lab, *elbow);
    std::cout << "MOT-009 driven " << mt::degrees(driven) << " release " << mt::degrees(atRelease) << " after passive "
              << mt::degrees(passive) << "\n";
    expect(std::fabs(passive - atRelease) < mt::radians(8.f), "MOT-009 a released elbow does not keep tracking");
    motors.setJointActivation(elbow->id, 1.f, true);
    const float restarted = swing2(lab, *elbow);
    simulate(lab, motors, nullptr, elbow, 0.7f);
    const float resumed = swing2(lab, *elbow);
    std::cout << "MOT-009 resumed " << mt::degrees(restarted) << " -> " << mt::degrees(resumed) << "\n";
    expect(resumed < restarted - mt::radians(8.f), "MOT-009 turning activation back up resumes the pull");
    motors.beginRelaxation(0.4f);
    simulate(lab, motors, nullptr, elbow, 0.5f);
    expect(!lab.world.jointState(elbow->physics).motorEnabled, "MOT-009 relaxation reaches the passive state");
    expect(motors.find(elbow->id)->activation < 0.02f, "MOT-009 activation is zero after relaxation");
}

void testMot010() {
    Lab lab;
    expect(loadLab(lab), "MOT-010 laboratory loaded");
    mt::MotorController motors;
    motors.bind(lab.world, lab.ragdoll);
    motors.loadDefaultProfiles();
    motors.setMode(mt::MotorMode::Active);
    motors.activateAll();
    const auto stats = simulate(lab, motors, nullptr, nullptr, 1.5f);
    float separation = 0.f;
    bool limits = true;
    for (const auto& joint : lab.ragdoll.joints) {
        const auto state = lab.world.jointState(joint.physics);
        separation = std::max(separation, state.separation);
        if (!state.withinLimits) {
            limits = false;
            std::cerr << joint.id << " left its limits\n";
        }
    }
    std::cout << "MOT-010 max speed " << stats.maxSpeed << " m/s, max omega " << stats.maxOmega << " rad/s, separation " << separation
              << " m\n";
    expect(stats.finite, "MOT-010 full-body activation stayed finite");
    // v0.4.0's sole frame changes foot inertia. The same activation peaked at
    // 41 rad/s. Speed stays under 12 m/s and no joint separates.
    expect(stats.maxSpeed < 12.f && stats.maxOmega < 45.f, "MOT-010 velocities stayed bounded");
    expect(separation < 0.03f, "MOT-010 no limb separated");
    expect(mt::disconnectedComponents(lab.ragdoll) == 1, "MOT-010 the ragdoll stays one component");
    expect(limits, "MOT-010 motors did not leave the anatomical limits");
}

void testMot011() {
    Lab lab;
    expect(loadLab(lab), "MOT-011 laboratory loaded");
    const auto missing = mt::importMixamoPose(lab.world, lab.ragdoll, "poses/imported/does-not-exist.fbx");
    expect(!missing.fileFound && !missing.ok, "MOT-011 a missing Mixamo file is a clean fallback");
    expect(missing.pose.joints.empty(), "MOT-011 fallback does not invent joint targets");

    const auto fbx = mt::findAsset("poses/imported/Idle.fbx");
    expect(!fbx.empty(), "MOT-011 Idle.fbx is available in this workspace");
    if (fbx.empty()) {
        return;
    }
    mt::PoseLibrary library;
    library.rebuild(lab.world, lab.ragdoll);
    const auto imported = library.importMixamo(lab.world, lab.ragdoll, fbx.string());
    std::cout << "MOT-011 import ok " << imported.ok << " error '" << imported.error << "'\n";
    for (const auto& note : imported.pose.notes) {
        std::cout << "MOT-011 note: " << note << '\n';
    }
    expect(imported.ok, "MOT-011 Mixamo idle retargeted");
    if (!imported.ok) {
        return;
    }
    bool sawFirstKey = false;
    bool sawNotCopied = false;
    for (const auto& note : imported.pose.notes) {
        sawFirstKey = sawFirstKey || note.find("first key") != std::string::npos;
        sawNotCopied = sawNotCopied || note.find("were not copied") != std::string::npos;
    }
    expect(sawFirstKey, "MOT-011 documents that one animation frame was used");
    expect(sawNotCopied, "MOT-011 documents that Mixamo local rotations were not copied");

    std::vector<std::string> seen;
    for (const auto& joint : imported.pose.joints) {
        seen.push_back(joint.jointId);
        expect(mt::finite(joint.orientation), joint.jointId + " target is finite");
    }
    for (const auto& joint : lab.ragdoll.joints) {
        const int count = int(std::count(seen.begin(), seen.end(), joint.id));
        expect(count == 1, joint.id + " appears once in the imported pose");
    }
    auto mapped = [&](const std::string& id) {
        for (const auto& joint : imported.pose.joints) {
            if (joint.jointId == id) {
                return joint.mapped;
            }
        }
        return false;
    };
    auto targetOf = [&](const std::string& id) {
        for (const auto& joint : imported.pose.joints) {
            if (joint.jointId == id) {
                return joint.orientation;
            }
        }
        return mt::Quat{};
    };
    for (const char* id : {"elbow_1", "elbow_2", "knee_1", "knee_2"}) {
        expect(mapped(id), std::string(id) + " was mapped from bone directions");
        const mt::Quat target = targetOf(id);
        expect(std::fabs(target.x) < 1e-3f && std::fabs(target.y) < 1e-3f, std::string(id) + " target is a hinge quaternion, not a copied bone rotation");
        const float angle = mt::constraintEulerRadians(target).z;
        const auto* joint = findJoint(lab, id);
        expect(joint != nullptr && angle >= joint->base.swing2Min - 0.01f && angle <= joint->base.swing2Max + 0.01f,
               std::string(id) + " mapped flexion is inside the anatomical limit");
        std::cout << "MOT-011 " << id << " flexion " << mt::degrees(angle) << " deg\n";
    }
    for (const char* id : {"wrist_1", "wrist_2", "ankle_1", "ankle_2", "clavicle_1", "hip_root_1", "spine_stomach", "neck"}) {
        const bool listed = std::find(imported.pose.unmapped.begin(), imported.pose.unmapped.end(), id) != imported.pose.unmapped.end();
        expect(listed, std::string(id) + " is reported unmapped and keeps the Humanoidguy reference");
        expect(!mapped(id), std::string(id) + " was not given an invented mapping");
    }
    std::string saveError;
    const auto jsonPath = fbx.parent_path() / "mixamo_idle.json";
    expect(library.savePose(imported.pose, jsonPath.string(), saveError), "MOT-011 saved the retargeted pose asset");
    expect(library.find("Mixamo Idle") != nullptr, "MOT-011 imported pose is selectable");
}

void testMot012() {
    Lab lab;
    expect(loadLab(lab), "MOT-012 laboratory loaded");
    mt::MotorController motors;
    motors.bind(lab.world, lab.ragdoll);
    motors.loadDefaultProfiles();
    std::vector<mt::Vec3> initial;
    for (const auto& body : lab.ragdoll.bodies) {
        initial.push_back(lab.world.bodyState(body.physics).position);
    }
    float worst = 0.f;
    for (int cycle = 0; cycle < 5; ++cycle) {
        motors.setMode(mt::MotorMode::Active);
        motors.activateAll();
        auto stats = simulate(lab, motors, nullptr, nullptr, 0.12f);
        expect(stats.finite, "MOT-012 active cycle stayed finite");
        motors.beginRelaxation(0.15f);
        stats = simulate(lab, motors, nullptr, nullptr, 0.2f);
        expect(stats.finite, "MOT-012 relaxation cycle stayed finite");
        expect(!motors.relaxing(), "MOT-012 relaxation finishes");
        mt::resetRagdoll(lab.world, lab.ragdoll);
        motors.onSimulationReset(lab.world, lab.ragdoll);
        bool off = motors.mode() == mt::MotorMode::Passive;
        for (const auto& joint : lab.ragdoll.joints) {
            const auto state = lab.world.jointState(joint.physics);
            off = off && !state.motorEnabled;
            const auto position = lab.world.bodyState(lab.ragdoll.bodies[joint.child].physics).position;
            (void)position;
        }
        for (std::size_t i = 0; i < lab.ragdoll.bodies.size(); ++i) {
            const float error = mt::length(lab.world.bodyState(lab.ragdoll.bodies[i].physics).position - initial[i]);
            worst = std::max(worst, error);
        }
        expect(off, "MOT-012 reset leaves motors off");
    }
    std::cout << "MOT-012 max reset position error " << worst << " m\n";
    expect(worst < 0.002f, "MOT-012 reset does not accumulate pose error");
    std::string error;
    const auto custom = std::filesystem::temp_directory_path() / "mortrauma-motors-custom.json";
    motors.setMode(mt::MotorMode::Compliant);
    motors.setJointActivation("knee_2", 0.4f, true);
    expect(motors.saveConfiguration(custom.string(), error), "MOT-012 saved a custom configuration");
    mt::MotorController loaded;
    loaded.bind(lab.world, lab.ragdoll);
    loaded.loadDefaultProfiles();
    expect(loaded.loadConfiguration(custom.string(), error), "MOT-012 loaded the custom configuration");
    expect(loaded.mode() == mt::MotorMode::Compliant, "MOT-012 loaded mode is compliant");
    expect(loaded.find("knee_2") != nullptr && std::fabs(loaded.find("knee_2")->activation - 0.4f) < 1e-4f, "MOT-012 loaded knee_2 activation");
    loaded.loadConfiguration(custom.string() + ".missing", error);
    expect(!error.empty(), "MOT-012 a missing configuration is rejected");
}

}  // namespace

void runMotorTest(const std::string& which) {
    if (which == "MOT-001") testMot001();
    else if (which == "MOT-002") testMot002();
    else if (which == "MOT-003") testMot003();
    else if (which == "MOT-004") testMot004();
    else if (which == "MOT-005") testMot005();
    else if (which == "MOT-006") testMot006();
    else if (which == "MOT-007") testMot007();
    else if (which == "MOT-008") testMot008();
    else if (which == "MOT-009") testMot009();
    else if (which == "MOT-010") testMot010();
    else if (which == "MOT-011") testMot011();
    else if (which == "MOT-012") testMot012();
    else {
        std::cerr << "unknown motor test " << which << '\n';
        ++gFailures;
    }
}

void runAllMotorTests() {
    testMot001();
    testMot002();
    testMot003();
    testMot004();
    testMot005();
    testMot006();
    testMot007();
    testMot008();
    testMot009();
    testMot010();
    testMot011();
    testMot012();
}
