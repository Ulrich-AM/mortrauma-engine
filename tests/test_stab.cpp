#include "mortrauma/assets/ModelLoader.hpp"
#include "mortrauma/balance/BalanceController.hpp"
#include "mortrauma/balance/StandingPose.hpp"
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
    int floor = -1;
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
    if (!lab.model.report.ok || lab.model.report.usedFallback || !config.error.empty() || !error.empty()) {
        std::cerr << lab.model.report.error << " " << config.error << " " << error << '\n';
        return false;
    }
    lab.ragdoll = mt::RagdollBuilder::build(lab.world, config, lab.model, preset);
    if (!lab.ragdoll.ok || !lab.motors.bind(lab.world, lab.ragdoll)) {
        std::cerr << lab.ragdoll.error << '\n';
        return false;
    }
    lab.motors.loadDefaultProfiles();
    if (!lab.balance.loadPreset("normal", error)) {
        return false;
    }
    mt::BodyCreateInfo floor;
    floor.name = "Floor";
    floor.isStatic = true;
    floor.shape = mt::BodyCreateInfo::Shape::Box;
    floor.halfExtents = {8.f, 0.1f, 8.f};
    floor.position = {0.f, -0.1f, 0.f};
    floor.friction = 0.9f;
    lab.floor = lab.world.createBody(floor);
    return lab.floor >= 0 && std::fabs(lab.world.settings().fixedTimestep - 1.f / 120.f) < 1e-6f;
}

bool place(Lab& lab) {
    const auto placement = mt::placeStandingPose(lab.world, lab.ragdoll, lab.balance.config());
    if (!placement.ok) {
        std::cerr << placement.error << '\n';
        return false;
    }
    lab.balance.setReference(placement.reference);
    lab.balance.resetState();
    return true;
}

void stepPassive(Lab& lab, int steps) {
    lab.balance.setEnabled(false, lab.motors);
    lab.motors.setMode(mt::MotorMode::Passive);
    lab.motors.clearMotorTask();
    const float dt = lab.world.settings().fixedTimestep;
    for (int i = 0; i < steps; ++i) {
        lab.balance.observe(lab.world, lab.ragdoll, dt);
        lab.motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
    }
}

float outsideBox(mt::Vec3 point, mt::Vec3 center, mt::Quat orient, mt::Vec3 half) {
    const mt::Vec3 local = mt::rotate(mt::conjugate(orient), point - center);
    return std::max(0.f, std::max(std::fabs(local.x) - half.x, std::max(std::fabs(local.y) - half.y, std::fabs(local.z) - half.z)));
}

void test001() {
    Lab lab;
    expect(loadLab(lab), "STAB-001 Humanoidguy loads");
    if (!lab.ragdoll.ok) {
        return;
    }
    expect(lab.ragdoll.bodies.size() == 21 && lab.ragdoll.joints.size() == 20 && lab.ragdoll.legsAttached, "STAB-001 topology");
    expect(lab.model.findBone("Pelvis") >= 0 && !lab.model.report.usedFallback, "STAB-001 skinned Humanoidguy, not a substitute");
    expect(std::fabs(lab.motors.profile(mt::MotorMode::Active).torqueFor("ankle_1") - 12.f) < 0.01f, "STAB-001 ankle limit is still 12 N·m");
}

void test002() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STAB-002 placed");
    if (!lab.ragdoll.ok) {
        return;
    }
    const mt::PosePalette palette = mt::posePalette(lab.world, lab.ragdoll, lab.model);
    bool aligned = true;
    for (const char* id : {"foot_1", "foot_2"}) {
        const auto* foot = lab.ragdoll.find(id);
        const auto view = lab.world.shapeView(foot->physics);
        const mt::Vec3 center = view.position + mt::rotate(view.rotation, view.centerOffset);
        const mt::Quat orient = mt::normalize(mt::mul(view.rotation, view.shapeRotation));
        mt::Vec3 drawnMin{1e9f, 1e9f, 1e9f};
        mt::Vec3 drawnMax{-1e9f, -1e9f, -1e9f};
        const float signs[2] = {-1.f, 1.f};
        for (float sx : signs) {
            for (float sy : signs) {
                for (float sz : signs) {
                    const mt::Vec3 corner = center + mt::rotate(orient, mt::Vec3{sx * view.halfExtents.x, sy * view.halfExtents.y, sz * view.halfExtents.z});
                    drawnMin.x = std::min(drawnMin.x, corner.x);
                    drawnMin.y = std::min(drawnMin.y, corner.y);
                    drawnMin.z = std::min(drawnMin.z, corner.z);
                    drawnMax.x = std::max(drawnMax.x, corner.x);
                    drawnMax.y = std::max(drawnMax.y, corner.y);
                    drawnMax.z = std::max(drawnMax.z, corner.z);
                }
            }
        }
        mt::Vec3 joltMin{};
        mt::Vec3 joltMax{};
        const bool bounds = lab.world.worldBounds(foot->physics, joltMin, joltMax);
        const float boundsMiss = std::max(std::fabs(drawnMin.x - joltMin.x),
                                          std::max(std::fabs(drawnMin.y - joltMin.y),
                                                   std::max(std::fabs(drawnMin.z - joltMin.z),
                                                            std::max(std::fabs(drawnMax.x - joltMax.x),
                                                                     std::max(std::fabs(drawnMax.y - joltMax.y), std::fabs(drawnMax.z - joltMax.z))))));
        float outside = 0.f;
        int samples = 0;
        for (const auto& mesh : lab.model.meshes) {
            bool owned = false;
            for (int bone : foot->bones) {
                owned = owned || mesh.bone == bone;
            }
            if (!owned || mesh.bone < 0 || mesh.bone >= int(palette.boneWorld.size())) {
                continue;
            }
            for (const auto& vertex : mesh.vertices) {
                const mt::Vec3 world = mt::transformPoint(palette.boneWorld[mesh.bone], vertex.position);
                outside = std::max(outside, outsideBox(world, center, orient, view.halfExtents));
                ++samples;
            }
        }
        std::cout << id << " samples " << samples << " meshOutside " << outside << " boundsMiss " << boundsMiss << " half "
                  << view.halfExtents.x << " " << view.halfExtents.y << " " << view.halfExtents.z << '\n';
        if (!bounds || samples < 50 || outside > 0.008f || boundsMiss > 0.002f) {
            aligned = false;
        }
        if (view.halfExtents.y > 0.22f || view.halfExtents.x > 0.12f) {
            aligned = false;
        }
    }
    expect(aligned, "STAB-002 foot boxes match the meshes and Jolt's bounds");
}

void test003() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STAB-003 placed");
    bool soles = true;
    float left = 0.f;
    float right = 0.f;
    for (const char* id : {"foot_1", "foot_2"}) {
        const auto* foot = lab.ragdoll.find(id);
        const auto state = lab.world.bodyState(foot->physics);
        const mt::Vec3 up = mt::normalize(mt::rotate(state.rotation, foot->soleNormalLocal));
        const mt::Vec3 heel = state.position + mt::rotate(state.rotation, foot->heelLocal);
        const mt::Vec3 toe = state.position + mt::rotate(state.rotation, foot->forefootLocal);
        const float mean = 0.5f * (heel.y + toe.y);
        if (id[5] == '1') {
            right = mean;
        } else {
            left = mean;
        }
        std::cout << id << " soleY " << up.y << " heel " << heel.y << " toe " << toe.y << '\n';
        if (up.y < 0.98f || std::fabs(heel.y - toe.y) > 0.015f || heel.y < -0.005f || heel.y > 0.03f) {
            soles = false;
        }
    }
    expect(soles && std::fabs(left - right) < 0.02f, "STAB-003 both soles are level and near the floor");
}

void test004() {
    Lab lab;
    expect(loadLab(lab), "STAB-004 built");
    bool ankles = true;
    for (const auto& joint : lab.ragdoll.joints) {
        if (joint.id != "ankle_1" && joint.id != "ankle_2") {
            continue;
        }
        const auto state = lab.world.jointState(joint.physics);
        const auto* foot = lab.ragdoll.find(lab.ragdoll.bodies[joint.child].id);
        const float along = mt::length(state.anchorParentWorld - foot->segmentStart);
        std::cout << joint.id << " separation " << state.separation << " anchorToSegmentStart " << along << '\n';
        if (!state.positionLocked || state.separation > 0.005f || joint.base.swing2Min >= joint.base.swing2Max) {
            ankles = false;
        }
        if (along > 0.08f) {
            ankles = false;
        }
    }
    expect(ankles, "STAB-004 ankles stay locked to the feet");
}

void test005() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STAB-005 placed");
    lab.balance.observe(lab.world, lab.ragdoll, lab.world.settings().fixedTimestep);
    mt::Vec3 com{};
    float mass = 0.f;
    for (const auto& body : lab.ragdoll.bodies) {
        const auto state = lab.world.bodyState(body.physics);
        com = com + state.position * body.mass;
        mass += body.mass;
    }
    com = com * (1.f / mass);
    const auto& state = lab.balance.state();
    const float miss = mt::length(state.com - com);
    std::cout << "com miss " << miss << " mass " << mass << " estimator " << state.totalMass << '\n';
    expect(miss < 0.002f && std::fabs(state.totalMass - mass) < 0.05f && state.com.y > 0.8f && state.com.y < 1.4f,
           "STAB-005 COM matches the rigid-body masses");
}

struct Effort {
    float maxAnkle = 0.f;
    float maxHip = 0.f;
    float maxOver = 0.f;
    int saturated = 0;
    int steps = 0;
    float minComY = 1e9f;
    float maxTilt = 0.f;
    bool finite = true;
    bool support = false;
};

Effort runActive(Lab& lab, int steps) {
    Effort effort;
    lab.balance.setEnabled(true, lab.motors);
    const float dt = lab.world.settings().fixedTimestep;
    for (int i = 0; i < steps; ++i) {
        lab.balance.observe(lab.world, lab.ragdoll, dt);
        lab.balance.command(lab.ragdoll, lab.motors);
        lab.motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
        const auto& state = lab.balance.state();
        effort.finite = effort.finite && state.finite;
        effort.minComY = std::min(effort.minComY, state.com.y);
        effort.maxTilt = std::max(effort.maxTilt, state.torsoTilt);
        effort.maxAnkle = std::max(effort.maxAnkle, state.ankleEffort);
        effort.maxHip = std::max(effort.maxHip, state.hipEffort);
        effort.saturated += state.saturatedMotors > 0 ? 1 : 0;
        effort.steps += 1;
        effort.support = effort.support || lab.world.supportActive();
        for (const auto& joint : lab.ragdoll.joints) {
            const auto sample = lab.world.jointState(joint.physics);
            if (!sample.valid || sample.motorTorqueLimit <= 0.f) {
                continue;
            }
            const float peak = std::max(std::fabs(sample.motorTorque.x), std::max(std::fabs(sample.motorTorque.y), std::fabs(sample.motorTorque.z)));
            effort.maxOver = std::max(effort.maxOver, peak - sample.motorTorqueLimit);
        }
    }
    return effort;
}

void test006() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STAB-006 placed");
    const Effort effort = runActive(lab, 120);
    std::cout << "ankle " << effort.maxAnkle << " hip " << effort.maxHip << " over " << effort.maxOver << " satSteps " << effort.saturated << '\n';
    expect(effort.finite && effort.maxOver < 0.75f, "STAB-006 commands stay inside the torque limits");
    expect(!effort.support, "STAB-006 no support fixture while balancing");
}

void test007() {
    mt::BalanceConfig config;
    const auto forward = mt::balanceCorrection(config, 0.04f, 0.f, 0.f, 0.f, true, true);
    const auto none = mt::balanceCorrection(config, 0.04f, 0.f, 0.f, 0.f, false, true);
    expect(forward.ankleSwing2 < 0.f && forward.hipSwing2 < 0.f, "STAB-007 a forward COM error commands extension");
    expect(std::fabs(none.ankleSwing2) < 1e-6f, "STAB-007 ankle correction is off without foot authority");
}

void test008() {
    mt::BalanceConfig config;
    const auto backward = mt::balanceCorrection(config, -0.04f, 0.f, 0.f, 0.f, true, true);
    expect(backward.ankleSwing2 > 0.f && backward.hipSwing2 > 0.f, "STAB-008 a backward COM error commands flexion");
    expect(backward.ankleSwing2 * mt::balanceCorrection(config, 0.04f, 0.f, 0.f, 0.f, true, false).ankleSwing2 < 0.f,
           "STAB-008 forward and backward ankle signs oppose");
}

void test009() {
    mt::BalanceConfig config;
    const auto right = mt::balanceCorrection(config, 0.f, 0.04f, 0.f, 0.f, true, true);
    const auto left = mt::balanceCorrection(config, 0.f, -0.04f, 0.f, 0.f, true, true);
    expect(right.ankleSwing1 < 0.f && right.hipSwing1 < 0.f, "STAB-009 a rightward error commands negative swing1");
    expect(left.ankleSwing1 > 0.f && left.hipSwing1 > 0.f, "STAB-009 a leftward error commands positive swing1");
    expect(right.ankleSwing1 * left.ankleSwing1 < 0.f, "STAB-009 the lateral ankle signs oppose");
}

void test010() {
    Lab passive;
    Lab active;
    expect(loadLab(passive) && place(passive) && loadLab(active) && place(active), "STAB-010 both laboratories");
    stepPassive(passive, 180);
    const Effort effort = runActive(active, 180);
    const float passiveY = passive.balance.state().com.y;
    std::cout << "passive comY " << passiveY << " tilt " << passive.balance.state().torsoTilt << " active minY " << effort.minComY << " tilt "
              << effort.maxTilt << '\n';
    expect(effort.minComY > passiveY + 0.05f, "STAB-010 active balance holds the body higher than a passive fall");
}

void test011() {
    Lab a;
    Lab b;
    expect(loadLab(a) && place(a) && loadLab(b) && place(b), "STAB-011 both runs");
    runActive(a, 120);
    runActive(b, 120);
    a.balance.observe(a.world, a.ragdoll, a.world.settings().fixedTimestep);
    b.balance.observe(b.world, b.ragdoll, b.world.settings().fixedTimestep);
    const float miss = mt::length(a.balance.state().com - b.balance.state().com);
    std::cout << "replay miss " << miss << '\n';
    expect(miss < 0.001f, "STAB-011 the same stand repeats");
}

void test012() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STAB-012 placed");
    lab.balance.setEnabled(true, lab.motors);
    const float dt = lab.world.settings().fixedTimestep;
    for (int i = 0; i < 30; ++i) {
        lab.balance.observe(lab.world, lab.ragdoll, dt);
        lab.balance.command(lab.ragdoll, lab.motors);
        lab.motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
    }
    const auto* chest = lab.ragdoll.find("chest");
    const auto chestState = lab.world.bodyState(chest->physics);
    lab.world.addImpulse(chest->physics, lab.balance.state().facing * 55.f, chestState.position);
    bool fell = false;
    bool finite = true;
    for (int i = 0; i < 240; ++i) {
        lab.balance.observe(lab.world, lab.ragdoll, dt);
        lab.balance.command(lab.ragdoll, lab.motors);
        lab.motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
        const auto& state = lab.balance.state();
        finite = finite && state.finite;
        if (state.phase == mt::BalancePhase::Falling || state.com.y < 0.7f || state.torsoTilt > 0.7f) {
            fell = true;
            break;
        }
    }
    std::cout << "fell " << fell << " comY " << lab.balance.state().com.y << " tilt " << lab.balance.state().torsoTilt << '\n';
    expect(finite && fell && !lab.world.supportActive(), "STAB-012 a 55 N·s push falls without a fixture");
}

void test013() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STAB-013 placed");
    runActive(lab, 60);
    expect(!lab.world.supportActive(), "STAB-013 balance does not enable a support fixture");
    for (const auto& joint : lab.ragdoll.joints) {
        const auto state = lab.world.jointState(joint.physics);
        expect(state.motorTorqueLimit < 500.f, "STAB-013 " + joint.id + " torque is finite");
    }
}

void test014() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STAB-014 placed");
    const auto* pelvis = lab.ragdoll.find("pelvis");
    const auto state = lab.world.bodyState(pelvis->physics);
    const int grab = lab.world.beginGrab(pelvis->physics, state.position, state.position + mt::Vec3{0.f, 0.15f, 0.f}, 8.f, 1.f);
    expect(grab >= 0, "STAB-014 grab attaches");
    const float dt = lab.world.settings().fixedTimestep;
    bool finite = true;
    for (int i = 0; i < 30; ++i) {
        lab.world.step(dt, 1);
        const auto sample = lab.world.bodyState(pelvis->physics);
        finite = finite && sample.valid && mt::finite(sample.position);
    }
    lab.world.endGrab(grab);
    const auto* chest = lab.ragdoll.find("chest");
    const auto chestState = lab.world.bodyState(chest->physics);
    lab.world.addImpulse(chest->physics, {0.f, 0.f, 4.f}, chestState.position);
    lab.world.step(dt, 1);
    const auto after = lab.world.bodyState(chest->physics);
    expect(finite && after.valid && mt::finite(after.linearVelocity) && !lab.world.supportActive(), "STAB-014 grab and push stay finite");
}

void test015() {
    Lab lab;
    expect(loadLab(lab) && place(lab), "STAB-015 Humanoidguy still places a stand");
    expect(lab.ragdoll.bodies.size() == 21 && lab.ragdoll.joints.size() == 20, "STAB-015 earlier body count");
    const auto* foot = lab.ragdoll.find("foot_1");
    expect(foot != nullptr && std::fabs(foot->mass - 1.3f) < 0.01f && foot->collider == mt::BodyCreateInfo::Shape::Box, "STAB-015 foot mass and box");
    expect(!lab.balance.config().equilibriumReference == false, "STAB-015 the ankle equilibrium reference stays on");
}

void printJoints(const Lab& lab) {
    const auto& state = lab.balance.state();
    std::cout << " t " << state.stoodSeconds << " phase " << mt::balancePhaseName(state.phase) << " y " << state.com.y << " tilt " << state.torsoTilt
              << " coplanar " << state.coplanarityM << " moment " << state.estimatedAnkleMoment << " why " << state.instability << " contacts "
              << state.leftHeel << "/" << state.leftFore << " " << state.rightHeel << "/" << state.rightFore << '\n';
    for (const char* id : {"ankle_1", "ankle_2", "hip_1", "hip_2"}) {
        for (const auto& joint : lab.ragdoll.joints) {
            if (joint.id != id) {
                continue;
            }
            const auto sample = lab.world.jointState(joint.physics);
            const mt::Vec3 error = mt::constraintEulerRadians(mt::mul(mt::conjugate(sample.constraintRotation), sample.motorTarget));
            std::cout << "  " << id << " torque " << sample.motorTorque.x << " " << sample.motorTorque.y << " " << sample.motorTorque.z << " errDeg "
                      << mt::degrees(error.x) << " " << mt::degrees(error.y) << " " << mt::degrees(error.z) << " sat " << sample.motorSaturated << '\n';
        }
    }
}

void logStand() {
    Lab lab;
    if (!loadLab(lab) || !place(lab)) {
        ++gFailures;
        return;
    }
    lab.balance.setEnabled(true, lab.motors);
    const float dt = lab.world.settings().fixedTimestep;
    lab.balance.observe(lab.world, lab.ragdoll, dt);
    const auto origin = lab.balance.state();
    std::cout << "initial moment " << origin.estimatedAnkleMoment << " coplanar " << origin.coplanarityM << " why " << origin.instability << '\n';
    float maxSlip = 0.f;
    for (int i = 0; i < 30 * 120; ++i) {
        lab.balance.observe(lab.world, lab.ragdoll, dt);
        lab.balance.command(lab.ragdoll, lab.motors);
        lab.motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
        for (const char* id : {"foot_1", "foot_2"}) {
            const auto sample = lab.world.bodyState(lab.ragdoll.find(id)->physics);
            maxSlip = std::max(maxSlip, std::hypot(sample.linearVelocity.x, sample.linearVelocity.z));
        }
        if ((i + 1) % 120 == 0 && ((i + 1) / 120 <= 8 || (i + 1) / 120 == 15 || (i + 1) / 120 == 30)) {
            printJoints(lab);
        }
    }
    const auto& state = lab.balance.state();
    const float drift = std::hypot(state.com.x - origin.com.x, state.com.z - origin.com.z);
    std::cout << "30s drift " << drift << " maxFootSlip " << maxSlip << " support " << lab.world.supportActive() << '\n';

    Lab hold;
    if (!loadLab(hold)) {
        ++gFailures;
        return;
    }
    const auto placement = mt::placeStandingPose(hold.world, hold.ragdoll, hold.balance.config());
    if (!placement.ok) {
        ++gFailures;
        return;
    }
    hold.balance.setReference(placement.reference);
    hold.balance.resetState();
    hold.motors.setMode(mt::MotorMode::Active);
    std::vector<mt::MotorTaskCommand> task = placement.reference;
    for (auto& entry : task) {
        entry.activation = 1.f;
        const bool ankle = entry.jointId.rfind("ankle_", 0) == 0;
        const bool weight = entry.jointId.rfind("hip_", 0) == 0 || entry.jointId.rfind("knee_", 0) == 0 || entry.jointId.rfind("spine_", 0) == 0;
        entry.frequencyHz = weight ? 80.f : (ankle ? 40.f : 4.f);
    }
    const float holdDt = hold.world.settings().fixedTimestep;
    float holdMinY = 1e9f;
    for (int i = 0; i < 5 * 120; ++i) {
        hold.motors.setMotorTask(task);
        hold.motors.apply(hold.world, hold.ragdoll);
        hold.world.step(holdDt, 1);
        hold.balance.observe(hold.world, hold.ragdoll, holdDt);
        holdMinY = std::min(holdMinY, hold.balance.state().com.y);
    }
    std::cout << "hold5 minY " << holdMinY << " tilt " << hold.balance.state().torsoTilt << " phase " << mt::balancePhaseName(hold.balance.state().phase)
              << '\n';

    for (const mt::Vec3 impulse : {mt::Vec3{0.f, 0.f, -6.f}, {0.f, 0.f, 6.f}, {-6.f, 0.f, 0.f}, {6.f, 0.f, 0.f}}) {
        Lab push;
        if (!loadLab(push) || !place(push)) {
            ++gFailures;
            return;
        }
        push.balance.setEnabled(true, push.motors);
        const float pushDt = push.world.settings().fixedTimestep;
        for (int i = 0; i < 30; ++i) {
            push.balance.observe(push.world, push.ragdoll, pushDt);
            push.balance.command(push.ragdoll, push.motors);
            push.motors.apply(push.world, push.ragdoll);
            push.world.step(pushDt, 1);
        }
        const auto* chest = push.ragdoll.find("chest");
        const auto chestState = push.world.bodyState(chest->physics);
        push.world.addImpulse(chest->physics, impulse, chestState.position);
        bool fell = false;
        for (int i = 0; i < 3 * 120; ++i) {
            push.balance.observe(push.world, push.ragdoll, pushDt);
            push.balance.command(push.ragdoll, push.motors);
            push.motors.apply(push.world, push.ragdoll);
            push.world.step(pushDt, 1);
            const auto& sample = push.balance.state();
            if (sample.phase == mt::BalancePhase::Falling || sample.com.y < 0.75f || sample.torsoTilt > 0.7f) {
                fell = true;
                break;
            }
        }
        std::cout << "push " << impulse.x << " " << impulse.z << " fell " << fell << " y " << push.balance.state().com.y << " tilt "
                  << push.balance.state().torsoTilt << " stood " << push.balance.state().stoodSeconds << '\n';
    }
}

}  // namespace

void runStabTest(const std::string& which) {
    if (which == "STAB-001") test001();
    else if (which == "STAB-002") test002();
    else if (which == "STAB-003") test003();
    else if (which == "STAB-004") test004();
    else if (which == "STAB-005") test005();
    else if (which == "STAB-006") test006();
    else if (which == "STAB-007") test007();
    else if (which == "STAB-008") test008();
    else if (which == "STAB-009") test009();
    else if (which == "STAB-010") test010();
    else if (which == "STAB-011") test011();
    else if (which == "STAB-012") test012();
    else if (which == "STAB-013") test013();
    else if (which == "STAB-014") test014();
    else if (which == "STAB-015") test015();
    else if (which == "STAB-LOG") logStand();
    else {
        std::cerr << "unknown test " << which << '\n';
        ++gFailures;
    }
}

void runAllStabTests() {
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
}
