#include "mortrauma/assets/ModelLoader.hpp"
#include "mortrauma/balance/BalanceController.hpp"
#include "mortrauma/balance/StandingPose.hpp"
#include "mortrauma/core/Paths.hpp"
#include "mortrauma/motors/MotorController.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

extern int gFailures;
void expect(bool condition, const std::string& message);

namespace {

bool ybotFilePresent() {
    const std::filesystem::path bundled = std::filesystem::path(MORTRAUMA_SOURCE_DIR) / "assets" / "characters" / "ybot.fbx";
    return std::filesystem::exists(bundled);
}

bool skipWithoutModel(const char* id) {
    if (ybotFilePresent()) {
        return false;
    }
    std::cout << "skip: " << id << " (assets/characters/ybot.fbx is not in this checkout)\n";
    return true;
}

struct Lab {
    mt::PhysicsWorld world;
    mt::ImportedModel model;
    mt::CharacterConfig config;
    mt::Ragdoll ragdoll;
    mt::MotorController motors;
    mt::BalanceController balance;
    int floor = -1;
};

bool buildCharacter(Lab& lab, const std::string& rigRelative, const std::string& modelRelative, const std::string& balancePreset,
                    const std::string& motorDirectory, bool withFloor) {
    mt::PhysicsSettings settings;
    settings.allowSleeping = true;
    if (!lab.world.initialize(settings)) {
        return false;
    }
    const auto rigPath = mt::findAsset(rigRelative);
    const auto modelPath = mt::findAsset(modelRelative);
    if (rigPath.empty() || modelPath.empty()) {
        return false;
    }
    lab.config = mt::loadCharacterConfig(rigPath.string());
    lab.model = mt::ModelLoader::load(modelPath.string());
    std::string error;
    const auto preset = mt::loadJointPreset(mt::findAsset("configurations/presets/natural.json").string(), error);
    if (!lab.model.report.ok || lab.model.report.usedFallback || !lab.config.error.empty() || !error.empty()) {
        std::cerr << "load failed " << lab.model.report.error << " " << lab.config.error << " " << error << '\n';
        return false;
    }
    lab.ragdoll = mt::RagdollBuilder::build(lab.world, lab.config, lab.model, preset);
    if (!lab.ragdoll.ok) {
        std::cerr << "ragdoll failed " << lab.ragdoll.error << '\n';
        return false;
    }
    if (!lab.motors.bind(lab.world, lab.ragdoll)) {
        return false;
    }
    lab.motors.loadProfilesFrom(motorDirectory);
    if (!lab.balance.loadPreset(balancePreset, error)) {
        std::cerr << error << '\n';
        return false;
    }
    if (withFloor) {
        mt::BodyCreateInfo floor;
        floor.name = "Floor";
        floor.isStatic = true;
        floor.shape = mt::BodyCreateInfo::Shape::Box;
        floor.halfExtents = {8.f, 0.1f, 8.f};
        floor.position = {0.f, -0.1f, 0.f};
        floor.friction = 0.9f;
        lab.floor = lab.world.createBody(floor);
    }
    return !withFloor || lab.floor >= 0;
}

const mt::RagdollJoint* findJoint(const mt::Ragdoll& ragdoll, const std::string& id) {
    for (const auto& joint : ragdoll.joints) {
        if (joint.id == id) {
            return &joint;
        }
    }
    return nullptr;
}

void test001() {
    if (skipWithoutModel("YBOT-001")) {
        return;
    }
    const auto model = mt::ModelLoader::load(mt::findAsset("characters/ybot.fbx").string());
    expect(model.report.ok && !model.report.usedFallback, "YBOT-001 import succeeds");
    expect(model.report.boneCount > 20 && model.report.meshCount > 10 && model.report.vertexCount > 1000, "YBOT-001 has bones and meshes");
    expect(std::fabs(model.report.unitScale - 0.01f) < 1e-4f, "YBOT-001 centimeter scale");
    expect(model.report.upAxis.find('Y') != std::string::npos, "YBOT-001 Y up");
    expect(model.findBone("Cadera") >= 0 && model.findBone("Tobillo") >= 0 && model.findBone("Muñeca") >= 0, "YBOT-001 named pivots");
}

void test002() {
    if (skipWithoutModel("YBOT-002")) {
        return;
    }
    const auto model = mt::ModelLoader::load(mt::findAsset("characters/ybot.fbx").string());
    const auto config = mt::loadCharacterConfig(mt::findAsset("configurations/ybot.json").string());
    expect(model.report.ok && config.error.empty(), "YBOT-002 config and model load");
    const char* required[] = {"Cadera", "Cintura", "Plexo", "Cabeza", "Cuello", "Humero", "Humero_2", "Cubito", "Cubito_2",
                              "Palma", "Palma_2", "Muslo", "Muslo_2", "Tibia", "Tibia_2", "Pie", "Pie_2", "Dedos", "Dedos_2",
                              "Tobillo", "Tobillo_2", "Rodilla", "Rodilla_2", "Cadera1", "Cadera2", "Dedos_3", "Dedos_4",
                              "Hombro", "Hombro_2", "Codo", "Codo_2", "Muñeca", "Palma_3", "Plexo_2", "Cadera_2"};
    bool mapped = true;
    for (const char* name : required) {
        if (model.findBone(name) < 0) {
            std::cerr << "missing bone " << name << '\n';
            mapped = false;
        }
    }
    for (const auto& body : config.bodies) {
        if (model.findBone(body.primaryBone) < 0) {
            mapped = false;
        }
        if (!body.endBone.empty() && model.findBone(body.endBone) < 0) {
            mapped = false;
        }
        for (const auto& bone : body.bones) {
            if (model.findBone(bone) < 0) {
                mapped = false;
            }
        }
    }
    expect(mapped, "YBOT-002 skeleton mappings resolve");
    bool driven = true;
    for (const auto& mesh : model.meshes) {
        if (mesh.bone < 0 || mesh.bone >= int(model.bones.size())) {
            driven = false;
        }
    }
    expect(driven && !model.meshes.empty(), "YBOT-002 every imported mesh names a bone");
}

void test003() {
    if (skipWithoutModel("YBOT-003")) {
        return;
    }
    Lab lab;
    expect(buildCharacter(lab, "configurations/ybot.json", "characters/ybot.fbx", "ybot", "configurations/motors/ybot", true), "YBOT-003 builds");
    if (!lab.ragdoll.ok) {
        return;
    }
    const char* bodies[] = {"pelvis", "stomach", "chest", "neck", "head", "upper_arm_1", "upper_arm_2", "forearm_1", "forearm_2",
                            "hand_1", "hand_2", "thigh_1", "thigh_2", "calf_1", "calf_2", "foot_1", "foot_2"};
    bool present = true;
    for (const char* id : bodies) {
        present = present && lab.ragdoll.find(id) != nullptr;
    }
    const char* joints[] = {"spine_stomach", "spine_chest", "spine_neck", "neck", "shoulder_1", "shoulder_2", "elbow_1", "elbow_2",
                            "wrist_1", "wrist_2", "hip_1", "hip_2", "knee_1", "knee_2", "ankle_1", "ankle_2"};
    for (const char* id : joints) {
        present = present && findJoint(lab.ragdoll, id) != nullptr;
    }
    expect(present, "YBOT-003 required bodies and joints exist");
    expect(lab.ragdoll.legsAttached && lab.ragdoll.componentCount() == 1, "YBOT-003 one component and both legs reach the pelvis");
    expect(lab.ragdoll.find("hip_1") == nullptr && lab.ragdoll.find("clavicle_1") == nullptr, "YBOT-003 no separate hip or clavicle body");
    float mass = 0.f;
    for (const auto& body : lab.ragdoll.bodies) {
        mass += body.mass;
    }
    expect(mass > 60.f && mass < 80.f, "YBOT-003 mass is a body-sized total, not Humanoidguy's table copied by accident");
    std::cout << "ybot bodies " << lab.ragdoll.bodies.size() << " joints " << lab.ragdoll.joints.size() << " mass " << mass << '\n';
}

void test004() {
    if (skipWithoutModel("YBOT-004")) {
        return;
    }
    Lab lab;
    if (!buildCharacter(lab, "configurations/ybot.json", "characters/ybot.fbx", "ybot", "configurations/motors/ybot", false)) {
        expect(false, "YBOT-004 builds");
        return;
    }
    bool limits = true;
    for (const auto& joint : lab.ragdoll.joints) {
        const auto& base = joint.base;
        if (!(base.twistMin < base.twistMax && base.swing1Min < base.swing1Max && base.swing2Min < base.swing2Max)) {
            limits = false;
        }
        if (!std::isfinite(base.twistMin) || !std::isfinite(base.swing2Max)) {
            limits = false;
        }
    }
    const auto* knee = findJoint(lab.ragdoll, "knee_1");
    const auto* elbow = findJoint(lab.ragdoll, "elbow_1");
    const auto* hip = findJoint(lab.ragdoll, "hip_1");
    const auto* ankle = findJoint(lab.ragdoll, "ankle_1");
    expect(knee != nullptr && knee->base.swing2Min >= -0.02f && knee->base.swing2Max > 2.0f, "YBOT-004 knee flexion range");
    expect(elbow != nullptr && elbow->base.swing2Min > -0.25f && elbow->base.swing2Min < 0.05f && elbow->base.swing2Max > 2.0f,
           "YBOT-004 elbow flexion range");
    expect(hip != nullptr && hip->base.swing2Max > 1.0f && hip->base.swing2Min < 0.2f, "YBOT-004 hip flexion range");
    expect(ankle != nullptr && ankle->base.swing2Min < 0.f && ankle->base.swing2Max > 0.f, "YBOT-004 ankle crosses the level sole");
    expect(limits, "YBOT-004 every joint limit is ordered and finite");
}

void test005() {
    if (skipWithoutModel("YBOT-005")) {
        return;
    }
    Lab lab;
    if (!buildCharacter(lab, "configurations/ybot.json", "characters/ybot.fbx", "ybot", "configurations/motors/ybot", true)) {
        expect(false, "YBOT-005 builds");
        return;
    }
    const auto placement = mt::placeStandingPose(lab.world, lab.ragdoll, lab.balance.config());
    expect(placement.ok, "YBOT-005 standing placement");
    bool feet = true;
    for (const char* id : {"foot_1", "foot_2"}) {
        const auto* foot = lab.ragdoll.find(id);
        if (foot == nullptr || foot->collider != mt::BodyCreateInfo::Shape::Box) {
            feet = false;
            continue;
        }
        const auto state = lab.world.bodyState(foot->physics);
        const mt::Vec3 sole = mt::rotate(state.rotation, foot->soleNormalLocal);
        const mt::Vec3 heel = state.position + mt::rotate(state.rotation, foot->heelLocal);
        const mt::Vec3 fore = state.position + mt::rotate(state.rotation, foot->forefootLocal);
        std::cout << id << " half " << foot->boxHalfExtents.x << " " << foot->boxHalfExtents.y << " " << foot->boxHalfExtents.z
                  << " sole " << sole.x << " " << sole.y << " " << sole.z << " heelY " << heel.y << " foreY " << fore.y << '\n';
        if (sole.y < 0.8f) {
            feet = false;
        }
        if (foot->boxHalfExtents.y < foot->boxHalfExtents.z) {
            feet = false;
        }
        if (foot->boxHalfExtents.x < 0.03f || foot->boxHalfExtents.y < 0.06f || foot->boxHalfExtents.y > 0.25f) {
            feet = false;
        }
        if (std::fabs(heel.y - fore.y) > 0.03f || heel.y < -0.01f || heel.y > 0.05f) {
            feet = false;
        }
    }
    expect(feet, "YBOT-005 flat foot boxes sit on the ground");
}

void test006() {
    if (skipWithoutModel("YBOT-006")) {
        return;
    }
    Lab lab;
    if (!buildCharacter(lab, "configurations/ybot.json", "characters/ybot.fbx", "ybot", "configurations/motors/ybot", true)) {
        expect(false, "YBOT-006 builds");
        return;
    }
    const auto& active = lab.motors.profile(mt::MotorMode::Active);
    expect(std::fabs(active.torqueFor("ankle_1") - 11.f) < 0.01f, "YBOT-006 ankle torque is the Y-Bot limit");
    expect(std::fabs(active.torqueFor("hip_1") - 45.f) < 0.01f, "YBOT-006 hip torque is the Y-Bot limit");
    expect(std::fabs(active.torqueFor("ankle_1") - 12.f) > 0.1f, "YBOT-006 ankle torque is not Humanoidguy's 12 N m");
    lab.motors.setMode(mt::MotorMode::Active);
    lab.motors.setAllActivations(1.f, true);
    const auto placement = mt::placeStandingPose(lab.world, lab.ragdoll, lab.balance.config());
    lab.balance.setReference(placement.reference);
    expect(placement.ok, "YBOT-006 pose for the motor step");
    lab.balance.setEnabled(true, lab.motors);
    const float dt = lab.world.settings().fixedTimestep;
    bool finite = true;
    for (int i = 0; i < 30; ++i) {
        lab.balance.observe(lab.world, lab.ragdoll, dt);
        lab.balance.command(lab.ragdoll, lab.motors);
        lab.motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
        for (const auto& body : lab.ragdoll.bodies) {
            const auto state = lab.world.bodyState(body.physics);
            finite = finite && state.valid && mt::finite(state.position) && mt::finite(state.linearVelocity);
        }
    }
    expect(finite, "YBOT-006 motor step stays finite");
    expect(!lab.world.supportActive(), "YBOT-006 no hidden support fixture");
}

void test007() {
    if (skipWithoutModel("YBOT-007")) {
        return;
    }
    Lab lab;
    if (!buildCharacter(lab, "configurations/ybot.json", "characters/ybot.fbx", "ybot", "configurations/motors/ybot", true)) {
        expect(false, "YBOT-007 builds");
        return;
    }
    const auto placement = mt::placeStandingPose(lab.world, lab.ragdoll, lab.balance.config());
    expect(placement.ok, "YBOT-007 standing initialization");
    expect(placement.maxSeparation < 0.02f, "YBOT-007 joint anchors meet");
    const auto* hand = lab.ragdoll.find("hand_1");
    const auto* shoulder = lab.ragdoll.find("upper_arm_1");
    if (hand != nullptr && shoulder != nullptr) {
        const float handY = lab.world.bodyState(hand->physics).position.y;
        const float shoulderY = lab.world.bodyState(shoulder->physics).position.y;
        std::cout << "hand y " << handY << " shoulder y " << shoulderY << '\n';
        expect(handY < shoulderY - 0.15f, "YBOT-007 arms are below the T-pose");
    } else {
        expect(false, "YBOT-007 arm bodies");
    }
    lab.balance.observe(lab.world, lab.ragdoll, lab.world.settings().fixedTimestep);
    const auto& state = lab.balance.state();
    expect(state.finite && state.totalMass > 60.f, "YBOT-007 estimator reads the new rig");
    std::cout << "com " << state.com.x << " " << state.com.y << " " << state.com.z << " target " << state.equilibriumTarget.x << " "
              << state.equilibriumTarget.z << " tilt " << state.torsoTilt << '\n';
}

void test008() {
    if (skipWithoutModel("YBOT-008")) {
        return;
    }
    Lab lab;
    if (!buildCharacter(lab, "configurations/ybot.json", "characters/ybot.fbx", "ybot", "configurations/motors/ybot", true)) {
        expect(false, "YBOT-008 builds");
        return;
    }
    const auto placement = mt::placeStandingPose(lab.world, lab.ragdoll, lab.balance.config());
    expect(placement.ok, "YBOT-008 pose");
    lab.balance.setReference(placement.reference);
    lab.balance.setEnabled(true, lab.motors);
    const float dt = lab.world.settings().fixedTimestep;
    bool finite = true;
    float maxSeparation = 0.f;
    for (int i = 0; i < 240; ++i) {
        lab.balance.observe(lab.world, lab.ragdoll, dt);
        lab.balance.command(lab.ragdoll, lab.motors);
        lab.motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
        const auto& state = lab.balance.state();
        finite = finite && state.finite && state.com.y > -1.f && state.com.y < 5.f;
        for (const auto& joint : lab.ragdoll.joints) {
            const auto sample = lab.world.jointState(joint.physics);
            if (sample.valid) {
                maxSeparation = std::max(maxSeparation, sample.separation);
            }
        }
    }
    expect(finite, "YBOT-008 balance stays numerically finite");
    expect(maxSeparation < 0.05f, "YBOT-008 constraint separation stays bounded");
    expect(!lab.world.supportActive(), "YBOT-008 remains fully dynamic");
}

void test009() {
    if (skipWithoutModel("YBOT-009")) {
        return;
    }
    Lab lab;
    expect(buildCharacter(lab, "configurations/humanoidguy.json", "characters/humanoidguy_rigged.fbx", "normal", "configurations/motors", true),
           "YBOT-009 starts on Humanoidguy");
    std::vector<int> oldBodies;
    for (const auto& body : lab.ragdoll.bodies) {
        oldBodies.push_back(body.physics);
    }
    const int oldCount = int(lab.ragdoll.bodies.size());
    mt::destroyRagdoll(lab.world, lab.ragdoll);
    bool dead = true;
    for (int index : oldBodies) {
        const auto state = lab.world.bodyState(index);
        dead = dead && !state.valid;
    }
    expect(dead && lab.ragdoll.bodies.empty(), "YBOT-009 Humanoidguy bodies are destroyed");
    const auto rig = mt::findAsset("configurations/ybot.json");
    const auto modelPath = mt::findAsset("characters/ybot.fbx");
    std::string error;
    const auto preset = mt::loadJointPreset(mt::findAsset("configurations/presets/natural.json").string(), error);
    lab.config = mt::loadCharacterConfig(rig.string());
    lab.model = mt::ModelLoader::load(modelPath.string());
    lab.ragdoll = mt::RagdollBuilder::build(lab.world, lab.config, lab.model, preset);
    expect(lab.ragdoll.ok && lab.motors.bind(lab.world, lab.ragdoll), "YBOT-009 Y-Bot is created after the destroy");
    lab.motors.loadProfilesFrom("configurations/motors/ybot");
    expect(lab.balance.loadPreset("ybot", error), "YBOT-009 Y-Bot balance preset binds");
    const auto placement = mt::placeStandingPose(lab.world, lab.ragdoll, lab.balance.config());
    expect(placement.ok, "YBOT-009 Y-Bot standing reset");
    std::vector<int> ybotBodies;
    for (const auto& body : lab.ragdoll.bodies) {
        ybotBodies.push_back(body.physics);
    }
    mt::destroyRagdoll(lab.world, lab.ragdoll);
    dead = true;
    for (int index : ybotBodies) {
        dead = dead && !lab.world.bodyState(index).valid;
    }
    expect(dead, "YBOT-009 Y-Bot bodies are destroyed");
    lab.config = mt::loadCharacterConfig(mt::findAsset("configurations/humanoidguy.json").string());
    lab.model = mt::ModelLoader::load(mt::findAsset("characters/humanoidguy_rigged.fbx").string());
    lab.ragdoll = mt::RagdollBuilder::build(lab.world, lab.config, lab.model, preset);
    expect(lab.ragdoll.ok && int(lab.ragdoll.bodies.size()) == oldCount && lab.ragdoll.legsAttached, "YBOT-009 Humanoidguy is recreated");
    const auto floor = lab.world.bodyState(lab.floor);
    expect(floor.valid, "YBOT-009 the floor survives the switch");
}

void test010() {
    Lab lab;
    expect(buildCharacter(lab, "configurations/humanoidguy.json", "characters/humanoidguy_rigged.fbx", "normal", "configurations/motors", true),
           "YBOT-010 Humanoidguy still builds");
    if (!lab.ragdoll.ok) {
        return;
    }
    expect(lab.ragdoll.bodies.size() == 21 && lab.ragdoll.joints.size() == 20 && lab.ragdoll.legsAttached, "YBOT-010 Humanoidguy topology");
    const auto& active = lab.motors.profile(mt::MotorMode::Active);
    expect(std::fabs(active.torqueFor("ankle_1") - 12.f) < 0.01f && std::fabs(active.torqueFor("hip_1") - 50.f) < 0.01f,
           "YBOT-010 Humanoidguy torque limits");
    const auto* foot = lab.ragdoll.find("foot_1");
    expect(foot != nullptr && std::fabs(foot->mass - 1.3f) < 0.01f, "YBOT-010 Humanoidguy foot mass");
    const auto placement = mt::placeStandingPose(lab.world, lab.ragdoll, lab.balance.config());
    expect(placement.ok && placement.maxSeparation < 0.02f, "YBOT-010 Humanoidguy standing pose");
    expect(!lab.balance.config().relaxArms && std::fabs(lab.balance.config().hipAbductionDeg - 14.f) < 0.01f, "YBOT-010 Humanoidguy pose defaults");
}

struct TrialReport {
    float duration = 0.f;
    float initialComX = 0.f;
    float initialComY = 0.f;
    float initialComZ = 0.f;
    float supportArea = 0.f;
    float torsoTilt = 0.f;
    float maxTilt = 0.f;
    float ankle = 0.f;
    float hip = 0.f;
    int saturatedSteps = 0;
    float maxSlip = 0.f;
    float maxSeparation = 0.f;
    int leftPoints = 0;
    int rightPoints = 0;
    int leftHeel = 0;
    int leftMid = 0;
    int leftFore = 0;
    int rightHeel = 0;
    int rightMid = 0;
    int rightFore = 0;
    std::string fall = "stood";
    float mass = 0.f;
};

TrialReport runTrial(const std::string& rig, const std::string& model, const std::string& preset, const std::string& motors, int steps,
                     mt::Vec3 impulse) {
    TrialReport report;
    Lab lab;
    if (!buildCharacter(lab, rig, model, preset, motors, true)) {
        report.fall = "build failed";
        return report;
    }
    const auto placement = mt::placeStandingPose(lab.world, lab.ragdoll, lab.balance.config());
    if (!placement.ok) {
        report.fall = "pose failed";
        return report;
    }
    lab.balance.setReference(placement.reference);
    lab.balance.setEnabled(true, lab.motors);
    const float dt = lab.world.settings().fixedTimestep;
    lab.balance.observe(lab.world, lab.ragdoll, dt);
    report.initialComX = lab.balance.state().com.x;
    report.initialComY = lab.balance.state().com.y;
    report.initialComZ = lab.balance.state().com.z;
    report.supportArea = lab.balance.state().supportArea;
    report.mass = lab.balance.state().totalMass;
    report.leftPoints = lab.balance.state().leftPoints;
    report.rightPoints = lab.balance.state().rightPoints;
    if (mt::length(impulse) > 0.f) {
        const auto* chest = lab.ragdoll.find("chest");
        if (chest != nullptr) {
            const auto state = lab.world.bodyState(chest->physics);
            const mt::Vec3 direction = impulse.z != 0.f ? lab.balance.state().facing : lab.balance.state().right;
            const float sign = impulse.z != 0.f ? impulse.z : impulse.x;
            lab.world.addImpulse(chest->physics, direction * sign, state.position);
        }
    }
    mt::Vec3 fallVelocity{};
    for (int i = 0; i < steps; ++i) {
        lab.balance.observe(lab.world, lab.ragdoll, dt);
        lab.balance.command(lab.ragdoll, lab.motors);
        lab.motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
        const auto& state = lab.balance.state();
        report.duration = state.stoodSeconds;
        report.maxTilt = std::max(report.maxTilt, state.torsoTilt);
        report.torsoTilt = state.torsoTilt;
        report.ankle = std::max(report.ankle, state.ankleEffort);
        report.hip = std::max(report.hip, state.hipEffort);
        report.supportArea = std::max(report.supportArea, state.supportArea);
        if (state.leftPoints + state.rightPoints >= report.leftPoints + report.rightPoints) {
            report.leftPoints = state.leftPoints;
            report.rightPoints = state.rightPoints;
            report.leftHeel = state.leftHeel;
            report.leftMid = state.leftMid;
            report.leftFore = state.leftFore;
            report.rightHeel = state.rightHeel;
            report.rightMid = state.rightMid;
            report.rightFore = state.rightFore;
        }
        if (state.saturatedMotors > 0) {
            ++report.saturatedSteps;
        }
        if (i > 60) {
            for (const char* id : {"foot_1", "foot_2"}) {
                const auto* foot = lab.ragdoll.find(id);
                if (foot == nullptr) {
                    continue;
                }
                const auto body = lab.world.bodyState(foot->physics);
                const float slip = std::sqrt(body.linearVelocity.x * body.linearVelocity.x + body.linearVelocity.z * body.linearVelocity.z);
                report.maxSlip = std::max(report.maxSlip, slip);
            }
        }
        for (const auto& joint : lab.ragdoll.joints) {
            const auto sample = lab.world.jointState(joint.physics);
            if (sample.valid) {
                report.maxSeparation = std::max(report.maxSeparation, sample.separation);
            }
        }
        if (state.phase == mt::BalancePhase::Falling || state.com.y < 0.55f) {
            fallVelocity = state.comVelocity;
            report.fall = "fell";
            break;
        }
    }
    if (report.fall == "fell") {
        const auto& facing = lab.balance.state().facing;
        const auto right = lab.balance.state().right;
        const float forward = mt::dot(fallVelocity, facing);
        const float lateral = mt::dot(fallVelocity, right);
        if (std::fabs(forward) > std::fabs(lateral)) {
            report.fall = forward >= 0.f ? "fell forward" : "fell backward";
        } else {
            report.fall = lateral >= 0.f ? "fell right" : "fell left";
        }
    }
    return report;
}

void printTrial(const std::string& name, const TrialReport& report) {
    std::cout << name << " duration " << report.duration << " com " << report.initialComX << " " << report.initialComY << " " << report.initialComZ
              << " area " << report.supportArea << " contacts L " << report.leftPoints << " (h" << report.leftHeel << " m" << report.leftMid
              << " f" << report.leftFore << ") R " << report.rightPoints << " (h" << report.rightHeel << " m" << report.rightMid << " f"
              << report.rightFore << ") tilt " << report.torsoTilt
              << " maxTilt " << report.maxTilt << " ankle " << report.ankle << " hip " << report.hip << " satSteps " << report.saturatedSteps
              << " slip " << report.maxSlip << " sep " << report.maxSeparation << " mass " << report.mass << " " << report.fall << '\n';
}

void reportComparison() {
    if (!ybotFilePresent()) {
        std::cout << "skip comparison, ybot.fbx missing\n";
        return;
    }
    const int quiet = 8 * 120;
    const int longer = 15 * 120;
    const int disturbed = 6 * 120;
    const struct {
        const char* name;
        const char* rig;
        const char* model;
        const char* preset;
        const char* motors;
    } characters[] = {
        {"humanoidguy", "configurations/humanoidguy.json", "characters/humanoidguy_rigged.fbx", "normal", "configurations/motors"},
        {"ybot", "configurations/ybot.json", "characters/ybot.fbx", "ybot", "configurations/motors/ybot"},
    };
    for (const auto& character : characters) {
        Lab measure;
        if (buildCharacter(measure, character.rig, character.model, character.preset, character.motors, false)) {
            float mass = 0.f;
            for (const auto& body : measure.ragdoll.bodies) {
                mass += body.mass;
                if (body.id == "thigh_1" || body.id == "calf_1" || body.id == "upper_arm_1" || body.id == "foot_1") {
                    std::cout << character.name << " " << body.id << " length " << mt::length(body.segmentEnd - body.segmentStart) << " mass " << body.mass;
                    if (body.collider == mt::BodyCreateInfo::Shape::Box) {
                        std::cout << " box " << body.boxHalfExtents.x * 2.f << "x" << body.boxHalfExtents.y * 2.f << "x" << body.boxHalfExtents.z * 2.f;
                    }
                    std::cout << '\n';
                }
            }
            std::cout << character.name << " totalMass " << mass << " bodies " << measure.ragdoll.bodies.size() << " joints " << measure.ragdoll.joints.size()
                      << " ankleTorque " << measure.motors.profile(mt::MotorMode::Active).torqueFor("ankle_1") << " hipTorque "
                      << measure.motors.profile(mt::MotorMode::Active).torqueFor("hip_1") << " kneeTorque "
                      << measure.motors.profile(mt::MotorMode::Active).torqueFor("knee_1") << '\n';
        }
        printTrial(std::string(character.name) + " quiet",
                   runTrial(character.rig, character.model, character.preset, character.motors, quiet, {}));
        printTrial(std::string(character.name) + " forward2",
                   runTrial(character.rig, character.model, character.preset, character.motors, disturbed, {0.f, 0.f, 2.f}));
        printTrial(std::string(character.name) + " lateral2",
                   runTrial(character.rig, character.model, character.preset, character.motors, disturbed, {2.f, 0.f, 0.f}));
        printTrial(std::string(character.name) + " long",
                   runTrial(character.rig, character.model, character.preset, character.motors, longer, {}));
    }
}

}  // namespace

void runYbotTest(const std::string& which) {
    if (which == "YBOT-001") test001();
    else if (which == "YBOT-002") test002();
    else if (which == "YBOT-003") test003();
    else if (which == "YBOT-004") test004();
    else if (which == "YBOT-005") test005();
    else if (which == "YBOT-006") test006();
    else if (which == "YBOT-007") test007();
    else if (which == "YBOT-008") test008();
    else if (which == "YBOT-009") test009();
    else if (which == "YBOT-010") test010();
    else if (which == "YBOT-REPORT") reportComparison();
    else {
        std::cerr << "unknown test " << which << '\n';
        ++gFailures;
    }
}

void runAllYbotTests() {
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
}
