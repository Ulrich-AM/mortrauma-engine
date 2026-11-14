#include "mortrauma/assets/ModelLoader.hpp"
#include "mortrauma/core/Log.hpp"
#include "mortrauma/core/Paths.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

int gFailures = 0;

void expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++gFailures;
    } else {
        std::cout << "ok: " << message << '\n';
    }
}

namespace {

struct Lab {
    mt::PhysicsWorld world;
    mt::ImportedModel model;
    mt::CharacterConfig config;
    mt::Ragdoll ragdoll;
    mt::JointPreset preset;
    int cube = -1;
    mt::Vec3 cubeStart{};
};

bool loadLab(Lab& lab, bool withCube) {
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
        std::cerr << "asset load failed: " << lab.model.report.error << " " << lab.config.error << " " << error << '\n';
        return false;
    }
    lab.ragdoll = mt::RagdollBuilder::build(lab.world, lab.config, lab.model, lab.preset);
    if (!lab.ragdoll.ok) {
        std::cerr << "ragdoll build failed: " << lab.ragdoll.error << '\n';
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
    if (withCube) {
        mt::BodyCreateInfo cube;
        cube.name = "Cube";
        cube.shape = mt::BodyCreateInfo::Shape::Box;
        cube.halfExtents = {0.2f, 0.2f, 0.2f};
        cube.position = {1.3f, 1.2f, 1.1f};
        cube.mass = 2.f;
        cube.friction = 0.6f;
        lab.cubeStart = cube.position;
        lab.cube = lab.world.createBody(cube);
    }
    // Lift the character if the bind pose intersects the floor.
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

void step(Lab& lab, float seconds) {
    const float dt = lab.world.settings().fixedTimestep;
    const int steps = int(std::ceil(seconds / dt));
    for (int i = 0; i < steps; ++i) {
        lab.world.step(dt, 1);
    }
}

bool allFinite(const Lab& lab, float speedLimit) {
    for (const auto& body : lab.ragdoll.bodies) {
        const auto state = lab.world.bodyState(body.physics);
        if (!state.valid || !mt::finite(state.position) || !mt::finite(state.linearVelocity) || !mt::finite(state.angularVelocity)) {
            return false;
        }
        if (mt::length(state.linearVelocity) > speedLimit || mt::length(state.angularVelocity) > speedLimit) {
            return false;
        }
    }
    return true;
}

float maxSeparation(const Lab& lab) {
    float worst = 0.f;
    for (const auto& joint : lab.ragdoll.joints) {
        worst = std::max(worst, lab.world.jointState(joint.physics).separation);
    }
    return worst;
}

bool limitsHeld(const Lab& lab, float extraRadians) {
    for (const auto& joint : lab.ragdoll.joints) {
        const auto state = lab.world.jointState(joint.physics);
        if (joint.base.swing == mt::SwingKind::Cone) {
            const float cone = std::max(std::fabs(state.swing1Max), std::fabs(state.swing2Max));
            if (state.swingAngle > cone + extraRadians) {
                std::cerr << joint.id << " swing " << state.swingAngle << " cone " << cone << '\n';
                return false;
            }
        } else {
            if (state.rotationRadians.y < state.swing1Min - extraRadians || state.rotationRadians.y > state.swing1Max + extraRadians ||
                state.rotationRadians.z < state.swing2Min - extraRadians || state.rotationRadians.z > state.swing2Max + extraRadians) {
                std::cerr << joint.id << " swing euler " << state.rotationRadians.y << " " << state.rotationRadians.z << '\n';
                return false;
            }
        }
        if (state.rotationRadians.x < state.twistMin - extraRadians || state.rotationRadians.x > state.twistMax + extraRadians) {
            std::cerr << joint.id << " twist " << state.rotationRadians.x << '\n';
            return false;
        }
    }
    return true;
}

std::vector<mt::Vec3> snapshot(const Lab& lab) {
    std::vector<mt::Vec3> positions;
    for (const auto& body : lab.ragdoll.bodies) {
        positions.push_back(lab.world.bodyState(body.physics).position);
    }
    return positions;
}

void testRig001(Lab& lab) {
    expect(lab.ragdoll.ok, "ragdoll constructed from humanoidguy");
    expect(lab.ragdoll.bodies.size() == 21, "21 intended rigid bodies");
    expect(lab.ragdoll.joints.size() == 20, "20 anatomical joints");
    expect(mt::disconnectedComponents(lab.ragdoll) == 1, "RIG-001 single constraint component");
    expect(lab.model.report.unweightedMeshes == 0, "no unweighted meshes were required");
    expect(lab.model.report.boneCount >= 28, "imported bone count");
    expect(lab.model.report.meshCount == 56, "56 capsule meshes preserved");
}

void testRig002(Lab& lab) {
    expect(lab.ragdoll.legsAttached, "RIG-002 both legs reach the pelvis");
    bool hip1 = false;
    bool hip2 = false;
    bool knee1 = false;
    bool knee2 = false;
    bool ankle1 = false;
    bool ankle2 = false;
    for (const auto& joint : lab.ragdoll.joints) {
        const auto state = lab.world.jointState(joint.physics);
        expect(state.positionLocked, joint.id + " translation locked");
        expect(state.separation < 0.005f, joint.id + " bind separation < 5 mm");
        hip1 = hip1 || joint.id == "hip_root_1";
        hip2 = hip2 || joint.id == "hip_root_2";
        knee1 = knee1 || joint.id == "knee_1";
        knee2 = knee2 || joint.id == "knee_2";
        ankle1 = ankle1 || joint.id == "ankle_1";
        ankle2 = ankle2 || joint.id == "ankle_2";
        if (joint.id == "hip_1" || joint.id == "hip_2") {
            expect(lab.ragdoll.bodies[joint.parent].id.find("hip_") == 0, joint.id + " parent is a hip body");
            expect(lab.ragdoll.bodies[joint.child].id.find("thigh_") == 0, joint.id + " child is a thigh");
        }
    }
    expect(hip1 && hip2 && knee1 && knee2 && ankle1 && ankle2, "hip, knee and ankle joints exist on both sides");
}

void testRig003(Lab& lab) {
    step(lab, 2.0f);
    expect(allFinite(lab, 40.f), "RIG-003 finite motion while limits are checked");
    expect(limitsHeld(lab, 0.2f), "RIG-003 joint rotations stay within limits + 0.2 rad");
    expect(maxSeparation(lab) < 0.02f, "RIG-003 joints stay within 2 cm");
}

void testPhy001(Lab& lab) {
    const float start = lab.world.bodyState(lab.ragdoll.find("pelvis")->physics).position.y;
    step(lab, 0.8f);
    const float after = lab.world.bodyState(lab.ragdoll.find("pelvis")->physics).position.y;
    expect(allFinite(lab, 30.f), "PHY-001 no numerical blow-up during passive fall");
    expect(after < start - 0.15f, "PHY-001 pelvis drops at least 15 cm");
    for (const auto& body : lab.ragdoll.bodies) {
        expect(lab.world.bodyState(body.physics).isDynamic, body.id + " stays dynamic");
    }
}

void testPhy002(Lab& lab) {
    step(lab, 6.0f);
    expect(allFinite(lab, 30.f), "PHY-002 remains finite");
    float maxLinear = 0.f;
    float maxAngular = 0.f;
    float lowest = 1e9f;
    for (const auto& body : lab.ragdoll.bodies) {
        const auto state = lab.world.bodyState(body.physics);
        maxLinear = std::max(maxLinear, mt::length(state.linearVelocity));
        maxAngular = std::max(maxAngular, mt::length(state.angularVelocity));
        lowest = std::min(lowest, state.position.y);
    }
    std::cout << "settle max linear " << maxLinear << " angular " << maxAngular << " lowest " << lowest << '\n';
    expect(maxLinear < 0.35f, "PHY-002 linear speed below 0.35 m/s");
    expect(maxAngular < 2.0f, "PHY-002 angular speed below 2 rad/s");
    expect(lowest > -0.3f, "PHY-002 no body falls through the floor");
    expect(mt::disconnectedComponents(lab.ragdoll) == 1, "PHY-002 graph still connected");
    expect(maxSeparation(lab) < 0.025f, "PHY-002 separation below 2.5 cm");
    if (lab.cube >= 0) {
        const auto cube = lab.world.bodyState(lab.cube);
        expect(std::fabs(cube.position.y - 0.2f) < 0.08f, "cube rests on the floor");
        expect(mt::length(cube.linearVelocity) < 0.4f, "cube has settled");
    }
}

void testPhy003(Lab& lab) {
    lab.world.setGravity({});
    mt::BodyCreateInfo box;
    box.name = "Impulse probe";
    box.shape = mt::BodyCreateInfo::Shape::Box;
    box.halfExtents = {0.1f, 0.1f, 0.1f};
    box.position = {0.f, 3.f, 0.f};
    box.mass = 2.f;
    const int probe = lab.world.createBody(box);
    lab.world.step(lab.world.settings().fixedTimestep, 1);
    lab.world.addImpulse(probe, {4.f, 0.f, 0.f}, lab.world.bodyState(probe).position);
    lab.world.step(lab.world.settings().fixedTimestep, 1);
    const float vx = lab.world.bodyState(probe).linearVelocity.x;
    expect(std::fabs(vx - 2.f) < 0.05f, "PHY-003 probe velocity matches impulse/mass (4 Ns / 2 kg)");

    const auto pelvis = lab.ragdoll.find("pelvis");
    const float before = lab.world.bodyState(pelvis->physics).linearVelocity.x;
    lab.world.addImpulse(pelvis->physics, {25.f, 0.f, 0.f}, lab.world.bodyState(pelvis->physics).position);
    lab.world.step(lab.world.settings().fixedTimestep, 1);
    const float after = lab.world.bodyState(pelvis->physics).linearVelocity.x;
    expect(after > before + 0.2f, "PHY-003 character pelvis gains horizontal velocity");
    expect(allFinite(lab, 40.f), "PHY-003 character stays finite");

    const mt::Vec3 probePosition = lab.world.bodyState(probe).position;
    const int grab = lab.world.beginGrab(probe, probePosition, probePosition, 12.f, 0.9f);
    expect(grab >= 0, "grab constraint is created");
    const float dt = lab.world.settings().fixedTimestep;
    for (int i = 0; i < 40; ++i) {
        lab.world.updateGrab(grab, probePosition + mt::Vec3{0.f, 0.5f, 0.f});
        lab.world.step(dt, 1);
    }
    const float lifted = lab.world.bodyState(probe).position.y - probePosition.y;
    std::cout << "grab lift " << lifted << '\n';
    expect(lifted > 0.15f, "PHY-003 grab spring moves the body toward the target");
    lab.world.endGrab(grab);
    expect(lab.world.jointCount() == int(lab.ragdoll.joints.size()), "ending the grab removes the temporary constraint");
}

void testPhy004(Lab& lab) {
    step(lab, 1.0f);
    mt::resetRagdoll(lab.world, lab.ragdoll);
    for (const auto& body : lab.ragdoll.bodies) {
        const auto state = lab.world.bodyState(body.physics);
        const float positionError = mt::length(state.position - body.initialPosition);
        const float velocity = mt::length(state.linearVelocity) + mt::length(state.angularVelocity);
        expect(positionError < 1e-4f, "PHY-004 " + body.id + " pose restored");
        expect(velocity < 1e-4f, "PHY-004 " + body.id + " velocity cleared");
    }
}

std::vector<mt::Vec3> runRepeat(bool impulse) {
    Lab lab;
    loadLab(lab, false);
    lab.world.setGravity({0.f, -9.81f, 0.f});
    const float dt = lab.world.settings().fixedTimestep;
    for (int i = 0; i < 240; ++i) {
        if (impulse && i == 60) {
            const auto* chest = lab.ragdoll.find("chest");
            lab.world.addImpulse(chest->physics, {12.f, 4.f, -6.f}, lab.world.bodyState(chest->physics).position);
        }
        lab.world.step(dt, 1);
    }
    return snapshot(lab);
}

void testPhy005() {
    const auto a = runRepeat(true);
    const auto b = runRepeat(true);
    expect(a.size() == b.size() && !a.empty(), "PHY-005 produced two trajectories");
    float worst = 0.f;
    for (size_t i = 0; i < a.size(); ++i) {
        worst = std::max(worst, mt::length(a[i] - b[i]));
    }
    std::cout << "repeatability deviation " << worst << '\n';
    expect(worst < 1e-4f, "PHY-005 identical replay within 0.1 mm");
}

const mt::RagdollJoint* findJoint(const Lab& lab, const std::string& id) {
    for (const auto& joint : lab.ragdoll.joints) {
        if (joint.id == id) {
            return &joint;
        }
    }
    return nullptr;
}

mt::Quat basisQuat(mt::Vec3 x, mt::Vec3 y, mt::Vec3 z) {
    mt::Mat4 matrix;
    matrix.m[0] = x.x;
    matrix.m[1] = x.y;
    matrix.m[2] = x.z;
    matrix.m[4] = y.x;
    matrix.m[5] = y.y;
    matrix.m[6] = y.z;
    matrix.m[8] = z.x;
    matrix.m[9] = z.y;
    matrix.m[10] = z.z;
    return matrix.rotation();
}

void poseSwing(Lab& lab, const mt::RagdollJoint& joint, float swing2) {
    const auto& child = lab.ragdoll.bodies[joint.child];
    const mt::Vec3 worldX = mt::normalize(joint.extensionAxis * std::cos(swing2) + joint.flexionDirection * std::sin(swing2));
    const mt::Vec3 worldZ = mt::normalize(joint.flexionAxis);
    const mt::Vec3 worldY = mt::normalize(mt::cross(worldZ, worldX));
    const mt::Vec3 localX = mt::normalize(mt::rotate(mt::conjugate(child.initialRotation), child.segmentEnd - child.segmentStart));
    mt::Vec3 localZ = mt::rotate(mt::conjugate(child.initialRotation), joint.flexionAxis);
    localZ = localZ - localX * mt::dot(localZ, localX);
    localZ = mt::normalize(localZ);
    const mt::Vec3 localY = mt::normalize(mt::cross(localZ, localX));
    const mt::Quat rotation = mt::mul(basisQuat(worldX, worldY, worldZ), mt::conjugate(basisQuat(localX, localY, localZ)));
    const mt::Vec3 centerLocal = mt::rotate(mt::conjugate(child.initialRotation), child.initialPosition - child.segmentStart);
    lab.world.setBodyPose(child.physics, child.segmentStart + mt::rotate(rotation, centerLocal), rotation, true);
}

float geometricBend(const Lab& lab, const mt::RagdollJoint& joint) {
    const auto& parent = lab.ragdoll.bodies[joint.parent];
    const auto& child = lab.ragdoll.bodies[joint.child];
    const auto parentState = lab.world.bodyState(parent.physics);
    const auto childState = lab.world.bodyState(child.physics);
    const mt::Vec3 parentLocal = mt::rotate(mt::conjugate(parent.initialRotation), parent.segmentEnd - parent.segmentStart);
    const mt::Vec3 childLocal = mt::rotate(mt::conjugate(child.initialRotation), child.segmentEnd - child.segmentStart);
    const mt::Vec3 parentDir = mt::normalize(mt::rotate(parentState.rotation, parentLocal));
    const mt::Vec3 childDir = mt::normalize(mt::rotate(childState.rotation, childLocal));
    return std::acos(mt::clampf(mt::dot(parentDir, childDir), -1.f, 1.f));
}

void testRig004(Lab& lab) {
    mt::resetRagdoll(lab.world, lab.ragdoll);
    const char* ids[] = {"elbow_1", "elbow_2", "knee_1", "knee_2"};
    for (const char* id : ids) {
        const auto* joint = findJoint(lab, id);
        expect(joint != nullptr, std::string(id) + " exists");
        if (joint == nullptr) {
            continue;
        }
        const auto bind = lab.world.jointState(joint->physics);
        const float geometric = geometricBend(lab, *joint);
        std::cout << id << " before-correction equivalent was 0 deg at a geometric bend of " << mt::degrees(geometric)
                  << " deg; measured swing2 now " << mt::degrees(bind.rotationRadians.z) << " deg\n";
        expect(std::fabs(bind.rotationRadians.x) < mt::radians(4.f), std::string(id) + " bind twist is near zero");
        expect(std::fabs(bind.rotationRadians.y) < mt::radians(4.f), std::string(id) + " bind side swing is near zero");
        expect(bind.rotationRadians.z > mt::radians(60.f) && bind.rotationRadians.z < mt::radians(110.f),
               std::string(id) + " bind pose is flexed, not treated as extension");
        expect(std::fabs(bind.rotationRadians.z - geometric) < mt::radians(4.f),
               std::string(id) + " swing2 matches the geometric bend");
        expect(bind.withinLimits, std::string(id) + " crouched bind pose is inside the anatomical limits");

        poseSwing(lab, *joint, 0.f);
        const auto straight = lab.world.jointState(joint->physics);
        std::cout << id << " straight pose swing2 " << mt::degrees(straight.rotationRadians.z) << " separation " << straight.separation
                  << "\n";
        expect(std::fabs(straight.rotationRadians.z) < mt::radians(4.f), std::string(id) + " full extension reads near 0 deg");
        expect(straight.withinLimits, std::string(id) + " full extension is inside the limits");
        expect(straight.separation < 0.01f, std::string(id) + " extension keeps the pivot attached");

        poseSwing(lab, *joint, mt::radians(-25.f));
        const auto hyper = lab.world.jointState(joint->physics);
        expect(!hyper.withinLimits, std::string(id) + " 25 deg hyperextension is outside the limits");

        poseSwing(lab, *joint, mt::radians(160.f));
        const auto folded = lab.world.jointState(joint->physics);
        expect(!folded.withinLimits, std::string(id) + " 160 deg flexion is outside the 145 deg limit");
        mt::resetRagdoll(lab.world, lab.ragdoll);
    }
}

void testRig005(Lab& lab) {
    mt::resetRagdoll(lab.world, lab.ragdoll);
    const char* ids[] = {"elbow_1", "elbow_2", "knee_1", "knee_2"};
    for (const char* id : ids) {
        const auto* joint = findJoint(lab, id);
        if (joint == nullptr) {
            expect(false, std::string(id) + " exists");
            continue;
        }
        const float bind = lab.world.jointState(joint->physics).rotationRadians.z;
        poseSwing(lab, *joint, bind + mt::radians(20.f));
        const float flexed = lab.world.jointState(joint->physics).rotationRadians.z;
        std::cout << id << " bend direction " << mt::degrees(bind) << " -> " << mt::degrees(flexed) << "\n";
        expect(flexed > bind + mt::radians(12.f), std::string(id) + " positive swing2 increases flexion on this side");
        expect(lab.world.jointState(joint->physics).withinLimits, std::string(id) + " 20 deg more flexion stays inside 145");
        mt::resetRagdoll(lab.world, lab.ragdoll);
    }
}

void testRig006(Lab& lab) {
    mt::resetRagdoll(lab.world, lab.ragdoll);
    for (const char* id : {"hip_1", "hip_2"}) {
        const auto* joint = findJoint(lab, id);
        expect(joint != nullptr, std::string(id) + " exists");
        if (joint == nullptr) {
            continue;
        }
        const auto bind = lab.world.jointState(joint->physics);
        std::cout << id << " bind twist/abduction/flexion " << mt::degrees(bind.rotationRadians.x) << " "
                  << mt::degrees(bind.rotationRadians.y) << " " << mt::degrees(bind.rotationRadians.z) << "\n";
        expect(bind.rotationRadians.z > mt::radians(50.f) && bind.rotationRadians.z < mt::radians(100.f),
               std::string(id) + " crouched thigh is flexed forward from vertical");
        expect(bind.withinLimits, std::string(id) + " crouched hip is inside the anatomical range");
        expect(std::fabs(bind.rotationRadians.y) < mt::radians(50.f), std::string(id) + " abduction is not inverted past 50 deg");

        poseSwing(lab, *joint, 0.f);
        const auto extended = lab.world.jointState(joint->physics);
        std::cout << id << " thigh-down swing2 " << mt::degrees(extended.rotationRadians.z) << " within " << extended.withinLimits << "\n";
        expect(std::fabs(extended.rotationRadians.z) < mt::radians(6.f), std::string(id) + " thigh-down is extension, near 0");
        expect(extended.withinLimits, std::string(id) + " a hanging thigh is inside the hip limits");
        expect(extended.separation < 0.01f, std::string(id) + " hip pivot stays attached at extension");

        poseSwing(lab, *joint, mt::radians(-45.f));
        expect(!lab.world.jointState(joint->physics).withinLimits, std::string(id) + " 45 deg hip hyperextension is blocked");
        mt::resetRagdoll(lab.world, lab.ragdoll);
    }
    const auto* hip1 = findJoint(lab, "hip_1");
    const auto* hip2 = findJoint(lab, "hip_2");
    if (hip1 != nullptr && hip2 != nullptr) {
        const float side1 = lab.world.jointState(hip1->physics).rotationRadians.y;
        const float side2 = lab.world.jointState(hip2->physics).rotationRadians.y;
        expect(side1 * side2 < 0.f, "hip abduction signs are opposite, matching the two sides");
    }

    for (const char* id : {"shoulder_1", "shoulder_2"}) {
        const auto* joint = findJoint(lab, id);
        if (joint == nullptr) {
            expect(false, std::string(id) + " exists");
            continue;
        }
        const auto& arm = lab.ragdoll.bodies[joint->child];
        const auto bind = lab.world.jointState(joint->physics);
        const float cone = std::max(std::fabs(bind.swing1Max), std::fabs(bind.swing2Max));
        const mt::Vec3 side = arm.segmentEnd.x >= 0.f ? mt::Vec3{1.f, 0.f, 0.f} : mt::Vec3{-1.f, 0.f, 0.f};
        const mt::Vec3 hang = mt::normalize(side * 0.3f + mt::Vec3{0.f, -1.f, 0.f});
        const mt::Vec3 local = mt::normalize(mt::rotate(mt::conjugate(arm.initialRotation), arm.segmentEnd - arm.segmentStart));
        const mt::Quat rotation = mt::quatFromTo(local, hang);
        const mt::Vec3 centerLocal = mt::rotate(mt::conjugate(arm.initialRotation), arm.initialPosition - arm.segmentStart);
        lab.world.setBodyPose(arm.physics, arm.segmentStart + mt::rotate(rotation, centerLocal), rotation, true);
        const auto hung = lab.world.jointState(joint->physics);
        std::cout << id << " hang swing " << mt::degrees(hung.swingAngle) << " cone " << mt::degrees(cone) << "\n";
        expect(hung.swingAngle < cone, std::string(id) + " arm hanging at the side is inside the shoulder cone");
        expect(hung.positionLocked && hung.separation < 0.01f, std::string(id) + " shoulder pivot stays attached");
        mt::resetRagdoll(lab.world, lab.ragdoll);
    }
}

void driveTowardExtension(Lab& lab, const mt::RagdollJoint& joint, float friction) {
    lab.world.setJointAngularFriction(joint.physics, friction);
    const auto& parent = lab.ragdoll.bodies[joint.parent];
    const auto& child = lab.ragdoll.bodies[joint.child];
    lab.world.setBodyDamping(child.physics, 0.02f, 0.05f);
    const float dt = lab.world.settings().fixedTimestep;
    for (int i = 0; i < int(1.5f / dt); ++i) {
        lab.world.setBodyPose(parent.physics, parent.initialPosition, parent.initialRotation, true);
        lab.world.setBodyVelocity(parent.physics, {}, {});
        const auto state = lab.world.jointState(joint.physics);
        const float speed = mt::clampf(state.rotationRadians.z, 0.f, 1.5f) * 12.f;
        lab.world.setBodyVelocity(child.physics, {}, state.bendAxisWorld * -speed);
        lab.world.step(dt, 1);
    }
}

void testLim001(Lab& lab) {
    mt::resetRagdoll(lab.world, lab.ragdoll);
    const mt::Vec3 gravity = lab.world.gravity();
    lab.world.setGravity({});
    for (auto& body : lab.ragdoll.bodies) {
        body.initialPosition.y += 3.f;
        body.segmentStart.y += 3.f;
        body.segmentEnd.y += 3.f;
        lab.world.setBodyPose(body.physics, body.initialPosition, body.initialRotation, true);
    }
    const char* ids[] = {"elbow_1", "elbow_2", "knee_1", "knee_2"};
    for (const char* id : ids) {
        const auto* joint = findJoint(lab, id);
        if (joint == nullptr) {
            expect(false, std::string(id) + " exists");
            continue;
        }
        const float before = lab.world.jointState(joint->physics).rotationRadians.z;
        driveTowardExtension(lab, *joint, 0.f);
        const auto after = lab.world.jointState(joint->physics);
        const int contacts = lab.world.activeContactCount();
        std::cout << id << " extension drive " << mt::degrees(before) << " -> " << mt::degrees(after.rotationRadians.z)
                  << " deg, separation " << after.separation << " m, contacts " << contacts << ", at swing2 limit "
                  << after.swing2AtLimit << "\n";
        expect(after.rotationRadians.z < mt::radians(12.f), std::string(id) + " reaches near-full extension (not stopped by the old crouch limit)");
        expect(after.rotationRadians.z > mt::radians(-10.f), std::string(id) + " does not hyperextend past solver tolerance");
        expect(after.separation < 0.02f, std::string(id) + " stays attached while extending");
        expect(after.swing2AtLimit, std::string(id) + " stops on the extension limit rather than on a contact");
        mt::resetRagdoll(lab.world, lab.ragdoll);
        for (auto& body : lab.ragdoll.bodies) {
            lab.world.setBodyPose(body.physics, body.initialPosition, body.initialRotation, true);
        }
        driveTowardExtension(lab, *joint, lab.preset.angularFriction);
        const auto withFriction = lab.world.jointState(joint->physics);
        std::cout << id << " with natural friction " << lab.preset.angularFriction << " Nm -> " << mt::degrees(withFriction.rotationRadians.z)
                  << " deg\n";
        expect(withFriction.rotationRadians.z < mt::radians(15.f),
               std::string(id) + " still reaches near extension with natural angular friction");
        mt::resetRagdoll(lab.world, lab.ragdoll);
        for (auto& body : lab.ragdoll.bodies) {
            lab.world.setBodyPose(body.physics, body.initialPosition, body.initialRotation, true);
        }
    }
    for (auto& body : lab.ragdoll.bodies) {
        body.initialPosition.y -= 3.f;
        body.segmentStart.y -= 3.f;
        body.segmentEnd.y -= 3.f;
    }
    lab.world.setGravity(gravity);
    mt::resetRagdoll(lab.world, lab.ragdoll);
    mt::applyPreset(lab.world, lab.ragdoll, lab.preset);
}

void testPhy006(Lab& lab) {
    const auto* chest = lab.ragdoll.find("chest");
    const mt::Vec3 directions[] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -0.2f, 1}, {0, 0.5f, -1}, {1, 0.2f, 1}};
    const float magnitudes[] = {5.f, 20.f, 80.f};
    for (float magnitude : magnitudes) {
        for (mt::Vec3 direction : directions) {
            mt::resetRagdoll(lab.world, lab.ragdoll);
            const mt::Vec3 point = lab.world.bodyState(chest->physics).position + mt::Vec3{0.05f, 0.02f, -0.04f};
            lab.world.addImpulse(chest->physics, mt::normalize(direction) * magnitude, point);
            step(lab, 1.5f);
            expect(allFinite(lab, 60.f), "PHY-006 finite after impulse " + std::to_string(magnitude));
            expect(maxSeparation(lab) < 0.05f, "PHY-006 separation below 5 cm");
            expect(mt::disconnectedComponents(lab.ragdoll) == 1, "PHY-006 still connected");
            if (gFailures > 12) {
                return;
            }
        }
    }
}

}  // namespace

void runMotorTest(const std::string& which);
void runAllMotorTests();
void runGrabTest(const std::string& which);
void runAllGrabTests();
void runBalanceTest(const std::string& which);
void runAllBalanceTests();
void runEquilibriumTest(const std::string& which);
void runAllEquilibriumTests();
void runFootTest(const std::string& which);
void runAllFootTests();
void runStabTest(const std::string& which);
void runAllStabTests();
void runAlignTest(const std::string& which);
void runAllAlignTests();
void runStepTest(const std::string& which);
void runAllStepTests();
void runUnloadTest(const std::string& which);
void runAllUnloadTests();
void recordStandingCsv(const std::string& path);
void runEquilibriumSweep();
void runLongStand();

int main(int argc, char** argv) {
    const std::string which = argc > 1 ? argv[1] : "ALL";
    if (which.rfind("MOT-", 0) == 0) {
        runMotorTest(which);
        std::cout << (gFailures == 0 ? "ALL PASSED\n" : "FAILURES " + std::to_string(gFailures) + "\n");
        return gFailures == 0 ? 0 : 1;
    }
    if (which.rfind("GRAB-", 0) == 0) {
        runGrabTest(which);
        std::cout << (gFailures == 0 ? "ALL PASSED\n" : "FAILURES " + std::to_string(gFailures) + "\n");
        return gFailures == 0 ? 0 : 1;
    }
    if (which == "EQ-BASE") {
        recordStandingCsv(argc > 2 ? argv[2] : "standing_baseline.csv");
        return gFailures == 0 ? 0 : 1;
    }
    if (which == "EQ-SWEEP") {
        runEquilibriumSweep();
        return gFailures == 0 ? 0 : 1;
    }
    if (which == "EQ-LONG") {
        runLongStand();
        return gFailures == 0 ? 0 : 1;
    }
    if (which.rfind("FOOT-", 0) == 0) {
        runFootTest(which);
        std::cout << (gFailures == 0 ? "ALL PASSED\n" : "FAILURES " + std::to_string(gFailures) + "\n");
        return gFailures == 0 ? 0 : 1;
    }
    if (which.rfind("EQ-", 0) == 0 && which != "EQ-BASE" && which != "EQ-SWEEP" && which != "EQ-LONG") {
        runEquilibriumTest(which);
        std::cout << (gFailures == 0 ? "ALL PASSED\n" : "FAILURES " + std::to_string(gFailures) + "\n");
        return gFailures == 0 ? 0 : 1;
    }
    if (which.rfind("STAB-", 0) == 0) {
        runStabTest(which);
        std::cout << (gFailures == 0 ? "ALL PASSED\n" : "FAILURES " + std::to_string(gFailures) + "\n");
        return gFailures == 0 ? 0 : 1;
    }
    if (which.rfind("ALIGN-", 0) == 0) {
        runAlignTest(which);
        std::cout << (gFailures == 0 ? "ALL PASSED\n" : "FAILURES " + std::to_string(gFailures) + "\n");
        return gFailures == 0 ? 0 : 1;
    }
    if (which.rfind("UNLOAD-", 0) == 0) {
        runUnloadTest(which);
        std::cout << (gFailures == 0 ? "ALL PASSED\n" : "FAILURES " + std::to_string(gFailures) + "\n");
        return gFailures == 0 ? 0 : 1;
    }
    if (which.rfind("STEP-", 0) == 0) {
        runStepTest(which);
        std::cout << (gFailures == 0 ? "ALL PASSED\n" : "FAILURES " + std::to_string(gFailures) + "\n");
        return gFailures == 0 ? 0 : 1;
    }
    if (which.rfind("BAL-", 0) == 0) {
        runBalanceTest(which);
        std::cout << (gFailures == 0 ? "ALL PASSED\n" : "FAILURES " + std::to_string(gFailures) + "\n");
        return gFailures == 0 ? 0 : 1;
    }
    if (which == "DUMP") {
        const auto model = mt::ModelLoader::load(mt::findAsset("characters/humanoidguy_rigged.fbx").string());
        if (!model.report.ok) {
            return 1;
        }
        mt::Vec3 lo{1e9f, 1e9f, 1e9f};
        mt::Vec3 hi{-1e9f, -1e9f, -1e9f};
        for (const auto& mesh : model.meshes) {
            const mt::Mat4& bone = model.bones[mesh.bone].bindWorld;
            for (const auto& vertex : mesh.vertices) {
                const mt::Vec3 world = mt::transformPoint(bone, vertex.position);
                lo.x = std::min(lo.x, world.x);
                lo.y = std::min(lo.y, world.y);
                lo.z = std::min(lo.z, world.z);
                hi.x = std::max(hi.x, world.x);
                hi.y = std::max(hi.y, world.y);
                hi.z = std::max(hi.z, world.z);
            }
        }
        std::cout << "bounds min " << lo.x << " " << lo.y << " " << lo.z << " max " << hi.x << " " << hi.y << " " << hi.z << '\n';
        for (const auto& bone : model.bones) {
            float nearest = 1e9f;
            mt::Vec3 nearestPoint{};
            for (const auto& mesh : model.meshes) {
                if (mesh.boneName != bone.name) {
                    continue;
                }
                const mt::Mat4& bind = model.bones[mesh.bone].bindWorld;
                for (const auto& vertex : mesh.vertices) {
                    const mt::Vec3 world = mt::transformPoint(bind, vertex.position);
                    const float distance = mt::length(world - bone.bindPosition);
                    if (distance < nearest) {
                        nearest = distance;
                        nearestPoint = world;
                    }
                }
            }
            if (nearest < 1e8f) {
                std::cout << "nearest " << bone.name << " " << nearest << " at " << nearestPoint.x << " " << nearestPoint.y << " "
                          << nearestPoint.z << '\n';
            }
        }
        for (const auto& bone : model.bones) {
            const auto& p = bone.bindPosition;
            const char* parent = bone.parent >= 0 ? model.bones[bone.parent].name.c_str() : "-";
            std::cout << bone.name << " parent=" << parent << " pos=" << p.x << " " << p.y << " " << p.z << '\n';
        }
        return 0;
    }
    if (which == "PHY-005" || which == "ALL") {
        testPhy005();
        if (which == "PHY-005") {
            return gFailures == 0 ? 0 : 1;
        }
    }
    Lab lab;
    if (!loadLab(lab, true)) {
        std::cerr << "failed to load laboratory\n";
        return 1;
    }
    if (which == "RIG-001" || which == "ALL") testRig001(lab);
    if (which == "RIG-002" || which == "ALL") testRig002(lab);
    if (which == "RIG-003" || which == "ALL") testRig003(lab);
    if (which == "RIG-004" || which == "ALL") testRig004(lab);
    if (which == "RIG-005" || which == "ALL") testRig005(lab);
    if (which == "RIG-006" || which == "ALL") testRig006(lab);
    if (which == "PHY-001" || which == "ALL") {
        mt::resetRagdoll(lab.world, lab.ragdoll);
        testPhy001(lab);
    }
    if (which == "PHY-002" || which == "ALL") {
        mt::resetRagdoll(lab.world, lab.ragdoll);
        testPhy002(lab);
    }
    if (which == "PHY-003" || which == "ALL") {
        mt::resetRagdoll(lab.world, lab.ragdoll);
        testPhy003(lab);
    }
    if (which == "PHY-004" || which == "ALL") {
        mt::resetRagdoll(lab.world, lab.ragdoll);
        testPhy004(lab);
    }
    if (which == "PHY-006" || which == "ALL") {
        testPhy006(lab);
    }
    if (which == "LIM-001" || which == "ALL") {
        mt::resetRagdoll(lab.world, lab.ragdoll);
        testLim001(lab);
    }
    if (which == "ALL") {
        runAllMotorTests();
        runAllGrabTests();
        runAllFootTests();
        runAllBalanceTests();
        runAllEquilibriumTests();
        runAllStabTests();
        runAllAlignTests();
        runAllStepTests();
        runAllUnloadTests();
    }
    if (which != "ALL" && which != "RIG-001" && which != "RIG-002" && which != "RIG-003" && which != "RIG-004" &&
        which != "RIG-005" && which != "RIG-006" && which != "PHY-001" && which != "PHY-002" && which != "PHY-003" &&
        which != "PHY-004" && which != "PHY-005" && which != "PHY-006" && which != "LIM-001") {
        std::cerr << "unknown test " << which << '\n';
        return 2;
    }
    std::cout << (gFailures == 0 ? "ALL PASSED\n" : "FAILURES " + std::to_string(gFailures) + "\n");
    return gFailures == 0 ? 0 : 1;
}
