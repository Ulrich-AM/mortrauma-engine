#include "mortrauma/assets/ModelLoader.hpp"
#include "mortrauma/core/Log.hpp"
#include "mortrauma/core/Paths.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

int gFailures = 0;

void expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++gFailures;
    } else {
        std::cout << "ok: " << message << '\n';
    }
}

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

int main(int argc, char** argv) {
    const std::string which = argc > 1 ? argv[1] : "ALL";
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
    if (which != "ALL" && which != "RIG-001" && which != "RIG-002" && which != "RIG-003" && which != "PHY-001" &&
        which != "PHY-002" && which != "PHY-003" && which != "PHY-004" && which != "PHY-005" && which != "PHY-006") {
        std::cerr << "unknown test " << which << '\n';
        return 2;
    }
    std::cout << (gFailures == 0 ? "ALL PASSED\n" : "FAILURES " + std::to_string(gFailures) + "\n");
    return gFailures == 0 ? 0 : 1;
}
