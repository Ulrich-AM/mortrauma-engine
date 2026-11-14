#include "mortrauma/assets/ModelLoader.hpp"
#include "mortrauma/balance/BalanceController.hpp"
#include "mortrauma/balance/StandingPose.hpp"
#include "mortrauma/core/Paths.hpp"
#include "mortrauma/motors/MotorController.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

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

bool loadLab(Lab& lab, bool floor) {
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
        return false;
    }
    lab.ragdoll = mt::RagdollBuilder::build(lab.world, config, lab.model, preset);
    if (!lab.ragdoll.ok || !lab.motors.bind(lab.world, lab.ragdoll) || !lab.balance.loadPreset("normal", error)) {
        return false;
    }
    lab.motors.loadDefaultProfiles();
    if (floor) {
        mt::BodyCreateInfo ground;
        ground.name = "Floor";
        ground.isStatic = true;
        ground.shape = mt::BodyCreateInfo::Shape::Box;
        ground.halfExtents = {8.f, 0.1f, 8.f};
        ground.position = {0.f, -0.1f, 0.f};
        ground.friction = 0.9f;
        if (lab.world.createBody(ground) < 0) {
            return false;
        }
    }
    return true;
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

std::vector<mt::Vec3> footPoints(const Lab& lab, const mt::RagdollBody& foot) {
    const mt::PosePalette palette = mt::posePalette(lab.world, lab.ragdoll, lab.model);
    std::vector<mt::Vec3> points;
    for (const auto& mesh : lab.model.meshes) {
        bool owned = false;
        for (int bone : foot.bones) {
            owned = owned || mesh.bone == bone;
        }
        if (!owned || mesh.bone < 0 || mesh.bone >= int(palette.boneWorld.size())) {
            continue;
        }
        for (const auto& vertex : mesh.vertices) {
            points.push_back(mt::transformPoint(palette.boneWorld[mesh.bone], vertex.position));
        }
    }
    return points;
}

float soleAngleDeg(const Lab& lab, const mt::RagdollBody& foot) {
    auto points = footPoints(lab, foot);
    if (points.size() < 30) {
        return 180.f;
    }
    std::sort(points.begin(), points.end(), [](mt::Vec3 a, mt::Vec3 b) { return a.y < b.y; });
    const size_t count = std::max<size_t>(12, points.size() / 8);
    points.resize(count);
    mt::Vec3 visual{};
    if (!mt::fitThinNormal(points, visual)) {
        return 180.f;
    }
    if (visual.y < 0.f) {
        visual = -visual;
    }
    const auto state = lab.world.bodyState(foot.physics);
    const mt::Vec3 physical = mt::normalize(mt::rotate(state.rotation, foot.soleNormalLocal));
    return mt::degrees(std::acos(std::clamp(mt::dot(visual, physical), -1.f, 1.f)));
}

void test001() {
    Lab lab;
    expect(loadLab(lab, true) && place(lab), "ALIGN-001 placed");
    for (const char* id : {"foot_1", "foot_2"}) {
        const auto* foot = lab.ragdoll.find(id);
        const float angle = soleAngleDeg(lab, *foot);
        std::cout << id << " visual-to-box sole " << angle << " deg\n";
        expect(angle < 2.f, std::string(id) + " visual and physical soles agree within 2 deg");
    }
}

void test002() {
    Lab lab;
    expect(loadLab(lab, false), "ALIGN-002 built");
    for (const auto& joint : lab.ragdoll.joints) {
        if (joint.id != "ankle_1" && joint.id != "ankle_2") {
            continue;
        }
        const auto state = lab.world.jointState(joint.physics);
        const auto& foot = lab.ragdoll.bodies[joint.child];
        const float along = mt::length(state.anchorParentWorld - foot.segmentStart);
        expect(state.positionLocked && state.separation < 0.005f && along < 0.02f, joint.id + " stays on the ankle pivot");
    }
}

void test003() {
    Lab lab;
    expect(loadLab(lab, true) && place(lab), "ALIGN-003 placed");
    for (const char* id : {"foot_1", "foot_2"}) {
        const auto* foot = lab.ragdoll.find(id);
        const auto state = lab.world.bodyState(foot->physics);
        const mt::Vec3 heel = state.position + mt::rotate(state.rotation, foot->heelLocal);
        const mt::Vec3 toe = state.position + mt::rotate(state.rotation, foot->forefootLocal);
        const mt::Vec3 forward = mt::normalize(toe - heel);
        auto points = footPoints(lab, *foot);
        float minAlong = 1e9f;
        float maxAlong = -1e9f;
        for (const auto& point : points) {
            const float along = mt::dot(point - heel, forward);
            minAlong = std::min(minAlong, along);
            maxAlong = std::max(maxAlong, along);
        }
        const float toeAlong = mt::dot(toe - heel, forward);
        std::cout << id << " heelBehindMesh " << -minAlong << " toePastMesh " << toeAlong - maxAlong << '\n';
        expect(std::fabs(minAlong) < 0.03f && std::fabs(toeAlong - maxAlong) < 0.03f, std::string(id) + " heel and toe sit on the visible ends");
    }
}

void test004() {
    Lab lab;
    expect(loadLab(lab, true) && place(lab), "ALIGN-004 placed");
    lab.balance.setEnabled(true, lab.motors);
    const float dt = lab.world.settings().fixedTimestep;
    bool contact = false;
    for (int i = 0; i < 40; ++i) {
        lab.balance.observe(lab.world, lab.ragdoll, dt);
        lab.balance.command(lab.ragdoll, lab.motors);
        lab.motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
        contact = contact || (lab.balance.state().leftPoints > 0 && lab.balance.state().rightPoints > 0);
    }
    expect(contact, "ALIGN-004 both feet make floor contact");
}

void test005() {
    Lab lab;
    expect(loadLab(lab, false), "ALIGN-005 built");
    for (const char* id : {"foot_1", "foot_2"}) {
        const auto* foot = lab.ragdoll.find(id);
        const auto points = footPoints(lab, *foot);
        const auto state = lab.world.bodyState(foot->physics);
        const mt::Quat orient = mt::normalize(mt::mul(state.rotation, foot->shapeRotationLocal));
        const mt::Vec3 center = state.position + mt::rotate(state.rotation, foot->shapeCenterLocal);
        mt::Vec3 span{};
        for (const auto& point : points) {
            const mt::Vec3 local = mt::rotate(mt::conjugate(orient), point - center);
            span.x = std::max(span.x, std::fabs(local.x));
            span.y = std::max(span.y, std::fabs(local.y));
            span.z = std::max(span.z, std::fabs(local.z));
        }
        std::cout << id << " half " << foot->boxHalfExtents.x << " " << foot->boxHalfExtents.y << " " << foot->boxHalfExtents.z << " span " << span.x
                  << " " << span.y << " " << span.z << '\n';
        const mt::Vec3 miss{std::fabs(span.x - foot->boxHalfExtents.x), std::fabs(span.y - foot->boxHalfExtents.y),
                            std::fabs(span.z - foot->boxHalfExtents.z)};
        expect(miss.x < 0.002f && miss.y < 0.002f && miss.z < 0.002f, std::string(id) + " box is the mesh bound, not a resized pad");
        expect(std::fabs(foot->mass - 1.3f) < 0.01f, std::string(id) + " mass unchanged");
    }
}

void test006() {
    Lab lab;
    expect(loadLab(lab, true) && place(lab), "ALIGN-006 placed inside the joint limits");
    for (const auto& joint : lab.ragdoll.joints) {
        const auto state = lab.world.jointState(joint.physics);
        expect(state.withinLimits, joint.id + " standing pose respects the limit");
    }
}

void test007() {
    Lab lab;
    expect(loadLab(lab, true) && place(lab), "ALIGN-007 placed");
    for (const char* id : {"foot_1", "foot_2"}) {
        const auto* foot = lab.ragdoll.find(id);
        const auto view = lab.world.shapeView(foot->physics);
        const mt::Vec3 center = view.position + mt::rotate(view.rotation, view.centerOffset);
        const mt::Quat orient = mt::normalize(mt::mul(view.rotation, view.shapeRotation));
        mt::Vec3 drawnMin{1e9f, 1e9f, 1e9f};
        mt::Vec3 drawnMax{-1e9f, -1e9f, -1e9f};
        for (float sx : {-1.f, 1.f}) {
            for (float sy : {-1.f, 1.f}) {
                for (float sz : {-1.f, 1.f}) {
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
        expect(lab.world.worldBounds(foot->physics, joltMin, joltMax), std::string(id) + " bounds");
        const float miss = std::max(std::fabs(drawnMin.x - joltMin.x),
                                    std::max(std::fabs(drawnMax.x - joltMax.x),
                                             std::max(std::fabs(drawnMin.y - joltMin.y),
                                                      std::max(std::fabs(drawnMax.y - joltMax.y),
                                                               std::max(std::fabs(drawnMin.z - joltMin.z), std::fabs(drawnMax.z - joltMax.z))))));
        expect(miss < 0.002f, std::string(id) + " debug box matches Jolt");
    }
}

void test008() {
    Lab lab;
    expect(loadLab(lab, false), "ALIGN-008 loaded");
    int vertices = 0;
    for (const auto& mesh : lab.model.meshes) {
        vertices += int(mesh.vertices.size());
    }
    expect(lab.model.bones.size() == 28 && lab.model.meshes.size() == 56 && vertices == 12656, "ALIGN-008 the imported Humanoidguy mesh is unchanged");
    expect(lab.model.findBone("Heel_1") >= 0 && lab.model.findBone("Foot_1") >= 0 && lab.model.findBone("Toes_1") >= 0, "ALIGN-008 foot bones remain");
}

void test009() {
    Lab lab;
    expect(loadLab(lab, true) && place(lab), "ALIGN-009 placed");
    lab.balance.setEnabled(true, lab.motors);
    const float dt = lab.world.settings().fixedTimestep;
    float slip = 0.f;
    float minY = 1e9f;
    for (int i = 0; i < 240; ++i) {
        lab.balance.observe(lab.world, lab.ragdoll, dt);
        lab.balance.command(lab.ragdoll, lab.motors);
        lab.motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
        minY = std::min(minY, lab.balance.state().com.y);
        if (i > 60) {
            for (const char* id : {"foot_1", "foot_2"}) {
                const auto sample = lab.world.bodyState(lab.ragdoll.find(id)->physics);
                slip = std::max(slip, std::hypot(sample.linearVelocity.x, sample.linearVelocity.z));
            }
        }
    }
    std::cout << "slip " << slip << " minY " << minY << '\n';
    expect(lab.balance.state().finite && minY > 0.9f && slip < 0.75f && !lab.world.supportActive(), "ALIGN-009 the stand does not introduce a new slide or a fixture");
}

void test010() {
    Lab lab;
    expect(loadLab(lab, true) && place(lab), "ALIGN-010 placed");
    lab.balance.setEnabled(true, lab.motors);
    const float dt = lab.world.settings().fixedTimestep;
    float minY = 1e9f;
    float tilt = 0.f;
    for (int i = 0; i < 120; ++i) {
        lab.balance.observe(lab.world, lab.ragdoll, dt);
        lab.balance.command(lab.ragdoll, lab.motors);
        lab.motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
        minY = std::min(minY, lab.balance.state().com.y);
        tilt = std::max(tilt, lab.balance.state().torsoTilt);
    }
    std::cout << "first second minY " << minY << " tilt " << tilt << '\n';
    expect(minY > 0.75f && tilt < 0.45f && lab.balance.state().phase != mt::BalancePhase::Falling, "ALIGN-010 the first second of balance still holds");
}

}  // namespace

void runAlignTest(const std::string& which) {
    if (which == "ALIGN-001") test001();
    else if (which == "ALIGN-002") test002();
    else if (which == "ALIGN-003") test003();
    else if (which == "ALIGN-004") test004();
    else if (which == "ALIGN-005") test005();
    else if (which == "ALIGN-006") test006();
    else if (which == "ALIGN-007") test007();
    else if (which == "ALIGN-008") test008();
    else if (which == "ALIGN-009") test009();
    else if (which == "ALIGN-010") test010();
    else {
        std::cerr << "unknown test " << which << '\n';
        ++gFailures;
    }
}

void runAllAlignTests() {
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
