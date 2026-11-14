#include "mortrauma/assets/ModelLoader.hpp"
#include "mortrauma/balance/BalanceController.hpp"
#include "mortrauma/balance/StandingPose.hpp"
#include "mortrauma/core/Paths.hpp"
#include "mortrauma/motors/MotorController.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

extern int gFailures;
void expect(bool condition, const std::string& message);

namespace {

struct Lab {
    mt::PhysicsWorld world;
    mt::Ragdoll ragdoll;
    int floor = -1;
    int bodiesAtRest = 0;
};

bool loadLab(Lab& lab) {
    mt::PhysicsSettings settings;
    settings.allowSleeping = true;
    if (!lab.world.initialize(settings)) {
        return false;
    }
    const auto config = mt::loadCharacterConfig(mt::findAsset("configurations/humanoidguy.json").string());
    const auto model = mt::ModelLoader::load(mt::findAsset("characters/humanoidguy_rigged.fbx").string());
    std::string error;
    const auto preset = mt::loadJointPreset(mt::findAsset("configurations/presets/natural.json").string(), error);
    if (!model.report.ok || !config.error.empty() || !error.empty()) {
        return false;
    }
    lab.ragdoll = mt::RagdollBuilder::build(lab.world, config, model, preset);
    if (!lab.ragdoll.ok) {
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
    lab.bodiesAtRest = lab.world.bodyCount();
    return lab.floor >= 0;
}

void step(Lab& lab, int count) {
    const float dt = lab.world.settings().fixedTimestep;
    for (int i = 0; i < count; ++i) {
        lab.world.step(dt, 1);
    }
}

bool finiteRagdoll(const Lab& lab) {
    for (const auto& body : lab.ragdoll.bodies) {
        const auto state = lab.world.bodyState(body.physics);
        if (!state.valid || !mt::finite(state.position) || !mt::finite(state.linearVelocity) || !mt::finite(state.angularVelocity) ||
            !mt::finite(state.rotation)) {
            return false;
        }
    }
    return true;
}

int firstDynamic(const Lab& lab) {
    return lab.ragdoll.bodies.front().physics;
}

bool open(Lab& lab, const char* name) {
    if (!loadLab(lab) || lab.ragdoll.bodies.empty()) {
        expect(false, std::string(name) + " laboratory loaded");
        return false;
    }
    expect(true, std::string(name) + " laboratory loaded");
    return true;
}

}  // namespace

void testGrab001() {
    Lab lab;
    if (!open(lab, "GRAB-001")) {
        return;
    }
    const int body = firstDynamic(lab);
    for (int i = 0; i < 25; ++i) {
        const auto point = lab.world.bodyState(body).position;
        const int grab = lab.world.beginGrab(body, point, point, 8.f, 0.8f);
        expect(grab >= 0 && lab.world.grabAlive(grab), "GRAB-001 grab " + std::to_string(i) + " begins");
        step(lab, i == 0 ? 240 : 5);
        lab.world.updateGrab(grab, point + mt::Vec3{0.05f * std::sin(float(i)), 0.02f, 0.04f});
        step(lab, 3);
        const int anchor = lab.world.grabAnchor(grab);
        lab.world.endGrab(grab);
        expect(!lab.world.grabAlive(grab), "GRAB-001 grab released");
        expect(!lab.world.bodyState(anchor).valid, "GRAB-001 anchor destroyed");
    }
    expect(lab.world.bodyCount() == lab.bodiesAtRest, "GRAB-001 body count restored");
    expect(finiteRagdoll(lab), "GRAB-001 state stayed finite");
}

void testGrab002() {
    Lab lab;
    if (!open(lab, "GRAB-002")) {
        return;
    }
    const auto point = lab.world.bodyState(firstDynamic(lab)).position;
    const int grab = lab.world.beginGrab(firstDynamic(lab), point, point, 8.f, 0.8f);
    step(lab, 15);
    mt::resetRagdoll(lab.world, lab.ragdoll);
    expect(!lab.world.grabAlive(grab), "GRAB-002 reset releases the grab");
    lab.world.endGrab(grab);
    lab.world.updateGrab(grab, point + mt::Vec3{1.f, 0.f, 0.f});
    step(lab, 20);
    expect(lab.world.bodyCount() == lab.bodiesAtRest, "GRAB-002 no leftover anchor");
    expect(finiteRagdoll(lab), "GRAB-002 finite after reset");
}

void testGrab003() {
    Lab lab;
    if (!open(lab, "GRAB-003")) {
        return;
    }
    mt::MotorController motors;
    expect(motors.bind(lab.world, lab.ragdoll), "GRAB-003 motors bind");
    motors.loadDefaultProfiles();
    motors.setMode(mt::MotorMode::Active);
    motors.activateAll();
    const int body = lab.ragdoll.find("forearm_1")->physics;
    const auto point = lab.world.bodyState(body).position;
    const int grab = lab.world.beginGrab(body, point, point, 10.f, 0.6f);
    expect(grab >= 0, "GRAB-003 grab while active");
    for (int i = 0; i < 40; ++i) {
        expect(motors.apply(lab.world, lab.ragdoll), "GRAB-003 motors apply");
        lab.world.updateGrab(grab, point + mt::Vec3{0.1f, 0.15f * std::sin(i * 0.2f), 0.05f});
        step(lab, 1);
    }
    lab.world.endGrab(grab);
    expect(motors.apply(lab.world, lab.ragdoll), "GRAB-003 motors apply after release");
    expect(finiteRagdoll(lab), "GRAB-003 finite with motors");
    expect(lab.world.bodyCount() == lab.bodiesAtRest, "GRAB-003 anchors removed");
}

void testGrab004() {
    Lab lab;
    if (!open(lab, "GRAB-004")) {
        return;
    }
    const auto* pelvis = lab.ragdoll.find("pelvis");
    expect(lab.world.beginSupport(pelvis->physics, 6.f, 1.f), "GRAB-004 support on");
    const int hand = lab.ragdoll.find("hand_2")->physics;
    const auto point = lab.world.bodyState(hand).position;
    const int grab = lab.world.beginGrab(hand, point, point, 8.f, 0.8f);
    expect(grab >= 0, "GRAB-004 grab with support");
    for (int i = 0; i < 30; ++i) {
        lab.world.updateGrab(grab, point + mt::Vec3{0.2f, 0.1f, -0.1f});
        step(lab, 1);
    }
    lab.world.endGrab(grab);
    expect(lab.world.supportActive(), "GRAB-004 support remains after release");
    step(lab, 10);
    lab.world.endSupport();
    expect(!lab.world.supportActive(), "GRAB-004 support removed");
    expect(finiteRagdoll(lab), "GRAB-004 finite");
    expect(lab.world.bodyCount() == lab.bodiesAtRest, "GRAB-004 temporary bodies removed");
}

void testGrab005() {
    Lab lab;
    if (!open(lab, "GRAB-005")) {
        return;
    }
    const int body = firstDynamic(lab);
    const auto origin = lab.world.bodyState(body).position;
    const int grab = lab.world.beginGrab(body, origin, origin, 14.f, 0.2f);
    expect(grab >= 0, "GRAB-005 grab begins");
    const mt::Vec3 wild[] = {origin + mt::Vec3{30.f, 10.f, -20.f}, origin + mt::Vec3{-8.f, 4.f, 12.f},
                             {std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f}, origin + mt::Vec3{0.4f, 0.2f, -0.3f}};
    for (int i = 0; i < 48; ++i) {
        lab.world.updateGrab(grab, wild[i % 4]);
        step(lab, 1);
        expect(finiteRagdoll(lab), "GRAB-005 finite during a fast drag");
        if (gFailures > 8) {
            break;
        }
    }
    lab.world.endGrab(grab);
    step(lab, 10);
    expect(finiteRagdoll(lab), "GRAB-005 finite after release");
    expect(lab.world.bodyCount() == lab.bodiesAtRest, "GRAB-005 anchor removed");
}

void testGrab006() {
    Lab lab;
    if (!open(lab, "GRAB-006")) {
        return;
    }
    const int joints = lab.world.jointCount();
    const auto point = lab.world.bodyState(firstDynamic(lab)).position;
    const int grab = lab.world.beginGrab(firstDynamic(lab), point, point, 8.f, 0.8f);
    const int anchor = lab.world.grabAnchor(grab);
    expect(anchor >= 0, "GRAB-006 anchor exists");
    lab.world.endGrab(grab);
    lab.world.endGrab(grab);
    lab.world.clearGrabs();
    lab.world.updateGrab(grab, point);
    expect(!lab.world.bodyState(anchor).valid, "GRAB-006 anchor id is dead");
    expect(!lab.world.grabAlive(grab), "GRAB-006 grab record is dead");
    expect(lab.world.jointCount() == joints, "GRAB-006 ragdoll joint count unchanged");
    expect(lab.world.bodyCount() == lab.bodiesAtRest, "GRAB-006 body count unchanged");
    const int again = lab.world.beginGrab(firstDynamic(lab), point, point, 8.f, 0.8f);
    expect(again >= 0, "GRAB-006 a later grab still works");
    lab.world.shutdown();
    expect(!lab.world.isReady(), "GRAB-006 shutdown during a grab");
}

void testGrab007() {
    Lab lab;
    if (!open(lab, "GRAB-007")) {
        return;
    }
    mt::MotorController motors;
    mt::BalanceController balance;
    std::string error;
    expect(balance.loadPreset("normal", error), "GRAB-007 preset " + error);
    expect(motors.bind(lab.world, lab.ragdoll), "GRAB-007 motors bind");
    motors.loadDefaultProfiles();
    const auto placement = mt::placeStandingPose(lab.world, lab.ragdoll, balance.config());
    expect(placement.ok, "GRAB-007 standing placement " + placement.error);
    balance.setReference(placement.reference);
    if (lab.world.supportActive()) {
        lab.world.endSupport();
    }
    balance.setEnabled(true, motors);
    const float dt = lab.world.settings().fixedTimestep;
    for (int i = 0; i < 30; ++i) {
        balance.observe(lab.world, lab.ragdoll, dt);
        balance.command(lab.ragdoll, motors);
        motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
    }
    const int chest = lab.ragdoll.find("chest")->physics;
    const auto point = lab.world.bodyState(chest).position;
    const int grab = lab.world.beginGrab(chest, point, point, 8.f, 0.8f);
    expect(grab >= 0, "GRAB-007 grab during balance");
    for (int i = 0; i < 40; ++i) {
        balance.observe(lab.world, lab.ragdoll, dt);
        balance.command(lab.ragdoll, motors);
        motors.apply(lab.world, lab.ragdoll);
        lab.world.updateGrab(grab, point + mt::Vec3{0.15f, 0.05f, -0.2f});
        lab.world.step(dt, 1);
    }
    lab.world.endGrab(grab);
    for (int i = 0; i < 20; ++i) {
        balance.observe(lab.world, lab.ragdoll, dt);
        balance.command(lab.ragdoll, motors);
        motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
    }
    expect(!lab.world.supportActive(), "GRAB-007 support fixture stayed off");
    expect(finiteRagdoll(lab), "GRAB-007 finite during balance grab");
    expect(!lab.world.grabAlive(grab), "GRAB-007 grab released");
}

void runGrabTest(const std::string& which) {
    if (which == "GRAB-001") testGrab001();
    else if (which == "GRAB-002") testGrab002();
    else if (which == "GRAB-003") testGrab003();
    else if (which == "GRAB-004") testGrab004();
    else if (which == "GRAB-005") testGrab005();
    else if (which == "GRAB-006") testGrab006();
    else if (which == "GRAB-007") testGrab007();
    else {
        std::cerr << "unknown grab test " << which << '\n';
        ++gFailures;
    }
}

void runAllGrabTests() {
    testGrab001();
    testGrab002();
    testGrab003();
    testGrab004();
    testGrab005();
    testGrab006();
    testGrab007();
}
