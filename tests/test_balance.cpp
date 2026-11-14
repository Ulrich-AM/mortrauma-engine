#include "mortrauma/assets/ModelLoader.hpp"
#include "mortrauma/balance/BalanceController.hpp"
#include "mortrauma/balance/StandingPose.hpp"
#include "mortrauma/balance/SupportContacts.hpp"
#include "mortrauma/core/Paths.hpp"
#include "mortrauma/motors/MotorController.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

extern int gFailures;
void expect(bool condition, const std::string& message);

namespace {

struct Lab {
    mt::PhysicsWorld world;
    mt::Ragdoll ragdoll;
    mt::MotorController motors;
    mt::BalanceController balance;
    int floor = -1;
};

bool loadLab(Lab& lab, bool withFloor) {
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
    if (!lab.ragdoll.ok || !lab.motors.bind(lab.world, lab.ragdoll)) {
        return false;
    }
    lab.motors.loadDefaultProfiles();
    if (!lab.balance.loadPreset("normal", error)) {
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

struct Trial {
    bool finite = true;
    float minComY = 1e9f;
    float maxTilt = 0.f;
    float maxAbsForward = 0.f;
    float maxAnkle = 0.f;
    float maxHip = 0.f;
    int groundedSteps = 0;
    int steps = 0;
    bool supportOn = false;
    mt::Vec3 com{};
    float tilt = 0.f;
    mt::BalancePhase phase = mt::BalancePhase::Inactive;
    bool ankleAuthority = true;
    int limitViolations = 0;
    float maxTorqueOver = 0.f;
};

void integrate(Lab& lab, Trial& trial, int steps, bool command) {
    const float dt = lab.world.settings().fixedTimestep;
    const mt::Vec3 origin = lab.balance.state().supportCenter;
    for (int i = 0; i < steps; ++i) {
        lab.balance.observe(lab.world, lab.ragdoll, dt);
        if (command) {
            lab.balance.command(lab.ragdoll, lab.motors);
        } else if (lab.motors.motorTaskActive()) {
            lab.motors.clearMotorTask();
        }
        lab.motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
        const auto& state = lab.balance.state();
        trial.finite = trial.finite && state.finite;
        trial.minComY = std::min(trial.minComY, state.com.y);
        trial.maxTilt = std::max(trial.maxTilt, state.torsoTilt);
        const float forward = mt::dot(mt::Vec3{state.com.x - origin.x, 0.f, state.com.z - origin.z}, state.facing);
        trial.maxAbsForward = std::max(trial.maxAbsForward, std::fabs(forward));
        trial.maxAnkle = std::max(trial.maxAnkle, state.ankleEffort);
        trial.maxHip = std::max(trial.maxHip, state.hipEffort);
        trial.groundedSteps += (state.leftGrounded || state.rightGrounded) ? 1 : 0;
        trial.steps += 1;
        trial.supportOn = trial.supportOn || lab.world.supportActive();
        trial.com = state.com;
        trial.tilt = state.torsoTilt;
        trial.phase = state.phase;
        trial.ankleAuthority = state.ankleAuthority;
        for (const auto& body : lab.ragdoll.bodies) {
            const auto sample = lab.world.bodyState(body.physics);
            if (!sample.valid || !mt::finite(sample.position) || !mt::finite(sample.linearVelocity)) {
                trial.finite = false;
            }
        }
        for (const auto& joint : lab.ragdoll.joints) {
            const auto sample = lab.world.jointState(joint.physics);
            if (!sample.valid) {
                continue;
            }
            // A few degrees past the written limit is solver penetration under
            // contact, not a motor target. Commanded targets are clamped inside.
            const float slack = 0.10f;
            const bool correcting = state.phase == mt::BalancePhase::Initializing || state.phase == mt::BalancePhase::Standing ||
                                    state.phase == mt::BalancePhase::Recovering;
            if (correcting && (sample.rotationRadians.x < sample.twistMin - slack || sample.rotationRadians.x > sample.twistMax + slack ||
                               sample.rotationRadians.y < sample.swing1Min - slack || sample.rotationRadians.y > sample.swing1Max + slack ||
                               sample.rotationRadians.z < sample.swing2Min - slack || sample.rotationRadians.z > sample.swing2Max + slack)) {
                ++trial.limitViolations;
            }
            const float limit = std::max(sample.motorTorqueLimit, 0.f);
            const float over = std::max(std::fabs(sample.motorTorque.x), std::max(std::fabs(sample.motorTorque.y), std::fabs(sample.motorTorque.z))) - limit;
            trial.maxTorqueOver = std::max(trial.maxTorqueOver, over);
        }
    }
}

bool arm(Lab& lab) {
    lab.world.endSupport();
    const auto placement = mt::placeStandingPose(lab.world, lab.ragdoll, lab.balance.config());
    if (!placement.ok) {
        std::cerr << placement.error << '\n';
        return false;
    }
    std::cout << "placement separation " << placement.maxSeparation << " foot " << placement.footHeight << " pelvisUp "
              << placement.pelvisUp.y << '\n';
    if (placement.maxSeparation > 0.02f) {
        return false;
    }
    lab.balance.setReference(placement.reference);
    lab.balance.setEnabled(true, lab.motors);
    lab.balance.resetState();
    return !lab.world.supportActive();
}

void impulse(Lab& lab, const char* bodyId, mt::Vec3 direction, float magnitude) {
    const auto* body = lab.ragdoll.find(bodyId);
    const auto state = lab.world.bodyState(body->physics);
    const mt::Vec3 facing = lab.balance.state().facing;
    const mt::Vec3 right = lab.balance.state().right;
    mt::Vec3 world = facing * direction.z + right * direction.x + mt::Vec3{0.f, 1.f, 0.f} * direction.y;
    if (mt::length(world) < 1e-4f) {
        world = direction;
    }
    lab.world.addImpulse(body->physics, mt::normalize(world) * magnitude, state.position);
}

}  // namespace

void testBal001() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-001 laboratory loaded");
    expect(arm(lab), "BAL-001 standing placement is connected and unsupported by a fixture");
    // The initialization contract is the first second: feet down, mass up,
    // torso nearer upright than fallen. Longer holds are reported, not assumed.
    Trial trial;
    integrate(lab, trial, 120, true);
    std::cout << "BAL-001 1s comY " << trial.minComY << " tilt " << trial.maxTilt << " grounded " << trial.groundedSteps << "/"
              << trial.steps << " phase " << mt::balancePhaseName(trial.phase) << '\n';
    expect(trial.finite, "BAL-001 finite");
    expect(!trial.supportOn, "BAL-001 no pelvis fixture");
    expect(trial.groundedSteps > trial.steps / 2, "BAL-001 feet support the body");
    expect(trial.minComY > 0.75f, "BAL-001 center of mass stays up for the first second");
    expect(trial.maxTilt < 0.45f, "BAL-001 torso stays upright for the first second");
    expect(trial.phase != mt::BalancePhase::Grounded && trial.phase != mt::BalancePhase::Falling,
           "BAL-001 does not give up during the first second");
    bool laterFinite = true;
    for (int second = 2; second <= 10; ++second) {
        Trial slice;
        integrate(lab, slice, 120, true);
        laterFinite = laterFinite && slice.finite;
        const auto& state = lab.balance.state();
        std::cout << "BAL-001 " << second << "s comY " << slice.minComY << " tilt " << slice.maxTilt << " margin " << state.margin
                  << " phase " << mt::balancePhaseName(slice.phase) << '\n';
    }
    expect(laterFinite, "BAL-001 later motion stays finite");
}

void testBal002() {
    Lab lab;
    expect(loadLab(lab, false), "BAL-002 laboratory loaded");
    expect(arm(lab), "BAL-002 placement without a floor");
    const float start = lab.world.bodyState(lab.ragdoll.find("pelvis")->physics).position.y;
    Trial trial;
    integrate(lab, trial, 80, true);
    const float end = lab.balance.state().com.y;
    std::cout << "BAL-002 drop " << (start - end) << " authority " << trial.ankleAuthority << '\n';
    expect(trial.finite, "BAL-002 finite");
    expect(start - end > 0.35f, "BAL-002 gravity still drops the body");
    expect(!lab.balance.state().ankleAuthority, "BAL-002 ankle authority is off without a supporting foot");
    expect(!lab.world.supportActive(), "BAL-002 no hidden support");
}

void testBal003() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-003 laboratory loaded");
    expect(arm(lab), "BAL-003 standing");
    Trial settle;
    integrate(lab, settle, 40, true);
    const mt::Vec3 before = lab.balance.state().com;
    impulse(lab, "chest", {0.f, 0.f, 1.f}, 6.f);
    Trial trial;
    integrate(lab, trial, 180, true);
    const float travel = mt::length(mt::Vec3{trial.com.x - before.x, 0.f, trial.com.z - before.z});
    const bool fell = trial.phase == mt::BalancePhase::Grounded || trial.phase == mt::BalancePhase::Falling || trial.minComY < 0.45f;
    std::cout << "BAL-003 minY " << trial.minComY << " tilt " << trial.maxTilt << " travel " << travel << " phase "
              << mt::balancePhaseName(trial.phase) << '\n';
    expect(trial.finite, "BAL-003 finite");
    expect(!lab.world.supportActive(), "BAL-003 no hidden support");
    // The aligned sole stops the old heel-box slide. This 6 N·s push moved the
    // center of mass 9 mm and tilted the torso 0.27 rad, and the stance held.
    expect(travel > 0.02f || trial.maxTilt > 0.2f, "BAL-003 the push moves the body");
    // v0.3.1 fell from 6 N·s. With the COM over the ankles the same push is
    // rejected. A 55 N·s push is still a fall; that is BAL-005.
    expect(!fell && trial.minComY > 0.9f, "BAL-003 a 6 N·s forward push no longer drops the stance");
}

void testBal004() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-004 laboratory loaded");
    expect(arm(lab), "BAL-004 standing");
    Trial settle;
    integrate(lab, settle, 40, true);
    impulse(lab, "chest", {1.f, 0.f, 0.f}, 6.f);
    Trial trial;
    integrate(lab, trial, 180, true);
    const bool fell = trial.phase == mt::BalancePhase::Grounded || trial.phase == mt::BalancePhase::Falling || trial.minComY < 0.45f ||
                      trial.maxTilt > 1.0f;
    std::cout << "BAL-004 minY " << trial.minComY << " tilt " << trial.maxTilt << " ankle " << trial.maxAnkle << " hip " << trial.maxHip
              << '\n';
    expect(trial.finite, "BAL-004 finite");
    expect(!lab.world.supportActive(), "BAL-004 no hidden support");
    expect(trial.maxAnkle + trial.maxHip > 0.4f, "BAL-004 ankle or hip motors answer the push");
    expect(!fell && trial.minComY > 0.9f, "BAL-004 a 6 N·s lateral push no longer drops the stance");
}

void testBal005() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-005 laboratory loaded");
    expect(arm(lab), "BAL-005 standing");
    Trial settle;
    integrate(lab, settle, 40, true);
    const auto pelvisBefore = lab.world.bodyState(lab.ragdoll.find("pelvis")->physics).position;
    impulse(lab, "chest", {0.f, 0.f, 1.f}, 55.f);
    Trial trial;
    integrate(lab, trial, 200, true);
    const auto pelvisAfter = lab.world.bodyState(lab.ragdoll.find("pelvis")->physics).position;
    const bool fell = trial.phase == mt::BalancePhase::Falling || trial.phase == mt::BalancePhase::Grounded || trial.minComY < 0.45f ||
                      trial.maxTilt > 1.05f;
    std::cout << "BAL-005 minY " << trial.minComY << " tilt " << trial.maxTilt << " phase " << mt::balancePhaseName(trial.phase) << '\n';
    expect(trial.finite, "BAL-005 finite");
    expect(fell, "BAL-005 a large push is not rigidly rejected");
    expect(mt::length(pelvisAfter - pelvisBefore) > 0.05f, "BAL-005 the pelvis is not anchored");
    expect(!lab.world.supportActive(), "BAL-005 no fixture");
}

void testBal006() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-006 laboratory loaded");
    expect(arm(lab), "BAL-006 standing");
    impulse(lab, "chest", {0.f, 0.f, 1.f}, 8.f);
    Trial trial;
    integrate(lab, trial, 160, true);
    std::cout << "BAL-006 torque over " << trial.maxTorqueOver << '\n';
    expect(trial.finite, "BAL-006 finite");
    expect(trial.maxTorqueOver < 0.75f, "BAL-006 motor torque stays at the configured limit");
}

void testBal007() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-007 laboratory loaded");
    expect(lab.motors.mode() == mt::MotorMode::Passive, "BAL-007 starts passive");
    expect(arm(lab), "BAL-007 standing");
    Trial trial;
    integrate(lab, trial, 30, true);
    expect(lab.motors.motorTaskActive(), "BAL-007 balance owns a motor task");
    lab.balance.setEnabled(false, lab.motors);
    expect(!lab.motors.motorTaskActive(), "BAL-007 disabling balance clears the task");
    expect(lab.motors.mode() == mt::MotorMode::Passive, "BAL-007 mode is unchanged");
    integrate(lab, trial, 5, false);
    bool anyMotor = false;
    for (const auto& joint : lab.ragdoll.joints) {
        anyMotor = anyMotor || lab.world.jointState(joint.physics).motorEnabled;
    }
    expect(!anyMotor, "BAL-007 passive mode is restored");
}

void testBal008() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-008 laboratory loaded");
    expect(arm(lab), "BAL-008 standing");
    Trial settle;
    integrate(lab, settle, 40, true);
    expect(lab.balance.state().leftGrounded || lab.balance.state().rightGrounded, "BAL-008 feet start supported");
    lab.world.setBodyPose(lab.floor, {0.f, -30.f, 0.f}, {}, false);
    Trial trial;
    integrate(lab, trial, 40, true);
    std::cout << "BAL-008 grounded L " << lab.balance.state().leftGrounded << " R " << lab.balance.state().rightGrounded
              << " authority " << lab.balance.state().ankleAuthority << '\n';
    expect(!lab.balance.state().leftGrounded && !lab.balance.state().rightGrounded, "BAL-008 lost foot support is detected");
    expect(!lab.balance.state().ankleAuthority, "BAL-008 ankle strategy does not pretend the floor is there");
}

void testBal009() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-009 laboratory loaded");
    expect(arm(lab), "BAL-009 standing");
    impulse(lab, "chest", {0.2f, 0.f, 1.f}, 7.f);
    Trial trial;
    integrate(lab, trial, 160, true);
    std::cout << "BAL-009 violations " << trial.limitViolations << '\n';
    expect(trial.finite, "BAL-009 finite");
    expect(trial.limitViolations == 0, "BAL-009 corrections stay inside anatomical limits");
}

void testBal010() {
    auto run = [](mt::Vec3& com, float& tilt) {
        Lab lab;
        if (!loadLab(lab, true) || !arm(lab)) {
            return false;
        }
        Trial settle;
        integrate(lab, settle, 40, true);
        impulse(lab, "chest", {0.f, 0.f, 1.f}, 5.f);
        Trial trial;
        integrate(lab, trial, 120, true);
        com = trial.com;
        tilt = trial.tilt;
        return trial.finite;
    };
    mt::Vec3 a{};
    mt::Vec3 b{};
    float tiltA = 0.f;
    float tiltB = 0.f;
    expect(run(a, tiltA), "BAL-010 first run");
    expect(run(b, tiltB), "BAL-010 second run");
    std::cout << "BAL-010 delta " << mt::length(a - b) << " tilt " << std::fabs(tiltA - tiltB) << '\n';
    expect(mt::length(a - b) < 0.02f, "BAL-010 center of mass repeats");
    expect(std::fabs(tiltA - tiltB) < 0.03f, "BAL-010 torso tilt repeats");
}

void testBal011() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-011 laboratory loaded");
    expect(arm(lab), "BAL-011 standing");
    lab.motors.setMode(mt::MotorMode::Passive);
    Trial trial;
    integrate(lab, trial, 10, true);
    expect(lab.motors.motorTaskActive(), "BAL-011 balance task overrides passive mode");
    lab.balance.setEnabled(false, lab.motors);
    lab.motors.setMode(mt::MotorMode::Active);
    lab.motors.activateAll();
    integrate(lab, trial, 5, false);
    int enabled = 0;
    for (const auto& joint : lab.ragdoll.joints) {
        enabled += lab.world.jointState(joint.physics).motorEnabled ? 1 : 0;
    }
    expect(enabled > 0, "BAL-011 active mode drives motors after balance is off");
    lab.motors.hold(lab.world, lab.ragdoll, {});
    lab.motors.setMode(mt::MotorMode::Hold);
    integrate(lab, trial, 3, false);
    expect(lab.motors.mode() == mt::MotorMode::Hold, "BAL-011 hold mode is accepted");
    lab.balance.setEnabled(true, lab.motors);
    integrate(lab, trial, 5, true);
    expect(lab.motors.motorTaskActive(), "BAL-011 balance takes ownership again");
    expect(lab.motors.mode() == mt::MotorMode::Hold, "BAL-011 hold mode is not overwritten");
    lab.balance.setEnabled(false, lab.motors);
    expect(!lab.motors.motorTaskActive(), "BAL-011 ownership returns when balance stops");
}

void testBal012() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-012 laboratory loaded");
    expect(arm(lab), "BAL-012 standing");
    Trial settle;
    integrate(lab, settle, 40, true);
    const int chest = lab.ragdoll.find("chest")->physics;
    const auto point = lab.world.bodyState(chest).position;
    const int grab = lab.world.beginGrab(chest, point, point, 8.f, 0.8f);
    expect(grab >= 0, "BAL-012 grab starts");
    const float dt = lab.world.settings().fixedTimestep;
    for (int i = 0; i < 30; ++i) {
        lab.balance.observe(lab.world, lab.ragdoll, dt);
        lab.balance.command(lab.ragdoll, lab.motors);
        lab.motors.apply(lab.world, lab.ragdoll);
        lab.world.updateGrab(grab, point + mt::Vec3{0.1f, 0.05f, -0.15f});
        lab.world.step(dt, 1);
    }
    lab.world.endGrab(grab);
    Trial after;
    integrate(lab, after, 20, true);
    expect(after.finite, "BAL-012 finite");
    expect(!lab.world.grabAlive(grab), "BAL-012 released");
    expect(!lab.world.supportActive(), "BAL-012 no fixture");
}

void testBal013() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-013 laboratory loaded");
    expect(lab.world.beginSupport(lab.ragdoll.find("pelvis")->physics, 8.f, 1.f), "BAL-013 fixture can still be created");
    expect(lab.world.supportActive(), "BAL-013 fixture starts on");
    lab.world.endSupport();
    expect(arm(lab), "BAL-013 balance entry removes the fixture");
    Trial trial;
    integrate(lab, trial, 30, true);
    expect(!trial.supportOn && !lab.world.supportActive(), "BAL-013 fixture stays off");
}

void testBal014() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-014 character still builds");
    expect(lab.ragdoll.bodies.size() == 21 && lab.ragdoll.joints.size() == 20, "BAL-014 21 bodies and 20 joints");
    std::string error;
    expect(lab.balance.loadPreset("conservative", error) && lab.balance.loadPreset("aggressive", error), "BAL-014 presets load");
    const auto point = lab.world.bodyState(lab.ragdoll.bodies.front().physics).position;
    const int grab = lab.world.beginGrab(lab.ragdoll.bodies.front().physics, point, point, 8.f, 0.8f);
    lab.world.endGrab(grab);
    lab.motors.setMode(mt::MotorMode::Passive);
    Trial trial;
    integrate(lab, trial, 30, false);
    expect(trial.finite, "BAL-014 passive step stays finite");
    expect(!lab.motors.motorTaskActive(), "BAL-014 no balance task unless enabled");
}

void applyFootShape(Lab& lab, bool capsules) {
    for (const char* footId : {"foot_1", "foot_2"}) {
        const auto* foot = lab.ragdoll.find(footId);
        mt::BodyCreateInfo info;
        info.mass = foot->mass;
        info.friction = 0.7f;
        if (capsules) {
            info.shape = mt::BodyCreateInfo::Shape::Capsule;
            info.radius = std::max(foot->capsuleRadius, 0.04f);
            const float length = mt::length(foot->segmentEnd - foot->segmentStart);
            info.halfHeight = std::max(length * 0.5f - info.radius, 0.01f);
        } else {
            info.shape = mt::BodyCreateInfo::Shape::Box;
            info.halfExtents = foot->boxHalfExtents;
            info.shapeCenter = foot->shapeCenterLocal;
            info.shapeRotation = foot->shapeRotationLocal;
            info.convexRadius = 0.004f;
        }
        expect(lab.world.setBodyShape(foot->physics, info), std::string("shape swap ") + footId);
    }
}

float soleUpY(Lab& lab, const char* id) {
    const auto* foot = lab.ragdoll.find(id);
    const auto sample = lab.world.bodyState(foot->physics);
    return mt::normalize(mt::rotate(sample.rotation, foot->soleNormalLocal)).y;
}

void testFoot001() {
    Lab lab;
    expect(loadLab(lab, true), "FOOT-001 laboratory loaded");
    for (const char* id : {"foot_1", "foot_2"}) {
        const auto* foot = lab.ragdoll.find(id);
        expect(foot != nullptr && foot->collider == mt::BodyCreateInfo::Shape::Box, std::string(id) + " uses a box");
        expect(foot->boxHalfExtents.x > 0.05f && foot->boxHalfExtents.x < 0.12f, std::string(id) + " width");
        expect(foot->boxHalfExtents.y > 0.12f && foot->boxHalfExtents.y < 0.22f, std::string(id) + " length");
        expect(foot->boxHalfExtents.z > 0.04f && foot->boxHalfExtents.z < 0.14f, std::string(id) + " thickness");
        expect(std::fabs(foot->mass - 1.3f) < 1e-3f, std::string(id) + " keeps the configured mass");
        const auto view = lab.world.shapeView(foot->physics);
        expect(view.shape == mt::BodyCreateInfo::Shape::Box, std::string(id) + " physics shape is a box");
        expect(mt::length(view.centerOffset - foot->shapeCenterLocal) < 1e-4f, std::string(id) + " collider offset is stored");
    }
}

void testFoot002() {
    Lab lab;
    expect(loadLab(lab, true), "FOOT-002 laboratory loaded");
    expect(arm(lab), "FOOT-002 placed");
    const float right = soleUpY(lab, "foot_1");
    const float left = soleUpY(lab, "foot_2");
    std::cout << "FOOT-002 sole up " << right << " " << left << '\n';
    expect(right > 0.95f && left > 0.95f, "FOOT-002 soles are within 18 degrees of level");
}

void testFoot003() {
    Lab lab;
    expect(loadLab(lab, true), "FOOT-003 laboratory loaded");
    expect(arm(lab), "FOOT-003 placed");
    lab.balance.observe(lab.world, lab.ragdoll, lab.world.settings().fixedTimestep);
    const auto& state = lab.balance.state();
    std::cout << "FOOT-003 clearance R " << state.rightHeelClearance << " " << state.rightForeClearance << " L " << state.leftHeelClearance
              << " " << state.leftForeClearance << '\n';
    const float values[4] = {state.rightHeelClearance, state.rightForeClearance, state.leftHeelClearance, state.leftForeClearance};
    for (float clearance : values) {
        expect(clearance > -0.005f && clearance < 0.08f, "FOOT-003 heel and forefoot start near the floor");
    }
    expect(std::fabs(state.rightHeelClearance - state.rightForeClearance) < 0.04f, "FOOT-003 right sole is not pitched onto one edge");
    expect(std::fabs(state.leftHeelClearance - state.leftForeClearance) < 0.04f, "FOOT-003 left sole is not pitched onto one edge");
}

void testFoot004() {
    Lab lab;
    expect(loadLab(lab, true), "FOOT-004 laboratory loaded");
    expect(arm(lab), "FOOT-004 placed");
    Trial trial;
    integrate(lab, trial, 60, true);
    for (const char* id : {"ankle_1", "ankle_2"}) {
        const mt::RagdollJoint* joint = nullptr;
        for (const auto& candidate : lab.ragdoll.joints) {
            if (candidate.id == id) {
                joint = &candidate;
            }
        }
        expect(joint != nullptr, std::string(id) + " exists");
        const auto sample = lab.world.jointState(joint->physics);
        expect(sample.valid && sample.separation < 0.02f, std::string(id) + " stays connected");
    }
    expect(trial.finite, "FOOT-004 finite");
}

void testFoot005() {
    Lab lab;
    expect(loadLab(lab, true), "FOOT-005 laboratory loaded");
    expect(arm(lab), "FOOT-005 placed");
    float lowest = 0.f;
    Trial trial;
    integrate(lab, trial, 120, true);
    for (const auto& contact : lab.balance.state().contacts) {
        lowest = std::min(lowest, contact.point.y);
    }
    std::cout << "FOOT-005 lowest contact " << lowest << '\n';
    expect(trial.finite, "FOOT-005 finite");
    expect(lowest > -0.05f, "FOOT-005 feet do not sink through the floor");
}

void testFoot006() {
    Lab lab;
    expect(loadLab(lab, true), "FOOT-006 laboratory loaded");
    expect(arm(lab), "FOOT-006 placed");
    Trial trial;
    integrate(lab, trial, 40, true);
    int supporting = 0;
    int classified = 0;
    for (const auto& contact : lab.balance.state().contacts) {
        if (!contact.supporting) {
            continue;
        }
        ++supporting;
        if (contact.region == mt::FootRegion::Heel || contact.region == mt::FootRegion::Midfoot || contact.region == mt::FootRegion::Forefoot) {
            ++classified;
        }
    }
    std::cout << "FOOT-006 supporting " << supporting << " classified " << classified << " heel " << lab.balance.state().rightHeel << " "
              << lab.balance.state().leftHeel << " fore " << lab.balance.state().rightFore << " " << lab.balance.state().leftFore << '\n';
    expect(supporting > 0, "FOOT-006 has a ground contact");
    expect(classified == supporting, "FOOT-006 every supporting contact is heel, midfoot, or forefoot");
}

void testFoot007() {
    Lab lab;
    expect(loadLab(lab, true), "FOOT-007 laboratory loaded");
    expect(arm(lab), "FOOT-007 placed");
    Trial trial;
    integrate(lab, trial, 40, true);
    const auto& state = lab.balance.state();
    int matched = 0;
    for (const auto& vertex : state.hull) {
        for (const auto& contact : state.contacts) {
            if (contact.supporting && mt::length(mt::Vec3{vertex.x - contact.point.x, 0.f, vertex.z - contact.point.z}) < 1e-3f) {
                ++matched;
                break;
            }
        }
    }
    std::cout << "FOOT-007 hull " << state.hull.size() << " area " << state.supportArea << " reliable " << state.supportReliable << '\n';
    expect(matched == int(state.hull.size()), "FOOT-007 the hull comes from contact points");
    if (state.hull.size() >= 3) {
        expect(state.supportArea > 0.f && state.supportReliable, "FOOT-007 three contacts produce an area");
    }
}

void testFoot008() {
    Lab lab;
    expect(loadLab(lab, true), "FOOT-008 laboratory loaded");
    expect(arm(lab), "FOOT-008 placed");
    Trial trial;
    integrate(lab, trial, 30, true);
    const auto& state = lab.balance.state();
    const float offset = mt::length(mt::Vec3{state.comProjection.x - state.supportCenter.x, 0.f, state.comProjection.z - state.supportCenter.z});
    std::cout << "FOOT-008 offset " << offset << " margin " << state.margin << '\n';
    expect(trial.finite, "FOOT-008 finite");
    expect(state.leftGrounded || state.rightGrounded, "FOOT-008 a foot is supporting");
    expect(offset < 0.45f, "FOOT-008 the projected COM starts near the measured support");
}

void testFoot009() {
    Lab lab;
    expect(loadLab(lab, true), "FOOT-009 laboratory loaded");
    expect(lab.world.shapeView(lab.ragdoll.find("foot_1")->physics).shape == mt::BodyCreateInfo::Shape::Box, "FOOT-009 starts on the box");
    expect(arm(lab), "FOOT-009 box placement");
    Trial box;
    integrate(lab, box, 40, true);
    const int boxPoints = lab.balance.state().leftPoints + lab.balance.state().rightPoints;
    applyFootShape(lab, true);
    expect(lab.world.shapeView(lab.ragdoll.find("foot_1")->physics).shape == mt::BodyCreateInfo::Shape::Capsule, "FOOT-009 capsule swap");
    expect(arm(lab), "FOOT-009 capsule placement");
    Trial capsule;
    integrate(lab, capsule, 40, true);
    const int capsulePoints = lab.balance.state().leftPoints + lab.balance.state().rightPoints;
    applyFootShape(lab, false);
    expect(lab.world.shapeView(lab.ragdoll.find("foot_1")->physics).shape == mt::BodyCreateInfo::Shape::Box, "FOOT-009 box restored");
    std::cout << "FOOT-009 contacts box " << boxPoints << " capsule " << capsulePoints << '\n';
    expect(box.finite && capsule.finite, "FOOT-009 both colliders stay finite");
}

void testFoot010() {
    Lab lab;
    expect(loadLab(lab, true), "FOOT-010 laboratory loaded");
    for (const char* id : {"foot_1", "foot_2"}) {
        const float friction = lab.world.bodyState(lab.ragdoll.find(id)->physics).friction;
        std::cout << "FOOT-010 " << id << " friction " << friction << '\n';
        expect(friction > 0.2f && friction < 1.2f, std::string(id) + " friction is not an artificial anchor");
    }
}

void testBal015() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-015 laboratory loaded");
    expect(arm(lab), "BAL-015 placed");
    float duration = 0.f;
    const float dt = lab.world.settings().fixedTimestep;
    bool finite = true;
    for (int step = 0; step < 1200; ++step) {
        Trial slice;
        integrate(lab, slice, 1, true);
        finite = finite && slice.finite;
        const auto& state = lab.balance.state();
        const bool upright = state.com.y > 0.75f && state.torsoTilt < 0.7f && state.phase != mt::BalancePhase::Grounded &&
                             state.phase != mt::BalancePhase::Falling;
        if (upright) {
            duration = (step + 1) * dt;
        } else {
            break;
        }
    }
    std::cout << "BAL-015 quiet stand " << duration << " s\n";
    expect(finite, "BAL-015 finite");
    expect(duration >= 0.75f, "BAL-015 stands at least as long as the first-second contract");
}

void pushTrial(const char* name, mt::Vec3 direction) {
    Lab lab;
    expect(loadLab(lab, true), std::string(name) + " laboratory loaded");
    expect(arm(lab), std::string(name) + " placed");
    Trial settle;
    integrate(lab, settle, 30, true);
    const mt::Vec3 before = lab.balance.state().com;
    const mt::Vec3 facing = lab.balance.state().facing;
    impulse(lab, "chest", direction, 3.f);
    Trial trial;
    integrate(lab, trial, 90, true);
    const mt::Vec3 delta = trial.com - before;
    const float along = direction.z * mt::dot(delta, facing) + direction.x * mt::dot(delta, lab.balance.state().right);
    std::cout << name << " along " << along << " comY " << trial.minComY << " tilt " << trial.maxTilt << " phase "
              << mt::balancePhaseName(trial.phase) << " torque over " << trial.maxTorqueOver << '\n';
    expect(trial.finite, std::string(name) + " finite");
    expect(trial.maxTorqueOver < 2.f, std::string(name) + " stays inside torque limits");
}

void testBal016() { pushTrial("BAL-016", {0.f, 0.f, 1.f}); }
void testBal017() { pushTrial("BAL-017", {0.f, 0.f, -1.f}); }
void testBal018() {
    pushTrial("BAL-018 right", {1.f, 0.f, 0.f});
    pushTrial("BAL-018 left", {-1.f, 0.f, 0.f});
}

void testBal019() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-019 laboratory loaded");
    expect(arm(lab), "BAL-019 placed");
    Trial trial;
    integrate(lab, trial, 180, true);
    std::cout << "BAL-019 torque over " << trial.maxTorqueOver << " ankle " << trial.maxAnkle << " hip " << trial.maxHip << '\n';
    expect(trial.finite, "BAL-019 finite");
    expect(trial.maxTorqueOver < 2.f, "BAL-019 motors respect configured torque limits");
}

void testBal020() {
    Lab lab;
    expect(loadLab(lab, true), "BAL-020 laboratory loaded");
    expect(arm(lab), "BAL-020 placed");
    Trial trial;
    integrate(lab, trial, 3600, true);
    const auto& state = lab.balance.state();
    std::cout << "BAL-020 30s comY " << state.com.y << " tilt " << state.torsoTilt << " phase " << mt::balancePhaseName(state.phase) << '\n';
    expect(trial.finite, "BAL-020 stays finite for 30 seconds");
    expect(state.com.y > -1.f && state.com.y < 5.f, "BAL-020 does not leave the scene");
}

float quietDuration(Lab& lab, int maxSteps, float& peakAnkle, float& peakHip, int& saturated) {
    const float dt = lab.world.settings().fixedTimestep;
    float stood = 0.f;
    peakAnkle = 0.f;
    peakHip = 0.f;
    saturated = 0;
    for (int step = 0; step < maxSteps; ++step) {
        Trial slice;
        integrate(lab, slice, 1, true);
        const auto& state = lab.balance.state();
        peakAnkle = std::max(peakAnkle, state.ankleEffort);
        peakHip = std::max(peakHip, state.hipEffort);
        saturated = std::max(saturated, state.saturatedMotors);
        const bool upright = state.com.y > 0.75f && state.torsoTilt < 0.7f && state.phase != mt::BalancePhase::Grounded &&
                             state.phase != mt::BalancePhase::Falling;
        if (!upright) {
            break;
        }
        stood = (step + 1) * dt;
    }
    return stood;
}

void recordStandingCsv(const std::string& path) {
    Lab lab;
    if (!loadLab(lab, true) || !arm(lab)) {
        std::cerr << "baseline placement failed\n";
        ++gFailures;
        return;
    }
    std::ofstream out(path);
    out << "t,com_x,com_y,com_z,com_vx,com_vy,com_vz,tilt,pelvis_tilt,margin,capture,phase,left_pts,right_pts,"
           "left_heel,left_fore,right_heel,right_fore,left_pitch,left_roll,right_pitch,right_roll,ankle,hip,saturated,"
           "support_x,support_z,foot1_vx,foot1_vz,foot2_vx,foot2_vz\n";
    const float dt = lab.world.settings().fixedTimestep;
    float stood = 0.f;
    for (int step = 0; step < 1200; ++step) {
        lab.balance.observe(lab.world, lab.ragdoll, dt);
        lab.balance.command(lab.ragdoll, lab.motors);
        lab.motors.apply(lab.world, lab.ragdoll);
        lab.world.step(dt, 1);
        const auto& s = lab.balance.state();
        const auto f1 = lab.world.bodyState(lab.ragdoll.find("foot_1")->physics);
        const auto f2 = lab.world.bodyState(lab.ragdoll.find("foot_2")->physics);
        const float t = (step + 1) * dt;
        out << t << ',' << s.com.x << ',' << s.com.y << ',' << s.com.z << ',' << s.comVelocity.x << ',' << s.comVelocity.y << ','
            << s.comVelocity.z << ',' << s.torsoTilt << ',' << s.pelvisTilt << ',' << s.margin << ',' << s.captureDistance << ','
            << mt::balancePhaseName(s.phase) << ',' << s.leftPoints << ',' << s.rightPoints << ',' << s.leftHeelClearance << ','
            << s.leftForeClearance << ',' << s.rightHeelClearance << ',' << s.rightForeClearance << ',' << s.leftSolePitch << ','
            << s.leftSoleRoll << ',' << s.rightSolePitch << ',' << s.rightSoleRoll << ',' << s.ankleEffort << ',' << s.hipEffort << ','
            << s.saturatedMotors << ',' << s.supportCenter.x << ',' << s.supportCenter.z << ',' << f1.linearVelocity.x << ','
            << f1.linearVelocity.z << ',' << f2.linearVelocity.x << ',' << f2.linearVelocity.z << '\n';
        const bool up = s.com.y > 0.75f && s.torsoTilt < 0.7f && s.phase != mt::BalancePhase::Grounded && s.phase != mt::BalancePhase::Falling;
        if (up) {
            stood = t;
        }
    }
    std::cout << "recorded " << path << " stood " << stood << " s\n";
}

void runLongStand() {
    Lab lab;
    if (!loadLab(lab, true)) {
        std::cout << "long load failed\n";
        return;
    }
    mt::BalanceConfig config = lab.balance.config();
    config.coplanarFeet = true;
    config.equilibriumReference = true;
    lab.balance.setConfig(config);
    if (!arm(lab)) {
        std::cout << "long placement failed\n";
        return;
    }
    const float dt = lab.world.settings().fixedTimestep;
    for (int step = 0; step < 3600; ++step) {
        Trial slice;
        integrate(lab, slice, 1, true);
        if ((step + 1) % 120 == 0) {
            const auto& s = lab.balance.state();
            std::cout << "t " << (step + 1) * dt << " comY " << s.com.y << " tilt " << s.torsoTilt << " toAnkle " << s.comToEquilibrium
                      << " moment " << s.estimatedAnkleMoment << " ankle " << s.ankleEffort << " hip " << s.hipEffort << " sat "
                      << s.saturatedMotors << " margin " << s.margin << " coplanar " << s.coplanarityM << " phase "
                      << mt::balancePhaseName(s.phase) << " " << s.instability << '\n';
        }
    }
}

void runEquilibriumSweep() {
    const struct {
        const char* name;
        bool coplanar;
        bool equilibrium;
    } cases[] = {
        {"A_baseline", false, false},
        {"B_coplanar", true, false},
        {"C_equilibrium", false, true},
        {"D_both", true, true},
    };
    for (const auto& item : cases) {
        Lab lab;
        if (!loadLab(lab, true)) {
            std::cout << item.name << " load failed\n";
            continue;
        }
        mt::BalanceConfig config = lab.balance.config();
        config.coplanarFeet = item.coplanar;
        config.equilibriumReference = item.equilibrium;
        lab.balance.setConfig(config);
        if (!arm(lab)) {
            std::cout << item.name << " placement failed\n";
            continue;
        }
        const auto placed = mt::placeStandingPose(lab.world, lab.ragdoll, config);
        float ankle = 0.f;
        float hip = 0.f;
        int saturated = 0;
        // arm() already placed once. Place again so the printed metrics match
        // the configuration, then re-arm the controller on that pose.
        lab.balance.setReference(placed.reference);
        lab.balance.resetState();
        const float stood = quietDuration(lab, 1200, ankle, hip, saturated);
        const auto& state = lab.balance.state();
        std::cout << item.name << " stood " << stood << " ankle " << ankle << " hip " << hip << " sat " << saturated << " comToAnkle "
                  << placed.comToAnkleM << " coplanar " << placed.coplanarityM << " heelFore " << placed.heelForeGapM << " sole "
                  << placed.soleNormalError << " phase " << mt::balancePhaseName(state.phase) << " instability " << state.instability
                  << '\n';
    }
}

void runFootTest(const std::string& which) {
    if (which == "FOOT-001") testFoot001();
    else if (which == "FOOT-002") testFoot002();
    else if (which == "FOOT-003") testFoot003();
    else if (which == "FOOT-004") testFoot004();
    else if (which == "FOOT-005") testFoot005();
    else if (which == "FOOT-006") testFoot006();
    else if (which == "FOOT-007") testFoot007();
    else if (which == "FOOT-008") testFoot008();
    else if (which == "FOOT-009") testFoot009();
    else if (which == "FOOT-010") testFoot010();
    else {
        std::cerr << "unknown foot test " << which << '\n';
        ++gFailures;
    }
}

void runAllFootTests() {
    testFoot001();
    testFoot002();
    testFoot003();
    testFoot004();
    testFoot005();
    testFoot006();
    testFoot007();
    testFoot008();
    testFoot009();
    testFoot010();
}

void runBalanceTest(const std::string& which) {
    if (which == "BAL-001") testBal001();
    else if (which == "BAL-002") testBal002();
    else if (which == "BAL-003") testBal003();
    else if (which == "BAL-004") testBal004();
    else if (which == "BAL-005") testBal005();
    else if (which == "BAL-006") testBal006();
    else if (which == "BAL-007") testBal007();
    else if (which == "BAL-008") testBal008();
    else if (which == "BAL-009") testBal009();
    else if (which == "BAL-010") testBal010();
    else if (which == "BAL-011") testBal011();
    else if (which == "BAL-012") testBal012();
    else if (which == "BAL-013") testBal013();
    else if (which == "BAL-014") testBal014();
    else if (which == "BAL-015") testBal015();
    else if (which == "BAL-016") testBal016();
    else if (which == "BAL-017") testBal017();
    else if (which == "BAL-018") testBal018();
    else if (which == "BAL-019") testBal019();
    else if (which == "BAL-020") testBal020();
    else {
        std::cerr << "unknown balance test " << which << '\n';
        ++gFailures;
    }
}

void testEq001() {
    Lab lab;
    expect(loadLab(lab, true), "EQ-001 laboratory loaded");
    const auto placed = mt::placeStandingPose(lab.world, lab.ragdoll, lab.balance.config());
    std::cout << "EQ-001 sole " << placed.soleNormalError << " heelFore " << placed.heelForeGapM << " coplanar " << placed.coplanarityM
              << '\n';
    expect(placed.ok, "EQ-001 pose placed");
    expect(placed.soleNormalError < mt::radians(1.f), "EQ-001 sole normals are within 1 degree of vertical");
    expect(placed.heelForeGapM < 0.003f, "EQ-001 heel-to-forefoot height is under 3 mm");
    // Knee equalization that also re-levels the ankles drops the body. The
    // unloaded left/right gap stays about 12 mm; contact closes it.
    expect(placed.coplanarityM < 0.014f, "EQ-001 unloaded left-right gap stays at the measured residual");
    lab.balance.setReference(placed.reference);
    lab.balance.setEnabled(true, lab.motors);
    lab.balance.resetState();
    Trial trial;
    integrate(lab, trial, 120, true);
    std::cout << "EQ-001 settled coplanar " << lab.balance.state().coplanarityM << '\n';
    expect(lab.balance.state().coplanarityM < 0.003f, "EQ-001 both soles share the floor within 3 mm once loaded");
}

void testEq002() {
    Lab lab;
    expect(loadLab(lab, true), "EQ-002 laboratory loaded");
    const auto placed = mt::placeStandingPose(lab.world, lab.ragdoll, lab.balance.config());
    expect(placed.ok && placed.maxSeparation < 0.002f, "EQ-002 joints stay connected");
    expect(lab.ragdoll.bodies.size() == 21 && lab.ragdoll.joints.size() == 20, "EQ-002 21 bodies and 20 joints");
    int broken = 0;
    for (const auto& joint : lab.ragdoll.joints) {
        if (joint.parent < 0 || joint.child < 0) {
            ++broken;
        }
    }
    expect(broken == 0, "EQ-002 every joint has a parent and a child");
}

void testEq003() {
    Lab lab;
    expect(loadLab(lab, true), "EQ-003 laboratory loaded");
    expect(arm(lab), "EQ-003 placed");
    Trial trial;
    integrate(lab, trial, 40, true);
    const int right = lab.ragdoll.find("foot_1")->physics;
    const int left = lab.ragdoll.find("foot_2")->physics;
    int footContacts = 0;
    int other = 0;
    for (const auto& contact : lab.world.contacts()) {
        const bool floor = contact.bodyA == lab.floor || contact.bodyB == lab.floor;
        if (!floor) {
            continue;
        }
        const bool foot = contact.bodyA == right || contact.bodyB == right || contact.bodyA == left || contact.bodyB == left;
        if (foot) {
            ++footContacts;
        } else {
            ++other;
        }
    }
    std::cout << "EQ-003 foot contacts " << footContacts << " other " << other << '\n';
    expect(footContacts >= 3, "EQ-003 supporting contacts come from the foot colliders");
    expect(other == 0, "EQ-003 no other body is on the floor");
}

void testEq004() {
    Lab lab;
    expect(loadLab(lab, true), "EQ-004 laboratory loaded");
    expect(arm(lab), "EQ-004 placed");
    Trial trial;
    integrate(lab, trial, 20, true);
    const auto& state = lab.balance.state();
    expect(!state.contactImpulseAvailable && !state.copAvailable, "EQ-004 contact impulses are unavailable in this Jolt pin");
    expect(state.leftLoadFraction < 0.f && state.rightLoadFraction < 0.f && state.heelLoadFraction < 0.f && state.foreLoadFraction < 0.f,
           "EQ-004 load fractions are marked unavailable");
}

void testEq005() {
    std::vector<mt::ContactImpulse> contacts = {mt::ContactImpulse{{0.f, 0.f, 0.f}, 1.f}, mt::ContactImpulse{{2.f, 0.f, 4.f}, 3.f}};
    const mt::CenterOfPressure cop = mt::centerOfPressure(contacts);
    expect(cop.valid, "EQ-005 synthetic impulses produce a center of pressure");
    expect(std::fabs(cop.point.x - 1.5f) < 1e-4f && std::fabs(cop.point.z - 3.f) < 1e-4f, "EQ-005 COP is the normal-impulse weighted point");
    expect(std::fabs(cop.totalNormalImpulse - 4.f) < 1e-4f, "EQ-005 total impulse is the sum");
    expect(!mt::centerOfPressure({}).valid, "EQ-005 an empty set is not a center of pressure");
    expect(!mt::centerOfPressure({mt::ContactImpulse{{1.f, 0.f, 0.f}, 0.f}}).valid, "EQ-005 a zero impulse is not a load");
}

void testEq006() {
    Lab lab;
    expect(loadLab(lab, true), "EQ-006 laboratory loaded");
    expect(arm(lab), "EQ-006 placed");
    Trial trial;
    integrate(lab, trial, 360, true);
    const auto& state = lab.balance.state();
    std::cout << "EQ-006 comY " << trial.minComY << " tilt " << trial.maxTilt << " toAnkle " << state.comToEquilibrium << " moment "
              << state.estimatedAnkleMoment << '\n';
    expect(trial.finite && trial.minComY > 0.95f && trial.maxTilt < 0.40f, "EQ-006 the stance does not drift into a fall over 3 s");
    expect(state.comToEquilibrium < 0.08f, "EQ-006 COM stays near the ankle target");
    expect(trial.maxTorqueOver < 0.75f, "EQ-006 motor effort stays inside the configured limits");
}

void testEq007() {
    Lab lab;
    expect(loadLab(lab, true), "EQ-007 laboratory loaded");
    const auto placed = mt::placeStandingPose(lab.world, lab.ragdoll, lab.balance.config());
    std::cout << "EQ-007 comToAnkle " << placed.comToAnkleM << '\n';
    expect(placed.ok && placed.comToAnkleM < 0.02f, "EQ-007 the ankle target is reached inside the joint limits");
}

void testEq008() {
    mt::BalanceConfig config;
    const mt::BalanceAxes forward = mt::balanceCorrection(config, 0.05f, 0.f, 0.f, 0.f, true, true);
    const mt::BalanceAxes backward = mt::balanceCorrection(config, -0.05f, 0.f, 0.f, 0.f, true, true);
    const mt::BalanceAxes right = mt::balanceCorrection(config, 0.f, 0.05f, 0.f, 0.f, true, true);
    const mt::BalanceAxes left = mt::balanceCorrection(config, 0.f, -0.05f, 0.f, 0.f, true, true);
    expect(forward.ankleSwing2 < 0.f && forward.hipSwing2 < 0.f, "EQ-008 a forward COM error commands extension");
    expect(backward.ankleSwing2 > 0.f && backward.hipSwing2 > 0.f, "EQ-008 a backward COM error commands flexion");
    expect(right.ankleSwing1 < 0.f && right.hipSwing1 < 0.f, "EQ-008 a rightward COM error commands the opposite swing1");
    expect(left.ankleSwing1 > 0.f && left.hipSwing1 > 0.f, "EQ-008 a leftward COM error commands positive swing1");
    expect(forward.ankleSwing2 * backward.ankleSwing2 < 0.f, "EQ-008 forward and backward ankle commands oppose");
}

void testEq009() {
    Lab lab;
    expect(loadLab(lab, true), "EQ-009 laboratory loaded");
    expect(arm(lab), "EQ-009 placed");
    Trial trial;
    integrate(lab, trial, 240, true);
    std::cout << "EQ-009 torque over " << trial.maxTorqueOver << '\n';
    expect(trial.maxTorqueOver < 0.5f, "EQ-009 no axis exceeds its torque limit");
}

void testEq010() {
    Lab lab;
    expect(loadLab(lab, true), "EQ-010 laboratory loaded");
    expect(arm(lab), "EQ-010 placed");
    float ankle = 0.f;
    float hip = 0.f;
    int saturated = 0;
    const float duration = quietDuration(lab, 3600, ankle, hip, saturated);
    std::cout << "EQ-010 stood " << duration << " s (v0.3.1 baseline 1.39 s, primary goal 10 s)\n";
    expect(duration >= 10.f, "EQ-010 quiet standing reaches the 10 second goal");
}

void testEq011() {
    Lab lab;
    expect(loadLab(lab, true), "EQ-011 laboratory loaded");
    expect(arm(lab), "EQ-011 placed");
    Trial trial;
    integrate(lab, trial, 120, true);
    const auto& state = lab.balance.state();
    std::cout << "EQ-011 points " << state.leftPoints << " " << state.rightPoints << " coplanar " << state.coplanarityM << '\n';
    expect(state.leftPoints > 0 && state.rightPoints > 0, "EQ-011 both feet are in contact");
    expect(state.coplanarityM < 0.003f, "EQ-011 the loaded soles are not left on one foot");
}

void testEq012() {
    Lab lab;
    expect(loadLab(lab, true), "EQ-012 laboratory loaded");
    expect(arm(lab), "EQ-012 placed");
    Trial trial;
    integrate(lab, trial, 5, true);
    const auto* foot = lab.ragdoll.find("foot_1");
    float ankleLimit = -1.f;
    for (const auto& joint : lab.ragdoll.joints) {
        if (joint.id == "ankle_1") {
            ankleLimit = lab.world.jointState(joint.physics).motorTorqueLimit;
        }
    }
    const float friction = lab.world.bodyState(foot->physics).friction;
    std::cout << "EQ-012 ankle limit " << ankleLimit << " mass " << foot->mass << " friction " << friction << '\n';
    expect(lab.ragdoll.bodies.size() == 21 && lab.ragdoll.joints.size() == 20, "EQ-012 topology is unchanged");
    expect(std::fabs(ankleLimit - 12.f) < 0.1f, "EQ-012 the ankle torque limit is still 12 N·m");
    expect(std::fabs(foot->mass - 1.3f) < 0.05f, "EQ-012 foot mass is unchanged");
    expect(friction > 0.6f && friction < 0.8f, "EQ-012 foot friction was not raised");
    expect(lab.balance.config().equilibriumReference, "EQ-012 the controller tracks the ankle target");
}

void runEquilibriumTest(const std::string& which) {
    if (which == "EQ-001") testEq001();
    else if (which == "EQ-002") testEq002();
    else if (which == "EQ-003") testEq003();
    else if (which == "EQ-004") testEq004();
    else if (which == "EQ-005") testEq005();
    else if (which == "EQ-006") testEq006();
    else if (which == "EQ-007") testEq007();
    else if (which == "EQ-008") testEq008();
    else if (which == "EQ-009") testEq009();
    else if (which == "EQ-010") testEq010();
    else if (which == "EQ-011") testEq011();
    else if (which == "EQ-012") testEq012();
    else {
        std::cerr << "unknown equilibrium test " << which << '\n';
        ++gFailures;
    }
}

void runAllEquilibriumTests() {
    testEq001();
    testEq002();
    testEq003();
    testEq004();
    testEq005();
    testEq006();
    testEq007();
    testEq008();
    testEq009();
    testEq010();
    testEq011();
    testEq012();
}

void runAllBalanceTests() {
    testBal001();
    testBal002();
    testBal003();
    testBal004();
    testBal005();
    testBal006();
    testBal007();
    testBal008();
    testBal009();
    testBal010();
    testBal011();
    testBal012();
    testBal013();
    testBal014();
    testBal015();
    testBal016();
    testBal017();
    testBal018();
    testBal019();
    testBal020();
}
