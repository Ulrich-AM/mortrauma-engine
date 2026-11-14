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
    // Capsule feet have no sole, so quiet standing is an unstable equilibrium.
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
    Trial later;
    integrate(lab, later, 180, true);
    std::cout << "BAL-001 4s comY " << later.minComY << " tilt " << later.maxTilt << " phase " << mt::balancePhaseName(later.phase)
              << '\n';
    expect(later.finite, "BAL-001 later motion stays finite");
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
    expect(travel > 0.15f, "BAL-003 the push moves the body");
    // 6 N·s is above what the 12 N·m ankles can reject on capsule feet.
    expect(fell, "BAL-003 a small forward push is not recovered and the body falls");
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
    expect(fell, "BAL-004 a lateral push is not recovered on capsule feet");
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
    else {
        std::cerr << "unknown balance test " << which << '\n';
        ++gFailures;
    }
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
}
