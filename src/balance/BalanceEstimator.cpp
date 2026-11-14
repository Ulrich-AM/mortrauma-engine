#include "mortrauma/balance/BalanceEstimator.hpp"

#include "mortrauma/balance/SupportContacts.hpp"

#include <cmath>

namespace mt {
namespace {

const RagdollBody* findBody(const Ragdoll& ragdoll, const char* id) {
    return ragdoll.find(id);
}

Vec3 bodyUp(const RagdollBody& body, Quat rotation) {
    Vec3 local = body.segmentEnd - body.segmentStart;
    if (length(local) < 1e-4f) {
        local = {0.f, 1.f, 0.f};
    }
    local = rotate(conjugate(body.initialRotation), normalize(local));
    return normalize(rotate(rotation, local));
}

float tiltFromUp(Vec3 up) {
    return std::acos(clampf(dot(normalize(up), Vec3{0.f, 1.f, 0.f}), -1.f, 1.f));
}

}  // namespace

BalanceState BalanceEstimator::measure(const PhysicsWorld& world, const Ragdoll& ragdoll, float equilibriumLeadM) const {
    BalanceState state;
    Vec3 weighted{};
    Vec3 weightedVelocity{};
    float mass = 0.f;
    Vec3 facing{};
    for (const auto& body : ragdoll.bodies) {
        const BodyState sample = world.bodyState(body.physics);
        if (!sample.valid || !finite(sample.position) || !finite(sample.linearVelocity) || !finite(sample.angularVelocity) ||
            !(sample.mass > 0.f)) {
            state.finite = false;
            continue;
        }
        weighted = weighted + sample.position * sample.mass;
        weightedVelocity = weightedVelocity + sample.linearVelocity * sample.mass;
        mass += sample.mass;
    }
    if (!(mass > 0.f) || !state.finite) {
        state.finite = false;
        return state;
    }
    state.totalMass = mass;
    state.com = weighted * (1.f / mass);
    state.comVelocity = weightedVelocity * (1.f / mass);
    state.comProjection = {state.com.x, 0.f, state.com.z};

    for (const char* footId : {"foot_1", "foot_2"}) {
        if (const RagdollBody* foot = findBody(ragdoll, footId)) {
            Vec3 along = foot->segmentEnd - foot->segmentStart;
            along.y = 0.f;
            facing = facing + along;
        }
    }
    if (length(facing) < 1e-3f) {
        facing = {0.f, 0.f, -1.f};
    }
    state.facing = normalize(facing);
    state.right = normalize(cross(state.facing, Vec3{0.f, 1.f, 0.f}));
    if (length(state.right) < 1e-3f) {
        state.right = {1.f, 0.f, 0.f};
    }

    const SupportEstimate support = estimateSupport(world, ragdoll);
    state.leftGrounded = support.leftGrounded;
    state.rightGrounded = support.rightGrounded;
    state.leftTouching = support.leftTouching;
    state.rightTouching = support.rightTouching;
    state.leftPoints = support.leftPoints;
    state.rightPoints = support.rightPoints;
    state.leftLoadProxy = support.leftLoadProxy;
    state.rightLoadProxy = support.rightLoadProxy;
    state.leftHeel = support.leftHeel;
    state.leftMid = support.leftMid;
    state.leftFore = support.leftFore;
    state.rightHeel = support.rightHeel;
    state.rightMid = support.rightMid;
    state.rightFore = support.rightFore;
    state.supportArea = support.area;
    state.leftArea = support.leftArea;
    state.rightArea = support.rightArea;
    state.supportReliable = support.reliable;
    auto soleOf = [&](const char* footId, float& pitch, float& roll, float& heelClearance, float& foreClearance) {
        const RagdollBody* foot = findBody(ragdoll, footId);
        if (foot == nullptr) {
            return;
        }
        const BodyState sample = world.bodyState(foot->physics);
        if (!sample.valid) {
            return;
        }
        const Vec3 up = normalize(rotate(sample.rotation, foot->soleNormalLocal));
        pitch = std::atan2(dot(up, state.facing), std::max(up.y, 0.05f));
        roll = std::atan2(dot(up, state.right), std::max(up.y, 0.05f));
        heelClearance = (sample.position + rotate(sample.rotation, foot->heelLocal)).y;
        foreClearance = (sample.position + rotate(sample.rotation, foot->forefootLocal)).y;
    };
    soleOf("foot_2", state.leftSolePitch, state.leftSoleRoll, state.leftHeelClearance, state.leftForeClearance);
    soleOf("foot_1", state.rightSolePitch, state.rightSoleRoll, state.rightHeelClearance, state.rightForeClearance);
    state.contacts = support.contacts;
    state.hull = support.hull;
    state.supportCenter = support.center;
    state.contactImpulseAvailable = false;
    state.copAvailable = false;
    state.leftLoadFraction = -1.f;
    state.rightLoadFraction = -1.f;
    state.heelLoadFraction = -1.f;
    state.foreLoadFraction = -1.f;
    const int groundedFeet = (state.leftGrounded ? 1 : 0) + (state.rightGrounded ? 1 : 0);
    const float weight = state.totalMass * std::max(length(world.gravity()), 1.f);
    Vec3 equilibrium{};
    int equilibriumCount = 0;
    float moment = 0.f;
    for (const auto& joint : ragdoll.joints) {
        if (joint.id != "ankle_1" && joint.id != "ankle_2") {
            continue;
        }
        if (joint.child < 0 || joint.child >= int(ragdoll.bodies.size())) {
            continue;
        }
        const bool grounded = joint.id == "ankle_1" ? state.rightGrounded : state.leftGrounded;
        if (groundedFeet > 0 && !grounded) {
            continue;
        }
        const RagdollBody& foot = ragdoll.bodies[joint.child];
        const BodyState sample = world.bodyState(foot.physics);
        const JointState ankle = world.jointState(joint.physics);
        if (!sample.valid || !ankle.valid) {
            continue;
        }
        const Vec3 heel = sample.position + rotate(sample.rotation, foot.heelLocal);
        const Vec3 fore = sample.position + rotate(sample.rotation, foot.forefootLocal);
        equilibrium = equilibrium + feasibleSolePoint(ankle.anchorChildWorld, heel, fore, equilibriumLeadM);
        ++equilibriumCount;
        // No planted foot means there is no ankle moment yet. Using the
        // distance from the pelvis to an ankle made this read ~200 N·m before
        // the first contact, which is not a torque the joint is carrying.
        if (groundedFeet <= 0) {
            continue;
        }
        const Vec3 offset{state.com.x - ankle.anchorChildWorld.x, 0.f, state.com.z - ankle.anchorChildWorld.z};
        const float forwardArm = std::fabs(dot(offset, state.facing));
        // Two feet share a lateral offset as a force difference. The pitch
        // moment about each ankle is the forward arm times that foot's load.
        // A single supporting foot has to resist the whole horizontal offset.
        const float share = groundedFeet <= 1 ? 1.f : 0.5f;
        const float arm = groundedFeet <= 1 ? length(offset) : forwardArm;
        moment = std::max(moment, weight * share * arm);
    }
    if (equilibriumCount > 0) {
        state.equilibriumTarget = equilibrium * (1.f / float(equilibriumCount));
        state.equilibriumValid = true;
        state.estimatedAnkleMoment = moment;
        state.comToEquilibrium = length(Vec3{state.com.x - state.equilibriumTarget.x, 0.f, state.com.z - state.equilibriumTarget.z});
    }
    const float leftMean = 0.5f * (state.leftHeelClearance + state.leftForeClearance);
    const float rightMean = 0.5f * (state.rightHeelClearance + state.rightForeClearance);
    state.coplanarityM = std::fabs(leftMean - rightMean);
    state.heelForeGapM = std::max(std::fabs(state.leftHeelClearance - state.leftForeClearance),
                                  std::fabs(state.rightHeelClearance - state.rightForeClearance));
    state.soleNormalError = std::sqrt(state.leftSolePitch * state.leftSolePitch + state.leftSoleRoll * state.leftSoleRoll);
    state.soleNormalError = std::max(state.soleNormalError, std::sqrt(state.rightSolePitch * state.rightSolePitch + state.rightSoleRoll * state.rightSoleRoll));
    state.margin = supportMargin(support.hull, state.comProjection);
    if (!state.finite) {
        state.instability = "non-finite state";
    } else if (state.coplanarityM > 0.003f) {
        state.instability = "feet are not coplanar";
    } else if (state.estimatedAnkleMoment > 12.f) {
        state.instability = "estimated ankle moment exceeds 12 N·m";
    } else if (state.supportReliable && state.margin < 0.f) {
        state.instability = "COM is outside the support polygon";
    } else {
        state.instability = "within the estimated ankle capacity";
    }
    const float gravity = std::max(length(world.gravity()), 1.f);
    const float height = std::max(state.com.y, 0.05f);
    const float horizon = std::sqrt(height / gravity);
    const Vec3 capture{state.com.x + state.comVelocity.x * horizon, 0.f, state.com.z + state.comVelocity.z * horizon};
    state.captureDistance = distanceOutsideSupport(support.hull, capture);

    if (const RagdollBody* chest = findBody(ragdoll, "chest")) {
        const BodyState sample = world.bodyState(chest->physics);
        if (sample.valid) {
            state.torsoUp = bodyUp(*chest, sample.rotation);
            state.torsoTilt = tiltFromUp(state.torsoUp);
            state.torsoAngularVelocity = sample.angularVelocity;
        }
    }
    if (const RagdollBody* pelvis = findBody(ragdoll, "pelvis")) {
        const BodyState sample = world.bodyState(pelvis->physics);
        if (sample.valid) {
            state.pelvisUp = bodyUp(*pelvis, sample.rotation);
            state.pelvisTilt = tiltFromUp(state.pelvisUp);
        }
    }
    float ankle = 0.f;
    float hip = 0.f;
    int saturated = 0;
    for (const auto& joint : ragdoll.joints) {
        const JointState sample = world.jointState(joint.physics);
        if (!sample.valid) {
            continue;
        }
        if (sample.motorSaturated) {
            ++saturated;
        }
        const float effort = length(sample.motorTorque);
        if (joint.id.rfind("ankle_", 0) == 0) {
            ankle = std::max(ankle, effort);
        }
        if (joint.id.rfind("hip_", 0) == 0) {
            hip = std::max(hip, effort);
        }
    }
    state.ankleEffort = ankle;
    state.hipEffort = hip;
    state.saturatedMotors = saturated;
    state.finite = state.finite && finite(state.com) && finite(state.comVelocity);
    return state;
}

}  // namespace mt
