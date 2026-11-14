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

BalanceState BalanceEstimator::measure(const PhysicsWorld& world, const Ragdoll& ragdoll) const {
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
    state.contacts = support.contacts;
    state.hull = support.hull;
    state.supportCenter = support.center;
    state.margin = supportMargin(support.hull, state.comProjection);
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
