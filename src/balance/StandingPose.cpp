#include "mortrauma/balance/StandingPose.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace mt {
namespace {

Quat frameQuat(Vec3 axisX, Vec3 axisY) {
    const Vec3 x = normalize(axisX);
    Vec3 y = axisY - x * dot(axisY, x);
    if (length(y) < 1e-5f) {
        const Vec3 fallback = std::fabs(x.y) < 0.9f ? Vec3{0.f, 1.f, 0.f} : Vec3{1.f, 0.f, 0.f};
        y = normalize(cross(x, fallback));
    } else {
        y = normalize(y);
    }
    const Vec3 z = normalize(cross(x, y));
    y = normalize(cross(z, x));
    Mat4 basis;
    basis.m[0] = x.x;
    basis.m[1] = x.y;
    basis.m[2] = x.z;
    basis.m[4] = y.x;
    basis.m[5] = y.y;
    basis.m[6] = y.z;
    basis.m[8] = z.x;
    basis.m[9] = z.y;
    basis.m[10] = z.z;
    return basis.rotation();
}

Quat uprightPelvis(const RagdollBody& pelvis) {
    Vec3 localUp = pelvis.segmentEnd - pelvis.segmentStart;
    if (length(localUp) < 1e-4f) {
        localUp = {0.f, 1.f, 0.f};
    }
    localUp = normalize(rotate(conjugate(pelvis.initialRotation), localUp));
    Vec3 localForward = rotate(conjugate(pelvis.initialRotation), Vec3{0.f, 0.f, -1.f});
    localForward = localForward - localUp * dot(localForward, localUp);
    if (length(localForward) < 1e-4f) {
        localForward = {0.f, 0.f, -1.f};
    }
    localForward = normalize(localForward);
    const Vec3 srcX = normalize(cross(localForward, localUp));
    const Vec3 srcY = localUp;
    const Vec3 srcZ = normalize(cross(srcX, srcY));
    Mat4 local;
    local.m[0] = srcX.x;
    local.m[1] = srcX.y;
    local.m[2] = srcX.z;
    local.m[4] = srcY.x;
    local.m[5] = srcY.y;
    local.m[6] = srcY.z;
    local.m[8] = srcZ.x;
    local.m[9] = srcZ.y;
    local.m[10] = srcZ.z;
    return normalize(conjugate(local.rotation()));
}

float flexion(const RagdollJoint& joint, float degrees) {
    const float angle = radians(degrees);
    const float lower = joint.base.swing2Min + radians(2.f);
    const float upper = joint.base.swing2Max - radians(2.f);
    if (upper <= lower) {
        return clampf(angle, joint.base.swing2Min, joint.base.swing2Max);
    }
    return clampf(angle, lower, upper);
}

float capsuleBottom(const ShapeView& shape) {
    if (shape.shape == BodyCreateInfo::Shape::Sphere || shape.halfHeight <= 0.f) {
        return shape.position.y - shape.radius;
    }
    const Vec3 axis = rotate(shape.rotation, {0.f, 1.f, 0.f});
    const float y0 = shape.position.y + axis.y * shape.halfHeight;
    const float y1 = shape.position.y - axis.y * shape.halfHeight;
    return std::min(y0, y1) - shape.radius;
}

}  // namespace

Quat standingReference(const RagdollJoint& joint, const BalanceConfig& config) {
    if (joint.id == "knee_1" || joint.id == "knee_2") {
        return quatAxisAngle({0.f, 0.f, 1.f}, flexion(joint, config.kneeFlexionDeg));
    }
    if (joint.id == "hip_1" || joint.id == "hip_2") {
        // The shared hip frame points both swing1 positives the same way, but the
        // imported stance uses opposite signs so the thighs clear each other.
        const float spread = joint.id == "hip_1" ? -18.f : 18.f;
        return quatFromEulerXYZ(0.f, flexion(joint, spread), flexion(joint, config.hipFlexionDeg));
    }
    if (joint.id == "ankle_1" || joint.id == "ankle_2") {
        return quatAxisAngle({0.f, 0.f, 1.f}, flexion(joint, config.ankleFlexionDeg));
    }
    return finite(joint.bindConstraint) ? normalize(joint.bindConstraint) : Quat::identity();
}

StandingPlacement placeStandingPose(PhysicsWorld& world, Ragdoll& ragdoll, const BalanceConfig& config) {
    StandingPlacement placement;
    const RagdollBody* pelvis = ragdoll.find("pelvis");
    if (pelvis == nullptr) {
        placement.error = "Standing placement needs a pelvis body.";
        return placement;
    }
    int pelvisIndex = -1;
    for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
        if (ragdoll.bodies[i].id == "pelvis") {
            pelvisIndex = i;
        }
    }
    std::vector<Vec3> position(ragdoll.bodies.size());
    std::vector<Quat> rotation(ragdoll.bodies.size());
    std::vector<char> placed(ragdoll.bodies.size(), 0);
    rotation[pelvisIndex] = uprightPelvis(*pelvis);
    position[pelvisIndex] = pelvis->initialPosition;
    placed[pelvisIndex] = 1;
    placement.pelvisUp = normalize(rotate(rotation[pelvisIndex], rotate(conjugate(pelvis->initialRotation),
                                                                         normalize(pelvis->segmentEnd - pelvis->segmentStart))));

    std::unordered_map<std::string, Quat> reference;
    for (const auto& joint : ragdoll.joints) {
        Quat target = standingReference(joint, config);
        if (joint.id == "hip_root_1" || joint.id == "hip_root_2") {
            if (joint.child >= 0 && joint.child < int(ragdoll.bodies.size())) {
                const Quat parentFrame = frameQuat(joint.axisXInParent, joint.axisYInParent);
                const Quat childFrame = frameQuat(joint.axisXInChild, joint.axisYInChild);
                const Quat parentConstraint = mul(rotation[pelvisIndex], parentFrame);
                const Quat childConstraint = mul(ragdoll.bodies[joint.child].initialRotation, childFrame);
                target = normalize(mul(conjugate(parentConstraint), childConstraint));
            }
            // Keeping the hip bone at its bind orientation can demand more twist
            // than the ±8° hip-root limit. A target outside the limit never matches,
            // so the motor saturates and shoves the foot. Clamp before posing.
            const Vec3 euler = constraintEulerRadians(target);
            const float margin = radians(1.f);
            target = quatFromEulerXYZ(clampf(euler.x, joint.base.twistMin + margin, joint.base.twistMax - margin),
                                      clampf(euler.y, joint.base.swing1Min + margin, joint.base.swing1Max - margin),
                                      clampf(euler.z, joint.base.swing2Min + margin, joint.base.swing2Max - margin));
        }
        reference[joint.id] = target;
        MotorTaskCommand command;
        command.jointId = joint.id;
        command.target = target;
        command.activation = 1.f;
        placement.reference.push_back(command);
    }

    auto rebuild = [&]() {
        std::fill(placed.begin(), placed.end(), 0);
        placed[pelvisIndex] = 1;
        bool progress = true;
        for (int guard = 0; progress && guard < 32; ++guard) {
            progress = false;
            for (const auto& joint : ragdoll.joints) {
                if (joint.parent < 0 || joint.child < 0 || placed[joint.parent] == 0 || placed[joint.child] != 0) {
                    continue;
                }
                const Quat target = reference[joint.id];
                const Quat parentFrame = frameQuat(joint.axisXInParent, joint.axisYInParent);
                const Quat childFrame = frameQuat(joint.axisXInChild, joint.axisYInChild);
                const Quat parentConstraint = mul(rotation[joint.parent], parentFrame);
                const Quat childConstraint = mul(parentConstraint, target);
                rotation[joint.child] = normalize(mul(childConstraint, conjugate(childFrame)));
                const Vec3 anchor = position[joint.parent] + rotate(rotation[joint.parent], joint.anchorInParent);
                position[joint.child] = anchor - rotate(rotation[joint.child], joint.anchorInChild);
                placed[joint.child] = 1;
                progress = true;
            }
        }
    };
    auto footPitch = [&](const std::string& footId) {
        const RagdollBody* foot = ragdoll.find(footId);
        if (foot == nullptr) {
            return 0.f;
        }
        int index = -1;
        for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
            if (ragdoll.bodies[i].id == footId) {
                index = i;
            }
        }
        const Vec3 local = rotate(conjugate(foot->initialRotation), foot->segmentEnd - foot->segmentStart);
        const Vec3 direction = rotate(rotation[index], local);
        const float scale = std::max(length(direction), 1e-4f);
        return direction.y / scale;
    };
    rebuild();
    for (const char* ankleId : {"ankle_1", "ankle_2"}) {
        const RagdollJoint* ankle = nullptr;
        for (const auto& joint : ragdoll.joints) {
            if (joint.id == ankleId) {
                ankle = &joint;
            }
        }
        if (ankle == nullptr || ankle->child < 0) {
            continue;
        }
        const std::string footId = ragdoll.bodies[ankle->child].id;
        float bestDegrees = config.ankleFlexionDeg;
        float bestScore = 1e9f;
        const float lower = degrees(ankle->base.swing2Min);
        const float upper = degrees(ankle->base.swing2Max);
        for (float angle = lower; angle <= upper + 0.1f; angle += 2.f) {
            reference[ankleId] = quatAxisAngle({0.f, 0.f, 1.f}, radians(std::min(angle, upper)));
            rebuild();
            const float pitch = footPitch(footId);
            const float score = std::fabs(pitch) + (pitch < 0.f ? 0.35f : 0.f);
            if (score < bestScore) {
                bestScore = score;
                bestDegrees = std::min(angle, upper);
            }
        }
        reference[ankleId] = quatAxisAngle({0.f, 0.f, 1.f}, radians(bestDegrees));
    }
    auto chestAxes = [&]() {
        const RagdollBody* chest = ragdoll.find("chest");
        Vec3 up{0.f, 1.f, 0.f};
        if (chest == nullptr) {
            return up;
        }
        int index = pelvisIndex;
        for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
            if (ragdoll.bodies[i].id == "chest") {
                index = i;
            }
        }
        const Vec3 local = rotate(conjugate(chest->initialRotation), chest->segmentEnd - chest->segmentStart);
        return normalize(rotate(rotation[index], local));
    };
    float bestStomach = 0.f;
    float bestChest = 0.f;
    float bestSpine = 1e9f;
    for (float stomach = -18.f; stomach <= 18.1f; stomach += 3.f) {
        for (float chest = -18.f; chest <= 18.1f; chest += 3.f) {
            reference["spine_stomach"] = quatFromEulerXYZ(0.f, 0.f, radians(stomach));
            reference["spine_chest"] = quatFromEulerXYZ(0.f, 0.f, radians(chest));
            rebuild();
            const Vec3 up = chestAxes();
            const float score = std::fabs(up.x) + std::fabs(up.z) + (up.y < 0.75f ? 2.f : 0.f);
            if (score < bestSpine) {
                bestSpine = score;
                bestStomach = stomach;
                bestChest = chest;
            }
        }
    }
    reference["spine_stomach"] = quatFromEulerXYZ(0.f, 0.f, radians(bestStomach));
    reference["spine_chest"] = quatFromEulerXYZ(0.f, 0.f, radians(bestChest));
    float hipDegrees = config.hipFlexionDeg;
    for (int iteration = 0; iteration < 14; ++iteration) {
        for (const char* hipId : {"hip_1", "hip_2"}) {
            for (const auto& joint : ragdoll.joints) {
                if (joint.id == hipId) {
                    const float limited = clampf(radians(hipDegrees), joint.base.swing2Min + radians(2.f), joint.base.swing2Max - radians(2.f));
                    const float spread = joint.id == "hip_1" ? radians(-18.f) : radians(18.f);
                    reference[hipId] = quatFromEulerXYZ(0.f, clampf(spread, joint.base.swing1Min, joint.base.swing1Max), limited);
                }
            }
        }
        rebuild();
        Vec3 com{};
        float mass = 0.f;
        Vec3 feet{};
        int footCount = 0;
        for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
            com = com + position[i] * ragdoll.bodies[i].mass;
            mass += ragdoll.bodies[i].mass;
            if (ragdoll.bodies[i].id == "foot_1" || ragdoll.bodies[i].id == "foot_2") {
                feet = feet + position[i];
                ++footCount;
            }
        }
        if (!(mass > 0.f) || footCount == 0) {
            break;
        }
        com = com * (1.f / mass);
        feet = feet * (1.f / float(footCount));
        const float error = com.z - feet.z;
        if (std::fabs(error) < 0.008f) {
            break;
        }
        hipDegrees = clampf(hipDegrees - error * 50.f, -25.f, 35.f);
    }
    rebuild();
    for (auto& command : placement.reference) {
        command.target = reference[command.jointId];
    }
    for (int i = 0; i < int(placed.size()); ++i) {
        if (placed[i] == 0) {
            placement.error = "Standing placement could not reach body '" + ragdoll.bodies[i].id + "'.";
            return placement;
        }
        if (!finite(position[i]) || !finite(rotation[i])) {
            placement.error = "Standing placement produced a non-finite pose for '" + ragdoll.bodies[i].id + "'.";
            return placement;
        }
    }

    for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
        world.setBodyPose(ragdoll.bodies[i].physics, position[i], rotation[i], true);
        world.setBodyVelocity(ragdoll.bodies[i].physics, {}, {});
    }

    float lowestFoot = 1e9f;
    float lowestBody = 1e9f;
    for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
        const float bottom = capsuleBottom(world.shapeView(ragdoll.bodies[i].physics));
        lowestBody = std::min(lowestBody, bottom);
        if (ragdoll.bodies[i].id == "foot_1" || ragdoll.bodies[i].id == "foot_2") {
            lowestFoot = std::min(lowestFoot, bottom);
        }
    }
    const float lift = 0.004f - std::min(lowestFoot, lowestBody);
    for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
        position[i].y += lift;
        world.setBodyPose(ragdoll.bodies[i].physics, position[i], rotation[i], true);
        world.setBodyVelocity(ragdoll.bodies[i].physics, {}, {});
    }
    placement.footHeight = lowestFoot + lift;

    placement.maxSeparation = 0.f;
    for (const auto& joint : ragdoll.joints) {
        placement.maxSeparation = std::max(placement.maxSeparation, world.jointState(joint.physics).separation);
    }
    // The forward-kinematics quaternion can differ slightly from the quaternion
    // Jolt reports in constraint space (hip roots especially). Motors must hold
    // the pose that was actually placed, or a few newton-metres of mismatch
    // torque walks the pelvis off the feet.
    for (auto& command : placement.reference) {
        for (const auto& joint : ragdoll.joints) {
            if (joint.id != command.jointId) {
                continue;
            }
            const Quat measured = world.jointState(joint.physics).constraintRotation;
            if (finite(measured)) {
                command.target = normalize(measured);
            }
        }
    }
    placement.ok = true;
    return placement;
}

}  // namespace mt
