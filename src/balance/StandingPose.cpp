#include "mortrauma/balance/StandingPose.hpp"

#include "mortrauma/balance/SupportContacts.hpp"
#include "mortrauma/core/Log.hpp"

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
    if (shape.shape == BodyCreateInfo::Shape::Box) {
        float lowest = 1e9f;
        const float signs[2] = {-1.f, 1.f};
        for (float sx : signs) {
            for (float sy : signs) {
                for (float sz : signs) {
                    const Vec3 local = shape.centerOffset + rotate(shape.shapeRotation, Vec3{sx * shape.halfExtents.x, sy * shape.halfExtents.y, sz * shape.halfExtents.z});
                    lowest = std::min(lowest, (shape.position + rotate(shape.rotation, local)).y);
                }
            }
        }
        return lowest;
    }
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
    auto soleUp = [&](const std::string& footId) {
        const RagdollBody* foot = ragdoll.find(footId);
        Vec3 up{0.f, 1.f, 0.f};
        if (foot == nullptr) {
            return up;
        }
        int index = pelvisIndex;
        for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
            if (ragdoll.bodies[i].id == footId) {
                index = i;
            }
        }
        return normalize(rotate(rotation[index], foot->soleNormalLocal));
    };
    auto levelAnkles = [&](bool local) {
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
        float bestTwist = 0.f;
        float bestSwing1 = 0.f;
        float bestSwing2 = config.ankleFlexionDeg;
        float bestScore = 1e9f;
        float x0 = degrees(ankle->base.twistMin);
        float x1 = degrees(ankle->base.twistMax);
        float y0 = degrees(ankle->base.swing1Min);
        float y1 = degrees(ankle->base.swing1Max);
        float z0 = degrees(ankle->base.swing2Min);
        float z1 = degrees(ankle->base.swing2Max);
        const Vec3 current = constraintEulerRadians(reference[ankleId]);
        float stepX = 4.f;
        float stepY = 3.f;
        float stepZ = 3.f;
        if (local) {
            x0 = std::max(x0, degrees(current.x) - 6.f);
            x1 = std::min(x1, degrees(current.x) + 6.f);
            y0 = std::max(y0, degrees(current.y) - 6.f);
            y1 = std::min(y1, degrees(current.y) + 6.f);
            z0 = std::max(z0, degrees(current.z) - 6.f);
            z1 = std::min(z1, degrees(current.z) + 6.f);
            stepX = 2.f;
            stepY = 2.f;
            stepZ = 2.f;
        }
        if (x1 < x0) {
            x0 = x1;
        }
        if (y1 < y0) {
            y0 = y1;
        }
        if (z1 < z0) {
            z0 = z1;
        }
        for (float twist = x0; twist <= x1 + 0.1f; twist += stepX) {
            for (float swing1 = y0; swing1 <= y1 + 0.1f; swing1 += stepY) {
                for (float swing2 = z0; swing2 <= z1 + 0.1f; swing2 += stepZ) {
                    const float x = std::min(twist, x1);
                    const float y = std::min(swing1, y1);
                    const float z = std::min(swing2, z1);
                    reference[ankleId] = quatFromEulerXYZ(radians(x), radians(y), radians(z));
                    rebuild();
                    const Vec3 up = soleUp(footId);
                    const float score = (1.f - up.y) + std::fabs(up.x) + std::fabs(up.z);
                    if (score < bestScore) {
                        bestScore = score;
                        bestTwist = x;
                        bestSwing1 = y;
                        bestSwing2 = z;
                    }
                }
            }
        }
        reference[ankleId] = quatFromEulerXYZ(radians(bestTwist), radians(bestSwing1), radians(bestSwing2));
        Log::info(std::string(ankleId) + " sole angles twist " + std::to_string(bestTwist) + " swing1 " + std::to_string(bestSwing1) +
                  " swing2 " + std::to_string(bestSwing2) + " score " + std::to_string(bestScore));
    }
    rebuild();
    };
    levelAnkles(false);
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
    if (config.relaxArms) {
        auto indexOf = [&](const char* id) {
            for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
                if (ragdoll.bodies[i].id == id) {
                    return i;
                }
            }
            return -1;
        };
        auto dropArm = [&](const char* shoulderId, const char* elbowId, const char* handId) {
            const int hand = indexOf(handId);
            const RagdollJoint* shoulder = nullptr;
            const RagdollJoint* elbow = nullptr;
            for (const auto& joint : ragdoll.joints) {
                if (joint.id == shoulderId) {
                    shoulder = &joint;
                }
                if (joint.id == elbowId) {
                    elbow = &joint;
                }
            }
            if (hand < 0 || shoulder == nullptr) {
                return;
            }
            float twist0 = degrees(shoulder->base.twistMin);
            float twist1 = degrees(shoulder->base.twistMax);
            float swing10 = degrees(shoulder->base.swing1Min);
            float swing11 = degrees(shoulder->base.swing1Max);
            float swing20 = degrees(shoulder->base.swing2Min);
            float swing21 = degrees(shoulder->base.swing2Max);
            if (shoulder->base.swing == SwingKind::Cone) {
                const float cone = std::max(std::fabs(swing11), std::fabs(swing21));
                swing10 = 0.f;
                swing11 = cone;
                swing20 = 0.f;
                swing21 = cone;
            }
            float best = 1e9f;
            float bestTwist = 0.f;
            float bestSwing1 = 0.f;
            float bestSwing2 = 0.f;
            for (float twist = twist0; twist <= twist1 + 0.1f; twist += 20.f) {
                for (float swing1 = swing10; swing1 <= swing11 + 0.1f; swing1 += 15.f) {
                    for (float swing2 = swing20; swing2 <= swing21 + 0.1f; swing2 += 15.f) {
                        const float x = std::min(twist, twist1);
                        const float y = std::min(swing1, swing11);
                        const float z = std::min(swing2, swing21);
                        reference[shoulderId] = quatFromEulerXYZ(radians(x), radians(y), radians(z));
                        rebuild();
                        const Vec3 point = position[hand];
                        const float lateral = std::fabs(point.x);
                        float score = std::fabs(point.y - 0.95f);
                        if (lateral < 0.14f) {
                            score += 3.f;
                        }
                        score += std::fabs(lateral - 0.26f) * 0.5f;
                        score += std::fabs(point.z) * 0.2f;
                        if (score < best) {
                            best = score;
                            bestTwist = x;
                            bestSwing1 = y;
                            bestSwing2 = z;
                        }
                    }
                }
            }
            reference[shoulderId] = quatFromEulerXYZ(radians(bestTwist), radians(bestSwing1), radians(bestSwing2));
            if (elbow != nullptr) {
                const float elbowMax = std::min(degrees(elbow->base.swing2Max), std::max(config.armDropDeg, 0.f));
                const float elbowMin = std::max(degrees(elbow->base.swing2Min), 0.f);
                float bestElbow = elbowMin;
                float bestElbowScore = 1e9f;
                for (float angle = elbowMin; angle <= elbowMax + 0.1f; angle += 10.f) {
                    const float limited = std::min(angle, elbowMax);
                    reference[elbowId] = quatFromEulerXYZ(0.f, 0.f, radians(limited));
                    rebuild();
                    const Vec3 point = position[hand];
                    float score = std::fabs(point.y - 0.9f) + std::fabs(point.z) * 0.3f;
                    if (std::fabs(point.x) < 0.12f) {
                        score += 2.f;
                    }
                    if (score < bestElbowScore) {
                        bestElbowScore = score;
                        bestElbow = limited;
                    }
                }
                reference[elbowId] = quatFromEulerXYZ(0.f, 0.f, radians(bestElbow));
            }
            Log::info(std::string(shoulderId) + " relaxed twist " + std::to_string(bestTwist) + " swing1 " + std::to_string(bestSwing1) +
                      " swing2 " + std::to_string(bestSwing2));
        };
        dropArm("shoulder_1", "elbow_1", "hand_1");
        dropArm("shoulder_2", "elbow_2", "hand_2");
        rebuild();
    }
    auto supportOf = [&](int bodyIndex) {
        const RagdollBody& foot = ragdoll.bodies[bodyIndex];
        const Vec3 heel = position[bodyIndex] + rotate(rotation[bodyIndex], foot.heelLocal);
        const Vec3 fore = position[bodyIndex] + rotate(rotation[bodyIndex], foot.forefootLocal);
        if (!config.equilibriumReference) {
            return heel * 0.75f + fore * 0.25f;
        }
        Vec3 ankle = position[bodyIndex];
        for (const auto& joint : ragdoll.joints) {
            if (joint.child == bodyIndex && (joint.id == "ankle_1" || joint.id == "ankle_2")) {
                ankle = position[bodyIndex] + rotate(rotation[bodyIndex], joint.anchorInChild);
            }
        }
        return feasibleSolePoint(ankle, heel, fore, config.equilibriumLeadM);
    };
    auto alignCom = [&]() {
        float hipDegrees = config.hipFlexionDeg;
        for (int iteration = 0; iteration < 14; ++iteration) {
            for (const char* hipId : {"hip_1", "hip_2"}) {
                for (const auto& joint : ragdoll.joints) {
                    if (joint.id == hipId) {
                        const float limited = clampf(radians(hipDegrees), joint.base.swing2Min + radians(2.f), joint.base.swing2Max - radians(2.f));
                        const float spread = (joint.id == "hip_1" ? -1.f : 1.f) * radians(config.hipAbductionDeg);
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
                    feet = feet + supportOf(i);
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
    };
    auto soleMeanY = [&](const char* footId) {
        int index = 0;
        for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
            if (ragdoll.bodies[i].id == footId) {
                index = i;
            }
        }
        const Vec3 heel = position[index] + rotate(rotation[index], ragdoll.bodies[index].heelLocal);
        const Vec3 toe = position[index] + rotate(rotation[index], ragdoll.bodies[index].forefootLocal);
        return 0.5f * (heel.y + toe.y);
    };
    auto kneeDegrees = [&](const char* id) { return degrees(constraintEulerRadians(reference[id]).z); };
    auto setKnee = [&](const char* id, float angle) {
        for (const auto& joint : ragdoll.joints) {
            if (joint.id == id) {
                reference[id] = quatFromEulerXYZ(0.f, 0.f, flexion(joint, clampf(angle, 4.f, 32.f)));
            }
        }
    };
    alignCom();
    levelAnkles(false);
    if (config.coplanarFeet) {
        // One knee correction and no second ankle search. Re-leveling the
        // ankles after a knee change walks the sole off the zero-moment pose
        // and the character falls. A single small knee change is kept only
        // when it actually closes the gap.
        rebuild();
        const float before = soleMeanY("foot_1") - soleMeanY("foot_2");
        const float knee1 = kneeDegrees("knee_1");
        const float knee2 = kneeDegrees("knee_2");
        const float step = clampf(before * 70.f, -2.5f, 2.5f);
        setKnee("knee_1", knee1 - step);
        setKnee("knee_2", knee2 + step);
        alignCom();
        rebuild();
        const float after = soleMeanY("foot_1") - soleMeanY("foot_2");
        if (std::fabs(after) > std::fabs(before) - 0.001f) {
            setKnee("knee_1", knee1);
            setKnee("knee_2", knee2);
            alignCom();
        }
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
    Vec3 com{};
    float mass = 0.f;
    Vec3 equilibrium{};
    int equilibriumCount = 0;
    float leftMean = 0.f;
    float rightMean = 0.f;
    for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
        com = com + position[i] * ragdoll.bodies[i].mass;
        mass += ragdoll.bodies[i].mass;
    }
    if (mass > 0.f) {
        com = com * (1.f / mass);
    }
    for (const char* footId : {"foot_1", "foot_2"}) {
        const RagdollBody* foot = ragdoll.find(footId);
        if (foot == nullptr) {
            continue;
        }
        int index = 0;
        for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
            if (ragdoll.bodies[i].id == footId) {
                index = i;
            }
        }
        const Vec3 up = normalize(rotate(rotation[index], foot->soleNormalLocal));
        const Vec3 heel = position[index] + rotate(rotation[index], foot->heelLocal);
        const Vec3 toe = position[index] + rotate(rotation[index], foot->forefootLocal);
        const float mean = 0.5f * (heel.y + toe.y);
        if (std::string(footId) == "foot_2") {
            leftMean = mean;
        } else {
            rightMean = mean;
        }
        placement.heelForeGapM = std::max(placement.heelForeGapM, std::fabs(heel.y - toe.y));
        placement.soleNormalError = std::max(placement.soleNormalError, std::acos(clampf(up.y, -1.f, 1.f)));
        Vec3 ankle = position[index];
        for (const auto& joint : ragdoll.joints) {
            if (joint.child == index && (joint.id == "ankle_1" || joint.id == "ankle_2")) {
                ankle = position[index] + rotate(rotation[index], joint.anchorInChild);
            }
        }
        equilibrium = equilibrium + feasibleSolePoint(ankle, heel, toe, config.equilibriumLeadM);
        ++equilibriumCount;
        const float ankleOffset = length(Vec3{ankle.x - heel.x, 0.f, ankle.z - heel.z});
        Log::info(std::string(footId) + " soleUp " + std::to_string(up.x) + " " + std::to_string(up.y) + " " + std::to_string(up.z) +
                  " heelY " + std::to_string(heel.y) + " toeY " + std::to_string(toe.y) + " ankleAheadOfHeel " + std::to_string(ankleOffset));
    }
    placement.coplanarityM = std::fabs(leftMean - rightMean);
    if (equilibriumCount > 0) {
        placement.equilibriumTarget = equilibrium * (1.f / float(equilibriumCount));
        placement.comToAnkleM = length(Vec3{com.x - placement.equilibriumTarget.x, 0.f, com.z - placement.equilibriumTarget.z});
    }
    Log::info("stance coplanar " + std::to_string(placement.coplanarityM) + " heelFore " + std::to_string(placement.heelForeGapM) +
              " soleError " + std::to_string(placement.soleNormalError) + " comToAnkle " + std::to_string(placement.comToAnkleM));

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
