#include "mortrauma/rig/Ragdoll.hpp"

#include "mortrauma/core/Log.hpp"

#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace mt {
namespace {

float pointSegmentDistance(Vec3 point, Vec3 start, Vec3 end) {
    const Vec3 edge = end - start;
    const float denom = lengthSq(edge);
    if (denom < 1e-8f) {
        return length(point - start);
    }
    const float t = clampf(dot(point - start, edge) / denom, 0.f, 1.f);
    return length(point - (start + edge * t));
}

float segmentDistance(Vec3 a0, Vec3 a1, Vec3 b0, Vec3 b1) {
    // Endpoint sampling is enough to reject obvious self-intersections.
    return std::min(std::min(pointSegmentDistance(a0, b0, b1), pointSegmentDistance(a1, b0, b1)),
                    std::min(pointSegmentDistance(b0, a0, a1), pointSegmentDistance(b1, a0, a1)));
}

void orthonormal(Vec3 x, Vec3 yHint, Vec3& axisX, Vec3& axisY) {
    axisX = normalize(x);
    Vec3 z = cross(axisX, yHint);
    if (length(z) < 1e-4f) {
        const Vec3 fallback = std::fabs(axisX.y) < 0.9f ? Vec3{0.f, 1.f, 0.f} : Vec3{1.f, 0.f, 0.f};
        z = cross(axisX, fallback);
    }
    z = normalize(z);
    axisY = normalize(cross(z, axisX));
}

Vec3 localPoint(Quat rotation, Vec3 position, Vec3 world) { return rotate(conjugate(rotation), world - position); }
Vec3 localDir(Quat rotation, Vec3 world) { return rotate(conjugate(rotation), world); }

}  // namespace

const RagdollBody* Ragdoll::find(const std::string& id) const {
    for (const auto& body : bodies) {
        if (body.id == id) {
            return &body;
        }
    }
    return nullptr;
}

int disconnectedComponents(const Ragdoll& ragdoll) {
    const int count = int(ragdoll.bodies.size());
    if (count == 0) {
        return 0;
    }
    std::vector<std::vector<int>> neighbors(count);
    for (const auto& joint : ragdoll.joints) {
        if (joint.parent < 0 || joint.child < 0) {
            continue;
        }
        neighbors[joint.parent].push_back(joint.child);
        neighbors[joint.child].push_back(joint.parent);
    }
    std::vector<char> seen(count, 0);
    int components = 0;
    for (int i = 0; i < count; ++i) {
        if (seen[i]) {
            continue;
        }
        ++components;
        std::queue<int> pending;
        pending.push(i);
        seen[i] = 1;
        while (!pending.empty()) {
            const int current = pending.front();
            pending.pop();
            for (int next : neighbors[current]) {
                if (!seen[next]) {
                    seen[next] = 1;
                    pending.push(next);
                }
            }
        }
    }
    return components;
}

int Ragdoll::componentCount() const { return mt::disconnectedComponents(*this); }

Ragdoll RagdollBuilder::build(PhysicsWorld& world, const CharacterConfig& config, const ImportedModel& model,
                              const JointPreset& preset) {
    Ragdoll ragdoll;
    if (!config.error.empty()) {
        ragdoll.error = config.error;
        Log::error(ragdoll.error);
        return ragdoll;
    }
    if (!model.report.ok) {
        ragdoll.error = model.report.error.empty() ? "Cannot build a ragdoll from a failed import." : model.report.error;
        Log::error(ragdoll.error);
        return ragdoll;
    }

    std::unordered_map<std::string, int> bodyIndex;
    std::unordered_map<int, int> boneOwner;
    std::vector<Vec3> centers(config.bodies.size());
    std::vector<Quat> rotations(config.bodies.size());

    for (int i = 0; i < int(config.bodies.size()); ++i) {
        const BodyConfig& spec = config.bodies[i];
        const int primary = model.findBone(spec.primaryBone);
        if (primary < 0) {
            ragdoll.error = "Body '" + spec.id + "' references missing bone '" + spec.primaryBone + "'.";
            Log::error(ragdoll.error);
            return ragdoll;
        }
        Vec3 start = model.bones[primary].bindPosition;
        Vec3 end = start + rotate(model.bones[primary].bindRotation, Vec3{0.f, spec.endExtension, 0.f});
        if (!spec.endBone.empty()) {
            const int endBone = model.findBone(spec.endBone);
            if (endBone < 0) {
                ragdoll.error = "Body '" + spec.id + "' references missing end bone '" + spec.endBone + "'.";
                Log::error(ragdoll.error);
                return ragdoll;
            }
            const Vec3 endPoint = model.bones[endBone].bindPosition;
            const Vec3 direction = endPoint - start;
            if (length(direction) > 0.01f) {
                end = endPoint + normalize(direction) * spec.endExtension;
            }
        }
        if (length(end - start) < 0.04f) {
            end = start + Vec3{0.f, 0.08f, 0.f};
        }

        float radius = spec.radius;
        if (radius <= 0.f) {
            float farthest = spec.minRadius;
            for (const auto& mesh : model.meshes) {
                if (mesh.bone < 0) {
                    continue;
                }
                bool owned = false;
                for (const auto& name : spec.bones) {
                    if (model.bones[mesh.bone].name == name) {
                        owned = true;
                        break;
                    }
                }
                if (!owned) {
                    continue;
                }
                for (const auto& vertex : mesh.vertices) {
                    const Vec3 worldPoint = transformPoint(model.bones[mesh.bone].bindWorld, vertex.position);
                    farthest = std::max(farthest, pointSegmentDistance(worldPoint, start, end));
                }
            }
            radius = clampf(farthest * 0.92f, spec.minRadius, spec.maxRadius);
        }

        const Vec3 center = (start + end) * 0.5f;
        const Quat rotation = quatFromTo(Vec3{0.f, 1.f, 0.f}, end - start);
        const float full = length(end - start);
        BodyCreateInfo info;
        info.name = spec.name;
        info.position = center;
        info.rotation = rotation;
        info.mass = spec.mass;
        info.friction = spec.friction;
        info.restitution = spec.restitution;
        info.linearDamping = preset.linearDamping;
        info.angularDamping = preset.angularDamping;
        info.collisionSubGroup = i;
        if (full <= radius * 2.05f) {
            info.shape = BodyCreateInfo::Shape::Sphere;
            info.radius = std::max(radius, full * 0.5f);
            info.halfHeight = 0.f;
        } else {
            info.shape = BodyCreateInfo::Shape::Capsule;
            info.radius = radius;
            info.halfHeight = full * 0.5f - radius;
        }
        const int physics = world.createBody(info);
        if (physics < 0) {
            ragdoll.error = "Failed to create physical body '" + spec.id + "'.";
            Log::error(ragdoll.error);
            return ragdoll;
        }

        RagdollBody body;
        body.id = spec.id;
        body.name = spec.name;
        body.physics = physics;
        body.subGroup = i;
        body.primaryBone = primary;
        body.segmentStart = start;
        body.segmentEnd = end;
        body.radius = info.radius;
        body.mass = spec.mass;
        body.initialPosition = center;
        body.initialRotation = rotation;
        const Mat4 bodyWorld = Mat4::fromRotationTranslation(rotation, center);
        const Mat4 inverseBody = inverseRigid(bodyWorld);
        for (const auto& boneName : spec.bones) {
            const int bone = model.findBone(boneName);
            if (bone < 0) {
                ragdoll.warnings.push_back("Body '" + spec.id + "' lists unknown bone '" + boneName + "'.");
                continue;
            }
            if (boneOwner.count(bone) != 0) {
                ragdoll.error = "Bone '" + boneName + "' is assigned to more than one rigid body.";
                Log::error(ragdoll.error);
                return ragdoll;
            }
            boneOwner[bone] = i;
            body.bones.push_back(bone);
            body.boneFromBody.push_back(mul(inverseBody, model.bones[bone].bindWorld));
        }
        if (body.bones.empty()) {
            ragdoll.error = "Body '" + spec.id + "' has no resolved bones.";
            Log::error(ragdoll.error);
            return ragdoll;
        }
        centers[i] = center;
        rotations[i] = rotation;
        bodyIndex.emplace(body.id, int(ragdoll.bodies.size()));
        ragdoll.bodies.push_back(std::move(body));
    }

    for (const auto& mesh : model.meshes) {
        if (mesh.bone >= 0 && boneOwner.count(mesh.bone) == 0) {
            ragdoll.error = "Skinned bone '" + mesh.boneName + "' is not driven by a rigid body.";
            Log::error(ragdoll.error);
            return ragdoll;
        }
    }

    const int pelvis = bodyIndex.count("pelvis") ? bodyIndex["pelvis"] : -1;
    Vec3 forward{0.f, 0.f, 1.f};
    if (bodyIndex.count("hip_1") && bodyIndex.count("hip_2")) {
        const Vec3 left = ragdoll.bodies[bodyIndex["hip_1"]].segmentStart;
        const Vec3 right = ragdoll.bodies[bodyIndex["hip_2"]].segmentStart;
        const Vec3 across = right - left;
        Vec3 guessed = cross(Vec3{0.f, 1.f, 0.f}, across);
        if (length(guessed) > 1e-4f) {
            forward = normalize(guessed);
        }
    }

    world.configureRagdollFilter(int(ragdoll.bodies.size()));

    for (const auto& spec : config.joints) {
        if (!bodyIndex.count(spec.parent) || !bodyIndex.count(spec.child)) {
            ragdoll.error = "Joint '" + spec.id + "' references an unknown body.";
            Log::error(ragdoll.error);
            return ragdoll;
        }
        const int parent = bodyIndex[spec.parent];
        const int child = bodyIndex[spec.child];
        const auto& parentBody = ragdoll.bodies[parent];
        const auto& childBody = ragdoll.bodies[child];
        const Vec3 anchor = childBody.segmentStart;
        Vec3 twist = childBody.segmentEnd - childBody.segmentStart;
        if (length(twist) < 1e-4f) {
            twist = {0.f, -1.f, 0.f};
        }
        Vec3 axisX;
        Vec3 axisY;
        if (spec.orient == BendOrient::None) {
            orthonormal(twist, Vec3{0.f, 1.f, 0.f}, axisX, axisY);
        } else {
            Vec3 desired{0.f, 1.f, 0.f};
            if (spec.orient == BendOrient::Elbow) {
                desired = parentBody.segmentStart - childBody.segmentEnd;
            } else if (spec.orient == BendOrient::Knee && pelvis >= 0) {
                desired = ragdoll.bodies[pelvis].initialPosition - childBody.segmentEnd;
            } else if (spec.orient == BendOrient::Hip) {
                desired = forward + Vec3{0.f, 0.6f, 0.f};
            }
            Vec3 hinge = cross(twist, desired);
            if (length(hinge) < 1e-4f) {
                hinge = cross(twist, Vec3{1.f, 0.f, 0.f});
            }
            hinge = normalize(hinge);
            if (dot(cross(hinge, normalize(twist)), desired) < 0.f) {
                hinge = -hinge;
            }
            axisX = normalize(twist);
            axisY = normalize(cross(hinge, axisX));
        }

        const float scale = preset.limitScale;
        JointCreateInfo info;
        info.name = spec.name;
        info.parent = parentBody.physics;
        info.child = childBody.physics;
        info.anchorInParent = localPoint(rotations[parent], centers[parent], anchor);
        info.axisXInParent = localDir(rotations[parent], axisX);
        info.axisYInParent = localDir(rotations[parent], axisY);
        info.anchorInChild = localPoint(rotations[child], centers[child], anchor);
        info.axisXInChild = localDir(rotations[child], axisX);
        info.axisYInChild = localDir(rotations[child], axisY);
        info.swing = spec.swing == SwingKind::Cone ? SwingType::Cone : SwingType::Pyramid;
        info.twistMin = spec.twistMin * scale;
        info.twistMax = spec.twistMax * scale;
        info.swing1Min = spec.swing1Min * scale;
        info.swing1Max = spec.swing1Max * scale;
        info.swing2Min = spec.swing2Min * scale;
        info.swing2Max = spec.swing2Max * scale;
        if (spec.swing == SwingKind::Cone) {
            const float cone = std::max(std::fabs(info.swing1Max), std::fabs(info.swing2Max));
            info.swing1Min = 0.f;
            info.swing1Max = cone;
            info.swing2Min = 0.f;
            info.swing2Max = cone;
        }
        info.angularFriction = preset.angularFriction;
        const int joint = world.createJoint(info);
        if (joint < 0) {
            ragdoll.error = "Failed to create joint '" + spec.id + "'.";
            Log::error(ragdoll.error);
            return ragdoll;
        }
        RagdollJoint record;
        record.id = spec.id;
        record.name = spec.name;
        record.physics = joint;
        record.parent = parent;
        record.child = child;
        record.base = spec;
        ragdoll.joints.push_back(record);
        world.setPairCollision(parentBody.subGroup, childBody.subGroup, false);
    }

    const int components = disconnectedComponents(ragdoll);
    ragdoll.disconnectedComponents = components;
    if (components != 1) {
        ragdoll.error = "Ragdoll constraint graph has " + std::to_string(components) + " components; expected 1.";
        Log::error(ragdoll.error);
        return ragdoll;
    }

    auto reaches = [&](const std::string& root, const std::string& target) {
        if (!bodyIndex.count(root) || !bodyIndex.count(target)) {
            return false;
        }
        std::vector<char> seen(ragdoll.bodies.size(), 0);
        std::queue<int> pending;
        pending.push(bodyIndex[root]);
        seen[bodyIndex[root]] = 1;
        while (!pending.empty()) {
            const int current = pending.front();
            pending.pop();
            if (current == bodyIndex[target]) {
                return true;
            }
            for (const auto& joint : ragdoll.joints) {
                int next = -1;
                if (joint.parent == current) {
                    next = joint.child;
                } else if (joint.child == current) {
                    next = joint.parent;
                }
                if (next >= 0 && !seen[next]) {
                    seen[next] = 1;
                    pending.push(next);
                }
            }
        }
        return false;
    };
    ragdoll.legsAttached = reaches("pelvis", "foot_1") && reaches("pelvis", "foot_2") && reaches("pelvis", "thigh_1") &&
                           reaches("pelvis", "thigh_2") && reaches("pelvis", "hip_1") && reaches("pelvis", "hip_2");
    if (!ragdoll.legsAttached) {
        ragdoll.error = "One or both legs are not constrained to the pelvis.";
        Log::error(ragdoll.error);
        return ragdoll;
    }

    // Direct joints plus bodies one joint away, and any bind-pose overlap.
    std::vector<std::vector<int>> neighbors(ragdoll.bodies.size());
    for (const auto& joint : ragdoll.joints) {
        neighbors[joint.parent].push_back(joint.child);
        neighbors[joint.child].push_back(joint.parent);
    }
    for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
        std::unordered_set<int> near;
        for (int n : neighbors[i]) {
            near.insert(n);
            for (int n2 : neighbors[n]) {
                near.insert(n2);
            }
        }
        for (int other = i + 1; other < int(ragdoll.bodies.size()); ++other) {
            bool disable = near.count(other) != 0;
            if (!disable) {
                const float gap = segmentDistance(ragdoll.bodies[i].segmentStart, ragdoll.bodies[i].segmentEnd,
                                                  ragdoll.bodies[other].segmentStart, ragdoll.bodies[other].segmentEnd);
                disable = gap < (ragdoll.bodies[i].radius + ragdoll.bodies[other].radius) * 0.75f;
            }
            if (disable) {
                world.setPairCollision(ragdoll.bodies[i].subGroup, ragdoll.bodies[other].subGroup, false);
            }
        }
    }

    for (const auto& warning : ragdoll.warnings) {
        Log::warn(warning);
    }
    ragdoll.ok = true;
    Log::info("Ragdoll built bodies=" + std::to_string(ragdoll.bodies.size()) +
              " joints=" + std::to_string(ragdoll.joints.size()) + " components=1");
    return ragdoll;
}

void applyPreset(PhysicsWorld& world, Ragdoll& ragdoll, const JointPreset& preset) {
    for (const auto& body : ragdoll.bodies) {
        world.setBodyDamping(body.physics, preset.linearDamping, preset.angularDamping);
    }
    for (auto& joint : ragdoll.joints) {
        const float scale = preset.limitScale;
        const auto& spec = joint.base;
        float twistMin = spec.twistMin * scale;
        float twistMax = spec.twistMax * scale;
        float swing1Min = spec.swing1Min * scale;
        float swing1Max = spec.swing1Max * scale;
        float swing2Min = spec.swing2Min * scale;
        float swing2Max = spec.swing2Max * scale;
        if (spec.swing == SwingKind::Cone) {
            const float cone = std::max(std::fabs(swing1Max), std::fabs(swing2Max));
            swing1Min = 0.f;
            swing1Max = cone;
            swing2Min = 0.f;
            swing2Max = cone;
        }
        world.setJointLimits(joint.physics, twistMin, twistMax, swing1Min, swing1Max, swing2Min, swing2Max);
        world.setJointAngularFriction(joint.physics, preset.angularFriction);
    }
}

void resetRagdoll(PhysicsWorld& world, Ragdoll& ragdoll) {
    world.clearGrabs();
    for (const auto& body : ragdoll.bodies) {
        world.setBodyPose(body.physics, body.initialPosition, body.initialRotation, true);
        world.setBodyVelocity(body.physics, {}, {});
    }
}

PosePalette posePalette(const PhysicsWorld& world, const Ragdoll& ragdoll, const ImportedModel& model) {
    PosePalette palette;
    palette.boneWorld.resize(model.bones.size(), Mat4::identity());
    std::vector<char> driven(model.bones.size(), 0);
    for (const auto& body : ragdoll.bodies) {
        const BodyState state = world.bodyState(body.physics);
        if (!state.valid) {
            continue;
        }
        const Mat4 current = Mat4::fromRotationTranslation(state.rotation, state.position);
        for (int i = 0; i < int(body.bones.size()); ++i) {
            const int bone = body.bones[i];
            palette.boneWorld[bone] = mul(current, body.boneFromBody[i]);
            driven[bone] = 1;
        }
    }
    for (int i = 0; i < int(model.bones.size()); ++i) {
        if (!driven[i]) {
            palette.boneWorld[i] = model.bones[i].bindWorld;
        }
    }
    return palette;
}

}  // namespace mt
