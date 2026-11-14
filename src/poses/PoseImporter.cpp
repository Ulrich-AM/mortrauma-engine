#include "mortrauma/poses/PoseImporter.hpp"

#include "mortrauma/core/Log.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <unordered_map>

namespace mt {
namespace {

std::string canonBone(std::string name) {
    const auto colon = name.rfind(':');
    if (colon != std::string::npos) {
        name = name.substr(colon + 1);
    }
    const auto bar = name.rfind('|');
    if (bar != std::string::npos) {
        name = name.substr(bar + 1);
    }
    std::string out;
    out.reserve(name.size());
    for (unsigned char character : name) {
        if (std::isalnum(character)) {
            out.push_back(static_cast<char>(std::tolower(character)));
        }
    }
    return out;
}

aiQuaternion rotationAt(const aiNodeAnim* channel) {
    if (channel == nullptr || channel->mNumRotationKeys == 0) {
        return aiQuaternion();
    }
    // The reference frame is the clip's first key, not a playback of the curve.
    return channel->mRotationKeys[0].mValue;
}

aiVector3D positionAt(const aiNodeAnim* channel, const aiVector3D& fallback) {
    if (channel == nullptr || channel->mNumPositionKeys == 0) {
        return fallback;
    }
    return channel->mPositionKeys[0].mValue;
}

Quat clampRotation(Quat rotation, float maxRadians) {
    rotation = normalize(rotation);
    if (rotation.w < 0.f) {
        rotation = -rotation;
    }
    const float angle = quatAngle(rotation);
    if (angle <= maxRadians || angle < 1e-5f) {
        return rotation;
    }
    return quatAxisAngle(normalize(Vec3{rotation.x, rotation.y, rotation.z}), maxRadians);
}

struct SampledRig {
    std::unordered_map<std::string, Vec3> position;
    std::vector<std::string> names;
    bool animated = false;
    float scale = 1.f;
};

void walk(const aiNode* node, Vec3 parentPos, Quat parentRot, const std::unordered_map<std::string, const aiNodeAnim*>& channels, SampledRig& rig) {
    aiVector3D scale;
    aiQuaternion rotation;
    aiVector3D translation;
    node->mTransformation.Decompose(scale, rotation, translation);
    const auto found = channels.find(node->mName.C_Str());
    const aiNodeAnim* channel = found == channels.end() ? nullptr : found->second;
    if (channel != nullptr && channel->mNumRotationKeys > 0) {
        rotation = rotationAt(channel);
    }
    translation = positionAt(channel, translation);
    const Vec3 localT{translation.x, translation.y, translation.z};
    const Quat localR{rotation.x, rotation.y, rotation.z, rotation.w};
    const Vec3 world = parentPos + rotate(parentRot, localT);
    const Quat worldR = normalize(mul(parentRot, localR));
    const std::string key = canonBone(node->mName.C_Str());
    rig.position[key] = world;
    rig.names.push_back(key);
    for (unsigned child = 0; child < node->mNumChildren; ++child) {
        walk(node->mChildren[child], world, worldR, channels, rig);
    }
}

bool sample(const std::string& path, SampledRig& rig, std::string& error, bool preservePivots) {
    Assimp::Importer importer;
    importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, preservePivots);
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
    if (scene == nullptr || scene->mRootNode == nullptr) {
        error = importer.GetErrorString();
        return false;
    }
    std::unordered_map<std::string, const aiNodeAnim*> channels;
    if (scene->mNumAnimations > 0 && scene->mAnimations[0] != nullptr) {
        rig.animated = true;
        const aiAnimation* animation = scene->mAnimations[0];
        for (unsigned index = 0; index < animation->mNumChannels; ++index) {
            const aiNodeAnim* channel = animation->mChannels[index];
            if (channel != nullptr) {
                channels[channel->mNodeName.C_Str()] = channel;
            }
        }
    }
    walk(scene->mRootNode, {}, {}, channels, rig);
    float peak = 0.f;
    for (const auto& [name, position] : rig.position) {
        (void)name;
        peak = std::max(peak, std::max(std::fabs(position.x), std::max(std::fabs(position.y), std::fabs(position.z))));
    }
    if (peak > 20.f) {
        rig.scale = 0.01f;
        for (auto& [name, position] : rig.position) {
            (void)name;
            position = position * rig.scale;
        }
    }
    return rig.position.find("hips") != rig.position.end() || rig.position.find("rightarm") != rig.position.end();
}

Vec3 requireBone(const SampledRig& rig, const char* name, bool& ok) {
    const auto found = rig.position.find(name);
    if (found == rig.position.end()) {
        ok = false;
        return {};
    }
    return found->second;
}

float flexionAngle(Vec3 start, Vec3 mid, Vec3 end, bool& ok) {
    const Vec3 incoming = mid - start;
    const Vec3 outgoing = end - mid;
    if (length(incoming) < 1e-4f || length(outgoing) < 1e-4f) {
        ok = false;
        return 0.f;
    }
    return std::acos(clampf(dot(normalize(incoming), normalize(outgoing)), -1.f, 1.f));
}

const RagdollJoint* findJoint(const Ragdoll& ragdoll, const std::string& id) {
    for (const auto& joint : ragdoll.joints) {
        if (joint.id == id) {
            return &joint;
        }
    }
    return nullptr;
}

Quat directionTarget(const PhysicsWorld& world, const RagdollJoint& joint, Vec3 desiredWorld, float maxRadians) {
    const auto state = world.jointState(joint.physics);
    const Vec3 local{dot(desiredWorld, state.twistAxisWorld), dot(desiredWorld, state.swingAxisWorld), dot(desiredWorld, state.bendAxisWorld)};
    if (!state.valid || length(local) < 1e-5f) {
        return state.constraintRotation;
    }
    return clampRotation(quatFromTo({1.f, 0.f, 0.f}, normalize(local)), maxRadians);
}

}  // namespace

PoseImportResult importMixamoPose(const PhysicsWorld& world, const Ragdoll& ragdoll, const std::string& path) {
    PoseImportResult result;
    result.sourcePath = path;
    result.fileFound = !path.empty() && std::filesystem::exists(path);
    if (!result.fileFound) {
        result.error = "Mixamo reference file is not available.";
        Log::info(result.error);
        return result;
    }

    SampledRig rig;
    std::string error;
    if (!sample(path, rig, error, false) && !sample(path, rig, error, true)) {
        result.error = "Mixamo file could not be read: " + error;
        Log::error(result.error);
        return result;
    }

    result.pose.name = "Mixamo Idle";
    result.pose.procedure = "mixamo_idle";
    result.pose.description = "One retargeted Mixamo frame. Bone directions were converted into Humanoidguy constraint frames.";
    result.pose.notes.push_back(rig.animated ? "Source is an animation. The reference frame is the first key of each channel, not a played timeline."
                                            : "Source has no animation stack. Node bind transforms were used.");
    result.pose.notes.push_back(
        "Humanoidguy side 1 is +X, the character's right when facing -Z. Mixamo Right maps to side 1 and Mixamo Left to side 2.");
    result.pose.notes.push_back("Mixamo local rotations were not copied. Elbow and knee targets are flexion angles about constraint Z.");
    if (rig.scale != 1.f) {
        result.pose.notes.push_back("Source positions were larger than 20 and were scaled by 0.01 before measuring directions.");
    }

    bool bonesOk = true;
    const Vec3 rightArm = requireBone(rig, "rightarm", bonesOk);
    const Vec3 rightForearm = requireBone(rig, "rightforearm", bonesOk);
    const Vec3 rightHand = requireBone(rig, "righthand", bonesOk);
    const Vec3 leftArm = requireBone(rig, "leftarm", bonesOk);
    const Vec3 leftForearm = requireBone(rig, "leftforearm", bonesOk);
    const Vec3 leftHand = requireBone(rig, "lefthand", bonesOk);
    const Vec3 rightUp = requireBone(rig, "rightupleg", bonesOk);
    const Vec3 rightLeg = requireBone(rig, "rightleg", bonesOk);
    const Vec3 rightFoot = requireBone(rig, "rightfoot", bonesOk);
    const Vec3 leftUp = requireBone(rig, "leftupleg", bonesOk);
    const Vec3 leftLeg = requireBone(rig, "leftleg", bonesOk);
    const Vec3 leftFoot = requireBone(rig, "leftfoot", bonesOk);
    if (!bonesOk) {
        result.error = "Mixamo skeleton is missing one of the limb bones required for retargeting.";
        result.pose.notes.push_back("Bones seen: " + std::to_string(rig.names.size()));
        Log::error(result.error);
        return result;
    }

    Vec3 across = rightUp - leftUp;
    across.y = 0.f;
    if (length(across) < 1e-3f) {
        across = rightArm - leftArm;
        across.y = 0.f;
    }
    const Vec3 up{0.f, 1.f, 0.f};
    const Vec3 mixamoRight = normalize(across);
    const Vec3 mixamoForward = normalize(cross(mixamoRight, up));

    Vec3 humanFacing{0.f, 0.f, -1.f};
    if (const RagdollBody* foot = ragdoll.find("foot_1")) {
        Vec3 facing = foot->segmentEnd - foot->segmentStart;
        facing.y = 0.f;
        if (length(facing) > 0.02f) {
            humanFacing = normalize(facing);
        }
    }
    const Vec3 humanRight = normalize(cross(humanFacing, up));

    auto inHuman = [&](Vec3 direction) {
        direction = normalize(direction);
        const float right = dot(direction, mixamoRight);
        const float upward = dot(direction, up);
        const float forward = dot(direction, mixamoForward);
        return normalize(humanRight * right + up * upward + humanFacing * forward);
    };

    auto flexion = [&](const char* jointId, Vec3 proximal, Vec3 mid, Vec3 distal) {
        bool ok = true;
        const float angle = flexionAngle(proximal, mid, distal, ok);
        const auto* joint = findJoint(ragdoll, jointId);
        if (!ok || joint == nullptr) {
            result.pose.unmapped.push_back(jointId);
            return;
        }
        const float clamped = clampf(angle, joint->base.swing2Min, joint->base.swing2Max);
        PoseJointTarget target;
        target.jointId = jointId;
        target.orientation = quatAxisAngle({0.f, 0.f, 1.f}, clamped);
        target.mapped = true;
        result.pose.joints.push_back(target);
        result.pose.driven.push_back(jointId);
        result.pose.notes.push_back(std::string(jointId) + " flexion " + std::to_string(degrees(clamped)) + " deg from bone directions.");
    };
    flexion("elbow_1", rightArm, rightForearm, rightHand);
    flexion("elbow_2", leftArm, leftForearm, leftHand);
    flexion("knee_1", rightUp, rightLeg, rightFoot);
    flexion("knee_2", leftUp, leftLeg, leftFoot);

    auto direction = [&](const char* jointId, Vec3 from, Vec3 to, float maxDegrees) {
        const auto* joint = findJoint(ragdoll, jointId);
        const Vec3 bone = to - from;
        if (joint == nullptr || length(bone) < 1e-4f) {
            result.pose.unmapped.push_back(jointId);
            return;
        }
        PoseJointTarget target;
        target.jointId = jointId;
        target.orientation = directionTarget(world, *joint, inHuman(bone), radians(maxDegrees));
        target.mapped = true;
        result.pose.joints.push_back(target);
        result.pose.driven.push_back(jointId);
    };
    direction("shoulder_1", rightArm, rightForearm, 75.f);
    direction("shoulder_2", leftArm, leftForearm, 75.f);
    direction("hip_1", rightUp, rightLeg, 110.f);
    direction("hip_2", leftUp, leftLeg, 110.f);

    const char* retained[] = {"spine_stomach", "spine_chest", "spine_neck", "neck",     "clavicle_1", "clavicle_2", "wrist_1", "wrist_2",
                              "hip_root_1",    "hip_root_2",  "ankle_1",     "ankle_2"};
    for (const char* id : retained) {
        const auto* joint = findJoint(ragdoll, id);
        if (joint == nullptr) {
            continue;
        }
        const auto state = world.jointState(joint->physics);
        PoseJointTarget target;
        target.jointId = id;
        target.orientation = state.valid ? normalize(state.constraintRotation) : Quat{};
        target.mapped = false;
        result.pose.joints.push_back(target);
        result.pose.unmapped.push_back(id);
    }
    result.pose.notes.push_back("Unmapped joints keep the Humanoidguy reference orientation. No mapping was invented for them.");

    for (const auto& joint : result.pose.joints) {
        if (!finite(joint.orientation) || !(dot(joint.orientation, joint.orientation) > 0.5f)) {
            result.error = "Retarget produced a non-finite target for " + joint.jointId;
            Log::error(result.error);
            return result;
        }
    }
    result.ok = true;
    Log::info("Retargeted Mixamo idle onto " + std::to_string(result.pose.driven.size()) + " Humanoidguy joints. " +
              std::to_string(result.pose.unmapped.size()) + " joints kept the Humanoidguy reference.");
    return result;
}

}  // namespace mt
