#include "mortrauma/poses/PoseLibrary.hpp"

#include "mortrauma/core/Log.hpp"
#include "mortrauma/core/Paths.hpp"
#include "mortrauma/poses/PoseImporter.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>

namespace mt {
namespace {

std::filesystem::path optionalAsset(const std::filesystem::path& relative) {
    for (const auto& root : assetRoots()) {
        const auto direct = root / relative;
        if (std::filesystem::exists(direct)) {
            return std::filesystem::weakly_canonical(direct);
        }
        const auto underAssets = root / "assets" / relative;
        if (std::filesystem::exists(underAssets)) {
            return std::filesystem::weakly_canonical(underAssets);
        }
    }
    return {};
}

const RagdollJoint* findJoint(const Ragdoll& ragdoll, const std::string& id) {
    for (const auto& joint : ragdoll.joints) {
        if (joint.id == id) {
            return &joint;
        }
    }
    return nullptr;
}

Quat neutralOf(const PhysicsWorld& world, const RagdollJoint& joint) {
    const auto state = world.jointState(joint.physics);
    return state.valid ? normalize(state.constraintRotation) : Quat{};
}

Pose basePose(const PhysicsWorld& world, const Ragdoll& ragdoll, const std::string& name, const std::string& procedure,
              const std::string& description) {
    Pose pose;
    pose.name = name;
    pose.procedure = procedure;
    pose.description = description;
    for (const auto& joint : ragdoll.joints) {
        PoseJointTarget target;
        target.jointId = joint.id;
        target.orientation = neutralOf(world, joint);
        target.mapped = false;
        pose.joints.push_back(target);
    }
    return pose;
}

void setTarget(Pose& pose, const std::string& id, Quat orientation, bool driven) {
    for (auto& joint : pose.joints) {
        if (joint.jointId == id) {
            joint.orientation = normalize(orientation);
            joint.mapped = true;
            break;
        }
    }
    if (driven) {
        pose.driven.push_back(id);
    }
}

float readDegrees(const nlohmann::json& json, const char* key, float fallback, float lo, float hi) {
    if (!json.contains(key) || !json[key].is_number()) {
        return fallback;
    }
    const float value = json[key].get<float>();
    if (!std::isfinite(value) || value < lo || value > hi) {
        Log::warn(std::string("Ignoring out-of-range pose field ") + key);
        return fallback;
    }
    return value;
}

nlohmann::json readPoseFile(const char* relative) {
    const auto path = optionalAsset(relative);
    if (path.empty()) {
        return {};
    }
    std::ifstream input(path);
    if (!input) {
        return {};
    }
    try {
        nlohmann::json json;
        input >> json;
        return json;
    } catch (const std::exception& exception) {
        Log::warn(std::string("Pose file ignored: ") + exception.what());
        return {};
    }
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
    const Vec3 axis = normalize(Vec3{rotation.x, rotation.y, rotation.z});
    return quatAxisAngle(axis, maxRadians);
}

Quat swingAbout(const PhysicsWorld& world, const RagdollJoint& joint, Vec3 desiredWorld, float maxRadians) {
    const auto state = world.jointState(joint.physics);
    const Vec3 local{dot(desiredWorld, state.twistAxisWorld), dot(desiredWorld, state.swingAxisWorld), dot(desiredWorld, state.bendAxisWorld)};
    if (length(local) < 1e-5f) {
        return neutralOf(world, joint);
    }
    return clampRotation(quatFromTo({1.f, 0.f, 0.f}, normalize(local)), maxRadians);
}

}  // namespace

void PoseLibrary::rebuild(const PhysicsWorld& world, const Ragdoll& ragdoll) {
    poses_.clear();
    const auto armRaise = readPoseFile("poses/arm_raise.json");
    const auto armExtension = readPoseFile("poses/arm_extension.json");
    const auto legFlexion = readPoseFile("poses/leg_flexion.json");
    const auto compact = readPoseFile("poses/compact.json");
    const float raiseDegrees = readDegrees(armRaise, "raise_degrees", 65.f, 10.f, 75.f);
    const float kneeDegrees = readDegrees(legFlexion, "flex_degrees", 120.f, 20.f, 145.f);
    const float elbowDegrees = readDegrees(compact, "elbow_degrees", 120.f, 30.f, 145.f);
    const float compactKnee = readDegrees(compact, "knee_degrees", 125.f, 30.f, 145.f);
    const float hipDegrees = readDegrees(compact, "hip_degrees", 100.f, 40.f, 110.f);
    (void)armExtension;

    poses_.push_back(basePose(world, ragdoll, "Neutral", "neutral",
                              "Validated joint-relative reference. Each target is the constraint orientation of the imported pose."));
    for (auto& joint : poses_.back().joints) {
        joint.mapped = true;
        poses_.back().driven.push_back(joint.jointId);
    }

    for (int side = 1; side <= 2; ++side) {
        const std::string suffix = side == 1 ? "_1" : "_2";
        Pose raised = basePose(world, ragdoll, std::string("Arm Raise ") + (side == 1 ? "Right" : "Left"), "arm_raise",
                               "Raises one arm in the shoulder constraint frame, staying inside the shoulder cone.");
        if (const auto* shoulder = findJoint(ragdoll, "shoulder" + suffix)) {
            const auto state = world.jointState(shoulder->physics);
            const Vec3 bone = normalize(state.twistAxisWorld);
            const Vec3 up{0.f, 1.f, 0.f};
            Vec3 axis = cross(bone, up);
            float available = std::acos(clampf(dot(bone, up), -1.f, 1.f));
            float use = std::min(radians(raiseDegrees), std::max(0.f, available - radians(8.f)));
            if (length(axis) < 1e-4f) {
                axis = state.swingAxisWorld;
                use = radians(std::min(raiseDegrees, 70.f));
            }
            const Vec3 desired = rotate(quatAxisAngle(normalize(axis), use), bone);
            setTarget(raised, shoulder->id, swingAbout(world, *shoulder, desired, radians(75.f)), true);
        }
        if (const auto* clavicle = findJoint(ragdoll, "clavicle" + suffix)) {
            setTarget(raised, clavicle->id, neutralOf(world, *clavicle), true);
        }
        if (const auto* elbow = findJoint(ragdoll, "elbow" + suffix)) {
            setTarget(raised, elbow->id, neutralOf(world, *elbow), true);
        }
        poses_.push_back(raised);

        Pose extended = basePose(world, ragdoll, std::string("Arm Extension ") + (side == 1 ? "Right" : "Left"), "arm_extension",
                                 "Drives the elbow toward anatomical extension (constraint swing 2 of 0).");
        if (const auto* elbow = findJoint(ragdoll, "elbow" + suffix)) {
            setTarget(extended, elbow->id, Quat::identity(), true);
        }
        poses_.push_back(extended);

        Pose flexed = basePose(world, ragdoll, std::string("Leg Flexion ") + (side == 1 ? "Right" : "Left"), "leg_flexion",
                               "Bends one knee in the anatomical hinge frame, inside the 0 to 145 degree limit.");
        if (const auto* knee = findJoint(ragdoll, "knee" + suffix)) {
            const float angle = clampf(radians(kneeDegrees), knee->base.swing2Min, knee->base.swing2Max);
            setTarget(flexed, knee->id, quatAxisAngle({0.f, 0.f, 1.f}, angle), true);
        }
        poses_.push_back(flexed);
    }

    Pose curled = basePose(world, ragdoll, "Compact", "compact", "Curls both elbows, both knees and both hips in their joint frames.");
    for (const char* id : {"elbow_1", "elbow_2"}) {
        if (const auto* joint = findJoint(ragdoll, id)) {
            const float angle = clampf(radians(elbowDegrees), joint->base.swing2Min, joint->base.swing2Max);
            setTarget(curled, id, quatAxisAngle({0.f, 0.f, 1.f}, angle), true);
        }
    }
    for (const char* id : {"knee_1", "knee_2"}) {
        if (const auto* joint = findJoint(ragdoll, id)) {
            const float angle = clampf(radians(compactKnee), joint->base.swing2Min, joint->base.swing2Max);
            setTarget(curled, id, quatAxisAngle({0.f, 0.f, 1.f}, angle), true);
        }
    }
    for (const char* id : {"hip_1", "hip_2"}) {
        if (const auto* joint = findJoint(ragdoll, id)) {
            const float angle = clampf(radians(hipDegrees), joint->base.swing2Min, joint->base.swing2Max);
            setTarget(curled, id, quatAxisAngle({0.f, 0.f, 1.f}, angle), true);
        }
    }
    poses_.push_back(curled);

    Pose torso = basePose(world, ragdoll, "Torso Resistance", "torso_resistance",
                          "Holds the spine joints at their imported orientation relative to the pelvis.");
    for (const char* id : {"spine_stomach", "spine_chest", "spine_neck"}) {
        if (const auto* joint = findJoint(ragdoll, id)) {
            setTarget(torso, id, neutralOf(world, *joint), true);
        }
    }
    poses_.push_back(torso);

    Pose hold;
    hold.name = "Hold Captured Pose";
    hold.procedure = "hold_captured";
    hold.description = "Captures the current constraint orientation of every joint. This is not a world-space freeze.";
    hold.captureOnSelect = true;
    poses_.push_back(hold);
}

PoseImportResult PoseLibrary::importMixamo(const PhysicsWorld& world, const Ragdoll& ragdoll, const std::string& path) {
    import_ = importMixamoPose(world, ragdoll, path);
    if (import_.ok) {
        bool replaced = false;
        for (auto& pose : poses_) {
            if (pose.procedure == "mixamo_idle") {
                pose = import_.pose;
                replaced = true;
                break;
            }
        }
        if (!replaced) {
            poses_.push_back(import_.pose);
        }
    }
    return import_;
}

PoseImportResult PoseLibrary::importDefaultMixamo(const PhysicsWorld& world, const Ragdoll& ragdoll) {
    const auto path = optionalAsset("poses/imported/Idle.fbx");
    if (path.empty()) {
        import_ = {};
        import_.ok = false;
        import_.fileFound = false;
        import_.error = "No Mixamo reference file was supplied. Procedural poses are unchanged.";
        Log::info(import_.error);
        return import_;
    }
    return importMixamo(world, ragdoll, path.string());
}

const Pose* PoseLibrary::find(const std::string& name) const {
    for (const auto& pose : poses_) {
        if (pose.name == name) {
            return &pose;
        }
    }
    return nullptr;
}

bool PoseLibrary::savePose(const Pose& pose, const std::string& path, std::string& error) const {
    nlohmann::json json;
    json["name"] = pose.name;
    json["procedure"] = pose.procedure;
    json["description"] = pose.description;
    json["driven"] = pose.driven;
    json["unmapped"] = pose.unmapped;
    json["notes"] = pose.notes;
    for (const auto& joint : pose.joints) {
        json["joints"][joint.jointId] = {{"x", joint.orientation.x},
                                         {"y", joint.orientation.y},
                                         {"z", joint.orientation.z},
                                         {"w", joint.orientation.w},
                                         {"mapped", joint.mapped}};
    }
    std::ofstream output(path);
    if (!output) {
        error = "Could not write pose " + path;
        Log::error(error);
        return false;
    }
    output << json.dump(2) << '\n';
    return true;
}

}  // namespace mt
