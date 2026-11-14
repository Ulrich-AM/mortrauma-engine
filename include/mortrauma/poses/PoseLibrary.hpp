#pragma once

#include "mortrauma/core/Math.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <string>
#include <vector>

namespace mt {

struct PoseJointTarget {
    std::string jointId;
    Quat orientation{};
    bool mapped = false;
};

struct Pose {
    std::string name;
    std::string procedure;
    std::string description;
    // Joints this pose intends to drive. Other joints still carry a valid
    // target (the bind orientation) so a transition has somewhere to go.
    std::vector<std::string> driven;
    std::vector<PoseJointTarget> joints;
    std::vector<std::string> unmapped;
    std::vector<std::string> notes;
    // Selecting this entry captures the live constraint pose instead of
    // playing stored targets.
    bool captureOnSelect = false;
};

struct PoseImportResult {
    bool fileFound = false;
    bool ok = false;
    std::string error;
    std::string sourcePath;
    Pose pose;
};

class PoseLibrary {
public:
    void rebuild(const PhysicsWorld& world, const Ragdoll& ragdoll);
    // Optional Mixamo reference. A missing file leaves the procedural poses
    // in place and returns fileFound = false.
    PoseImportResult importMixamo(const PhysicsWorld& world, const Ragdoll& ragdoll, const std::string& path);
    PoseImportResult importDefaultMixamo(const PhysicsWorld& world, const Ragdoll& ragdoll);

    const std::vector<Pose>& poses() const { return poses_; }
    const Pose* find(const std::string& name) const;
    const PoseImportResult& lastImport() const { return import_; }

    bool savePose(const Pose& pose, const std::string& path, std::string& error) const;

private:
    std::vector<Pose> poses_;
    PoseImportResult import_{};
};

}  // namespace mt
