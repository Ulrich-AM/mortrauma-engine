#pragma once

#include "mortrauma/assets/Model.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/HumanoidConfig.hpp"

#include <string>
#include <vector>

namespace mt {

struct RagdollBody {
    std::string id;
    std::string name;
    int physics = -1;
    int subGroup = -1;
    std::vector<int> bones;
    int primaryBone = -1;
    Vec3 segmentStart{};
    Vec3 segmentEnd{};
    float radius = 0.f;
    float mass = 1.f;
    Vec3 initialPosition{};
    Quat initialRotation{};
    // boneBind = bodyBind * boneFromBody. Stored as body-local bone transform.
    std::vector<Mat4> boneFromBody;
};

struct RagdollJoint {
    std::string id;
    std::string name;
    int physics = -1;
    int parent = -1;
    int child = -1;
    JointConfig base;
    // Constraint angles at the imported pose, measured from the anatomical
    // neutral frame. Zero is extension for elbows, knees and hips, not the
    // authored crouch. Other joints stay centered on the imported pose, so
    // this offset is near zero for them.
    Vec3 neutralOffset{};
    Vec3 extensionAxis{};
    Vec3 flexionDirection{};
    Vec3 flexionAxis{};
};

struct Ragdoll {
    bool ok = false;
    std::string error;
    std::vector<std::string> warnings;
    std::vector<RagdollBody> bodies;
    std::vector<RagdollJoint> joints;
    int disconnectedComponents = 0;
    bool legsAttached = false;

    const RagdollBody* find(const std::string& id) const;
    int componentCount() const;
};

struct PosePalette {
    std::vector<Mat4> boneWorld;
};

class RagdollBuilder {
public:
    static Ragdoll build(PhysicsWorld& world, const CharacterConfig& config, const ImportedModel& model, const JointPreset& preset);
};

void applyPreset(PhysicsWorld& world, Ragdoll& ragdoll, const JointPreset& preset);
void resetRagdoll(PhysicsWorld& world, Ragdoll& ragdoll);
PosePalette posePalette(const PhysicsWorld& world, const Ragdoll& ragdoll, const ImportedModel& model);
int disconnectedComponents(const Ragdoll& ragdoll);

}  // namespace mt
