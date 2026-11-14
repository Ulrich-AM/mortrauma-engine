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
    // Mesh-derived capsule radius, kept so the foot can be compared with the
    // v0.3.0 collider without refitting the mesh.
    float capsuleRadius = 0.06f;
    float mass = 1.f;
    BodyCreateInfo::Shape collider = BodyCreateInfo::Shape::Capsule;
    Vec3 boxHalfExtents{};
    // Collider center, sole normal, and sole points are in the rigid body's
    // local frame. The sole normal points up, away from the ground, when the
    // sole is planted. Heel and forefoot lie on the opposite face.
    Vec3 shapeCenterLocal{};
    Quat shapeRotationLocal{};
    Vec3 soleNormalLocal{0.f, 0.f, 1.f};
    Vec3 soleForwardLocal{0.f, 1.f, 0.f};
    Vec3 heelLocal{};
    Vec3 forefootLocal{};
    // Vertices within 5 mm of the fitted bottom face, and how far the 10th
    // percentile sits above that face. A large gap means one low vertex, not
    // a flat sole, defined the collider.
    int soleUndersideVertices = 0;
    float soleUndersideGapM = 0.f;
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
    // Body-local constraint frames at the imported pose. Standing placement
    // uses these so a reset can rebuild a valid chain without writing transforms
    // again while the simulation is running.
    Vec3 anchorInParent{};
    Vec3 anchorInChild{};
    Vec3 axisXInParent{1.f, 0.f, 0.f};
    Vec3 axisYInParent{0.f, 1.f, 0.f};
    Vec3 axisXInChild{1.f, 0.f, 0.f};
    Vec3 axisYInChild{0.f, 1.f, 0.f};
    Quat bindConstraint{};
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
void destroyRagdoll(PhysicsWorld& world, Ragdoll& ragdoll);
void resetRagdoll(PhysicsWorld& world, Ragdoll& ragdoll);
PosePalette posePalette(const PhysicsWorld& world, const Ragdoll& ragdoll, const ImportedModel& model);
int disconnectedComponents(const Ragdoll& ragdoll);

}  // namespace mt
