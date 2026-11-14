#pragma once

#include "mortrauma/core/Math.hpp"

#include <string>
#include <vector>

namespace mt {

struct PhysicsSettings {
    Vec3 gravity{0.f, -9.81f, 0.f};
    int velocityIterations = 12;
    int positionIterations = 3;
    int collisionSteps = 1;
    float fixedTimestep = 1.f / 120.f;
    bool allowSleeping = true;
};

struct BodyCreateInfo {
    std::string name;
    Vec3 position{0.f, 0.f, 0.f};
    Quat rotation{};
    // Capsule is centered on the body and aligned with local +Y.
    // radius <= 0 selects a box described by halfExtents.
    enum class Shape { Capsule, Sphere, Box } shape = Shape::Capsule;
    float radius = 0.08f;
    float halfHeight = 0.1f;  // cylindrical half-height for capsules
    Vec3 halfExtents{0.1f, 0.1f, 0.1f};
    float mass = 1.f;
    float friction = 0.7f;
    float restitution = 0.02f;
    float linearDamping = 0.05f;
    float angularDamping = 0.15f;
    bool isStatic = false;
    bool isKinematic = false;
    bool collides = true;
    int collisionSubGroup = -1;  // >= 0 joins the ragdoll filter group
};

struct BodyState {
    bool valid = false;
    std::string name;
    Vec3 position{};
    Quat rotation{};
    Vec3 linearVelocity{};
    Vec3 angularVelocity{};
    float mass = 0.f;
    float friction = 0.f;
    float restitution = 0.f;
    float linearDamping = 0.f;
    float angularDamping = 0.f;
    bool isStatic = false;
    bool isDynamic = false;
    bool isKinematic = false;
};

enum class SwingType { Cone, Pyramid };

struct JointCreateInfo {
    std::string name;
    int parent = -1;
    int child = -1;
    // Constraint frames in each body's local space. Frames coincide in the
    // bind pose so the relative angle is zero. Translation is fixed.
    Vec3 anchorInParent{};
    Vec3 axisXInParent{1.f, 0.f, 0.f};
    Vec3 axisYInParent{0.f, 1.f, 0.f};
    Vec3 anchorInChild{};
    Vec3 axisXInChild{1.f, 0.f, 0.f};
    Vec3 axisYInChild{0.f, 1.f, 0.f};
    SwingType swing = SwingType::Pyramid;
    // Radians. RotationX is twist. RotationY/Z are the swing axes.
    float twistMin = 0.f;
    float twistMax = 0.f;
    float swing1Min = 0.f;
    float swing1Max = 0.f;
    float swing2Min = 0.f;
    float swing2Max = 0.f;
    // Passive resistance. This is constraint friction torque, not a motor
    // pulling toward the bind pose. Motors stay off in v0.1.
    float angularFriction = 2.f;
    // Stored for a future torque-limited motor. Not enabled.
    float motorFrequency = 0.f;
    float motorDamping = 1.f;
    float motorTorqueLimit = 80.f;
};

struct JointState {
    bool valid = false;
    std::string name;
    int parent = -1;
    int child = -1;
    std::string parentName;
    std::string childName;
    std::string type;
    Vec3 anchorParentWorld{};
    Vec3 anchorChildWorld{};
    float separation = 0.f;
    Vec3 twistAxisWorld{};
    Vec3 bendAxisWorld{};
    // Relative rotation in constraint space (radians, XYZ = twist, swing1, swing2).
    Vec3 rotationRadians{};
    float swingAngle = 0.f;
    float twistMin = 0.f;
    float twistMax = 0.f;
    float swing1Min = 0.f;
    float swing1Max = 0.f;
    float swing2Min = 0.f;
    float swing2Max = 0.f;
    float angularFriction = 0.f;
    float motorFrequency = 0.f;
    float motorDamping = 0.f;
    float motorTorqueLimit = 0.f;
    bool motorEnabled = false;
    bool positionLocked = false;
    bool withinLimits = false;
    std::string status;
};

struct ContactRecord {
    Vec3 point{};
    Vec3 normal{};
    float depth = 0.f;
    int bodyA = -1;
    int bodyB = -1;
};

struct RayHit {
    bool hit = false;
    int body = -1;
    Vec3 point{};
    Vec3 normal{};
    float fraction = 1.f;
};

struct ShapeView {
    int body = -1;
    BodyCreateInfo::Shape shape = BodyCreateInfo::Shape::Box;
    Vec3 position{};
    Quat rotation{};
    float radius = 0.f;
    float halfHeight = 0.f;
    Vec3 halfExtents{};
};

// Jolt adapter. The public surface is creation, deletion, configuration and
// queries. Dear ImGui is not visible from this module.
class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    bool initialize(const PhysicsSettings& settings);
    void shutdown();
    bool isReady() const { return ready_; }

    int createBody(const BodyCreateInfo& info);
    void destroyBody(int body);
    int createJoint(const JointCreateInfo& info);
    void destroyJoint(int joint);

    // One fixed step. Re-entrant calls are rejected so a frame cannot step twice.
    bool step(float dt, int collisionSteps);

    void setGravity(Vec3 gravity);
    Vec3 gravity() const { return settings_.gravity; }
    void setSolverIterations(int velocitySteps, int positionSteps);
    int velocityIterations() const { return settings_.velocityIterations; }
    int positionIterations() const { return settings_.positionIterations; }

    // Ragdoll bodies that share a collision group. Pairs are enabled by default.
    void configureRagdollFilter(int subGroupCount);
    void setPairCollision(int subGroupA, int subGroupB, bool enabled);

    int bodyCount() const;
    int jointCount() const;
    int activeContactCount() const { return static_cast<int>(contacts_.size()); }

    BodyState bodyState(int body) const;
    bool setBodyPose(int body, Vec3 position, Quat rotation, bool wake);
    bool setBodyVelocity(int body, Vec3 linear, Vec3 angular);
    bool setBodyMass(int body, float mass);
    bool setBodyFriction(int body, float friction);
    bool setBodyRestitution(int body, float restitution);
    bool setBodyDamping(int body, float linear, float angular);
    void addImpulse(int body, Vec3 impulse, Vec3 worldPoint);

    JointState jointState(int joint) const;
    bool setJointAngularFriction(int joint, float friction);
    bool setJointLimits(int joint, float twistMin, float twistMax, float swing1Min, float swing1Max, float swing2Min,
                        float swing2Max);

    RayHit rayCast(Vec3 origin, Vec3 direction, float maxDistance, int ignoreBody = -1) const;
    std::vector<ContactRecord> contacts() const { return contacts_; }
    ShapeView shapeView(int body) const;

    // Spring grab. A kinematic anchor is tied to the body with a soft
    // positional constraint. Rotation of the grabbed body stays free.
    int beginGrab(int body, Vec3 bodyPointWorld, Vec3 targetWorld, float frequency, float damping);
    void updateGrab(int grab, Vec3 targetWorld);
    void endGrab(int grab);
    void clearGrabs();

    const PhysicsSettings& settings() const { return settings_; }

private:
    struct Impl;
    Impl* impl_ = nullptr;
    PhysicsSettings settings_{};
    bool ready_ = false;
    bool stepping_ = false;
    std::vector<ContactRecord> contacts_;
};

}  // namespace mt
