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
    // Geometry center in the body's local frame. The center of mass stays at
    // the body origin; this only moves the collider.
    Vec3 shapeCenter{};
    // Box axes in the body frame. Identity keeps the box aligned with the body.
    Quat shapeRotation{};
    // Edge rounding for boxes. The outer size stays halfExtents.
    float convexRadius = 0.05f;
    float mass = 1.f;
    float friction = 0.7f;
    float restitution = 0.02f;
    float linearDamping = 0.05f;
    float angularDamping = 0.15f;
    bool isStatic = false;
    bool isKinematic = false;
    bool collides = true;
    // Kinematic grab anchors must not sleep. A sleeping body that is still
    // attached to a constraint is not a valid island member.
    bool allowSleeping = true;
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
    // Constraint frames in each body's local space. Translation is fixed.
    // Elbows, knees and hips use anatomical frames, so the imported pose is
    // not the zero rotation. Other joints are centered on that pose.
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
    // Passive resistance. Constraint friction torque, independent of the
    // active motor. Friction remains when the motor is switched off.
    float angularFriction = 2.f;
    // Initial motor settings copied onto the constraint. The motor is created
    // off; MotorController is what enables it.
    float motorFrequency = 0.f;
    float motorDamping = 1.f;
    float motorTorqueLimit = 80.f;
};

// Angular position drive, in the constraint frame of body 1 (the same
// quaternion as JointState::constraintRotation). This is not a world pose.
struct AngularMotorCommand {
    bool enabled = false;
    Quat targetOrientation{};
    // Stored for velocity mode. The position drive used by M2 does not treat
    // this as motor strength.
    Vec3 targetAngularVelocity{};
    float frequencyHz = 0.f;
    float dampingRatio = 1.f;
    // Symmetric limit in N·m. Zero disables the motor.
    float torqueLimit = 0.f;
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
    Vec3 swingAxisWorld{};
    Vec3 bendAxisWorld{};
    Vec3 childTwistAxisWorld{};
    Vec3 childSwingAxisWorld{};
    Vec3 childBendAxisWorld{};
    bool twistAtLimit = false;
    bool swing1AtLimit = false;
    bool swing2AtLimit = false;
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
    // Relative orientation of the constraint, body-1 constraint space.
    Quat constraintRotation{};
    Quat motorTarget{};
    // Motor impulse from the last step divided by the timestep, along the
    // three rotational constraint axes. This is an estimate of torque (N·m),
    // not a separate sensor. Zero while the motor is off. Friction impulse
    // is not included.
    Vec3 motorTorque{};
    // Child angular velocity minus parent, projected on the parent constraint
    // axes (rad/s). Jolt does not expose a joint angular velocity.
    Vec3 relativeAngularVelocity{};
    // True when any axis of motorTorque is at least 90% of the torque limit.
    bool motorSaturated = false;
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
    Vec3 centerOffset{};
    Quat shapeRotation{};
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
    // Replaces a dynamic body's collider and inertia. Mass is unchanged.
    bool setBodyShape(int body, const BodyCreateInfo& info);
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
    // The only physics-side writer for angular motors. MotorController is the
    // only caller that decides targets. Targets are constraint-space.
    bool setAngularMotor(int joint, const AngularMotorCommand& command);
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
    bool grabAlive(int grab) const;
    int grabAnchor(int grab) const;

    // Laboratory fixture. A kinematic anchor holds the body with a soft
    // positional spring and leaves rotation free. It is not a balance
    // controller. clearGrabs does not remove it.
    bool beginSupport(int body, float frequency, float damping);
    void endSupport();
    bool supportActive() const;
    int supportAnchor() const;

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
