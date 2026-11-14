#include "mortrauma/physics/PhysicsWorld.hpp"

#include "mortrauma/core/Log.hpp"

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <cmath>
#include <cstdarg>
#include <mutex>
#include <unordered_map>

namespace mt {
namespace {

using namespace JPH;

constexpr uint kRagdollGroup = 1;

mt::Vec3 toMt(Vec3Arg v) { return {v.GetX(), v.GetY(), v.GetZ()}; }
JPH::Quat toJolt(mt::Quat q) { return JPH::Quat(q.x, q.y, q.z, q.w); }
mt::Quat toMt(JPH::Quat q) { return {q.GetX(), q.GetY(), q.GetZ(), q.GetW()}; }
RVec3 toR(mt::Vec3 v) { return RVec3(v.x, v.y, v.z); }
JPH::Vec3 toV(mt::Vec3 v) { return JPH::Vec3(v.x, v.y, v.z); }

void traceImpl(const char* fmt, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    Log::info(std::string("Jolt: ") + buffer);
}

#ifdef JPH_ENABLE_ASSERTS
bool assertImpl(const char* expression, const char* message, const char* file, uint line) {
    Log::error(std::string("Jolt assert ") + expression + " " + (message ? message : "") + " at " + file + ":" +
               std::to_string(line));
    return true;
}
#endif

class ContactSink final : public ContactListener {
public:
    std::mutex mutex;
    std::vector<ContactRecord>* output = nullptr;
    std::unordered_map<uint32_t, int> idToIndex;

    void push(const Body& bodyA, const Body& bodyB, const ContactManifold& manifold) {
        if (output == nullptr) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        if (output->size() > 1024) {
            return;
        }
        const auto findIndex = [&](const Body& body) {
            const auto it = idToIndex.find(body.GetID().GetIndexAndSequenceNumber());
            return it == idToIndex.end() ? -1 : it->second;
        };
        const int indexA = findIndex(bodyA);
        const int indexB = findIndex(bodyB);
        const Vec3 normal = toMt(manifold.mWorldSpaceNormal);
        const int count = int(manifold.mRelativeContactPointsOn1.size());
        for (int i = 0; i < count; ++i) {
            ContactRecord record;
            record.point = toMt(manifold.GetWorldSpaceContactPointOn1(uint(i)));
            record.normal = normal;
            record.depth = manifold.mPenetrationDepth;
            record.bodyA = indexA;
            record.bodyB = indexB;
            output->push_back(record);
        }
    }

    void OnContactAdded(const Body& bodyA, const Body& bodyB, const ContactManifold& manifold, ContactSettings&) override {
        push(bodyA, bodyB, manifold);
    }
    void OnContactPersisted(const Body& bodyA, const Body& bodyB, const ContactManifold& manifold,
                            ContactSettings&) override {
        push(bodyA, bodyB, manifold);
    }
};

class IgnoreBodyFilter final : public BodyFilter {
public:
    BodyID ignore;
    explicit IgnoreBodyFilter(BodyID id) : ignore(id) {}
    bool ShouldCollide(const BodyID& id) const override { return id != ignore; }
    bool ShouldCollideLocked(const Body& body) const override { return body.GetID() != ignore; }
};

namespace Layers {
constexpr ObjectLayer kNonMoving = 0;
constexpr ObjectLayer kMoving = 1;
constexpr ObjectLayer kNoCollide = 2;
constexpr ObjectLayer kCount = 3;
}  // namespace Layers

namespace BroadLayers {
constexpr BroadPhaseLayer kNonMoving(0);
constexpr BroadPhaseLayer kMoving(1);
constexpr uint kCount = 2;
}  // namespace BroadLayers

class BroadPhaseMap final : public BroadPhaseLayerInterface {
public:
    BroadPhaseMap() {
        map_[Layers::kNonMoving] = BroadLayers::kNonMoving;
        map_[Layers::kMoving] = BroadLayers::kMoving;
        map_[Layers::kNoCollide] = BroadLayers::kMoving;
    }
    uint GetNumBroadPhaseLayers() const override { return BroadLayers::kCount; }
    BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer layer) const override { return map_[layer]; }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(BroadPhaseLayer) const override { return "layer"; }
#endif
private:
    BroadPhaseLayer map_[Layers::kCount]{};
};

class ObjectVsBroadPhase final : public ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(ObjectLayer layer, BroadPhaseLayer broad) const override {
        if (layer == Layers::kNoCollide) {
            return false;
        }
        if (layer == Layers::kNonMoving) {
            return broad == BroadLayers::kMoving;
        }
        return true;
    }
};

class ObjectPairFilter final : public ObjectLayerPairFilter {
public:
    bool ShouldCollide(ObjectLayer a, ObjectLayer b) const override {
        if (a == Layers::kNoCollide || b == Layers::kNoCollide) {
            return false;
        }
        if (a == Layers::kNonMoving && b == Layers::kNonMoving) {
            return false;
        }
        return true;
    }
};

int gJoltUsers = 0;

struct BodyRecord {
    bool alive = false;
    std::string name;
    BodyID id;
    BodyCreateInfo shape;
    int subGroup = -1;
};

struct JointRecord {
    bool alive = false;
    JointCreateInfo info;
    Ref<SixDOFConstraint> constraint;
    bool grab = false;
    bool fixture = false;
};

struct GrabRecord {
    bool alive = false;
    int body = -1;
    int anchorBody = -1;
    int joint = -1;
};

}  // namespace

struct PhysicsWorld::Impl {
    BroadPhaseMap broadPhase;
    ObjectVsBroadPhase objectVsBroad;
    ObjectPairFilter objectPairs;
    ContactSink contacts;
    TempAllocatorImpl* temp = nullptr;
    JobSystemSingleThreaded* jobs = nullptr;
    PhysicsSystem system;
    Ref<GroupFilterTable> filter;
    int filterGroups = 0;
    std::vector<BodyRecord> bodies;
    std::vector<JointRecord> joints;
    std::vector<GrabRecord> grabs;
    GrabRecord support;
    float lastDt = 1.f / 120.f;

    BodyInterface& bodiesApi() { return system.GetBodyInterface(); }

    void remember(const BodyRecord& record, int index) {
        contacts.idToIndex[record.id.GetIndexAndSequenceNumber()] = index;
    }
};

PhysicsWorld::PhysicsWorld() = default;

PhysicsWorld::~PhysicsWorld() { shutdown(); }

bool PhysicsWorld::initialize(const PhysicsSettings& settings) {
    if (ready_) {
        return true;
    }
    settings_ = settings;
    if (gJoltUsers == 0) {
        JPH::RegisterDefaultAllocator();
        JPH::Trace = traceImpl;
        JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = assertImpl;)
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }
    ++gJoltUsers;

    impl_ = new Impl();
    impl_->temp = new JPH::TempAllocatorImpl(16 * 1024 * 1024);
    impl_->jobs = new JPH::JobSystemSingleThreaded(JPH::cMaxPhysicsJobs);
    impl_->system.Init(1024, 0, 65536, 10240, impl_->broadPhase, impl_->objectVsBroad, impl_->objectPairs);
    impl_->contacts.output = &contacts_;
    impl_->system.SetContactListener(&impl_->contacts);
    setGravity(settings.gravity);
    setSolverIterations(settings.velocityIterations, settings.positionIterations);
    JPH::PhysicsSettings physics = impl_->system.GetPhysicsSettings();
    physics.mAllowSleeping = settings.allowSleeping;
    impl_->system.SetPhysicsSettings(physics);
    ready_ = true;
    Log::info("Physics world initialized (fixed step " + std::to_string(settings.fixedTimestep) + " s).");
    return true;
}

void PhysicsWorld::shutdown() {
    if (!ready_ && impl_ == nullptr) {
        return;
    }
    if (impl_ != nullptr) {
        clearGrabs();
        impl_->system.SetContactListener(nullptr);
        for (auto& joint : impl_->joints) {
            if (joint.alive && joint.constraint != nullptr) {
                impl_->system.RemoveConstraint(joint.constraint);
            }
        }
        for (auto& body : impl_->bodies) {
            if (body.alive) {
                impl_->bodiesApi().RemoveBody(body.id);
                impl_->bodiesApi().DestroyBody(body.id);
            }
        }
        delete impl_->jobs;
        delete impl_->temp;
        delete impl_;
        impl_ = nullptr;
    }
    ready_ = false;
    if (gJoltUsers > 0) {
        --gJoltUsers;
        if (gJoltUsers == 0) {
            JPH::UnregisterTypes();
            delete JPH::Factory::sInstance;
            JPH::Factory::sInstance = nullptr;
        }
    }
}

int PhysicsWorld::createBody(const BodyCreateInfo& info) {
    if (!ready_) {
        Log::error("createBody called before physics initialization.");
        return -1;
    }
    JPH::Ref<JPH::Shape> shape;
    if (info.shape == BodyCreateInfo::Shape::Sphere || (info.shape == BodyCreateInfo::Shape::Capsule && info.halfHeight <= 0.001f)) {
        const float radius = std::max(info.radius, 0.02f);
        shape = new JPH::SphereShape(radius);
    } else if (info.shape == BodyCreateInfo::Shape::Capsule) {
        const float radius = std::max(info.radius, 0.02f);
        const float halfHeight = std::max(info.halfHeight, 0.01f);
        shape = new JPH::CapsuleShape(halfHeight, radius);
    } else {
        const JPH::Vec3 half(std::max(info.halfExtents.x, 0.01f), std::max(info.halfExtents.y, 0.01f),
                             std::max(info.halfExtents.z, 0.01f));
        shape = new JPH::BoxShape(half);
    }

    const JPH::EMotionType motion =
        info.isStatic ? JPH::EMotionType::Static : (info.isKinematic ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic);
    JPH::ObjectLayer layer = Layers::kMoving;
    if (info.isStatic) {
        layer = Layers::kNonMoving;
    }
    if (!info.collides) {
        layer = Layers::kNoCollide;
    }

    JPH::BodyCreationSettings settings(shape, toR(info.position), toJolt(info.rotation), motion, layer);
    if (!info.isStatic) {
        settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = std::max(info.mass, 0.05f);
        settings.mLinearDamping = clampf(info.linearDamping, 0.f, 0.95f);
        settings.mAngularDamping = clampf(info.angularDamping, 0.f, 0.95f);
        settings.mMotionQuality = JPH::EMotionQuality::LinearCast;
    }
    settings.mFriction = info.friction;
    settings.mRestitution = info.restitution;
    settings.mAllowSleeping = settings_.allowSleeping;

    JPH::Body* created = impl_->bodiesApi().CreateBody(settings);
    if (created == nullptr) {
        Log::error("Jolt rejected body '" + info.name + "'.");
        return -1;
    }
    impl_->bodiesApi().AddBody(created->GetID(), JPH::EActivation::Activate);

    BodyRecord record;
    record.alive = true;
    record.name = info.name;
    record.id = created->GetID();
    record.shape = info;
    record.shape.radius = info.shape == BodyCreateInfo::Shape::Box ? 0.f : (info.radius > 0.f ? info.radius : 0.05f);
    record.subGroup = info.collisionSubGroup;
    const int index = int(impl_->bodies.size());
    impl_->bodies.push_back(record);
    impl_->remember(record, index);

    if (info.collisionSubGroup >= 0 && impl_->filter != nullptr) {
        JPH::BodyLockWrite lock(impl_->system.GetBodyLockInterface(), record.id);
        if (lock.Succeeded()) {
            lock.GetBody().SetCollisionGroup(
                JPH::CollisionGroup(impl_->filter, kRagdollGroup, JPH::CollisionGroup::SubGroupID(info.collisionSubGroup)));
        }
    }
    return index;
}

void PhysicsWorld::destroyBody(int body) {
    if (!ready_ || body < 0 || body >= int(impl_->bodies.size()) || !impl_->bodies[body].alive) {
        return;
    }
    for (int i = 0; i < int(impl_->joints.size()); ++i) {
        if (!impl_->joints[i].alive) {
            continue;
        }
        if (impl_->joints[i].info.parent == body || impl_->joints[i].info.child == body) {
            destroyJoint(i);
        }
    }
    auto& record = impl_->bodies[body];
    impl_->contacts.idToIndex.erase(record.id.GetIndexAndSequenceNumber());
    impl_->bodiesApi().RemoveBody(record.id);
    impl_->bodiesApi().DestroyBody(record.id);
    record.alive = false;
}

int PhysicsWorld::createJoint(const JointCreateInfo& info) {
    if (!ready_) {
        Log::error("createJoint called before physics initialization.");
        return -1;
    }
    if (info.parent < 0 || info.child < 0 || info.parent >= int(impl_->bodies.size()) ||
        info.child >= int(impl_->bodies.size()) || !impl_->bodies[info.parent].alive || !impl_->bodies[info.child].alive) {
        Log::error("Joint '" + info.name + "' references a missing body.");
        return -1;
    }

    JPH::SixDOFConstraintSettings settings;
    settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
    settings.mPosition1 = toV(info.anchorInParent);
    settings.mAxisX1 = toV(normalize(info.axisXInParent));
    settings.mAxisY1 = toV(normalize(info.axisYInParent));
    settings.mPosition2 = toV(info.anchorInChild);
    settings.mAxisX2 = toV(normalize(info.axisXInChild));
    settings.mAxisY2 = toV(normalize(info.axisYInChild));
    settings.MakeFixedAxis(JPH::SixDOFConstraintSettings::TranslationX);
    settings.MakeFixedAxis(JPH::SixDOFConstraintSettings::TranslationY);
    settings.MakeFixedAxis(JPH::SixDOFConstraintSettings::TranslationZ);
    settings.mSwingType = info.swing == SwingType::Cone ? JPH::ESwingType::Cone : JPH::ESwingType::Pyramid;
    settings.SetLimitedAxis(JPH::SixDOFConstraintSettings::RotationX, info.twistMin, info.twistMax);
    settings.SetLimitedAxis(JPH::SixDOFConstraintSettings::RotationY, info.swing1Min, info.swing1Max);
    settings.SetLimitedAxis(JPH::SixDOFConstraintSettings::RotationZ, info.swing2Min, info.swing2Max);
    const float friction = std::max(info.angularFriction, 0.f);
    settings.mMaxFriction[JPH::SixDOFConstraintSettings::RotationX] = friction;
    settings.mMaxFriction[JPH::SixDOFConstraintSettings::RotationY] = friction;
    settings.mMaxFriction[JPH::SixDOFConstraintSettings::RotationZ] = friction;
    for (int axis = JPH::SixDOFConstraintSettings::RotationX; axis < JPH::SixDOFConstraintSettings::Num; ++axis) {
        auto& motor = settings.mMotorSettings[axis];
        motor.mSpringSettings.mMode = JPH::ESpringMode::FrequencyAndDamping;
        motor.mSpringSettings.mFrequency = std::max(info.motorFrequency, 0.f);
        motor.mSpringSettings.mDamping = std::max(info.motorDamping, 0.f);
        motor.SetTorqueLimit(std::max(info.motorTorqueLimit, 0.f));
    }

    JPH::BodyLockWrite lockA(impl_->system.GetBodyLockInterface(), impl_->bodies[info.parent].id);
    JPH::BodyLockWrite lockB(impl_->system.GetBodyLockInterface(), impl_->bodies[info.child].id);
    if (!lockA.Succeeded() || !lockB.Succeeded()) {
        Log::error("Could not lock bodies for joint '" + info.name + "'.");
        return -1;
    }
    JPH::Ref<JPH::TwoBodyConstraint> created = settings.Create(lockA.GetBody(), lockB.GetBody());
    impl_->system.AddConstraint(created);

    JointRecord record;
    record.alive = true;
    record.info = info;
    record.constraint = static_cast<JPH::SixDOFConstraint*>(created.GetPtr());
    // Created off. MotorController enables the position drive later.
    record.constraint->SetMotorState(JPH::SixDOFConstraintSettings::RotationX, JPH::EMotorState::Off);
    record.constraint->SetMotorState(JPH::SixDOFConstraintSettings::RotationY, JPH::EMotorState::Off);
    record.constraint->SetMotorState(JPH::SixDOFConstraintSettings::RotationZ, JPH::EMotorState::Off);
    const int index = int(impl_->joints.size());
    impl_->joints.push_back(record);
    return index;
}

void PhysicsWorld::destroyJoint(int joint) {
    if (!ready_ || joint < 0 || joint >= int(impl_->joints.size()) || !impl_->joints[joint].alive) {
        return;
    }
    auto& record = impl_->joints[joint];
    if (record.constraint != nullptr) {
        impl_->system.RemoveConstraint(record.constraint);
    }
    record.alive = false;
    record.constraint = nullptr;
}

bool PhysicsWorld::step(float dt, int collisionSteps) {
    if (!ready_) {
        Log::error("Physics step requested before initialization.");
        return false;
    }
    if (stepping_) {
        Log::error("Duplicate physics step rejected.");
        return false;
    }
    if (!(dt > 0.f) || !std::isfinite(dt)) {
        Log::error("Rejected non-finite physics timestep.");
        return false;
    }
    stepping_ = true;
    contacts_.clear();
    impl_->lastDt = dt;
    const int steps = std::max(1, collisionSteps);
    impl_->system.Update(dt, steps, impl_->temp, impl_->jobs);
    stepping_ = false;
    return true;
}

void PhysicsWorld::setGravity(Vec3 gravity) {
    settings_.gravity = gravity;
    if (ready_) {
        impl_->system.SetGravity(toV(gravity));
    }
}

void PhysicsWorld::setSolverIterations(int velocitySteps, int positionSteps) {
    settings_.velocityIterations = std::max(1, velocitySteps);
    settings_.positionIterations = std::max(1, positionSteps);
    if (!ready_) {
        return;
    }
    JPH::PhysicsSettings physics = impl_->system.GetPhysicsSettings();
    physics.mNumVelocitySteps = uint(settings_.velocityIterations);
    physics.mNumPositionSteps = uint(settings_.positionIterations);
    impl_->system.SetPhysicsSettings(physics);
}

void PhysicsWorld::configureRagdollFilter(int subGroupCount) {
    if (!ready_ || subGroupCount <= 1) {
        return;
    }
    impl_->filter = new JPH::GroupFilterTable(uint(subGroupCount));
    impl_->filterGroups = subGroupCount;
    for (auto& body : impl_->bodies) {
        if (!body.alive || body.subGroup < 0 || body.subGroup >= subGroupCount) {
            continue;
        }
        JPH::BodyLockWrite lock(impl_->system.GetBodyLockInterface(), body.id);
        if (lock.Succeeded()) {
            lock.GetBody().SetCollisionGroup(
                JPH::CollisionGroup(impl_->filter, kRagdollGroup, JPH::CollisionGroup::SubGroupID(body.subGroup)));
        }
    }
}

void PhysicsWorld::setPairCollision(int subGroupA, int subGroupB, bool enabled) {
    if (!ready_ || impl_->filter == nullptr || subGroupA == subGroupB) {
        return;
    }
    if (subGroupA < 0 || subGroupB < 0 || subGroupA >= impl_->filterGroups || subGroupB >= impl_->filterGroups) {
        return;
    }
    if (enabled) {
        impl_->filter->EnableCollision(JPH::CollisionGroup::SubGroupID(subGroupA), JPH::CollisionGroup::SubGroupID(subGroupB));
    } else {
        impl_->filter->DisableCollision(JPH::CollisionGroup::SubGroupID(subGroupA), JPH::CollisionGroup::SubGroupID(subGroupB));
    }
}

int PhysicsWorld::bodyCount() const {
    if (!ready_) {
        return 0;
    }
    int count = 0;
    for (const auto& body : impl_->bodies) {
        count += body.alive ? 1 : 0;
    }
    return count;
}

int PhysicsWorld::jointCount() const {
    if (!ready_) {
        return 0;
    }
    int count = 0;
    for (const auto& joint : impl_->joints) {
        if (joint.alive && !joint.grab) {
            ++count;
        }
    }
    return count;
}

BodyState PhysicsWorld::bodyState(int body) const {
    BodyState state;
    if (!ready_ || body < 0 || body >= int(impl_->bodies.size()) || !impl_->bodies[body].alive) {
        return state;
    }
    const auto& record = impl_->bodies[body];
    state.valid = true;
    state.name = record.name;
    state.position = toMt(impl_->bodiesApi().GetPosition(record.id));
    state.rotation = toMt(impl_->bodiesApi().GetRotation(record.id));
    state.linearVelocity = toMt(impl_->bodiesApi().GetLinearVelocity(record.id));
    state.angularVelocity = toMt(impl_->bodiesApi().GetAngularVelocity(record.id));
    state.friction = impl_->bodiesApi().GetFriction(record.id);
    state.restitution = impl_->bodiesApi().GetRestitution(record.id);
    JPH::BodyLockRead lock(impl_->system.GetBodyLockInterface(), record.id);
    if (lock.Succeeded()) {
        const JPH::MotionProperties* motion = lock.GetBody().GetMotionProperties();
        state.isStatic = lock.GetBody().IsStatic();
        state.isKinematic = lock.GetBody().IsKinematic();
        state.isDynamic = lock.GetBody().IsDynamic();
        if (motion != nullptr) {
            state.mass = motion->GetInverseMass() > 0.f ? 1.f / motion->GetInverseMass() : 0.f;
            state.linearDamping = motion->GetLinearDamping();
            state.angularDamping = motion->GetAngularDamping();
        }
    }
    return state;
}

bool PhysicsWorld::setBodyPose(int body, Vec3 position, Quat rotation, bool wake) {
    if (!ready_ || body < 0 || body >= int(impl_->bodies.size()) || !impl_->bodies[body].alive) {
        return false;
    }
    impl_->bodiesApi().SetPositionAndRotation(impl_->bodies[body].id, toR(position), toJolt(rotation),
                                              wake ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
    return true;
}

bool PhysicsWorld::setBodyVelocity(int body, Vec3 linear, Vec3 angular) {
    if (!ready_ || body < 0 || body >= int(impl_->bodies.size()) || !impl_->bodies[body].alive) {
        return false;
    }
    impl_->bodiesApi().SetLinearAndAngularVelocity(impl_->bodies[body].id, toV(linear), toV(angular));
    return true;
}

bool PhysicsWorld::setBodyMass(int body, float mass) {
    if (!ready_ || body < 0 || body >= int(impl_->bodies.size()) || !impl_->bodies[body].alive) {
        return false;
    }
    if (!(mass > 0.f)) {
        Log::error("Rejected non-positive mass for '" + impl_->bodies[body].name + "'.");
        return false;
    }
    JPH::BodyLockWrite lock(impl_->system.GetBodyLockInterface(), impl_->bodies[body].id);
    if (!lock.Succeeded() || lock.GetBody().GetMotionProperties() == nullptr) {
        return false;
    }
    JPH::MassProperties properties = lock.GetBody().GetShape()->GetMassProperties();
    properties.ScaleToMass(mass);
    lock.GetBody().GetMotionProperties()->SetMassProperties(JPH::EAllowedDOFs::All, properties);
    impl_->bodies[body].shape.mass = mass;
    return true;
}

bool PhysicsWorld::setBodyFriction(int body, float friction) {
    if (!ready_ || body < 0 || body >= int(impl_->bodies.size()) || !impl_->bodies[body].alive) {
        return false;
    }
    impl_->bodiesApi().SetFriction(impl_->bodies[body].id, std::max(friction, 0.f));
    return true;
}

bool PhysicsWorld::setBodyRestitution(int body, float restitution) {
    if (!ready_ || body < 0 || body >= int(impl_->bodies.size()) || !impl_->bodies[body].alive) {
        return false;
    }
    impl_->bodiesApi().SetRestitution(impl_->bodies[body].id, clampf(restitution, 0.f, 1.f));
    return true;
}

bool PhysicsWorld::setBodyDamping(int body, float linear, float angular) {
    if (!ready_ || body < 0 || body >= int(impl_->bodies.size()) || !impl_->bodies[body].alive) {
        return false;
    }
    JPH::BodyLockWrite lock(impl_->system.GetBodyLockInterface(), impl_->bodies[body].id);
    if (!lock.Succeeded() || lock.GetBody().GetMotionProperties() == nullptr) {
        return false;
    }
    lock.GetBody().GetMotionProperties()->SetLinearDamping(clampf(linear, 0.f, 0.95f));
    lock.GetBody().GetMotionProperties()->SetAngularDamping(clampf(angular, 0.f, 0.95f));
    return true;
}

void PhysicsWorld::addImpulse(int body, Vec3 impulse, Vec3 worldPoint) {
    if (!ready_ || body < 0 || body >= int(impl_->bodies.size()) || !impl_->bodies[body].alive) {
        return;
    }
    impl_->bodiesApi().AddImpulse(impl_->bodies[body].id, toV(impulse), toR(worldPoint));
}

JointState PhysicsWorld::jointState(int joint) const {
    JointState state;
    if (!ready_ || joint < 0 || joint >= int(impl_->joints.size()) || !impl_->joints[joint].alive) {
        return state;
    }
    const auto& record = impl_->joints[joint];
    state.valid = true;
    state.name = record.info.name;
    state.parent = record.info.parent;
    state.child = record.info.child;
    state.parentName = impl_->bodies[record.info.parent].name;
    state.childName = impl_->bodies[record.info.child].name;
    if (record.fixture) {
        state.type = "Laboratory support fixture";
    } else if (record.grab) {
        state.type = "Grab spring";
    } else {
        state.type = record.info.swing == SwingType::Cone ? "SixDOF cone" : "SixDOF pyramid";
    }
    state.twistMin = record.constraint->GetLimitsMin(JPH::SixDOFConstraintSettings::RotationX);
    state.twistMax = record.constraint->GetLimitsMax(JPH::SixDOFConstraintSettings::RotationX);
    state.swing1Min = record.constraint->GetLimitsMin(JPH::SixDOFConstraintSettings::RotationY);
    state.swing1Max = record.constraint->GetLimitsMax(JPH::SixDOFConstraintSettings::RotationY);
    state.swing2Min = record.constraint->GetLimitsMin(JPH::SixDOFConstraintSettings::RotationZ);
    state.swing2Max = record.constraint->GetLimitsMax(JPH::SixDOFConstraintSettings::RotationZ);
    state.angularFriction = record.constraint->GetMaxFriction(JPH::SixDOFConstraintSettings::RotationX);
    const auto& motorSettings = record.constraint->GetMotorSettings(JPH::SixDOFConstraintSettings::RotationX);
    state.motorFrequency = motorSettings.mSpringSettings.mFrequency;
    state.motorDamping = motorSettings.mSpringSettings.mDamping;
    state.motorTorqueLimit = motorSettings.mMaxTorqueLimit;
    const bool motorX = record.constraint->GetMotorState(JPH::SixDOFConstraintSettings::RotationX) != JPH::EMotorState::Off;
    const bool motorY = record.constraint->GetMotorState(JPH::SixDOFConstraintSettings::RotationY) != JPH::EMotorState::Off;
    const bool motorZ = record.constraint->GetMotorState(JPH::SixDOFConstraintSettings::RotationZ) != JPH::EMotorState::Off;
    state.motorEnabled = motorX || motorY || motorZ;
    state.motorTarget = toMt(record.constraint->GetTargetOrientationCS());
    state.positionLocked = record.constraint->IsFixedAxis(JPH::SixDOFConstraintSettings::TranslationX) &&
                           record.constraint->IsFixedAxis(JPH::SixDOFConstraintSettings::TranslationY) &&
                           record.constraint->IsFixedAxis(JPH::SixDOFConstraintSettings::TranslationZ);

    const BodyState parent = bodyState(record.info.parent);
    const BodyState child = bodyState(record.info.child);
    state.anchorParentWorld = parent.position + rotate(parent.rotation, record.info.anchorInParent);
    state.anchorChildWorld = child.position + rotate(child.rotation, record.info.anchorInChild);
    state.separation = length(state.anchorParentWorld - state.anchorChildWorld);
    state.twistAxisWorld = rotate(parent.rotation, normalize(record.info.axisXInParent));
    state.swingAxisWorld = rotate(parent.rotation, normalize(record.info.axisYInParent));
    state.bendAxisWorld = cross(state.twistAxisWorld, state.swingAxisWorld);
    state.childTwistAxisWorld = rotate(child.rotation, normalize(record.info.axisXInChild));
    state.childSwingAxisWorld = rotate(child.rotation, normalize(record.info.axisYInChild));
    state.childBendAxisWorld = cross(state.childTwistAxisWorld, state.childSwingAxisWorld);

    const mt::Quat relative = toMt(record.constraint->GetRotationInConstraintSpace());
    state.constraintRotation = relative;
    const Vec3 twisted = rotate(relative, Vec3{1.f, 0.f, 0.f});
    state.swingAngle = std::acos(clampf(twisted.x, -1.f, 1.f));
    state.rotationRadians = constraintEulerRadians(relative);
    const Vec3 omega = child.angularVelocity - parent.angularVelocity;
    state.relativeAngularVelocity = {dot(omega, state.twistAxisWorld), dot(omega, state.swingAxisWorld), dot(omega, state.bendAxisWorld)};
    if (state.motorEnabled) {
        const JPH::Vec3 lambda = record.constraint->GetTotalLambdaMotorRotation();
        const float dt = impl_->lastDt > 1e-6f ? impl_->lastDt : settings_.fixedTimestep;
        state.motorTorque = {lambda.GetX() / dt, lambda.GetY() / dt, lambda.GetZ() / dt};
        const float limit = std::max(state.motorTorqueLimit, 0.f);
        if (limit > 0.f) {
            state.motorSaturated = std::fabs(state.motorTorque.x) >= 0.9f * limit || std::fabs(state.motorTorque.y) >= 0.9f * limit ||
                                   std::fabs(state.motorTorque.z) >= 0.9f * limit;
        }
    }

    const float tolerance = 0.08f;
    bool inside = true;
    if (record.info.swing == SwingType::Cone) {
        const float cone = std::max(std::fabs(state.swing1Max), std::fabs(state.swing2Max));
        inside = state.swingAngle <= cone + tolerance;
    } else {
        inside = state.rotationRadians.y >= state.swing1Min - tolerance && state.rotationRadians.y <= state.swing1Max + tolerance &&
                 state.rotationRadians.z >= state.swing2Min - tolerance && state.rotationRadians.z <= state.swing2Max + tolerance;
    }
    inside = inside && state.rotationRadians.x >= state.twistMin - tolerance && state.rotationRadians.x <= state.twistMax + tolerance;
    state.withinLimits = inside || record.grab || record.fixture;
    const float edge = 0.02f;
    state.twistAtLimit = state.rotationRadians.x <= state.twistMin + edge || state.rotationRadians.x >= state.twistMax - edge;
    if (record.info.swing == SwingType::Cone) {
        const float cone = std::max(std::fabs(state.swing1Max), std::fabs(state.swing2Max));
        state.swing1AtLimit = state.swingAngle >= cone - edge;
        state.swing2AtLimit = state.swing1AtLimit;
    } else {
        state.swing1AtLimit = state.rotationRadians.y <= state.swing1Min + edge || state.rotationRadians.y >= state.swing1Max - edge;
        state.swing2AtLimit = state.rotationRadians.z <= state.swing2Min + edge || state.rotationRadians.z >= state.swing2Max - edge;
    }
    if (!state.positionLocked) {
        state.status = "Position DOF open";
    } else if (state.separation > 0.02f) {
        state.status = "Separated";
    } else if (!state.withinLimits) {
        state.status = "Beyond angular limit";
    } else {
        state.status = "Locked position";
    }
    return state;
}

bool PhysicsWorld::setJointAngularFriction(int joint, float friction) {
    if (!ready_ || joint < 0 || joint >= int(impl_->joints.size()) || !impl_->joints[joint].alive) {
        return false;
    }
    const float value = std::max(friction, 0.f);
    auto* constraint = impl_->joints[joint].constraint.GetPtr();
    constraint->SetMaxFriction(JPH::SixDOFConstraintSettings::RotationX, value);
    constraint->SetMaxFriction(JPH::SixDOFConstraintSettings::RotationY, value);
    constraint->SetMaxFriction(JPH::SixDOFConstraintSettings::RotationZ, value);
    impl_->joints[joint].info.angularFriction = value;
    return true;
}

bool PhysicsWorld::setJointLimits(int joint, float twistMin, float twistMax, float swing1Min, float swing1Max, float swing2Min,
                                  float swing2Max) {
    if (!ready_ || joint < 0 || joint >= int(impl_->joints.size()) || !impl_->joints[joint].alive) {
        return false;
    }
    impl_->joints[joint].constraint->SetRotationLimits(JPH::Vec3(twistMin, swing1Min, swing2Min),
                                                      JPH::Vec3(twistMax, swing1Max, swing2Max));
    auto& info = impl_->joints[joint].info;
    info.twistMin = twistMin;
    info.twistMax = twistMax;
    info.swing1Min = swing1Min;
    info.swing1Max = swing1Max;
    info.swing2Min = swing2Min;
    info.swing2Max = swing2Max;
    return true;
}

RayHit PhysicsWorld::rayCast(Vec3 origin, Vec3 direction, float maxDistance, int ignoreBody) const {
    RayHit hit;
    if (!ready_) {
        return hit;
    }
    direction = normalize(direction);
    JPH::RRayCast ray(toR(origin), toV(direction * maxDistance));
    JPH::RayCastResult result;
    const JPH::BodyID ignore = (ignoreBody >= 0 && ignoreBody < int(impl_->bodies.size()) && impl_->bodies[ignoreBody].alive)
                                   ? impl_->bodies[ignoreBody].id
                                   : JPH::BodyID();
    IgnoreBodyFilter filter(ignore);
    const bool any = ignore.IsInvalid() ? impl_->system.GetNarrowPhaseQuery().CastRay(ray, result)
                                        : impl_->system.GetNarrowPhaseQuery().CastRay(ray, result, {}, {}, filter);
    if (!any) {
        return hit;
    }
    hit.hit = true;
    hit.fraction = result.mFraction;
    hit.point = origin + direction * (maxDistance * result.mFraction);
    const auto found = impl_->contacts.idToIndex.find(result.mBodyID.GetIndexAndSequenceNumber());
    hit.body = found == impl_->contacts.idToIndex.end() ? -1 : found->second;
    JPH::BodyLockRead lock(impl_->system.GetBodyLockInterface(), result.mBodyID);
    if (lock.Succeeded()) {
        hit.normal = toMt(lock.GetBody().GetWorldSpaceSurfaceNormal(result.mSubShapeID2, toR(hit.point)));
    }
    return hit;
}

ShapeView PhysicsWorld::shapeView(int body) const {
    ShapeView view;
    if (!ready_ || body < 0 || body >= int(impl_->bodies.size()) || !impl_->bodies[body].alive) {
        return view;
    }
    const auto state = bodyState(body);
    view.body = body;
    view.shape = impl_->bodies[body].shape.shape;
    view.position = state.position;
    view.rotation = state.rotation;
    view.radius = impl_->bodies[body].shape.radius;
    view.halfHeight = impl_->bodies[body].shape.halfHeight;
    view.halfExtents = impl_->bodies[body].shape.halfExtents;
    return view;
}

int PhysicsWorld::beginGrab(int body, Vec3 bodyPointWorld, Vec3 targetWorld, float frequency, float damping) {
    if (!ready_ || body < 0 || body >= int(impl_->bodies.size()) || !impl_->bodies[body].alive) {
        return -1;
    }
    const BodyState state = bodyState(body);
    if (!state.isDynamic) {
        return -1;
    }
    BodyCreateInfo anchor;
    anchor.name = "Grab anchor";
    anchor.position = targetWorld;
    anchor.shape = BodyCreateInfo::Shape::Sphere;
    anchor.radius = 0.02f;
    anchor.halfHeight = 0.f;
    anchor.mass = 1.f;
    anchor.isKinematic = true;
    anchor.collides = false;
    const int anchorBody = createBody(anchor);
    if (anchorBody < 0) {
        return -1;
    }

    JointCreateInfo joint;
    joint.name = "Grab";
    joint.parent = anchorBody;
    joint.child = body;
    joint.anchorInParent = {};
    joint.axisXInParent = {1.f, 0.f, 0.f};
    joint.axisYInParent = {0.f, 1.f, 0.f};
    joint.anchorInChild = rotate(conjugate(state.rotation), bodyPointWorld - state.position);
    joint.axisXInChild = {1.f, 0.f, 0.f};
    joint.axisYInChild = {0.f, 1.f, 0.f};
    joint.swing = SwingType::Pyramid;
    joint.twistMin = -kPi;
    joint.twistMax = kPi;
    joint.swing1Min = -kPi;
    joint.swing1Max = kPi;
    joint.swing2Min = -kPi;
    joint.swing2Max = kPi;
    joint.angularFriction = 0.f;

    // A zero-width translational limit with a spring is a soft grab. Jolt only
    // supports soft limits on translation, which is the degree of freedom we
    // want to be compliant here. Rotation stays free.
    const int jointIndex = createJoint(joint);
    if (jointIndex < 0) {
        destroyBody(anchorBody);
        return -1;
    }
    auto* constraint = impl_->joints[jointIndex].constraint.GetPtr();
    const float freq = clampf(frequency, 0.5f, 30.f);
    const float damp = clampf(damping, 0.f, 2.f);
    // A zero-width range is a hard lock in Jolt (min >= max). A few millimetres
    // of slack with a limit spring is the supported way to make translation compliant.
    const float slack = 0.002f;
    const JPH::Vec3 softMin(-slack, -slack, -slack);
    const JPH::Vec3 softMax(slack, slack, slack);
    for (int axis = 0; axis < 3; ++axis) {
        JPH::SpringSettings spring(JPH::ESpringMode::FrequencyAndDamping, freq, damp);
        constraint->SetLimitsSpringSettings(JPH::SixDOFConstraintSettings::EAxis(axis), spring);
    }
    constraint->SetTranslationLimits(softMin, softMax);
    impl_->joints[jointIndex].grab = true;
    impl_->joints[jointIndex].info.name = "Grab";

    GrabRecord grab;
    grab.alive = true;
    grab.body = body;
    grab.anchorBody = anchorBody;
    grab.joint = jointIndex;
    const int index = int(impl_->grabs.size());
    impl_->grabs.push_back(grab);
    return index;
}

void PhysicsWorld::updateGrab(int grab, Vec3 targetWorld) {
    if (!ready_ || grab < 0 || grab >= int(impl_->grabs.size()) || !impl_->grabs[grab].alive) {
        return;
    }
    setBodyPose(impl_->grabs[grab].anchorBody, targetWorld, {}, true);
}

void PhysicsWorld::endGrab(int grab) {
    if (!ready_ || grab < 0 || grab >= int(impl_->grabs.size()) || !impl_->grabs[grab].alive) {
        return;
    }
    destroyJoint(impl_->grabs[grab].joint);
    destroyBody(impl_->grabs[grab].anchorBody);
    impl_->grabs[grab].alive = false;
}

void PhysicsWorld::clearGrabs() {
    if (!ready_) {
        return;
    }
    for (int i = 0; i < int(impl_->grabs.size()); ++i) {
        endGrab(i);
    }
}

bool PhysicsWorld::setAngularMotor(int joint, const AngularMotorCommand& command) {
    if (!ready_ || joint < 0 || joint >= int(impl_->joints.size()) || !impl_->joints[joint].alive || impl_->joints[joint].constraint == nullptr) {
        Log::error("Angular motor update rejected for a missing joint.");
        return false;
    }
    if (impl_->joints[joint].grab || impl_->joints[joint].fixture) {
        Log::error("Angular motor update rejected for a grab or support fixture.");
        return false;
    }
    const bool finiteCommand = finite(command.targetOrientation) && finite(command.targetAngularVelocity) &&
                               std::isfinite(command.frequencyHz) && std::isfinite(command.dampingRatio) &&
                               std::isfinite(command.torqueLimit);
    const float qLen = std::sqrt(dot(command.targetOrientation, command.targetOrientation));
    if (!finiteCommand || !(qLen > 0.5f)) {
        Log::error("Angular motor update rejected because '" + impl_->joints[joint].info.name + "' received a non-finite command.");
        return false;
    }

    auto* constraint = impl_->joints[joint].constraint.GetPtr();
    const float frequency = clampf(command.frequencyHz, 0.f, 40.f);
    const float damping = clampf(command.dampingRatio, 0.f, 2.f);
    const float torque = clampf(command.torqueLimit, 0.f, 400.f);
    const bool enable = command.enabled && frequency > 1e-3f && torque > 1e-3f;
    const mt::Quat target = normalize(command.targetOrientation);

    const JPH::SixDOFConstraintSettings::EAxis axes[3] = {
        JPH::SixDOFConstraintSettings::RotationX,
        JPH::SixDOFConstraintSettings::RotationY,
        JPH::SixDOFConstraintSettings::RotationZ,
    };
    for (JPH::SixDOFConstraintSettings::EAxis axis : axes) {
        auto& motor = constraint->GetMotorSettings(axis);
        motor.mSpringSettings.mMode = JPH::ESpringMode::FrequencyAndDamping;
        motor.mSpringSettings.mFrequency = frequency;
        motor.mSpringSettings.mDamping = damping;
        motor.SetTorqueLimit(torque);
    }
    constraint->SetTargetOrientationCS(toJolt(target));
    constraint->SetTargetAngularVelocityCS(toV(command.targetAngularVelocity));
    const JPH::EMotorState motorState = enable ? JPH::EMotorState::Position : JPH::EMotorState::Off;
    for (JPH::SixDOFConstraintSettings::EAxis axis : axes) {
        constraint->SetMotorState(axis, motorState);
    }

    auto& info = impl_->joints[joint].info;
    info.motorFrequency = frequency;
    info.motorDamping = damping;
    info.motorTorqueLimit = torque;
    if (enable) {
        impl_->bodiesApi().ActivateBody(impl_->bodies[info.parent].id);
        impl_->bodiesApi().ActivateBody(impl_->bodies[info.child].id);
    }
    return true;
}

bool PhysicsWorld::beginSupport(int body, float frequency, float damping) {
    if (!ready_ || body < 0 || body >= int(impl_->bodies.size()) || !impl_->bodies[body].alive) {
        Log::error("Support fixture rejected for a missing body.");
        return false;
    }
    const BodyState state = bodyState(body);
    if (!state.isDynamic) {
        Log::error("Support fixture can only hold a dynamic body.");
        return false;
    }
    endSupport();

    BodyCreateInfo anchor;
    anchor.name = "Laboratory support fixture";
    anchor.position = state.position;
    anchor.shape = BodyCreateInfo::Shape::Sphere;
    anchor.radius = 0.05f;
    anchor.mass = 1.f;
    anchor.isKinematic = true;
    anchor.collides = false;
    const int anchorBody = createBody(anchor);
    if (anchorBody < 0) {
        return false;
    }

    JointCreateInfo joint;
    joint.name = "Laboratory support fixture";
    joint.parent = anchorBody;
    joint.child = body;
    joint.anchorInParent = {};
    joint.axisXInParent = {1.f, 0.f, 0.f};
    joint.axisYInParent = {0.f, 1.f, 0.f};
    joint.anchorInChild = {};
    joint.axisXInChild = {1.f, 0.f, 0.f};
    joint.axisYInChild = {0.f, 1.f, 0.f};
    joint.swing = SwingType::Pyramid;
    joint.twistMin = -kPi;
    joint.twistMax = kPi;
    joint.swing1Min = -kPi;
    joint.swing1Max = kPi;
    joint.swing2Min = -kPi;
    joint.swing2Max = kPi;
    joint.angularFriction = 0.f;
    const int jointIndex = createJoint(joint);
    if (jointIndex < 0) {
        destroyBody(anchorBody);
        return false;
    }
    auto* constraint = impl_->joints[jointIndex].constraint.GetPtr();
    const float freq = clampf(frequency, 0.5f, 30.f);
    const float damp = clampf(damping, 0.f, 2.f);
    const float slack = 0.002f;
    for (int axis = 0; axis < 3; ++axis) {
        JPH::SpringSettings spring(JPH::ESpringMode::FrequencyAndDamping, freq, damp);
        constraint->SetLimitsSpringSettings(JPH::SixDOFConstraintSettings::EAxis(axis), spring);
    }
    constraint->SetTranslationLimits(JPH::Vec3(-slack, -slack, -slack), JPH::Vec3(slack, slack, slack));
    impl_->joints[jointIndex].fixture = true;
    impl_->joints[jointIndex].info.name = "Laboratory support fixture";
    impl_->support.alive = true;
    impl_->support.body = body;
    impl_->support.anchorBody = anchorBody;
    impl_->support.joint = jointIndex;
    Log::info("Laboratory support fixture enabled. This is not autonomous balance.");
    return true;
}

void PhysicsWorld::endSupport() {
    if (!ready_ || !impl_->support.alive) {
        return;
    }
    destroyJoint(impl_->support.joint);
    destroyBody(impl_->support.anchorBody);
    impl_->support = {};
}

bool PhysicsWorld::supportActive() const { return ready_ && impl_ != nullptr && impl_->support.alive; }

int PhysicsWorld::supportAnchor() const { return supportActive() ? impl_->support.anchorBody : -1; }

}  // namespace mt
