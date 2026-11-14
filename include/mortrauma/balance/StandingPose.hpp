#pragma once

#include "mortrauma/balance/BalanceConfig.hpp"
#include "mortrauma/motors/MotorController.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <string>
#include <vector>

namespace mt {

struct StandingPlacement {
    bool ok = false;
    std::string error;
    float maxSeparation = 0.f;
    float footHeight = 0.f;
    float coplanarityM = 0.f;
    float heelForeGapM = 0.f;
    float soleNormalError = 0.f;
    float comToAnkleM = 0.f;
    Vec3 equilibriumTarget{};
    Vec3 pelvisUp{0.f, 1.f, 0.f};
    std::vector<MotorTaskCommand> reference;
};

// Constraint-space standing target for one joint. Identity is not assumed:
// knees and hips use the anatomical frames, and every other joint keeps the
// imported constraint orientation.
Quat standingReference(const RagdollJoint& joint, const BalanceConfig& config);

// Places the character into the standing reference before simulation.
// This writes body poses once, as a laboratory reset. It is not balance.
StandingPlacement placeStandingPose(PhysicsWorld& world, Ragdoll& ragdoll, const BalanceConfig& config);

}  // namespace mt
