#pragma once

#include "mortrauma/balance/BalanceState.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

namespace mt {

class BalanceEstimator {
public:
    // Mass-weighted center of mass of every ragdoll body, foot contacts, and
    // torso/pelvis orientation. Does not write physics.
    BalanceState measure(const PhysicsWorld& world, const Ragdoll& ragdoll, float equilibriumLeadM = 0.f) const;
};

}  // namespace mt
