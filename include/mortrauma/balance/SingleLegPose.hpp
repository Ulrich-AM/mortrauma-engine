#pragma once

#include "mortrauma/balance/BalanceConfig.hpp"
#include "mortrauma/balance/StandingPose.hpp"
#include "mortrauma/motors/MotorController.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <string>
#include <vector>

namespace mt {

// Static single-leg pose search. It does not step the simulation.
// Torques are the joint moments of gravity plus one vertical contact force
// at the whole-body COM projection. They are not measured motor torques.
// Solved contact impulses are not used.
struct SingleLegCandidate {
    bool searched = false;
    bool feasible = false;
    bool supportRight = true;
    float totalMass = 0.f;
    float comToAnkleM = 0.f;
    float hipToAnkleM = 0.f;
    float soleMarginM = 0.f;
    float swingClearanceM = 0.f;
    float torsoTilt = 0.f;
    float pelvisTilt = 0.f;
    float ankleTorque = 0.f;
    float kneeTorque = 0.f;
    float hipTorque = 0.f;
    float hipRootTorque = 0.f;
    float spineTorque = 0.f;
    float limitingTorque = 0.f;
    float limitingLimit = 0.f;
    std::string limitingJoint;
    std::string rejection;
    std::string lightest;
    Vec3 com{};
    Vec3 ankle{};
    float stanceHipSwing1Deg = 0.f;
    float stanceHipSwing2Deg = 0.f;
    float stanceKneeDeg = 0.f;
    float swingHipSwing1Deg = 0.f;
    float swingHipSwing2Deg = 0.f;
    float swingKneeDeg = 0.f;
    float spineSwing1Deg = 0.f;
    float maxSeparation = 0.f;
    std::vector<MotorTaskCommand> reference;
};

// Searches joint angles, writes the best pose, and returns the static result.
// The write happens before simulation, the same way a standing reset does.
// A feasible pose has the COM projection inside the stance sole, within the
// ankle's 12 N·m arm, every joint moment inside its motor limit, a flat sole,
// and the swing foot clear of the floor. An infeasible best pose is still
// written so a simulation can show the failure.
SingleLegCandidate placeSingleLegCandidate(PhysicsWorld& world, Ragdoll& ragdoll, const BalanceConfig& config, bool supportRight);

}  // namespace mt
