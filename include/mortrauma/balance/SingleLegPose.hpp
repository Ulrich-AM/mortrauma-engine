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
    // True only when the bodies were actually written. An infeasible result
    // is not written unless the caller asked for a diagnostic placement.
    bool placed = false;
    bool diagnostic = false;
    bool supportRight = true;
    float totalMass = 0.f;
    float comToAnkleM = 0.f;
    float hipToAnkleM = 0.f;
    float soleMarginM = 0.f;
    float swingClearanceM = 0.f;
    float torsoTilt = 0.f;
    float pelvisTilt = 0.f;
    // Peak axis component, and the length of the stance-ankle moment vector.
    // gravityMoment is mass * g * the horizontal COM-to-ankle distance.
    float ankleTorque = 0.f;
    float ankleMomentMagnitude = 0.f;
    float gravityMoment = 0.f;
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
    float postureCost = 0.f;
    float handOffsetM = 0.f;
    float referenceError = 0.f;
    float maxSeparation = 0.f;
    std::vector<MotorTaskCommand> reference;
};

// Searches joint angles. A feasible pose is written, the same way a standing
// reset is written. Feasible means the COM projection is inside the stance
// sole, within 2 cm of the ankle, every joint moment is inside its motor
// limit, the sole is flat, the swing foot is clear, and the torso tilt is
// under 0.45 rad. Among those poses the lowest posture cost is kept.
// An infeasible pose is not written unless placeRejected is set. That path
// is diagnostic only and is labeled as such.
SingleLegCandidate placeSingleLegCandidate(PhysicsWorld& world, Ragdoll& ragdoll, const BalanceConfig& config, bool supportRight, bool placeRejected = false);

}  // namespace mt
