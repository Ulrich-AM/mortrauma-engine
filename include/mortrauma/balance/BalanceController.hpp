#pragma once

#include "mortrauma/balance/BalanceConfig.hpp"
#include "mortrauma/balance/BalanceDiagnostics.hpp"
#include "mortrauma/balance/BalanceEstimator.hpp"
#include "mortrauma/balance/BalanceState.hpp"
#include "mortrauma/motors/MotorController.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace mt {

// Reads a balance estimate and submits one motor task. It does not call
// setAngularMotor and it does not move bodies.
class BalanceController {
public:
    void setConfig(const BalanceConfig& config);
    const BalanceConfig& config() const { return config_; }
    bool loadPreset(const std::string& name, std::string& error);

    void setEnabled(bool enabled, MotorController& motors);
    bool enabled() const { return enabled_; }

    void setReference(std::vector<MotorTaskCommand> reference);
    void resetState();

    void observe(const PhysicsWorld& world, const Ragdoll& ragdoll, float dt);
    void command(const Ragdoll& ragdoll, MotorController& motors);

    const BalanceState& state() const { return state_; }
    BalanceDiagnostics& diagnostics() { return diagnostics_; }
    const BalanceDiagnostics& diagnostics() const { return diagnostics_; }

private:
    Quat referenceFor(const RagdollJoint& joint) const;
    void transition(BalancePhase next, bool force);
    void pushHistory();

    BalanceConfig config_{};
    BalanceEstimator estimator_{};
    BalanceState state_{};
    BalanceDiagnostics diagnostics_{};
    bool enabled_ = false;
    float timeInPhase_ = 0.f;
    std::unordered_map<std::string, Quat> reference_;
    std::unordered_map<std::string, Quat> commanded_;
    float lastDt_ = 0.f;
    bool anklesSettled_ = false;
};

}  // namespace mt
