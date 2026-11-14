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
// Positive forward points toward the toes. Positive swing2 pitches the torso
// further that way, so a positive COM error commands a negative swing2.
struct BalanceAxes {
    float ankleSwing2 = 0.f;
    float ankleSwing1 = 0.f;
    float hipSwing2 = 0.f;
    float hipSwing1 = 0.f;
};

BalanceAxes balanceCorrection(const BalanceConfig& config, float forward, float lateral, float forwardRate, float lateralRate, bool ankles, bool hips);

// Extra joint command owned by the stepping supervisor. replace drops the
// balance correction on that joint and uses euler alone, still added to the
// standing reference. The balance controller remains the only motor writer.
struct JointBias {
    Vec3 euler{};
    bool replace = false;
};

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
    // Applied on the next command. The offset moves the tracked point; it does
    // not pin a foot. Cleared by passing an empty bias and a zero offset.
    void setStepBias(Vec3 referenceOffset, std::unordered_map<std::string, JointBias> bias);
    // 0 keeps the two-foot law. 1 corrects only the right leg, 2 only the left.
    // The swing hip and knee stay on the placed reference. Hip 2 uses the
    // mirrored swing1 sign from v0.4.1.
    void setStanceLeg(int stanceLeg) { stanceLeg_ = stanceLeg; }
    int stanceLeg() const { return stanceLeg_; }

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
    bool fellLatch_ = false;
    float stoodSeconds_ = 0.f;
    Vec3 stepOffset_{};
    int stanceLeg_ = 0;
    std::unordered_map<std::string, JointBias> stepBias_;
};

}  // namespace mt
