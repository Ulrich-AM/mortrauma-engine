#pragma once

#include "mortrauma/balance/BalanceController.hpp"
#include "mortrauma/balance/BalanceState.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <string>

namespace mt {

enum class StepPhase {
    Idle,
    TransferWeight,
    StabilizeSupport,
    BeginUnloading,
    ConfirmSupport,
    LiftFoot,
    HoldFoot,
    LowerFoot,
    RestoreSupport,
    Completed,
    Aborted,
    Falling
};

enum class StepMode { None, Transfer, Support, Unload, Lift, Sequence, Recover };

const char* stepPhaseName(StepPhase phase);

// One supervisor for weight transfer, single-leg support, and a foot lift.
// It only writes a bias into the balance controller. Feet stay dynamic.
// Recovery stepping is intentionally not extended beyond this foundation.
class StepController {
public:
    void requestTransfer(bool supportRight);
    void requestSupport(bool supportRight);
    void requestUnload(bool supportRight);
    void requestLift(bool supportRight);
    void requestLower();
    void requestSequence(bool supportRight);
    void requestRecovery(bool supportRight, Vec3 directionWorld);
    void abort();
    void cancel();

    void update(float dt, const PhysicsWorld& world, const BalanceState& state, const Ragdoll& ragdoll, BalanceController& balance);

    void setLiftHeight(float meters) { liftHeight_ = meters; }
    void setStepDistance(float meters) { stepDistance_ = meters; }
    void setPhaseDuration(float seconds) { transferDuration_ = seconds; }
    void setTransferDuration(float seconds) { transferDuration_ = seconds; }
    void setLiftDuration(float seconds) { liftDuration_ = seconds; }
    void setHoldDuration(float seconds) { holdDuration_ = seconds; }
    void setSupportRight(bool supportRight) { supportRight_ = supportRight; }

    float liftHeight() const { return liftHeight_; }
    float stepDistance() const { return stepDistance_; }
    float phaseDuration() const { return transferDuration_; }
    float transferDuration() const { return transferDuration_; }
    float liftDuration() const { return liftDuration_; }
    float holdDuration() const { return holdDuration_; }

    StepPhase phase() const { return phase_; }
    StepMode mode() const { return mode_; }
    bool supportRight() const { return supportRight_; }
    bool active() const;
    Vec3 stepTarget() const { return swingTarget_; }
    Vec3 comTarget() const { return comTarget_; }
    Vec3 swingOrigin() const { return swingOrigin_; }
    Vec3 swingActual() const { return swingActual_; }
    Vec3 swingTarget() const { return swingTarget_; }
    float comError() const { return comError_; }
    float supportMargin() const { return supportMargin_; }
    float swingClearance() const { return swingClearance_; }
    float swingTargetHeight() const { return swingTargetHeight_; }
    float heightError() const { return heightError_; }
    float hipSwing1Command() const { return hipSwing1_; }
    float peakUprightClearance() const { return peakUprightClearance_; }
    float swingVerticalVelocity() const { return swingVerticalVelocity_; }
    float stanceSlip() const { return stanceSlip_; }
    float stanceAnkleTorque() const { return stanceAnkleTorque_; }
    float stanceHipTorque() const { return stanceHipTorque_; }
    float jacobianHip() const { return jacobianHip_; }
    float jacobianKnee() const { return jacobianKnee_; }
    bool loadMeasured() const { return false; }
    const std::string& supportEstimate() const { return supportEstimate_; }
    const std::string& note() const { return note_; }
    const std::string& failure() const { return failure_; }

private:
    void begin(StepMode mode, bool supportRight);
    void fail(const std::string& reason);
    void publish(BalanceController& balance, Vec3 offset, const std::unordered_map<std::string, JointBias>& bias);

    StepPhase phase_ = StepPhase::Idle;
    StepMode mode_ = StepMode::None;
    bool supportRight_ = true;
    bool goalReady_ = false;
    float phaseTime_ = 0.f;
    float settleTime_ = 0.f;
    float liftHeight_ = 0.03f;
    float stepDistance_ = 0.18f;
    float transferDuration_ = 4.2f;
    float liftDuration_ = 1.2f;
    float holdDuration_ = 0.45f;
    float desiredClearance_ = 0.f;
    float flexHip_ = 0.f;
    float flexKnee_ = 0.f;
    float hipSwing1_ = 0.f;
    float hipIntegral_ = 0.f;
    float comError_ = 0.f;
    float supportMargin_ = 0.f;
    float swingClearance_ = 0.f;
    float swingTargetHeight_ = 0.f;
    float heightError_ = 0.f;
    float baseSoleY_ = 0.f;
    float peakUprightClearance_ = 0.f;
    float previousSoleY_ = 0.f;
    float swingVerticalVelocity_ = 0.f;
    float stanceSlip_ = 0.f;
    float stanceAnkleTorque_ = 0.f;
    float stanceHipTorque_ = 0.f;
    float jacobianHip_ = 0.12f;
    float jacobianKnee_ = 0.22f;
    float unloadSoleY_ = 0.f;
    int swingPointsAtEntry_ = 0;
    Vec3 direction_{0.f, 0.f, -1.f};
    Vec3 offset_{};
    Vec3 goalOffset_{};
    Vec3 comTarget_{};
    Vec3 swingOrigin_{};
    Vec3 swingActual_{};
    Vec3 swingTarget_{};
    std::string supportEstimate_ = "not evaluated";
    std::string note_ = "idle";
    std::string failure_;
};

}  // namespace mt
