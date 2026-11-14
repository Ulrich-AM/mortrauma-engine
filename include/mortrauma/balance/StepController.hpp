#pragma once

#include "mortrauma/balance/BalanceController.hpp"
#include "mortrauma/balance/BalanceState.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <string>

namespace mt {

enum class StepPhase { Idle, WeightTransfer, FootLift, Swing, FootPlacement, Reestablish, Falling };

enum class StepMode { None, Transfer, Lift, Recover };

const char* stepPhaseName(StepPhase phase);

// One supervisor for a single weight shift, foot lift, or recovery step.
// It only writes a bias into the balance controller. Feet stay dynamic.
class StepController {
public:
    void requestTransfer(bool supportRight);
    void requestLift(bool supportRight);
    void requestRecovery(bool supportRight, Vec3 directionWorld);
    void cancel();

    void update(float dt, const PhysicsWorld& world, const BalanceState& state, const Ragdoll& ragdoll, BalanceController& balance);

    void setLiftHeight(float meters) { liftHeight_ = meters; }
    void setStepDistance(float meters) { stepDistance_ = meters; }
    void setPhaseDuration(float seconds) { phaseDuration_ = seconds; }
    float liftHeight() const { return liftHeight_; }
    float stepDistance() const { return stepDistance_; }
    float phaseDuration() const { return phaseDuration_; }

    StepPhase phase() const { return phase_; }
    StepMode mode() const { return mode_; }
    bool supportRight() const { return supportRight_; }
    Vec3 stepTarget() const { return stepTarget_; }
    const std::string& note() const { return note_; }

private:
    void begin(StepMode mode, bool supportRight, Vec3 directionWorld);
    void advance(const BalanceState& state);
    float envelope() const;

    StepPhase phase_ = StepPhase::Idle;
    StepMode mode_ = StepMode::None;
    bool supportRight_ = true;
    float phaseTime_ = 0.f;
    float liftHeight_ = 0.04f;
    float stepDistance_ = 0.18f;
    float phaseDuration_ = 0.7f;
    Vec3 direction_{0.f, 0.f, -1.f};
    Vec3 stepTarget_{};
    std::string note_ = "idle";
};

}  // namespace mt
