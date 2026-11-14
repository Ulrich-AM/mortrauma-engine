#include "mortrauma/balance/StepController.hpp"

#include "mortrauma/core/Math.hpp"

#include <cmath>

namespace mt {
namespace {

const RagdollBody* findBody(const Ragdoll& ragdoll, const char* id) {
    for (const auto& body : ragdoll.bodies) {
        if (body.id == id) {
            return &body;
        }
    }
    return nullptr;
}

Vec3 flat(Vec3 value) {
    value.y = 0.f;
    return value;
}

}  // namespace

const char* stepPhaseName(StepPhase phase) {
    switch (phase) {
        case StepPhase::Idle: return "Idle";
        case StepPhase::WeightTransfer: return "WeightTransfer";
        case StepPhase::FootLift: return "FootLift";
        case StepPhase::Swing: return "Swing";
        case StepPhase::FootPlacement: return "FootPlacement";
        case StepPhase::Reestablish: return "Reestablish";
        case StepPhase::Falling: return "Falling";
    }
    return "Idle";
}

void StepController::begin(StepMode mode, bool supportRight, Vec3 directionWorld) {
    mode_ = mode;
    supportRight_ = supportRight;
    direction_ = flat(directionWorld);
    if (length(direction_) < 1e-4f) {
        direction_ = {0.f, 0.f, -1.f};
    }
    direction_ = normalize(direction_);
    phase_ = StepPhase::WeightTransfer;
    phaseTime_ = 0.f;
    note_ = "started";
}

void StepController::requestTransfer(bool supportRight) { begin(StepMode::Transfer, supportRight, {}); }

void StepController::requestLift(bool supportRight) { begin(StepMode::Lift, supportRight, {}); }

void StepController::requestRecovery(bool supportRight, Vec3 directionWorld) { begin(StepMode::Recover, supportRight, directionWorld); }

void StepController::cancel() {
    phase_ = StepPhase::Idle;
    mode_ = StepMode::None;
    phaseTime_ = 0.f;
    note_ = "cancelled";
}

float StepController::envelope() const {
    const float duration = std::max(phaseDuration_, 0.05f);
    return clampf(phaseTime_ / duration, 0.f, 1.f);
}

void StepController::advance(const BalanceState& state) {
    const bool lost = state.com.y < 0.72f || state.torsoTilt > 0.7f || state.phase == BalancePhase::Falling || state.phase == BalancePhase::Grounded;
    if (phase_ != StepPhase::Idle && phase_ != StepPhase::Falling && lost && phaseTime_ > 0.05f) {
        phase_ = StepPhase::Falling;
        phaseTime_ = 0.f;
        note_ = "lost the stand; motors stay on the balance fall";
        return;
    }
    if (phaseTime_ < std::max(phaseDuration_, 0.05f)) {
        return;
    }
    phaseTime_ = 0.f;
    switch (phase_) {
        case StepPhase::WeightTransfer:
            phase_ = mode_ == StepMode::Transfer ? StepPhase::Reestablish : StepPhase::FootLift;
            break;
        case StepPhase::FootLift:
            phase_ = mode_ == StepMode::Recover ? StepPhase::Swing : StepPhase::FootPlacement;
            break;
        case StepPhase::Swing:
            phase_ = StepPhase::FootPlacement;
            break;
        case StepPhase::FootPlacement:
            phase_ = StepPhase::Reestablish;
            break;
        case StepPhase::Reestablish:
        case StepPhase::Falling:
            phase_ = StepPhase::Idle;
            mode_ = StepMode::None;
            note_ = "returned to stationary balance";
            break;
        case StepPhase::Idle:
            break;
    }
}

void StepController::update(float dt, const PhysicsWorld& world, const BalanceState& state, const Ragdoll& ragdoll, BalanceController& balance) {
    if (phase_ == StepPhase::Idle) {
        balance.setStepBias({}, {});
        return;
    }
    phaseTime_ += std::max(dt, 0.f);
    advance(state);
    if (phase_ == StepPhase::Idle || phase_ == StepPhase::Falling) {
        balance.setStepBias({}, {});
        return;
    }

    const char* stanceId = supportRight_ ? "foot_1" : "foot_2";
    const char* swingId = supportRight_ ? "foot_2" : "foot_1";
    const char* swingHip = supportRight_ ? "hip_2" : "hip_1";
    const char* swingKnee = supportRight_ ? "knee_2" : "knee_1";
    const RagdollBody* stance = findBody(ragdoll, stanceId);
    const RagdollBody* swing = findBody(ragdoll, swingId);
    Vec3 offset{};
    if (stance != nullptr && state.equilibriumValid) {
        const Vec3 stancePos = world.bodyState(stance->physics).position;
        Vec3 toward = flat(stancePos - state.equilibriumTarget);
        if (length(toward) > 1e-4f) {
            float share = 0.7f;
            if (phase_ == StepPhase::WeightTransfer) {
                share *= envelope();
            } else if (phase_ == StepPhase::Reestablish) {
                share *= (1.f - envelope());
            }
            offset = toward * share;
        }
        if (swing != nullptr) {
            const Vec3 swingPos = world.bodyState(swing->physics).position;
            stepTarget_ = flat(stancePos) + direction_ * stepDistance_;
            stepTarget_.y = swingPos.y;
        }
    }

    const float liftScale = clampf(liftHeight_ / 0.04f, 0.f, 2.f);
    float flex = 0.f;
    float reach = 0.f;
    if (phase_ == StepPhase::FootLift) {
        flex = envelope();
    } else if (phase_ == StepPhase::Swing) {
        flex = 1.f;
        reach = envelope();
    } else if (phase_ == StepPhase::FootPlacement) {
        flex = 1.f - envelope();
        reach = 1.f - envelope();
    }

    std::unordered_map<std::string, JointBias> bias;
    if (flex > 1e-4f || reach > 1e-4f) {
        JointBias hip;
        hip.replace = true;
        hip.euler.z = (0.28f * flex + 0.2f * reach) * liftScale;
        hip.euler.y = (supportRight_ ? 0.12f : -0.12f) * reach * liftScale;
        JointBias knee;
        knee.replace = true;
        knee.euler.z = 0.4f * flex * liftScale;
        bias.emplace(swingHip, hip);
        bias.emplace(swingKnee, knee);
    }
    note_ = std::string(stepPhaseName(phase_)) + (supportRight_ ? " support right" : " support left");
    balance.setStepBias(offset, std::move(bias));
}

}  // namespace mt
