#include "mortrauma/balance/StepController.hpp"

#include "mortrauma/core/Math.hpp"

#include <cmath>
#include <unordered_map>

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

const RagdollJoint* findJoint(const Ragdoll& ragdoll, const char* id) {
    for (const auto& joint : ragdoll.joints) {
        if (joint.id == id) {
            return &joint;
        }
    }
    return nullptr;
}

Vec3 flat(Vec3 value) {
    value.y = 0.f;
    return value;
}

float horizontal(Vec3 value) { return length(flat(value)); }

// Hip swing1 uses one shared facing axis on both legs, so the same Euler
// sign abducts one thigh and adducts the other. A pelvis shift needs the
// opposite sign on hip_2. The magnitude is the same on both sides.
float mirroredHipSwing(float comMinusTargetAlongRight) { return clampf(-2.2f * comMinusTargetAlongRight, -0.30f, 0.30f); }

}  // namespace

const char* stepPhaseName(StepPhase phase) {
    switch (phase) {
        case StepPhase::Idle: return "Idle";
        case StepPhase::TransferWeight: return "TransferWeight";
        case StepPhase::StabilizeSupport: return "StabilizeSupport";
        case StepPhase::UnloadSwingFoot: return "UnloadSwingFoot";
        case StepPhase::LiftFoot: return "LiftFoot";
        case StepPhase::HoldFoot: return "HoldFoot";
        case StepPhase::LowerFoot: return "LowerFoot";
        case StepPhase::RestoreSupport: return "RestoreSupport";
        case StepPhase::Completed: return "Completed";
        case StepPhase::Aborted: return "Aborted";
        case StepPhase::Falling: return "Falling";
    }
    return "Idle";
}

bool StepController::active() const {
    return phase_ != StepPhase::Idle && phase_ != StepPhase::Completed && phase_ != StepPhase::Aborted && phase_ != StepPhase::Falling;
}

void StepController::begin(StepMode mode, bool supportRight) {
    mode_ = mode;
    supportRight_ = supportRight;
    phase_ = StepPhase::TransferWeight;
    phaseTime_ = 0.f;
    settleTime_ = 0.f;
    goalReady_ = false;
    offset_ = {};
    goalOffset_ = {};
    desiredClearance_ = 0.f;
    flexHip_ = 0.f;
    flexKnee_ = 0.f;
    hipSwing1_ = 0.f;
    hipIntegral_ = 0.f;
    peakUprightClearance_ = 0.f;
    baseSoleY_ = 0.f;
    failure_.clear();
    note_ = "started";
}

void StepController::requestTransfer(bool supportRight) { begin(StepMode::Transfer, supportRight); }

void StepController::requestSupport(bool supportRight) { begin(StepMode::Support, supportRight); }

void StepController::requestLift(bool supportRight) { begin(StepMode::Lift, supportRight); }

void StepController::requestSequence(bool supportRight) { begin(StepMode::Sequence, supportRight); }

void StepController::requestRecovery(bool supportRight, Vec3 directionWorld) {
    direction_ = flat(directionWorld);
    if (length(direction_) < 1e-4f) {
        direction_ = {0.f, 0.f, -1.f};
    } else {
        direction_ = normalize(direction_);
    }
    // v0.4.0 recovery placement is not extended. The request only shifts
    // weight onto the selected foot and returns.
    begin(StepMode::Recover, supportRight);
    note_ = "recovery stepping is not expanded; weight transfer only";
}

void StepController::requestLower() {
    if (phase_ == StepPhase::LiftFoot || phase_ == StepPhase::HoldFoot || phase_ == StepPhase::UnloadSwingFoot) {
        phase_ = StepPhase::LowerFoot;
        phaseTime_ = 0.f;
        settleTime_ = 0.f;
        note_ = "lowering";
        return;
    }
    if (!active()) {
        note_ = "no lifted foot to lower";
    }
}

void StepController::abort() {
    if (!active()) {
        return;
    }
    fail("aborted by request");
}

void StepController::cancel() {
    phase_ = StepPhase::Idle;
    mode_ = StepMode::None;
    phaseTime_ = 0.f;
    offset_ = {};
    goalOffset_ = {};
    flexHip_ = 0.f;
    flexKnee_ = 0.f;
    hipSwing1_ = 0.f;
    hipIntegral_ = 0.f;
    desiredClearance_ = 0.f;
    note_ = "cancelled";
}

void StepController::fail(const std::string& reason) {
    failure_ = reason;
    phase_ = StepPhase::Aborted;
    phaseTime_ = 0.f;
    note_ = reason;
}

void StepController::publish(BalanceController& balance, Vec3 offset, const std::unordered_map<std::string, JointBias>& bias) {
    balance.setStepBias(offset, bias);
}

void StepController::update(float dt, const PhysicsWorld& world, const BalanceState& state, const Ragdoll& ragdoll, BalanceController& balance) {
    if (phase_ == StepPhase::Idle || phase_ == StepPhase::Completed || phase_ == StepPhase::Falling) {
        publish(balance, {}, {});
        return;
    }
    dt = std::max(dt, 0.f);
    phaseTime_ += dt;

    const bool falling = state.com.y < 0.72f || state.torsoTilt > 0.7f || state.phase == BalancePhase::Falling || state.phase == BalancePhase::Grounded;
    if (falling && phaseTime_ > 0.05f && phase_ != StepPhase::Aborted) {
        phase_ = StepPhase::Falling;
        failure_ = "lost the stand";
        note_ = failure_;
        offset_ = {};
        flexHip_ = 0.f;
        flexKnee_ = 0.f;
        publish(balance, {}, {});
        return;
    }

    const char* stanceId = supportRight_ ? "foot_1" : "foot_2";
    const char* swingId = supportRight_ ? "foot_2" : "foot_1";
    const char* swingHip = supportRight_ ? "hip_2" : "hip_1";
    const char* swingKnee = supportRight_ ? "knee_2" : "knee_1";
    const char* swingAnkle = supportRight_ ? "ankle_2" : "ankle_1";
    const RagdollBody* stance = findBody(ragdoll, stanceId);
    const RagdollBody* swing = findBody(ragdoll, swingId);
    const bool stanceGrounded = supportRight_ ? state.rightGrounded : state.leftGrounded;
    const bool swingTouching = supportRight_ ? state.leftTouching : state.rightTouching;
    const int stancePoints = supportRight_ ? state.rightPoints : state.leftPoints;
    const int swingPoints = supportRight_ ? state.leftPoints : state.rightPoints;

    Vec3 stancePos{};
    Vec3 swingPos{};
    float sole = 0.f;
    if (stance != nullptr) {
        stancePos = world.bodyState(stance->physics).position;
    }
    if (swing != nullptr) {
        const auto sample = world.bodyState(swing->physics);
        swingPos = sample.position;
        const Vec3 heel = sample.position + rotate(sample.rotation, swing->heelLocal);
        const Vec3 toe = sample.position + rotate(sample.rotation, swing->forefootLocal);
        sole = std::min(heel.y, toe.y);
        swingActual_ = (heel + toe) * 0.5f;
    }
    swingClearance_ = sole;

    if (state.leftGrounded && state.rightGrounded) {
        supportEstimate_ = "both feet touching; contact counts are not loads";
    } else if (state.leftGrounded) {
        supportEstimate_ = "left foot touching, right foot not; contact estimate";
    } else if (state.rightGrounded) {
        supportEstimate_ = "right foot touching, left foot not; contact estimate";
    } else {
        supportEstimate_ = "no foot touching";
    }
    if (stancePoints > 0 && swingPoints == 0) {
        supportEstimate_ += supportRight_ ? "; right contact only" : "; left contact only";
    }

    const Vec3 equilibrium = state.equilibriumValid ? flat(state.equilibriumTarget) : flat(state.com);
    if (!goalReady_ && stance != nullptr) {
        const Vec3 towardFoot = flat(stancePos - state.com);
        const float available = std::max(0.f, horizontal(towardFoot) - 0.06f);
        // A few centimetres toward the foot. The foot center is outside the
        // moment the 50 N·m hips can hold (weight times a 15 cm arm).
        const float reach = std::min(0.055f, available);
        const Vec3 goal = horizontal(towardFoot) > 1e-4f ? flat(state.com) + normalize(towardFoot) * reach : flat(state.com);
        goalOffset_ = flat(goal - equilibrium);
        goalReady_ = true;
        baseSoleY_ = sole;
        swingOrigin_ = swingActual_;
        swingPointsAtEntry_ = swingPoints;
    }

    const float transferRate = std::max(0.06f, 0.055f / std::max(transferDuration_, 0.4f));
    if (phase_ == StepPhase::RestoreSupport || phase_ == StepPhase::Aborted) {
        const Vec3 back = -offset_;
        const float step = transferRate * dt;
        if (horizontal(back) <= step) {
            offset_ = {};
        } else {
            offset_ = offset_ + normalize(back) * step;
        }
        flexHip_ = std::max(0.f, flexHip_ - dt * 0.8f);
        flexKnee_ = std::max(0.f, flexKnee_ - dt * 1.1f);
        desiredClearance_ = std::max(0.f, desiredClearance_ - dt * (liftHeight_ / std::max(liftDuration_, 0.3f)));
    } else if (phase_ != StepPhase::Falling) {
        const Vec3 delta = goalOffset_ - offset_;
        const float step = transferRate * dt;
        if (horizontal(delta) <= step) {
            offset_ = goalOffset_;
        } else if (horizontal(delta) > 1e-6f) {
            offset_ = offset_ + normalize(delta) * step;
        }
    }

    comTarget_ = equilibrium + offset_;
    comTarget_.y = 0.f;
    // Hips aim at the finished support target. The reference offset still
    // ramps, so the ankles are not asked to jump there in one step.
    const Vec3 hipGoal = equilibrium + goalOffset_;
    const Vec3 comErrorVec = flat(state.com - hipGoal);
    comError_ = horizontal(comErrorVec);
    const float lateralError = state.equilibriumValid ? dot(comErrorVec, state.right) : 0.f;
    const float speed = horizontal(state.comVelocity);
    const bool upright = state.com.y > 0.90f && state.torsoTilt < 0.40f;
    const float solePitch = supportRight_ ? state.leftSolePitch : state.rightSolePitch;

    // Support margin is a geometry estimate from the stance foot position,
    // not a measured center of pressure. Negative means the COM is outside
    // a 4 cm radius around that foot.
    supportMargin_ = 0.04f - horizontal(flat(state.com - stancePos));
    if (state.leftGrounded && state.rightGrounded) {
        supportMargin_ = state.margin;
    }

    if (phase_ == StepPhase::RestoreSupport || phase_ == StepPhase::Aborted) {
        hipIntegral_ = clampf(hipIntegral_ - std::copysign(std::min(std::fabs(hipIntegral_), 0.4f * dt), hipIntegral_), -0.22f, 0.22f);
    } else {
        hipIntegral_ += clampf(-lateralError, -0.02f, 0.02f) * dt * 1.6f;
        hipIntegral_ = clampf(hipIntegral_, -0.22f, 0.22f);
    }
    const bool close = comError_ < 0.018f && speed < 0.10f;
    if (close) {
        settleTime_ += dt;
    } else if (phase_ == StepPhase::TransferWeight || phase_ == StepPhase::StabilizeSupport) {
        settleTime_ = 0.f;
    }

    const bool lifting = phase_ == StepPhase::LiftFoot || phase_ == StepPhase::HoldFoot || phase_ == StepPhase::LowerFoot;
    if (lifting) {
        float desired = 0.f;
        if (phase_ == StepPhase::LiftFoot) {
            const float ramp = clampf(phaseTime_ / std::max(liftDuration_, 0.3f), 0.f, 1.f);
            desired = liftHeight_ * ramp;
        } else if (phase_ == StepPhase::HoldFoot) {
            desired = liftHeight_;
        } else {
            const float ramp = clampf(phaseTime_ / std::max(liftDuration_, 0.3f), 0.f, 1.f);
            desired = liftHeight_ * (1.f - ramp);
        }
        desiredClearance_ = desired;
        swingTargetHeight_ = baseSoleY_ + desiredClearance_;
        heightError_ = swingTargetHeight_ - sole;
        // Damped least-squares step of a 1x2 Jacobian. The columns are the
        // expected sole rise, in metres per radian, of swing-hip flexion and
        // swing-knee flexion. The residual is the measured height error.
        // Metres per radian, from a two-link leg at a standing knee bend.
        // One damped step per frame, with the height error clamped so a bad
        // reading cannot demand a full extension in a single tick.
        const float hipColumn = 0.12f;
        const float kneeColumn = 0.22f;
        const float damping = 0.01f;
        const float denominator = hipColumn * hipColumn + kneeColumn * kneeColumn + damping;
        const float bounded = clampf(heightError_, -0.006f, 0.006f);
        flexHip_ += (hipColumn / denominator) * bounded;
        flexKnee_ += (kneeColumn / denominator) * bounded;
        flexHip_ = clampf(flexHip_, 0.f, 0.85f);
        flexKnee_ = clampf(flexKnee_, 0.f, 1.05f);
        if (const RagdollJoint* knee = findJoint(ragdoll, swingKnee)) {
            const auto joint = world.jointState(knee->physics);
            if (joint.valid && joint.rotationRadians.z > joint.swing2Max - 0.08f) {
                flexKnee_ = std::min(flexKnee_, std::max(0.f, flexKnee_ - dt));
            }
        }
        if (const RagdollJoint* hip = findJoint(ragdoll, swingHip)) {
            const auto joint = world.jointState(hip->physics);
            if (joint.valid && joint.rotationRadians.z > joint.swing2Max - 0.08f) {
                flexHip_ = std::min(flexHip_, std::max(0.f, flexHip_ - dt));
            }
        }
        swingTarget_ = swingOrigin_;
        swingTarget_.y = swingTargetHeight_;
        if (upright) {
            peakUprightClearance_ = std::max(peakUprightClearance_, sole - baseSoleY_);
        }
    } else {
        heightError_ = 0.f;
        swingTargetHeight_ = baseSoleY_ + liftHeight_;
        swingTarget_ = swingOrigin_;
        swingTarget_.y = swingTargetHeight_;
    }

    if (phase_ == StepPhase::Aborted) {
        if (horizontal(offset_) < 0.005f && flexHip_ < 0.05f && flexKnee_ < 0.05f) {
            flexHip_ = 0.f;
            flexKnee_ = 0.f;
            offset_ = {};
            publish(balance, {}, {});
            return;
        }
    } else if (phase_ == StepPhase::TransferWeight) {
        if (settleTime_ > 0.20f) {
            phase_ = (mode_ == StepMode::Transfer || mode_ == StepMode::Recover) ? StepPhase::RestoreSupport : StepPhase::StabilizeSupport;
            phaseTime_ = 0.f;
            settleTime_ = 0.f;
            note_ = "weight transfer reached the support target";
        } else if (phaseTime_ > transferDuration_ + 0.5f) {
            fail("weight transfer timed out before the COM reached the support target");
        }
    } else if (phase_ == StepPhase::StabilizeSupport) {
        if (stanceGrounded && (settleTime_ > 0.15f || (phaseTime_ > 0.40f && comError_ < 0.035f && upright))) {
            phase_ = StepPhase::UnloadSwingFoot;
            phaseTime_ = 0.f;
            settleTime_ = 0.f;
            swingPointsAtEntry_ = swingPoints;
            baseSoleY_ = sole;
            note_ = "support settled";
        } else if (phaseTime_ > 1.0f) {
            fail("support did not settle");
        }
    } else if (phase_ == StepPhase::UnloadSwingFoot) {
        const bool unloaded = swingPoints == 0 || (swingPoints < swingPointsAtEntry_ && sole > baseSoleY_ + 0.008f);
        const bool supportOk = stanceGrounded && stancePoints > 0 && upright && comError_ < 0.035f;
        if (!stanceGrounded && phaseTime_ > 0.10f) {
            fail("supporting foot lost contact");
        } else if (mode_ == StepMode::Support) {
            if (phaseTime_ > holdDuration_ + 0.3f) {
                phase_ = StepPhase::RestoreSupport;
                phaseTime_ = 0.f;
                note_ = unloaded ? "single-leg contact estimate, returning" : "swing foot stayed in contact; lift refused";
            }
        } else if (unloaded && supportOk) {
            phase_ = StepPhase::LiftFoot;
            phaseTime_ = 0.f;
            baseSoleY_ = sole;
            swingOrigin_ = swingActual_;
            note_ = "swing foot unloaded; lifting";
        } else if (phaseTime_ > 1.0f) {
            fail("swing foot did not unload; lift refused");
        }
    } else if (phase_ == StepPhase::LiftFoot) {
        if (!stanceGrounded && phaseTime_ > 0.08f) {
            phase_ = StepPhase::LowerFoot;
            phaseTime_ = 0.f;
            note_ = "support lost during the lift; lowering";
        } else if (desiredClearance_ >= liftHeight_ - 1e-3f && sole - baseSoleY_ > liftHeight_ * 0.75f && upright) {
            phase_ = StepPhase::HoldFoot;
            phaseTime_ = 0.f;
            note_ = "clearance reached while upright";
        } else if (phaseTime_ > liftDuration_ + 0.8f) {
            if (peakUprightClearance_ > 0.01f) {
                phase_ = StepPhase::HoldFoot;
                phaseTime_ = 0.f;
                note_ = "lift timed out below the target; holding the measured rise";
            } else {
                phase_ = StepPhase::LowerFoot;
                phaseTime_ = 0.f;
                note_ = "lift timed out with no useful clearance";
            }
        }
    } else if (phase_ == StepPhase::HoldFoot) {
        const float limit = mode_ == StepMode::Lift ? std::max(holdDuration_, 0.2f) + 4.f : std::max(holdDuration_, 0.2f);
        if (phaseTime_ > limit || (mode_ == StepMode::Sequence && phaseTime_ > holdDuration_)) {
            phase_ = StepPhase::LowerFoot;
            phaseTime_ = 0.f;
            note_ = "lowering";
        }
    } else if (phase_ == StepPhase::LowerFoot) {
        const bool down = desiredClearance_ < 0.008f && swingTouching && sole < baseSoleY_ + 0.02f;
        if (down) {
            phase_ = StepPhase::RestoreSupport;
            phaseTime_ = 0.f;
            flexHip_ = 0.f;
            flexKnee_ = 0.f;
            note_ = "swing foot contact returned";
        } else if (phaseTime_ > liftDuration_ + 0.8f) {
            phase_ = StepPhase::RestoreSupport;
            phaseTime_ = 0.f;
            note_ = swingTouching ? "lower timed out with the foot touching" : "lower timed out with the foot still up";
        }
    } else if (phase_ == StepPhase::RestoreSupport) {
        const bool restored = horizontal(offset_) < 0.008f && flexKnee_ < 0.08f && state.leftTouching && state.rightTouching;
        if (restored) {
            phase_ = StepPhase::Completed;
            phaseTime_ = 0.f;
            offset_ = {};
            flexHip_ = 0.f;
            flexKnee_ = 0.f;
            note_ = failure_.empty() ? "returned to two-foot balance" : failure_;
            publish(balance, {}, {});
            return;
        }
        if (phaseTime_ > transferDuration_ + 0.8f) {
            phase_ = StepPhase::Completed;
            offset_ = {};
            flexHip_ = 0.f;
            flexKnee_ = 0.f;
            note_ = "restore timed out; bias cleared";
            publish(balance, {}, {});
            return;
        }
    }

    if (phase_ == StepPhase::Idle || phase_ == StepPhase::Completed || phase_ == StepPhase::Falling) {
        publish(balance, {}, {});
        return;
    }
    if (phase_ == StepPhase::Aborted && horizontal(offset_) < 0.005f && flexHip_ < 0.05f && flexKnee_ < 0.05f) {
        publish(balance, {}, {});
        return;
    }

    hipSwing1_ = clampf(mirroredHipSwing(lateralError) + hipIntegral_, -0.30f, 0.30f);
    std::unordered_map<std::string, JointBias> bias;
    JointBias hip1;
    hip1.euler.y = hipSwing1_;
    JointBias hip2;
    hip2.euler.y = -hipSwing1_;
    bias.emplace("hip_1", hip1);
    bias.emplace("hip_2", hip2);
    if (flexHip_ > 1e-4f || flexKnee_ > 1e-4f || std::fabs(solePitch) > 0.02f) {
        JointBias hip = bias[swingHip];
        hip.euler.z += flexHip_;
        bias[swingHip] = hip;
        JointBias knee;
        knee.euler.z = flexKnee_;
        bias.emplace(swingKnee, knee);
        JointBias ankle;
        ankle.euler.z = clampf(-solePitch, -0.2f, 0.2f);
        bias.emplace(swingAnkle, ankle);
    }
    if (note_.empty() || note_ == "started") {
        note_ = std::string(stepPhaseName(phase_)) + (supportRight_ ? " support right" : " support left");
    }
    publish(balance, offset_, bias);
}

}  // namespace mt
