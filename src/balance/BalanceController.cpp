#include "mortrauma/balance/BalanceController.hpp"

#include "mortrauma/balance/StandingPose.hpp"

#include <algorithm>
#include <cmath>

namespace mt {
namespace {

float clampAxis(float value, float minimum, float maximum) {
    const float margin = radians(1.5f);
    if (maximum - minimum <= margin * 2.f + 0.01f) {
        return clampf(value, minimum, maximum);
    }
    return clampf(value, minimum + margin, maximum - margin);
}

Vec3 clampEuler(Vec3 euler, const RagdollJoint& joint) {
    euler.x = clampAxis(euler.x, joint.base.twistMin, joint.base.twistMax);
    euler.y = clampAxis(euler.y, joint.base.swing1Min, joint.base.swing1Max);
    euler.z = clampAxis(euler.z, joint.base.swing2Min, joint.base.swing2Max);
    return euler;
}

}  // namespace

BalanceAxes balanceCorrection(const BalanceConfig& config, float forward, float lateral, float forwardRate, float lateralRate, bool ankles, bool hips) {
    BalanceAxes axes;
    const float damp = config.comVelocityDamping;
    if (ankles) {
        axes.ankleSwing2 = -(config.ankleGainRadPerM * config.ankleCorrection) * (forward + config.ankleDampingRadPerMps * damp * forwardRate);
        axes.ankleSwing1 = -(config.ankleGainRadPerM * config.ankleCorrection) * (lateral + config.ankleDampingRadPerMps * damp * lateralRate);
    }
    if (hips) {
        axes.hipSwing2 = -(config.hipGainRadPerM * config.hipCorrection) * (forward + config.hipDampingRadPerMps * damp * forwardRate);
        axes.hipSwing1 = -(config.hipGainRadPerM * config.hipCorrection) * (lateral + config.hipDampingRadPerMps * damp * lateralRate);
    }
    return axes;
}

void BalanceController::setConfig(const BalanceConfig& config) { config_ = config; }

bool BalanceController::loadPreset(const std::string& name, std::string& error) {
    const BalanceConfig loaded = BalanceConfigLoader::preset(name, error);
    if (!error.empty()) {
        return false;
    }
    config_ = loaded;
    return true;
}

void BalanceController::setEnabled(bool enabled, MotorController& motors) {
    if (!enabled) {
        enabled_ = false;
        state_.phase = BalancePhase::Inactive;
        state_.strategy = "none";
        timeInPhase_ = 0.f;
        motors.clearMotorTask();
        return;
    }
    if (!enabled_) {
        state_.phase = BalancePhase::Initializing;
        timeInPhase_ = 0.f;
        commanded_.clear();
        anklesSettled_ = false;
        fellLatch_ = false;
        stoodSeconds_ = 0.f;
    }
    enabled_ = true;
}

void BalanceController::setStepBias(Vec3 referenceOffset, std::unordered_map<std::string, JointBias> bias) {
    stepOffset_ = referenceOffset;
    stepBias_ = std::move(bias);
    state_.balanceReference = state_.balanceReference + stepOffset_;
}

void BalanceController::setReference(std::vector<MotorTaskCommand> reference) {
    reference_.clear();
    for (const auto& command : reference) {
        reference_[command.jointId] = command.target;
    }
    commanded_.clear();
}

void BalanceController::resetState() {
    timeInPhase_ = 0.f;
    commanded_.clear();
    anklesSettled_ = false;
    fellLatch_ = false;
    stoodSeconds_ = 0.f;
    if (enabled_) {
        state_.phase = BalancePhase::Initializing;
    } else {
        state_.phase = BalancePhase::Inactive;
    }
}

Quat BalanceController::referenceFor(const RagdollJoint& joint) const {
    const auto found = reference_.find(joint.id);
    if (found != reference_.end() && finite(found->second)) {
        return normalize(found->second);
    }
    return standingReference(joint, config_);
}

void BalanceController::transition(BalancePhase next, bool force) {
    if (next == state_.phase) {
        return;
    }
    if (!force && timeInPhase_ < config_.minStateSeconds) {
        return;
    }
    state_.phase = next;
    timeInPhase_ = 0.f;
}

void BalanceController::pushHistory() {
    BalanceSample sample;
    sample.time = diagnostics_.time;
    const Vec3 horizontal = state_.com - state_.balanceReference;
    sample.comForward = dot(Vec3{horizontal.x, 0.f, horizontal.z}, state_.facing);
    sample.comSpeed = length(Vec3{state_.comVelocity.x, 0.f, state_.comVelocity.z});
    sample.torsoTilt = state_.torsoTilt;
    sample.ankleEffort = state_.ankleEffort;
    sample.hipEffort = state_.hipEffort;
    sample.phase = float(int(state_.phase));
    diagnostics_.history.push_back(sample);
    constexpr size_t kHistory = 360;
    if (diagnostics_.history.size() > kHistory) {
        diagnostics_.history.erase(diagnostics_.history.begin(), diagnostics_.history.begin() + int(diagnostics_.history.size() - kHistory));
    }
}

void BalanceController::observe(const PhysicsWorld& world, const Ragdoll& ragdoll, float dt) {
    const float step = std::isfinite(dt) && dt > 0.f ? dt : 0.f;
    lastDt_ = step;
    diagnostics_.time += step;
    timeInPhase_ += step;
    BalanceState measured = estimator_.measure(world, ragdoll, config_.equilibriumLeadM);
    measured.phase = enabled_ ? state_.phase : BalancePhase::Inactive;
    if (!enabled_) {
        measured.strategy = "none";
        measured.ankleAuthority = false;
        measured.hipAuthority = false;
        state_ = measured;
        pushHistory();
        return;
    }
    if (!measured.finite) {
        transition(BalancePhase::Falling, true);
        measured.phase = state_.phase;
        measured.strategy = "passive";
        state_ = measured;
        pushHistory();
        return;
    }

    const bool supported = measured.leftGrounded || measured.rightGrounded;
    const bool upright = measured.torsoTilt < 0.35f && measured.pelvisTilt < 0.55f;
    const bool settled = measured.captureDistance < 0.03f && length(measured.comVelocity) < 0.35f && measured.torsoTilt < 0.22f;
    const bool lost = !supported;
    // Capture distance on a two-point capsule support leaves the segment as
    // soon as the body has any horizontal speed. That is a reason to use the
    // hip strategy, not a reason to drop motor torque while the torso is
    // still upright and a foot is on the ground.
    const bool tipping = measured.torsoTilt > config_.fallTiltRad || measured.com.y < 0.45f ||
                         (measured.captureDistance > config_.recoveryRadiusM && measured.torsoTilt > 0.55f);

    switch (state_.phase) {
        case BalancePhase::Inactive:
            transition(BalancePhase::Initializing, true);
            break;
        case BalancePhase::Initializing:
            if (lost && timeInPhase_ > 0.25f) {
                transition(BalancePhase::Falling, false);
            } else if (supported && upright) {
                transition(BalancePhase::Standing, false);
            } else if (timeInPhase_ > 0.8f) {
                transition(tipping || lost ? BalancePhase::Falling : BalancePhase::Recovering, false);
            }
            break;
        case BalancePhase::Standing:
            if (lost || tipping) {
                transition(BalancePhase::Falling, false);
            } else if (!settled) {
                transition(BalancePhase::Recovering, false);
            }
            break;
        case BalancePhase::Recovering:
            if (lost || tipping) {
                transition(BalancePhase::Falling, false);
            } else if (settled && supported) {
                transition(BalancePhase::Standing, false);
            }
            break;
        case BalancePhase::Falling:
            if (measured.torsoTilt > 0.8f && std::fabs(measured.comVelocity.y) < 0.45f && measured.com.y < 0.55f) {
                transition(BalancePhase::Grounded, false);
            }
            break;
        case BalancePhase::Grounded:
            break;
    }
    measured.phase = state_.phase;

    Vec3 referencePoint = measured.comProjection;
    if (config_.equilibriumReference && measured.equilibriumValid) {
        referencePoint = measured.equilibriumTarget;
    } else if (length(measured.supportCenter) > 1e-4f || supported) {
        referencePoint = measured.supportCenter;
    }
    measured.balanceReference = referencePoint;
    const Vec3 error = measured.com - referencePoint;
    const float forward = dot(Vec3{error.x, 0.f, error.z}, measured.facing);
    const float lateral = dot(Vec3{error.x, 0.f, error.z}, measured.right);
    measured.ankleAuthority = supported && state_.phase != BalancePhase::Grounded && state_.phase != BalancePhase::Falling;
    measured.hipAuthority = supported && measured.ankleAuthority &&
                            (std::fabs(forward) > 0.03f || std::fabs(lateral) > 0.03f || measured.torsoTilt > 0.12f ||
                             measured.captureDistance > 0.02f);
    if (state_.phase == BalancePhase::Falling || state_.phase == BalancePhase::Grounded || !supported) {
        measured.ankleAuthority = false;
    }
    if (!supported || state_.phase == BalancePhase::Grounded) {
        measured.hipAuthority = false;
    }
    if (measured.ankleAuthority && !measured.hipAuthority) {
        measured.strategy = "ankle";
    } else if (measured.hipAuthority) {
        measured.strategy = "hip";
    } else if (state_.phase == BalancePhase::Falling) {
        measured.strategy = config_.fallingActivation > 0.02f ? "reduced" : "passive";
    } else if (state_.phase == BalancePhase::Grounded) {
        measured.strategy = "passive";
    } else {
        measured.strategy = "hold";
    }
    const float sign = -1.f;
    measured.correction = measured.facing * (sign * forward) + measured.right * (sign * lateral);
    // The unloaded ankle angle is not the angle the floor will allow. After the
    // feet take weight, hold the loaded angle so the ankle budget is free for
    // balance instead of being spent prying the sole off the floor.
    if (!anklesSettled_ && timeInPhase_ > 0.12f && (measured.leftGrounded || measured.rightGrounded) &&
        (state_.phase == BalancePhase::Initializing || state_.phase == BalancePhase::Standing || state_.phase == BalancePhase::Recovering)) {
        for (const auto& joint : ragdoll.joints) {
            if (joint.id != "ankle_1" && joint.id != "ankle_2") {
                continue;
            }
            const Quat loaded = world.jointState(joint.physics).constraintRotation;
            if (finite(loaded)) {
                reference_[joint.id] = normalize(loaded);
                commanded_.erase(joint.id);
            }
        }
        anklesSettled_ = true;
    }
    const bool down = measured.phase == BalancePhase::Falling || measured.phase == BalancePhase::Grounded || measured.com.y < 0.75f ||
                      measured.torsoTilt > 0.7f;
    if (!fellLatch_) {
        if (down && diagnostics_.time > 0.02f) {
            fellLatch_ = true;
        } else {
            stoodSeconds_ = diagnostics_.time;
        }
    }
    measured.stoodSeconds = stoodSeconds_;
    if (measured.phase == BalancePhase::Falling || measured.phase == BalancePhase::Grounded || measured.torsoTilt > 0.7f) {
        measured.instability = "falling";
    } else if (measured.estimatedAnkleMoment > 12.f) {
        measured.instability = "pitch moment above 12 N·m; hips carry the residual";
    } else if (measured.saturatedMotors > 0 &&
               (measured.phase == BalancePhase::Standing || measured.phase == BalancePhase::Recovering)) {
        measured.instability = "standing; an ankle or hip motor is saturated, pitch moment is within 12 N·m";
    }
    state_ = measured;
    pushHistory();
}

void BalanceController::command(const Ragdoll& ragdoll, MotorController& motors) {
    if (!enabled_ || !motors.bound()) {
        if (motors.motorTaskActive()) {
            motors.clearMotorTask();
        }
        return;
    }
    const float strength = clampf(config_.balanceStrength, 0.f, 1.5f);
    float activation = strength;
    if (state_.phase == BalancePhase::Falling) {
        activation = strength * clampf(config_.fallingActivation, 0.f, 1.f);
    } else if (state_.phase == BalancePhase::Grounded || state_.phase == BalancePhase::Inactive || !state_.finite) {
        activation = 0.f;
    }
    const bool correct = state_.finite && (state_.phase == BalancePhase::Initializing || state_.phase == BalancePhase::Standing ||
                                           state_.phase == BalancePhase::Recovering);
    const Vec3 error = state_.com - state_.balanceReference;
    const float forward = dot(Vec3{error.x, 0.f, error.z}, state_.facing);
    const float lateral = dot(Vec3{error.x, 0.f, error.z}, state_.right);
    const float forwardRate = dot(state_.comVelocity, state_.facing);
    const float lateralRate = dot(state_.comVelocity, state_.right);
    // Positive swing2 flexes the hip and, with the foot planted, pitches the
    // torso further forward. A forward COM error therefore commands extension.
    const bool hips = correct && (state_.leftGrounded || state_.rightGrounded);
    const BalanceAxes axes = balanceCorrection(config_, forward, lateral, forwardRate, lateralRate, state_.ankleAuthority && correct, hips);
    const float ankleForward = axes.ankleSwing2;
    const float ankleLateral = axes.ankleSwing1;
    const float hipForward = axes.hipSwing2;
    const float hipLateral = axes.hipSwing1;
    const float torso = correct ? config_.torsoGain * config_.torsoUpright : 0.f;
    const float leanForward = dot(state_.torsoUp, state_.facing);
    const float leanLateral = dot(state_.torsoUp, state_.right);

    std::vector<MotorTaskCommand> task;
    task.reserve(ragdoll.joints.size());
    for (const auto& joint : ragdoll.joints) {
        const Quat reference = referenceFor(joint);
        Vec3 delta{};
        const bool footPlanted = (joint.id == "ankle_1" && state_.rightGrounded) || (joint.id == "ankle_2" && state_.leftGrounded);
        if ((joint.id == "ankle_1" || joint.id == "ankle_2") && footPlanted) {
            delta.z = clampf(ankleForward, -0.4f, 0.4f);
            delta.y = clampf(ankleLateral, -0.3f, 0.3f);
        } else if (joint.id == "hip_1" || joint.id == "hip_2") {
            delta.z = clampf(hipForward, -0.45f, 0.45f);
            // Both hips share one facing swing1 axis, so this sign is not a
            // mirrored abduction. Quiet standing keeps the shared sign.
            // Weight transfer adds the opposite sign on hip_2 as a step bias.
            delta.y = clampf(hipLateral, -0.35f, 0.35f);
        } else if (joint.id == "spine_stomach" || joint.id == "spine_chest" || joint.id == "spine_neck") {
            const float share = joint.id == "spine_stomach" ? 0.45f : (joint.id == "spine_chest" ? 0.35f : 0.2f);
            delta.z = clampf(-torso * leanForward * share, -0.22f, 0.22f);
            delta.y = clampf(-torso * leanLateral * share, -0.22f, 0.22f);
        }
        const auto bias = stepBias_.find(joint.id);
        if (bias != stepBias_.end()) {
            delta = bias->second.replace ? bias->second.euler : delta + bias->second.euler;
        }
        Quat target = reference;
        if (std::fabs(delta.x) + std::fabs(delta.y) + std::fabs(delta.z) > 1e-5f) {
            const Vec3 euler = clampEuler(constraintEulerRadians(reference) + delta, joint);
            target = quatFromEulerXYZ(euler.x, euler.y, euler.z);
        }
        const auto previous = commanded_.find(joint.id);
        const Quat from = previous == commanded_.end() ? reference : previous->second;
        const float angle = quatAngle(mul(conjugate(from), target));
        const float maxStep = std::max(config_.maxTargetRate, 0.f) * std::max(lastDt_, 0.f);
        const float blend = angle < 1e-4f ? 1.f : std::min(1.f, maxStep / angle);
        const Quat commanded = slerp(from, target, blend);
        commanded_[joint.id] = commanded;
        MotorTaskCommand entry;
        entry.jointId = joint.id;
        entry.target = commanded;
        entry.activation = activation;
        const bool weightBearing = joint.id == "hip_1" || joint.id == "hip_2" || joint.id == "knee_1" || joint.id == "knee_2" ||
                                   joint.id == "hip_root_1" || joint.id == "hip_root_2" || joint.id == "spine_stomach" ||
                                   joint.id == "spine_chest" || joint.id == "spine_neck";
        const bool ankle = joint.id == "ankle_1" || joint.id == "ankle_2";
        // Ankles and arms stay compliant. A high bandwidth on a 12 N·m ankle
        // saturates against the floor and shoves the whole body.
        entry.frequencyHz = weightBearing ? config_.motorFrequencyHz : (ankle ? std::min(config_.motorFrequencyHz, 40.f) : 4.f);
        task.push_back(entry);
    }
    motors.setMotorTask(std::move(task));
}

}  // namespace mt
