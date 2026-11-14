#include "mortrauma/motors/PoseController.hpp"

#include "mortrauma/core/Log.hpp"

#include <cmath>

namespace mt {

void PoseController::begin(const Pose& pose, const MotorController& motors, float durationSeconds) {
    channels_.clear();
    if (pose.captureOnSelect || !std::isfinite(durationSeconds) || durationSeconds < 0.f) {
        active_ = false;
        return;
    }
    duration_ = durationSeconds;
    elapsed_ = 0.f;
    active_ = true;
    for (const auto& joint : pose.joints) {
        Channel channel;
        channel.id = joint.jointId;
        channel.from = motors.target(joint.jointId);
        channel.to = joint.orientation;
        if (!finite(channel.from) || !finite(channel.to)) {
            Log::error("Skipped a non-finite pose channel for '" + joint.jointId + "'.");
            continue;
        }
        channel.current = channel.from;
        channels_.push_back(channel);
    }
    if (channels_.empty()) {
        active_ = false;
    }
}

void PoseController::cancel() {
    active_ = false;
    channels_.clear();
    elapsed_ = 0.f;
}

void PoseController::update(float dt, MotorController& motors) {
    if (!active_) {
        return;
    }
    if (!(dt > 0.f) || !std::isfinite(dt)) {
        Log::error("Pose transition ignored a non-finite timestep.");
        return;
    }
    elapsed_ += dt;
    const float timeAlpha = duration_ <= 1e-4f ? 1.f : clampf(elapsed_ / duration_, 0.f, 1.f);
    bool settled = timeAlpha >= 1.f;
    for (auto& channel : channels_) {
        const Quat timed = slerp(channel.from, channel.to, timeAlpha);
        const float remaining = quatAngle(mul(conjugate(channel.current), timed));
        const float maxStep = maxRate_ * dt;
        const float fraction = remaining <= maxStep || remaining < 1e-5f ? 1.f : maxStep / remaining;
        channel.current = slerp(channel.current, timed, fraction);
        if (!finite(channel.current)) {
            Log::error("Pose transition became non-finite for '" + channel.id + "'. The target was held.");
            channel.current = channel.to;
        }
        motors.setTarget(channel.id, channel.current);
        if (quatAngle(mul(conjugate(channel.current), channel.to)) > radians(0.5f)) {
            settled = false;
        }
    }
    if (settled) {
        active_ = false;
    }
}

}  // namespace mt
