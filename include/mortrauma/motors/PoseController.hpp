#pragma once

#include "mortrauma/motors/MotorController.hpp"
#include "mortrauma/poses/PoseLibrary.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace mt {

// Interpolates constraint-space targets. It never writes a body transform
// and never calls the physics motor API. MotorController::apply does that.
class PoseController {
public:
    void begin(const Pose& pose, const MotorController& motors, float durationSeconds);
    void cancel();
    void update(float dt, MotorController& motors);

    bool active() const { return active_; }
    float duration() const { return duration_; }
    float elapsed() const { return elapsed_; }
    // Hard cap on how fast a target quaternion may change, radians per second.
    void setMaxTargetRate(float radiansPerSecond) { maxRate_ = std::max(radiansPerSecond, 0.f); }
    float maxTargetRate() const { return maxRate_; }

private:
    struct Channel {
        std::string id;
        Quat from{};
        Quat to{};
        Quat current{};
    };

    std::vector<Channel> channels_;
    float duration_ = 0.f;
    float elapsed_ = 0.f;
    float maxRate_ = 3.14159265f;
    bool active_ = false;
};

}  // namespace mt
