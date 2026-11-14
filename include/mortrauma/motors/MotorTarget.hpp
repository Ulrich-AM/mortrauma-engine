#pragma once

#include "mortrauma/core/Math.hpp"

#include <string>

namespace mt {

// One joint-relative setpoint. The quaternion is the constraint orientation
// (body-1 constraint space), never a world transform.
struct MotorTarget {
    std::string jointId;
    Quat orientation{};
};

}  // namespace mt
