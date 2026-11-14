#pragma once

#include <string>

namespace mt {

// Controller tuning. Gains are multipliers on documented base rates.
// They are not success guarantees and they cannot raise a joint past its
// muscle-profile torque limit.
struct BalanceConfig {
    std::string name = "Normal";
    // Scales every balance motor activation. 1 uses the active profile torque.
    float balanceStrength = 1.f;
    // Scales ankle, hip, and torso feedback angles. 1 is the preset's base gain.
    float ankleCorrection = 1.f;
    float hipCorrection = 1.f;
    float torsoUpright = 1.f;
    // Scales how strongly center-of-mass velocity is added to the position error.
    float comVelocityDamping = 1.f;
    // Capture point may sit this far outside the raw support hull before the
    // controller treats the disturbance as unrecoverable. This is a decision
    // threshold, not an inflated foot shape. Metres.
    float recoveryRadiusM = 0.18f;
    // Base feedback, radians per metre of horizontal center-of-mass error.
    float ankleGainRadPerM = 2.0f;
    float ankleDampingRadPerMps = 0.45f;
    float hipGainRadPerM = 1.3f;
    float hipDampingRadPerMps = 0.30f;
    float torsoGain = 0.9f;
    float torsoDamping = 0.22f;
    // Standing reference, degrees of anatomical flexion. Knees stay short of
    // the extension stop. Hips are measured from straight down, not the crouch.
    float kneeFlexionDeg = 14.f;
    float hipFlexionDeg = 10.f;
    float ankleFlexionDeg = 0.f;
    // Fraction of balance strength kept while falling. 0 is passive.
    float fallingActivation = 0.2f;
    float fallTiltRad = 0.95f;
    float minStateSeconds = 0.12f;
    // Maximum speed of a balance target, radians per second.
    float maxTargetRate = 2.4f;
    // Balance task frequency. Torque is still the active-profile limit.
    float motorFrequencyHz = 12.f;
};

class BalanceConfigLoader {
public:
    static BalanceConfig preset(const std::string& name, std::string& error);
};

}  // namespace mt
