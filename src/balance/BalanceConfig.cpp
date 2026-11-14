#include "mortrauma/balance/BalanceConfig.hpp"

#include "mortrauma/core/Paths.hpp"

#include <nlohmann/json.hpp>

#include <fstream>

namespace mt {
namespace {

float numberOr(const nlohmann::json& json, const char* key, float fallback) {
    if (!json.contains(key) || !json[key].is_number()) {
        return fallback;
    }
    return json[key].get<float>();
}

}  // namespace

BalanceConfig BalanceConfigLoader::preset(const std::string& name, std::string& error) {
    BalanceConfig config;
    config.name = name;
    const auto path = findAsset("configurations/balance/" + name + ".json");
    if (path.empty()) {
        error = "Balance preset '" + name + "' was not found.";
        return config;
    }
    std::ifstream input(path);
    if (!input) {
        error = "Balance preset '" + name + "' could not be opened.";
        return config;
    }
    nlohmann::json json;
    try {
        input >> json;
    } catch (const std::exception& exception) {
        error = exception.what();
        return config;
    }
    config.name = json.value("name", name);
    config.balanceStrength = numberOr(json, "balance_strength", config.balanceStrength);
    config.ankleCorrection = numberOr(json, "ankle_correction", config.ankleCorrection);
    config.hipCorrection = numberOr(json, "hip_correction", config.hipCorrection);
    config.torsoUpright = numberOr(json, "torso_upright", config.torsoUpright);
    config.comVelocityDamping = numberOr(json, "com_velocity_damping", config.comVelocityDamping);
    config.recoveryRadiusM = numberOr(json, "recovery_radius_m", config.recoveryRadiusM);
    config.ankleGainRadPerM = numberOr(json, "ankle_gain_rad_per_m", config.ankleGainRadPerM);
    config.ankleDampingRadPerMps = numberOr(json, "ankle_damping_rad_per_mps", config.ankleDampingRadPerMps);
    config.hipGainRadPerM = numberOr(json, "hip_gain_rad_per_m", config.hipGainRadPerM);
    config.hipDampingRadPerMps = numberOr(json, "hip_damping_rad_per_mps", config.hipDampingRadPerMps);
    config.torsoGain = numberOr(json, "torso_gain", config.torsoGain);
    config.torsoDamping = numberOr(json, "torso_damping", config.torsoDamping);
    config.kneeFlexionDeg = numberOr(json, "knee_flexion_deg", config.kneeFlexionDeg);
    config.hipFlexionDeg = numberOr(json, "hip_flexion_deg", config.hipFlexionDeg);
    config.ankleFlexionDeg = numberOr(json, "ankle_flexion_deg", config.ankleFlexionDeg);
    config.fallingActivation = numberOr(json, "falling_activation", config.fallingActivation);
    config.fallTiltRad = numberOr(json, "fall_tilt_rad", config.fallTiltRad);
    config.minStateSeconds = numberOr(json, "min_state_seconds", config.minStateSeconds);
    config.maxTargetRate = numberOr(json, "max_target_rate", config.maxTargetRate);
    config.motorFrequencyHz = numberOr(json, "motor_frequency_hz", config.motorFrequencyHz);
    error.clear();
    return config;
}

}  // namespace mt
