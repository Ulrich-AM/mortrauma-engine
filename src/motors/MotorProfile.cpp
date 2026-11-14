#include "mortrauma/motors/MotorProfile.hpp"

#include "mortrauma/core/Log.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>

namespace mt {
namespace {

void fillTorque(MotorProfile& profile, float scale) {
    const struct {
        const char* id;
        float torque;
    } rows[] = {
        {"spine_stomach", 55.f}, {"spine_chest", 50.f}, {"spine_neck", 32.f}, {"neck", 12.f},
        {"clavicle_1", 16.f},    {"clavicle_2", 16.f},  {"shoulder_1", 26.f}, {"shoulder_2", 26.f},
        {"elbow_1", 18.f},       {"elbow_2", 18.f},     {"wrist_1", 6.f},     {"wrist_2", 6.f},
        {"hip_root_1", 18.f},    {"hip_root_2", 18.f},  {"hip_1", 50.f},      {"hip_2", 50.f},
        {"knee_1", 35.f},        {"knee_2", 35.f},      {"ankle_1", 12.f},    {"ankle_2", 12.f},
    };
    profile.defaultTorque = 20.f * scale;
    for (const auto& row : rows) {
        profile.torqueNm[row.id] = row.torque * scale;
    }
}

bool finiteRange(float value, float lo, float hi) { return std::isfinite(value) && value >= lo && value <= hi; }

}  // namespace

float MotorProfile::torqueFor(const std::string& jointId) const {
    const auto found = torqueNm.find(jointId);
    return found == torqueNm.end() ? defaultTorque : found->second;
}

MotorProfile builtinPassiveProfile() {
    MotorProfile profile;
    profile.name = "passive";
    profile.frequencyHz = 0.f;
    profile.dampingRatio = 1.f;
    fillTorque(profile, 0.f);
    return profile;
}

MotorProfile builtinCompliantProfile() {
    MotorProfile profile;
    profile.name = "compliant";
    profile.frequencyHz = 1.2f;
    profile.dampingRatio = 1.f;
    fillTorque(profile, 0.25f);
    return profile;
}

MotorProfile builtinActiveProfile() {
    MotorProfile profile;
    profile.name = "active";
    profile.frequencyHz = 3.5f;
    profile.dampingRatio = 1.f;
    fillTorque(profile, 1.f);
    return profile;
}

bool loadMotorProfile(MotorProfile& profile, const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        Log::warn("Motor profile not found, using built-in values: " + path);
        return false;
    }
    nlohmann::json json;
    try {
        input >> json;
    } catch (const std::exception& exception) {
        Log::error(std::string("Motor profile JSON is invalid: ") + exception.what());
        return false;
    }
    if (!json.is_object()) {
        Log::error("Motor profile is not an object: " + path);
        return false;
    }
    const float frequency = json.value("frequency_hz", profile.frequencyHz);
    const float damping = json.value("damping_ratio", profile.dampingRatio);
    if (!finiteRange(frequency, 0.f, 30.f) || !finiteRange(damping, 0.f, 2.f)) {
        Log::error("Motor profile frequency or damping is out of range: " + path);
        return false;
    }
    profile.frequencyHz = frequency;
    profile.dampingRatio = damping;
    if (json.contains("name") && json["name"].is_string()) {
        profile.name = json["name"].get<std::string>();
    }
    if (!json.contains("torque_nm")) {
        return true;
    }
    if (!json["torque_nm"].is_object()) {
        Log::error("Motor profile torque_nm must be an object: " + path);
        return false;
    }
    for (auto it = json["torque_nm"].begin(); it != json["torque_nm"].end(); ++it) {
        if (!it.value().is_number()) {
            Log::error("Motor profile torque for " + it.key() + " is not a number.");
            return false;
        }
        const float torque = it.value().get<float>();
        if (!finiteRange(torque, 0.f, 300.f)) {
            Log::error("Motor profile torque for " + it.key() + " is out of range.");
            return false;
        }
        profile.torqueNm[it.key()] = torque;
    }
    return true;
}

}  // namespace mt
