#pragma once

#include <string>
#include <unordered_map>

namespace mt {

// Active-motor gains. Passive joint friction is not stored here.
struct MotorProfile {
    std::string name;
    float frequencyHz = 0.f;
    float dampingRatio = 1.f;
    float defaultTorque = 20.f;
    std::unordered_map<std::string, float> torqueNm;

    float torqueFor(const std::string& jointId) const;
};

MotorProfile builtinPassiveProfile();
MotorProfile builtinCompliantProfile();
MotorProfile builtinActiveProfile();

// Missing file keeps `profile` unchanged and returns false. Invalid numbers
// are rejected and logged. Unknown joint ids are ignored.
bool loadMotorProfile(MotorProfile& profile, const std::string& path);

}  // namespace mt
