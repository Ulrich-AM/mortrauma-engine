#include "mortrauma/rig/HumanoidConfig.hpp"

#include "mortrauma/core/Log.hpp"
#include "mortrauma/core/Math.hpp"

#include <nlohmann/json.hpp>

#include <fstream>

namespace mt {
namespace {

float degreesOr(const nlohmann::json& value, const char* key, float fallback) {
    if (!value.contains(key) || !value[key].is_number()) {
        return fallback;
    }
    return radians(value[key].get<float>());
}

}  // namespace

const BodyConfig* CharacterConfig::findBody(const std::string& id) const {
    for (const auto& body : bodies) {
        if (body.id == id) {
            return &body;
        }
    }
    return nullptr;
}

CharacterConfig loadCharacterConfig(const std::string& path) {
    CharacterConfig config;
    std::ifstream input(path);
    if (!input) {
        config.error = "Could not open character configuration: " + path;
        Log::error(config.error);
        return config;
    }
    nlohmann::json json;
    try {
        input >> json;
    } catch (const std::exception& exception) {
        config.error = std::string("Invalid character JSON: ") + exception.what();
        Log::error(config.error);
        return config;
    }
    config.name = json.value("name", "humanoid");
    if (!json.contains("bodies") || !json["bodies"].is_array() || !json.contains("joints") || !json["joints"].is_array()) {
        config.error = "Character configuration is missing bodies or joints.";
        Log::error(config.error);
        return config;
    }
    for (const auto& item : json["bodies"]) {
        BodyConfig body;
        body.id = item.value("id", "");
        body.name = item.value("name", body.id);
        body.primaryBone = item.value("primary_bone", "");
        body.endBone = item.value("end_bone", "");
        body.endExtension = item.value("end_extension", 0.04f);
        body.mass = item.value("mass", 1.f);
        body.radius = item.value("radius", 0.f);
        body.minRadius = item.value("min_radius", 0.045f);
        body.maxRadius = item.value("max_radius", 0.2f);
        body.friction = item.value("friction", 0.7f);
        body.restitution = item.value("restitution", 0.02f);
        const std::string collider = item.value("collider", "auto");
        if (collider == "box") {
            body.collider = ColliderKind::Box;
        } else if (collider == "capsule") {
            body.collider = ColliderKind::Capsule;
        } else {
            body.collider = ColliderKind::Auto;
        }
        if (item.contains("bones")) {
            for (const auto& bone : item["bones"]) {
                body.bones.push_back(bone.get<std::string>());
            }
        }
        if (body.id.empty() || body.primaryBone.empty() || body.mass <= 0.f) {
            config.error = "Body configuration '" + body.id + "' is incomplete or has a non-positive mass.";
            Log::error(config.error);
            return config;
        }
        config.bodies.push_back(std::move(body));
    }
    for (const auto& item : json["joints"]) {
        JointConfig joint;
        joint.id = item.value("id", "");
        joint.name = item.value("name", joint.id);
        joint.parent = item.value("parent", "");
        joint.child = item.value("child", "");
        const std::string swing = item.value("swing", "pyramid");
        joint.swing = swing == "cone" ? SwingKind::Cone : SwingKind::Pyramid;
        const std::string orient = item.value("orient", "none");
        if (orient == "elbow") {
            joint.orient = BendOrient::Elbow;
        } else if (orient == "knee") {
            joint.orient = BendOrient::Knee;
        } else if (orient == "hip") {
            joint.orient = BendOrient::Hip;
        }
        const auto& limits = item.contains("limits_deg") ? item["limits_deg"] : nlohmann::json::object();
        joint.twistMin = degreesOr(limits, "twist_min", -10.f);
        joint.twistMax = degreesOr(limits, "twist_max", 10.f);
        joint.swing1Min = degreesOr(limits, "swing1_min", -20.f);
        joint.swing1Max = degreesOr(limits, "swing1_max", 20.f);
        joint.swing2Min = degreesOr(limits, "swing2_min", -20.f);
        joint.swing2Max = degreesOr(limits, "swing2_max", 20.f);
        if (joint.twistMin > joint.twistMax || joint.swing1Min > joint.swing1Max || joint.swing2Min > joint.swing2Max) {
            config.error = "Joint '" + joint.id + "' has inverted limits.";
            Log::error(config.error);
            return config;
        }
        if (joint.parent.empty() || joint.child.empty()) {
            config.error = "Joint '" + joint.id + "' is missing a parent or child body.";
            Log::error(config.error);
            return config;
        }
        config.joints.push_back(std::move(joint));
    }
    return config;
}

CharacterProfile loadCharacterProfile(const std::string& path) {
    CharacterProfile profile;
    std::ifstream input(path);
    if (!input) {
        profile.error = "Could not open character profile: " + path;
        Log::error(profile.error);
        return profile;
    }
    try {
        nlohmann::json json;
        input >> json;
        profile.id = json.value("id", "");
        profile.label = json.value("label", profile.id);
        profile.model = json.value("model", "");
        profile.rig = json.value("rig", "");
        profile.balancePreset = json.value("balance_preset", profile.balancePreset);
        profile.motors = json.value("motors", profile.motors);
        profile.notes = json.value("notes", "");
    } catch (const std::exception& exception) {
        profile.error = std::string("Invalid character profile: ") + exception.what();
        Log::error(profile.error);
        return profile;
    }
    if (profile.id.empty() || profile.model.empty() || profile.rig.empty()) {
        profile.error = "Character profile is missing an id, model, or rig.";
        Log::error(profile.error);
    }
    return profile;
}

JointPreset loadJointPreset(const std::string& path, std::string& error) {
    JointPreset preset;
    std::ifstream input(path);
    if (!input) {
        error = "Could not open preset: " + path;
        Log::error(error);
        return preset;
    }
    try {
        nlohmann::json json;
        input >> json;
        preset.name = json.value("name", preset.name);
        preset.linearDamping = json.value("linear_damping", preset.linearDamping);
        preset.angularDamping = json.value("angular_damping", preset.angularDamping);
        preset.angularFriction = json.value("angular_friction", preset.angularFriction);
        preset.limitScale = json.value("limit_scale", preset.limitScale);
    } catch (const std::exception& exception) {
        error = std::string("Invalid preset JSON: ") + exception.what();
        Log::error(error);
    }
    if (preset.linearDamping < 0.f || preset.angularDamping < 0.f || preset.angularFriction < 0.f || preset.limitScale <= 0.f) {
        error = "Preset '" + preset.name + "' contains a negative damping, friction, or limit scale.";
        Log::error(error);
    }
    return preset;
}

bool saveJointPreset(const std::string& path, const JointPreset& preset, std::string& error) {
    if (preset.linearDamping < 0.f || preset.angularDamping < 0.f || preset.angularFriction < 0.f || preset.limitScale <= 0.f ||
        preset.linearDamping > 0.95f || preset.angularDamping > 0.95f) {
        error = "Preset values are outside the supported range.";
        Log::error(error);
        return false;
    }
    nlohmann::json json;
    json["name"] = preset.name;
    json["linear_damping"] = preset.linearDamping;
    json["angular_damping"] = preset.angularDamping;
    json["angular_friction"] = preset.angularFriction;
    json["limit_scale"] = preset.limitScale;
    json["note"] = "Passive compliance only. Motors that pull toward the bind pose are not used.";
    std::ofstream output(path);
    if (!output) {
        error = "Could not write preset: " + path;
        Log::error(error);
        return false;
    }
    output << json.dump(2) << '\n';
    return true;
}

}  // namespace mt
