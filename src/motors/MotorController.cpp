#include "mortrauma/motors/MotorController.hpp"

#include "mortrauma/core/Log.hpp"
#include "mortrauma/core/Paths.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace mt {
namespace {

std::string groupForJoint(const std::string& id) {
    if (id.rfind("spine_", 0) == 0) {
        return "core";
    }
    if (id == "neck") {
        return "neck";
    }
    if (id.rfind("clavicle_", 0) == 0 || id.rfind("shoulder_", 0) == 0) {
        return "shoulders";
    }
    if (id.rfind("elbow_", 0) == 0 || id.rfind("wrist_", 0) == 0) {
        return "arms";
    }
    if (id.rfind("hip_", 0) == 0) {
        return "hips";
    }
    if (id.rfind("knee_", 0) == 0 || id.rfind("ankle_", 0) == 0) {
        return "legs";
    }
    return "other";
}

bool inRange(float value, float lo, float hi) { return std::isfinite(value) && value >= lo && value <= hi; }

}  // namespace

const char* motorModeName(MotorMode mode) {
    switch (mode) {
    case MotorMode::Passive:
        return "passive";
    case MotorMode::Compliant:
        return "compliant";
    case MotorMode::Active:
        return "active";
    case MotorMode::Hold:
        return "hold";
    }
    return "passive";
}

MotorMode motorModeFromName(const std::string& name, bool& ok) {
    ok = true;
    if (name == "passive") {
        return MotorMode::Passive;
    }
    if (name == "compliant") {
        return MotorMode::Compliant;
    }
    if (name == "active") {
        return MotorMode::Active;
    }
    if (name == "hold") {
        return MotorMode::Hold;
    }
    ok = false;
    return MotorMode::Passive;
}

void MotorController::ensureGroups() {
    if (!groups_.empty()) {
        return;
    }
    groups_ = {
        {"core", "Core", 0.f, 1.f},
        {"neck", "Neck", 0.f, 1.f},
        {"shoulders", "Shoulders and clavicles", 0.f, 1.f},
        {"arms", "Arms", 0.f, 1.f},
        {"hips", "Hips", 0.f, 1.f},
        {"legs", "Legs", 0.f, 1.f},
        {"other", "Other", 0.f, 1.f},
    };
}

MotorGroup* MotorController::findGroup(const std::string& id) {
    for (auto& group : groups_) {
        if (group.id == id) {
            return &group;
        }
    }
    return nullptr;
}

bool MotorController::bind(const PhysicsWorld& world, const Ragdoll& ragdoll) {
    ensureGroups();
    joints_.clear();
    index_.clear();
    passive_ = builtinPassiveProfile();
    compliant_ = builtinCompliantProfile();
    active_ = builtinActiveProfile();
    for (const auto& joint : ragdoll.joints) {
        const auto state = world.jointState(joint.physics);
        if (!state.valid || !finite(state.constraintRotation)) {
            Log::error("Cannot bind motor for joint '" + joint.id + "'.");
            joints_.clear();
            bound_ = false;
            return false;
        }
        JointMotor motor;
        motor.id = joint.id;
        motor.group = groupForJoint(joint.id);
        motor.frequencyHz = active_.frequencyHz;
        motor.dampingRatio = active_.dampingRatio;
        motor.torqueLimit = active_.torqueFor(joint.id);
        motor.target = state.constraintRotation;
        motor.neutral = state.constraintRotation;
        index_[motor.id] = int(joints_.size());
        joints_.push_back(motor);
    }
    bound_ = !joints_.empty();
    mode_ = MotorMode::Passive;
    relaxing_ = false;
    return bound_;
}

void MotorController::loadDefaultProfiles() {
    passive_ = builtinPassiveProfile();
    compliant_ = builtinCompliantProfile();
    active_ = builtinActiveProfile();
    if (const auto path = findAsset("configurations/motors/passive.json"); !path.empty()) {
        loadMotorProfile(passive_, path.string());
    }
    if (const auto path = findAsset("configurations/motors/compliant.json"); !path.empty()) {
        loadMotorProfile(compliant_, path.string());
    }
    if (const auto path = findAsset("configurations/motors/active.json"); !path.empty()) {
        loadMotorProfile(active_, path.string());
    }
}

const MotorProfile& MotorController::profile(MotorMode mode) const {
    if (mode == MotorMode::Compliant) {
        return compliant_;
    }
    if (mode == MotorMode::Passive) {
        return passive_;
    }
    return active_;
}

const MotorProfile& MotorController::activeProfile() const { return profile(mode_); }

void MotorController::setMode(MotorMode mode) {
    mode_ = mode;
    if (mode == MotorMode::Passive) {
        relaxing_ = false;
    }
}

void MotorController::setGroupActivation(const std::string& group, float activation) {
    if (!inRange(activation, 0.f, 1.f)) {
        Log::error("Rejected group activation for '" + group + "'.");
        return;
    }
    MotorGroup* slot = findGroup(group);
    if (slot == nullptr) {
        Log::error("Unknown muscle group '" + group + "'.");
        return;
    }
    slot->activation = activation;
    for (auto& joint : joints_) {
        if (joint.group == group && !joint.overrideGroup) {
            joint.activation = activation;
        }
    }
}

void MotorController::setGroupStrength(const std::string& group, float strength) {
    if (!inRange(strength, 0.f, 2.f)) {
        Log::error("Rejected group strength for '" + group + "'.");
        return;
    }
    if (MotorGroup* slot = findGroup(group)) {
        slot->strength = strength;
    }
}

void MotorController::setJointActivation(const std::string& jointId, float activation, bool asOverride) {
    JointMotor* joint = find(jointId);
    if (joint == nullptr || !inRange(activation, 0.f, 1.f)) {
        Log::error("Rejected activation for joint '" + jointId + "'.");
        return;
    }
    joint->activation = activation;
    if (asOverride) {
        joint->overrideGroup = true;
    }
}

void MotorController::setJointGains(const std::string& jointId, float frequencyHz, float dampingRatio, float torqueNm) {
    JointMotor* joint = find(jointId);
    if (joint == nullptr || !inRange(frequencyHz, 0.f, 30.f) || !inRange(dampingRatio, 0.f, 2.f) || !inRange(torqueNm, 0.f, 300.f)) {
        Log::error("Rejected motor gains for joint '" + jointId + "'.");
        return;
    }
    joint->overrideGains = true;
    joint->frequencyHz = frequencyHz;
    joint->dampingRatio = dampingRatio;
    joint->torqueLimit = torqueNm;
}

void MotorController::setUseProfileGains(const std::string& jointId, bool useProfile) {
    if (JointMotor* joint = find(jointId)) {
        joint->overrideGains = !useProfile;
    }
}

void MotorController::setTarget(const std::string& jointId, Quat constraintOrientation) {
    JointMotor* joint = find(jointId);
    if (joint == nullptr || !finite(constraintOrientation) || !(dot(constraintOrientation, constraintOrientation) > 0.5f)) {
        Log::error("Rejected non-finite motor target for '" + jointId + "'.");
        return;
    }
    joint->target = normalize(constraintOrientation);
}

void MotorController::setAllActivations(float activation, bool clearOverrides) {
    if (!inRange(activation, 0.f, 1.f)) {
        Log::error("Rejected activation command.");
        return;
    }
    for (auto& group : groups_) {
        group.activation = activation;
    }
    for (auto& joint : joints_) {
        if (clearOverrides) {
            joint.overrideGroup = false;
        }
        if (clearOverrides || !joint.overrideGroup) {
            joint.activation = activation;
        }
    }
}

void MotorController::hold(const PhysicsWorld& world, const Ragdoll& ragdoll, const std::vector<std::string>& jointIds) {
    const bool all = jointIds.empty();
    for (const auto& ragdollJoint : ragdoll.joints) {
        const bool selected = all || std::find(jointIds.begin(), jointIds.end(), ragdollJoint.id) != jointIds.end();
        if (!selected) {
            continue;
        }
        JointMotor* motor = find(ragdollJoint.id);
        const auto state = world.jointState(ragdollJoint.physics);
        if (motor == nullptr || !state.valid || !finite(state.constraintRotation)) {
            Log::error("Hold pose could not read joint '" + ragdollJoint.id + "'.");
            continue;
        }
        motor->target = normalize(state.constraintRotation);
        motor->activation = 1.f;
        motor->overrideGroup = !all;
    }
    mode_ = MotorMode::Hold;
    relaxing_ = false;
}

void MotorController::relaxAll() {
    relaxing_ = false;
    setAllActivations(0.f, true);
}

void MotorController::activateAll() {
    if (mode_ == MotorMode::Passive) {
        mode_ = MotorMode::Active;
    }
    setAllActivations(1.f, true);
}

void MotorController::restoreDefaults() {
    mode_ = MotorMode::Passive;
    relaxing_ = false;
    for (auto& group : groups_) {
        group.activation = 0.f;
        group.strength = 1.f;
    }
    for (auto& joint : joints_) {
        joint.activation = 0.f;
        joint.overrideGroup = false;
        joint.overrideGains = false;
        joint.frequencyHz = active_.frequencyHz;
        joint.dampingRatio = active_.dampingRatio;
        joint.torqueLimit = active_.torqueFor(joint.id);
        joint.target = joint.neutral;
    }
}

void MotorController::beginRelaxation(float seconds) {
    float peak = 0.f;
    for (const auto& joint : joints_) {
        peak = std::max(peak, joint.activation);
    }
    if (!(seconds > 0.f) || !std::isfinite(seconds) || peak < 0.02f) {
        relaxAll();
        return;
    }
    relaxRate_ = peak / seconds;
    relaxing_ = true;
}

void MotorController::updateRelaxation(float dt) {
    if (!relaxing_ || !(dt > 0.f) || !std::isfinite(dt)) {
        return;
    }
    const float step = relaxRate_ * dt;
    bool any = false;
    for (auto& group : groups_) {
        group.activation = std::max(0.f, group.activation - step);
    }
    for (auto& joint : joints_) {
        joint.activation = std::max(0.f, joint.activation - step);
        if (joint.activation >= 0.02f) {
            any = true;
        }
    }
    if (!any) {
        relaxing_ = false;
    }
}

bool MotorController::apply(PhysicsWorld& world, const Ragdoll& ragdoll) {
    if (!bound_) {
        Log::error("Motor controller apply called before bind.");
        return false;
    }
    const MotorProfile& gains = activeProfile();
    bool ok = true;
    for (const auto& ragdollJoint : ragdoll.joints) {
        const auto found = index_.find(ragdollJoint.id);
        if (found == index_.end()) {
            Log::error("Motor controller has no state for joint '" + ragdollJoint.id + "'.");
            ok = false;
            continue;
        }
        JointMotor& joint = joints_[found->second];
        const MotorGroup* group = nullptr;
        for (const auto& candidate : groups_) {
            if (candidate.id == joint.group) {
                group = &candidate;
                break;
            }
        }
        const float strength = group != nullptr ? group->strength : 1.f;
        const float activation = clampf(joint.activation, 0.f, 1.f);
        const bool passive = mode_ == MotorMode::Passive || activation < 0.02f;
        AngularMotorCommand command;
        command.enabled = !passive;
        command.targetOrientation = finite(joint.target) ? normalize(joint.target) : Quat::identity();
        command.targetAngularVelocity = {};
        const float frequency = joint.overrideGains ? joint.frequencyHz : gains.frequencyHz;
        const float damping = joint.overrideGains ? joint.dampingRatio : gains.dampingRatio;
        const float torque = joint.overrideGains ? joint.torqueLimit : gains.torqueFor(joint.id);
        command.frequencyHz = frequency * activation;
        command.dampingRatio = damping;
        command.torqueLimit = torque * strength * activation;
        if (!finite(command.targetOrientation) || !std::isfinite(command.frequencyHz) || !std::isfinite(command.torqueLimit)) {
            Log::error("Invalid motor state on '" + joint.id + "'. The joint was left passive.");
            command.enabled = false;
            command.targetOrientation = joint.neutral;
            command.frequencyHz = 0.f;
            command.torqueLimit = 0.f;
            ok = false;
        }
        if (passive) {
            command.enabled = false;
        }
        if (!world.setAngularMotor(ragdollJoint.physics, command)) {
            ok = false;
        }
    }
    return ok;
}

void MotorController::onSimulationReset(PhysicsWorld& world, const Ragdoll& ragdoll) {
    relaxing_ = false;
    mode_ = MotorMode::Passive;
    for (auto& group : groups_) {
        group.activation = 0.f;
    }
    for (const auto& ragdollJoint : ragdoll.joints) {
        JointMotor* joint = find(ragdollJoint.id);
        if (joint == nullptr) {
            continue;
        }
        const auto state = world.jointState(ragdollJoint.physics);
        joint->activation = 0.f;
        joint->overrideGroup = false;
        if (state.valid && finite(state.constraintRotation)) {
            joint->neutral = normalize(state.constraintRotation);
            joint->target = joint->neutral;
        }
    }
    apply(world, ragdoll);
}

const JointMotor* MotorController::find(const std::string& jointId) const {
    const auto found = index_.find(jointId);
    return found == index_.end() ? nullptr : &joints_[found->second];
}

JointMotor* MotorController::find(const std::string& jointId) {
    const auto found = index_.find(jointId);
    return found == index_.end() ? nullptr : &joints_[found->second];
}

Quat MotorController::target(const std::string& jointId) const {
    const JointMotor* joint = find(jointId);
    return joint != nullptr ? joint->target : Quat{};
}

Quat MotorController::neutral(const std::string& jointId) const {
    const JointMotor* joint = find(jointId);
    return joint != nullptr ? joint->neutral : Quat{};
}

bool MotorController::saveConfiguration(const std::string& path, std::string& error) const {
    nlohmann::json json;
    json["version"] = 1;
    json["mode"] = motorModeName(mode_);
    for (const auto& group : groups_) {
        json["groups"][group.id] = {{"activation", group.activation}, {"strength", group.strength}};
    }
    for (const auto& joint : joints_) {
        json["joints"][joint.id] = {{"activation", joint.activation},
                                    {"override_group", joint.overrideGroup},
                                    {"override_gains", joint.overrideGains},
                                    {"frequency_hz", joint.frequencyHz},
                                    {"damping_ratio", joint.dampingRatio},
                                    {"torque_nm", joint.torqueLimit},
                                    {"target", {joint.target.x, joint.target.y, joint.target.z, joint.target.w}}};
    }
    std::ofstream output(path);
    if (!output) {
        error = "Could not write " + path;
        Log::error(error);
        return false;
    }
    output << json.dump(2) << '\n';
    return true;
}

bool MotorController::loadConfiguration(const std::string& path, std::string& error) {
    std::ifstream input(path);
    if (!input) {
        error = "Motor configuration not found: " + path;
        Log::error(error);
        return false;
    }
    nlohmann::json json;
    try {
        input >> json;
    } catch (const std::exception& exception) {
        error = std::string("Motor configuration is invalid: ") + exception.what();
        Log::error(error);
        return false;
    }
    if (!json.is_object() || json.value("version", 0) != 1 || !json.contains("mode") || !json["joints"].is_object()) {
        error = "Motor configuration is missing version, mode or joints.";
        Log::error(error);
        return false;
    }
    bool modeOk = false;
    const MotorMode loadedMode = motorModeFromName(json["mode"].get<std::string>(), modeOk);
    if (!modeOk) {
        error = "Unknown motor mode in " + path;
        Log::error(error);
        return false;
    }
    std::vector<JointMotor> updated = joints_;
    if (json.contains("groups")) {
        if (!json["groups"].is_object()) {
            error = "Motor groups must be an object.";
            Log::error(error);
            return false;
        }
        for (auto it = json["groups"].begin(); it != json["groups"].end(); ++it) {
            const float activation = it.value().value("activation", 0.f);
            const float strength = it.value().value("strength", 1.f);
            if (!inRange(activation, 0.f, 1.f) || !inRange(strength, 0.f, 2.f)) {
                error = "Group '" + it.key() + "' is out of range.";
                Log::error(error);
                return false;
            }
        }
    }
    for (auto it = json["joints"].begin(); it != json["joints"].end(); ++it) {
        auto found = index_.find(it.key());
        if (found == index_.end()) {
            Log::warn("Ignoring unknown joint in motor configuration: " + it.key());
            continue;
        }
        const auto& value = it.value();
        const float activation = value.value("activation", 0.f);
        const float frequency = value.value("frequency_hz", active_.frequencyHz);
        const float damping = value.value("damping_ratio", active_.dampingRatio);
        const float torque = value.value("torque_nm", active_.torqueFor(it.key()));
        if (!inRange(activation, 0.f, 1.f) || !inRange(frequency, 0.f, 30.f) || !inRange(damping, 0.f, 2.f) || !inRange(torque, 0.f, 300.f) ||
            !value.contains("target") || !value["target"].is_array() || value["target"].size() != 4) {
            error = "Joint '" + it.key() + "' has an invalid motor field.";
            Log::error(error);
            return false;
        }
        Quat target{value["target"][0].get<float>(), value["target"][1].get<float>(), value["target"][2].get<float>(),
                    value["target"][3].get<float>()};
        if (!finite(target) || !(dot(target, target) > 0.5f)) {
            error = "Joint '" + it.key() + "' has a non-finite target.";
            Log::error(error);
            return false;
        }
        JointMotor& joint = updated[found->second];
        joint.activation = activation;
        joint.overrideGroup = value.value("override_group", false);
        joint.overrideGains = value.value("override_gains", false);
        joint.frequencyHz = frequency;
        joint.dampingRatio = damping;
        joint.torqueLimit = torque;
        joint.target = normalize(target);
    }
    joints_ = std::move(updated);
    if (json.contains("groups")) {
        for (auto it = json["groups"].begin(); it != json["groups"].end(); ++it) {
            if (MotorGroup* group = findGroup(it.key())) {
                group->activation = it.value().value("activation", group->activation);
                group->strength = it.value().value("strength", group->strength);
            }
        }
    }
    mode_ = loadedMode;
    relaxing_ = false;
    return true;
}

}  // namespace mt
