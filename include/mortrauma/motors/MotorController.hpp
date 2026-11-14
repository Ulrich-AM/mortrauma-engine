#pragma once

#include "mortrauma/motors/MotorProfile.hpp"
#include "mortrauma/motors/MotorTarget.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace mt {

enum class MotorMode { Passive, Compliant, Active, Hold };

struct MotorTaskCommand {
    std::string jointId;
    Quat target{};
    float activation = 1.f;
    // Negative keeps the active profile frequency. Balance raises this so a
    // torque-limited spring resists a buckle before the error runs away.
    float frequencyHz = -1.f;
};

struct MotorGroup {
    std::string id;
    std::string label;
    float activation = 0.f;
    float strength = 1.f;
};

struct JointMotor {
    std::string id;
    std::string group;
    // 0 turns the motor off. Group sliders write this unless overrideGroup is set.
    float activation = 0.f;
    bool overrideGroup = false;
    bool overrideGains = false;
    float frequencyHz = 4.f;
    float dampingRatio = 1.f;
    float torqueLimit = 20.f;
    Quat target{};
    Quat neutral{};
};

// Decides every angular-motor command and is the only caller of
// PhysicsWorld::setAngularMotor. Joints are addressed by ragdoll id.
class MotorController {
public:
    bool bind(const PhysicsWorld& world, const Ragdoll& ragdoll);
    bool bound() const { return bound_; }

    void loadDefaultProfiles();
    const MotorProfile& profile(MotorMode mode) const;

    void setMode(MotorMode mode);
    MotorMode mode() const { return mode_; }

    void setGroupActivation(const std::string& group, float activation);
    void setGroupStrength(const std::string& group, float strength);
    void setJointActivation(const std::string& jointId, float activation, bool asOverride);
    void setJointGains(const std::string& jointId, float frequencyHz, float dampingRatio, float torqueNm);
    void setUseProfileGains(const std::string& jointId, bool useProfile);
    void setTarget(const std::string& jointId, Quat constraintOrientation);
    void setAllActivations(float activation, bool clearOverrides);

    // One explicit task. While it is set, listed joints take these targets and
    // activations instead of the mode/pose values. Unlisted joints are unchanged.
    // This is the ownership hook balance uses. It is not a full behavior arbiter.
    void setMotorTask(std::vector<MotorTaskCommand> commands);
    void clearMotorTask();
    bool motorTaskActive() const { return taskActive_; }

    // Copies current constraint orientations into targets and enables those
    // joints. An empty id list captures every joint. World transforms are not read.
    void hold(const PhysicsWorld& world, const Ragdoll& ragdoll, const std::vector<std::string>& jointIds);

    void relaxAll();
    void activateAll();
    void restoreDefaults();
    void beginRelaxation(float seconds);
    void updateRelaxation(float dt);
    bool relaxing() const { return relaxing_; }

    // Writes the final commands. Returns false if any joint was rejected.
    bool apply(PhysicsWorld& world, const Ragdoll& ragdoll);

    // After the bodies have been put back on their initial poses.
    void onSimulationReset(PhysicsWorld& world, const Ragdoll& ragdoll);

    const JointMotor* find(const std::string& jointId) const;
    JointMotor* find(const std::string& jointId);
    const std::vector<JointMotor>& joints() const { return joints_; }
    const std::vector<MotorGroup>& groups() const { return groups_; }
    Quat target(const std::string& jointId) const;
    Quat neutral(const std::string& jointId) const;

    bool saveConfiguration(const std::string& path, std::string& error) const;
    bool loadConfiguration(const std::string& path, std::string& error);

private:
    const MotorProfile& activeProfile() const;
    void ensureGroups();
    MotorGroup* findGroup(const std::string& id);

    bool bound_ = false;
    MotorMode mode_ = MotorMode::Passive;
    std::vector<JointMotor> joints_;
    std::vector<MotorGroup> groups_;
    std::unordered_map<std::string, int> index_;
    MotorProfile passive_{};
    MotorProfile compliant_{};
    MotorProfile active_{};
    bool relaxing_ = false;
    float relaxRate_ = 0.f;
    bool taskActive_ = false;
    std::vector<MotorTaskCommand> task_;
    std::unordered_map<std::string, int> taskIndex_;
};

const char* motorModeName(MotorMode mode);
MotorMode motorModeFromName(const std::string& name, bool& ok);

}  // namespace mt
