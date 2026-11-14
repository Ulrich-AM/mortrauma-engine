#pragma once

#include <string>
#include <vector>

namespace mt {

enum class SwingKind { Cone, Pyramid };
enum class BendOrient { None, Elbow, Knee, Hip };

struct BodyConfig {
    std::string id;
    std::string name;
    std::vector<std::string> bones;
    std::string primaryBone;
    std::string endBone;
    float endExtension = 0.04f;
    float mass = 1.f;
    float radius = 0.f;
    float minRadius = 0.045f;
    float maxRadius = 0.2f;
    float friction = 0.7f;
    float restitution = 0.02f;
};

struct JointConfig {
    std::string id;
    std::string name;
    std::string parent;
    std::string child;
    SwingKind swing = SwingKind::Pyramid;
    BendOrient orient = BendOrient::None;
    float twistMin = 0.f;
    float twistMax = 0.f;
    float swing1Min = 0.f;
    float swing1Max = 0.f;
    float swing2Min = 0.f;
    float swing2Max = 0.f;
};

struct JointPreset {
    std::string name = "Natural";
    float linearDamping = 0.08f;
    float angularDamping = 0.18f;
    float angularFriction = 2.f;
    float limitScale = 1.f;
};

struct CharacterConfig {
    std::string name;
    std::vector<BodyConfig> bodies;
    std::vector<JointConfig> joints;
    std::string error;

    const BodyConfig* findBody(const std::string& id) const;
};

CharacterConfig loadCharacterConfig(const std::string& path);
JointPreset loadJointPreset(const std::string& path, std::string& error);
bool saveJointPreset(const std::string& path, const JointPreset& preset, std::string& error);

}  // namespace mt
