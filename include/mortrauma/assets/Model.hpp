#pragma once

#include "mortrauma/core/Math.hpp"

#include <string>
#include <vector>

namespace mt {

struct Bone {
    std::string name;
    int parent = -1;
    Mat4 bindWorld = Mat4::identity();
    Vec3 bindPosition{};
    Quat bindRotation{};
};

struct SkinnedVertex {
    Vec3 position{};  // bone-local meters, in the dominant bone frame
    Vec3 normal{};
    int bone = 0;
};

struct MeshPart {
    std::string name;
    std::string boneName;
    int bone = -1;
    std::vector<SkinnedVertex> vertices;
    std::vector<uint32_t> indices;
};

struct ImportReport {
    bool ok = false;
    bool usedFallback = false;
    std::string path;
    std::string error;
    std::vector<std::string> warnings;
    int boneCount = 0;
    int meshCount = 0;
    int vertexCount = 0;
    int rigidMeshes = 0;
    int blendedMeshes = 0;
    int unweightedMeshes = 0;
    float unitScale = 1.f;
    std::string upAxis;
};

struct ImportedModel {
    ImportReport report;
    std::vector<Bone> bones;
    std::vector<MeshPart> meshes;

    int findBone(const std::string& name) const;
};

}  // namespace mt
