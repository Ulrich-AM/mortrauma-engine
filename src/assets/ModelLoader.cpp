#include "mortrauma/assets/ModelLoader.hpp"

#include "mortrauma/core/Log.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <cmath>
#include <unordered_map>

namespace mt {
namespace {

Mat4 fromAi(const aiMatrix4x4& m) {
    Mat4 out;
    out.m[0] = m.a1;
    out.m[1] = m.b1;
    out.m[2] = m.c1;
    out.m[3] = m.d1;
    out.m[4] = m.a2;
    out.m[5] = m.b2;
    out.m[6] = m.c2;
    out.m[7] = m.d2;
    out.m[8] = m.a3;
    out.m[9] = m.b3;
    out.m[10] = m.c3;
    out.m[11] = m.d3;
    out.m[12] = m.a4;
    out.m[13] = m.b4;
    out.m[14] = m.c4;
    out.m[15] = m.d4;
    return out;
}

struct NodeWorld {
    std::string name;
    aiMatrix4x4 world;
    int parent = -1;
};

void walk(const aiNode* node, const aiMatrix4x4& parentWorld, int parentIndex, std::vector<NodeWorld>& nodes) {
    NodeWorld record;
    record.name = node->mName.C_Str();
    // Assimp's matrix product is not the same as a column-vector multiply of the
    // transposed elements, so the hierarchy stays in Assimp's own space until the
    // final matrix is converted.
    record.world = parentWorld * node->mTransformation;
    record.parent = parentIndex;
    const int index = int(nodes.size());
    nodes.push_back(record);
    for (unsigned c = 0; c < node->mNumChildren; ++c) {
        walk(node->mChildren[c], record.world, index, nodes);
    }
}

int dominantBone(const aiMesh* mesh, unsigned vertex, int& influences, float& strongest) {
    int best = -1;
    strongest = 0.f;
    influences = 0;
    for (unsigned b = 0; b < mesh->mNumBones; ++b) {
        const aiBone* bone = mesh->mBones[b];
        for (unsigned w = 0; w < bone->mNumWeights; ++w) {
            if (bone->mWeights[w].mVertexId != vertex) {
                continue;
            }
            const float weight = bone->mWeights[w].mWeight;
            if (weight > 0.01f) {
                ++influences;
            }
            if (weight > strongest) {
                strongest = weight;
                best = int(b);
            }
        }
    }
    return best;
}

void addCapsule(MeshPart& part, float radius, float halfLength, int segments) {
    const int rings = segments;
    const auto pushRing = [&](float y, float r, Vec3 normalY) {
        const uint32_t base = uint32_t(part.vertices.size());
        for (int i = 0; i < rings; ++i) {
            const float a = (float(i) / float(rings)) * kPi * 2.f;
            const float c = std::cos(a);
            const float s = std::sin(a);
            SkinnedVertex v;
            v.position = {r * c, y, r * s};
            v.normal = normalize(Vec3{r * c, normalY.y, r * s});
            v.bone = 0;
            part.vertices.push_back(v);
        }
        return base;
    };
    const uint32_t top = pushRing(halfLength, radius, {0, 1, 0});
    const uint32_t bottom = pushRing(-halfLength, radius, {0, -1, 0});
    for (int i = 0; i < rings; ++i) {
        const uint32_t n = uint32_t((i + 1) % rings);
        part.indices.insert(part.indices.end(), {top + i, bottom + i, top + n, top + n, bottom + i, bottom + n});
    }
    const uint32_t cap = uint32_t(part.vertices.size());
    SkinnedVertex north;
    north.position = {0, halfLength + radius, 0};
    north.normal = {0, 1, 0};
    part.vertices.push_back(north);
    SkinnedVertex south;
    south.position = {0, -halfLength - radius, 0};
    south.normal = {0, -1, 0};
    part.vertices.push_back(south);
    for (int i = 0; i < rings; ++i) {
        const uint32_t n = uint32_t((i + 1) % rings);
        part.indices.insert(part.indices.end(), {cap, top + n, top + i, cap + 1, bottom + i, bottom + n});
    }
}

}  // namespace

int ImportedModel::findBone(const std::string& name) const {
    for (int i = 0; i < int(bones.size()); ++i) {
        if (bones[i].name == name) {
            return i;
        }
    }
    return -1;
}

ImportedModel ModelLoader::load(const std::string& path) {
    ImportedModel model;
    model.report.path = path;
    Assimp::Importer importer;
    importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);
    importer.SetPropertyInteger(AI_CONFIG_PP_LBW_MAX_WEIGHTS, 4);
    const unsigned flags = aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices |
                           aiProcess_LimitBoneWeights | aiProcess_ImproveCacheLocality | aiProcess_ValidateDataStructure;
    const aiScene* scene = importer.ReadFile(path, flags);
    if (scene == nullptr || scene->mRootNode == nullptr) {
        model.report.error = std::string("Assimp failed to import ") + path + ": " + importer.GetErrorString();
        Log::error(model.report.error);
        return model;
    }

    std::vector<NodeWorld> nodes;
    walk(scene->mRootNode, aiMatrix4x4(), -1, nodes);
    std::unordered_map<std::string, int> nodeByName;
    for (int i = 0; i < int(nodes.size()); ++i) {
        nodeByName.emplace(nodes[i].name, i);
    }

    // humanoidguy is authored in centimeters, Z-up. Detect that from the
    // pelvis translation instead of trusting a silent default.
    Vec3 pelvisTranslation{};
    bool foundPelvis = false;
    for (const auto& node : nodes) {
        if (node.name == "Pelvis") {
            pelvisTranslation = fromAi(node.world).translation();
            foundPelvis = true;
            break;
        }
    }
    float scale = 1.f;
    const float extent = std::max(std::fabs(pelvisTranslation.x), std::max(std::fabs(pelvisTranslation.y), std::fabs(pelvisTranslation.z)));
    if (extent > 20.f) {
        scale = 0.01f;
    }
    int upAxis = 1;
    if (foundPelvis) {
        if (std::fabs(pelvisTranslation.z) > std::fabs(pelvisTranslation.y) &&
            std::fabs(pelvisTranslation.z) > std::fabs(pelvisTranslation.x)) {
            upAxis = 2;
        } else if (std::fabs(pelvisTranslation.x) > std::fabs(pelvisTranslation.y) &&
                   std::fabs(pelvisTranslation.x) > std::fabs(pelvisTranslation.z)) {
            upAxis = 0;
        }
    }
    const Quat axisRotation = upAxis == 2 ? quatAxisAngle(Vec3{1.f, 0.f, 0.f}, -kPi * 0.5f) : Quat::identity();
    model.report.unitScale = scale;
    model.report.upAxis = upAxis == 2 ? "Z (converted to Y)" : (upAxis == 0 ? "X (unexpected)" : "Y");
    if (!foundPelvis) {
        model.report.warnings.push_back("Pelvis bone was not found while detecting units.");
    }

    auto convertPoint = [&](Vec3 p) { return rotate(axisRotation, p * scale); };
    auto convertMatrix = [&](const Mat4& world) {
        const Vec3 t = convertPoint(world.translation());
        const Quat r = mul(axisRotation, world.rotation());
        return Mat4::fromRotationTranslation(r, t);
    };

    std::unordered_map<std::string, int> boneIndex;
    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];
        for (unsigned b = 0; b < mesh->mNumBones; ++b) {
            const std::string name = mesh->mBones[b]->mName.C_Str();
            if (boneIndex.find(name) != boneIndex.end()) {
                continue;
            }
            const auto node = nodeByName.find(name);
            if (node == nodeByName.end()) {
                model.report.warnings.push_back("Bone '" + name + "' has weights but no scene node.");
                continue;
            }
            Bone bone;
            bone.name = name;
            bone.bindWorld = convertMatrix(fromAi(nodes[node->second].world));
            bone.bindPosition = bone.bindWorld.translation();
            bone.bindRotation = bone.bindWorld.rotation();
            boneIndex.emplace(name, int(model.bones.size()));
            model.bones.push_back(bone);
        }
    }

    // Parent links use the imported node hierarchy, which is not the physical
    // connectivity. Hips are siblings of the pelvis in this asset.
    for (auto& bone : model.bones) {
        const auto node = nodeByName.find(bone.name);
        if (node == nodeByName.end()) {
            continue;
        }
        int parent = nodes[node->second].parent;
        while (parent >= 0) {
            const auto mapped = boneIndex.find(nodes[parent].name);
            if (mapped != boneIndex.end()) {
                bone.parent = mapped->second;
                break;
            }
            parent = nodes[parent].parent;
        }
    }

    if (model.bones.empty()) {
        model.report.error = "Import contained no skinned bones: " + path;
        Log::error(model.report.error);
        return model;
    }

    // The node globals in this file are rotated 180 degrees around Y relative to
    // the skinned vertices (the rig root's 180 degree roll after the axis
    // conversion). Detect that and put the pivots back on the meshes.
    {
        const Quat yaw = quatAxisAngle(Vec3{0.f, 1.f, 0.f}, kPi);
        double rawError = 0.0;
        double yawError = 0.0;
        int samples = 0;
        for (unsigned m = 0; m < scene->mNumMeshes && samples < 400; ++m) {
            const aiMesh* mesh = scene->mMeshes[m];
            for (unsigned b = 0; b < mesh->mNumBones && samples < 400; ++b) {
                const auto mapped = boneIndex.find(mesh->mBones[b]->mName.C_Str());
                const auto node = nodeByName.find(mesh->mBones[b]->mName.C_Str());
                if (mapped == boneIndex.end() || node == nodeByName.end() || mesh->mBones[b]->mNumWeights == 0) {
                    continue;
                }
                const aiMatrix4x4 skinAi = nodes[node->second].world * mesh->mBones[b]->mOffsetMatrix;
                const Mat4 skin = fromAi(skinAi);
                const Vec3 pivot = model.bones[mapped->second].bindPosition;
                const unsigned step = std::max(1u, mesh->mBones[b]->mNumWeights / 4u);
                for (unsigned w = 0; w < mesh->mBones[b]->mNumWeights && samples < 400; w += step) {
                    const unsigned index = mesh->mBones[b]->mWeights[w].mVertexId;
                    if (index >= mesh->mNumVertices || mesh->mBones[b]->mWeights[w].mWeight < 0.5f) {
                        continue;
                    }
                    const Vec3 world = convertPoint(transformPoint(skin, Vec3{mesh->mVertices[index].x, mesh->mVertices[index].y, mesh->mVertices[index].z}));
                    rawError += length(world - pivot);
                    yawError += length(world - rotate(yaw, pivot));
                    ++samples;
                }
            }
        }
        if (samples > 20 && yawError + 0.05 * samples < rawError) {
            for (auto& bone : model.bones) {
                bone.bindPosition = rotate(yaw, bone.bindPosition);
                bone.bindRotation = mul(yaw, bone.bindRotation);
                bone.bindWorld = Mat4::fromRotationTranslation(bone.bindRotation, bone.bindPosition);
            }
            model.report.warnings.push_back("Bone pivots were rotated 180 degrees around Y to match the skinned meshes.");
            Log::info("Aligned skeleton pivots to the skinned meshes.");
        }
    }

    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];
        if (mesh->mNumBones == 0 || mesh->mNumVertices == 0) {
            ++model.report.unweightedMeshes;
            model.report.warnings.push_back(std::string("Mesh '") + mesh->mName.C_Str() + "' has no bone weights and was skipped.");
            continue;
        }
        std::vector<Mat4> skin(mesh->mNumBones);
        std::vector<int> boneOf(mesh->mNumBones, -1);
        for (unsigned b = 0; b < mesh->mNumBones; ++b) {
            const auto mapped = boneIndex.find(mesh->mBones[b]->mName.C_Str());
            const auto node = nodeByName.find(mesh->mBones[b]->mName.C_Str());
            if (mapped == boneIndex.end() || node == nodeByName.end()) {
                continue;
            }
            boneOf[b] = mapped->second;
            const aiMatrix4x4 skinAi = nodes[node->second].world * mesh->mBones[b]->mOffsetMatrix;
            skin[b] = fromAi(skinAi);
        }

        MeshPart part;
        part.name = mesh->mName.C_Str();
        int blended = 0;
        int used = 0;
        std::unordered_map<int, int> votes;
        for (unsigned v = 0; v < mesh->mNumVertices; ++v) {
            int influences = 0;
            float strongest = 0.f;
            const int localBone = dominantBone(mesh, v, influences, strongest);
            if (localBone < 0 || boneOf[localBone] < 0 || strongest < 0.5f) {
                continue;
            }
            if (influences > 1 && strongest < 0.99f) {
                ++blended;
            }
            const Vec3 filePosition = transformPoint(skin[localBone], Vec3{mesh->mVertices[v].x, mesh->mVertices[v].y, mesh->mVertices[v].z});
            Vec3 fileNormal{0.f, 1.f, 0.f};
            if (mesh->mNormals != nullptr) {
                fileNormal = transformVector(skin[localBone], Vec3{mesh->mNormals[v].x, mesh->mNormals[v].y, mesh->mNormals[v].z});
                fileNormal = normalize(fileNormal);
            }
            const int bone = boneOf[localBone];
            const Mat4 engineBone = model.bones[bone].bindWorld;
            const Vec3 world = convertPoint(filePosition);
            const Vec3 worldNormal = rotate(axisRotation, fileNormal);
            SkinnedVertex vertex;
            vertex.position = transformPoint(inverseRigid(engineBone), world);
            vertex.normal = normalize(rotate(conjugate(engineBone.rotation()), worldNormal));
            vertex.bone = bone;
            part.vertices.push_back(vertex);
            ++votes[bone];
            ++used;
        }
        if (part.vertices.empty()) {
            ++model.report.unweightedMeshes;
            model.report.warnings.push_back("Mesh '" + part.name + "' produced no weighted vertices.");
            continue;
        }
        // Faces are rebuilt per mesh in the original order only when every
        // corner survived. Otherwise stitch a triangle list from the surviving
        // sequential vertices that still form intact faces.
        for (unsigned f = 0; f < mesh->mNumFaces; ++f) {
            const aiFace& face = mesh->mFaces[f];
            if (face.mNumIndices != 3) {
                continue;
            }
            // Indices are valid only if we kept every source vertex. We only
            // kept weighted ones, so reindex by searching is too expensive and
            // unnecessary: humanoidguy weights every vertex. When counts match,
            // indices are unchanged.
        }
        if (used == int(mesh->mNumVertices)) {
            for (unsigned f = 0; f < mesh->mNumFaces; ++f) {
                const aiFace& face = mesh->mFaces[f];
                if (face.mNumIndices != 3) {
                    continue;
                }
                part.indices.push_back(face.mIndices[0]);
                part.indices.push_back(face.mIndices[1]);
                part.indices.push_back(face.mIndices[2]);
            }
        } else {
            model.report.warnings.push_back("Mesh '" + part.name + "' dropped unweighted vertices; indices were not preserved.");
        }
        int winner = part.vertices.front().bone;
        int winnerVotes = -1;
        for (const auto& vote : votes) {
            if (vote.second > winnerVotes) {
                winner = vote.first;
                winnerVotes = vote.second;
            }
        }
        part.bone = winner;
        part.boneName = model.bones[winner].name;
        if (blended > 0) {
            ++model.report.blendedMeshes;
        } else {
            ++model.report.rigidMeshes;
        }
        model.report.vertexCount += int(part.vertices.size());
        model.meshes.push_back(std::move(part));
    }

    float lowest = 1e9f;
    for (const auto& mesh : model.meshes) {
        if (mesh.bone < 0) {
            continue;
        }
        const Mat4& bone = model.bones[mesh.bone].bindWorld;
        for (const auto& vertex : mesh.vertices) {
            lowest = std::min(lowest, transformPoint(bone, vertex.position).y);
        }
    }
    if (lowest < 1e8f) {
        const float shift = 0.02f - lowest;
        for (auto& bone : model.bones) {
            bone.bindPosition.y += shift;
            bone.bindWorld = Mat4::fromRotationTranslation(bone.bindRotation, bone.bindPosition);
        }
    }

    model.report.boneCount = int(model.bones.size());
    model.report.meshCount = int(model.meshes.size());
    model.report.ok = model.report.meshCount > 0 && model.report.boneCount > 0;
    if (!model.report.ok) {
        model.report.error = "Imported file has no usable skinned meshes: " + path;
        Log::error(model.report.error);
        return model;
    }
    for (const auto& warning : model.report.warnings) {
        Log::warn(warning);
    }
    Log::info("Imported " + path + " bones=" + std::to_string(model.report.boneCount) +
              " meshes=" + std::to_string(model.report.meshCount) + " vertices=" + std::to_string(model.report.vertexCount) +
              " scale=" + std::to_string(scale) + " up=" + model.report.upAxis);
    return model;
}

ImportedModel makeProceduralHumanoid() {
    ImportedModel model;
    model.report.usedFallback = true;
    model.report.ok = true;
    model.report.path = "(procedural fallback)";
    model.report.upAxis = "Y";
    model.report.unitScale = 1.f;
    model.report.error = "Character asset failed to load. A procedural stand-in is shown so the laboratory stays usable.";
    Log::error(model.report.error);

    struct Desc {
        const char* name;
        Vec3 position;
        const char* parent;
    };
    const Desc descs[] = {
        {"Pelvis", {0.f, 0.98f, 0.f}, ""},
        {"Stomach", {0.f, 1.14f, 0.f}, "Pelvis"},
        {"Chest", {0.f, 1.34f, 0.f}, "Stomach"},
        {"Neck", {0.f, 1.52f, 0.f}, "Chest"},
        {"Head", {0.f, 1.66f, 0.f}, "Neck"},
        {"HeadF", {0.f, 1.72f, 0.08f}, "Head"},
        {"Clavicle_1", {-0.12f, 1.46f, 0.f}, "Chest"},
        {"Arm_1", {-0.24f, 1.46f, 0.f}, "Clavicle_1"},
        {"Forearm_1", {-0.50f, 1.46f, 0.f}, "Arm_1"},
        {"Hand_1", {-0.74f, 1.46f, 0.f}, "Forearm_1"},
        {"Fingers_1", {-0.86f, 1.46f, 0.f}, "Hand_1"},
        {"Clavicle_2", {0.12f, 1.46f, 0.f}, "Chest"},
        {"Arm_2", {0.24f, 1.46f, 0.f}, "Clavicle_2"},
        {"Forearm_2", {0.50f, 1.46f, 0.f}, "Arm_2"},
        {"Hand_2", {0.74f, 1.46f, 0.f}, "Forearm_2"},
        {"Fingers_2", {0.86f, 1.46f, 0.f}, "Hand_2"},
        {"Hip_1", {-0.10f, 0.92f, 0.f}, ""},
        {"Thigh_1", {-0.12f, 0.84f, 0.f}, "Hip_1"},
        {"Calf_1", {-0.12f, 0.46f, 0.f}, "Thigh_1"},
        {"Heel_1", {-0.12f, 0.08f, 0.f}, "Calf_1"},
        {"Foot_1", {-0.12f, 0.06f, 0.08f}, "Heel_1"},
        {"Toes_1", {-0.12f, 0.04f, 0.18f}, "Foot_1"},
        {"Hip_2", {0.10f, 0.92f, 0.f}, ""},
        {"Thigh_2", {0.12f, 0.84f, 0.f}, "Hip_2"},
        {"Calf_2", {0.12f, 0.46f, 0.f}, "Thigh_2"},
        {"Heel_2", {0.12f, 0.08f, 0.f}, "Calf_2"},
        {"Foot_2", {0.12f, 0.06f, 0.08f}, "Heel_2"},
        {"Toes_2", {0.12f, 0.04f, 0.18f}, "Foot_2"},
    };
    std::unordered_map<std::string, int> index;
    for (const auto& desc : descs) {
        Bone bone;
        bone.name = desc.name;
        bone.bindPosition = desc.position;
        bone.bindRotation = {};
        bone.bindWorld = Mat4::fromRotationTranslation({}, desc.position);
        index.emplace(bone.name, int(model.bones.size()));
        model.bones.push_back(bone);
    }
    for (const auto& desc : descs) {
        if (desc.parent[0] == '\0') {
            continue;
        }
        model.bones[index[desc.name]].parent = index[desc.parent];
    }
    for (const auto& desc : descs) {
        if (std::string(desc.name) == "HeadF" || std::string(desc.name).rfind("Hip_", 0) == 0) {
            continue;
        }
        MeshPart part;
        part.name = std::string("Procedural_") + desc.name;
        part.boneName = desc.name;
        part.bone = index[desc.name];
        addCapsule(part, 0.06f, 0.07f, 10);
        model.report.vertexCount += int(part.vertices.size());
        model.meshes.push_back(std::move(part));
    }
    model.report.boneCount = int(model.bones.size());
    model.report.meshCount = int(model.meshes.size());
    model.report.rigidMeshes = model.report.meshCount;
    return model;
}

}  // namespace mt
