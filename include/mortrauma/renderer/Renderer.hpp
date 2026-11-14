#pragma once

#include "mortrauma/assets/Model.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace mt {

class Renderer {
public:
    bool initialize(std::string& error);
    void shutdown();
    void resize(int width, int height);
    int width() const { return width_; }
    int height() const { return height_; }
    uint32_t colorTexture() const { return colorTexture_; }

    int createMesh(const std::vector<SkinnedVertex>& vertices, const std::vector<uint32_t>& indices);
    void beginScene(const Mat4& viewProjection, bool wireframe);
    void drawMesh(int mesh, const Mat4& model, Vec3 color);
    void drawLine(Vec3 a, Vec3 b, Vec3 color);
    void drawBox(Vec3 center, Quat rotation, Vec3 halfExtents, Vec3 color);
    void drawCapsule(Vec3 center, Quat rotation, float radius, float cylinderHalfHeight, Vec3 color);
    void drawSphere(Vec3 center, float radius, Vec3 color);
    void endScene();

private:
    struct GpuMesh {
        uint32_t vao = 0;
        uint32_t vbo = 0;
        uint32_t ebo = 0;
        int count = 0;
    };
    bool createTargets(int width, int height, std::string& error);
    void destroyTargets();
    void flushLines();

    std::vector<GpuMesh> meshes_;
    uint32_t program_ = 0;
    uint32_t lineProgram_ = 0;
    uint32_t lineVao_ = 0;
    uint32_t lineVbo_ = 0;
    uint32_t fbo_ = 0;
    uint32_t colorTexture_ = 0;
    uint32_t depthBuffer_ = 0;
    uint32_t msaaFbo_ = 0;
    uint32_t msaaColor_ = 0;
    uint32_t msaaDepth_ = 0;
    int samples_ = 1;
    int width_ = 1;
    int height_ = 1;
    int viewLoc_ = -1;
    int modelLoc_ = -1;
    int colorLoc_ = -1;
    int litLoc_ = -1;
    int lineViewLoc_ = -1;
    int lineColorLoc_ = -1;
    bool wireframe_ = false;
    std::vector<float> lineVertices_;
    bool ready_ = false;
};

}  // namespace mt
