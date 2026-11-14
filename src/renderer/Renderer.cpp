#include "mortrauma/renderer/Renderer.hpp"

#include "mortrauma/core/Log.hpp"

#include <GL/gl.h>

#include <GLFW/glfw3.h>

#include <cmath>
#include <cstddef>
#include <vector>

namespace mt {
namespace {

#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#define GL_ELEMENT_ARRAY_BUFFER 0x8893
#define GL_STATIC_DRAW 0x88E4
#define GL_DYNAMIC_DRAW 0x88E8
#define GL_FRAGMENT_SHADER 0x8B30
#define GL_VERTEX_SHADER 0x8B31
#define GL_COMPILE_STATUS 0x8B81
#define GL_LINK_STATUS 0x8B82
#define GL_INFO_LOG_LENGTH 0x8B84
#endif
#ifndef GL_FRAMEBUFFER
#define GL_FRAMEBUFFER 0x8D40
#define GL_READ_FRAMEBUFFER 0x8CA8
#define GL_DRAW_FRAMEBUFFER 0x8CA9
#define GL_COLOR_ATTACHMENT0 0x8CE0
#define GL_DEPTH_ATTACHMENT 0x8D00
#define GL_RENDERBUFFER 0x8D41
#define GL_DEPTH_COMPONENT24 0x81A6
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5
#define GL_RGBA8 0x8058
#define GL_MULTISAMPLE 0x809D
#define GL_MAX_SAMPLES 0x8D57
#define GL_CLAMP_TO_EDGE 0x812F
#endif
#ifndef GL_FILL
#define GL_FILL 0x1B02
#define GL_LINE 0x1B01
#define GL_FRONT_AND_BACK 0x0408
#endif

// The Windows SDK gl.h is OpenGL 1.1 and does not define these names.
using GlSizeiptr = std::ptrdiff_t;

using GlGen = void (*)(GLsizei, GLuint*);
using GlBind = void (*)(GLenum, GLuint);
using GlBindVertexArrayFn = void (*)(GLuint);
using GlBufferDataFn = void (*)(GLenum, GlSizeiptr, const void*, GLenum);
using GlAttrib = void (*)(GLuint, GLint, GLenum, GLboolean, GLsizei, const void*);
using GlEnableAttrib = void (*)(GLuint);
using GlShaderSourceFn = void (*)(GLuint, GLsizei, const char* const*, const GLint*);
using GlGetiv = void (*)(GLuint, GLenum, GLint*);
using GlGetInfo = void (*)(GLuint, GLsizei, GLsizei*, char*);
using GlUniformMat = void (*)(GLint, GLsizei, GLboolean, const GLfloat*);
using GlUniform3 = void (*)(GLint, GLfloat, GLfloat, GLfloat);
using GlUniform1iFn = void (*)(GLint, GLint);
using GlDrawElementsFn = void (*)(GLenum, GLsizei, GLenum, const void*);
using GlDrawArraysFn = void (*)(GLenum, GLint, GLsizei);
using GlRenderbufferStorageFn = void (*)(GLenum, GLenum, GLsizei, GLsizei);
using GlRenderbufferStorageMsFn = void (*)(GLenum, GLsizei, GLenum, GLsizei, GLsizei);
using GlFramebufferRenderbufferFn = void (*)(GLenum, GLenum, GLenum, GLuint);
using GlFramebufferTextureFn = void (*)(GLenum, GLenum, GLenum, GLuint, GLint);
using GlBlitFn = void (*)(GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLbitfield, GLenum);
using GlTexImageFn = void (*)(GLenum, GLint, GLint, GLsizei, GLsizei, GLint, GLenum, GLenum, const void*);
using GlTexParamFn = void (*)(GLenum, GLenum, GLint);
using GlGetIntegervFn = void (*)(GLenum, GLint*);
using GlPolygonModeFn = void (*)(GLenum, GLenum);

GlGen glGenVertexArrays_ = nullptr;
GlBindVertexArrayFn glBindVertexArray_ = nullptr;
GlGen glGenBuffers_ = nullptr;
GlBind glBindBuffer_ = nullptr;
GlBufferDataFn glBufferData_ = nullptr;
GlAttrib glVertexAttribPointer_ = nullptr;
GlEnableAttrib glEnableVertexAttribArray_ = nullptr;
using GlCreateShaderFn = GLuint (*)(GLenum);
GlCreateShaderFn glCreateShader_ = nullptr;
GlShaderSourceFn glShaderSource_ = nullptr;
using GlCompileFn = void (*)(GLuint);
GlCompileFn glCompileShader_ = nullptr;
GlGetiv glGetShaderiv_ = nullptr;
GlGetInfo glGetShaderInfoLog_ = nullptr;
using GlCreateProgramFn = GLuint (*)();
GlCreateProgramFn glCreateProgram_ = nullptr;
using GlAttachFn = void (*)(GLuint, GLuint);
GlAttachFn glAttachShader_ = nullptr;
using GlLinkFn = void (*)(GLuint);
GlLinkFn glLinkProgram_ = nullptr;
GlGetiv glGetProgramiv_ = nullptr;
GlGetInfo glGetProgramInfoLog_ = nullptr;
using GlDeleteShaderFn = void (*)(GLuint);
GlDeleteShaderFn glDeleteShader_ = nullptr;
using GlUseFn = void (*)(GLuint);
GlUseFn glUseProgram_ = nullptr;
using GlGetUniformFn = GLint (*)(GLuint, const char*);
GlGetUniformFn glGetUniformLocation_ = nullptr;
GlUniformMat glUniformMatrix4fv_ = nullptr;
GlUniform3 glUniform3f_ = nullptr;
GlUniform1iFn glUniform1i_ = nullptr;
GlDrawElementsFn glDrawElements_ = nullptr;
GlDrawArraysFn glDrawArrays_ = nullptr;
GlGen glGenFramebuffers_ = nullptr;
GlBind glBindFramebuffer_ = nullptr;
GlGen glGenRenderbuffers_ = nullptr;
GlBind glBindRenderbuffer_ = nullptr;
GlRenderbufferStorageFn glRenderbufferStorage_ = nullptr;
GlRenderbufferStorageMsFn glRenderbufferStorageMultisample_ = nullptr;
GlFramebufferRenderbufferFn glFramebufferRenderbuffer_ = nullptr;
GlFramebufferTextureFn glFramebufferTexture2D_ = nullptr;
using GlCheckFn = GLenum (*)(GLenum);
GlCheckFn glCheckFramebufferStatus_ = nullptr;
GlBlitFn glBlitFramebuffer_ = nullptr;
GlGen glGenTextures_ = nullptr;
GlBind glBindTexture_ = nullptr;
GlTexImageFn glTexImage2D_ = nullptr;
GlTexParamFn glTexParameteri_ = nullptr;
GlGetIntegervFn glGetIntegerv_ = nullptr;
GlPolygonModeFn glPolygonMode_ = nullptr;
using GlDeleteFn = void (*)(GLsizei, const GLuint*);
GlDeleteFn glDeleteFramebuffers_ = nullptr;
GlDeleteFn glDeleteRenderbuffers_ = nullptr;
GlDeleteFn glDeleteTextures_ = nullptr;
GlDeleteFn glDeleteBuffers_ = nullptr;
GlDeleteFn glDeleteVertexArrays_ = nullptr;
using GlDeleteProgramFn = void (*)(GLuint);
GlDeleteProgramFn glDeleteProgram_ = nullptr;

template <typename T>
bool load(T& fn, const char* name) {
    fn = reinterpret_cast<T>(glfwGetProcAddress(name));
    return fn != nullptr;
}

const char* kMeshVertex = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
uniform mat4 uViewProj;
uniform mat4 uModel;
out vec3 vNormal;
out vec3 vWorld;
void main() {
    vec4 world = uModel * vec4(aPos, 1.0);
    vWorld = world.xyz;
    vNormal = mat3(uModel) * aNormal;
    gl_Position = uViewProj * world;
})";

const char* kMeshFragment = R"(#version 330 core
in vec3 vNormal;
in vec3 vWorld;
uniform vec3 uColor;
uniform int uLit;
out vec4 fragColor;
void main() {
    if (uLit == 0) {
        fragColor = vec4(uColor, 1.0);
        return;
    }
    vec3 n = normalize(vNormal);
    vec3 lightDir = normalize(vec3(0.35, 0.85, 0.25));
    float ndotl = max(dot(n, lightDir), 0.0);
    float wrap = max(dot(n, lightDir) * 0.5 + 0.5, 0.0);
    vec3 view = normalize(vec3(0.0, 0.6, 1.0));
    float spec = pow(max(dot(n, normalize(lightDir + view)), 0.0), 24.0);
    vec3 color = uColor * (0.22 + 0.62 * wrap + 0.28 * ndotl) + vec3(spec) * 0.12;
    fragColor = vec4(color, 1.0);
})";

const char* kLineVertex = R"(#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uViewProj;
void main() { gl_Position = uViewProj * vec4(aPos, 1.0); })";

const char* kLineFragment = R"(#version 330 core
uniform vec3 uColor;
out vec4 fragColor;
void main() { fragColor = vec4(uColor, 1.0); })";

GLuint compile(GLenum type, const char* source, std::string& error) {
    const GLuint shader = glCreateShader_(type);
    glShaderSource_(shader, 1, &source, nullptr);
    glCompileShader_(shader);
    GLint ok = 0;
    glGetShaderiv_(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog_(shader, sizeof(log), nullptr, log);
        error = log;
        glDeleteShader_(shader);
        return 0;
    }
    return shader;
}

GLuint linkProgram(const char* vertex, const char* fragment, std::string& error) {
    const GLuint vs = compile(GL_VERTEX_SHADER, vertex, error);
    const GLuint fs = compile(GL_FRAGMENT_SHADER, fragment, error);
    if (vs == 0 || fs == 0) {
        return 0;
    }
    const GLuint program = glCreateProgram_();
    glAttachShader_(program, vs);
    glAttachShader_(program, fs);
    glLinkProgram_(program);
    glDeleteShader_(vs);
    glDeleteShader_(fs);
    GLint ok = 0;
    glGetProgramiv_(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog_(program, sizeof(log), nullptr, log);
        error = log;
        glDeleteProgram_(program);
        return 0;
    }
    return program;
}

}  // namespace

bool Renderer::initialize(std::string& error) {
    const bool ok = load(glGenVertexArrays_, "glGenVertexArrays") && load(glBindVertexArray_, "glBindVertexArray") &&
                    load(glGenBuffers_, "glGenBuffers") && load(glBindBuffer_, "glBindBuffer") &&
                    load(glBufferData_, "glBufferData") && load(glVertexAttribPointer_, "glVertexAttribPointer") &&
                    load(glEnableVertexAttribArray_, "glEnableVertexAttribArray") && load(glCreateShader_, "glCreateShader") &&
                    load(glShaderSource_, "glShaderSource") && load(glCompileShader_, "glCompileShader") &&
                    load(glGetShaderiv_, "glGetShaderiv") && load(glGetShaderInfoLog_, "glGetShaderInfoLog") &&
                    load(glCreateProgram_, "glCreateProgram") && load(glAttachShader_, "glAttachShader") &&
                    load(glLinkProgram_, "glLinkProgram") && load(glGetProgramiv_, "glGetProgramiv") &&
                    load(glGetProgramInfoLog_, "glGetProgramInfoLog") && load(glDeleteShader_, "glDeleteShader") &&
                    load(glUseProgram_, "glUseProgram") && load(glGetUniformLocation_, "glGetUniformLocation") &&
                    load(glUniformMatrix4fv_, "glUniformMatrix4fv") && load(glUniform3f_, "glUniform3f") &&
                    load(glUniform1i_, "glUniform1i") && load(glDrawElements_, "glDrawElements") &&
                    load(glDrawArrays_, "glDrawArrays") && load(glGenFramebuffers_, "glGenFramebuffers") &&
                    load(glBindFramebuffer_, "glBindFramebuffer") && load(glGenRenderbuffers_, "glGenRenderbuffers") &&
                    load(glBindRenderbuffer_, "glBindRenderbuffer") && load(glRenderbufferStorage_, "glRenderbufferStorage") &&
                    load(glRenderbufferStorageMultisample_, "glRenderbufferStorageMultisample") &&
                    load(glFramebufferRenderbuffer_, "glFramebufferRenderbuffer") &&
                    load(glFramebufferTexture2D_, "glFramebufferTexture2D") &&
                    load(glCheckFramebufferStatus_, "glCheckFramebufferStatus") && load(glBlitFramebuffer_, "glBlitFramebuffer") &&
                    load(glGenTextures_, "glGenTextures") && load(glBindTexture_, "glBindTexture") &&
                    load(glTexImage2D_, "glTexImage2D") && load(glTexParameteri_, "glTexParameteri") &&
                    load(glGetIntegerv_, "glGetIntegerv") && load(glPolygonMode_, "glPolygonMode") &&
                    load(glDeleteFramebuffers_, "glDeleteFramebuffers") && load(glDeleteRenderbuffers_, "glDeleteRenderbuffers") &&
                    load(glDeleteTextures_, "glDeleteTextures") && load(glDeleteBuffers_, "glDeleteBuffers") &&
                    load(glDeleteVertexArrays_, "glDeleteVertexArrays") && load(glDeleteProgram_, "glDeleteProgram");
    if (!ok) {
        error = "OpenGL 3.3 entry points are missing.";
        Log::error(error);
        return false;
    }
    program_ = linkProgram(kMeshVertex, kMeshFragment, error);
    lineProgram_ = linkProgram(kLineVertex, kLineFragment, error);
    if (program_ == 0 || lineProgram_ == 0) {
        Log::error("Shader compile failed: " + error);
        return false;
    }
    viewLoc_ = glGetUniformLocation_(program_, "uViewProj");
    modelLoc_ = glGetUniformLocation_(program_, "uModel");
    colorLoc_ = glGetUniformLocation_(program_, "uColor");
    litLoc_ = glGetUniformLocation_(program_, "uLit");
    lineViewLoc_ = glGetUniformLocation_(lineProgram_, "uViewProj");
    lineColorLoc_ = glGetUniformLocation_(lineProgram_, "uColor");
    glGenVertexArrays_(1, &lineVao_);
    glGenBuffers_(1, &lineVbo_);
    glBindVertexArray_(lineVao_);
    glBindBuffer_(GL_ARRAY_BUFFER, lineVbo_);
    glBufferData_(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray_(0);
    glVertexAttribPointer_(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, nullptr);
    glBindVertexArray_(0);
    GLint samples = 1;
    glGetIntegerv_(GL_MAX_SAMPLES, &samples);
    samples_ = samples >= 4 ? 4 : 1;
    if (!createTargets(8, 8, error)) {
        return false;
    }
    ready_ = true;
    Log::info(samples_ > 1 ? "Renderer ready with 4x MSAA." : "Renderer ready without MSAA.");
    return true;
}

void Renderer::shutdown() {
    if (!ready_) {
        return;
    }
    destroyTargets();
    for (auto& mesh : meshes_) {
        glDeleteVertexArrays_(1, &mesh.vao);
        glDeleteBuffers_(1, &mesh.vbo);
        glDeleteBuffers_(1, &mesh.ebo);
    }
    meshes_.clear();
    glDeleteVertexArrays_(1, &lineVao_);
    glDeleteBuffers_(1, &lineVbo_);
    glDeleteProgram_(program_);
    glDeleteProgram_(lineProgram_);
    ready_ = false;
}

bool Renderer::createTargets(int width, int height, std::string& error) {
    width_ = std::max(width, 1);
    height_ = std::max(height, 1);
    glGenTextures_(1, &colorTexture_);
    glBindTexture_(GL_TEXTURE_2D, colorTexture_);
    glTexImage2D_(GL_TEXTURE_2D, 0, GL_RGBA8, width_, height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri_(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri_(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri_(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri_(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glGenFramebuffers_(1, &fbo_);
    glBindFramebuffer_(GL_FRAMEBUFFER, fbo_);
    glFramebufferTexture2D_(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTexture_, 0);
    if (samples_ <= 1) {
        glGenRenderbuffers_(1, &depthBuffer_);
        glBindRenderbuffer_(GL_RENDERBUFFER, depthBuffer_);
        glRenderbufferStorage_(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width_, height_);
        glFramebufferRenderbuffer_(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBuffer_);
    }
    if (glCheckFramebufferStatus_(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        error = "Viewport framebuffer is incomplete.";
        Log::error(error);
        return false;
    }
    if (samples_ > 1) {
        glGenFramebuffers_(1, &msaaFbo_);
        glBindFramebuffer_(GL_FRAMEBUFFER, msaaFbo_);
        glGenRenderbuffers_(1, &msaaColor_);
        glBindRenderbuffer_(GL_RENDERBUFFER, msaaColor_);
        glRenderbufferStorageMultisample_(GL_RENDERBUFFER, samples_, GL_RGBA8, width_, height_);
        glFramebufferRenderbuffer_(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, msaaColor_);
        glGenRenderbuffers_(1, &msaaDepth_);
        glBindRenderbuffer_(GL_RENDERBUFFER, msaaDepth_);
        glRenderbufferStorageMultisample_(GL_RENDERBUFFER, samples_, GL_DEPTH_COMPONENT24, width_, height_);
        glFramebufferRenderbuffer_(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, msaaDepth_);
        if (glCheckFramebufferStatus_(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            Log::warn("MSAA framebuffer failed. Falling back to a single sample.");
            glDeleteFramebuffers_(1, &msaaFbo_);
            glDeleteRenderbuffers_(1, &msaaColor_);
            glDeleteRenderbuffers_(1, &msaaDepth_);
            msaaFbo_ = msaaColor_ = msaaDepth_ = 0;
            samples_ = 1;
            glBindFramebuffer_(GL_FRAMEBUFFER, fbo_);
            glGenRenderbuffers_(1, &depthBuffer_);
            glBindRenderbuffer_(GL_RENDERBUFFER, depthBuffer_);
            glRenderbufferStorage_(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width_, height_);
            glFramebufferRenderbuffer_(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBuffer_);
            if (glCheckFramebufferStatus_(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
                error = "Viewport framebuffer is incomplete.";
                Log::error(error);
                return false;
            }
        }
    }
    glBindFramebuffer_(GL_FRAMEBUFFER, 0);
    return true;
}

void Renderer::destroyTargets() {
    if (colorTexture_ != 0) {
        glDeleteTextures_(1, &colorTexture_);
    }
    if (depthBuffer_ != 0) {
        glDeleteRenderbuffers_(1, &depthBuffer_);
    }
    if (fbo_ != 0) {
        glDeleteFramebuffers_(1, &fbo_);
    }
    if (msaaColor_ != 0) {
        glDeleteRenderbuffers_(1, &msaaColor_);
    }
    if (msaaDepth_ != 0) {
        glDeleteRenderbuffers_(1, &msaaDepth_);
    }
    if (msaaFbo_ != 0) {
        glDeleteFramebuffers_(1, &msaaFbo_);
    }
    colorTexture_ = depthBuffer_ = fbo_ = msaaFbo_ = msaaColor_ = msaaDepth_ = 0;
}

void Renderer::resize(int width, int height) {
    if (!ready_ || (width == width_ && height == height_)) {
        return;
    }
    const int samples = samples_;
    destroyTargets();
    samples_ = samples;
    std::string error;
    if (!createTargets(width, height, error)) {
        Log::error(error);
    }
}

int Renderer::createMesh(const std::vector<SkinnedVertex>& vertices, const std::vector<uint32_t>& indices) {
    std::vector<float> packed;
    packed.reserve(vertices.size() * 6);
    for (const auto& vertex : vertices) {
        packed.push_back(vertex.position.x);
        packed.push_back(vertex.position.y);
        packed.push_back(vertex.position.z);
        packed.push_back(vertex.normal.x);
        packed.push_back(vertex.normal.y);
        packed.push_back(vertex.normal.z);
    }
    GpuMesh mesh;
    mesh.count = int(indices.size());
    glGenVertexArrays_(1, &mesh.vao);
    glGenBuffers_(1, &mesh.vbo);
    glGenBuffers_(1, &mesh.ebo);
    glBindVertexArray_(mesh.vao);
    glBindBuffer_(GL_ARRAY_BUFFER, mesh.vbo);
    glBufferData_(GL_ARRAY_BUFFER, GlSizeiptr(packed.size() * sizeof(float)), packed.data(), GL_STATIC_DRAW);
    glBindBuffer_(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);
    glBufferData_(GL_ELEMENT_ARRAY_BUFFER, GlSizeiptr(indices.size() * sizeof(uint32_t)), indices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray_(0);
    glVertexAttribPointer_(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 6, nullptr);
    glEnableVertexAttribArray_(1);
    glVertexAttribPointer_(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 6, reinterpret_cast<void*>(sizeof(float) * 3));
    glBindVertexArray_(0);
    const int id = int(meshes_.size());
    meshes_.push_back(mesh);
    return id;
}

void Renderer::beginScene(const Mat4& viewProjection, bool wireframe) {
    wireframe_ = wireframe;
    lineVertices_.clear();
    const GLuint target = msaaFbo_ != 0 ? msaaFbo_ : fbo_;
    glBindFramebuffer_(GL_FRAMEBUFFER, target);
    glViewport(0, 0, width_, height_);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    if (samples_ > 1) {
        glEnable(GL_MULTISAMPLE);
    }
    glClearColor(0.16f, 0.17f, 0.19f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram_(program_);
    glUniformMatrix4fv_(viewLoc_, 1, GL_FALSE, viewProjection.m);
    glUseProgram_(lineProgram_);
    glUniformMatrix4fv_(lineViewLoc_, 1, GL_FALSE, viewProjection.m);
    glUseProgram_(program_);
}

void Renderer::drawMesh(int mesh, const Mat4& model, Vec3 color) {
    if (mesh < 0 || mesh >= int(meshes_.size())) {
        return;
    }
    glUseProgram_(program_);
    glUniformMatrix4fv_(modelLoc_, 1, GL_FALSE, model.m);
    glUniform3f_(colorLoc_, color.x, color.y, color.z);
    glUniform1i_(litLoc_, 1);
    glPolygonMode_(GL_FRONT_AND_BACK, wireframe_ ? GL_LINE : GL_FILL);
    glBindVertexArray_(meshes_[mesh].vao);
    glDrawElements_(GL_TRIANGLES, meshes_[mesh].count, GL_UNSIGNED_INT, nullptr);
    glPolygonMode_(GL_FRONT_AND_BACK, GL_FILL);
    glBindVertexArray_(0);
}

void Renderer::drawLine(Vec3 a, Vec3 b, Vec3 color) {
    lineVertices_.insert(lineVertices_.end(), {a.x, a.y, a.z, color.x, color.y, color.z, b.x, b.y, b.z, color.x, color.y, color.z});
}

void Renderer::drawBox(Vec3 center, Quat rotation, Vec3 halfExtents, Vec3 color) {
    const Vec3 corners[8] = {{-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {-1, -1, 1}, {-1, 1, -1}, {1, 1, -1}, {1, 1, 1}, {-1, 1, 1}};
    Vec3 world[8];
    for (int i = 0; i < 8; ++i) {
        world[i] = center + rotate(rotation, Vec3{corners[i].x * halfExtents.x, corners[i].y * halfExtents.y, corners[i].z * halfExtents.z});
    }
    const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const auto& edge : edges) {
        drawLine(world[edge[0]], world[edge[1]], color);
    }
}

void Renderer::drawCapsule(Vec3 center, Quat rotation, float radius, float cylinderHalfHeight, Vec3 color) {
    constexpr int kSegments = 12;
    auto ring = [&](float y) {
        for (int i = 0; i < kSegments; ++i) {
            const float a0 = (float(i) / kSegments) * kPi * 2.f;
            const float a1 = (float(i + 1) / kSegments) * kPi * 2.f;
            const Vec3 p0 = center + rotate(rotation, Vec3{std::cos(a0) * radius, y, std::sin(a0) * radius});
            const Vec3 p1 = center + rotate(rotation, Vec3{std::cos(a1) * radius, y, std::sin(a1) * radius});
            drawLine(p0, p1, color);
        }
    };
    ring(cylinderHalfHeight);
    ring(-cylinderHalfHeight);
    for (int i = 0; i < 4; ++i) {
        const float a = (float(i) / 4.f) * kPi * 2.f;
        const Vec3 offset{std::cos(a) * radius, 0.f, std::sin(a) * radius};
        drawLine(center + rotate(rotation, offset + Vec3{0, cylinderHalfHeight, 0}),
                 center + rotate(rotation, offset + Vec3{0, -cylinderHalfHeight, 0}), color);
    }
}

void Renderer::drawSphere(Vec3 center, float radius, Vec3 color) {
    drawCapsule(center, {}, radius, 0.f, color);
}

void Renderer::flushLines() {
    if (lineVertices_.empty()) {
        return;
    }
    // Interleaved position/color is not what the line shader reads. Split into
    // one draw per color run by rebinding a position-only buffer.
    glUseProgram_(lineProgram_);
    glDisable(GL_CULL_FACE);
    glBindVertexArray_(lineVao_);
    glBindBuffer_(GL_ARRAY_BUFFER, lineVbo_);
    std::vector<float> positions;
    Vec3 current{-1000.f, -1000.f, -1000.f};
    auto flush = [&](Vec3 color) {
        if (positions.empty()) {
            return;
        }
        glUniform3f_(lineColorLoc_, color.x, color.y, color.z);
        glBufferData_(GL_ARRAY_BUFFER, GlSizeiptr(positions.size() * sizeof(float)), positions.data(), GL_DYNAMIC_DRAW);
        glDrawArrays_(GL_LINES, 0, GLsizei(positions.size() / 3));
        positions.clear();
    };
    for (size_t i = 0; i + 5 < lineVertices_.size(); i += 6) {
        const Vec3 color{lineVertices_[i + 3], lineVertices_[i + 4], lineVertices_[i + 5]};
        if (length(color - current) > 0.001f && !positions.empty()) {
            flush(current);
        }
        current = color;
        positions.insert(positions.end(), {lineVertices_[i], lineVertices_[i + 1], lineVertices_[i + 2]});
    }
    flush(current);
    glBindVertexArray_(0);
    glEnable(GL_CULL_FACE);
}

void Renderer::endScene() {
    flushLines();
    if (msaaFbo_ != 0) {
        glBindFramebuffer_(GL_READ_FRAMEBUFFER, msaaFbo_);
        glBindFramebuffer_(GL_DRAW_FRAMEBUFFER, fbo_);
        glBlitFramebuffer_(0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT, GL_LINEAR);
    }
    glBindFramebuffer_(GL_FRAMEBUFFER, 0);
}

}  // namespace mt
