#include "mortrauma/app/Application.hpp"

#include "mortrauma/assets/ModelLoader.hpp"
#include "mortrauma/core/Log.hpp"
#include "mortrauma/core/Paths.hpp"
#include "mortrauma/renderer/Renderer.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

// GLFW includes the platform headers before GL/gl.h. Including GL/gl.h first
// leaves WINGDIAPI undefined in the Windows SDK.
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>

namespace mt {
namespace {

enum class Tool { Select, Grab, Push };
enum class SelectionKind { None, Body, Joint };
enum class PushDirection { Camera, WorldX, WorldY, WorldZ };

struct Camera {
    Vec3 target{0.f, 1.05f, 0.f};
    float yaw = 0.8f;
    float pitch = 0.35f;
    float distance = 4.2f;
    float fov = radians(42.f);

    Vec3 eye() const {
        const Vec3 offset{std::cos(pitch) * std::sin(yaw), std::sin(pitch), std::cos(pitch) * std::cos(yaw)};
        return target + offset * distance;
    }
    Vec3 forward() const { return normalize(target - eye()); }
    Vec3 right() const { return normalize(cross(forward(), Vec3{0.f, 1.f, 0.f})); }
    Vec3 up() const { return cross(right(), forward()); }
    Vec3 ray(float ndcX, float ndcY, float aspect) const {
        const float half = std::tan(fov * 0.5f);
        return normalize(forward() + right() * (ndcX * half * aspect) + up() * (ndcY * half));
    }
};

struct EnvBody {
    std::string name;
    int physics = -1;
    Vec3 initialPosition{};
    Quat initialRotation{};
    bool optional = false;
    bool dynamic = false;
};

struct App {
    GLFWwindow* window = nullptr;
    Renderer renderer;
    PhysicsWorld world;
    ImportedModel model;
    CharacterConfig config;
    Ragdoll ragdoll;
    JointPreset preset;
    JointPreset defaultPreset;
    std::vector<int> meshGpu;
    std::vector<Vec3> meshColor;
    std::vector<EnvBody> environment;
    Camera camera;
    Tool tool = Tool::Select;
    SelectionKind selection = SelectionKind::None;
    int selectedBody = -1;
    int selectedJoint = -1;
    bool playing = false;
    bool wireframe = false;
    bool showCollision = false;
    bool showJoints = true;
    bool showContacts = false;
    bool showCenters = false;
    bool showVelocity = true;
    bool backfaceCulling = true;
    bool platform = false;
    bool ramp = false;
    float accumulator = 0.f;
    float speed = 1.f;
    int collisionSteps = 1;
    int velocityIterations = 12;
    int positionIterations = 3;
    Vec3 gravity{0.f, -9.81f, 0.f};
    float grabFrequency = 8.f;
    float grabDamping = 0.8f;
    float pushStrength = 18.f;
    PushDirection pushDirection = PushDirection::Camera;
    int grab = -1;
    Vec3 grabPlanePoint{};
    Vec3 grabPlaneNormal{};
    double lastTime = 0.0;
    float renderFps = 0.f;
    float physicsFps = 0.f;
    int physicsStepsThisWindow = 0;
    double fpsWindowStart = 0.0;
    bool layoutBuilt = false;
    std::string presetStatus;
    int cube = -1;
    int platformBody = -1;
    int rampBody = -1;

    Vec3 bodyColor(int index) const {
        const float t = float(index) * 0.37f;
        return {0.62f + 0.08f * std::sin(t), 0.5f + 0.05f * std::cos(t * 1.3f), 0.42f + 0.04f * std::sin(t * 0.7f)};
    }

    bool boot() {
        PhysicsSettings settings;
        if (!world.initialize(settings)) {
            return false;
        }
        gravity = settings.gravity;
        velocityIterations = settings.velocityIterations;
        positionIterations = settings.positionIterations;
        const auto configPath = findAsset("configurations/humanoidguy.json");
        const auto modelPath = findAsset("characters/humanoidguy_rigged.fbx");
        const auto presetPath = findAsset("configurations/presets/natural.json");
        std::string presetError;
        preset = loadJointPreset(presetPath.string(), presetError);
        defaultPreset = preset;
        config = loadCharacterConfig(configPath.string());
        if (modelPath.empty()) {
            Log::error("humanoidguy_rigged.fbx was not found next to the executable or in the source tree.");
            model = makeProceduralHumanoid();
        } else {
            model = ModelLoader::load(modelPath.string());
            if (!model.report.ok) {
                model = makeProceduralHumanoid();
            }
        }
        if (!config.error.empty()) {
            return false;
        }
        ragdoll = RagdollBuilder::build(world, config, model, preset);
        if (!ragdoll.ok) {
            return false;
        }
        float lowest = 1e9f;
        for (const auto& body : ragdoll.bodies) {
            lowest = std::min(lowest, body.initialPosition.y - body.radius);
        }
        if (lowest < 0.02f) {
            const float lift = 0.02f - lowest;
            for (auto& body : ragdoll.bodies) {
                body.initialPosition.y += lift;
                body.segmentStart.y += lift;
                body.segmentEnd.y += lift;
                world.setBodyPose(body.physics, body.initialPosition, body.initialRotation, false);
                world.setBodyVelocity(body.physics, {}, {});
            }
        }
        loadEnvironment();
        for (const auto& mesh : model.meshes) {
            meshGpu.push_back(renderer.createMesh(mesh.vertices, mesh.indices));
            meshColor.push_back(bodyColor(std::max(mesh.bone, 0)));
        }
        return true;
    }

    void addStaticBox(const std::string& name, Vec3 center, Vec3 half, Quat rotation, bool optional) {
        BodyCreateInfo info;
        info.name = name;
        info.isStatic = true;
        info.shape = BodyCreateInfo::Shape::Box;
        info.position = center;
        info.rotation = rotation;
        info.halfExtents = half;
        info.friction = 0.85f;
        info.restitution = 0.02f;
        EnvBody body;
        body.name = name;
        body.physics = world.createBody(info);
        body.initialPosition = center;
        body.initialRotation = rotation;
        body.optional = optional;
        environment.push_back(body);
        if (name == "Platform") {
            platformBody = body.physics;
        }
        if (name == "Ramp") {
            rampBody = body.physics;
        }
    }

    void loadEnvironment() {
        const auto path = findAsset("environments/lab.json");
        nlohmann::json json;
        std::ifstream input(path);
        if (input) {
            try {
                input >> json;
            } catch (const std::exception& exception) {
                Log::error(std::string("Lab JSON is invalid: ") + exception.what());
            }
        } else {
            Log::error("Lab description was not found. Using built-in floor and walls.");
        }
        auto vec3 = [](const nlohmann::json& value, Vec3 fallback) {
            if (!value.is_array() || value.size() != 3) {
                return fallback;
            }
            return Vec3{value[0].get<float>(), value[1].get<float>(), value[2].get<float>()};
        };
        const auto floor = json.contains("floor") ? json["floor"] : nlohmann::json::object();
        addStaticBox("Floor", vec3(floor.value("center", nlohmann::json::array({0, -0.1, 0})), {0, -0.1f, 0}),
                     vec3(floor.value("half_extents", nlohmann::json::array({15, 0.1, 15})), {15, 0.1f, 15}), {}, false);
        if (json.contains("walls")) {
            for (const auto& wall : json["walls"]) {
                addStaticBox(wall.value("name", "Wall"), vec3(wall["center"], {}), vec3(wall["half_extents"], {1, 1, 1}), {}, false);
            }
        }
        const auto cubeJson = json.contains("cube") ? json["cube"] : nlohmann::json::object();
        BodyCreateInfo box;
        box.name = "Test cube";
        box.shape = BodyCreateInfo::Shape::Box;
        box.halfExtents = vec3(cubeJson.value("half_extents", nlohmann::json::array({0.2, 0.2, 0.2})), {0.2f, 0.2f, 0.2f});
        box.position = vec3(cubeJson.value("center", nlohmann::json::array({1.3, 0.45, 1.1})), {1.3f, 0.45f, 1.1f});
        box.mass = cubeJson.value("mass", 8.f);
        box.friction = 0.55f;
        box.restitution = 0.05f;
        EnvBody dynamic;
        dynamic.name = box.name;
        dynamic.physics = world.createBody(box);
        dynamic.initialPosition = box.position;
        dynamic.dynamic = true;
        cube = dynamic.physics;
        environment.push_back(dynamic);
    }

    void setOptional(const std::string& name, bool enabled, Vec3 center, Vec3 half, Quat rotation) {
        int* slot = name == "Platform" ? &platformBody : &rampBody;
        if (!enabled) {
            if (*slot >= 0) {
                world.destroyBody(*slot);
                environment.erase(std::remove_if(environment.begin(), environment.end(),
                                                 [&](const EnvBody& body) { return body.physics == *slot; }),
                                  environment.end());
                if (selectedBody == *slot) {
                    selection = SelectionKind::None;
                    selectedBody = -1;
                }
                *slot = -1;
            }
            return;
        }
        if (*slot >= 0) {
            return;
        }
        addStaticBox(name, center, half, rotation, true);
    }

    void resetSimulation() {
        world.clearGrabs();
        grab = -1;
        resetRagdoll(world, ragdoll);
        for (const auto& body : environment) {
            world.setBodyPose(body.physics, body.initialPosition, body.initialRotation, body.dynamic);
            if (body.dynamic) {
                world.setBodyVelocity(body.physics, {}, {});
            }
        }
        accumulator = 0.f;
    }

    void applyPhysicsSettings() {
        world.setGravity(gravity);
        world.setSolverIterations(velocityIterations, positionIterations);
    }

    void stepOnce() {
        if (!world.step(world.settings().fixedTimestep, collisionSteps)) {
            return;
        }
        ++physicsStepsThisWindow;
    }

    void simulate(float frameSeconds) {
        if (!playing) {
            return;
        }
        accumulator += std::min(frameSeconds, 0.25f) * speed;
        accumulator = std::min(accumulator, 0.25f);
        const float dt = world.settings().fixedTimestep;
        int steps = 0;
        while (accumulator >= dt && steps < 6) {
            stepOnce();
            accumulator -= dt;
            ++steps;
        }
    }

    int physicsIndexOfRagdoll(int body) const {
        if (body < 0 || body >= int(ragdoll.bodies.size())) {
            return -1;
        }
        return ragdoll.bodies[body].physics;
    }

    const RagdollBody* ragdollFromPhysics(int physics) const {
        for (const auto& body : ragdoll.bodies) {
            if (body.physics == physics) {
                return &body;
            }
        }
        return nullptr;
    }

    void drawWorld() {
        const float aspect = float(std::max(renderer.width(), 1)) / float(std::max(renderer.height(), 1));
        const Mat4 view = lookAt(camera.eye(), camera.target, {0.f, 1.f, 0.f});
        const Mat4 projection = perspective(camera.fov, aspect, 0.05f, 80.f);
        const Mat4 viewProjection = mul(projection, view);
        renderer.beginScene(viewProjection, wireframe);
        if (!backfaceCulling) {
            glDisable(GL_CULL_FACE);
        }
        const PosePalette palette = posePalette(world, ragdoll, model);
        for (int i = 0; i < int(model.meshes.size()); ++i) {
            const int bone = model.meshes[i].bone;
            Mat4 modelMatrix = bone >= 0 && bone < int(palette.boneWorld.size()) ? palette.boneWorld[bone] : Mat4::identity();
            Vec3 color = meshColor[i];
            if (const auto* body = ragdollFromPhysics(selectedBody); body != nullptr && selection == SelectionKind::Body) {
                for (int owned : body->bones) {
                    if (owned == bone) {
                        color = color + Vec3{0.18f, 0.16f, 0.05f};
                    }
                }
            }
            renderer.drawMesh(meshGpu[i], modelMatrix, color);
        }
        glEnable(GL_CULL_FACE);
        for (int x = -10; x <= 10; ++x) {
            const Vec3 color = x == 0 ? Vec3{0.45f, 0.28f, 0.28f} : Vec3{0.28f, 0.3f, 0.32f};
            renderer.drawLine({float(x), 0.002f, -10.f}, {float(x), 0.002f, 10.f}, color);
        }
        for (int z = -10; z <= 10; ++z) {
            const Vec3 color = z == 0 ? Vec3{0.28f, 0.45f, 0.32f} : Vec3{0.28f, 0.3f, 0.32f};
            renderer.drawLine({-10.f, 0.002f, float(z)}, {10.f, 0.002f, float(z)}, color);
        }
        for (const auto& body : environment) {
            const auto viewShape = world.shapeView(body.physics);
            const Vec3 color = body.physics == selectedBody ? Vec3{0.95f, 0.78f, 0.25f} : Vec3{0.34f, 0.36f, 0.4f};
            if (viewShape.shape == BodyCreateInfo::Shape::Box) {
                renderer.drawBox(viewShape.position, viewShape.rotation, viewShape.halfExtents, color);
            }
        }
        if (showCollision || (selection == SelectionKind::Body && selectedBody >= 0)) {
            const auto drawShape = [&](int physics, Vec3 color) {
                const auto viewShape = world.shapeView(physics);
                if (viewShape.shape == BodyCreateInfo::Shape::Capsule) {
                    renderer.drawCapsule(viewShape.position, viewShape.rotation, viewShape.radius, viewShape.halfHeight, color);
                } else if (viewShape.shape == BodyCreateInfo::Shape::Sphere) {
                    renderer.drawSphere(viewShape.position, viewShape.radius, color);
                } else {
                    renderer.drawBox(viewShape.position, viewShape.rotation, viewShape.halfExtents, color);
                }
            };
            if (showCollision) {
                for (const auto& body : ragdoll.bodies) {
                    drawShape(body.physics, {0.25f, 0.75f, 0.85f});
                }
            }
            if (selection == SelectionKind::Body && ragdollFromPhysics(selectedBody) != nullptr) {
                drawShape(selectedBody, {0.95f, 0.78f, 0.2f});
            }
        }
        if (showCenters) {
            for (const auto& body : ragdoll.bodies) {
                const auto state = world.bodyState(body.physics);
                const float s = 0.04f;
                renderer.drawLine(state.position - Vec3{s, 0, 0}, state.position + Vec3{s, 0, 0}, {1, 0.4f, 0.3f});
                renderer.drawLine(state.position - Vec3{0, s, 0}, state.position + Vec3{0, s, 0}, {0.4f, 1, 0.4f});
                renderer.drawLine(state.position - Vec3{0, 0, s}, state.position + Vec3{0, 0, s}, {0.4f, 0.6f, 1});
            }
        }
        if (showJoints) {
            for (const auto& joint : ragdoll.joints) {
                const auto state = world.jointState(joint.physics);
                const Vec3 color = (selection == SelectionKind::Joint && selectedJoint == joint.physics) ? Vec3{1.f, 0.85f, 0.2f}
                                                                                                         : Vec3{0.95f, 0.55f, 0.2f};
                renderer.drawLine(state.anchorParentWorld, state.anchorChildWorld, color);
                renderer.drawLine(state.anchorParentWorld, state.anchorParentWorld + state.twistAxisWorld * 0.12f, {0.9f, 0.25f, 0.25f});
                renderer.drawLine(state.anchorParentWorld, state.anchorParentWorld + state.bendAxisWorld * 0.1f, {0.3f, 0.85f, 0.35f});
            }
        }
        if (showContacts) {
            for (const auto& contact : world.contacts()) {
                renderer.drawLine(contact.point, contact.point + contact.normal * 0.12f, {0.4f, 0.9f, 1.f});
            }
        }
        if (showVelocity && selection == SelectionKind::Body && selectedBody >= 0) {
            const auto state = world.bodyState(selectedBody);
            renderer.drawLine(state.position, state.position + state.linearVelocity * 0.2f, {1.f, 0.95f, 0.4f});
        }
        renderer.endScene();
    }

    bool rayFromViewport(ImVec2 imageMin, ImVec2 imageMax, Vec3& origin, Vec3& direction) const {
        const ImVec2 mouse = ImGui::GetMousePos();
        if (mouse.x < imageMin.x || mouse.y < imageMin.y || mouse.x > imageMax.x || mouse.y > imageMax.y) {
            return false;
        }
        const float w = std::max(1.f, imageMax.x - imageMin.x);
        const float h = std::max(1.f, imageMax.y - imageMin.y);
        const float ndcX = ((mouse.x - imageMin.x) / w) * 2.f - 1.f;
        const float ndcY = 1.f - ((mouse.y - imageMin.y) / h) * 2.f;
        origin = camera.eye();
        direction = camera.ray(ndcX, ndcY, w / h);
        return true;
    }

    bool rayPlane(Vec3 origin, Vec3 direction, Vec3 point, Vec3 normal, Vec3& hit) const {
        const float denom = dot(normal, direction);
        if (std::fabs(denom) < 1e-5f) {
            return false;
        }
        const float t = dot(point - origin, normal) / denom;
        if (t < 0.f) {
            return false;
        }
        hit = origin + direction * t;
        return true;
    }

    void handleViewport(ImVec2 imageMin, ImVec2 imageMax) {
        Vec3 origin;
        Vec3 direction;
        const bool inside = rayFromViewport(imageMin, imageMax, origin, direction);
        const bool hovered = inside && !ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) ? inside : ImGui::IsItemHovered();
        if (!hovered) {
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && grab >= 0) {
                world.endGrab(grab);
                grab = -1;
            }
            return;
        }
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
            camera.yaw -= ImGui::GetIO().MouseDelta.x * 0.01f;
            camera.pitch = clampf(camera.pitch + ImGui::GetIO().MouseDelta.y * 0.01f, -1.2f, 1.2f);
        }
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
            const float scale = camera.distance * 0.0022f;
            camera.target = camera.target - camera.right() * ImGui::GetIO().MouseDelta.x * scale +
                            camera.up() * ImGui::GetIO().MouseDelta.y * scale;
        }
        const float wheel = ImGui::GetIO().MouseWheel;
        if (wheel != 0.f && hovered) {
            camera.distance = clampf(camera.distance * (1.f - wheel * 0.12f), 0.35f, 40.f);
        }
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            const RayHit hit = world.rayCast(origin, direction, 80.f);
            if (tool == Tool::Select || !playing) {
                if (hit.hit) {
                    selection = SelectionKind::Body;
                    selectedBody = hit.body;
                    selectedJoint = -1;
                }
            }
            if (tool == Tool::Push && playing && hit.hit && world.bodyState(hit.body).isDynamic) {
                Vec3 directionWorld = camera.forward();
                if (pushDirection == PushDirection::WorldX) directionWorld = {1, 0, 0};
                if (pushDirection == PushDirection::WorldY) directionWorld = {0, 1, 0};
                if (pushDirection == PushDirection::WorldZ) directionWorld = {0, 0, 1};
                world.addImpulse(hit.body, directionWorld * pushStrength, hit.point);
                selection = SelectionKind::Body;
                selectedBody = hit.body;
            }
            if (tool == Tool::Grab && playing && hit.hit && world.bodyState(hit.body).isDynamic) {
                grabPlanePoint = hit.point;
                grabPlaneNormal = camera.forward();
                grab = world.beginGrab(hit.body, hit.point, hit.point, grabFrequency, grabDamping);
                selection = SelectionKind::Body;
                selectedBody = hit.body;
            }
        }
        if (grab >= 0 && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            Vec3 target;
            if (rayPlane(origin, direction, grabPlanePoint, grabPlaneNormal, target)) {
                world.updateGrab(grab, target);
            }
        }
        if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && grab >= 0) {
            world.endGrab(grab);
            grab = -1;
        }
    }

    void drawUi() {
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::SetNextWindowViewport(viewport->ID);
        const ImGuiWindowFlags hostFlags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
                                           ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                           ImGuiWindowFlags_NoNavFocus;
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.f);
        ImGui::Begin("DockHost", nullptr, hostFlags);
        ImGui::PopStyleVar(2);
        const ImGuiID dockspace = ImGui::GetID("MortraumaDock");
        ImGui::DockSpace(dockspace, ImVec2(0, 0), ImGuiDockNodeFlags_PassthruCentralNode);
        if (!layoutBuilt) {
            layoutBuilt = true;
            ImGui::DockBuilderRemoveNode(dockspace);
            ImGui::DockBuilderAddNode(dockspace, ImGuiDockNodeFlags_DockSpace);
            ImGui::DockBuilderSetNodeSize(dockspace, viewport->WorkSize);
            ImGuiID center = dockspace;
            ImGuiID left = 0;
            ImGuiID right = 0;
            ImGuiID bottom = 0;
            ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.18f, &left, &center);
            ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.24f, &right, &center);
            ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.24f, &bottom, &center);
            ImGui::DockBuilderDockWindow("Viewport", center);
            ImGui::DockBuilderDockWindow("Hierarchy", left);
            ImGui::DockBuilderDockWindow("Inspector", right);
            ImGui::DockBuilderDockWindow("Simulation", bottom);
            ImGui::DockBuilderDockWindow("Physics", bottom);
            ImGui::DockBuilderDockWindow("Diagnostics", bottom);
            ImGui::DockBuilderFinish(dockspace);
        }
        ImGui::End();

        ImGui::Begin("Viewport");
        if (model.report.usedFallback) {
            ImGui::TextColored(ImVec4(1.f, 0.45f, 0.35f, 1.f), "Asset import failed. Procedural fallback is visible.");
            ImGui::TextWrapped("%s", model.report.error.c_str());
        }
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const int width = std::max(1, int(available.x));
        const int height = std::max(1, int(available.y));
        renderer.resize(width, height);
        drawWorld();
        const ImVec2 imagePos = ImGui::GetCursorScreenPos();
        ImGui::Image(static_cast<ImTextureID>(static_cast<intptr_t>(renderer.colorTexture())), available, ImVec2(0, 1), ImVec2(1, 0));
        handleViewport(imagePos, ImVec2(imagePos.x + available.x, imagePos.y + available.y));
        ImGui::End();

        ImGui::Begin("Hierarchy");
        if (ImGui::TreeNodeEx("Environment", ImGuiTreeNodeFlags_DefaultOpen)) {
            for (const auto& body : environment) {
                if (ImGui::Selectable(body.name.c_str(), selection == SelectionKind::Body && selectedBody == body.physics)) {
                    selection = SelectionKind::Body;
                    selectedBody = body.physics;
                    selectedJoint = -1;
                }
            }
            ImGui::TreePop();
        }
        if (ImGui::TreeNodeEx("Humanoid", ImGuiTreeNodeFlags_DefaultOpen)) {
            for (const auto& body : ragdoll.bodies) {
                if (ImGui::Selectable(body.name.c_str(), selection == SelectionKind::Body && selectedBody == body.physics)) {
                    selection = SelectionKind::Body;
                    selectedBody = body.physics;
                    selectedJoint = -1;
                }
            }
            if (ImGui::TreeNode("Joints")) {
                for (const auto& joint : ragdoll.joints) {
                    if (ImGui::Selectable(joint.name.c_str(), selection == SelectionKind::Joint && selectedJoint == joint.physics)) {
                        selection = SelectionKind::Joint;
                        selectedJoint = joint.physics;
                    }
                }
                ImGui::TreePop();
            }
            ImGui::TreePop();
        }
        ImGui::End();

        ImGui::Begin("Inspector");
        if (selection == SelectionKind::Body && selectedBody >= 0) {
            auto state = world.bodyState(selectedBody);
            if (state.valid) {
                ImGui::Text("%s", state.name.c_str());
                ImGui::Separator();
                ImGui::Text("Mass %.3f kg", state.mass);
                if (state.isDynamic) {
                    float mass = state.mass;
                    if (ImGui::SliderFloat("Mass", &mass, 0.2f, 40.f)) {
                        world.setBodyMass(selectedBody, mass);
                    }
                }
                ImGui::Text("Position  %.3f  %.3f  %.3f", state.position.x, state.position.y, state.position.z);
                ImGui::Text("Rotation  %.3f  %.3f  %.3f  %.3f", state.rotation.x, state.rotation.y, state.rotation.z, state.rotation.w);
                ImGui::Text("Linear velocity  %.3f  %.3f  %.3f", state.linearVelocity.x, state.linearVelocity.y, state.linearVelocity.z);
                ImGui::Text("Angular velocity %.3f  %.3f  %.3f", state.angularVelocity.x, state.angularVelocity.y, state.angularVelocity.z);
                float friction = state.friction;
                float restitution = state.restitution;
                float linear = state.linearDamping;
                float angular = state.angularDamping;
                if (ImGui::SliderFloat("Friction", &friction, 0.f, 2.f)) world.setBodyFriction(selectedBody, friction);
                if (ImGui::SliderFloat("Restitution", &restitution, 0.f, 1.f)) world.setBodyRestitution(selectedBody, restitution);
                if (state.isDynamic && ImGui::SliderFloat("Linear damping", &linear, 0.f, 0.9f)) {
                    world.setBodyDamping(selectedBody, linear, angular);
                }
                if (state.isDynamic && ImGui::SliderFloat("Angular damping", &angular, 0.f, 0.9f)) {
                    world.setBodyDamping(selectedBody, linear, angular);
                }
                ImGui::Text("Motion  %s", state.isStatic ? "Static" : (state.isKinematic ? "Kinematic" : "Dynamic"));
            }
        } else if (selection == SelectionKind::Joint && selectedJoint >= 0) {
            const auto state = world.jointState(selectedJoint);
            if (state.valid) {
                ImGui::Text("%s", state.name.c_str());
                ImGui::Separator();
                ImGui::Text("Type  %s", state.type.c_str());
                ImGui::Text("Bodies  %s  ->  %s", state.parentName.c_str(), state.childName.c_str());
                ImGui::Text("Anchor A  %.3f %.3f %.3f", state.anchorParentWorld.x, state.anchorParentWorld.y, state.anchorParentWorld.z);
                ImGui::Text("Anchor B  %.3f %.3f %.3f", state.anchorChildWorld.x, state.anchorChildWorld.y, state.anchorChildWorld.z);
                ImGui::Text("Separation  %.4f m", state.separation);
                ImGui::Text("Twist limit  %.1f .. %.1f deg", degrees(state.twistMin), degrees(state.twistMax));
                ImGui::Text("Swing 1      %.1f .. %.1f deg", degrees(state.swing1Min), degrees(state.swing1Max));
                ImGui::Text("Swing 2      %.1f .. %.1f deg", degrees(state.swing2Min), degrees(state.swing2Max));
                ImGui::Text("Current rotation (deg)  %.1f  %.1f  %.1f", degrees(state.rotationRadians.x), degrees(state.rotationRadians.y),
                            degrees(state.rotationRadians.z));
                ImGui::Text("Angular friction  %.2f Nm", state.angularFriction);
                ImGui::Text("Motor  %s", state.motorEnabled ? "enabled" : "off (reserved)");
                ImGui::Text("Motor frequency %.2f  damping %.2f  torque %.1f", state.motorFrequency, state.motorDamping, state.motorTorqueLimit);
                ImGui::Text("Position DOF  %s", state.positionLocked ? "locked" : "open");
                ImGui::Text("Status  %s", state.status.c_str());
            }
        } else {
            ImGui::TextUnformatted("Select a body or joint.");
        }
        ImGui::End();

        ImGui::Begin("Simulation");
        if (ImGui::Button(playing ? "Pause" : "Play")) {
            playing = !playing;
        }
        ImGui::SameLine();
        if (ImGui::Button("Step")) {
            playing = false;
            stepOnce();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            resetSimulation();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140);
        ImGui::SliderFloat("Speed", &speed, 0.1f, 2.f);
        ImGui::Text("Fixed timestep  %.4f s  (%.0f Hz)    substeps %d", world.settings().fixedTimestep,
                    1.f / world.settings().fixedTimestep, collisionSteps);
        ImGui::Text("Tools");
        int toolIndex = int(tool);
        if (ImGui::RadioButton("Select", &toolIndex, 0)) tool = Tool::Select;
        ImGui::SameLine();
        if (ImGui::RadioButton("Grab", &toolIndex, 1)) tool = Tool::Grab;
        ImGui::SameLine();
        if (ImGui::RadioButton("Push", &toolIndex, 2)) tool = Tool::Push;
        ImGui::SliderFloat("Grab strength", &grabFrequency, 1.f, 20.f);
        ImGui::SliderFloat("Grab damping", &grabDamping, 0.f, 1.5f);
        ImGui::SliderFloat("Impulse", &pushStrength, 1.f, 200.f);
        int direction = int(pushDirection);
        const char* directions[] = {"Camera forward", "+X", "+Y", "+Z"};
        if (ImGui::Combo("Impulse direction", &direction, directions, 4)) {
            pushDirection = PushDirection(direction);
        }
        ImGui::TextUnformatted("Right drag orbits, middle drag pans, wheel zooms. Space toggles play.");
        ImGui::End();

        ImGui::Begin("Physics");
        bool changed = false;
        changed |= ImGui::SliderFloat("Gravity X", &gravity.x, -10.f, 10.f);
        changed |= ImGui::SliderFloat("Gravity Y", &gravity.y, -30.f, 10.f);
        changed |= ImGui::SliderFloat("Gravity Z", &gravity.z, -10.f, 10.f);
        changed |= ImGui::SliderInt("Substeps", &collisionSteps, 1, 8);
        changed |= ImGui::SliderInt("Velocity iterations", &velocityIterations, 1, 64);
        changed |= ImGui::SliderInt("Position iterations", &positionIterations, 1, 16);
        if (changed) {
            applyPhysicsSettings();
        }
        ImGui::Checkbox("Collision shapes", &showCollision);
        ImGui::Checkbox("Constraints", &showJoints);
        ImGui::Checkbox("Contact points", &showContacts);
        ImGui::Checkbox("Centers of mass", &showCenters);
        ImGui::Checkbox("Selected velocity", &showVelocity);
        ImGui::Checkbox("Wireframe", &wireframe);
        ImGui::Checkbox("Backface culling", &backfaceCulling);
        if (ImGui::Checkbox("Elevated platform", &platform)) {
            setOptional("Platform", platform, {2.4f, 0.55f, -1.6f}, {0.8f, 0.08f, 0.8f}, {});
        }
        if (ImGui::Checkbox("Ramp", &ramp)) {
            setOptional("Ramp", ramp, {-2.2f, 0.35f, 2.2f}, {1.1f, 0.06f, 0.45f}, quatAxisAngle({1, 0, 0}, radians(-18.f)));
        }
        ImGui::Separator();
        int presetIndex = 1;
        if (preset.name == "Loose") presetIndex = 0;
        if (preset.name == "Firm") presetIndex = 2;
        const char* presets[] = {"Loose", "Natural", "Firm"};
        if (ImGui::Combo("Joint preset", &presetIndex, presets, 3)) {
            const char* file = presetIndex == 0 ? "configurations/presets/loose.json"
                                                 : (presetIndex == 2 ? "configurations/presets/firm.json" : "configurations/presets/natural.json");
            std::string error;
            const auto loaded = loadJointPreset(findAsset(file).string(), error);
            if (error.empty()) {
                preset = loaded;
                applyPreset(world, ragdoll, preset);
                presetStatus.clear();
            }
        }
        ImGui::SliderFloat("Angular friction", &preset.angularFriction, 0.f, 30.f);
        ImGui::SliderFloat("Linear damping", &preset.linearDamping, 0.f, 0.9f);
        ImGui::SliderFloat("Angular damping", &preset.angularDamping, 0.f, 0.9f);
        ImGui::SliderFloat("Limit scale", &preset.limitScale, 0.5f, 1.5f);
        if (ImGui::Button("Apply preset values")) {
            applyPreset(world, ragdoll, preset);
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset settings")) {
            gravity = {0.f, -9.81f, 0.f};
            collisionSteps = 1;
            velocityIterations = 12;
            positionIterations = 3;
            speed = 1.f;
            preset = defaultPreset;
            applyPhysicsSettings();
            applyPreset(world, ragdoll, preset);
            grabFrequency = 8.f;
            grabDamping = 0.8f;
            pushStrength = 18.f;
        }
        if (ImGui::Button("Save preset JSON")) {
            const auto path = executableDirectory() / "configurations" / "user_preset.json";
            std::filesystem::create_directories(path.parent_path());
            std::string error;
            preset.name = "User";
            if (saveJointPreset(path.string(), preset, error)) {
                presetStatus = "Saved " + path.string();
            } else {
                presetStatus = error;
            }
        }
        if (!presetStatus.empty()) {
            ImGui::TextWrapped("%s", presetStatus.c_str());
        }
        ImGui::TextWrapped("Presets change passive friction, damping and limit scale. They do not drive the body back to the bind pose.");
        ImGui::End();

        ImGui::Begin("Diagnostics");
        ImGui::Text("Simulation timestep  %.4f s", world.settings().fixedTimestep);
        ImGui::Text("Physics FPS  %.1f", physicsFps);
        ImGui::Text("Render FPS   %.1f", renderFps);
        ImGui::Text("Rigid bodies %d", world.bodyCount());
        ImGui::Text("Constraints  %d", world.jointCount());
        ImGui::Text("Active contacts %d", world.activeContactCount());
        ImGui::Text("Disconnected ragdoll components %d", disconnectedComponents(ragdoll));
        if (selection == SelectionKind::Body) {
            const auto state = world.bodyState(selectedBody);
            ImGui::Text("Selected body  %s", state.valid ? state.name.c_str() : "(invalid)");
        } else if (selection == SelectionKind::Joint) {
            const auto state = world.jointState(selectedJoint);
            ImGui::Text("Selected joint  %s", state.valid ? state.name.c_str() : "(invalid)");
        } else {
            ImGui::TextUnformatted("Selected body  (none)");
        }
        ImGui::TextWrapped("Last error: %s", Log::lastError().empty() ? "(none)" : Log::lastError().c_str());
        ImGui::Separator();
        ImGui::Text("Import  bones %d  meshes %d  vertices %d  scale %.3f  up %s", model.report.boneCount, model.report.meshCount,
                    model.report.vertexCount, model.report.unitScale, model.report.upAxis.c_str());
        ImGui::Text("Ragdoll bodies %d  joints %d  fallback %s", int(ragdoll.bodies.size()), int(ragdoll.joints.size()),
                    model.report.usedFallback ? "yes" : "no");
        if (!model.report.warnings.empty() || !ragdoll.warnings.empty()) {
            ImGui::TextUnformatted("Warnings");
            for (const auto& warning : model.report.warnings) ImGui::BulletText("%s", warning.c_str());
            for (const auto& warning : ragdoll.warnings) ImGui::BulletText("%s", warning.c_str());
        }
        if (ImGui::CollapsingHeader("Log")) {
            for (const auto& line : Log::lines()) {
                ImGui::TextUnformatted(line.c_str());
            }
        }
        ImGui::End();
    }
};

void applyStyle() {
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 0.f;
    style.ChildRounding = 0.f;
    style.FrameRounding = 0.f;
    style.PopupRounding = 0.f;
    style.ScrollbarRounding = 0.f;
    style.GrabRounding = 0.f;
    style.TabRounding = 0.f;
    style.WindowBorderSize = 1.f;
    style.FramePadding = ImVec2(6.f, 3.f);
    style.ItemSpacing = ImVec2(6.f, 4.f);
    style.WindowPadding = ImVec2(8.f, 8.f);
    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.11f, 0.11f, 0.12f, 1.f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.11f, 0.11f, 0.12f, 1.f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.07f, 0.07f, 0.08f, 1.f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.14f, 0.14f, 0.15f, 1.f);
    colors[ImGuiCol_Header] = ImVec4(0.2f, 0.2f, 0.22f, 1.f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.28f, 0.28f, 0.3f, 1.f);
    colors[ImGuiCol_Button] = ImVec4(0.18f, 0.18f, 0.2f, 1.f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.28f, 0.28f, 0.3f, 1.f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.16f, 0.16f, 0.17f, 1.f);
    colors[ImGuiCol_Tab] = ImVec4(0.14f, 0.14f, 0.15f, 1.f);
    colors[ImGuiCol_TabSelected] = ImVec4(0.22f, 0.22f, 0.24f, 1.f);
}

}  // namespace

int Application::run() {
    if (!glfwInit()) {
        Log::error("GLFW failed to initialize.");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    GLFWwindow* window = glfwCreateWindow(1600, 900, "MORTRAUMA | Physical Reactions Laboratory", nullptr, nullptr);
    if (window == nullptr) {
        Log::error("Could not create an OpenGL 3.3 window.");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    App app;
    app.window = window;
    std::string error;
    if (!app.renderer.initialize(error) || !app.boot()) {
        Log::error(error.empty() ? "Application startup failed. See the log above." : error);
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    static std::string iniPath = (executableDirectory() / "mortrauma_layout.ini").string();
    io.IniFilename = iniPath.c_str();
    const auto fontPath = findAsset("fonts/LiberationSans-Regular.ttf");
    if (!fontPath.empty()) {
        io.Fonts->AddFontFromFileTTF(fontPath.string().c_str(), 16.f);
    }
    applyStyle();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    app.lastTime = glfwGetTime();
    app.fpsWindowStart = app.lastTime;
    while (!glfwWindowShouldClose(window)) {
        const double now = glfwGetTime();
        const float frame = float(std::min(now - app.lastTime, 0.25));
        app.lastTime = now;
        glfwPollEvents();
        static bool spaceWasDown = false;
        const bool spaceDown = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
        if (spaceDown && !spaceWasDown) {
            app.playing = !app.playing;
        }
        spaceWasDown = spaceDown;
        static double lastReset = 0;
        if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS && now - lastReset > 0.3) {
            app.resetSimulation();
            lastReset = now;
        }
        if (app.grab >= 0) {
            // Target is refreshed from the viewport while the mouse is down.
        }
        app.simulate(frame);
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        app.drawUi();
        ImGui::Render();
        int framebufferWidth = 0;
        int framebufferHeight = 0;
        glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
        glViewport(0, 0, framebufferWidth, framebufferHeight);
        glClearColor(0.08f, 0.08f, 0.09f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);

        if (now - app.fpsWindowStart >= 0.5) {
            const double elapsed = now - app.fpsWindowStart;
            app.renderFps = float(ImGui::GetFrameCount() > 0 ? 1.f / std::max(frame, 1e-4f) : 0.f);
            app.physicsFps = float(app.physicsStepsThisWindow / elapsed);
            app.physicsStepsThisWindow = 0;
            app.fpsWindowStart = now;
        }
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    app.renderer.shutdown();
    app.world.shutdown();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

}  // namespace mt
