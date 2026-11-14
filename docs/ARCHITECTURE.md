# Architecture

MORTRAUMA keeps the simulation independent of the user interface. v0.2.0 adds motor and pose modules. v0.3.0 adds a balance estimate and a standing controller that submits motor tasks. Dear ImGui edits settings and submits commands. It does not integrate physics or compute motor torques.

```text
Application (GLFW, Dear ImGui)
    |  uses
    +-- Renderer (OpenGL 3.3, no physics types beyond math)
    +-- MotorController  -- the only writer of angular motor commands
    +-- BalanceController -- one motor task for standing; does not touch Jolt
    +-- PoseController   -- interpolates constraint-space targets
    +-- Ragdoll builder
            |  uses
            +-- PhysicsWorld   Jolt adapter
            +-- ModelLoader    Assimp
            +-- Character JSON
```

`mortrauma_engine` contains core, physics, assets, rig, motors, balance, and poses. It does not include Dear ImGui headers. Button handling lives in `src/app/Application.cpp` and calls the engine. Nothing in the physics step is wired to a particular key code or to one hardcoded mesh. The balance controller names bodies and joints by id (`pelvis`, `foot_1`, `hip_1`, `ankle_1`, and so on).

## Physics adapter

`PhysicsWorld` creates and destroys bodies and joints, steps the solver, and queries transforms, velocities, contacts, and constraint errors. Jolt types do not cross the header. Bodies are dynamic, kinematic, or static. Ragdoll parts share one collision group; a `GroupFilterTable` disables selected pairs. Static and dynamic props use the default group, so filtering inside the ragdoll cannot disable the floor, walls, cube, platform, or ramp.

The step function rejects re-entry. The application owns the fixed-step accumulator (120 Hz, at most 0.25 s of backlog and 6 steps per frame) so a hitch cannot spiral.

Joints are Jolt `SixDOFConstraint`s:

- Translation X/Y/Z are fixed. A limb cannot separate from its parent by stretching a soft positional spring.
- Angular motion is a cone or a pyramid of limits. Elbows, knees and hips use an anatomical neutral, so the imported crouch is not zero. Other joints still coincide at the imported pose.
- Angular friction torque is the passive resistance. Presets scale it. It stays in effect when motors are off and is not a motor parameter.
- Angular motors are position drives in constraint space. `MotorController::apply` is the only code that enables them and writes their targets. Balance and poses submit tasks. They do not call `setAngularMotor`. See `docs/MOTOR_SYSTEM.md` and `docs/BALANCE_SYSTEM.md`.
- Creating a joint locks both bodies with one `BodyLockMultiWrite`. Two separate write locks deadlock when Jolt hashes the bodies onto the same mutex. See `docs/GRAB_CRASH_INVESTIGATION.md`.

Jolt 5.3 does not support soft angular limits. Compliance is friction plus body damping, not a pose spring. That is deliberate: a motor targeted at the bind pose would be a hidden muscle.

## Character data

`assets/configurations/humanoidguy.json` lists anatomical bodies and joints: bone names, masses, limit angles, and whether a joint is a cone, a pyramid, or a hinge-oriented pyramid (elbow, knee, hip flexion). The builder refuses to continue if a referenced bone is missing, if a skinned bone is not driven, or if the constraint graph is not a single component. Hips are connected because the JSON says so, not because the FBX node parent happens to be the pelvis. In this file the hip bones are siblings of the pelvis.

v0.3.4 loads that file only. There is no character catalog. Another JSON with the same schema can still be built by `RagdollBuilder`; the application does not offer a second one. The importer requires skin bones.

The foot box is not perpendicular to the foot bone. Its sole normal is the visible underside: the average of the mesh normals that point downward in the bind pose, then the thin axis of the vertices that share that normal. Heel-to-toe stays the bone direction projected onto that plane. The half extents are the bounds of the same vertices on those axes. `StepController` does not write motors itself. It sets a reference offset and joint bias on `BalanceController`, which remains the only caller that builds the motor task.

## Rendering

Each mesh is rigidly weighted to one bone. Vertices are baked into that bone's local frame. Every frame the bone matrix is `bodyTransform * boneFromBody`, and the mesh is drawn with that matrix. Most collision shapes are capsules or spheres. The feet are boxes fitted to those vertices. Debug drawing uses the body pose, `shapeCenter`, and `shapeRotation`, which is the same placement Jolt simulates (`RotatedTranslatedShape` plus a center-of-mass shift back to the body origin). `PhysicsWorld::worldBounds` reads `Body::GetWorldSpaceBounds` so a test can compare that box with the drawn corners.

The viewport is a multisampled framebuffer (4x when the driver allows it) displayed in the docking layout. The default dock puts Hierarchy on the left, the viewport in the center, Inspector on the right, and Simulation / Physics / Diagnostics along the bottom.

## Identifiers

Every physical body keeps the anatomical id from the JSON (`pelvis`, `hip_1`, `thigh_1`, ...). Those ids are the stable names future balance, stepping, and injury controllers should use. The code does not assume a second character cannot be described by another JSON file with the same schema.
