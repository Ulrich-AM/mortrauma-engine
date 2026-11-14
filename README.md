# MORTRAUMA

MORTRAUMA v0.4.0 is a native C++ laboratory for physically simulated humanoid reactions. The body is a fully dynamic ragdoll. Humanoidguy is the only character. v0.3.2 placed the center of mass over the ankles. v0.3.4 removed the Y-Bot experiment and remeasured a 30 s quiet stand. v0.4.0 turns the foot boxes onto the visible sole. That sole was about 33° off the old bone-perpendicular face. The quiet stand still reaches 30 s. The Stepping Laboratory can attempt one weight shift, one foot lift, or one recovery step. Those attempts are experimental: a lift currently topples the character, and a hard push still knocks him down. He does not walk. v0.1.1's anatomical joint frames are unchanged. See `docs/FOOT_ALIGNMENT.md` and `docs/RECOVERY_STEPPING.md`.

The window title is **MORTRAUMA | Physical Reactions Laboratory**.

## Layout

```text
include/mortrauma/    public headers (no Dear ImGui in the engine)
src/core/             log, paths, math
src/physics/          Jolt adapter
src/rig/              character configuration and ragdoll builder
src/motors/           torque-limited angular motor controller
src/balance/          center-of-mass estimate and standing controller
src/poses/            procedural poses and optional Mixamo retarget
src/assets/           Assimp import
src/renderer/         OpenGL viewport
src/app/              GLFW window, docking UI, tools
assets/characters/    humanoidguy_rigged.fbx
assets/configurations character rig, joint presets, and motor profiles
assets/poses/         procedural pose parameters and an optional Mixamo reference
assets/environments/  test-lab and standing-scene descriptions
tests/                CTest programs RIG, PHY, LIM, MOT, GRAB, BAL, FOOT, EQ, STAB, ALIGN, and STEP
```

The engine target `mortrauma_engine` does not link Dear ImGui or OpenGL. The executable does.

## Dependencies

Fetched and pinned by CMake (`FetchContent`):

| Library | Pin |
| --- | --- |
| Jolt Physics | v5.3.0 |
| GLFW | 3.4 |
| Dear ImGui | v1.91.8-docking |
| Assimp | v5.4.3 (FBX and glTF importers only) |
| nlohmann/json | v3.11.3 |

Jolt is built with its default instruction set for the compiler, which includes AVX2 on this configuration. A packaged Windows build targets machines with AVX2.

## Build

Windows (the release configuration):

```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
build\bin\Release\MORTRAUMA.exe
```

The Windows executable is linked against the static MSVC runtime, so a clean machine does not need the Visual C++ redistributable. OpenGL 3.3 is provided by the graphics driver.

Linux is supported for development and for the automated tests:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/bin/MORTRAUMA
```

Assets are resolved from the executable directory first (`assets/` next to the executable), then from parent directories and the source tree recorded at configure time. The process working directory is not used.

## Using the laboratory

The simulation starts paused on the imported bind pose so the rig can be inspected. Press **Play** (or Space) to let it fall. **Step** advances one fixed 1/120 s update. **Reset** (or R) restores poses, clears velocities and grab constraints, removes the laboratory support fixture, and switches motors off.

The **Motor Laboratory** selects Passive, Compliant, Active, or Hold Current Pose, edits muscle groups and individual joints, and transitions to procedural or imported poses. Movement comes from the joint motors. Resetting or choosing Passive returns the passive ragdoll. The pelvis support button is a laboratory fixture, not balance.

The **Balance Laboratory** enables a separate standing mode. It does not turn on when Active is selected. **Reset to standing test pose** places the character once, then the motors try to hold it. Presets are Conservative, Normal, and Aggressive. The panel also applies a known impulse at the chest, pelvis, or head. Balance refuses the pelvis fixture. On capsule feet the quiet stand lasts about a second and then the character falls. See `docs/BALANCE_SYSTEM.md`.

- Right drag orbits, middle drag pans, the wheel zooms.
- **Select** picks a rigid body.
- **Grab** attaches a soft positional constraint to a kinematic target while the mouse is held. Limbs follow through the joints. Rotation of the grabbed body stays free.
- **Push** applies an impulse at the hit point. Strength and direction are on the Simulation panel.

Joint presets **Loose**, **Natural** (default), and **Firm** change passive friction, damping, and limit scale. They do not pull the body back to the bind pose. **Save preset JSON** writes `configurations/user_preset.json` next to the executable.

## Character asset

`assets/characters/humanoidguy_rigged.fbx` is the default humanoid, supplied with this project by the repository owner. It is a rig of separate rounded capsule meshes and a skeleton. No third-party license file was included with the asset. If that file is missing, the application reports the failure in the log and in the viewport and spawns a clearly labeled procedural stand-in so the laboratory still runs. It does not pretend the stand-in is the imported character.

See `docs/RIG_SPECIFICATION.md` for the bone-to-body mapping.

## Documentation

- `docs/ARCHITECTURE.md`
- `docs/RIG_SPECIFICATION.md`
- `docs/PHYSICS_CONFIGURATION.md`
- `docs/MOTOR_SYSTEM.md`
- `docs/MOTOR_TUNING.md`
- `docs/POSE_SYSTEM.md`
- `docs/TESTING.md`
- `docs/M2_TESTING.md`
- `docs/BALANCE_SYSTEM.md`
- `docs/BALANCE_TUNING.md`
- `docs/BALANCE_TESTING.md`
- `docs/GRAB_CRASH_INVESTIGATION.md`
- `docs/ROADMAP.md`
