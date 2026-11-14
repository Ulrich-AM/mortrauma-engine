# MORTRAUMA

MORTRAUMA v0.1.0 is a native C++ laboratory for passive, physically simulated humanoid reactions. This release is the application foundation (M0) and a fully dynamic ragdoll (M1). It does not balance, walk, or react intelligently. The body is a connected articulated rigid-body system: it falls, collides, tumbles, and settles.

The window title is **MORTRAUMA | Physical Reactions Laboratory**.

## Layout

```text
include/mortrauma/    public headers (no Dear ImGui in the engine)
src/core/             log, paths, math
src/physics/          Jolt adapter
src/rig/              character configuration and ragdoll builder
src/assets/           Assimp import
src/renderer/         OpenGL viewport
src/app/              GLFW window, docking UI, tools
assets/characters/    humanoidguy_rigged.fbx
assets/configurations character and joint presets
assets/environments/  test-lab description
tests/                CTest programs RIG-001..003 and PHY-001..006
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

The simulation starts paused on the imported bind pose so the rig can be inspected. Press **Play** (or Space) to let it fall. **Step** advances one fixed 1/120 s update. **Reset** (or R) restores poses and clears velocities and grab constraints.

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
- `docs/TESTING.md`
- `docs/ROADMAP.md`
