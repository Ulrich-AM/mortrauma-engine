# MORTRAUMA v0.1.0

Passive ragdoll laboratory. No balancing, walking, or injury behavior.

## Implemented

- Native C++20 application, OpenGL 3.3 viewport, Dear ImGui docking, dark compact UI.
- Jolt Physics adapter with fixed 120 Hz stepping, a bounded accumulator, and a single-threaded deterministic solver.
- Humanoidguy imported from FBX: 28 bones, 56 capsule meshes, 12,656 vertices. Meshes stay the render geometry. Collision shapes are capsules.
- 21 dynamic bodies and 20 joints in one constraint component. Both legs are constrained to the pelvis even though the FBX parents the hips to the rig root, not to the pelvis. Positional degrees of freedom are locked. Angular limits are anatomical and passive.
- Mouse orbit, pan, zoom, select, grab (soft positional constraint), and push (impulse at the hit point).
- Floor, two walls, a dynamic cube, and optional platform and ramp.
- Loose, Natural, and Firm presets. Values can be edited and saved to JSON. Motors that would hold the bind pose are present on the joints and left off.
- Diagnostics: collision shapes, centers of mass, joint anchors, axes, contacts, velocity, timestep, physics and render FPS, body and constraint counts, disconnected components, and the last error.
- CTest: RIG-001, RIG-002, RIG-003, PHY-001 through PHY-006.

## Tests

Linux Release, Clang 18, Jolt 5.3.0, 14 Nov 2026: all 9 tests passed.

Windows Release, MSVC 19.51 (Visual Studio 18) and Ninja, GitHub Actions run [43537435747](https://github.com/Ulrich-AM/mortrauma-engine/actions/runs/43537435747) on `ae14226`: all 9 tests passed. The uploaded package is `MORTRAUMA-v0.1.0-Windows-x64.zip` (about 6.4 MB) and contains `MORTRAUMA/MORTRAUMA.exe` with no extra runtime DLLs.

PHY-005 replay deviation was 0. PHY-002 settled to sleeping bodies with the lowest center at 0.081 m.

Interactive viewport checks (clicking grab and push, toggling every debug view) were not performed on a physical desktop. The paused bind pose was captured under Xvfb and shows the imported capsules, joint axes, and a single connected component.

## Known limitations

- The FBX bind pose is a crouch, not a neutral stand. v0.1.0 does not repose it.
- Bone pivots in the file are a 180 degree yaw away from the skinned vertices. The importer corrects that and reports it. It is not a general FBX repair.
- Jolt 5.3 cannot soften angular limits. Passive resistance is friction torque and damping.
- Angular limit directions for hips and knees are heuristic. They are wide enough to collapse, not a clinical range of motion.
- The Windows package requires an AVX2 CPU because Jolt is compiled with that instruction set.
- No active balance. The character will fall.

## Package

`MORTRAUMA-v0.1.0-Windows-x64.zip` contains `MORTRAUMA/MORTRAUMA.exe`, `assets/`, `configurations/`, and `README.txt`. The executable loads assets relative to itself. The MSVC runtime is linked statically.
