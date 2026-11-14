# Testing

Automated tests are a CTest executable, `mortrauma_tests`, with one case per name. RIG, PHY, MOT, GRAB, FOOT, BAL, EQ, STAB, ALIGN, and STEP load `humanoidguy_rigged.fbx`. A missing Humanoidguy asset fails those tests. The procedural stand-in is not accepted as a pass. `mortrauma_tests DUMP` prints bind positions and is not part of CTest. `mortrauma_tests STAB-LOG` prints the standing trial and is not part of CTest.

v0.2.0 adds MOT-001 through MOT-012. The motor numbers are in `docs/M2_TESTING.md`. v0.3.0 adds GRAB-001 through GRAB-007 and BAL-001 through BAL-014. The standing numbers and the cases that are expected to fall are in `docs/BALANCE_TESTING.md`. The table below is the original v0.1.0 record.

The run recorded below is a Linux Release build (Clang 18, Jolt 5.3.0, single-threaded solver) on 14 Nov 2026. All 9 tests passed. Total CTest time was about 0.7 s.

```text
RIG-001  Passed
RIG-002  Passed
RIG-003  Passed
PHY-001  Passed
PHY-002  Passed
PHY-003  Passed
PHY-004  Passed
PHY-005  Passed
PHY-006  Passed
```

## Tolerances

| Test | Pass condition |
| --- | --- |
| RIG-001 | 21 bodies, 20 joints, exactly one constraint component, 28 bones, 56 meshes, no unweighted mesh |
| RIG-002 | Both feet, thighs, and hips are reachable from the pelvis. Every joint locks translation. Bind-pose anchor separation < 5 mm. Hip joints connect a hip body to a thigh. |
| RIG-003 | After 2 s of passive motion, every finite speed stays under 40 m/s or rad/s, joint angles stay inside the configured limits plus 0.2 rad, separation < 2 cm |
| PHY-001 | After 0.8 s the pelvis has dropped at least 15 cm, motion stays finite and under 30 m/s, every ragdoll body is still dynamic |
| PHY-002 | After 6 s, linear speed < 0.35 m/s, angular speed < 2 rad/s, no body center below y = -0.3 m, graph still connected, separation < 2.5 cm, the dynamic cube rests within 8 cm of y = 0.2 |
| PHY-003 | A 4 N·s impulse on a 2 kg free body produces 2 m/s ± 0.05. A 25 N·s impulse on the pelvis increases its horizontal speed by more than 0.2 m/s. A grab spring lifts the probe more than 15 cm toward a target 50 cm above it (measured lift 0.50 m), and releasing the grab removes the extra constraint. |
| PHY-004 | After 1 s and a reset, every body is within 0.1 mm of its initial pose and velocity is below 1e-4 |
| PHY-005 | Two replays of 2 s with the same impulse at step 60 differ by less than 0.1 mm. The measured deviation was 0. |
| PHY-006 | Impulses of 5, 20, and 80 N·s on the chest, in six directions, applied off-center. After 1.5 s each, motion is finite, speed is under 60, separation is under 5 cm, and the graph is still one component. |
| RIG-004 | Bind-pose elbow and knee swing 2 matches the geometric bend (about 79° and 85–89°), not 0°. A straight pose reads about 0° and is inside the limits. 25° of hyperextension and 160° of flexion are outside. Pivots stay under 1 cm. |
| RIG-005 | Adding 20° of flexion increases swing 2 by about 20° on both elbows and both knees. |
| RIG-006 | Hip flexion at the crouch is about 71–74° from vertical. A thigh pointing down reads 0° and is inside −30° to 110°. 45° of hip hyperextension is outside. Abduction signs are opposite. Each shoulder cone of 80° contains an arm hanging at the side (about 36–38°). |
| LIM-001 | With the parent held, zero gravity and the body lifted clear of the floor, both elbows and both knees reach the 0° extension limit (solver slop about −0.2°). Separation stays under 2 cm. The same drive with 2 N·m friction reaches under 15°. Contacts may exist; the stop that matters is the limit flag. |

PHY-002 on this run settled to exactly 0 linear and angular speed (bodies slept) with the lowest body center at y = 0.081 m.

A test is not a pass merely because the process stayed alive. Each case asserts the numeric condition above.

## Manual checklist

These items need a person at the window. An automated screenshot of the paused bind pose was captured under Xvfb with llvmpipe (`docs` does not store the image; the run log showed a successful import, 21 bodies, 20 constraints, and 1 component). Interactive mouse checks were **not** executed on a physical desktop in this environment.

| Check | Result |
| --- | --- |
| Window title and dark, sharp-cornered docking layout | Observed in the Xvfb capture: Hierarchy, Viewport, Inspector, Simulation, Diagnostics |
| Humanoidguy capsules visible in the authored crouched pose, feet at the grid | Observed in the capture. Joint axes sat on the limbs. |
| Play collapses the ragdoll onto the floor without a visible explosion | Not executed interactively. PHY-001 and PHY-002 cover the same scenario without rendering. |
| Pause and single step | Not executed interactively. The step function is what the tests call. |
| Reset restores the pose | Covered by PHY-004. The button calls the same `resetRagdoll`. |
| Grab drags a limb and the body follows; release drops it | Grab spring covered by PHY-003 on a free body. Mouse picking in the viewport was not exercised by hand. |
| Push at an off-center point spins a part | Impulse-at-point is the API tested by PHY-003 and PHY-006. The mouse path was not exercised by hand. |
| Wireframe, collision, contacts, centers of mass | Toggles exist. Not clicked by hand. Joint axes were visible by default in the capture. |
| Loose / Natural / Firm and save JSON | Preset files load through the same function the tests use. The save button was not clicked by hand. |
| Platform and ramp toggles | Not clicked by hand. They create and destroy static boxes through the physics adapter. |

## Windows

`.github/workflows/windows-release.yml` builds on `windows-latest` with the installed MSVC toolset (the image no longer exposes the Visual Studio 2022 CMake generator) and Ninja.

GitHub Actions run [43537435747](https://github.com/Ulrich-AM/mortrauma-engine/actions/runs/43537435747), commit `ae14226`, MSVC 19.51: configure, compile, and all 9 CTest cases passed. The artifact `MORTRAUMA-v0.1.0-Windows-x64.zip` contains `MORTRAUMA/MORTRAUMA.exe` (3.8 MB), `assets/`, `configurations/`, and `README.txt`. No extra DLLs were produced; the MSVC runtime is linked statically. The executable was not launched on a Windows desktop in this session.
