# M2 testing

Automated cases are `MOT-001` through `MOT-012` in `mortrauma_tests`, beside the existing RIG, PHY, and LIM tests. A parent body may be held with `setBodyPose` as a bench fixture, the same way LIM-001 holds a parent. The child body is never posed onto the target. The motor moves it.

Linux Release, 14 Nov 2026, g++ / Ninja, Jolt 5.3.0, single-threaded solver. `ctest --test-dir build --output-on-failure`: 25 of 25 passed (RIG-001–006, PHY-001–006, LIM-001, MOT-001–012) in 1.56 s.

The Motor Laboratory was compiled into `MORTRAUMA` and was not clicked through in this environment, so the panels are not reported as a manual pass. The Windows Release run is recorded in the release notes once GitHub Actions finishes.

## Measured results

| Test | Result |
| --- | --- |
| MOT-001 Single joint target | `elbow_1` swing 2 moved from 79.20° to 0.26° in 1.6 s. Extension error 0.26°. Separation 0.027 mm. Peak angular speed 5.88 rad/s. The motor stayed enabled. |
| MOT-002 Torque limiting | At 12 Hz, a 3 N·m limit peaked at 3.00 N·m and left the elbow at 68.6° after 0.12 s. A 40 N·m limit peaked at 36.4 N·m and reached 20.5°. The 3 N·m cap still read 3.00 N·m over a 0.5 s pull. |
| MOT-003 External disturbance | After the elbow had reached extension (−0.27°), an impulse peaked the hinge at 9.13° and the motor returned it to −0.35°. |
| MOT-004 Passive compatibility | A 0.45 s fall with motors explicitly off matched a fall that never called the motor API. Position deviation was 0. Every motor reported off. |
| MOT-005 Joint limit compliance | A 170° flexion target stopped at 144.82° against the 145° limit. Separation 0.11 mm. The pivot stayed locked. |
| MOT-006 Independent activation | Arming `elbow_1` enabled exactly one motor. The other 19 joints, including `elbow_2` and `knee_1`, stayed off through the following steps. Every joint resolved to a named muscle group. |
| MOT-007 Pose transition | An 0.8 s slerp to elbow extension finished at −0.18°. The largest target step was 0.83°. Peak angular speed was 2.53 rad/s. State stayed finite. |
| MOT-008 Hold Current Pose | Captured quaternion matched the live constraint quaternion (absolute dot product 1). An impulse displaced the hinge by 7.73°. Recovery error was 0.64°. No child transform was written. |
| MOT-009 Relaxation | At 67.3° the motor was switched off and the velocities were cleared. Over 0.45 s the passive hinge drifted to 69.0° (1.7°), not toward the target. Turning activation back up carried it to −1.66°. A 0.4 s relaxation then reported the motor off and activation at 0. |
| MOT-010 Full-body stability | All 20 motors at the active profile for 1.5 s under gravity. Peak speed 4.12 m/s, peak angular speed 28.1 rad/s, largest separation 0.91 mm, one constraint component, limits held. |
| MOT-011 Reference pose import | A missing file returned no targets. `Idle.fbx` is an animation; the first key was retargeted. Elbows 6.69° and 1.46°, knees 8.95° and 15.68°, all inside 0–145° and stored as pure hinge quaternions. Wrists, ankles, clavicles, hip roots, spine, and neck were reported unmapped and kept the Humanoidguy quaternion. The pose was written to `assets/poses/imported/mixamo_idle.json`. |
| MOT-012 Reset and repeated activation | Five cycles of activate, relax, and reset. Motors were off after every reset. Accumulated position error was 0. A saved configuration reloaded compliant mode and a `knee_2` activation of 0.4. A missing file was rejected. |

## What was not tested by hand

Pushing a raised arm in the viewport, dragging the support fixture, and reading the live graphs were not done interactively. The same situations are covered numerically by MOT-003 (impulse and return), MOT-008 (hold and return), MOT-010 (full-body motors under gravity), and the support-fixture API, which is the grab-style spring already covered by PHY-003's grab case. Do not treat that as a substitute for looking at the window.
