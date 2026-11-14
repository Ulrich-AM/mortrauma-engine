# MORTRAUMA v0.2.0

Active, torque-limited joint motors on the v0.1.1 ragdoll. The character can attempt a pose, resist a disturbance, relax, and yield. It still does not balance, walk, grab, get up, or play an animation. Every anatomical joint keeps its v0.1.1 frame, limit, and positional lock. v0.1.0 and v0.1.1 tags are unchanged.

## Motors

- Jolt SixDOF angular position motors, target in the parent constraint frame, spring in hertz and damping ratio, symmetric torque limit in N·m.
- The velocity solve clamps motor impulse to `dt` times that limit. A 3 N·m elbow peaked at 3.00 N·m in MOT-002. The comment in Jolt's `MotorSettings.h` that position motors ignore torque limits does not match this solver.
- One writer, `MotorController::apply`, addressed by joint id (`elbow_1`, `knee_2`), never by a Jolt body index.
- Modes: Passive, Compliant, Active, Hold Current Pose (whole body, one joint, or a region). Activation 0 switches the motor off. Passive friction is untouched.
- Groups: Core, Neck, Shoulders and clavicles, Arms, Hips, Legs, with per-joint overrides.

## Poses

Procedural poses: Neutral, Arm Raise (right and left), Arm Extension, Leg Flexion, Compact, Torso Resistance, and Hold Captured Pose. Transitions slerp the constraint quaternion and cap the setpoint rate at 180°/s.

An optional Mixamo idle (`assets/poses/imported/Idle.fbx`) is retargeted from one animation frame by bone direction. Elbows and knees become hinge angles. Unmapped joints keep the Humanoidguy reference and are listed. The clip is not played back. The Humanoidguy mesh is unchanged.

## Laboratory

The Motor Laboratory window edits mode, groups, per-joint targets and gains, pose transitions, and live graphs. Graphs show the constraint angle, the body-to-body angular velocity on constraint Z, the quaternion error, motor impulse divided by the timestep, and saturation. Missing Jolt measurements are not filled in.

A support fixture can suspend the pelvis from a kinematic spring. It is labeled as a fixture and is removed with its own button or with Reset. It is not balance.

## Tests

Linux Release: 25/25 CTest cases passed, including MOT-001 through MOT-012 and the existing RIG, PHY, and LIM tests. Numbers are in `docs/M2_TESTING.md`.

## Package

The Windows artifact is `MORTRAUMA-v0.2.0-Windows-x64.zip`. The workflow reads the version from `PACKAGE_VERSION` instead of a filename frozen at v0.1.0. The zip contains `MORTRAUMA.exe`, the assets (character, motor profiles, poses, the optional Mixamo reference), and the static MSVC runtime. AVX2 is required through Jolt. OpenGL 3.3 comes from the driver.
