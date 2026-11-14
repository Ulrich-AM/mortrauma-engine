# Motor system

v0.2.0 drives the existing ragdoll with torque-limited angular motors. The bodies stay dynamic. Nothing in this system sets a body transform in order to reach a pose.

## What a target is

A motor target is the relative orientation of a Jolt SixDOF constraint, in the constraint frame of body 1 (the parent). That is the same quaternion `SixDOFConstraint::GetRotationInConstraintSpace` returns, and the same value `JointState::constraintRotation` reports.

It is not a world rotation. Holding the elbows does not lock the arms to the world when the torso turns. The elbow target stays a rotation of the forearm relative to the upper arm.

For elbows and knees the v0.1.1 frame puts full extension at the identity quaternion. Positive swing 2, about constraint Z, is flexion, and the limit remains 0° to 145°. Hips use the same shared frame as v0.1.1: constraint X is down, positive swing 2 swings the thigh toward the toes. Shoulders and the other joints are still centered on the imported pose, so identity there means "keep the imported direction," not "point the limb down."

The controller does not subtract Euler angles to compute the error. It passes the quaternion to Jolt. Euler angles in the Motor Laboratory are only a display and an editing aid.

## The one writer

`MotorController::apply` is the only function that calls `PhysicsWorld::setAngularMotor`. Pose interpolation, the Motor Laboratory, and tests change targets and activations on the controller. The next `apply`, once per physics step, writes the constraint.

`setAngularMotor` then, for all three rotation axes:

1. Sets the spring to frequency-and-damping mode.
2. Sets the symmetric torque limit with `MotorSettings::SetTorqueLimit`.
3. Calls `SetTargetOrientationCS` with the constraint-space quaternion.
4. Calls `SetTargetAngularVelocityCS` with zero. That value is not the motor's strength.
5. Calls `SetMotorState` with `EMotorState::Position`, or `Off` when the joint is passive.

`SetMotorState` is last so Jolt rebuilds its "position motor active" cache from the frequency that was just written. A non-finite target, frequency, or torque limit is rejected and logged. The positional lock on the joint is never opened.

## Jolt motor mode and the torque limit

The drive is Jolt's built-in angular **position** motor on `SixDOFConstraint` (Jolt 5.3.0). The spring is `ESpringMode::FrequencyAndDamping`:

- `frequency` is the spring frequency in hertz. It is not a stiffness in N·m/rad and it is not a torque.
- `damping` is the damping ratio. 1 is critical for that frequency.

`MotorSettings.h` comments that the torque limits are "not used when the motor is a position motor." The SixDOF velocity solve does not follow that comment. For both velocity and position rotation motors it clamps the motor's accumulated angular impulse to `dt * torqueLimit`. MOT-002 measures the consequence: with the limit set to 3 N·m the estimated torque peaked at 3.00 N·m, while a 40 N·m limit on the same joint peaked at 36.4 N·m and moved the elbow about 48° farther in the same 0.12 s.

The estimate is `GetTotalLambdaMotorRotation() / dt`, in N·m, along the three rotational constraint axes. It is the motor impulse from the last step, not a separate torque sensor, and it is reported as zero while the motor is off. Friction impulse is `GetTotalLambdaRotation` and is not shown as motor torque. Saturation means any axis of that estimate has reached at least 90% of the configured limit.

Jolt does not expose a joint angular velocity. The value in the laboratory is the child body's angular velocity minus the parent's, projected onto the parent constraint axes. The label in the UI says so.

## Operating modes

| Mode | Behavior |
| --- | --- |
| Passive | Every angular motor is set to `Off`. Anatomical limits and passive friction remain. This matches a ragdoll that never had motors enabled (MOT-004 deviation was 0). |
| Compliant | Position motors use `assets/configurations/motors/compliant.json` (1.2 Hz, about a quarter of the active torque). |
| Active | Position motors use `active.json` (3.5 Hz, the torques listed there). |
| Hold Current Pose | Reads each selected joint's current constraint quaternion and stores that as the target, then runs the active profile. An empty selection captures every joint. A single joint or a region can be held instead. World transforms are not captured. |

Activation is a 0 to 1 value per joint. Group sliders write it for every joint in the group that is not overridden. Effective frequency is `profileFrequency * activation`. Effective torque is `profileTorque * groupStrength * activation`. The damping ratio is not scaled, so lowering activation does not make the spring underdamped. Below 0.02 the motor is switched fully off rather than left at a tiny frequency. Relaxation moves activation toward zero over a requested duration and ends in that passive state.

Joints are addressed by the ids in `humanoidguy.json` (`elbow_1`, `knee_2`, …). The controller does not store a Jolt body index as the joint's identity. Groups are Core (`spine_*`), Neck (`neck`), Shoulders and clavicles, Arms (elbows and wrists), Hips (`hip_*`, including the hip roots), and Legs (knees and ankles).

Passive friction stays on the constraint's `mMaxFriction`. Motor profiles do not change it.

## Failure

A target past a joint limit is allowed as a request. The limit constraint stops the joint. MOT-005 asked an elbow for 170° of flexion and the hinge stopped at 144.8° against a 145° limit, with the pivot still attached. The body is not teleported to close the error. Tracking error is the angle of `inverse(current) * target`.

A high-frequency, low-torque motor can transiently pass the extension stop by a few degrees while it is saturated (about 7° was observed at 12 Hz and 3 N·m) and then settle. That is solver give under a hard pull, not an extra range in the configuration. The settled MOT-001 extension error was 0.26°.

## Laboratory fixture

`PhysicsWorld::beginSupport` hangs a kinematic anchor on a body with a soft positional spring and free rotation. The Motor Laboratory uses it on the pelvis and labels it as a laboratory support fixture. It is not a balance controller. `Reset` and **Remove fixture** delete it. With it off, the scene is the same dynamic ragdoll as before.
