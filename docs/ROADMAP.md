# Roadmap

v0.2.0 ships M0, M1, and M2. Later phases should add behavior on top of the existing bodies, joint ids, and `MotorController`. They should not replace the physics adapter, write Jolt motor targets themselves, or bake a single character into the simulation loop.

## M2 — Active joint motors

Implemented in v0.2.0. Torque-limited angular position motors drive constraint-space targets. Muscle groups and per-joint overrides select which joints are armed. `MotorController::apply` is the single command path. The character stays fully dynamic: motors can fail, saturate, and be turned off. There is no balance controller. See `docs/MOTOR_SYSTEM.md` and `docs/M2_TESTING.md`.

## M3 — Autonomous standing

A balance controller that holds the center of mass over the support polygon using the hip, spine, and ankle motors. The ragdoll stays fully dynamic. Failure falls through the passive joints instead of being projected upright.

## M4 — Momentum and recovery stepping

Detect a loss of balance and take a step. The existing hip and knee limits are the range the step has to respect. Foot bodies are the contactors.

## M5 — Fall protection

Choose a fall direction and use the arms. Shoulder and elbow motors from M2, with the same locked positional degrees of freedom so the arms cannot detach.

## M6 — Wall grabbing

Ray or contact queries against the existing static environment. A temporary constraint of the same kind as the mouse grab, attached at a hand body.

## M7 — Getting up

A staged motor sequence from a settled pose back to the M3 stand. Reset and the bind pose remain available as a laboratory control, not as the recovery strategy.

## M8 — Wound holding and injury

Per-body damage that reduces motor torque limits and tightens or opens joint limits. Body ids stay stable so an injury map can name `thigh_1` without knowing the Jolt body index.

## M9 — Gaze and attention

A look target driving `head` and `neck` only. No new physical bodies. `HeadF` stays an auxiliary visual bone.

## M10 — Behavior arbitration

A single owner of motor setpoints so standing, stepping, falling, grabbing, and injury do not write the same joint in the same step. The physics world remains unaware of behaviors. It only applies the motors and integrates.

v0.2.0 does not implement M3–M10. M2's motor controller is the setpoint owner those phases should share.
