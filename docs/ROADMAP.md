# Roadmap

v0.1.0 ships M0 and M1 only. Later phases should add behavior on top of the existing bodies, joint ids, and motor fields. They should not replace the physics adapter or bake a single character into the simulation loop.

## M2 — Active joint motors

Turn on the torque-limited motors that v0.1.0 already stores (frequency, damping, torque limit) without changing joint identities. Targets are explicit controller setpoints, not an implicit return to the bind pose.

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

v0.1.0 does not implement any of M2–M10.
