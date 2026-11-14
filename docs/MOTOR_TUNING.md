# Motor tuning

Gains live in `assets/configurations/motors/`. Passive joint friction stays in the ragdoll presets (`loose`, `natural`, `firm`) and is not retuned here.

## Units

| Control | Unit | What it does |
| --- | --- | --- |
| Motor frequency | Hz | Jolt spring frequency of the position drive. Higher is stiffer. It is not a torque. |
| Damping ratio | dimensionless | 1 is critical damping for that frequency. |
| Torque limit | N·m | Symmetric cap on the motor impulse per step, divided by the timestep. |
| Activation | 0–1 | Scales frequency and torque. 0 turns the motor off. |
| Strength | torque scale | Multiplies the group's torque limit. 1 uses the profile. |
| Transition duration | seconds | How long pose targets take to slerp. A separate cap of 180°/s stops a very short duration from spinning the setpoint. |

The active profile used as the default is 3.5 Hz and damping ratio 1, with roughly these torque limits: spine 55/50/32 N·m, neck 12, clavicles 16, shoulders 26, elbows 18, wrists 6, hip roots 18, hips 50, knees 35, ankles 12. Compliant is 1.2 Hz and about a quarter of that torque. Those numbers are enough to move a limb and to lose to a hard push. They are not enough to stand the character up.

## Starting points

- One joint on the bench: frequency 4–6 Hz, damping ratio 1, torque near the profile value, friction can be lowered so the motor is what you see.
- A limb that buzzes: lower the frequency before raising the damping. Damping ratio above 1 makes the joint sluggish. It will not hide a target that jumps.
- A limb that will not lift: raise the torque limit, not the frequency. Frequency without torque saturates and then just vibrates against the limit. MOT-002's 3 N·m elbow was still near 69° after 0.12 s while the 40 N·m elbow had reached about 20°.
- A limb that should give way: lower activation or strength. Activation also lowers frequency, so the joint goes soft instead of staying stiff and weak. At 0 the motor state is `Off`.
- Full body: use the active profile before inventing higher gains. MOT-010 ran every motor at that profile for 1.5 s. Peak speed was 4.1 m/s, peak angular speed 28 rad/s, and the largest pivot separation was 0.9 mm. The body can still fall. That is not a tuning failure.

The solver stays at 12 velocity iterations and 3 position iterations. Do not crank those to hide a bad gain.

## Troubleshooting

**Oscillation.** The usual cause is frequency too high for the torque limit, or a target that the limit will never reach, so the motor stays saturated and chatters on the stop. Drop the frequency toward 2–4 Hz and keep the damping ratio at 1. Confirm the target quaternion is finite. The log records a rejected non-finite command.

**Saturation.** The laboratory marks saturation when the estimated motor torque (`|lambda| / dt`) is at least 90% of the limit on any axis. A saturated joint is doing all it was allowed to do. Raise the torque limit if it should win, or accept the sag if it should not. A 1.5 N·m elbow, the "Weak elbow" experiment, is not meant to hold the forearm against gravity.

**Wrong axis direction.** Elbow and knee flexion is positive swing 2 in the v0.1.1 frame. Identity is extension, not the crouch. If a pose flexes when it should extend, the target was built in the wrong frame (a world rotation, or a Mixamo local quaternion). Procedural poses and the importer build constraint quaternions. The joint overlay still draws red along extension, blue along positive flexion, and green on the hinge.

**Tracking error that never closes.** Read the error as the angle between the current constraint quaternion and the target. If the joint is on its limit, the error is the unreachable remainder and the status line will say the limit is reached. If another body is in the way, the motor will not win by opening the positional lock. If the motor is off, activation or the mode is still passive. Hold Current Pose stores the constraint quaternion from the moment you click it. Moving the torso afterwards does not retarget the arms in world space.

**The body flies apart.** It should not. Translation on every anatomical joint stays locked. If a separation grows past a couple of centimetres, treat it as a bug, not as a gain to damp out. MOT-010's full-body run stayed under 1 mm.

**Passive mode still resists.** That resistance is joint friction from the ragdoll preset, in N·m, plus body damping. It is not a hidden motor. MOT-004 compared a fall with motors explicitly off to a fall that never touched the motor API. The positions matched exactly.
