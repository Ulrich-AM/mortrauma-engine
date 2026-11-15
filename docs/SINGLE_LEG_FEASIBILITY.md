# Single-leg feasibility

v0.4.3 measured the static poses below and wrote both of them. v0.4.4 keeps those measurements as the original result and changes which pose is allowed to start the simulation. The later section records the corrected selection.

# Original search (v0.4.3)

The question is whether the current ragdoll can stand on one foot. It is not whether a larger knee command will peel a planted sole off the floor. Those trials were not repeated.

## What the 150 N·m figure is

77.5 kg times gravity times a 20 cm horizontal arm is about 150 N·m. That is the moment of the whole body about a point on the ground. It is not the torque the hip motor must produce.

If the center of mass is directly above the ankle, the ankle moment is small. The hip moment is the weight above the hip times the horizontal distance from the hip joint to that same vertical line. The knee moment is small when the knee lies on the line. The ground reaction is a force at the sole, not a motor.

The ankle limit is 12 N·m, so a static pose has to keep the center of mass within about 1.6 cm of the ankle. The sole is much larger than that disk. The foot half-extents are about 7.7 cm by 17.5 cm by 8.0 cm. Being inside the sole is not enough.

Solved contact impulses are still not available. The moments below are a static balance of gravity and one vertical contact force at the center-of-mass projection. They are not measured motor torques.

## Search

The pelvis stays upright. The stance hip, stance knee, swing hip, swing knee, and a lateral spine bend are sampled inside the joint limits. Each foot is leveled when the ankle limits allow it. The swing foot has to clear the floor. The pose is written only as a reset, before the first physics step.

## Right leg

A pose was found inside every current limit.

| Quantity | Value | Limit |
| --- | --- | --- |
| Body mass | 77.5 kg | |
| Center of mass to ankle | 1.5 cm | 1.6 cm for 12 N·m |
| Sole margin | 2.4 cm inside | |
| Swing-foot clearance | 57 cm | this is the reset, not a lift |
| Torso tilt | 0.046 rad | |
| Ankle moment | 10.2 N·m | 12 |
| Knee moment | 31.7 N·m | 35 |
| Hip moment | 38.2 N·m | 50 |
| Hip-root moment | 15.5 N·m | 18 |
| Spine moment | 24.5 N·m | 55 |

The knee is the tight joint. It is 3 N·m under its limit. The hip root on the swing side is 2.5 N·m under its limit.

## Left leg

The same search did not find a pose inside the limits. The closest one puts the center of mass 1.1 cm from the ankle, with an ankle moment of 7.5 N·m, a knee moment of 13 N·m, and a hip moment of 34 N·m. The stance hip root needs 20.3 N·m and its limit is 18. The torso tilt in that pose is 0.51 rad. The two legs are not the same shape: the right hip sits closer to a pose the limits can hold.

## Simulation

Placing the right-leg pose and running the torque-limited motors does not hold it.

With the normal 80 Hz task, the ankle command is already saturated at 0.05 s (the torque vector length is 20.8 N·m, which is 12 N·m on every axis) while the gravity moment about the ankle is still about 5 N·m. The servo is fighting the foot on the floor. The right foot stays down and the left foot stays clear, and the center of mass falls through 0.85 m at about 0.59 s. Torso tilt is then about 0.52 rad.

Turning the balance feedback off does not change that. Correcting only the stance hip, instead of both hips, changes it from 0.59 s to 0.58 s.

A 6 Hz task stops the saturation and the ankle only produces about 4 N·m, which is less than the pose needs. The hold is 0.40 s. A 16 Hz task lasts 0.54 s and the hip command climbs to about 60 N·m. Both were reverted. The two-foot frequency is unchanged.

No torque limit and no friction value was changed.

## What was not attempted

A transition from the two-foot stand was not added. The pre-positioned pose does not last one second, so a dynamic unloading sequence would be the same failure with a different entry. A 5 mm or 10 mm lift from two-foot standing was not claimed. The 57 cm clearance exists only because the reset starts with the foot already up.

## Why the two starts did not match (v0.4.4)

The angle grid is the same for both legs. Hip 2 keeps the mirrored swing1 sign from v0.4.1, and the arm joints stay on the standing reference. The arms are children of the chest, so a leaned chest carries them out; they are not a separate raised-arm target.

The selection was not the same as the feasibility gate. A pose counted as "light" when the torso was under 0.55 rad and the ankle, knee, hip, and spine moments fit, even if the hip root was over 18 N·m. The left pose was light (torso 0.51 rad, hip root 20.3 N·m) and was written. The right pose was actually feasible, and the swing-leg sweep then minimized the hip-root moment, which kept the swing knee at 110°.

v0.4.4 still searches that region, so a feasible folded pose is not lost. The written pose is the feasible candidate with the lowest posture cost. An infeasible candidate is not written. Its rejection names the failed check. The laboratory can place it only through "Place rejected pose (diagnostic only)", and that note is prefixed with "diagnostic only, not a standing pose."

## Corrected poses

The right pose is still feasible and is the one that is written.

| Quantity | v0.4.3 | v0.4.4 | Limit |
| --- | --- | --- | --- |
| Stance hip swing1 / swing2 / knee | 4° / 16° / 10° | 4° / 16° / 10° | |
| Torso tilt | 0.046 rad | 0.046 rad | 0.45 rad |
| Spine lateral | −1° | −1° | |
| Swing hip swing1 / swing2 | −30° / 15° | −30° / 15° | |
| Swing knee | 110° | 95° | |
| Hand offset from the upright pose | not scored | 2.0 cm | |
| COM to ankle | 1.5 cm | 1.3 cm | 2 cm |
| Sole margin | 2.4 cm | 2.6 cm | |
| Swing clearance | 57 cm | 47 cm | reset, not a lift |
| Ankle axis / vector / Mg·arm | 10.2 / — / — | 8.6 / 10.1 / 9.7 N·m | 12 |
| Knee / hip / hip root | 31.7 / 38.2 / 15.5 | 30.9 / 36.4 / 17.2 | 35 / 50 / 18 |
| Motor reference error | not checked | 0 rad | 0.25 rad |

The stance-ankle moment vector is 10.1 N·m and mass times gravity times the horizontal arm is 9.7 N·m. The gap is the foot's own weight and the height between the sole and the ankle. The largest axis component, 8.6 N·m, is smaller than the vector because the moment is not aligned with one constraint axis. The gate remains the per-axis peak against the existing motor limit. After the bodies are written, the measured constraint rotation matches the joint quaternion that was posed, so the motor reference is that configuration.

The hip root is now the tight joint, 0.8 N·m under 18. A calmer swing knee raises that moment over the limit, so 95° is the least-flexed feasible sample, not a relaxed stance. The search did not discard it to look more natural.

The left pose is not written. The closest rejected sample has the COM 0.8 cm from the ankle and the torso at 0.39 rad, with the swing knee at 100° and 13° of lateral spine. The stance hip root needs 24.0 N·m and the limit is 18. The rejection says the limit is `hip_root_2` and that the pose was not placed. A rejected search does not move a pose that was already written. The two hips are not a perfect mirror: their anatomical offsets differ by a few degrees, and the same Euler grid does not put both ankles under the COM inside 18 N·m.

## Simulation after the corrected pose

The two-foot preset is still 80 Hz. At 0.05 s the ankle command vector is 20.8 N·m, which is 12 N·m on every axis, while the balance estimate of the gravity moment is 3.7 N·m. Six motors are saturated. The right foot is down and the left foot is clear at 0.2 s, with the center of mass still above 0.90 m. The center of mass passes 0.85 m at 0.63 s and the torso tilt is then 0.47 rad. That is not a sustained single-leg stand.

The 6 Hz and 16 Hz trials from v0.4.3 were not repeated. Both lost the pose sooner than 80 Hz. No torque, friction, or frequency was changed. The 30 s two-foot stand, the 55 N·s fall, and the chest grab are unchanged. There is still no walking and no recovery step.
