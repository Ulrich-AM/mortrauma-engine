# Single-leg feasibility (v0.4.3)

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
