# MORTRAUMA v0.4.1

v0.4.1 teaches the existing balance controller to shift Humanoidguy's weight toward either foot. The body stays a torque-limited ragdoll. He does not walk.

The v0.4.0 weight shift looked one-sided because the measurement included a center of mass that already sits left of the ankles, and because both hips were given the same swing1 sign. Those hip frames share one facing axis, so that sign abducts one thigh and adducts the other. The standing law is unchanged. A weight shift commands the opposite sign on hip 2, with the same magnitude, and aims a few centimetres toward the selected foot rather than at the foot center.

From a rest offset of 33 mm to the left, the center of mass moved 53 mm further toward the left foot and 47 mm toward the right foot, with the center of mass still near 1.01 m. The right-foot shift completes and returns to two-foot standing. The left-foot shift reaches the target and does not settle inside the request. The swing foot keeps its contacts, so a foot lift is refused. A trial that lifted anyway rose about 11 mm and then fell. That trial is not the shipped behavior. A 3 cm lift was not achieved.

The 30 s quiet stand still finishes upright: center of mass 1.028 m, tilt 0.096 rad. A 55 N·s push still falls. Grab and push still move the chest. Nothing pins the feet.

`MORTRAUMA-v0.4.1-Windows-x64.zip` contains `MORTRAUMA.exe`, Humanoidguy, and the configurations. Releases v0.1.0 through v0.4.0 are not replaced.
