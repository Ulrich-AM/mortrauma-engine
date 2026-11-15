# Swing-leg coordination (v0.4.3)

The swing foot in the feasible right-leg pose is raised by the reset. v0.4.3 used about 15° of hip flexion, 30° of hip swing, and 110° of knee flexion. v0.4.4 keeps the hip angles and reduces the knee to 95°, which is the least flexed sample that still fits the hip-root limit. The sole is leveled when the ankle can do it. The foot is not kinematic and its collisions stay on. The arms stay on the standing reference; a leaned chest is what moves them, and the left lean is no longer written.

That command is not a tracked foot target during the simulation. v0.4.2 already showed that a swing-knee flexion, with the foot on the floor, pitches the pelvis instead of raising the sole. The signed geometric Jacobian in that release pointed the knee the wrong way and was rejected. Neither result was rerun.

During the 0.63 s hold the left foot stays clear of the ground while the right foot keeps contact. The body then drops. There is no measured airborne interval produced by a lifting controller, and the initial clearance is not counted as one.

The swing-side hip root is part of the limit even when the stance joints fit. On the corrected right-leg pose it carries 17.2 N·m of an 18 N·m budget. On the left-leg pose the stance hip root is the joint that exceeds 18 N·m, at 24 N·m, and that pose is not written.
