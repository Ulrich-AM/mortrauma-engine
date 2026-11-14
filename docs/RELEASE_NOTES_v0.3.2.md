# MORTRAUMA v0.3.2

Standing equilibrium. One Humanoidguy, the same 21 bodies and 20 joints, the same flat soles, the same 12 N·m ankles.

v0.3.1 stood for 1.39 s and fell forward. The pose aimed the center of mass at a 75% heel / 25% forefoot blend, about 60 mm ahead of the ankles, and the controller then chased the middle of the contact hull. Half the body weight at that arm is about 23 N·m per ankle. The motors cannot hold it.

v0.3.2 places and tracks the center of mass over the ankles, where that static moment is zero. On the Normal preset the quiet stand now lasts 30 s (EQ-010; the primary goal was 10 s). The torso still sways by about 10°. A 6 N·s push is rejected. A 55 N·s push still knocks the character down. There is no step and no get-up.

The unloaded left foot is still about 12 mm higher than the right. Contact brings the soles within 2 mm in the first second. Closing that gap with the knees, then re-leveling the ankles, makes the stand fall, so that correction is not enabled. Solved contact impulses are not available from this Jolt pin, so center of pressure and foot load fractions are reported as unavailable.

Torque limits, foot friction, and the motor gains are unchanged. The Standing Equilibrium panel and a CSV recorder are in the Balance Laboratory.

`MORTRAUMA-v0.3.2-Windows-x64.zip` contains `MORTRAUMA.exe`, the Humanoidguy asset, configurations, and the runtime note. Releases v0.1.0 through v0.3.1 are not replaced.
