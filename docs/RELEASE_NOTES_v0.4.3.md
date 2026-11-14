# MORTRAUMA v0.4.3

v0.4.3 asks whether Humanoidguy can stand on one leg with the motors he already has. He cannot hold it. The limits were not raised, and nothing pins a foot.

A static pose on the right leg does fit. The center of mass is 1.5 cm from that ankle, inside the sole, with the torso at 0.05 rad and the other foot clear. The static moments are 10 N·m at the ankle, 32 at the knee, 38 at the hip, and 15 at the hip root. The limits are 12, 35, 50, and 18. The knee is the joint with the least margin.

The same search on the left leg needs 20 N·m at the hip root. The limit is 18, so that pose is rejected.

Placing the right-leg pose and simulating it lasts about 0.6 s before the center of mass drops below 0.85 m. The ankle motor is already saturated at 0.05 s, while the gravity moment is still about 5 N·m: the position task is pushing the foot against the floor. Slower tasks were tried and held the pose for less time, so the normal task rate was kept. Two-foot standing is unchanged and still lasts 30 s.

The clear foot in that pose is part of the reset. It is not a step, and a lift from two-foot standing was not added.

`MORTRAUMA-v0.4.3-Windows-x64.zip` contains `MORTRAUMA.exe`, Humanoidguy, and the configurations. Releases v0.1.0 through v0.4.2 are not replaced.
