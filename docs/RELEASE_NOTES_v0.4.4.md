# MORTRAUMA v0.4.4

v0.4.4 corrects which single-leg pose is allowed to start. v0.4.3 wrote a left-leg pose whose torso leaned 0.51 rad and whose hip root needed 20.3 N·m, over the 18 N·m limit. The arms moved because they are attached to that chest. The right leg started more upright, with the swing knee at 110°.

The search grid is unchanged. A pose is written only when it is inside the sole, within 2 cm of the ankle, and inside every motor limit, including the hip root. Among those poses the calmer one is kept. The left pose is not written. The note names `hip_root_2` at 24 N·m. A checkbox labeled "diagnostic only" can show a rejected pose, and the note says it is not a standing pose.

The right pose that is written keeps the torso at 0.046 rad and the swing hip at −30° and 15°, and reduces the swing knee from 110° to 95°. The hip root is 17.2 N·m of 18, so a straighter knee does not fit. The stance-ankle moment vector is 10.1 N·m against 9.7 N·m from mass times gravity times the COM arm. The motor reference matches the written joints.

Simulating that pose does not produce a single-leg stand. At 0.05 s the ankle command is saturated at 20.8 N·m while the gravity moment is about 3.7 N·m. The body drops through 0.85 m at 0.63 s. The 30 s two-foot stand is unchanged. Nothing was added for walking or a recovery step, and no torque limit was raised.

`MORTRAUMA-v0.4.4-Windows-x64.zip` contains `MORTRAUMA.exe`, Humanoidguy, and the configurations. Releases v0.1.0 through v0.4.3 are not replaced.
