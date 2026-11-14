# MORTRAUMA v0.4.0

v0.4.0 turns Humanoidguy's foot boxes onto the visible sole and adds a first, experimental recovery-step attempt.

The v0.3.4 box was perpendicular to the foot bone. That face was 33° off the right shoe and 31° off the left, so leveling the box left the mesh pitched. The sole now comes from the underside normals of the foot mesh. The box is still the bound of those vertices, not a larger pad. Mass, friction, and the 12 N·m ankle limit are unchanged. After placement the visible sole and the collider agree within 2°. The Normal quiet stand is still upright at 30 s (center of mass 1.028 m, tilt 0.093 rad, drift 27 mm). A push to the character's left that v0.3.4 survived now topples him. The left ankle uses its twist limit to hold the sole level.

The Stepping Laboratory can request one weight shift, one foot lift, or one recovery step. The balance controller remains the only motor writer. Nothing pins a foot. A shift toward the left foot moved the center of mass about 48 mm and stayed up. A shift toward the right foot did not. A foot lift gained about 5 mm while the body was upright, then the character fell. A 55 N·s push still falls. This is not a walking controller.

`MORTRAUMA-v0.4.0-Windows-x64.zip` contains `MORTRAUMA.exe`, Humanoidguy, and the configurations. Releases v0.1.0 through v0.3.4 are not replaced.
