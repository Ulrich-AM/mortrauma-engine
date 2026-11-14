# Single-leg support control (v0.4.2)

Single-leg support was not established.

The stance model while both feet touch is still the two-foot hull. The controller does not switch to the stance foot alone, because the swing foot never stops providing contact. The support text says when the full weight at the current arm would exceed the 50 N·m hip. That sentence is an estimate from mass, gravity, and the horizontal distance to the stance foot. It is not a measured joint torque and it is not a load fraction.

The stance foot's horizontal speed is monitored. A lift is not entered while that speed is above 0.12 m/s, and a slip above 0.20 m/s cannot enter the lift. In the unloading trials the stance foot did not slip; the sole simply did not rise.

Ankle and hip lateral signs are the v0.4.1 pair: hip 1 keeps the balance-law sign and hip 2 is mirrored, with the same magnitude. The two-foot standing preset was not changed. There is no separate single-leg gain. Adding one would not move the center of mass onto a foot that is outside the hip's moment budget.

If the torso tilt passes 0.32 rad, the center of mass falls below 0.94 m, or the stance foot loses contact during unloading, the swing command is reduced and two-foot standing is restored when the feet are still on the ground. A 55 N·s push still falls with no hidden support.
