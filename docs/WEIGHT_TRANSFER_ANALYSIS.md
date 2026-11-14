# Weight transfer analysis (v0.4.1)

## What v0.4.0 actually measured

The v0.4.0 report said a leftward request moved the center of mass 48 mm toward the left foot and a rightward request moved it 23 mm away. That number was the peak of the center of mass projected onto the vector from the ankle equilibrium to the foot body, not the change from the start of the request.

At rest the center of mass is already about 33 to 46 mm toward character-left of the ankle equilibrium. `foot_1` (character right) sits about +234 mm along the character-right axis. `foot_2` sits about −309 mm. The two feet are not equidistant from the equilibrium, so the absolute projection is large and negative for the right foot even when the body has not moved. Replaying the same requests and subtracting the starting projection removes that offset.

## Hip frames

Both hip constraints are built with the same parent frame: twist down the thigh, swing1 about the character facing, swing2 about character-right. Positive swing2 flexes either thigh toward the facing direction, and that shared sign is correct. Positive swing1 does not abduct both thighs. It abducts one and adducts the other, because the axis direction is shared while the joints sit on opposite sides of the pelvis.

The same additive bias, with the standing controller still running, moved the center of mass as follows over about 1.2 s:

| hip 1 swing1 | hip 2 swing1 | Center-of-mass change along character-right | Torso tilt |
| --- | --- | --- | --- |
| +0.16 rad | −0.16 rad | +12 mm | 0.10 rad |
| −0.16 rad | +0.16 rad | −21 mm | 0.29 rad |
| +0.16 rad | +0.16 rad | −21 mm | 0.43 rad |
| −0.16 rad | −0.16 rad | −41 mm | 0.08 rad |

A shared sign cannot command a rightward shift. The standing balance law still uses that shared sign, and v0.4.1 does not change it. Quiet standing was not retuned. The weight-shift supervisor commands hip 2 swing1 as the negation of hip 1 swing1, with the same magnitude. That is an anatomical mapping, not a larger gain on one leg.

Ankle swing1 uses the same sign on both ankles. Opposing the ankles moved the center of mass by only a few millimetres. The left ankle already rests on the −20° twist limit. Knee flexion is positive swing2 on both knees; the hinge is mirrored when the joint is built. Neither ankles nor knees were given a new sign.

## Controller

The target is not the foot center. A foot is 23 to 31 cm from the midline. The whole body weight at a 15 cm arm is about 114 N·m, and the hip limit is 50 N·m. The supervisor picks a point 5.5 cm from the current center of mass toward the selected foot, stopping 6 cm short of the foot, and moves the balance reference toward that point at 6 cm/s. The hips aim at the finished point immediately. An integral of the lateral error, clamped to 0.22 rad and using the same gain for both directions, holds the shift against the pose that otherwise sits left of the ankles.

## Result

From a center of mass 33 mm left of the equilibrium, over 4 s, center of mass height stayed near 1.01 m:

| Request | Peak lateral position | Displacement toward the foot | End |
| --- | --- | --- | --- |
| Left foot | −86 mm | 53 mm | Still transferring. It reaches the target and does not stay there for the settle window. |
| Right foot | +14 mm | 47 mm | The shift completes and the reference returns to two-foot standing. |

Both requests move the body toward the selected foot. The rightward pair is the weaker of the two measured hip pairs, and the shift does not reach the foot. Torque limits were not raised.

v0.4.2 measured the same shifts again before changing the unload logic: 53 mm toward the left foot and 47 mm toward the right foot, with the center of mass still near 1.01 m. The support target was not moved closer to the foot. A 50 N·m hip can hold the full 77 kg only about 6 cm from the ankle if that whole-body moment is assigned to the hip. v0.4.3 does not use that assignment. Once the ankle is under the center of mass, the ankle moment is the weight times the remaining offset, and the hip moment is the load above the hip times the hip's own offset. See `docs/SINGLE_LEG_FEASIBILITY.md`.
