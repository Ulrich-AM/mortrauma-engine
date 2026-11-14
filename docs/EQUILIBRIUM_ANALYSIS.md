# Equilibrium analysis

v0.3.2 asks whether Humanoidguy can stand still on the existing 12 N·m ankles, or whether some other limit is what knocks him forward. The measurements below are from the Normal preset, one character, no pelvis fixture, fixed step 1/120 s. The v0.3.1 trace is `docs/data/standing_baseline_v031.csv`. The same recorder on the v0.3.2 pose is `docs/data/standing_v032.csv` (10 s of a stand that continues).

## What v0.3.1 was doing

The flat soles were level. They were not on one plane.

| Foot | Heel height | Forefoot height |
| --- | ---: | ---: |
| Right (`foot_1`) | 4.1 mm | 4.4 mm |
| Left (`foot_2`) | 15.0 mm | 17.6 mm |

Both sole normals were within a degree of vertical. The left foot simply started 12 mm higher, because one rigid lift uses the lowest point on the whole body. The quiet stand lasted 1.39 s and became unrecoverable at 1.40 s (phase Falling, center of mass still 0.91 m, torso 0.56 rad, forward velocity 0.83 m/s). At 2.0 s the center of mass was 0.185 m and the phase was Grounded.

The pose put the center of mass on a 75% heel / 25% forefoot blend. That blend is not the ankle. On this rig the ankle sits 43 mm ahead of the heel mark, and the sole is about 314 mm long, so the blend is about 35 mm ahead of the ankle. The placed center of mass was 60 mm horizontally from the ankles. With the feet sharing 77.5 kg, half the weight is about 380 N, and 380 N times 60 mm is 23 N·m at each ankle. The motor limit is 12 N·m, which can hold a forward arm of about 32 mm (12 / 380). The heel blend was already outside that, and the controller then steered toward the contact-hull centroid, which is further forward still. A center of mass behind that centroid produces a positive swing2 command, and positive swing2 pitches the torso further forward. The heels were a symptom. The target was the cause.

## A target the ankles can hold

The static moment about an ankle is the vertical load times the forward distance from that ankle to the center of pressure. Putting the center of mass over the ankles makes that arm zero. `feasibleSolePoint` is the ankle's ground projection, shifted by `equilibriumLeadM` toward the toes and clamped to the sole. The lead used here is 0.

The standing pose now matches the center of mass to that point instead of the 75/25 blend. After placement the horizontal miss is 6.9 mm. The controller tracks the same point, recomputed from the live ankle anchors, instead of the contact centroid. Motor targets are still the constraint orientations Jolt reports after the pose is written.

At 12 N·m and equal load, a forward arm past about 32 mm is not a static equilibrium for the ankles alone. The pose is inside that. During the stand the center of mass wanders about 40 mm from the target in the horizontal plane, but the forward component of that offset is smaller: the equal-load pitch estimate stays near 5 N·m. The hips carry the rest. The ankle limit was not raised.

## What 30 seconds looks like

Normal preset, equilibrium target, no gain change, no friction change:

| Time | COM height | Torso tilt | COM to target | Pitch estimate | Margin | Phase |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| 1 s | 1.038 m | 0.099 rad | 33 mm | 5.8 N·m | +46 mm | Standing |
| 10 s | 1.032 m | 0.190 rad | 42 mm | 5.6 N·m | +58 mm | Standing |
| 30 s | 1.032 m | 0.166 rad | 38 mm | 5.0 N·m | +57 mm | Standing |

The torso sways between about 0.13 and 0.22 rad. It does not run away. Two joints sit near saturation. Per-axis torque does not exceed the configured limit (EQ-009, overshoot 0). A 6 N·s chest push is now rejected. A 55 N·s push still knocks the body down, and the pelvis is not anchored.

## Coplanarity

Proposed tolerances, before the first physics step: sole normal within 1° of vertical, heel-to-forefoot under 3 mm, left-to-right under 3 mm.

| Check | Unloaded pose | After 1 s of contact |
| --- | ---: | ---: |
| Sole normal | 0.45° | stays under 1° |
| Heel to forefoot | 1.8 mm | under 3 mm |
| Left to right | 12.5 mm | 1.9 mm |

The unloaded left-right gap is the one that misses. Extending the higher knee and then searching the ankles again for a level sole walks the ankle angles off the zero-moment pose, and that character falls (1.6 s and 2.6 s in the two attempts). A single knee nudge of under 2.5°, without a second ankle search, does not shrink the gap, so it is discarded. The ankles are already on their twist limit (±20°) to kill sole roll. Differing the knees asks for a sole orientation those limits do not contain if the sole is also kept level. Contact itself settles the higher foot onto the floor within a second. The 3 mm left-right tolerance is met under load and not met in the unloaded pose.

## Sole geometry

The box is still the mesh fit from v0.3.1. Half extents are about 0.088 × 0.157 × 0.102 m on the right and 0.086 × 0.159 × 0.100 m on the left. The bottom face is the minimum projection of the heel, foot, and toe vertices onto the sole normal, so upper-surface vertices thicken the box rather than lifting the sole. 55 of 1808 vertices on each foot lie within 5 mm of that face. The 10th percentile sits 22 mm above it, so the visible underside is curved and the collider is the plane through the lowest patch, not a copy of the whole sole and not a platform added under the mesh. The ankle is 43 mm ahead of the heel reference, inside the sole.

## What is still unresolved

The sway is bounded, not gone. There is no measured center of pressure, so left/right and heel/forefoot load fractions are unavailable. The unloaded feet differ by 12 mm. A later phase that needs a quieter statue should add whole-body inverse dynamics or contact-force control. Replacing the joint-space servo was not justified: the same servo holds a 30 s stand once the reference is the ankle.
