# Standing stabilization (v0.3.4)

Humanoidguy is the only character. The v0.3.3 Y-Bot catalog, rig, motor profiles, balance preset, tests, and the rigid-mesh importer are removed. The motor controller, the balance controller, the foot boxes, and the ankle equilibrium target from v0.3.2 stay.

This is not a new balance law. A quiet stand on the Normal preset still lasts 30 s. The ankle torque limit stays 12 N·m. There is no step and no pelvis fixture.

## What was checked before any gain change

Linux Release, fixed step 1/120 s, `humanoidguy_rigged.fbx`, Normal preset, one floor, no support fixture.

Foot boxes against the skinned vertices and against `Body::GetWorldSpaceBounds`:

| Foot | Mesh samples outside the box | Debug AABB vs Jolt bounds | Half extents (m) |
| --- | --- | --- | --- |
| foot_1 | ~0 | ~0 | 0.088 × 0.157 × 0.102 |
| foot_2 | ~0 | ~0 | 0.086 × 0.159 × 0.100 |

The drawn box is the body pose times `shapeCenter` and `shapeRotation`. That is the shape Jolt simulates. The visible feet are not outside the colliders, and the colliders are the mesh bounds, not a larger platform.

Unloaded soles after `placeStandingPose`:

| Foot | Sole up · Y | Heel height | Toe height |
| --- | --- | --- | --- |
| foot_1 (character right) | 0.99998 | 6.0 mm | 4.2 mm |
| foot_2 | 0.99997 | 16.7 mm | 18.5 mm |

Left-to-right coplanarity is 12.5 mm before contact. Heel-to-toe on each foot is under 2 mm. Ankle separation is numerical noise, and the ankle anchor is the start of the foot segment. The center of mass matches the mass-weighted body positions (77.5 kg) with no residual. The placed horizontal miss to the ankle reference is 6.9 mm.

After the feet are loaded, coplanarity is about 1.6 mm. The pose solver was not changed. v0.3.2 already found that forcing the knees to close the unloaded gap made the stand worse. Contact closes it.

## Equilibrium

The 75/25 heel blend is still in `supportOf`, and it runs only when `equilibriumReference` is false. The Normal pose leaves that flag on, so the target is the ankle projection with lead 0.

Before the first contact the moment estimate used to be about 196 N·m. That was the full horizontal distance from the center of mass to an ankle, treated as if one foot carried the body, while neither foot was down. It is now 0 until a foot is planted.

During the 30 s stand both feet are down and the estimate is about 5 to 6 N·m. That is under 12 N·m, so the cap was not raised. Hips sit near 21 N·m on one swing axis, under the 50 N·m profile. Knees are not the saturated joints in the samples.

A pose that only holds with the motors off is not an equilibrium. With motors passive the center of mass is at 0.15 m and the torso tilt is 1.84 rad after 1.5 s. Holding the placement targets with the motors on, and with balance feedback off, also falls inside 5 s (center of mass 0.15 m). The same initial pose with balance on is still Standing at 30 s. The feedback, including the ankle settle after 0.12 s, is what keeps the body up. The unloaded ankle angles pry against the floor if they are held as the motor target.

## Quiet stand (Normal, no impulse)

| Time | COM height | Torso tilt | Coplanarity | Pitch moment | Phase |
| --- | --- | --- | --- | --- | --- |
| 1 s | 1.038 m | 0.100 rad | 2.0 mm | 5.9 N·m | Standing |
| 8 s | 1.033 m | 0.164 rad | 1.6 mm | 5.2 N·m | Standing |
| 30 s | 1.032 m | 0.167 rad | 1.7 mm | 4.7 N·m | Standing |

Horizontal center-of-mass drift over 30 s is 37 mm. Peak foot horizontal speed during the trial is 0.22 m/s. Each foot keeps heel and forefoot contacts (2 and 2 in the samples). No support fixture. Center of pressure is not measured: Jolt 5.3's contact listener does not provide the solved impulse.

Ankle 1 torque at the sampled seconds is `(-12, ~1.5, ~2)` N·m. The twist axis is on the cap and the twist error stays about 2.7°. Ankle 2 is usually under the cap; at 2 s its swing2 axis hits -12 N·m. Hip effort peaks near 38 N·m in the first second and is about 21 N·m later. Saturation in the first second is 115 of 120 steps, almost all of it ankle 1's twist axis. The pitch moment does not explain that twist torque.

## Disturbances

Chest impulses after 0.25 s of standing, then 3 s of balance. 6 N·s is the vector magnitude. Forward is the character's facing (−Z).

| Impulse | Result at 3 s |
| --- | --- |
| 6 N·s forward | Still Standing, tilt 0.24 rad |
| 6 N·s backward | Fell at about 2.0 s |
| 6 N·s to the character's left | Still Standing, tilt 0.21 rad |
| 6 N·s to the character's right | Still Standing, tilt 0.17 rad |
| 55 N·s forward | Falls. STAB-012 requires this. |

## Controller change that was not kept

Hypothesis: ankle 1 is saturating because the settled twist target is a few degrees off the yaw the floor will allow. Matching the measured twist each step, and leaving swing1/swing2 as the balance command, should drop that torque.

Result: twist torque stayed at −12 N·m. The reported twist error fell from about 2.7° to about 1.8°, drift stayed 37 mm, and the 30 s stand was unchanged. The Euler rewrite does not cancel the constraint-space twist the motor integrates, and a one-step lag still produces a stiff twist torque at 40 Hz. The change was removed.

Gains, frequency, friction, and torque limits are the v0.3.2 values.

## What is still limited

The quiet stand is real: the torso stays near 0.17 rad and the center of mass stays near 1.03 m for 30 s, with no fixture. It is not a claim that every small push is recoverable. Backward 6 N·s falls. Ankle twist stays saturated for the whole quiet trial, so the ankle has little spare twist authority. Foot slip still appears. Contact force and center of pressure are not available from this Jolt pin. There is no recovery step.
