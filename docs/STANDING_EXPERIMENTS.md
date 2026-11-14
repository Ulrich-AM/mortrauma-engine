# Standing experiments (v0.3.4)

Each row is one change, compared with the Humanoidguy Normal stand from v0.3.2. Fixed step 1/120 s. No pelvis fixture. Ankle limit 12 N·m, hip 50 N·m, knee 35 N·m, foot friction 0.7, foot mass 1.3 kg.

## Baseline, after Y-Bot removal and before controller edits

Hypothesis: the Y-Bot visual mismatch was also present on Humanoidguy, and the stand would collapse once that character was the only one loaded.

The foot meshes lie inside the boxes (outside distance ~0). The eight corners of the debug box match Jolt's world AABB. Ankles are locked. The quiet stand lasts 30 s. The mismatch was not reproduced on Humanoidguy. No collider or pose edit.

| Trial | Stood | COM y at the end | Tilt | Notes |
| --- | --- | --- | --- | --- |
| Passive, motors off, 1.5 s | falls | 0.15 m | 1.84 rad | Distinguishable from balance. STAB-010. |
| Motor hold of the placement pose, 5 s, no feedback | falls | 0.15 m | 1.51 rad | The unloaded ankle target fights the floor. |
| Balance on, 30 s | 30.0 s, Standing | 1.032 m | 0.167 rad | Drift 36.6 mm. Ankle 1 twist −12 N·m the whole time. |
| 6 N·s forward / left / right | recovered at 3 s | ~1.03 m | 0.17–0.24 rad | |
| 6 N·s backward | fell ~2.0 s | 0.75 m | 0.47 rad | |
| 55 N·s forward | falls | — | — | Natural fall. STAB-012. |

The pre-contact moment readout was 196 N·m. That number is not a joint torque.

## Kept: moment estimate with no planted foot

Hypothesis: the 196 N·m figure is the pelvis-to-ankle horizontal distance times body weight, counted even when `groundedFeet` is 0. Skipping the moment until a foot is planted removes a false "above 12 N·m" reason and does not move the body.

Result: the first sample is 0 N·m and the reason is "feet are not coplanar" (the unloaded 12.5 mm gap). After contact the moment is 4.7–6.1 N·m and the reason is the capacity line, then the saturation line once a motor is on its cap. The 30 s stand, drift, and pushes match the baseline. Kept. It does not change targets or torques.

## Reverted: follow the measured ankle twist

Hypothesis: ankle 1's twist error of ~2.7° at 40 Hz is an unreachable yaw hold. Writing the measured twist into the ankle reference and the slewed command, while keeping swing corrections, should take twist torque off the 12 N·m cap.

Variable: ankle twist component of the reference and of `commanded_`, grounded feet only, after the 0.12 s settle.

| | Twist torque at 30 s | Twist error | Drift | Stood |
| --- | --- | --- | --- | --- |
| Baseline | −12 N·m | ~2.8° | 36.6 mm | 30 s |
| Twist follow | −12 N·m | ~1.8° | 36.7 mm | 30 s |

The torque did not move. Reverted. No second variant was run. The limit is not a static 2.7° offset that an Euler assign can cancel, and it is not a pitch moment above 12 N·m.

## Not tried

- Raising the 12 N·m ankle cap. The loaded pitch estimate stays near 5 N·m.
- Moving the center of mass to midfoot. That target is the one v0.3.2 left, because it asked for more than 12 N·m.
- Flattening the 12 mm unloaded sole gap with the knees. Contact already reduces it to ~1.6 mm, and the earlier knee search made stands fall.
- Larger invisible soles, extra rigs, recovery steps, gain sweeps.
