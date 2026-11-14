# M4.3 experiments (v0.4.3)

One Humanoidguy, the standing floor, fixed step 1/120 s. Torque limits and friction are the v0.4.2 values. No foot was pinned.

## Static search

| Side | Feasible | COM to ankle | Ankle | Knee | Hip | Hip root | Torso | Swing clearance |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Right | yes | 1.5 cm | 10.2 N·m | 31.7 | 38.2 | 15.5 | 0.046 rad | 57 cm at the reset |
| Left | no | 1.1 cm | 7.5 N·m | 13.2 | 34.1 | 20.3 | 0.51 rad | 8 cm at the reset |

The left rejection is the hip root, whose limit is 18 N·m.

## Pre-positioned right leg

The pose is written before the first step. The motors then hold it.

| Task rate | Balance feedback | Time above 0.85 m | Notes |
| --- | --- | --- | --- |
| 80 Hz, the shipped task | on, both hips | 0.59 s | Ankle saturated from 0.05 s |
| 80 Hz | on, stance hip only | 0.58 s | Kept. The swing hip is not driven |
| 80 Hz | off | 0.58 s | The reference alone does not hold |
| 6 Hz | off | 0.40 s | Ankle effort about 4 N·m. Reverted |
| 16 Hz | off | 0.54 s | Hip effort climbed to about 60 N·m. Reverted |

At 0.2 s in the shipped run the right foot has contact, the left foot has none, and the center of mass is still above 0.90 m. By 0.59 s the center of mass is under 0.85 m. That is not a one-second stand.

## Not run

Two-foot to one-foot transfer, a 5 mm lift, a 10 mm lift, and a return from a held lift. The pre-positioned pose does not last long enough to make those a different experiment.

## Regression

Quiet standing for 30 s still finishes upright with no stance leg selected. A 55 N·s chest impulse still falls. A grab still moves the chest. No body is made kinematic.
