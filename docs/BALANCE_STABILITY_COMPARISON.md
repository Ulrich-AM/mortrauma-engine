# Balance stability comparison

All runs are Linux Release, fixed step 1/120 s, one Humanoidguy, Normal preset unless noted, standing floor, no pelvis fixture, no extra impulse except experiment E. The v0.3.0 capsule numbers are the recorded baseline. The flat-sole numbers are this build. Controller gains and the Active torque limits are the v0.3.0 values except where a trial says otherwise.

The 10 second target and the 30 second stretch target were not met.

## Experiment A — capsule feet, original controller

v0.3.0 quiet stand:

```text
t = 1.0 s   com height min 0.947 m   tilt max 0.170 rad   feet down 119/120   phase Recovering
t = 4.0 s   com height min 0.127 m   tilt max 1.84 rad    phase Grounded
```

A 6 N·s forward push fell (minimum height 0.134 m, tilt 1.77 rad). A 6 N·s lateral push produced on the order of 20 N·m of ankle effort and 35 N·m of hip effort, then fell. Contact on a capsule is a point or a short patch, not a sole.

## Experiment B and C — flat sole and sole-leveled pose

The box and the new standing placement were not shipped as separate binaries, so they are not an independent A/B pair. The intermediate shapes that were tried are listed below and were not kept.

Quiet stand with the flat sole, sole-leveled pose, and unchanged Normal controller:

```text
t = 1.0 s   com height min 1.012 m   tilt max 0.247 rad   feet down 119/120   phase Recovering
t = 2.0 s   com height min 0.185 m   tilt max 2.12 rad    phase Grounded
quiet duration before height < 0.75 m or tilt > 0.7 rad or Falling/Grounded: 1.39 s
```

At about 0.3 s the support hull had area 0.309 m², margin +0.149 m, and the projected center of mass was 0.062 m from the hull center. Contacts were on both the heel and the forefoot (2 and 2 per foot). Ankle effort during the first 1.5 s reached 20.8 N·m as a vector length. No single axis exceeded its torque limit (`torque over` 0). Hip effort reached 34.3 N·m, under the 50 N·m hip limit.

Compared with the capsule baseline, the first second is still upright and the feet are on a real sole. The hold is longer by a fraction of a second (about 1.4 s versus about 1 s). It is not a 10 s stand. Tilt during the first second is larger than the capsule run (0.247 rad versus 0.170 rad).

## Failed geometry, not used

| Attempt | First second |
| --- | --- |
| Axis-aligned mesh box, sole chosen as the lowest bind-pose face | com height 0.720 m, tilt 1.85 rad, Falling |
| Thin principal axis used as the sole, ignoring the ankle hinge | sole up component about 0.22, character fell in the first second |

Feet placed on top of each other (hip abduction near zero) also fell, because the 0.17 m wide boxes overlapped. At 14 degrees of hip abduction the foot origins are about half a metre apart and the ankle can still level the sole. That is the pose in this build. The previous pose used 18 degrees.

## Experiment D — controller retune

Not adopted.

| Change | Result |
| --- | --- |
| Ankle limit 28 N·m, same gains | t = 2 s com height min 0.598 m, tilt 1.45 rad, Falling |
| Ankle limit 48 N·m | Fell inside the first second. The ankle spring hits the cap over a fraction of a degree and the sole shoves the body. |
| Ankle frequency capped at 8 Hz | Collapsed in the first second. The two-body inertia is too small for 8 Hz to hold the weight. |
| Balance corrections set to zero, 80 Hz hold of the loaded pose | Center of mass still accelerated forward and the stand ended. |

The Normal gains and the 12 N·m ankle limit were put back. The limit is the reason a flat sole does not yet hold a midfoot center of pressure: half of 77.5 kg on a lever of about 0.12 m from the ankle to the middle of the sole is on the order of 45 N·m per ankle. The pose therefore keeps the center of mass toward the heel, where the required moment is smaller. That is still not enough for a long stand once the body starts to lean, because 12 N·m only balances a center of pressure a few centimetres from the ankle.

## Experiment E — small pushes

3 N·s at the chest, after 0.25 s of standing, then 0.75 s of observation. Torque stayed inside the configured limits.

| Push | Motion along the push | Minimum com height | Maximum tilt | Phase at the end of the window |
| --- | ---: | ---: | ---: | --- |
| Forward | +0.173 m | 1.007 m | 0.253 rad | Recovering |
| Backward | response mixed with the existing forward drift (reported along −0.118 m) | 1.018 m | 0.243 rad | Recovering |
| To the right | −0.026 m | 1.012 m | 0.230 rad | Recovering |
| To the left | +0.040 m | 1.011 m | 0.272 rad | Recovering |

These windows are shorter than the 1.39 s quiet stand, so they do not show a recovery of a stable stance. They show that a small push does not instantly throw the body down, and that the motors do not exceed their limits. The 6 N·s and 55 N·s cases from v0.3.0 still fall. BAL-003 and BAL-004 were not weakened.

## Experiment F — 30 seconds

The run continues after the fall. At 30 s the state is finite, phase Grounded, center of mass height 0.180 m, torso tilt 1.91 rad. No non-finite state and no departure from the scene. The character does not get back up.
