# Balance testing

Balance tests are `BAL-001` through `BAL-014` in `mortrauma_tests`. Grab regression is `GRAB-001` through `GRAB-007`. Run one case with `mortrauma_tests BAL-001` or the full program with no arguments.

The standing scene is `assets/environments/standing.json`: an 8 m floor, friction 0.9, no moving props, no pelvis fixture. `placeStandingPose` runs before the trial. The controller then only commands motors.

## What each case checks

| Test | What is required |
| --- | --- |
| BAL-001 | Feet support the body for the first second, center of mass stays above 0.75 m, torso tilt stays under 0.45 rad, phase is not Falling or Grounded, no pelvis fixture. The following three seconds are logged. On the Normal preset the body then falls. That fall is reported and is not a pass of "stands indefinitely". |
| BAL-002 | With the floor removed, the center of mass drops by more than 0.35 m and ankle authority turns off. |
| BAL-003 | A 6 N·s forward impulse moves the body and is not recovered. The character falls. No support fixture. |
| BAL-004 | A 6 N·s lateral impulse produces ankle or hip effort above 0.4 N·m and is not recovered. |
| BAL-005 | A 55 N·s impulse is not rigidly rejected. The pelvis moves and the body falls. |
| BAL-006 | Reported motor torque does not exceed the commanded limit by more than 0.75 N·m. |
| BAL-007 | Disabling balance clears the motor task and restores passive mode. |
| BAL-008 | Moving the floor away clears foot support and ankle authority. |
| BAL-009 | While Initializing, Standing, or Recovering, joint angles stay within 0.10 rad of the configured limits. Ground impacts after Falling are not counted. |
| BAL-010 | Two runs with the same impulse match within 2 cm and 0.03 rad. |
| BAL-011 | Balance, Passive, Active, and Hold exchange motor ownership without leaving a task behind. |
| BAL-012 | Grab, drag, and release during balance stay finite and leave no grab and no fixture. |
| BAL-013 | Entering balance removes the pelvis fixture and does not turn it back on. |
| BAL-014 | The rig is still 21 bodies and 20 joints, both extra presets load, and a passive step stays finite. |

## Measured quiet stand (Normal)

Linux Release, 120 Hz, Humanoidguy, capsule feet, Active torque limits unchanged:

```text
t = 1.0 s   com height min 0.947 m   torso tilt max 0.170 rad   feet down 119/120   phase Recovering
t = 4.0 s   com height min 0.127 m   torso tilt max 1.84 rad    phase Grounded
```

The first second is the BAL-001 pass. The later fall is the limitation in `docs/BALANCE_SYSTEM.md`. Do not tighten or loosen the first-second numbers to hide a change in that hold. If a future change drops the first second below 0.75 m or past 0.45 rad, BAL-001 should fail.

## v0.3.1 foot and duration cases

`FOOT-001` through `FOOT-010` and `BAL-015` through `BAL-020` are in the same executable. The numbers from the flat-sole build are in `docs/FOOT_CONTACT_TESTING.md` and `docs/BALANCE_STABILITY_COMPARISON.md`.

BAL-001 still requires the first second upright. On the flat-sole Normal preset that second measured a minimum center-of-mass height of 1.012 m and a maximum torso tilt of 0.247 rad, with the feet down on 119 of 120 steps. BAL-015 then measured a quiet stand of 1.39 s. The 10 s and 30 s targets were not met. The 30 s run stays finite and ends on the ground.

## v0.3.2 equilibrium cases

`EQ-001` through `EQ-012` cover coplanarity, joint connectivity, foot-only floor contacts, the unavailable impulse flag, the synthetic center of pressure, a 3 s hold, reachability of the ankle target, correction signs, per-axis torque, a 30 s quiet stand, both feet loaded, and the preserved 12 N·m / 1.3 kg / 0.7 friction limits.

On the v0.3.2 Normal pose, BAL-001's first second measures a minimum center-of-mass height of 1.027 m and a maximum tilt of 0.265 rad. EQ-010 measures 30 s. BAL-003 and BAL-004 used to require a fall from 6 N·s, which was true on the heel target. That push is now rejected, and the tests expect the recovery. BAL-005 still requires the 55 N·s push to knock the body down. The comparison table is `docs/M3_2_EXPERIMENTS.md`.

## Grab

GRAB-001 repeats begin and release. GRAB-002 resets during a grab. GRAB-003 grabs with motors active. GRAB-004 grabs with the pelvis fixture. GRAB-005 moves the target quickly. GRAB-006 checks the temporary constraint is gone. GRAB-007 grabs while balance is active. The lock bug they cover is in `docs/GRAB_CRASH_INVESTIGATION.md`.
