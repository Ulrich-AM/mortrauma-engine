# M4.2 experiments (v0.4.2)

One Humanoidguy, the standing floor, fixed step 1/120 s. Torque limits, friction, and the sole boxes are the v0.4.1 values. The weight-shift target was not moved.

## A. Quiet standing, 30 s

No step command. Phase Standing, center of mass height above 0.95 m, torso tilt under 0.35 rad. The v0.4.1 stand at 1.028 m and 0.096 rad still holds. UNLOAD-016 and STEP-015 both pass.

## B and C. Support shift

Same controller as v0.4.1, measured again before the unload change was accepted:

| Foot | Displacement toward the foot | Height |
| --- | --- | --- |
| Left | 53 mm | above 0.90 m |
| Right | 47 mm | above 0.90 m |

The left shift still does not settle inside the request. The right shift still completes. Hip 1 swing1 is positive for a rightward request and negative for a leftward request. Hip 2 is the negation.

## D. Unloading only

The swing knee is commanded up to 0.12 rad after the shift, while the foot still has its contacts. Upright sole rise is about 1 mm. The center of mass stays near 1.01 m. The note is "unloading command did not raise the swing foot". That timeout is an abort, not a success.

The trials that flexed the hip as well, or that allowed 0.20 to 0.35 rad at the knee, are in `docs/ACTIVE_FOOT_UNLOADING.md`. They either did not raise the sole or they lost the stand. They were not kept.

## E and F. 1 cm and 3 cm lifts

Not run as successes. The lift state is not entered, because the sole never rose 4 mm while the stance was stable. A 3 cm target was not attempted on top of a failed 1 cm unload.

## G. Hold and lower

There is no held foot. When unloading stops, the knee command decays, and both feet are required to be touching for 0.2 s before the bias is cleared.

## H. Abort

Abort during the shift leaves the Aborted state and a balance offset below 2 cm. The balance controller still owns the motor task. Bodies stay dynamic.

## I. Disturbance

A 55 N·s chest impulse falls. A grab still moves the chest. No support fixture is created.

## Repeatability

Two identical unloading runs end in the same terminal state.
