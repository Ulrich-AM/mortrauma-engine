# M4.1 experiments (v0.4.1)

One Humanoidguy, the standing floor, fixed step 1/120 s. Torque limits and friction are the v0.4.0 values. The sole geometry was not moved.

## A. Quiet standing, 30 s

No step command. Phase Standing, center of mass height 1.028 m, torso tilt 0.096 rad. The largest horizontal distance from the ankle equilibrium during the trial was 50 mm. v0.4.0 ended a 30 s stand at 1.028 m, tilt 0.093 rad, and 27 mm of drift. The stand still holds. The drift figure is a peak over the trial, not the same end-point measurement.

## B and C. Weight transfer

Target 5.5 cm from the current center of mass toward the selected foot. Starting lateral position −33 mm (character-left of the equilibrium).

| Foot | Peak lateral | Toward the foot | Height | Result |
| --- | --- | --- | --- | --- |
| Left | −86 mm | 53 mm | 1.019 m | Direction is correct. The target is visited and not held for the settle window, so a 4 s request is still in TransferWeight. |
| Right | +14 mm | 47 mm | 1.013 m | Direction is correct. The shift completes and two-foot standing is restored. |

v0.4.0's "48 mm left, 23 mm the wrong way" used the absolute projection and a shared hip sign. Both directions now move toward the requested foot. Neither puts the center of mass over the foot.

## D. Single-leg support

After the shift, the swing foot still reported 4 contacts. The lift was refused. There is no measured single-leg interval. Contact counts are not reported as loads.

## E and F. Foot lift

The shipped sequence does not lift, so the upright sole rise on that path is about 1 mm, which is stance motion, and the controller's lift clearance stays 0. A probe that lifted anyway, after the shift and while the other foot was still touching, reached 11 mm of sole rise with the center of mass above 0.9 m and then fell to 0.85 m. A slower Jacobian on that same probe reached about 3 mm and still fell. The 5 cm lift was not run. The 3 cm target was not achieved.

## G. Lift and return

Not completed. The sequence returns to two-foot contact because the lift is refused or the shift times out, not because a raised foot was set back down.

## H. Abort

Abort during the shift and during a sequence enters Aborted, ramps the reference back, and leaves a balance offset below 2 cm. A second abort does not start a new attempt. Cancel returns to Idle and clears the offset on the next step. The balance controller still owns the motor task.

## I. Disturbance

A 55 N·s chest impulse during a recovery request, which is only a weight shift in this version, drops the center of mass through 0.71 m. The character falls. No support fixture is created.

Grabbing the chest and moving the target 12 cm moved the chest 108 mm. A following impulse still moves the body.

## Motors

During the sequence the excess of motor torque over the configured limit was 0. Joint angles stayed inside the anatomical limits, with the test slack of 0.08 rad. No torque limit was increased.
