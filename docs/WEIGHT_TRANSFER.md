# Weight transfer (v0.4.0)

`StepController` can move the balance reference toward one foot. The offset ramps to 70% of the vector from the ankle equilibrium point to the stance foot, holds, then ramps back. The stance foot is not locked. `BalanceController` is still the only code that builds the motor task.

Normal preset, after a short settle, phase duration 0.7 s. The body stayed up in both directions (center of mass near 1.02 m). The shift is not symmetric:

| Requested support | Peak COM motion toward that foot | Result |
| --- | --- | --- |
| Left (foot_2) | +48 mm | Stayed Standing |
| Right (foot_1) | −23 mm (moved away) | Stayed Standing |

The right-foot request does not unload the left foot. The reference moves toward `foot_1`, and the existing hip correction does not carry the center of mass with it. No second gain pass was run. Torque limits and friction are the v0.3.4 values. There is no pelvis fixture.
