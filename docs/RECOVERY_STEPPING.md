# Recovery stepping

v0.4.3 does not extend recovery placement. The single-leg reset is not a step. See `docs/SINGLE_LEG_FEASIBILITY.md`.

v0.4.2 does not extend recovery placement either. Unloading is a separate state and is documented in `docs/ACTIVE_FOOT_UNLOADING.md`.

v0.4.1 does not extend recovery placement. The Recovery control now runs only the weight shift onto the selected foot and says so. The fixed hip and knee offsets below are the v0.4.0 controller. They are not what v0.4.1 commands. The current states and the height-error lift are in `docs/FOOT_LIFTING.md`.

## v0.4.0 record

One supervisor, `StepController`, owns a single attempt. States are Idle, WeightTransfer, FootLift, Swing, FootPlacement, Reestablish, and Falling. The balance controller keeps the final motor targets. A joint bias with `replace` set drops the balance correction on the swing hip and swing knee and adds flexion on top of the standing reference. The swing ankle is left to the planted-foot rule, so it stops receiving a balance command once that foot leaves the ground. Nothing teleports a body or switches a foot to kinematic.

The nominal lift is 0.28 rad of extra hip flexion and 0.40 rad of extra knee flexion at a requested 4 cm sole height. A larger pair, 0.50 rad and 0.85 rad, was measured first.

| Command | Sole rise while COM height stayed above 0.9 m | Lowest COM | Outcome |
| --- | --- | --- | --- |
| 0.50 / 0.85 rad | 8 mm | 0.15 m | Fell |
| 0.28 / 0.40 rad (kept) | 5 mm | 0.19 m | Fell |

The foot does not hold a 3–5 cm clearance. The body loses the stand before the sole is clear. The larger command was not kept because it did not lift any further while upright.

A recovery request adds a short reach after the lift: a bit more hip flexion and a lateral hip component, toward a target one step-distance from the stance foot. With a 55 N·s chest impulse the character falls, which is the required failure. A completed step that plants a new support was not observed. The attempt returns to Idle and clears the bias, or enters Falling and clears it, and the balance fall is left alone.

The Stepping Laboratory is a collapsed section. It does not run unless a button is pressed. Quiet standing is unchanged by its presence.
