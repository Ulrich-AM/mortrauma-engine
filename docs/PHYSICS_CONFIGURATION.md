# Physics configuration

## Integrator

- Fixed timestep: 1/120 s. Displayed on the Simulation panel. Rendering is not the clock.
- Accumulator is capped at 0.25 s. At most 6 fixed steps run each frame. Extra time is discarded rather than producing an unbounded backlog.
- `PhysicsWorld::step` returns immediately if a step is already running.
- Jolt collision substeps: 1 by default, slider 1–8.
- Velocity iterations: 12 (range 1–64). Position iterations: 3 (range 1–16).
- Motion quality is `LinearCast` so fast limbs are less likely to tunnel.
- The job system is single-threaded. That keeps a replay bit-identical (PHY-005 measured 0 deviation).
- Sleeping is enabled. Settled bodies report zero velocity.

## Defaults

| Property | Value |
| --- | --- |
| Gravity | (0, -9.81, 0) m/s². Sliders cover a useful range and can be reset. |
| Body friction | 0.7 (floor 0.85, cube 0.55) |
| Restitution | 0.02 (cube 0.05) |
| Linear / angular damping | Natural preset 0.08 / 0.18. Jolt expects these coefficients in [0, 1]. |
| Angular friction | Natural preset 2 N·m on each rotational axis of each SixDOF joint |
| Simulation speed | 1, range 0.1–2. This scales the accumulator, not the timestep size. |

There is no additional force pulling the character upright. Every ragdoll body is dynamic. The pelvis is not anchored to the world.

## Presets

Files in `assets/configurations/presets/`.

| Preset | Linear damping | Angular damping | Angular friction (N·m) | Limit scale |
| --- | ---: | ---: | ---: | ---: |
| Loose | 0.02 | 0.05 | 0.4 | 1.15 |
| Natural | 0.08 | 0.18 | 2.0 | 1.0 |
| Firm | 0.15 | 0.35 | 8.0 | 0.8 |

Limit scale multiplies the anatomical angles. A scale above 1 only widens limits; the bind pose remains inside them because it is the zero angle. Saving a preset rejects negative damping, friction above the slider range, or a non-positive limit scale.

## Grab and push

Grab creates a kinematic anchor and a SixDOF constraint. Translation limits are a ±2 mm window with a frequency/damping spring (Jolt treats a zero-width limit as a hard lock, and it only springs translation). Rotation is free. Releasing the mouse removes the constraint and the anchor. Strength is the spring frequency (1–20 Hz). Damping is the spring damping ratio (0–1.5).

Push calls `AddImpulse` at the ray hit, not at the center of mass, so an off-center click produces rotation. Direction is the camera forward or a world axis. Strength is 1–200 N·s.

## Environment

`assets/environments/lab.json`:

- Floor: 30 m × 30 m static box, top at y = 0.
- Two static walls.
- Dynamic 0.4 m cube, mass 8 kg.
- Optional platform and ramp, created and destroyed from the Physics panel.

## Self-collision

Ragdoll bodies use collision group 1 and a subgroup per body. Pairs closer than two joints, and pairs whose bind segments overlap, are disabled. Bodies in any other group, including the floor, are not filtered. The grab anchor is in a non-colliding layer.

## What is intentionally absent

No pose-matching motor, no root drive, no friction boost that exists only to fake stability, no hidden damping above the preset. If the character jitters, the constraint graph and the iteration counts are the place to look. Diagnostics show separation, limit status, contact count, and the number of disconnected components (expected value: 1).
