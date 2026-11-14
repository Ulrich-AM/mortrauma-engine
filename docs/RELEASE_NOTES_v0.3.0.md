# MORTRAUMA v0.3.0

Autonomous standing on the existing Humanoidguy ragdoll. One character. No stepping, no getting up, no second rig.

## What changed

- The grab crash is fixed. Creating a constraint locks both bodies with `BodyLockMultiWrite`, which is the call Jolt uses internally. Two separate write locks deadlocked when the kinematic anchor and the grabbed body shared a mutex, and the process aborted with "Resource deadlock avoided". Grab and support anchors no longer sleep. Grab targets that are non-finite or more than 4 m away are rejected. See `docs/GRAB_CRASH_INVESTIGATION.md`.
- A balance estimate computes a mass-weighted center of mass and velocity, foot support from real contacts, a support hull, and a capture distance.
- Reset can place the character in a solved standing pose. The running simulation does not teleport it back.
- A balance controller commands the v0.2.0 motors through one motor task. `MotorController::apply` is still the only writer of motor targets. Torque limits are the Active profile. Hips, knees, and spine use a higher servo frequency because Jolt's spring is scaled by the small two-body inertia and otherwise never reaches those limits.
- States are Inactive, Initializing, Standing, Recovering, Falling, and Grounded. Falling reduces torque. Grounded goes passive. Nothing stands the character back up.
- The Balance Laboratory shows the estimate, overlays, plots, presets, and fixed impulses. The mouse Push tool is unchanged. Balance turns the pelvis fixture off.
- Presets: `assets/configurations/balance/conservative.json`, `normal.json`, `aggressive.json`. Scene: `assets/environments/standing.json`.

## Measured behavior

On the Normal preset the character stands for about one second: center of mass above 0.94 m, torso tilt under 0.18 rad, feet on the floor, no pelvis fixture. It then leans and falls. A 6 N·s push is not recovered. A 55 N·s push falls. Capsule feet do not give the ankles a sole, and the ankle torque limit is 12 N·m, so this is a physical limit of the current rig rather than a hidden constraint failing to hold.

## Tests

GRAB-001..007 and BAL-001..014 are in `mortrauma_tests`. Details and the one-second numbers are in `docs/BALANCE_TESTING.md`.

## Package

`MORTRAUMA-v0.3.0-Windows-x64.zip` contains `MORTRAUMA.exe`, the Humanoidguy asset, motor profiles, balance presets, and the standing scene. Previous releases are not replaced.
