# Balance tuning

Presets live in `assets/configurations/balance/`. They scale the controller. They do not change the Active profile's torque limits, the joint limits, or the floor friction (0.9 in `assets/environments/standing.json`).

v0.3.1 did not change the Normal gains or the 12 N·m ankle limit. A trial at 28 N·m still fell during the second second (center of mass down to 0.60 m). A trial at 48 N·m fell faster, because the ankle spring reaches that cap over a fraction of a degree and shoves the sole. Those limits were reverted. Foot friction stays 0.7.

| Field | Unit | Role |
| --- | --- | --- |
| `balance_strength` | scale, 0–1.5 | Multiplies torque and, through activation, the servo frequency |
| `ankle_correction`, `hip_correction`, `torso_upright` | scale | Multiply the corresponding gains |
| `com_velocity_damping` | scale | Multiplies the velocity term inside the ankle and hip laws |
| `ankle_gain_rad_per_m` | rad/m | Ankle swing per metre of horizontal center-of-mass error |
| `ankle_damping_rad_per_mps` | rad per (m/s) | Ankle swing per metre/second, before the damping scale |
| `hip_gain_rad_per_m` | rad/m | Hip swing per metre of error |
| `hip_damping_rad_per_mps` | rad per (m/s) | Hip swing per metre/second |
| `torso_gain`, `torso_damping` | scale | Upright correction on stomach, chest, and neck |
| `recovery_radius_m` | m | Capture-point distance that, together with 0.55 rad of tilt, enters Falling |
| `fall_tilt_rad` | rad | Torso tilt that enters Falling on its own |
| `falling_activation` | 0–1 | Torque scale after Falling, before Grounded |
| `motor_frequency_hz` | Hz | Servo bandwidth for hips, knees, hip roots, and spine. Clamped to 80 |
| `knee_flexion_deg`, `hip_flexion_deg` | deg | Starting point for the standing-pose search |
| `max_target_rate` | rad/s | How fast a commanded quaternion may move |

## Presets

**Conservative** (`conservative.json`). Strength 0.65, hip gain 0.8 rad/m, frequency 50 Hz, recovery radius 0.10 m. Gives up sooner and pushes less.

**Normal** (`normal.json`). The default. Ankle gain 0.8 rad/m, hip gain 0.45 rad/m, torso gain 0.45, frequency 80 Hz, recovery radius 0.18 m. The low position gains are deliberate. Larger hip gains on this rig pitch the torso over instead of parking the center of mass.

**Aggressive** (`aggressive.json`). Ankle gain 2.8 rad/m, hip gain 1.8 rad/m, frequency 80 Hz, recovery radius 0.28 m. Faster targets, still inside the same torque caps. On capsule feet this usually falls sooner, not later.

## What was measured

Linux, fixed step 1/120 s, Normal preset, no pelvis fixture, one Humanoidguy:

- First second of quiet standing: center of mass stayed above 0.94 m, torso tilt stayed under 0.18 rad, both feet supported, phase stayed Standing or Recovering.
- By about 1.5–2 s the same trial's tilt passed 0.7 rad, Falling reduced the motors, and the body reached the floor (center of mass near 0.13 m).
- A 6 N·s forward impulse at the chest, applied while the character was still up, ended Grounded. Travel was about 1.2 m. The body was not held in place.
- The same impulse to the side produced about 20 N·m at an ankle and 35 N·m at a hip, then the body fell.
- A 55 N·s impulse also fell. That is the expected failure, not a recovery case.

Raising frequency from 3.5 Hz to 80 Hz is what made the knees and hips hold the pose. It does not raise the 50 N·m hip cap or the 12 N·m ankle cap. Raising those caps is a different change and is not part of these presets.

Floor friction was left at 0.9. Increasing it to stop the slide would hide the contact limitation.
