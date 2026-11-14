# Balance system

v0.3.0 adds a balance estimate and a standing controller for the one Humanoidguy ragdoll. The controller submits a motor task. `MotorController::apply` is still the only function that writes Jolt motor targets. Balance does not set body transforms, pin the pelvis, freeze the feet, or add a hidden force.

## Estimate

`BalanceEstimator` runs every simulation step.

Center of mass and center-of-mass velocity are mass-weighted over every dynamic body:

```text
c  = Σ m_i p_i / Σ m_i
v  = Σ m_i v_i / Σ m_i
```

`p_i` is the body's center-of-mass position from the physics world, not the pelvis alone.

Foot support comes from the contact manifolds. A contact counts as support only when the ground reaction on the foot points up (`reaction.y > 0.55`). Jolt's manifold normal pushes body 2 out of the contact, so the foot's reaction is `-normal` when the foot is body 1 and `+normal` when it is body 2. A foot near the floor with no such contact is not supporting. Load is the number of supporting contacts, not a measured force. Jolt does not expose a per-contact force in the listener this build uses.

The support region is the convex hull of those contact points on the ground plane. One point is a point. Two points are a segment. The character's feet are capsules, so the hull is usually a short segment between two contact patches, not a shoe-sized polygon. Margin is the signed distance from the projected center of mass to that hull. Capture distance is how far the point `c + v * sqrt(h/g)` sits outside the hull. A projection inside the segment does not mean the body can stop.

Torso and pelvis tilt are the angle between each body's segment axis and world up.

## Standing pose

The imported FBX is a crouch. `placeStandingPose` is a reset action. It solves a pelvis orientation whose segment is upright, keeps the hip bodies near their bind world rotation, abducts the hips by about 18° so the thigh capsules do not intersect, searches ankle flexion until each foot capsule is level, searches stomach and chest flexion until the chest segment is upright, then searches hip flexion until the center of mass is within about 8 mm of the foot origins in z. The lowest point is placed 4 mm above the floor. Nothing in the running step teleports the bodies back to this pose.

Hip-root twist in that solution can exceed the ±8° limit. The pose clamps hip-root Euler angles inside the limits before the bodies are placed. After the bodies are posed, each motor reference is replaced with the quaternion Jolt actually reports in constraint space, so the motors hold the placed pose instead of a nearby quaternion that the constraint will clamp.

## Controller

`BalanceController` owns one task for every joint. Other code does not write those targets while balance is enabled.

Phases:

| Phase | Meaning |
| --- | --- |
| Inactive | Balance is off. The motor task is cleared. |
| Initializing | Waiting for a foot contact and an upright torso. |
| Standing | Supported and near the reference, small capture error. |
| Recovering | Supported, but the capture point or the tilt says the pose is not settled. Ankle and hip corrections are active. |
| Falling | Torso tilt passed the configured limit, the center of mass is below 0.45 m, or the capture point is outside the recovery radius while the torso is already past 0.55 rad. Motors keep a reduced activation (`falling_activation`). Physics is not frozen. |
| Grounded | The torso is down and vertical speed is small. Motors go passive. The character is not stood back up. |

Minimum time in a phase is `min_state_seconds`, so the label does not flicker every step.

Ankle and hip corrections are proportional to the horizontal center-of-mass error and velocity, expressed in the foot facing frame. Positive swing 2 flexes the hip and, with the foot on the ground, pitches the torso further forward, so a forward error commands less swing 2 (extension), clamped to the joint limits with a small margin. The spine does the same for torso lean. Lateral corrections use swing 1. One supervisor writes both. There is no second controller on the same joint.

After the feet have been loaded for about 0.12 s, the ankle reference is replaced with the measured constraint orientation. The unloaded ankle angle is not the angle the floor will hold, and spending the whole 12 N·m ankle budget to chase it leaves nothing for a correction.

## Servo bandwidth and torque

Jolt's frequency spring scales stiffness by the two-body inertia of that joint. The thigh and hip segments are light, so the 3.5 Hz Active profile cannot hold body weight even when the torque limit is 50 N·m: the spring never reaches the limit. Balance tasks set a higher frequency on the hips, knees, hip roots, and spine (the preset's `motor_frequency_hz`, clamped to 80 Hz in `setAngularMotor`). Ankles stay at most 40 Hz. Arms stay at 4 Hz. The torque command is still the Active profile's limit times balance strength. Frequency does not add torque past that cap.

The pelvis support fixture is turned off when balance is enabled, and the balance tests refuse to run with it on.

## What this cannot do

Round capsule feet have no sole. Ankle torque mostly rolls the capsule and then saturates against the contact. Two ankles at 12 N·m cannot reject a growing gravitational moment once the center of mass is more than a few centimetres ahead of the contact segment. There is no step. A quiet stand on the Normal preset holds the center of mass above 0.94 m and the torso under about 0.17 rad for the first second, then the lean runs away and the character falls. A 6 N·s push to the chest is not recovered. See `docs/BALANCE_TESTING.md`.

## v0.3.1 feet

`foot_1` and `foot_2` can request a box collider in the character JSON. The box is fitted in the body frame: local +Y stays the heel-to-toe bone, and the sole normal is the part of world up perpendicular to that bone. The box is rotated onto those axes with `RotatedTranslatedShape`, then `OffsetCenterOfMassShape` keeps the center of mass at the body origin. Edge rounding is 4 mm. The visual mesh is not changed. Other bodies stay capsules or spheres.

The standing pose searches ankle twist, swing 1, and swing 2 so the sole normal points up, then shifts the hips so the center of mass sits toward the heel rather than the middle of the sole. The ankle joint is near the heel. A center of mass over the midfoot needs an ankle moment of roughly half the body weight times that lever, which is more than 12 N·m. Hip abduction in the pose is 14 degrees so the feet stay apart while the ankle can still level the sole inside its ±20 degree twist limit.

Supporting contacts are labeled heel, midfoot, or forefoot by where they sit between the heel and forefoot reference points. The support area is the shoelace area of the contact hull. Fewer than three contacts is reported as not a polygon. Load is still a contact count.

On the Normal preset the level soles and the heel-biased pose hold a quiet stand for about 1.4 s, then the character falls. Raising the ankle limit to 28 N·m or 48 N·m did not produce a 10 s stand, so the Active profile is unchanged at 12 N·m. Details and the failed trials are in `docs/BALANCE_STABILITY_COMPARISON.md`.

## v0.3.2 equilibrium

The 75/25 heel blend sat about 60 mm ahead of the ankles. At equal load that is roughly 23 N·m each, and the ankle cap is 12 N·m. The controller also used the contact-hull centroid as the error point, so a center of mass back toward the heel was commanded further forward.

The pose and the feedback law now use the ankle ground projection (`feasibleSolePoint`, lead 0). The placed miss is about 7 mm. The Normal quiet stand holds for 30 s with the torso under about 0.22 rad and the support margin still positive. Gains, torque limits, and friction are the ones from v0.3.1. The unloaded soles still differ by about 12 mm left to right; the ankle limits do not have a level solution once the knees are used to close that gap, and contact closes it in the first second. See `docs/EQUILIBRIUM_ANALYSIS.md` and `docs/M3_2_EXPERIMENTS.md`.

Center of pressure is not measured. The listener does not receive solved impulses. See `docs/COP_AND_LOAD_ESTIMATION.md`.

## v0.3.4 Humanoidguy only

The v0.3.3 character switch, its rig file, its motor directory, and its arm-drop pose search are gone. The application loads `configurations/humanoidguy.json` and `characters/humanoidguy_rigged.fbx`. An import with no skin bones fails instead of building a rigid stand-in. `hip_abduction_deg` remains and still defaults to 14.

The live balance target is still the ankle projection (`equilibriumReference`, lead 0). The 75/25 heel blend is used only when that flag is off. It is not the Normal pose.

Before the first contact the ankle-moment estimate is 0. It used to treat the horizontal distance from the pelvis to an ankle as a single-foot moment and reported about 196 N·m, which no joint was carrying. Once both feet are down, the estimate is half the weight times the forward arm. On the 30 s trial that is about 5 to 6 N·m, under the 12 N·m cap. The cap was not raised.

While Standing or Recovering, a saturated ankle or hip replaces the "within capacity" line with that fact. The pitch moment can be inside 12 N·m and the twist axis can still sit on the cap. Details and the trial that was reverted are in `docs/STANDING_STABILIZATION.md` and `docs/STANDING_EXPERIMENTS.md`.
