# MORTRAUMA v0.1.1

Patch for anatomical elbow, knee and hip limits. No balancing and no pose-driving motors. The v0.1.0 release is unchanged.

## Fixed

v0.1.0 measured elbow and knee limits from the imported crouch. That pose is already bent 79° at the elbows and 85–89° at the knees, and the limits started at 0°, so the limbs could not straighten. Hip flexion was aimed backward because the hip bones share a position and the old facing fallback pointed away from the toes.

v0.1.1 puts the elbow and knee zero at full extension, in the authored bend plane, with positive swing 2 as flexion on both sides. Limits remain 0° to 145°. Hips measure flexion from a thigh pointing down, toward the toes, and keep −30° to 110°. Shoulders are unchanged.

The Joint inspector shows twist, both swings, the limits, the anatomical offset of the imported pose, both reference frames, which limits are reached, friction and the constraint separation. While paused, those limits and the friction can be edited. The viewport draws the extension axis, the flexion direction, the hinge, and the selected joint's swing-2 range.

## Tests

Linux Release: the v0.1.0 suite plus RIG-004, RIG-005, RIG-006 and LIM-001 passed. Measured angles are in `docs/JOINT_LIMITS.md`. A zero-friction drive reached the 0° extension stop on both elbows and both knees (about −0.2° of solver slop) with pivots under 1 mm apart. The same drive with 2 N·m of friction reached 8.7° at the elbows and about 2° at the knees. Contacts were present and were not what stopped the limb: the joint was on its extension limit.

Windows Release, MSVC 19.51, GitHub Actions run [43541736222](https://github.com/Ulrich-AM/mortrauma-engine/actions/runs/43541736222) on `089e350`: all 13 tests passed and `MORTRAUMA-v0.1.1-Windows-x64.zip` was uploaded. The tag workflow rebuilds the release commit before publishing.

## Known limitations

- The bind pose is still the authored crouch. The character does not stand up by itself.
- Shoulder cones remain centered on the imported arm pose.
- Jolt 5.3 still has no soft angular limits. Compliance is friction and damping.
- Ankle, wrist and spine limits are still centered on the imported pose.
- Interactive use of the joint inspector was not clicked through on a physical desktop.
