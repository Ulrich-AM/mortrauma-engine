# MORTRAUMA v0.3.1

Foot contact for the existing standing controller. One Humanoidguy. No stepping, no extra characters, no hidden support.

## What changed

- `foot_1` and `foot_2` use a box collider fitted to the visible heel, foot, and toe meshes. The center of mass stays on the body origin. The visual mesh is unchanged.
- The standing pose levels the sole normal, not only the foot bone, and places the center of mass toward the heel.
- Supporting contacts are classified as heel, midfoot, or forefoot. The support area comes from the contact hull.
- The Balance Laboratory has a foot-contact readout and overlays for the sole frame, heel, forefoot, and foot collider, plus a capsule comparison that resets the pose.
- Tests FOOT-001 through FOOT-010 and BAL-015 through BAL-020.

The Active torque limits, joint limits, and Normal balance gains are the v0.3.0 values. Ankle limits of 28 N·m and 48 N·m were tried and did not produce a stable stand, so they were not kept.

## What was measured

On the Normal preset the quiet stand lasts about 1.4 s. The first second still passes BAL-001 (center of mass minimum 1.012 m, tilt maximum 0.247 rad). The 10 s target and the 30 s stretch target were not reached. At 30 s the character is on the ground and the state is finite. A flat sole gives heel and forefoot contact and a support area of about 0.31 m². The 12 N·m ankles cannot hold a center of pressure near the middle of that sole.

## Package

`MORTRAUMA-v0.3.1-Windows-x64.zip` contains `MORTRAUMA.exe`, the Humanoidguy asset, configurations, and the runtime note. Releases v0.1.0 through v0.3.0 are not replaced.
