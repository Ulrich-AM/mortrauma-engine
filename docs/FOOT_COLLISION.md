# Foot collision

v0.3.0 represented every limb, including the feet, as a capsule or a sphere aligned with the bone. The visible feet already have a flattened underside. A capsule contacts the floor on a point or a short segment, so ankle torque rolls the foot instead of shifting the center of pressure.

## Original collider

The capsule radius is the farthest mesh vertex from the heel-to-toe segment, clamped to the body's `min_radius` and `max_radius`. For both feet those clamps are 0.04 m and 0.08 m. The capsule is centered on the body origin, which is the segment midpoint, and it follows local +Y. Foot mass stays 1.3 kg. Foot friction stays 0.7. Floor friction in the standing scene stays 0.9.

## Revised collider

`foot_1` and `foot_2` set `"collider": "box"` in `assets/configurations/humanoidguy.json`. Other bodies stay on the automatic capsule or sphere.

The fit is done in the existing body frame. Local +Y is still the bone from heel toward toes. The sole normal is the part of world up that is perpendicular to that bone, expressed in the body. The box's local +Z is that normal, so the sole is the local −Z face. Width is the remaining axis. Extents are the mesh vertices owned by `Heel_*`, `Foot_*`, and `Toes_*`, projected onto those axes. The box is not a platform added under the mesh. It is the oriented bounds of those vertices.

Measured half extents (width, length, thickness), meters:

| Foot | Half width | Half length | Half thickness |
| --- | ---: | ---: | ---: |
| foot_1 | 0.0877 | 0.1569 | 0.1020 |
| foot_2 | 0.0859 | 0.1585 | 0.1003 |

Outer size is those half extents. Edges are rounded by a convex radius of 0.004 m, clamped so the rounding stays under a quarter of the smallest half extent. The full sole is about 0.31 m long, 0.17 m wide, and 0.20 m thick. The thickness is the mesh extent along the sole normal, including the upper part of the foot, not an extra slab.

The box is placed with Jolt's `RotatedTranslatedShape` and then wrapped in `OffsetCenterOfMassShape` so the center of mass remains the body origin. Inertia is the box inertia scaled to 1.3 kg. Ankle anchors are still the segment start of the foot body. The joint frames are not rotated to aim the collider.

Heel and forefoot reference points sit on the sole face at the rear and front center of the box, in body space. The sole normal points up when the sole is level.

## What was rejected

An axis-aligned box in the body frame used the lowest bind-pose face as the sole. That face is not the anatomical sole in the crouch, and the box was too thick in the wrong direction. With that shape the first second of BAL-001 failed: center of mass 0.72 m, torso tilt 1.85 rad, phase Falling. A principal-axis fit that ignored the ankle hinge picked a sole the ankle could not point at the floor (sole up component about 0.22). Both were discarded. The bone-locked sole above is the one that the ankle pitch can level.

## Comparison

The Balance Laboratory has a foot-contact section. "Capsule feet for comparison" rebuilds the foot shapes and resets the standing pose. It does not edit the shape during a physics step. FOOT-009 does the same swap in a test. After 40 steps the box had 8 supporting contacts and the restored capsule had 3.
