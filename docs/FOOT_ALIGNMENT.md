# Foot alignment (v0.4.0)

The foot boxes in v0.3.4 had about the right bulk, and the mesh vertices sat inside them. Their bottom face did not follow the shoe.

## Cause

`RagdollBuilder` locked one box axis to the foot bone (local +Y, heel toward toe). The sole normal was world up with the component along that bone removed, then rebuilt so the bottom face stayed perpendicular to the bone. The standing pose then rotated the ankles until that face was level.

The visible underside is not perpendicular to the bone. The shoe normals that point downward in the bind pose, and the vertices that share that normal, sit on a different plane:

| Foot | Angle of the old face off the visible sole | Old half extents (m) | New half extents (m) |
| --- | --- | --- | --- |
| foot_1 | 33.0° | 0.088 × 0.157 × 0.102 | 0.077 × 0.175 × 0.080 |
| foot_2 | 31.5° | 0.086 × 0.159 × 0.100 | 0.078 × 0.175 × 0.081 |

The old 10th-percentile gap from the mesh to the box bottom was about 22 mm, so the leveled box met the ground on a corner while the shoe was pitched. That is the heel-stand and the float. A whole-mesh principal axis was tried first and was still about 16° off the underside that actually faces the floor. It was not kept.

## Correction

The sole normal is the negated average of the foot-mesh normals that point downward relative to the bind-pose up. Vertices whose normals agree with that underside (dot above 0.75) then set the thin axis. Heel-to-toe remains the foot bone projected onto that plane, so the ankle pivot and the heel reference keep their sense. The half extents are the bounds of the same vertices on the new axes. Mass stays 1.3 kg. The mesh file is not moved. Nothing resizes the foot to a larger pad: each half extent matches the mesh span on that axis to well under a millimetre.

After `placeStandingPose`, the angle between the physical sole and the plane of the lowest eighth of the visible foot is 1.22° on the right and 0.46° on the left. Both are inside the 2° target. Ankle separation stays numerical noise. Heel and toe lie on the visible ends of the mesh. Debug box corners still match `Body::GetWorldSpaceBounds`.

The sole frame in the viewport draws the physical sole in cyan, the visible underside in magenta, the body axes in red/green/blue, the collider sole in orange, and world up at the heel in green.

## Standing after the correction

Normal preset, 1/120 s, no support fixture, no impulse.

| | v0.3.4 | v0.4.0 |
| --- | --- | --- |
| Stood | 30 s, Standing | 30 s, Standing |
| COM height at 30 s | 1.032 m | 1.028 m |
| Torso tilt at 30 s | 0.167 rad | 0.093 rad |
| Horizontal drift | 37 mm | 27 mm |
| Loaded coplanarity | 1.7 mm | 1.7 mm |
| Pitch moment | 4.7 N·m | 5.3 N·m |
| Peak foot slip | 0.22 m/s | 0.15 m/s |

The ankle limit is still 12 N·m and it still saturates. Friction was not raised. A 6 N·s forward push and a push to the character's right are recovered. Backward still falls, and a push to the character's left now falls as well. The left ankle uses the whole −20° twist limit just to level the sole, so that side has no twist left. Motor hold without balance still falls. A 55 N·s push still falls.
