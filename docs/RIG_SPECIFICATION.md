# Rig specification

## What was imported

`humanoidguy_rigged.fbx` (FBX 7400), loaded with Assimp 5.4.3.

| Quantity | Value |
| --- | --- |
| Limb bones | 28 |
| Visual meshes | 56 rounded capsules, 12,656 vertices |
| Materials | none in the file; the viewport uses a neutral clay color |
| File units | centimeters (`UnitScaleFactor` 1) |
| File up axis | +Z (FBX `UpAxis` 2) |
| After Assimp | Y-up, then scaled by 0.01 to meters |
| Bind-pose mesh bounds | x [-0.583, 0.548], y [0.020, 1.551], z [-0.755, 0.006] meters |

Assimp presents the scene Y-up. The node globals in that result sit 180 degrees of yaw away from the skinned vertices, which is the rig root's 180 degree roll after the axis change. The importer detects this (the yaw-corrected pivots are much closer to the meshes) and rotates bone frames so the joints land on the body. The correction is reported as a warning, not hidden. The whole skeleton is then shifted so the lowest vertex rests at y = 0.02 m.

Every mesh is rigidly skinned: one bone carries the full weight. `HeadF` has no mesh. `Hip_1` has no mesh. Six "muscle" capsules are weighted to existing bones (`Clavicle_1`, `Clavicle_2`, `Stomach`, `Pelvis`, `Hip_2`) and are not extra rigid bodies.

A bone is not a rigid body. Bodies are the rows below. Several bones are welded to one body and follow it rigidly.

## Visual bones and physical bodies

Side index `_1` / `_2` is the name in the file. After import, `_1` is the positive-X side and `_2` is the negative-X side.

| Body | Mass (kg) | Bones welded to it | Segment |
| --- | ---: | --- | --- |
| pelvis | 11.0 | Pelvis | Pelvis to Stomach |
| stomach | 8.0 | Stomach | Stomach to Chest |
| chest | 10.0 | Chest | Chest to Neck |
| neck | 1.4 | Neck | Neck to Head |
| head | 4.5 | Head, HeadF | Head to HeadF, plus 6 cm |
| clavicle_1 / _2 | 1.6 | Clavicle_* | Clavicle to Arm |
| upper_arm_1 / _2 | 2.4 | Arm_* | Arm to Forearm |
| forearm_1 / _2 | 1.5 | Forearm_* | Forearm to Hand |
| hand_1 / _2 | 0.7 | Hand_*, Fingers_* | Hand to Fingers, plus 6 cm |
| hip_1 / _2 | 2.2 | Hip_* | Hip origin to Thigh |
| thigh_1 / _2 | 8.0 | Thigh_* | Thigh to Calf |
| calf_1 / _2 | 3.6 | Calf_* | Calf to Heel |
| foot_1 / _2 | 1.3 | Heel_*, Foot_*, Toes_* | Heel to Toes, plus 5 cm |

Total mass is 77.5 kg. Connected mass ratios stay below 8:1 (chest 10 kg to neck 1.4 kg is the largest). Inertia comes from Jolt's capsule (or sphere, when the segment is shorter than the radius) scaled to the assigned mass. The capsule is centered on the body, along local +Y.

Collision radius is the maximum distance from the bone segment to that body's mesh vertices, clamped to the per-body min/max in the JSON. Directly connected bodies, bodies two joints apart, and any pair whose bind-pose segments overlap have collisions disabled. The floor and props are outside the ragdoll collision group and always collide.

## Why the hips are explicit

In the FBX node hierarchy the root `humanoidguy_RIG` has three children: `Pelvis`, `Hip_1`, and `Hip_2`. The legs are **not** children of the pelvis. A builder that only constrained each bone to its scene-graph parent would leave both legs disconnected. The physical joints `hip_root_1` and `hip_root_2` attach `hip_1` and `hip_2` to `pelvis` even though the nodes are siblings. `hip_1` / `hip_2` then attach the thighs. RIG-002 checks both paths.

`Hip_1`, `Hip_2`, and `Pelvis` share a bind position (the pelvic origin). Each hip body is the short segment from that origin out to the thigh, about 9 cm. The mobile hip joint is `hip_1` / `hip_2` (hip body to thigh), not the small root link.

## Bind positions (meters, after alignment and grounding)

| Bone | Parent in the FBX | Position |
| --- | --- | --- |
| Pelvis | (rig root) | (0.006, 0.698, -0.152) |
| Stomach | Pelvis | (0.016, 0.870, -0.122) |
| Chest | Stomach | (0.015, 1.045, -0.128) |
| Neck | Chest | (0.008, 1.208, -0.190) |
| Head | Neck | (0.012, 1.345, -0.299) |
| HeadF | Neck | same as Head; auxiliary, welded to the head body |
| Clavicle_1 / Arm_1 | Chest / Clavicle_1 | clavicle at the neck joint; arm (0.180, 1.201, -0.221) |
| Forearm_1 / Hand_1 / Fingers_1 | along the arm | forearm (0.387, 1.015, -0.333), hand (0.457, 1.157, -0.588) |
| Clavicle_2 / Arm_2 | Chest / Clavicle_2 | arm (-0.162, 1.199, -0.230) |
| Hip_1 / Hip_2 | (rig root, not Pelvis) | same point as Pelvis |
| Thigh_1 / Thigh_2 | Hip_* | (0.093, 0.691, -0.146) and (-0.081, 0.704, -0.159) |
| Calf_1 / Calf_2 | Thigh_* | (0.296, 0.574, -0.560) and (-0.290, 0.565, -0.562) |
| Heel / Foot / Toes | down the leg | heels near y = 0.18, toes near y = 0.07 to 0.09 |

## Joints

Twenty joints, one connected component, 21 bodies. Translation is locked on every joint. Angles are relative to the bind pose. Natural preset scale is 1.

| Joint | Parent body | Child body | Shape | Limits (degrees) |
| --- | --- | --- | --- | --- |
| spine_stomach | pelvis | stomach | pyramid | twist ±15, swing ±18 |
| spine_chest | stomach | chest | pyramid | twist ±15, swing ±18 |
| spine_neck | chest | neck | pyramid | twist ±20, swing ±22 |
| neck | neck | head | cone | twist ±50, swing 40 |
| clavicle_* | chest | clavicle | pyramid | twist ±12, swing ±18 |
| shoulder_* | clavicle | upper arm | cone | twist ±70, swing 80 |
| elbow_* | upper arm | forearm | pyramid, bend axis closes the elbow | twist ±60, side ±8, flexion 0 to 145 |
| wrist_* | forearm | hand | pyramid | twist ±25, swing ±45 |
| hip_root_* | pelvis | hip | pyramid | twist ±8, swing ±10 |
| hip_* | hip | thigh | pyramid, flexion axis toward the facing direction | twist ±40, abduction ±45, flexion -30 to 110 |
| knee_* | thigh | calf | pyramid, flexion toward the pelvis | twist ±10, side ±8, flexion 0 to 145 |
| ankle_* | calf | foot | pyramid | twist ±20, swing about ±30 to 40 |

Attachment frames are built in world space at the child segment start, with the twist axis along the child bone. Both bodies store that frame in local space, so the relative orientation is identity in the bind pose. Hinge axes for elbows, knees, and hips are oriented so the positive swing closes the joint (elbow toward the shoulder, knee toward the pelvis, hip toward forward-up).

Constraint anchors at the bind pose are coincident within 5 mm (RIG-002). During a passive fall they stay within 2 cm (RIG-003, PHY-002).
