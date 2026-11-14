# Y-Bot rig comparison (v0.3.3)

The question is whether the supplied figure is a better physical base for the existing balance controller than Humanoidguy. It is not a request to make Y-Bot stand longer, and the numbers below are not that claim.

Both trials use the same fixed step, 1/120 s, and the same gravity. The controller code is the same. The gains in `assets/configurations/balance/ybot.json` match Normal. What differs is the body, the pose, and the torque file.

## What was imported

`Y_Bot_Base.fbx` is not a skinned Mixamo Y-Bot. Assimp reports no skin bones and no animation. It is a rigid segmented figure in centimeters, Y up, with Spanish node names. Visible parts live under `Formas`. Joint-marker spheres live under `Esferas` and are used as pivots only. Their meshes are not drawn and are not colliders.

The importer's skinned path is unchanged. A scene with no skin bones builds one bone per visible part, stores finger meshes on the palm, and keeps the marker centers as extra bones. The file's toes point +Z. The rigid importer yaws the whole character 180° about Y so the toes point -Z, which is the laboratory forward Humanoidguy already uses. Vertex positions stay in the bone frame.

After import: 35 bones, 48 meshes, 22967 vertices, unit scale 0.01.

The file is listed in `.gitignore`. Adobe's Mixamo terms, and the fact that this file is not that skinned character, both argue against publishing it. Copy the supplied FBX to `assets/characters/ybot.fbx`. The Windows package does not contain it. The Character window names the path when the file is missing.

## Bodies and joints

Y-Bot has 17 bodies and 16 joints. Humanoidguy has 21 and 20. There is no clavicle body and no hip body. Each thigh is hinged on the pelvis. The graph is one component, and both feet are reachable from the pelvis. `legsAttached` still requires the hip bodies when a rig has them, so Humanoidguy's check is the same.

| Body | Bones | Mass (kg) |
| --- | --- | --- |
| Pelvis | Cadera, pivot Cadera_2 | 11.0 |
| Abdomen | Cintura | 8.0 |
| Chest | Plexo, pivot Plexo_2 | 11.0 |
| Neck | Cuello marker | 1.2 |
| Head | Cabeza | 4.2 |
| Upper arm, forearm, hand, each side | Hombro / Codo / wrist marker to the palm | 2.2, 1.4, 0.55 |
| Thigh, calf, foot, each side | hip sphere, knee, ankle, toe marker | 8.0, 3.5, 1.15 |

Total mass is 69.0 kg. Humanoidguy remains 77.5 kg. Side 1 is the character's right when facing -Z (`Der`, the `_2` meshes). Side 2 is the left (`Izq`, `Muñeca`).

Joint limits follow the same anatomical pattern as Humanoidguy: knee and elbow flexion about swing 2, hip flexion about swing 2 with a shared forward sign. Elbow swing 2 starts at -10° because the authored arm is about 6° past the "bone continues the parent" zero. Shoulder cones are 120° so the T-pose can drop. Knees stay 0° to 145°. Ankles are -35° to 40° on swing 2.

## Feet

The sole is not taken from the foot-bone axis. The box fitter uses world up, with the bone direction removed, as the sole normal, and fits the box to `Pie` and `Dedos`. The ankle-to-toe marker is only the long axis. After the 180° yaw that axis points toward -Z.

| | Humanoidguy foot | Y-Bot foot |
| --- | --- | --- |
| Box width, length, thickness | 0.175 m, 0.314 m, 0.204 m | 0.130 m, 0.291 m, 0.146 m |
| Segment length | 0.225 m | 0.186 m |
| Mass | 1.3 kg | 1.15 kg |
| Friction | 0.7 | 0.7 |

Y-Bot's underside is rounded. 17 of 1222 foot vertices lie within 5 mm of the lowest point, and the 10th percentile is 36 mm above it. The collider is still the mesh bounds, so the contact face is a flat rectangle at that lowest point, not a thin plate and not a larger platform than the mesh. The standing pose then levels that face. Measured after placement: sole normals about 0.999 up, heel-to-toe height difference under 5 mm, left/right sole difference 5.2 mm.

## Standing pose

The T-pose is not the stand. For Y-Bot only, `relax_arms` searches shoulder and elbow angles until the hands sit beside the hips. Measured hand height was 1.06 m against a shoulder height of 1.37 m. Hip abduction in the pose is 8° because the hips are already about 17 cm apart. Humanoidguy's preset does not set that field, so it stays at 14°. Knee flexion stays the Normal value of 12°. The center of mass is matched to the ankle projection with the existing equilibrium reference. Nothing is kinematic during the stand. There is no pelvis fixture.

Placement reported a center of mass 25 mm from the ankle target and a torso tilt of 0.022 rad.

## Torques

Active torque is Humanoidguy's table times 69.0/77.5 = 0.890. Measured lengths: thigh 0.411 m versus 0.475 m, calf 0.422 m versus 0.400 m, upper arm 0.274 m versus 0.300 m. A second length factor was not applied. Inertia is whatever Jolt assigns to the fitted capsule or box. The result is ankle 11 N·m, hip 45 N·m, knee 31 N·m, against Humanoidguy's 12, 50, and 35. Servo frequency stays 3.5 Hz in the profile and 80 Hz in the balance task, same as Normal. The files are `assets/configurations/motors/ybot/`. Humanoidguy still loads `assets/configurations/motors/active.json`.

The diagnostic string that mentions a 12 N·m pitch moment is still the Humanoidguy threshold. It does not change the command. Contact impulses are still unavailable from this Jolt pin, so center of pressure is not measured.

## Trials

Each character used its own files. "Quiet" and "long" are the same undisturbed stand. The forward and lateral pushes are 2 N·s on the chest, along the character's facing and to its right. A fall is a Falling phase or a center of mass below 0.55 m. Slip is the largest horizontal foot speed after the first 0.5 s. Ankle and hip figures are the peak length of the motor-torque vector, so they can exceed one axis limit. Saturation is the number of steps on which any motor reported the limit.

| Trial | Duration | Stood | Max torso tilt | Peak ankle | Peak hip | Saturated steps | Foot slip | Separation | Support area | Contacts |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Humanoidguy quiet 8 s | 8.01 s | yes | 0.264 rad | 16.6 N·m | 38.0 N·m | 955 | 0.080 m/s | 2.4 mm | 0.226 m² | 4 and 4, heel and forefoot |
| Humanoidguy forward 2 N·s | 6.01 s | yes | 0.272 rad | 17.1 N·m | 38.9 N·m | 715 | 0.063 m/s | 2.3 mm | 0.225 m² | same pattern |
| Humanoidguy lateral 2 N·s | 6.01 s | yes | 0.262 rad | 17.3 N·m | 38.8 N·m | 715 | 0.044 m/s | 2.6 mm | 0.225 m² | same pattern |
| Humanoidguy quiet 15 s | 15.01 s | yes | 0.264 rad | 16.6 N·m | 38.0 N·m | 1795 | 0.080 m/s | 2.4 mm | 0.226 m² | same pattern |
| Y-Bot quiet 8 s | 8.01 s | yes | 0.038 rad | 14.3 N·m | 43.5 N·m | 36 | 0.031 m/s | 3.4 mm | 0.087 m² | 4 and 4, heel and forefoot |
| Y-Bot forward 2 N·s | 6.01 s | yes | 0.035 rad | 13.1 N·m | 49.9 N·m | 16 | 0.036 m/s | 3.3 mm | 0.087 m² | same pattern |
| Y-Bot lateral 2 N·s | 6.01 s | yes | 0.038 rad | 14.5 N·m | 48.4 N·m | 27 | 0.026 m/s | 3.3 mm | 0.087 m² | same pattern |
| Y-Bot quiet 15 s | 15.01 s | yes | 0.038 rad | 14.3 N·m | 43.5 N·m | 36 | 0.031 m/s | 3.4 mm | 0.087 m² | same pattern |

Initial center of mass, before the first step: Humanoidguy (0.010, 1.088, -0.176) m, Y-Bot (0.006, 1.045, -0.003) m. The Z difference is where the bind pose sits in the room, not a stability margin. Both contact patterns are the four corners of a box sole, two at the heel and two at the forefoot, and none on a midfoot. That is the box, not a pressure map.

Duration does not separate them at 15 s. Humanoidguy's ankles are at the limit on almost every step (955 of about 960). Y-Bot's ankles are not, but its hip torque peaks at 43.5 N·m against a 45 N·m limit on the quiet stand and at 49.9 N·m on the forward push, so the hip budget is the one being used up. Y-Bot's support polygon is smaller because the feet are narrower and the stance is less abducted. The torso tilt is smaller. Those are consequences of a lighter body, a different foot box, lowered arms, and a lower torque table, running the same controller. They are not evidence that the skeleton is inherently more suitable.

## Switching

The Character window lists Humanoidguy and Y-Bot. A change pauses the simulation, ends grabs and the laboratory support fixture, destroys the ragdoll joints and bodies, loads the selected profile, binds motors and the balance preset, and clears the selection. The floor is not destroyed. YBOT-009 does that sequence and then builds Humanoidguy again. Only one character is created. There is no second spawn and no character-to-character contact.

## Limits of the comparison

- The model is rigid segments plus spheres, so there is nothing to learn here about Mixamo skinning.
- Fingers are glued to the palm. There is no finger motor.
- The neck has a marker body and no neck mesh.
- The laboratory still draws flat colors, not the file's materials.
- The 12 N·m sentence in the balance diagnostic was left as Humanoidguy's.
- A 15 s stand that both complete cannot say who falls first.
