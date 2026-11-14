# Pose system

A pose is a set of constraint-space quaternions, one per ragdoll joint id, plus the list of joints it intends to drive. It is not a list of mesh matrices and it is not a world-space snapshot.

`PoseController` slerps from the controller's current targets to the pose over the requested duration. Each step also refuses to rotate a target faster than 180°/s, so a very short duration cannot snap the setpoint. The interpolated quaternion is written onto `MotorController` only. The bodies move on the next physics step, and only as far as the motors, limits, and collisions allow. MOT-007's extension transition moved the target at most 0.83° per step and kept the forearm under 2.6 rad/s.

## Procedural poses

Parameters live in `assets/poses/*.json`. The library rebuilds the quaternions from the live ragdoll, so the files do not bake a quaternion that would go stale if the frame changed.

| Pose | Target |
| --- | --- |
| Neutral | The constraint orientation of the imported pose, for every joint. |
| Arm Raise Right / Left | Shoulder rotated toward world up by `raise_degrees` (default 65), then clamped to 75° so it stays inside the 80° cone. The result is stored as a shoulder-relative quaternion, so it does not stay locked to world up after the torso moves. Side 1 is the character's right (+X while facing −Z). |
| Arm Extension Right / Left | That elbow's target is the identity quaternion: anatomical extension. |
| Leg Flexion Right / Left | That knee's target is a rotation about constraint Z of `flex_degrees` (default 120), clamped to 0–145. |
| Compact | Both elbows, both knees, and both hips curled (`elbow_degrees`, `knee_degrees`, `hip_degrees`, defaults 120 / 125 / 100) in their own joint frames. |
| Torso Resistance | Spine joints held at the imported pelvis-relative orientation. |
| Hold Captured Pose | Not a stored quaternion. Selecting it reads the current constraint orientations. |

"Drive only the pose's joints" leaves every other activation at zero, so an arm pose does not switch the legs on.

## Mixamo reference

`assets/poses/imported/Idle.fbx` is an optional owner-supplied Mixamo clip (a binary FBX animation, not a static bind pose). If it is absent, import logs a single informational line and the procedural poses stay as they are. MOT-011 checks that path with a missing file and confirms no targets are invented.

When the file is present the importer:

1. Loads it with Assimp without the Humanoidguy yaw correction or floor snap.
2. Uses the first key of each animation channel. This clip has an animation stack, so that is frame 0 of the idle, not a played timeline.
3. Rebuilds bone directions in a character frame (right, up, forward) and expresses them in the Humanoidguy frame. Humanoidguy side 1 is +X, which is the character's right when facing −Z, so Mixamo Right maps to `_1` and Mixamo Left to `_2`.
4. Elbow and knee targets are the angle between the two limb bones, written as a pure swing about constraint Z and clamped to the joint limit. They are not Mixamo local rotations. The idle frame measured here was elbow 6.7° and 1.5°, knee 9.0° and 15.7°.
5. Shoulder and hip targets are those bone directions converted into the joint's constraint axes, with the rotation clamped (75° at the shoulder, 110° at the hip).
6. Spine, neck, clavicles, wrists, hip roots, and ankles have no defensible mapping from this clip. They keep the Humanoidguy reference quaternion and are listed in `unmapped`. The importer does not guess them.

The result is saved to `assets/poses/imported/mixamo_idle.json` and appears in the Motor Laboratory as "Mixamo Idle." The Humanoidguy mesh and physical rig are not replaced. Continuous playback of the Mixamo clip is out of scope.
