# M4 experiments (v0.4.0)

Fixed step 1/120 s, Normal preset, Humanoidguy, no pelvis fixture. Ankle torque 12 N·m, hip 50 N·m, knee 35 N·m, foot friction 0.7, foot mass 1.3 kg.

## Sole from the whole-mesh thin axis

Hypothesis: the smallest principal axis of the foot vertices is the visible sole, so leveling it levels the shoe.

The axis was 45° and 39° off the old bone-perpendicular face. After the standing pose leveled it, the lowest eighth of the mesh was still 16° off the box. The 10th-percentile gap to the box bottom stayed about 25 mm. Reverted. The volume axis is not the underside.

## Sole from the underside normals

Hypothesis: normals that point downward in the bind pose describe the shoe surface the eye sees. Fitting the box to those vertices, without scaling past the mesh bounds, removes the pitch between the shoe and the collider.

Result: 33.0° and 31.5° off the old face. After placement the visible underside and the box agree within 1.22° and 0.46°. The box half extents equal the mesh span on those axes. The 30 s quiet stand still finishes Standing, with less tilt (0.093 rad) and less drift (27 mm) than v0.3.4. Kept. A 6 N·s push to the character's left, which v0.3.4 recovered, now falls. The left ankle is already on its −20° twist limit in the standing pose.

## Weight transfer

Hypothesis: moving the tracked point 70% of the way from the ankle midpoint to one foot shifts the center of mass onto that foot.

Left: the center of mass moved 48 mm toward that foot and the stand held. Right: it moved 23 mm away and the stand held. The mechanism is not reliable in both directions. No gain change.

## Foot lift

Hypothesis: extra hip and knee flexion on the unloaded leg raises the sole 4 cm.

Neither 0.50/0.85 rad nor 0.28/0.40 rad raised the sole more than 8 mm while the center of mass stayed above 0.9 m. Both attempts ended in a fall. The smaller pair is the one in the controller. Feet were not pinned and the motion was continuous (largest single-step foot move in the early window was 1.4 mm).

## Recovery step and a hard push

A 55 N·s impulse during a recovery request still drops the body. The step does not catch it. That is the expected failure, not a recovered step.

## Not tried

- A larger invisible sole, higher friction, or a higher ankle torque to buy the lift back.
- A second character, a gait cycle, or a walking animation.
- Repeating the right-foot transfer with a different share. One direction already failed, and the lift falls before a step can plant.
