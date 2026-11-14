# Joint limits, v0.1.1

## What was wrong

v0.1.0 built both constraint frames from the same world axes, so every joint read 0° in the imported pose. The elbow and knee limits were 0° to 145° of swing 2. That range was applied on top of a pose that is already bent, so the 0° stop was the crouch and the limb could only bend further.

The geometric bend is the angle between the parent bone and the child bone. 0° means the bones continue in a straight line.

| Joint | Geometric bend in the imported pose | v0.1.0 swing 2 at that pose | Smallest swing 2 the limit allowed |
| --- | ---: | ---: | ---: |
| elbow_1 | 79.2° | 0° | 0° (still 79° bent) |
| elbow_2 | 78.5° | 0° | 0° (still 79° bent) |
| knee_1 | 89.0° | 0° | 0° (still 89° bent) |
| knee_2 | 85.0° | 0° | 0° (still 85° bent) |

Hips had the same centering, and the flexion hint pointed the wrong way. `Hip_1` and `Hip_2` share the pelvis position, so a cross product of the hip bones is zero. The fallback facing was +Z. The toes point −Z, so that hint was the character's back. A thigh hanging down was about 71–74° outside the old negative allowance.

Natural angular friction is 2 N·m on every axis. It was not what stopped extension. With that friction still applied, a velocity drive reached under 9° at the elbows and under 3° at the knees.

## What changed

Elbow and knee parent frames use the parent bone as X. That direction is full extension. The hinge is the cross product of the parent bone and the child bone, signed so the imported bend is positive flexion. The child frame uses the child bone as X. Positive swing 2 increases that bend on both sides. The JSON limits stay 0° to 145°. A straight pose is legal. 25° of hyperextension and 160° of flexion are not.

Hips use one frame for both sides. X is world down. Positive swing 2 swings the thigh toward the toes. The limits stay twist ±40°, abduction ±45°, flexion −30° to 110° from vertical. The crouched pose is inside that range, a hanging thigh is 0°, and 45° of hyperextension is outside it.

Shoulder cones are unchanged: 80° around the imported arm direction. An arm hanging at the side is 38° and 36° from that pose, inside the cone.

Positional degrees of freedom stay locked. Motors stay off.

## Measured after the correction

Linux Release, same character and Natural preset.

| Joint | Bind swing 2 | Straight pose | Zero-friction drive | Drive with 2 N·m friction | Separation at extension |
| --- | ---: | ---: | ---: | ---: | ---: |
| elbow_1 | 79.2° | 0.0° | −0.13° (on the 0° limit) | 8.7° | 0.6 mm |
| elbow_2 | 78.5° | 0.0° | −0.20° | 8.7° | 0.5 mm |
| knee_1 | 89.0° | 0.0° | −0.06° | 2.2° | 0.3 mm |
| knee_2 | 85.0° | 0.0° | −0.18° | 2.0° | 0.2 mm |

The sub-degree negative values are solver slop against the 0° stop, not a configured hyperextension range. Adding 20° of flexion increased swing 2 by 20.0° on all four joints.

| Hip | Bind twist | Bind abduction | Bind flexion from vertical | Thigh-down swing 2 |
| --- | ---: | ---: | ---: | ---: |
| hip_1 | −18.2° | −23.8° | 74.4° | 0.0° |
| hip_2 | +19.7° | +27.5° | 70.6° | 0.0° |

The abduction signs are opposite, which matches the thighs pointing outward. Both hanging poses stayed inside the limits and the pivots stayed within 1 cm.

The extension drive was run in zero gravity with the parent body held, 3 m above the floor. One or two contacts were reported, and the joint still reached the 0° swing-2 limit, so those contacts were not what had been holding the limb bent.
