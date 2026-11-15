# M4.4 experiments (v0.4.4)

The question is why the left-leg and right-leg equilibrium resets started in different postures, and whether a calmer pose can be selected without crossing a torque, joint, or support limit. Walking and recovery stepping were not added. Torque limits, friction, the sole geometry, and the single motor-command path are unchanged.

## Selection bug

v0.4.3 scored a pose as usable when the torso was under 0.55 rad and the ankle, knee, hip, and spine moments were inside their limits. The hip-root limit was not part of that score. The left pose failed the real gate (hip root 20.3 N·m against 18, torso 0.51 rad) and was written anyway. The arms looked raised because they are fixed to the chest. The swing sweep then minimized the hip-root moment, which is why the right swing knee stayed at 110°.

Both sides use one angle grid. Hip 2 still receives the mirrored swing1 sign. The skeleton is not a perfect mirror: the hip anatomical offsets differ by a few degrees, so the same Euler samples do not both fit inside 18 N·m.

## Corrected rule

A pose is feasible only when the COM projection is inside the sole, within 2 cm of the stance ankle, the sole is flat, the swing foot is clear, the torso tilt is under 0.45 rad, and every joint moment, including the hip root, is inside its motor limit. Among feasible poses the lowest posture cost wins. Posture cost adds torso tilt, lateral spine past 6°, swing knee past 75°, swing-hip flexion past 50°, swing abduction past 20°, and hand displacement from the upright reference. It cannot promote an infeasible pose.

If nothing is feasible, the bodies are left as they were and the note names the failed check. "Place rejected pose (diagnostic only)" writes that sample and prefixes the note with "diagnostic only, not a standing pose."

## Measured poses

Right, written: stance hip 4° and 16°, stance knee 10°, torso 0.046 rad, spine −1°, swing hip −30° and 15°, swing knee 95°, hands 2.0 cm from the upright pose, COM 1.3 cm from the ankle, sole margin 2.6 cm, clearance 47 cm. Static moments: ankle axis 8.6 N·m, ankle vector 10.1 N·m, mass·g·arm 9.7 N·m, knee 30.9, hip 36.4, hip root 17.2. The vector and the gravity arm agree within 0.4 N·m. The measured constraint rotation matches the posed joint quaternion (error 0). The hip root is 0.8 N·m under its limit, so a straighter swing knee is not feasible on this grid.

Left, not written: closest sample has torso 0.39 rad, spine 13°, swing knee 100°, COM 0.8 cm from the ankle, and hip root 24.0 N·m at `hip_root_2`. The rejection says that limit and that the pose was not placed. Searching it does not move a right pose that is already in the world.

## First 0.6 s after the corrected right pose

Normal 80 Hz task, stance-hip correction, no hidden support. At 0.05 s the center of mass is at 1.10 m, torso tilt is 0.065 rad, the ankle command vector is 20.8 N·m, the hip command is 56 N·m, six motors are saturated, and the balance estimate of the ankle gravity moment is 3.7 N·m. The servo is still pushing the planted foot. At 0.2 s the right foot is down and the left foot is clear. The center of mass passes 0.85 m at 0.63 s, with torso tilt 0.47 rad. This is not a sustained stand.

The v0.4.3 6 Hz and 16 Hz trials held the older pose for 0.40 s and 0.54 s and were not repeated. The task rate stays at the two-foot preset.

## Regression

The 30 s two-foot stand, the 55 N·s chest impulse, and the chest grab are the same checks as v0.4.3. No torque or friction value was raised.
