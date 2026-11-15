# Foot lifting

v0.4.4 does not add a dynamic lift. The pre-positioned right-leg pose still starts with the other foot clear, and that clearance is part of the reset, not a step. The motors lose the pose in about 0.63 s. The left pose is not placed. See `docs/SINGLE_LEG_FEASIBILITY.md`.

v0.4.2 does not start a lift from two-foot standing until the sole has risen by 4 mm while the stance foot is still holding the torso. That rise was not measured. The Jacobian below is unchanged. The unloading trials are in `docs/ACTIVE_FOOT_UNLOADING.md`.

# Foot lifting (v0.4.1)

## Why the v0.4.0 offsets were not kept

A fixed extra flexion of 0.28 rad at the hip and 0.40 rad at the knee raised the sole about 5 mm while the center of mass stayed above 0.9 m, then the character fell. A larger pair, 0.50 and 0.85 rad, raised it about 8 mm and also fell. Those commands replaced the balance correction on the swing hip and knee, so the stance lost that hip as soon as the lift started. v0.4.1 does not use a fixed offset as the lift.

## Tracking

When a lift is allowed, the sole height is the lower of the heel and forefoot points. The target height ramps from the grounded sole to the requested clearance over the lift duration, holds, then ramps back down. The error is that target minus the measured sole height.

The correction is one damped least-squares step per frame of a 1×2 Jacobian. The columns are 0.12 and 0.22 m/rad for swing-hip flexion and swing-knee flexion, with damping 0.01. The height error used in one frame is clamped to 6 mm so a bad reading cannot demand a full step. Hip flexion is clamped to 0.85 rad and knee flexion to 1.05 rad, and either one stops growing when the joint is within 0.08 rad of its swing2 limit. The bias is added to the balance correction. It does not replace it. The swing ankle receives at most 0.2 rad against the measured sole pitch so the foot stays nearer to level without being driven through the joint limit. The final Euler is still clamped to the anatomical range by the balance controller.

The planned target used for the overlay is the grounded sole plus the requested clearance. The foot is not moved to that point. The largest foot step in the trajectory test was 1.7 mm.

## When a lift is allowed

The lift starts only after the weight shift and a short settle, and only if the swing foot has unloaded by the contact rule above while the stance foot is still touching, the torso tilt is under 0.40 rad, and the center of mass is within 3.5 cm of the support target. A timer does not start the lift. If the swing foot is still touching after one second, the attempt aborts with "swing foot did not unload; lift refused" and returns to two-foot standing.

## What a forced lift did

One trial ignored that gate and started the lift after the center of mass had reached the shifted target, while the swing foot still had its contacts. The sole rose 11 mm while the center of mass stayed above 0.9 m. The stand was then lost and the center of mass fell to 0.85 m. Slowing the same Jacobian cut the upright rise to about 3 mm and the body still fell. That gate was removed. The shipped controller refuses the lift instead of toppling.

A 3 cm clearance, a hold, and a controlled lower were not demonstrated. Lowering is implemented: the target ramps down, and the state leaves LowerFoot when the swing foot is touching and the sole is within 2 cm of the height it had at the start of the lift. That path is not reached while the foot never leaves the ground. Joint commands stayed inside the anatomical limits, and motor torque did not exceed the configured limit (the excess measured during the sequence was 0).
