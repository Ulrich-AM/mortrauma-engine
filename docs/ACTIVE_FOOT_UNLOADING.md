# Active foot unloading (v0.4.2)

## Why v0.4.1 refused the lift

v0.4.1 entered the lift only when the swing foot had zero contacts, or when the contact count fell and the sole had already risen 8 mm. After a weight shift the swing foot still had four contacts, so the lift never started. That gate is circular if the only way to lose contact is the lift itself.

Jolt's contact listener runs before the solver. The manifold has a normal, a penetration, and contact points. It does not carry the solved impulse, and `PhysicsSystem` does not expose the contact-constraint lambdas. `contactImpulseAvailable` stays false. No load percentage is displayed.

## What replaced the gate

After the center of mass has been shifted toward the stance foot, `BeginUnloading` commands the swing knee to flex by up to 0.12 rad. The swing hip is left to the balance law. The command starts while the swing foot is still touching. A drop in contact count is recorded and is not treated as a load.

The lift starts only if, while the stance foot is still touching, the torso tilt is under 0.32 rad, the center of mass is above 0.94 m, and the stance foot's horizontal speed is under 0.12 m/s, one of these is also true:

- the sole has risen at least 4 mm and is not moving downward, or
- the contact count has fallen and the sole has risen at least 1 mm.

A timeout without that evidence aborts. It is not a successful unload. If the torso, height, or stance slip leaves that region, the swing foot is lowered and the bias is cleared.

## Trials

Each trial uses the same standing pose and the v0.4.1 weight shift. The sole rise is sampled only while the center of mass is above 0.90 m.

| Swing command | Upright sole rise | Lowest center of mass | Result |
| --- | --- | --- | --- |
| Signed geometric Jacobian, uncapped | 0.4 mm at 0.45 s, hip flexion already 0.26 rad | Fell | The bend-axis sign did not match positive swing2. Reverted. |
| Positive flexion capped at 0.12 rad hip and knee | about 1 mm | 1.01 m | Stayed up. The sole did not clear. |
| Positive flexion capped at 0.20 rad hip and 0.30 rad knee | about 2 mm | 0.86 m | Lost the stand. Reverted. |
| Knee only, up to 0.35 rad, hip left to balance | about 2 mm | 0.97 m, and a longer hold fell | The planted foot turns the flexion into pelvis motion. Reverted. |
| Knee only, up to 0.12 rad (kept) | about 1 mm | 1.01 m | The command runs. The sole does not rise. The stand is kept. |

The kept command is the largest shortening that did not take the center of mass down. It is not a successful unload.

## Why the foot stays down

During double support the center of mass moves about 5 cm toward the selected foot. That foot is still about 20 cm from the center of mass. The full body weight at a 20 cm arm is about 150 N·m. The hip limit is 50 N·m, which can balance the body only about 6 cm from the ankle. The feasible single-leg region does not overlap the posture the double-support shift can hold.

Flexing the swing leg while it is a load-bearing contact does not peel the sole off the floor. The floor reaction absorbs the motion until the pelvis tips. Torque limits and friction were not increased.

v0.4.3 did not repeat those flexion trials. It asked a different question: whether any pose, placed before the simulation, puts the center of mass over one ankle inside the joint torque limits. That result is in `docs/SINGLE_LEG_FEASIBILITY.md`.
