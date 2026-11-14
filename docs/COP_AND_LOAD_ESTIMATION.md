# Center of pressure and load

## What this Jolt pin can report

The contact listener used here is Jolt 5.3 `ContactListener`. `OnContactAdded` and `OnContactPersisted` receive the manifold point and normal. They do not receive the solved impulse. `PhysicsSystem::mContactManager` is private, so the solver's contact constraints are not readable after the step either.

`BalanceState::contactImpulseAvailable` is therefore false, and `copAvailable` is false. Left, right, heel, and forefoot load fractions are stored as −1, which the laboratory prints as unavailable. Contact counts remain on the older fields and are labeled as counts. They are not forces.

## The formula, tested without the solver

When a step does have normal impulses, the center of pressure on a flat floor is

```text
p = Σ (J_n,i * p_i) / Σ J_n,i
```

`centerOfPressure` implements that. Impulses that are missing, zero, or negative are ignored. If nothing positive remains, the result is invalid rather than a geometric average of the points. EQ-005 checks a two-point case: impulses 1 and 3 at (0,0,0) and (2,0,4) give (1.5, 0, 3) and a total of 4. An empty list and a zero impulse are invalid.

Nothing in the running character calls this with live contacts, because those impulses are not available. The magenta COP marker in the laboratory is drawn only when `copAvailable` is true, so it stays off.

## What is shown instead

The support center is the average of the contact-hull vertices. It is a shape, not a load.

The equilibrium target is the average of the feasible sole points: each ankle projected onto the ground, plus the configured lead toward the toes (0 in v0.3.2), clamped onto that foot's heel-to-forefoot segment. Feet that are off the ground are left out once any foot is supporting. This is a kinematic reference. It is not a center of pressure.

The pitch-moment estimate assumes the two feet split the weight when both are supporting, and uses only the forward component of the center-of-mass offset from each ankle. A lateral offset between the feet is a force difference, not an ankle pitch. With one foot down, the estimate uses the full horizontal offset and the whole weight. The number is an estimate. It is not the motor torque, which is read separately from the joint.

## Laboratory

The Standing Equilibrium panel states which lines are calculated and which are unavailable. Record CSV writes time, center of mass, the equilibrium target, the pitch estimate, tilt, motor effort, margin, and phase. The COP column is 0 until a solver hook exists.
