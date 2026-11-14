# Single-leg balance (v0.4.1)

Jolt's contact listener in this pin does not report the solved impulse. The controller does not invent a load percentage. `leftLoadFraction` and `rightLoadFraction` stay negative. Support is described from contact counts and from whether a foot is touching:

- both feet touching
- only the left foot touching
- only the right foot touching
- no foot touching

A swing foot is treated as unloaded only when its contact count reaches zero, or when the count drops and the sole has risen at least 8 mm. That is a contact estimate, not a force.

The single-leg test shifts the center of mass toward the selected foot and then waits. On both feet the swing foot still had 4 contacts after the shift. The note is "swing foot stayed in contact; lift refused" or, if the shift itself does not settle, "weight transfer timed out". The state machine returns through RestoreSupport to Completed, or to Aborted, and clears the balance offset. It does not enter LiftFoot.

The support margin shown for two-foot contact is the existing hull margin. Once a single foot is the only contact, the margin is the horizontal distance from the center of mass to that foot body, subtracted from a 4 cm radius. That radius is the foot half-width used as a label. It is not a measured center of pressure. With the center of mass still tens of centimetres from the foot, the margin is negative, which matches the refused lift.

No foot is pinned, the pelvis stays dynamic, and no extra force holds the body up. A 55 N·s push still falls.
