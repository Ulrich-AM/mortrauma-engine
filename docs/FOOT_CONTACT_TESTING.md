# Foot contact testing

Linux Release, 120 Hz, one Humanoidguy, Normal balance preset, standing floor, no pelvis fixture. The flat-sole collider and the sole-leveled pose are both active unless a row says capsule.

Initial sole, before the first physics step:

| Check | foot_1 (right) | foot_2 (left) |
| --- | ---: | ---: |
| Sole normal, world up component | 0.999998 | 0.999962 |
| Heel clearance | 0.0041 m | 0.0150 m |
| Forefoot clearance | 0.0044 m | 0.0176 m |

The floor top is y = 0. A level sole here means the sole normal's vertical component is above 0.95, about 18 degrees. The measured soles are much closer than that. Heel and forefoot on the same foot differ by about 0.3 mm on the right and 2.6 mm on the left, so the stance is not starting on one edge.

After 30 to 40 steps of the Normal controller:

| Measurement | Value |
| --- | --- |
| Supporting contacts | 8, all classified |
| Heel / forefoot counts | 2 and 2 on each foot |
| Support polygon area | 0.309 m², built from the contact hull |
| Projected COM offset from the support center | 0.062 m |
| Support margin | +0.149 m (inside) |
| Lowest contact | −0.015 m |
| Foot friction | 0.7 |

Ankle separation stayed under 2 cm. Mass of each foot stayed 1.3 kg.

## Cases

| Test | Result |
| --- | --- |
| FOOT-001 | Box half extents match the ranges above, mass is 1.3 kg, and the physics shape is a box with the stored offset. |
| FOOT-002 | Both sole normals have an up component above 0.95. |
| FOOT-003 | Heel and forefoot clearances are between the floor and 8 cm, and the two ends of each sole are within 4 cm of each other. |
| FOOT-004 | Both ankles still join the calf to the foot after 0.5 s. |
| FOOT-005 | One second stays finite and the deepest contact is above −5 cm. |
| FOOT-006 | Every supporting contact is heel, midfoot, or forefoot. |
| FOOT-007 | Hull vertices are contact points. Three or more contacts produce a positive area. |
| FOOT-008 | The projected center of mass starts within 0.45 m of the measured support center. |
| FOOT-009 | Swapping to the capsule and back stays finite. Contact counts were 8 for the box and 3 for the capsule. |
| FOOT-010 | Foot friction is 0.7, not an extreme value. |

These checks do not claim a long stand. Duration is in `docs/BALANCE_STABILITY_COMPARISON.md`.
