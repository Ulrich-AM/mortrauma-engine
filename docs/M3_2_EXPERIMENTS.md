# M3.2 experiments

Each row is one change from the v0.3.1 Normal stand. Same character, same floor, no fixture, no impulse unless the row says so. Duration is the last time the center of mass is above 0.75 m, torso tilt is under 0.70 rad, and the phase is not Falling or Grounded. The v0.3.1 baseline is also in `docs/data/standing_baseline_v031.csv`.

| Run | Change | Duration | COM to ankles at placement | Unloaded left-right sole gap | Result |
| --- | --- | ---: | ---: | ---: | --- |
| A | v0.3.1 pose and centroid tracking | 1.39 s | 60 mm | 12.0 mm | Falls forward. Unrecoverable at 1.40 s. |
| B | Knee height match only, still the 75/25 target | 1.28 s to 1.73 s | 35–67 mm | 1.4 mm, then 12 mm on the one-shot | Falls. Re-leveling the ankles after the knees increases sole error past 1.5°. The one-shot knee change does not shrink the gap. |
| C | Ankle equilibrium target only | 30 s | 6.9 mm | 12.5 mm | Stays Standing. This is the change that matters. |
| D | Ankle target plus ankle re-search after the knees | 1.58 s and 2.55 s | 30–49 mm | 2.3 mm, sole error 1.2° | Falls. The second ankle search leaves the zero-moment pose. |
| D′ | Ankle target plus one knee nudge, reverted when the gap does not shrink | 30 s | 7.2 mm | 12.5 mm | Same stand as C. The nudge is not kept. |
| E | Gain, damping, frequency, torque, friction | not changed | — | — | C already held 30 s. No second knob was turned. |
| F | 6 N·s forward and lateral chest impulses | recovered | — | — | Center of mass stays above 1.02 m. v0.3.1 fell. |
| F | 55 N·s forward | falls | — | — | BAL-005. The pelvis is not anchored. |
| G | Quiet stand to 30 s | 30 s | 6.9 mm | 1.9 mm once loaded | Phase Standing at every sample. Tilt stays under 0.22 rad. Margin stays about +57 mm. |

Failed rows were not folded into the default. `coplanarFeet` remains false because B and D fall or do nothing. `equilibriumReference` is true because C holds.

At 30 s on run G the pitch-moment estimate is about 5 N·m, ankle motor effort about 12 N·m as a vector, hip effort about 23 N·m, and two joints are near saturation. No axis exceeds its limit. The earliest divergence in the runs that fall is the center of mass walking forward of the ankles while the ankles are already at the cap. That is the reference, not a lack of torque: 12 N·m is enough when the arm is a centimetre, and it is not enough at 60 mm.
