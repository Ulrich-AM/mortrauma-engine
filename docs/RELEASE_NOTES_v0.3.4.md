# MORTRAUMA v0.3.4

v0.3.4 restores a Humanoidguy-only laboratory and remeasures the standing controller from v0.3.2. The Y-Bot character added in v0.3.3 is gone: no character window, no second rig, no Y-Bot motor or balance files, and no rigid-mesh importer. A model with no skin bones fails to import. Humanoidguy's bodies, 12 N·m ankles, foot boxes, and ankle equilibrium target are unchanged.

The visible feet and the collision boxes use the same transforms. Mesh vertices sit inside the boxes, and the debug corners match Jolt's world bounds. Unloaded soles differ by about 12 mm left to right; under load that gap is about 2 mm. The pose was not retuned.

On the Normal preset, fixed step 1/120 s, no support fixture, a quiet stand is still upright at 30 s (center of mass near 1.03 m, torso tilt near 0.17 rad, drift about 37 mm). Passive motors and a motor hold without balance feedback both fall. A 6 N·s backward push still topples the character. A 55 N·s push falls. Ankle 1's twist axis stays at its 12 N·m cap for the whole quiet stand. The cap was not raised: the pitch moment once the feet are down is about 5 N·m. An attempt to stop commanding ankle twist did not reduce that torque and was reverted.

The Balance Laboratory keeps the Motor Laboratory. Traces sit in a collapsible section. The panel shows how many ankle and hip motors are saturated, and the standing CSV adds heel and forefoot contact counts. The pre-contact ankle-moment readout is 0 instead of a false ~196 N·m.

`MORTRAUMA-v0.3.4-Windows-x64.zip` contains `MORTRAUMA.exe`, Humanoidguy, and the configurations. It does not contain a second character. Releases v0.1.0 through v0.3.3 are not replaced.

This is a measured restoration of the v0.3.2 quiet stand, not a claim that balance rejects every modest push. There is still no recovery step.
