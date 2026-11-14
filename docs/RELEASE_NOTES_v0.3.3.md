# MORTRAUMA v0.3.3

v0.3.3 adds the supplied Y-Bot figure as a second character. Humanoidguy is still the default, with the same bodies, the same 12 N·m ankles, and the same standing pose as v0.3.2. Both use the existing motor controller and the existing balance controller. One character is in the scene. The Character window swaps them by destroying the ragdoll and building the selected one.

The supplied file is a rigid segmented model, not a skinned Mixamo skeleton. It is not in this package. Copy it to `assets/characters/ybot.fbx` to select Y-Bot. The comparison, including masses, foot boxes, torque limits, and the standing trials, is in the repository at `docs/YBOT_COMPARISON.md`. Both characters completed a 15 s quiet stand and a 2 N·s push in those trials. Y-Bot's torso moved less and its ankles saturated less, while its hips ran closer to a lower torque limit. That is a different body and a different torque file, not a claim that the skeleton is a better foundation.

`MORTRAUMA-v0.3.3-Windows-x64.zip` contains `MORTRAUMA.exe`, Humanoidguy, and the configurations. It does not contain the Y-Bot FBX. Releases v0.1.0 through v0.3.2 are not replaced.
