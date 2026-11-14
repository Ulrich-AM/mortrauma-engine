# MORTRAUMA v0.4.2

v0.4.2 changes how a foot lift is allowed to start. The body stays a torque-limited ragdoll. He does not walk, and he does not yet stand on one leg.

v0.4.1 refused the lift until the swing foot reported no contacts. That foot still had four contacts after the weight shift, so the lift never ran. Waiting for the contact count also could not start the motion that would remove the contact.

Solved contact impulses are not available from this Jolt pin. The replacement is a small swing-knee flexion, at most 0.12 rad, commanded while the foot may still be touching. The swing hip is left to the standing balance law. The sole rise under that command is about 1 mm, and the center of mass stays near 1.01 m. Larger knee or hip flexion was tried. It tips the pelvis before the sole clears, because the center of mass is still about 20 cm from the stance foot and the 50 N·m hip cannot hold the body there. Those larger commands were not kept.

The 30 s quiet stand still finishes upright. A shift toward either foot still moves the center of mass that way: about 53 mm to the left and 47 mm to the right. A 55 N·s push still falls. Nothing pins the feet.

`MORTRAUMA-v0.4.2-Windows-x64.zip` contains `MORTRAUMA.exe`, Humanoidguy, and the configurations. Releases v0.1.0 through v0.4.1 are not replaced.
