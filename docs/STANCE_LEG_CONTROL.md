# Stance-leg control (v0.4.3)

The balance controller remains the only writer of motor targets. A stance selection changes which joints receive the balance delta. It does not add a second motor task.

- Both legs, the normal standing mode: both hips keep the shared swing1 sign. This is the two-foot preset and it is not retuned.
- Right leg: the right hip takes the standing swing1 sign. The left hip and knee stay on the placed reference.
- Left leg: the left hip takes the opposite swing1 sign. That is the mirrored mapping from v0.4.1. The right hip and knee stay on the placed reference.

Forward hip swing2 keeps the shared sign on the stance hip. The ankles are still corrected only while that foot is touching. Spine corrections are unchanged.

The selection is cleared by Reset to Standing and by Abort Experiment. It is not a welded foot, a kinematic body, or a support fixture.

This mode does not make the right-leg pose stable. After the v0.4.4 pose ranking the hold is about 0.63 s. It is kept so the swing hip is not driven by the two-foot law while that foot is supposed to stay up. The left candidate is not placed, so this mode is not entered for that side unless the diagnostic checkbox is on. That checkbox is labeled diagnostic only and is not a standing pose.

The support polygon used once a foot leaves the ground is the contact hull of the feet that are still touching. A contact count is not a force. The center of pressure remains unavailable. The static margin in the feasibility report is the geometric sole, computed before the simulation, and it is labeled as a moment balance rather than a measured load.
