# Grab crash investigation

The Grab tool creates a kinematic anchor and a SixDOF constraint, then moves the anchor with the cursor. Releasing the mouse, resetting, switching tools, or closing the window destroys that constraint and the anchor.

## What crashed

The failure is in `PhysicsWorld::createJoint`, which the grab path calls from `beginGrab`. The function locked the parent body and the child body with two separate `JPH::BodyLockWrite` calls. Jolt hashes body ids onto a small set of mutexes. When the new kinematic anchor and the grabbed body landed in the same bucket, the second lock was the same mutex the thread already held. `pthread_mutex_lock` returned `EDEADLK`, and libstdc++ threw `std::system_error` with "Resource deadlock avoided". The process aborted (exit 134) before the constraint existed.

This is not a double-free of the constraint and not an illegal angular limit. Jolt's own `CreateConstraint` uses `BodyLockMultiWrite`, which sorts and deduplicates the mutexes and locks each one once.

## Fix

`createJoint` now takes one `BodyLockMultiWrite` over the parent and child ids, then creates the constraint. The same path serves ragdoll joints, the grab spring, and the pelvis support fixture.

Two related failures showed up while the grab was exercised under a debug Jolt build:

- A kinematic anchor that is allowed to sleep, while constrained to a dynamic body, trips a Jolt assert. Grab anchors and support anchors are created with `allowSleeping = false`.
- A grab target that is non-finite, or many metres away from the hit, makes the next constraint solve non-finite. `updateGrab` rejects non-finite targets and clamps the anchor to 4 m from the grabbed body.

Reset, tool changes, and shutdown call `clearGrabs`, which ends every live grab.

## Evidence

GRAB-001 through GRAB-007 repeat begin/release, reset during a grab, grab with motors, grab with the pelvis fixture, rapid target motion, and grab while balance is commanding motors. They run in the Linux test binary. The original abort was the `std::system_error` from the non-recursive lock, which these tests used to hit when the anchor and the grabbed body shared a mutex. After the multi-lock change they complete without that exception.

The Windows Release run is the CI job for this branch. This document does not claim a crash that was only seen on one mutex hash is impossible under every future Jolt version. The lock pattern now matches the one Jolt uses internally.
