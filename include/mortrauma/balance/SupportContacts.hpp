#pragma once

#include "mortrauma/balance/BalanceState.hpp"
#include "mortrauma/physics/PhysicsWorld.hpp"
#include "mortrauma/rig/Ragdoll.hpp"

#include <string>
#include <vector>

namespace mt {

struct SupportEstimate {
    bool leftGrounded = false;
    bool rightGrounded = false;
    bool leftTouching = false;
    bool rightTouching = false;
    int leftPoints = 0;
    int rightPoints = 0;
    float leftLoadProxy = 0.f;
    float rightLoadProxy = 0.f;
    int leftHeel = 0;
    int leftMid = 0;
    int leftFore = 0;
    int rightHeel = 0;
    int rightMid = 0;
    int rightFore = 0;
    float area = 0.f;
    float leftArea = 0.f;
    float rightArea = 0.f;
    bool reliable = false;
    Vec3 center{};
    float margin = 0.f;
    std::vector<BalanceContact> contacts;
    std::vector<Vec3> hull;
    std::vector<Vec3> supportPoints;
};

// foot_1 is the character's right foot and foot_2 is the left foot when the
// character faces -Z. Support requires a real contact whose normal points up.
// Proximity to the floor is not support.
SupportEstimate estimateSupport(const PhysicsWorld& world, const Ragdoll& ragdoll);

// Signed distance from a ground-plane point to the hull. Positive is inside.
// A single point or a segment has no interior, so the margin is zero only on
// the feature and negative anywhere else.
float supportMargin(const std::vector<Vec3>& hull, Vec3 groundPoint);
float distanceOutsideSupport(const std::vector<Vec3>& hull, Vec3 groundPoint);

}  // namespace mt
