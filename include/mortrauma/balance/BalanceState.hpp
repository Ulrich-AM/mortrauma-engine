#pragma once

#include "mortrauma/core/Math.hpp"

#include <string>
#include <vector>

namespace mt {

enum class BalancePhase { Inactive, Initializing, Standing, Recovering, Falling, Grounded };

inline const char* balancePhaseName(BalancePhase phase) {
    switch (phase) {
        case BalancePhase::Inactive: return "Inactive";
        case BalancePhase::Initializing: return "Initializing";
        case BalancePhase::Standing: return "Standing";
        case BalancePhase::Recovering: return "Recovering";
        case BalancePhase::Falling: return "Falling";
        case BalancePhase::Grounded: return "Grounded";
    }
    return "Inactive";
}

enum class FootRegion { None, Heel, Midfoot, Forefoot };

inline const char* footRegionName(FootRegion region) {
    switch (region) {
        case FootRegion::Heel: return "heel";
        case FootRegion::Midfoot: return "midfoot";
        case FootRegion::Forefoot: return "forefoot";
        case FootRegion::None: return "none";
    }
    return "none";
}

struct BalanceContact {
    Vec3 point{};
    Vec3 normal{};
    bool supporting = false;
    FootRegion region = FootRegion::None;
};

// One simulation step of sensing. Geometric margin uses the raw contact hull.
// A positive margin does not mean a moving body can stop before it falls.
struct BalanceState {
    bool finite = true;
    BalancePhase phase = BalancePhase::Inactive;
    Vec3 com{};
    Vec3 comVelocity{};
    Vec3 comProjection{};
    Vec3 supportCenter{};
    Vec3 facing{0.f, 0.f, -1.f};
    Vec3 right{1.f, 0.f, 0.f};
    float totalMass = 0.f;
    float margin = 0.f;
    float captureDistance = 0.f;
    bool leftGrounded = false;
    bool rightGrounded = false;
    bool leftTouching = false;
    bool rightTouching = false;
    int leftPoints = 0;
    int rightPoints = 0;
    // Contact count, not a measured force. Jolt's contact callback does not
    // expose the solved impulse to this listener.
    float leftLoadProxy = 0.f;
    float rightLoadProxy = 0.f;
    int leftHeel = 0;
    int leftMid = 0;
    int leftFore = 0;
    int rightHeel = 0;
    int rightMid = 0;
    int rightFore = 0;
    float supportArea = 0.f;
    float leftArea = 0.f;
    float rightArea = 0.f;
    bool supportReliable = false;
    float leftSolePitch = 0.f;
    float leftSoleRoll = 0.f;
    float rightSolePitch = 0.f;
    float rightSoleRoll = 0.f;
    float leftHeelClearance = 0.f;
    float leftForeClearance = 0.f;
    float rightHeelClearance = 0.f;
    float rightForeClearance = 0.f;
    float torsoTilt = 0.f;
    Vec3 torsoUp{0.f, 1.f, 0.f};
    Vec3 torsoAngularVelocity{};
    float pelvisTilt = 0.f;
    Vec3 pelvisUp{0.f, 1.f, 0.f};
    bool ankleAuthority = false;
    bool hipAuthority = false;
    std::string strategy = "none";
    float ankleEffort = 0.f;
    float hipEffort = 0.f;
    int saturatedMotors = 0;
    Vec3 correction{};
    std::vector<BalanceContact> contacts;
    std::vector<Vec3> hull;
};

}  // namespace mt
