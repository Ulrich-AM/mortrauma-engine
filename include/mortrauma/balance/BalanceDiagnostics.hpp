#pragma once

#include <vector>

namespace mt {

struct BalanceSample {
    float time = 0.f;
    float comForward = 0.f;
    float comSpeed = 0.f;
    float torsoTilt = 0.f;
    float ankleEffort = 0.f;
    float hipEffort = 0.f;
    float phase = 0.f;
};

// Viewport toggles and the bounded plots. Each overlay is independent.
struct BalanceDiagnostics {
    bool showCom = true;
    bool showProjection = true;
    bool showVelocity = true;
    bool showContacts = true;
    bool showPolygon = true;
    bool showNormals = true;
    bool showTorso = true;
    bool showCorrection = true;
    bool showFootCollider = false;
    bool showSoleFrame = false;
    bool showHeel = true;
    bool showForefoot = true;
    bool showSolePlane = true;
    bool showEquilibrium = true;
    bool showCop = true;
    bool showAnkleTorque = true;
    float time = 0.f;
    std::vector<BalanceSample> history;
};

}  // namespace mt
