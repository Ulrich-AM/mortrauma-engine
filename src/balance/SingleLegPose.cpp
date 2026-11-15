#include "mortrauma/balance/SingleLegPose.hpp"

#include "mortrauma/balance/SupportContacts.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace mt {
namespace {

Quat frameQuat(Vec3 axisX, Vec3 axisY) {
    const Vec3 x = normalize(axisX);
    Vec3 y = axisY - x * dot(axisY, x);
    if (length(y) < 1e-5f) {
        const Vec3 fallback = std::fabs(x.y) < 0.9f ? Vec3{0.f, 1.f, 0.f} : Vec3{1.f, 0.f, 0.f};
        y = normalize(cross(x, fallback));
    } else {
        y = normalize(y);
    }
    const Vec3 z = normalize(cross(x, y));
    y = normalize(cross(z, x));
    Mat4 basis;
    basis.m[0] = x.x;
    basis.m[1] = x.y;
    basis.m[2] = x.z;
    basis.m[4] = y.x;
    basis.m[5] = y.y;
    basis.m[6] = y.z;
    basis.m[8] = z.x;
    basis.m[9] = z.y;
    basis.m[10] = z.z;
    return basis.rotation();
}

Quat uprightPelvis(const RagdollBody& pelvis) {
    Vec3 localUp = pelvis.segmentEnd - pelvis.segmentStart;
    if (length(localUp) < 1e-4f) {
        localUp = {0.f, 1.f, 0.f};
    }
    localUp = normalize(rotate(conjugate(pelvis.initialRotation), localUp));
    Vec3 localForward = rotate(conjugate(pelvis.initialRotation), Vec3{0.f, 0.f, -1.f});
    localForward = localForward - localUp * dot(localForward, localUp);
    if (length(localForward) < 1e-4f) {
        localForward = {0.f, 0.f, -1.f};
    }
    localForward = normalize(localForward);
    const Vec3 srcX = normalize(cross(localForward, localUp));
    const Vec3 srcY = localUp;
    const Vec3 srcZ = normalize(cross(srcX, srcY));
    Mat4 local;
    local.m[0] = srcX.x;
    local.m[1] = srcX.y;
    local.m[2] = srcX.z;
    local.m[4] = srcY.x;
    local.m[5] = srcY.y;
    local.m[6] = srcY.z;
    local.m[8] = srcZ.x;
    local.m[9] = srcZ.y;
    local.m[10] = srcZ.z;
    return normalize(conjugate(local.rotation()));
}

float torqueLimit(const std::string& id) {
    if (id == "hip_1" || id == "hip_2") {
        return 50.f;
    }
    if (id == "knee_1" || id == "knee_2") {
        return 35.f;
    }
    if (id == "ankle_1" || id == "ankle_2") {
        return 12.f;
    }
    if (id == "hip_root_1" || id == "hip_root_2") {
        return 18.f;
    }
    if (id == "spine_stomach") {
        return 55.f;
    }
    if (id == "spine_chest") {
        return 50.f;
    }
    if (id == "spine_neck") {
        return 32.f;
    }
    if (id == "neck") {
        return 12.f;
    }
    return 80.f;
}

struct Chain {
    std::vector<Vec3> position;
    std::vector<Quat> rotation;
    std::unordered_map<std::string, Quat> joint;
};

int indexOf(const Ragdoll& ragdoll, const std::string& id) {
    for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
        if (ragdoll.bodies[i].id == id) {
            return i;
        }
    }
    return -1;
}

void rebuild(const Ragdoll& ragdoll, Chain& chain) {
    const int count = int(ragdoll.bodies.size());
    std::vector<char> placed(count, 0);
    const int pelvis = indexOf(ragdoll, "pelvis");
    if (pelvis < 0) {
        return;
    }
    placed[pelvis] = 1;
    bool progress = true;
    for (int guard = 0; progress && guard < 32; ++guard) {
        progress = false;
        for (const auto& joint : ragdoll.joints) {
            if (joint.parent < 0 || joint.child < 0 || placed[joint.parent] == 0 || placed[joint.child] != 0) {
                continue;
            }
            const auto found = chain.joint.find(joint.id);
            const Quat target = found == chain.joint.end() ? Quat::identity() : found->second;
            const Quat parentFrame = frameQuat(joint.axisXInParent, joint.axisYInParent);
            const Quat childFrame = frameQuat(joint.axisXInChild, joint.axisYInChild);
            const Quat parentConstraint = mul(chain.rotation[joint.parent], parentFrame);
            const Quat childConstraint = mul(parentConstraint, target);
            chain.rotation[joint.child] = normalize(mul(childConstraint, conjugate(childFrame)));
            const Vec3 anchor = chain.position[joint.parent] + rotate(chain.rotation[joint.parent], joint.anchorInParent);
            chain.position[joint.child] = anchor - rotate(chain.rotation[joint.child], joint.anchorInChild);
            placed[joint.child] = 1;
            progress = true;
        }
    }
}

void levelFoot(const Ragdoll& ragdoll, Chain& chain, const std::string& ankleId) {
    const RagdollJoint* ankle = nullptr;
    for (const auto& joint : ragdoll.joints) {
        if (joint.id == ankleId) {
            ankle = &joint;
        }
    }
    if (ankle == nullptr || ankle->child < 0 || ankle->parent < 0) {
        return;
    }
    const RagdollBody& foot = ragdoll.bodies[ankle->child];
    const Vec3 sole = normalize(rotate(chain.rotation[ankle->child], foot.soleNormalLocal));
    const Quat leveled = normalize(mul(quatFromTo(sole, Vec3{0.f, 1.f, 0.f}), chain.rotation[ankle->child]));
    const Quat parentFrame = frameQuat(ankle->axisXInParent, ankle->axisYInParent);
    const Quat childFrame = frameQuat(ankle->axisXInChild, ankle->axisYInChild);
    const Quat parentConstraint = mul(chain.rotation[ankle->parent], parentFrame);
    const Quat childConstraint = mul(leveled, childFrame);
    const Quat target = normalize(mul(conjugate(parentConstraint), childConstraint));
    const Vec3 euler = constraintEulerRadians(target);
    const float margin = radians(1.f);
    chain.joint[ankleId] = quatFromEulerXYZ(clampf(euler.x, ankle->base.twistMin + margin, ankle->base.twistMax - margin),
                                            clampf(euler.y, ankle->base.swing1Min + margin, ankle->base.swing1Max - margin),
                                            clampf(euler.z, ankle->base.swing2Min + margin, ankle->base.swing2Max - margin));
    rebuild(ragdoll, chain);
}

float soleUpY(const Ragdoll& ragdoll, const Chain& chain, int foot) {
    if (foot < 0) {
        return 0.f;
    }
    return normalize(rotate(chain.rotation[foot], ragdoll.bodies[foot].soleNormalLocal)).y;
}

std::vector<Vec3> soleCorners(const Ragdoll& ragdoll, const Chain& chain, int foot) {
    std::vector<Vec3> corners;
    if (foot < 0) {
        return corners;
    }
    const RagdollBody& body = ragdoll.bodies[foot];
    const Vec3 half = body.boxHalfExtents;
    if (length(half) < 1e-4f) {
        return corners;
    }
    struct Scored {
        float score;
        Vec3 world;
    };
    std::vector<Scored> scored;
    const float signs[2] = {-1.f, 1.f};
    for (float sx : signs) {
        for (float sy : signs) {
            for (float sz : signs) {
                const Vec3 local = body.shapeCenterLocal + rotate(body.shapeRotationLocal, Vec3{sx * half.x, sy * half.y, sz * half.z});
                scored.push_back({dot(local, body.soleNormalLocal), chain.position[foot] + rotate(chain.rotation[foot], local)});
            }
        }
    }
    std::sort(scored.begin(), scored.end(), [](const Scored& a, const Scored& b) { return a.score < b.score; });
    for (int i = 0; i < 4 && i < int(scored.size()); ++i) {
        corners.push_back(scored[i].world);
    }
    if (corners.size() < 3) {
        return corners;
    }
    Vec3 center{};
    for (const Vec3& corner : corners) {
        center = center + corner;
    }
    center = center * (1.f / float(corners.size()));
    std::sort(corners.begin(), corners.end(), [&](Vec3 a, Vec3 b) {
        return std::atan2(a.z - center.z, a.x - center.x) < std::atan2(b.z - center.z, b.x - center.x);
    });
    return corners;
}

float lowestY(const std::vector<Vec3>& corners) {
    float y = 1e9f;
    for (const Vec3& corner : corners) {
        y = std::min(y, corner.y);
    }
    return y;
}

Vec3 bodyUp(const RagdollBody& body, Quat rotation) {
    Vec3 local = body.segmentEnd - body.segmentStart;
    if (length(local) < 1e-4f) {
        local = {0.f, 1.f, 0.f};
    }
    local = normalize(rotate(conjugate(body.initialRotation), local));
    return normalize(rotate(rotation, local));
}

struct Moments {
    float ankle = 0.f;
    float ankleMagnitude = 0.f;
    float knee = 0.f;
    float hip = 0.f;
    float hipRoot = 0.f;
    float spine = 0.f;
    float worstRatio = 0.f;
    std::string worstJoint;
    float worstTorque = 0.f;
    float worstLimit = 0.f;
};

Moments jointMoments(const Ragdoll& ragdoll, const Chain& chain, int stanceFoot, Vec3 cop) {
    Moments moments;
    const int count = int(ragdoll.bodies.size());
    std::vector<int> parentJoint(count, -1);
    std::vector<std::vector<int>> children(count);
    for (int j = 0; j < int(ragdoll.joints.size()); ++j) {
        const auto& joint = ragdoll.joints[j];
        if (joint.parent < 0 || joint.child < 0) {
            continue;
        }
        parentJoint[joint.child] = j;
        children[joint.parent].push_back(j);
    }
    const Vec3 gravity{0.f, -9.81f, 0.f};
    float mass = 0.f;
    for (const auto& body : ragdoll.bodies) {
        mass += body.mass;
    }
    const Vec3 contactForce{0.f, mass * 9.81f, 0.f};
    std::vector<Vec3> force(ragdoll.joints.size());
    std::vector<Vec3> moment(ragdoll.joints.size());
    std::vector<char> done(count, 0);
    int remaining = count;
    while (remaining > 0) {
        bool progressed = false;
        for (int body = 0; body < count; ++body) {
            if (done[body]) {
                continue;
            }
            bool childrenDone = true;
            for (int child : children[body]) {
                if (!done[ragdoll.joints[child].child]) {
                    childrenDone = false;
                }
            }
            if (!childrenDone) {
                continue;
            }
            const int parent = parentJoint[body];
            if (parent < 0) {
                done[body] = 1;
                --remaining;
                progressed = true;
                continue;
            }
            const auto& joint = ragdoll.joints[parent];
            const Vec3 anchor = chain.position[joint.parent] + rotate(chain.rotation[joint.parent], joint.anchorInParent);
            Vec3 netForce = gravity * ragdoll.bodies[body].mass;
            Vec3 netMoment = cross(chain.position[body] - anchor, gravity * ragdoll.bodies[body].mass);
            if (body == stanceFoot) {
                netForce = netForce + contactForce;
                netMoment = netMoment + cross(cop - anchor, contactForce);
            }
            for (int child : children[body]) {
                const auto& childJoint = ragdoll.joints[child];
                const Vec3 childAnchor = chain.position[childJoint.parent] + rotate(chain.rotation[childJoint.parent], childJoint.anchorInParent);
                netForce = netForce - force[child];
                netMoment = netMoment - moment[child] - cross(childAnchor - anchor, force[child]);
            }
            force[parent] = -netForce;
            moment[parent] = -netMoment;
            const Quat parentFrame = frameQuat(joint.axisXInParent, joint.axisYInParent);
            const Quat parentConstraint = mul(chain.rotation[joint.parent], parentFrame);
            const float twist = std::fabs(dot(moment[parent], rotate(parentConstraint, Vec3{1.f, 0.f, 0.f})));
            const float swing1 = std::fabs(dot(moment[parent], rotate(parentConstraint, Vec3{0.f, 1.f, 0.f})));
            const float swing2 = std::fabs(dot(moment[parent], rotate(parentConstraint, Vec3{0.f, 0.f, 1.f})));
            const float peak = std::max(twist, std::max(swing1, swing2));
            const float limit = torqueLimit(joint.id);
            if (limit > 1.f && peak / limit > moments.worstRatio) {
                moments.worstRatio = peak / limit;
                moments.worstJoint = joint.id;
                moments.worstTorque = peak;
                moments.worstLimit = limit;
            }
            if (joint.id.rfind("ankle_", 0) == 0) {
                moments.ankle = std::max(moments.ankle, peak);
                if (joint.child == stanceFoot) {
                    moments.ankleMagnitude = length(moment[parent]);
                }
            } else if (joint.id.rfind("knee_", 0) == 0) {
                moments.knee = std::max(moments.knee, peak);
            } else if (joint.id == "hip_1" || joint.id == "hip_2") {
                moments.hip = std::max(moments.hip, peak);
            } else if (joint.id == "hip_root_1" || joint.id == "hip_root_2") {
                moments.hipRoot = std::max(moments.hipRoot, peak);
            } else if (joint.id.rfind("spine_", 0) == 0) {
                moments.spine = std::max(moments.spine, peak);
            }
            done[body] = 1;
            --remaining;
            progressed = true;
        }
        if (!progressed) {
            break;
        }
    }
    return moments;
}

struct Scored {
    Chain chain;
    SingleLegCandidate result;
    float score = 1e9f;
};

void clampChain(const Ragdoll& ragdoll, Chain& chain) {
    for (const auto& joint : ragdoll.joints) {
        const auto found = chain.joint.find(joint.id);
        if (found == chain.joint.end()) {
            continue;
        }
        const Vec3 euler = constraintEulerRadians(found->second);
        const float margin = radians(1.f);
        const float span1 = joint.base.swing1Max - joint.base.swing1Min;
        const float span2 = joint.base.swing2Max - joint.base.swing2Min;
        const float spanT = joint.base.twistMax - joint.base.twistMin;
        const float pad1 = span1 > radians(4.f) ? margin : 0.f;
        const float pad2 = span2 > radians(4.f) ? margin : 0.f;
        const float padT = spanT > radians(4.f) ? margin : 0.f;
        chain.joint[joint.id] = quatFromEulerXYZ(clampf(euler.x, joint.base.twistMin + padT, joint.base.twistMax - padT),
                                                 clampf(euler.y, joint.base.swing1Min + pad1, joint.base.swing1Max - pad1),
                                                 clampf(euler.z, joint.base.swing2Min + pad2, joint.base.swing2Max - pad2));
    }
}

float postureCost(const SingleLegCandidate& pose, float handOffset) {
    // Radians and degrees are mixed on purpose: each term is a fraction of a
    // "noticeably away from the standing reference" budget, not a torque.
    return pose.torsoTilt + std::max(0.f, std::fabs(pose.spineSwing1Deg) - 6.f) / 12.f +
           std::max(0.f, pose.swingKneeDeg - 75.f) / 40.f + std::max(0.f, pose.swingHipSwing2Deg - 50.f) / 40.f +
           std::max(0.f, std::fabs(pose.swingHipSwing1Deg) - 20.f) / 30.f + handOffset * 4.f;
}

void consider(const Ragdoll& ragdoll, const Chain& neutral, Chain chain, bool supportRight, Scored& calm, Scored& nearest, Scored& best,
              Scored& probe) {
    clampChain(ragdoll, chain);
    rebuild(ragdoll, chain);
    const std::string stanceAnkle = supportRight ? "ankle_1" : "ankle_2";
    const std::string swingAnkle = supportRight ? "ankle_2" : "ankle_1";
    const int stanceFoot = indexOf(ragdoll, supportRight ? "foot_1" : "foot_2");
    const int swingFoot = indexOf(ragdoll, supportRight ? "foot_2" : "foot_1");
    const int chest = indexOf(ragdoll, "chest");
    const int pelvis = indexOf(ragdoll, "pelvis");
    levelFoot(ragdoll, chain, stanceAnkle);
    levelFoot(ragdoll, chain, swingAnkle);
    float mass = 0.f;
    Vec3 com{};
    for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
        mass += ragdoll.bodies[i].mass;
        com = com + chain.position[i] * ragdoll.bodies[i].mass;
    }
    if (mass > 0.f) {
        com = com * (1.f / mass);
    }
    Vec3 ankle = chain.position[stanceFoot];
    for (const auto& joint : ragdoll.joints) {
        if (joint.id == stanceAnkle && joint.child == stanceFoot) {
            ankle = chain.position[stanceFoot] + rotate(chain.rotation[stanceFoot], joint.anchorInChild);
        }
    }
    const auto stanceCorners = soleCorners(ragdoll, chain, stanceFoot);
    const auto swingCorners = soleCorners(ragdoll, chain, swingFoot);
    const float stanceLow = lowestY(stanceCorners);
    const float swingLow = lowestY(swingCorners);
    const float clearance = swingLow - stanceLow;
    const Vec3 projected{com.x, stanceLow, com.z};
    const float margin = supportMargin(stanceCorners, projected);
    const float arm = length(Vec3{com.x - ankle.x, 0.f, com.z - ankle.z});
    Vec3 hipAnchor = ankle;
    for (const auto& joint : ragdoll.joints) {
        if ((supportRight && joint.id == "hip_1") || (!supportRight && joint.id == "hip_2")) {
            hipAnchor = chain.position[joint.parent] + rotate(chain.rotation[joint.parent], joint.anchorInParent);
        }
    }
    const float hipArm = length(Vec3{hipAnchor.x - ankle.x, 0.f, hipAnchor.z - ankle.z});
    const Vec3 cop{com.x, stanceLow, com.z};
    const Moments moments = jointMoments(ragdoll, chain, stanceFoot, cop);
    const float torso = chest >= 0 ? std::acos(clampf(bodyUp(ragdoll.bodies[chest], chain.rotation[chest]).y, -1.f, 1.f)) : 1.f;
    const float pelvisTilt = pelvis >= 0 ? std::acos(clampf(bodyUp(ragdoll.bodies[pelvis], chain.rotation[pelvis]).y, -1.f, 1.f)) : 1.f;
    const float sole = soleUpY(ragdoll, chain, stanceFoot);
    const bool flat = sole > 0.98f;
    const bool inside = margin > 0.004f;
    const bool clear = clearance > 0.008f;
    const bool ankleOk = moments.ankle <= 12.f;
    const bool torquesOk = moments.worstRatio <= 1.f;
    const bool upright = torso < 0.45f;
    SingleLegCandidate result;
    result.searched = true;
    result.supportRight = supportRight;
    result.totalMass = mass;
    result.comToAnkleM = arm;
    result.hipToAnkleM = hipArm;
    result.soleMarginM = margin;
    result.swingClearanceM = clearance;
    result.torsoTilt = torso;
    result.pelvisTilt = pelvisTilt;
    result.ankleTorque = moments.ankle;
    result.ankleMomentMagnitude = moments.ankleMagnitude;
    result.gravityMoment = mass * 9.81f * arm;
    result.kneeTorque = moments.knee;
    result.hipTorque = moments.hip;
    result.hipRootTorque = moments.hipRoot;
    result.spineTorque = moments.spine;
    result.limitingJoint = moments.worstJoint;
    result.limitingTorque = moments.worstTorque;
    result.limitingLimit = moments.worstLimit;
    result.com = com;
    result.ankle = ankle;
    auto eulerOf = [&](const std::string& id) {
        const auto found = chain.joint.find(id);
        return found == chain.joint.end() ? Vec3{} : constraintEulerRadians(found->second);
    };
    const Vec3 stanceHipEuler = eulerOf(supportRight ? "hip_1" : "hip_2");
    const Vec3 swingHipEuler = eulerOf(supportRight ? "hip_2" : "hip_1");
    result.stanceHipSwing1Deg = degrees(stanceHipEuler.y);
    result.stanceHipSwing2Deg = degrees(stanceHipEuler.z);
    result.stanceKneeDeg = degrees(eulerOf(supportRight ? "knee_1" : "knee_2").z);
    result.swingHipSwing1Deg = degrees(swingHipEuler.y);
    result.swingHipSwing2Deg = degrees(swingHipEuler.z);
    result.swingKneeDeg = degrees(eulerOf(supportRight ? "knee_2" : "knee_1").z);
    result.spineSwing1Deg = degrees(eulerOf("spine_stomach").y);
    float handOffset = 0.f;
    int hands = 0;
    for (const char* handId : {"hand_1", "hand_2"}) {
        const int hand = indexOf(ragdoll, handId);
        if (hand < 0 || hand >= int(neutral.position.size())) {
            continue;
        }
        handOffset += length(chain.position[hand] - neutral.position[hand]);
        ++hands;
    }
    if (hands > 0) {
        handOffset /= float(hands);
    }
    result.handOffsetM = handOffset;
    result.postureCost = postureCost(result, handOffset);
    result.feasible = inside && clear && flat && ankleOk && torquesOk && upright && arm < 0.02f;
    if (!inside) {
        result.rejection = "COM projection is outside the stance sole";
    } else if (!flat) {
        result.rejection = "stance sole cannot be leveled inside the ankle limits";
    } else if (!clear) {
        result.rejection = "swing foot is not clear of the floor";
    } else if (!ankleOk) {
        result.rejection = "static ankle moment exceeds 12 N·m";
    } else if (!torquesOk) {
        result.rejection = "static moment exceeds the motor limit at " + moments.worstJoint;
    } else if (!upright) {
        result.rejection = "torso tilt is above 0.45 rad";
    } else if (!(arm < 0.02f)) {
        result.rejection = "COM is more than 2 cm from the stance ankle";
    }
    // Feasible poses are ranked only by how close they sit to a relaxed
    // standing shape. Torque is a gate, not something a folded leg can buy
    // its way past. Infeasible poses are remembered for the rejection text
    // and are not a placement candidate unless the caller asks.
    if (result.feasible && result.postureCost < calm.score) {
        calm.score = result.postureCost;
        calm.chain = chain;
        calm.result = result;
    }
    const float mechanical = moments.worstRatio + result.postureCost * 0.15f + (arm > 0.02f ? arm : 0.f) + (upright ? 0.f : torso);
    if (!result.feasible && mechanical < nearest.score) {
        nearest.score = mechanical;
        nearest.chain = chain;
        nearest.result = result;
    }
    // The geometric score only chooses where the local search looks. It is
    // not the pose that gets written.
    const float geometric = arm + hipArm + (inside ? 0.f : 0.25f) + (clear ? 0.f : 0.15f) + (flat ? 0.f : 0.2f) +
                            std::max(0.f, moments.worstRatio - 1.f) * 0.15f + std::max(0.f, torso - 0.45f);
    if (geometric < best.score) {
        best.score = geometric;
        best.chain = chain;
        best.result = result;
    }
    // The probe is the old hip-root minimum. It is searched so a folded leg
    // that is actually inside the limits can still be found, then rejected
    // by posture cost if a calmer feasible pose exists.
    const bool probeRegion = moments.ankle <= 12.f && moments.knee <= 35.f && moments.hip <= 50.f && moments.spine <= 55.f && arm < 0.016f &&
                             clear && flat && torso < 0.55f;
    const float probeScore = (probeRegion ? 0.f : 100.f) + moments.hipRoot + (probeRegion ? 0.f : arm);
    if (probeScore < probe.score) {
        probe.score = probeScore;
        probe.chain = std::move(chain);
        probe.result = result;
    }
}

Chain baseChain(const Ragdoll& ragdoll, const BalanceConfig& config) {
    Chain chain;
    chain.position.resize(ragdoll.bodies.size());
    chain.rotation.resize(ragdoll.bodies.size());
    const int pelvis = indexOf(ragdoll, "pelvis");
    if (pelvis >= 0) {
        chain.rotation[pelvis] = uprightPelvis(ragdoll.bodies[pelvis]);
        chain.position[pelvis] = ragdoll.bodies[pelvis].initialPosition;
    }
    for (const auto& joint : ragdoll.joints) {
        chain.joint[joint.id] = standingReference(joint, config);
    }
    const int chest = indexOf(ragdoll, "chest");
    float bestSpine = 1e9f;
    float bestStomach = 0.f;
    float bestChest = 0.f;
    if (chest >= 0) {
        const RagdollBody& chestBody = ragdoll.bodies[chest];
        for (float stomach = -18.f; stomach <= 18.1f; stomach += 3.f) {
            for (float chestDeg = -18.f; chestDeg <= 18.1f; chestDeg += 3.f) {
                chain.joint["spine_stomach"] = quatFromEulerXYZ(0.f, 0.f, radians(stomach));
                chain.joint["spine_chest"] = quatFromEulerXYZ(0.f, 0.f, radians(chestDeg));
                rebuild(ragdoll, chain);
                const Vec3 up = bodyUp(chestBody, chain.rotation[chest]);
                const float score = std::fabs(up.x) + std::fabs(up.z) + (up.y < 0.75f ? 2.f : 0.f);
                if (score < bestSpine) {
                    bestSpine = score;
                    bestStomach = stomach;
                    bestChest = chestDeg;
                }
            }
        }
        chain.joint["spine_stomach"] = quatFromEulerXYZ(0.f, 0.f, radians(bestStomach));
        chain.joint["spine_chest"] = quatFromEulerXYZ(0.f, 0.f, radians(bestChest));
        rebuild(ragdoll, chain);
    }
    return chain;
}

}  // namespace

SingleLegCandidate placeSingleLegCandidate(PhysicsWorld& world, Ragdoll& ragdoll, const BalanceConfig& config, bool supportRight, bool placeRejected) {
    const std::string stanceHip = supportRight ? "hip_1" : "hip_2";
    const std::string swingHip = supportRight ? "hip_2" : "hip_1";
    const std::string stanceKnee = supportRight ? "knee_1" : "knee_2";
    const std::string swingKnee = supportRight ? "knee_2" : "knee_1";
    Scored calm;
    Scored nearest;
    Scored best;
    Scored probe;
    probe.score = 1e9f;
    const Chain start = baseChain(ragdoll, config);
    const float hip1[] = {-42.f, -28.f, -14.f, 0.f, 14.f, 28.f, 42.f};
    const float hip2[] = {-12.f, 0.f, 12.f, 24.f, 36.f};
    const float knee[] = {6.f, 16.f};
    const float swing2[] = {40.f, 70.f, 100.f};
    const float swingKneeA[] = {60.f, 100.f};
    const float swing1[] = {-20.f, 10.f};
    const float spine1[] = {-18.f, -9.f, 0.f, 9.f, 18.f};
    const Vec3 stomachPitch = constraintEulerRadians(start.joint.at("spine_stomach"));
    const Vec3 chestPitch = constraintEulerRadians(start.joint.at("spine_chest"));
    // Lateral spine flexion is added on top of the pitch that stands the chest
    // up. A pure upright chest leaves the COM several centimetres off the hip.
    for (float stanceSwing1 : hip1) {
        for (float stanceSwing2 : hip2) {
            for (float stanceKneeDeg : knee) {
                for (float swingSwing2 : swing2) {
                    for (float swingKneeDeg : swingKneeA) {
                        for (float swingSwing1 : swing1) {
                            for (float lateral : spine1) {
                                Chain chain = start;
                                chain.joint[stanceHip] = quatFromEulerXYZ(0.f, radians(stanceSwing1), radians(stanceSwing2));
                                chain.joint[swingHip] = quatFromEulerXYZ(0.f, radians(swingSwing1), radians(swingSwing2));
                                chain.joint[stanceKnee] = quatFromEulerXYZ(0.f, 0.f, radians(stanceKneeDeg));
                                chain.joint[swingKnee] = quatFromEulerXYZ(0.f, 0.f, radians(swingKneeDeg));
                                chain.joint["spine_stomach"] = quatFromEulerXYZ(stomachPitch.x, radians(lateral), stomachPitch.z);
                                chain.joint["spine_chest"] = quatFromEulerXYZ(chestPitch.x, radians(lateral * 0.7f), chestPitch.z);
                                rebuild(ragdoll, chain);
                                consider(ragdoll, start, chain, supportRight, calm, nearest, best, probe);
                            }
                        }
                    }
                }
            }
        }
    }
    const SingleLegCandidate seed = best.result.searched ? best.result : nearest.result;
    for (float ds1 = -8.f; ds1 <= 8.1f; ds1 += 4.f) {
        for (float ds2 = -8.f; ds2 <= 8.1f; ds2 += 4.f) {
            for (float dk = -6.f; dk <= 6.1f; dk += 3.f) {
                for (float dSpine = -8.f; dSpine <= 8.1f; dSpine += 4.f) {
                    Chain chain = start;
                    chain.joint[stanceHip] = quatFromEulerXYZ(0.f, radians(seed.stanceHipSwing1Deg + ds1), radians(seed.stanceHipSwing2Deg + ds2));
                    chain.joint[swingHip] = quatFromEulerXYZ(0.f, radians(seed.swingHipSwing1Deg), radians(seed.swingHipSwing2Deg));
                    chain.joint[stanceKnee] = quatFromEulerXYZ(0.f, 0.f, radians(seed.stanceKneeDeg + dk));
                    chain.joint[swingKnee] = quatFromEulerXYZ(0.f, 0.f, radians(seed.swingKneeDeg));
                    chain.joint["spine_stomach"] = quatFromEulerXYZ(stomachPitch.x, radians(seed.spineSwing1Deg + dSpine), stomachPitch.z);
                    chain.joint["spine_chest"] = quatFromEulerXYZ(chestPitch.x, radians((seed.spineSwing1Deg + dSpine) * 0.7f), chestPitch.z);
                    rebuild(ragdoll, chain);
                    consider(ragdoll, start, chain, supportRight, calm, nearest, best, probe);
                }
            }
        }
    }
    const SingleLegCandidate held = probe.score < 100.f ? probe.result : best.result;
    for (float swing2Deg = 15.f; swing2Deg <= 110.1f; swing2Deg += 10.f) {
        for (float swingKneeDeg = 20.f; swingKneeDeg <= 140.1f; swingKneeDeg += 15.f) {
            for (float swing1Deg = -40.f; swing1Deg <= 40.1f; swing1Deg += 10.f) {
                Chain chain = start;
                chain.joint[stanceHip] = quatFromEulerXYZ(0.f, radians(held.stanceHipSwing1Deg), radians(held.stanceHipSwing2Deg));
                chain.joint[swingHip] = quatFromEulerXYZ(0.f, radians(swing1Deg), radians(swing2Deg));
                chain.joint[stanceKnee] = quatFromEulerXYZ(0.f, 0.f, radians(held.stanceKneeDeg));
                chain.joint[swingKnee] = quatFromEulerXYZ(0.f, 0.f, radians(swingKneeDeg));
                chain.joint["spine_stomach"] = quatFromEulerXYZ(stomachPitch.x, radians(held.spineSwing1Deg), stomachPitch.z);
                chain.joint["spine_chest"] = quatFromEulerXYZ(chestPitch.x, radians(held.spineSwing1Deg * 0.7f), chestPitch.z);
                rebuild(ragdoll, chain);
                consider(ragdoll, start, chain, supportRight, calm, nearest, best, probe);
            }
        }
    }
    const bool haveFeasible = calm.result.feasible;
    const Scored& chosen = haveFeasible ? calm : nearest;
    SingleLegCandidate result = chosen.result;
    if (nearest.result.searched) {
        result.lightest = "nearest rejected torso " + std::to_string(nearest.result.torsoTilt) + " swingKnee " +
                          std::to_string(nearest.result.swingKneeDeg) + " hipRoot " + std::to_string(nearest.result.hipRootTorque) + " " +
                          nearest.result.rejection;
    }
    if (!result.searched) {
        result.rejection = "the pose search did not run";
        return result;
    }
    if (!haveFeasible && !placeRejected) {
        if (result.rejection.empty()) {
            result.rejection = "no feasible single-leg pose";
        }
        result.rejection += ". Not placed.";
        return result;
    }
    result.diagnostic = !haveFeasible;
    if (result.diagnostic) {
        result.rejection = "diagnostic only, not a standing pose. " + result.rejection;
    }
    Chain placed = chosen.chain;
    const int stanceFoot = indexOf(ragdoll, supportRight ? "foot_1" : "foot_2");
    const float lift = 0.004f - lowestY(soleCorners(ragdoll, placed, stanceFoot));
    for (int i = 0; i < int(ragdoll.bodies.size()); ++i) {
        placed.position[i].y += lift;
        world.setBodyPose(ragdoll.bodies[i].physics, placed.position[i], placed.rotation[i], true);
        world.setBodyVelocity(ragdoll.bodies[i].physics, {}, {});
    }
    result.com.y += lift;
    result.ankle.y += lift;
    StandingPlacement placement;
    placement.ok = true;
    for (const auto& joint : ragdoll.joints) {
        const Quat measured = world.jointState(joint.physics).constraintRotation;
        MotorTaskCommand command;
        command.jointId = joint.id;
        command.target = finite(measured) ? normalize(measured) : placed.joint[joint.id];
        command.activation = 1.f;
        placement.reference.push_back(command);
        placement.maxSeparation = std::max(placement.maxSeparation, world.jointState(joint.physics).separation);
    }
    result.maxSeparation = placement.maxSeparation;
    result.reference = std::move(placement.reference);
    result.placed = true;
    float referenceError = 0.f;
    for (const auto& joint : ragdoll.joints) {
        const auto intended = placed.joint.find(joint.id);
        if (intended == placed.joint.end()) {
            continue;
        }
        const Quat measured = world.jointState(joint.physics).constraintRotation;
        if (!finite(measured)) {
            continue;
        }
        referenceError = std::max(referenceError, quatAngle(mul(conjugate(intended->second), measured)));
    }
    result.referenceError = referenceError;
    if (result.feasible && !result.diagnostic) {
        result.rejection = "static single-leg pose is inside the sole and the torque limits";
    }
    if (placement.maxSeparation > 0.02f) {
        result.feasible = false;
        result.placed = true;
        result.rejection = "posed chain separation exceeds 2 cm";
    }
    if (referenceError > 0.25f) {
        result.feasible = false;
        result.rejection = "measured joint angle does not match the written pose";
    }
    return result;
}

}  // namespace mt
