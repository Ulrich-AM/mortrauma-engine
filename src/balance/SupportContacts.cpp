#include "mortrauma/balance/SupportContacts.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace mt {
namespace {

constexpr float kSupportNormalY = 0.55f;

struct GroundPoint {
    float x = 0.f;
    float z = 0.f;
    Vec3 world{};
};

float cross2(GroundPoint a, GroundPoint b, GroundPoint c) {
    return (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
}

float dist2(float x, float z, GroundPoint a, GroundPoint b) {
    const float vx = b.x - a.x;
    const float vz = b.z - a.z;
    const float len2 = vx * vx + vz * vz;
    float t = 0.f;
    if (len2 > 1e-10f) {
        t = clampf(((x - a.x) * vx + (z - a.z) * vz) / len2, 0.f, 1.f);
    }
    const float dx = x - (a.x + vx * t);
    const float dz = z - (a.z + vz * t);
    return std::sqrt(dx * dx + dz * dz);
}

bool insideHull(const std::vector<GroundPoint>& hull, float x, float z) {
    if (hull.size() < 3) {
        return false;
    }
    float sign = 0.f;
    for (size_t i = 0; i < hull.size(); ++i) {
        const GroundPoint& a = hull[i];
        const GroundPoint& b = hull[(i + 1) % hull.size()];
        const float c = (b.x - a.x) * (z - a.z) - (b.z - a.z) * (x - a.x);
        if (std::fabs(c) <= 1e-6f) {
            continue;
        }
        if (sign == 0.f) {
            sign = c > 0.f ? 1.f : -1.f;
        } else if ((c > 0.f) != (sign > 0.f)) {
            return false;
        }
    }
    return true;
}

std::vector<GroundPoint> convexHull(std::vector<GroundPoint> points) {
    if (points.size() < 2) {
        return points;
    }
    std::sort(points.begin(), points.end(), [](const GroundPoint& a, const GroundPoint& b) {
        return a.x < b.x || (a.x == b.x && a.z < b.z);
    });
    points.erase(std::unique(points.begin(), points.end(),
                             [](const GroundPoint& a, const GroundPoint& b) {
                                 return std::fabs(a.x - b.x) < 1e-5f && std::fabs(a.z - b.z) < 1e-5f;
                             }),
                 points.end());
    if (points.size() < 2) {
        return points;
    }
    std::vector<GroundPoint> lower;
    std::vector<GroundPoint> upper;
    for (const auto& point : points) {
        while (lower.size() >= 2 && cross2(lower[lower.size() - 2], lower.back(), point) <= 1e-8f) {
            lower.pop_back();
        }
        lower.push_back(point);
    }
    for (int i = int(points.size()) - 1; i >= 0; --i) {
        while (upper.size() >= 2 && cross2(upper[upper.size() - 2], upper.back(), points[i]) <= 1e-8f) {
            upper.pop_back();
        }
        upper.push_back(points[i]);
    }
    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
}

float polygonArea(const std::vector<GroundPoint>& hull) {
    if (hull.size() < 3) {
        return 0.f;
    }
    double area = 0.0;
    for (size_t i = 0; i < hull.size(); ++i) {
        const GroundPoint& a = hull[i];
        const GroundPoint& b = hull[(i + 1) % hull.size()];
        area += double(a.x) * b.z - double(b.x) * a.z;
    }
    return float(std::fabs(area) * 0.5);
}

FootRegion classifySole(const RagdollBody& foot, Vec3 localPoint) {
    const Vec3 axis = foot.forefootLocal - foot.heelLocal;
    const float span = dot(axis, axis);
    if (!(span > 1e-8f)) {
        return FootRegion::None;
    }
    const float u = dot(localPoint - foot.heelLocal, axis) / span;
    if (u < 0.33f) {
        return FootRegion::Heel;
    }
    if (u < 0.66f) {
        return FootRegion::Midfoot;
    }
    return FootRegion::Forefoot;
}

float featureDistance(const std::vector<GroundPoint>& hull, float x, float z) {
    if (hull.empty()) {
        return 0.f;
    }
    if (hull.size() == 1) {
        const float dx = x - hull[0].x;
        const float dz = z - hull[0].z;
        return std::sqrt(dx * dx + dz * dz);
    }
    float best = 1e9f;
    for (size_t i = 0; i < hull.size(); ++i) {
        const size_t next = hull.size() == 2 ? 1 : (i + 1) % hull.size();
        if (hull.size() == 2 && i > 0) {
            break;
        }
        best = std::min(best, dist2(x, z, hull[i], hull[next]));
    }
    return best;
}

}  // namespace

float supportMargin(const std::vector<Vec3>& hull, Vec3 groundPoint) {
    std::vector<GroundPoint> points;
    points.reserve(hull.size());
    for (const Vec3& point : hull) {
        points.push_back({point.x, point.z, point});
    }
    const float outside = featureDistance(points, groundPoint.x, groundPoint.z);
    if (points.size() >= 3 && insideHull(points, groundPoint.x, groundPoint.z)) {
        return outside;
    }
    if (outside < 1e-4f) {
        return 0.f;
    }
    return -outside;
}

float distanceOutsideSupport(const std::vector<Vec3>& hull, Vec3 groundPoint) {
    const float margin = supportMargin(hull, groundPoint);
    return margin >= 0.f ? 0.f : -margin;
}

SupportEstimate estimateSupport(const PhysicsWorld& world, const Ragdoll& ragdoll) {
    SupportEstimate estimate;
    const RagdollBody* rightFoot = ragdoll.find("foot_1");
    const RagdollBody* leftFoot = ragdoll.find("foot_2");
    std::unordered_set<int> character;
    for (const auto& body : ragdoll.bodies) {
        character.insert(body.physics);
    }
    auto accumulate = [&](const RagdollBody* foot, bool& grounded, bool& touching, int& points, float& load, int& heel, int& mid,
                          int& fore, float& area) {
        if (foot == nullptr) {
            return;
        }
        const BodyState footState = world.bodyState(foot->physics);
        std::vector<GroundPoint> footRaw;
        int supportCount = 0;
        for (const auto& contact : world.contacts()) {
            const bool onFoot = contact.bodyA == foot->physics || contact.bodyB == foot->physics;
            if (!onFoot) {
                continue;
            }
            const int other = contact.bodyA == foot->physics ? contact.bodyB : contact.bodyA;
            if (other >= 0 && character.count(other) != 0) {
                continue;
            }
            // Jolt's normal is the direction that moves body 2 out of body 1.
            // The ground's push on the foot is the opposite of that when the foot is body 1.
            Vec3 normal = contact.normal;
            if (contact.bodyA == foot->physics) {
                normal = -normal;
            }
            touching = true;
            BalanceContact draw;
            draw.point = contact.point;
            draw.normal = normal;
            draw.supporting = normal.y >= kSupportNormalY && footState.valid;
            if (!draw.supporting) {
                estimate.contacts.push_back(draw);
                continue;
            }
            const Vec3 local = rotate(conjugate(footState.rotation), contact.point - footState.position);
            draw.region = classifySole(*foot, local);
            if (draw.region == FootRegion::Heel) {
                ++heel;
            } else if (draw.region == FootRegion::Midfoot) {
                ++mid;
            } else if (draw.region == FootRegion::Forefoot) {
                ++fore;
            }
            grounded = true;
            ++supportCount;
            const Vec3 flat{contact.point.x, 0.f, contact.point.z};
            estimate.supportPoints.push_back(flat);
            footRaw.push_back({flat.x, flat.z, flat});
            estimate.contacts.push_back(draw);
        }
        points = supportCount;
        load = float(supportCount);
        area = polygonArea(convexHull(footRaw));
    };
    accumulate(rightFoot, estimate.rightGrounded, estimate.rightTouching, estimate.rightPoints, estimate.rightLoadProxy, estimate.rightHeel,
               estimate.rightMid, estimate.rightFore, estimate.rightArea);
    accumulate(leftFoot, estimate.leftGrounded, estimate.leftTouching, estimate.leftPoints, estimate.leftLoadProxy, estimate.leftHeel,
               estimate.leftMid, estimate.leftFore, estimate.leftArea);

    std::vector<GroundPoint> raw;
    for (const Vec3& point : estimate.supportPoints) {
        raw.push_back({point.x, point.z, point});
    }
    const std::vector<GroundPoint> hull = convexHull(raw);
    Vec3 center{};
    for (const auto& point : hull) {
        estimate.hull.push_back(point.world);
        center = center + point.world;
    }
    if (!hull.empty()) {
        center = center * (1.f / float(hull.size()));
    }
    estimate.center = center;
    estimate.margin = 0.f;
    estimate.area = polygonArea(hull);
    estimate.reliable = hull.size() >= 3 && estimate.area > 1e-4f;
    return estimate;
}

}  // namespace mt
