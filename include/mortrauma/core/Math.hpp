#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace mt {

struct Vec3 {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;

    Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(Vec3 o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(Vec3 o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
};

inline Vec3 operator*(float s, Vec3 v) { return v * s; }

inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline float lengthSq(Vec3 a) { return dot(a, a); }
inline float length(Vec3 a) { return std::sqrt(lengthSq(a)); }

inline Vec3 normalize(Vec3 a) {
    const float len = length(a);
    if (len < 1e-8f) {
        return {0.f, 1.f, 0.f};
    }
    return a * (1.f / len);
}

inline bool finite(Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

struct Quat {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float w = 1.f;

    static Quat identity() { return {}; }
};

inline bool finite(Quat q) {
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

inline float length(Quat q) { return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w); }

inline Quat normalize(Quat q) {
    const float len = length(q);
    if (len < 1e-8f) {
        return {};
    }
    return {q.x / len, q.y / len, q.z / len, q.w / len};
}

inline Quat conjugate(Quat q) { return {-q.x, -q.y, -q.z, q.w}; }

inline Quat mul(Quat a, Quat b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

inline Vec3 rotate(Quat q, Vec3 v) {
    const Quat r = mul(mul(q, Quat{v.x, v.y, v.z, 0.f}), conjugate(q));
    return {r.x, r.y, r.z};
}

inline Quat quatAxisAngle(Vec3 axis, float radians) {
    axis = normalize(axis);
    const float s = std::sin(radians * 0.5f);
    return normalize(Quat{axis.x * s, axis.y * s, axis.z * s, std::cos(radians * 0.5f)});
}

inline Quat quatFromTo(Vec3 from, Vec3 to) {
    from = normalize(from);
    to = normalize(to);
    const float d = dot(from, to);
    if (d > 0.9999f) {
        return {};
    }
    if (d < -0.9999f) {
        Vec3 ortho = std::fabs(from.y) < 0.9f ? cross(from, Vec3{0.f, 1.f, 0.f}) : cross(from, Vec3{1.f, 0.f, 0.f});
        return quatAxisAngle(ortho, 3.14159265f);
    }
    const Vec3 c = cross(from, to);
    return normalize(Quat{c.x, c.y, c.z, 1.f + d});
}

// Column-major 4x4, column vectors.
struct Mat4 {
    float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

    static Mat4 identity() { return {}; }

    static Mat4 translation(Vec3 t) {
        Mat4 r;
        r.m[12] = t.x;
        r.m[13] = t.y;
        r.m[14] = t.z;
        return r;
    }

    static Mat4 fromRotationTranslation(Quat q, Vec3 t) {
        q = normalize(q);
        const float x = q.x;
        const float y = q.y;
        const float z = q.z;
        const float w = q.w;
        const float x2 = x + x;
        const float y2 = y + y;
        const float z2 = z + z;
        const float xx = x * x2;
        const float xy = x * y2;
        const float xz = x * z2;
        const float yy = y * y2;
        const float yz = y * z2;
        const float zz = z * z2;
        const float wx = w * x2;
        const float wy = w * y2;
        const float wz = w * z2;
        Mat4 r;
        r.m[0] = 1.f - (yy + zz);
        r.m[1] = xy + wz;
        r.m[2] = xz - wy;
        r.m[3] = 0.f;
        r.m[4] = xy - wz;
        r.m[5] = 1.f - (xx + zz);
        r.m[6] = yz + wx;
        r.m[7] = 0.f;
        r.m[8] = xz + wy;
        r.m[9] = yz - wx;
        r.m[10] = 1.f - (xx + yy);
        r.m[11] = 0.f;
        r.m[12] = t.x;
        r.m[13] = t.y;
        r.m[14] = t.z;
        r.m[15] = 1.f;
        return r;
    }

    Vec3 translation() const { return {m[12], m[13], m[14]}; }

    Quat rotation() const {
        Quat q;
        const float trace = m[0] + m[5] + m[10];
        if (trace > 0.f) {
            const float s = std::sqrt(trace + 1.f) * 2.f;
            q.w = 0.25f * s;
            q.x = (m[6] - m[9]) / s;
            q.y = (m[8] - m[2]) / s;
            q.z = (m[1] - m[4]) / s;
        } else if (m[0] > m[5] && m[0] > m[10]) {
            const float s = std::sqrt(1.f + m[0] - m[5] - m[10]) * 2.f;
            q.w = (m[6] - m[9]) / s;
            q.x = 0.25f * s;
            q.y = (m[1] + m[4]) / s;
            q.z = (m[8] + m[2]) / s;
        } else if (m[5] > m[10]) {
            const float s = std::sqrt(1.f + m[5] - m[0] - m[10]) * 2.f;
            q.w = (m[8] - m[2]) / s;
            q.x = (m[1] + m[4]) / s;
            q.y = 0.25f * s;
            q.z = (m[6] + m[9]) / s;
        } else {
            const float s = std::sqrt(1.f + m[10] - m[0] - m[5]) * 2.f;
            q.w = (m[1] - m[4]) / s;
            q.x = (m[8] + m[2]) / s;
            q.y = (m[6] + m[9]) / s;
            q.z = 0.25f * s;
        }
        return normalize(q);
    }
};

inline Mat4 mul(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int c = 0; c < 4; ++c) {
        for (int row = 0; row < 4; ++row) {
            r.m[c * 4 + row] = a.m[0 * 4 + row] * b.m[c * 4 + 0] + a.m[1 * 4 + row] * b.m[c * 4 + 1] +
                               a.m[2 * 4 + row] * b.m[c * 4 + 2] + a.m[3 * 4 + row] * b.m[c * 4 + 3];
        }
    }
    return r;
}

inline Vec3 transformPoint(const Mat4& a, Vec3 v) {
    const float x = a.m[0] * v.x + a.m[4] * v.y + a.m[8] * v.z + a.m[12];
    const float y = a.m[1] * v.x + a.m[5] * v.y + a.m[9] * v.z + a.m[13];
    const float z = a.m[2] * v.x + a.m[6] * v.y + a.m[10] * v.z + a.m[14];
    const float w = a.m[3] * v.x + a.m[7] * v.y + a.m[11] * v.z + a.m[15];
    if (std::fabs(w) > 1e-8f && std::fabs(w - 1.f) > 1e-5f) {
        return {x / w, y / w, z / w};
    }
    return {x, y, z};
}

inline Vec3 transformVector(const Mat4& a, Vec3 v) {
    return {a.m[0] * v.x + a.m[4] * v.y + a.m[8] * v.z, a.m[1] * v.x + a.m[5] * v.y + a.m[9] * v.z,
            a.m[2] * v.x + a.m[6] * v.y + a.m[10] * v.z};
}

inline Mat4 inverseRigid(const Mat4& a) {
    const Quat q = conjugate(a.rotation());
    const Vec3 t = a.translation();
    return Mat4::fromRotationTranslation(q, rotate(q, -t));
}

inline Mat4 perspective(float fovYRadians, float aspect, float nearZ, float farZ) {
    const float f = 1.f / std::tan(fovYRadians * 0.5f);
    Mat4 r;
    for (float& value : r.m) {
        value = 0.f;
    }
    r.m[0] = f / aspect;
    r.m[5] = f;
    r.m[10] = (farZ + nearZ) / (nearZ - farZ);
    r.m[11] = -1.f;
    r.m[14] = (2.f * farZ * nearZ) / (nearZ - farZ);
    return r;
}

inline Mat4 lookAt(Vec3 eye, Vec3 target, Vec3 up) {
    const Vec3 f = normalize(target - eye);
    const Vec3 s = normalize(cross(f, up));
    const Vec3 u = cross(s, f);
    Mat4 r;
    r.m[0] = s.x;
    r.m[1] = u.x;
    r.m[2] = -f.x;
    r.m[3] = 0.f;
    r.m[4] = s.y;
    r.m[5] = u.y;
    r.m[6] = -f.y;
    r.m[7] = 0.f;
    r.m[8] = s.z;
    r.m[9] = u.z;
    r.m[10] = -f.z;
    r.m[11] = 0.f;
    r.m[12] = -dot(s, eye);
    r.m[13] = -dot(u, eye);
    r.m[14] = dot(f, eye);
    r.m[15] = 1.f;
    return r;
}

constexpr float kPi = 3.14159265358979323846f;
inline float radians(float degrees) { return degrees * kPi / 180.f; }
inline float degrees(float radiansValue) { return radiansValue * 180.f / kPi; }

inline float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

inline float dot(Quat a, Quat b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }

inline Quat operator-(Quat q) { return {-q.x, -q.y, -q.z, -q.w}; }

// Shortest-arc blend. t is clamped to [0, 1].
inline Quat slerp(Quat a, Quat b, float t) {
    a = normalize(a);
    b = normalize(b);
    float cosine = dot(a, b);
    if (cosine < 0.f) {
        b = -b;
        cosine = -cosine;
    }
    t = clampf(t, 0.f, 1.f);
    if (cosine > 0.9995f) {
        return normalize(Quat{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t});
    }
    const float theta = std::acos(clampf(cosine, -1.f, 1.f));
    const float sine = std::sin(theta);
    const float w1 = std::sin((1.f - t) * theta) / sine;
    const float w2 = std::sin(t * theta) / sine;
    return normalize(Quat{a.x * w1 + b.x * w2, a.y * w1 + b.y * w2, a.z * w1 + b.z * w2, a.w * w1 + b.w * w2});
}

// Rotation angle in radians, ignoring the quaternion's double cover.
inline float quatAngle(Quat q) {
    q = normalize(q);
    return 2.f * std::acos(clampf(std::fabs(q.w), 0.f, 1.f));
}

// XYZ Euler used only to display a constraint quaternion. Motor control does
// not subtract these angles; it sends the quaternion itself.
inline Vec3 constraintEulerRadians(Quat relative) {
    const float sinX = 2.f * (relative.w * relative.x + relative.y * relative.z);
    const float cosX = 1.f - 2.f * (relative.x * relative.x + relative.y * relative.y);
    const float sinY = clampf(2.f * (relative.w * relative.y - relative.z * relative.x), -1.f, 1.f);
    const float sinZ = 2.f * (relative.w * relative.z + relative.x * relative.y);
    const float cosZ = 1.f - 2.f * (relative.y * relative.y + relative.z * relative.z);
    return {std::atan2(sinX, cosX), std::asin(sinY), std::atan2(sinZ, cosZ)};
}

inline Quat quatFromEulerXYZ(float x, float y, float z) {
    const float cx = std::cos(x * 0.5f);
    const float sx = std::sin(x * 0.5f);
    const float cy = std::cos(y * 0.5f);
    const float sy = std::sin(y * 0.5f);
    const float cz = std::cos(z * 0.5f);
    const float sz = std::sin(z * 0.5f);
    return normalize(Quat{sx * cy * cz - cx * sy * sz, cx * sy * cz + sx * cy * sz, cx * cy * sz - sx * sy * cz,
                          cx * cy * cz + sx * sy * sz});
}

}  // namespace mt
