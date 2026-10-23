#pragma once
/**
 * Core 3D Vector, Quaternion, Matrix, and Coordinate Frame Utilities.
 *
 * Coordinate Convention (Right-Handed, matching Three.js & Rapier 3D):
 *   +X : Vehicle Left  (-X is Right)
 *   +Y : Vehicle Up    (-Y is Down / Gravity direction)
 *   +Z : Vehicle Forward (-Z is Rear)
 *
 * All quantities use strict SI units:
 *   Length: meters (m)
 *   Mass: kilograms (kg)
 *   Time: seconds (s)
 *   Force: Newtons (N)
 *   Torque: Newton-meters (N*m)
 *   Angles: radians (rad), Angular Velocity: rad/s
 */

#include <algorithm>
#include <array>
#include <cmath>

namespace torquelab {

inline constexpr double PI_D = 3.14159265358979323846;
inline constexpr float PI_F = 3.14159265358979323846f;
inline constexpr double GRAVITY = 9.81; // m/s^2
inline constexpr double RPM_TO_RAD_PER_SEC = (2.0 * PI_D) / 60.0;
inline constexpr double RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * PI_D);
inline constexpr double MPS_TO_KMH = 3.6;
inline constexpr double KMH_TO_MPS = 1.0 / 3.6;

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

struct Quat {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{1.0};

    constexpr Quat() = default;
    constexpr Quat(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}
};

inline constexpr double clamp(double value, double minVal, double maxVal) {
    return value < minVal ? minVal : (value > maxVal ? maxVal : value);
}

inline constexpr double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

inline double smoothSign(double x, double eps = 1e-4) {
    if (std::abs(x) < eps) {
        return x / eps;
    }
    return x > 0.0 ? 1.0 : -1.0;
}

inline constexpr double degToRad(double deg) {
    return (deg * PI_D) / 180.0;
}

inline constexpr double radToDeg(double rad) {
    return (rad * 180.0) / PI_D;
}

inline constexpr Vec3 vec3(double x = 0.0, double y = 0.0, double z = 0.0) {
    return Vec3{x, y, z};
}

inline constexpr Vec3 vec3Add(const Vec3& a, const Vec3& b) {
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

inline constexpr Vec3 vec3Sub(const Vec3& a, const Vec3& b) {
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

inline constexpr Vec3 vec3Scale(const Vec3& v, double s) {
    return Vec3{v.x * s, v.y * s, v.z * s};
}

inline constexpr double vec3Dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline constexpr Vec3 vec3Cross(const Vec3& a, const Vec3& b) {
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

inline constexpr double vec3LengthSq(const Vec3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

inline double vec3Length(const Vec3& v) {
    return std::sqrt(vec3LengthSq(v));
}

inline Vec3 vec3Normalize(const Vec3& v) {
    const double len = vec3Length(v);
    if (len < 1e-9) {
        return Vec3{0.0, 0.0, 0.0};
    }
    return Vec3{v.x / len, v.y / len, v.z / len};
}

inline constexpr Vec3 vec3Lerp(const Vec3& a, const Vec3& b, double t) {
    return Vec3{
        lerp(a.x, b.x, t),
        lerp(a.y, b.y, t),
        lerp(a.z, b.z, t)
    };
}

inline bool isVec3Finite(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline constexpr Quat quatIdentity() {
    return Quat{0.0, 0.0, 0.0, 1.0};
}

inline constexpr Quat quatConjugate(const Quat& q) {
    return Quat{-q.x, -q.y, -q.z, q.w};
}

inline Quat quatNormalize(const Quat& q) {
    const double len = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (len < 1e-9) {
        return quatIdentity();
    }
    return Quat{q.x / len, q.y / len, q.z / len, q.w / len};
}

inline constexpr Quat quatMultiply(const Quat& a, const Quat& b) {
    return Quat{
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    };
}

inline Quat quatFromAxisAngle(const Vec3& axis, double angleRad) {
    const Vec3 normAxis = vec3Normalize(axis);
    const double half = angleRad * 0.5;
    const double s = std::sin(half);
    return Quat{
        normAxis.x * s,
        normAxis.y * s,
        normAxis.z * s,
        std::cos(half)
    };
}

inline Quat quatSlerp(const Quat& a, const Quat& b, double t) {
    double bx = b.x, by = b.y, bz = b.z, bw = b.w;
    double cosHalfTheta = a.x * bx + a.y * by + a.z * bz + a.w * bw;

    if (cosHalfTheta < 0.0) {
        bx = -bx;
        by = -by;
        bz = -bz;
        bw = -bw;
        cosHalfTheta = -cosHalfTheta;
    }

    if (cosHalfTheta >= 0.9995) {
        return quatNormalize(Quat{
            lerp(a.x, bx, t),
            lerp(a.y, by, t),
            lerp(a.z, bz, t),
            lerp(a.w, bw, t)
        });
    }

    const double halfTheta = std::acos(clamp(cosHalfTheta, -1.0, 1.0));
    const double sinHalfTheta = std::sqrt(1.0 - cosHalfTheta * cosHalfTheta);
    const double ratioA = std::sin((1.0 - t) * halfTheta) / sinHalfTheta;
    const double ratioB = std::sin(t * halfTheta) / sinHalfTheta;

    return Quat{
        a.x * ratioA + bx * ratioB,
        a.y * ratioA + by * ratioB,
        a.z * ratioA + bz * ratioB,
        a.w * ratioA + bw * ratioB
    };
}

/**
 * Rotate a 3D vector by unit quaternion q: v' = q * v * q^-1
 */
inline constexpr Vec3 quatRotateVec3(const Quat& q, const Vec3& v) {
    const double uvx = q.y * v.z - q.z * v.y;
    const double uvy = q.z * v.x - q.x * v.z;
    const double uvz = q.x * v.y - q.y * v.x;

    const double uuvx = q.y * uvz - q.z * uvy;
    const double uuvy = q.z * uvx - q.x * uvz;
    const double uuvz = q.x * uvy - q.y * uvx;

    return Vec3{
        v.x + 2.0 * (q.w * uvx + uuvx),
        v.y + 2.0 * (q.w * uvy + uuvy),
        v.z + 2.0 * (q.w * uvz + uuvz)
    };
}

inline constexpr Vec3 localPointToWorld(const Vec3& localPoint, const Vec3& bodyPos, const Quat& bodyRot) {
    return vec3Add(bodyPos, quatRotateVec3(bodyRot, localPoint));
}

inline constexpr Vec3 worldPointToLocal(const Vec3& worldPoint, const Vec3& bodyPos, const Quat& bodyRot) {
    const Vec3 diff = vec3Sub(worldPoint, bodyPos);
    return quatRotateVec3(quatConjugate(bodyRot), diff);
}

inline constexpr Vec3 localVectorToWorld(const Vec3& localVec, const Quat& bodyRot) {
    return quatRotateVec3(bodyRot, localVec);
}

inline constexpr Vec3 worldVectorToLocal(const Vec3& worldVec, const Quat& bodyRot) {
    return quatRotateVec3(quatConjugate(bodyRot), worldVec);
}

struct VehicleBasis {
    Vec3 left;    // +X local in world
    Vec3 right;   // -X local in world
    Vec3 up;      // +Y local in world
    Vec3 down;    // -Y local in world
    Vec3 forward; // +Z local in world
};

inline constexpr VehicleBasis getVehicleBasis(const Quat& bodyRot) {
    const Vec3 left = quatRotateVec3(bodyRot, Vec3{1.0, 0.0, 0.0});
    const Vec3 up = quatRotateVec3(bodyRot, Vec3{0.0, 1.0, 0.0});
    const Vec3 forward = quatRotateVec3(bodyRot, Vec3{0.0, 0.0, 1.0});
    return VehicleBasis{
        left,
        vec3Scale(left, -1.0),
        up,
        vec3Scale(up, -1.0),
        forward
    };
}

/**
 * Extracts vehicle pitch, roll, and yaw angles (in radians) from its orientation.
 * - Pitch: positive when nose pitches UP (rear squat), negative when nose dips DOWN (brake dive).
 * - Roll: positive when body rolls RIGHT (during a left-hand turn), negative when rolling LEFT.
 * - Yaw: heading angle in horizontal XZ plane (0 along +Z, positive toward +X / Left).
 */
struct VehicleEulerAngles {
    double pitch{0.0};
    double roll{0.0};
    double yaw{0.0};
};

inline VehicleEulerAngles getVehicleEulerAngles(const Quat& bodyRot) {
    const VehicleBasis basis = getVehicleBasis(bodyRot);
    const double pitch = std::atan2(basis.forward.y, std::hypot(basis.forward.x, basis.forward.z));
    const double roll = std::atan2(-basis.right.y, basis.up.y);
    const double yaw = std::atan2(basis.forward.x, basis.forward.z);
    return VehicleEulerAngles{pitch, roll, yaw};
}

/**
 * Column-major 4x4 float matrix for OpenGL 3.3 Core uniforms.
 */
struct Mat4 {
    std::array<float, 16> m{
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };

    const float* data() const { return m.data(); }

    static Mat4 identity() {
        return Mat4{};
    }

    static Mat4 multiply(const Mat4& a, const Mat4& b) {
        Mat4 out;
        for (int c = 0; c < 4; ++c) {
            for (int r = 0; r < 4; ++r) {
                out.m[c * 4 + r] =
                    a.m[0 * 4 + r] * b.m[c * 4 + 0] +
                    a.m[1 * 4 + r] * b.m[c * 4 + 1] +
                    a.m[2 * 4 + r] * b.m[c * 4 + 2] +
                    a.m[3 * 4 + r] * b.m[c * 4 + 3];
            }
        }
        return out;
    }

    static Mat4 fromTRS(const Vec3& pos, const Quat& rot, const Vec3& scale = Vec3{1.0, 1.0, 1.0}) {
        const float x = static_cast<float>(rot.x);
        const float y = static_cast<float>(rot.y);
        const float z = static_cast<float>(rot.z);
        const float w = static_cast<float>(rot.w);

        const float x2 = x + x, y2 = y + y, z2 = z + z;
        const float xx = x * x2, xy = x * y2, xz = x * z2;
        const float yy = y * y2, yz = y * z2, zz = z * z2;
        const float wx = w * x2, wy = w * y2, wz = w * z2;

        const float sx = static_cast<float>(scale.x);
        const float sy = static_cast<float>(scale.y);
        const float sz = static_cast<float>(scale.z);

        Mat4 out;
        out.m[0] = (1.0f - (yy + zz)) * sx;
        out.m[1] = (xy + wz) * sx;
        out.m[2] = (xz - wy) * sx;
        out.m[3] = 0.0f;

        out.m[4] = (xy - wz) * sy;
        out.m[5] = (1.0f - (xx + zz)) * sy;
        out.m[6] = (yz + wx) * sy;
        out.m[7] = 0.0f;

        out.m[8] = (xz + wy) * sz;
        out.m[9] = (yz - wx) * sz;
        out.m[10] = (1.0f - (xx + yy)) * sz;
        out.m[11] = 0.0f;

        out.m[12] = static_cast<float>(pos.x);
        out.m[13] = static_cast<float>(pos.y);
        out.m[14] = static_cast<float>(pos.z);
        out.m[15] = 1.0f;
        return out;
    }

    static Mat4 perspective(float fovYRad, float aspect, float zNear, float zFar) {
        const float f = 1.0f / std::tan(fovYRad * 0.5f);
        Mat4 out{};
        out.m.fill(0.0f);
        out.m[0] = f / std::max(0.001f, aspect);
        out.m[5] = f;
        out.m[10] = (zFar + zNear) / (zNear - zFar);
        out.m[11] = -1.0f;
        out.m[14] = (2.0f * zFar * zNear) / (zNear - zFar);
        return out;
    }

    static Mat4 ortho(float left, float right, float bottom, float top, float zNear, float zFar) {
        Mat4 out = identity();
        out.m[0] = 2.0f / (right - left);
        out.m[5] = 2.0f / (top - bottom);
        out.m[10] = -2.0f / (zFar - zNear);
        out.m[12] = -(right + left) / (right - left);
        out.m[13] = -(top + bottom) / (top - bottom);
        out.m[14] = -(zFar + zNear) / (zFar - zNear);
        return out;
    }

    static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
        const Vec3 f = vec3Normalize(vec3Sub(center, eye));
        const Vec3 s = vec3Normalize(vec3Cross(f, up));
        const Vec3 u = vec3Cross(s, f);

        Mat4 out = identity();
        out.m[0] = static_cast<float>(s.x);
        out.m[4] = static_cast<float>(s.y);
        out.m[8] = static_cast<float>(s.z);

        out.m[1] = static_cast<float>(u.x);
        out.m[5] = static_cast<float>(u.y);
        out.m[9] = static_cast<float>(u.z);

        out.m[2] = static_cast<float>(-f.x);
        out.m[6] = static_cast<float>(-f.y);
        out.m[10] = static_cast<float>(-f.z);

        out.m[12] = static_cast<float>(-vec3Dot(s, eye));
        out.m[13] = static_cast<float>(-vec3Dot(u, eye));
        out.m[14] = static_cast<float>(vec3Dot(f, eye));
        return out;
    }
};

} // namespace torquelab
