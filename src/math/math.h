#pragma once

#include <algorithm>
#include <array>
#include <cmath>

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

inline Vec3 operator+(Vec3 a, Vec3 b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 operator-(Vec3 a, Vec3 b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 operator-(Vec3 v) {
    return {-v.x, -v.y, -v.z};
}

inline Vec3 operator*(Vec3 v, float s) {
    return {v.x * s, v.y * s, v.z * s};
}

inline Vec3 operator*(float s, Vec3 v) {
    return v * s;
}

inline Vec3 operator/(Vec3 v, float s) {
    return {v.x / s, v.y / s, v.z / s};
}

inline Vec3& operator+=(Vec3& a, Vec3 b) {
    a = a + b;
    return a;
}

inline Vec3& operator-=(Vec3& a, Vec3 b) {
    a = a - b;
    return a;
}

inline Vec3& operator*=(Vec3& v, float s) {
    v = v * s;
    return v;
}

inline float dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(Vec3 a, Vec3 b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

inline float lengthSquared(Vec3 v) {
    return dot(v, v);
}

inline float length(Vec3 v) {
    return std::sqrt(lengthSquared(v));
}

inline Vec3 normalized(Vec3 v) {
    const float len = length(v);
    if (len <= 0.000001f) {
        return {};
    }
    return v / len;
}

struct Mat3 {
    std::array<std::array<float, 3>, 3> m {{
        {{1.0f, 0.0f, 0.0f}},
        {{0.0f, 1.0f, 0.0f}},
        {{0.0f, 0.0f, 1.0f}},
    }};
};

inline Mat3 identityMat3() {
    return {};
}

inline Mat3 diagonalMat3(float x, float y, float z) {
    Mat3 out {};
    out.m[0][0] = x;
    out.m[1][1] = y;
    out.m[2][2] = z;
    return out;
}

inline Vec3 operator*(const Mat3& a, Vec3 v) {
    return {
        a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
        a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
        a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z,
    };
}

inline Mat3 operator*(const Mat3& a, const Mat3& b) {
    Mat3 out {};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out.m[row][col] = 0.0f;
            for (int k = 0; k < 3; ++k) {
                out.m[row][col] += a.m[row][k] * b.m[k][col];
            }
        }
    }
    return out;
}

inline Mat3 transpose(const Mat3& a) {
    Mat3 out {};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out.m[row][col] = a.m[col][row];
        }
    }
    return out;
}

struct Quat {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

inline Quat operator+(Quat a, Quat b) {
    return {a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Quat operator*(Quat q, float s) {
    return {q.w * s, q.x * s, q.y * s, q.z * s};
}

inline Quat operator*(Quat a, Quat b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

inline Quat& operator+=(Quat& a, Quat b) {
    a = a + b;
    return a;
}

inline Quat normalized(Quat q) {
    const float len = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (len <= 0.000001f) {
        return {};
    }
    return {q.w / len, q.x / len, q.y / len, q.z / len};
}

inline Quat quatFromAxisAngle(Vec3 axis, float radians) {
    axis = normalized(axis);
    const float halfAngle = radians * 0.5f;
    const float s = std::sin(halfAngle);
    return normalized({std::cos(halfAngle), axis.x * s, axis.y * s, axis.z * s});
}

inline Mat3 toMat3(Quat q) {
    q = normalized(q);

    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;

    Mat3 out {};
    out.m[0][0] = 1.0f - 2.0f * (yy + zz);
    out.m[0][1] = 2.0f * (xy - wz);
    out.m[0][2] = 2.0f * (xz + wy);
    out.m[1][0] = 2.0f * (xy + wz);
    out.m[1][1] = 1.0f - 2.0f * (xx + zz);
    out.m[1][2] = 2.0f * (yz - wx);
    out.m[2][0] = 2.0f * (xz - wy);
    out.m[2][1] = 2.0f * (yz + wx);
    out.m[2][2] = 1.0f - 2.0f * (xx + yy);
    return out;
}

inline Vec3 rotate(Quat q, Vec3 v) {
    const Vec3 qv {q.x, q.y, q.z};
    const Vec3 t = cross(qv, v) * 2.0f;
    return v + t * q.w + cross(qv, t);
}

inline Mat3 boxInverseInertiaTensor(float mass, Vec3 size) {
    const float x2 = size.x * size.x;
    const float y2 = size.y * size.y;
    const float z2 = size.z * size.z;

    const float ix = (1.0f / 12.0f) * mass * (y2 + z2);
    const float iy = (1.0f / 12.0f) * mass * (x2 + z2);
    const float iz = (1.0f / 12.0f) * mass * (x2 + y2);

    return diagonalMat3(1.0f / ix, 1.0f / iy, 1.0f / iz);
}

inline Mat3 sphereInverseInertiaTensor(float mass, float radius) {
    const float inertia = (2.0f / 5.0f) * mass * radius * radius;
    return diagonalMat3(1.0f / inertia, 1.0f / inertia, 1.0f / inertia);
}
