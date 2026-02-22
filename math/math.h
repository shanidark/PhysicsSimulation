#include <algorithm> // swap
#include <array>

// 3d vectors & matrices and quaternions (for orientation)

// Vectors
// обусловимся, что они вертикальны
struct Vector3d {
    float x, y, z; // float is used in vulkan
};

inline Vector3d operator+(const Vector3d& a, const Vector3d& b) {
    return Vector3d{a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vector3d operator-(const Vector3d& a, const Vector3d& b) {
    return Vector3d{a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vector3d operator*(const Vector3d& a, const float& f) {
    return Vector3d{a.x * f, a.y * f, a.z * f};
}

inline float dot(const Vector3d& a, const Vector3d& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vector3d cross(const Vector3d& a, const Vector3d& b) {
    float x, y, z;
    x = a.y*b.z - a.z*b.y;
    y = a.z*b.x - a.x*b.z;
    z = a.x*b.y - a.y*b.x;
    return Vector3d{x, y, z};
}

// Matrices
struct Matrix3d {
    std::array<std::array<float, 3>, 3> mat;
};


// почему не скалярное произведение 2 векторов в x, y, z? потому что в матрице
// по традиции в памяти она представлена как 3 подряд строки
// а если я ее сделаю тремя векторами, то будет проблема, потому что векторы
// вертикальные, а столбцами хранить матрицу не очень выгодно
inline Vector3d operator*(const Matrix3d& m, const Vector3d& v) {
    float x, y, z;
    x = m.mat[0][0] * v.x + m.mat[0][1] * v.y + m.mat[0][2] * v.z;
    y = m.mat[1][0] * v.x + m.mat[1][1] * v.y + m.mat[1][2] * v.z;
    z = m.mat[2][0] * v.x + m.mat[2][1] * v.y + m.mat[2][2] * v.z;
    return Vector3d{x, y, z};
}

inline Vector3d operator*(const Vector3d& v, const Matrix3d& m) {
    float x, y, z;
    x = v.x * m.mat[0][0] + v.y * m.mat[1][0] + v.z * m.mat[2][0];
    y = v.x * m.mat[0][1] + v.y * m.mat[1][1] + v.z * m.mat[2][1];
    z = v.x * m.mat[0][2] + v.y * m.mat[1][2] + v.z * m.mat[2][2];
    return Vector3d{x, y, z};
}

// Quaternions
struct Quat {
    float a, b, c, d;
};

