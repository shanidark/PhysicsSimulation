#pragma once

#include "math/math.h"

enum class ShapeType {
    Box,
    Sphere,
};

struct RigidBody {
    ShapeType shape = ShapeType::Box;

    Vec3 position {};
    Quat orientation {};

    Vec3 linearVelocity {};
    Vec3 angularVelocity {};

    Vec3 forceAccum {};
    Vec3 torqueAccum {};

    float inverseMass = 1.0f;
    Mat3 inverseInertiaBody {};
    Mat3 inverseInertiaWorld {};

    float linearDamping = 0.995f;
    float angularDamping = 0.995f;
    float rollingResistance = 0.35f;
    float restitution = 0.45f;

    Vec3 halfExtents {0.5f, 0.5f, 0.5f};
    float radius = 0.5f;

    bool isStatic = false;

    void setBox(float mass, Vec3 size);
    void setSphere(float mass, float sphereRadius);
    void applyForce(Vec3 force);
    void applyForceAtPoint(Vec3 force, Vec3 worldPoint);
    void applyImpulse(Vec3 impulse, Vec3 worldPoint);
    void clearAccumulators();
    void updateDerivedData();
    void integrate(float dt);
};
