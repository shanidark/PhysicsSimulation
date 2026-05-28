#include "physics/rigid_body.h"

#include <cmath>

void RigidBody::setBox(float mass, Vec3 size) {
    shape = ShapeType::Box;
    halfExtents = size * 0.5f;
    radius = 0.5f * length(size);
    sleeping = false;
    sleepTimer = 0.0f;

    if (mass <= 0.0f) {
        inverseMass = 0.0f;
        inverseInertiaBody = {};
        inverseInertiaWorld = {};
        return;
    }

    inverseMass = 1.0f / mass;
    inverseInertiaBody = boxInverseInertiaTensor(mass, size);
    updateDerivedData();
}

void RigidBody::setSphere(float mass, float sphereRadius) {
    shape = ShapeType::Sphere;
    radius = sphereRadius;
    halfExtents = {sphereRadius, sphereRadius, sphereRadius};
    sleeping = false;
    sleepTimer = 0.0f;

    if (mass <= 0.0f) {
        inverseMass = 0.0f;
        inverseInertiaBody = {};
        inverseInertiaWorld = {};
        return;
    }

    inverseMass = 1.0f / mass;
    inverseInertiaBody = sphereInverseInertiaTensor(mass, sphereRadius);
    updateDerivedData();
}

void RigidBody::applyForce(Vec3 force) {
    if (isStatic()) return;
    forceAccum += force;
}

void RigidBody::applyForceAtPoint(Vec3 force, Vec3 worldPoint) {
    if (isStatic()) return;
    const Vec3 offset = worldPoint - position;
    forceAccum += force;
    torqueAccum += cross(offset, force);
}

void RigidBody::applyImpulse(Vec3 impulse, Vec3 worldPoint) {
    if (isStatic()) return;
    // Don't wake for negligible impulse (e.g. zero impulse from two resting bodies)
    if (sleeping && lengthSquared(impulse) < 1e-10f) return;
    sleeping = false;
    sleepTimer = 0.0f;
    const Vec3 offset = worldPoint - position;
    linearVelocity += impulse * inverseMass;
    angularVelocity += inverseInertiaWorld * cross(offset, impulse);
}

void RigidBody::clearAccumulators() {
    forceAccum = {};
    torqueAccum = {};
}

void RigidBody::updateDerivedData() {
    orientation = normalized(orientation);
    rotation = toMat3(orientation);
    inverseInertiaWorld = rotation * inverseInertiaBody * transpose(rotation);
}

void RigidBody::integrate(float dt) {
    if (isStatic() || dt <= 0.0f) {
        clearAccumulators();
        return;
    }

    linearVelocity += forceAccum * inverseMass * dt;
    angularVelocity += (inverseInertiaWorld * torqueAccum) * dt;

    position += linearVelocity * dt;

    const Quat spin {0.0f, angularVelocity.x, angularVelocity.y, angularVelocity.z};
    orientation += (spin * orientation) * (0.5f * dt);
    orientation = normalized(orientation);

    const float linearScale = std::pow(linearDamping, dt);
    const float angularScale = std::pow(angularDamping, dt);
    linearVelocity *= linearScale;
    angularVelocity *= angularScale;

    updateDerivedData();
    clearAccumulators();
}
