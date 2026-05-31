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

void RigidBody::applyImpulse(Vec3 impulse, Vec3 worldPoint) {
    if (isStatic()) return;
    if (sleeping && lengthSquared(impulse) < 1e-10f) return;
    sleeping = false;
    sleepTimer = 0.0f;
    const Vec3 offset = worldPoint - position;
    // linearVelocity += J * invMass: толчок в линейную скорость.
    // angularVelocity += I_world^-1 * (offset × J): cross product расстояния до точки и импульса
    // даёт момент — чем дальше от центра масс приложен импульс, тем сильнее закрутит.
    // Как шлепок по заднице: куда попал и на каком расстоянии от центра — столько и завертится.
    linearVelocity += impulse * inverseMass;
    angularVelocity += inverseInertiaWorld * cross(offset, impulse);
}

void RigidBody::clearAccumulators() {
    forceAccum = {};
    torqueAccum = {};
}

// inverseInertiaWorld = R * I_body^-1 * R^T — переносим тензор инерции из локальных координат в мировые.
// Тело повернулось — распределение масс относительно мировых осей изменилось, надо пересчитать.
// Без этого вращение ехало бы так, будто тело торчит в одну сторону независимо от ориентации.
void RigidBody::updateDerivedData() {
    orientation = normalized(orientation);
    rotation = toMat3(orientation);
    inverseInertiaWorld = rotation * inverseInertiaBody * transpose(rotation);
}

// Паша, тут интегрируем движение за шаг методом Эйлера.
// Сначала скорости от накопленных сил: v += F*invMass*dt, omega += I^-1*T*dt.
// Потом позиция и ориентация: p += v*dt.
//
// Кватернион вращения обновляем по формуле Пуассона: dq/dt = 0.5*(0,wx,wy,wz)*q.
// spin * orientation * 0.5*dt — прибавляем к q и нормализуем.
// Нормализация обязательна каждый кадр: за тысячи итераций float-погрешность раздует кватернион
// и вращение поедет в случайную сторону — как кабак в конце вечера.
//
// pow(damping, dt) масштабирует затухание под реальный dt — не зависит от FPS.
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
