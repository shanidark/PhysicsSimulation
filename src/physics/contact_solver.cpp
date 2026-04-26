#include "physics/contact_solver.h"

#include <algorithm>

namespace {
Vec3 velocityAtPoint(const RigidBody* body, Vec3 point) {
    if (!body || body->isStatic) {
        return {};
    }

    const Vec3 radius = point - body->position;
    return body->linearVelocity + cross(body->angularVelocity, radius);
}

float inverseMass(const RigidBody* body) {
    if (!body || body->isStatic) {
        return 0.0f;
    }
    return body->inverseMass;
}

float impulseDenominator(const RigidBody* body, Vec3 point, Vec3 direction) {
    if (!body || body->isStatic) {
        return 0.0f;
    }

    const Vec3 radius = point - body->position;
    const Vec3 angular = cross(body->inverseInertiaWorld * cross(radius, direction), radius);
    return body->inverseMass + dot(angular, direction);
}

void applyImpulse(RigidBody* body, Vec3 impulse, Vec3 point) {
    if (body && !body->isStatic) {
        body->applyImpulse(impulse, point);
    }
}
}

void ContactSolver::solve(std::vector<Contact>& contacts, int iterations) {
    for (int i = 0; i < iterations; ++i) {
        for (Contact& contact : contacts) {
            solveContact(contact);
        }
    }

    for (Contact& contact : contacts) {
        correctPosition(contact);
    }
}

void ContactSolver::solveContact(Contact& contact) {
    const Vec3 velocityA = velocityAtPoint(contact.a, contact.point);
    const Vec3 velocityB = velocityAtPoint(contact.b, contact.point);
    const Vec3 relativeVelocity = velocityA - velocityB;
    const float normalVelocity = dot(relativeVelocity, contact.normal);

    if (normalVelocity > 0.0f) {
        return;
    }

    const float denominator =
        impulseDenominator(contact.a, contact.point, contact.normal) +
        impulseDenominator(contact.b, contact.point, contact.normal);
    if (denominator <= 0.000001f) {
        return;
    }

    const float restitution = normalVelocity < -1.0f ? contact.restitution : 0.0f;
    const float normalImpulseMagnitude = -(1.0f + restitution) * normalVelocity / denominator;
    const Vec3 normalImpulse = contact.normal * normalImpulseMagnitude;
    applyImpulse(contact.a, normalImpulse, contact.point);
    applyImpulse(contact.b, -normalImpulse, contact.point);

    const Vec3 updatedRelativeVelocity =
        velocityAtPoint(contact.a, contact.point) - velocityAtPoint(contact.b, contact.point);
    Vec3 tangent = updatedRelativeVelocity - contact.normal * dot(updatedRelativeVelocity, contact.normal);
    const float tangentLength = length(tangent);
    if (tangentLength <= 0.000001f) {
        return;
    }

    tangent = tangent / tangentLength;
    const float tangentDenominator =
        impulseDenominator(contact.a, contact.point, tangent) +
        impulseDenominator(contact.b, contact.point, tangent);
    if (tangentDenominator <= 0.000001f) {
        return;
    }

    float tangentImpulseMagnitude = -dot(updatedRelativeVelocity, tangent) / tangentDenominator;
    const float maxFriction = contact.friction * normalImpulseMagnitude;
    tangentImpulseMagnitude = std::clamp(tangentImpulseMagnitude, -maxFriction, maxFriction);

    const Vec3 frictionImpulse = tangent * tangentImpulseMagnitude;
    applyImpulse(contact.a, frictionImpulse, contact.point);
    applyImpulse(contact.b, -frictionImpulse, contact.point);
}

void ContactSolver::correctPosition(Contact& contact) {
    constexpr float allowedPenetration = 0.01f;
    constexpr float correctionPercent = 0.75f;

    const float totalInverseMass = inverseMass(contact.a) + inverseMass(contact.b);
    if (totalInverseMass <= 0.000001f) {
        return;
    }

    const float correctionDepth = std::max(contact.penetration - allowedPenetration, 0.0f);
    const Vec3 correction =
        contact.normal * (correctionDepth * correctionPercent * contact.correctionWeight / totalInverseMass);

    if (contact.a && !contact.a->isStatic) {
        contact.a->position += correction * contact.a->inverseMass;
        contact.a->updateDerivedData();
    }
    if (contact.b && !contact.b->isStatic) {
        contact.b->position -= correction * contact.b->inverseMass;
        contact.b->updateDerivedData();
    }
}
