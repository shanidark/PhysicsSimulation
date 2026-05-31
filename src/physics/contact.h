#pragma once

#include "physics/rigid_body.h"

struct Contact {
    RigidBody* a = nullptr;
    RigidBody* b = nullptr;

    Vec3 point {};
    Vec3 normal {0.0f, 1.0f, 0.0f};
    float penetration = 0.0f;
    float correctionWeight = 1.0f;

    float restitution = 0.0f;
    float friction = 0.6f;
    float restitutionThreshold = 1.0f;

    float accumulatedNormalImpulse = 0.0f;
    float accumulatedTangentImpulse = 0.0f;
};
