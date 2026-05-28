#pragma once

#include "physics/rigid_body.h"
#include "physics/contact.h"

#include <cstddef>
#include <vector>

struct PhysicsWorld {
    std::vector<RigidBody> bodies;
    std::vector<Contact> contacts;
    Vec3 gravity {0.0f, -9.81f, 0.0f};
    float groundY = 0.0f;
    float groundFriction = 0.6f;
    float restVelocityThreshold = 1.0f;
    float sleepThreshold = 0.06f;
    float sleepDelay = 0.4f;
    int solverIterations = 8;

    size_t createBox(float mass, Vec3 size, Vec3 position);
    size_t createSphere(float mass, float radius, Vec3 position);
    void step(float dt);

private:
    void generateContacts();
    void applyRollingResistance(float dt);
    void updateSleep(float dt);
    void generateBoxGroundContacts(RigidBody& body);
    void generateSphereGroundContact(RigidBody& body);
    void generateBoxBoxContact(RigidBody& a, RigidBody& b);
    void generateSphereBoxContact(RigidBody& sphere, RigidBody& box);
    void generateSphereSphereContact(RigidBody& a, RigidBody& b);
};
