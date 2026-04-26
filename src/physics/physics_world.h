#pragma once

#include "physics/rigid_body.h"
#include "physics/contact.h"

#include <vector>

struct PhysicsWorld {
    std::vector<RigidBody> bodies;
    std::vector<Contact> contacts;
    Vec3 gravity {0.0f, -9.81f, 0.0f};
    float groundY = 0.0f;
    int solverIterations = 8;

    RigidBody& createBox(float mass, Vec3 size, Vec3 position);
    RigidBody& createSphere(float mass, float radius, Vec3 position);
    void step(float dt);

private:
    void generateContacts();
    void applyRollingResistance(float dt);
    void generateBoxGroundContacts(RigidBody& body);
    void generateSphereGroundContact(RigidBody& body);
    void generateBoxBoxContact(RigidBody& a, RigidBody& b);
    void generateSphereBoxContact(RigidBody& sphere, RigidBody& box);
    void generateSphereSphereContact(RigidBody& a, RigidBody& b);
};
