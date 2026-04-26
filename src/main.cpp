#include "physics/physics_world.h"
#include "render/debug_renderer.h"

#include <chrono>
#include <iostream>

int main() {
    PhysicsWorld world;

    RigidBody& box = world.createBox(2.0f, {1.2f, 1.2f, 1.2f}, {0.0f, 5.0f, 0.0f});
    box.linearVelocity = {1.1f, 0.0f, 0.4f};
    box.angularVelocity = {1.4f, 0.7f, 0.2f};

    RigidBody& second = world.createBox(1.0f, {0.8f, 0.8f, 0.8f}, {-2.0f, 8.0f, -0.8f});
    second.linearVelocity = {0.5f, -0.5f, 0.2f};
    second.angularVelocity = {-0.6f, 1.2f, 0.4f};

    RigidBody& boxA = world.createBox(1.5f, {1.0f, 1.0f, 1.0f}, {-2.0f, 1.0f, 2.0f});
    boxA.linearVelocity = {1.6f, 0.0f, 0.0f};
    boxA.angularVelocity = {0.0f, 0.8f, 0.4f};
    boxA.restitution = 0.35f;

    RigidBody& boxB = world.createBox(1.5f, {1.0f, 1.0f, 1.0f}, {2.0f, 1.0f, 2.0f});
    boxB.linearVelocity = {-1.6f, 0.0f, 0.0f};
    boxB.angularVelocity = {0.2f, -0.7f, 0.0f};
    boxB.restitution = 0.35f;

    RigidBody& obstacle = world.createBox(0.0f, {1.5f, 1.5f, 1.5f}, {0.0f, 0.75f, -2.2f});
    obstacle.orientation = quatFromAxisAngle({0.0f, 1.0f, 0.0f}, 0.35f);
    obstacle.updateDerivedData();

    RigidBody& sphereVsBox = world.createSphere(1.0f, 0.45f, {-3.0f, 0.55f, -2.2f});
    sphereVsBox.linearVelocity = {3.2f, 0.0f, 0.0f};
    sphereVsBox.restitution = 0.55f;

    RigidBody& sphereA = world.createSphere(1.0f, 0.55f, {2.0f, 6.0f, 0.0f});
    sphereA.linearVelocity = {-0.8f, 0.0f, 0.0f};
    sphereA.restitution = 0.65f;

    RigidBody& sphereB = world.createSphere(1.0f, 0.55f, {0.7f, 4.0f, 0.0f});
    sphereB.linearVelocity = {0.45f, 0.0f, 0.0f};
    sphereB.restitution = 0.65f;

    DebugRenderer renderer;
    if (!renderer.init(1280, 720, "Rigid body simulation")) {
        std::cerr << "Failed to initialize OpenGL window\n";
        return 1;
    }

    using clock = std::chrono::steady_clock;
    auto previous = clock::now();
    float accumulator = 0.0f;
    constexpr float fixedDt = 1.0f / 120.0f;

    while (!renderer.shouldClose()) {
        const auto now = clock::now();
        const float frameDt = std::chrono::duration<float>(now - previous).count();
        previous = now;

        if (!renderer.paused()) {
            accumulator += frameDt;
            while (accumulator >= fixedDt) {
                world.step(fixedDt);
                accumulator -= fixedDt;
            }
        } else if (renderer.stepRequested()) {
            accumulator = 0.0f;
            world.step(fixedDt);
        }

        renderer.beginFrame();
        renderer.drawWorld(world);
        renderer.endFrame();
    }

    renderer.shutdown();
    return 0;
}
