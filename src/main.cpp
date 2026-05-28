#include "physics/physics_world.h"
#include "render/debug_renderer.h"

#include <chrono>
#include <iostream>

int main() {
    PhysicsWorld world;

    const size_t boxIdx = world.createBox(2.0f, {1.2f, 1.2f, 1.2f}, {0.0f, 5.0f, 0.0f});
    world.bodies[boxIdx].linearVelocity = {1.1f, 0.0f, 0.4f};
    world.bodies[boxIdx].angularVelocity = {1.4f, 0.7f, 0.2f};

    const size_t secondIdx = world.createBox(1.0f, {0.8f, 0.8f, 0.8f}, {-2.0f, 8.0f, -0.8f});
    world.bodies[secondIdx].linearVelocity = {0.5f, -0.5f, 0.2f};
    world.bodies[secondIdx].angularVelocity = {-0.6f, 1.2f, 0.4f};

    const size_t boxAIdx = world.createBox(1.5f, {1.0f, 1.0f, 1.0f}, {-2.0f, 1.0f, 2.0f});
    world.bodies[boxAIdx].linearVelocity = {1.6f, 0.0f, 0.0f};
    world.bodies[boxAIdx].angularVelocity = {0.0f, 0.8f, 0.4f};
    world.bodies[boxAIdx].restitution = 0.35f;

    const size_t boxBIdx = world.createBox(1.5f, {1.0f, 1.0f, 1.0f}, {2.0f, 1.0f, 2.0f});
    world.bodies[boxBIdx].linearVelocity = {-1.6f, 0.0f, 0.0f};
    world.bodies[boxBIdx].angularVelocity = {0.2f, -0.7f, 0.0f};
    world.bodies[boxBIdx].restitution = 0.35f;

    const size_t obstacleIdx = world.createBox(0.0f, {1.5f, 1.5f, 1.5f}, {0.0f, 0.75f, -2.2f});
    world.bodies[obstacleIdx].orientation = quatFromAxisAngle({0.0f, 1.0f, 0.0f}, 0.35f);
    world.bodies[obstacleIdx].updateDerivedData();

    const size_t sphereVsBoxIdx = world.createSphere(1.0f, 0.45f, {-3.0f, 0.55f, -2.2f});
    world.bodies[sphereVsBoxIdx].linearVelocity = {3.2f, 0.0f, 0.0f};
    world.bodies[sphereVsBoxIdx].restitution = 0.55f;

    const size_t sphereAIdx = world.createSphere(1.0f, 0.55f, {2.0f, 6.0f, 0.0f});
    world.bodies[sphereAIdx].linearVelocity = {-0.8f, 0.0f, 0.0f};
    world.bodies[sphereAIdx].restitution = 0.65f;

    const size_t sphereBIdx = world.createSphere(1.0f, 0.55f, {0.7f, 4.0f, 0.0f});
    world.bodies[sphereBIdx].linearVelocity = {0.45f, 0.0f, 0.0f};
    world.bodies[sphereBIdx].restitution = 0.65f;

    DebugRenderer renderer;
    if (!renderer.init(1280, 720, "Rigid body simulation")) {
        std::cerr << "Failed to initialize OpenGL window\n";
        return 1;
    }

    using clock = std::chrono::steady_clock;
    auto previous = clock::now();
    float accumulator = 0.0f;
    constexpr float fixedDt = 1.0f / 120.0f;
    constexpr float maxAccumulator = 0.25f;

    while (!renderer.shouldClose()) {
        const auto now = clock::now();
        const float frameDt = std::chrono::duration<float>(now - previous).count();
        previous = now;

        if (!renderer.paused()) {
            accumulator += frameDt;
            if (accumulator > maxAccumulator) accumulator = maxAccumulator;
            while (accumulator >= fixedDt) {
                world.step(fixedDt);
                accumulator -= fixedDt;
            }
        } else if (renderer.stepRequested()) {
            accumulator = 0.0f;
            world.step(fixedDt);
        }

        renderer.beginFrame(frameDt);
        renderer.drawWorld(world);
        renderer.endFrame();
    }

    renderer.shutdown();
    return 0;
}
