#pragma once

#include "physics/physics_world.h"
#include "render/font_renderer.h"

struct Camera {
    float yaw = 38.0f;
    float pitch = 24.0f;
    float distance = 10.0f;
    Vec3 target {0.0f, 1.0f, 0.0f};
};

class DebugRenderer {
public:
    bool init(int width, int height, const char* title);
    bool shouldClose() const;
    void beginFrame(float dt);
    void drawWorld(const PhysicsWorld& world);
    void drawGui(PhysicsWorld& world);
    void endFrame();
    void shutdown();
    void setPaused(bool value);
    bool paused() const;
    bool stepRequested();
    bool resetRequested();

private:
    void handleInput(float dt);
    void setupCamera();
    void drawSky();
    void drawGround();
    void drawBody(const RigidBody& body);
    void drawContacts(const PhysicsWorld& world);
    void drawOverlay();

    struct GLFWwindow* window_ = nullptr;
    FontRenderer font_ {};
    Camera camera_ {};
    int framebufferWidth_ = 1;
    int framebufferHeight_ = 1;
    bool paused_ = false;
    bool drawContacts_ = false;
    bool showGui_ = false;
    bool stepRequested_ = false;
    bool previousSpaceDown_ = false;
    bool previousStepDown_ = false;
    bool previousContactsDown_ = false;
    bool previousGuiDown_ = false;
    bool resetRequested_ = false;

    // Spawner state
    int spawnType_ = 0;  // 0 = Box, 1 = Sphere
    Vec3 spawnPosition_ {0.0f, 5.0f, 0.0f};
    Vec3 spawnBoxSize_ {1.0f, 1.0f, 1.0f};
    float spawnRadius_ = 0.45f;
    float spawnMass_ = 1.0f;
    bool spawnStatic_ = false;
};
