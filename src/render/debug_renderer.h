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
    void endFrame();
    void shutdown();
    void setPaused(bool value);
    bool paused() const;
    bool stepRequested();

private:
    void handleInput(float dt);
    void setupCamera();
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
    bool stepRequested_ = false;
    bool previousSpaceDown_ = false;
    bool previousStepDown_ = false;
    bool previousContactsDown_ = false;
};
