#include "render/debug_renderer.h"

#include <GLFW/glfw3.h>

#include <cmath>

namespace {
static constexpr float pi = 3.14159265358979323846f;

float radians(float degrees) {
    return degrees * pi / 180.0f;
}

void perspective(float fovYDegrees, float aspect, float zNear, float zFar) {
    const float f = 1.0f / std::tan(radians(fovYDegrees) * 0.5f);
    const float m[16] = {
        f / aspect, 0.0f, 0.0f, 0.0f,
        0.0f, f, 0.0f, 0.0f,
        0.0f, 0.0f, (zFar + zNear) / (zNear - zFar), -1.0f,
        0.0f, 0.0f, (2.0f * zFar * zNear) / (zNear - zFar), 0.0f,
    };
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(m);
}

void lookAt(Vec3 eye, Vec3 center, Vec3 up) {
    const Vec3 f = normalized(center - eye);
    const Vec3 s = normalized(cross(f, up));
    const Vec3 u = cross(s, f);

    const float m[16] = {
        s.x, u.x, -f.x, 0.0f,
        s.y, u.y, -f.y, 0.0f,
        s.z, u.z, -f.z, 0.0f,
        -dot(s, eye), -dot(u, eye), dot(f, eye), 1.0f,
    };

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(m);
}

void drawUnitCube() {
    glBegin(GL_QUADS);

    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(-1.0f, -1.0f, 1.0f);
    glVertex3f(1.0f, -1.0f, 1.0f);
    glVertex3f(1.0f, 1.0f, 1.0f);
    glVertex3f(-1.0f, 1.0f, 1.0f);

    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(1.0f, -1.0f, -1.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f);
    glVertex3f(-1.0f, 1.0f, -1.0f);
    glVertex3f(1.0f, 1.0f, -1.0f);

    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, -1.0f, 1.0f);
    glVertex3f(1.0f, -1.0f, -1.0f);
    glVertex3f(1.0f, 1.0f, -1.0f);
    glVertex3f(1.0f, 1.0f, 1.0f);

    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f);
    glVertex3f(-1.0f, -1.0f, 1.0f);
    glVertex3f(-1.0f, 1.0f, 1.0f);
    glVertex3f(-1.0f, 1.0f, -1.0f);

    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(-1.0f, 1.0f, 1.0f);
    glVertex3f(1.0f, 1.0f, 1.0f);
    glVertex3f(1.0f, 1.0f, -1.0f);
    glVertex3f(-1.0f, 1.0f, -1.0f);

    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f);
    glVertex3f(1.0f, -1.0f, -1.0f);
    glVertex3f(1.0f, -1.0f, 1.0f);
    glVertex3f(-1.0f, -1.0f, 1.0f);

    glEnd();
}

void drawUnitSphere(int slices = 24, int stacks = 12) {
    for (int stack = 0; stack < stacks; ++stack) {
        const float phi0 = -pi * 0.5f + pi * static_cast<float>(stack) / static_cast<float>(stacks);
        const float phi1 = -pi * 0.5f + pi * static_cast<float>(stack + 1) / static_cast<float>(stacks);
        const float y0 = std::sin(phi0);
        const float y1 = std::sin(phi1);
        const float r0 = std::cos(phi0);
        const float r1 = std::cos(phi1);

        glBegin(GL_QUAD_STRIP);
        for (int slice = 0; slice <= slices; ++slice) {
            const float theta = 2.0f * pi * static_cast<float>(slice) / static_cast<float>(slices);
            const float x = std::cos(theta);
            const float z = std::sin(theta);

            glNormal3f(x * r0, y0, z * r0);
            glVertex3f(x * r0, y0, z * r0);
            glNormal3f(x * r1, y1, z * r1);
            glVertex3f(x * r1, y1, z * r1);
        }
        glEnd();
    }
}

void drawRect(float x, float y, float width, float height) {
    glBegin(GL_QUADS);
    glVertex2f(x, y);
    glVertex2f(x + width, y);
    glVertex2f(x + width, y + height);
    glVertex2f(x, y + height);
    glEnd();
}

void drawCorner(float cx, float cy, float radius, float startAngle, float endAngle) {
    constexpr int segments = 10;
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(cx, cy);
    for (int i = 0; i <= segments; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(segments);
        const float angle = startAngle + (endAngle - startAngle) * t;
        glVertex2f(cx + std::cos(angle) * radius, cy + std::sin(angle) * radius);
    }
    glEnd();
}

void drawRoundedRect(float x, float y, float width, float height, float radius) {
    radius = std::min(radius, std::min(width, height) * 0.5f);

    drawRect(x + radius, y, width - radius * 2.0f, height);
    drawRect(x, y + radius, radius, height - radius * 2.0f);
    drawRect(x + width - radius, y + radius, radius, height - radius * 2.0f);

    drawCorner(x + radius, y + radius, radius, pi, pi * 1.5f);
    drawCorner(x + width - radius, y + radius, radius, pi * 1.5f, pi * 2.0f);
    drawCorner(x + width - radius, y + height - radius, radius, 0.0f, pi * 0.5f);
    drawCorner(x + radius, y + height - radius, radius, pi * 0.5f, pi);
}
}

bool DebugRenderer::init(int width, int height, const char* title) {
    if (!glfwInit()) {
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);

    const float lightPosition[4] = {2.0f, 6.0f, 4.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

    font_.init(22);

    return true;
}

bool DebugRenderer::shouldClose() const {
    return glfwWindowShouldClose(window_);
}

void DebugRenderer::beginFrame() {
    handleInput();

    glfwGetFramebufferSize(window_, &framebufferWidth_, &framebufferHeight_);
    glViewport(0, 0, framebufferWidth_, framebufferHeight_);
    perspective(60.0f,
                static_cast<float>(framebufferWidth_) / static_cast<float>(framebufferHeight_),
                0.1f,
                100.0f);
    setupCamera();

    glClearColor(0.08f, 0.09f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void DebugRenderer::drawWorld(const PhysicsWorld& world) {
    drawGround();
    for (const RigidBody& body : world.bodies) {
        drawBody(body);
    }
    if (drawContacts_) {
        drawContacts(world);
    }
    drawOverlay();
}

void DebugRenderer::endFrame() {
    glfwSwapBuffers(window_);
    glfwPollEvents();
}

void DebugRenderer::shutdown() {
    font_.shutdown();

    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
}

void DebugRenderer::setPaused(bool value) {
    paused_ = value;
}

bool DebugRenderer::paused() const {
    return paused_;
}

bool DebugRenderer::stepRequested() {
    const bool requested = stepRequested_;
    stepRequested_ = false;
    return requested;
}

void DebugRenderer::handleInput() {
    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }

    const bool spaceDown = glfwGetKey(window_, GLFW_KEY_SPACE) == GLFW_PRESS;
    if (spaceDown && !previousSpaceDown_) {
        paused_ = !paused_;
    }
    previousSpaceDown_ = spaceDown;

    const bool stepDown = glfwGetKey(window_, GLFW_KEY_N) == GLFW_PRESS;
    if (stepDown && !previousStepDown_) {
        stepRequested_ = true;
    }
    previousStepDown_ = stepDown;

    const bool contactsDown = glfwGetKey(window_, GLFW_KEY_C) == GLFW_PRESS;
    if (contactsDown && !previousContactsDown_) {
        drawContacts_ = !drawContacts_;
    }
    previousContactsDown_ = contactsDown;

    if (glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS) {
        camera_.yaw -= 70.0f * 0.016f;
    }
    if (glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS) {
        camera_.yaw += 70.0f * 0.016f;
    }
    if (glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS) {
        camera_.pitch = std::min(camera_.pitch + 55.0f * 0.016f, 82.0f);
    }
    if (glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS) {
        camera_.pitch = std::max(camera_.pitch - 55.0f * 0.016f, -15.0f);
    }
    if (glfwGetKey(window_, GLFW_KEY_Q) == GLFW_PRESS) {
        camera_.distance = std::min(camera_.distance + 5.0f * 0.016f, 25.0f);
    }
    if (glfwGetKey(window_, GLFW_KEY_E) == GLFW_PRESS) {
        camera_.distance = std::max(camera_.distance - 5.0f * 0.016f, 3.0f);
    }
}

void DebugRenderer::setupCamera() {
    const float yaw = radians(camera_.yaw);
    const float pitch = radians(camera_.pitch);
    const Vec3 eye {
        camera_.target.x + camera_.distance * std::cos(pitch) * std::sin(yaw),
        camera_.target.y + camera_.distance * std::sin(pitch),
        camera_.target.z + camera_.distance * std::cos(pitch) * std::cos(yaw),
    };
    lookAt(eye, camera_.target, {0.0f, 1.0f, 0.0f});
}

void DebugRenderer::drawGround() {
    glDisable(GL_LIGHTING);
    glColor3f(0.34f, 0.38f, 0.34f);
    glBegin(GL_LINES);
    for (int i = -12; i <= 12; ++i) {
        glVertex3f(static_cast<float>(i), 0.0f, -12.0f);
        glVertex3f(static_cast<float>(i), 0.0f, 12.0f);
        glVertex3f(-12.0f, 0.0f, static_cast<float>(i));
        glVertex3f(12.0f, 0.0f, static_cast<float>(i));
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

void DebugRenderer::drawBody(const RigidBody& body) {
    const Mat3 r = toMat3(body.orientation);
    const float transform[16] = {
        r.m[0][0], r.m[1][0], r.m[2][0], 0.0f,
        r.m[0][1], r.m[1][1], r.m[2][1], 0.0f,
        r.m[0][2], r.m[1][2], r.m[2][2], 0.0f,
        body.position.x, body.position.y, body.position.z, 1.0f,
    };

    glPushMatrix();
    glMultMatrixf(transform);
    if (body.shape == ShapeType::Sphere) {
        glScalef(body.radius, body.radius, body.radius);
        glColor3f(0.20f, 0.58f, 0.88f);
        drawUnitSphere();
    } else {
        glScalef(body.halfExtents.x, body.halfExtents.y, body.halfExtents.z);
        glColor3f(0.86f, 0.34f, 0.24f);
        drawUnitCube();
    }
    glPopMatrix();
}

void DebugRenderer::drawContacts(const PhysicsWorld& world) {
    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    glColor3f(0.15f, 0.75f, 0.95f);

    glBegin(GL_LINES);
    for (const Contact& contact : world.contacts) {
        const Vec3 end = contact.point + contact.normal * 0.45f;
        glVertex3f(contact.point.x, contact.point.y + 0.01f, contact.point.z);
        glVertex3f(end.x, end.y + 0.01f, end.z);
    }
    glEnd();

    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void DebugRenderer::drawOverlay() {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, static_cast<double>(framebufferWidth_),
            static_cast<double>(framebufferHeight_), 0.0, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    const float helpX = 14.0f;
    const float helpY = 14.0f;
    const float helpWidth = 252.0f;
    const float helpHeight = 166.0f;
    const float helpTextScale = 0.75f;
    const float helpLineGap = 22.0f;
    const float helpLineCount = 6.0f;
    const float helpBlockHeight = font_.lineHeight(helpTextScale) + helpLineGap * (helpLineCount - 1.0f);
    const float helpBaselineY =
        helpY + (helpHeight - helpBlockHeight) * 0.5f + font_.lineHeight(helpTextScale) * 0.78f;

    glColor4f(0.02f, 0.025f, 0.03f, 0.68f);
    drawRoundedRect(helpX, helpY, helpWidth, helpHeight, 8.0f);

    glColor4f(0.92f, 0.95f, 0.96f, 1.0f);
    font_.drawText(28.0f, helpBaselineY, helpTextScale, "Space  Pause");
    font_.drawText(28.0f, helpBaselineY + helpLineGap, helpTextScale, "N      Step");
    font_.drawText(28.0f, helpBaselineY + helpLineGap * 2.0f, helpTextScale, "C      Contacts");
    font_.drawText(28.0f, helpBaselineY + helpLineGap * 3.0f, helpTextScale, "WASD   Camera");
    font_.drawText(28.0f, helpBaselineY + helpLineGap * 4.0f, helpTextScale, "Q/E    Zoom");
    font_.drawText(28.0f, helpBaselineY + helpLineGap * 5.0f, helpTextScale, "Esc    Exit");

    if (paused_) {
        const float pauseWidth = 212.0f;
        const float pauseHeight = 58.0f;
        const float pauseX = static_cast<float>(framebufferWidth_) * 0.5f - pauseWidth * 0.5f;
        const float pauseY = 22.0f;
        const float pauseScale = 1.15f;
        const char* pauseText = "PAUSED";
        const float pauseTextWidth = font_.measureText(pauseText, pauseScale);
        const float pauseTextX = pauseX + (pauseWidth - pauseTextWidth) * 0.5f;
        const float pauseTextY =
            pauseY + (pauseHeight - font_.lineHeight(pauseScale)) * 0.5f +
            font_.lineHeight(pauseScale) * 0.78f;

        glColor4f(0.02f, 0.025f, 0.03f, 0.78f);
        drawRoundedRect(pauseX, pauseY, pauseWidth, pauseHeight, 8.0f);
        glColor4f(0.98f, 0.72f, 0.24f, 1.0f);
        font_.drawText(pauseTextX, pauseTextY, pauseScale, pauseText);
    }

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_CULL_FACE);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}
