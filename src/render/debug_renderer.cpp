#include "render/debug_renderer.h"

#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <random>

namespace {
constexpr float pi = std::numbers::pi_v<float>;

float radians(float degrees) {
    return degrees * pi / 180.0f;
}

// Матрица проекции — строим вручную, потому что gluPerspective в OpenGL 2.1 deprecated.
// f = 1/tan(fov/2): меньший угол обзора — больший зум. Делим f на aspect — иначе
// кубики расплющатся на широких мониторах.
// Третья строка кодирует глубину для z-буфера через near и far.
// Последний столбец [0, 0, -1, 0]: после деления вершины на w дальние объекты маленькие.
// Это и есть перспектива — как женщина уходит: всё есть, но становится меньше и недоступнее.
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

// Строим матрицу вида: откуда смотришь (eye) и куда (center).
// f — вперёд, s — вправо (cross(вперёд, вверх)), u — реальный вверх (cross(вправо, вперёд)).
// Три взаимно перпендикулярных вектора — ортонормальный базис камеры (метод Грама-Шмидта).
// dot-ы в последней строке считают сколько нужно сдвинуть мир чтоб камера стояла в нуле.
// Матрица записана column-major — OpenGL ест именно так, поэтому при записи строками транспонируем.
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

// Рисуем сферу горизонтальными полосами (стеки × слайсы) — как мандарин разрезанный по широте.
// phi — широта: от -pi/2 (низ) до pi/2 (верх). theta — долгота по кругу.
// y = sin(phi) — высота кольца, r = cos(phi) — радиус этого кольца.
// GL_QUAD_STRIP: два кольца + лента четырёхугольников между ними.
// Нормаль = позиция вершины (для единичной сферы это математически правильно).
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

// Четверть круга через GL_TRIANGLE_FAN: центр + дуга из 10 треугольников.
// Как кусок пирога — центр и лучи наружу.
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

// В OpenGL 2.1 нет закруглённых прямоугольников — лепим из трёх прямоугольников (крест без углов)
// плюс четыре четверти круга по углам. Каждый угол — вызов drawCorner с нужным диапазоном углов.
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
    // GL_COLOR_MATERIAL: glColor3f автоматически влияет на материал освещения —
    // не нужен отдельный glMaterialfv для каждого тела. Удобно.
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    // Холодный небесный ambient + один нейтральный направленный источник без бликов.
    // Specular убрали — с ним кубики блестели как влажный пластик, выглядело дёшево.
    const float ambient[4] = {0.28f, 0.30f, 0.34f, 1.0f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

    const float zero4[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    const float white[4] = {0.82f, 0.82f, 0.78f, 1.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT,  zero4);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  white);
    glLightfv(GL_LIGHT0, GL_SPECULAR, zero4);

    // GL_FOG: линейный туман от 12 до 24 метров от камеры.
    // Цвет тумана = цвет горизонта неба — края земли плавно уходят в небо.
    // Без тумана земля просто обрезалась бы как край карты в дешёвой игре.
    const float fogColor[4] = {0.54f, 0.72f, 0.88f, 1.0f};
    glEnable(GL_FOG);
    glFogi(GL_FOG_MODE, GL_LINEAR);
    glFogf(GL_FOG_START, 12.0f);
    glFogf(GL_FOG_END,   24.0f);
    glFogfv(GL_FOG_COLOR, fogColor);

    if (!font_.init(22)) {
        std::fprintf(stderr, "Font initialization failed; HUD text will be hidden\n");
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 6.0f;
    style.FrameRounding  = 4.0f;
    style.WindowBorderSize = 1.0f;
    style.Colors[ImGuiCol_WindowBg]     = ImVec4(0.14f, 0.15f, 0.18f, 0.96f);
    style.Colors[ImGuiCol_TitleBg]      = ImVec4(0.10f, 0.18f, 0.28f, 1.00f);
    style.Colors[ImGuiCol_TitleBgActive]= ImVec4(0.14f, 0.26f, 0.42f, 1.00f);
    style.Colors[ImGuiCol_FrameBg]      = ImVec4(0.20f, 0.22f, 0.27f, 1.00f);
    style.Colors[ImGuiCol_SliderGrab]   = ImVec4(0.36f, 0.60f, 0.90f, 1.00f);

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL2_Init();

    return true;
}

bool DebugRenderer::shouldClose() const {
    return glfwWindowShouldClose(window_);
}

void DebugRenderer::beginFrame(float dt) {
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    handleInput(dt);

    glfwGetFramebufferSize(window_, &framebufferWidth_, &framebufferHeight_);
    glViewport(0, 0, framebufferWidth_, framebufferHeight_);
    perspective(60.0f,
                static_cast<float>(framebufferWidth_) / static_cast<float>(framebufferHeight_),
                0.1f,
                100.0f);
    setupCamera();

    // Позиция света ставится ПОСЛЕ setupCamera (которая загружает modelview).
    // glLightfv с GL_POSITION трансформирует координату текущей modelview-матрицей —
    // т.е. {5, 10, 6} задаём в мировых координатах, OpenGL сам переводит в eye space.
    // Свет стоит на месте в мире, не вращается вместе с камерой.
    const float keyPos[4] = {5.0f, 10.0f, 6.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, keyPos);

    glClearColor(0.15f, 0.24f, 0.48f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void DebugRenderer::drawWorld(const PhysicsWorld& world) {
    drawSky();
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
    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window_);
    glfwPollEvents();
}

void DebugRenderer::shutdown() {
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

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

bool DebugRenderer::resetRequested() {
    const bool requested = resetRequested_;
    resetRequested_ = false;
    return requested;
}

// Space/N/C/Tab срабатывают ОДИН РАЗ при нажатии (edge trigger), а не каждый кадр пока держишь.
// Для этого запоминаем состояние прошлого кадра (previous*Down_) и сравниваем с текущим.
// "Не держал → держу" = нажатие. Без этого пауза включалась бы 60 раз в секунду — абсурд.
// WantCaptureKeyboard: если ImGui редактирует текстовое поле — шорткаты не трогаем.
// Camera (WASD/QE) наоборот — непрерывный ввод пока держишь клавишу.
void DebugRenderer::handleInput(float dt) {
    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }

    const bool uiCapture = ImGui::GetIO().WantCaptureKeyboard;

    const bool spaceDown = glfwGetKey(window_, GLFW_KEY_SPACE) == GLFW_PRESS;
    if (spaceDown && !previousSpaceDown_ && !uiCapture) {
        paused_ = !paused_;
    }
    previousSpaceDown_ = spaceDown;

    const bool stepDown = glfwGetKey(window_, GLFW_KEY_N) == GLFW_PRESS;
    if (stepDown && !previousStepDown_ && !uiCapture) {
        stepRequested_ = true;
    }
    previousStepDown_ = stepDown;

    const bool contactsDown = glfwGetKey(window_, GLFW_KEY_C) == GLFW_PRESS;
    if (contactsDown && !previousContactsDown_ && !uiCapture) {
        drawContacts_ = !drawContacts_;
    }
    previousContactsDown_ = contactsDown;

    const bool guiDown = glfwGetKey(window_, GLFW_KEY_TAB) == GLFW_PRESS;
    if (guiDown && !previousGuiDown_ && !uiCapture) {
        showGui_ = !showGui_;
    }
    previousGuiDown_ = guiDown;

    if (!uiCapture) {
        if (glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS) {
            camera_.yaw -= 70.0f * dt;
        }
        if (glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS) {
            camera_.yaw += 70.0f * dt;
        }
        if (glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS) {
            camera_.pitch = std::min(camera_.pitch + 55.0f * dt, 82.0f);
        }
        if (glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS) {
            camera_.pitch = std::max(camera_.pitch - 55.0f * dt, -15.0f);
        }
        if (glfwGetKey(window_, GLFW_KEY_Q) == GLFW_PRESS) {
            camera_.distance = std::min(camera_.distance + 5.0f * dt, 25.0f);
        }
        if (glfwGetKey(window_, GLFW_KEY_E) == GLFW_PRESS) {
            camera_.distance = std::max(camera_.distance - 5.0f * dt, 3.0f);
        }
    }
}

// Сферические координаты → декартовы. yaw — горизонтальный угол, pitch — вертикальный.
// cos(pitch)*sin(yaw) и cos(pitch)*cos(yaw) — X и Z на окружности радиуса cos(pitch).
// sin(pitch) — высота. Всё смещено на camera_.target, а не на ноль координат.
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

void DebugRenderer::drawSky() {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    // Хитрость скайбокса: берём текущую матрицу вида (с поворотом камеры),
    // обнуляем трансляцию (mv[12..14] = 0) и загружаем обратно.
    // Куб вращается вместе с камерой, но никуда не движется — выглядит как бесконечное небо.
    // Без этого небо ходило бы за тобой як бывшая которая знает где ты живёшь.
    float mv[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, mv);
    mv[12] = 0.0f; mv[13] = 0.0f; mv[14] = 0.0f;
    glLoadMatrixf(mv);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_FOG);
    glDisable(GL_CULL_FACE);
    glDepthMask(GL_FALSE); // небо не пишет в z-буфер — всегда позади всего

    constexpr float S = 40.0f;
    const float zenith[]  = {0.15f, 0.24f, 0.48f};
    const float horizon[] = {0.54f, 0.72f, 0.88f};
    const float nadir[]   = {0.22f, 0.26f, 0.20f};

    glBegin(GL_QUADS);

    glColor3fv(zenith);
    glVertex3f(-S,  S, -S); glVertex3f( S,  S, -S);
    glVertex3f( S,  S,  S); glVertex3f(-S,  S,  S);

    glColor3fv(nadir);
    glVertex3f(-S, -S,  S); glVertex3f( S, -S,  S);
    glVertex3f( S, -S, -S); glVertex3f(-S, -S, -S);

    // Боковые грани: зенит наверху вершин, горизонт внизу — градиент неба.
    glColor3fv(zenith);  glVertex3f(-S,  S,  S);
    glColor3fv(zenith);  glVertex3f( S,  S,  S);
    glColor3fv(horizon); glVertex3f( S, -S,  S);
    glColor3fv(horizon); glVertex3f(-S, -S,  S);

    glColor3fv(zenith);  glVertex3f( S,  S, -S);
    glColor3fv(zenith);  glVertex3f(-S,  S, -S);
    glColor3fv(horizon); glVertex3f(-S, -S, -S);
    glColor3fv(horizon); glVertex3f( S, -S, -S);

    glColor3fv(zenith);  glVertex3f( S,  S,  S);
    glColor3fv(zenith);  glVertex3f( S,  S, -S);
    glColor3fv(horizon); glVertex3f( S, -S, -S);
    glColor3fv(horizon); glVertex3f( S, -S,  S);

    glColor3fv(zenith);  glVertex3f(-S,  S, -S);
    glColor3fv(zenith);  glVertex3f(-S,  S,  S);
    glColor3fv(horizon); glVertex3f(-S, -S,  S);
    glColor3fv(horizon); glVertex3f(-S, -S, -S);

    glEnd();

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_FOG);
    glEnable(GL_CULL_FACE);

    glPopMatrix();
}

void DebugRenderer::drawGround() {
    glDisable(GL_LIGHTING);

    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
    glColor3f(0.22f, 0.26f, 0.20f);
    glBegin(GL_QUADS);
    glVertex3f(-25.0f, 0.0f, -25.0f);
    glVertex3f( 25.0f, 0.0f, -25.0f);
    glVertex3f( 25.0f, 0.0f,  25.0f);
    glVertex3f(-25.0f, 0.0f,  25.0f);
    glEnd();
    glDisable(GL_POLYGON_OFFSET_FILL);

    glColor3f(0.32f, 0.38f, 0.28f);
    glBegin(GL_LINES);
    for (int i = -12; i <= 12; ++i) {
        glVertex3f(static_cast<float>(i), 0.0f, -12.0f);
        glVertex3f(static_cast<float>(i), 0.0f,  12.0f);
        glVertex3f(-12.0f, 0.0f, static_cast<float>(i));
        glVertex3f( 12.0f, 0.0f, static_cast<float>(i));
    }
    glEnd();

    glEnable(GL_LIGHTING);
}

void DebugRenderer::drawBody(const RigidBody& body) {
    if (body.isStatic()) {
        glColor3f(0.50f, 0.54f, 0.58f);
    } else if (body.sleeping) {
        if (body.shape == ShapeType::Sphere)
            glColor3f(0.10f, 0.30f, 0.50f);
        else
            glColor3f(0.45f, 0.18f, 0.12f);
    } else {
        if (body.shape == ShapeType::Sphere)
            glColor3f(0.22f, 0.62f, 0.92f);
        else
            glColor3f(0.90f, 0.38f, 0.22f);
    }

    // Собираем трансформацию из матрицы вращения и позиции тела в одну 4×4 OpenGL матрицу.
    // rotation — уже готовый Mat3 в мировых координатах (обновляется в updateDerivedData).
    // OpenGL column-major: элементы идут по столбцам, поэтому транспонируем при заполнении.
    const Mat3& r = body.rotation;
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
        drawUnitSphere();
    } else {
        glScalef(body.halfExtents.x, body.halfExtents.y, body.halfExtents.z);
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

// Переключаемся в 2D для рисования HUD. Паша, тут важно всё делать по порядку:
// 1) PushMatrix сохраняет обе матрицы (3D состояние не теряется).
// 2) glOrtho: пиксели вместо 3D-координат, (0,0) = левый верхний угол экрана.
// 3) Выключаем depth/lighting/fog/cull — в 2D они только мешают.
// 4) Рисуем подсказки и банер PAUSED.
// 5) PopMatrix восстанавливает 3D. Без этого следующий кадр будет в 2D
//    и вся сцена рассыплется как карточный домик.
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
    glDisable(GL_FOG);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    const float helpX = 14.0f;
    const float helpY = 14.0f;
    const float helpWidth = 252.0f;
    const float helpHeight = 188.0f;
    const float helpTextScale = 0.75f;
    const float helpLineGap = 22.0f;
    const float helpLineCount = 7.0f;
    const float helpBlockHeight = font_.lineHeight(helpTextScale) + helpLineGap * (helpLineCount - 1.0f);
    const float helpBaselineY =
        helpY + (helpHeight - helpBlockHeight) * 0.5f + font_.lineHeight(helpTextScale) * 0.78f;

    glColor4f(0.02f, 0.025f, 0.03f, 0.68f);
    drawRoundedRect(helpX, helpY, helpWidth, helpHeight, 8.0f);

    glColor4f(0.92f, 0.95f, 0.96f, 1.0f);
    font_.drawText(28.0f, helpBaselineY, helpTextScale, "Space  Pause");
    font_.drawText(28.0f, helpBaselineY + helpLineGap, helpTextScale, "N      Step");
    font_.drawText(28.0f, helpBaselineY + helpLineGap * 2.0f, helpTextScale, "C      Contacts");
    font_.drawText(28.0f, helpBaselineY + helpLineGap * 3.0f, helpTextScale, "Tab    Physics UI");
    font_.drawText(28.0f, helpBaselineY + helpLineGap * 4.0f, helpTextScale, "WASD   Camera");
    font_.drawText(28.0f, helpBaselineY + helpLineGap * 5.0f, helpTextScale, "Q/E    Zoom");
    font_.drawText(28.0f, helpBaselineY + helpLineGap * 6.0f, helpTextScale, "Esc    Exit");

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
    glEnable(GL_FOG);
    glEnable(GL_CULL_FACE);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void DebugRenderer::drawGui(PhysicsWorld& world) {
    if (!showGui_) return;

    int winW = 0, winH = 0;
    glfwGetWindowSize(window_, &winW, &winH);
    (void)winH;

    ImGui::SetNextWindowPos({(float)winW - 14.0f, 14.0f}, ImGuiCond_Always, {1.0f, 0.0f});
    ImGui::Begin("Physics", nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_NoMove);

    ImGui::SeparatorText("Spawn");
    ImGui::RadioButton("Box",    &spawnType_, 0); ImGui::SameLine();
    ImGui::RadioButton("Sphere", &spawnType_, 1);

    ImGui::DragFloat3("Pos",  &spawnPosition_.x,  0.1f, -20.0f, 20.0f,  "%.1f");

    ImGui::Checkbox("Static", &spawnStatic_);
    ImGui::SameLine();
    ImGui::BeginDisabled(spawnStatic_);
    ImGui::SetNextItemWidth(80.0f);
    ImGui::DragFloat("Mass", &spawnMass_, 0.05f, 0.1f, 50.0f, "%.2f");
    spawnMass_ = std::clamp(spawnMass_, 0.1f, 50.0f);
    ImGui::EndDisabled();

    if (spawnType_ == 0) {
        ImGui::DragFloat3("Size", &spawnBoxSize_.x, 0.02f, 0.1f, 10.0f, "%.2f");
        spawnBoxSize_.x = std::clamp(spawnBoxSize_.x, 0.1f, 10.0f);
        spawnBoxSize_.y = std::clamp(spawnBoxSize_.y, 0.1f, 10.0f);
        spawnBoxSize_.z = std::clamp(spawnBoxSize_.z, 0.1f, 10.0f);
    } else {
        ImGui::DragFloat("Radius", &spawnRadius_, 0.01f, 0.05f, 5.0f, "%.2f");
        spawnRadius_ = std::clamp(spawnRadius_, 0.05f, 5.0f);
    }

    const float btnHalf = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    if (ImGui::Button("Spawn", {btnHalf, 0.0f})) {
        const float mass = spawnStatic_ ? 0.0f : spawnMass_;
        // Tiny XZ jitter: без него две сферы спавнятся идеально симметрично —
        // никакой тангенциальной силы, вторая сфера просто стоит сверху не падая.
        // ±0.02 ломает симметрию и конструкция распадается естественно.
        static std::mt19937 rng{std::random_device{}()};
        std::uniform_real_distribution<float> jitter(-0.02f, 0.02f);
        const Vec3 pos = {spawnPosition_.x + jitter(rng), spawnPosition_.y, spawnPosition_.z + jitter(rng)};
        if (spawnType_ == 0)
            world.createBox(mass, spawnBoxSize_, pos);
        else
            world.createSphere(mass, spawnRadius_, pos);
    }
    ImGui::SameLine();
    if (ImGui::Button("Delete last", {btnHalf, 0.0f})) {
        world.deleteLastBody();
    }

    ImGui::SeparatorText("World");
    ImGui::DragFloat3("Gravity",        &world.gravity.x,              0.1f,  -30.0f, 30.0f,  "%.1f");
    ImGui::DragFloat("Ground friction", &world.groundFriction,         0.01f,   0.0f,  2.0f,  "%.2f");
    // Жёсткий clamp: при friction > 2 sequential impulse solver расходится и объекты улетают вверх.
    world.groundFriction = std::clamp(world.groundFriction, 0.0f, 2.0f);
    ImGui::DragFloat("Rest velocity",   &world.restVelocityThreshold,  0.05f,   0.1f, 10.0f,  "%.2f");
    world.restVelocityThreshold = std::clamp(world.restVelocityThreshold, 0.1f, 10.0f);
    ImGui::DragInt("Solver iters",      &world.solverIterations,       1,       1,    64);
    world.solverIterations = std::clamp(world.solverIterations, 1, 64);

    ImGui::SeparatorText("Sleeping");
    ImGui::DragFloat("Velocity thr", &world.sleepThreshold, 0.005f, 0.0f, 1.0f, "%.3f");
    world.sleepThreshold = std::clamp(world.sleepThreshold, 0.0f, 1.0f);
    ImGui::DragFloat("Delay (s)",    &world.sleepDelay,     0.05f,  0.0f, 5.0f, "%.2f");
    world.sleepDelay = std::clamp(world.sleepDelay, 0.0f, 5.0f);

    ImGui::SeparatorText("Scene");
    int sleeping = 0, staticCount = 0;
    for (const RigidBody& b : world.bodies) {
        if (b.isStatic()) ++staticCount;
        else if (b.sleeping) ++sleeping;
    }
    const int dynamic = static_cast<int>(world.bodies.size()) - staticCount;
    ImGui::Text("Bodies:   %d  (static: %d)", static_cast<int>(world.bodies.size()), staticCount);
    ImGui::Text("Contacts: %d",               static_cast<int>(world.contacts.size()));
    ImGui::Text("Sleeping: %d  Active: %d",   sleeping, dynamic - sleeping);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    if (ImGui::Button("Restart scene", {-1.0f, 0.0f})) {
        resetRequested_ = true;
    }

    ImGui::End();
}
