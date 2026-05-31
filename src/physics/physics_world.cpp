#include "physics/physics_world.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace {
Vec3 boxAxis(const RigidBody& body, int axisIndex) {
    return {
        body.rotation.m[0][axisIndex],
        body.rotation.m[1][axisIndex],
        body.rotation.m[2][axisIndex],
    };
}

float boxHalfExtent(const RigidBody& body, int axisIndex) {
    if (axisIndex == 0) return body.halfExtents.x;
    if (axisIndex == 1) return body.halfExtents.y;
    return body.halfExtents.z;
}

float projectedBoxRadius(const RigidBody& body, Vec3 axis) {
    float radius = 0.0f;
    for (int i = 0; i < 3; ++i) {
        radius += boxHalfExtent(body, i) * std::abs(dot(boxAxis(body, i), axis));
    }
    return radius;
}

Vec3 boxSupportPoint(const RigidBody& body, Vec3 direction) {
    Vec3 point = body.position;
    for (int i = 0; i < 3; ++i) {
        const Vec3 axis = boxAxis(body, i);
        const float sign = dot(axis, direction) >= 0.0f ? 1.0f : -1.0f;
        point += axis * (boxHalfExtent(body, i) * sign);
    }
    return point;
}

std::array<Vec3, 8> boxCorners(const RigidBody& body) {
    const Vec3 h = body.halfExtents;
    const std::array<Vec3, 8> localCorners {{
        {-h.x, -h.y, -h.z},
        { h.x, -h.y, -h.z},
        {-h.x, -h.y,  h.z},
        { h.x, -h.y,  h.z},
        {-h.x,  h.y, -h.z},
        { h.x,  h.y, -h.z},
        {-h.x,  h.y,  h.z},
        { h.x,  h.y,  h.z},
    }};

    std::array<Vec3, 8> worldCorners {};
    for (std::size_t i = 0; i < localCorners.size(); ++i) {
        worldCorners[i] = body.position + body.rotation * localCorners[i];
    }
    return worldCorners;
}

bool pointInsideBox(const RigidBody& box, Vec3 point, float tolerance = 0.001f) {
    const Vec3 delta = point - box.position;
    for (int i = 0; i < 3; ++i) {
        const float projected = dot(delta, boxAxis(box, i));
        if (std::abs(projected) > boxHalfExtent(box, i) + tolerance) {
            return false;
        }
    }
    return true;
}

float pointBoxPenetrationAlongAxis(const RigidBody& box, Vec3 point, Vec3 normal) {
    const Vec3 delta = point - box.position;
    float support = 0.0f;
    for (int i = 0; i < 3; ++i) {
        support += boxHalfExtent(box, i) * std::abs(dot(boxAxis(box, i), normal));
    }
    return support - dot(delta, normal);
}

bool tooCloseToExistingPoint(const std::vector<Vec3>& points, Vec3 candidate) {
    for (Vec3 point : points) {
        if (lengthSquared(point - candidate) < 0.01f) {
            return true;
        }
    }
    return false;
}

float combinedFriction(float a, float b) {
    return std::sqrt(a * b);
}

// Паша, налей водочки под шашлычокс и слушай.
// Имеем два отрезка в 3D — рёбра двух боксов. Нужно найти ближайшие точки между ними.
// Сначала находим "опорные точки" рёбер: для каждого ребра берём центр тела и смещаемся
// по всем остальным осям в сторону нормали контакта (туда где ребро ближайшее к другому боксу).
// Потом параметрически ищем ближайшие точки двух прямых:
//   r = pointA - pointB, dotAB = cos угла между рёбрами.
//   denom = sin²(угол) — если рёбра параллельны, denom ≈ 0, берём s=t=0.
//   s и t — параметры на отрезках, clamp чтоб не выходить за края ребра.
// Возвращаем середину между двумя ближайшими точками — это и есть точка контакта.
Vec3 closestPointOnBoxEdges(const RigidBody& a, const RigidBody& b, Vec3 normal,
                            int edgeAxisA, int edgeAxisB) {
    const Vec3 dirA = boxAxis(a, edgeAxisA);
    const Vec3 dirB = boxAxis(b, edgeAxisB);

    Vec3 pointA = a.position;
    for (int k = 0; k < 3; ++k) {
        if (k == edgeAxisA) continue;
        const Vec3 axis = boxAxis(a, k);
        const float sign = dot(axis, -normal) >= 0.0f ? 1.0f : -1.0f;
        pointA += axis * (boxHalfExtent(a, k) * sign);
    }
    Vec3 pointB = b.position;
    for (int k = 0; k < 3; ++k) {
        if (k == edgeAxisB) continue;
        const Vec3 axis = boxAxis(b, k);
        const float sign = dot(axis, normal) >= 0.0f ? 1.0f : -1.0f;
        pointB += axis * (boxHalfExtent(b, k) * sign);
    }

    const Vec3 r = pointA - pointB;
    const float dotAB = dot(dirA, dirB);
    const float denom = 1.0f - dotAB * dotAB;
    const float dA = dot(dirA, r);
    const float dB = dot(dirB, r);

    float s = 0.0f;
    float t = 0.0f;
    if (denom > 0.000001f) {
        s = (dotAB * dB - dA) / denom;
        t = (dB - dotAB * dA) / denom;
    }
    s = std::clamp(s, -boxHalfExtent(a, edgeAxisA), boxHalfExtent(a, edgeAxisA));
    t = std::clamp(t, -boxHalfExtent(b, edgeAxisB), boxHalfExtent(b, edgeAxisB));

    const Vec3 closestA = pointA + dirA * s;
    const Vec3 closestB = pointB + dirB * t;
    return (closestA + closestB) * 0.5f;
}
}

void PhysicsWorld::reset() {
    bodies.clear();
    contacts.clear();
    solver_.clearHistory();
}

void PhysicsWorld::deleteLastBody() {
    if (bodies.empty()) return;
    contacts.clear();
    solver_.clearHistory();
    bodies.pop_back();
}

size_t PhysicsWorld::createBox(float mass, Vec3 size, Vec3 position) {
    RigidBody body;
    body.position = position;
    body.setBox(mass, size);
    bodies.push_back(body);
    return bodies.size() - 1;
}

size_t PhysicsWorld::createSphere(float mass, float radius, Vec3 position) {
    RigidBody body;
    body.position = position;
    body.setSphere(mass, radius);
    bodies.push_back(body);
    return bodies.size() - 1;
}

void PhysicsWorld::step(float dt) {
    for (RigidBody& body : bodies) {
        if (body.sleeping) {
            body.clearAccumulators();
            continue;
        }
        if (!body.isStatic()) {
            body.applyForce(gravity * (1.0f / body.inverseMass));
        }
        body.integrate(dt);
    }

    generateContacts();
    resolveSleepingContacts();

    solver_.solve(contacts, solverIterations);

    applyRollingResistance(dt);
    updateSleep(dt);
}

void PhysicsWorld::generateContacts() {
    contacts.clear();

    for (RigidBody& body : bodies) {
        if (!body.isStatic()) {
            if (body.shape == ShapeType::Box) {
                generateBoxGroundContacts(body);
            } else if (body.shape == ShapeType::Sphere) {
                generateSphereGroundContact(body);
            }
        }
    }

    // Грубая фаза O(n²): каждое с каждым — тел мало, норм.
    // Два статичных тела — нечего решать. Bounding sphere: если расстояние между центрами
    // больше суммы радиусов — боксы точно не пересекаются, пропускаем дорогой SAT.
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        for (std::size_t j = i + 1; j < bodies.size(); ++j) {
            RigidBody& a = bodies[i];
            RigidBody& b = bodies[j];
            if (a.isStatic() && b.isStatic()) continue;
            const float reach = a.radius + b.radius;
            if (lengthSquared(a.position - b.position) > reach * reach) continue;
            if (a.shape == ShapeType::Box && b.shape == ShapeType::Box) {
                generateBoxBoxContact(a, b);
            } else if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Box) {
                generateSphereBoxContact(a, b);
            } else if (a.shape == ShapeType::Box && b.shape == ShapeType::Sphere) {
                generateSphereBoxContact(b, a);
            } else if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Sphere) {
                generateSphereSphereContact(a, b);
            }
        }
    }
}

// Активное тело разбудило спящее — но то может касаться ещё одного спящего, и так далее.
// Гоняем цикл пока есть изменения: волна пробуждения расползается по стопке объектов.
// В конце выбрасываем контакты где оба тела неподвижны — солвер там ничего не делает,
// но warm starting бы их будил снова следующего кадра. Порочный круг.
void PhysicsWorld::resolveSleepingContacts() {
    auto inactive = [](const RigidBody* body) {
        return !body || body->isStatic() || body->sleeping;
    };

    bool changed = true;
    for (std::size_t pass = 0; changed && pass <= bodies.size(); ++pass) {
        changed = false;
        for (const Contact& contact : contacts) {
            const bool aActive = contact.a && !contact.a->isStatic() && !contact.a->sleeping;
            const bool bActive = contact.b && !contact.b->isStatic() && !contact.b->sleeping;
            if (aActive && contact.b && contact.b->sleeping) {
                contact.b->sleeping = false;
                contact.b->sleepTimer = 0.0f;
                changed = true;
            }
            if (bActive && contact.a && contact.a->sleeping) {
                contact.a->sleeping = false;
                contact.a->sleepTimer = 0.0f;
                changed = true;
            }
        }
    }

    contacts.erase(
        std::remove_if(contacts.begin(), contacts.end(),
                       [&](const Contact& c) { return inactive(c.a) && inactive(c.b); }),
        contacts.end());
}

void PhysicsWorld::applyRollingResistance(float dt) {
    for (const Contact& contact : contacts) {
        RigidBody* body = contact.a;
        if (!body || body->isStatic() || body->sleeping || body->shape != ShapeType::Sphere || contact.b != nullptr) {
            continue;
        }

        if (dot(contact.normal, {0.0f, 1.0f, 0.0f}) < 0.95f) {
            continue;
        }

        const float scale = std::pow(body->rollingResistance, dt);
        body->linearVelocity.x *= scale;
        body->linearVelocity.z *= scale;
        body->angularVelocity *= scale;

        if (std::abs(body->linearVelocity.x) < 0.015f) body->linearVelocity.x = 0.0f;
        if (std::abs(body->linearVelocity.z) < 0.015f) body->linearVelocity.z = 0.0f;
        if (lengthSquared(body->angularVelocity) < 0.0004f) body->angularVelocity = {};
    }
}

void PhysicsWorld::updateSleep(float dt) {
    for (RigidBody& body : bodies) {
        if (body.isStatic()) continue;
        const float v2 = lengthSquared(body.linearVelocity) + lengthSquared(body.angularVelocity);
        if (v2 < sleepThreshold * sleepThreshold) {
            body.sleepTimer += dt;
            if (body.sleepTimer >= sleepDelay) {
                body.sleeping = true;
                body.linearVelocity = {};
                body.angularVelocity = {};
            }
        } else {
            body.sleepTimer = 0.0f;
            body.sleeping = false;
        }
    }
}

// Считаем сколько угловых корнеров проникло в пол и делим вес коррекции между ними.
// Без этого каждый из 4 углов толкал бы на полную — 4× перекоррекция, коробка дрожала и уползала.
void PhysicsWorld::generateBoxGroundContacts(RigidBody& body) {
    const auto corners = boxCorners(body);
    int count = 0;
    for (Vec3 c : corners) {
        if (c.y < groundY) ++count;
    }
    if (count == 0) return;

    const float weight = 1.0f / static_cast<float>(count);
    for (Vec3 worldCorner : corners) {
        if (worldCorner.y >= groundY) continue;

        Contact contact;
        contact.a = &body;
        contact.correctionWeight = weight;
        contact.point = {worldCorner.x, groundY, worldCorner.z};
        contact.normal = {0.0f, 1.0f, 0.0f};
        contact.penetration = groundY - worldCorner.y;
        contact.restitution = body.restitution;
        contact.friction = combinedFriction(body.friction, groundFriction);
        contact.restitutionThreshold = restVelocityThreshold;
        contacts.push_back(contact);
    }
}

void PhysicsWorld::generateBoxBoxContact(RigidBody& a, RigidBody& b) {
    Vec3 bestAxis {};
    float bestPenetration = std::numeric_limits<float>::max();
    int bestEdgeA = -1;
    int bestEdgeB = -1;
    const Vec3 centerDelta = a.position - b.position;

    // SAT — Separating Axis Theorem (Теорема о Разделяющей Оси).
    // Два выпуклых тела НЕ пересекаются, если найдётся хоть одна ось,
    // на которую их проекции не перекрываются.
    // Для двух боксов проверяем 15 кандидатов: 3 нормали граней A + 3 нормали граней B +
    // 9 cross-products пар рёбер (3×3).
    // Если хоть одна ось показала зазор — return, контакта нет.
    // Ось с МИНИМАЛЬНЫМ перекрытием из всех 15 = нормаль контакта + глубина проникновения.
    //
    // Как проверить, совокупились ли двое в темноте: если хоть с одной стороны между ними зазор —
    // ещё нет. Если отовсюду впритык, без просвета — всё, контакт состоялся.
    const auto testAxis = [&](Vec3 axis, int edgeA, int edgeB) {
        if (lengthSquared(axis) <= 0.000001f) return true;

        axis = normalized(axis);
        const float distance = std::abs(dot(centerDelta, axis));
        const float penetration = projectedBoxRadius(a, axis) + projectedBoxRadius(b, axis) - distance;
        if (penetration < 0.0f) return false; // зазор найден — расходимся

        if (penetration < bestPenetration) {
            bestPenetration = penetration;
            bestAxis = dot(axis, centerDelta) >= 0.0f ? axis : -axis; // всегда от b к a
            bestEdgeA = edgeA;
            bestEdgeB = edgeB;
        }
        return true;
    };

    for (int i = 0; i < 3; ++i) {
        if (!testAxis(boxAxis(a, i), -1, -1)) return;
        if (!testAxis(boxAxis(b, i), -1, -1)) return;
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!testAxis(cross(boxAxis(a, i), boxAxis(b, j)), i, j)) return;
        }
    }

    if (bestPenetration == std::numeric_limits<float>::max()) return;

    // Ищем точки контакта.
    // 1) Углы A внутри B и углы B внутри A — классическое проникновение угол-грань.
    // 2) Если таких нет — контакт рёбро-в-ребро. Берём ближайшую точку между рёбрами.
    // Сортируем по глубине проникновения, берём топ-4 — солвер больше не переваривает.
    std::vector<Vec3> points;
    points.reserve(8);

    for (Vec3 point : boxCorners(a)) {
        if (pointInsideBox(b, point) && !tooCloseToExistingPoint(points, point)) {
            points.push_back(point);
        }
    }
    for (Vec3 point : boxCorners(b)) {
        if (pointInsideBox(a, point) && !tooCloseToExistingPoint(points, point)) {
            points.push_back(point);
        }
    }

    if (points.empty()) {
        if (bestEdgeA >= 0 && bestEdgeB >= 0) {
            points.push_back(closestPointOnBoxEdges(a, b, bestAxis, bestEdgeA, bestEdgeB));
        } else {
            const Vec3 pointOnA = boxSupportPoint(a, -bestAxis);
            const Vec3 pointOnB = boxSupportPoint(b, bestAxis);
            points.push_back((pointOnA + pointOnB) * 0.5f);
        }
    }

    std::sort(points.begin(), points.end(), [&](Vec3 lhs, Vec3 rhs) {
        const float lhsDepth = std::min(
            pointBoxPenetrationAlongAxis(a, lhs, -bestAxis),
            pointBoxPenetrationAlongAxis(b, lhs, bestAxis));
        const float rhsDepth = std::min(
            pointBoxPenetrationAlongAxis(a, rhs, -bestAxis),
            pointBoxPenetrationAlongAxis(b, rhs, bestAxis));
        return lhsDepth > rhsDepth;
    });

    const std::size_t contactCount = std::min<std::size_t>(points.size(), 4);
    for (std::size_t i = 0; i < contactCount; ++i) {
        Contact contact;
        contact.a = &a;
        contact.b = &b;
        contact.normal = bestAxis;
        contact.point = points[i];
        const float pointPenetration = std::min(
            pointBoxPenetrationAlongAxis(a, points[i], -bestAxis),
            pointBoxPenetrationAlongAxis(b, points[i], bestAxis));
        contact.penetration = std::max(pointPenetration, 0.0f);
        contact.correctionWeight = 1.0f / static_cast<float>(contactCount);
        contact.restitution = std::min(a.restitution, b.restitution);
        contact.friction = combinedFriction(a.friction, b.friction);
        contact.restitutionThreshold = restVelocityThreshold;
        contacts.push_back(contact);
    }
}

void PhysicsWorld::generateSphereBoxContact(RigidBody& sphere, RigidBody& box) {
    const Vec3 centerDelta = sphere.position - box.position;
    Vec3 closestPoint = box.position;
    bool centerInsideBox = true;

    for (int i = 0; i < 3; ++i) {
        const Vec3 axis = boxAxis(box, i);
        const float halfExtent = boxHalfExtent(box, i);
        const float projected = dot(centerDelta, axis);
        const float clamped = std::clamp(projected, -halfExtent, halfExtent);
        if (std::abs(projected - clamped) > 0.0001f) centerInsideBox = false;
        closestPoint += axis * clamped;
    }

    Vec3 normal {};
    float penetration = 0.0f;

    if (centerInsideBox) {
        // Сфера целиком внутри бокса — это как засунуть слишком далеко и не найти выход.
        // Находим ближайшую грань (минимальное расстояние до каждой из 6) — это выход.
        // Нормаль наружу от этой грани, closestPoint на ней.
        int closestFaceAxis = 0;
        float closestFaceDistance = std::numeric_limits<float>::max();
        float closestFaceSign = 1.0f;

        for (int i = 0; i < 3; ++i) {
            const Vec3 axis = boxAxis(box, i);
            const float projected = dot(centerDelta, axis);
            const float distanceToPositiveFace = boxHalfExtent(box, i) - projected;
            const float distanceToNegativeFace = boxHalfExtent(box, i) + projected;

            if (distanceToPositiveFace < closestFaceDistance) {
                closestFaceDistance = distanceToPositiveFace;
                closestFaceAxis = i;
                closestFaceSign = 1.0f;
            }
            if (distanceToNegativeFace < closestFaceDistance) {
                closestFaceDistance = distanceToNegativeFace;
                closestFaceAxis = i;
                closestFaceSign = -1.0f;
            }
        }

        normal = boxAxis(box, closestFaceAxis) * closestFaceSign;
        closestPoint = sphere.position + normal * closestFaceDistance;
        penetration = sphere.radius + closestFaceDistance;
    } else {
        const Vec3 delta = sphere.position - closestPoint;
        const float distanceSquared = lengthSquared(delta);
        if (distanceSquared >= sphere.radius * sphere.radius) return;

        const float distance = std::sqrt(std::max(distanceSquared, 0.000001f));
        normal = distance > 0.0001f ? delta / distance : Vec3 {0.0f, 1.0f, 0.0f};
        penetration = sphere.radius - distance;
    }

    Contact contact;
    contact.a = &sphere;
    contact.b = &box;
    contact.normal = normal;
    contact.point = closestPoint;
    contact.penetration = penetration;
    contact.restitution = std::min(sphere.restitution, box.restitution);
    contact.friction = combinedFriction(sphere.friction, box.friction);
    contact.restitutionThreshold = restVelocityThreshold;
    contacts.push_back(contact);
}

void PhysicsWorld::generateSphereGroundContact(RigidBody& body) {
    const float bottom = body.position.y - body.radius;
    if (bottom >= groundY) return;

    Contact contact;
    contact.a = &body;
    contact.point = {body.position.x, groundY, body.position.z};
    contact.normal = {0.0f, 1.0f, 0.0f};
    contact.penetration = groundY - bottom;
    contact.restitution = body.restitution;
    contact.friction = combinedFriction(body.friction, groundFriction);
    contact.restitutionThreshold = restVelocityThreshold;
    contacts.push_back(contact);
}

void PhysicsWorld::generateSphereSphereContact(RigidBody& a, RigidBody& b) {
    const Vec3 delta = a.position - b.position;
    const float distanceSquared = lengthSquared(delta);
    const float radiusSum = a.radius + b.radius;

    if (distanceSquared >= radiusSum * radiusSum) return;

    const float distance = std::sqrt(std::max(distanceSquared, 0.000001f));
    const Vec3 normal = distance > 0.0001f ? delta / distance : Vec3 {0.0f, 1.0f, 0.0f};
    const float penetration = radiusSum - distance;

    Contact contact;
    contact.a = &a;
    contact.b = &b;
    contact.normal = normal;
    contact.point = b.position + normal * b.radius;
    contact.penetration = penetration;
    contact.restitution = std::min(a.restitution, b.restitution);
    contact.friction = combinedFriction(a.friction, b.friction);
    contact.restitutionThreshold = restVelocityThreshold;
    contacts.push_back(contact);
}
