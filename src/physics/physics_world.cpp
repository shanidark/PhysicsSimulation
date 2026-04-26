#include "physics/physics_world.h"

#include "physics/contact_solver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace {
Vec3 boxAxis(const RigidBody& body, int axisIndex) {
    const Mat3 rotation = toMat3(body.orientation);
    return normalized(Vec3 {
        rotation.m[0][axisIndex],
        rotation.m[1][axisIndex],
        rotation.m[2][axisIndex],
    });
}

float boxHalfExtent(const RigidBody& body, int axisIndex) {
    if (axisIndex == 0) {
        return body.halfExtents.x;
    }
    if (axisIndex == 1) {
        return body.halfExtents.y;
    }
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
        worldCorners[i] = body.position + rotate(body.orientation, localCorners[i]);
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
}

RigidBody& PhysicsWorld::createBox(float mass, Vec3 size, Vec3 position) {
    RigidBody body;
    body.position = position;
    body.setBox(mass, size);
    bodies.push_back(body);
    return bodies.back();
}

RigidBody& PhysicsWorld::createSphere(float mass, float radius, Vec3 position) {
    RigidBody body;
    body.position = position;
    body.setSphere(mass, radius);
    bodies.push_back(body);
    return bodies.back();
}

void PhysicsWorld::step(float dt) {
    for (RigidBody& body : bodies) {
        if (!body.isStatic) {
            body.applyForce(gravity * (1.0f / body.inverseMass));
        }

        body.integrate(dt);
    }

    generateContacts();

    ContactSolver solver;
    solver.solve(contacts, solverIterations);

    applyRollingResistance(dt);
}

void PhysicsWorld::generateContacts() {
    contacts.clear();

    for (RigidBody& body : bodies) {
        if (!body.isStatic) {
            if (body.shape == ShapeType::Box) {
                generateBoxGroundContacts(body);
            } else if (body.shape == ShapeType::Sphere) {
                generateSphereGroundContact(body);
            }
        }
    }

    for (std::size_t i = 0; i < bodies.size(); ++i) {
        for (std::size_t j = i + 1; j < bodies.size(); ++j) {
            RigidBody& a = bodies[i];
            RigidBody& b = bodies[j];
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

void PhysicsWorld::applyRollingResistance(float dt) {
    for (const Contact& contact : contacts) {
        RigidBody* body = contact.a;
        if (!body || body->isStatic || body->shape != ShapeType::Sphere || contact.b != nullptr) {
            continue;
        }

        if (dot(contact.normal, {0.0f, 1.0f, 0.0f}) < 0.95f) {
            continue;
        }

        const float scale = std::pow(body->rollingResistance, dt);
        body->linearVelocity.x *= scale;
        body->linearVelocity.z *= scale;
        body->angularVelocity *= scale;

        if (std::abs(body->linearVelocity.x) < 0.015f) {
            body->linearVelocity.x = 0.0f;
        }
        if (std::abs(body->linearVelocity.z) < 0.015f) {
            body->linearVelocity.z = 0.0f;
        }
        if (lengthSquared(body->angularVelocity) < 0.0004f) {
            body->angularVelocity = {};
        }
    }
}

void PhysicsWorld::generateBoxGroundContacts(RigidBody& body) {
    for (Vec3 worldCorner : boxCorners(body)) {
        if (worldCorner.y >= groundY) {
            continue;
        }

        Contact contact;
        contact.a = &body;
        contact.point = {worldCorner.x, groundY, worldCorner.z};
        contact.normal = {0.0f, 1.0f, 0.0f};
        contact.penetration = groundY - worldCorner.y;
        contact.restitution = body.restitution;
        contact.friction = 0.65f;
        contacts.push_back(contact);
    }
}

void PhysicsWorld::generateBoxBoxContact(RigidBody& a, RigidBody& b) {
    Vec3 bestAxis {};
    float bestPenetration = std::numeric_limits<float>::max();
    const Vec3 centerDelta = a.position - b.position;

    const auto testAxis = [&](Vec3 axis) {
        if (lengthSquared(axis) <= 0.000001f) {
            return true;
        }

        axis = normalized(axis);
        const float distance = std::abs(dot(centerDelta, axis));
        const float penetration = projectedBoxRadius(a, axis) + projectedBoxRadius(b, axis) - distance;
        if (penetration < 0.0f) {
            return false;
        }

        if (penetration < bestPenetration) {
            bestPenetration = penetration;
            bestAxis = dot(axis, centerDelta) >= 0.0f ? axis : -axis;
        }

        return true;
    };

    for (int i = 0; i < 3; ++i) {
        if (!testAxis(boxAxis(a, i))) {
            return;
        }
        if (!testAxis(boxAxis(b, i))) {
            return;
        }
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!testAxis(cross(boxAxis(a, i), boxAxis(b, j)))) {
                return;
            }
        }
    }

    if (bestPenetration == std::numeric_limits<float>::max()) {
        return;
    }

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
        const Vec3 pointOnA = boxSupportPoint(a, -bestAxis);
        const Vec3 pointOnB = boxSupportPoint(b, bestAxis);
        points.push_back((pointOnA + pointOnB) * 0.5f);
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
        contact.penetration = bestPenetration;
        contact.correctionWeight = 1.0f / static_cast<float>(contactCount);
        contact.restitution = std::min(a.restitution, b.restitution);
        contact.friction = 0.65f;
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
        if (std::abs(projected - clamped) > 0.0001f) {
            centerInsideBox = false;
        }
        closestPoint += axis * clamped;
    }

    Vec3 normal {};
    float penetration = 0.0f;

    if (centerInsideBox) {
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
        if (distanceSquared >= sphere.radius * sphere.radius) {
            return;
        }

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
    contact.friction = 0.55f;
    contacts.push_back(contact);
}

void PhysicsWorld::generateSphereGroundContact(RigidBody& body) {
    const float bottom = body.position.y - body.radius;
    if (bottom >= groundY) {
        return;
    }

    Contact contact;
    contact.a = &body;
    contact.point = {body.position.x, groundY, body.position.z};
    contact.normal = {0.0f, 1.0f, 0.0f};
    contact.penetration = groundY - bottom;
    contact.restitution = body.restitution;
    contact.friction = 0.45f;
    contacts.push_back(contact);
}

void PhysicsWorld::generateSphereSphereContact(RigidBody& a, RigidBody& b) {
    const Vec3 delta = a.position - b.position;
    const float distanceSquared = lengthSquared(delta);
    const float radiusSum = a.radius + b.radius;

    if (distanceSquared >= radiusSum * radiusSum) {
        return;
    }

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
    contact.friction = 0.5f;
    contacts.push_back(contact);
}
