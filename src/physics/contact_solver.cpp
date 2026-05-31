#include "physics/contact_solver.h"

#include <algorithm>

namespace {
Vec3 velocityAtPoint(const RigidBody* body, Vec3 point) {
    if (!body || body->isStatic()) return {};
    const Vec3 radius = point - body->position;
    return body->linearVelocity + cross(body->angularVelocity, radius);
}

float inverseMass(const RigidBody* body) {
    if (!body || body->isStatic()) return 0.0f;
    return body->inverseMass;
}

float impulseDenominator(const RigidBody* body, Vec3 point, Vec3 direction) {
    if (!body || body->isStatic()) return 0.0f;
    const Vec3 radius = point - body->position;
    const Vec3 angular = cross(body->inverseInertiaWorld * cross(radius, direction), radius);
    return body->inverseMass + dot(angular, direction);
}

void applyImpulse(RigidBody* body, Vec3 impulse, Vec3 point) {
    if (body && !body->isStatic()) {
        body->applyImpulse(impulse, point);
    }
}
}

void ContactSolver::solve(std::vector<Contact>& contacts, int iterations) {
    matchWithPrevious(contacts);
    warmStart(contacts);

    for (int i = 0; i < iterations; ++i) {
        for (Contact& contact : contacts) {
            solveContact(contact);
        }
    }

    for (Contact& contact : contacts) {
        correctPosition(contact);
    }

    previousContacts_ = contacts;
}

// Warm starting: ищем контакт из прошлого кадра чтоб унаследовать накопленные импульсы.
// "Тот же контакт" = те же указатели тел + точка ближе 4 см (порог 0.04 в квадрате).
// Новые тела имеют другие указатели — им чужой warm start не достанется, что правильно.
void ContactSolver::matchWithPrevious(std::vector<Contact>& contacts) {
    for (Contact& current : contacts) {
        for (const Contact& prev : previousContacts_) {
            if (current.a != prev.a || current.b != prev.b) continue;
            if (lengthSquared(current.point - prev.point) > 0.04f) continue;
            current.accumulatedNormalImpulse = prev.accumulatedNormalImpulse;
            current.accumulatedTangentImpulse = prev.accumulatedTangentImpulse;
            break;
        }
    }
}

// Применяем сохранённые импульсы как начальное приближение — warm start.
// Без этого солвер начинает с нуля и за 8 итераций не сходится, нужно было бы 80+.
// Как секс с партнёром которого уже знаешь: прелюдия короче, сразу к делу.
// tangent пересчитываем из текущей скорости — направление скольжения могло измениться.
void ContactSolver::warmStart(std::vector<Contact>& contacts) {
    for (Contact& contact : contacts) {
        if (contact.accumulatedNormalImpulse == 0.0f && contact.accumulatedTangentImpulse == 0.0f) continue;

        const Vec3 relVel = velocityAtPoint(contact.a, contact.point)
                          - velocityAtPoint(contact.b, contact.point);
        Vec3 tangent = relVel - contact.normal * dot(relVel, contact.normal);
        const float tangentLen = length(tangent);
        if (tangentLen > 0.000001f) tangent = tangent / tangentLen;

        const Vec3 impulse = contact.normal * contact.accumulatedNormalImpulse
                           + tangent * contact.accumulatedTangentImpulse;
        applyImpulse(contact.a, impulse, contact.point);
        applyImpulse(contact.b, -impulse, contact.point);
    }
}

void ContactSolver::solveContact(Contact& contact) {
    const Vec3 velocityA = velocityAtPoint(contact.a, contact.point);
    const Vec3 velocityB = velocityAtPoint(contact.b, contact.point);
    const Vec3 relativeVelocity = velocityA - velocityB;
    const float normalVelocity = dot(relativeVelocity, contact.normal);

    // denominator = 1/mA + 1/mB + вклад угловой инерции обоих тел.
    // По сути — эффективная обратная масса системы в точке контакта.
    const float denominator =
        impulseDenominator(contact.a, contact.point, contact.normal) +
        impulseDenominator(contact.b, contact.point, contact.normal);
    if (denominator <= 0.000001f) return;

    // Упругость применяем только при первом соприкосновении (accumulated == 0, warm start не принёс)
    // и только если скорость удара выше порога — иначе лежащий кубик вечно нано-подпрыгивает.
    const float restitution = (normalVelocity < -contact.restitutionThreshold
                               && contact.accumulatedNormalImpulse == 0.0f)
                                  ? contact.restitution
                                  : 0.0f;
    // rawDelta = -(1 + e)*v_n / denom: e=0 — замерли, e=1 — идеальный отскок.
    // normalVelocity < 0 означает тела сближаются — нужно толкнуть их назад.
    // Как коитус: скорость сближения определяет силу отдачи. Больше скорость — сильнее отлетят.
    const float rawDelta = -(1.0f + restitution) * normalVelocity / denominator;

    // Clamp: суммарный накопленный импульс не может быть отрицательным —
    // тела не должны тянуться друг к другу как намагниченные.
    // Применяем только разницу (appliedNormal), не весь rawDelta — иначе посчитаем дважды.
    const float prevNormal = contact.accumulatedNormalImpulse;
    contact.accumulatedNormalImpulse = std::max(0.0f, prevNormal + rawDelta);
    const float appliedNormal = contact.accumulatedNormalImpulse - prevNormal;

    if (std::abs(appliedNormal) > 0.000001f) {
        applyImpulse(contact.a, contact.normal * appliedNormal, contact.point);
        applyImpulse(contact.b, -contact.normal * appliedNormal, contact.point);
    }

    // Трение — импульс вдоль касательной (перпендикуляр нормали в направлении скольжения).
    // Конус Кулона: |трение| <= mu * |нормальный импульс|.
    // Как прижать кого-то к стене: чем сильнее давишь — тем сильнее можешь держать.
    // Если нормального давления нет — хоть какой коэффициент трения, всё равно соскользнёт.
    const Vec3 updatedRelVel =
        velocityAtPoint(contact.a, contact.point) - velocityAtPoint(contact.b, contact.point);
    Vec3 tangent = updatedRelVel - contact.normal * dot(updatedRelVel, contact.normal);
    const float tangentLength = length(tangent);
    if (tangentLength <= 0.000001f) return;

    tangent = tangent / tangentLength;
    const float tangentDenominator =
        impulseDenominator(contact.a, contact.point, tangent) +
        impulseDenominator(contact.b, contact.point, tangent);
    if (tangentDenominator <= 0.000001f) return;

    const float rawTangentDelta = -dot(updatedRelVel, tangent) / tangentDenominator;
    const float maxFriction = contact.friction * contact.accumulatedNormalImpulse;
    const float prevTangent = contact.accumulatedTangentImpulse;
    contact.accumulatedTangentImpulse = std::clamp(prevTangent + rawTangentDelta, -maxFriction, maxFriction);
    const float appliedTangent = contact.accumulatedTangentImpulse - prevTangent;

    if (std::abs(appliedTangent) > 0.000001f) {
        applyImpulse(contact.a, tangent * appliedTangent, contact.point);
        applyImpulse(contact.b, -tangent * appliedTangent, contact.point);
    }
}

// Позиционная коррекция. После солвера тела могут немного перекрываться —
// двигаем позиции напрямую, без импульсов.
// 75% от (penetration - 1мм): не убираем всё сразу чтоб не дрожало.
//
// correctionWeight: если 4 угла коробки касаются пола — каждый делает ровно 1/4 работы.
// Без weight каждый угол толкал бы на полную — 4× перекоррекция, коробка взлетала бы как пробка.
// inverseMass в знаменателе: тяжёлые тела двигаются меньше — физически правильно.
void ContactSolver::correctPosition(Contact& contact) {
    constexpr float allowedPenetration = 0.01f;
    constexpr float correctionPercent = 0.75f;

    const float totalInverseMass = inverseMass(contact.a) + inverseMass(contact.b);
    if (totalInverseMass <= 0.000001f) return;

    const float correctionDepth = std::max(contact.penetration - allowedPenetration, 0.0f);
    const Vec3 correction =
        contact.normal * (correctionDepth * correctionPercent * contact.correctionWeight / totalInverseMass);

    if (contact.a && !contact.a->isStatic()) {
        contact.a->position += correction * contact.a->inverseMass;
        contact.a->updateDerivedData();
    }
    if (contact.b && !contact.b->isStatic()) {
        contact.b->position -= correction * contact.b->inverseMass;
        contact.b->updateDerivedData();
    }
}
