#include "detail/collision.hpp"
#include "detail/dynamics.hpp"
#include "detail/math.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <type_traits>

namespace physeng::detail {

namespace {

constexpr float contactTolerance = 0.001f;

std::array<Vec2, 2> Axes(const Body& body) {
  return {Rotate({1, 0}, body.angle), Rotate({0, 1}, body.angle)};
}

float ProjectionRadius(BoxShape shape, const std::array<Vec2, 2>& axes, Vec2 normal) {
  return shape.halfExtents.x * std::abs(Dot(axes[0], normal)) +
         shape.halfExtents.y * std::abs(Dot(axes[1], normal));
}

std::array<Vec2, 4> Vertices(const Body& body, BoxShape shape) {
  const Vec2 x = Rotate({shape.halfExtents.x, 0}, body.angle);
  const Vec2 y = Rotate({0, shape.halfExtents.y}, body.angle);
  return {Sub(Sub(body.position, x), y),
          Add(Sub(body.position, y), x),
          Add(Add(body.position, x), y),
          Add(Sub(body.position, x), y)};
}

int Clip(std::array<Vec2, 2>& points, int count, Vec2 normal, float offset) {
  if (count == 0) {
    return 0;
  }
  const auto original = points;
  const float d0 = Dot(normal, original[0]) - offset;
  if (count == 1) {
    return d0 <= 0.0f ? 1 : 0;
  }
  const float d1 = Dot(normal, original[1]) - offset;
  int result = 0;
  if (d0 <= 0.0f) {
    points[result++] = original[0];
  }
  if (d1 <= 0.0f) {
    points[result++] = original[1];
  }
  if ((d0 < 0.0f && d1 > 0.0f) || (d0 > 0.0f && d1 < 0.0f)) {
    points[result++] = Add(original[0], Scale(Sub(original[1], original[0]), d0 / (d0 - d1)));
  }
  return result;
}

float EffectiveMass(const Body& a, const Body& b, Vec2 offsetA, Vec2 offsetB, Vec2 direction) {
  const float ca = Cross(offsetA, direction), cb = Cross(offsetB, direction);
  return a.inverseMass + b.inverseMass + ca * ca * InverseInertia(a) + cb * cb * InverseInertia(b);
}

void PairImpulse(Body& a, Body& b, Vec2 offsetA, Vec2 offsetB, Vec2 impulse) {
  ImpulseAtOffset(a, Scale(impulse, -1.0f), offsetA);
  ImpulseAtOffset(b, impulse, offsetB);
}
} // namespace

std::optional<Contact>
ContactBetween(const Body& a, CircleShape sa, const Body& b, CircleShape sb) {
  const Vec2 delta = Sub(b.position, a.position);
  const float distance = Length(delta);
  const float penetration = sa.radius + sb.radius - distance;
  if (penetration < -contactTolerance) {
    return std::nullopt;
  }
  const Vec2 normal = distance > 0.0f ? Scale(delta, 1.0f / distance) : Vec2{1, 0};
  const Vec2 point = Scale(
      Add(Add(a.position, Scale(normal, sa.radius)), Sub(b.position, Scale(normal, sb.radius))),
      0.5f);
  return Contact{normal, std::max(penetration, 0.0f), {point, {}}, 1};
}

std::optional<Contact> ContactBetween(const Body& a, BoxShape sa, const Body& b, BoxShape sb) {
  const auto axesA = Axes(a), axesB = Axes(b);
  const Vec2 delta = Sub(b.position, a.position);
  const std::array<Vec2, 4> candidates{axesA[0], axesA[1], axesB[0], axesB[1]};
  float penetration = std::numeric_limits<float>::max();
  int referenceAxis = 0;
  Vec2 normal;
  for (int i = 0; i < 4; ++i) {
    const float separation = Dot(delta, candidates[i]);
    const float overlap = ProjectionRadius(sa, axesA, candidates[i]) +
                          ProjectionRadius(sb, axesB, candidates[i]) - std::abs(separation);
    if (overlap < -contactTolerance) {
      return std::nullopt;
    }
    if (overlap < penetration) {
      penetration = overlap;
      referenceAxis = i;
      normal = Scale(candidates[i], separation >= 0.0f ? 1.0f : -1.0f);
    }
  }

  const bool referenceA = referenceAxis < 2;
  const Body& reference = referenceA ? a : b;
  const Body& incident = referenceA ? b : a;
  const BoxShape referenceShape = referenceA ? sa : sb;
  const BoxShape incidentShape = referenceA ? sb : sa;
  const auto referenceAxes = referenceA ? axesA : axesB;
  const auto incidentAxes = referenceA ? axesB : axesA;
  const int axis = referenceAxis % 2;
  const Vec2 referenceNormal = referenceA ? normal : Scale(normal, -1.0f);
  const Vec2 tangent = referenceAxes[1 - axis];
  const float normalExtent =
      axis == 0 ? referenceShape.halfExtents.x : referenceShape.halfExtents.y;
  const float tangentExtent =
      axis == 0 ? referenceShape.halfExtents.y : referenceShape.halfExtents.x;
  const Vec2 faceCenter = Add(reference.position, Scale(referenceNormal, normalExtent));

  const int incidentAxis = std::abs(Dot(incidentAxes[0], referenceNormal)) >
                                   std::abs(Dot(incidentAxes[1], referenceNormal))
                               ? 0
                               : 1;
  const Vec2 incidentNormal =
      Scale(incidentAxes[incidentAxis],
            Dot(incidentAxes[incidentAxis], referenceNormal) > 0.0f ? -1.0f : 1.0f);
  const float incidentExtent =
      incidentAxis == 0 ? incidentShape.halfExtents.x : incidentShape.halfExtents.y;
  const float edgeExtent =
      incidentAxis == 0 ? incidentShape.halfExtents.y : incidentShape.halfExtents.x;
  const Vec2 incidentCenter = Add(incident.position, Scale(incidentNormal, incidentExtent));
  const Vec2 edge = Scale(incidentAxes[1 - incidentAxis], edgeExtent);
  std::array<Vec2, 2> points{Sub(incidentCenter, edge), Add(incidentCenter, edge)};
  int count = Clip(points, 2, tangent, Dot(tangent, faceCenter) + tangentExtent);
  count = Clip(points, count, Scale(tangent, -1.0f), -Dot(tangent, faceCenter) + tangentExtent);

  Contact contact{normal, std::max(penetration, 0.0f)};
  for (int i = 0; i < count; ++i) {
    const float distance = Dot(referenceNormal, Sub(points[i], faceCenter));
    if (distance <= contactTolerance) {
      contact.points[contact.pointCount++] =
          Sub(points[i], Scale(referenceNormal, distance * 0.5f));
    }
  }
  if (contact.pointCount == 0) {
    return std::nullopt;
  }
  return contact;
}

std::optional<Contact> ContactBetween(const Body& a, CircleShape sa, const Body& b, BoxShape sb) {
  const Vec2 local = Rotate(Sub(a.position, b.position), -b.angle);
  Vec2 closest{std::clamp(local.x, -sb.halfExtents.x, sb.halfExtents.x),
               std::clamp(local.y, -sb.halfExtents.y, sb.halfExtents.y)};
  const Vec2 towardBox = Sub(closest, local);
  const float distance = Length(towardBox);
  Vec2 localNormal;
  float penetration;
  if (distance > 0.0f) {
    if (distance > sa.radius + contactTolerance) {
      return std::nullopt;
    }
    localNormal = Scale(towardBox, 1.0f / distance);
    penetration = std::max(sa.radius - distance, 0.0f);
  } else {
    const float gapX = sb.halfExtents.x - std::abs(local.x);
    const float gapY = sb.halfExtents.y - std::abs(local.y);
    if (gapX < gapY) {
      localNormal = {local.x >= 0 ? -1.0f : 1.0f, 0};
      closest.x = local.x >= 0 ? sb.halfExtents.x : -sb.halfExtents.x;
      penetration = gapX + sa.radius;
    } else {
      localNormal = {0, local.y >= 0 ? -1.0f : 1.0f};
      closest.y = local.y >= 0 ? sb.halfExtents.y : -sb.halfExtents.y;
      penetration = gapY + sa.radius;
    }
  }
  const Vec2 normal = Rotate(localNormal, b.angle);
  const Vec2 boxPoint = Add(b.position, Rotate(closest, b.angle));
  const Vec2 point = Scale(Add(boxPoint, Add(a.position, Scale(normal, sa.radius))), 0.5f);
  return Contact{normal, penetration, {point, {}}, 1};
}

std::optional<Contact> ContactBetween(const Body& a, BoxShape sa, const Body& b, CircleShape sb) {
  auto contact = ContactBetween(b, sb, a, sa);
  if (contact) {
    contact->normal = Scale(contact->normal, -1.0f);
  }
  return contact;
}

void ResolveContact(Body& a,
                    Body& b,
                    Contact contact,
                    Material materialA,
                    Material materialB,
                    float restitutionThreshold,
                    const Body& impactA,
                    const Body& impactB) {
  const float weight = a.inverseMass + b.inverseMass;
  if (weight == 0.0f || contact.pointCount == 0) {
    return;
  }
  std::array<Vec2, 2> offsetA, offsetB;
  std::array<float, 2> velocityError{}, effectiveMass{}, normalImpulses{};
  for (int i = 0; i < contact.pointCount; ++i) {
    offsetA[i] = Sub(contact.points[i], a.position);
    offsetB[i] = Sub(contact.points[i], b.position);
    const float speed =
        Dot(Sub(PointVelocity(b, offsetB[i]), PointVelocity(a, offsetA[i])), contact.normal);
    const float incomingSpeed =
        Dot(Sub(PointVelocity(impactB, offsetB[i]), PointVelocity(impactA, offsetA[i])),
            contact.normal);
    const float bounceSpeed =
        -incomingSpeed > restitutionThreshold
            ? -std::max(materialA.restitution, materialB.restitution) * incomingSpeed
            : 0.0f;
    // Keep the original impact target; later solver passes must not erase restitution.
    velocityError[i] = speed - bounceSpeed;
    effectiveMass[i] = EffectiveMass(a, b, offsetA[i], offsetB[i], contact.normal);
  }

  if (contact.pointCount == 2) {
    const float coupledMass =
        weight +
        InverseInertia(a) * Cross(offsetA[0], contact.normal) * Cross(offsetA[1], contact.normal) +
        InverseInertia(b) * Cross(offsetB[0], contact.normal) * Cross(offsetB[1], contact.normal);
    const float determinant = effectiveMass[0] * effectiveMass[1] - coupledMass * coupledMass;
    if (determinant > 1.0e-5f * effectiveMass[0] * effectiveMass[1]) {
      // Contact impulses can push surfaces apart, but cannot pull them together.
      const float j0 =
          (coupledMass * velocityError[1] - effectiveMass[1] * velocityError[0]) / determinant;
      const float j1 =
          (coupledMass * velocityError[0] - effectiveMass[0] * velocityError[1]) / determinant;
      if (j0 >= 0.0f && j1 >= 0.0f) {
        normalImpulses = {j0, j1};
      } else {
        const float single0 = std::max(-velocityError[0] / effectiveMass[0], 0.0f);
        const float single1 = std::max(-velocityError[1] / effectiveMass[1], 0.0f);
        if (coupledMass * single0 + velocityError[1] >= 0.0f) {
          normalImpulses[0] = single0;
        } else if (coupledMass * single1 + velocityError[0] >= 0.0f) {
          normalImpulses[1] = single1;
        }
      }
    } else {
      // Near-duplicate points or fixed rotations: one midpoint is sufficient.
      offsetA[0] = Scale(Add(offsetA[0], offsetA[1]), 0.5f);
      offsetB[0] = Scale(Add(offsetB[0], offsetB[1]), 0.5f);
      const float speed =
          Dot(Sub(PointVelocity(b, offsetB[0]), PointVelocity(a, offsetA[0])), contact.normal);
      const float incomingSpeed =
          Dot(Sub(PointVelocity(impactB, offsetB[0]), PointVelocity(impactA, offsetA[0])),
              contact.normal);
      const float bounceSpeed =
          -incomingSpeed > restitutionThreshold
              ? -std::max(materialA.restitution, materialB.restitution) * incomingSpeed
              : 0.0f;
      normalImpulses[0] = std::max((bounceSpeed - speed) /
                                       EffectiveMass(a, b, offsetA[0], offsetB[0], contact.normal),
                                   0.0f);
      contact.pointCount = 1;
    }
  } else {
    normalImpulses[0] = std::max(-velocityError[0] / effectiveMass[0], 0.0f);
  }
  for (int i = 0; i < contact.pointCount; ++i) {
    PairImpulse(a, b, offsetA[i], offsetB[i], Scale(contact.normal, normalImpulses[i]));
  }

  const Vec2 tangent = Perpendicular(contact.normal);
  const double friction = std::sqrt(static_cast<double>(materialA.friction) * materialB.friction);
  for (int i = 0; i < contact.pointCount; ++i) {
    const float slip =
        Dot(Sub(PointVelocity(b, offsetB[i]), PointVelocity(a, offsetA[i])), tangent);
    const double limit = friction * normalImpulses[i];
    const float j = static_cast<float>(std::clamp(
        -static_cast<double>(slip) / EffectiveMass(a, b, offsetA[i], offsetB[i], tangent),
        -limit,
        limit));
    PairImpulse(a, b, offsetA[i], offsetB[i], Scale(tangent, j));
  }

  // Remove overlap separately from velocity so correction does not add bounce energy.
  const Vec2 correction =
      Scale(contact.normal, std::max(contact.penetration - 0.0001f, 0.0f) / weight);
  a.position = Sub(a.position, Scale(correction, a.inverseMass));
  b.position = Add(b.position, Scale(correction, b.inverseMass));
}

void ResolveFloor(Body& body,
                  const Shape& shape,
                  float floorY,
                  Material material,
                  Material floorMaterial,
                  float restitutionThreshold,
                  const Body& impactBody) {
  if (body.inverseMass == 0.0f) {
    return;
  }
  Contact contact{{0, -1}, 0};
  std::visit(
      [&](const auto& value) {
        if constexpr (std::is_same_v<std::decay_t<decltype(value)>, CircleShape>) {
          const float bottom = body.position.y - value.radius;
          if (bottom > floorY + contactTolerance) {
            return;
          }
          contact.penetration = std::max(floorY - bottom, 0.0f);
          contact.points[0] = {body.position.x, bottom};
          contact.pointCount = 1;
        } else if constexpr (std::is_same_v<std::decay_t<decltype(value)>, PolygonShape>) {
          float bottom = std::numeric_limits<float>::infinity();
          for (Vec2 vertex : value.Vertices()) {
            bottom = std::min(bottom, body.LocalToWorld(vertex).y);
          }
          if (bottom > floorY + contactTolerance) {
            return;
          }
          contact.penetration = std::max(floorY - bottom, 0.0f);
          Vec2 left, right;
          bool found = false;
          for (Vec2 local : value.Vertices()) {
            const Vec2 vertex = body.LocalToWorld(local);
            if (vertex.y <= bottom + contactTolerance) {
              if (!found) {
                left = right = vertex;
                found = true;
              }
              if (vertex.x < left.x) {
                left = vertex;
              }
              if (vertex.x > right.x) {
                right = vertex;
              }
            }
          }
          if (found) {
            contact.points = {left, right};
            contact.pointCount = Length(Sub(right, left)) > contactTolerance ? 2 : 1;
          }
        } else {
          const auto vertices = Vertices(body, value);
          float bottom = vertices[0].y;
          for (Vec2 vertex : vertices) {
            bottom = std::min(bottom, vertex.y);
          }
          if (bottom > floorY + contactTolerance) {
            return;
          }
          contact.penetration = std::max(floorY - bottom, 0.0f);
          for (Vec2 vertex : vertices) {
            if (vertex.y <= bottom + contactTolerance && contact.pointCount < 2) {
              contact.points[contact.pointCount++] = vertex;
            }
          }
        }
      },
      shape);
  Body floor;
  floor.inverseMass = 0.0f;
  ResolveContact(
      body, floor, contact, material, floorMaterial, restitutionThreshold, impactBody, floor);
}

} // namespace physeng::detail
