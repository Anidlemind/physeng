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

void PrepareContact(ContactState& state,
                    Body& a,
                    Body& b,
                    Vec2 normal,
                    const std::array<Vec2, 2>& offsetA,
                    const std::array<Vec2, 2>& offsetB,
                    int count,
                    float threshold,
                    float restitution,
                    const Body& impactA,
                    const Body& impactB,
                    bool circles = false) {
  const bool firstPass = !state.prepared;
  bool matches = state.pointCount == count && Dot(state.normal, normal) > 0.95f;
  std::array<Vec2, 2> localA{}, localB{};
  if (!circles) {
    for (int i = 0; i < count; ++i) {
      localA[i] = Rotate(offsetA[i], -a.angle);
      localB[i] = Rotate(offsetB[i], -b.angle);
    }
    const auto distance = [&](int current, int previous) {
      const Vec2 da = Sub(localA[current], state.localAnchorA[previous]);
      const Vec2 db = Sub(localB[current], state.localAnchorB[previous]);
      return Dot(da, da) + Dot(db, db);
    };
    if (matches && count == 2 &&
        distance(0, 1) + distance(1, 0) < distance(0, 0) + distance(1, 1)) {
      std::swap(state.normalImpulse[0], state.normalImpulse[1]);
      std::swap(state.tangentImpulse[0], state.tangentImpulse[1]);
      std::swap(state.bounceSpeed[0], state.bounceSpeed[1]);
      std::swap(state.localAnchorA[0], state.localAnchorA[1]);
      std::swap(state.localAnchorB[0], state.localAnchorB[1]);
    }
    for (int i = 0; i < count && matches; ++i) {
      matches = distance(i, i) < 0.005f;
    }
  }
  if (!matches) {
    state = {};
  }
  if (firstPass || !matches) {
    for (int i = 0; i < count; ++i) {
      const float incoming =
          Dot(Sub(PointVelocity(impactB, offsetB[i]), PointVelocity(impactA, offsetA[i])), normal);
      state.bounceSpeed[i] = -incoming > threshold ? -restitution * incoming : 0.0f;
      if (firstPass && matches) {
        // Reapply the last step's support and friction impulses before improving them.
        PairImpulse(a,
                    b,
                    offsetA[i],
                    offsetB[i],
                    Add(Scale(normal, state.normalImpulse[i]),
                        Scale(Perpendicular(normal), state.tangentImpulse[i])));
      }
    }
  }
  state.normal = normal;
  state.pointCount = count;
  state.localAnchorA = localA;
  state.localAnchorB = localB;
  state.prepared = true;
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

void ResolveCircleContact(Body& a,
                          Body& b,
                          CircleShape sa,
                          CircleShape sb,
                          Material materialA,
                          Material materialB,
                          float restitutionThreshold,
                          const Body& impactA,
                          const Body& impactB,
                          ContactState* saved,
                          bool warmStartOnly) {
  ContactState scratch;
  auto& state = saved ? *saved : scratch;
  const Vec2 delta = Sub(b.position, a.position);
  const float distanceSquared = Dot(delta, delta);
  const float radii = sa.radius + sb.radius;
  const float limit = radii + contactTolerance;
  const float weight = a.inverseMass + b.inverseMass;
  if (distanceSquared > limit * limit || weight == 0.0f) {
    state = {};
    return;
  }
  const float distance = std::sqrt(distanceSquared);
  const Vec2 normal = distance > 0.0f ? Scale(delta, 1.0f / distance) : Vec2{1, 0};
  const Vec2 tangent = Perpendicular(normal);
  const float leverA = 0.5f * (distance + sa.radius - sb.radius), leverB = leverA - distance;
  const std::array<Vec2, 2> offsetA{Scale(normal, leverA), Vec2{}},
      offsetB{Scale(normal, leverB), Vec2{}};
  PrepareContact(state,
                 a,
                 b,
                 normal,
                 offsetA,
                 offsetB,
                 1,
                 restitutionThreshold,
                 std::max(materialA.restitution, materialB.restitution),
                 impactA,
                 impactB,
                 true);
  if (warmStartOnly) {
    return;
  }
  const float speed = Dot(Sub(b.velocity, a.velocity), normal);
  const float previous = state.normalImpulse[0];
  state.normalImpulse[0] = std::max(previous + (state.bounceSpeed[0] - speed) / weight, 0.0f);
  const Vec2 impulse = Scale(normal, state.normalImpulse[0] - previous);
  if (a.inverseMass > 0.0f) {
    a.velocity = Sub(a.velocity, Scale(impulse, a.inverseMass));
  }
  if (b.inverseMass > 0.0f) {
    b.velocity = Add(b.velocity, Scale(impulse, b.inverseMass));
  }
  const float inertiaA = InverseInertia(a), inertiaB = InverseInertia(b);
  const float slip = Dot(Sub(b.velocity, a.velocity), tangent) + b.angularVelocity * leverB -
                     a.angularVelocity * leverA;
  const float mass = weight + leverA * leverA * inertiaA + leverB * leverB * inertiaB;
  const double frictionLimit =
      std::sqrt(static_cast<double>(materialA.friction) * materialB.friction) *
      state.normalImpulse[0];
  const float oldFriction = state.tangentImpulse[0];
  state.tangentImpulse[0] = static_cast<float>(
      std::clamp(static_cast<double>(oldFriction) - slip / mass, -frictionLimit, frictionLimit));
  const float friction = state.tangentImpulse[0] - oldFriction;
  const Vec2 tangentialImpulse = Scale(tangent, friction);
  if (a.inverseMass > 0.0f) {
    a.velocity = Sub(a.velocity, Scale(tangentialImpulse, a.inverseMass));
    a.angularVelocity -= leverA * friction * inertiaA;
  }
  if (b.inverseMass > 0.0f) {
    b.velocity = Add(b.velocity, Scale(tangentialImpulse, b.inverseMass));
    b.angularVelocity += leverB * friction * inertiaB;
  }
  const float penetration = std::max(radii - distance, 0.0f);
  // Relax moving pairs instead of teleporting the full overlap into their neighbors.
  // Static surfaces still project fully, so enclosure walls remain solid.
  const float relaxation = a.inverseMass > 0.0f && b.inverseMass > 0.0f ? 0.25f : 1.0f;
  const Vec2 correction = Scale(normal, relaxation * std::max(penetration - 0.001f, 0.0f) / weight);
  if (a.inverseMass > 0.0f) {
    a.position = Sub(a.position, Scale(correction, a.inverseMass));
  }
  if (b.inverseMass > 0.0f) {
    b.position = Add(b.position, Scale(correction, b.inverseMass));
  }
}

void ResolveContact(Body& a,
                    Body& b,
                    Contact contact,
                    Material materialA,
                    Material materialB,
                    float restitutionThreshold,
                    const Body& impactA,
                    const Body& impactB,
                    ContactState* saved,
                    bool warmStartOnly) {
  ContactState scratch;
  auto& state = saved ? *saved : scratch;
  const float weight = a.inverseMass + b.inverseMass;
  if (weight == 0.0f || contact.pointCount == 0) {
    state = {};
    return;
  }
  std::array<Vec2, 2> offsetA{}, offsetB{};
  std::array<float, 2> mass{}, error{}, impulse{};
  for (int i = 0; i < contact.pointCount; ++i) {
    offsetA[i] = Sub(contact.points[i], a.position);
    offsetB[i] = Sub(contact.points[i], b.position);
    mass[i] = EffectiveMass(a, b, offsetA[i], offsetB[i], contact.normal);
  }
  const float coupledMass = contact.pointCount == 2
                                ? weight +
                                      InverseInertia(a) * Cross(offsetA[0], contact.normal) *
                                          Cross(offsetA[1], contact.normal) +
                                      InverseInertia(b) * Cross(offsetB[0], contact.normal) *
                                          Cross(offsetB[1], contact.normal)
                                : 0.0f;
  const float determinant = mass[0] * mass[1] - coupledMass * coupledMass;
  if (contact.pointCount == 2 && determinant <= 1.0e-5f * mass[0] * mass[1]) {
    // Fixed rotation or nearly duplicate points: solve at their midpoint.
    offsetA[0] = Scale(Add(offsetA[0], offsetA[1]), 0.5f);
    offsetB[0] = Scale(Add(offsetB[0], offsetB[1]), 0.5f);
    mass[0] = EffectiveMass(a, b, offsetA[0], offsetB[0], contact.normal);
    contact.pointCount = 1;
  }
  PrepareContact(state,
                 a,
                 b,
                 contact.normal,
                 offsetA,
                 offsetB,
                 contact.pointCount,
                 restitutionThreshold,
                 std::max(materialA.restitution, materialB.restitution),
                 impactA,
                 impactB);
  if (warmStartOnly) {
    return;
  }
  for (int i = 0; i < contact.pointCount; ++i) {
    error[i] =
        Dot(Sub(PointVelocity(b, offsetB[i]), PointVelocity(a, offsetA[i])), contact.normal) -
        state.bounceSpeed[i];
  }
  if (contact.pointCount == 2) {
    // Solve for total nonnegative impulses, then apply only their change.
    const float b0 =
        error[0] - mass[0] * state.normalImpulse[0] - coupledMass * state.normalImpulse[1];
    const float b1 =
        error[1] - coupledMass * state.normalImpulse[0] - mass[1] * state.normalImpulse[1];
    const float j0 = (coupledMass * b1 - mass[1] * b0) / determinant;
    const float j1 = (coupledMass * b0 - mass[0] * b1) / determinant;
    if (j0 >= 0.0f && j1 >= 0.0f) {
      impulse = {j0, j1};
    } else if (b0 >= 0.0f && b1 >= 0.0f) {
      impulse = {};
    } else {
      const float single0 = std::max(-b0 / mass[0], 0.0f);
      const float single1 = std::max(-b1 / mass[1], 0.0f);
      if (coupledMass * single0 + b1 >= 0.0f) {
        impulse[0] = single0;
      } else if (coupledMass * single1 + b0 >= 0.0f) {
        impulse[1] = single1;
      }
    }
  } else {
    impulse[0] = std::max(state.normalImpulse[0] - error[0] / mass[0], 0.0f);
  }
  for (int i = 0; i < contact.pointCount; ++i) {
    PairImpulse(
        a, b, offsetA[i], offsetB[i], Scale(contact.normal, impulse[i] - state.normalImpulse[i]));
    state.normalImpulse[i] = impulse[i];
  }
  const Vec2 tangent = Perpendicular(contact.normal);
  const double friction = std::sqrt(static_cast<double>(materialA.friction) * materialB.friction);
  for (int i = 0; i < contact.pointCount; ++i) {
    const float slip =
        Dot(Sub(PointVelocity(b, offsetB[i]), PointVelocity(a, offsetA[i])), tangent);
    const double limit = friction * state.normalImpulse[i];
    const float previous = state.tangentImpulse[i];
    state.tangentImpulse[i] = static_cast<float>(std::clamp(
        static_cast<double>(previous) - slip / EffectiveMass(a, b, offsetA[i], offsetB[i], tangent),
        -limit,
        limit));
    PairImpulse(a, b, offsetA[i], offsetB[i], Scale(tangent, state.tangentImpulse[i] - previous));
  }
  const float relaxation = a.inverseMass > 0.0f && b.inverseMass > 0.0f ? 0.25f : 1.0f;
  const Vec2 correction =
      Scale(contact.normal, relaxation * std::max(contact.penetration - 0.001f, 0.0f) / weight);
  if (a.inverseMass > 0.0f) {
    a.position = Sub(a.position, Scale(correction, a.inverseMass));
  }
  if (b.inverseMass > 0.0f) {
    b.position = Add(b.position, Scale(correction, b.inverseMass));
  }
}

void ResolveFloor(Body& body,
                  const Shape& shape,
                  float floorY,
                  Material material,
                  Material floorMaterial,
                  float restitutionThreshold,
                  const Body& impactBody,
                  ContactState* state,
                  bool warmStartOnly) {
  if (body.inverseMass == 0.0f) {
    if (state) {
      *state = {};
    }
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
  ResolveContact(body,
                 floor,
                 contact,
                 material,
                 floorMaterial,
                 restitutionThreshold,
                 impactBody,
                 floor,
                 state,
                 warmStartOnly);
}

} // namespace physeng::detail
