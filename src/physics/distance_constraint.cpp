#include "detail/distance_constraint.hpp"
#include "detail/dynamics.hpp"
#include "detail/math.hpp"
#include "physeng/world.hpp"
#include <stdexcept>

namespace physeng {

DistanceConstraintId World::AddDistanceConstraint(BodyId first,
                                                  BodyId second,
                                                  float length,
                                                  common::Vec2 firstAnchor,
                                                  common::Vec2 secondAnchor) {
  if (first >= bodies_.size() || second >= bodies_.size() || first == second ||
      !detail::Positive(length) || !detail::Finite(firstAnchor) || !detail::Finite(secondAnchor)) {
    throw std::invalid_argument("Invalid distance constraint");
  }
  const auto id = constraints_.size();
  constraints_.push_back({first, second, length, firstAnchor, secondAnchor});
  return id;
}
} // namespace physeng

namespace physeng::detail {

void SolveDistance(Body& a, Body& b, const DistanceConstraint& constraint, float dt) {
  const Vec2 pa = a.LocalToWorld(constraint.firstAnchor);
  const Vec2 pb = b.LocalToWorld(constraint.secondAnchor);
  const Vec2 ra = Sub(pa, a.position), rb = Sub(pb, b.position);
  const Vec2 delta = Sub(pb, pa);
  const float distance = Length(delta);
  const Vec2 normal = distance > 0.0f ? Scale(delta, 1.0f / distance) : Vec2{1, 0};
  const float ca = Cross(ra, normal), cb = Cross(rb, normal);
  const float weight =
      a.inverseMass + b.inverseMass + ca * ca * InverseInertia(a) + cb * cb * InverseInertia(b);
  if (weight == 0.0f) {
    return;
  }
  const float error = distance - constraint.length;
  const float speed = Dot(Sub(PointVelocity(b, rb), PointVelocity(a, ra)), normal);
  const Vec2 impulse = Scale(normal, -(speed + 0.1f * error / dt) / weight);
  ImpulseAtOffset(a, Scale(impulse, -1), ra);
  ImpulseAtOffset(b, impulse, rb);
  const float correction = -0.8f * error / weight;
  a.position = Sub(a.position, Scale(normal, correction * a.inverseMass));
  b.position = Add(b.position, Scale(normal, correction * b.inverseMass));
  a.angle -= ca * correction * InverseInertia(a);
  b.angle += cb * correction * InverseInertia(b);
}
} // namespace physeng::detail
