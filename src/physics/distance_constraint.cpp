#include "detail/distance_constraint.hpp"
#include "detail/dynamics.hpp"
#include "detail/math.hpp"
#include "physeng/world.hpp"
#include <cmath>
#include <stdexcept>

namespace physeng {

void World::SetDistanceConstraintLength(DistanceConstraintId id, float length) {
  if (!detail::Positive(length)) {
    throw std::invalid_argument("Constraint length must be positive and finite");
  }
  auto& constraint = constraints_.at(id);
  if (constraint.length != length) {
    constraint.length = length;
    constraint.accumulatedLambda = 0;
  }
}

DistanceConstraintId World::AddDistanceConstraint(BodyId first,
                                                  BodyId second,
                                                  float length,
                                                  common::Vec2 firstAnchor,
                                                  common::Vec2 secondAnchor,
                                                  float compliance,
                                                  std::optional<float> breakStretchRatio) {
  if (first >= bodies_.size() || second >= bodies_.size() || first == second ||
      !detail::Positive(length) || !detail::Finite(firstAnchor) || !detail::Finite(secondAnchor) ||
      !detail::Nonnegative(compliance) ||
      (breakStretchRatio && (!std::isfinite(*breakStretchRatio) || *breakStretchRatio <= 1.0f))) {
    throw std::invalid_argument("Invalid distance constraint");
  }
  const auto id = constraints_.size();
  constraints_.push_back({.first = first,
                          .second = second,
                          .length = length,
                          .firstAnchor = firstAnchor,
                          .secondAnchor = secondAnchor,
                          .compliance = compliance,
                          .breakStretchRatio = breakStretchRatio,
                          .id = id});
  return id;
}
} // namespace physeng

namespace physeng::detail {

void SolveDistance(Body& a, Body& b, DistanceConstraint& constraint, float dt) {
  if (constraint.broken) {
    return;
  }
  const Vec2 pa = a.LocalToWorld(constraint.firstAnchor);
  const Vec2 pb = b.LocalToWorld(constraint.secondAnchor);
  const Vec2 ra = Sub(pa, a.position), rb = Sub(pb, b.position);
  const Vec2 delta = Sub(pb, pa);
  const float distance = Length(delta);
  const float stretchRatio = distance / constraint.length;
  if (constraint.breakStretchRatio && stretchRatio > *constraint.breakStretchRatio) {
    constraint.broken = true;
    constraint.accumulatedLambda = 0.0f;
    return;
  }
  const Vec2 normal = distance > 0.0f ? Scale(delta, 1.0f / distance) : Vec2{1, 0};
  const float ca = Cross(ra, normal), cb = Cross(rb, normal);
  const float weight =
      a.inverseMass + b.inverseMass + ca * ca * InverseInertia(a) + cb * cb * InverseInertia(b);
  if (weight == 0.0f) {
    return;
  }
  const float error = distance - constraint.length;
  if (constraint.compliance > 0.0f) {
    // XPBD: compliance allows a load-dependent extension, rather than a hard length.
    // Accumulate across solver passes, but reset once at the start of each step.
    const double alpha =
        static_cast<double>(constraint.compliance) / (static_cast<double>(dt) * dt);
    const float correction = static_cast<float>(
        (-static_cast<double>(error) - alpha * constraint.accumulatedLambda) / (weight + alpha));
    constraint.accumulatedLambda += correction;
    a.position = Sub(a.position, Scale(normal, correction * a.inverseMass));
    b.position = Add(b.position, Scale(normal, correction * b.inverseMass));
    a.angle -= ca * correction * InverseInertia(a);
    b.angle += cb * correction * InverseInertia(b);

    // The positional change also changes velocity; otherwise the bodies keep
    // accelerating under load while being held in place by position correction.
    const Vec2 impulse = Scale(normal, correction / dt);
    ImpulseAtOffset(a, Scale(impulse, -1.0f), ra);
    ImpulseAtOffset(b, impulse, rb);
    return;
  }
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
