#include "detail/dynamics.hpp"
#include "detail/math.hpp"
#include "detail/spring.hpp"
#include "physeng/world.hpp"
#include <stdexcept>

namespace physeng {

SpringId World::AddSpring(Spring spring) {
  if (spring.first >= bodies_.size() || spring.second >= bodies_.size() ||
      spring.first == spring.second || !detail::Nonnegative(spring.restLength) ||
      !detail::Nonnegative(spring.stiffness) || !detail::Nonnegative(spring.damping) ||
      !detail::Finite(spring.firstAnchor) || !detail::Finite(spring.secondAnchor)) {
    throw std::invalid_argument("Invalid spring");
  }
  const auto id = springs_.size();
  springs_.push_back(spring);
  return id;
}
} // namespace physeng

namespace physeng::detail {

void ApplySpring(Body& first, Body& second, const Spring& spring) {
  const Vec2 a = first.LocalToWorld(spring.firstAnchor);
  const Vec2 b = second.LocalToWorld(spring.secondAnchor);
  const Vec2 delta = Sub(b, a);
  const float length = Length(delta);
  const Vec2 normal = length > 0.0f ? Scale(delta, 1.0f / length) : Vec2{0, 1};
  const float speed = Dot(Sub(second.VelocityAtPoint(b), first.VelocityAtPoint(a)), normal);
  const Vec2 force =
      Scale(normal, spring.stiffness * (length - spring.restLength) + spring.damping * speed);
  if (first.inverseMass > 0.0f) {
    first.force = Add(first.force, force);
    first.torque += Cross(Sub(a, first.position), force);
  }
  if (second.inverseMass > 0.0f) {
    second.force = Sub(second.force, force);
    second.torque -= Cross(Sub(b, second.position), force);
  }
}
} // namespace physeng::detail
