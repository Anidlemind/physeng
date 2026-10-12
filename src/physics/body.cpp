#include "physeng/body.hpp"
#include "detail/math.hpp"

namespace physeng {

common::Vec2 Body::LocalToWorld(common::Vec2 localPoint) const {
  return detail::Add(position, detail::Rotate(localPoint, angle));
}

common::Vec2 Body::WorldToLocal(common::Vec2 worldPoint) const {
  return detail::Rotate(detail::Sub(worldPoint, position), -angle);
}

common::Vec2 Body::VelocityAtPoint(common::Vec2 worldPoint) const {
  const auto radius = detail::Sub(worldPoint, position);
  return detail::Add(velocity, detail::Scale(detail::Perpendicular(radius), angularVelocity));
}

float Body::EffectiveInverseInertia() const {
  return inverseMass == 0.0f || fixedRotation ? 0.0f : inverseInertia;
}

} // namespace physeng
