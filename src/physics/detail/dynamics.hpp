#pragma once

#include "math.hpp"
#include "physeng/body.hpp"

namespace physeng::detail {

inline float InverseInertia(const Body& body) {
  return body.EffectiveInverseInertia();
}

inline Vec2 PointVelocity(const Body& body, Vec2 offset) {
  return Add(body.velocity, Scale(Perpendicular(offset), body.angularVelocity));
}

inline void ImpulseAtOffset(Body& body, Vec2 impulse, Vec2 offset) {
  if (body.inverseMass == 0.0f) {
    return;
  }
  body.velocity = Add(body.velocity, Scale(impulse, body.inverseMass));
  body.angularVelocity += Cross(offset, impulse) * InverseInertia(body);
}
} // namespace physeng::detail
