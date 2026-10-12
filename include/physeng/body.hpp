#pragma once

#include "common/vec2.hpp"

namespace physeng {

// Motion state only. Zero inverse mass makes a body static.
struct Body {
  common::Vec2 position;
  common::Vec2 velocity;
  float inverseMass = 1.0f;
  common::Vec2 force;
  float linearDamping = 0.0f;   // Velocity decay rate in inverse seconds.
  float angle = 0.0f;           // Radians; positive is counterclockwise.
  float angularVelocity = 0.0f; // Radians/second.
  float torque = 0.0f;
  float inverseInertia = 0.0f; // Auto-filled from the first collider if zero.
  float angularDamping = 0.0f;
  bool fixedRotation = false;

  common::Vec2 LocalToWorld(common::Vec2 localPoint) const;
  common::Vec2 WorldToLocal(common::Vec2 worldPoint) const;
  common::Vec2 VelocityAtPoint(common::Vec2 worldPoint) const;
  float EffectiveInverseInertia() const;
};

} // namespace physeng
