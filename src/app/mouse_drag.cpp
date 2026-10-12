#include "mouse_drag.hpp"

#include <cmath>
#include <type_traits>
#include <variant>

namespace app {

std::optional<physeng::BodyId> PickBody(const physeng::World& world, common::Vec2 cursor) {
  const auto colliders = world.Colliders();
  // Pick the last drawn dynamic shape when shapes overlap.
  for (auto it = colliders.rbegin(); it != colliders.rend(); ++it) {
    const auto& body = world.GetBody(it->body);
    if (body.inverseMass == 0.0f) {
      continue;
    }
    const common::Vec2 offset = body.WorldToLocal(cursor);
    const bool inside = std::visit(
        [&](const auto& shape) {
          if constexpr (std::is_same_v<std::decay_t<decltype(shape)>, physeng::BoxShape>) {
            return std::abs(offset.x) <= shape.halfExtents.x &&
                   std::abs(offset.y) <= shape.halfExtents.y;
          } else {
            return std::hypot(offset.x, offset.y) <= shape.radius;
          }
        },
        it->shape);
    if (inside) {
      return it->body;
    }
  }
  return std::nullopt;
}

void MouseDrag::Begin(const physeng::World& world, common::Vec2 cursor) {
  body_ = PickBody(world, cursor);
  if (body_) {
    grabOffset_ = world.GetBody(*body_).WorldToLocal(cursor);
  }
}

void MouseDrag::End() {
  body_.reset();
}

void MouseDrag::Update(physeng::World& world, common::Vec2 cursor, double timeStep) {
  if (!body_) {
    return;
  }
  const auto& body = world.GetBody(*body_);
  if (body.inverseMass == 0.0f) {
    End();
    return;
  }
  const float dt = static_cast<float>(timeStep);
  const auto grabPoint = body.LocalToWorld(grabOffset_);
  const common::Vec2 offset{grabPoint.x - body.position.x, grabPoint.y - body.position.y};
  const auto pointVelocity = body.VelocityAtPoint(grabPoint);
  constexpr float springFrequency = 30.0f; // Radians/second.
  constexpr float maxSpeed = 15.0f;        // Prevent huge cursor jumps from launching bodies.
  constexpr float maxAcceleration = 300.0f;
  const common::Vec2 error{cursor.x - offset.x - body.position.x,
                           cursor.y - offset.y - body.position.y};
  // Pull toward the cursor without teleporting. Damping keeps the grab from oscillating.
  const float stiffnessStep = springFrequency * springFrequency * dt;
  const float divisor = 1.0f + 2.0f * springFrequency * dt + stiffnessStep * dt;
  common::Vec2 targetVelocity{(pointVelocity.x + stiffnessStep * error.x) / divisor,
                              (pointVelocity.y + stiffnessStep * error.y) / divisor};
  const float speed = std::hypot(targetVelocity.x, targetVelocity.y);
  if (speed > maxSpeed) {
    targetVelocity.x *= maxSpeed / speed;
    targetVelocity.y *= maxSpeed / speed;
  }
  common::Vec2 change{targetVelocity.x - pointVelocity.x, targetVelocity.y - pointVelocity.y};
  const float magnitude = std::hypot(change.x, change.y);
  const float maxChange = maxAcceleration * dt;
  if (magnitude > maxChange) {
    change.x *= maxChange / magnitude;
    change.y *= maxChange / magnitude;
  }
  // Solve the 2D point-response matrix: an off-center impulse also changes spin.
  const float inverseInertia = body.EffectiveInverseInertia();
  const float kxx = body.inverseMass + inverseInertia * offset.y * offset.y;
  const float kyy = body.inverseMass + inverseInertia * offset.x * offset.x;
  const float kxy = -inverseInertia * offset.x * offset.y;
  const float determinant = kxx * kyy - kxy * kxy;
  const common::Vec2 impulse{(kyy * change.x - kxy * change.y) / determinant,
                             (kxx * change.y - kxy * change.x) / determinant};
  world.ApplyImpulseAtPoint(*body_, impulse, grabPoint);
}

std::optional<common::Vec2> MouseDrag::GrabPosition(const physeng::World& world) const {
  if (!body_) {
    return std::nullopt;
  }
  const auto& body = world.GetBody(*body_);
  return body.LocalToWorld(grabOffset_);
}

} // namespace app
