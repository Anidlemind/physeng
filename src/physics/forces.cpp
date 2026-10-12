#include "detail/dynamics.hpp"
#include "detail/math.hpp"
#include "physeng/world.hpp"

namespace physeng {

using detail::Finite;

void World::ApplyForce(BodyId id, common::Vec2 force) {
  Body& body = bodies_.at(id);
  if (body.inverseMass == 0 || !Finite(force)) {
    return;
  }
  body.force.x += force.x;
  body.force.y += force.y;
}

void World::ApplyImpulse(BodyId id, common::Vec2 impulse) {
  Body& body = bodies_.at(id);
  if (body.inverseMass == 0 || !Finite(impulse)) {
    return;
  }
  body.velocity.x += impulse.x * body.inverseMass;
  body.velocity.y += impulse.y * body.inverseMass;
}

void World::ApplyForceAtPoint(BodyId id, common::Vec2 force, common::Vec2 worldPoint) {
  Body& body = bodies_.at(id);
  if (body.inverseMass == 0.0f || !Finite(force) || !Finite(worldPoint)) {
    return;
  }
  body.force = detail::Add(body.force, force);
  if (!body.fixedRotation) {
    body.torque += detail::Cross(detail::Sub(worldPoint, body.position), force);
  }
}

void World::ApplyImpulseAtPoint(BodyId id, common::Vec2 impulse, common::Vec2 worldPoint) {
  Body& body = bodies_.at(id);
  if (!Finite(impulse) || !Finite(worldPoint)) {
    return;
  }
  detail::ImpulseAtOffset(body, impulse, detail::Sub(worldPoint, body.position));
}

void World::ApplyTorque(BodyId id, float torque) {
  Body& body = bodies_.at(id);
  if (body.inverseMass == 0.0f || body.fixedRotation || !std::isfinite(torque)) {
    return;
  }
  body.torque += torque;
}

void World::ApplyAngularImpulse(BodyId id, float impulse) {
  Body& body = bodies_.at(id);
  if (!std::isfinite(impulse)) {
    return;
  }
  body.angularVelocity += impulse * detail::InverseInertia(body);
}

} // namespace physeng
