#include "physeng/world.hpp"
#include "detail/collision.hpp"
#include "detail/distance_constraint.hpp"
#include "detail/dynamics.hpp"
#include "detail/material.hpp"
#include "detail/math.hpp"
#include "detail/spring.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace physeng {

using namespace detail;

World::World(PhysicsSettings settings) : settings_(settings) {
  if (!Finite(settings.gravity) || settings.solverIterations <= 0 ||
      !Nonnegative(settings.restitutionThreshold) || !ValidMaterial(settings.floorMaterial) ||
      (settings.floorY && !std::isfinite(*settings.floorY))) {
    throw std::invalid_argument("Invalid physics settings");
  }
}

BodyId World::AddBody(Body body) {
  if (!Finite(body.position) || !Finite(body.velocity) || !Finite(body.force) ||
      !Nonnegative(body.inverseMass) || !Nonnegative(body.linearDamping) ||
      !std::isfinite(body.angle) || !std::isfinite(body.angularVelocity) ||
      !std::isfinite(body.torque) || !Nonnegative(body.inverseInertia) ||
      !Nonnegative(body.angularDamping) ||
      ((body.inverseMass == 0.0f || body.fixedRotation) && body.angularVelocity != 0.0f) ||
      (body.inverseMass == 0.0f && (body.velocity.x != 0.0f || body.velocity.y != 0.0f))) {
    throw std::invalid_argument("Invalid body state");
  }
  const BodyId id = bodies_.size();
  bodies_.push_back(body);
  return id;
}

ColliderId World::AddCollider(BodyId body, Shape shape, Material material) {
  if (body >= bodies_.size()) {
    throw std::invalid_argument("Collider must reference an existing body");
  }
  const bool validShape = std::visit(
      [](const auto& value) {
        if constexpr (std::is_same_v<std::decay_t<decltype(value)>, BoxShape>) {
          return Positive(value.halfExtents.x) && Positive(value.halfExtents.y);
        } else {
          return Positive(value.radius);
        }
      },
      shape);
  if (!validShape) {
    throw std::invalid_argument("Shape dimensions must be positive and finite");
  }
  if (!ValidMaterial(material)) {
    throw std::invalid_argument("Invalid collider material");
  }
  const ColliderId id = colliders_.size();
  auto& state = bodies_[body];
  float inverseInertia = state.inverseInertia;
  if (state.inverseMass > 0.0f && !state.fixedRotation && inverseInertia == 0.0f) {
    const double radiusSquared = std::visit(
        [](const auto& value) -> double {
          if constexpr (std::is_same_v<std::decay_t<decltype(value)>, BoxShape>) {
            return (static_cast<double>(value.halfExtents.x) * value.halfExtents.x +
                    static_cast<double>(value.halfExtents.y) * value.halfExtents.y) /
                   3.0;
          } else {
            return 0.5 * static_cast<double>(value.radius) * value.radius;
          }
        },
        shape);
    const double candidate = state.inverseMass / radiusSquared;
    if (candidate > std::numeric_limits<float>::max()) {
      throw std::invalid_argument("Shape inertia exceeds float range");
    }
    inverseInertia = static_cast<float>(candidate);
    if (!Positive(inverseInertia)) {
      throw std::invalid_argument("Shape inertia is not representable");
    }
  }
  colliders_.push_back({body, std::move(shape), material});
  state.inverseInertia = inverseInertia;
  return id;
}

BodyId World::AddBody(Body body, Shape shape, Material material) {
  const BodyId id = AddBody(body);
  try {
    AddCollider(id, std::move(shape), material);
  } catch (...) {
    bodies_.pop_back();
    throw;
  }
  return id;
}

const Body& World::GetBody(BodyId id) const {
  return bodies_.at(id);
}

void World::Step(double timeStep) {
  const float dt = static_cast<float>(timeStep);
  if (!Positive(dt)) {
    throw std::invalid_argument("Time step must be positive and finite");
  }
  for (const auto& spring : springs_) {
    ApplySpring(bodies_[spring.first], bodies_[spring.second], spring);
  }
  for (auto& body : bodies_) {
    if (body.inverseMass == 0.0f) {
      continue;
    }
    body.velocity = Add(body.velocity, Scale(settings_.gravity, dt));
    body.velocity = Add(body.velocity, Scale(body.force, dt * body.inverseMass));
    body.velocity = Scale(body.velocity, std::exp(-body.linearDamping * dt));
    body.force = {0, 0};
    body.position = Add(body.position, Scale(body.velocity, dt));
    if (!body.fixedRotation) {
      body.angularVelocity += body.torque * InverseInertia(body) * dt;
      body.angularVelocity *= std::exp(-body.angularDamping * dt);
      body.angle =
          std::remainder(body.angle + body.angularVelocity * dt, 2.0f * std::numbers::pi_v<float>);
    }
    body.torque = 0.0f;
  }
  impactStates_ = bodies_;
  // Repeated passes let contacts and connected bodies influence one another.
  for (int iteration = 0; iteration < settings_.solverIterations; ++iteration) {
    for (const auto& constraint : constraints_) {
      SolveDistance(bodies_[constraint.first], bodies_[constraint.second], constraint, dt);
    }
    for (std::size_t i = 0; i < colliders_.size(); ++i) {
      for (std::size_t j = i + 1; j < colliders_.size(); ++j) {
        const auto& ca = colliders_[i];
        const auto& cb = colliders_[j];
        if (!CanCollide(ca.body, cb.body)) {
          continue;
        }
        auto& a = bodies_[ca.body];
        auto& b = bodies_[cb.body];
        if (a.inverseMass + b.inverseMass == 0.0f) {
          continue;
        }
        auto contact =
            std::visit([&](const auto& sa, const auto& sb) { return ContactBetween(a, sa, b, sb); },
                       ca.shape,
                       cb.shape);
        if (contact) {
          ResolveContact(a,
                         b,
                         *contact,
                         ca.material,
                         cb.material,
                         settings_.restitutionThreshold,
                         impactStates_[ca.body],
                         impactStates_[cb.body]);
        }
      }
    }
    if (settings_.floorY) {
      for (const auto& collider : colliders_) {
        ResolveFloor(bodies_[collider.body],
                     collider.shape,
                     *settings_.floorY,
                     collider.material,
                     settings_.floorMaterial,
                     settings_.restitutionThreshold,
                     impactStates_[collider.body]);
      }
    }
  }
}

} // namespace physeng
