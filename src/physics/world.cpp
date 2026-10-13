#include "physeng/world.hpp"
#include "detail/collision.hpp"
#include "detail/distance_constraint.hpp"
#include "detail/dynamics.hpp"
#include "detail/material.hpp"
#include "detail/math.hpp"
#include "detail/spring.hpp"
#include "physeng/shape.hpp"

#include <algorithm>
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
      (settings.floorY && !std::isfinite(*settings.floorY)) ||
      !Positive(settings.broadPhaseCellSize)) {
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
        } else if constexpr (std::is_same_v<std::decay_t<decltype(value)>, CircleShape>) {
          return Positive(value.radius);
        } else if constexpr (std::is_same_v<std::decay_t<decltype(value)>, PolygonShape>) {
          return value.Vertices().size() >= 3;
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
          } else if constexpr (std::is_same_v<std::decay_t<decltype(value)>, CircleShape>) {
            return 0.5 * static_cast<double>(value.radius) * value.radius;
          } else if constexpr (std::is_same_v<std::decay_t<decltype(value)>, PolygonShape>) {
            return value.InertiaPerMass();
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

void World::SetCircleRadius(ColliderId id, float radius) {
  auto& collider = colliders_.at(id);
  auto* circle = std::get_if<CircleShape>(&collider.shape);
  if (!circle || !Positive(radius)) {
    throw std::invalid_argument("Circle radius must be positive and finite");
  }
  if (circle->radius == radius) {
    return;
  }
  auto& body = bodies_[collider.body];
  float inertia = 0;
  if (body.inverseMass > 0 && !body.fixedRotation) {
    inertia = static_cast<float>(2.0 * body.inverseMass / (static_cast<double>(radius) * radius));
    if (!Positive(inertia)) {
      throw std::invalid_argument("Circle inertia is not representable");
    }
  }
  circle->radius = radius;
  body.inverseInertia = inertia;
  collisionBounds_.clear();
  // Cached impulses describe the old geometry; rebuild support after a size change.
  std::fill(contactStates_.begin(), contactStates_.end(), detail::ContactState{});
  if (id < floorContactStates_.size()) {
    floorContactStates_[id] = {};
  }
}

void World::Step(double timeStep) {
  const float dt = static_cast<float>(timeStep);
  if (!Positive(dt)) {
    throw std::invalid_argument("Time step must be positive and finite");
  }
  floorContactStates_.resize(colliders_.size());
  const float impulseScale =
      previousTimeStep_ > 0.0f ? std::min(dt / previousTimeStep_, 2.0f) : 0.0f;
  const auto beginContacts = [&](auto& states) {
    for (auto& state : states) {
      state.prepared = false;
      for (auto& impulse : state.normalImpulse) {
        impulse *= impulseScale;
      }
      for (auto& impulse : state.tangentImpulse) {
        impulse *= impulseScale;
      }
    }
  };
  beginContacts(contactStates_);
  beginContacts(floorContactStates_);
  previousTimeStep_ = dt;
  for (auto& constraint : constraints_) {
    constraint.accumulatedLambda = 0.0f;
  }
  for (const auto& spring : springs_) {
    ApplySpring(bodies_[spring.first], bodies_[spring.second], spring);
  }
  impactStates_.resize(bodies_.size());
  previousStates_.resize(bodies_.size());
  for (std::size_t index = 0; index < bodies_.size(); ++index) {
    auto& body = bodies_[index];
    previousStates_[index] = body;
    if (body.inverseMass == 0.0f) {
      impactStates_[index] = body;
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
      body.angle = std::remainder(body.angle + body.angularVelocity * dt,
                                  2.0f * std::numbers::pi_v<float>);
    }
    body.torque = 0.0f;
    impactStates_[index] = body;
  }
  // Repeated passes let contacts and connected bodies influence one another.
  for (int iteration = 0; iteration < settings_.solverIterations; ++iteration) {
    bool linkBroke = false;
    for (auto& constraint : constraints_) {
      const bool wasBroken = constraint.broken;
      SolveDistance(bodies_[constraint.first], bodies_[constraint.second], constraint, dt);
      linkBroke = linkBroke || (!wasBroken && constraint.broken);
    }
    if (linkBroke) {
      RefreshConnectedCollisionExclusions();
    }
    if (CollisionPairsNeedRebuild()) {
      BuildCollisionPairs();
    }
    // Apply all cached support impulses before any contact starts solving.
    if (iteration == 0) {
      SolveCollisions(true);
      if (settings_.floorY) {
        for (ColliderId id = 0; id < colliders_.size(); ++id) {
          const auto& collider = colliders_[id];
          ResolveFloor(bodies_[collider.body],
                       collider.shape,
                       *settings_.floorY,
                       collider.material,
                       settings_.floorMaterial,
                       settings_.restitutionThreshold,
                       impactStates_[collider.body],
                       &floorContactStates_[id],
                       true);
        }
      }
    }
    SolveCollisions();
    // Neighbor corrections must not leave a particle embedded in a fixed wall.
    if (CollisionPairsNeedRebuild()) {
      BuildCollisionPairs();
    }
    for (auto index : staticCollisionPairs_) {
      SolveCollisionPair(index, false);
    }
    if (settings_.floorY) {
      for (ColliderId id = 0; id < colliders_.size(); ++id) {
        const auto& collider = colliders_[id];
        ResolveFloor(bodies_[collider.body],
                     collider.shape,
                     *settings_.floorY,
                     collider.material,
                     settings_.floorMaterial,
                     settings_.restitutionThreshold,
                     impactStates_[collider.body],
                     &floorContactStates_[id]);
      }
    }
  }
}

} // namespace physeng
