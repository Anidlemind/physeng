#pragma once

#include "common/vec2.hpp"
#include "physeng/body.hpp"
#include "physeng/constraint.hpp"
#include "physeng/material.hpp"
#include "physeng/shape.hpp"
#include "physeng/spring.hpp"

#include <cstddef>
#include <functional>
#include <optional>
#include <span>
#include <unordered_set>
#include <vector>

namespace physeng {

struct Collider {
  BodyId body;
  Shape shape;
  Material material;
};

struct PhysicsSettings {
  common::Vec2 gravity{0.0f, -9.81f};
  std::optional<float> floorY = -4.0f;
  int solverIterations = 8;
  float restitutionThreshold = 0.5f;
  Material floorMaterial;
};

class World {
public:
  explicit World(PhysicsSettings settings = {});
  BodyId AddBody(Body body);
  BodyId AddBody(Body body, Shape shape, Material material = {});
  ColliderId AddCollider(BodyId body, Shape shape, Material material = {});
  DistanceConstraintId AddDistanceConstraint(BodyId first,
                                             BodyId second,
                                             float length,
                                             common::Vec2 firstAnchor = {},
                                             common::Vec2 secondAnchor = {});
  SpringId AddSpring(Spring spring);

  void SetCollisionEnabled(BodyId first, BodyId second, bool enabled);
  bool CanCollide(BodyId first, BodyId second) const;
  void Step(double timeStep);

  const Body& GetBody(BodyId id) const;

  std::span<const Body> Bodies() const { return bodies_; }

  std::span<const Collider> Colliders() const { return colliders_; }

  std::span<const DistanceConstraint> Constraints() const { return constraints_; }

  std::span<const Spring> Springs() const { return springs_; }

  void ApplyForce(BodyId id, common::Vec2 force);
  void ApplyImpulse(BodyId id, common::Vec2 impulse);
  void ApplyForceAtPoint(BodyId id, common::Vec2 force, common::Vec2 worldPoint);
  void ApplyImpulseAtPoint(BodyId id, common::Vec2 impulse, common::Vec2 worldPoint);
  void ApplyTorque(BodyId id, float torque);
  void ApplyAngularImpulse(BodyId id, float impulse);

private:
  struct BodyPair {
    BodyId first, second;
    bool operator==(const BodyPair&) const = default;
  };

  struct BodyPairHash {
    std::size_t operator()(const BodyPair& pair) const noexcept {
      const auto first = std::hash<BodyId>{}(pair.first);
      const auto second = std::hash<BodyId>{}(pair.second);
      return first ^ (second + 0x9e3779b9u + (first << 6) + (first >> 2));
    }
  };

  std::unordered_set<BodyPair, BodyPairHash> disabledCollisionPairs_;
  PhysicsSettings settings_;
  std::vector<Body> bodies_;
  std::vector<Collider> colliders_;
  std::vector<DistanceConstraint> constraints_;
  std::vector<Spring> springs_;
  std::vector<Body> impactStates_;
};

} // namespace physeng
