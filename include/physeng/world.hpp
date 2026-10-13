#pragma once

#include "common/vec2.hpp"
#include "physeng/body.hpp"
#include "physeng/constraint.hpp"
#include "physeng/detail/contact_state.hpp"
#include "physeng/material.hpp"
#include "physeng/shape.hpp"
#include "physeng/spring.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <unordered_map>
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
  float broadPhaseCellSize = 0.5f; // Grid cell width in world units.
};

class World {
public:
  explicit World(PhysicsSettings settings = {});
  BodyId AddBody(Body body);
  BodyId AddBody(Body body, Shape shape, Material material = {});
  ColliderId AddCollider(BodyId body, Shape shape, Material material = {});
  void SetCircleRadius(ColliderId id, float radius);
  void SetDistanceConstraintLength(DistanceConstraintId id, float length);
  DistanceConstraintId AddDistanceConstraint(BodyId first,
                                             BodyId second,
                                             float length,
                                             common::Vec2 firstAnchor = {},
                                             common::Vec2 secondAnchor = {},
                                             float compliance = 0.0f,
                                             std::optional<float> breakStretchRatio = std::nullopt);
  SpringId AddSpring(Spring spring);

  void SetCollisionEnabled(BodyId first, BodyId second, bool enabled);
  // A separate exclusion that expires when any link in this path breaks.
  void DisableCollisionWhileConnected(BodyId first,
                                      BodyId second,
                                      std::span<const DistanceConstraintId> path);
  bool CanCollide(BodyId first, BodyId second) const;
  void Step(double timeStep);

  const Body& GetBody(BodyId id) const;
  bool IsGrounded(BodyId id, float distance = 0.05f,
                   std::span<const BodyId> excluded = {}) const;

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
  friend class Fluid; // Fluid coupling updates poses and exchanges impulses during a step.
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

  struct ConnectedCollisionExclusion {
    BodyPair pair;
    std::vector<DistanceConstraintId> path;
    bool active = true;
  };

  void RefreshConnectedCollisionExclusions();
  void BuildCollisionPairs();
  bool CollisionPairsNeedRebuild() const;
  void SolveCollisionPair(std::size_t index, bool warmStartOnly);
  void SolveCollisions(bool warmStartOnly = false);

  struct CollisionBounds {
    common::Vec2 minimum, maximum;
  };

  CollisionBounds ComputeCollisionBounds(ColliderId id, float padding) const;

  struct CellOrigin {
    std::int64_t x, y;
  };

  struct CellEntry {
    std::int64_t x, y;
    ColliderId collider;
  };

  struct ColliderPair {
    ColliderId first, second;
    bool operator==(const ColliderPair&) const = default;
  };

  std::vector<CollisionBounds> collisionBounds_;
  std::vector<CellOrigin> cellOrigins_;
  std::vector<bool> largeColliderFlags_;
  std::vector<CellEntry> cellEntries_;
  std::vector<ColliderId> largeColliders_;
  std::vector<ColliderPair> collisionPairs_;
  std::vector<ColliderPair> previousCollisionPairs_;
  std::vector<std::size_t> staticCollisionPairs_;
  std::vector<detail::ContactState> contactStates_, previousContactStates_, floorContactStates_;
  float previousTimeStep_ = 0.0f;

  std::unordered_set<BodyPair, BodyPairHash> disabledCollisionPairs_;
  // Count independent exclusions so one broken rope cannot undo another's filter.
  std::unordered_map<BodyPair, std::size_t, BodyPairHash> connectedCollisionPairs_;
  std::vector<ConnectedCollisionExclusion> connectedCollisionExclusions_;
  PhysicsSettings settings_;
  std::vector<Body> bodies_;
  std::vector<Collider> colliders_;
  std::vector<DistanceConstraint> constraints_;
  std::vector<Spring> springs_;
  std::vector<Body> impactStates_;
  std::vector<Body> previousStates_;
};

} // namespace physeng
