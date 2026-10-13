#pragma once

#include "physeng/world.hpp"
#include <vector>

namespace physeng::objects {

struct SoftBodyDesc {
  common::Vec2 position{};
  float radius = 0.5f;
  float totalMass = 3.0f;
  float compliance = 0.0003f;
  float linearDamping = 0.8f;
  Material material{.friction = 0.3f};
};

struct SoftBodyLink {
  DistanceConstraintId constraint;
  float lengthPerRadius;
};

// A triangulated hexagonal cluster: 19 equal-mass circles connected by elastic links.
struct SoftBodyHandle {
  std::vector<BodyId> nodes;
  std::vector<ColliderId> colliders;
  std::vector<BodyId> surfaceNodes; // Counterclockwise boundary, for drawing the skin.
  std::vector<SoftBodyLink> links;
  float radius;

  common::Vec2 Center(const World& world) const;
  common::Vec2 Velocity(const World& world) const;
};

SoftBodyHandle CreateSoftBody(World& world, const SoftBodyDesc& desc);
// Change rest geometry, preserving mass and letting the solver move the nodes.
void SetSoftBodyRadius(World& world, SoftBodyHandle& body, float radius);

} // namespace physeng::objects
