#pragma once

#include "physeng/world.hpp"

#include <optional>
#include <vector>

namespace physeng::objects {

struct RopeDesc {
  common::Vec2 start{-4.0f, 3.5f};
  common::Vec2 end{-1.0f, 3.5f};
  int segmentCount = 6;
  float nodeRadius = 0.08f; // Adjacent colliders may overlap; radius <= segment length.
  float totalMass = 6.0f;
  bool anchorStart = true;
  bool anchorEnd = false;
  bool enableCollisions = true; // Without colliders, nodeRadius is ignored.
  Material material;
  float linearDamping = 0.0f;
  // Existing endpoint bodies override start/end positions and anchor flags.
  std::optional<BodyId> startBody;
  std::optional<BodyId> endBody;
  common::Vec2 startAnchor{}; // Body-local attachment points.
  common::Vec2 endAnchor{};
  float compliance = 0.0f; // Whole rope compliance in metres/newton, shared by its segments.
  std::optional<float> breakStretchRatio; // Per-segment limit; e.g. 1.5 allows 50% extension.
};

struct RopeHandle {
  // Includes reused endpoint bodies; only intermediate/free-end nodes are created.
  std::vector<BodyId> bodies;
  std::vector<DistanceConstraintId> constraints; // Stable IDs, including any broken links.
};

RopeHandle CreateRope(World& world, const RopeDesc& desc);

} // namespace physeng::objects
