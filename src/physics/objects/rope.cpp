#include "physeng/objects/rope.h"
#include "../detail/material.hpp"
#include "../detail/math.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace physeng::objects {

RopeHandle CreateRope(World& world, const RopeDesc& desc) {
  const common::Vec2 start =
      desc.startBody ? world.GetBody(*desc.startBody).LocalToWorld(desc.startAnchor) : desc.start;
  const common::Vec2 end =
      desc.endBody ? world.GetBody(*desc.endBody).LocalToWorld(desc.endAnchor) : desc.end;
  if (desc.segmentCount <= 0 || !detail::Finite(start) || !detail::Finite(end) ||
      !detail::Finite(desc.startAnchor) || !detail::Finite(desc.endAnchor) ||
      (desc.enableCollisions && !detail::Positive(desc.nodeRadius)) ||
      !detail::Positive(desc.totalMass) || !detail::Nonnegative(desc.linearDamping) ||
      (desc.enableCollisions && !detail::ValidMaterial(desc.material))) {
    throw std::invalid_argument("Rope needs finite endpoints, a positive segment count and total "
                                "mass, and a positive radius when collisions are enabled");
  }

  if (desc.segmentCount == 1 && desc.startBody && desc.endBody && desc.startBody == desc.endBody) {
    throw std::invalid_argument("A one-segment rope cannot connect a body to itself");
  }
  const double dx = static_cast<double>(end.x) - start.x;
  const double dy = static_cast<double>(end.y) - start.y;
  const double segmentLength = std::hypot(dx, dy) / desc.segmentCount;
  if (segmentLength > std::numeric_limits<float>::max()) {
    throw std::invalid_argument("Rope segment length exceeds float range");
  }
  const float constraintLength = static_cast<float>(segmentLength);
  if (!detail::Positive(constraintLength)) {
    throw std::invalid_argument(
        "Rope segment length must be positive and representable as a finite float");
  }
  if (desc.enableCollisions && desc.nodeRadius > segmentLength) {
    throw std::invalid_argument("Rope node radius must not exceed segment length");
  }

  const auto nodeCount = static_cast<std::size_t>(desc.segmentCount) + 1;
  const auto dynamicCount = nodeCount - static_cast<std::size_t>(desc.startBody.has_value()) -
                            static_cast<std::size_t>(desc.endBody.has_value()) -
                            static_cast<std::size_t>(!desc.startBody && desc.anchorStart) -
                            static_cast<std::size_t>(!desc.endBody && desc.anchorEnd);
  const double inverseMassValue = static_cast<double>(dynamicCount) / desc.totalMass;
  if (inverseMassValue > std::numeric_limits<float>::max()) {
    throw std::invalid_argument("Rope node inverse mass exceeds float range");
  }
  const float inverseMass = static_cast<float>(inverseMassValue);
  if (dynamicCount != 0 && !detail::Positive(inverseMass)) {
    throw std::invalid_argument("Rope node inverse mass must be positive and finite");
  }

  std::vector<Body> nodes;
  nodes.reserve(nodeCount);
  RopeHandle rope;
  rope.bodies.reserve(nodeCount);
  for (std::size_t i = 0; i < nodeCount; ++i) {
    const double t = static_cast<double>(i) / desc.segmentCount;
    const common::Vec2 position{
        static_cast<float>(std::lerp(static_cast<double>(start.x), static_cast<double>(end.x), t)),
        static_cast<float>(std::lerp(static_cast<double>(start.y), static_cast<double>(end.y), t))};
    if (!detail::Finite(position)) {
      throw std::invalid_argument("Rope node position must be finite");
    }
    const bool anchored = (i == 0 && desc.anchorStart) || (i == nodeCount - 1 && desc.anchorEnd);
    nodes.push_back({.position = position,
                     .inverseMass = anchored ? 0.0f : inverseMass,
                     .linearDamping = desc.linearDamping,
                     .fixedRotation = true});
  }

  for (std::size_t i = 0; i < nodeCount; ++i) {
    BodyId id;
    if (i == 0 && desc.startBody) {
      id = *desc.startBody;
    } else if (i == nodeCount - 1 && desc.endBody) {
      id = *desc.endBody;
    } else {
      id = desc.enableCollisions
               ? world.AddBody(nodes[i], CircleShape{desc.nodeRadius}, desc.material)
               : world.AddBody(nodes[i]);
    }
    if (!rope.bodies.empty()) {
      const common::Vec2 firstAnchor = i == 1 && desc.startBody ? desc.startAnchor : common::Vec2{};
      const common::Vec2 secondAnchor =
          i == nodeCount - 1 && desc.endBody ? desc.endAnchor : common::Vec2{};
      world.AddDistanceConstraint(
          rope.bodies.back(), id, constraintLength, firstAnchor, secondAnchor);
      // Adjacent rope particles may overlap without fighting the link constraint.
      // A direct link between two existing bodies does not change their collision policy.
      const bool bothExistingEndpoints = nodeCount == 2 && desc.startBody && desc.endBody;
      if (!bothExistingEndpoints) {
        world.SetCollisionEnabled(rope.bodies.back(), id, false);
      }
    }
    const bool reusedEndpoint = (i == 0 && desc.startBody) || (i == nodeCount - 1 && desc.endBody);
    if (!reusedEndpoint) {
      // A rope does not collide with its own attached payload or support.
      if (desc.startBody) {
        world.SetCollisionEnabled(id, *desc.startBody, false);
      }
      if (desc.endBody) {
        world.SetCollisionEnabled(id, *desc.endBody, false);
      }
    }
    rope.bodies.push_back(id);
  }
  return rope;
}

} // namespace physeng::objects
