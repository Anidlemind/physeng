#include "physeng/objects/softbody.hpp"
#include "../detail/material.hpp"
#include "../detail/math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace physeng::objects {
using namespace detail;

common::Vec2 SoftBodyHandle::Center(const World& world) const {
  common::Vec2 sum{};
  for (auto id : nodes) {
    sum = Add(sum, world.GetBody(id).position);
  }
  return nodes.empty() ? sum : Scale(sum, 1.0f / nodes.size());
}

common::Vec2 SoftBodyHandle::Velocity(const World& world) const {
  common::Vec2 sum{};
  for (auto id : nodes) {
    sum = Add(sum, world.GetBody(id).velocity);
  }
  return nodes.empty() ? sum : Scale(sum, 1.0f / nodes.size());
}

SoftBodyHandle CreateSoftBody(World& world, const SoftBodyDesc& desc) {
  if (!Finite(desc.position) || !Positive(desc.radius) || !Positive(desc.totalMass) ||
      !Positive(desc.radius / 5) || !Positive(19.0f / desc.totalMass) ||
      !Nonnegative(desc.compliance) || !Nonnegative(desc.linearDamping) ||
      !ValidMaterial(desc.material)) {
    throw std::invalid_argument("Invalid soft-body settings");
  }

  struct Node {
    common::Vec2 position;
    bool surface;
  };

  std::vector<Node> layout;
  const float spacing = desc.radius * 0.4f;
  // Axial hex-grid coordinates give a mesh of equilateral triangles.
  for (int q = -2; q <= 2; ++q) {
    for (int r = -2; r <= 2; ++r) {
      const int layer = std::max({std::abs(q), std::abs(r), std::abs(q + r)});
      if (layer > 2) {
        continue;
      }
      const common::Vec2 position =
          Add(desc.position, {spacing * (q + r * 0.5f), spacing * (std::sqrt(3.0f) * 0.5f * r)});
      if (!Finite(position)) {
        throw std::invalid_argument("Soft-body positions exceed float range");
      }
      layout.push_back({position, layer == 2});
    }
  }
  SoftBodyHandle result{.radius = desc.radius};
  for (const auto& node : layout) {
    const auto id = world.AddBody({.position = node.position,
                                   .inverseMass = 19.0f / desc.totalMass,
                                   .linearDamping = desc.linearDamping,
                                   .fixedRotation = true});
    result.nodes.push_back(id);
    result.colliders.push_back(world.AddCollider(id, CircleShape{desc.radius / 5}, desc.material));
    if (node.surface) {
      result.surfaceNodes.push_back(id);
    }
  }
  std::sort(result.surfaceNodes.begin(), result.surfaceNodes.end(), [&](auto a, auto b) {
    const auto pa = Sub(world.GetBody(a).position, desc.position);
    const auto pb = Sub(world.GetBody(b).position, desc.position);
    return std::atan2(pa.y, pa.x) < std::atan2(pb.y, pb.x);
  });
  for (std::size_t i = 0; i < result.nodes.size(); ++i) {
    for (std::size_t j = i + 1; j < result.nodes.size(); ++j) {
      const float length = Length(Sub(layout[i].position, layout[j].position));
      if (length <= spacing * 1.01f) {
        // Linked neighbors can compress elastically; other nodes cannot fold through each other.
        world.SetCollisionEnabled(result.nodes[i], result.nodes[j], false);
        result.links.push_back(
            {world.AddDistanceConstraint(
                 result.nodes[i], result.nodes[j], length, {}, {}, desc.compliance),
             length / desc.radius});
      }
    }
  }
  return result;
}

void SetSoftBodyRadius(World& world, SoftBodyHandle& body, float radius) {
  if (!Positive(radius) || !Positive(radius / 5)) {
    throw std::invalid_argument("Soft-body radius must be positive and finite");
  }
  if (radius == body.radius) {
    return;
  }
  for (const auto& link : body.links) {
    if (!Positive(link.lengthPerRadius * radius)) {
      throw std::invalid_argument("Soft-body link length is not representable");
    }
  }
  for (auto id : body.colliders) {
    world.SetCircleRadius(id, radius / 5);
  }
  for (const auto& link : body.links) {
    world.SetDistanceConstraintLength(link.constraint, link.lengthPerRadius * radius);
  }
  body.radius = radius;
}

} // namespace physeng::objects
