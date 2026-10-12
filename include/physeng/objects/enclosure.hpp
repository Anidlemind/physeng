#pragma once

#include "physeng/world.hpp"
#include <array>

namespace physeng::objects {

struct EnclosureDesc {
  common::Vec2 center{};
  common::Vec2 halfExtents{7.0f, 4.0f}; // Interior dimensions, excluding walls.
  float wallThickness = 0.25f;
  Material material;
};

struct EnclosureHandle {
  std::array<BodyId, 4> walls; // Left, right, bottom, top.
};

EnclosureHandle CreateEnclosure(World& world, const EnclosureDesc& desc);

} // namespace physeng::objects
