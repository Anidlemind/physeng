#pragma once

#include "physeng/world.hpp"
#include <optional>

namespace physeng::objects {

struct SpringDesc {
  BodyId first;
  BodyId second;
  common::Vec2 firstAnchor{};
  common::Vec2 secondAnchor{};
  std::optional<float> restLength; // Omit to use the current attachment distance.
  float stiffness = 50.0f;
  float damping = 2.0f;
};

struct SpringHandle {
  SpringId spring;
};

SpringHandle CreateSpring(World& world, const SpringDesc& desc);
} // namespace physeng::objects
