#pragma once

#include "common/vec2.hpp"
#include <array>

namespace physeng::detail {

// Velocity impulses belong to a collider pair, not to either body's motion state.
struct ContactState {
  common::Vec2 normal{};
  std::array<common::Vec2, 2> localAnchorA{}, localAnchorB{};
  std::array<float, 2> normalImpulse{}, tangentImpulse{}, bounceSpeed{};
  int pointCount = 0;
  bool prepared = false;
};

} // namespace physeng::detail
