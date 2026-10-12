#pragma once

#include "common/vec2.hpp"

#include <variant>

namespace physeng {

struct BoxShape {
  common::Vec2 halfExtents{0.5f, 0.5f};
};

struct CircleShape {
  float radius = 0.5f;
};

// Currently shapes are centered on their body and boxes follow its angle.
// TODO: Add complex shapes
using Shape = std::variant<BoxShape, CircleShape>;

} // namespace physeng
