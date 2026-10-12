#pragma once

#include "common/vec2.hpp"
#include "physeng/id.hpp"

namespace physeng {

struct Spring {
  BodyId first;
  BodyId second;
  float restLength = 1.0f;
  float stiffness = 50.0f; // Newtons/meter.
  float damping = 2.0f;    // Newton-seconds/meter; opposes relative axial motion.
  common::Vec2 firstAnchor{};
  common::Vec2 secondAnchor{};
};
} // namespace physeng
