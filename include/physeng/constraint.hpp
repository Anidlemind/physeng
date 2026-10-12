#pragma once

#include "common/vec2.hpp"
#include "physeng/id.hpp"

namespace physeng {

struct DistanceConstraint {
  BodyId first;
  BodyId second;
  float length;
  common::Vec2 firstAnchor{};
  common::Vec2 secondAnchor{};
};
} // namespace physeng
