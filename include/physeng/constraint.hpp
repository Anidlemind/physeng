#pragma once

#include "common/vec2.hpp"
#include "physeng/id.hpp"

#include <optional>

namespace physeng {

struct DistanceConstraint {
  BodyId first;
  BodyId second;
  float length;
  common::Vec2 firstAnchor{};
  common::Vec2 secondAnchor{};

  float compliance = 0.0f;                // Metres/newton; zero keeps the connection rigid.
  std::optional<float> breakStretchRatio; // Current length / rest length; absent means unbreakable.
  bool broken = false;
  float accumulatedLambda = 0.0f;

  DistanceConstraintId id = 0;
};
} // namespace physeng
