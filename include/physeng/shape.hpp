#pragma once

#include "common/vec2.hpp"

#include <span>
#include <variant>
#include <vector>

namespace physeng {

struct BoxShape {
  common::Vec2 halfExtents{0.5f, 0.5f};
};

struct CircleShape {
  float radius = 0.5f;
};

class PolygonShape {
public:
  explicit PolygonShape(std::vector<common::Vec2> vertices);

  std::span<const common::Vec2> Vertices() const { return vertices_; }

  double InertiaPerMass() const { return inertiaPerMass_; }

private:
  std::vector<common::Vec2> vertices_;
  double inertiaPerMass_ = 0.0;
};

// Shapes are centered on their body and follow its angle.
using Shape = std::variant<BoxShape, CircleShape, PolygonShape>;

} // namespace physeng
