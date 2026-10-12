#pragma once

#include "physeng/world.hpp"
#include <optional>

namespace app {

// Return the topmost movable shape under a world-space point.
std::optional<physeng::BodyId> PickBody(const physeng::World& world, common::Vec2 point);

class MouseDrag {
public:
  void Begin(const physeng::World& world, common::Vec2 cursor);
  void End();
  void Update(physeng::World& world, common::Vec2 cursor, double timeStep);
  std::optional<common::Vec2> GrabPosition(const physeng::World& world) const;

  std::optional<physeng::BodyId> SelectedBody() const { return body_; }

private:
  std::optional<physeng::BodyId> body_;
  common::Vec2 grabOffset_;
};

} // namespace app
