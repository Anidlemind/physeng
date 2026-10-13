#include "detail/collision.hpp"
#include "detail/math.hpp"
#include "physeng/world.hpp"

#include <stdexcept>
#include <algorithm>
#include <limits>
#include <type_traits>

namespace physeng {

bool World::IsGrounded(BodyId id, float distance, std::span<const BodyId> excluded) const {
  if (!detail::Nonnegative(distance)) {
    throw std::invalid_argument("Ground query distance must be finite and nonnegative");
  }
  Body probe = bodies_.at(id);
  probe.position.y -= distance;
  for (const auto& own : colliders_) {
    if (own.body != id) {
      continue;
    }
    if (settings_.floorY) {
      const float bottom = std::visit([&](const auto& shape) {
        using T = std::decay_t<decltype(shape)>;
        if constexpr (std::is_same_v<T, CircleShape>) {
          return probe.position.y - shape.radius;
        } else if constexpr (std::is_same_v<T, BoxShape>) {
          return probe.position.y - std::abs(std::sin(probe.angle)) * shape.halfExtents.x -
                 std::abs(std::cos(probe.angle)) * shape.halfExtents.y;
        } else {
          float minimum = std::numeric_limits<float>::infinity();
          for (auto vertex : shape.Vertices()) {
            minimum = std::min(minimum, probe.LocalToWorld(vertex).y);
          }
          return minimum;
        }
      }, own.shape);
      if (bottom <= *settings_.floorY + 0.001f) {
        return true;
      }
    }
    for (const auto& other : colliders_) {
      if (other.body == id || std::find(excluded.begin(), excluded.end(), other.body) != excluded.end() ||
          !CanCollide(id, other.body)) {
        continue;
      }
      const auto contact = std::visit(
          [&](const auto& a, const auto& b) {
            return detail::ContactBetween(probe, a, bodies_[other.body], b);
          },
          own.shape,
          other.shape);
      // A wall or ceiling is not support. Slopes up to about 60 degrees count.
      if (contact && contact->normal.y < -0.5f) {
        return true;
      }
    }
  }
  return false;
}

} // namespace physeng
