#include "detail/collision.hpp"
#include "detail/math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace physeng::detail {

std::optional<Contact> SweptCircleBoxContact(const Body& previous,
                                             const Body& circle,
                                             CircleShape shape,
                                             const Body& box,
                                             BoxShape boxShape) {
  const Vec2 start = box.WorldToLocal(previous.position);
  const Vec2 end = box.WorldToLocal(circle.position);
  const Vec2 travel = Sub(end, start);
  const Vec2 extent = boxShape.halfExtents;
  const Vec2 closest{std::clamp(start.x, -extent.x, extent.x),
                     std::clamp(start.y, -extent.y, extent.y)};
  const Vec2 separation = Sub(start, closest);
  const float distance = Length(separation);
  float firstHit = std::numeric_limits<float>::infinity();
  Vec2 normal{}, hit{};
  const auto accept = [&](float time, Vec2 outward) {
    if (time >= 0.0f && time <= 1.0f && time < firstHit) {
      firstHit = time;
      normal = outward;
      hit = Add(start, Scale(travel, time));
    }
  };
  // Resting contacts begin within the small penetration allowance.
  if (distance > 0.0f && distance <= shape.radius && Dot(travel, separation) < 0.0f) {
    normal = Scale(separation, 1.0f / distance);
    hit = Add(closest, Scale(normal, shape.radius));
    firstHit = 0.0f;
  }
  // A swept circle meets either a flat face or one of the rounded corners.
  for (float sign : {-1.0f, 1.0f}) {
    if (sign * travel.x < 0.0f) {
      const float time = (sign * (extent.x + shape.radius) - start.x) / travel.x;
      if (std::abs(start.y + time * travel.y) <= extent.y) {
        accept(time, {sign, 0});
      }
    }
    if (sign * travel.y < 0.0f) {
      const float time = (sign * (extent.y + shape.radius) - start.y) / travel.y;
      if (std::abs(start.x + time * travel.x) <= extent.x) {
        accept(time, {0, sign});
      }
    }
  }
  const double speedSquared =
      static_cast<double>(travel.x) * travel.x + static_cast<double>(travel.y) * travel.y;
  if (speedSquared > 0.0) {
    for (float sx : {-1.0f, 1.0f}) {
      for (float sy : {-1.0f, 1.0f}) {
        const Vec2 corner{sx * extent.x, sy * extent.y};
        const Vec2 offset = Sub(start, corner);
        const double projection =
            static_cast<double>(offset.x) * travel.x + static_cast<double>(offset.y) * travel.y;
        const double c = static_cast<double>(offset.x) * offset.x +
                         static_cast<double>(offset.y) * offset.y -
                         static_cast<double>(shape.radius) * shape.radius;
        const double discriminant = projection * projection - speedSquared * c;
        if (discriminant < 0.0) {
          continue;
        }
        const float time =
            static_cast<float>((-projection - std::sqrt(discriminant)) / speedSquared);
        const Vec2 atHit = Add(start, Scale(travel, time));
        if (sx * atHit.x >= extent.x && sy * atHit.y >= extent.y) {
          const Vec2 outward = Sub(atHit, corner);
          const float length = Length(outward);
          if (length > 0.0f) {
            accept(time, Scale(outward, 1.0f / length));
          }
        }
      }
    }
  }
  if (!std::isfinite(firstHit)) {
    return std::nullopt;
  }
  const float penetration = Dot(Sub(hit, end), normal);
  if (penetration < -0.001f) {
    return std::nullopt;
  }
  const Vec2 worldNormal = Scale(Rotate(normal, box.angle), -1.0f);
  return Contact{worldNormal,
                 std::max(penetration, 0.0f),
                 {Add(circle.position, Scale(worldNormal, shape.radius)), Vec2{}},
                 1};
}

} // namespace physeng::detail
