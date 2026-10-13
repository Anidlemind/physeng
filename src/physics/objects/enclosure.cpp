#include "physeng/objects/enclosure.hpp"
#include "../detail/material.hpp"
#include "../detail/math.hpp"

#include <stdexcept>

namespace physeng::objects {

EnclosureHandle CreateEnclosure(World& world, const EnclosureDesc& desc) {
  if (!detail::Finite(desc.center) || !detail::Positive(desc.halfExtents.x) ||
      !detail::Positive(desc.halfExtents.y) || !detail::Positive(desc.wallThickness) ||
      !detail::ValidMaterial(desc.material)) {
    throw std::invalid_argument("Invalid enclosure dimensions or material");
  }
  const float halfThickness = desc.wallThickness * 0.5f;
  const float sideOffset = desc.halfExtents.x + halfThickness;
  const float verticalOffset = desc.halfExtents.y + halfThickness;
  const std::array<common::Vec2, 4> positions{{{desc.center.x - sideOffset, desc.center.y},
                                               {desc.center.x + sideOffset, desc.center.y},
                                               {desc.center.x, desc.center.y - verticalOffset},
                                               {desc.center.x, desc.center.y + verticalOffset}}};
  const std::array<BoxShape, 4> shapes{
      // Overlap wall ends: a buried circle must exit toward the interior,
      // rather than alternate between the exposed ends of two touching walls.
      {{{halfThickness, desc.halfExtents.y + desc.wallThickness}},
       {{halfThickness, desc.halfExtents.y + desc.wallThickness}},
       {{desc.halfExtents.x + desc.wallThickness, halfThickness}},
       {{desc.halfExtents.x + desc.wallThickness, halfThickness}}}};
  // Validate computed geometry before adding any walls.
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (!detail::Finite(positions[i]) || !detail::Positive(shapes[i].halfExtents.x) ||
        !detail::Positive(shapes[i].halfExtents.y)) {
      throw std::invalid_argument("Enclosure wall geometry is not representable");
    }
  }
  EnclosureHandle result;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    result.walls[i] =
        world.AddBody({.position = positions[i], .inverseMass = 0.0f}, shapes[i], desc.material);
  }
  return result;
}

} // namespace physeng::objects
