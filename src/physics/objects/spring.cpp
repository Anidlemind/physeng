#include "physeng/objects/spring.hpp"
#include "../detail/math.hpp"

namespace physeng::objects {

SpringHandle CreateSpring(World& world, const SpringDesc& desc) {
  const auto a = world.GetBody(desc.first).LocalToWorld(desc.firstAnchor);
  const auto b = world.GetBody(desc.second).LocalToWorld(desc.secondAnchor);
  const float length = desc.restLength.value_or(detail::Length(detail::Sub(b, a)));
  return {world.AddSpring({desc.first,
                           desc.second,
                           length,
                           desc.stiffness,
                           desc.damping,
                           desc.firstAnchor,
                           desc.secondAnchor})};
}
} // namespace physeng::objects
