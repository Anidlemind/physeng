#pragma once

#include "math.hpp"
#include "physeng/material.hpp"

namespace physeng::detail {

inline bool ValidMaterial(Material material) {
  return Nonnegative(material.restitution) && material.restitution <= 1.0f &&
         Nonnegative(material.friction);
}
} // namespace physeng::detail
