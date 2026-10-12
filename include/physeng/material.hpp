#pragma once

namespace physeng {

struct Material {
  float restitution = 0.0f; // [0, 1]: fraction of normal approach speed recovered.
  float friction = 0.5f;    // Nonnegative Coulomb friction coefficient.
};

} // namespace physeng
