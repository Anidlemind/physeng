#pragma once

#include "renderer/renderer.hpp"
#include <vector>

namespace physeng {
class Fluid;
}

namespace app {

// Reuse the instance buffer for each tank rather than allocating every frame.
class WaterDrawing {
public:
  void Draw(renderer::Renderer& renderer, const physeng::Fluid& water);

private:
  std::vector<renderer::CircleInstance> circles_;
};

} // namespace app
