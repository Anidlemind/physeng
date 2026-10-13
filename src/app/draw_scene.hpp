#pragma once

#include "common/vec2.hpp"
#include <optional>

namespace physeng {

struct Scene;
}

namespace renderer {

class Renderer;
}

namespace app {

class MouseDrag;
class WaterDrawing;

void DrawScene(renderer::Renderer& renderer,
               const physeng::Scene& scene,
               const MouseDrag& drag,
               std::optional<common::Vec2> cursor,
               WaterDrawing& waterDrawing);
} // namespace app
