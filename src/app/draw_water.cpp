#include "draw_water.hpp"
#include "physeng/fluid.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace app {
namespace {
constexpr float colorSpeedLimit = 20.0f;

renderer::Color SpeedColor(common::Vec2 velocity) {
  constexpr std::array<renderer::Color, 4> colors{{
      {0.12f, 0.35f, 0.85f, 1}, // Still water: blue.
      {0.10f, 0.85f, 0.90f, 1}, // Cyan.
      {1.00f, 0.85f, 0.15f, 1}, // Yellow.
      {0.95f, 0.20f, 0.12f, 1}, // Fast water: red.
  }};
  const float speed = std::hypot(velocity.x, velocity.y);
  const float ramp = std::clamp(speed / colorSpeedLimit, 0.0f, 1.0f) * (colors.size() - 1);
  const auto index = std::min(static_cast<std::size_t>(ramp), colors.size() - 2);
  const float blend = ramp - index;
  const auto& a = colors[index];
  const auto& b = colors[index + 1];
  return {std::lerp(a.r, b.r, blend), std::lerp(a.g, b.g, blend), std::lerp(a.b, b.b, blend), 1};
}
} // namespace

void WaterDrawing::Draw(renderer::Renderer& renderer, const physeng::Fluid& water) {
  circles_.clear();
  for (const auto& particle : water.Particles()) {
    circles_.push_back({particle.position,
                        water.Settings().particleSpacing * 0.85f,
                        0,
                        SpeedColor(particle.velocity)});
  }
  renderer.SetColor({1, 1, 1, 1});
  renderer.DrawCircles(circles_, false);
}

} // namespace app
