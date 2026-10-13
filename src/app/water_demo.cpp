#include "water_demo.hpp"
#include "draw_water.hpp"
#include "physeng/fluid.hpp"
#include "renderer/renderer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>

namespace app {

namespace {
constexpr float dragRadius = 1.8f;
constexpr float dragStrength = 80.0f; // Rate at which water follows the cursor, per second.
constexpr float dragSpeedMultiplier = 2.0f; // Help a grabbed blob keep up against gravity and viscosity.
constexpr float maximumDragSpeed = 35.0f;
} // namespace

void RunWaterDemo(renderer::Renderer& renderer) {
  physeng::FluidSettings settings;
  settings.particleSpacing = 0.32f;
  settings.smoothingRadius = 0.96f;
  const physeng::FluidBlockDesc block{.minimum = {-11.7f, -6.7f}, .columns = 32, .rows = 16};
  physeng::Fluid water(settings);
  // Release a tall column on the left: it falls, spreads, and forms waves.
  water.AddBlock(block);
  WaterDrawing waterDrawing;
  renderer.SetVisibleHeight(18);
  renderer.SetVSync(false);
  std::puts("Water demo: hold left mouse and drag to pull water. Right mouse resets the water.");

  using Clock = std::chrono::steady_clock;
  auto previousFrame = Clock::now();
  double accumulator = 0, sampleSeconds = 0, frameMilliseconds = 0;
  std::size_t sampleFrames = 0;
  std::optional<common::Vec2> previousCursor;
  bool previousReset = false;
  // Two fluid substeps per 60 Hz tick give a 120 Hz integration rate.
  constexpr double timeStep = 1.0 / 60;
  while (!renderer.ShouldClose()) {
    renderer.PollEvents();
    const auto now = Clock::now();
    const double frameSeconds = std::chrono::duration<double>(now - previousFrame).count();
    previousFrame = now;
    sampleSeconds += frameSeconds;
    ++sampleFrames;
    if (sampleSeconds >= 0.5) {
      frameMilliseconds = sampleSeconds * 1000.0 / sampleFrames;
      sampleSeconds = 0;
      sampleFrames = 0;
    }
    const bool reset = renderer.IsMouseButtonDown(renderer::MouseButton::Right);
    if (reset && !previousReset) {
      water = physeng::Fluid(settings);
      water.AddBlock(block);
      accumulator = 0;
      previousCursor.reset();
    }
    previousReset = reset;
    const auto cursor = renderer.CursorWorldPosition();
    const bool dragging = renderer.IsMouseButtonDown(renderer::MouseButton::Left);
    common::Vec2 brushVelocity{};
    if (dragging && cursor && previousCursor && frameSeconds > 0) {
      brushVelocity = {
          static_cast<float>((cursor->x - previousCursor->x) / frameSeconds) * dragSpeedMultiplier,
          static_cast<float>((cursor->y - previousCursor->y) / frameSeconds) * dragSpeedMultiplier};
      const float speed = std::hypot(brushVelocity.x, brushVelocity.y);
      if (speed > maximumDragSpeed) {
        brushVelocity.x *= maximumDragSpeed / speed;
        brushVelocity.y *= maximumDragSpeed / speed;
      }
    }
    previousCursor = dragging ? cursor : std::nullopt;
    accumulator += std::min(frameSeconds, 0.1);
    while (accumulator >= timeStep) {
      if (dragging && cursor) {
        water.Stir(*cursor, brushVelocity, dragRadius, 1.0f - std::exp(-dragStrength * timeStep));
      }
      water.Step(timeStep);
      accumulator -= timeStep;
    }
    renderer.BeginFrame();
    waterDrawing.Draw(renderer, water);
    renderer.SetColor({0.32f, 0.36f, 0.42f, 1});
    const float wall = 0.3f;
    const common::Vec2 center{(settings.minimum.x + settings.maximum.x) / 2,
                              (settings.minimum.y + settings.maximum.y) / 2};
    const common::Vec2 extent{(settings.maximum.x - settings.minimum.x) / 2,
                              (settings.maximum.y - settings.minimum.y) / 2};
    renderer.DrawRectangle({settings.minimum.x - wall / 2, center.y}, {wall / 2, extent.y + wall});
    renderer.DrawRectangle({settings.maximum.x + wall / 2, center.y}, {wall / 2, extent.y + wall});
    renderer.DrawRectangle({center.x, settings.minimum.y - wall / 2}, {extent.x + wall, wall / 2});
    renderer.DrawRectangle({center.x, settings.maximum.y + wall / 2}, {extent.x + wall, wall / 2});
    if (dragging && cursor) {
      renderer.SetColor({0.85f, 0.93f, 1.0f, 0.2f});
      renderer.DrawCircle(*cursor, dragRadius);
    }
    renderer.SetColor({0.04f, 0.05f, 0.07f, 0.94f});
    renderer.DrawScreenRectangle({12, 12}, {360, 132});
    char text[256];
    std::snprintf(
        text,
        sizeof(text),
        "WATER: %zu\nFRAME: %.2f MS / %.1f FPS\nLEFT DRAG: PULL WATER\nRIGHT CLICK: RESET\nSPEED: BLUE 0 -> RED 20+",
        water.Particles().size(),
        frameMilliseconds,
        frameMilliseconds > 0 ? 1000 / frameMilliseconds : 0);
    renderer.SetColor({0.87f, 0.91f, 0.96f, 1});
    renderer.DrawText({24, 24}, text);
    renderer.EndFrame();
  }
}

} // namespace app
