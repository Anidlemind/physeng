#include "renderer/renderer.hpp"
#include "src/app/demo_scene.hpp"
#include "src/app/draw_scene.hpp"
#include "src/app/mouse_drag.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <exception>

int main() {
  try {
    renderer::Renderer renderer;
    auto scene = physeng::CreateScene(app::MakeDemoScene());
    auto& world = scene.world;

    using Clock = std::chrono::steady_clock;
    auto timestamp = Clock::now();
    double accumulator = 0.0;
    constexpr double timeStep = 1.0 / 120.0;

    app::MouseDrag drag;
    bool previousMouseDown = false;
    std::puts("Left mouse: drag bodies; release to drop.");

    while (!renderer.ShouldClose()) {
      renderer.PollEvents();
      const auto cursor = renderer.CursorWorldPosition();
      const bool mouseDown = renderer.IsMouseButtonDown(renderer::MouseButton::Left);
      if (!mouseDown || !cursor) {
        drag.End();
      } else if (!previousMouseDown) {
        drag.Begin(world, *cursor);
      }
      previousMouseDown = mouseDown;

      const auto currentTime = Clock::now();
      // Limit catch-up after a debugger pause or a stalled frame.
      accumulator += std::min(std::chrono::duration<double>(currentTime - timestamp).count(), 0.1);
      timestamp = currentTime;
      while (accumulator >= timeStep) {
        if (cursor) {
          drag.Update(world, *cursor, timeStep);
        }
        world.Step(timeStep);
        accumulator -= timeStep;
      }

      app::DrawScene(renderer, scene, drag, cursor);
    }
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "%s\n", error.what());
    return 1;
  }
}
