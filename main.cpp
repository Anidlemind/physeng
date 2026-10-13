#include "renderer/renderer.hpp"
#include "src/app/demo_scene.hpp"
#include "src/app/draw_scene.hpp"
#include "src/app/draw_water.hpp"
#include "src/app/mouse_drag.hpp"
#include "src/app/particle_demo.hpp"
#include "src/app/puffer_demo.hpp"
#include "src/app/water_demo.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <exception>
#include <string_view>

int main(int argc, char** argv) {
  const bool bridge = argc == 2 && std::string_view(argv[1]) == "--bridge";
  const bool water = argc == 2 && std::string_view(argv[1]) == "--water";
  const bool puffer = argc == 2 && std::string_view(argv[1]) == "--puffer";
  const bool pufferWater = argc == 2 && std::string_view(argv[1]) == "--puffer-water";
  if (argc > 2 || (argc == 2 && !bridge && !water && !puffer && !pufferWater && std::string_view(argv[1]) != "--particles")) {
    std::fprintf(stderr, "Usage: %s [--particles|--bridge|--water|--puffer|--puffer-water]\n", argv[0]);
    return 1;
  }
  try {
    renderer::Renderer renderer;
    if (puffer || pufferWater) {
      app::RunPufferDemo(renderer, pufferWater);
      return 0;
    }
    if (water) {
      app::RunWaterDemo(renderer);
      return 0;
    }
    if (!bridge) {
      app::RunParticleDemo(renderer);
      return 0;
    }
    auto scene = physeng::CreateScene(app::MakeDemoScene());
    auto& world = scene.world;

    using Clock = std::chrono::steady_clock;
    auto timestamp = Clock::now();
    double accumulator = 0.0;
    constexpr double timeStep = 1.0 / 120.0;

    app::MouseDrag drag;
    app::WaterDrawing waterDrawing;
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
        scene.Step(timeStep);
        accumulator -= timeStep;
      }

      app::DrawScene(renderer, scene, drag, cursor, waterDrawing);
    }
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "%s\n", error.what());
    return 1;
  }
}
