#include "renderer/renderer.hpp"
#include "src/app/demo_scene.hpp"
#include "src/app/draw_scene.hpp"
#include "src/app/mouse_drag.hpp"
#include "src/app/window_motion.hpp"

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
    app::WindowMotion windowMotion;
    windowMotion.Update(renderer.WindowPosition(), timestamp);
    common::Vec2 pendingWindowVelocityChange;
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
      const auto windowVelocityChange = windowMotion.Update(renderer.WindowPosition(), currentTime);
      pendingWindowVelocityChange.x += windowVelocityChange.x;
      pendingWindowVelocityChange.y += windowVelocityChange.y;

      accumulator += std::min(std::chrono::duration<double>(currentTime - timestamp).count(), 0.1);
      timestamp = currentTime;
      while (accumulator >= timeStep) {
        // In the moving window's frame, bodies lag behind its change in velocity.
        // Consume once, even when this frame needs several physics steps.
        if (pendingWindowVelocityChange.x != 0.0f || pendingWindowVelocityChange.y != 0.0f) {
          const auto bodies = world.Bodies();
          for (physeng::BodyId id = 0; id < bodies.size(); ++id) {
            if (bodies[id].inverseMass > 0.0f) {
              world.ApplyImpulse(id,
                                 {-pendingWindowVelocityChange.x / bodies[id].inverseMass,
                                  -pendingWindowVelocityChange.y / bodies[id].inverseMass});
            }
          }
          pendingWindowVelocityChange = {};
        }
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
