#include "puffer_demo.hpp"
#include "draw_water.hpp"
#include "puffer_scene.hpp"
#include "renderer/renderer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

namespace app {
namespace {

void DrawPuffer(renderer::Renderer& renderer,
                const physeng::World& world,
                const physeng::objects::SoftBodyHandle& fish,
                float inflation,
                std::vector<renderer::CircleInstance>& circles,
                std::vector<common::Vec2>& skin) {
  const auto center = fish.Center(world);
  const float nodeRadius = fish.radius / 5;
  const renderer::Color outline{0.23f, 0.18f, 0.12f, 1};
  const renderer::Color bodyColor{
      std::lerp(0.62f, 0.98f, inflation), std::lerp(0.68f, 0.76f, inflation), 0.32f, 1};
  skin.clear();
  skin.push_back(center);
  for (auto id : fish.surfaceNodes) {
    skin.push_back(world.GetBody(id).position);
  }
  skin.push_back(skin[1]);

  // The skin follows actual node positions, including dents and squashing.
  circles.clear();
  renderer.SetColor(outline);
  for (std::size_t i = 0; i < fish.surfaceNodes.size(); ++i) {
    const auto p = world.GetBody(fish.surfaceNodes[i]).position;
    const auto next = world.GetBody(fish.surfaceNodes[(i + 1) % fish.surfaceNodes.size()]).position;
    renderer.DrawSegment(p, next, nodeRadius * 2 + 0.04f);
    circles.push_back({p, nodeRadius + 0.02f});
    const common::Vec2 delta{p.x - center.x, p.y - center.y};
    const float distance = std::hypot(delta.x, delta.y);
    if (distance > 0) {
      const common::Vec2 direction{delta.x / distance, delta.y / distance};
      const float spine = 0.05f + 0.17f * inflation;
      renderer.DrawSegment(
          {p.x + direction.x * nodeRadius, p.y + direction.y * nodeRadius},
          {p.x + direction.x * (nodeRadius + spine), p.y + direction.y * (nodeRadius + spine)},
          0.045f);
    }
  }
  renderer.DrawCircles(circles, false);
  renderer.SetColor(bodyColor);
  renderer.DrawPolygon({}, skin);
  for (std::size_t i = 0; i < fish.surfaceNodes.size(); ++i) {
    const auto a = skin[i + 1], b = skin[i + 2];
    const float dx = b.x - a.x, dy = b.y - a.y;
    const float length = std::hypot(dx, dy);
    if (length > 0) {
      const common::Vec2 side{-dy * nodeRadius / length, dx * nodeRadius / length};
      const std::array<common::Vec2, 4> edge{{{a.x - side.x, a.y - side.y},
                                              {b.x - side.x, b.y - side.y},
                                              {b.x + side.x, b.y + side.y},
                                              {a.x + side.x, a.y + side.y}}};
      renderer.DrawPolygon({}, edge);
    }
  }
  circles.clear();
  for (auto id : fish.nodes) {
    circles.push_back({world.GetBody(id).position, nodeRadius});
  }
  renderer.DrawCircles(circles, false);

}
} // namespace

void RunPufferDemo(renderer::Renderer& renderer, bool waterVersion) {
  const PufferSettings settings;
  const auto makeScene = [&] {
    return physeng::CreateScene(waterVersion ? MakePufferWaterScene(settings)
                                             : MakePufferScene(settings));
  };
  auto scene = makeScene();
  PufferControlState control;
  WaterDrawing waterDrawing;
  std::vector<renderer::CircleInstance> circles;
  std::vector<common::Vec2> skin;
  circles.reserve(19);
  skin.reserve(14);
  renderer.SetVisibleHeight(waterVersion ? 10 : 11);
  renderer.SetVSync(true);
  std::puts(
      "Pufferfish: A/D or arrows move; Space/Up/W puffs and jumps from ground; "
      "release or Down/S squeezes. R resets. Platforms left, low tunnel and squeeze slot right.");

  using Clock = std::chrono::steady_clock;
  auto previous = Clock::now();
  double accumulator = 0;
  constexpr double timeStep = 1.0 / 120;
  bool previousReset = false;
  while (!renderer.ShouldClose()) {
    renderer.PollEvents();
    const auto now = Clock::now();
    accumulator += std::min(std::chrono::duration<double>(now - previous).count(), 0.1);
    previous = now;
    const bool reset = renderer.IsKeyDown(renderer::Key::Reset);
    if (reset && !previousReset) {
      scene = makeScene();
      control = {};
      accumulator = 0;
    }
    previousReset = reset;
    const bool left =
        renderer.IsKeyDown(renderer::Key::Left) || renderer.IsKeyDown(renderer::Key::A);
    const bool right =
        renderer.IsKeyDown(renderer::Key::Right) || renderer.IsKeyDown(renderer::Key::D);
    const bool squeeze =
        renderer.IsKeyDown(renderer::Key::Down) || renderer.IsKeyDown(renderer::Key::S);
    const bool inflate =
        !squeeze && (renderer.IsKeyDown(renderer::Key::Space) ||
                     renderer.IsKeyDown(renderer::Key::Up) || renderer.IsKeyDown(renderer::Key::W));
    const float horizontal = static_cast<float>(right) - static_cast<float>(left);
    while (accumulator >= timeStep) {
      UpdatePuffer(scene, settings, horizontal, inflate, timeStep, control);
      scene.Step(timeStep);
      LimitPufferRiseSpeed(scene, settings);
      accumulator -= timeStep;
    }

    renderer.BeginFrame({0.035f, 0.07f, 0.11f, 1});
    for (const auto& water : scene.fluids) {
      waterDrawing.Draw(renderer, water);
    }
    for (const auto& collider : scene.world.Colliders()) {
      const auto& body = scene.world.GetBody(collider.body);
      if (body.inverseMass == 0) {
        renderer.SetColor({0.32f, 0.39f, 0.43f, 1});
        for (const auto& [name, id] : scene.bodies) {
          if (id == collider.body) {
            renderer.SetColor(name == "tunnel_roof"      ? renderer::Color{0.44f, 0.37f, 0.57f, 1}
                              : name.starts_with("slot") ? renderer::Color{0.60f, 0.39f, 0.23f, 1}
                                                         : renderer::Color{0.25f, 0.49f, 0.42f, 1});
          }
        }
        if (const auto* box = std::get_if<physeng::BoxShape>(&collider.shape)) {
          renderer.DrawRectangle(body.position, box->halfExtents, body.angle);
        }
      }
    }
    const float inflation = PufferInflation(scene.softBodies.front(), settings);
    DrawPuffer(renderer, scene.world, scene.softBodies.front(), inflation, circles, skin);
    renderer.SetColor({0.04f, 0.05f, 0.07f, 0.94f});
    renderer.DrawScreenRectangle({12, 12}, {610, 156});
    char text[256];
    std::snprintf(text,
                  sizeof(text),
                  "PUFFERFISH - INFLATION: %.0f%%\nA/D OR LEFT/RIGHT: MOVE\nSPACE/UP/W: PUFF "
                  "%s\nRELEASE OR DOWN/S: SQUEEZE\nR: RESET    %s",
                  inflation * 100,
                  waterVersion ? "AND FLOAT" : "AND JUMP",
                  waterVersion ? "WATER PROTOTYPE" : "PLATFORMS / LOW TUNNEL / SQUEEZE SLOT");
    renderer.SetColor({0.90f, 0.93f, 0.95f, 1});
    renderer.DrawText({24, 24}, text);
    renderer.EndFrame();
  }
}

} // namespace app
