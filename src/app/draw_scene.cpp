#include "draw_scene.hpp"
#include "mouse_drag.hpp"
#include "physeng/scene.hpp"
#include "physeng/shape.hpp"
#include "renderer/renderer.hpp"

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <variant>

namespace app {

namespace {

using renderer::Color;
constexpr Color backgroundColor{0.075f, 0.085f, 0.105f, 1.0f};
constexpr Color wallColor{0.32f, 0.36f, 0.42f, 1.0f};
constexpr Color boxColor{0.88f, 0.57f, 0.38f, 1.0f};
constexpr Color circleColor{0.40f, 0.65f, 0.79f, 1.0f};
constexpr Color ropeColor{0.76f, 0.72f, 0.62f, 1.0f};
constexpr Color springColor{0.53f, 0.76f, 0.62f, 1.0f};
constexpr Color highlightColor{0.98f, 0.79f, 0.43f, 1.0f};

void DrawGrid(renderer::Renderer& renderer) {
  const auto view = renderer.ViewHalfExtents();
  renderer.SetColor({0.19f, 0.21f, 0.25f, 0.55f});
  for (int x = static_cast<int>(std::ceil(-view.x)); x <= view.x; ++x) {
    renderer.DrawSegment({static_cast<float>(x), -view.y}, {static_cast<float>(x), view.y}, 0.02f);
  }
  for (int y = static_cast<int>(std::ceil(-view.y)); y <= view.y; ++y) {
    renderer.DrawSegment({-view.x, static_cast<float>(y)}, {view.x, static_cast<float>(y)}, 0.02f);
  }
}

void DrawShape(renderer::Renderer& renderer,
               const physeng::Shape& shape,
               common::Vec2 position,
               float angle,
               float padding = 0.0f) {
  std::visit(
      [&](const auto& value) {
        if constexpr (std::is_same_v<std::decay_t<decltype(value)>, physeng::BoxShape>) {
          renderer.DrawRectangle(
              position, {value.halfExtents.x + padding, value.halfExtents.y + padding}, angle);
        } else if constexpr (std::is_same_v<std::decay_t<decltype(value)>, physeng::CircleShape>) {
          renderer.DrawCircle(position, value.radius + padding, angle);
        } else if constexpr (std::is_same_v<std::decay_t<decltype(value)>, physeng::PolygonShape>) {
          renderer.DrawPolygon(position, value.Vertices(), angle);
        }
      },
      shape);
}

bool IsRopeBody(const physeng::Scene& scene, physeng::BodyId body) {
  return std::any_of(scene.ropes.begin(), scene.ropes.end(), [&](const auto& rope) {
    return std::find(rope.bodies.begin(), rope.bodies.end(), body) != rope.bodies.end();
  });
}

Color BodyColor(const physeng::Scene& scene, const physeng::Collider& collider) {
  if (scene.world.GetBody(collider.body).inverseMass == 0.0f) {
    return wallColor;
  }
  if (std::holds_alternative<physeng::BoxShape>(collider.shape) ||
      std::holds_alternative<physeng::PolygonShape>(collider.shape)) {
    return boxColor;
  }
  return IsRopeBody(scene, collider.body) ? ropeColor : circleColor;
}

void DrawBodies(renderer::Renderer& renderer,
                const physeng::Scene& scene,
                std::optional<physeng::BodyId> highlighted) {
  const auto& world = scene.world;

  // Shadows are a drawing offset, not an extra physics shape.
  renderer.SetColor({0.0f, 0.0f, 0.0f, 0.22f});
  for (const auto& collider : world.Colliders()) {
    const auto& body = world.GetBody(collider.body);
    if (body.inverseMass > 0.0f) {
      DrawShape(
          renderer, collider.shape, {body.position.x + 0.04f, body.position.y - 0.06f}, body.angle);
    }
  }

  for (const auto& collider : world.Colliders()) {
    const auto& body = world.GetBody(collider.body);
    if (highlighted == collider.body) {
      renderer.SetColor(highlightColor);
      DrawShape(renderer, collider.shape, body.position, body.angle, 0.035f);
    }
    renderer.SetColor(highlighted == collider.body &&
                              std::holds_alternative<physeng::PolygonShape>(collider.shape)
                          ? highlightColor
                          : BodyColor(scene, collider));
    DrawShape(renderer, collider.shape, body.position, body.angle);
  }
}

void DrawConnections(renderer::Renderer& renderer, const physeng::World& world) {
  renderer.SetColor(ropeColor);
  for (const auto& connection : world.Constraints()) {
    const auto start = world.GetBody(connection.first).LocalToWorld(connection.firstAnchor);
    const auto end = world.GetBody(connection.second).LocalToWorld(connection.secondAnchor);
    renderer.DrawSegment(start, end, 0.035f);
  }
}
} // namespace

void DrawScene(renderer::Renderer& renderer,
               const physeng::Scene& scene,
               const MouseDrag& drag,
               std::optional<common::Vec2> cursor) {
  auto highlighted = drag.SelectedBody();
  if (!highlighted && cursor) {
    highlighted = PickBody(scene.world, *cursor);
  }

  renderer.BeginFrame(backgroundColor);
  DrawGrid(renderer);
  DrawConnections(renderer, scene.world);
  DrawBodies(renderer, scene, highlighted);

  renderer.SetColor(springColor);
  for (const auto& spring : scene.world.Springs()) {
    const auto start = scene.world.GetBody(spring.first).LocalToWorld(spring.firstAnchor);
    const auto end = scene.world.GetBody(spring.second).LocalToWorld(spring.secondAnchor);
    renderer.DrawSpring(start, end);
  }

  if (cursor) {
    if (const auto grab = drag.GrabPosition(scene.world)) {
      renderer.SetColor(highlightColor);
      renderer.DrawSegment(*grab, *cursor, 0.02f);
      renderer.DrawCircle(*cursor, 0.04f);
    }
  }
  renderer.EndFrame();
}
} // namespace app
