#include "demo_scene.hpp"

#include <string>

namespace app {

physeng::SceneDesc MakeDemoScene() {
  physeng::SceneDesc scene;
  scene.physics.solverIterations = 24;

  constexpr int plankCount = 12;
  constexpr float spacing = 0.8f;
  constexpr float halfWidth = 0.34f;
  constexpr float deckHeight = 1.15f;
  constexpr float floorHeight = -4.0f;
  constexpr float supportHalfHeight = (deckHeight - floorHeight) * 0.5f;
  constexpr float supportCenterY = (deckHeight + floorHeight) * 0.5f;
  constexpr float linkCompliance = 0.0004f;
  constexpr float breakStretchRatio = 2.0f;

  scene.enclosures.push_back({
      .halfExtents = {7.0f, -floorHeight},
      .wallThickness = 0.25f,
      .material = {.friction = 0.7f},
  });

  // The supports' inner faces and the enclosure floor bound this water tank.
  scene.fluids.push_back({
      .settings = {.minimum = {-4.9f, floorHeight},
                   .maximum = {4.9f, -floorHeight},
                   .particleSpacing = 0.28f,
                   .smoothingRadius = 0.84f,
                   .restDensity = 3.0f, // Light objects float; the heavier crate sinks.
                   .substeps = 1}, // The bridge already steps at 120 Hz.
      .blocks = {{.minimum = {-4.4f, floorHeight + 0.2f}, .columns = 32, .rows = 6}},
  });

  scene.bodies = {
      {
          .name = "left_support",
          .body = {.position = {-5.5f, supportCenterY}, .inverseMass = 0.0f},
          .shape = physeng::BoxShape{{0.6f, supportHalfHeight}},
          .material = {.friction = 0.8f},
      },
      {
          .name = "right_support",
          .body = {.position = {5.5f, supportCenterY}, .inverseMass = 0.0f},
          .shape = physeng::BoxShape{{0.6f, supportHalfHeight}},
          .material = {.friction = 0.8f},
      },
      {
          .name = "ball",
          .body = {.position = {-0.9f, 2.4f}, .inverseMass = 1.0f / 0.6f, .linearDamping = 0.05f},
          .shape = physeng::CircleShape{0.35f},
          .material = {.restitution = 0.25f, .friction = 0.5f},
      },
      {
          .name = "crate",
          .body = {.position = {-5.5f, 2.1f}, .inverseMass = 0.5f, .angularDamping = 0.15f},
          .shape = physeng::BoxShape{{0.38f, 0.38f}},
          .material = {.friction = 0.7f},
      },
      {
          .name = "polygon",
          .body = {.position = {5.5f, 2.1f}, .angularDamping = 0.15f},
          .shape =
              physeng::PolygonShape({{-0.4f, -0.3f}, {0.4f, -0.3f}, {0.3f, 0.3f}, {-0.2f, 0.5f}}),
          .material = {.friction = 0.7f},
      },
  };

  // Planks are the rope's rigid nodes. Each link attaches at the facing edges,
  // so pulling on a link can also rotate a plank.
  for (int i = 0; i < plankCount; ++i) {
    const float x = (i - (plankCount - 1) * 0.5f) * spacing;
    scene.bodies.push_back({
        .name = "plank_" + std::to_string(i),
        .body = {.position = {x, deckHeight},
                 .inverseMass = 4.0f,
                 .linearDamping = 0.6f,
                 .angularDamping = 1.5f},
        .shape = physeng::BoxShape{{halfWidth, 0.12f}},
        .material = {.friction = 0.8f},
    });
  }

  for (int i = 0; i <= plankCount; ++i) {
    const bool first = i == 0, last = i == plankCount;
    scene.ropes.push_back({
        .settings = {.segmentCount = 1,
                     .startAnchor =
                         first ? common::Vec2{0.6f, supportHalfHeight} : common::Vec2{halfWidth, 0},
                     .endAnchor = last ? common::Vec2{-0.6f, supportHalfHeight}
                                       : common::Vec2{-halfWidth, 0},
                     .compliance = linkCompliance,
                     .breakStretchRatio = breakStretchRatio},
        .startBody = first ? "left_support" : "plank_" + std::to_string(i - 1),
        .endBody = last ? "right_support" : "plank_" + std::to_string(i),
    });
  }

  return scene;
}

} // namespace app
