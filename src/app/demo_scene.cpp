#include "demo_scene.hpp"

namespace app {

physeng::SceneDesc MakeDemoScene() {
  physeng::SceneDesc scene;
  scene.physics.solverIterations = 16;

  scene.enclosures.push_back({
      .halfExtents = {7.0f, 4.0f},
      .wallThickness = 0.25f,
      .material = {.friction = 0.7f},
  });

  scene.bodies = {
      {
          .name = "spring_anchor",
          .body = {.position = {-2.0f, 3.0f}, .inverseMass = 0.0f},
          .shape = physeng::BoxShape{{0.25f, 0.12f}},
      },
      {
          .name = "spring_weight",
          .body = {.position = {-2.0f, 0.5f}, .angularDamping = 0.1f},
          .shape = physeng::CircleShape{0.35f},
      },
      {
          .name = "rope_anchor",
          .body = {.position = {1.5f, 3.0f}, .inverseMass = 0.0f},
          .shape = physeng::BoxShape{{0.25f, 0.12f}},
      },
      {
          .name = "rope_weight",
          .body =
              {
                  .position = {3.2f, 0.7f},
                  .linearDamping = 0.05f,
                  .angularDamping = 0.1f,
              },
          .shape = physeng::BoxShape{{0.35f, 0.35f}},
      },
      {
          .name = "rope_obstacle",
          .body = {.position = {1.5f, 1.6f}, .inverseMass = 0.0f},
          .shape = physeng::BoxShape{{0.35f, 0.15f}},
      },
      {
          .name = "ball",
          .body = {.position = {5.0f, 2.0f}, .linearDamping = 0.02f},
          .shape = physeng::CircleShape{0.4f},
          .material = {.restitution = 0.8f, .friction = 0.2f},
      },
  };

  scene.bodies.push_back({
      .name = "ramp",
      .body = {.position = {-4.0f, -2.8f}, .inverseMass = 0.0f},
      .shape = physeng::PolygonShape({{-1.5f, -0.5f}, {1.5f, -0.5f}, {-1.5f, 1.0f}}),
      .material = {.friction = 0.6f},
  });
  scene.bodies.push_back({
      .name = "polygon",
      .body = {.position = {-4.0f, 1.5f}, .angularDamping = 0.05f},
      .shape =
          physeng::PolygonShape({{-0.55f, -0.35f}, {0.55f, -0.35f}, {0.4f, 0.35f}, {-0.3f, 0.6f}}),
      .material = {.restitution = 0.15f, .friction = 0.5f},
  });

  scene.springs.push_back({
      .first = "spring_anchor",
      .second = "spring_weight",
      .firstAnchor = {0.0f, -0.12f},
      .restLength = 1.4f,
      .stiffness = 40.0f,
      .damping = 2.0f,
  });

  scene.ropes.push_back({
      .settings =
          {
              .segmentCount = 8,
              .nodeRadius = 0.08f,
              .totalMass = 0.7f,
              .enableCollisions = true,
              .linearDamping = 0.1f,
              .startAnchor = {0.0f, -0.12f},
              .endAnchor = {0.0f, 0.35f},
          },
      .startBody = "rope_anchor",
      .endBody = "rope_weight",
  });

  return scene;
}

} // namespace app
