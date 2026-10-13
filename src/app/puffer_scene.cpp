#include "puffer_scene.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace app {
namespace {
void ValidateSettings(const PufferSettings& s) {
  if (!std::isfinite(s.squeezedRadius) || !std::isfinite(s.inflatedRadius) ||
      !std::isfinite(s.inflateSpeed) || !std::isfinite(s.squeezeSpeed) ||
      !std::isfinite(s.moveSpeed) || !std::isfinite(s.moveAcceleration) ||
      !std::isfinite(s.jumpSpeed) || s.squeezedRadius <= 0 ||
      s.inflatedRadius <= s.squeezedRadius || s.inflateSpeed <= 0 || s.squeezeSpeed <= 0 ||
      s.moveSpeed < 0 || s.moveAcceleration < 0 || s.jumpSpeed <= 0) {
    throw std::invalid_argument("Invalid puffer control settings");
  }
}
} // namespace

physeng::SceneDesc MakePufferScene(const PufferSettings& settings) {
  ValidateSettings(settings);
  physeng::SceneDesc scene;
  scene.physics.solverIterations = 24;
  scene.enclosures.push_back(
      {.halfExtents = {8, 4.5f}, .wallThickness = 0.3f, .material = {.friction = 0.6f}});
  scene.softBodies.push_back({.position = {-6.5f, -4.5f + settings.squeezedRadius * 0.9f},
                              .radius = settings.squeezedRadius,
                              .totalMass = 3,
                              .compliance = 0.0003f});
  scene.bodies = {
      {.name = "first_step",
       .body = {.position = {-4.2f, -4.15f}, .inverseMass = 0},
       .shape = physeng::BoxShape{{0.8f, 0.35f}},
       .material = {.friction = 0.6f}},
      {.name = "middle_platform",
       .body = {.position = {-1.8f, -2.75f}, .inverseMass = 0},
       .shape = physeng::BoxShape{{0.85f, 0.25f}},
       .material = {.friction = 0.6f}},
      {.name = "upper_platform",
       .body = {.position = {-4.2f, -0.85f}, .inverseMass = 0},
       .shape = physeng::BoxShape{{1.1f, 0.2f}},
       .material = {.friction = 0.6f}},
      {.name = "tunnel_roof",
       .body = {.position = {2, -3.1f}, .inverseMass = 0},
       .shape = physeng::BoxShape{{1.65f, 0.25f}},
       .material = {.friction = 0.8f}},
      {.name = "slot_left",
       .body = {.position = {4.6f, -3.05f}, .inverseMass = 0},
       .shape = physeng::BoxShape{{0.2f, 1.45f}},
       .material = {.friction = 0.8f}},
      {.name = "slot_right",
       .body = {.position = {6.6f, -3.05f}, .inverseMass = 0},
       .shape = physeng::BoxShape{{0.2f, 1.45f}},
       .material = {.friction = 0.8f}},
  };
  return scene;
}

physeng::SceneDesc MakePufferWaterScene(const PufferSettings& settings) {
  ValidateSettings(settings);
  physeng::SceneDesc scene;
  scene.physics.solverIterations = 16;
  scene.enclosures.push_back(
      {.halfExtents = {6, 4}, .wallThickness = 0.25f, .material = {.friction = 0.3f}});
  scene.softBodies.push_back({.position = {-3, -1.8f},
                              .radius = settings.squeezedRadius,
                              .totalMass = 3,
                              .compliance = 0.0003f});
  scene.fluids.push_back({
      .settings = {.minimum = {-6, -4},
                   .maximum = {6, 4},
                   .particleSpacing = 0.3f,
                   .smoothingRadius = 0.9f,
                   .restDensity = 2,
                   .substeps = 1},
      .blocks = {{.minimum = {-5.7f, -3.7f}, .columns = 39, .rows = 12}},
  });
  return scene;
}

float PufferInflation(const physeng::objects::SoftBodyHandle& body,
                      const PufferSettings& settings) {
  ValidateSettings(settings);
  return std::clamp((body.radius - settings.squeezedRadius) /
                        (settings.inflatedRadius - settings.squeezedRadius),
                    0.0f,
                    1.0f);
}

void UpdatePuffer(physeng::Scene& scene,
                  const PufferSettings& settings,
                  float horizontal,
                  bool inflate,
                  double timeStep,
                  PufferControlState& control) {
  ValidateSettings(settings);
  auto& body = scene.softBodies.at(0);
  const float dt = static_cast<float>(timeStep);
  if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(horizontal)) {
    throw std::invalid_argument("Puffer controls need a finite positive step and finite input");
  }
  const auto velocity = body.Velocity(scene.world);
  control.grounded =
      velocity.y <= 1.0f && std::any_of(body.nodes.begin(), body.nodes.end(), [&](auto id) {
        return scene.world.IsGrounded(id, 0.05f, body.nodes);
      });
  control.groundGrace = control.grounded ? 0.08f : std::max(0.0f, control.groundGrace - dt);
  control.jumpBuffer =
      inflate && !control.previousInflate ? 0.12f : std::max(0.0f, control.jumpBuffer - dt);
  // A fresh puff launches the mesh from actual support; holding it never creates extra jumps.
  if (control.jumpBuffer > 0 && control.groundGrace > 0) {
    const float change = settings.jumpSpeed - velocity.y;
    for (auto id : body.nodes) {
      scene.world.ApplyImpulse(id, {0, change / scene.world.GetBody(id).inverseMass});
    }
    control.jumpBuffer = 0;
    control.groundGrace = 0;
    control.grounded = false;
  }
  control.previousInflate = inflate;
  const float target = inflate ? settings.inflatedRadius : settings.squeezedRadius;
  const float speed = inflate ? settings.inflateSpeed : settings.squeezeSpeed;
  const float change = std::clamp(target - body.radius, -speed * dt, speed * dt);
  physeng::objects::SetSoftBodyRadius(scene.world, body, body.radius + change);

  const float targetVelocity = std::clamp(horizontal, -1.0f, 1.0f) * settings.moveSpeed;
  const float acceleration = std::clamp((targetVelocity - body.Velocity(scene.world).x) * 6.0f,
                                        -settings.moveAcceleration,
                                        settings.moveAcceleration);
  for (auto id : body.nodes) {
    const auto& node = scene.world.GetBody(id);
    scene.world.ApplyForce(id, {acceleration / node.inverseMass, 0});
  }
}

void LimitPufferRiseSpeed(physeng::Scene& scene, const PufferSettings& settings) {
  const auto& body = scene.softBodies.front();
  const float excess = body.Velocity(scene.world).y - settings.jumpSpeed;
  if (excess > 0) {
    // Expansion against the floor can add another kick. Keep jump height controllable.
    for (auto id : body.nodes) {
      scene.world.ApplyImpulse(id, {0, -excess / scene.world.GetBody(id).inverseMass});
    }
  }
}

} // namespace app
