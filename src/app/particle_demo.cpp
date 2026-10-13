#include "particle_demo.hpp"
#include "renderer/renderer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <variant>

namespace app {
namespace {
constexpr common::Vec2 enclosureHalfExtents{12.0f, 7.0f};
constexpr float particleRadius = 0.1f;
constexpr std::size_t particleLimit = 8000;
constexpr double measurementWindow = 0.5;
constexpr double warmupSeconds = 1.0;
constexpr double particlesPerSecond = 60.0;
constexpr float particleMass = 1.0f;
constexpr common::Vec2 spawnPoint{-10.8f, 6.0f}; // Above the pile at the target load.
constexpr common::Vec2 launchImpulse{18.0f, 5.0f};

physeng::SceneDesc MakeParticleScene() {
  physeng::SceneDesc desc;
  desc.physics.floorY = std::nullopt; // The enclosure supplies the actual floor.
  desc.physics.broadPhaseCellSize = 0.25f;
  desc.enclosures.push_back(
      {.halfExtents = enclosureHalfExtents, .wallThickness = 0.3f, .material = {.friction = 0.5f}});
  return desc;
}
} // namespace

ParticleDemo::ParticleDemo() : scene_(physeng::CreateScene(MakeParticleScene())) {
}

void ParticleDemo::Update(double frameSeconds) {
  if (!std::isfinite(frameSeconds) || frameSeconds <= 0.0) {
    return;
  }
  elapsed_ += frameSeconds;
  sampleSeconds_ += frameSeconds;
  ++sampleFrames_;
  if (frameSeconds_ == 0.0) {
    frameSeconds_ = frameSeconds;
  }
  if (sampleSeconds_ >= measurementWindow) {
    frameSeconds_ = sampleSeconds_ / sampleFrames_;
    if (elapsed_ + 1.0e-9 >= warmupSeconds) {
      warmingUp_ = false;
    }
    sampleSeconds_ = 0.0;
    sampleFrames_ = 0;
  }
}

void ParticleDemo::Step(double timeStep) {
  if (!std::isfinite(timeStep) || timeStep <= 0.0) {
    throw std::invalid_argument("Particle physics step must be positive and finite");
  }
  if (!warmingUp_ && !stopped_) {
    // Emit between physics steps so a slow render frame cannot spawn an overlapping batch.
    spawnBudget_ += timeStep * particlesPerSecond;
    while (spawnBudget_ >= 1.0 && particleCount_ < particleLimit) {
      if (!SpawnPointIsClear()) {
        spawnBudget_ = 1.0; // Wait for room without accumulating a burst of emissions.
        break;
      }
      SpawnParticle();
      spawnBudget_ -= 1.0;
    }
    stopped_ = particleCount_ == particleLimit;
    if (stopped_) { spawnBudget_ = 0.0; }
  }
  scene_.world.Step(timeStep);
}

bool ParticleDemo::SpawnPointIsClear() const {
  for (const auto& collider : scene_.world.Colliders()) {
    if (const auto* circle = std::get_if<physeng::CircleShape>(&collider.shape)) {
      const auto position = scene_.world.GetBody(collider.body).position;
      const float dx = position.x - spawnPoint.x, dy = position.y - spawnPoint.y;
      const float clearance = particleRadius + circle->radius;
      if (dx * dx + dy * dy < clearance * clearance) { return false; }
    }
  }
  return true;
}

void ParticleDemo::SpawnParticle() {
  const auto id = scene_.world.AddBody({.position = spawnPoint,
                                        .inverseMass = 1.0f / particleMass,
                                        .linearDamping = 0.05f,
                                        .angularDamping = 0.1f},
                                       physeng::CircleShape{particleRadius},
                                       {.restitution = 0.15f, .friction = 0.5f});
  scene_.world.ApplyImpulse(id, launchImpulse);
  ++particleCount_;
}

void ParticleDemo::Draw(renderer::Renderer& renderer) {
  renderer.BeginFrame();
  renderer.SetColor({0.32f, 0.36f, 0.42f, 1.0f});
  for (const auto& collider : scene_.world.Colliders()) {
    if (const auto* box = std::get_if<physeng::BoxShape>(&collider.shape)) {
      const auto& body = scene_.world.GetBody(collider.body);
      renderer.DrawRectangle(body.position, box->halfExtents, body.angle);
    }
  }
  renderer.SetColor({0.40f, 0.65f, 0.79f, 1.0f});
  circleInstances_.clear();
  for (const auto& collider : scene_.world.Colliders()) {
    if (const auto* circle = std::get_if<physeng::CircleShape>(&collider.shape)) {
      const auto& body = scene_.world.GetBody(collider.body);
      circleInstances_.push_back({body.position, circle->radius, body.angle});
    }
  }

  renderer.DrawCircles(circleInstances_);
  renderer.SetColor({0.98f, 0.65f, 0.35f, 1.0f});
  const float launchMagnitude = std::hypot(launchImpulse.x, launchImpulse.y);
  if (launchMagnitude > 0.0f) {
    renderer.DrawSegment({spawnPoint.x - 0.5f * launchImpulse.x / launchMagnitude,
                          spawnPoint.y - 0.5f * launchImpulse.y / launchMagnitude},
                         spawnPoint,
                         0.15f);
  }

  renderer.SetColor({0.04f, 0.05f, 0.07f, 0.94f});
  renderer.DrawScreenRectangle({12, 12}, {300, 108});
  char text[192];
  std::snprintf(text,
                sizeof(text),
                "PARTICLES: %zu / %zu\nFRAME: %.2f MS\nFPS: %.1f\n%s",
                particleCount_,
                particleLimit,
                FrameMilliseconds(),
                FPS(),
                stopped_     ? "TARGET REACHED"
                : warmingUp_ ? "WARMING UP"
                             : "SPAWNING");
  renderer.SetColor(stopped_ ? renderer::Color{0.98f, 0.65f, 0.35f, 1.0f}
                             : renderer::Color{0.87f, 0.91f, 0.96f, 1.0f});
  renderer.DrawText({24, 24}, text);
  renderer.EndFrame();
}

void RunParticleDemo(renderer::Renderer& renderer) {
  ParticleDemo demo;
  renderer.SetVisibleHeight(18.0f);
  renderer.SetVSync(false);
  std::printf("Particle stress test: spawning %zu particles, then continuing the simulation.\n",
              particleLimit);

  using Clock = std::chrono::steady_clock;
  auto previousFrame = Clock::now();
  double accumulator = 0.0;
  constexpr double timeStep = 1.0 / 120.0;
  while (!renderer.ShouldClose()) {
    renderer.PollEvents();
    const auto now = Clock::now();
    const double frameSeconds = std::chrono::duration<double>(now - previousFrame).count();
    previousFrame = now;
    demo.Update(frameSeconds);
    accumulator += std::min(frameSeconds, 0.1);
    while (accumulator >= timeStep) {
      demo.Step(timeStep);
      accumulator -= timeStep;
    }
    demo.Draw(renderer);
  }
}

} // namespace app
