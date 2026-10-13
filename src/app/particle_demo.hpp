#pragma once

#include "physeng/scene.hpp"
#include "renderer/renderer.hpp"

#include <cstddef>

namespace app {

class ParticleDemo {
public:
  ParticleDemo();
  void Update(double frameSeconds);
  void Step(double timeStep);
  void Draw(renderer::Renderer& renderer);

  physeng::World& World() { return scene_.world; }

  const physeng::World& World() const { return scene_.world; }

  std::size_t ParticleCount() const { return particleCount_; }

  double FrameMilliseconds() const { return frameSeconds_ * 1000.0; }

  double FPS() const { return frameSeconds_ > 0.0 ? 1.0 / frameSeconds_ : 0.0; }

  bool Stopped() const { return stopped_; }

private:
  bool SpawnPointIsClear() const;
  void SpawnParticle();

  physeng::Scene scene_;
  std::vector<renderer::CircleInstance> circleInstances_;
  std::size_t particleCount_ = 0;
  std::size_t sampleFrames_ = 0;
  double elapsed_ = 0.0;
  double sampleSeconds_ = 0.0;
  double frameSeconds_ = 0.0;
  double spawnBudget_ = 0.0;
  bool warmingUp_ = true;
  bool stopped_ = false;
};

void RunParticleDemo(renderer::Renderer& renderer);

} // namespace app
