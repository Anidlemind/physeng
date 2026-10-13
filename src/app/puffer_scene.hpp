#pragma once

#include "physeng/scene.hpp"

namespace app {

struct PufferSettings {
  float squeezedRadius = 0.5f;
  float inflatedRadius = 0.9f;
  float inflateSpeed = 24.0f;
  float squeezeSpeed = 20.0f;
  float moveSpeed = 4.2f;
  float moveAcceleration = 30.0f;
  float jumpSpeed = 8.0f;
};

struct PufferControlState {
  bool previousInflate = false;
  bool grounded = false;
  float groundGrace = 0;
  float jumpBuffer = 0;
};

physeng::SceneDesc MakePufferScene(const PufferSettings& settings = {});
physeng::SceneDesc MakePufferWaterScene(const PufferSettings& settings = {});
void UpdatePuffer(physeng::Scene& scene,
                  const PufferSettings& settings,
                  float horizontal,
                  bool inflate,
                  double timeStep,
                  PufferControlState& control);
void LimitPufferRiseSpeed(physeng::Scene& scene, const PufferSettings& settings);
float PufferInflation(const physeng::objects::SoftBodyHandle& body, const PufferSettings& settings);

} // namespace app
