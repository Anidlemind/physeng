#pragma once

#include "renderer/renderer.hpp"

namespace app {

class WindowMotion {
public:
  using Clock = std::chrono::steady_clock;

  common::Vec2 Update(std::optional<renderer::WindowPositionSample> position,
                      Clock::time_point now);

private:
  std::optional<renderer::WindowPositionSample> previousPosition_;
  Clock::time_point previousUpdate_;
  common::Vec2 targetVelocity_;
  common::Vec2 velocity_;
};

} // namespace app
