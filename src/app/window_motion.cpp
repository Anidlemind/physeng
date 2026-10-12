#include "window_motion.hpp"

#include <algorithm>
#include <cmath>

namespace app {
namespace {
constexpr double screenUnitsPerMeter = 100.0;
constexpr double smoothingTime = 0.04;
constexpr double stopDelay = 0.06;
constexpr float maxSpeed = 5.0f;
} // namespace

common::Vec2 WindowMotion::Update(std::optional<renderer::WindowPositionSample> position,
                                  Clock::time_point now) {
  if (!position) {
    previousPosition_.reset();
    targetVelocity_ = {};
    velocity_ = {};
    return {};
  }
  if (!previousPosition_) {
    previousPosition_ = position;
    previousUpdate_ = now;
    return {};
  }

  const double frameTime =
      std::clamp(std::chrono::duration<double>(now - previousUpdate_).count(), 0.0, 0.1);
  previousUpdate_ = now;

  if (position->timestamp > previousPosition_->timestamp) {
    const double elapsed = std::max(
        std::chrono::duration<double>(position->timestamp - previousPosition_->timestamp).count(),
        0.001);
    const double scale = screenUnitsPerMeter * elapsed;
    targetVelocity_ = {
        static_cast<float>((static_cast<double>(position->x) - previousPosition_->x) / scale),
        // Desktop Y points down; physics Y points up.
        static_cast<float>(-(static_cast<double>(position->y) - previousPosition_->y) / scale)};
    const float speed = std::hypot(targetVelocity_.x, targetVelocity_.y);
    if (speed > maxSpeed) {
      targetVelocity_.x *= maxSpeed / speed;
      targetVelocity_.y *= maxSpeed / speed;
    }
    previousPosition_ = position;
  }

  // No movement events means the window has stopped, rather than kept its last speed.
  if (std::chrono::duration<double>(now - position->timestamp).count() > stopDelay) {
    targetVelocity_ = {};
  }

  const common::Vec2 previousVelocity = velocity_;
  const float blend = static_cast<float>(1.0 - std::exp(-frameTime / smoothingTime));
  velocity_.x += (targetVelocity_.x - velocity_.x) * blend;
  velocity_.y += (targetVelocity_.y - velocity_.y) * blend;
  return {velocity_.x - previousVelocity.x, velocity_.y - previousVelocity.y};
}

} // namespace app
