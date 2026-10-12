#pragma once

#include "common/vec2.hpp"
#include <cmath>

namespace physeng::detail {

using common::Vec2;

inline Vec2 Add(Vec2 a, Vec2 b) {
  return {a.x + b.x, a.y + b.y};
}

inline Vec2 Sub(Vec2 a, Vec2 b) {
  return {a.x - b.x, a.y - b.y};
}

inline Vec2 Scale(Vec2 v, float s) {
  return {v.x * s, v.y * s};
}

inline float Dot(Vec2 a, Vec2 b) {
  return a.x * b.x + a.y * b.y;
}

inline float Cross(Vec2 a, Vec2 b) {
  return a.x * b.y - a.y * b.x;
}

inline Vec2 Perpendicular(Vec2 v) {
  return {-v.y, v.x};
}

inline Vec2 Rotate(Vec2 v, float angle) {
  const float c = std::cos(angle), s = std::sin(angle);
  return {c * v.x - s * v.y, s * v.x + c * v.y};
}

inline float Length(Vec2 v) {
  return std::sqrt(Dot(v, v));
}

inline bool Finite(Vec2 v) {
  return std::isfinite(v.x) && std::isfinite(v.y);
}

inline bool Positive(float v) {
  return std::isfinite(v) && v > 0.0f;
}

inline bool Nonnegative(float v) {
  return std::isfinite(v) && v >= 0.0f;
}
} // namespace physeng::detail
