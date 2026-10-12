#include "physeng/shape.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace physeng {
namespace {
double Cross(common::Vec2 a, common::Vec2 b) {
  return static_cast<double>(a.x) * b.y - static_cast<double>(a.y) * b.x;
}

double Dot(common::Vec2 a, common::Vec2 b) {
  return static_cast<double>(a.x) * b.x + static_cast<double>(a.y) * b.y;
}
} // namespace

PolygonShape::PolygonShape(std::vector<common::Vec2> vertices) : vertices_(std::move(vertices)) {
  if (vertices_.size() < 3) {
    throw std::invalid_argument("A polygon needs at least three vertices");
  }
  for (auto v : vertices_) {
    if (!std::isfinite(v.x) || !std::isfinite(v.y)) {
      throw std::invalid_argument("Polygon vertices must be finite");
    }
  }
  // Translate before the area sums to avoid cancellation for offset input.
  const auto origin = vertices_.front();
  double extent = 0.0;
  for (auto& v : vertices_) {
    v = {static_cast<float>(static_cast<double>(v.x) - origin.x),
         static_cast<float>(static_cast<double>(v.y) - origin.y)};
    if (!std::isfinite(v.x) || !std::isfinite(v.y)) {
      throw std::invalid_argument("Polygon dimensions exceed float range");
    }
    extent = std::max(
        extent, std::max(std::abs(static_cast<double>(v.x)), std::abs(static_cast<double>(v.y))));
  }
  double areaTwice = 0.0;
  for (std::size_t i = 0; i < vertices_.size(); ++i) {
    areaTwice += Cross(vertices_[i], vertices_[(i + 1) % vertices_.size()]);
  }
  if (areaTwice < 0.0) {
    std::reverse(vertices_.begin(), vertices_.end());
    areaTwice = -areaTwice;
  }
  const double tolerance = extent * extent * 1.0e-7;
  if (areaTwice <= tolerance) {
    throw std::invalid_argument("Polygon must have nonzero area");
  }
  // Every other vertex must lie strictly left of each counterclockwise edge.
  // This rejects concavity, repeated vertices, collinear corners and crossed edges.
  for (std::size_t i = 0; i < vertices_.size(); ++i) {
    const auto a = vertices_[i], b = vertices_[(i + 1) % vertices_.size()];
    for (std::size_t j = 0; j < vertices_.size(); ++j) {
      if (j == i || j == (i + 1) % vertices_.size()) {
        continue;
      }
      const auto v = vertices_[j];
      const double side = (static_cast<double>(b.x) - a.x) * (static_cast<double>(v.y) - a.y) -
                          (static_cast<double>(b.y) - a.y) * (static_cast<double>(v.x) - a.x);
      if (side <= tolerance) {
        throw std::invalid_argument("Polygon boundary must be strictly convex");
      }
    }
  }
  double cx = 0.0, cy = 0.0;
  for (std::size_t i = 0; i < vertices_.size(); ++i) {
    const auto a = vertices_[i], b = vertices_[(i + 1) % vertices_.size()];
    const double cross = Cross(a, b);
    cx += (static_cast<double>(a.x) + b.x) * cross;
    cy += (static_cast<double>(a.y) + b.y) * cross;
  }
  cx /= 3.0 * areaTwice;
  cy /= 3.0 * areaTwice;
  for (auto& v : vertices_) {
    v = {static_cast<float>(v.x - cx), static_cast<float>(v.y - cy)};
    if (!std::isfinite(v.x) || !std::isfinite(v.y)) {
      throw std::invalid_argument("Centered polygon vertices exceed float range");
    }
  }
  double centeredArea = 0.0, moment = 0.0;
  for (std::size_t i = 0; i < vertices_.size(); ++i) {
    const auto a = vertices_[i], b = vertices_[(i + 1) % vertices_.size()];
    const double cross = Cross(a, b);
    centeredArea += cross;
    moment += cross * (Dot(a, a) + Dot(a, b) + Dot(b, b));
  }
  inertiaPerMass_ = moment / (6.0 * centeredArea);
  if (!std::isfinite(inertiaPerMass_) || inertiaPerMass_ <= 0.0) {
    throw std::invalid_argument("Polygon inertia is not representable");
  }
}
} // namespace physeng
