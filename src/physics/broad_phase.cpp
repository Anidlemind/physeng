#include "detail/math.hpp"
#include "physeng/world.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace physeng {

World::CollisionBounds World::ComputeCollisionBounds(ColliderId id, float padding) const {
  const auto& collider = colliders_[id];
  const auto& body = bodies_[collider.body];
  auto bounds = std::visit(
      [&](const auto& shape) -> CollisionBounds {
        using T = std::decay_t<decltype(shape)>;
        if constexpr (std::is_same_v<T, PolygonShape>) {
          const float infinity = std::numeric_limits<float>::infinity();
          CollisionBounds result{{infinity, infinity}, {-infinity, -infinity}};
          for (auto local : shape.Vertices()) {
            const auto v = body.LocalToWorld(local);
            result.minimum.x = std::min(result.minimum.x, v.x);
            result.minimum.y = std::min(result.minimum.y, v.y);
            result.maximum.x = std::max(result.maximum.x, v.x);
            result.maximum.y = std::max(result.maximum.y, v.y);
          }
          return result;
        } else {
          common::Vec2 extent;
          if constexpr (std::is_same_v<T, CircleShape>) {
            extent = {shape.radius, shape.radius};
          } else {
            const float c = std::abs(std::cos(body.angle)), s = std::abs(std::sin(body.angle));
            extent = {c * shape.halfExtents.x + s * shape.halfExtents.y,
                      s * shape.halfExtents.x + c * shape.halfExtents.y};
          }
          return {detail::Sub(body.position, extent), detail::Add(body.position, extent)};
        }
      },
      collider.shape);
  if (body.inverseMass > 0.0f && previousStates_.size() == bodies_.size()) {
    if (const auto* circle = std::get_if<CircleShape>(&collider.shape)) {
      const auto previous = previousStates_[collider.body].position;
      bounds.minimum.x = std::min(bounds.minimum.x, previous.x - circle->radius);
      bounds.minimum.y = std::min(bounds.minimum.y, previous.y - circle->radius);
      bounds.maximum.x = std::max(bounds.maximum.x, previous.x + circle->radius);
      bounds.maximum.y = std::max(bounds.maximum.y, previous.y + circle->radius);
    }
  }
  return {detail::Sub(bounds.minimum, {padding, padding}),
          detail::Add(bounds.maximum, {padding, padding})};
}

bool World::CollisionPairsNeedRebuild() const {
  if (collisionBounds_.size() != colliders_.size()) {
    return true;
  }
  for (ColliderId id = 0; id < colliders_.size(); ++id) {
    if (bodies_[colliders_[id].body].inverseMass == 0.0f) {
      continue;
    }
    const auto current = ComputeCollisionBounds(id, 0.001f);
    const auto& cached = collisionBounds_[id];
    if (current.minimum.x < cached.minimum.x || current.minimum.y < cached.minimum.y ||
        current.maximum.x > cached.maximum.x || current.maximum.y > cached.maximum.y) {
      return true;
    }
  }
  return false;
}

void World::BuildCollisionPairs() {
  constexpr float padding = 0.051f; // Contact tolerance plus a 0.05-unit movement margin.
  constexpr std::int64_t maxCells = 64;
  // Keep floating-point cell coordinates well inside the exact integer range.
  constexpr double coordinateLimit = 9.0e15;
  collisionBounds_.resize(colliders_.size());
  cellOrigins_.resize(colliders_.size());
  largeColliderFlags_.assign(colliders_.size(), false);
  cellEntries_.clear();
  largeColliders_.clear();
  previousCollisionPairs_.swap(collisionPairs_);
  previousContactStates_.swap(contactStates_);
  collisionPairs_.clear();
  contactStates_.clear();

  for (ColliderId id = 0; id < colliders_.size(); ++id) {
    collisionBounds_[id] = ComputeCollisionBounds(id, padding);
  }
  for (ColliderId id = 0; id < colliders_.size(); ++id) {
    const auto& expanded = collisionBounds_[id];
    const double x0 =
        std::floor(static_cast<double>(expanded.minimum.x) / settings_.broadPhaseCellSize);
    const double x1 =
        std::floor(static_cast<double>(expanded.maximum.x) / settings_.broadPhaseCellSize);
    const double y0 =
        std::floor(static_cast<double>(expanded.minimum.y) / settings_.broadPhaseCellSize);
    const double y1 =
        std::floor(static_cast<double>(expanded.maximum.y) / settings_.broadPhaseCellSize);
    if (!std::isfinite(x0) || !std::isfinite(x1) || !std::isfinite(y0) || !std::isfinite(y1) ||
        std::abs(x0) > coordinateLimit || std::abs(x1) > coordinateLimit ||
        std::abs(y0) > coordinateLimit || std::abs(y1) > coordinateLimit) {
      largeColliders_.push_back(id);
      largeColliderFlags_[id] = true;
      continue;
    }
    const auto minX = static_cast<std::int64_t>(x0), maxX = static_cast<std::int64_t>(x1);
    const auto minY = static_cast<std::int64_t>(y0), maxY = static_cast<std::int64_t>(y1);
    const auto width = maxX - minX + 1, height = maxY - minY + 1;
    if (width > maxCells || height > maxCells || width * height > maxCells) {
      // Large walls must not fill thousands of cells. Check their bounds directly.
      largeColliders_.push_back(id);
      largeColliderFlags_[id] = true;
      continue;
    }
    cellOrigins_[id] = {minX, minY};
    for (auto x = minX; x <= maxX; ++x) {
      for (auto y = minY; y <= maxY; ++y) {
        cellEntries_.push_back({x, y, id});
      }
    }
  }

  const auto addPair = [&](ColliderId first, ColliderId second) {
    if (first == second) {
      return;
    }
    const auto bodyA = colliders_[first].body, bodyB = colliders_[second].body;
    if (bodyA == bodyB || (bodies_[bodyA].inverseMass == 0 && bodies_[bodyB].inverseMass == 0)) {
      return;
    }
    const auto& a = collisionBounds_[first];
    const auto& b = collisionBounds_[second];
    if (a.minimum.x > b.maximum.x || b.minimum.x > a.maximum.x || a.minimum.y > b.maximum.y ||
        b.minimum.y > a.maximum.y) {
      return;
    }
    collisionPairs_.push_back({std::min(first, second), std::max(first, second)});
  };
  std::sort(cellEntries_.begin(), cellEntries_.end(), [](const auto& a, const auto& b) {
    if (a.x != b.x) {
      return a.x < b.x;
    }
    if (a.y != b.y) {
      return a.y < b.y;
    }
    return a.collider < b.collider;
  });
  for (std::size_t begin = 0; begin < cellEntries_.size();) {
    std::size_t end = begin + 1;
    while (end < cellEntries_.size() && cellEntries_[end].x == cellEntries_[begin].x &&
           cellEntries_[end].y == cellEntries_[begin].y) {
      ++end;
    }
    for (std::size_t i = begin; i < end; ++i) {
      for (std::size_t j = i + 1; j < end; ++j) {
        const auto first = cellEntries_[i].collider, second = cellEntries_[j].collider;
        // Emit a pair only in the first shared cell, avoiding duplicates at source.
        if (cellEntries_[begin].x == std::max(cellOrigins_[first].x, cellOrigins_[second].x) &&
            cellEntries_[begin].y == std::max(cellOrigins_[first].y, cellOrigins_[second].y)) {
          addPair(first, second);
        }
      }
    }
    begin = end;
  }
  for (auto large : largeColliders_) {
    for (ColliderId other = 0; other < colliders_.size(); ++other) {
      if (other < large && largeColliderFlags_[other]) {
        continue;
      }
      addPair(large, other);
    }
  }
  // Preserve a deterministic collider-pair solve order.
  std::sort(collisionPairs_.begin(), collisionPairs_.end(), [](const auto& a, const auto& b) {
    return a.first != b.first ? a.first < b.first : a.second < b.second;
  });
  contactStates_.resize(collisionPairs_.size());
  staticCollisionPairs_.clear();
  // Pair order is stable, so a linear merge keeps impulses through grid rebuilds.
  std::size_t previous = 0;
  const auto less = [](const auto& a, const auto& b) {
    return a.first != b.first ? a.first < b.first : a.second < b.second;
  };
  for (std::size_t current = 0; current < collisionPairs_.size(); ++current) {
    const auto& pair = collisionPairs_[current];
    if (bodies_[colliders_[pair.first].body].inverseMass == 0.0f ||
        bodies_[colliders_[pair.second].body].inverseMass == 0.0f) {
      staticCollisionPairs_.push_back(current);
    }
    while (previous < previousCollisionPairs_.size() &&
           less(previousCollisionPairs_[previous], collisionPairs_[current])) {
      ++previous;
    }
    if (previous < previousCollisionPairs_.size() &&
        previousCollisionPairs_[previous] == collisionPairs_[current]) {
      contactStates_[current] = previousContactStates_[previous];
    }
  }
}

} // namespace physeng
