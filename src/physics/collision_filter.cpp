#include "physeng/world.hpp"

#include <algorithm>
#include <stdexcept>

namespace physeng {

void World::SetCollisionEnabled(BodyId first, BodyId second, bool enabled) {
  if (first >= bodies_.size() || second >= bodies_.size()) {
    throw std::out_of_range("Collision filter must reference existing bodies");
  }
  if (first == second) {
    return;
  }
  const BodyPair pair{std::min(first, second), std::max(first, second)};
  if (enabled) {
    disabledCollisionPairs_.erase(pair);
  } else {
    disabledCollisionPairs_.insert(pair);
  }
}

void World::DisableCollisionWhileConnected(BodyId first,
                                           BodyId second,
                                           std::span<const DistanceConstraintId> path) {
  if (first >= bodies_.size() || second >= bodies_.size()) {
    throw std::out_of_range("Collision filter must reference existing bodies");
  }
  if (path.empty() || first == second) {
    throw std::invalid_argument("Connected collision exclusion needs distinct bodies and a path");
  }
  BodyId current = first;
  bool intact = true;
  for (auto id : path) {
    if (id >= constraints_.size()) {
      throw std::out_of_range("Collision exclusion path references an unknown constraint");
    }
    const auto& link = constraints_[id];
    if (link.first == current) {
      current = link.second;
    } else if (link.second == current) {
      current = link.first;
    } else {
      throw std::invalid_argument("Collision exclusion path must be a connected chain");
    }
    intact = intact && !link.broken;
  }
  if (current != second) {
    throw std::invalid_argument("Collision exclusion path must connect the given bodies");
  }
  if (!intact) {
    return;
  }

  const BodyPair pair{std::min(first, second), std::max(first, second)};
  connectedCollisionExclusions_.push_back({pair, {path.begin(), path.end()}});
  try {
    ++connectedCollisionPairs_[pair];
  } catch (...) {
    connectedCollisionExclusions_.pop_back();
    throw;
  }
}

void World::RefreshConnectedCollisionExclusions() {
  for (auto& exclusion : connectedCollisionExclusions_) {
    if (!exclusion.active) {
      continue;
    }
    const bool broken = std::any_of(exclusion.path.begin(), exclusion.path.end(), [&](auto id) {
      return constraints_[id].broken;
    });
    if (!broken) {
      continue;
    }
    exclusion.active = false;
    const auto found = connectedCollisionPairs_.find(exclusion.pair);
    if (--found->second == 0) {
      connectedCollisionPairs_.erase(found);
    }
  }
}

bool World::CanCollide(BodyId first, BodyId second) const {
  if (first >= bodies_.size() || second >= bodies_.size()) {
    throw std::out_of_range("Collision filter must reference existing bodies");
  }
  const BodyPair pair{std::min(first, second), std::max(first, second)};
  return first != second && !disabledCollisionPairs_.contains(pair) &&
         !connectedCollisionPairs_.contains(pair);
}

} // namespace physeng
