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

bool World::CanCollide(BodyId first, BodyId second) const {
  if (first >= bodies_.size() || second >= bodies_.size()) {
    throw std::out_of_range("Collision filter must reference existing bodies");
  }
  return first != second &&
         !disabledCollisionPairs_.contains({std::min(first, second), std::max(first, second)});
}

} // namespace physeng
