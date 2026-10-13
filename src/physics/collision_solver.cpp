#include "detail/collision.hpp"
#include "physeng/world.hpp"

namespace physeng {
using namespace detail;

void World::SolveCollisionPair(std::size_t index, bool warmStartOnly) {
  const auto& pair = collisionPairs_[index];
  const auto& ca = colliders_[pair.first];
  const auto& cb = colliders_[pair.second];
  if ((!disabledCollisionPairs_.empty() || !connectedCollisionPairs_.empty()) &&
      !CanCollide(ca.body, cb.body)) {
    contactStates_[index] = {};
    return;
  }
  auto& a = bodies_[ca.body];
  auto& b = bodies_[cb.body];
  // Avoid building a general manifold for the stress demo's circle pairs.
  const auto* circleA = std::get_if<CircleShape>(&ca.shape);
  const auto* circleB = std::get_if<CircleShape>(&cb.shape);
  if (circleA && circleB) {
    ResolveCircleContact(a,
                         b,
                         *circleA,
                         *circleB,
                         ca.material,
                         cb.material,
                         settings_.restitutionThreshold,
                         impactStates_[ca.body],
                         impactStates_[cb.body],
                         &contactStates_[index],
                         warmStartOnly);
    return;
  }
  auto contact =
      std::visit([&](const auto& sa, const auto& sb) { return ContactBetween(a, sa, b, sb); },
                 ca.shape,
                 cb.shape);
  if (previousStates_.size() == bodies_.size()) {
    if (circleA && b.inverseMass == 0.0f) {
      if (const auto* box = std::get_if<BoxShape>(&cb.shape)) {
        if (auto swept = SweptCircleBoxContact(previousStates_[ca.body], a, *circleA, b, *box)) {
          contact = swept;
        }
      }
    } else if (circleB && a.inverseMass == 0.0f) {
      if (const auto* box = std::get_if<BoxShape>(&ca.shape)) {
        if (auto swept = SweptCircleBoxContact(previousStates_[cb.body], b, *circleB, a, *box)) {
          swept->normal = {-swept->normal.x, -swept->normal.y};
          contact = swept;
        }
      }
    }
  }
  if (contact) {
    ResolveContact(a,
                   b,
                   *contact,
                   ca.material,
                   cb.material,
                   settings_.restitutionThreshold,
                   impactStates_[ca.body],
                   impactStates_[cb.body],
                   &contactStates_[index],
                   warmStartOnly);
  } else {
    contactStates_[index] = {};
  }
}

void World::SolveCollisions(bool warmStartOnly) {
  for (std::size_t index = 0; index < collisionPairs_.size(); ++index) {
    SolveCollisionPair(index, warmStartOnly);
  }
}

} // namespace physeng
