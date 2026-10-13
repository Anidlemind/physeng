#pragma once

#include "physeng/body.hpp"
#include "physeng/constraint.hpp"

namespace physeng::detail {

void SolveDistance(Body& a, Body& b, DistanceConstraint& constraint, float dt);
} // namespace physeng::detail
