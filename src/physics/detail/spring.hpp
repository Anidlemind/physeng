#pragma once

#include "physeng/body.hpp"
#include "physeng/spring.hpp"

namespace physeng::detail {

void ApplySpring(Body& first, Body& second, const Spring& spring);
} // namespace physeng::detail
