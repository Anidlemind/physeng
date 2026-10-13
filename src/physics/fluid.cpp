#include "physeng/fluid.hpp"
#include "detail/math.hpp"
#include "physeng/world.hpp"
#include "detail/collision.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace physeng {
using namespace detail;

Fluid::Fluid(FluidSettings settings) : settings_(settings) {
  const auto size = Sub(settings.maximum, settings.minimum);
  if (!Finite(settings.minimum) || !Finite(settings.maximum) || !Finite(size) ||
      !Finite(settings.gravity) || !Positive(settings.particleSpacing) ||
      !Positive(settings.restDensity) ||
      !Positive(settings.smoothingRadius) || settings.smoothingRadius <= settings.particleSpacing ||
      !Nonnegative(settings.viscosity) || settings.viscosity > 1 ||
      settings.solverIterations <= 0 || settings.substeps <= 0 ||
      size.x <= settings.particleSpacing || size.y <= settings.particleSpacing) {
    throw std::invalid_argument("Invalid fluid settings");
  }
  const double mass = static_cast<double>(settings.restDensity) * settings.particleSpacing *
                      settings.particleSpacing;
  particleMass_ = static_cast<float>(mass);
  if (!Positive(particleMass_) || !Positive(1.0f / particleMass_)) {
    throw std::invalid_argument("Fluid particle mass is not representable");
  }
  neighborRadius_ = settings.smoothingRadius * 1.25f;
  inverseRadius_ = 1.0f / settings.smoothingRadius;
  const double columns = std::ceil(static_cast<double>(size.x) / neighborRadius_);
  const double rows = std::ceil(static_cast<double>(size.y) / neighborRadius_);
  if (!std::isfinite(columns * rows) || columns * rows > 1000000) {
    throw std::invalid_argument("Fluid grid too large: increase smoothing radius or shrink tank");
  }
  gridColumns_ = static_cast<std::size_t>(columns);
  gridRows_ = static_cast<std::size_t>(rows);
  const float ratio = settings.particleSpacing / settings.smoothingRadius;
  // Normalized 2D Poly6 density and Spiky gradient, multiplied by particle area.
  densityScale_ = 4.0f / std::numbers::pi_v<float> * ratio * ratio;
  gradientScale_ = -30.0f / std::numbers::pi_v<float> * ratio * ratio / settings.smoothingRadius;
  if (!Positive(densityScale_) || !std::isfinite(gradientScale_) || gradientScale_ >= 0 ||
      !Positive(settings.smoothingRadius * settings.smoothingRadius)) {
    throw std::invalid_argument("Fluid kernel scale is not representable");
  }
  referenceKernel_ = densityScale_ * std::pow(1.0f - 0.3f * 0.3f, 3);
  // Sample the solid region on a regular lattice. Match the wall edges when possible.
  // Sampling at h/3 keeps the extra work bounded even when particle spacing is tiny.
  const float spacing = settings.smoothingRadius / 3.0f;
  if (!Positive(spacing)) {
    throw std::invalid_argument("Fluid boundary spacing is not representable");
  }
  const auto pitch = [spacing](float extent) {
    return extent < spacing ? spacing : extent / std::ceil(extent / spacing);
  };
  boundarySpacing_ = {pitch(size.x), pitch(size.y)};
  const float boundaryArea = (boundarySpacing_.x * inverseRadius_) *
                             (boundarySpacing_.y * inverseRadius_);
  boundaryDensityScale_ = 4.0f / std::numbers::pi_v<float> * boundaryArea;
  boundaryGradientScale_ = -30.0f / std::numbers::pi_v<float> * boundaryArea * inverseRadius_;
  if (!Positive(boundaryDensityScale_) || !std::isfinite(boundaryGradientScale_)) {
    throw std::invalid_argument("Fluid boundary kernel scale is not representable");
  }
  offsets_.resize(gridColumns_ * gridRows_ + 1);
  cursors_.resize(gridColumns_ * gridRows_);
}

common::Vec2 Fluid::ProjectToTank(common::Vec2 position) const {
  const float r = ParticleRadius();
  return {std::clamp(position.x, settings_.minimum.x + r, settings_.maximum.x - r),
          std::clamp(position.y, settings_.minimum.y + r, settings_.maximum.y - r)};
}

std::size_t Fluid::AddParticle(common::Vec2 position, common::Vec2 velocity) {
  const auto projected = ProjectToTank(position);
  if (!Finite(position) || !Finite(velocity) || projected.x != position.x ||
      projected.y != position.y) {
    throw std::invalid_argument("Fluid particle must start inside the tank with finite state");
  }
  const auto id = particles_.size();
  particles_.push_back({position, velocity});
  try {
    densities_.push_back(0);
  } catch (...) {
    particles_.pop_back();
    throw;
  }
  return id;
}

void Fluid::AddBlock(const FluidBlockDesc& block) {
  AddBlockInternal(block, nullptr);
}

void Fluid::AddBlock(const FluidBlockDesc& block, const World& world) {
  AddBlockInternal(block, &world);
}

void Fluid::AddBlockInternal(const FluidBlockDesc& block, const World* world) {
  if (block.columns == 0 || block.rows == 0 || !Finite(block.minimum) || !Finite(block.velocity) ||
      block.columns > (particles_.max_size() - particles_.size()) / block.rows) {
    throw std::invalid_argument("Invalid fluid block");
  }
  const common::Vec2 last{block.minimum.x + (block.columns - 1) * settings_.particleSpacing,
                          block.minimum.y + (block.rows - 1) * settings_.particleSpacing};
  const auto firstProjected = ProjectToTank(block.minimum), lastProjected = ProjectToTank(last);
  if (!Finite(last) || firstProjected.x != block.minimum.x || firstProjected.y != block.minimum.y ||
      lastProjected.x != last.x || lastProjected.y != last.y) {
    throw std::invalid_argument("Fluid block does not fit inside the tank");
  }
  const auto count = particles_.size() + block.columns * block.rows;
  particles_.reserve(count);
  densities_.reserve(count);
  for (std::size_t y = 0; y < block.rows; ++y) {
    for (std::size_t x = 0; x < block.columns; ++x) {
      const common::Vec2 position{block.minimum.x + x * settings_.particleSpacing,
                                  block.minimum.y + y * settings_.particleSpacing};
      if (world) {
        const Body particle{.position = position};
        const bool occupied = std::any_of(world->Colliders().begin(), world->Colliders().end(),
            [&](const auto& collider) {
              return std::visit([&](const auto& shape) {
                return ContactBetween(particle, CircleShape{ParticleRadius()},
                                      world->GetBody(collider.body), shape).has_value();
              }, collider.shape);
            });
        if (occupied) {
          continue;
        }
      }
      AddParticle(position, block.velocity);
    }
  }
}

void Fluid::Step(double timeStep) {
  StepInternal(timeStep, nullptr);
}

void Fluid::Step(double timeStep, World& world) {
  StepInternal(timeStep, &world);
}

void Fluid::StepInternal(double timeStep, World* world) {
  const float dt = static_cast<float>(timeStep / settings_.substeps);
  if (!Positive(dt) || !Positive(1.0f / dt) || !std::isfinite(timeStep) || timeStep > 0.1) {
    throw std::invalid_argument("Fluid step must be finite and in (0, 0.1] seconds");
  }
  if (particles_.empty()) {
    return;
  }
  const auto count = particles_.size();
  previous_.resize(count);
  predicted_.resize(count);
  corrections_.resize(count);
  boundaryGradients_.resize(count);
  movingSolidGradients_.resize(count);
  correctionScales_.resize(count);
  velocities_.resize(count);
  lambdas_.resize(count);
  cells_.resize(count);
  sortedParticles_.resize(count);
  neighborCounts_.resize(count);
  neighborOffsets_.resize(count + 1);
  if (world) {
    PrepareSolids(*world);
  }
  for (int i = 0; i < settings_.substeps; ++i) {
    Substep(dt, world);
  }
}

std::size_t Fluid::Cell(common::Vec2 position) const {
  const auto x = std::min(gridColumns_ - 1,
                          static_cast<std::size_t>(std::max(
                              0.0f, (position.x - settings_.minimum.x) / neighborRadius_)));
  const auto y = std::min(gridRows_ - 1,
                          static_cast<std::size_t>(std::max(
                              0.0f, (position.y - settings_.minimum.y) / neighborRadius_)));
  return y * gridColumns_ + x;
}

void Fluid::BuildGrid() {
  std::fill(offsets_.begin(), offsets_.end(), 0);
  for (std::size_t i = 0; i < predicted_.size(); ++i) {
    cells_[i] = Cell(predicted_[i]);
    ++offsets_[cells_[i] + 1];
  }
  for (std::size_t cell = 1; cell < offsets_.size(); ++cell) {
    offsets_[cell] += offsets_[cell - 1];
  }
  std::copy(offsets_.begin(), offsets_.end() - 1, cursors_.begin());
  for (std::size_t i = 0; i < cells_.size(); ++i) {
    sortedParticles_[cursors_[cells_[i]]++] = i;
  }
}

void Fluid::FindNeighbors() {
  // A 25% skin permits two particles to each move 12.5% of h before rebuilding.
  bool reusable = neighborPositions_.size() == predicted_.size();
  if (reusable) {
    for (std::size_t i = 0; i < predicted_.size(); ++i) {
      const auto move = Scale(Sub(predicted_[i], neighborPositions_[i]), inverseRadius_);
      if (Dot(move, move) >= 0.125f * 0.125f) {
        reusable = false;
        break;
      }
    }
  }
  if (reusable) {
    return;
  }
  BuildGrid();
  const auto visit = [&](std::size_t i, const auto& accept) {
    const auto cx = cells_[i] % gridColumns_, cy = cells_[i] / gridColumns_;
    for (auto y = cy == 0 ? 0 : cy - 1; y <= std::min(cy + 1, gridRows_ - 1); ++y) {
      for (auto x = cx == 0 ? 0 : cx - 1; x <= std::min(cx + 1, gridColumns_ - 1); ++x) {
        const auto cell = y * gridColumns_ + x;
        for (auto entry = offsets_[cell]; entry < offsets_[cell + 1]; ++entry) {
          const auto j = sortedParticles_[entry];
          const auto q = Scale(Sub(predicted_[i], predicted_[j]), 1.0f / neighborRadius_);
          if (i != j && Dot(q, q) < 1.0f) {
            accept(j);
          }
        }
      }
    }
  };
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    std::size_t count = 0;
    visit(i, [&](auto) { ++count; });
    neighborCounts_[i] = count;
  }
  neighborOffsets_[0] = 0;
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    neighborOffsets_[i + 1] = neighborOffsets_[i] + neighborCounts_[i];
  }
  neighbors_.resize(neighborOffsets_.back());
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    auto index = neighborOffsets_[i];
    visit(i, [&](auto j) { neighbors_[index++] = j; });
  }
  neighborPositions_ = predicted_;
}

float Fluid::Kernel(common::Vec2 delta) const {
  const auto q = Scale(delta, inverseRadius_);
  const float remaining = std::max(1.0f - Dot(q, q), 0.0f);
  return densityScale_ * remaining * remaining * remaining;
}

common::Vec2 Fluid::Gradient(std::size_t i, std::size_t j, common::Vec2 delta) const {
  const auto q = Scale(delta, inverseRadius_);
  const float distance = Length(q);
  common::Vec2 direction;
  if (distance > 0.0f) {
    direction = Scale(q, 1.0f / distance);
  } else {
    // A deterministic, antisymmetric direction separates exact coincidences.
    const auto hash = static_cast<std::uint64_t>(std::min(i, j)) * 73856093u ^
                      static_cast<std::uint64_t>(std::max(i, j)) * 19349663u;
    const float angle = (hash % 65536) * (2.0f * std::numbers::pi_v<float> / 65536);
    direction = Scale({std::cos(angle), std::sin(angle)}, i < j ? 1.0f : -1.0f);
  }
  const float remaining = std::max(1.0f - distance, 0.0f);
  return Scale(direction, gradientScale_ * remaining * remaining);
}

float Fluid::BoundaryDensity(common::Vec2 position, common::Vec2* gradient) const {
  const float h = settings_.smoothingRadius;
  if (position.x - settings_.minimum.x >= h && settings_.maximum.x - position.x >= h &&
      position.y - settings_.minimum.y >= h && settings_.maximum.y - position.y >= h) {
    return 0;
  }
  const auto firstCell = [](float position, float minimum, float pitch, float radius) {
    return static_cast<std::int64_t>(std::ceil((static_cast<double>(position) - minimum - radius) /
                                             pitch - 0.5));
  };
  const auto lastCell = [](float position, float minimum, float pitch, float radius) {
    return static_cast<std::int64_t>(std::floor((static_cast<double>(position) - minimum + radius) /
                                              pitch - 0.5));
  };
  const auto x0 = firstCell(position.x, settings_.minimum.x, boundarySpacing_.x, h);
  const auto x1 = lastCell(position.x, settings_.minimum.x, boundarySpacing_.x, h);
  const auto y0 = firstCell(position.y, settings_.minimum.y, boundarySpacing_.y, h);
  const auto y1 = lastCell(position.y, settings_.minimum.y, boundarySpacing_.y, h);
  float density = 0;
  for (auto y = y0; y <= y1; ++y) {
    for (auto x = x0; x <= x1; ++x) {
      const common::Vec2 sample{
          static_cast<float>(settings_.minimum.x + (x + 0.5) * boundarySpacing_.x),
          static_cast<float>(settings_.minimum.y + (y + 0.5) * boundarySpacing_.y)};
      // Sample only solid space. A single lattice also avoids double-counting corners.
      if (sample.x >= settings_.minimum.x && sample.x <= settings_.maximum.x &&
          sample.y >= settings_.minimum.y && sample.y <= settings_.maximum.y) {
        continue;
      }
      const auto q = Scale(Sub(position, sample), inverseRadius_);
      const float squared = Dot(q, q);
      if (squared >= 1.0f) {
        continue;
      }
      const float remaining = 1.0f - squared;
      density += boundaryDensityScale_ * remaining * remaining * remaining;
      if (gradient && squared > 0) {
        const float distance = std::sqrt(squared);
        const float falloff = 1.0f - distance;
        *gradient = Add(*gradient, Scale(q, boundaryGradientScale_ * falloff * falloff / distance));
      }
    }
  }
  return density;
}

void Fluid::CalculateDensity(std::size_t i, bool calculateLambda, World* world) {
  float density = densityScale_; // Self contribution to the density estimate.
  float sumSquared = 0;
  common::Vec2 sumGradient{};
  common::Vec2 boundaryGradient{};
  density += BoundaryDensity(predicted_[i], calculateLambda ? &boundaryGradient : nullptr);
  if (world) {
    density += SolidDensity(i, *world, calculateLambda, boundaryGradient, sumSquared);
  }
  for (auto entry = neighborOffsets_[i]; entry < neighborOffsets_[i + 1]; ++entry) {
    const auto j = neighbors_[entry];
    const auto delta = Sub(predicted_[i], predicted_[j]);
    const float kernel = Kernel(delta);
    density += kernel;
    if (calculateLambda && kernel > 0.0f) {
      const auto gradient = Gradient(i, j, delta);
      sumGradient = Add(sumGradient, gradient);
      sumSquared += Dot(gradient, gradient);
    }
  }
  densities_[i] = density;
  if (calculateLambda) {
    boundaryGradients_[i] = boundaryGradient;
    // Wall samples are fixed: they contribute to this particle's gradient, not movable neighbors.
    sumGradient = Add(sumGradient, boundaryGradient);
    if (world) {
      sumGradient = Add(sumGradient, movingSolidGradients_[i]);
    }
    // Only resist compression: sparse free-surface particles must not pull into clumps.
    const float error = std::max(density - 1.0f, 0.0f);
    const float regularization = 0.001f / (settings_.smoothingRadius * settings_.smoothingRadius);
    lambdas_[i] = -error / (sumSquared + Dot(sumGradient, sumGradient) + regularization);
  }
}

void Fluid::Substep(float dt, World* world) {
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    previous_[i] = particles_[i].position;
    const auto velocity = Add(particles_[i].velocity, Scale(settings_.gravity, dt));
    predicted_[i] = ProjectToTank(Add(previous_[i], Scale(velocity, dt)));
  }
  if (world) {
    ProjectSolidContacts(*world, dt);
  }
  for (int iteration = 0; iteration < settings_.solverIterations; ++iteration) {
    FindNeighbors();
    if (world) {
      UpdateSolids(*world);
      solidGradients_.clear();
    }
    for (std::size_t i = 0; i < particles_.size(); ++i) {
      CalculateDensity(i, true, world);
    }
    for (std::size_t i = 0; i < particles_.size(); ++i) {
      common::Vec2 correction = Scale(boundaryGradients_[i], lambdas_[i]);
      if (world) {
        correction = Add(correction,
                         Scale(movingSolidGradients_[i], lambdas_[i] * solidPressureRelaxation_));
      }
      for (auto entry = neighborOffsets_[i]; entry < neighborOffsets_[i + 1]; ++entry) {
        const auto j = neighbors_[entry];
        const auto delta = Sub(predicted_[i], predicted_[j]);
        const float kernel = Kernel(delta);
        if (kernel == 0.0f) {
          continue;
        }
        const float ratio = kernel / referenceKernel_;
        const float artificialPressure = -0.0001f * settings_.smoothingRadius *
                                         settings_.smoothingRadius * ratio * ratio * ratio *
                                         ratio;
        correction =
            Add(correction,
                Scale(Gradient(i, j, delta), lambdas_[i] + lambdas_[j] + artificialPressure));
      }
      const float length = Length(correction), limit = 0.2f * settings_.smoothingRadius;
      correctionScales_[i] = length > limit ? limit / length : 1.0f;
      if (length > limit) {
        correction = Scale(correction, limit / length);
      }
      corrections_[i] = correction;
    }
    if (world) {
      ApplySolidPressure(*world, dt);
    }
    // Apply corrections together, after calculating them from the same position snapshot.
    for (std::size_t i = 0; i < particles_.size(); ++i) {
      predicted_[i] = ProjectToTank(Add(predicted_[i], corrections_[i]));
    }
    if (world) {
      ProjectSolidContacts(*world, dt);
    }
  }
  FindNeighbors();
  if (world) {
    UpdateSolids(*world);
  }
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    CalculateDensity(i, false, world);
    velocities_[i] = Scale(Sub(predicted_[i], previous_[i]), 1.0f / dt);
  }
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    common::Vec2 smoothing{};
    for (auto entry = neighborOffsets_[i]; entry < neighborOffsets_[i + 1]; ++entry) {
      const auto j = neighbors_[entry];
      const float weight =
          Kernel(Sub(predicted_[i], predicted_[j])) / std::max(densities_[i], densities_[j]);
      smoothing = Add(smoothing, Scale(Sub(velocities_[j], velocities_[i]), weight));
    }
    auto velocity = Add(velocities_[i], Scale(smoothing, settings_.viscosity));
    if (world) {
      velocity = ApplySolidViscosity(i, velocity, *world);
    }
    const auto position = predicted_[i];
    const float r = ParticleRadius();
    if ((position.x <= settings_.minimum.x + r && velocity.x < 0) ||
        (position.x >= settings_.maximum.x - r && velocity.x > 0)) {
      velocity.x = 0;
    }
    if ((position.y <= settings_.minimum.y + r && velocity.y < 0) ||
        (position.y >= settings_.maximum.y - r && velocity.y > 0)) {
      velocity.y = 0;
    }
    particles_[i] = {position, velocity};
  }
}

void Fluid::Stir(common::Vec2 center, common::Vec2 velocity, float radius, float blend) {
  if (!Finite(center) || !Finite(velocity) || !Positive(radius) || !Nonnegative(blend) ||
      blend > 1) {
    throw std::invalid_argument("Invalid fluid stirring brush");
  }
  for (auto& particle : particles_) {
    const auto q = Scale(Sub(particle.position, center), 1.0f / radius);
    const float weight = blend * std::max(1.0f - Dot(q, q), 0.0f);
    particle.velocity = Add(Scale(particle.velocity, 1.0f - weight), Scale(velocity, weight));
  }
}

} // namespace physeng
