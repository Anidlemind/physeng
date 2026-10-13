#include "detail/collision.hpp"
#include "detail/dynamics.hpp"
#include "detail/math.hpp"
#include "physeng/fluid.hpp"
#include "physeng/world.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <type_traits>

namespace physeng {
using namespace detail;

namespace {
struct SamplingRegion {
  common::Vec2 minimum, maximum;
  double area;
};

SamplingRegion Region(const Shape& shape) {
  return std::visit(
      [](const auto& s) -> SamplingRegion {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, BoxShape>) {
          return {Scale(s.halfExtents, -1), s.halfExtents, 4.0 * s.halfExtents.x * s.halfExtents.y};
        } else if constexpr (std::is_same_v<T, CircleShape>) {
          return {
              {-s.radius, -s.radius}, {s.radius, s.radius}, std::numbers::pi * s.radius * s.radius};
        } else {
          const float infinity = std::numeric_limits<float>::infinity();
          SamplingRegion result{{infinity, infinity}, {-infinity, -infinity}, 0};
          const auto vertices = s.Vertices();
          for (std::size_t i = 0; i < vertices.size(); ++i) {
            const auto a = vertices[i], b = vertices[(i + 1) % vertices.size()];
            result.minimum.x = std::min(result.minimum.x, a.x);
            result.minimum.y = std::min(result.minimum.y, a.y);
            result.maximum.x = std::max(result.maximum.x, a.x);
            result.maximum.y = std::max(result.maximum.y, a.y);
            result.area += static_cast<double>(a.x) * b.y - static_cast<double>(a.y) * b.x;
          }
          result.area = std::abs(result.area) * 0.5;
          return result;
        }
      },
      shape);
}

bool Contains(const Shape& shape, common::Vec2 point) {
  return std::visit(
      [&](const auto& s) {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, BoxShape>) {
          return std::abs(point.x) <= s.halfExtents.x && std::abs(point.y) <= s.halfExtents.y;
        } else if constexpr (std::is_same_v<T, CircleShape>) {
          return Dot(point, point) <= s.radius * s.radius;
        } else {
          const auto vertices = s.Vertices();
          for (std::size_t i = 0; i < vertices.size(); ++i) {
            if (Cross(Sub(vertices[(i + 1) % vertices.size()], vertices[i]),
                      Sub(point, vertices[i])) < 0) {
              return false;
            }
          }
          return true;
        }
      },
      shape);
}
} // namespace

void Fluid::PrepareSolids(World& world) {
  solidSamples_.clear();
  bodyGradients_.assign(world.bodies_.size(), {});
  bodyAngularGradients_.assign(world.bodies_.size(), 0);
  bodyTouched_.assign(world.bodies_.size(), 0);
  touchedBodies_.clear();
  for (ColliderId id = 0; id < world.colliders_.size(); ++id) {
    const auto bounds = world.ComputeCollisionBounds(id, settings_.smoothingRadius);
    if (bounds.maximum.x < settings_.minimum.x || bounds.minimum.x > settings_.maximum.x ||
        bounds.maximum.y < settings_.minimum.y || bounds.minimum.y > settings_.maximum.y) {
      continue;
    }
    const auto& collider = world.colliders_[id];
    const auto region = Region(collider.shape);
    const double width = static_cast<double>(region.maximum.x) - region.minimum.x;
    const double height = static_cast<double>(region.maximum.y) - region.minimum.y;
    double pitch = settings_.smoothingRadius / 3.0;
    // Large colliders retain bounded sampling cost; contact geometry remains exact.
    pitch = std::max(pitch, std::max(width, height) / 64.0);
    const auto nx = static_cast<std::size_t>(std::max(1.0, std::ceil(width / pitch)));
    const auto ny = static_cast<std::size_t>(std::max(1.0, std::ceil(height / pitch)));
    const auto first = solidSamples_.size();
    for (std::size_t y = 0; y < ny; ++y) {
      for (std::size_t x = 0; x < nx; ++x) {
        const common::Vec2 local{static_cast<float>(region.minimum.x + (x + 0.5) * width / nx),
                                 static_cast<float>(region.minimum.y + (y + 0.5) * height / ny)};
        if (Contains(collider.shape, local)) {
          solidSamples_.push_back({local, {}, 0, collider.body});
        }
      }
    }
    // Thin polygons may miss every cell center. Their center of mass is inside.
    if (solidSamples_.size() == first) {
      solidSamples_.push_back({{}, {}, 0, collider.body});
    }
    const double weight =
        region.area / (solidSamples_.size() - first) /
        (static_cast<double>(settings_.particleSpacing) * settings_.particleSpacing);
    if (!Positive(static_cast<float>(weight))) {
      throw std::invalid_argument("Rigid-body fluid sample volume is not representable");
    }
    for (auto index = first; index < solidSamples_.size(); ++index) {
      solidSamples_[index].weight = static_cast<float>(weight);
    }
  }
  solidOffsets_.resize(gridColumns_ * gridRows_ + 1);
  solidCursors_.resize(gridColumns_ * gridRows_);
  solidCells_.resize(solidSamples_.size());
  solidSorted_.resize(solidSamples_.size());
  solidNeighborOffsets_.resize(particles_.size() + 1);
}

void Fluid::UpdateSolids(const World& world) {
  std::fill(solidOffsets_.begin(), solidOffsets_.end(), 0);
  for (std::size_t i = 0; i < solidSamples_.size(); ++i) {
    auto& sample = solidSamples_[i];
    sample.position = world.bodies_[sample.body].LocalToWorld(sample.local);
    solidCells_[i] = std::numeric_limits<std::size_t>::max();
    // The tank's existing wall lattice covers volume outside its bounds.
    if (sample.position.x < settings_.minimum.x || sample.position.x > settings_.maximum.x ||
        sample.position.y < settings_.minimum.y || sample.position.y > settings_.maximum.y) {
      continue;
    }
    solidCells_[i] = Cell(sample.position);
    ++solidOffsets_[solidCells_[i] + 1];
  }
  for (std::size_t cell = 1; cell < solidOffsets_.size(); ++cell) {
    solidOffsets_[cell] += solidOffsets_[cell - 1];
  }
  std::copy(solidOffsets_.begin(), solidOffsets_.end() - 1, solidCursors_.begin());
  for (std::size_t i = 0; i < solidCells_.size(); ++i) {
    if (solidCells_[i] != std::numeric_limits<std::size_t>::max()) {
      solidSorted_[solidCursors_[solidCells_[i]]++] = i;
    }
  }
  solidNeighbors_.clear();
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    solidNeighborOffsets_[i] = solidNeighbors_.size();
    const auto cell = Cell(predicted_[i]);
    const auto cx = cell % gridColumns_, cy = cell / gridColumns_;
    for (auto y = cy == 0 ? 0 : cy - 1; y <= std::min(cy + 1, gridRows_ - 1); ++y) {
      for (auto x = cx == 0 ? 0 : cx - 1; x <= std::min(cx + 1, gridColumns_ - 1); ++x) {
        const auto neighborCell = y * gridColumns_ + x;
        for (auto entry = solidOffsets_[neighborCell]; entry < solidOffsets_[neighborCell + 1];
             ++entry) {
          const auto sample = solidSorted_[entry];
          if (Kernel(Sub(predicted_[i], solidSamples_[sample].position)) > 0) {
            solidNeighbors_.push_back(sample);
          }
        }
      }
    }
  }
  solidNeighborOffsets_.back() = solidNeighbors_.size();
}

float Fluid::SolidDensity(std::size_t i,
                          World& world,
                          bool calculateLambda,
                          common::Vec2& fixedGradient,
                          float& sumSquared) {
  if (calculateLambda) {
    movingSolidGradients_[i] = {};
    touchedBodies_.clear();
  }
  float density = 0;
  for (auto entry = solidNeighborOffsets_[i]; entry < solidNeighborOffsets_[i + 1]; ++entry) {
    const auto sampleId = solidNeighbors_[entry];
    const auto& sample = solidSamples_[sampleId];
    const auto delta = Sub(predicted_[i], sample.position);
    density += sample.weight * Kernel(delta);
    if (!calculateLambda) {
      continue;
    }
    const auto gradient = Scale(Gradient(i, particles_.size() + sampleId, delta), sample.weight);
    const auto& body = world.bodies_[sample.body];
    if (body.inverseMass == 0) {
      fixedGradient = Add(fixedGradient, gradient);
      continue;
    }
    movingSolidGradients_[i] = Add(movingSolidGradients_[i], gradient);
    if (!bodyTouched_[sample.body]) {
      bodyTouched_[sample.body] = 1;
      touchedBodies_.push_back(sample.body);
    }
    bodyGradients_[sample.body] = Add(bodyGradients_[sample.body], gradient);
    bodyAngularGradients_[sample.body] += Cross(Sub(sample.position, body.position), gradient);
  }
  if (calculateLambda) {
    for (auto id : touchedBodies_) {
      const auto& body = world.bodies_[id];
      const auto g = bodyGradients_[id];
      const float angular = bodyAngularGradients_[id];
      sumSquared +=
          particleMass_ * (body.inverseMass * Dot(g, g) + InverseInertia(body) * angular * angular);
      solidGradients_.push_back({id, i, g, angular});
      bodyGradients_[id] = {};
      bodyAngularGradients_[id] = 0;
      bodyTouched_[id] = 0;
    }
  }
  return density;
}

void Fluid::ApplySolidPressure(World& world, float dt) {
  for (const auto& row : solidGradients_) {
    auto& body = world.bodies_[row.body];
    const float lambda = lambdas_[row.particle] * correctionScales_[row.particle];
    const float impulseScale = -particleMass_ * solidPressureRelaxation_ * lambda / dt;
    const auto impulse = Scale(row.gradient, impulseScale);
    const auto move = Scale(impulse, body.inverseMass * dt);
    const float turn = impulseScale * row.angularGradient * InverseInertia(body) * dt;
    body.position = Add(body.position, move);
    body.angle += turn;
    body.velocity = Add(body.velocity, Scale(move, 1 / dt));
    body.angularVelocity += turn / dt;
  }
}

void Fluid::ProjectSolidContacts(World& world, float dt) {
  solidBounds_.resize(world.colliders_.size());
  for (ColliderId id = 0; id < world.colliders_.size(); ++id) {
    const auto bounds =
        world.ComputeCollisionBounds(id, ParticleRadius() + settings_.smoothingRadius);
    solidBounds_[id] = {bounds.minimum, bounds.maximum};
  }
  const float inverseParticleMass = 1 / particleMass_;
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    for (ColliderId id = 0; id < world.colliders_.size(); ++id) {
      const auto& bounds = solidBounds_[id];
      const auto p = predicted_[i];
      const auto start = previous_[i];
      // Keep crossed walls in the candidate list even if the endpoint is past them.
      if (std::max(p.x, start.x) < bounds.minimum.x || std::min(p.x, start.x) > bounds.maximum.x ||
          std::max(p.y, start.y) < bounds.minimum.y || std::min(p.y, start.y) > bounds.maximum.y) {
        continue;
      }
      const auto& collider = world.colliders_[id];
      auto& body = world.bodies_[collider.body];
      const Body particle{.position = p};
      auto contact = std::visit(
          [&](const auto& shape) {
            return ContactBetween(particle, CircleShape{ParticleRadius()}, body, shape);
          },
          collider.shape);
      if (body.inverseMass == 0) {
        if (const auto* box = std::get_if<BoxShape>(&collider.shape)) {
          if (auto swept = SweptCircleBoxContact(
                  Body{.position = previous_[i]}, particle, {ParticleRadius()}, body, *box)) {
            contact = swept;
          }
        }
      }
      if (!contact || contact->penetration <= 0.001f) {
        continue;
      }
      const auto offset = Sub(contact->points[0], body.position);
      const float lever = Cross(offset, contact->normal);
      const float weight =
          inverseParticleMass + body.inverseMass + lever * lever * InverseInertia(body);
      const float correction = (contact->penetration - 0.001f) / weight;
      predicted_[i] = ProjectToTank(
          Sub(predicted_[i], Scale(contact->normal, correction * inverseParticleMass)));
      if (body.inverseMass > 0) {
        const auto move = Scale(contact->normal, correction * body.inverseMass);
        const float turn = correction * lever * InverseInertia(body);
        body.position = Add(body.position, move);
        body.angle += turn;
        body.velocity = Add(body.velocity, Scale(move, 1 / dt));
        body.angularVelocity += turn / dt;
      }
    }
  }
}

common::Vec2 Fluid::ApplySolidViscosity(std::size_t i, common::Vec2 velocity, World& world) {
  const float inverseParticleMass = 1 / particleMass_;
  for (auto entry = solidNeighborOffsets_[i]; entry < solidNeighborOffsets_[i + 1]; ++entry) {
    const auto& sample = solidSamples_[solidNeighbors_[entry]];
    auto& body = world.bodies_[sample.body];
    if (body.inverseMass == 0) {
      continue; // Preserve free slip against static walls.
    }
    // Apply the reaction at the particle position so drag also conserves angular momentum.
    const auto offset = Sub(predicted_[i], body.position);
    const auto slip = Sub(velocity, body.VelocityAtPoint(predicted_[i]));
    const float weight =
        sample.weight * Kernel(Sub(predicted_[i], sample.position)) / densities_[i];
    const float blend = std::clamp(settings_.viscosity * weight, 0.0f, 1.0f);
    const float linear = inverseParticleMass + body.inverseMass;
    const float inertia = InverseInertia(body);
    const auto tangent = Perpendicular(offset);
    // Invert the two-dimensional effective mass, including the lever arm.
    const auto response = Sub(Scale(slip, 1 / linear),
                              Scale(tangent,
                                    inertia * Dot(tangent, slip) /
                                        (linear * (linear + inertia * Dot(tangent, tangent)))));
    const auto impulse = Scale(response, blend);
    velocity = Sub(velocity, Scale(impulse, inverseParticleMass));
    ImpulseAtOffset(body, impulse, offset);
  }
  return velocity;
}

} // namespace physeng
