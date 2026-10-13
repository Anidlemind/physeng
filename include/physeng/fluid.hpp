#pragma once

#include "common/vec2.hpp"
#include "physeng/id.hpp"
#include <cstddef>
#include <span>
#include <vector>

namespace physeng {
class World;

struct FluidParticle {
  common::Vec2 position;
  common::Vec2 velocity;
};

struct FluidSettings {
  common::Vec2 minimum{-12, -7}; // Interior of the tank, in world units.
  common::Vec2 maximum{12, 7};
  common::Vec2 gravity{0, -9.81f};
  float particleSpacing = 0.16f;
  float smoothingRadius = 0.48f; // Only particles within this distance interact.
  float restDensity = 1.0f;      // Mass per unit area for interaction with rigid bodies.
  float viscosity = 0.04f;       // Velocity smoothing per substep, from 0 to 1.
  int solverIterations = 4;
  int substeps = 2;
};

struct FluidBlockDesc {
  common::Vec2 minimum;
  std::size_t columns = 1, rows = 1;
  common::Vec2 velocity{};
};

// Equal-volume water particles; density constraints replace rigid circle contacts.
class Fluid {
public:
  explicit Fluid(FluidSettings settings = {});
  std::size_t AddParticle(common::Vec2 position, common::Vec2 velocity = {});
  void AddBlock(const FluidBlockDesc& block);
  void AddBlock(const FluidBlockDesc& block, const World& world); // Skip occupied geometry.
  void Step(double timeStep);
  void Step(double timeStep, World& world);
  // Blend local velocities toward a brush velocity. Called between steps.
  void Stir(common::Vec2 center, common::Vec2 velocity, float radius, float blend);

  std::span<const FluidParticle> Particles() const { return particles_; }

  std::span<const float> Densities() const { return densities_; } // Ratio to rest density (1).

  const FluidSettings& Settings() const { return settings_; }

  float ParticleRadius() const { return settings_.particleSpacing * 0.45f; }

private:
  // Relax both sides equally when many fluid constraints share a rigid body.
  static constexpr float solidPressureRelaxation_ = 0.25f;
  void AddBlockInternal(const FluidBlockDesc& block, const World* world);
  void StepInternal(double timeStep, World* world);
  void Substep(float dt, World* world);
  void BuildGrid();
  void FindNeighbors();
  void CalculateDensity(std::size_t i, bool calculateLambda, World* world = nullptr);
  float BoundaryDensity(common::Vec2 position, common::Vec2* gradient) const;
  common::Vec2 ProjectToTank(common::Vec2 position) const;
  float Kernel(common::Vec2 delta) const;
  common::Vec2 Gradient(std::size_t i, std::size_t j, common::Vec2 delta) const;
  std::size_t Cell(common::Vec2 position) const;

  struct SolidSample {
    common::Vec2 local, position;
    float weight;
    BodyId body;
  };
  struct SolidGradient {
    BodyId body;
    std::size_t particle;
    common::Vec2 gradient;
    float angularGradient;
  };
  struct SolidBounds {
    common::Vec2 minimum, maximum;
  };
  void PrepareSolids(World& world);
  void UpdateSolids(const World& world);
  float SolidDensity(std::size_t particle, World& world, bool calculateLambda,
                     common::Vec2& gradient, float& sumSquared);
  void ApplySolidPressure(World& world, float dt);
  void ProjectSolidContacts(World& world, float dt);
  common::Vec2 ApplySolidViscosity(std::size_t particle, common::Vec2 velocity, World& world);

  float particleMass_;
  std::vector<SolidSample> solidSamples_;
  std::vector<std::size_t> solidOffsets_, solidSorted_, solidCells_, solidCursors_;
  std::vector<std::size_t> solidNeighborOffsets_, solidNeighbors_;
  std::vector<SolidGradient> solidGradients_;
  std::vector<SolidBounds> solidBounds_;
  std::vector<common::Vec2> movingSolidGradients_, bodyGradients_;
  std::vector<float> bodyAngularGradients_, correctionScales_;
  std::vector<BodyId> touchedBodies_;
  std::vector<unsigned char> bodyTouched_;

  FluidSettings settings_;
  float densityScale_, gradientScale_, referenceKernel_, inverseRadius_, neighborRadius_;
  common::Vec2 boundarySpacing_;
  float boundaryDensityScale_, boundaryGradientScale_;
  std::size_t gridColumns_, gridRows_;
  std::vector<FluidParticle> particles_;
  std::vector<common::Vec2> previous_, predicted_, corrections_, velocities_;
  std::vector<common::Vec2> boundaryGradients_;
  std::vector<common::Vec2> neighborPositions_;
  std::vector<float> densities_, lambdas_;
  std::vector<std::size_t> cells_, offsets_, cursors_, sortedParticles_;
  // CSR neighbor storage: each particle has one contiguous span.
  std::vector<std::size_t> neighborCounts_, neighborOffsets_, neighbors_;
};

} // namespace physeng
