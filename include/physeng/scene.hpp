#pragma once

#include "physeng/fluid.hpp"
#include "physeng/objects/enclosure.hpp"
#include "physeng/objects/rope.h"
#include "physeng/objects/spring.hpp"
#include "physeng/objects/softbody.hpp"
#include "physeng/world.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace physeng {

struct SceneBodyDesc {
  std::string name; // Unique within this scene; required for lookup.
  Body body;
  std::optional<Shape> shape; // Omit for a particle without collision geometry.
  Material material;
};

struct SceneSpringDesc {
  std::string first;
  std::string second;
  common::Vec2 firstAnchor{};
  common::Vec2 secondAnchor{};
  std::optional<float> restLength;
  float stiffness = 50.0f;
  float damping = 2.0f;
};

struct SceneRopeDesc {
  objects::RopeDesc settings;
  std::string startBody;
  std::string endBody;
};

struct SceneFluidDesc {
  FluidSettings settings;
  std::vector<FluidBlockDesc> blocks;
};

struct SceneDesc {
  PhysicsSettings physics{.floorY = std::nullopt};
  std::vector<SceneBodyDesc> bodies;
  std::vector<SceneSpringDesc> springs;
  std::vector<objects::EnclosureDesc> enclosures;
  std::vector<SceneRopeDesc> ropes;
  std::vector<SceneFluidDesc> fluids;
  std::vector<objects::SoftBodyDesc> softBodies;
};

struct Scene {
  World world;
  std::unordered_map<std::string, BodyId> bodies;
  std::vector<objects::SpringHandle> springs;
  std::vector<objects::EnclosureHandle> enclosures;
  std::vector<objects::RopeHandle> ropes;
  std::vector<Fluid> fluids;
  std::vector<objects::SoftBodyHandle> softBodies;

  BodyId GetBodyId(std::string_view name) const;
  void Step(double timeStep); // Advance rigid bodies and each fluid tank.
};

Scene CreateScene(const SceneDesc& desc);

} // namespace physeng
