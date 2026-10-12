#include "physeng/scene.hpp"

#include <stdexcept>
#include <unordered_set>

namespace physeng {

BodyId Scene::GetBodyId(std::string_view name) const {
  const auto found = bodies.find(std::string(name));
  if (found == bodies.end()) {
    throw std::out_of_range("Scene body not found: " + std::string(name));
  }
  return found->second;
}

Scene CreateScene(const SceneDesc& desc) {
  std::unordered_set<std::string> names;
  for (const auto& body : desc.bodies) {
    if (body.name.empty() || !names.insert(body.name).second) {
      throw std::invalid_argument("Scene body names must be nonempty and unique");
    }
  }
  Scene scene{.world = World(desc.physics)};
  scene.bodies.reserve(desc.bodies.size());
  scene.springs.reserve(desc.springs.size());
  scene.enclosures.reserve(desc.enclosures.size());
  scene.ropes.reserve(desc.ropes.size());
  for (const auto& enclosure : desc.enclosures) {
    scene.enclosures.push_back(objects::CreateEnclosure(scene.world, enclosure));
  }
  for (const auto& body : desc.bodies) {
    const BodyId id = body.shape ? scene.world.AddBody(body.body, *body.shape, body.material)
                                 : scene.world.AddBody(body.body);
    scene.bodies.emplace(body.name, id);
  }
  for (const auto& spring : desc.springs) {
    scene.springs.push_back(objects::CreateSpring(scene.world,
                                                  {.first = scene.GetBodyId(spring.first),
                                                   .second = scene.GetBodyId(spring.second),
                                                   .firstAnchor = spring.firstAnchor,
                                                   .secondAnchor = spring.secondAnchor,
                                                   .restLength = spring.restLength,
                                                   .stiffness = spring.stiffness,
                                                   .damping = spring.damping}));
  }
  for (const auto& rope : desc.ropes) {
    auto settings = rope.settings;
    if (!rope.startBody.empty()) {
      settings.startBody = scene.GetBodyId(rope.startBody);
    }
    if (!rope.endBody.empty()) {
      settings.endBody = scene.GetBodyId(rope.endBody);
    }
    scene.ropes.push_back(objects::CreateRope(scene.world, settings));
  }
  return scene;
}

} // namespace physeng
