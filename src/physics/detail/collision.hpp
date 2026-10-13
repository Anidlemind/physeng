#pragma once

#include "physeng/body.hpp"
#include "physeng/detail/contact_state.hpp"
#include "physeng/material.hpp"
#include "physeng/shape.hpp"
#include <array>
#include <optional>

namespace physeng::detail {

using common::Vec2;

struct Contact {
  Vec2 normal; // From the first collider toward the second.
  float penetration;
  std::array<Vec2, 2> points{}; // World-space contact manifold.
  int pointCount = 0;
};

std::optional<Contact> SweptCircleBoxContact(const Body& previous,
                                             const Body& circle,
                                             CircleShape shape,
                                             const Body& box,
                                             BoxShape boxShape);

std::optional<Contact> ContactBetween(const Body& a, CircleShape sa, const Body& b, CircleShape sb);
std::optional<Contact> ContactBetween(const Body& a, BoxShape sa, const Body& b, BoxShape sb);
std::optional<Contact> ContactBetween(const Body& a, CircleShape sa, const Body& b, BoxShape sb);
std::optional<Contact> ContactBetween(const Body& a, BoxShape sa, const Body& b, CircleShape sb);
std::optional<Contact>
ContactBetween(const Body& a, const PolygonShape& sa, const Body& b, const PolygonShape& sb);
std::optional<Contact>
ContactBetween(const Body& a, const PolygonShape& sa, const Body& b, BoxShape sb);
std::optional<Contact>
ContactBetween(const Body& a, BoxShape sa, const Body& b, const PolygonShape& sb);
std::optional<Contact>
ContactBetween(const Body& a, CircleShape sa, const Body& b, const PolygonShape& sb);
std::optional<Contact>
ContactBetween(const Body& a, const PolygonShape& sa, const Body& b, CircleShape sb);
void ResolveCircleContact(Body& a,
                          Body& b,
                          CircleShape sa,
                          CircleShape sb,
                          Material materialA,
                          Material materialB,
                          float restitutionThreshold,
                          const Body& impactA,
                          const Body& impactB,
                          ContactState* state = nullptr,
                          bool warmStartOnly = false);
void ResolveContact(Body& a,
                    Body& b,
                    Contact contact,
                    Material ma,
                    Material mb,
                    float restitutionThreshold,
                    const Body& impactA,
                    const Body& impactB,
                    ContactState* state = nullptr,
                    bool warmStartOnly = false);
void ResolveFloor(Body& body,
                  const Shape& shape,
                  float floorY,
                  Material material,
                  Material floorMaterial,
                  float restitutionThreshold,
                  const Body& impactBody,
                  ContactState* state = nullptr,
                  bool warmStartOnly = false);
} // namespace physeng::detail
