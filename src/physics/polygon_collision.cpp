#include "detail/collision.hpp"
#include "detail/math.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <span>

namespace physeng::detail {
namespace {
constexpr float contactTolerance = 0.001f;

// Borrow local vertices and transform on demand; collision checks allocate nothing.
struct Boundary {
  const Body& body;
  std::span<const Vec2> vertices;
  float cosine, sine;

  Boundary(const Body& state, std::span<const Vec2> points)
      : body(state), vertices(points), cosine(std::cos(state.angle)), sine(std::sin(state.angle)) {}

  Vec2 RotateLocal(Vec2 v) const {
    return {cosine * v.x - sine * v.y, sine * v.x + cosine * v.y};
  }

  Vec2 Vertex(std::size_t i) const {
    return Add(body.position, RotateLocal(vertices[i % vertices.size()]));
  }

  Vec2 Normal(std::size_t i) const {
    const Vec2 edge = Sub(vertices[(i + 1) % vertices.size()], vertices[i]);
    return RotateLocal(Scale(Vec2{edge.y, -edge.x}, 1.0f / Length(edge)));
  }
};

std::array<Vec2, 4> BoxVertices(BoxShape box) {
  const auto h = box.halfExtents;
  return {{{-h.x, -h.y}, {h.x, -h.y}, {h.x, h.y}, {-h.x, h.y}}};
}

struct Face {
  float separation = -std::numeric_limits<float>::infinity();
  std::size_t index = 0;
};

Face SeparatingFace(const Boundary& reference, const Boundary& other) {
  Face result;
  for (std::size_t i = 0; i < reference.vertices.size(); ++i) {
    const Vec2 normal = reference.Normal(i), origin = reference.Vertex(i);
    float separation = std::numeric_limits<float>::infinity();
    for (std::size_t j = 0; j < other.vertices.size(); ++j) {
      separation = std::min(separation, Dot(normal, Sub(other.Vertex(j), origin)));
    }
    if (separation > result.separation) {
      result = {separation, i};
    }
  }
  return result;
}

int Clip(std::array<Vec2, 2>& points, int count, Vec2 normal, float offset) {
  if (count == 0) {
    return 0;
  }
  const auto original = points;
  const float d0 = Dot(normal, original[0]) - offset;
  if (count == 1) {
    return d0 <= 0.0f ? 1 : 0;
  }
  const float d1 = Dot(normal, original[1]) - offset;
  int result = 0;
  if (d0 <= 0.0f) {
    points[result++] = original[0];
  }
  if (d1 <= 0.0f) {
    points[result++] = original[1];
  }
  if ((d0 < 0.0f && d1 > 0.0f) || (d0 > 0.0f && d1 < 0.0f)) {
    points[result++] = Add(original[0], Scale(Sub(original[1], original[0]), d0 / (d0 - d1)));
  }
  return result;
}

std::optional<Contact> PolygonContact(const Boundary& a, const Boundary& b) {
  const Face faceA = SeparatingFace(a, b), faceB = SeparatingFace(b, a);
  if (faceA.separation > contactTolerance || faceB.separation > contactTolerance) {
    return std::nullopt;
  }
  const bool referenceA = faceA.separation >= faceB.separation;
  const auto& reference = referenceA ? a : b;
  const auto& incident = referenceA ? b : a;
  const Face face = referenceA ? faceA : faceB;
  const Vec2 normal = reference.Normal(face.index);
  std::size_t incidentFace = 0;
  float alignment = std::numeric_limits<float>::infinity();
  for (std::size_t i = 0; i < incident.vertices.size(); ++i) {
    const float candidate = Dot(normal, incident.Normal(i));
    if (candidate < alignment) {
      alignment = candidate;
      incidentFace = i;
    }
  }
  const Vec2 start = reference.Vertex(face.index), end = reference.Vertex(face.index + 1);
  const Vec2 edge = Sub(end, start), tangent = Scale(edge, 1.0f / Length(edge));
  std::array<Vec2, 2> points{incident.Vertex(incidentFace), incident.Vertex(incidentFace + 1)};
  int count = Clip(points, 2, Scale(tangent, -1.0f), -Dot(tangent, start));
  count = Clip(points, count, tangent, Dot(tangent, end));
  Contact contact{referenceA ? normal : Scale(normal, -1.0f), std::max(-face.separation, 0.0f)};
  for (int i = 0; i < count; ++i) {
    const float distance = Dot(normal, Sub(points[i], start));
    if (distance <= contactTolerance) {
      contact.points[contact.pointCount++] = Sub(points[i], Scale(normal, 0.5f * distance));
    }
  }
  if (contact.pointCount == 0) {
    return std::nullopt;
  }
  return contact;
}

std::optional<Contact> Reverse(std::optional<Contact> contact) {
  if (contact) {
    contact->normal = Scale(contact->normal, -1.0f);
  }
  return contact;
}
} // namespace

std::optional<Contact>
ContactBetween(const Body& a, const PolygonShape& sa, const Body& b, const PolygonShape& sb) {
  return PolygonContact({a, sa.Vertices()}, {b, sb.Vertices()});
}

std::optional<Contact>
ContactBetween(const Body& a, const PolygonShape& sa, const Body& b, BoxShape sb) {
  const auto vertices = BoxVertices(sb);
  return PolygonContact({a, sa.Vertices()}, {b, vertices});
}

std::optional<Contact>
ContactBetween(const Body& a, BoxShape sa, const Body& b, const PolygonShape& sb) {
  return Reverse(ContactBetween(b, sb, a, sa));
}

std::optional<Contact>
ContactBetween(const Body& a, CircleShape sa, const Body& b, const PolygonShape& sb) {
  const Vec2 center = b.WorldToLocal(a.position);
  const auto vertices = sb.Vertices();
  float maxSeparation = -std::numeric_limits<float>::infinity();
  float closestDistanceSquared = std::numeric_limits<float>::infinity();
  Vec2 closest, nearestNormal;
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const Vec2 start = vertices[i], edge = Sub(vertices[(i + 1) % vertices.size()], start);
    const Vec2 outward = Scale(Vec2{edge.y, -edge.x}, 1.0f / Length(edge));
    const float separation = Dot(outward, Sub(center, start));
    if (separation > sa.radius + contactTolerance) {
      return std::nullopt;
    }
    if (separation > maxSeparation) {
      maxSeparation = separation;
      nearestNormal = outward;
    }
    const float t = std::clamp(Dot(Sub(center, start), edge) / Dot(edge, edge), 0.0f, 1.0f);
    const Vec2 candidate = Add(start, Scale(edge, t));
    const Vec2 delta = Sub(candidate, center);
    const float distanceSquared = Dot(delta, delta);
    if (distanceSquared < closestDistanceSquared) {
      closestDistanceSquared = distanceSquared;
      closest = candidate;
    }
  }
  Vec2 localNormal;
  float penetration;
  if (maxSeparation <= 0.0f) {
    // For an enclosed center, move the circle out through the nearest face.
    localNormal = Scale(nearestNormal, -1.0f);
    closest = Sub(center, Scale(nearestNormal, maxSeparation));
    penetration = sa.radius - maxSeparation;
  } else {
    const float distance = std::sqrt(closestDistanceSquared);
    if (distance > sa.radius + contactTolerance) {
      return std::nullopt;
    }
    localNormal = distance > 0.0f ? Scale(Sub(closest, center), 1.0f / distance)
                                  : Scale(nearestNormal, -1.0f);
    penetration = std::max(sa.radius - distance, 0.0f);
  }
  const Vec2 normal = Rotate(localNormal, b.angle);
  const Vec2 point =
      Scale(Add(b.LocalToWorld(closest), Add(a.position, Scale(normal, sa.radius))), 0.5f);
  return Contact{normal, penetration, {point, {}}, 1};
}

std::optional<Contact>
ContactBetween(const Body& a, const PolygonShape& sa, const Body& b, CircleShape sb) {
  return Reverse(ContactBetween(b, sb, a, sa));
}
} // namespace physeng::detail
