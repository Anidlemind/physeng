#pragma once

#include "common/vec2.hpp"
#include <optional>
#include <span>
#include <string_view>
#include <vector>

struct GLFWwindow;

namespace renderer {

enum class Key { Left, Right, Up, Down, Space, A, D, W, S, Reset };

enum class MouseButton { Left, Right };

struct Color {
  float r = 1.0f, g = 0.5f, b = 0.0f, a = 1.0f;
};

struct CircleInstance {
  common::Vec2 position;
  float radius;
  float angle = 0.0f;
  Color tint{1, 1, 1, 1}; // Multiplied by the color set through SetColor.
};

class Renderer {
public:
  Renderer();
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;
  Renderer(Renderer&&) = delete;
  Renderer& operator=(Renderer&&) = delete;

  bool ShouldClose() const;
  void PollEvents();

  void BeginFrame(Color background = {0.075f, 0.085f, 0.105f, 1.0f});
  void SetColor(Color color);
  void DrawRectangle(common::Vec2 position, common::Vec2 halfExtents, float angle = 0.0f);
  void DrawCircle(common::Vec2 position, float radius, float angle = 0.0f);
  // Disable outlines and orientation marks for fluid particle rendering.
  void DrawCircles(std::span<const CircleInstance> circles, bool showDetails = true);
  void
  DrawPolygon(common::Vec2 position, std::span<const common::Vec2> vertices, float angle = 0.0f);
  void DrawSegment(common::Vec2 first, common::Vec2 second, float thickness = 0.04f);
  void DrawSpring(common::Vec2 first, common::Vec2 second, float width = 0.12f);
  // Screen coordinates use logical window pixels, with the origin at top left.
  void DrawScreenRectangle(common::Vec2 position, common::Vec2 size);
  void DrawText(common::Vec2 position, std::string_view text, float pixelSize = 2.0f);
  void SetVisibleHeight(float height);
  void SetVSync(bool enabled);
  void EndFrame();

  bool IsKeyDown(Key key) const;
  bool IsMouseButtonDown(MouseButton button) const;

  std::optional<common::Vec2> CursorWorldPosition() const;

  common::Vec2 ViewHalfExtents() const { return cameraHalfExtents_; }

private:
  static void FramebufferSizeCallback(GLFWwindow* window, int width, int height);
  void UpdateFramebufferView(int width, int height);
  void Shutdown();
  void DrawShape(common::Vec2 position, common::Vec2 halfExtents, float angle, bool circle);

  std::vector<common::Vec2> textVertices_;
  GLFWwindow* window_ = nullptr;
  bool glfwInitialized_ = false;
  unsigned int program_ = 0;
  unsigned int vertexBuffer_ = 0;
  unsigned int indexBuffer_ = 0;
  unsigned int vertexArray_ = 0;
  unsigned int polygonVertexBuffer_ = 0;
  unsigned int polygonVertexArray_ = 0;
  unsigned int circleVertexArray_ = 0;
  unsigned int circleInstanceBuffer_ = 0;
  int instancingLocation_ = -1;
  int positionLocation_ = -1;
  int halfExtentsLocation_ = -1;
  int cameraLocation_ = -1;
  int angleLocation_ = -1;
  int shapeKindLocation_ = -1;
  int colorLocation_ = -1;
  float visibleHeight_ = 10.0f;
  common::Vec2 cameraHalfExtents_{1.0f, 5.0f};
};

} // namespace renderer
