#include "renderer/renderer.hpp"

#include <GLFW/glfw3.h>
#include <glad/gl.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace renderer {

namespace {

constexpr const char* vertexSource = R"glsl(#version 410 core
layout(location = 0) in vec2 position;
uniform vec2 objectPosition;
uniform vec2 objectHalfExtents;
uniform vec2 cameraHalfExtents;
uniform float objectAngle;
out vec2 localPosition;

void main() {
  localPosition = position;
  float c = cos(objectAngle);
  float s = sin(objectAngle);
  mat2 rotation = mat2(c, s, -s, c);
  vec2 worldPosition = rotation * (position * objectHalfExtents) + objectPosition;
  vec2 normalizedPosition = worldPosition / cameraHalfExtents;
  gl_Position = vec4(normalizedPosition, 0.0, 1.0);
}
)glsl";

constexpr const char* fragmentSource = R"glsl(#version 410 core
layout(location = 0) out vec4 fragmentColor;
in vec2 localPosition;
uniform int drawCircle;
uniform vec4 objectColor;
uniform vec2 objectHalfExtents;

void main() {
  vec2 localPoint = localPosition * objectHalfExtents;
  float distanceToEdge = drawCircle != 0
    ? length(localPoint) - objectHalfExtents.x
    : max(abs(localPoint.x) - objectHalfExtents.x, abs(localPoint.y) - objectHalfExtents.y);
  float pixelWidth = max(fwidth(distanceToEdge), 0.0001);
  float coverage = 1.0 - smoothstep(-pixelWidth, pixelWidth, distanceToEdge);
  if (coverage <= 0.0) discard;

  float border = smoothstep(-max(1.5 * pixelWidth, 0.015), 0.0, distanceToEdge);
  float lighting = 0.96 + 0.04 * localPosition.y;
  vec3 color = objectColor.rgb * lighting * mix(1.0, 0.72, border);
  // A radial mark makes a circle's otherwise invisible rotation observable.
  if (drawCircle != 0 && abs(localPosition.y) < 0.06 &&
      localPosition.x > 0.15 && localPosition.x < 0.85) {
    color = objectColor.rgb * 0.3;
  }
  fragmentColor = vec4(color, objectColor.a * coverage);
}
)glsl";

std::string ShaderLog(GLuint shader) {
  GLint length = 0;
  glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
  if (length <= 0) {
    return "No shader diagnostic log available";
  }
  std::vector<char> log(length);
  glGetShaderInfoLog(shader, length, nullptr, log.data());
  return log.data();
}

std::string ProgramLog(GLuint program) {
  GLint length = 0;
  glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
  if (length <= 0) {
    return "No program diagnostic log available";
  }
  std::vector<char> log(length);
  glGetProgramInfoLog(program, length, nullptr, log.data());
  return log.data();
}

GLuint CompileShader(GLenum stage, const char* source) {
  const GLuint shader = glCreateShader(stage);
  if (shader == 0) {
    throw std::runtime_error("Failed to create shader");
  }
  try {
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint status = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (status != GL_TRUE) {
      throw std::runtime_error("Shader compilation failed: " + ShaderLog(shader));
    }
  } catch (...) {
    glDeleteShader(shader);
    throw;
  }
  return shader;
}

GLuint CreateProgram() {
  GLuint vertex = 0, fragment = 0, program = 0;
  try {
    vertex = CompileShader(GL_VERTEX_SHADER, vertexSource);
    fragment = CompileShader(GL_FRAGMENT_SHADER, fragmentSource);
    program = glCreateProgram();
    if (program == 0) {
      throw std::runtime_error("Failed to create shader program");
    }
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glLinkProgram(program);
    GLint status = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &status);
    if (status != GL_TRUE) {
      throw std::runtime_error("Shader linking failed: " + ProgramLog(program));
    }
    glDetachShader(program, vertex);
    glDetachShader(program, fragment);
  } catch (...) {
    if (program != 0) {
      glDeleteProgram(program);
    }
    if (fragment != 0) {
      glDeleteShader(fragment);
    }
    if (vertex != 0) {
      glDeleteShader(vertex);
    }
    throw;
  }
  glDeleteShader(fragment);
  glDeleteShader(vertex);
  return program;
}

} // namespace

Renderer::Renderer() {
  try {
    glfwSetErrorCallback([](int code, const char* description) {
      std::fprintf(stderr, "GLFW error %d: %s\n", code, description);
    });
    if (!glfwInit()) {
      throw std::runtime_error("Failed to initialize GLFW");
    }
    glfwInitialized_ = true;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
#ifdef __APPLE__
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#else
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
#endif
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    window_ = glfwCreateWindow(1280, 720, "Physeng | grab a body and drag", nullptr, nullptr);
    if (!window_) {
      throw std::runtime_error("Failed to create OpenGL window");
    }
    glfwMakeContextCurrent(window_);
    if (!gladLoadGL(glfwGetProcAddress)) {
      throw std::runtime_error("Failed to load OpenGL functions");
    }
    if (!GLAD_GL_VERSION_4_1) {
      throw std::runtime_error("OpenGL 4.1 or newer is required");
    }

    program_ = CreateProgram();
    positionLocation_ = glGetUniformLocation(program_, "objectPosition");
    halfExtentsLocation_ = glGetUniformLocation(program_, "objectHalfExtents");
    cameraLocation_ = glGetUniformLocation(program_, "cameraHalfExtents");
    angleLocation_ = glGetUniformLocation(program_, "objectAngle");
    circleLocation_ = glGetUniformLocation(program_, "drawCircle");
    colorLocation_ = glGetUniformLocation(program_, "objectColor");
    if (positionLocation_ == -1 || halfExtentsLocation_ == -1 || cameraLocation_ == -1 ||
        angleLocation_ == -1 || circleLocation_ == -1 || colorLocation_ == -1) {
      throw std::runtime_error("Required shader uniform not found");
    }

    // Reusable corners from -1 to +1; the shader scales them to the body's size.
    constexpr std::array<float, 8> vertices{-1, -1, 1, -1, 1, 1, -1, 1};
    constexpr std::array<GLuint, 6> indices{0, 1, 2, 2, 3, 0};

    glGenVertexArrays(1, &vertexArray_);
    glBindVertexArray(vertexArray_);
    glGenBuffers(1, &vertexBuffer_);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);
    glGenBuffers(1, &indexBuffer_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glfwSetWindowUserPointer(window_, this);
    InitializeWindowPosition();
    glfwSetFramebufferSizeCallback(window_, FramebufferSizeCallback);
    int width = 0, height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    UpdateFramebufferView(width, height);
    glfwSwapInterval(1);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    SetColor({});
  } catch (...) {
    Shutdown();
    throw;
  }
}

Renderer::~Renderer() {
  Shutdown();
}

void Renderer::Shutdown() {
  if (window_) {
    glfwMakeContextCurrent(window_);
    if (vertexArray_ != 0) {
      glDeleteVertexArrays(1, &vertexArray_);
    }
    if (indexBuffer_ != 0) {
      glDeleteBuffers(1, &indexBuffer_);
    }
    if (vertexBuffer_ != 0) {
      glDeleteBuffers(1, &vertexBuffer_);
    }
    if (program_ != 0) {
      glDeleteProgram(program_);
    }
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }
  if (glfwInitialized_) {
    glfwTerminate();
    glfwInitialized_ = false;
  }
}

bool Renderer::ShouldClose() const {
  return glfwWindowShouldClose(window_);
}

void Renderer::PollEvents() {
  glfwPollEvents();
}

void Renderer::FramebufferSizeCallback(GLFWwindow* window, int width, int height) {
  auto* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
  renderer->UpdateFramebufferView(width, height);
}

void Renderer::InitializeWindowPosition() {
#if GLFW_VERSION_MAJOR > 3 || (GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 4)
  // Wayland deliberately does not expose desktop window positions.
  if (glfwGetPlatform() == GLFW_PLATFORM_WAYLAND) {
    return;
  }
#endif
  int x = 0, y = 0;
  glfwGetWindowPos(window_, &x, &y);
  if (glfwGetError(nullptr) != GLFW_NO_ERROR) {
    return;
  }
  windowPosition_ = WindowPositionSample{x, y, std::chrono::steady_clock::now()};
  glfwSetWindowPosCallback(window_, WindowPositionCallback);
}

void Renderer::WindowPositionCallback(GLFWwindow* window, int x, int y) {
  auto* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
  renderer->windowPosition_ = WindowPositionSample{x, y, std::chrono::steady_clock::now()};
}

void Renderer::UpdateFramebufferView(int width, int height) {
  glViewport(0, 0, width, height);
  if (width <= 0 || height <= 0) {
    return;
  }
  const float aspectRatio = static_cast<float>(width) / static_cast<float>(height);
  const float halfHeight = visibleHeight_ * 0.5f;
  cameraHalfExtents_ = {halfHeight * aspectRatio, halfHeight};
  glProgramUniform2f(program_, cameraLocation_, cameraHalfExtents_.x, cameraHalfExtents_.y);
}

void Renderer::BeginFrame(Color background) {
  glClearColor(background.r, background.g, background.b, background.a);
  glClear(GL_COLOR_BUFFER_BIT);
}

void Renderer::SetColor(Color color) {
  glProgramUniform4f(program_, colorLocation_, color.r, color.g, color.b, color.a);
}

void Renderer::DrawShape(common::Vec2 position,
                         common::Vec2 halfExtents,
                         float angle,
                         bool circle) {
  glProgramUniform2f(program_, positionLocation_, position.x, position.y);
  glProgramUniform2f(program_, halfExtentsLocation_, halfExtents.x, halfExtents.y);
  glProgramUniform1f(program_, angleLocation_, angle);
  glProgramUniform1i(program_, circleLocation_, circle ? 1 : 0);
  glUseProgram(program_);
  glBindVertexArray(vertexArray_);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
}

void Renderer::DrawRectangle(common::Vec2 position, common::Vec2 halfExtents, float angle) {
  DrawShape(position, halfExtents, angle, false);
}

void Renderer::DrawCircle(common::Vec2 position, float radius, float angle) {
  DrawShape(position, {radius, radius}, angle, true);
}

void Renderer::DrawSegment(common::Vec2 first, common::Vec2 second, float thickness) {
  const float dx = second.x - first.x;
  const float dy = second.y - first.y;
  const float length = std::hypot(dx, dy);
  if (length == 0.0f) {
    return;
  }
  DrawRectangle({(first.x + second.x) * 0.5f, (first.y + second.y) * 0.5f},
                {length * 0.5f, thickness * 0.5f},
                std::atan2(dy, dx));
}

void Renderer::DrawSpring(common::Vec2 first, common::Vec2 second, float width) {
  const float dx = second.x - first.x, dy = second.y - first.y;
  const float length = std::hypot(dx, dy);
  if (length == 0.0f) {
    return;
  }
  const common::Vec2 side{-dy / length, dx / length};
  common::Vec2 previous = first;
  constexpr int coils = 16;
  for (int i = 1; i <= coils; ++i) {
    const float t = static_cast<float>(i) / coils;
    const float offset = i == coils ? 0.0f : (i % 2 == 0 ? width : -width);
    const common::Vec2 point{first.x + dx * t + side.x * offset,
                             first.y + dy * t + side.y * offset};
    DrawSegment(previous, point, 0.025f);
    previous = point;
  }
}

void Renderer::EndFrame() {
  glfwSwapBuffers(window_);
}

bool Renderer::IsKeyDown(Key key) const {
  static const std::unordered_map<Key, int> keyMapping_{
      {Key::Left, GLFW_KEY_LEFT},
      {Key::Right, GLFW_KEY_RIGHT},
      {Key::Up, GLFW_KEY_UP},
  };

  return glfwGetKey(window_, keyMapping_.at(key)) == GLFW_PRESS;
}

bool Renderer::IsMouseButtonDown(MouseButton button) const {
  if (!glfwGetWindowAttrib(window_, GLFW_FOCUSED)) {
    return false;
  }
  const int glfwButton =
      button == MouseButton::Left ? GLFW_MOUSE_BUTTON_LEFT : GLFW_MOUSE_BUTTON_RIGHT;
  return glfwGetMouseButton(window_, glfwButton) == GLFW_PRESS;
}

std::optional<common::Vec2> Renderer::CursorWorldPosition() const {
  if (!glfwGetWindowAttrib(window_, GLFW_FOCUSED) || glfwGetWindowAttrib(window_, GLFW_ICONIFIED)) {
    return std::nullopt;
  }
  int width = 0, height = 0, framebufferWidth = 0, framebufferHeight = 0;
  glfwGetWindowSize(window_, &width, &height);
  glfwGetFramebufferSize(window_, &framebufferWidth, &framebufferHeight);
  if (width <= 0 || height <= 0 || framebufferWidth <= 0 || framebufferHeight <= 0) {
    return std::nullopt;
  }
  double x = 0.0, y = 0.0;
  glfwGetCursorPos(window_, &x, &y);
  // Cursor coordinates use window units, not framebuffer pixels (important for HiDPI).
  return common::Vec2{static_cast<float>((2.0 * x / width - 1.0) * cameraHalfExtents_.x),
                      static_cast<float>((1.0 - 2.0 * y / height) * cameraHalfExtents_.y)};
}

} // namespace renderer
