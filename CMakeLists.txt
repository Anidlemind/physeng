cmake_minimum_required(VERSION 3.28)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(Physeng
  VERSION 0.1.0
  LANGUAGES C CXX)

function(configure_cpp_target target)
  target_compile_features(${target} PUBLIC cxx_std_20)
  set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
endfunction()

add_library(common INTERFACE)
target_include_directories(common INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(physeng_physics STATIC
  src/physics/body.cpp
  src/physics/world.cpp
  src/physics/collision.cpp
  src/physics/collision_filter.cpp
  src/physics/distance_constraint.cpp
  src/physics/forces.cpp
  src/physics/springs.cpp
  src/physics/scene.cpp
  src/physics/objects/enclosure.cpp
  src/physics/objects/spring.cpp
  src/physics/objects/rope.cpp)
target_link_libraries(physeng_physics PUBLIC common)
configure_cpp_target(physeng_physics)

option(PHYSENG_BUILD_APP "Build the OpenGL application" ON)
option(PHYSENG_USE_SYSTEM_GLFW "Use an installed GLFW instead of downloading it" OFF)
if(PHYSENG_BUILD_APP)
  if(PHYSENG_USE_SYSTEM_GLFW)
    find_package(glfw3 3.3 REQUIRED)
  else()
    include(FetchContent)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    if(UNIX AND NOT APPLE)
      # X11 works with WSLg; Wayland can be enabled when its development files are installed.
      option(GLFW_BUILD_WAYLAND "Build GLFW's Wayland backend" OFF)
    endif()
    FetchContent_Declare(glfw
      URL https://github.com/glfw/glfw/archive/refs/tags/3.4.tar.gz
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
      TIMEOUT 60
      TLS_VERIFY TRUE)
    FetchContent_MakeAvailable(glfw)
  endif()
  add_library(glad STATIC external/glad/src/gl.c)
  target_include_directories(glad PUBLIC external/glad/include)

  add_library(renderer STATIC src/renderer/renderer.cpp)
  target_link_libraries(renderer PUBLIC common PRIVATE glfw glad)
  # GLAD provides the OpenGL declarations; GLFW only handles the window.
  target_compile_definitions(renderer PRIVATE GLFW_INCLUDE_NONE)
  configure_cpp_target(renderer)

  add_executable(Physeng
    main.cpp
    src/app/demo_scene.cpp
    src/app/draw_scene.cpp
    src/app/mouse_drag.cpp
    src/app/window_motion.cpp)
  target_link_libraries(Physeng PRIVATE physeng_physics renderer)
  configure_cpp_target(Physeng)
endif()

