# Physeng

A small 2D physics sandbox. The demo has a hanging
spring, a rope attached to a box, a ball, and a convex polygon you can throw around.

**Hold the left mouse button to grab a body. Release it to let go.** Movable bodies
highlight when you hover over them.

## Run it

Build with a C++20-capable compiler:

```sh
cmake -S . -B build
cmake --build build -j 4
./build/Physeng
```

## Change the scene

Start with [src/app/demo_scene.cpp](src/app/demo_scene.cpp). It contains all the
positions, shapes, materials, spring settings, and rope attachments in the demo.
After editing it, rebuild and run again.

A scene is a description of what to create. Give each body a name, then use those
names when connecting springs and ropes.

`inverseMass = 0` makes a body static. Omit its shape to create a particle without
collision geometry. A shape is not needed for gravity, forces, or connections.
