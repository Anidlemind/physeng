# Physeng

A small 2D physics sandbox. The default demo fills a large enclosure with colliding
circle particles until the count reaches 8,000, regardless of FPS. Particles launch from one fixed
point with an identical impulse; the orange marker shows the emitter. Frame time, FPS, and particle
count appear in the top-left corner. Spawning stops permanently at the limit;
the simulation continues. Measurements average half-second windows after a
one-second warm-up, with VSync disabled. Frame time includes physics, rendering,
the overlay, and buffer swapping.

Run `./build/Physeng --water` for a water tank with 512 particles. A tall column
falls and spreads across the tank. Hold the left mouse button and drag to pull
water with a wide, strong brush; right-click to reset. Particles change from blue
through cyan and yellow to red as their speed increases from 0 to 20 world units
per second. Faster particles stay red. The overlay shows particle count, frame
time, FPS, and the speed color scale.

Run `./build/Physeng --bridge` for the rope bridge: twelve movable planks
connected by elastic, breakable links between two fixed supports. A ball lands on
the deck; a heavy crate and a polygon wait on the ledges for you to drag onto it.
A water block fills the space beneath the bridge between its supports.
It uses the same speed colors as the water demo. Grab a body and lower it into the
water: it displaces particles, receives pressure and drag, and can rotate. Light
objects float while sufficiently heavy objects sink.

Run `./build/Physeng --puffer` for the soft-body jumping test level. Use A/D or
Left/Right to move. Press Space, Up, or W to puff up and jump from the ground;
holding the button keeps the character inflated. Release it or hold Down/S to
squeeze down. R resets the level.

The separate `--puffer-water` scene preserves the buoyancy prototype for later
swimming work.

**In the bridge demo, hold the left mouse button to grab a body. Release it to let go.**
Movable bodies highlight when you hover over them. Pull a plank or add weight to stretch the
bridge. Links fade from beige toward red as they approach their breaking length.
To snap it, drag the crate onto the middle of the deck and press it downward.

## Run it

```sh
cmake -S . -B build
cmake --build build -j 4
./build/Physeng             # particle stress test
./build/Physeng --bridge    # rope bridge
./build/Physeng --water     # particle water
./build/Physeng --puffer    # inflatable soft-body character
./build/Physeng --puffer-water # optional buoyancy prototype
```

## Change the scene

Start with [src/app/demo_scene.cpp](src/app/demo_scene.cpp). It contains all the
positions, shapes, materials, spring settings, and rope attachments in the demo.
The stress test settings (enclosure, radius, spawn rate, and particle limit) are
in [src/app/particle_demo.cpp](src/app/particle_demo.cpp), including `spawnPoint`,
`launchImpulse`, `particleMass`, `particlesPerSecond`, and `particleLimit`. The default emitter
attempts to launch 60 particles/second with impulse `(18, 5)` and mass `1`, giving initial
velocity `(18, 5)` world units/second. Emission happens between fixed physics
steps, rather than in render-frame batches. The emitter sits at `(-10.8, 6)` above
the pile and waits when its spawn point is occupied, without building up a burst.
After editing either
demo, rebuild and run again.

## Water

The reusable [Fluid](include/physeng/fluid.hpp) stores compact particles with a
position and velocity. `AddBlock` fills a rectangular region at the configured
spacing; `AddParticle` adds one particle. `Step` advances the water, and `Stir`
blends nearby velocities toward a brush velocity. Access positions through
`Particles()` and the density ratios through `Densities()` after a step.

```cpp
physeng::Fluid water({.particleSpacing = 0.32f, .smoothingRadius = 0.96f});
water.AddBlock({.minimum = {-11.7f, -6.7f}, .columns = 32, .rows = 16});
water.Step(1.0 / 60.0);
```

This is a 2D adaptation of [Position Based Fluids](https://mmacklin.com/pbf_sig_preprint.pdf).
Particles estimate local density with a smooth distance-weighted kernel. Each
solver pass computes density corrections from a shared position snapshot, then
applies them together. This resists compression while allowing particles to flow.
Free-surface particles do not pull themselves into clumps to fill missing neighbors.
A small artificial-pressure term helps separate close particles. Velocities are
reconstructed from the corrected positions and smoothed with viscosity.

The fluid uses the renderer's instanced circles. Its own bounded grid groups
neighbors in contiguous arrays, and cached neighbor lists include a movement
margin. The demo uses larger particle spacing to keep roughly the same water
volume with fewer particles, at the cost of finer detail. It uses a 60 Hz tick,
two integration substeps, and four density passes per substep. Smooth blue splats hide rigid-body
outlines and rotation marks; this is particle rendering, not a reconstructed surface.

Tune `FluidSettings::particleSpacing`, `smoothingRadius` (usually about three times
the spacing), `restDensity`, `viscosity`, `solverIterations`, and `substeps`.
The tank interior is `minimum` to `maximum`; change the demo in
[src/app/water_demo.cpp](src/app/water_demo.cpp). Density is normalized to `1` at
rest; this 2D model assigns each equal-volume particle an area of `spacing²`.
More solver passes improve incompressibility at extra CPU cost.
The water demo's `dragRadius`, `dragStrength`, `dragSpeedMultiplier`, and
`maximumDragSpeed` constants control the brush. The shared `colorSpeedLimit` in
[src/app/draw_water.cpp](src/app/draw_water.cpp) controls speed colors. Strength
controls how quickly local water velocities approach the brush velocity. The
brush targets twice the cursor speed, capped at 35 world units per second, to
help lift water against gravity and viscosity; its influence fades toward the
edge. Colors use speed (the length of velocity), so direction does
not change the color. Each circle carries its own tint in the same instanced draw.

Water remains inside a rectangular tank. `Step(dt)` runs it independently;
`Step(dt, world)` also couples it to the world's box, circle, and convex polygon
colliders. Shapes carry volume samples which contribute to water density and
pressure. Moving bodies receive the opposite pressure impulse, including torque.
Mass-weighted contact corrections keep particles out of shapes, and viscosity
exchanges drag impulses. Buoyancy emerges from pressure and contact with the water;
there is no separate upward force switched on when an object enters the tank.
`restDensity` is water mass per unit area: particle mass is `restDensity * spacing²`.
The bridge uses density `3`, so its ball and light planks can float and the heavier
crate sinks. Bodies without colliders do not interact with water; rope nodes that
have colliders do. Multiple colliders on one body should describe disjoint volume,
because overlapping samples would count that volume twice.

Solid volume sampling is approximate and capped at 64 cells per axis per collider;
larger shapes have coarser pressure sampling. Fluid contacts use the existing shape
geometry and sweep against static boxes. Other fluid contact pairs are discrete.
Tank walls project particles
back inside. A regular lattice of stationary samples behind the walls also
contributes to density and pressure, following the boundary-particle approach in
[Unified Particle Physics](https://mmacklin.com/uppfrta_preprint.pdf). Without that
support, missing neighbors behind a wall allow water to compress into a dense
floor layer. Samples use their represented area as a weight, are evaluated only
near walls, and never enter rendering or viscosity. They add pressure support
without making water stick to a stationary wall. The same lattice covers corners
without counting their solid volume twice. Free-surface density remains approximate.
Call fluid methods and read its state from the same thread.

## Pufferfish and soft bodies

The puffer scene uses 19 equal-mass circle bodies on a hexagonal grid, connected
by 42 elastic distance constraints. The triangles keep the mesh together while
allowing local deformation. Linked neighbors can compress elastically; other
nodes collide to prevent the mesh folding through itself. Water and other world
colliders interact with each node. Ground checks exclude the character's own
nodes. This is a coarse particle
soft body, rather than a continuous elastic surface.

`objects::CreateSoftBody(world, desc)` creates that mesh.
`objects::SetSoftBodyRadius(world, handle, radius)` changes its circle radii and
constraint rest lengths. It preserves node masses and lets the solver move the
nodes, so inflation pushes water aside instead of teleporting the character.
The main level has no water. A fresh puff applies a launch impulse across the
mesh when its nodes have actual ground support; walls and ceilings do not count.
Holding the button does not create repeated jumps or air jumps. Ground grace
lasts 80 ms and a held early jump can buffer for 120 ms. Expansion against the
floor can add another kick, so the controller limits average upward speed to
keep jumps controllable. Horizontal control applies forces across the nodes.

In the water prototype, displaced solid area grows roughly with radius squared.
The puffer's mass stays at `3`: it is denser than water when squeezed and less
dense when inflated. Buoyancy then provides vertical movement while submerged.

Tune the two radii, breathing speeds, movement speed/acceleration, and jump speed
in [PufferSettings](src/app/puffer_scene.hpp). Level geometry, the optional water
scene, spawn position, mass, and mesh compliance are in
[src/app/puffer_scene.cpp](src/app/puffer_scene.cpp). Drawing and
keyboard controls are in [src/app/puffer_demo.cpp](src/app/puffer_demo.cpp).
`World::SetCircleRadius` updates collision bounds, inertia, and contact caches;
`World::SetDistanceConstraintLength` updates a link's rest length.

A scene is a description of what to create. Give each body a name, then use those
names when connecting springs and ropes.
Add a fluid tank to `SceneDesc::fluids` with its `FluidSettings` and initial
`FluidBlockDesc` blocks. `CreateScene` creates those tanks, and `Scene::Step`
advances both the rigid world and the fluids. The bridge's tank and block dimensions
are configured in [src/app/demo_scene.cpp](src/app/demo_scene.cpp).
`SceneDesc::softBodies` also accepts soft-body descriptions. Scene creation fills
water after creating bodies, skipping particles that would start inside their
colliders. `Fluid::AddBlock(block, world)` provides the same exclusion when
building a tank manually; the overload without a world fills every grid point.

`inverseMass = 0` makes a body static. Omit its shape to create a particle without
collision geometry. A shape is not needed for gravity, forces, or connections.

Ropes can stretch and snap. Set `RopeDesc::compliance` to control elasticity:
zero is rigid, and larger values stretch more under the same load. This is the
whole rope's compliance in metres/newton; it is divided across its segments.
Set `breakStretchRatio` to a value above one (for example, `1.5f` allows 50%
extension). Omit it for an unbreakable rope. The bridge uses a break ratio of
`2.0f`: a link snaps above twice its original length. Tune `plankCount`,
`linkCompliance`, and `breakStretchRatio` in the demo scene. Broken links stay in
storage so their IDs remain valid, but they are neither solved nor drawn. Rope particles keep moving and
colliding. Detached pieces regain collisions with the support, payload, and former
neighbor across a broken link; pieces still attached keep their exclusions.
Explicit exclusions made through `SetCollisionEnabled` remain in effect.

The rope color uses `clamp((currentLength / restLength - 1) / (breakStretchRatio - 1), 0, 1)`.
An unstretched link is beige, and a link at its breaking length is red. Compressed
links keep their normal color; unbreakable links do not show a breaking warning.

The physics broad phase puts each collider's axis-aligned bounds into a spatial
grid and only checks overlapping candidates. Large colliders use a bounds fallback
instead of filling excessive cells. Candidate pairs are cached with padded bounds;
a collider added to the world or moving outside those bounds triggers a rebuild.
Bounds are checked after constraints on each solver pass, and pairs keep a
consistent order. Circle contacts use a specialized solver that retains friction,
restitution, rotation, and the general solver's position correction. Tune
`PhysicsSettings::broadPhaseCellSize` for scenes with different object sizes
(default: `0.5` world units). Solver iterations and the 120 Hz physics step are
unchanged. Contacts cache their normal and friction impulses across steps. The
solver applies that cached support before iterating, then adjusts the accumulated
impulses in either direction while keeping the total normal impulse nonnegative.
This lets resting piles keep their support without rebuilding it from zero every
frame. Contact caches survive grid rebuilds, and separated contacts clear them.
A 0.001-unit penetration allowance avoids chasing tiny overlaps. Moving pairs
correct a quarter of their overlap per pass, so one correction does not shove a
whole pile of neighbors around. Static surfaces still project fully. Fixed-wall
contacts get a final solve after neighbor corrections. The particle scene disables
the implicit floor and uses all four visible enclosure walls. Wall ends overlap
at enclosure corners so a deeply pressed particle is pushed back into the box.
Circles also sweep from their previous position against static boxes, including
rotated faces and rounded corner contacts. Swept bounds keep crossed walls in
the candidate list, so integration or neighbor corrections cannot skip a thin
wall. Other shape pairs still use discrete collision detection.
The particle demo draws all circles in one instanced draw call.

For performance comparisons, keep the build type, scene, timestep, and solver
settings the same. FPS includes rendering and the HUD, so hardware and window size
also affect the measured FPS and the time needed to reach the particle limit.

Both rigid-body physics and fluids run sequentially on the calling thread.
Spatial grids, cached contacts and neighbors, and instanced rendering remain in use.
