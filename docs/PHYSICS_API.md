# Physics API Reference (`cvc::physics`)

`cvc::physics` is libcvc's engine-agnostic rigid-body facade. A `world` holds bodies made of
shapes, steps in fixed quanta and reports contacts. It saves, restores and hashes its whole state.
A `world_binding` configures a world from an app's state tree and publishes its telemetry there.
The backends are `null` (bookkeeping only) and `jolt` (Jolt Physics, when the build has it).

Roadmap: [`roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md`](roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md).

## Headers

| Header | What |
|---|---|
| `cvc/physics/types.h` | `vec3`, `quat`, `motion_type`, `body_id`, `material` |
| `cvc/physics/shape.h` | `sphere`, `box`, `convex_hull`, `mesh` (a `cvc::geometry`), `terrain` (a `cvc::height_field`) |
| `cvc/physics/body.h` | `body_desc` (with an optional `hull`), `body_state` |
| `cvc/physics/backend.h` | `backend`, capability bits, `available_backends()` |
| `cvc/physics/world.h` | `world_config`, `world`, `contact_event`, `ray_hit`, `step_report` |
| `cvc/physics/field.h` | `field` (a `cvc::volume` as water, snow or wind), `field_load`, `water_sample` (`world::water_at`) |
| `cvc/physics/flow_velocity.h` | `flow_velocity`, a flow field's velocity in space and time (`field::wind`), and `wind_flow`, the scene's wind: `cvc::algo`'s gusts and height profile (below, and OCEAN-AND-WEATHER-ROADMAP.md E5, O14) |
| `cvc/physics/fluid_surface.h` | `fluid_surface`, a fluid field's free surface (its level, waves and water velocity, read at `world::field_time()`); `still_surface`, `wave_train_surface` (linear Airy trains). A field reads a cell's height and velocity with `height_with` / `velocity_with` and a `surface_memo` it owns, so a surface can share work between the two reads |
| `cvc/physics/hull.h` | `hull_mesh`, `box_hull()`, `loft_hull()` (a closed hull through stations, each a half-section from keel to sheer: a model's hull baked by `tools/boat_hulls/bake_loft.py`), `hull_waterline_box()` (the largest box inside a hull at a waterline, for a renderer's water cut-out): a boat's hull, a closed triangle list apart from the collision shape, that a fluid pushes on triangle by triangle (hydrostatic pressure split at the waterline, ITTC-57 friction, pressure drag, slamming, and an opt-in lift, `lift_coefficient`, for a V's grip and a planing bottom) in place of the body's sample cells |
| `cvc/physics/immersion.h` | `immersion_body` (a body's outline as the water sees it: its box, sphere or convex hull, or its `hull_mesh`), `immersion_water` (a fluid field's surface, time and box), `immersion_grid`, `add_immersion()`: a body's columns of immersed depth zeta over a grid window's cells, from its shape, its pose and the water, as the field step immerses it. The wake windows' sources (OCEAN-AND-WEATHER-ROADMAP.md C2, O9), computable without a world (a renderer, a client from a replicated pose); the same bits on every platform |
| `cvc/physics/ocean_surface.h` | `ocean_surface`: a `cvc::ocean::sea_surface` (`cvc/ocean/sea_surface.h`) as a free surface. That is the sea a renderer draws from the same cascades and shore layer, with breakers and the swash. For many floating bodies, `sea_params::source_cell_m` (0.5 m in lsystem_coast) lets their field cells share the sea's inversions, and a cell's height and velocity share one source through the field's memo |
| `cvc/physics/boat.h` | `boat_desc` (a hull body, `propeller_desc`, `rudder_desc`, the actuators' time constant), `boat_controls`, `boat_state`: a boat whose propeller and rudder (or outboard) the world applies every step |
| `cvc/physics/vehicle.h` | `vehicle_desc` (with `humvee()`, `roboracer()`), `drive_command`, `vehicle_controls`, `vehicle_state`, `wheel_state` |
| `cvc/physics/world_binding.h` | `world_binding`: the state-tree face of a world |
| `cvc/physics/pose_tracker.h` | `pose_tracker`, `pose`, `interpolate()`, `pose_matrix()`: the last two states, for a view |
| `cvc/gl/physics_scene_binding.h` (cvcGL) | `physics_scene_binding`: a world's bodies drawn in a scene graph |
| `cvc/physics/world_geometry.h` | `add_world_geometry`: a `cvc::world::world_geometry`'s static bodies and materials |
| `cvc/physics/nav_plant.h` | `nav_plant`: a world's vehicles as `cvc::nav`'s plant (`cvc/nav/vehicle_plant.h`) |
| `cvc/physics/dyn_boat_plant.h` | `dyn_boat_plant`, `dyn_boat_desc` (with `launch()`, `jonboat()`, `tactical()`): a world's boats as `cvc::nav`'s plant, through a driver, with an energy ledger |
| `cvc/physics/jolt_rollout.h` | `jolt_rollout`: the Jolt vehicle as `cvc::nav`'s training rollout (`rollout_kind::jolt`, `cvc/nav/rollout_plant.h`; NAV_TRAINING.md, "Training on Jolt") |
| `cvc/physics/mujoco_robot.h` | `mujoco_robot`: one articulated robot (the Unitree Go2 first) simulated by MuJoCo on a `cvc::height_field`, with servo and torque commands, pushes, contacts and MuJoCo's energies; `run_mujoco_trial`. Optional (`CVC_WITH_MUJOCO`); [MUJOCO.md](MUJOCO.md) |

The physics headers include only the standard library, except:

- `world_binding.h`, which needs `cvc::app`;
- `world_geometry.h`, which reads `cvc::world`;
- `nav_plant.h` and `dyn_boat_plant.h`, which implement `cvc::nav`'s plant interface;
- `jolt_rollout.h`, which implements `cvc::nav`'s training rollout interface.

No header names an engine. `cvc::nav` never includes `cvc::physics`; a configure-time check in
`src/cvc/CMakeLists.txt` enforces it.

## Units and frame

- **SI throughout:** metres, kilograms, seconds and newtons. Z is up and gravity points along −z
  (standard gravity by default).
- **Double-precision positions.** The world keeps a floating origin (`world_config::origin`,
  `world::rebase`) and hands the engine float offsets from it, so city-scale coordinates (UTM
  eastings, say) keep sub-millimetre resolution.
- **Converting to nav's frame.** `cvc::world_units` converts SI values to and from `cvc::nav`'s
  normalized frame (`to_normalized`, `nav::frame_of`).

## A world

```cpp
#include <cvc/physics/world.h>
using namespace cvc::physics;

world w;                                            // Jolt, 0.01 s steps, standard gravity
const material_id ice = w.add_material({"ice", 0.1f});

// The world's terrain: the same cvc::height_field the renderer draws.
auto ground = std::make_shared<cvc::height_field>(257, 257, x0, y0, 1.0, 1.0, heights);
ground->set_cell_materials(surfaces);               // one surface id per cell
w.add_body({terrain{ground, {0, ice}}, motion_type::static_});  // surface id -> world material

body_desc crate{box{{0.2, 0.2, 0.2}}};
crate.position = {x0 + 10.0, y0 + 10.0, 5.0};
const body_id c = w.add_body(crate);

for (int i = 0; i < 100; ++i) {
  w.step();                                         // 0.01 s
  for (const contact_event &e : w.contacts()) ...   // sorted, engine-thread independent
}
const body_state s = w.state(c);
```

### `world_config`

| Field | Default | Changeable later |
|---|---|---|
| `backend` | `jolt` | no |
| `gravity` | (0, 0, −9.80665) m/s² | yes |
| `fixed_dt` | 0.01 s | no |
| `collision_steps` | 2 | yes |
| `velocity_steps`, `position_steps` | 10, 2 | yes |
| `penetration_slop` | 0.002 m | yes |
| `threads` | 0 (no worker threads) | no |
| `field_threads` | 0 (the stepping thread reads the fields alone) | yes |
| `determinism` | `cross_platform` | no |
| `friction`, `restitution` (combine rules) | `product`, `max` | yes |
| `contacts` (reporting) | `added_removed` | yes |
| `origin` | (0, 0, 0) | via `rebase()` |
| `max_bodies`, `max_body_pairs`, `max_contact_constraints`, `temp_allocator_bytes` | 65536, 65536, 65536, 32 MiB | no |

A vehicle's drivetrain runs once per collision step, so energy-accurate vehicle runs want more
of them than the default: the energy ledger's drivetrain error is about 1% at 8 for the Humvee
and at 32 for the RoboRacer (see [the energy ledger](#the-energy-ledger-p3)).

`world::reconfigure(cfg)` applies the changeable fields. A different value for a fixed one throws
`cvc::physics_error` naming it and changes nothing.

### Materials

- **Material 0** is `default`: friction 1, restitution 0. Under the default `product` rule, a body
  on material 0 takes the other surface's friction exactly. That is how a vehicle (P2) sees the
  terrain's μ.
- **`add_material`** adds an entry to the table, and **`set_material`** changes an entry's values
  from the next step. Use it for rain on a road, say.
- **`nav_material`** records the `cvc::nav` material a surface carries, and **`rolling_resistance`**
  feeds the vehicle and the energy ledger.
- **`wet_friction`** is the friction soaked, what a wading tire finds under water (Vehicles,
  "Wading"); 0, the default, means not known, and the tire keeps `friction`. `add_world_geometry`
  sets it to the surface class's `mu_wet`, the value `apply_wetness` runs `friction` toward in
  the rain, so the rain and the water agree.

### Shapes

| Shape | Notes |
|---|---|
| `sphere`, `box`, `convex_hull` | Body-local; any motion type. |
| `mesh{geometry, materials}` | Static only (buildings). A `std::shared_ptr<const cvc::geometry>`: its triangles, and its quads split as `tri_surface` splits them. Each face's surface material (`face_materials`) is looked up in `materials`, and an empty table uses the surface id as the world material id. A geometry without face materials takes the body's material. |
| `terrain{field, materials, max_error}` | Static only. A `std::shared_ptr<const cvc::height_field>`, placed by its own grid rather than the body's pose. Rows run along +y. The engine collides with the field's own triangles, the diagonal split `height_field::height()` describes, so a body rests where `field->height(x, y)` says. Cell materials map like a mesh's, with at most 256 distinct world materials. The engine stores heights compressed and checks every sample against `max_error` (default 5 mm), refusing a field it cannot hold that closely. |

Meshes and terrain are the world's one representation (`docs/roadmap/WORLD-GEOMETRY-ROADMAP.md`):
the renderer and the navigator share the same objects, and physics never keeps a copy of its own.
The physics headers stay standard-library only by forward-declaring the two types; include
`cvc/geometry/height_field.h` to build them.

### Bodies and time

- **Body operations:** `add_body`, `remove_body`, `contains`, `state`, `set_state`, `add_force`
  (applied through the next step), `add_impulse` and `move_kinematic`.
- **Forces at a point and torques:** `add_force_at(id, force, point)` applies a world-frame force
  (N) at a world-frame point (m, world coordinates, not the floating origin's offsets), so off the
  centre of mass it also torques the body by `(point - com) x force`. `add_torque(id, torque)`
  applies a world-frame torque (N·m) about the centre of mass. Both act through the next step
  only and add up with `add_force` and each other, as `add_force` does: they wake a sleeping
  dynamic body, do nothing to a static or kinematic one, and throw `cvc::physics_error` for a
  value that is not finite or an unknown id. A force at the centre of mass gives the same state
  hash as `add_force`. `body_state::position` is the body's origin, not its centre of mass: the
  two coincide for a sphere or box, but a convex hull's centre of mass is its volume centroid
  and a vehicle chassis's is `vehicle_desc::center_of_mass`, so a force at such a body's
  position also torques it. The null backend checks the id and does nothing else. A boat's hull
  forces (roadmap `OCEAN-AND-WEATHER-ROADMAP.md`, Phase D) are applied this way.
- **Body settings:** `get_body(id)` and `set_body(id, s)` read and change a running body's
  `body_settings` (its material, mass, linear and angular damping, and whether it may sleep)
  from the next step on, and wake it.
  - A static body takes only a new material.
  - A mesh or terrain body keeps its materials: they are its triangles' or cells', and
    `set_material` changes those.
  - Only a dynamic body has a mass; `get_body` reports the engine's, computed from the density
    when the description gave none.
  - A vehicle's chassis is refused (`set_vehicle` tunes a vehicle).
  - Like materials, the settings are not part of `save()`.
- **Batch reads:** `states(ids)` returns several bodies' states and the tick under one lock, so a
  batch never mixes two steps. An unknown or removed id reads as empty instead of throwing.
- **Generation-checked ids.** Body ids carry a generation, so a stale id is refused and never
  aliases a newer body.
- **Stepping:** `step()` advances `fixed_dt`, and `step(n)` runs n steps under one lock.
- **Following a clock:** `follow(clock)` runs the steps due on a `cvc::world_clock` since the last
  call. `fixed_dt` must be a whole number of the clock's quanta.
- **Reports:** `last_report()` returns the last step's `step_report`. It carries wall time (and
  how much of it the fields' pushes, `fields_ms`, and the engine's own step, `engine_ms`, took),
  active bodies, the number of contact events, and `saturated` when an engine buffer filled.
- **Field threads:** a fluid with a free surface reads the surface's height and velocity at every
  cell of every body in it, every step: the field step's main cost. `field_threads` reads them on
  that many worker threads beside the stepping thread (and does the surface's per-time work, an
  ocean surface's transforms, as their tasks too). Each cell's reads land in its own slot and the
  sums run over them in the cells' order, so a run's values, and its `state_hash`, do not depend
  on it. A build without threads (single-threaded WebAssembly) ignores it. An Ariadne scene's
  world (`SceneWorld`) uses up to three.

### Determinism, save and restore

- **Hashing:** `state_hash()` digests the whole simulation state: bodies, velocities, sleep state,
  the contact cache, the tick and the origin.
- **Capabilities:** with `determinism::cross_platform`, the Jolt build must be cross-platform
  deterministic (it is, from 5.6.0+cvc.5); otherwise construction throws. 1 thread and N threads
  give the same hash (`physics_world_test`).
- **Across platforms:** the same calls give the same `state_hash()` on every target, wasm
  included, vehicles and fields too, not only the engine's bodies.
  - `physics_replay_test` replays a checked-in trace, `src/cvc/tests/data/physics_replay_trace.json`,
    and requires the hash after every step. The trace was recorded on Windows (MSVC, x86_64) and
    has 1240 steps in three scenes:
    - the golden scene;
    - a Humvee through the driver model, across a rebase;
    - a RoboRacer on raw pedals over two terrain materials, through mud that keeps ruts, past a
      crate floating in water.
  - It runs natively under ctest on every CI platform, and as wasm-mt under node in CI's
    `physics-wasm` job.
  - To build it for wasm without the test tree, configure with `CVC_BUILD_PHYSICS_REPLAY=ON`,
    then run `node physics_replay_test.js <trace>`.
  - After a change that is meant to move the physics, record the trace again with
    `physics_replay_test --record <trace>`. A platform that disagrees with the trace has found a
    determinism leak. Fix the leak; never record a trace per platform.
- **What keeps it so:**
  - Jolt's own cross-platform build.
  - No libm in anything that feeds the hash. libm's transcendentals are not correctly rounded,
    and MSVC's, glibc's, Apple's and musl's (emscripten) each miss in their own cases by an ULP.
    The vehicle step, the fields, Bekker sinkage and `nav_plant` therefore call `det::`, which is
    `cvc/algo/det_math.h` (shared with `cvc::ocean`, whose sea physics will read). It computes
    exp, expm1, log, pow, sin, cos, tan, atan, atan2, asin, acos, sinh, cosh, tanh, hypot and cbrt
    from correctly rounded operations, to within a few ULP; `algo_det_math_test` pins their bits.
  - No FMA contraction in those sources (`-ffp-contract=off`, `/fp:precise`).
  - Wheels whose contact fields start defined. Jolt leaves them uninitialized until a wheel first
    lands, but saves them every step, so before this every vehicle hashed garbage for its first
    steps, and two identical runs disagreed.
- **Hulls:** a body with a `hull` (`hull.h`) adds a section to `save()` holding the hull's digest
  and its triangles' last wetted areas (slamming reads them), so `state_hash()` covers the hull's
  description. A world without hulls saves and hashes exactly as before. `restore()` refuses a
  world whose hulls differ from the state's. `PhysicsHull.CrossPlatformGoldenHash` pins a hull
  riding a swell (`2f71a1fe229afa75`, measured on Windows MSVC; to be confirmed by CI on Linux and macOS).
  The digest has the hull's `lift_coefficient` only when it is not 0, so a hull without lift
  hashes as it did before the term.
- **Immersion:** `add_immersion()` (`immersion.h`) is plain IEEE arithmetic and sqrt, built without
  FMA contraction, so a body's columns are the same bits on every platform for the same pose and
  surface. `PhysicsImmersion.IsTheSameOnEveryPlatform` pins four bodies' columns on a swell
  (`b582404bf028ed63`, measured on Windows MSVC; to be confirmed by CI on Linux and macOS).
- **Boats:** a world with boats (`boat.h`) adds a section after the hulls' holding each boat's id,
  its hull's body id, its description's digest (propeller, rudder, time constant), its controls,
  its actuators and the last step's forces. A world without boats saves and hashes exactly as
  before. `restore()` refuses a world whose boats differ. `PhysicsBoat.CrossPlatformGoldenHash`
  pins a boat under way and turning on a swell (`bb8e1b22f58136b2`, measured on Windows MSVC; to
  be confirmed by CI on Linux and macOS).
- **Winds:** a flow field's `wind` (below) is hashed as a free surface is: its `digest()` goes into
  the section after the bodies with the field time, so `state_hash()` covers the wind, and
  `restore()` refuses a world whose winds differ from the state's. A world without winds or free
  surfaces saves and hashes exactly as before. `PhysicsField.TheSceneWindIsTheSameOnEveryPlatform`
  pins a `wind_flow`'s digest (`c7ec24178041550e`) and its velocities at points and times
  (`581744564ad471ef`), measured on Windows MSVC; to be confirmed by CI on Linux and macOS.
- **Restoring:** `save()` returns the state as bytes. `restore()` takes them on a world holding the
  same bodies, created in the same order, with the same materials. A restored world continues
  exactly as the original did.

### The wind: a flow field's `flow_velocity`

```cpp
#include <cvc/physics/field.h>          // flow_velocity.h with it

cvc::physics::field air;
air.kind = cvc::physics::field_kind::flow;
air.drag_coefficient = 1.0;
air.wind = std::make_shared<cvc::physics::wind_flow>(
    cvc::algo::wind_state{15.0, 1.5708, 0.25}, // m/s 10 m up, toward +y, gustiness
    0.0);                                      // the ground's or the water's level
const cvc::physics::field_id id = w.add_field(air);
w.set_field_time(t);                           // or follow(clock): the time the wind is read at
```

- **The hook.** `field::wind` (a flow only) is a `flow_velocity`: `velocity(p, t)` in m/s and a
  `digest()` of everything it depends on. It replaces the flow's velocity volumes. It must be
  deterministic (`det::` maths, no libm) and immutable once a field holds it; `set_field` with
  another changes the wind.
- **No volume.** A flow with a wind may leave `volume` empty: it is then air of 1.225 kg/m³
  everywhere (10⁷ m each way), the scene-wide wind. With a volume it blows in that box only, at
  the volume's density.
- **What it pushes.** Every dynamic body, with the flow's quadratic drag cell by cell,
  ½ ρ C_d a |u| u, u the wind minus the cell's velocity and a the cell's share of the body's area
  across u, so torques follow.
- **Read once per body.** A step costs O(bodies) calls into the hook, not one per cell. The hook
  says how: its velocity is `reference(p, t)` times `profile(p.z)`, and the flow fills only the
  space above `base()` (the defaults: `velocity()` itself, at every height, everywhere). The step
  reads `reference()` at the body's centre of mass at `world::field_time()` (the gusts vary over
  tens of metres) and `profile()` at nine heights from the base (or the body's bottom) to its top,
  closer together near the bottom, and a cell's wind is the reference times the profile
  interpolated at its height. A cell is clipped to the base as to a free surface, so a floating
  body feels the wind on its part above the water only. The air does not count in
  `field_load::immersed_volume`.
- **Vehicles.** A vehicle's chassis is not dragged by flow fields; its own aero drag reads the
  wind's `velocity()` at its centre of mass (`vehicle_desc::drag_coefficient`, `frontal_area`),
  so a car feels a crosswind as `aero_force` and `vehicle_state::air_velocity`.
- **`wind_flow`** is the scene's wind of `cvc/algo/wind_field.h`: the mean wind (`wind_state`:
  speed 10 m up, direction in radians from +x toward +y, gustiness) times the gust factor at
  (x, y, t), times `wind_profile(z − base_z)`, horizontal. The profile is the open surface layer's
  power law, (h / 10)^(1/7), linear below 0.1 m, held above 500 m, 0 at and below `base_z`: 0.52
  of the 10 m wind at 0.1 m, 0.72 at 1 m, 1.26 at 50 m. The water, the clouds and the trees read
  the same gusts (OCEAN-AND-WEATHER-ROADMAP.md E3, E5).
- **Measured** (`physics_field_test`, `physics_vehicle_test`):
  - a 1 kg ball 0.5 m in radius, 10 m up in a 10 m/s wind (C_d 0.5, no gravity), reaches
    8.285 m/s downwind in 2 s, against 8.279 m/s on the drag curve;
  - a floating crate, 1 × 1 × 0.5 m at density 500, in a 10 m/s wind drifts 0.154 m/s on the
    water's drag alone, against 0.156 m/s where the wind's drag on its top half balances the
    water's on its bottom half; with the water's damping at 1e4 N s/m⁴ in every direction,
    0.0013 m/s, and in an inertial fluid with that damping, 0.154 m/s (below);
  - a Humvee at 25 m/s in a 15 m/s crosswind is pushed 406 N aside (0.15 m/s²), the air at its
    centre of mass, 0.75 m up, 10.4 m/s;
  - the state hash moves with the wind, a world without one hashes as before, and `restore()`
    refuses another wind.

### Floating bodies: an inertial fluid (`field::inertial`)

A fluid's `damping` stands for the waves a floating body radiates, so a body stops bobbing. It
acts in every direction, though, so it also holds the body to the water's velocity sideways: the
body follows the waves and the current, but resists any drift through the water as hard as it
resists heaving. A 1 m crate in a 10 m/s wind on water damped at 1e4 N s/m⁴ drifts about a
millimetre a second.

```cpp
cvc::physics::field sea;
sea.kind = cvc::physics::field_kind::fluid;
sea.volume = water;          // kg/m^3 over the sea's box
sea.surface = ocean;         // the waves and the current (fluid_surface.h)
sea.damping = 1.0e4;         // now on the vertical only: the heave
sea.inertial = true;
```

- **Vertical.** The damping acts on the vertical alone, as before on the heave. It is also left out
  of the rate the horizontal drag is scaled by, as a hull's damping is.
- **Sideways.** The body is pushed by the water's acceleration over its immersed volume: the
  Froude–Krylov force ρ V du/dt, at the immersed part's centroid, where du/dt is the water's
  velocity a step either side (two `velocity()` reads per body per step). It drags against the
  water's velocity, the waves' orbital motion plus the current, with the fluid's C_d. A floating
  body weighs what it displaces, so the water's acceleration moves it as it moves the water: it
  rides the waves. Drag alone brings it to the current, and a wind blows it through the water at
  the speed where the air's drag on its freeboard balances the water's on its hull.
- **The default is off**, so a world that doesn't set it steps and hashes exactly as before. A
  hull (`hull.h`) is unaffected: its triangles take the water's pressure, which already moves it
  sideways. The Ariadne `water` node's field is inertial.
- **Measured** (`physics_field_test`):
  - the 1 × 1 × 0.5 m crate of the wind's measurements above, at damping 1e4: 0.0013 m/s
    downwind in a 10 m/s wind when damped in every direction; 0.154 m/s when inertial, against
    0.156 m/s from the drag balance (`AFloatingCrateDriftsDownwind`);
  - in a 0.5 m/s current it reaches 0.469 m/s in 60 s from rest, on drag alone; with a 10 m/s
    wind across the current it moves at (0.483, 0.155) m/s, the current plus its 0.156 m/s leeway
    (`AnInertialCrateDriftsWithTheCurrent`);
  - in a 0.2 m, 4 s swell, a slab heaves 1.04 times the surface (correlation 1.000) as a fully
    damped one does, its velocity along the swell is 0.99 of the water's orbital velocity
    (correlation 0.991), and it moves 0.01 m in 16 s: to and fro with the waves, not away
    (`PhysicsFluidSurface.AnInertialBodyRidesTheSwell`);
  - a world without `inertial` hashes as before: every pinned hash and the replay trace pass
    unchanged.

### Bodies pushed by each other's waves (`field::wake`)

A fluid with a free surface may keep a wake (OCEAN-AND-WEATHER-ROADMAP.md C5, D6): the waves its
floating bodies make, stepped in the physics step, pushing the other bodies. Off by default.

```cpp
sea.wake.cells = 64; // a 64 x 64 window (a power of two, 8 .. 128; 0, the default: no wake)
sea.wake.cell = 1.0; // of 1 m cells, deep water (wake.depth <= 0)
w.add_field(sea);
...
w.field_wake_info(id);              // where the window is, which bodies make waves, who is outside
w.field_wake_height(id, x, y);      // all their waves' height there
```

- **The window** sits on the world's grid of whole cells where it holds the most bodies in the
  water (a boat, a body with a hull, counting as four) and follows them by whole cells; bodies
  outside it make and feel no waves (`wake_info::outside`); placing it weighs at most 64 places a
  wave step (every boat, and one body per square of a coarse grid). Each awake body that makes
  waves (at most `wake.sources`, 8, each holding a slot; a body that sleeps or leaves gives its slot
  up, and a free slot goes to a boat first) has a wave state of its own there: its immersed
  columns over the window (`immersion.h`), cut against the surface and the others' waves, are its
  source, and the states step by `cvc::ocean::ewave`'s exact linear step every `wake.period`
  (1/30 s) in whole steps. A sponge takes the waves that leave; the window has no shores.
- **Each body reads the others' waves, never its own**: their height under its cells and hull,
  their orbital velocity for its drag and damping, and their acceleration for an inertial fluid's
  push. Its own, read back, would undo its heave's spring as it moves and count its wave-making
  twice. A body alone floats exactly as it does without a wake; a sleeping body makes none, and one
  the others' waves reach is woken. While every body sleeps and no wave is left, nothing is read.
- **Limits and memory:** cells 8 .. 128, cell 0.05 .. 100 m, period (0, 10] s, sources 1 .. 16,
  samples 1 .. 8. A state is 80 cells² bytes (320 KiB at 64²), at most sources + 4 of them, and a
  save carries 32 cells² bytes of each.
- **World state:** the window and the states are saved with every wake parameter (another wake's
  state is refused) and hashed; restore() reads the wakes' section whole before any wake takes it
  (a failure in it leaves the wakes as they were; the world's restore as a whole is not
  transactional). `set_field` with the same wake keeps the waves. `water_at` reads the water
  without them.
- **Measured** (`physics_field_test`, `PhysicsFieldWake.*`): a crate afloat 5 m from one dropped
  in heaves 0.038 m, drifts 0.082 m and tilts 0.035 rad with a wake, and does not move without one;
  on still water the push costs 0.14 ms a step with two sources against 0.005 ms without, and on a
  three-cascade sea with 3 field workers it is within the runs' spread (8.30 against 8.04 ms); a
  world without a wake hashes as before, and one with a wake is pinned
  (`CrossPlatformGoldenHash`).

### Threads

The world is thread-safe. Every call locks it, and `step()` holds the lock for the whole engine
step, so a call from another thread waits at most one step. Accessors return copies. To read a
running world from many threads without waiting, read the telemetry a `world_binding` publishes.

### The whole world at once: `snapshot()`

`world::snapshot()` copies everything at one tick under one lock into a `world_snapshot`:

- the model: the config, the materials, every field as given (with its box, rut count and
  deepest rut), and every vehicle's description as tuned;
- the state: every body (id, motion type, settings, user data, pose, velocities, awake, and
  what the fields did to it), every vehicle's `vehicle_state` with its wheels, and the last
  step's contacts.

Every list is in ascending id order.

- **Moving and static bodies.** `bodies` holds the dynamic and kinematic bodies, read in one
  batch. `static_bodies` holds the static ones in a list that successive snapshots share until
  a static body is added, removed, moved or given a material; `static_version` changes then.
  So a city of static buildings costs a snapshot nothing.
- **A partial snapshot.** `snapshot(bodies, vehicles, fields)` reads just those, under one lock,
  without the contacts. This is what a `world_binding` reads for its named actors when no whole
  snapshot is due.
- **Cost.** With 1000 moving bodies on Windows (MSVC), a snapshot took about 0.18 ms against a
  2.1 ms step on an idle machine (`physics_world_test`: `WhatPublishingCostsAStep`).
- **JSON.** `to_json(snapshot)` (`cvc/physics/snapshot_json.h`) writes it as JSON, format
  `cvc.physics.snapshot/1`, for readers outside C++.
  - A vector is `[x, y, z]` and a rotation `[w, x, y, z]`, and enums are their names.
  - A double is written in its shortest exact form (`std::to_chars`), always with a `.` or an
    exponent, so it reads back as the same double. NaN and infinities are written as `null`.
  - `json_bodies::moving` and `json_bodies::static_only` split the moving and the static
    bodies, as the binding publishes them.
  - It costs about 4 µs a body (1000 bodies are about 560 KB).

A `world_binding` publishes snapshots to the state tree as it steps (below), so any thread,
any Python script and any remote peer can read the latest without taking the world's lock.

Jolt's process globals (allocator hooks, factory, type registry) exist while at least one Jolt
world does. The first world sets them up and the last one tears them down. If the application set
up Jolt itself, that setup is used and left in place. libcvc keeps no singleton of its own.

## Vehicles

```cpp
#include <cvc/physics/world.h>

cvc::physics::world w;                                   // Jolt (cap_vehicles)
const auto car = w.add_vehicle(cvc::physics::vehicle_desc::humvee(), {0, 0, 1.2});
w.drive(car, {0.2, 1.5});        // the navigator's command: steer 0.2 rad left, a_cmd 1.5 m/s^2
w.step();
const auto s = w.vehicle_state(car); // speed, steer, pedals, gear, rpm, aero, every wheel
```

A vehicle is a chassis body (a box with its centre of mass offset) on wheels that Jolt's vehicle
model rolls over the world's bodies. The chassis frame is x forward, y left, z up.

- **Wheels.** Each axle is a left and right wheel (wheel 2a is the left one of axle a).
  - Each wheel hangs on its own suspension: a spring-damper given by frequency and damping
    ratio, with travel from `suspension_min` to `suspension_max` and a hard stop.
  - An optional anti-roll bar couples an axle's two wheels.
  - `wheel_state` reads back each wheel's steer, spin, suspension length, contact, load, slip,
    the μ it used, and what the medium did to it.
- **Steering.** `drive` and `set_controls` name the bicycle angle at the centre of the front
  axle.
  - An actuator slews it toward the command at `max_steer_rate`, clamped to `max_steer`.
  - Each steered wheel takes its Ackermann angle about the rear axle:
    `cot(δ_out) − cot(δ_in) = track / wheelbase`.
- **The driver model.** `drive` turns `a_cmd` into throttle or brake, never both: a
  feed-forward on `max_accel` / `max_decel`, corrected by the measured acceleration.
  - The correction is proportional, plus an integral trim (2/s). The trim learns what the
    feed-forward gets wrong: a converter's launch, a grade, a load.
  - `max_speed` (m/s, 0 for none) is the top speed it holds. The wanted acceleration eases off
    to close the gap to it in 0.5 s. The presets carry their rated speeds: 24.6 m/s (55 mph) for
    the Humvee and 10 m/s for the RoboRacer. Raw pedals are not held.
  - The trim restarts when the command turns from speeding up to slowing down, and holds
    while its pedal is saturated.
  - The transmission shifts on its own; `reverse` backs up.
  - A vehicle moving the wrong way brakes first.
  - One with no command (|a_cmd| at most 0.05) and slower than 0.3 m/s holds on the full
    brake, so it stands on a grade its tires can hold; released (`neutral`, no brake) it
    rolls down at g (sin θ − C_rr cos θ). A change of input wakes a sleeping vehicle.
  - `set_controls` takes pedals directly (and `neutral`).
- **The powertrain.** The engine (motor) drives the wheels through Jolt's clutch, an automatic
  transmission and the axles' differentials.
  - `torque_curve` is the engine's net torque at the flywheel against its speed, as a
    manufacturer publishes it.
  - `engine_friction_curve` is its motoring friction against its speed. It brakes the
    vehicle with the throttle shut; under throttle the torque curve is already net of it.
  - `clutch_strength` couples engine and gearbox (N m per rad/s of slip).
  - With `converter_k` > 0, that coupling is a torque converter's:
    - it multiplies the engine's torque, from `converter_stall` with the gearbox input still
      to 1 at the speed ratio `converter_coupling`. The gear ratios are scaled by that torque
      ratio, so Jolt's clutch slips against it and books the converter's loss, and no
      energy is made;
    - it carries at most (rpm / `converter_k`)² of torque, so an idling engine creeps the
      vehicle as a converter does. Without it, Jolt's idle-speed clamp drives the wheels with
      the whole clutch, more than half the brakes hold.
  - `driveline_efficiency` is the share of the engine's torque the gears deliver.
  - The automatic shifts at `shift_up_rpm` / `shift_down_rpm` at full throttle and at the
    `_light` points with it shut, between them with the throttle. It shifts on its input's
    speed, not the engine's that a slipping converter lifts.
- **Tires.** Slip curves peak at 1.0 times the μ of what the tire touches: the material of the
  terrain cell or mesh face under it, the table every plant reads.
- **Resistance**, applied to the chassis each step from the last step's wheels:
  - aero, −½ ρ C_d A |u| u, with u relative to a flow field's wind where one covers the chassis
    (a vehicle's chassis is not dragged by flow fields otherwise), read at its centre of mass at
    the field time, a `flow_velocity`'s where the flow has one (the wind, above);
  - rolling, C_rr times each wheel's load, with C_rr from the ground material's
    `rolling_resistance` (the vehicle's own when the material has none).
- **Snow, mud and sand.** In a medium field (`field.h`), a wheel sinks to the rigid-wheel Bekker
  depth for its load: z = (3N / (2 b k √D))^(2/3), at most the medium's depth.
  - It rides on the compacted surface, depth − z above the ground; a vehicle landing on deep
    snow lands on the snow.
  - It is held back by the compaction resistance b k z² / 2.
  - Its tires get the medium's `traction`, if it has one.
- **Ruts.** A medium with `ruts = true` keeps a rut map, `rut_cell` metres a cell (default
  0.05; at most 16M cells over the field's bounds).
  - Each wheel presses its contact patch (2√(2rz − z²) long, the tire's width across) to its
    sinkage: the half behind the axle as it rolls, all of it standing still.
  - A wheel sinks at least as deep as the rut it finds, and only snow below the rut ahead of
    it resists: b k (z² − rut²) / 2. A second vehicle in the tracks, or a rear wheel in its
    front wheel's, sinks no further and rolls almost free.
  - `world::field_ruts(id)` copies the map; `wheel_state::rut` is the rut a wheel found.
  - The maps are state: `save()` carries them and `state_hash()` covers them.
- **Water.** With `vehicle_desc::wading` on, the vehicle wades: its body floods, the water drags
  and floats its tires and underbody, soaks and aquaplanes its tires, and floods its engine past
  its fording depth (below, "Wading"). Off, its chassis is a sealed box that floats.
- **Save and restore** carry the actuators and the driver's memory, so a replay reproduces a
  drive (`physics_vehicle_test`).
- **Tuning a running vehicle.** `get_vehicle(id)` returns its description and `set_vehicle(id, d)`
  changes it from the next step on. These values may change:
  - the steering: `max_steer`, `max_steer_rate`;
  - the driver: `max_accel`, `max_decel`, `max_speed`;
  - the resistance: `drag_coefficient`, `frontal_area`, `air_density`, `rolling_resistance`;
  - the powertrain: `engine_max_torque`, `engine_min_rpm`, `engine_max_rpm`, `shift_up_rpm`,
    `shift_down_rpm`, `shift_up_rpm_light`, `shift_down_rpm_light`, `engine_friction_curve`,
    `clutch_strength`, `driveline_efficiency`;
  - each axle: `max_brake_torque`, `max_hand_brake_torque`, `suspension_frequency`,
    `suspension_damping`, and `anti_roll_stiffness` on an axle that was added with a bar;
  - wading, all but `wading.enabled` and `wading.displacement` (fitting the deep-water kit is
    `fording_depth` 1.52; it does not unflood an engine).

  A change to anything else (the chassis, the mass, the wheels' geometry, the gearing, the
  torque and tire curves, the energy source, whether it wades and its body's displacement)
  throws `cvc::physics_error` naming it and changes nothing. Like materials, a tuning is not
  part of `save()`.

  Through the tree an invalid tuning value is ignored on its own; the other knobs still apply.

### Wading: `vehicle_desc::wading`

```cpp
cvc::physics::vehicle_desc d = cvc::physics::vehicle_desc::humvee(); // wades: 0.76 m
d.wading.fording_depth = 1.52;      // with the deep-water fording kit
const auto car = w.add_vehicle(d, {0, 0, 1.2});
w.step();
const auto s = w.vehicle_state(car);
s.wheels[0].water_depth;            // m of water over its contact
s.water_force;                      // N, the water on the tires and underbody
s.engine_flooded;                   // past the fording depth, for good
```

Off (`wading.enabled` false, the default for a description you build), a vehicle's chassis
meets a fluid field as a sealed box, buoyant over all of its volume: a Humvee floats off its
wheels like a raft in 0.8 m of water, and nothing below the box feels the water. The presets
wade. On, from the water at each wheel and under the body (`world::water_at`, at the field time,
so a free surface's waves and a current count):

- **The body floods.** Only `displacement` of the chassis box's volume displaces water (the
  field step scales its cells' buoyancy): what is left of a soft-doored body is its solid and
  sealed parts. A heavy vehicle stays on its wheels as it wades and sits on the bottom in water
  over its roof, rather than floating.
- **The tires and underbody are dragged**, 1/2 ρ C_d A |u| u, u the water's velocity (a current,
  the waves' orbital motion) less the part's:
  - a tire along its travel: its width times its immersed height, with `wheel_drag` (AMC 25.1591's
    displacement drag, C_d 0.75 for an isolated tire below aquaplaning speed); across it, its
    immersed side (a segment of its disc). That is the tire's extra rolling resistance in water,
    and it is what pushes a vehicle sideways in a current;
  - the underbody (axles, frame, skid plates), a slab `underbody_depth` deep under the box, with
    `underbody_drag` on the box's width or length times its immersed height.

  The drag is integrated implicitly in its own rate, as the fields' is. A vehicle standing in
  flowing water does not fall asleep, since the water keeps pushing it.
- **The tires float a little.** Each displaces `wheel_displacement` of its immersed cylinder (the
  tire; the rim's dish floods), which takes that much load off it.
- **The grip.** A tire in water is on soaked ground: its μ runs from the material's `friction` to
  its `wet_friction` as the water over its contact deepens to 3 mm (AMC 25.1591's standing water).
  On ground the rain has already soaked (`apply_wetness`), `friction` is `wet_friction` already,
  so it is no slicker under water. In standing water a tire with a `tire_pressure` aquaplanes:
  its μ falls as 1 − (v / v_p)² to 0.05 at v_p = 0.85 × 9 √p knots (p in psi; AMC 25.1591,
  7.1.1), v its speed over the ground; the shape between the ends is ours. A tire in a medium
  (mud, snow) keeps the medium's traction. `wheel_state::friction` reads the μ it used.
- **The engine floods.** Once the water over the ground at the front axle (the mean of its
  wheels) is deeper than `fording_depth`, the engine takes water and stops for good: no throttle,
  no idle (Jolt's idle speed drops to 0 and its friction runs it down), no fuel, and the
  automatic, without line pressure, leaves the wheels free as in neutral. The brakes still work.
  It is state: `save()` carries it in a spare bit of a byte it already wrote, so a vehicle whose
  engine has not flooded saves the bytes it always has.
- **The ledger.** The water's work on the tires and underbody is the ledger's `fields`, with the
  box's buoyancy and drag.
- **Readbacks.** `wheel_state::water_depth` and `water_drag`, `vehicle_state::water_force` (the
  tires' and underbody's, not the box's, which is `world::snapshot`'s field load) and
  `engine_flooded`; the snapshot's JSON has them, and the description's `wading`.

A dry vehicle that wades steps exactly as one that does not: nothing touches its loads, its
grip or its saved bytes until a part of it is in water.

What `physics_vehicle_test` measures (Windows MSVC):

| Check | Model | The twin |
|---|---|---|
| A Humvee standing in 0.5 m: what its tires float | 3738 N | 3733 N (26222 N dry, 22489 N wading) |
| Coasting at 3.2 m/s in 0.5 m: a tire's drag | 760 N | 757 N |
| Coasting at 3.2 m/s in 0.5 m: its deceleration (2829 N of water on it) | 1.143 m/s² | 1.137 m/s² |
| Pulling away for 3 s at full throttle in 0.5 m | 5.44 m/s: its traction on land at each speed, less the water's drag | 5.67 m/s (8.94 m/s on land) |
| Standing in 1.2 m (past 0.76 m) | planted, flooded | tires carry 12518 of 26243 N; chassis 1.103 m up (1.030 m dry); engine at 0 rpm |
| The same with wading off | floats | chassis 1.337 m up, tires carry 0 N |
| Driving at 3 m/s from 0.5 m into 0.9 m | floods as the front axle crosses | flooded at 40.06 m, stopped at 45.06 m; with the kit it drives on |
| A 1.5 m/s current along it, at rest in 0.6 m | 833 N | 833 N; in neutral it drifts downstream at 0.44 m/s after 5 s |
| A 1.5 m/s current across it | 2001 N | 1994 N; its tires hold it (0.002 m/s) |
| A 0.15 m, 5 s swell over 0.6 m (orbital speed up to 0.59 m/s) | 131 N on a held vehicle | the push swings −41 to +79 N; it rocks 2.7 cm |
| μ on asphalt (0.9 dry, 0.55 soaked) in 2 cm at 2 m/s, 20 psi tires | 0.543 | 0.543 |
| in 1 mm (a third soaked) | 0.780 | 0.780 |
| in 2 cm at 14 m/s (aquaplaning, v_p 17.6 m/s) | 0.202 | 0.202 |

### The presets

`vehicle_desc::humvee()` is an M1097A2 HMMWV and `vehicle_desc::roboracer()` the UT-AV lab's
RoboRacer. In `vehicle.cpp` every value names its source:

- [AMG], AM General's M1097A2 specification sheet;
- [TM], TM 9-2320-280-10;
- [DTIC], the Army's dynamometer tests of the 6.5 L engine (TARDEC TFLRF, DTIC ADA449196);
- [lab], the lab's configuration (ut-av.pages.dev, ut_automata's `vesc.lua`);
- [trx], Traxxas's listing for the chassis;
- [doc31], the scope's spec table;
- [AMC], EASA's AMC 25.1591, the standard method for a tire in standing water;
- [derived], a value that follows from those;
- [est], an estimate to replace with a measurement.

The two presets:

- **The M1097A2.**
  - The vehicle: 2676 kg at curb (4672 kg GVW), a 3.30 m wheelbase, a 1.82 m track and
    37 x 12.5R16.5 tires.
  - The engine: the naturally aspirated 6.5 L V8 diesel, 119 kW at 3400 rpm and 393 N m at
    1700 rpm.
  - The driveline: the four-speed automatic (2.48, 1.48, 1.0, 0.75), 2.73 axles and 1.92
    geared hubs, in full-time four-wheel drive.
  - The brakes are proportioned to the axle loads at 0.8 g. The tank holds 95 L.
  - **Friction and fuel, from the dynamometer modes [DTIC].** At rated speed the fuel flow
    is a Willans line: 0.331 lb/hp-h over 20.3 lb/h at no load. That is an indicated
    efficiency of 0.42 and 128 N m of friction. The intermediate-speed and idle modes leave
    87 and 78 N m. The table gives power and flow, not speed, so the modes' speeds (3400,
    2040 and 700 rpm) are assumed.
  - **Estimates:**
    - the driveline's 82% (gearbox, transfer case, axles, hubs);
    - the 4L80-E's converter: stalling near 1900 rpm against the peak torque (K = 95.8, 53 N m
      of creep at idle), 2:1 at stall, coupling at 0.85;
    - 2.5 kW of accessories;
    - C_d A of 2.45 m^2;
    - C_rr of 0.015.
  - **Wading.** It fords 0.76 m (30 in) of hard-bottomed water, 1.52 m (60 in) with the
    deep-water fording kit, which moves the intake, exhaust and tank vents up [AMG]. The tires
    take [AMC]'s C_d of 0.75; they displace 0.8 of their cylinder (the 37 in tire round the
    16.5 in rim) [derived]. The underbody reaches 0.16 m under the box: the box's bottom rides
    0.57 m up, the ground clearance is 0.41 m [AMG]. Estimates: the body displaces 0.12 of the
    box (1.1 of its 9.5 m^3: the solid of the body, frame and drivetrain, the sealed tank and
    frame rails), the underbody's C_d of 0.5, and Goodyear's 38 psi cross-country pressure for
    the tire (aquaplaning from 24 m/s, about the top speed).
- **The RoboRacer.**
  - The car: 3.5 kg, a 0.324 m wheelbase, 108 mm tires and shaft-driven 4WD.
  - The motor: an ARRMA 3800 Kv motor at the VESC's current limit (an estimated 60 A, so
    0.15 N m), flat to the VESC's 10 m/s speed limit, geared 15.15:1 to the wheels
    ([derived] from `speed_to_erpm_gain`).
  - The battery: a 3S LiPo. Its 55.5 Wh is an estimate: read the pack.
  - Braking is the VESC's, an estimated 0.57 N m a wheel, 70% of it regenerated up to 150 W.
  - Wading, all estimates: water 3 cm deep reaches the VESC and motor; the pack and electronics
    keep 0.2 of the box dry; the foam-insert tires displace 0.3 of their cylinder and, not
    being pneumatic, never aquaplane; the chassis plate hangs 1 cm under the box.

Against the field (`physics_energy_test`, `physics_vehicle_test`, Windows MSVC, the default 2
collision steps):

| Check | Published | The twin |
|---|---|---|
| M1097A2 at GVW, full throttle, 0-30 mph | 9.4 s, the Army's requirement [AMG] | 8.9 s |
| M1097A2 at GVW, 0-50 mph | 26.1 s [AMG] | 23.6 s |
| Idle fuel | 2.6 lb/h (1.4 L/h) on the dynamometer, without the vehicle's accessories [DTIC] | 1.95 L/h, with 2.5 kW of them |
| Range at GVW, 30-40 mph, hard surface, hilly | 442 km on bias-ply tires, about 490 km on radials [TM] | At 35 mph on radials: 596 km level, 600 km over 3 m hills 400 m apart (15.8-15.9 L/100 km) |

The acceleration meets the requirement. The converter's torque multiplication gives the launch.

The range does not match yet. The twin goes about 22% further on a tank than the TM says, and
its losses so far are only the documented ones. Still to model or measure:

- an unlocked converter's slip at cruise;
- the cross-country tread's rolling resistance on a real road;
- steeper hills and the braking they take;
- the fan's duty.

A coast-down test gives C_rr and C_d A, and a logged fuel flow gives the rest. DTIC ADA259919
measured an M998's steady-speed fuel consumption on a level road and is the next published
check. The range test pins the present calibration until then.

### The energy ledger (P3)

`vehicle_state::energy` is a `vehicle_energy`: the vehicle's energy since it was added, in
joules, term by term. The vehicle is the chassis, its wheels and its drivetrain, engine
included (doc 31's D8). Each engine sub-step the ledger samples the vehicle and charges every
flow to its cause:

```
supplied + idle + fields + drivetrain_split
  = brake + tire + aero + rolling + medium + drivetrain + suspension + chassis_damping
    + stored + residual
```

| Term | What it is | How it is measured |
|---|---|---|
| `supplied` | The engine's (motor's) shaft work, the integral of T w | Exactly, from Jolt's engine torque and the speed its implicit drivetrain step leaves |
| `idle` | What Jolt's idle-speed clamp adds to keep the engine at `engine_min_rpm` (< 0 when the rev limit holds it down) | Exactly |
| `brake` | The brakes: a wheel's spin they take, and a locked wheel's sliding | Exactly |
| `tire` | Tire slip, longitudinal and lateral | Exactly, impulse by impulse in the friction solve; the lateral warm start from the impulse and the velocities it left |
| `aero`, `rolling`, `medium` | The facade's drag, rolling resistance and compaction forces | Their work, at the mean of the sub-step's velocities |
| `drivetrain` | Engine and wheel damping, clutch slip, engine braking, and the drivetrain step's own damping | Exactly, from Jolt's implicit engine / clutch / wheel step |
| `suspension` | The dampers | c v^2 on each wheel's compression rate, c as Jolt sets it |
| `chassis_damping` | The chassis body's own damping | As Jolt applies it |
| `fields` | Work the fields did on the chassis (buoyancy, a current), and a wading vehicle's water on its tires and underbody (< 0 as it drags) | At the mean velocity |
| `stored` | The change in kinetic (chassis, wheels, engine), potential and spring energy | From the samples; `kinetic`, `potential`, `spring` are the values now |
| `drivetrain_split` | Energy Jolt's drivetrain step makes. It solves engine, clutch and wheels with its estimate of the brake and ground torques, then undoes the estimate before the friction solve, which leaves each driven wheel dL tau / I more (dL the clutch's angular impulse, tau the estimate's) | Exactly |
| `residual` | Everything else the simulation does not conserve | The balance |

So `drivetrain_split` and `residual` together are the simulation's own energy error: the first,
the part it is known to make, itemized; the second, the rest.

- The split stays its own term, not part of the residual. It is the drivetrain step's known
  error, and it is proportional to the collision step: it halves when the step does.
- The residual is small at any step.
- So the gate is on their sum, at the step a twin runs at.

- **Measured, not modelled.** With a ledger, the backend's Jolt controller runs copies of Jolt
  5.6.0's `PostCollide` and friction solve with measurements between their steps. A measured
  run is bit for bit an unmeasured one (`vehicle_desc::energy_ledger = false`;
  `physics_energy_test`): measuring changes none of the P2 measurements.
- **The source.** `vehicle_desc::energy` (`energy_source`) is bookkeeping on the shaft work,
  and an empty battery or tank gives no more throttle:
  - a battery draws `(supplied + idle) / motor_efficiency`, and braking returns
    `regen_efficiency` of the braking energy, at most `max_regen_power`, while there is room.
    That never includes what the drivetrain step made in a braked wheel. `charge` is the state
    of charge;
  - fuel burns on a Willans line: `(supplied + friction + accessory_power) /
    (indicated_efficiency x heating_value)` while the engine is fuelled, which is under
    throttle or at idle. The friction is `engine_friction_curve`'s torque times the engine's
    speed. On the overrun (throttle shut, above idle) a diesel cuts its fuel and burns none.
    `fuel_used` is the total, and `fuel_left` what is left of a `tank`;
  - `source` is the electrical or chemical energy drawn, and `wh_per_km()` is that per
    kilometre of `distance`. `power` is the last step's supplied power.
- **Presets.** The Humvee burns diesel on the 6.5 L's dynamometer Willans line from a 95 L
  (80 kg) tank, about 1.95 L/h at idle. The RoboRacer has a 55.5 Wh battery, with 85% from
  pack to shaft and 70% regeneration up to 150 W (estimates until measured). See
  [the presets](#the-presets).
- **State.** The ledger is in `save()`, so a replay reproduces it, and in the snapshot, its
  JSON (`state.energy`) and the published keys (`vehicles.<name>.energy.*`).

Gates (`physics_energy_test`, Windows MSVC):

| Gate | Measured |
|---|---|
| A closed cycle over five 1 m hills: accelerate to 10 m/s, hold it (throttle up, brake down), stop and stand. \|residual\| <= 2% of supplied at any collision step, \|residual + drivetrain_split\| <= 2% at 8 | At 8: 640 kJ supplied; residual -64 J (0.01%), drivetrain_split +4.5 kJ (0.70%). The split is 3.9% at 2 steps and 1.4% at 4 |
| Coasting with no input from 15 m/s: supplied = 0, and dKE + dPE = -dissipated within 2% | supplied 0; 211 kJ stored lost against 211 kJ dissipated, residual +52 J |
| Coasting down a 6 degree ramp in neutral: the kinetic energy gained = -dPE - dissipated within 2% | +62.4 kJ kinetic, -73.9 kJ potential; residual +46 J |
| Regeneration never credits more than the brakes take | RoboRacer, three 5 m/s stops: 79.4 J returned of 117.9 J of braking (67%); at a 2 W cap, 2.9 J |
| The RoboRacer's split shrinks with the step: \|residual + drivetrain_split\| <= 2% at 32 | The split is 20.8%, 11.5%, 5.2%, 2.2% and 1.1% of supplied at 2, 4, 8, 16 and 32 steps; the residual is at most 0.06% |
| Fuel follows the Willans line | Idle in neutral: 1.653 kg/h, the line's 1.653 kg/h; none on the overrun |

What the ledger showed about the first presets, and what became of it:

- **The Humvee's drivetrain lost about half the engine's work** to Jolt's defaults: engine
  damping of 0.2/s at every throttle, and a clutch of strength 10 slipping 51 rad/s at full
  torque.
  - Now the engine's friction is the dynamometer's, and it acts only with the throttle shut.
  - The clutch is 40, a converter's, and the gears deliver 82%.
  - At a steady 35 mph the drivetrain takes 20% of the engine's work. On the hilly cycle it
    takes about half, because it also holds the converter's slip at the launches and the
    engine braking down the hills.
- **The RoboRacer's split error exceeded its supplied energy while braking.** Its motor
  inertia was 1e-4 kg m^2, behind 12:1 gearing.
  - With the lab's gearing (15.15:1) and an estimated 2e-6 kg m^2 rotor, the split is 21% at
    2 collision steps and halves with each halving of the step.
  - Energy-accurate runs of the RoboRacer want 32 collision steps (1.1%), the Humvee's 8
    (1.0%).
  - Most of the RoboRacer's split comes from braking its wheels while the motor stays
    coupled. The real car brakes through its motor, so modelling the VESC's braking as
    motor torque would remove most of it.
- **The converter's creep** (found by P4's plant):
  - Jolt holds an idling engine at its idle speed, and with a stiff clutch that clamp drove
    the wheels with about 2.9 kN m of creep.
  - So a Humvee at a few m/s, throttle shut and brakes at half, sped up at 3.4 m/s^2.
  - The converter's capacity now limits the creep to about 53 N m at the engine (doubled at
    stall), and its torque multiplication gives back the launch the clamp had faked.

## Boats

```cpp
#include <cvc/physics/world.h>

cvc::physics::boat_desc d;
d.hull.shape = cvc::physics::box{{2.0, 0.75, 0.3}};    // what it collides with
d.hull.hull = cvc::physics::box_hull({2.0, 0.75, 0.3}); // what the water pushes on (hull.h)
d.hull.mass = 900.0;
d.hull.linear_damping = d.hull.angular_damping = 0.0;   // the water is its resistance
d.propeller.position = {-2.1, 0.0, -0.35};               // body frame: x forward, y left, z up
d.rudder.position = {-2.3, 0.0, -0.35};
d.rudder.area = 0.12;
const auto b = w.add_boat(d, {0, 0, 0.15});
w.set_boat_controls(b, {1.0, 0.45});  // throttle, rudder in [-1, 1]; + rudder turns left
w.step(100);
const auto s = w.boat_state(b);       // speed through the water, heading, thrust, rudder force
```

A boat is a dynamic hull body (`boat_state::hull`) that the fluid fields float and resist like any
body, plus a propeller, a rudder and two actuators that the world applies **every step**, before
the engine step, in ascending boat id order. So `world::follow`, which may take several steps a
call, drives a boat exactly as `step()` does.

| Call | What |
|---|---|
| `add_boat(desc, position, rotation = {})` | Adds the hull from `desc.hull` at that pose (its own pose ignored); throws `cvc::physics_error` for a hull that is not dynamic, `add_body`'s refusals, or propeller, rudder and actuator values that are not finite or out of range. Any backend |
| `remove_boat(id)` | Removes the boat and its hull; `remove_body` refuses a boat's hull |
| `boat_count()` | |
| `set_boat_controls(id, {throttle, rudder})` | What the actuators move toward, from the next step; each in [-1, 1], else `cvc::physics_error` |
| `boat_state(id)` | The hull's state; `speed` and `sway` through the water (the water's velocity at the centre of mass taken off); `heading`, `yaw_rate`; the command and the actuators; and the last step's `propeller_wet`, `inflow`, `thrust`, `thrust_force`, `outboard_angle`, `rudder_wet`, `rudder_angle`, `angle_of_attack`, `rudder_lift`, `rudder_drag`, `rudder_force` |
| `get_boat(id)` | The description as added |

Each step:

- **Actuators.** The throttle and rudder move toward the controls as a first-order lag, a += (c − a)(1 − e^(−Δt/τ)), τ = `actuator_time_constant` (0: at once).
- **Propeller.** While its position is under a fluid field's free surface (the surface's height there, or the box's top), it pushes along its direction at its position with T = |throttle|·T_max·(1 − u_a/`max_inflow_speed`), clamped to [0, |throttle|·T_max]. T_max is `max_thrust` ahead and `max_reverse_thrust` astern, and u_a is the propeller's velocity through the water along its push. An outboard (`outboard_max_angle` > 0) swings that direction about the body's z axis by the rudder actuator times the angle, pushing the stern right to turn left.
- **Rudder.** While wet, the water flowing past it (the water's velocity there less the rudder's point velocity v + ω × (p − com)) meets the blade, deflected by the rudder actuator times `max_angle`, at an angle of attack α in the body's horizontal plane. It makes lift ½ρAV²·C_L(α) normal to the flow and drag ½ρAV²·(`drag_coefficient` + `cross_flow_drag`·sin²α) along it. C_L = `lift_slope`·α up to `stall_angle`; past it the blade is a flat plate, C_L = `cross_flow_drag`·sin 2α / 2. Water from astern meets the trailing edge first, and α is taken from that edge, so a rudder works reversed going astern.
- Both forces go through the backend's `add_force_at` at their points, so they torque the hull about its centre of mass.

`world::water_at(point)` is the water there as the propeller and rudder read it, at `field_time()`:
the first fluid field, in id order, whose box holds the point in x and y and reaches below it. It
gives a `water_sample`: `wet`, `depth` below the top (negative above it), `density` and the water's
`velocity`. Where no fluid field covers the point it is dry, still and of density 0.

Not modelled: the propeller's slipstream over the rudder (at low speed an outboard steers and a rudder barely does), propeller torque and walk, ventilation of a propeller near the surface (it is wet or dry at its centre), and a partly immersed rudder. `snapshot()`, its JSON and `physics_codec` do not carry boats yet (pycvc binds them: "Python: pycvc"). `physics_boat_test` checks the balance at terminal speed, backing, the turning circle against the rudder angle, a dry propeller, an outboard at low speed, pitching on a swell, and the state (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md, "As built (O11b: boats)").

## cvc::nav's plant: `nav_plant` (P4)

```cpp
#include <cvc/nav/sim_world.h>
#include <cvc/physics/nav_plant.h>

cvc::physics::world w(cfg);          // the ground, the buildings: the same map as nav's
cvc::physics::nav_plant plant(w, {cvc::physics::vehicle_desc::humvee()});
sim.set_plant(&plant);               // sim: a cvc::nav::sim_world; its agents now drive Humvees
for (;;)
  sim.step();                        // nav's command, six physics steps, the readback
```

A `sim_world` integrates its agents with its fused kinematic bicycle unless it has a plant.
Without one (the default) nothing changes, bit for bit.

With a plant, every `sim_world::step()` does this:

- It reads each agent back at the tick's start. The adapter reads three things: the rear
  axle's position, the body's heading, and the signed forward speed at the rear axle. The rear
  axle is the point nav's bicycle tracks, and it has the least sideslip. The speed is the
  centre of mass's velocity carried to the rear axle by the body's rotation.
- Nav's controller computes the command (`delta_cmd`, `a_cmd`) without integrating it. This
  plant arm (`veh_params::plant`, `cvc/nav/drive.h`) holds the command through the tick, and
  its governors respect the plant (doc 31 D14):
  - the wheelbase and steering lock are the vehicle's;
  - the steering command stays within `max_steer_rate x dt` of the actuator's angle;
  - `v_stop` allows for the distance the vehicle covers before it responds (`latency`) and
    plans with the deceleration it can make (`brake_margin x max_decel`); to keep under it
    the command may brake that hard;
  - the creep floor never exceeds the stopping speed;
  - the envelope and the size are the vehicle's own too (below).
- `nav_plant` maps the command to `world::drive`. The acceleration goes along the direction
  the agent is to move at the tick's end, with `reverse` when that is backwards.
- It steps the world `dt / fixed_dt` times (or, with `step_world = false`, leaves the stepping
  to a clock both follow).
- It reads every agent back again.
- It measures the tick's drift: nav's bicycle, run from the plant's own start state with the
  same command, against where the plant is. The bicycle takes the plant's
  `reference_substeps()`, its physics steps a tick (D14's `nsub = k`, 6 at 100 Hz) unless
  `drift_nsub` says otherwise.

The hard de-overlap is skipped with a plant, because the bodies collide instead.

Positions, speeds and accelerations cross nav's normalized frame and SI only here, through
`world_units` with the sim_world's `nav::frame_of(cfg)`.

**The drift** is published per vehicle and binned by the bicycle's lateral acceleration
(`drift_bins_g`, default 0.1, 0.2, 0.3 and 0.5 g):

- `last_drift(i)` and `drift_summary()`, in SI;
- with `publish(ctx, prefix, names)`, `<prefix>.vehicles.<name>.nav.{steer_command,
  accel_command, drift.{position_m, heading_rad, speed_mps, lateral_mps2, stop_speed_mps}}`, next
  to the physics binding's `vehicles.<name>.*` and its energy keys;
- `<prefix>.nav_plant.drift.<b>.{lo_mps2, hi_mps2, ticks, position_rms_m, position_max_m,
  heading_rms_rad, heading_max_rad, speed_rms_mps, speed_max_mps}`;
- `<prefix>.nav_plant.{vehicles, advance_us, advance_us_per_vehicle}`, the cost;
- `<prefix>.nav_plant.settings.{step_world, ground_z, spawn_height, metres_per_world_unit,
  physics_steps, drift_bins_mps2}`, the settings fixed at construction.

Accelerations in the tree are in m/s², like every other quantity there. The C++ options, the drift
summaries and the plant log keep lateral acceleration in g (9.80665 m/s²): `lateral_g`,
`drift_bins_g`, `lo_g` / `hi_g`.

**Knobs.** `pull(ctx, prefix)` reads `<prefix>.nav_plant.{latency, brake_margin, drift_nsub}`.
These are seeded with the plant's values, applied when they change, and an invalid one is
ignored. They are also `set_latency`, `set_brake_margin` and `set_drift_nsub`. The drive reads
the latency and the braking from `limits()` every tick, so a change takes effect on the next
one.

**The vehicle's own plan** (`nav_plant_options::plan_with_vehicle`, on by default). The navigator
plans with each vehicle's limits and size, not with its own `veh_params`, which the trained
metric sizes for no vehicle in particular. At Austin's 0.05 per metre, nav's defaults mean a 3 m
disc, 18 m/s and 30 m/s^2. `limits()` gives `sim_world`:

- the acceleration, `accel_margin` (0.8) x `max_accel`;
- the top speed, `max_speed`, but no more than nav's `vmax`: that is a speed limit as well
  (`nav.vehicle.vmax` in the state tree), so it still caps a plant's agents;
- the cornering limit, `lateral_g` (0.5) g, but at most `tip_margin` (0.8) of the static
  stability factor (half the track over the centre of gravity's height) in g;
- the body: half its width, and how far the chassis box and the wheels reach ahead of and behind
  the rear axle.

`sim_world` drives with them in place of `cfg.veh`'s `a_max`, `vmax` and `a_lat_max` and of
`set_vehicle_kinematics`. It covers the body with `sim_world::kPlantDiscs` (4) footprint discs
from tail to nose. The discs are inset 0.7 half-widths at the ends and are 1.3 half-widths wide,
so they cover the corners; they act at `body_gain` 1/4. They replace nav's disc or footprint.
`plant_plan_of(i)` reads them back.

- On a Humvee: 2.0 m/s^2, 24.6 m/s (18 m/s under nav's Austin defaults), 0.5 g, and four discs
  of 1.42 m from 0.0 to 3.3 m ahead of the rear axle.
- `nav_plant_test`'s PlansWithTheVehiclesOwnEnvelope drives a sim_world sized with nav's defaults
  through a hard turn. On the vehicle's plan it commands at most 2.0 m/s^2 and 0.50 g; on nav's
  own numbers, 30 m/s^2 and 0.98 g.
- The knobs are `<prefix>.nav_plant.plan.{with_vehicle, accel_margin, lateral_g, tip_margin}`,
  and `set_plan_with_vehicle` and the rest.
- The P4 gates below size nav by hand from the vehicle, at chosen corner caps, so they turn the
  plan off to compare the plant against the same kinematic arm.

Like the physics binding's, every one of these keys says what it is: a comment, `access`, `unit`
and, on a knob, a recommended `min` and `max`. So do the `nav_stats` leaves that
`publish_nav_stats` writes.

**nav_stats** gains `energy_J`, `energy_per_km` (J per km of path) and `building_contacts`
per vehicle, and `total_energy_J` and `total_building_contacts` per episode. `energy_J` is
what the vehicle drew from its source, net of regeneration. `building_contacts` counts the
contacts a chassis starts with static or kinematic geometry other than the ground: name the
ground's bodies in `nav_plant_options::ground_bodies` (pycvc: `nav_plant(..., ground_bodies=[id])`;
`plant_host`, `nav_eval` and the Jolt rollout name theirs), or a chassis scraping it counts.

These fields, and their JSON, appear only when a plant measures energy. The kinematic record
is unchanged, and `fuel_used` keeps its proxy meaning.

cvcGL's `publish_nav_stats` writes them too, again only when tracked:

- `<prefix>.nav_stats.veh.<i>.{energy_J, energy_per_km, building_contacts}`;
- `<prefix>.nav_stats.episode.{total_energy_J, total_building_contacts}`.

The plant also keeps a **plant ledger** for nav_stats (NAV_STATS.md, "The plant ledger"):
`nav_plant::sample(i)` is agent i's `cvc::nav::plant_sample` at the end of the last tick (speed,
attitude, nav's commands, accelerations, roll rate, the lateral load transfer ratio, wheels down,
static contacts; SI), measured in `advance()` from the same vehicle state as the energy, and
`sim_world` feeds it to the collector. `veh_nav_stats` then has `rolled_over` (tipped past 60
degrees for 1 s, latched) and when, the peak tilt, roll and pitch, the load transfer, the lateral
acceleration and how much of the steering and the brakes nav used; the episode `vehicles_rolled`
and `first_roll_s`; the scorecard `rollover_rate`. Only on a plant: the kinematic record is
unchanged. `planar_sample::roll` (the plant log) now has the sign its comment gives, + left side
up (it was the right side's), so the two agree.

The sample also carries `vehicle_contact` (`vehicle_contact_step`: the agent whose vehicle it
touched over the tick), which nav_stats reduces to `vehicle_hits` and `vehicle_hit_s`. The dynamic
plants (`dyn_bike_plant`, `dyn_4w_plant`) fill the same sample from their models and their chassis
boxes' contacts, and say in `plant_sample::measured` what their models lack (the single track's
load transfer, roll rate and wheels off the ground; both plants' contacts without a world). Those
items are not applicable, and `cvc/nav/plant_ledger.h` lists every item with what it needs, so
the JSON (null), `publish_nav_stats` ("n/a") and a screen show the whole ledger on any plant.

**The ledger a tick** (`nav_plant::ledger_step(i)`): agent i's `vehicle_energy` over the last
tick, `energy_lines` (vehicle.h) of its ledger at the tick's start and end. Each line (supplied
through residual, and engine, regen, source, fuel, distance) is the difference; the levels
(kinetic, potential, spring), the charge, the fuel left and the power are the end's. So a tick's
lines balance as the ledger does, and they sum to it. `energy_step(i)` is what the source paid of
them. The Jolt rollout calibrates the four-wheel model's books against these lines.

**Replay** (`nav_plant::save()` / `restore()`). The run's whole state: `world::save` and each
agent's last command, energy so far and over the last tick, contacts, sample and drift, and the
drift summary. A run restored to it continues bit for bit: the states, samples and ledger lines
repeat exactly. `restore()` needs the same plant attached to the same agents, on a world that
`world::restore` accepts. Otherwise it throws and changes nothing. The cost (`advance_us`) and the
settings are not part of it, as a world's materials are not part of `world::save`.

Gates (`nav_plant_test`, Windows MSVC, the Humvee in nav's trained metric, 1 unit = 20 m):

| Gate | Measured |
|---|---|
| (1) Every nav golden and parity test unchanged | The kinematic arm is untouched without a plant; the nav suites pass as before |
| (2) Below 0.3 g on flat dry ground, within 0.5 m of the kinematic trajectory over 200 ticks | A curving route, 79 m at up to 0.09 g: the Jolt path strays at most 0.03 m from the bicycle's. At a tick the two differ by up to 0.9 m along the path (the powertrain's response to an instant acceleration). Reported, not gated: a swerve round a block at 0.25 g strays 1.2 m, and a step change of goal 0.58 m, from the Humvee's 0.35 rad/s steering (0.73 and 0.46 m with an instant actuator) |
| (3) At the limit the plants diverge, reported | A 1 g corner cap on mu 0.6: position rms 3 mm below 0.1 g, 0.1 m above 0.5 g, binned in the summary |
| (4) Cost per Jolt vehicle | 0.57 ms a Humvee a tick (six physics steps, the energy ledger on) with eight among walls, against 60 ms of real time; 32 on open ground take 5.4 ms a tick, 3.2 ms on three Jolt worker threads (`physics_plant_host_test`) |
| (5) The adapter residual at nsub = k | An instant steering actuator on mu 1.2: at most 7 mm, 0.009 rad and 0.1 m/s a tick |
| (6) The drift published, binned by lateral g | Per vehicle and per bin, under `vehicles.<name>.nav.drift` and `nav_plant.drift`, with the plant's knobs and settings under `nav_plant` |
| (7) The speed never above the latency-aware v_stop | Driving at a wall at up to 11.5 m/s: at most 0.04 m/s above it at a tick; no contact with the wall |

## Choosing the plant: `plant_host`

```cpp
#include <cvc/physics/plant_host.h>

cvc::physics::plant_host host;      // owns the physics world and the nav_plant
host.use(sim, "humvee");            // sim's agents drive Humvees on Jolt
host.use(sim, "humvee, roboracer"); // a list of presets, cycled over the agents
host.use(sim, "kinematic");         // back to nav's bicycle
host.pull(app, "demo", sim);        // or live, from the state tree
```

`plant_host` puts a `sim_world`'s agents on a plant by name. It builds a new world on each
switch: a flat ground (`plant_host_options::ground_z`, `ground_mu`), or, after `set_geometry`, a
`cvc::world`'s collide items (`add_world_geometry`). It owns the world and the `nav_plant`, which
plans with its vehicles' own limits.

- Without a Jolt backend a preset falls back to the kinematic bicycle: `driving()` says
  "kinematic" and `fallback()` is true.
- An unknown name throws and changes nothing.
- `pull(ctx, prefix, sim)` reads the knob `<prefix>.nav_plant.vehicle`; a change switches the
  plant. It writes `nav_plant.driving` (telemetry) and `nav_plant.available` (build), and on
  vehicles runs the plant's own `pull`.
- The host must outlive the sim_world's use of the plant. Call `use()` and `pull()` from the
  thread that steps the sim.
- `plant_host_options::world.threads` puts Jolt's job system on worker threads: cheaper, and the
  same state hash (`physics_plant_host_test`).
- `"dynamicboat"` (the launch) and `"dynamicboat:<presets>"` (`launch`, `jonboat`, cycled) put the
  agents on boats (`dyn_boat_plant`, below; `dynamic_boat()`). Their world has no flat ground: it
  holds the geometry, if set, and `set_water(field)`'s fluid field (still, or with a free surface
  such as a swell). Starts with no water under them throw after the old plant has gone, and the
  agents are then on the kinematic bicycle (`fallback()` is true). Boats need Jolt: without it
  the name falls back to the kinematic bicycle as a preset does.

The cvcGL nav demos take `--plant` (`kinematic`, `humvee`, `roboracer`), and the restart panel
has a plant control. They use three Jolt worker threads natively and none in the browser.

- They default to Humvees on Jolt when the build has it (`nav_city_swarm`'s hundreds of agents
  excepted). The navigator's A* routes and reactive drive still assume the bicycle's tight turns.
  In the Austin finale a Humvee (a 6.7 m turning circle) on A* routes wedges in a corner after
  act 2 replans it, upright and at full lock; 7 of its 8 Humvees end up wedged.
- On a plant, `nav_finale` drives vehicle routes instead (`plan_vehicle_route` and
  `route_follower`, docs/NAV_VEHICLE.md). Its Humvees then drive 5387 m in 900 frames, against
  1903 m on A* routes, and none wedges.
- `nav_material_demo` runs the bicycle and the Humvee side by side. The material layer keeps the
  bicycle out of the lake (0 entries) but not the Humvee (65): it was tuned on the bicycle.
- Training through the vehicle (P8) is what would get the reactive drive through on its own.

## Boats as nav's plant: `dyn_boat_plant` (O12)

```cpp
#include <cvc/physics/dyn_boat_plant.h>

cvc::physics::world w;                       // Jolt
w.add_field(sea);                            // a fluid field: still, or with a free surface
cvc::physics::dyn_boat_plant_options o;
o.physics = &w;
cvc::physics::dyn_boat_plant plant({cvc::physics::dyn_boat_desc::launch(),
                                    cvc::physics::dyn_boat_desc::jonboat()}, o);
sim.set_plant(&plant);                       // agent i drives descs[i % 2] on the water
for (;;) sim.step();
plant.books(0);                              // where agent 0's energy went
```

`dyn_boat_plant` puts a `sim_world`'s agents on boats (above, "Boats") in a world that floats
them, beside `dyn_bike_plant` and `dyn_4w_plant`, and `plant_host` selects it as `dynamicboat`.
Its `kind()` is `dynamic`, and its `medium()` is `cvc::nav::plant_medium::water`
(`cvc/nav/plant_types.h`): every other plant's is `land`, so a spawner can keep boats on water and
cars on land.

- **The boats.** A `dyn_boat_desc` is a `boat_desc` whose hull has a `hull_mesh`, plus what nav
  plans with: `max_speed`, its top speed through calm water, and `turn_radius`, its circle at full
  rudder and full throttle. The presets are measured (`PresetsDoWhatTheySay`): `launch()` is
  `physics_boat_test`'s 4 m box of 900 kg with a shaft propeller and a rudder; `jonboat()` a 3.6 m
  flat box of 350 kg with a 1.5 kN outboard that steers; `tactical()` the `vehicle-tactical-boat`
  package's patrol boat of 2800 kg on its own V hull, baked from the model into 16 stations of 8
  points (`loft_hull`; 242 vertices, 480 triangles; 6.85 m long, 2.33 m in beam, 7.8 m overall
  with its outboards, `length_overall`), floating 0.49 m over its keel, its collision shape the
  hull's convex hull. Its hull has its own lift (`hull_mesh::lift_coefficient` 0.56): the grip
  sideways of its keel and chines, which the triangles' pressure drag barely gives a V bottom (it
  goes as the flow's normal component squared), and the dynamic lift that has it rise and trim
  onto the plane at speed. Its two outboards are one thrust (8 kN ahead, 3.5 kN astern, none past
  18 m/s) that swings 0.3 rad, and their legs one 0.2 m² rudder at the propellers that swings with
  them: the hull's side force acts ahead of its centre of mass, so without the legs aft it yaws off
  its course and spins.
- **Attach.** Each agent gets a boat (`world::add_boat`) with its hull's origin over its start,
  heading its way, upright and at rest, floating at its draft on the water there:
  `float_height(desc, mass, density)` is how far the origin rides over the waterline when the
  `hull_mesh` displaces the hull's mass. A start with no fluid field under it
  (`world::water_at`), water shallower than the draft (the field's box, or a body without mass,
  such as the terrain, that a ray from the hull's top meets above the boat's deepest point), or a
  hull too heavy to float is refused, and a refused attach leaves none of its boats.
  `draft(desc, mass, density)` is how deep the boat reaches below the waterline: its keel, or its
  propeller or rudder where they reach deeper. Nav's point is the hull's origin: `state()` is
  its position, the forward axis's heading and its velocity over the ground along it.
- **The driver.** Nav's command is a bicycle angle and an acceleration, as for the wheeled plants.
  Every world step the plant turns it into `set_boat_controls`. The throttle is the thrust per
  unit mass for the acceleration, taken as `dyn_driver.h` takes it (half of what the last step
  missed by and an integral trim that learns what the water holds back), over the propeller's
  thrust at the boat's speed through the water; with no command at a crawl it holds the boat
  still. The rudder holds the yaw rate the bicycle would turn at, r = u·tan δ / L with L the
  hull's length: the rudder share tan δ / tan(max_steer) for that curvature, plus a proportional
  and integral term on the yaw rate's error as a curvature, (r_cmd − r)/u. A rudder works reversed
  astern, and so does that curvature. `options.driver = false` gives the controls raw.
- **The plan.** `limits()` is a bicycle of wheelbase L and lock atan(L / `turn_radius`), the
  rudder's rate (the lock over the actuators' time constant), braking on `max_reverse_thrust`/m,
  the latency plus the actuators' time constant, and with `plan_with_vehicle` `max_thrust`/m,
  `max_speed`, the lateral acceleration of the tightest turn at top speed and the hull's outline.
- **The books** (`dyn_boat_books`), every world step:
  kinetic now = kinetic at attach + drive − brake − rudder − damping + water + gravity + residual.
  `kinetic` is the rigid hull's, ½m|v|² + ½ω·Iω, its inertia the collision shape's (a box's or a
  sphere's exactly; a convex hull's from the `hull_mesh` as a solid). `drive` and `brake` are the
  thrust's power through the water, T·u_a with u_a the propeller's velocity through the water
  along its push, where it drives the boat on and where it holds it back. `rudder` is the work of
  the rudder's force against the water past it, its drag's; `damping` the hull body's own; `water`
  what the water does to the hull (`world::field_load_on`: buoyancy, resistance, damping,
  slamming) and its own motion's push on the propeller and the rudder (a swell's orbital
  velocity, a current); `gravity` m·g along the centre of mass's fall. Each force's work is taken
  at the step's mean velocity, which is what the integrator moves the hull by, so `residual` is
  the integrator's error and any collision's. `energy_step(i)` (nav_stats' energy) is the last
  tick's `drive`.
- **The plant ledger.** `sample(i)` has the centre of mass's velocity in the heading's frame, the
  yaw rate, the steering (nav's bicycle angle the driver holds), the hull's attitude and roll rate, its accelerations
  over the tick, and its contacts with static geometry (a pier, a shore) and with the other boats;
  no wheels and no load transfer.
- **Stepping.** A tick is dt / `fixed_dt` world steps. With `step_world` off, a host that steps
  the world calls `before_world_step()` and `after_world_step()` around each step and `advance()`
  between ticks, as with `dyn_bike_plant`; `world_gone()` forgets a world rebuilt in place. Boats
  come and go at run time (`add_vehicle`, `remove_vehicle`, `add_desc`).

Gates (`physics_dyn_boat_plant_test`, Windows MSVC Release):

| Gate | Measured |
|---|---|
| The presets do what they say | The launch tops out at 3.1915 m/s and turns at full rudder and full throttle on a 15.39 m radius (`max_speed` 3.19, `turn_radius` 15.4); the jonboat 4.0054 m/s and 4.12 m (4.0, 4.1); the tactical boat 10.2258 m/s, and turns at full lock at 8.935 m/s and 0.2770 rad/s, a 32.25 m radius (10.2, 32.3) |
| The tactical hull grips, does not spin and planes | Its lift more than triples the hull's side force as it drifts, and turns it at full lock on 32.25 m where the same boat without the lift needs 48.6 m (the bare hull, without lift or legs, spins); at full lock from throttle 0.15 to full, either way, 19.5-32.3 m at 15-17° of drift; at full throttle it trims 2.8° bow up from rest, rises 0.14 m and runs on 10.2 of its 13.3 m² of bottom, and given 30 kN it planes at 32 m/s on 3.5 m², risen 0.35 m (`TheTacticalHullGripsWithoutAFoil`, `TheTacticalBoatDoesNotSpinAtFullLock`, `TheTacticalHullPlanesAtSpeed`) |
| A speed and a yaw rate held | Nav's commands for 2.5 m/s, then 2.5 m/s at 0.08 rad/s: 2.5000 m/s and 0.08000 rad/s on calm water for both; on a 0.3 m, 5 s swell, 2.478–2.513 m/s and 0.0795–0.0798 rad/s |
| The books balance | A weave and a stop astern: the residual 6·10⁻⁸ to 2·10⁻⁷ of the flow on calm water, up to 3.5·10⁻⁵ on the swell. Steady at 2.5 m/s, the thrust's 11 305.28 J go to the water (11 248.80 J) and the rudder (56.25 J) |
| Nav drives the boats | A launch and a jonboat reach goals 120 m east from rest in 37.9 s, then turn round to goals about 100 m behind them and reach those in 61.0 s |
| plant_host | `dynamicboat`, `dynamicboat:jonboat,launch`; an unknown preset refused and the plant kept; no water refused, back on the bicycle |
| Hosting, contacts | A host stepping the world matches the plant's own stepping bit for bit; a hull hitting a pier counts a static contact |
| Pinned | `DynBoatPlant.CrossPlatformGoldenHash` `96abfc30fcae4fd4` (Windows MSVC; CI confirms Linux and macOS) |

pycvc binds it (`dyn_boat_plant`; "Python: pycvc"), and a scene's world vehicles do not spawn boats yet
(docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md, "As built (O12: the boat plant)").

## Terrain in the navigator's energy (P5)

```cpp
#include <cvc/nav/sim_world.h>
#include <cvc/nav/terrain.h>
#include <cvc/physics/nav_plant.h>

// heights: the sim grid's rows x cols in metres, row 0 at min_y
cvc::nav::terrain_config tc = cvc::physics::nav_plant::terrain_config_of(desc); // its limits
tc.lam_grade = 2.0f;           // the soft grade force (H1); 0 = off
sim.set_terrain(heights.data(), tc);
cvc::physics::nav_plant plant(w, {desc});
cvc::nav::plant_modulation m;  // H4, off by default
m.tire_gain = 3.0f;
plant.set_modulation(m);
sim.set_plant(&plant);
```

`cvc/nav/terrain.h` turns a height raster and a vehicle's limits into three planes
(`terrain_build`):

- **the slope**, dz/dx and dz/dy, for the governors;
- **the grade cost** r̃_g in [0, 1]: sin θ over the sine of the steepest feasible slope, so 1 is
  the vehicle's limit;
- **the infeasible cells**: steeper than `max_grade`, or, with a `tip_ratio`, than
  `tip_margin` × the static stability factor.

The steepest feasible slope is `terrain_limit(tc)`. Everything is per unit mass, so the trained
coefficients keep their meaning. Without a terrain every tick is byte-identical to before.

`nav_plant::terrain_config_of(desc, mu, gravity)` gives a vehicle's limits:

- its steepest climb is the lesser of what its tires hold on `mu` and what its first gear pulls
  at its mass, with the torque converter's stall multiplication;
- its static stability factor is the track over twice the centre of gravity's height above the
  ground.

For the M1097A2 that is a 0.68 climb and an SSF of 1.23 (CG 0.74 m).

The five terms (doc 31 D9):

- **H1, the grade force and the hard barrier.** The grade cost goes through the material
  pipeline (`material_build`): `F = -lam_grade ∇r̃_g` leans the drive toward gentler ground,
  and the infeasible cells are its hard plane (`lam_grade_hard`, with the material channel's
  `k_sharp` and `d_hat_m`). Slope is a material risk, not a wall: by default (`lam_grade` 0.5,
  `lam_grade_hard` 3) the drive weighs it like any other material, and a vehicle the planner
  sends over a steep cell it can physically drive is not pinned there as against a building. The
  hard weight is three times the material channel's: at 1 the Jolt Humvee crossing Austin's
  twisted ramps rolled in most runs, at 3 its lead never did. The terrain's risk is material
  risk where the drive reports and reads it: `drive_telemetry.mrisk` (and with it the nav
  ledger's `mrisk_mean`/`mrisk_max`) is the worse of the material's risk and the grade cost, and
  a risk net's look-ahead (`kFlagFeatRisk`) reads the worse of the two ahead, the terrain's alone
  when there is no material stack. With `block_infeasible` (off by default) the infeasible
  cells also join every belief plane's occupancy, so the SDF's barrier keeps the agents out.
- **H2, gravity in the governors.** With `governors` (the default) the drive samples the slope
  under each agent:
  - the stopping governor plans with `a_stop = max(min(a_max, a_max + g_along), 0.1 a_max)`, so
    with less braking downhill. It never counts on gravity's help uphill, because the command
    never brakes harder than `a_max`;
  - the corner cap loses gravity's pull across the slope;
  - the plant arm's achievable braking is adjusted the same way.

  On the Jolt plant the driver model also feeds gravity along the ground forward. It takes the
  grade from its wheels' contact normals, not the chassis' pitch.

  The forces steer a vehicle off risky ground but never slowed it there, and on a straight line
  the corner cap allows any speed. A Humvee routed along a 24-degree sidehill (risk 0.85)
  accelerated to 9.5 m/s across it and rolled where the ramp twisted. The **risk governor**
  (`veh_params::risk_governor`, off by default) caps the speed by material risk, the worse of
  the material's and the terrain's (read with or without the terrain's weights):
  `v_risk(r) = vmax - (vmax - risk_vmin) s`, `s = clamp((r - risk_onset) / (1 - risk_onset), 0, 1)`.
  `risk_probes` points spaced out to `risk_lookahead` ahead each allow the speed from which the
  stopping governor's braking (with the plant's latency) is down to that point's `v_risk` by the
  probe before it, so within one probe spacing of risky ground the vehicle is at its `v_risk`.
  `drive_telemetry.v_risk` reports the cap (0 when off). The CPU drive, both GPU drives and the
  plant arm apply it; the persistent `sim_world_cuda` path has no material or terrain. A
  `sim_world` switches it from its next step with `set_risk_governor(on, vmin, onset, lookahead,
  probes)` (pycvc: `nav_sim_world_set_risk_governor`). On a plant the forces' braking is often
  nothing at all: they slow an agent only by what they take from its goal pull's surplus over its
  top speed, and a plant's top speed is usually under nav's cruise, so a vehicle driving square at
  a hazard meets it at its top speed with its run unchanged. The governor slows it there. A route
  follower drives in the drive's place (`sim_world::override_command`) at its own route speed,
  so `sim_world` holds an overriding command to the speeds the drive's own command is held to:
  the agent's top speed (`vmax`; on a plant the lower of the plant's and nav's) and, with the
  governor, its cap (`risk_speed_cap`, with the plant's braking and latency). Without that, on
  cvcdbg's convoy, where every Humvee follows a vehicle route, neither nav's `vmax` nor the
  governor changed how fast they went.
- **H3, the slope look-ahead feature.** `coef_feats(..., slope)` appends the worst grade cost
  between the agent and its carrot, after grip and risk. A coefficient net that takes it sets
  `kFlagFeatSlope` and needs format v3, which an older host refuses rather than reading it as a
  grip net. The CUDA drive does not run slope nets yet.
- **H4, measured slip and load as modulators.** `plant_state` carries `tire_use` (the most used
  tire's combined slip over its peak; 0 below 2 m/s), `load_transfer` (|left − right| over the
  total wheel load), `pitch` and `roll`. Past their onsets they scale each agent's gamma (its
  speed damping) and, for load transfer, its hard barriers' weight. The gains are the plant's
  (`vehicle_plant::modulation()`, `nav_plant::set_modulation`); all are 0 by default.
- **H5, an energy route cost.** `astar(..., cost, climb)` and `astar_query::climb` charge each
  move `per_metre` × its rise. With `climb_weight(c_rr, cell_w)` a metre of climb costs what
  rolling 1 / C_rr metres on the flat does: the energy at the wheels. Downhill costs nothing
  more, since brakes do not give the energy back.

`sim_world::infeasible_entries()` counts each agent's ticks with its footprint on an infeasible
cell (the rear axle, or each body disc). `nav_stats` carries the count as
`infeasible_slope_steps` per vehicle and `total_infeasible_slope_steps` per episode, only when
a terrain is set. `set_terrain_config(tc)` rebuilds the planes from the same heights, keeping
the counts.

**The state tree.**

- `sim_world::pull` (below, "sim_world's settings in the state tree") reads the terrain's knobs:
  - `<prefix>.nav.terrain.{max_grade, tip_ratio, tip_margin, sigma}`;
  - `<prefix>.nav.terrain.{lam_grade, lam_grade_hard, k_sharp, d_hat_m}`;
  - `<prefix>.nav.terrain.{block_infeasible, governors}`;
  - `<prefix>.nav.vehicle.{risk_governor, risk_vmin, risk_onset, risk_lookahead, risk_probes}`
    (speeds in m/s, the look-ahead in metres).

  They are seeded with the world's values. A change rebuilds the planes, and an invalid value
  is ignored.
- `sim_world::publish` writes `<prefix>.nav.terrain.{active, gravity, limit, infeasible_cells}`.
- `nav_plant::pull` reads `<prefix>.nav_plant.modulation.{tire_onset, tire_gain, tip_onset,
  tip_gain}`.
- `nav_plant::publish` adds each vehicle's `nav.{tire_use, load_transfer, pitch_rad, roll_rad}`.
- cvcGL's `publish_nav_stats` writes `veh.<i>.infeasible_slope_steps` and
  `episode.total_infeasible_slope_steps` when a terrain is tracked.

Every one of these keys has a comment, `access` and `unit`, and every knob has a recommended
`min` and `max`.

Gates (`nav_plant_test` on the Jolt plant, `nav_terrain_test` on the kinematic one; Windows
MSVC, the Humvee):

| Gate | Measured |
|---|---|
| (1) Parity goldens | Without a terrain and without modulation gains the drive is untouched; the nav suites pass as before |
| (2) Energy per km drops on a hilly route, the path growing under a fixed bound | A 12 m hill between start and goal. The plain run drives 135 m over it: 2081 kJ, 15403 kJ/km. With H1, H2 and H5 it drives 194 m round it: 1518 kJ, 7826 kJ/km. The bound is 1.5 × the path |
| (3) Zero entries into infeasible slope | A ridge with slopes of 1.2 (past the Humvee's 0.68 climb and its tip limit) across the way: 0 ticks on it, Jolt and kinematic |

Also measured:

- **H2.** Driving at a wall, 15 m out the agent does 6.39 m/s on the flat and 3.37 down a 15%
  grade, and 6.39 up it. It stops 4.45-4.50 m short in every case.
- **H1.** Past a hill, the steepest grade cost driven is 0.727 without the soft force and 0.228
  with it.
- **H5.** Over a 12 m hill the plain route is 140 m and climbs 11.99 m. The energy route is
  194 m and climbs 0.24 m. For a Humvee the climb is about 315 kJ and the detour about 30 kJ.
- **H4.** Round a 30 m circle on μ 0.7, under a 0.9 g corner cap, mean tire use is 1.36 at
  15.1 m/s without the modulator. With `tire_gain` 3 it is 0.79 at 11.9 m/s.

Open: GRL-SNAM's Python twin of H1 and H5, a coefficient net retrained on the slope feature (H3),
and H3 in the CUDA drive.

### sim_world's settings in the state tree

A `sim_world` binds its own settings, as `world_binding` and `nav_plant` do:

```cpp
sim.pull(app, "scene");    // between steps: the tree's knobs into the world
sim.step();
sim.publish(app, "scene"); // the fixed settings and the tick
```

Lengths, speeds and accelerations in the tree are in nav's world frame (metres, seconds), never
the normalized one (normalized = world × `config.scale`) or the map's cells. So
`nav.vehicle.vmax` is in m/s whatever metric the net was trained in. `nav.sep_gain` (the carrot
shift at contact) is in metres, and `nav.terrain.sigma` (the grade cost's blur, kept in cells by
`terrain_config`) is in metres too. The force weights are in world units as well:

- `nav.material.lam_hard` and `nav.terrain.lam_grade_hard` (a barrier's push at full strength)
  are in m/s² (normalized ÷ scale);
- `nav.material.lam_soft` and `nav.terrain.lam_grade` (the push per unit of risk or grade cost
  gained per metre) are in m²/s² (normalized ÷ scale²);
- `lam_soft_scale` and `lam_hard_scale` are plain multipliers.

`pull()` reads these knobs. Each is seeded with the world's value when unset, applied when it
changes, and ignored when invalid:

- `<prefix>.nav.{range_m, n_rays, fov_rad, sense_every, freeze_sense, l_occ, l_free, optimistic,
  p_thresh, band, ttl_s, reach_tol, sep_radius, sep_gain, min_gap, min_gap_iters}`: sensing,
  belief, arrival and the crowd (`set_live_knobs`' set);
- `<prefix>.nav.vehicle.{vmax, a_max, a_lat_max, wheelbase, delta_max, track_width, k_steer,
  nsub, allow_reverse, rr, d_hat, body_rr, body_gain, mass, mu_lookahead, mu_probes}`:
  `veh_params`' scalars. A per-agent column (`set_vehicle_kinematics` and the like) overrides
  its scalar;
- `<prefix>.nav.material.{lam_soft, lam_hard, lam_soft_scale, lam_hard_scale}`, once a material
  is set;
- `<prefix>.nav.terrain.*`, once a terrain is set (above).

`publish()` writes:

- `<prefix>.nav.settings.{rows, cols, min_x, min_y, max_x, max_y, cx, cy, scale, l_clamp, dt,
  agents, planes, plant}`, fixed when the world is built;
- `<prefix>.nav.tick`;
- `<prefix>.nav.terrain.active`, and with a terrain its telemetry.

A plant's knobs stay the plant's (`nav_plant.*`), and the episode's stats stay nav_stats'.

Every key says what it is in `cvc/state/state_key_meta.h`'s form, the one `world_binding` and
`nav_plant` use: a comment, `access` (build, knob, command or telemetry), `unit` and, on a knob,
a recommended `min` and `max`. `describe_state_keys(ctx, base, table)` describes a table of keys
for any binding.

## Python: pycvc (P6)

pycvc wraps the world, its vehicles, the state-tree binding and the navigator's plant
(`bindings/pycvc/pycvc_physics.i`). Python drives the same calls as C++ and reads the same
values:

```python
import pycvc

w = pycvc.physics_world(backend="jolt", fixed_dt=0.01)    # world_config's fields as keywords
ground = w.add_material("ground", friction=0.9, rolling_resistance=0.015)
w.add_box((150, 150, 0.5), position=(0, 0, -0.5), motion="static", material=ground)
car = w.add_vehicle(pycvc.vehicle_desc.humvee(), position=(0, 0, 1.2))
w.drive(car, steer=0.05, accel=1.5)                       # or set_controls(throttle=..., ...)
w.step(100)
w.vehicle_state(car)["energy"]["source"]                  # J drawn from the fuel
w.state_hash(), w.save(), w.snapshot_json()

app = pycvc.make_app()
water = pycvc.volume(app)                                 # kg/m^3 over the field's box
water.set_float_grid([1000.0] * 8, 2, 2, 2, -5, -5, -5, 5, 5, 1)
sea = pycvc.wave_train_surface(depth=0.0, trains=[{"amplitude": 0.2, "period_s": 4.0}])
lake = w.add_field(water, kind="fluid", damping=1e4, surface=sea)
w.follow(clock)                                           # a pycvc.world_clock
w.field_time(), sea.height(x, y, w.field_time())          # where the bodies feel the sea

binding = pycvc.physics_binding(app, w, "physics")       # world_binding
binding.publish_vehicle("car", car)
binding.push()
pycvc.state_get(app, "physics.vehicles.car.speed")
pycvc.state_property(app, "physics.vehicles.car.speed", "unit")   # "m/s"

plant = pycvc.nav_plant(w, [pycvc.vehicle_desc.humvee()])
pycvc.nav_sim_world_set_plant(sim, plant)                # the agents drive Humvees
pycvc.nav_sim_world_set_plant(sim, None)                 # back to nav's bicycle
pycvc.nav_sim_world_set_terrain(sim, heights, pycvc.nav_plant.terrain_config_of(desc))
pycvc.nav_sim_world_pull(sim, app, "scene")              # sim_world's settings in the tree
```

The parts:

- **The world.** Bodies come from `add_box`, `add_sphere`, `add_hull`, `add_mesh` (a
  `pycvc.geometry`) and `add_terrain` (a `pycvc.height_field`). Bodies and vehicles are ints,
  and a stale id raises as it does in C++. Vectors are 3-sequences and rotations `(w, x, y, z)`.
  A body's or vehicle's state comes back as a dict of floats and tuples, wheels and energy
  ledger included. `add_force`, `add_force_at(id, force, point)`, `add_torque` and
  `add_impulse` take 3-sequences. `contacts()`, `raycast()`, `save()` / `restore()` and
  `state_hash()` work as in C++. `snapshot_json()` is C++'s own serializer (`snapshot_json.h`).
- **Fields and free surfaces.** These follow `field.h` and `fluid_surface.h`.
  - **Fields.** `add_field(volume, kind, velocity, drag_coefficient, damping, traction, ruts,
    rut_cell, surface)` takes a `pycvc.volume` and `field`'s values as keywords. `kind` is
    `"fluid"`, `"medium"` or `"flow"`. `velocity` is None or three volumes. Field ids are ints.
    `set_field` takes the same keywords and sets the whole field, as C++ does.
  - **Reading them.** `remove_field`, `field_count` and `field_load_on(body)` (force, torque,
    immersed volume) work as in C++, and `physics_binding.publish_field` names a field in the
    tree.
  - **Surfaces.** `still_surface(level, current)` and `wave_train_surface(level, depth, trains,
    gravity)` are C++'s classes. `trains` is a list of dicts of `wave_train`'s fields. Both
    have `level`, `height`, `velocity` and `digest`, and a wave train also has `train(i)`,
    `wavenumber(i)` and `omega(i)`.
  - **Lifetimes.** A field shares its surface, which stays alive after Python drops it, and
    copies its volumes. Python cannot subclass a surface: the step reads it without the GIL.
  - **The time base.** `follow(clock)` takes a `pycvc.world_clock`. `field_time()` and
    `set_field_time(t)` are the time base the surfaces are read at.
- **Vehicle presets.** `vehicle_desc.humvee()` and `roboracer()`, with the scalar fields
  readable and writable, and `axle(i)`, `gear_ratios()`, `half_extents()` and
  `center_of_mass()`.
- **The plant.** `nav_plant(world, descs, ...)` takes `nav_plant_options` as keywords. It has
  `state(i)` (the measured tire use, load transfer, pitch and roll included), `vehicle(i)`,
  `last_drift(i)`, `drift_summary()`, `set_modulation()`, `pull()` / `publish()`, and the
  static `terrain_config_of(desc, mu)`.
- **The sim_world.** `nav_sim_world_set_plant` selects the plant and `nav_sim_world_plant_kind`
  reports it. `nav_sim_world_set_terrain` and `set_terrain_config` set the terrain;
  `terrain_settings` and `infeasible_entries` read it back. `nav_sim_world_pull` / `publish`
  bind its settings, and `nav_sim_world_tick` counts its ticks.
  `nav_sim_world_create` takes `""` for the built-in policy (`coef_mlp::default_biased`), and
  an optional `map_id` / `n_planes` that puts agents in their own belief planes.
- **The terrain planes.** `nav_terrain_build(heights, cell_w, config)` is `terrain_build`: the
  slope, the grade cost, the infeasible cells and the limit, as numpy arrays. It is the C++
  reference a Python twin of the planes is checked against.
- **The wake** (`field.h`'s `field_wake`). `pycvc.field_wake(cells=64, cell=1.0, ...,
  shore=None)` holds `field_wake`'s values; `add_field(..., inertial=True, wake=...)` puts it on
  a fluid with a surface. `field_wake_info(field)` is a dict of `wake_info` (placed, x0, y0,
  cell, cells, windows, sources, outside); `field_wake_height(field, x, y)` and
  `field_wake_of(field)` read it, and `water_at(p, body=None)` is the water at a point (wet,
  depth, density, velocity) with the waves in it, less `body`'s own. `shore_grid_mask(depth,
  shore_m, min_x, min_y, cell, slope=None)` is a coast from (rows, cols) planes. `add_box`,
  `add_sphere` and `add_hull` take `hull=` (a `hull_mesh`) for a body the water pushes on by
  its triangles.
- **Wading** (`vehicle.h`'s `wading_desc`). `vehicle_desc.wading` is a `pycvc.wading_desc`
  whose fields read and write in place (`d.wading.fording_depth = 1.52`); `as_dict()` gives
  them all. `vehicle_state` adds `water_force` and `engine_flooded`, each wheel `water_depth`
  and `water_drag`. `set_vehicle(id, desc)` tunes a running vehicle, wading's knobs included;
  `wading.enabled` and `wading.displacement` are fixed when it is added. The binding's
  `vehicles.<name>.tuning.wading.*` knobs do the same from the tree.
- **The wind** (`flow_velocity.h`). `wind_flow(speed_mps, dir_rad, gustiness, base_z)` is the
  scene's wind; `scene_wind_flow(speed, dir, gustiness=0.25, base_z=0)` is the one Ariadne's
  `wind.speed`, `wind.dir` (degrees) and `wind.gustiness` keys make. Both have `velocity(p,
  t)`, `reference(p, t)`, `profile(z)`, `base()` and `digest()`. `physics_world.set_wind(speed,
  dir, gustiness, base_z)` keeps one as a flow of air everywhere (`add_wind`; a second call
  changes it in place, `None` takes it away; `wind()` and `wind_field()` read it).
  `add_field(None, kind="flow", wind=...)` adds one by hand. `mujoco_robot.set_wind(air,
  time_offset, drag_coefficient, density)` blows on a robot; `wind_force()` and
  `wind_torque()` read it.
- **Boats** (`hull.h`, `boat.h`, `dyn_boat_plant.h`). `hull_mesh(vertices, triangles, ...)`,
  `box_hull(half_extents)` and `loft_hull(stations)` (each `{"x": x, "section": [(half_width,
  height), ...]}`) make hulls, with `vertices()`, `triangles()`, `bounds()` and
  `waterline_box(z)`. `boat_desc` holds the hull body (`hull()`, `set_hull(half_extents=...,
  mass=..., ...)`), its mesh (`hull_mesh()`, `set_hull_mesh()`, copied in and out), and the
  propeller and rudder as dicts (`propeller()`, `set_propeller({...})`, `rudder()`,
  `set_rudder({...})`). `dyn_boat_desc.launch()`, `jonboat()`, `tactical()` and `preset(name)`
  add `max_speed`, `turn_radius` and `length_overall`. `physics_world.add_boat(desc, position)`,
  `set_boat_controls(id, throttle, rudder)`, `boat_state(id)`, `remove_boat` and
  `boat_desc_of`. `dyn_boat_plant(descs, world, ...)` is the boat plant, with `state(i)`,
  `boat_state(i)`, `books(i)`, `hold_controls(i, ...)` and the static `float_height(boat,
  mass, density)` and `draft(...)`; `nav_sim_world_set_plant` takes it. `plant_host` adds
  `boat_state(i)` and `boat_books(i)` on "dynamicboat".
- **Key metadata.** `state_comment`, `state_property` and `state_property_keys` read what a
  key says it is.
- **Lifetimes.** A `nav_plant` keeps its world alive, a `physics_binding` its app and world, and
  a sim_world handle the plant set on it, so dropping the Python names never leaves a dangling
  pointer.

**The gate: Python reads the same values as C++.** `test_pycvc_physics.py` checks it three
ways:

- It rebuilds the cross-platform golden scene (`PhysicsJolt.CrossPlatformGoldenHash`) through
  pycvc and gets C++'s pinned `70b2cc0bd34ea4be`. It does the same for the swell scene, a slab
  and a ball in a wave train (`PhysicsFluidSurface.CrossPlatformGoldenHash`), and gets
  `f0085b4212e1a8c0`.
- It runs `physics_parity_test` (the ctest entry passes its path in `CVC_PHYSICS_PARITY_EXE`),
  whose scenarios it rebuilds call for call. It then requires their state hashes and values
  bit for bit: a Humvee through the driver model (ending at 10.06 m) and a sim_world's agent on
  the Jolt plant (42.9 m), 3 hashes and 22 values.
- In one process, a vehicle's state, the snapshot JSON and the `physics.vehicles.<name>.*`
  keys carry the same numbers.

It also checks:
- save / restore;
- plant selection (the plant outlives its Python names while set);
- the sim_world's and plant's keys with their metadata;
- the terrain;
- the free surfaces: the surfaces against Airy's formulas, half draft on calm water, the field
  time following a clock, a save that restores the field time and refuses another surface, and
  a field whose damping the tree sets while its surface is kept.

That is 135 checks in all.

## Static world geometry: `add_world_geometry`

```cpp
#include <cvc/physics/world_geometry.h>
#include <cvc/world/city_bundle.h>

const auto city = cvc::world::load_city_bundle(app, dir);         // terrain + buildings
const auto bodies = cvc::physics::add_world_geometry(w, city.geometry);
const body_id ground = bodies.find("terrain");
```

- **One static body per item with `role_collide`:** a height field as `terrain`, and a mesh as a
  `mesh` at its item's placement. These are the same objects `gl::add_world_geometry` draws, never
  a copy.
- **One world material per surface the items use**, named after the surface, with its `mu`,
  `restitution` and `rolling_resistance` from `world::surface_registry`. A material of that name
  already in the world is reused. `bodies.materials[surface id]` is the world material id.
- **Not standard-library only**, unlike the rest of `cvc/physics`: it reads
  `cvc::world::world_geometry`.

## Drawing a world: `pose_tracker` and `physics_scene_binding`

A world steps in fixed quanta and a display runs at its own rate. Drawing each body at its newest
state judders whenever the two rates beat. Instead, a view draws between the world's last two
states, at the fraction of a step the clock reports, as `world_clock.h` describes. The view lags
the world by one step and never extrapolates.

```cpp
#include <cvc/gl/physics_scene_binding.h>

cvc::gl::physics_scene_binding view(w);                 // per world and scene; no globals
view.bind(crate, sg.addGraphics("crate", crate_mesh));   // a dynamic body
view.bind_group(swarm, [&](const auto &poses) { pack(poses); });  // one merged mesh
... per frame, before rendering:
const auto s = app.world_clock().advance(wall_dt);
view.follow(app.world_clock());                          // steps the world
view.present(app.world_clock(), s.alpha);                // poses the nodes
```

- **`pose_tracker` (core)** keeps the last two states of the bodies it tracks.
  - `follow(clock)` steps the world itself, in two parts when several steps are due, so the two
    samples are the last two ticks. Use it instead of `world::follow`, never both: each keeps its
    own cursor, and the world would take every step twice.
  - A world stepped some other way (on a simulation thread) calls `capture()` after each step.
  - `alpha(clock, clock_alpha)` turns the clock's fraction of a quantum into the world's fraction
    of a step, for a world that steps once every several quanta.
  - `sample_all(alpha)` / `sample_one(id, alpha)` lerp the position and blend the rotation along
    the shorter arc. A body without a previous sample exactly one step before its newest is drawn
    at its newest: just tracked, after a gap, or after `snap(id)` for a teleport.
  - A body removed from the world drops out at the next capture.
- **`physics_scene_binding` (cvcGL)** owns a tracker and writes the scene.
  - `bind(id, node, body_from_visual)` sets the node's local transform with
    `GraphicsNode::setPoseMatrix` every `present()`. The node's parent frame is the world frame, so
    a scene transform (a scale, a floating origin) belongs on a parent node. The optional row-major
    offset places geometry not modelled about the body's origin.
  - `bind_group(ids, sink)` calls the sink once per `present()` with those bodies' poses in group
    order, so a swarm is one merged mesh (`GeometryNode::updateVertices`,
    `StreamingGeometryNode::writePoints`), not one node per body.
  - Nodes are held weakly. A node that is destroyed, or a body removed from the world, is unbound
    at the next `present()`, and the node keeps its last pose.
- **One writer per actor.** Bind a body when physics decides where it is: a dynamic body. A
  kinematic actor driven by its own source (a nav agent on the kinematic plant, a tracked person, a
  scripted mover) is drawn by that source, which also moves its body with `move_kinematic` so it
  pushes things. Binding it too would make two writers fight over the node. Static bodies need no
  binding.
- **Threads.** Both are thread-safe. `present()` may run on any thread, since `setPoseMatrix`
  applies on the scene's owner thread and is posted to it from any other. Group sinks run on the
  thread that calls `present()`.
- **With a `world_binding`.** Its `follow()` steps the world too. Keep the state tree in step
  around the scene binding instead: `bind.pull(); view.follow(clock); bind.push();`.
- **Clock quanta.** `cvc::app`'s clock defaults to 1/120 s and a world to 0.01 s. That is not a
  whole number of quanta, so `follow` and `alpha` throw. Set one of them: the nav demos run the
  clock at 0.01 s.

## The state tree: `world_binding`

```cpp
#include <cvc/physics/world_binding.h>

auto cfg = cvc::physics::world_binding::config_from_state(app);   // seeds unset keys
cvc::physics::world w(cfg);
cvc::physics::world_binding bind(app, w);                          // prefix "physics"
const auto ice = bind.material("ice");                             // after the first pull()
... per frame:  bind.follow(app.world_clock());                    // pull, step, push
```

Keys, relative to the prefix. An unset key is seeded with the world's value. An unreadable or
invalid value is ignored and never thrown into a frame. `pull()` reads the tree only when
something under the prefix changed since the last pull, so an unchanged tree costs a pull about
2 µs. If a key is a link into another scope, it reads every time. A program's own `reconfigure`
or `set_material` stands until the tree's value changes.

**What each key is.** Every key the binding owns describes itself, for other applications.
Each has a comment (`cvc::state::comment`) and properties (`cvc::state::property`):

- `access`: `build` (read when the world is built), `knob` (applied when it changes), `command`
  (an input the world acts on) or `telemetry` (written by `push()`);
- `unit` on a number: SI, `1` for a ratio, empty for a count or a flag;
- a recommended `min` and `max` on a knob or a command, for a UI's slider or a validator.

The ranges are recommendations; the world validates on its own. The table is
`src/cvc/physics/binding_meta.cpp`, and `physics_vehicle_test` checks that every published key
has a comment and an access.

| Keys | Read | Notes |
|---|---|---|
| `backend`, `fixed_dt`, `threads`, `determinism`, `max_bodies`, `max_body_pairs`, `max_contact_constraints`, `origin.{x,y,z}` | `config_from_state` | A later change sets `restart_required` and leaves the running world alone |
| `gravity.{x,y,z}`, `collision_steps`, `velocity_steps`, `position_steps`, `penetration_slop`, `friction_combine`, `restitution_combine`, `contacts` | every `pull()` | Applied with `reconfigure` |
| `materials.<name>.{friction, restitution, rolling_resistance, nav_material}` | every `pull()` | A new child adds a material, and a changed value updates it. `default` is material 0. |
| `publish_every`, `snapshot_every`, `snapshot_json_every`, `hash_every`, `publish_thread` | every `pull()` | How often and where publishing happens (below) |
| `tick`, `time`, `step_ms`, `bodies`, `active_bodies`, `contact_events`, `saturated`, `version`, `restart_required`, `state_hash` | written by `push()` | Telemetry |
| `snapshot` | written by `push()` | The `world_snapshot` as `std::shared_ptr<const world_snapshot>` in the node's `data()`. `world_binding::snapshot_from_state(app)` reads it from any thread |
| `snapshot_json` | written by `push()` | That snapshot as JSON without the static bodies: Python reads it as `json.loads(pycvc.state_get(app, "physics.snapshot_json"))`, and a remote peer of the tree receives it like any value |
| `snapshot_json_static` | written by `push()` | The static bodies as JSON, rewritten only when one is added, removed, moved or given a material. Its `static_version` matches `snapshot_json`'s then |
| `publish_dropped` | written by `push()` | Publications the publish thread skipped because a newer one replaced them |

### How often, and on which thread

Publishing runs on the thread that steps, after the step, every `publish_every` steps. Each
kind of output has its own rate, counted in steps, so with `fixed_dt = 0.01` a value of 10 means
10 Hz:

| Key | Default | What it publishes |
|---|---|---|
| `publish_every` | 1 (every step) | Each push: the telemetry and the named actors' keys |
| `snapshot_every` | 1 (every push) | The `snapshot` data. 0 turns it off |
| `snapshot_json_every` | 0 (off) | `snapshot_json`, and `snapshot_json_static` when the static bodies changed |
| `hash_every` | 0 (off) | `state_hash`, which digests the whole state |
| `publish_thread` | 0 | 1: a background thread writes what each push publishes, so the step pays only for copying the snapshot. A newer publication replaces one the thread has not started; `flush()` waits for it to catch up. 0: `push()` writes before it returns |

Measured on Windows (MSVC) with 1000 moving bodies and four named bodies, on a loaded machine
(a step took about 3 ms). These are medians of the time on the stepping thread
(`WhatPublishingCostsAStep`):

| What a push does | Time on the step |
|---|---|
| `snapshot()` alone | ~0.3 ms |
| `pull()` with nothing changed | ~1.5 µs |
| Telemetry and 4 named bodies (about 80 keys) | ~0.4 ms |
| That and the snapshot data | ~0.6 ms |
| That and the JSON (560 KB) | ~4.5 ms |
| All of it, with `publish_thread` = 1 | ~0.2 ms |

So: keep the snapshot every step if in-process readers want it; publish JSON at the rate its
readers need (10–30 Hz); and set `publish_thread` when publishing must stay off the step.

### Named bodies, vehicles and fields

```cpp
bind.publish_vehicle("truck", truck);   // physics.vehicles.truck.*
bind.publish_body("crate", crate);      // physics.bodies.crate.*
bind.publish_field("snow", snow);       // physics.fields.snow.*
bind.publish(add_world_geometry(w, g)); // every collide item and volume, by name
```

A published actor gets its own keys. `push()` writes its telemetry from the same snapshot it
publishes, so all of a push's keys agree on one tick. Doubles are written to full precision, so
a key reads back exactly as the world held it.

| Keys, under `bodies.<name>`, `vehicles.<name>`, `fields.<name>` | Read | Notes |
|---|---|---|
| `bodies.<name>.{id, motion, position.*, rotation.*, velocity.*, angular_velocity.*, active, field_force.*, immersed_volume}` | written by `push()` | `motion` is `static`, `kinematic` or `dynamic` |
| `bodies.<name>.{material, mass, linear_damping, angular_damping, allow_sleeping}` | every `pull()` | `world::set_body`; the material by name |
| `vehicles.<name>.{chassis, speed, steer, steer_command, throttle, brake, hand_brake, gear, engine_rpm, accel_trim, position.*, rotation.*, velocity.*, angular_velocity.*, aero_force.*, air_velocity.*}` | written by `push()` | The `vehicle_state` |
| `vehicles.<name>.wheels.<i>.{steer, angular_velocity, suspension_length, contact, contact_point.*, load, longitudinal_slip, lateral_slip, friction, rolling_resistance, medium_depth, sinkage, rut, medium_resistance}` | written by `push()` | Each `wheel_state` |
| `vehicles.<name>.energy.{P_W, supplied_J, idle_J, brake_J, tire_J, aero_J, roll_J, medium_J, drivetrain_J, drivetrain_split_J, suspension_J, chassis_damping_J, fields_J, stored_J, residual_J, regen_J, source_J, soc, fuel_kg, fuel_left_kg, distance_m, Wh_per_km}` | written by `push()` | The energy ledger, in doc 31's D11 names |
| `vehicles.<name>.command.{steer, accel, reverse}` | every `pull()` | A change calls `world::drive` |
| `vehicles.<name>.controls.{steer, throttle, brake, hand_brake, reverse, neutral}` | every `pull()` | A change calls `world::set_controls` |
| `vehicles.<name>.tuning.<value>`, `tuning.axles.<i>.<value>` | every `pull()` | `world::set_vehicle`, with the values above; `engine_friction_curve` is a string of `rpm:N m` points, `"700:78, 2040:87, 3400:128"`, or `"none"` |
| `fields.<name>.{drag_coefficient, damping, traction}` | every `pull()` | `world::set_field`, which keeps a medium's ruts |
| `fields.<name>.{kind, ruts.cells, ruts.deepest}` | written by `push()` | Telemetry |

- **Commands** act only when they change, so a program can still drive the vehicle through the
  world between changes. A command already in the tree when the vehicle is published is
  applied at once.
- **Knobs** follow the material idiom: they are seeded with the world's values and applied when
  the tree's value changes. An invalid value is ignored, and the world keeps its last good
  value.
- **Removed actors** are no longer published once the world drops them; their keys keep their
  last values.
- **Cost.** A key write costs about 5 µs, so a published vehicle (about 90 keys) adds about
  0.34 ms to a push. `publish_every` throttles it, `publish_thread` moves it off the step, and
  in-process readers can use the snapshot instead. When no whole snapshot is due, a push reads
  only the named actors.

Each binding belongs to one app and one world, so two apps configure two worlds independently.
