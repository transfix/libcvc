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
| `cvc/physics/body.h` | `body_desc`, `body_state` |
| `cvc/physics/backend.h` | `backend`, capability bits, `available_backends()` |
| `cvc/physics/world.h` | `world_config`, `world`, `contact_event`, `ray_hit`, `step_report` |
| `cvc/physics/field.h` | `field` (a `cvc::volume` as water, snow or wind), `field_load` |
| `cvc/physics/vehicle.h` | `vehicle_desc` (with `humvee()`, `roboracer()`), `drive_command`, `vehicle_controls`, `vehicle_state`, `wheel_state` |
| `cvc/physics/world_binding.h` | `world_binding`: the state-tree face of a world |
| `cvc/physics/pose_tracker.h` | `pose_tracker`, `pose`, `interpolate()`, `pose_matrix()`: the last two states, for a view |
| `cvc/gl/physics_scene_binding.h` (cvcGL) | `physics_scene_binding`: a world's bodies drawn in a scene graph |
| `cvc/physics/world_geometry.h` | `add_world_geometry`: a `cvc::world::world_geometry`'s static bodies and materials |

The physics headers include only the standard library, except `world_binding.h`, which needs
`cvc::app`, and `world_geometry.h`, which reads `cvc::world`. No header names an engine.

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
| `determinism` | `cross_platform` | no |
| `friction`, `restitution` (combine rules) | `product`, `max` | yes |
| `contacts` (reporting) | `added_removed` | yes |
| `origin` | (0, 0, 0) | via `rebase()` |
| `max_bodies`, `max_body_pairs`, `max_contact_constraints`, `temp_allocator_bytes` | 65536, 65536, 65536, 32 MiB | no |

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
- **Batch reads:** `states(ids)` returns several bodies' states and the tick under one lock, so a
  batch never mixes two steps. An unknown or removed id reads as empty instead of throwing.
- **Generation-checked ids.** Body ids carry a generation, so a stale id is refused and never
  aliases a newer body.
- **Stepping:** `step()` advances `fixed_dt`, and `step(n)` runs n steps under one lock.
- **Following a clock:** `follow(clock)` runs the steps due on a `cvc::world_clock` since the last
  call. `fixed_dt` must be a whole number of the clock's quanta.
- **Reports:** `last_report()` returns the last step's `step_report`. It carries wall time, active
  bodies, the number of contact events, and `saturated` when an engine buffer filled.

### Determinism, save and restore

- **Hashing:** `state_hash()` digests the whole simulation state: bodies, velocities, sleep state,
  the contact cache, the tick and the origin.
- **Capabilities:** with `determinism::cross_platform`, the Jolt build must be cross-platform
  deterministic (it is, from 5.6.0+cvc.5); otherwise construction throws. 1 thread and N threads
  give the same hash (`physics_world_test`).
- **Restoring:** `save()` returns the state as bytes. `restore()` takes them on a world holding the
  same bodies, created in the same order, with the same materials. A restored world continues
  exactly as the original did.

### Threads

The world is thread-safe. Every call locks it, and `step()` holds the lock for the whole engine
step, so a call from another thread waits at most one step. Accessors return copies. To read a
running world from many threads without waiting, read the telemetry a `world_binding` publishes.

### The whole world at once: `snapshot()`

`world::snapshot()` copies everything at one tick under one lock into a `world_snapshot`:

- the model: the config, the materials, every field as given (with its rut count and deepest
  rut), and every vehicle's description as tuned;
- the state: every body (id, motion type, material, user data, pose, velocities, awake, and
  what the fields did to it), every vehicle's `vehicle_state` with its wheels, and the last
  step's contacts.

Every list is in ascending id order. It costs one pass over the moving bodies: static bodies are
read when they are added or moved. With 1001 moving bodies on Windows (MSVC), a snapshot takes
about 0.18 ms against a 2.1 ms step (`physics_world_test`). A `world_binding` publishes one to
the state tree as it steps (below), so any thread can read the latest without taking the
world's lock.

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
  - The transmission shifts on its own; `reverse` backs up.
  - A vehicle moving the wrong way brakes first.
  - One with no command (|a_cmd| at most 0.05) and slower than 0.3 m/s holds on the full
    brake, so it stands on a grade its tires can hold; released (`neutral`, no brake) it
    rolls down at g (sin θ − C_rr cos θ). A change of input wakes a sleeping vehicle.
  - `set_controls` takes pedals directly (and `neutral`).
- **Tires.** Slip curves peak at 1.0 times the μ of what the tire touches: the material of the
  terrain cell or mesh face under it, the table every plant reads.
- **Resistance**, applied to the chassis each step from the last step's wheels:
  - aero, −½ ρ C_d A |u| u, with u relative to a flow field's wind where one covers the chassis
    (a vehicle's chassis is not dragged by flow fields otherwise);
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
- **Save and restore** carry the actuators and the driver's memory, so a replay reproduces a
  drive (`physics_vehicle_test`).
- **Tuning a running vehicle.** `get_vehicle(id)` returns its description and `set_vehicle(id, d)`
  changes it from the next step on. These values may change:
  - the steering: `max_steer`, `max_steer_rate`;
  - the driver: `max_accel`, `max_decel`;
  - the resistance: `drag_coefficient`, `frontal_area`, `air_density`, `rolling_resistance`;
  - the powertrain: `engine_max_torque`, `engine_min_rpm`, `engine_max_rpm`, `shift_up_rpm`,
    `shift_down_rpm`;
  - each axle: `max_brake_torque`, `max_hand_brake_torque`, `suspension_frequency`,
    `suspension_damping`, and `anti_roll_stiffness` on an axle that was added with a bar.

  A change to anything else (the chassis, the mass, the wheels' geometry, the gearing, the
  tire curves) throws `cvc::physics_error` naming it and changes nothing. Like materials, a
  tuning is not part of `save()`.

## Static world geometry: `add_world_geometry`

```cpp
#include <cvc/physics/world_geometry.h>
#include <cvc/world/city_bundle.h>

const auto city = cvc::world::load_city_bundle(dir);         // terrain + buildings
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
invalid value is ignored and never thrown into a frame.

| Keys | Read | Notes |
|---|---|---|
| `backend`, `fixed_dt`, `threads`, `determinism`, `max_bodies`, `max_body_pairs`, `max_contact_constraints`, `origin.{x,y,z}` | `config_from_state` | A later change sets `restart_required` and leaves the running world alone |
| `gravity.{x,y,z}`, `collision_steps`, `velocity_steps`, `position_steps`, `penetration_slop`, `friction_combine`, `restitution_combine`, `contacts` | every `pull()` | Applied with `reconfigure` |
| `materials.<name>.{friction, restitution, rolling_resistance, nav_material}` | every `pull()` | A new child adds a material, and a changed value updates it. `default` is material 0. |
| `publish_every` | every `pull()` | `step()`/`follow()` push every N steps (default 1) |
| `hash_every` | every `pull()` | Publish `state_hash` every N steps; 0, the default, means never |
| `snapshot_every` | every `pull()` | Publish the snapshot every N steps (default 1); 0 means never |
| `tick`, `time`, `step_ms`, `bodies`, `active_bodies`, `contact_events`, `saturated`, `version`, `restart_required`, `state_hash` | written by `push()` | Telemetry |
| `snapshot` | written by `push()` | The `world_snapshot` as `std::shared_ptr<const world_snapshot>` in the node's `data()`. `world_binding::snapshot_from_state(app)` reads it from any thread |

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
| `vehicles.<name>.{chassis, speed, steer, steer_command, throttle, brake, hand_brake, gear, engine_rpm, position.*, rotation.*, velocity.*, angular_velocity.*, aero_force.*, air_velocity.*}` | written by `push()` | The `vehicle_state` |
| `vehicles.<name>.wheels.<i>.{steer, angular_velocity, suspension_length, contact, contact_point.*, load, longitudinal_slip, lateral_slip, friction, rolling_resistance, medium_depth, sinkage, rut, medium_resistance}` | written by `push()` | Each `wheel_state` |
| `vehicles.<name>.command.{steer, accel, reverse}` | every `pull()` | A change calls `world::drive` |
| `vehicles.<name>.controls.{steer, throttle, brake, hand_brake, reverse, neutral}` | every `pull()` | A change calls `world::set_controls` |
| `vehicles.<name>.tuning.<value>`, `tuning.axles.<i>.<value>` | every `pull()` | `world::set_vehicle`, with the values above |
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
- **Cost.** A key write costs a few microseconds, so a published vehicle (about 90 keys) adds
  about 0.34 ms to a push. `publish_every` throttles it. In-process readers can use the
  snapshot instead.

Each binding belongs to one app and one world, so two apps configure two worlds independently.
