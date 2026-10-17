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
| `cvc/physics/world_binding.h` | `world_binding`: the state-tree face of a world |

The physics headers include only the standard library, except `world_binding.h`, which needs
`cvc::app`. No header names an engine.

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

Jolt's process globals (allocator hooks, factory, type registry) exist while at least one Jolt
world does. The first world sets them up and the last one tears them down. If the application set
up Jolt itself, that setup is used and left in place. libcvc keeps no singleton of its own.

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
| `tick`, `time`, `step_ms`, `bodies`, `active_bodies`, `contact_events`, `saturated`, `version`, `restart_required`, `state_hash` | written by `push()` | Telemetry |

Each binding belongs to one app and one world, so two apps configure two worlds independently.
