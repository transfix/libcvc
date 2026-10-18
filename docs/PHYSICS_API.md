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
| `cvc/physics/nav_plant.h` | `nav_plant`: a world's vehicles as `cvc::nav`'s plant (`cvc/nav/vehicle_plant.h`) |
| `cvc/physics/nav_terrain_binding.h` | A `sim_world`'s terrain (P5) in the state tree |

The physics headers include only the standard library, except:

- `world_binding.h`, which needs `cvc::app`;
- `world_geometry.h`, which reads `cvc::world`;
- `nav_plant.h`, which implements `cvc::nav`'s plant interface.

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
    `shift_down_rpm`, `shift_up_rpm_light`, `shift_down_rpm_light`, `engine_friction_curve`,
    `clutch_strength`, `driveline_efficiency`;
  - each axle: `max_brake_torque`, `max_hand_brake_torque`, `suspension_frequency`,
    `suspension_damping`, and `anti_roll_stiffness` on an axle that was added with a bar.

  A change to anything else (the chassis, the mass, the wheels' geometry, the gearing, the
  torque and tire curves, the energy source) throws `cvc::physics_error` naming it and changes
  nothing. Like materials, a tuning is not part of `save()`.

  Through the tree an invalid tuning value is ignored on its own; the other knobs still apply.

### The presets

`vehicle_desc::humvee()` is an M1097A2 HMMWV and `vehicle_desc::roboracer()` the UT-AV lab's
RoboRacer. In `vehicle.cpp` every value names its source:

- [AMG], AM General's M1097A2 specification sheet;
- [TM], TM 9-2320-280-10;
- [DTIC], the Army's dynamometer tests of the 6.5 L engine (TARDEC TFLRF, DTIC ADA449196);
- [lab], the lab's configuration (ut-av.pages.dev, ut_automata's `vesc.lua`);
- [trx], Traxxas's listing for the chassis;
- [doc31], the scope's spec table;
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
- **The RoboRacer.**
  - The car: 3.5 kg, a 0.324 m wheelbase, 108 mm tires and shaft-driven 4WD.
  - The motor: an ARRMA 3800 Kv motor at the VESC's current limit (an estimated 60 A, so
    0.15 N m), flat to the VESC's 10 m/s speed limit, geared 15.15:1 to the wheels
    ([derived] from `speed_to_erpm_gain`).
  - The battery: a 3S LiPo. Its 55.5 Wh is an estimate: read the pack.
  - Braking is the VESC's, an estimated 0.57 N m a wheel, 70% of it regenerated up to 150 W.

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
| `fields` | Work the fields did on the chassis (buoyancy, a current) | At the mean velocity |
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
  - the creep floor never exceeds the stopping speed.
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
  accel_command, drift.{position_m, heading_rad, speed_mps, lateral_g, stop_speed_mps}}`, next
  to the physics binding's `vehicles.<name>.*` and its energy keys;
- `<prefix>.nav_plant.drift.<b>.{lo_g, hi_g, ticks, position_rms_m, position_max_m,
  heading_rms_rad, heading_max_rad, speed_rms_mps, speed_max_mps}`;
- `<prefix>.nav_plant.{vehicles, advance_us, advance_us_per_vehicle}`, the cost;
- `<prefix>.nav_plant.settings.{step_world, ground_z, spawn_height, metres_per_world_unit,
  physics_steps, drift_bins_g}`, the settings fixed at construction.

**Knobs.** `pull(ctx, prefix)` reads `<prefix>.nav_plant.{latency, brake_margin, drift_nsub}`.
These are seeded with the plant's values, applied when they change, and an invalid one is
ignored. They are also `set_latency`, `set_brake_margin` and `set_drift_nsub`. The drive reads
the latency and the braking from `limits()` every tick, so a change takes effect on the next
one.

Like the physics binding's, every one of these keys says what it is: a comment, `access`, `unit`
and, on a knob, a recommended `min` and `max`. So do the `nav_stats` leaves that
`publish_nav_stats` writes.

**nav_stats** gains `energy_J`, `energy_per_km` (J per km of path) and `building_contacts`
per vehicle, and `total_energy_J` and `total_building_contacts` per episode. `energy_J` is
what the vehicle drew from its source, net of regeneration.

These fields, and their JSON, appear only when a plant measures energy. The kinematic record
is unchanged, and `fuel_used` keeps its proxy meaning.

cvcGL's `publish_nav_stats` writes them too, again only when tracked:

- `<prefix>.nav_stats.veh.<i>.{energy_J, energy_per_km, building_contacts}`;
- `<prefix>.nav_stats.episode.{total_energy_J, total_building_contacts}`.

Gates (`nav_plant_test`, Windows MSVC, the Humvee in nav's trained metric, 1 unit = 20 m):

| Gate | Measured |
|---|---|
| (1) Every nav golden and parity test unchanged | The kinematic arm is untouched without a plant; the nav suites pass as before |
| (2) Below 0.3 g on flat dry ground, within 0.5 m of the kinematic trajectory over 200 ticks | A curving route, 79 m at up to 0.09 g: the Jolt path strays at most 0.03 m from the bicycle's. At a tick the two differ by up to 0.9 m along the path (the powertrain's response to an instant acceleration). Reported, not gated: a swerve round a block at 0.25 g strays 1.2 m, and a step change of goal 0.58 m, from the Humvee's 0.35 rad/s steering (0.73 and 0.46 m with an instant actuator) |
| (3) At the limit the plants diverge, reported | A 1 g corner cap on mu 0.6: position rms 3 mm below 0.1 g, 0.1 m above 0.5 g, binned in the summary |
| (4) Cost per Jolt vehicle | About 0.1 ms a Humvee a tick (six physics steps) with eight, against 60 ms of real time |
| (5) The adapter residual at nsub = k | An instant steering actuator on mu 1.2: at most 7 mm, 0.009 rad and 0.1 m/s a tick |
| (6) The drift published, binned by lateral g | Per vehicle and per bin, under `vehicles.<name>.nav.drift` and `nav_plant.drift`, with the plant's knobs and settings under `nav_plant` |
| (7) The speed never above the latency-aware v_stop | Driving at a wall at up to 11.5 m/s: at most 0.04 m/s above it at a tick; no contact with the wall |

## Terrain in the navigator's energy (P5)

```cpp
#include <cvc/nav/sim_world.h>
#include <cvc/nav/terrain.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/nav_terrain_binding.h>

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
  `k_sharp` and `d_hat_m`). With `block_infeasible` (the default) the infeasible cells also join
  every belief plane's occupancy, so the SDF's barrier and the governors keep the agents out.
- **H2, gravity in the governors.** With `governors` (the default) the drive samples the slope
  under each agent:
  - the stopping governor plans with `a_stop = max(min(a_max, a_max + g_along), 0.1 a_max)`, so
    with less braking downhill. It never counts on gravity's help uphill, because the command
    never brakes harder than `a_max`;
  - the corner cap loses gravity's pull across the slope;
  - the plant arm's achievable braking is adjusted the same way.

  On the Jolt plant the driver model also feeds gravity along the ground forward. It takes the
  grade from its wheels' contact normals, not the chassis' pitch.
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

**The state tree.** nav itself does not touch the state tree. Its terrain binds from the physics
side, as the plant does:

- `pull_nav_terrain(ctx, prefix, sim)` reads the knobs:
  - `<prefix>.nav_terrain.{max_grade, tip_ratio, tip_margin, sigma}`;
  - `<prefix>.nav_terrain.{lam_grade, lam_grade_hard, k_sharp, d_hat_m}`;
  - `<prefix>.nav_terrain.{block_infeasible, governors}`.

  They are seeded with the world's values. A change rebuilds the planes, and an invalid value
  is ignored.
- `publish_nav_terrain(ctx, prefix, sim)` writes `<prefix>.nav_terrain.{active, gravity, limit,
  infeasible_cells}`.
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
