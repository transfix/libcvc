# Your own multi-car controller (`pycvc_gl.cars`)

`pycvc_gl.cars` drives N Ackermann cars with a controller you write, in Python. The cars run on
one of these plants:

- nav's kinematic bicycle;
- Jolt vehicles;
- the dynamic single track.

They drive in one of three worlds:

- a city, from an Ariadne document's `data:` block;
- a world from libcvc's terrain generator;
- a flat square.

An Ariadne document shows them, with panels, sliders that feed your parameters, rasters around a
car and plots of your controller's coefficients. The same run works headless, for batch
experiments.

It comes as a template to copy: one Python file and one `.ari` document.

```bash
cvcpkg install pycvc-gl-cp312 --prefix ./cars-env     # or cp311 / cp313 for your Python
. ./cars-env/bin/activate                              # Windows: .\cars-env\Scripts\activate
python -m pycvc_gl.cars --copy my_cars                 # my_cars/cars.py and my_cars/cars.ari
python my_cars/cars.py                                 # Austin, 6 Humvees, the template's controller
python my_cars/cars.py --headless --frames 1200 --log run.npz      # 60 s, no GL
```

`python -m pycvc_gl.cars` runs the package's own copy. The first run downloads Austin
(`scene-austin-south-small`, about 10 MB) from cvcpkg.org into the data cache, and later runs read
the cache.

Contents:
- [Your controller](#your-controller)
- [The world](#the-world)
- [The plant](#the-plant)
- [What each plant means for your controller](#what-each-plant-means-for-your-controller)
- [Settings: the state tree](#settings-the-state-tree)
- [The panels](#the-panels)
- [Batch runs](#batch-runs)
- [From a script](#from-a-script)
- [Your own host](#your-own-host)
- [Known limits](#known-limits)

## Your controller

Edit `controller(s, p)` in `cars.py`. It runs every tick (0.05 s), once for all the cars, and
returns `(steer, accel)`: one front-wheel angle (rad, + left) and one acceleration along the
heading (m/s²) per car, as numpy arrays. Each car's plant takes them in the navigator's place
(`pycvc.nav_sim_world_override`), held to the car's steering lock, its acceleration and braking,
and its top speed. A NaN leaves that car to the navigator for the tick.

`s` holds every car's state as (n,) arrays, in SI units:

| field | meaning |
|---|---|
| `x`, `y`, `heading` | the rear axle's midpoint (the point the bicycle turns about), and the heading (0 along +x, counter-clockwise) |
| `speed`, `vx`, `vy` | the speed along the heading (negative backing up), and the velocity in the world |
| `steer` | the front wheels' angle now; on a vehicle it lags your command at the steering's rate |
| `goal` (n, 2), `goal_dist`, `reached` | each car's goal, the distance to it, and whether it is within `s.reach` |
| `ranges` (n, R), `ray_angles` (R,) | the distance to anything blocked (buildings, hard ground, deep water) along each ray, at angles off the heading, up to `range_max` |
| `collided` | touching something blocked or another car this tick |
| `max_steer`, `max_accel`, `max_decel`, `max_speed` | each car's limits |
| `length`, `width`, `wheelbase` | each car's size |
| `t`, `dt`, `n`, `pos` (n, 2), `driving`, `world` | the time, the tick, the number of cars, the positions, the plant, the world |
| `nav` | the navigator's per-car telemetry while it drives (below) |

The other cars are the rest of the arrays: car i's neighbours are every j != i.

`p` is your parameters. `PARAMS` in `cars.py` names them and their starting values, and the
document's Controller window has a slider bound to each `ctl.<name>`. A slider's value reaches
`p["<name>"]` on the next tick. To add a parameter, add it to `PARAMS` and a `slider_float` bound
to `ctl.<name>` to `cars.ari`.

A third return value, `{"coeffs": {name: (n,) array}}`, plots up to four of your controller's
terms for the chosen car and logs them all. `cost(s, p, i, X, Y)` (optional, in `cars.py`) is the
field your controller descends for car i, at the world points (X, Y). The Rasters window draws it,
and the log keeps it.

The shipped controller heads for the goal, keeps a spacing from the other cars, turns away from
anything blocked along its rays, slows into turns, for the goal and for what is ahead, and backs
out when wedged. It is a starting point, not a planner: in a city it can stall in a dead end.

## The world

`run.world` (the Run window's World combo, or `--world`) picks one:

- **`data` (the default)**: the document's `data.city`.
  - `cars.ari`'s `data:` block names `cvcpkg://cy-pca/scene-austin-south-small`, Austin at its
    real size.
  - The scene draws its terrain with the aerial image draped, and its buildings.
  - Run's URI field and Load button point `data.city` at another source: a `cvcpkg://ORG/NAME`
    package, an https archive, or a local bundle directory (`terrain.json` and `buildings.glb`).
    `--uri` does the same from the command line.
  - `run.crop` (m) crops it to a square around its centre.
- **`generated`**: libcvc's terrain generator (`pycvc.generate_world`, terrain_lab's world).
  - The World window sets its parameters, the `world.*` keys: `seed`, `half`, `amp` (relief),
    `water_level`, `buildings`, `trees`, `rocks`, `city` (`districts` or `none`) and `terrain`.
  - Regenerate builds the world again, and the run with it.
  - Python generates the world for the cars, the navigator and the physics. The scene's
    `generated_world` node draws the same world from the same keys: the generator is a pure
    function of them and of the building library (`$CVC_NAV_BUNDLE`'s, else procedural).
- **`flat`**: an open square 40 car lengths across, walled at its edge.

Every world gives the cars the same things:
- an occupancy grid (`pycvc.nav_world_rasterize`: buildings, hard ground and deep water blocked);
- the terrain's height;
- the `world_geometry` the physics collides with: the city's terrain and buildings, or the
  generated world's.

So a controller cannot tell a package from a generated world. The cars start on free ground and get
goals at least 0.3 of the world's span away; a car that reaches its goal gets a new one while
`run.regoal` is on.

## The plant

`nav_plant.vehicle` picks the plant. It is libcvc's `plant_host` knob (`pycvc.plant_host`):
- `kinematic`: nav's bicycle;
- `humvee` or `roboracer`: Jolt vehicles;
- a comma list, cycled over the cars (`humvee,roboracer`);
- `dynamic` or `dynamic:<presets>`: the presets' dynamic single track;
- `dynamic4w[:presets]`: their four-wheel model with suspension.

The Vehicle combo, the document's `init:`, `--vehicle`, `--set nav_plant.vehicle=...` and Python
(`host.set`) all set it. A change switches the plant on the next tick: the cars keep their poses,
start at rest in a new physics world, and the readouts and drawn bodies follow. The physics world
holds the world's own terrain and buildings, so the vehicles drive the relief and collide with the
buildings and with each other.

The Humvee is the default because Austin is at its real size. The RoboRacer, the lab's 1/10 car
(0.324 m wheelbase, 10 m/s), suits the flat square.

## What each plant means for your controller

- **Kinematic (nav's bicycle).**
  - The wheels point where you steer at once.
  - The speed integrates your acceleration exactly, between -0.25 and 1 times the top speed.
  - The yaw rate is `speed / wheelbase * tan(steer)`. The bicycle's size and limits are
    `nav.vehicle.wheelbase`, `delta_max`, `vmax` and `a_max`, the Humvee's by default.
  - Nothing slips and nothing collides: a car drives through a building or another car, and
    `collided` says so.

  Use it to get the logic right.
- **Jolt.** Each car is a rigid body on four wheels, with suspension, an engine and gearbox, and
  brakes. Three things differ from the bicycle:
  - *The steering has a rate.* `s.steer` follows your command at the vehicle's steering rate:
    measured, a RoboRacer reaches 90% of full lock in 0.15 s, a Humvee in 1.2 s. Gains that are
    stable on the bicycle can oscillate here.
  - *A driver model makes your acceleration.* It works throttle, brake and gear to give the
    acceleration you ask for. From rest it tracks it closely: 2 m/s² gives 1 m/s at 0.50 s on a
    RoboRacer and 0.60 s on a Humvee, against the bicycle's 0.50 s. But the engine's torque and
    the tires' grip cap what it can deliver.

    A negative acceleration from rest backs the vehicle up. The navigator's stopping limit allows
    for `nav_plant.latency` (0.1 s): the command itself is applied within the tick.
  - *The tires slip.* Below about 0.2 g of cornering a car turns as the bicycle predicts. Near
    the grip limit it turns less: measured at about 0.8 g on full lock, a RoboRacer at 3 m/s yaws
    at 0.64 of the bicycle's rate, and a Humvee at 8 m/s at 0.85. Past the limit it slides, and
    braking distances grow with speed squared over the grip.

    Leave room in your spacing. `s.vx`, `s.vy` against `s.heading` give the sideslip.
- **Dynamic.** A single-track model with Pacejka tires, load transfer and actuator lag, a box per
  chassis in a Jolt world for collisions. Near the limit it turns about as the Jolt car does.
  `dynamic4w` gives each wheel its own tire and suspension.

## Settings: the state tree

Every setting a researcher tunes is a state-tree node under the document's prefix. They are
libcvc's own keys: `sim_world::pull` reads `nav.*` and `nav.vehicle.*`, and `plant_host` reads
`nav_plant.*`. Each key carries its comment, unit and range (`pycvc.state_comment`,
`state_property`; `pycvc_gl.cars.settings(app, prefix)` lists them). You can set them in four
places:
- in the document's `init:`, with `(state-set "nav.vehicle.vmax" "12")`, so a `.ari` file carries
  a whole configuration;
- on a slider or combo bound to the key (the Settings window binds the common ones);
- from the command line, with `--set nav.vehicle.d_hat=5` (repeatable) or `--params run.json`
  (`{"nav.vehicle.vmax": 12, ...}`);
- from Python, with `host.set(...)`.

When a change applies:

| keys | applies |
|---|---|
| `nav.*`, `nav.vehicle.*`, `nav_plant.*` (but `vehicle`), `ctl.*`, `run.ticks`, `run.regoal`, `run.paused`, `view.*` | on the next tick |
| `nav_plant.vehicle` | the plant switches on the next tick; the cars keep their poses |
| `run.cars`, `run.world`, `run.controller`, a new `data.city` (Load) | the run is rebuilt: new cars, starts and goals |
| `world.*` | when Regenerate is pressed (or the world is next built) |

An unset key is seeded with the built world's value, so the panel starts at the effective setting.
The run log records every key's value at the end of the run.

Most of these settings tune libcvc's navigator, which drives only under `--controller navigator`
or for a car your controller leaves to it (NaN). What constrains your own commands:
- the lock and the top speed: `nav.vehicle.delta_max` and `vmax` on the bicycle, the lower of
  `vmax` and the vehicle's own on a plant;
- on a vehicle, `nav_plant.*`.

With yielding on (`nav.yield.*`), `nav.yield.steer_overrides` 1 also nudges your steer toward the
yielding shift. Set it to 0 to leave your command as it is.

The knobs, with the values a Humvee run starts from:

**The navigator's vehicle and nav's bicycle (`nav.vehicle.*`)**

| key | unit | range | Humvee default | what it is |
|---|---|---|---|---|
| `nav.vehicle.a_lat_max` | m/s^2 | 0.5 .. 15 | 4.9 | The sideways acceleration the corner speed limit allows |
| `nav.vehicle.a_max` | m/s^2 | 0.5 .. 15 | 2 | The acceleration and braking the drive commands, and the braking its stopping speed limit plans with |
| `nav.vehicle.allow_reverse` |  | 0 .. 1 | 1 | 1: the drive may back up, at up to a quarter of vmax, for example out of a dead end; 0: forward only |
| `nav.vehicle.body_gain` | 1 | 0 .. 1 | 1 | A scale on the multi-disc footprint's summed wall barrier |
| `nav.vehicle.body_rr` | m | 0 .. 5 | 0 | The disc radius of the multi-disc footprint (discs along the body), used in place of rr when the app sets one up |
| `nav.vehicle.d_hat` | m | 0.5 .. 30 | 3 | How far from a wall the barrier that pushes the vehicle away reaches; beyond it a wall exerts no push |
| `nav.vehicle.delta_max` | rad | 0.1 .. 1 | 0.457 | The steering lock: the largest angle the bicycle model's front wheel turns |
| `nav.vehicle.k_steer` | rad | 0 .. 2 | 0.8 | How much the push from walls, material and terrain adds to the steering: the most it can add, in radians |
| `nav.vehicle.mass` | kg | 1 .. 20000 | 1 | The vehicle's mass, carried for the fuel model and telemetry; the kinematic drive does not use it |
| `nav.vehicle.mu_lookahead` | m | 0 .. 40 | 6 | How far toward its carrot (the point it steers toward) a vehicle looks for the worst grip, for a grip-aware steering policy |
| `nav.vehicle.mu_probes` |  | 1 .. 8 | 3 | Grip samples along mu_lookahead |
| `nav.vehicle.nsub` |  | 1 .. 16 | 5 | Integration substeps per tick for the kinematic drive |
| `nav.vehicle.risk_governor` |  | 0 .. 1 | 0 | 1: material risk (the worse of the material's and the terrain's) caps the speed, so vehicles slow on and before risky ground; 0: risk only steers |
| `nav.vehicle.risk_lookahead` | m | 0 .. 80 | 0 | How far ahead along its travel the risk governor looks, so the vehicle slows before it reaches risky ground |
| `nav.vehicle.risk_onset` | 1 | 0 .. 0.95 | 0.3 | The risk below which the risk governor does not slow the vehicle |
| `nav.vehicle.risk_probes` |  | 1 .. 8 | 3 | Risk samples along risk_lookahead |
| `nav.vehicle.risk_vmin` | m/s | 0 .. 40 | 0 | The risk governor's speed at risk 1 |
| `nav.vehicle.rr` | m | 0.1 .. 5 | 1.2 | The radius of the single-disc footprint, centred on the rear axle, that clearances to walls are measured from |
| `nav.vehicle.track_width` | m | 0 .. 3 | 0 | The distance between the left and right wheels |
| `nav.vehicle.vmax` | m/s | 0.5 .. 40 | 24.6 | The top speed |
| `nav.vehicle.wheelbase` | m | 0.2 .. 8 | 3.3 | The distance between the axles in nav's bicycle model; with the steering lock it sets the tightest turn |

**Yielding between agents (`nav.yield.*`)**

| key | unit | range | Humvee default | what it is |
|---|---|---|---|---|
| `nav.yield.clearance` | m | 0 .. 30 | 0 | The half-width of the corridor round an announced path that others move out of |
| `nav.yield.cooperation` | 1 | 0 .. 1 | 0.5 | How far an agent passing another counts on that one moving aside |
| `nav.yield.creep_need` | m | 0 .. 30 | 0 | The smallest shift out of another's path that makes a stopped or held agent creep (creep_speed) |
| `nav.yield.creep_speed` | m/s | 0 .. 5 | 0 | An agent that must move out of another's path but is stopped or held creeps forward at this speed so it can steer aside |
| `nav.yield.gain` | 1 | 0 .. 4 | 1 | How far an agent shifts its carrot (the point it steers toward) out of another's path, per metre it is inside that path's corridor |
| `nav.yield.horizon_s` | s | 0 .. 10 | 0 | How far ahead agents announce the paths they mean to drive, and how soon a conflict on one must come to count |
| `nav.yield.min_len` | m | 0 .. 100 | 0 | The shortest path an agent announces, however slowly it moves |
| `nav.yield.min_speed` | m/s | 0 .. 10 | 1 | The speed below which arrival times are worked out as if at this speed, so a slow or stopped agent's conflicts still count |
| `nav.yield.near_dist` | m | -1 .. 100 | 0 | Along a path this close, a conflict counts whatever the speeds, so a stopped column still clears for one starting behind it |
| `nav.yield.pass_gain` | 1 | 0 .. 4 | 1 | How far an agent shifts its carrot to steer round another in its own path, per metre that one is inside its corridor |
| `nav.yield.pass_hold_s` | s | 0 .. 30 | 0 | How long an agent held behind one standing still waits before it is released to drive on (the higher priority first); released, it may push into the one ahead |
| `nav.yield.pass_time_s` | s | 0 .. 10 | 1.5 | Speed cap with another in its path: an agent drives no faster than would bring it to that one in this time, and stops short of one that stays put |
| `nav.yield.steer_look_s` | s | 0 .. 5 | 0.5 | For an agent the host steers: how far ahead, in seconds at its speed, the yielding correction aims |
| `nav.yield.steer_overrides` |  | 0 .. 1 | 1 | 1: an agent the host steers itself (on a route, for example) also steers toward the yielding shift while driving forward; 0: the host's steer is left as commanded |

**The plant (`nav_plant.*`)**

| key | unit | range | Humvee default | what it is |
|---|---|---|---|---|
| `nav_plant.brake_margin` | 1 | 0.3 .. 1 | 0.8 | The share of each vehicle's maximum braking the navigator plans its stops with |
| `nav_plant.drift_nsub` |  | 0 .. 64 | 0 | Substeps of nav's kinematic bicycle model in the per-tick prediction the vehicle's motion is compared with (the drift) |
| `nav_plant.latency` | s | 0 .. 0.5 | 0.1 | How long the vehicle takes to respond to a command, beyond the tick it holds it (driver, pedals, engine) |
| `nav_plant.modulation.tip_gain` | 1 | 0 .. 10 | 0 | How fast the speed damping and the barriers' weight grow per unit of load transfer past modulation.tip_onset |
| `nav_plant.modulation.tip_onset` | 1 | 0.1 .. 0.9 | 0.4 | The load transfer (the left-right difference in wheel load over the total; 1 = one side unloaded) past which the drive slows the vehicle and stiffens its barriers |
| `nav_plant.modulation.tire_gain` | 1 | 0 .. 10 | 0 | How fast the drive's speed damping grows per unit of tire use past modulation.tire_onset |
| `nav_plant.modulation.tire_onset` | 1 | 0.3 .. 1.2 | 0.7 | The tire use (the most used tire's combined slip over its peak; 1 = at the grip limit) past which the drive slows the vehicle |
| `nav_plant.plan.accel_margin` | 1 | 0.3 .. 1 | 0.8 | With plan.with_vehicle: the share of each vehicle's maximum acceleration the navigator plans with |
| `nav_plant.plan.lateral_mps2` | m/s^2 | 1 .. 12 | 4.9 | With plan.with_vehicle: the sideways acceleration the navigator corners at, capped at plan.tip_margin of the vehicle's rollover threshold |
| `nav_plant.plan.tip_margin` | 1 | 0.3 .. 1 | 0.8 | With plan.with_vehicle: the share of each vehicle's rollover threshold (its static stability factor, half its track over its centre of gravity's height, in g) that cornering may reach |
| `nav_plant.plan.with_vehicle` |  | 0 .. 1 | 1 | 1: the navigator plans with each vehicle's own acceleration, top speed, cornering and size; 0: with nav.vehicle's a_max, vmax, a_lat_max and footprint |
| `nav_plant.vehicle` |  |  | humvee | What the agents drive: kinematic (nav's bicycle), a vehicle preset on Jolt (humvee, roboracer), a comma-separated list of presets cycled over the agents, dynamic / dynamic:<presets> for the presets' dynamic single-track model, or dynamic4w / dynamic4w:<presets> for their four-wheel model with suspension |

**Sensing, arrival and the crowd (`nav.*`)**

| key | unit | range | Humvee default | what it is |
|---|---|---|---|---|
| `nav.range_m` | m | 5 .. 300 | 60 | How far each agent's sensor sees when it maps its surroundings |
| `nav.reach_tol` | m | 0.5 .. 20 | 7.26 | How close to its goal an agent counts as arrived; it then stops and parks |
| `nav.sense_every` | tick | 1 .. 30 | 4 | Ticks (one sim step each) between looks; the obstacle map is rebuilt after each |
| `nav.sep_gain` | m | 0 .. 10 | 0 | How hard nearby agents push each other's carrots apart: how far a neighbour at contact shifts an agent's carrot, falling off to nothing at sep_radius |
| `nav.sep_radius` | m | 0 .. 20 | 0 | Agents closer than this push their carrots (the points they steer toward) apart, so a crowd steers round itself |
| `nav.ttl_s` | s | 0 .. 60 | 4 | How long an obstacle added at run time stays on the map |

The template's own keys: `run.world`, `run.uri`, `run.data_path`, `run.crop` (m), `run.cars`,
`run.controller` (`yours`, `navigator`), `run.ticks` (ticks per frame), `run.regoal`,
`run.paused`, and `view.camera`, `view.car` (the chosen car), `view.horizon` (s of predicted
path), `view.raster_m` (the raster window, m), `show.plans`.

## The panels

Settings, World, Rasters, the coefficient plots and Run's world choice are units in the package's
`cars_panels.ari`, which `cars.ari` imports ([Your own host](#your-own-host)). To change one, put
a window of your own in its place in your copy of `cars.ari`.

- **Run**: the world (and a city's URI), the vehicle, the number of cars, the controller (yours,
  or libcvc's navigator as a baseline), ticks per frame, a new goal on arrival, pause, the camera
  and the chosen car. New goals and Reset are document buttons that call back into Python
  (`host.on`).
- **Settings**: the common navigator, vehicle and plant knobs, with their keys in the tooltips.
- **Controller**: your parameters (`ctl.*`) and how far ahead to predict.
- **World**: the generator's parameters and Regenerate (with World: Generated).
- **Cars**: the run's numbers, two plots (the chosen car's speed, the closest gap between two
  cars), four coefficient plots, and a row per car.
  - The coefficients are your controller's `info["coeffs"]`.
  - Under the navigator they are its telemetry: the policy's coefficients `alpha`, `beta` and
    `gamma`, `lam_soft`, `lam_hard`, `potential`, `clearance`.
- **Rasters**: around the chosen car, heading up, `view.raster_m` across.
  - The left one is the signed distance to anything blocked, with the car's predicted path
    (yellow), its goal (green) and the other cars (red).
  - The right one is your `cost()` field, or under the navigator its belief map.

  The predicted path is `pycvc_gl.cars.rollout`: your controller driving every car forward on
  nav's bicycle for `view.horizon` seconds. It is what the controller plans, not what the plant
  will do. The scene draws it too.
- **The scene**: each Humvee (and each car on nav's bicycle, which has a Humvee's body) is the
  Humvee's model, with a tag in the car's colour above it. The model is an installed
  `vehicle-humvee`'s, else the Austin package's `Humvee.glb` (from the city, or asked for as
  `data.vehicles` when another world is in use; from the data cache after the first download).
  Until it is there, and for a RoboRacer, a car is a box its size.

## Batch runs

```bash
python my_cars/cars.py --headless --frames 1200 --log run.npz
python my_cars/cars.py --headless --frames 1200 --world generated --set world.seed=4 \
    --vehicle roboracer --set nav_plant.latency=0.2 --log run.csv
python my_cars/cars.py --headless --frames 600 --controller navigator --min-arrivals 3
```

`--headless` runs the same controller, cars and document state with no window and no GL.
`--log` writes, in `.npz`:
- **Per tick, per car** (arrays of shape (ticks, cars)): `x`, `y`, `heading`, `speed`, `steer`,
  `vx`, `vy`, `goal_x`, `goal_y`, `goal_dist`, `reached`, `collided`, and the commands after the
  limits, `cmd_steer` and `cmd_accel`. `t` holds the ticks' times.
- **The coefficients:** `coef_<name>` (ticks, cars).
- **Every `--raster-every` ticks (10)**, the chosen car's rasters and every car's predicted path:
  - `raster_sdf`, `raster_occ` and `raster_cost` (or `raster_belief`), each (k, 96, 96);
  - `raster_plan` (k, cars, steps, 2);
  - `raster_t`.
- **`meta`**, a JSON string: the world (its source and URI, or the generator's parameters), the
  seed, the controller, every setting's value, your parameters, the `world.*` keys, the arrivals
  and collisions, and any error.

`.csv` writes one row per car per tick and puts the rest in a `.npz` beside it. `--seed` makes the
starts and goals repeatable. With the generator's parameters and the settings in `meta`, a run can
be reproduced.

The exit code is:
- 0 for a run that finished;
- 1 when the controller raised, or the run could not start;
- 3 when `--min-arrivals N` was not met.

## From a script

The pieces work without the document:

```python
from pycvc_gl import cars as C

world = C.world_from_generated(C.generate({"seed": 4, "half": 300, "amp": 20}))
cars = C.Cars(world, 8, "roboracer", seed=3)
log, s = C.CarsLog(), cars.state()
while s.t < 60.0:
    s = cars.step(*my_controller(s))
    log.record(s, cars.last_command)
log.save("run.npz")
```

`C.world_from_data(app, "<prefix>.data.city")` reads a city from an app's data store.
`C.flat_world(unit)` gives a flat square. `Cars(..., navigator=True)` with `step_navigator()`
runs libcvc's navigator. `nav_weights=` gives it a `.cvcnav` policy and `scale=` the frame
that policy was trained in (normalized units per metre; `--nav-weights` and `--nav-scale` on
the command line). `Cars(..., app=app,
prefix=...)` keeps its settings in that app's state tree (`pull()` and `publish()`).
`Cars(..., starts=..., goals=...)` places the cars ((n, 2) world positions) instead of picking
random ones.

## Your own host

A host with its own loop, its own document, or cars stepped some other way (its own navigator,
its own simulator) uses the template's pieces rather than copying them. GRL-SNAM's Ariadne ports
are built this way.

- **Panels.** `cars_panels.ari` holds the template's panels as units:
  - `cars_settings` (window): the navigator, vehicle and plant knobs;
  - `cars_world` (window): the generator's parameters, shown with World: Generated;
  - `cars_world_choice` (group): the World combo, the city's URI and Load;
  - `cars_coefficients` (group): four coefficient plots;
  - `cars_rasters` (window): the two pictures around the chosen car (arg `cost`: the second
    one's caption).

  Pass `C.COMPONENTS` as a component path and import the file by `cvc://`. A window's `pos:` and
  `size:` at the include site place it:

  ```python
  host = Host(mode=args.mode, name="mine", component_paths=[C.COMPONENTS])
  ```

  ```yaml
  import: cvc://cars_panels.ari
  windows:
    - include: cars_settings
      pos: [16, 422]
    - window: Mine
      children:
        - include: cars_coefficients
  ```

  The units bind the template's keys, so the document needs those keys too: `run.world`,
  `run.uri`, `run.data_path`, `world.*`, `view.car`, `view.raster_m`, and a scene with a
  `city` node (`source: { data: city }`) and a `generated_world` node named `gen`.
- **Flags and settings.** `C.add_args(parser)` adds `--world`, `--uri`, `--data-path`, `--set`,
  `--params`, `--log`, `--raster-every` and `--min-arrivals` to an `argparse` parser.
  `C.apply_settings(host, args, flags)` applies them over the document's `init:`:
  - the `--params` file first;
  - then each `--set`;
  - then `flags` ({key: value}; a None value is skipped);
  - then `--world`.

  Last, `--uri` points `data.city` at that world.
- **Worlds.** `C.make_world(host, kind)` builds the world `run.world` names, from the document's
  `data.city`, its `world.*` keys or a flat square. `C.show_world(host, kind, doc)` shows only
  that one in the scene.
- **State.** `C.state_from(world, x, y, heading, speed, goal, ...)` makes a `CarsState` from your
  own arrays, with its ranges and contacts worked out on the world. `CarsLog`, the pictures and the
  panels then take your cars as they take `Cars`'.
- **Pictures and plots.** `C.raster_arrays(world, s, k, size_m, belief=, cost=)` gives car `k`'s
  rasters as the log keeps them. `C.raster_pictures(r, s, k, size_m, path=)` turns them into the
  `cars_rasters` panel's two pictures (publish each with `host.publish_image`).
  `C.plot_coefficients(host, history, coeffs, k)` fills the `cars_coefficients` plots. `C.chosen(host, n)` is
  the chosen car, and `C.raster_size(host, length)` the pictures' width.
- **The end of a run.** `C.save_log(log, host, path, **meta)` saves the log with every setting
  and the `world.*` keys in its meta. `C.exit_code(code, error=, arrivals=, min_arrivals=)`
  gives the template's exit code: 0, 1 or 3.

## Known limits

- **A window loads Austin at start, whatever the world.** The scene's city node asks for its
  dataset when the document opens. It comes from the cache after the first run.
- **One shared steering lock and wheelbase on the bicycle.** With a mixed list of presets, nav's
  bicycle (for the navigator) uses one wheelbase and lock: the mean wheelbase and the smallest
  lock. Each vehicle on a plant has its own.
- **The navigator is tuned for its policy's bodies.** As a baseline on vehicles it often does
  worse than a controller made for them. GRL-SNAM's docs/PLANTS.md says why.
- **The template hides the unused world itself.** Before transfix/libcvc#1037, a scene node's
  `visible:` hid that node alone: cvcGL kept what a data node drew (its children), and the
  generated_world node's meshes (nodes beside it), on screen. So `show_world()` clears
  `data.city` and hides the generated world's nodes by name, which libcvc builds with #1037 no
  longer need but which does no harm.
