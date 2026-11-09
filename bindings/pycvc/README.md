# pycvc — Python bindings for libcvc

SWIG bindings that expose libcvc to Python (Phase-13 modernization
deliverable). The goal is to **drive C++ from Python** — build `cvc`
geometry/volume data in Python and hand it to a C++ host (e.g. an embedded
interpreter inside VolumeRover3) — so numerical/research code never has to
be rewritten in C++.

`pycvc` is **general-purpose**: it knows nothing about any downstream
project. Domain code (e.g. a movement-bundle loader) lives in a separate
Python package that imports `pycvc`.

## Status

pycvc covers geometry and volumes (with numpy views), the state tree (`State`),
images, models, world units and clocks, LOD, state_exec (below), and, in the
`pycvc_gl` module, cvcGL scenes, Ariadne and Dear ImGui HUDs. Each area has a
contract test (`test_pycvc_*.py`).

## Design

pycvc wraps the **real** libcvc types under their C++ names; there is no
parallel facade layer (`pycvc.i`). `pycvc.volume` is a `cvc::volume`, `pycvc.State`
is a `cvc::state`, `pycvc.parse` is `cvc::state_exec::parse`. The libcvc headers
are `%include`d and curated with `%ignore` for members that can't cross. A thin
`%extend` layer adds Python conveniences such as numpy views.

Every constructor or operation that needs an app takes it explicitly
(`pycvc.volume(app)`, `pycvc.State.instance(app)`). There is no module-global
"current app". Where C++ takes a `cvc::state&`, Python passes a `pycvc.State`.
The goal is for the Python API to match the C++ API. For state_exec the audit
and plan are in `docs/PYCVC_STATE_EXEC_PARITY.md`.

## Build

Consumes libcvc as an external SDK (`find_package(cvc CONFIG)`), exactly
like VolumeRover3 — so it builds against any installed or cvcpkg libcvc.

```bash
cmake -S bindings/pycvc -B build \
  -DCMAKE_PREFIX_PATH="<cvcpkg-deps-prefix>;<installed-libcvc-sdk>"
cmake --build build
ctest --test-dir build            # runs the smoke test
```

The importable module (`pycvc.py` + `_pycvc.so`) lands in `build/`. At
runtime, `libcvc.so` and its dependency prefix must be on the loader path
(`LD_LIBRARY_PATH`). On Windows, Python ignores PATH for an extension's DLLs:
the ctest entries list the build's and each `CMAKE_PREFIX_PATH` entry's `bin/`
in `CVC_DLL_DIRS`, which `ctest_site/sitecustomize.py` passes to
`os.add_dll_directory`; a script run by hand needs the same calls.

## Example

```python
import pycvc

g = pycvc.Geometry()
g.add_vertices([0,0,0, 1,0,0, 0,1,0])   # flat row-major xyz
g.add_triangle(0, 1, 2)
g.set_colors([1,0,0] * g.num_vertices())
g.compute_normals()
g.save("tri.off")
```

Requires SWIG ≥ 4.0 and Python 3 development headers. Build with the cvcpkg
`swig` (`cvcpkg install swig --prefix <p>`), never a distro one, and name it
explicitly; its binary reports a stale `-swiglib`, so pass the library too:

```bash
export SWIG_LIB=<p>/share/swig/4.4.1
cmake ... -DSWIG_EXECUTABLE=<p>/bin/swig -DSWIG_DIR="$SWIG_LIB" \
          -DCVC_PYCVC_REQUIRE_SWIG_VERSION=4.4   # any other SWIG: configure error
```

Without `-DSWIG_EXECUTABLE`, a `swig` in a `CMAKE_PREFIX_PATH` prefix is taken
(and its `share/swig/<ver>` used as `SWIG_DIR`) before CMake's FindSWIG searches
the system, where its versioned names (`swig4.0`) would otherwise win.

## In-library data prep

The canonical training/twin prep ops are wrapped in-library, so a script never
round-trips through numpy for the whole grid just to reshape or normalize:

```python
v.normalize(0.0, 1.0)          # affine-remap the data range to [0,1] / [-1,1]
v.fill_value(0.0)              # clear before stamping obstacles
v.resample(64, 64, 64)        # resize in place to the network-input grid
patch = v.crop(x, y, z, 16, 16, 16)   # a NEW sub-grid; the source is untouched

n = g.get_normals()           # read compute_normals()'s result out (flat xyz)
g.set_normals([...])          # or stamp authored normals in
g.invert_normals(); g.reorient()
g.extents()                   # mesh AABB as (minx..maxz), like model.extents()
```

(Contract test: `test_pycvc_dataprep.py`.)

## World units & scene dimensions (digital twin)

Beyond geometry/volume/state, pycvc wraps the **world-unit base** so the
training/twin layer works in real metres, kilometres and miles — the Python
counterpart to the C++ `cvc::world_units` (full reference, including the Python
name-reshaping rules, in [`docs/WORLD_UNITS_API.md`](../../docs/WORLD_UNITS_API.md#python-pycvc)):

```python
import pycvc, pycvc_gl

app = pycvc.make_app()
app.world_units().set_regime_name("imperial")   # app-wide display regime
app.world_clock()                               # the app's simulation clock

wu = pycvc.world_units(1000.0)                   # 1 world unit == 1 km
wu.format_d(1000.0, "force")                     # -> value/unit measurement
wu.world_point_to_real(2000, 0, 0)              # -> (x, y, z, unit) coordinate

sg = pycvc_gl.SceneGraph(app)
node = sg.addGraphics("wing", geom)
node.local_to_world([0, 0, 0])                   # point through the transform chain
node.real_dimensions(app.world_units())          # "how big is this, really"
r.pick_world(x, y)                               # a clicked point in world space
```

The same wrapping covers the built-in reference nodes (`getGridNode()` /
`getAxisNode()`), lights (`addLight` → `LightNode`), the software raycaster
scene node (`VolRenNode`, `add_volren`; value types in `pycvc_volren.i`) and the
view-aligned slice renderer (`VolSliceNode`, `add_volslice`; value types in
`pycvc_volslice.i` — see [`docs/VOLSLICE_API.md`](../../docs/VOLSLICE_API.md#python-pycvc)).
The `cvc::state`-driven viewer/scene controllers are wrapped too, mirroring
`CameraController`: `StageLighting(sg)` (a cinematic key/fill/back/wash rig —
`apply_preset('three_point'|…)`, `setKey`/`setWash`, `apply()`, headless) and
`ScreenTextHud` (screen-space captions/status lines — `setText`, `setPosition`,
`get_color`; construct headless via `ScreenTextHud(app, path, None)`).
Contract tests: `test_pycvc_world_units.py`, `test_pycvc_gl_world.py`, and the
`extents_metres` case in `test_pycvc_model.py`.

## Physics: the world, vehicles and the navigator's plant

`pycvc_physics.i` wraps `cvc::physics`. `physics_world` gives bodies, materials, vehicles,
`step`, `state_hash`, `save` / `restore` and `snapshot_json`. `vehicle_desc` has the
`humvee()` and `roboracer()` presets, and `physics_binding` is the world's state-tree
binding. `nav_plant` makes a world's vehicles a sim_world's plant
(`nav_sim_world_set_plant(sim, plant)`; `None` selects nav's bicycle). `dyn_bike_plant` is the
dynamic single-track vehicle as one (tires, load transfer, actuator lag), with its energy books
(`books(i)`, `last_tick(i)`, `kinetic_at_attach(i)`) and model state (`body(i)`); given a
`physics_world` and its ground bodies, each chassis is a box that collides in it. Terrain and the
sim_world's settings are reached through `nav_sim_world_set_terrain` and
`nav_sim_world_pull` / `publish`. `state_comment` / `state_property` read what a key says it
is.

Python reads the same values as C++: `test_pycvc_physics.py` rebuilds the cross-platform golden
scene and `physics_parity_test`'s scenarios through pycvc, and requires C++'s state hashes and
values bit for bit. See `docs/PHYSICS_API.md`, "Python: pycvc".

### Vehicle routes on a plant

A vehicle with a turning radius drives a planned route rather than nav's carrot: the carrot
FSM's wall-follow target can sit inside the turning circle, where the vehicle hunts between
gears. `pycvc_nav.i` wraps `plan_vehicle_route` and `route_follower` (lengths in the occupancy
grid's world units):

```python
r = pycvc.nav_plan_vehicle_route(occ, min_x, min_y, max_x, max_y, x, y, heading, gx, gy,
                                 {"min_radius": 4.8, "disc_offsets": [0.0, 2.0],
                                  "disc_radius": 0.6},          # route_vehicle
                                 {"max_expansions": 120000})    # route_options
# r: reached, poses (M,4) [x, y, heading, reverse], cusps, length, reverse_length, expansions
follow = pycvc.nav_route_follower_create(sim, {"lookahead": 48.0})  # follower_options
pycvc.nav_route_follower_set(follow, i, r["poses"])
while running:
    pycvc.nav_route_follower_apply(follow)   # before each step
    pycvc.nav_sim_world_step(sim)
pycvc.nav_route_follower_status(follow)      # following / leg / legs / reversing / done
```

The option dicts take exactly the C++ structs' fields (an unknown key is an error).
`nav_route_clearance` gives a pose's clearance, and `nav_sim_world_belief_occ(sim, m)` copies belief
plane m's planning raster to plan on. `nav_sim_world_route_vehicle(sim, i)` derives agent i's route
vehicle and steering geometry from the plant (None on nav's bicycle). The follower holds a
reference to its world.
`test_pycvc_nav_routes.py` covers them.

### Your own commands

A controller of your own drives the agents in the navigator's place, on any plant, with
`sim_world::override_command`:

```python
steer = np.array([...])   # (N,) bicycle angles, rad, + left
accel = np.array([...])   # (N,) along the heading, m/s^2 x the world's scale (normalized / s^2)
pycvc.nav_sim_world_override(sim, steer, accel)   # for the next step only; NaN: the navigator's
pycvc.nav_sim_world_step(sim)
```

On nav's bicycle the agent integrates the command; on a plant it is the plant's command (Jolt's
driver model, or the dynamic vehicle's actuators), which backs a vehicle up from rest on a
negative acceleration. Either way the steer is held to the lock and the speed to the agent's top
speed. `nav_sim_world_set_pose(sim, i, x, y, heading, speed=0)` places an agent on nav's bicycle
(normalized units); a plant owns its vehicles' poses, so place the agents before
`nav_sim_world_set_plant`, which attaches each vehicle where its agent is.
`test_pycvc_nav_override.py` covers both.

### The plant by name, and generated worlds

`pycvc.plant_host` is libcvc's `plant_host`: it owns the physics world and the plant a sim_world
drives on, and switches them by name (`kinematic`, `humvee`, `roboracer`, a comma list cycled over
the agents, `dynamic[:presets]`, `dynamic4w[:presets]`):

```python
host = pycvc.plant_host(latency=0.1, brake_margin=0.8)
host.set_geometry(world_geometry)      # a city's or a generated world's colliders; None: flat ground
host.use(sim, "humvee")                # the agents at their poses, at rest, on Jolt Humvees
host.pull(app, "cars", sim)            # live: cars.nav_plant.vehicle switches it, its knobs apply
host.kind(), host.driving(), host.limits(i), host.state(i), host.vehicle_state(i), host.body(i)
```

With a geometry the vehicles start on its terrain. The host holds the sim_world it drives and puts
it back on the bicycle before it goes.

`pycvc.generate_world(params, building_library="")` is libcvc's terrain generator (terrain_lab's
world, `cvc::world::generate_world`) on a dict of its params (the `generated_world` node's
`world.*` key names, or the struct's): `.world()` is the `world_geometry` for physics and nav,
`.terrain()` the ground, `.height(x, y)`, `.info()` the params and what came out. It is a pure
function of the params and the library, so the same keys give the scene node's world exactly.
`test_pycvc_plant_host.py` covers both. `pycvc_gl.cars` ([docs/CARS.md](../../docs/CARS.md)) is
built on them.

## Level of detail (cvc::simplify + cvc::lod)

The whole LOD family is wrapped directly (`pycvc_lod.i`, `LodGraphicsNode` in
`pycvc_gl.i`): QEM `simplify`, per-asset pyramids, the selection math, tile
partitioning, pooled tiled builds and the `scene.cvch5` store.

```python
pool = pycvc.thread_pool(3)                      # a pool the caller owns
mesh, res = pycvc.simplify(g, params)            # out-params come back as values
pyr = pycvc.build_mesh_pyramid(g, pool=pool)     # .rungs / .world_error_m / .rung(k)
tiles = pycvc.partition_parts([(name, geom), ...], 100.0, group_key=["_walls", "_roof"])
pyrs = pycvc.build_tiled_pyramids(tiles, pool=pool, on_tile=lambda i, tile, pyr: q.put(i))

w = pycvc.scene_writer(app)                      # in-memory scene.cvch5
w.write_mesh_pyramid("tile0", pyrs[0], "%016x" % tiles[0].content_hash)
blob = w.to_blob()                               # bytes
r = pycvc.scene_reader.open_verified(app, blob, hashlib.sha256(blob).hexdigest())

node = sg.add_lod("tile0"); node.setPyramid(r.read_mesh_pyramid("tile0"))
stats = sg.select_lod_stats(view.make_view_params(pycvc.preset_view("balanced")))
```

The long calls release the GIL (other Python threads keep running); Python
callbacks (`on_tile`, `group_key`, `LodGraphicsNode.setRungStyle`) reacquire it,
and an exception they raise comes back out of the call that started them.
Failures map to `ValueError` / `IndexError` / `OSError` (HDF5) / `RuntimeError`
(SHA-256 mismatch). `examples/lod_city.py` runs the whole pipeline on a
synthetic city -- progressive attach from a loader thread, bake to bytes,
verified reload, a camera fly-out printing `lod_stats` per pose (`--render DIR`
draws each pose offscreen). Contract tests: `test_pycvc_lod.py`,
`test_pycvc_gl_lod.py`.

## HUDs & UIs (Dear ImGui, no raw ImGui needed)

`ImGuiOverlay(view)` is the per-viewer Dear ImGui integration. Python draws a HUD
by setting a draw callback and calling the **state-bound `ui_*` widgets/panels**
(`pycvc_imgui.i`) inside it — their bodies run within cvcGL against the overlay's
own ImGui context, so a HUD never touches raw `ImGui::` (which would crash: the
extension has its own `GImGui`):

```python
ov = pycvc_gl.ImGuiOverlay(view)
ov.attachCamera(cam)

def draw():
    pycvc_gl.ui_slider_double(app, "Speed", "scene.viewers.left.camera.settings.move_speed", 1, 200)
    pycvc_gl.ui_combo(app, "Belief", "demo.swarm.belief", ["shared", "grouped", "private"])
    pycvc_gl.ui_scene_panel(sg)          # ready-made shadow/chrome panel
    pycvc_gl.ui_camera_menu_items(cam)

ov.setDrawCallback(draw)                  # re-invoked each frame; keep it cheap, don't raise
```

Every widget reads/writes `cvc::state`, so the same value is drivable from a
script, a config file or a replicated peer. Raw custom-widget ImGui (free-form
windows) is a separate, deferred design (a curated subset compiled inside cvcGL).
Contract test: `test_pycvc_gl_imgui.py`.

## state_exec DSL (`Exec`) and parser settings

`pycvc.Exec(app)` runs state_exec programs over the app's state tree. The parser
settings are the C++ surface, under the C++ names (see
`docs/STATE_EXEC_GRAMMAR.md`, "Parser settings"):

```python
import pycvc
app = pycvc.make_app()
ex = pycvc.Exec(app)
ex.run("(+ 1 2)")                                     # "3"

pycvc.parse_options()                                 # max_depth 100000, step_tokens 0
pycvc.async_scheduler.host_parser_defaults()          # step_tokens 1024
ex.parser_defaults()                                  # == host_parser_defaults()
pycvc.state_set(app, "state_exec.defaults.parser.max_depth", "64")  # tree overrides
root = pycvc.State.instance(app)                      # the tree root, as in C++
pycvc.parse_options_from_state(root, "", ex.parser_defaults())      # what Exec uses

# Deep-data settings ("Deep data on the executor"), the same way
pycvc.data_options()                                  # step_nodes 0
pycvc.async_scheduler.host_data_defaults()            # step_nodes 1024
ex.data_defaults()                                    # == host_data_defaults()
pycvc.data_options_from_state(root, "", ex.data_defaults())         # what Exec uses
```

`Exec` starts from the same defaults as the app's exec scheduler (Ariadne), and
the same `state_exec.defaults.parser.*` and `state_exec.defaults.data.*` keys
override both. A syntax error raises `pycvc.parse_error`, a `RuntimeError` with
`.line` and `.col`.

A program's top-level definitions persist from one `run()` to the next. A child it
spawns (`(spawn "...")`) sees the intrinsics and every function registered with
`register_fn`, `register_async_fn` or `register_async_coro`, but not the parent's
definitions. Registered functions live in a scope under the programs' own, so a
program that rebinds a registered name shadows it.

### Scheduling policy, priorities and threads

`Exec(app, policy=None, idle_timeout=60.0)` takes keywords
(`docs/STATE_EXEC_LOCKS.md` 7.3, 7.4). `policy` fixes the scheduler's policy for
its life: `None` reads the app tree's `state_exec.policy` (`round_robin` when it is
unset, as for the app's exec scheduler; an unknown value is logged and
`round_robin` used), or pass a `pycvc.scheduling_policy` member or its name
(`"round_robin"`, `"priority"`, `"priority_rr"`). `idle_timeout` is how long, in
seconds, `run()` waits while its program is parked with nothing pending before it
kills it (a lock wait's rule is under "Locks across `Exec`s and threads", below).
Anything else, or a negative or NaN `idle_timeout`, raises `ValueError`.

```python
ex = pycvc.Exec(app, policy="priority_rr")    # or policy=pycvc.scheduling_policy.priority_rr
ex.scheduler().policy()                       # scheduling_policy.priority_rr
ex.set_priority(pid, 5)                       # True; False for a pid it doesn't have
ex.get_priority(pid)   # {'base': 5, 'effective': 5, 'floor': 0, 'policy': scheduling_policy.priority_rr}
```

`set_priority(pid, p)` changes a process's base priority as a host does, unchecked
(lower is better; the DSL's `set-priority` is the checked one). `get_priority(pid)`
returns the dict above, or `None` for a pid the scheduler doesn't have (a reaped one
included). Under `round_robin` a change is stored and reported and changes no pick.

They and `lock_query` (below) are the `Exec` calls any thread may make. `lock_query` never
waits; these two act as follows:

- With no `run()` active, they act at once, as the scheduler's pump owner (so they raise while
  another thread pumps it through one of its raw calls, below).
- From inside `run()` (a registered function, coroutine or callback), they act at once.
- From another thread while `run()` pumps, the call is posted to the scheduler and
  `run()` answers it between steps; the caller waits with the GIL released. A call
  still waiting when `run()` ends is answered by `run()` before it returns.
- From a thread running another `Exec`'s `run()` (its registered function, say), they
  raise `RuntimeError("Exec.run is active on another thread")` at once, so two
  `Exec`s whose functions call each other never wait for each other.

`run()` itself stays one thread's: a second `run()` of the same `Exec`, from any
thread (a kernel's included), raises `RuntimeError` until the first has returned.

**The raw schedulers are the pump thread's.** `Exec.scheduler()`, `pycvc.scheduler` and
`pycvc.async_scheduler` are the C++ classes, and their calls (`execute`, `kill`, `pause`,
`resume`, `send_signal`, `fork`, `set_max_steps`, `list_processes`, `get_process_info`, ...)
belong on the thread that steps the scheduler: inside its `run()` (a registered function or
callback), or while nothing steps it. Each scheduler has a pump owner, the thread driving it:
`Exec.run()` until it returns (through its wait for its kernels and its reap), and the
scheduler's own `run`, `step`, `sync_run`, `sync_step`, `drain_ingress`, `wake_awaiting` and
`wait_for_work` for their call. While another thread is the owner, every other call raises
`RuntimeError("scheduler: another thread is pumping it")` instead of racing it, and so does a pump
call (and `Exec.run()`) made while a raw pump call holds it; from the owner's own thread (a
registered function) they work. (`Exec.kill`, `pause`, `resume` and `send_signal`, posted the way
`set_priority` is, are not built yet.) Some `Exec` calls are the same: `set_parser_defaults` and
`set_data_defaults` change its scheduler, `register_fn`, `register_async_fn` and
`register_async_coro` change the natives environment its programs share, and `tracked_objects()`
and `collect_cycles()` read or collect its cycle registry, which its steps change; they raise the
same way. The exceptions:

- `stop()` and `is_running()` may come from any thread. `stop()` ends the scheduler's `run()`
  or `sync_run()` at its next step boundary, and an idle sync `run()` at once; under `Exec.run()`
  it ends only the slice that is running, and `run()` goes on. A `stop()` made while no run is
  active is discarded by the next run.
- `async_scheduler.post_message()` may come from any thread.
- `Exec.set_priority`, `Exec.get_priority` and `Exec.lock_query`, above.

The tree helpers (`pycvc.state_get`, `state_has`, `state_set`, `state_children`,
`state_comment`, `state_property`, `state_property_keys`, `state_remove`) may be called from any
thread while `Exec`s run: each holds the nodes it walks, so a node an `Exec` deletes meanwhile is
freed only once the helper is done with it (`docs/STATE_EXEC_THREADED.md` 4.8).
`state_remove(app, path)` removes its one node, with its subtree, as the DSL's `state-delete`
does: it no longer sweeps the tree's other expired nodes, and an empty path does nothing. A
program sweeps with `(state-sweep-expired)`. A `pycvc.state_observer` dropped while another
thread's write is firing its slot is never called. The C++ pinned walks
(`findThroughLinksShared`, `childThroughLinksShared`, the start-pinned `findDescendantShared` and
`sharedChild`) and `removeChild` are not wrapped: these helpers are Python's way in.

A built-in exception that passes through Python (a registered function's return value, alone or
in a list, or `pycvc.make_exception`'s, given the class's name or its dict) comes back with its
built-in class, the shared read-only one, so a program can `state-data-set` it and its class
refuses `set-attr` (`docs/STATE_EXEC_EXCEPTIONS.md` 2.3).

`pycvc.set_thread_class(cls)` puts the calling thread in a `pycvc.thread_class`
(`background`, `utility`, `normal`) as far as the OS allows, and returns whether it
did (`docs/THREAD_POOL.md`, "Thread classes"). On Linux an unprivileged thread
can't raise itself back once lowered: that returns `False`. The app tree's
`state_exec.process_class` sets the whole process's class when the app's exec
scheduler is first built.

The rest of state_exec is the C++ API under the C++ names
(`docs/PYCVC_STATE_EXEC_PARITY.md` lists what is wrapped and what comes next). A scheduler built
this way runs the policy passed to it and does not read `state_exec.policy`; only `Exec(app)` and
the app's exec scheduler do (`docs/STATE_EXEC_DEVELOPER_GUIDE.md`, "Which Scheduler Runs a
Program"):

```python
from pycvc import symbol
pycvc.parse("(f 'x \"s\")")      # [symbol('f'), [symbol('quote'), symbol('x')], 's']
pycvc.parse_all("1 2")           # [1, 2]
pycvc.to_string([symbol("a"), 1])                  # '(a 1)'
pycvc.encode_value(root.child("snap"), {"k": [1, 2]})
pycvc.decode_value(root.child("snap"))             # {'k': [1, 2]}
```

Values convert to native Python types, with `pycvc.symbol` for symbols and
`pycvc.value` as an opaque handle for closures and other non-data values. Such
a handle converts back to the same value when passed into the DSL. Conversion
is strict: non-`str` dict keys, ints outside 64 bits and unsupported types
raise.

The schedulers are the real classes too, with environments, builtins and the
process types:

```python
from pycvc import process_status, scheduling_policy
env = pycvc.builtins.make_default_environment()
pycvc.builtins.register_fn(env, "greet", lambda n: "hi " + n)  # a Python callable
opts = pycvc.execute_options()
opts.env = env
opts.on_complete = print                    # called with the result
sched = pycvc.scheduler(scheduling_policy.round_robin)  # or pycvc.async_scheduler()
pid = sched.execute('(greet "ann")', opts)
sched.run()                                 # {pid: 'hi ann'}; releases the GIL
sched.get_process_info(pid).status          # process_status.terminated
ex.scheduler().list_processes()             # Exec's own async_scheduler
```

The enums are `enum.IntEnum` classes, and an enum argument must be one of their
members. Where C++ returns an empty `std::optional` or a null pointer, Python gets
`None`, except for a value: `get_result`, `pop_pending_message` and
`environment.lookup` raise `KeyError` instead, because `None` is the nil value.

The evaluators, their state, and the intrinsics are wrapped the same way:

```python
ev = pycvc.stackless_evaluator(pycvc.builtins.make_default_environment())
st = ev.create_state("(begin (set x 1) (+ x 41))")
while not ev.step(st):                      # releases the GIL; ev.interrupt() from
    pass                                    # another thread raises evaluation_interrupted
st.result, st.stats.get_step_count()        # 42, the step count
pycvc.encode_evaluator_state(root.child("snap"), st)     # snapshot, decode_... to restore

ctx = pycvc.intrinsics_context()            # state-get, msg-send, spawn, ... over a tree
ctx.sched, ctx.root = sched, root
env = pycvc.builtins.make_default_environment()
pycvc.register_intrinsics(env, ctx)         # each function (and with-lock) keeps ctx alive
ctx.lock_default_timeout = 1.0              # with-lock with no :timeout gives up after 1 s
```

Contract tests: `test_pycvc_exec.py`, `test_pycvc_state_exec.py`,
`test_pycvc_state_exec_sched.py`, `test_pycvc_state_exec_eval.py`,
`test_pycvc_state_exec_data.py`.

### Many programs at once: `ExecPool`

`pycvc.ExecPool(app, threads=None, size=None, prefix="exec_pool", fresh_env=False,
idle_timeout=60.0)` runs batches of programs on the app's pool of execution groups
(`docs/STATE_EXEC_THREADED.md` 4.1, 4.12). The app has one pool, sized by whoever builds it
first: `threads` (workers besides the calling thread), else the tree's `state_exec.threads`; a
later different request keeps the built size and logs a warning, and `pool.threads` is the real
count. The pool holds `size` pooled `Exec`s (the real `pool.threads` by default, at least 1),
each its own group: its own scheduler, async host, cycle registry and `math.random` stream, its
natives over the one frozen builtins scope, and its context chrooted at `"<prefix>.<i>"`, so its
programs' state keys, channels and lock keys resolve under it. The pooled `Exec`s are not exposed.

```python
pool = pycvc.ExecPool(app, threads=8)
pool.register_fn("score", score)               # in every pooled Exec
pool.run_all([f"(score {i})" for i in range(64)])   # ['...', ...], in program order
pool.close()
```

- `run_all(programs)` takes a list of `str` and returns their results in program order, rendered
  as `Exec.run` renders them. Program `i` runs on pooled `Exec` `i % size`, each pooled `Exec`'s
  programs one at a time and in order; the groups are pumped together in fork-join rounds, on the
  workers and the calling thread, until every program has ended. A failing program does not stop
  the others: once all have ended, `run_all` raises the first failure in program order, as
  `Exec.run` would raise it. A program parked for `idle_timeout` seconds with nothing pending is
  killed, as in `Exec.run`. No kernel of the batch outlives it.
- Top-level definitions persist per pooled `Exec` from one program to the next; with
  `fresh_env=True` each program gets a fresh scope over the natives instead.
- `register_fn` and `register_async_fn` register on every pooled `Exec`, as `Exec`'s do.
- `run_all` releases the GIL for the whole batch: pure-DSL programs run in parallel, and a
  registered Python function takes the GIL for its call, so such calls serialize on it.
- One call at a time: `run_all`, `register_fn` or `register_async_fn` while another is in progress
  (on another thread, or from a function the batch calls) raises `RuntimeError`.
- `close()` kills what a batch on another thread still runs (that `run_all` raises `the pool was
  closed`), waits for it with the GIL released, and destroys the pooled `Exec`s. It is idempotent;
  afterwards the calls raise `RuntimeError("... the pool is closed")`. From a function its own batch
  calls it raises instead.
- Without threads (the Emscripten pycvc), or at `threads=0`, every group is pumped on the calling
  thread, in creation order.

**The GIL and `state_observer`.** A `pycvc.state_observer`'s slot takes the GIL on the thread that
writes, which can be a pool worker. `run_all` and `AriApp.frame` release the GIL while workers
run; a host embedding pycvc that pumps the app's pool from C++ (`app::exec_pump_frame()`, or an
`exec_pool().pump_all`) must release it too, or the first worker write under an observer
deadlocks.

**`AriApp` and the GIL.** `AriApp.frame` (and `step`, `load`, `wait_idle` and the destructor)
releases the GIL while C++ runs (`PYCVC_ARIADNE_NOGIL`, `pycvc_ariadne.i`). A frame is the
document's drain, which pumps the app's scheduler and, for a document the app tree's
`ariadne.exec.isolate_documents` puts in a group of its own (`docs/ARIADNE.md` 2.5), that group
through the app's pool, whose workers may run its programs. Every Python callable libcvc calls
takes the GIL itself, for its call, on whichever thread calls it: a host function
(`pycvc.host_function`), an async function on a pool thread, a Python native called from a group a
worker pumps, a `state_observer`'s slot. A host function is `host_only`, so a document that can
call one is pumped on the frame's thread, never a worker. So a host of `app::exec_pump_frame()`,
the one pump a frame for several documents, releases the GIL across that call as `AriApp.frame`
does, or the first worker that calls into Python deadlocks. `AriApp.wait_idle()` waits, with the
GIL released, until no async call of the document is running: on the app's async host and, when the
document is isolated, on its group's host too, and on a group a move left behind while its calls
still run. It is not for inside a frame (`RuntimeError`).

Contract tests: `test_pycvc_exec_pool.py`, `test_pycvc_exec.py`.

### Locks across `Exec`s and threads

Every `Exec` attaches its scheduler to the app's lock manager (`app::lock_manager()`,
`docs/STATE_EXEC_LOCKS.md` 8.4, 9.2), so the `with-lock` keys of all an app's `Exec`s, and of its
exec scheduler (Ariadne's), are one namespace: a section in one excludes an overlapping section in
any other, whatever thread runs it. pycvc runs schedulers on threads of its own and says so at
import, so the manager's default mode (`state_exec.locks.threading` `auto`) is `shared`: one mutex
around every lock operation. With the setting at `single`, `Exec(app)` raises `RuntimeError`
naming it. A build without threads (the static Emscripten pycvc) makes a `single` manager and
attaches every `Exec` to it. The import's mark is process-wide and permanent, but it only counts
for a manager made after it: an app whose manager was already made (its exec scheduler built
first, by an Ariadne `AppRuntime` for instance) keeps the mode it got then.

```python
import threading
app = pycvc.make_app()
pycvc.state_set(app, "n", "0")
prog = '(for i (range 1000) (with-lock "k" (state-set "n" (+ (int (state-get "n")) 1))))'
execs = [pycvc.Exec(app) for _ in range(2)]
threads = [threading.Thread(target=ex.run, args=(prog,)) for ex in execs]
for t in threads: t.start()
for t in threads: t.join()
pycvc.state_get(app, "n")         # "2000"
execs[0].lock_query()             # {'held': [], 'waiting': []}
```

A grant to a waiter in another `Exec` is posted to that `Exec`'s scheduler, and its `run()`
applies it between steps; a `run()` waiting for work wakes on the post. An `Exec` with no `run()`
active keeps the grant, and the key, for its waiter until its next `run()`. A cycle across `Exec`s
raises `DeadlockError` in the request that closes it, when it blocks; its `cycle` shows another
`Exec`'s pids as `None`, since pids repeat across `Exec`s. Each handoff of a key between threads
costs a post and a wake, so keep a hot key's sections in one `Exec`.

`Exec.lock_query(prefix="")` returns the app's locks under `prefix` as `(lock-query prefix)` does
in the `Exec`'s programs: `{"held": [{"key", "mode", "pids", "count", "generation",
"abandoned"}], "waiting": [{"key", "mode", "pid"}]}`. `""` is every key; otherwise `prefix` is a
key as a program writes it. Counts include every `Exec`'s holds and waiters; pids show only for
this `Exec`'s processes (another's are left out of `pids`, and are `None` in `waiting`). Any thread
may call it, with or without a `run()` active: it reads the manager under the manager's mutex,
with the GIL released, and never waits for `run()`.

**`idle_timeout` and lock waits.** `run()` kills a program left parked with nothing pending for
`idle_timeout` seconds. A lock wait, the program's own or a spawned process's, counts as pending
only while one of the processes it ultimately waits for (a holder, or an earlier waiter, that waits
for no lock itself) is on another `Exec`, or a scheduler given the app's manager, that made
progress within the last `idle_timeout` seconds; the window then runs from that progress. A
scheduler makes progress at each step, and an `Exec`'s `run()` also whenever it has work pending.
So a waiter behind another `Exec` that keeps stepping, or awaits an async call, lives; one behind
an `Exec` that nobody runs, or that stopped, is killed about `idle_timeout` after that `Exec`'s last
progress. The rule is per `Exec`, not per holder: a holder parked for ever on an `Exec` that keeps
stepping other processes keeps the waiter alive. A holder on the waiter's own `Exec` counts only
through `run()`'s own test (async calls in flight, pending coroutines, runnable processes): one
parked on `msg-recv`, with nothing else pending there, is no progress. A post in the `Exec`'s lock port (a grant that landed, not yet
applied) is pending work too. The kill raises `RuntimeError`, naming the key:
`pycvc.Exec.run: parked with no pending work (pid N, waiting for lock k)`, or
`(pid N, pid M waiting for lock k)` when the waiter is a process the program spawned.

Two consequences. A holder inside a long synchronous Python call (`register_fn`) makes no progress
until its step ends, so a waiter on another `Exec` whose `idle_timeout` is shorter than the call is
killed meanwhile: make such a call async (`register_async_fn`), or give the waiter a longer
`idle_timeout`. And without threads (the Emscripten pycvc) nothing steps another `Exec` while one
`run()` waits, so a wait behind another `Exec`'s holder ends only at `idle_timeout`, blocking the
page that long.

`pycvc.scheduler` and `pycvc.async_scheduler` can take the app's manager too:
`sched.set_lock_manager(app)` gives the scheduler `app::lock_manager()`, so its programs' locks meet
the `Exec`s'. It attaches as an `Exec` does, since Python may step it on any thread: with
`state_exec.locks.threading` at `single` it raises `RuntimeError` naming the setting. It attaches
only a scheduler's first lock home: it raises `RuntimeError` when the scheduler already has one (an
`Exec`'s scheduler always does, and so does one whose first `with-lock` made a private home), so a
home in use is never replaced. A scheduler given no manager keeps a private one, as in C++.

Contract test: `test_pycvc_state_exec_locks.py`.
