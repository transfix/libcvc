# Input Traces (`cvc::input_trace`)

> Record the input, not the trajectory. A run on a `world_clock` is a function of its
> starting state, its seed and its inputs stamped by tick. An input trace holds exactly
> those, so a live session replays tick for tick and becomes a regression benchmark.
> Replay it on another build or another policy, and the trace shows where the trajectory
> departs.

This is R49 of the GRL-SNAM Laboratory spec (`cvc-engagement-docs`
`modernization/grl-snam-laboratory-camera-rigs-and-base-nav-demo.md`, §7 and §9 Phase 6).
The spec's acceptance test 6 is met for libcvc's kinematic `sim_world` by
`src/cvc/tests/nav_trace_test.cpp`:

- a live session replayed from its trace reproduces the trajectory exactly;
- two policy builds on one trace differ only where the policy differs.

Headers:

- [`cvc/core/input_trace.h`](../inc/cvc/core/input_trace.h): the format, the recorder and the
  player. It depends on no simulation.
- [`cvc/nav/sim_world_trace.h`](../inc/cvc/nav/sim_world_trace.h): a `sim_world`'s trace. It
  covers the header, the rebuild, nav's events, the state hash and the headless replay.

## The format: `cvc.input_trace/v1`

JSON lines, one object a line:

```text
{"schema":"cvc.input_trace/v1","header":{...}}
{"tick":120,"set":{"path":"nav.vehicle.vmax","value":"6"}}
{"tick":300,"event":{"name":"nav.retarget","args":{"agent":2,"goal":[0.5,-0.25]}}}
{"tick":306,"check":"sha256:..."}
{"end":{"tick":1200,"ticks":1200,"hash":"sha256:..."}}
```

**The header** holds the starting state.

| Field | What it is |
|---|---|
| `seed` | The host's seed. |
| `dt` | The simulation's step, in seconds. |
| `fixed_dt` | The clock's quantum. |
| `phase` | The step falls on ticks `t` with `t % (dt / fixed_dt) == phase`. |
| `start_tick` | The clock's tick when recording began. |
| `world_id` | The host's name for the world. |
| `world` | What a replay needs to build the world, if libcvc can build it (below). |
| `agents` | Each agent's `id`, `start`, `goal`, and `policy`: `"sha256:<hex>"` of its policy file, or `"default"`. |
| `settings` | Each watched key's value at the start. |
| `watch` | The watched prefixes. |
| `build` | The build that recorded it: `{"libcvc", "compiler", "arch", "os"}`. |
| `meta` | The host's own JSON object. |

**Records** are in tick order:

- `set`: a watched state key changed (`""` unsets it).
- `event`: a host verb that is not a write to the tree, with its arguments as a JSON object.
- `check`: the state's hash at that tick.

**The end** holds the last tick, the tick count and the final state's hash. A recording that
never finished has no end. It still reads, and a replay then runs to its last record.

**When a record applies.** A record stamped with tick T was made after the quanta up to T ran
and before quantum T + 1. A replay applies it there, so the first simulation step on a quantum
after T sees it. Pacing (speed, pause, the clock's mode) is not an input and is not recorded:
pacing decides when quanta run, never which.

Numbers keep their exact values. A double is written in its shortest round-trip form, so a
float32 knob reads back bit for bit, and a 64-bit seed is written whole. NaN and the
infinities are written as `null`.

## Recording

```cpp
cvc::input_trace_recorder rec(app, app.world_clock());   // owned by the host: no statics
world.pull(app, "");                                      // the knobs into the tree first
rec.start(cvc::nav::trace_header(world, opts), "session.trace");
for (;;) {                                                // the frame loop
  // the UI's writes to nav.* land here, between frames
  auto r = binding.drive(wall_dt);
  for (auto n = nav.take(clock); n > 0; --n) { world.pull(app, ""); world.step(); }
  rec.check(cvc::nav::state_hash(world));                 // optional: where a replay departs
}
rec.finish(cvc::nav::state_hash(world));
```

**What is recorded.** The recorder watches the app's state tree. It records a key if all of
these hold:

- the key is a watched prefix or under one. The default prefixes are `nav`, `nav_plant`,
  `run` and `world`;
- it is not an ignored prefix and not under one. The defaults are `nav.agents` and
  `nav.settings`, which `sim_world` publishes;
- its value changed;
- its `access` property (`cvc/state/state_key_meta.h`) is not `telemetry`, which marks an
  output. `skip_telemetry` turns this check off.

A key removed from the tree is not recorded in v1.

**Stamping.** There are two modes.

- `on_write` (the default) stamps a change with the clock's tick as it lands. That is exact
  for a host whose inputs are written between frames, on the thread that steps.
- `on_sample` records the changed keys when the host calls `sample()`, at the tick then. A
  host whose inputs come from another thread calls `sample()` just before it reads them. The
  trace then holds exactly what the simulation consumed.

**Host verbs.** Use `rec.event(name, args)` for anything that is not a write to the tree. nav's
own verbs record themselves:

| Call | Event | Args |
|---|---|---|
| `trace_retarget(world, &rec, id, gx, gy)` | `nav.retarget` | `{"agent", "goal": [x, y]}` |
| `trace_set_pose(...)` | `nav.set_pose` | `{"agent", "pose": [x, y, heading, speed]}` |
| `trace_override(...)` | `nav.override` | `{"agent", "command": [steer, accel]}` |
| `trace_add_obstacle(...)` | `nav.add_obstacle` | `{"rect": [r0, r1, c0, c1]}` |

Agents are named by id. Positions are in nav's normalized frame.

**A clock that goes back** breaks the recording, for example `reset()` or `configure()` while
recording. `finish()` then throws, naming the ticks.

## Replaying

`input_trace_player` plays the inputs back at their ticks. The clock is in `replay` mode, so it
moves only when the player moves it. The host builds its world as the header describes. In
C++:

```cpp
cvc::input_trace_player play(app, clock, trace);
play.on_event([&](const cvc::input_trace_record &e) { cvc::nav::apply_trace_event(world, e); });
play.set_state_hash([&] { return cvc::nav::state_hash(world); });
play.rewind();                            // replay mode, the start tick, the header's settings
cvc::clock_follower nav(cvc::ticks_per_step(h.dt, h.fixed_dt), h.phase, clock.tick());
play.run([&](std::uint64_t) {
  for (auto n = nav.take(clock); n > 0; --n) { world.pull(app, ""); world.step(); }
});
bool same = play.matched();               // every check and the end matched
std::uint64_t at = play.first_divergence();
```

A host that loops for itself, as a Python host does, steps quantum by quantum:

```cpp
while (!play.done()) {
  for (auto &e : play.apply_due()) apply(world, e);  // sets go to the tree; events come back
  play.next_quantum();
  // ...run what fell due on this tick...
  if (!play.expected_hash().empty()) play.verify(state_hash(world));
}
```

**Headless.** `cvc::nav::replay_trace(app, trace)` does all of the above for a kinematic
`sim_world`. It rebuilds the world from the header, applies nav's events, pulls the world's
settings before each step (when the header says the host did), and compares every hash. Use a
fresh app.

## `cvc-cli trace`

```bash
cvc trace info session.trace
```

```bash
cvc trace verify session.trace
```

```bash
cvc trace verify session.trace --policy 1=other.cvcnav
```

- `info` prints the world, the step, the span, the agents and their policies, and the inputs by
  key and by verb.
- `verify` replays the trace headlessly and compares every hash it holds. It exits 1 when the
  replay departs, and says at which tick. `--policy INDEX=FILE` swaps in another build of a
  policy, which is the second half of acceptance test 6.

The logic lives in the library: `cvc::describe(trace)` and `cvc::nav::replay_trace`.

## Python

```python
app = pycvc.App(); clock = app.world_clock()
pycvc.nav_sim_world_pull(sim, app, "")
h = pycvc.nav_trace_header(sim, "grid:demo", seed=7, policies={0: ""})  # "" = the default
rec = pycvc.input_trace_recorder(app, clock)
rec.start(h, "session.trace")
# ... pycvc.state_set(app, "nav.vehicle.vmax", "5") between frames,
#     pycvc.nav_trace_retarget(sim, rec, 2, gx, gy), rec.check(pycvc.nav_trace_state_hash(sim))
rec.finish(pycvc.nav_trace_state_hash(sim))

r = pycvc.nav_trace_replay(pycvc.App(), pycvc.input_trace.read("session.trace"))
assert r["matched"], r["report"]
```

The player has no callbacks in Python. `apply_due()` returns the events, and
`nav_trace_apply_event(sim, e)` applies nav's. `nav_sim_world_from_trace(header)` rebuilds the
world as a handle. `bindings/pycvc/test_pycvc_input_trace.py` is a worked example.

## The world section of a nav trace

`cvc::nav::trace_header(world, options)` describes a `sim_world` before its first step:

```text
{"kind":"cvc.nav.sim_world/v1","complete":true,"prefix":"","pull":"step","threads":1,
 "config":{rows, cols, bounds, scale, the sensing and belief knobs, "veh":{...}},
 "planes":1,"colors":[...],"policies":[{"index","key","uri","sha256"}],
 "truth":{"rows","cols","sha256","rle":[value,count,...]},"prior":{"same_as":"truth"}}
```

- **Agent starts** are `[x, y, heading, speed]` and goals are `[x, y]`, in nav's normalized
  frame. They are the floats the drive keeps, so the rebuild is exact.
- **Policies.** The world cannot name its own policy (the constructor's model), so the host
  states it in `trace_options::policies[0]`. An empty ref is the built-in default. A policy set
  through a `policy_library` is keyed `"sha256:<hex>"`, and its file's uri goes in `policies[p]`.
- **The maps** are run-length coded, each with its sha256.
- **State hash.** `state_hash(world)` is sha256 over the tick and each agent's id, position,
  goal, heading, speed, mode and reached flag (the float bits, in index order). Two worlds hash
  alike exactly when those are byte-identical.

## Limits (v1)

- **The kinematic plant only.** `trace_header` refuses a world v1 cannot rebuild, and says
  why. With `allow_incomplete` it marks the world `"complete": false` for a host that rebuilds
  it itself, and `trace verify` then refuses it. These worlds are refused:
  - a vehicle plant (Jolt or the dynamic ones);
  - terrain;
  - a material stack;
  - per-agent vehicle columns;
  - a multi-disc body;
  - agents added or removed before the start;
  - a world that has already stepped.
- **Jolt is deterministic only within one binary.** A Jolt trace could replay only on the
  build that recorded it. Other builds, compilers and platforms are not covered. The header's
  `build` says which build recorded it.
- **No going backwards.** `sim_world` has no save and restore. A replay seeks backwards by
  rebuilding the world and replaying from tick 0 (`rewind()`).
- **What the host does outside these channels is its own.** An external force, yielding,
  routes and intents are not recorded. The host records them as its own events, or does not
  use them. The headless replay stops at an event that is not nav's and names it.
- **Thread count.** A `sim_world` step is bit-identical on any thread count: `sense_batch` and
  the drive are. The header's `threads` is only a default.

Reading a trace is safe on any build. Replaying one exactly needs the same arithmetic. The
kinematic plant is plain float code, but a different compiler, flags or CPU may still round
differently. The check records show where.
