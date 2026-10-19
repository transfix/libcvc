# World Clock API Reference (`cvc::world_clock`)

> One notion of simulation time per process. One driver advances it. Every
> simulation follows it at its own whole-quantum rate. Speed, pause,
> single-step, lockstep and replay are set in one place and mean the same thing
> for every simulation on the clock.

Headers:

- [`cvc/core/world_clock.h`](../inc/cvc/core/world_clock.h): the clock.
- [`cvc/core/clock_follower.h`](../inc/cvc/core/clock_follower.h): following it at a period.
- [`cvc/core/world_clock_binding.h`](../inc/cvc/core/world_clock_binding.h): its state-tree face.

The migration plan is [`roadmap/WORLD-CLOCK-SYNC-ROADMAP.md`](roadmap/WORLD-CLOCK-SYNC-ROADMAP.md).

## Three clocks that must not be conflated

| Clock | What it is |
|---|---|
| wall time | What `steady_clock` reports. It is the only real input. |
| world time | What the simulation believes. It advances in whole `fixed_dt` quanta, and `t() = tick · fixed_dt`, never accumulated. |
| render time | How often we present. It is a policy, not a clock; rendering interpolates between states with `alpha`. |

## One driver, many followers

```cpp
cvc::world_clock &clock = app.world_clock();
cvc::world_clock::config cfg;
cfg.fixed_dt = 0.01;              // every simulation's step must be a whole number of these
cfg.cap_scales_with_rate = true;  // 2x asks for twice the quanta per frame
clock.configure(cfg);             // before any simulation starts; restarts at tick 0

cvc::world_clock_binding binding(app, clock);                 // sim.* keys on the state tree
cvc::nav::sim_thread nav(world, clock);                       // follower: one step per 0.06 s = 6 quanta
auto physics = cvc::clock_follower::for_step(clock, 0.01);    // follower: every quantum

// The frame loop is the ONE driver:
for (;;) {
  const auto r = binding.drive(wall_dt);   // read controls, advance, run requested steps, publish
  for (auto n = physics.take(clock); n > 0; --n)
    step_physics(0.01);
  render(nav.read(), cvc::clock_follower(6).alpha(clock.tick(), r.alpha));
}
```

- **The driver** is the frame loop, a headless harness, or `sim_thread` with
  `options::drive_clock` when nothing else advances the clock. It is the only
  caller of `advance()`, `advance_ticks()` or `drive()`.
- **A follower** is any simulation. It keeps a `clock_follower` with its period
  `k = ticks_per_step(dt, fixed_dt)`. `ticks_per_step` throws unless `dt` is a
  whole number of quanta (a 0.06 s step on a 1/120 s clock would be 7.2), and
  `take()` returns the steps that fell due. Followers never advance the clock.
- **Presentation.** A renderer interpolating a follower's last two states uses
  `clock_follower::alpha(tick, clock_alpha)`, the fraction into that
  follower's own step. The clock's `alpha` is a fraction of one quantum.

## Modes

| Mode | `advance(wall_dt)` | Use |
|---|---|---|
| `live` | `wall_dt × scale` banked into whole quanta, capped at `max_steps_per_advance` (dropped quanta are counted) | real time, 0.5×, 2× |
| `lockstep` | exactly `lockstep_steps` quanta, whatever the wall says; `alpha` is 0 | frame-by-frame inspection |
| `paused` | nothing; nothing banked | pause |
| `stepping` | nothing; `step_once()` runs one quantum | debugging |
| `replay` | nothing; `seek_tick()` positions | replay |

These run in any mode:

- `advance_ticks(n)` runs exactly `n` quanta now, with no cap (batch).
- `step_once()` runs exactly one.

For **capture**, run `live` and pass a fixed `wall_dt` (1/fps): the clock is
deterministic for a given sequence of deltas, and the video plays at the
chosen speed.

## Configuration

| Field | Default | Meaning |
|---|---|---|
| `fixed_dt` | 1/120 | The quantum. Simulation hosts use **0.01 s**, which divides a 0.06 s navigator step (6), a 20 Hz vehicle controller (5), and 0.1 s and 0.05 s rollouts. |
| `scale` | 1 | World seconds per wall second; 0 pauses. |
| `max_steps_per_advance` | 8 | Stall guard. Quanta beyond it are dropped and counted. |
| `cap_scales_with_rate` | false | The cap becomes `max_steps_per_advance × ceil(scale)` for `scale > 1`. |
| `lockstep_steps` | 1 | Quanta per `advance()` in `lockstep`. |

`configure(cfg)` replaces the configuration and restarts at tick 0. It is how a
host picks a quantum every simulation divides before any of them starts.

## Observability

| Call | Reports |
|---|---|
| `tick()`, `t()` | the authoritative integer, and world seconds derived from it |
| `total_dropped()` | quanta discarded by the stall guard; non-zero means the run is not a faithful simulation of the elapsed wall time |
| `realized_scale()` | world seconds actually simulated per wall second, over the last ≥ 0.5 s window. A speed the machine cannot deliver reads below the request instead of quietly running slower. |
| `wait_for_tick_change(tick, timeout)` | blocks a follower thread until the tick moves |
| `step_result::first_tick` | the quanta an `advance` ran are `first_tick + 1 … first_tick + steps` |

## The state tree (`cvc::world_clock_binding`)

Keys are relative to a prefix, default `sim`. These are the keys Ariadne's
`sim_transport.ari` binds.

| Key | Direction | Meaning |
|---|---|---|
| `sim.speed` | control | world seconds per wall second; 0 pauses |
| `sim.paused` | control | bool (`1`/`0`, `true`/`false`) |
| `sim.mode` | control | `live` \| `lockstep` \| `stepping` \| `replay` |
| `sim.lockstep_steps` | control | quanta per frame in lockstep |
| `sim.step` | control | request counter: each increment runs one quantum |
| `sim.time` | telemetry | world seconds |
| `sim.tick` | telemetry | the tick |
| `sim.fixed_dt` | telemetry | the quantum |
| `sim.realized_speed` | telemetry | the speed achieved |
| `sim.dropped` | telemetry | quanta discarded by the stall guard |

**Rules the binding applies:**

- Unset control keys are seeded from the clock.
- `paused` wins over `live` and `lockstep`.
- An unknown or unreadable control value is ignored, never thrown into a frame.

**Keys named one by one.** `cvc::world_clock_keys` holds one path per key, and
an empty path turns that lane off: it is never seeded, read or written.
`world_clock_keys::under("sim")` is the table above. A host with its own
names passes the struct instead of a prefix:

```cpp
cvc::world_clock_keys keys = cvc::world_clock_keys::under("coast.clock");
keys.speed = "coast.clock.scale";   // the demo's existing knob
keys.time = "coast.clock.t";
cvc::world_clock_binding binding(app, app.world_clock(), keys);
```

**Host verbs.** `binding.request_steps(n)` queues `n` single steps, the same as
`n` increments of `sim.step`; the next `drive()` runs them in any mode. A Step
button wired as a verb (`sim_transport.ari`'s `on: sim.step`) calls it with the
navigator's quanta, `ticks_per_step(veh.dt, fixed_dt)`, to run one navigator
step.

**Ariadne scenes.** A scene with a `clock:` block has `tick_scene` drive
`app.world_clock()` through a binding it makes on the first tick. The keys
come from `scene_clock_keys(scene.clock)`: the scene's own speed, paused, time
and tick keys, with mode, lockstep_steps, step and the remaining telemetry
beside them (`clk.sim.speed` gives `clk.sim.mode`, `clk.sim.step`, ...).
`tick_scene` returns the frame's `step_result`. A capture passes its fixed
frame dt; a live window passes nothing, and the elapsed wall time is measured.

**Threaded followers in a capture.** `sim_thread::wait_caught_up(timeout)`
blocks until the worker has run and published every step due up to the
clock's current tick. A capture calls it after `drive()` and before `read()`, so
each frame shows exactly the steps its quanta hold and a rerun reproduces it.

## The determinism contract

Pacing decides *when* quanta run, never *which* quanta run. A follower's
trajectory is the same under every mode:

- live at 1×, 2× or 0.5×;
- lockstep;
- batch;
- single-step.

`ClockFollowerTest.TheTrajectoryIsTheSameUnderEveryPacingMode` and
`NavSimThread.AFollowerMatchesDirectSteppingExactly` pin this. A run is a
function of its initial state, its seed and its input trace stamped by tick.

## Python (pycvc)

```python
import pycvc
app = pycvc.App()
clock = app.world_clock()
clock.set_config(0.01, 1.0, 8, 1, True)       # fixed_dt, scale, max_steps, lockstep_steps, cap_scales_with_rate
binding = pycvc.world_clock_binding(app, clock)  # sim.* keys
nav = pycvc.clock_follower.for_step(clock, 0.06)
r = binding.drive(1 / 60)
for _ in range(nav.take(clock)):
    world.step()
clock.set_mode_name("lockstep")                 # live | lockstep | stepping | paused | replay
```
