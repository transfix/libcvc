# World Clock Sync Roadmap — every simulation and demo on one `cvc::world_clock`

> **Goal:** every simulation, demo and capture in libcvc, and in the applications built on it, takes
> its time from one `cvc::world_clock` per process. They share one set of controls (speed, pause,
> single-step, lockstep, replay) on the state tree. A run's result never depends on how it was paced.
>
> This is the remaining half of the volrover3 roadmap's R48 (the clock on the state tree) and R49
> (live/replay). It is also the prerequisite for a physics engine: a physics tick without a fixed
> quantum shared with the navigator cannot be deterministic.

API reference: [`../WORLD_CLOCK_API.md`](../WORLD_CLOCK_API.md).

## Why

An inventory taken on 2026-10-14 found eleven ways that simulations and demos pace themselves:

1. the world clock;
2. a Python copy of it;
3. fixed-Hz sleeping worker threads;
4. three copies of a `SimPacer` accumulator;
5. a fixed number of steps per render frame;
6. lockstep credits;
7. variable wall-time accumulation;
8. synthetic `frame/fps` time;
9. fixed-dt batch loops;
10. render loops capped by sleeps or browser yields;
11. wall-time rate gates.

Three problems follow from that:

- **Booked time is not integrated time.** Some loops book `frame/fps` (0.033 s) while integrating two
  0.06 s navigator steps per frame, which is 3.6× too fast.
- **Worker threads set the world speed by accident.** At 60 Hz × 0.06 s, `nav::sim_thread` ran at
  3.6× real time.
- **Captures depend on wall time,** so they do not reproduce.

## The mechanism (landed with this roadmap)

- **`world_clock`** gains:
  - `lockstep` mode;
  - `advance_ticks(n)` for batch runs;
  - `configure()`, so a host picks a quantum every simulation divides;
  - `step_result::first_tick`;
  - `realized_scale()`;
  - `cap_scales_with_rate`;
  - `wait_for_tick_change()`.
- **`clock_follower`** provides `ticks_per_step` (which refuses a step that is not a whole number of
  quanta), `steps_due`, and `alpha` (interpolation into a follower's own step).
- **`world_clock_binding`** exposes `sim.speed`, `sim.paused`, `sim.mode`, `sim.lockstep_steps` and
  `sim.step` as controls, and publishes `sim.time`, `sim.tick`, `sim.fixed_dt`, `sim.realized_speed` and
  `sim.dropped`. These are the keys Ariadne's `sim_transport.ari` already binds.
- **`nav::sim_thread`** follows a shared clock: one step every `veh.dt / fixed_dt` quanta. Its rate-paced
  constructor keeps working on a private clock.
- **pycvc** wraps all of it.
- **Suite quantum for simulation hosts: 0.01 s.** It divides a 0.06 s navigator step (6), a 20 Hz vehicle
  controller (5), and 0.1 s and 0.05 s rollouts. Ariadne and L-system scenes without a navigator keep
  1/120 s.

## Migration checklist (libcvc)

| Loop | Today | Change |
|---|---|---|
| Ariadne `tick_scene` (`src/cvcGL/ariadne/scene_realize.cpp`) | Drives `app.world_clock()` from wall time even in capture loops; drops `steps`/`alpha`; ignores `sim.step` | Drive through `world_clock_binding`, so lockstep and single-step work; capture callers pass a fixed `wall_dt` |
| `navdemo::SimPacer` (`src/cvcGL/examples/nav_common.h`) | A duplicate accumulator that drops time silently; used by `nav_fog_ghost` | Delete it; `nav_fog_ghost` follows the app clock |
| `nav_city_swarm`, `nav_city_drive` | `sim_thread` at a fixed Hz; per-frame stepping without pthreads; "step one frame" un-pauses for a render frame | `sim_thread(world, app.world_clock())`, with the frame loop as the driver; the speed slider writes `sim.speed`; the initial speed keeps today's pace (3.6×) and says so |
| `nav_finale` | A fixed `hz/fps` steps per frame; targets move on a separate `t` | A navigator follower on the app clock; targets on `clock.t()` |
| `nav_convoy` | Its own `simT += wall_dt × sim.speed` | `simT = clock.t()` through the binding |
| `lsystem_coast` | A private clock for the ocean only; trees and clouds stay on wall time | The app clock for every animated layer |
| `lsystem_forest.cpp`, `terrain_lab` | `t` from wall time or `frame/fps` | `t` from the app clock |
| `ariadne_hello`, `ariadne_stream` | `tick_scene` on wall time in capture | Pass the capture dt |
| Frame-counted cadences (belief pulse, fog repaint) | Advance per render frame | Tick periods |

Camera smoothing in the bunny and volume demos is presentation only and may stay on wall time.

## Downstream applications

Applications built on `cvc::nav` and pycvc follow the same pattern once a libcvc release with this
mechanism is published:

- one `app.world_clock()` configured at 0.01 s;
- the frame loop or harness as the only driver;
- every simulation, schedule and statistic on `clock.t()`.

GRL-SNAM's pure-Python `grl_snam/clock.py` gains the same modes and a follower, with a parity test against
pycvc.

## Gates

- `ClockFollowerTest.TheTrajectoryIsTheSameUnderEveryPacingMode` and
  `NavSimThread.AFollowerMatchesDirectSteppingExactly` stay green.
- Every interactive demo answers `sim.speed`, `sim.paused`, `sim.mode` and `sim.step`.
- Every capture is reproducible bit for bit.
- No loop books time it did not integrate.
- The nav goldens are unchanged: the navigator kernels never see the clock, and `veh.dt` keeps its trained
  value.
