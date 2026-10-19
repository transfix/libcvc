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

## Migration checklist (libcvc) — step C2, done

Every row was migrated on branch `feat/world-clock-demos` (2026-10-14). Each demo was verified on
Windows by two identical offscreen captures, frame for frame.

| Loop | Before | Now |
|---|---|---|
| Ariadne `tick_scene` (`src/cvcGL/ariadne/scene_realize.cpp`) | Drove `app.world_clock()` from wall time even in capture loops; dropped `steps`/`alpha`; ignored `sim.step` | Drives through a `world_clock_binding` keyed by `scene_clock_keys()`, so lockstep, single-step and mode work; returns the frame's `step_result` |
| `navdemo::SimPacer` (`src/cvcGL/examples/nav_common.h`) | A duplicate accumulator that dropped time silently; used by `nav_fog_ghost` | Deleted; `nav_fog_ghost` follows the app clock at 0.01 s through a binding |
| `nav_city_swarm`, `nav_city_drive` | `sim_thread` at a fixed Hz; per-frame stepping without pthreads; "step one frame" un-paused for a render frame | `sim_thread(world, app.world_clock())` with the frame loop as the driver. The "world speed" slider writes `sim.speed`; Step runs one navigator step. The initial speed is `--hz × veh.dt` (3.6×), printed at start-up. A capture waits on `sim_thread::wait_caught_up` |
| `nav_finale` | A fixed `hz/fps` steps per frame; targets moved on a separate `t` | A navigator follower on the app clock. The acts, the targets and the replans run in world time |
| `nav_convoy` | Its own `simT += wall_dt × sim.speed` | The scene declares `clock: {}`; the convoy is drawn at `clock.t()`; Step is `request_steps` and Restart is `reset()` |
| `lsystem_coast` | A private clock for the ocean only; trees and clouds on wall time | The app clock for every animated layer, keyed by the demo's own `coast.clock.*` knobs |
| `lsystem_forest.cpp`, `terrain_lab` | `t` from wall time or `frame/fps` | `t` from the app clock |
| `ariadne_hello`, `ariadne_stream` | `tick_scene` on wall time in capture | Pass the capture dt |
| Frame-counted cadences (belief pulse, fog repaint) | Advanced per render frame | Fog and belief-wall repaints every 15 navigator steps; the belief pulse is a function of `clock.t()` |

Camera smoothing in the bunny and volume demos is presentation only and stays on wall time, as do
pure render throttles: recolour, restyle and HUD every few frames.

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
