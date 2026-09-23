# cvc::nav navigation statistics

`cvc::nav::nav_stats` (`inc/cvc/nav/nav_stats.h`) is the **base, RF-free** navigation-telemetry
layer: per-vehicle + per-episode motion/clearance/collision/material/time/fuel/budget stats over a
`sim_world` drive, plus a corpus scorecard for ranking training checkpoints. It is the shared base of
the two-layer nav-stats design (the RF/comms extension lives in `cvc::dbg`, in the CVC-DBG/cvcdbg
repo, and joins this record by `veh_index`; it never lives here). Field names mirror the Python
`grl_snam.scorecard` schema, and the two are held to the same hand-computed numbers by parity tests.

The collector is stdlib-only (no libcvc-internal dependency beyond the STL): it consumes the
`sim_world::snapshot` arrays plus optional position samplers, so grl-snam, the cvcdbg demos/harness,
and `sim_world` itself all drive it identically.

## Types

- **`veh_nav_stats`** — one per vehicle: identity (`veh_index`/`convoy_id`/`vehicle_class`/
  `robot_radius_m`/`mass_kg`), outcome (`arrived`/`time_to_goal_s`/`timed_out`/`over_budget`/
  `goals_reached`), motion (`total_path_m`/`straight_m`/`turn_total_rad`/`turn_events`/`time_in_wall_s`/
  speeds), clearance/collisions (`min_clearance_m`/`time_below_clear_s`/`penetration_steps`/
  `veh_contacts`), per-material dwell (`time_over_material_s`/`dist_over_material_m`, `kNumMaterials`
  buckets), and effort (`accel_integral`/`fuel_used`).
- **`episode_nav_stats`** — one per episode: the reduced fleet fields + `per_vehicle`, and `to_json()`
  (the base record; a DBG consumer nests an `"rf"` member itself). The `1e30` "unmeasured" sentinel
  for `min_clearance_m`/`min_sep_m` serializes as JSON `null`, not a huge finite number.
- **`nav_stats_params`** (thresholds) and **`budget_policy`** (time / ETA-multiple / fuel bounds).
- **`nav_samplers`** — optional per-position hooks: `material_id(x,y)`, `occupied(x,y)`, and
  `min_clearance_m` (a `const double*` **in metres** — see the units note below).
- **`nav_scorecard`** + **`aggregate_nav(episodes, checkpoint)`** — reduce a corpus of episodes into
  one RF-free fitness row (arrival, economy, safety, material) for ranking base-policy checkpoints.

## Collecting

Two ways, both producing the same `episode_nav_stats`:

**1. Directly, over any driver's snapshot arrays** (`nav_stats_collector`):

```cpp
cvc::nav::nav_stats_collector c;
c.begin_episode(n, dt_s, start_pos, goal_pos, budget, scene_id, seed, checkpoint);
// each frame, AFTER the drive advances:
c.step(pos, head, spd, mode, reached, samplers);   // pos/goal/start in WORLD metres
cvc::nav::episode_nav_stats e = c.finish();
```

**2. Via `sim_world`'s opt-in internal collector** (the torch-free native path): arm it before the
first `step()`, and every `step()` folds a tick in pure C++ (safe on the sim thread):

```cpp
world.begin_nav_stats(params, budget, scene_id, seed, checkpoint);  // captures the current pose as start
for (...) world.step();
cvc::nav::episode_nav_stats e = world.nav_stats();
```

Opt-in and default-off, so a plain `sim_world` run is byte-unchanged. Exposed to Python through the
`pycvc` bindings `nav_sim_world_begin_nav_stats` / `nav_sim_world_nav_stats` (JSON).

## Units — the one trap

`nav_samplers.min_clearance_m` is **metres** (compared against `nav_stats_params::clear_safety_m`).
`sim_world::min_clearance()` returns **normalized** clearance; use **`sim_world::min_clearance_world()`**
(= `min_clearance() / cfg.scale`, matching the `goals_world()`/`carrots_world()` convention) to feed
the sampler, never the raw `min_clearance()`. The internal collector (`begin_nav_stats`) does this for
you. Positions/goals must likewise be world metres (`snapshot()` / `goals_world()` deliver them).

## Per-vehicle heterogeneity

The identity fields (`robot_radius_m`/`mass_kg`/…) and the per-agent `veh_params` columns in
`drive.h` (`rr_col`/`body_rr_col`/`vmax_col`/`a_max_col`/`L_col`, honored on both the CPU rollout and
the batch CUDA entry points) let a mixed fleet be scored per vehicle even though today the demos run a
homogeneous convoy.

## Tests

`src/cvc/tests/nav_stats_test.cpp` (the base accumulators/scorecard over a scripted trajectory) and
`nav_test.cpp`'s `NavSimWorld` suite (the `sim_world` internal collector + `min_clearance_world`). The
scripted corpus + numbers are mirrored in cvcdbg's and grl-snam's tests — the shared-schema contract.
