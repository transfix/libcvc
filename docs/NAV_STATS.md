# cvc::nav navigation statistics

`cvc::nav::nav_stats` (`inc/cvc/nav/nav_stats.h`) is the **base, domain-neutral**
navigation-telemetry layer: per-vehicle + per-episode motion/clearance/collision/material/time/fuel/
budget stats over a `sim_world` drive, plus a corpus scorecard for ranking training checkpoints. It is
the open core of a two-layer design. Everything here is generic navigation; the second layer is an
**extension seam**. A downstream layer that needs domain-specific metrics (for example, about an
external force it injects through `ext_force`) keeps its own per-vehicle record, joins it to this one
by `veh_index`, and nests it in the JSON itself. Nothing domain-specific lives here. Field names
mirror the Python `grl_snam.scorecard` schema, and the two are held to the same hand-computed numbers
by parity tests.

The collector is stdlib-only (no libcvc-internal dependency beyond the STL): it consumes the
`sim_world::snapshot` arrays plus optional position samplers, so grl-snam, the cvcGL nav demos, a
downstream app or test harness, and `sim_world` itself all drive it identically.

## Types

- **`veh_nav_stats`** — one per vehicle: identity (`veh_index`/`convoy_id`/`vehicle_class`/
  `robot_radius_m`/`mass_kg`), outcome (`arrived`/`time_to_goal_s`/`timed_out`/`over_budget`/
  `goals_reached`), motion (`total_path_m`/`straight_m`/`turn_total_rad`/`turn_events`/`time_in_wall_s`/
  speeds), clearance/collisions (`min_clearance_m`/`time_below_clear_s`/`penetration_steps`/
  `veh_contacts`), per-material dwell (`time_over_material_s`/`dist_over_material_m`, `kNumMaterials`
  buckets), effort (`accel_integral`/`fuel_used`), **energy** measured by a physics plant
  (`energy_J`/`energy_per_km`/`building_contacts`, present only when `energy_tracked`: a
  `sim_world` with a plant that tracks energy feeds `nav_samplers::energy_step_J` and
  `static_contacts`; PHYSICS_API.md "cvc::nav's plant"; `publish_nav_stats` writes them as
  `veh.<i>.{energy_J, energy_per_km, building_contacts}` and
  `episode.{total_energy_J, total_building_contacts}`), **terrain**
  (`infeasible_slope_steps`, the ticks its footprint touched slope beyond its limits, present
  only when `terrain_tracked`: a `sim_world` with `set_terrain` feeds
  `nav_samplers::slope_entries`; `publish_nav_stats` writes `veh.<i>.infeasible_slope_steps` and
  `episode.total_infeasible_slope_steps`), the **plant ledger** (below, present only when
  `plant_tracked`), and **drive telemetry** — the reduced per-tick
  `drive_telemetry` (`drive_sample`): policy coefficients (`alpha_mean`/`beta_mean`/`gamma_mean`),
  grip (`mu_mean`/`mu_min`), material risk (`mrisk_mean`/`mrisk_max`), external-force magnitude
  (`ext_force_mean`), steering (`steer_abs_mean`/`steer_abs_max`), `binding_steps`, the material
  weights the drive used (`lam_soft_mean`/`lam_hard_mean`), its turning (`curvature_abs_mean`/
  `curvature_abs_max`, 1/m), and `binding_stopped_steps`, the ticks the barrier was active while the
  vehicle stood still (under `speed_eps_mps`): pinned against what nav takes for a wall, as a vehicle
  sits when its route sends it on and the barrier holds it back. These are
  populated only when the driver captures telemetry (`sim_world::set_capture_drive_telemetry(true)`,
  then feed `drive_sample` from `drive_telemetry_data()`); otherwise `mu_mean` stays 1, `mrisk_mean`
  0 (the no-material/no-grip neutral).
- **`episode_nav_stats`** — one per episode: the reduced fleet fields + `per_vehicle`, and `to_json()`
  (the base record; an extension layer nests its own member itself). The `1e30` "unmeasured" sentinel
  for `min_clearance_m`/`min_sep_m` serializes as JSON `null`, not a huge finite number.
- **`nav_stats_params`** (thresholds) and **`budget_policy`** (time / ETA-multiple / fuel bounds).
- **`nav_samplers`** — optional per-position hooks: `material_id(x,y)`, `occupied(x,y)`, and
  `min_clearance_m` (a `const double*` **in metres** — see the units note below).
- **`nav_scorecard`** + **`aggregate_nav(episodes, checkpoint)`** — reduce a corpus of episodes into
  one domain-neutral fitness row (arrival, economy, safety, material) for ranking base-policy
  checkpoints.
  Includes the fleet drive-telemetry means (`mean_alpha`/`mean_beta`/`mean_gamma`, `mean_mu`,
  `mean_mrisk`, `mean_ext_force`, `mean_lam_soft`, `mean_lam_hard`, `mean_curvature_abs`,
  `mean_binding_stopped_share`) and `material_time_share[kNumMaterials]` — the signals a grip/risk
  tuning or trained-policy A/B is judged on (lower `mean_mrisk` + off-hazard time-share at held
  arrival). A weighted composite score or a grip margin is left to a downstream extension's own
  scorecard, not computed here.

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

`finish()` is `const` and non-destructive: it reduces a copy of the accumulators, so it can be
called every tick as a running snapshot (e.g. to feed `cvc::gl::publish_nav_stats`), and `step()`
can keep accumulating after it.

**2. Via `sim_world`'s opt-in internal collector** (the torch-free native path): arm it before the
first `step()`, and every `step()` folds a tick in pure C++ (safe on the sim thread):

```cpp
world.begin_nav_stats(params, budget, scene_id, seed, checkpoint);  // captures the current pose as start
for (...) world.step();
cvc::nav::episode_nav_stats e = world.nav_stats();
```

Opt-in and default-off, so a plain `sim_world` run is byte-unchanged. Exposed to Python through the
`pycvc` bindings `nav_sim_world_begin_nav_stats` / `nav_sim_world_nav_stats` (JSON).

## The plant ledger — what a physics plant's vehicles did

A `sim_world` driving a physics plant that measures its vehicles (`vehicle_plant::tracks_samples()`;
`cvc::physics::nav_plant` on Jolt) hands the collector each vehicle's `plant_sample`
(`cvc/nav/plant_types.h`) every tick through `nav_samplers::plant` (`sim_world::plant_samples()`).
A sample is SI, in the physics world's frame: the centre of mass's velocity along the heading and
to its left (`vx`, `vy`), the yaw rate `r`, the steering it has and its lock (`steer`,
`max_steer`), nav's commands over the tick (`steer_cmd`, `accel_cmd` in m/s^2), the attitude as
sines (`grade` + nose up, `roll` + left side up) and `up_z` (the body's up axis's world z: 1
upright, 0 on its side), the acceleration over the tick in the heading's frame (`a_lat`, `a_lon`),
the roll rate about the body's forward axis, the lateral load transfer ratio `ltr` (left - right
over left + right of the wheels' loads, a wheel's side from its contact point along the body's
left axis), `wheels` / `wheels_down` and the tick's static `contacts`. Where it overlaps
`cvc::physics::planar_sample` (the plant log) the fields have its names and meaning, and the Jolt
plant computes both alike.

`veh_nav_stats` reduces it (`plant_tracked`, `plant_steps`): **`rolled_over`** latches once the
vehicle stays tipped past `nav_stats_params::roll_up_z` (0.5: 60 degrees) for `roll_hold_s` (1 s),
and `rolled_over_s` is the end of the tick it went over; `tilt_max_deg` / `roll_max_deg` /
`pitch_max_deg`; `tilt_s` (time past `tilt_deg`, 20 degrees); `ltr_max` and `lift_s` (time with
|ltr| past `lift_ltr`, 0.9: a side's wheels about to lift); `wheels_up_s` (two or more wheels off
the ground); `a_lat_max_g`; `steer_cmd_abs_mean` and `steer_sat_share` (ticks steered at
`steer_sat`, 95%, of the lock or more); `accel_cmd_mean` and `brake_share` (ticks it asked to slow).
The episode carries `vehicles_rolled` and `first_roll_s`, and the scorecard, over the runs a plant
fed (`plant_runs`), `rollover_rate`, `episode_rollover_rate` and the runs' means
(`mean_tilt_max_deg`, `mean_ltr_max`, `mean_lift_s`, `mean_a_lat_max_g`, `mean_steer_sat_share`,
`mean_brake_share`). cvcGL's `publish_nav_stats` writes `episode.{vehicles_rolled, first_roll_s}`
and `veh.<i>.{rolled_over, rolled_over_s, tilt_max_deg, ltr_max, a_lat_max_g, steer_sat_share}`.

All of it, and its JSON, appears only when a plant fed it: the kinematic arm's record and
scorecard are byte-unchanged. The grl-snam Python twin drives the kinematic plant, so it has
nothing to mirror and its parity corpora are unaffected. Training does not read the ledger yet;
a rollover penalty in checkpoint selection is a follow-up.

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

## Debugging convoy arrival — `reached` is the honest per-vehicle signal

`veh_nav_stats::arrived` / `time_to_goal_s` latch on the **rising edge of the sim's `reached[i]`**
flag (`nav_stats_params::reach_eps`), i.e. when a vehicle actually gets within `sim_world`'s
`reach_tol` of *its own* goal. That is the per-vehicle truth. A **convoy/harness may carry its own
coarser arrival tolerance** for a whole-column "done" check — e.g. one that grows with the number of
vehicles times their standoff, so a long column's tolerance is many times a single vehicle's
`reach_tol`. That column tolerance is fine as a formation check but **hides a tail follower that
parked short**: the harness can report the whole column at the objective while some followers never
latched `reached`. When you are debugging "did each vehicle arrive?", read the per-vehicle
`arrived`/`time_to_goal_s` (`time_to_goal_s < 0` = never reached), **not** the aggregate column
count.

The per-vehicle records (`to_json()`) are enough on their own to isolate a follower that fails to
arrive. A/B the same scenario with and without the suspect influence (e.g. an extra steering force
injected through `ext_force`): a follower that is **looping** shows `reached=0` with `turn_total_rad`
and `wall_entries` far above the baseline run, while a **frozen** one shows `time_stopped_s` climbing
instead. Two traps turn these stats into noise if ignored: (1) the coarse column tolerance above, and
(2) a harness whose disturbance schedule scales with total run length — hold the run length fixed
when A/B-ing.

## Tests

`src/cvc/tests/nav_stats_test.cpp` (the base accumulators/scorecard over a scripted trajectory, and
the plant ledger over synthetic samples: `PlantLedgerReduction`, `PlantLedgerUntrackedLeavesTheJsonUnchanged`),
`nav_test.cpp`'s `NavSimWorld` suite (the `sim_world` internal collector + `min_clearance_world`), and
`physics_plant_log_test.cpp` (the Jolt plant's `plant_sample` against the plant log, and a Humvee on
its side rolled over). The
scripted corpus + numbers are mirrored in grl-snam's tests (and in any downstream extension that
re-checks the base record) — the shared-schema contract.
