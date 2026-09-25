# nav_stats intrinsic-stats + formation promotion — cross-repo roadmap

Master plan for promoting the formation stats and the sim-intrinsic quantities into the shared
`cvc::nav` `nav_stats` contract, syncing them live into the `cvc::state` tree, and wiring them into
training. Specs (design of record) live in **cvcdbg-nativedemo**:
`docs/nav-stats-design.md` §10 (schema), `docs/sim-intrinsic-stats.md` (catalog), and
`docs/stats-statetree-and-training.md` (state-tree + training). This doc is the *execution* plan and
tracks status; the specs are the *what/why*.

## Placement (the load-bearing decision)
Formation + all **non-RF** sim internals live in the **`cvc::nav` base** (`veh_nav_stats` /
`episode_nav_stats` / `nav_scorecard` + the `grl_snam.metrics` Python twin) — formation is a navigation
concept (the grl-snam swarm can hold formations too). `cvc::dbg` adds **only RF** fields, joined to the
base by `(veh_index, convoy_id)`. Every field is **additive / off-by-default** (`formation_tol_m=0`,
null samplers) so no pinned parity number moves until a case exercises it. The C++ and Python corpora
are held byte-identical by the shared hand-computed fixture (`nav_stats_test.cpp` ⟷ `test_scorecard.py`).

## Tracks & PR breakdown (ordered)

### Track 1 — libcvc `cvc::nav` base  ← START HERE
The schema owner moves first; everything downstream depends on it. Sub-PRs (each additive, green-gated):
- **1a — Formation fields** *(MERGED, transfix/libcvc#421)*: `veh_nav_stats.{formation_parent,
  slot_error_mean_m, slot_error_max_m, formation_arrived}`; `nav_stats_params.formation_tol_m`;
  `nav_samplers.formation_slot` (per-vehicle slot target, null ⇒ off); `set_identity(..., formation_parent)`;
  collector accumulates slot error + latches `formation_arrived`; `nav_scorecard.{form_arrival_rate,
  form_mission_rate, mean_slot_error_m}` + `aggregate_nav`; `to_json` emit. Existing corpus numbers
  unchanged (feature off). (Geometry integrity/compression stats deferred to Track 3 with the harness.)
- **1b — stall / closest-approach** *(implemented, in review)*: `veh_nav_stats.{stall_steps,
  closest_approach_m}` + `nav_stats_params.stall_progress_eps_m` + `nav_scorecard.{mean_stall_steps,
  mean_closest_approach_m}`. **Computed collector-side from the snapshots + goal it already holds — NOT
  from `sim_world` getters.** On inspection `sim_world::stall_` is a reset-happy streak counter (zeroed
  on escapes/mode-transitions) and `best_`/`init_` are in normalized units; neither exists in grl-snam,
  so reading them would break the C++⟷Python fixture-parity invariant. Portable definition:
  `closest_approach_m` seeded to `straight_m` at `begin_episode`, then min of per-step distance-to-goal;
  `stall_steps` = steps that fail to beat the closest-so-far by more than `stall_progress_eps_m`
  (default **0.05 m — a shared cross-repo constant; grl_snam.metrics must match it**).
- **1c — belief/fog coverage**: `episode_nav_stats.{explored_frac, visible_frac, believed_free_frac,
  phantom_frac}` per belief plane, reduced from the already-public `truth()`/`belief_occ(m)`/
  `ever_seen(m)` rasters; `sense_flips` per vehicle (retain `flips[]` from `sense_batch` + getter).
- **1d — drive telemetry**: one per-tick `drive_telemetry` struct out of `drive_step`
  (`mu`, `mrisk`, applied `ext_force` mag/angle, CoefMLP `al/be/ga`(+`lam_soft`), steer δ, driven
  curvature, binding governor, `d<d_hat`); collector reduces per-vehicle means/peaks into
  `veh_nav_stats`. Deepest (touches `drive.cpp`/`drive.h`); do last in the track.
- Each sub-PR extends `nav_stats_test.cpp` with the feature ON in a small added case; the pre-existing
  fixture stays byte-identical.

### Track 2 — grl-snam Python parity (`grl_snam.metrics` + corpus)
Mirror every Track-1 base field field-for-field in `NavStats`, add the `formation_slot` sampler hook,
update the shared hand-computed corpus so `test_scorecard.py` matches `nav_stats_test.cpp`. One PR per
Track-1 sub-PR (or batched), gated on Track-1 landing.

### Track 3 — cvcdbg (RF ext + state bridge + harness re-point)
- **3a — RF ext fields**: fill the reserved `veh_rf_stats`/`episode_rf_stats` slots
  (`fjam/fbw/fsw` via a 4-way `comm_accel` split; `eff_rate`, `bandwidth_slack`, `backbone_uptime` via
  `comm_step_output`; `outage_prob`; `pingpong_rate`) + CoefNet `heads()` + RF-specific formation
  (per-slot RF exposure, connectivity) joined by `(veh_index, convoy_id)`.
- **3b — state-tree bridge** (SPEC 1): `cvc::gl/nav_stats_publish.{h,cpp}` (base+formation) +
  `cvc/dbg/nav_stats_publish.{h,cpp}` (RF), wired into `dbg_austin_live3` at the `finish()` seam; per-sim
  prefix keying; `nav_stats.sims` registry. Then the ImGui stats panel / DSL binding.
- **3c — harness re-point**: `dbg_arrival_check3` sets `formation_tol_m`, passes `formation_slot`,
  `set_identity(..., formation_parent)`, and drops the harness-side `FORMSTATS`/`form_*` bookkeeping in
  favour of the collector's; the scorecard then carries it natively.

### Track 4 — training (SPEC 2)
- **4-Phase 0**: scorecard plumbing — a Python reader over C++ `aggregate_nav`/`aggregate_rf` (no
  trainer consumes them today); guard the C++⟷Python fixture parity gate.
- **4-Phase 1** (SELECTION): recorded RF into `CommTrainer._composite_score`; formation / belief /
  grip-margin as `nav_scorecard` selection fields.
- **4-Phase 2** (LOSS on existing rollouts): RF exposure (`w_exposure>0` + ext-force rollout; re-baseline)
  and grip anticipation (`train_bicycle` + `material_train.h` forward/vjp), reusing the material CVaR
  pattern.
- **4-Phase 3** (new surrogates): coupled multi-agent **formation** surrogate; coverage stays
  SELECTION-only; RF true-field stays eval-only (the comm-steering-objective-gap).

## Status
- [x] Specs written & merged (schema §10, catalog, state-tree+training) — cvcdbg #125/#126.
- [x] Formation stats collected harness-side (`FORMSTATS`) — cvcdbg #120–#126.
- [x] **Track 1a — libcvc formation base fields — MERGED transfix/libcvc#421.**
- [~] **Track 1b — stall / closest-approach — implemented + adversarially reviewed; PR next.**
- [ ] Track 1c/1d; Track 2; Track 3; Track 4 — pending, in order.

## Invariants for every PR here
Additive / off-by-default; C++⟷Python fixture parity held; torch ⟷ torch-free (`material_train.h`)
parity for any differentiable term; base stays RF-free (only `cvc::dbg` links RF). Any threshold that a
downstream twin must match to reproduce a field bit-for-bit (e.g. `stall_progress_eps_m = 0.05`) is a
shared cross-repo constant — change it only in lockstep across libcvc + grl_snam.metrics.
