# cvc::nav Units Roadmap — dimensioning velocities, lengths and time

> **Goal:** make every physical quantity `cvc::nav` reports or accepts carry a
> correct, explicit dimension — length, velocity, acceleration, **time** —
> anchored to the two canonical bases the library now has: `cvc::world_units`
> (the spatial/unit base) and `cvc::world_clock` (the simulation-time base) —
> **without changing a single number the bit-identical GRL-SNAM kernels compute.**

## Why this is needed

`cvc::nav` is a bit/float-identical C++ port of a Python GRL-SNAM stack. It has
**no typed units anywhere**, and it runs in three implicit frames:

- **world** — bounds/positions/velocities, "metres by convention" only, never
  enforced or typed.
- **normalized/centered** — `on = (w − center)·S`, the dimensionless frame where
  **all** kernel math, all learned coefficients, and all persisted
  `.cvcnav`/CoefEnergyNet weights live. `S = config.scale`.
- **grid cells** — `cell_w = (max_x − min_x)/(cols − 1)` world-units per cell.

A handful of fields carry an `_m`/"metres" name (`range_m`, `phi_m`,
`hard_margin_m`, `d_hat_m`, `d_hat_sdf`, `k_sharp` = 1/m) but they are actually
**world units by convention** — genuinely metres only when one world unit is one
metre, which is exactly the invariant `world_units::metres_per_world_unit` makes
explicit. Time is "seconds by convention" (`dt`) with **no authoritative clock**
behind it.

### The velocity trap (the motivating bug)

Every speed and acceleration **inside the kernels is in the NORMALIZED frame**,
not metres: `veh_params.vmax`, `a_max`, `a_lat_max`, `L`, `rr`, `d_hat`,
`body_offsets`, `carrot_params.a_max`, and every derived `v_corner/v_stop/…`
(`inc/cvc/nav/drive.h:124–182`). Labelling `vmax` as "m/s" as-is is **wrong by a
factor of `S`**. The exceptions that need no `S`: `dt` (time), `delta_max`/
`k_steer` (radians/dimensionless), `mass` (nondimensional `1.0`).

Today the only correct conversion lives in `sim_world::snapshot`
(`src/cvc/nav/sim_world.cpp:700–707`): positions via `o/S + center` (world
metres), speed via `sp_/S` (world m/s), heading returned raw (radians — the map
is isotropic).

## The three "scales" (never conflate them)

| Symbol | Meaning | Owner | Dimension |
|---|---|---|---|
| **`S`** = `config.scale` / `field_stack.S` | world→normalized numerical **conditioner** (keeps coords ~O(1)) | **nav** (trace-sensitive) | *not a unit* (≈ 1/length) |
| **`mpwu`** = `metres_per_world_unit` | SI length of one world unit — **new**, currently implicit `1.0` | **`world_units`** | length pin |
| **`cell_w`** = `(max_x−min_x)/(cols−1)` | metres-per-grid-cell (derived, x-extent only) | nav (derived) | length |

`mpwu` and `S` are **independent multipliers**, so introducing `mpwu` cannot
perturb `S`. The conversion chain is two-step:

```
SI  <—(× metres_per_world_unit)—>  world-metres  <—(× S)—>  normalized
```

Velocities and accelerations convert by the **same** chain as length, because
time is seconds: `v_world_mps = sp_ / S`, then `world_units::to_display(v,
velocity)`. This is why `snapshot`'s existing `/S` is exactly right and is the
template for every other boundary.

## Time: lock the simulation-time dimension to `cvc::world_clock`

`world_units` gives velocity its **metres**; `world_clock` must give it its
**seconds**. `cvc::world_clock` is the authoritative simulation clock — a
fixed-quantum `fixed_dt`, an integer `tick()`, and derived `t() = tick·fixed_dt`
(store the integer, derive the seconds — never accumulate). nav's timestep is
today a magic constant (`veh_params.dt`/`carrot_params.dt` ≈ `0.06`, "world dt"),
so "m/s" is only honest by assertion.

**Bind nav's tick to `world_clock`:**

- One nav step is **one `world_clock` quantum**; `veh_params.dt` is sourced from
  `world_clock::fixed_dt()` at config construction, so "one tick = `fixed_dt`
  real seconds" is explicit and authoritative rather than conventional.
- **Direction of the binding matters (trace safety).** `dt` is trace-sensitive:
  the policy and every `.cvcnav` weight were trained at a specific `dt` (the
  GRL-SNAM `0.06 s`). So the authoritative value flows **nav-trained-dt →
  `world_clock.fixed_dt`**, not the reverse — configure the clock's quantum to
  the trained `dt` for a nav-driven sim; nav then reads `dt` back from
  `world_clock.fixed_dt()`, keeping the number byte-identical to the trained
  metric. Choosing a *different* `fixed_dt` is a retrain, not a config change.
- **Drive the sim loop from the clock.** Advance nav by
  `world_clock::advance(wall_dt).steps` whole quanta per frame, so nav integrates
  in fixed `fixed_dt` steps (deterministic, matching the clock's fixed-step
  contract), `world_clock::t()` is the authoritative sim time, and nav's internal
  tick count tracks `world_clock::tick()`.
- **Result:** length from `world_units` (metres via `mpwu` + world/`S`), time from
  `world_clock` (seconds via `fixed_dt`) ⇒ `sp_/S` is provably **metres per
  second** and `a/S` provably **m/s²**. This closes the implicit "tick == second"
  gap the output-boundary audit flagged (there is no `seconds_per_tick` anywhere
  today).
- **Boundary only.** `world_clock.fixed_dt()` feeds `veh_params.dt` at config
  construction and the demo/host sim loop calls `world_clock.advance()`. No
  kernel sees `world_clock`; `dt` keeps its trained value.

## The invariant that keeps this safe

**Convert only at three seams — config-in, the output accessors, and demo
constants/display — never inside a kernel.** At `metres_per_world_unit == 1.0`
and `fixed_dt ==` the trained `dt`, every golden trace stays byte-identical
because the kernels never see `world_units` or `world_clock`.

## Additional issues the audit surfaced (all fixable at boundaries)

1. **`config.range_m` is misnamed** — it is world-units passed into the sensor
   kernel with *no* scale conversion (`grid_nav.h:220` says "world units"; the
   `_m` name claims metres). Genuinely metres only at `mpwu == 1`. → rename
   `range_world`, or accept SI and convert at the single input line
   `sim_world.cpp:303` (guarded so `mpwu == 1` is bit-identical).
2. **`k_sharp` is `1/length`**, for which `world_units::dimension` has no member.
   → store a **barrier smoothing distance** `= 1/k_sharp` (a length) and derive
   `k_sharp`; that is the clean `world_units` fit.
3. **No real physics units** — `mass` is `1.0` nondimensional; there is no kg,
   newton, or gravity anywhere; "forces" are dimensionless normalized-frame field
   forces. Making them real (kg/N/gravity) requires re-dimensioning the learned
   `alpha/beta/gamma/lam_*` coefficients and a full **retrain** of every
   `.cvcnav` weight — it is a **separate, out-of-scope** effort that belongs to
   the physics/Jolt bridge, not to this units pass.
4. **`snapshot` speed convention** — `sp_/S` is genuinely m/s *given* `dt` is
   real seconds (Phase 0 verifies `sp_` is per-second: the integrator does
   `ox += hdt·spi`, `hdt = dt/nsub`). Making `dt` come from `world_clock`
   (above) is what turns this convention into a guarantee.

## DO NOT TOUCH (bit-identical / ABI / trained weights)

A units/time layer must **not** change any stored float, struct layout, or the
normalized kernel math:

- `src/cvc/nav/grid_nav.cpp` kernels — float64 parabola-envelope EDT, heapq-order
  A*, `sense_batch`; no `-ffast-math`/`-ffp-contract=fast` (`grid_nav.h:36–42`).
- `src/cvc/nav/material.cpp` `material_build` / `witness_gate` — compiled
  `-ffp-contract=off` / `/fp:precise` (`src/cvc/CMakeLists.txt`); op order is the
  contract (incl. `phi_m = sqrt(edt2)·cell_w`, `denom_n = cell_w·scale`).
- `veh_params` float32 fields/defaults/layout (`drive.h:124–182`) — the trained
  metric; changing a value or the layout changes the trace and invalidates
  `.cvcnav`.
- Borrowed torch layouts: `field_stack`/`friction_field` (`drive.h:58–79`),
  `material_stack` `[M,6,H,W]` channel order (`material.h:52,69–75`) — cross-ABI
  (torch/pycvc/CUDA) contracts.
- `.cvcnav` byte format + `coef_mlp` `Layer` layout + `CoefEnergyNet` weights —
  weight-parity contracts.
- The `/S + center` float32 arithmetic already inside `snapshot`/`goals_world`/
  `carrots_world` — keep evaluation order/casts stable; add `world_units` as a
  downstream step, do not rewrite these expressions.

## Phased plan

Each phase is independently shippable and needs **zero kernel edits**.

- **Phase 0 — Document, assert, verify (zero risk).** Annotate every nav field
  with its frame + `world_units::dimension`; document the three scales, the
  `world_clock` time binding, and the two conventions; fix `range_m` naming/docs;
  add a boundary unit-audit test (mirror `nav_material_test.cpp:113`, which
  already checks `phi_m == 3·cell_w`) that round-trips `phi_m`/`clear_m`/
  `hard_margin_m`/`d_hat_m` through `world_units` length; verify `sp_` is
  per-second.
- **Phase 1 — SI input builders (config seam).** New non-kernel TU
  `src/cvc/nav/veh_units.cpp`: `veh_params_from_si(world_units::config,
  world_clock, scene_scale, …)` converting SI (m, m/s, m/s², rad) → normalized
  floats, sourcing `dt` from `world_clock::fixed_dt()`; `material_config` SI
  setters that store metres and a barrier-distance (not `k_sharp`); derive
  `cfg.scale` from `mpwu` + scene bounds so the trained metric (`scale 0.05`,
  ±100) reproduces exactly; guarded `range_m` SI conversion.
- **Phase 2 — SI output accessors (display seam).** A header-only `cvc::nav ↔
  world_units` bridge `(value, dimension, world_units::config)` — position →
  length, speed → velocity, heading → angle (isotropic; only rad→deg on display);
  an optional `snapshot_si`; carry a `world_units`/`world_clock` tag on the
  `sim_thread::snapshot` struct so a renderer converts without re-deriving `S`,
  `center`, or `fixed_dt`.
- **Phase 3 — Demos derive from the bases.** One `navdemo` helper
  `(world_units, world_clock, Bounds) → {scale, cx, cy, veh_params}` replacing the
  per-demo hardcoded `scale = 0.05` / `worldIsMetres` toggle
  (`nav_city_drive.cpp:374,1004`); drive the loop from `world_clock::advance()`.
- **Phase 4 — (future, separate) real dynamics units.** kg / newton / gravity for
  the Jolt bridge: re-dimension the learned coefficients + retrain. Explicitly
  **not** part of this effort.

## Verification strategy

- **`mpwu = 1.0`, `fixed_dt =` trained `dt` → byte-identical golden traces** (the
  existing nav fuzz/golden harness must be unchanged).
- **`mpwu = 2.0` → only the reported metres / (m/s) scale**, never `phi`, paths,
  or policy behaviour.
- Boundary unit-audit tests assert `phi_m`, `clear_m`, `hard_margin_m`, `d_hat_m`
  round-trip through `world_units` length, and that `veh_params_from_si` +
  snapshot recover the input SI velocity to within float tolerance.

## Relationship to the rest of the modernization

`world_units` (spatial base, landed) and `world_clock` (temporal base, landed)
are the two anchors; this roadmap makes `cvc::nav` a disciplined *consumer* of
both at its edges while its bit-identical core stays in the dimensionless
normalized frame. The same pattern extends to any future physics engine, which
exchanges SI positions/velocities/forces across its boundary with only
`metres_per_world_unit` and `fixed_dt` applied.
