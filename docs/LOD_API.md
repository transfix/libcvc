# Level-of-Detail Selection API (`cvc::lod`)

*Reference for `inc/cvc/lod/select.h` — the single-process level-of-detail
selection math shared by the cvcGL nav demos and the L-System Laboratory.*

## Table of Contents

- [Overview](#overview)
- [What this is, and what it is not](#what-this-is-and-what-it-is-not)
- [Core concepts](#core-concepts)
  - [The rung convention: 0 is finest](#the-rung-convention-0-is-finest)
  - [The error metric](#the-error-metric)
  - [Two decisions the module owns](#two-decisions-the-module-owns)
  - [Hysteresis](#hysteresis)
  - [The budget solver](#the-budget-solver)
  - [Fades and determinism](#fades-and-determinism)
- [User-facing knobs](#user-facing-knobs)
  - [Quality presets](#quality-presets)
  - [Budget profiles](#budget-profiles)
  - [State-tree keys](#state-tree-keys)
- [API reference](#api-reference)
- [Worked examples](#worked-examples)
- [Performance](#performance)
- [Testing and benchmarking](#testing-and-benchmarking)
- [Design provenance](#design-provenance)

## Overview

`cvc::lod` answers one question, cheaply and deterministically, once per visible
group per frame:

> Given where the camera is and how big each thing is on screen, **which
> level of detail should each thing be drawn at**, and **what should be dropped
> or turned into a billboard** so the frame stays inside a triangle / draw-call /
> memory budget?

It is pure arithmetic — **no VTK, no OpenGL, no I/O, and no allocation on the hot
path** — so the Austin nav demos (`nav_city_swarm`, `nav_fog_ghost`) and the
L-System Laboratory both consume it from one place instead of re-deriving it.
The vocabulary (`desired_pixel_error`, priority = *projected area / distance²*)
is deliberately the modernization roadmap's own, so the future networked
streaming layer (`cvc::lod::pyramid_builder` / `lod_index` / the `CvcLod`
service, volrover3 §22.1) slots in on top without renaming anything a demo
already uses.

```cpp
#include <cvc/lod/select.h>
using namespace cvc::lod;
```

## What this is, and what it is not

**It is** the *selection* layer: given per-group bounds and a per-class LOD
ladder, decide rungs and fit a budget.

**It is not:**

| Concern | Where it lives |
|---|---|
| Culling (frustum, terrain-horizon, portals) | `cvc::vis` — `VISIBILITY-AND-LOD-ROADMAP.md` |
| Batching into merged actors, residency, fades on the GPU | `lsyslab_render` / `cvc::gl::BatchedScene` — Lab §8.6 |
| GPU sway / animation LOD (the actual wind vertex shader) | Lab §8.2, `SwayShader` |
| Impostor **atlas baking** | offline `vis_bake` — Lab §6.3 |
| Networked LOD **streaming** (pyramids, eviction) | volrover3 §22.1 — a later effort |

This module gives those layers the numbers they act on. The tile grid,
`fixed_mesh`, GPU sway, and impostor atlas are separate, later PRs
(`RENDER_PERF_ROADMAP.md` phases 1–5; Lab PRs L3–L7).

## Core concepts

### The rung convention: 0 is finest

Rungs count **downward in quality**, exactly as the roadmap ladders are written
(terrain `T0..T4`, vegetation `A0..A4`, buildings `B0..B2`, rocks `R0..R2`). So
`world_error_m[]` is **non-decreasing** in the rung index, `desired_rung` is the
finest a group wants, and `min_rung` is the *coarsest* fallback allowed (and is
numerically `>= desired_rung`). "Promote" means move toward finer — a *lower*
index.

### The error metric

Every rung has a **world-space error** in metres (a one-sided Hausdorff distance
from the finest rung, measured at bake time). Its cost on screen is

```
screen_error_px = world_error_m * k_px / dist          k_px = viewport_h / (2·tan(fov/2))
```

A rung is *affordable* at a distance once its screen error has fallen to
`desired_pixel_error` (default **2.0 px**). The distance where that happens is
its **switch radius**. Because error falls with distance, coarser rungs become
affordable as the camera retreats — a staircase.

`select_rung` forces the error table to be monotonic with a running max, so a
**mis-authored ladder cannot skip a rung or oscillate** across a distance shell;
`ladder_is_monotonic()` reports whether that repair had to happen.

### Two decisions the module owns

1. **Which rung** — `select_rung`, by error, as above.
2. **Mesh vs. impostor** — `impostor_switch_radius_m`, by projected **width**,
   *not* error: a billboard has no meaningful geometric error, so its switch is
   a pure screen-size decision (default 32 px wide). Pick the rung by error out
   to that radius, then draw a camera-facing card past it.

### Hysteresis

The rung boundary is **one-sided**: a group coarsens only past `switch_radius ·
(1 + hysteresis)` and refines only back inside `switch_radius`. Between the two
it holds last frame's rung, so a camera parked on a boundary **cannot
oscillate** — this is proven, not merely made unlikely, in the test suite. Pass
`current = -1` (no history) for a group entering the frustum or for a headless
render, which selects outright with no hysteresis.

### The budget solver

`solve()` takes a list of `candidate` groups and a `budget` (max props, max
triangles, max bytes) and returns a `plan` — a rung per group. It is greedy:
everything starts at its coarsest rung, then groups are promoted toward their
desired rung in **priority** order,

```
priority = projected_area / (1 + (dist_m / 1000)²)      (volrover3 §22.1.6)
```

until a ceiling binds. The plan is a **pure function of its inputs** (ties broken
by `group_id`), so a headless render's plan is bit-identical to an interactive
one. `plan.binding` names the ceiling that stopped the highest-priority
promotion — the one worth surfacing in an LOD overlay's budget bars. An
over-budget *baseline* (even the coarsest world does not fit) is reported, not
silently clamped.

`candidate::props_when_drawn` captures the one place the two roadmaps draw the
same grid differently: **1** when each tile is its own actor (RENDER_PERF phase
1 — the prop ceiling binds first at Austin's ~337 tiles), **0** when tiles draw
through a shared merged actor (Lab §8.6 — the tile count stops costing props).

`solve()` takes candidates by `const&`: the selector reads the scene and writes
only the plan, so *"LOD may never alter simulation correctness"* is enforced by
the type system. For the per-frame path, reuse a `solver` (and one `plan`) so
nothing allocates after the first `reserve()`.

### Fades and determinism

`fade_alpha(elapsed_s, tau_s)` is a **time constant off world dt**
(`1 − exp(−dt/τ)`, volrover3 §22.4.3), never a per-frame ratio, so a cross-fade
looks the same at 30 fps and 240 fps. `tau_s <= 0` returns 1.0 immediately —
the *deterministic* instant-switch every headless or batch render must use so
frames are exact.

## User-facing knobs

### Quality presets

`preset_view(quality_preset)` sets `desired_pixel_error` and `hysteresis`:

| preset | `desired_pixel_error` | notes |
|---|---|---|
| `pristine` | 1.0 px | volrover3 §22.1.6's documented default |
| `balanced` | 2.0 px | the shipped default; halves terrain triangles at an error nobody sees at 1280×800 |
| `aggressive` | 3.5 px | also the wasm profile (wider 0.20 hysteresis) |

A looser error budget never asks for *more* detail at a given distance, and all
three move **every** ladder boundary together — provided ladders are authored
once against the reference (`balanced`) view (see
[`world_error_for_switch_radius`](#api-reference)).

### Budget profiles

`preset_budget(budget_profile)` — per-frame ceilings, all under volrover3
§20.13.7's 4 M-triangle client default with headroom for the shadow re-render:

| profile | max props | max triangles | max bytes |
|---|---|---|---|
| `desktop_large` | 48 | 2.5 M | 700 MB |
| `desktop_default` | 48 | 1.6 M | 380 MB |
| `wasm` | 24 | 700 k | 90 MB |

> `max_props = 48` is a **placeholder** pending the `cvcgl_prop_sweep`
> measurement (Lab §8.7). Treat it as a starting point, not a proven cliff.

### State-tree keys

For interactive tuning the demos expose these under `lab.lod.*` (Lab) /
`demo.lod.*` (nav), snake_case, and **excluded** from the `.lsys` file and the
export manifest so LOD can never change a generated world:

```
lab.lod.preset                = "balanced"    # aggressive | balanced | pristine
lab.lod.desired_pixel_error   = 2.0
lab.lod.hysteresis            = 0.15
lab.lod.fade_tau_s            = 0.12
lab.lod.max_props             = 48
lab.lod.max_triangles_visible = 2500000
lab.lod.cpu_sway_budget       = 24            # constant at every world size
lab.lod.force_rung            = -1            # -1 = auto; pin a rung to inspect
lab.lod.freeze_camera         = false         # fly away and inspect the selection as it WAS
```

## API reference

All functions are `noexcept` and free of allocation unless noted.

**Projection**

| Function | Returns |
|---|---|
| `preset_view(quality_preset)` | a `view_params` for the preset |
| `k_px(view)` | pixels per unit of *(world size / distance)*; 0 for a degenerate camera |
| `screen_error_px(world_error_m, dist_m, view)` | a rung's on-screen error |
| `screen_radius_px(radius_m, dist_m, view)` | a bound's apparent radius in px |
| `switch_radius_m(world_error_m, view)` | distance at/beyond which the rung is affordable |
| `world_error_for_switch_radius(radius_m, view)` | inverse: author a ladder from published switch radii |
| `impostor_switch_radius_m(radius_m, impostor_px, view)` | distance beyond which to draw a billboard (width < `impostor_px`) |
| `bound_distance_m(centre[3], radius_m, view)` | bound-**nearest** eye distance, floored at `z_near` |

**Ladders & rung selection**

| Function | Returns |
|---|---|
| `ladder_is_monotonic(world_error_m, nrungs)` | whether the error table is already a staircase |
| `select_rung(dist_m, world_error_m, nrungs, current, view)` | hysteretic rung; `current < 0` = no history |

**Budget**

| Function | Returns |
|---|---|
| `preset_budget(budget_profile)` | a `budget` for the platform tier |
| `priority(candidate)` | *projected_area / (1 + (dist/1000)²)* |
| `draws(candidate, rung)` | whether the rung rasterizes anything (`tris_per_rung[rung] > 0`) |
| `solve(candidates, budget)` | a `plan` (allocates) |
| `solver::solve(candidates, budget, plan&)` | same, reusing storage — the per-frame path |

**Fades**

| Function | Returns |
|---|---|
| `fade_alpha(elapsed_s, tau_s)` | cross-fade weight; `tau_s <= 0` → instant 1.0 |

## Worked examples

**Author a ladder from the radii the roadmap publishes.** Do this **once**,
against the reference view, and store the errors — that is what lets the runtime
preset move every boundary together:

```cpp
// Buildings: B0 full shell to 200 m, B1 shell+windows to 900 m, B2 mass box beyond.
const view_params ref = preset_view(quality_preset::balanced);
double bldg_err[3] = {
    0.0,                                       // B0 is the reference; no error
    world_error_for_switch_radius(200.0, ref), // B1
    world_error_for_switch_radius(900.0, ref), // B2
};
```

**Per-frame rung selection with hysteresis:**

```cpp
view_params view = preset_view(quality_preset::balanced);
view.eye[0] = cam.x; view.eye[1] = cam.y; view.eye[2] = cam.z;
view.viewport_h_px = fb_height;

const double dist = bound_distance_m(tile.centre, tile.radius_m, view);
tile.rung = select_rung(dist, bldg_err, 3, tile.rung /* last frame */, view);

// Representation: past the width crossover, draw a card instead of the mesh.
const bool impostor = dist >= impostor_switch_radius_m(tile.radius_m, 32.0, view);
```

**Fit a tile grid to the frame budget (steady-state, allocation-free):**

```cpp
// Once:
solver slv;  slv.reserve(tiles.size());
plan   plan; plan.rung.reserve(tiles.size());
const budget budget = preset_budget(budget_profile::desktop_default);

// Every frame:
candidates.clear();
for (const auto& t : tiles) {
  candidate c;
  c.group_id       = t.morton;
  c.nrungs         = 3;
  c.desired_rung   = select_rung(t.dist, bldg_err, 3, -1, view);
  c.min_rung       = 2;               // coarsest fallback
  c.projected_area = screen_radius_px(t.radius_m, t.dist, view);  // area proxy
  c.dist_m         = t.dist;
  c.tris_per_rung  = t.tris;          // caller-owned, 3 entries
  c.bytes_per_rung = t.bytes;
  c.props_when_drawn = 1;             // one actor per tile (RENDER_PERF phase 1)
  candidates.push_back(c);
}
slv.solve(candidates, budget, plan);
// plan.rung[i] is tile i's rung; plan.binding says which ceiling bound.
```

## Performance

Measured on the real Austin bundle (978,242 triangles, 337 occupied 128 m
tiles, camera at the south edge), building ladder above:

| preset | planned triangles | vs. full mesh |
|---|---|---|
| pristine | 263,500 | 3.7× fewer |
| balanced | 192,900 | 5.1× fewer |
| aggressive | 162,100 | 6.0× fewer |

— inside `RENDER_PERF` phase 1's 4–6× target.

Per-frame cost (release, one desktop core): `select_rung` ≈ **18 ns/call**;
`solve()` for a 1024-tile grid ≈ **3.5 µs/frame** with a reused solver (and no
slower than the allocating `solve()`), negligible against a 16 ms frame.

## Testing and benchmarking

- **`src/cvc/tests/lod_select_test.cpp`** — 36 exact, hand-checkable cases:
  crossover round-trips, oscillation-is-*impossible* under hysteresis, a
  non-monotonic ladder cannot skip a rung, order-invariance of the plan, each
  budget ceiling binding, the width-based impostor switch, and the Austin frame
  under both the per-tile-actor and shared-merged-actor drawing shapes.
- **`src/cvc/tests/lod_select_bench_test.cpp`** — micro-benchmark, `GTEST_SKIP`ped
  unless `CVC_LOD_BENCH=1`, so a bare `ctest` ignores it:

  ```bash
  CVC_LOD_BENCH=1 ctest --test-dir build -R LodSelectBench -V
  ```

  It also asserts the reused solver is not materially slower than a fresh
  `solve()` — a regression tripwire on the "no allocation on the hot path"
  claim.

## Design provenance

`cvc::lod::select` reconciles the two independent selectors that landed in
PR #249 rather than implementing both:

- `LSYSTEM-LABORATORY-ROADMAP.md` §8.5 — the selector, thresholds, and state keys.
- `VISIBILITY-AND-LOD-ROADMAP.md` §6.1 — bound-nearest distance, forced
  monotonicity, and the width-based representation switch.
- `RENDER_PERF_ROADMAP.md` — the Austin targets and the phase ordering
  (animation → generation → update → draw calls → triangles) this selector sits
  under.
- volrover3 modernization roadmap §20.13 / §22.1 — the client-LOD vocabulary the
  streaming layer will reuse.

The header itself records, decision by decision, which roadmap each choice came
from and why.
