# cvc::nav vehicle refinements — footprint, steering lock, grip

Four optional additions to the torch-free bicycle drive (`inc/cvc/nav/drive.h`).
Each is **inert at its default** — a zero count, a zero width, a unit gain, a
null pointer — so an unmodified caller gets the legacy trace bit-for-bit and
every stored `.cvcnav` weight stays valid.

They live in `veh_params` rather than behind new entry points, so
`bicycle_rollout`, `bicycle_rollout_material`, `drive_step` and the CUDA path
all pick them up with **no signature change**.

The torch reference and the measured navigation results are in the grl-snam
repo: `docs/VEHICLE_REFINEMENTS.md`.

## Why there is no "Ackermann rollout"

The kinematic bicycle is already the exact kinematic reduction of an Ackermann
axle — `delta` is the *virtual* centre-wheel angle. A separate Ackermann rollout
would produce identical trajectories. What differs is the footprint, the
inner-wheel lock, and grip.

## `veh_params`

```cpp
struct veh_params {
  float rr, d_hat, dt, vmax, L, delta_max, a_max, a_lat_max, k_steer;
  int nsub;
  bool allow_reverse;

  const float *body_offsets = nullptr;  // [n_body], borrowed, along the heading
  int   n_body     = 0;                 // 0 = legacy single rr-disc
  float body_rr    = 0.0f;
  float body_gain  = 1.0f;              // SET THIS to 1/n_body
  float track_width = 0.0f;             // 0 = no inner-wheel lock
  const friction_field *grip = nullptr; // null = mu == 1 everywhere
};
```

**Footprint.** `n_body` discs of radius `body_rr` at the given longitudinal
offsets. Clearance is the MIN over discs (and the governor steers by *that*
disc's normal — "am I driving into the nearest wall" is a question about the
binding disc); the barrier force is their SUM. `rr` is then unused by the drive,
and the governor / creep / nose-blocked margins switch to `body_rr`.

**`body_gain`.** The SUM is a K-times gain on the learned `al`, which was fit
for one sample point. `1/n_body` cancels it exactly, so K coincident discs
reduce to one disc. Uncorrected this cost the grl-snam city story its entire
reach (45% → 0% at matched radius) while *improving* standoff and collision
rate; corrected it recovers to 35% and keeps both. Defaults to `1.0`, the
physically literal sum, so the parity numbers below describe that path.

**`track_width`.** The inner wheel reaches the mechanical lock first, so the
achievable virtual angle is `atan(L / (L/tan(delta_max) + t/2))`. At
`t = 0.6 L`: 14% less steer, 20% larger `R_min`.

**`grip`.** A `friction_field` — one plane, same `[M][H][W]` layout and
world↔grid constants as `field_stack`. `mu = 1` is the reference dry surface the
vehicle constants are already quoted against. Both actuator limits are
grip-limited, so `a_max` and `a_lat_max` scale together.

Kept deliberately separate from `material_stack`: risk and grip are independent
surface properties, and the 6-plane bit-identical `material_sample` twin stays
untouched.

## Entry points

| function | notes |
|---|---|
| `bicycle_rollout` | CPU, threaded |
| `bicycle_rollout_material` | CPU + the material force |
| `bicycle_rollout_ext` | CPU + a generic external force (e.g. a downstream layer's steering force) |
| `bicycle_rollout_material_ext` | CPU + **both** the material force AND the external force, fused |
| `bicycle_rollout_cuda` | GPU, **given** coefficients — the unfused device twin |
| `drive_step`, `drive_step_material` | fused: `coef_feats` → MLP → rollout |
| `drive_step_ext` | fused, with an external force channel |
| `drive_step_material_ext` | fused, with **both** the material drive AND an external force |
| `drive_step_cuda` | fused on GPU, one thread per agent |
| `sim_world_cuda` | device-resident world; holds its own refinement buffers |

`*_material_ext` fuses the two force channels. Without it a caller must pick one
— and `sim_world::step()` gives the material path priority, so attaching a
material stack silently dropped a set `ext_force`. `rollout_impl` already sums
both the material and external forces into one accumulator and one steering bias,
so the fused entry points just pass BOTH; a null `ext.sample` reproduces
`drive_step_material` byte-for-byte, a null material stack reproduces
`drive_step_ext`. `sim_world::step()` now selects `drive_step_material_ext`
whenever a material stack AND an external force are both attached, so a run can
steer on a learned grip/risk policy WHILE the external force also pushes.

`bicycle_rollout_cuda` was added with earlier work. CUDA previously had only
`sdf_sample_cuda` and the *fused* `drive_step_cuda`, so the device vehicle math
could not be compared against the reference without dragging a trained net
through the comparison — which is why `drive.cu` had been shipping
compiled-but-unattested.

`drive_cuda_available()` reports whether a device is actually present;
`false` in a non-CUDA build. Use it to skip rather than die in `cudaMalloc`.

## The sixth feature

`coef_feats(..., grip)` appends the sampled mu, making the stride 6 instead of
5, matching `sdf_nav.coef_feats(friction=...)`. `drive_step` and
`drive_step_cuda` read the width off `model.in_features()` rather than assuming.

A 6-feature net is not a retrain from scratch: `sdf_nav.widen_coef_mlp` lifts a
trained 5-feature net to one whose mu column is zero, which is output-identical
at init.

## Refusals

A native path honouring fewer constraints than the torch reference is the "fast
digital twin that moves differently" failure, and **no parity gate would catch
it** — the gates hand both paths the same params. So where a width or a field is
missing, the code raises instead of running:

- `drive_step` / `drive_step_cuda` refuse a model whose input width is neither 5
  nor 6, and refuse a 6-feature model with a null `grip`. Feeding the first
  layer a short vector is arithmetic that succeeds and is wrong.

`dev_veh`'s new members carry in-class initializers on purpose: every
construction site in `drive.cu` is a bare `dev_veh v;` followed by field
assignments, so without them the pointers would be indeterminate and the kernel
would dereference garbage.

## Testing

`src/cvc/tests/nav_drive_refinements_test.cpp` — 10 cases.

```bash
ctest -R NavDriveRefinements
```

It is **torch-independent by construction**: the inertness checks are internal
identities, so the gate holds with no reference implementation to hand.

- a one-disc footprint IS the legacy sample
- K coincident discs at gain `1/K` ARE one disc
- a uniform `mu = 1` plane IS the legacy actuator envelope

Byte-identity is asserted with `operator==`, not a tolerance: the whole value of
the defaults being inert is that stored `.cvcnav` weights and golden traces keep
working, so a near-miss is a failure.

Under `CVC_ENABLE_CUDA` it also pins `bicycle_rollout_cuda` against the CPU
rollout with every refinement on, at the ~1e-5 float-equivalence tolerance (not
bit-exact — the `.cu` stores world bounds as float where the CPU keeps them
double). It skips when `drive_cuda_available()` is false, because a CUDA-enabled
*build* is not a CUDA-capable *machine*: CI compiles some jobs with CUDA on
runners that have no GPU.

**Registering a new test is not optional.** `src/cvc/tests/CMakeLists.txt` fails
at configure time if a target is in `TEST_TARGETS` but never passed to
`cvc_discover_tests()`. That guard exists because `nav_coef_train_test` compiled
green for weeks while ctest ran none of its cases — a test that never runs is
worse than no test, because it reads as coverage.

### Parity against torch

The float-equivalence evidence comes from an out-of-tree harness that links
`drive.cpp` / `drive.cu` directly and diffs against `sdf_nav.bicycle_rollout`
(the SWIG binding does not expose the new `veh_params` fields yet, so the usual
Python gate cannot reach them):

| case | CPU | GPU (RTX 3050 Ti, sm_86) |
|---|---|---|
| **legacy (regression gate)** | **0.0 — bit-identical** | 2.98e-08 |
| footprint | **0.0 — bit-identical** | 2.98e-08 |
| footprint + gain 1/3 | **0.0 — bit-identical** | 2.98e-08 |
| steering lock | 1.79e-07 | 5.96e-08 |
| grip | 1.49e-08 | 1.19e-07 |
| all four | 1.79e-07 | 5.96e-08 |

against the ~1e-5 contract. The legacy zero is the one that matters: the
refactor (hoisted `ch`/`sh`, branched `F_rep`, `gov_rr`) perturbed nothing.

## Status

Not yet verified end-to-end: the SWIG binding is **compile-validated, not
runtime-validated**. `swig -c++ -python` generates clean and the wrapper
compiles against the real headers, but the link and import are unverified
because `numpy-cp312` / `python312` are absent from the cvcpkg catalog for this
platform tuple, so a local pycvc rebuild needs CPython and numpy from source.
It ships when pycvc is next republished — which also needs the usual
`cvc_revision` bump in the `pycvc-cp31X` recipes.

CUDA parity is measured on one device (sm_86) and is **not** a CI gate: CI has
no GPU, so `CudaMatchesCpuWithEveryRefinement` skips there.

## Routes a vehicle can drive: `plan_vehicle_route`

`inc/cvc/nav/vehicle_route.h`. `grid_nav::astar` plans for a point that turns on the spot. A
vehicle has a turning circle, a body and a reverse gear: a Humvee needs a 6.7 m circle and is
4.8 m long. In a street narrower than that circle it cannot follow an 8-connected path round a
corner, and it cannot turn back the way it came. The physics plant's Humvees wedge in exactly
those places (the vehicle-physics roadmap, P9).

`plan_vehicle_route` searches over the rear axle's pose (Hybrid A*):

- **The moves.** Each move is an arc of `step` at a curvature of -1, -0.5, 0, 0.5 or 1 times
  `1 / min_radius`, forward or backing up. The default step is 1.5 cells, but at most
  0.6 x `min_radius`. The closed set is keyed at step / 1.5 whatever the grid, so a coarse grid
  still turns in arcs it can resolve: Austin's 7.8 m cells are larger than a Humvee.
- **The footprint.** The footprint discs (`sim_world::plant_plan_of` gives a plant vehicle's) must
  keep `margin` from the walls. The walls' distance comes from the grid's exact distance
  transform, sampled between cells.
- **The cost.** The length driven; backing up costs `reverse_cost` (2) as much, and a change of
  direction `cusp_cost` (4 steps) more. A little is added for steering, for changing the steer,
  and for coming nearer a wall than `comfort`.
- **The heuristic.** The free-space distance to the goal, weighted by `heuristic_weight` (1.5): the
  route is at most that many times dearer than the cheapest, for far fewer expansions.

A dead end, or a corner too tight to take forward, comes back as a multi-point turn: the poses
between cusps are marked `reverse`. A vehicle that starts wedged, already inside its margin or
touching a wall, may stay as close as it is for its first few moves, so the route can lead it
out. A goal it cannot reach is reported, and the route ends at the nearest pose it found.

Measured (`nav_vehicle_route_test`, the Humvee on 1 m cells):

| Case | Route |
|---|---|
| Open ground, goal 70 m ahead | Straight, 70 m |
| An 8 m dead end, facing its end, goal behind | Backs out (42 m, all backing up); nav's bicycle turns round in the street (45.4 m, 1.2 m of it a backing nudge) |
| A right angle between 9 m streets | Forward, 106.5 m, 93 expansions |
| Wedged, the nose 0.4 m into the end wall | The first move backs away; clear of the walls after it |
| A 384 m city of 40 m blocks and 12 m streets | 648 m in 740 expansions and 8 ms (exact, heuristic x1: 636 m in 168,000 expansions and 0.63 s) |

Every route turns no tighter than the vehicle can, and keeps its body clear of the walls.

### Following a route: `route_follower` (P9 R2)

`inc/cvc/nav/route_follower.h` drives a sim_world's agents along their routes. A route splits into
legs at its cusps, and each leg is driven in one gear. Call `apply()` before each `step()`:

- **The last forward leg** belongs to the trained drive. The agent is retargeted to a carrot
  `lookahead` along the route, so it keeps off walls and parks at the end as it always has.
- **A leg that ends at a cusp** is driven geometrically: backing up, or the forward leg of a
  multi-point turn. It uses pure pursuit about the rear axle, in the leg's gear, at
  `manoeuvre_speed` (1.5 m/s). It slows so it stops at the cusp, then takes the next leg.
- **A route that ends backing up** stops at its end, and the agent is retargeted there.

It commands through `sim_world::override_command(i, steer, accel)`, which replaces the drive's
command for one step:

- on the kinematic arm, the bicycle is integrated with it;
- on a plant, it becomes the plant's command.

`sim_world::set_pose` places an agent on the kinematic arm. A plant owns its vehicles' poses.

Measured, a Humvee facing the end of an 8 m dead end with its goal behind it outside:

| | The trained drive alone | On a vehicle route |
|---|---|---|
| Kinematic arm (`nav_route_follower_test`) | Wedged at the end wall, 53 m from the goal after 90 s | Backs out 42 m to the goal in 29 s, never nearer a wall than 3.1 m |
| Jolt plant (`nav_plant_test`), the plant's own footprint and 6.71 m radius | 55 m from the goal after 90 s | Backs out 48 m to the goal in 31 s |
| Kinematic arm, to a goal round the corner (heuristic x1) | — | Backs out 22.5 m, stops at the cusp, then the drive takes it forward to the goal |

Next (P9 R3): the cvcGL finale and cvcdbg plan their vehicles' routes with it, and replan a
vehicle that wedges.
