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
| `bicycle_rollout_cuda` | GPU, **given** coefficients — the unfused device twin |
| `drive_step`, `drive_step_material` | fused: `coef_feats` → MLP → rollout |
| `drive_step_cuda` | fused on GPU, one thread per agent |
| `sim_world_cuda` | device-resident world; holds its own refinement buffers |

`bicycle_rollout_cuda` was added with this work. CUDA previously had only
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
