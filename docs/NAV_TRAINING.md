# cvc::nav — Self-Supervised Navigation Policy Training

**Version:** libcvc 3.3.0
**Status:** Production ✅ (CPU + CUDA, gradcheck-validated)
**Headers:** `cvc/nav/coef_train.h`, `cvc/nav/detail/diff_rollout.h`, `cvc/nav/coef_mlp.h`
**Tests:** `nav_coef_train_test` (10 gtests; CUDA parity when `CVC_ENABLE_CUDA`)

Train the `cvc::nav` navigation policy (`coef_mlp`) **from pure C++ — no libtorch,
no Python** — by self-supervised optimization over a scene's SDF. There is no
dataset and no labels: the gradient comes from a *differentiable rollout* ("did the
agent reach its goal without hitting a wall") straight into the coefficient net.
This is the training twin of the torch-free inference stack (`drive_step`,
`sim_world`, `sim_world_cuda`) — so a policy can be (re)trained on the box it
deploys on and dropped straight into the swarm.

## Table of Contents

- [Quick start](#quick-start)
- [What gets trained](#what-gets-trained)
- [The rollout switch: surrogate vs bicycle](#the-rollout-switch-surrogate-vs-bicycle)
- [Scene source](#scene-source)
- [CPU vs CUDA (device-resident)](#cpu-vs-cuda-device-resident)
- [Exporting weights](#exporting-weights)
- [Hyperparameters](#hyperparameters)
- [How it works](#how-it-works)
- [Correctness: the gradcheck](#correctness-the-gradcheck)

## Quick start

```cpp
#include <cvc/nav/coef_train.h>
using namespace cvc::nav;

training_scene scene = city_scene(96);   // the Python STORIES["city"], or occupancy_scene(...)

train_config cfg;                        // sensible defaults (surrogate rollout)
cfg.steps = 300;

coef_trainer tr(cfg);
tr.train(scene);                         // self-supervised; the differentiable rollout is the signal
coef_mlp policy = tr.to_coef_mlp();      // bake the frozen inference policy
policy.save("coef_mlp.cvcnav");          // persist; sim_world / sim_world_cuda load it
```

Or, on a machine with a GPU, run the **fully device-resident** loop:

```cpp
coef_mlp policy = train_coef_mlp_cuda(scene, cfg);   // requires CVC_ENABLE_CUDA + a device
policy.save("coef_mlp.cvcnav");
```

The `nav_train_demo` example (build with `-DCVC_BUILD_NAV_EXAMPLE=ON`) does exactly
this end to end and picks the CUDA path automatically when available.

## What gets trained

The policy is `coef_mlp`: a `5 → hidden → hidden → 3` SiLU MLP whose output is
`softplus(net(feat) + log(expm1(bias)))`, producing the three coefficients
`(alpha, beta, gamma)` = (barrier strength, goal-spring, damping) the vehicle drive
uses. It is *bias-centered* on the hand-tuned basin `(1, 3, 4)`, so training is a
**refinement** of a known-good policy, not from-scratch learning. The features are
`[phi, |goal-o|, goal_dir_x, goal_dir_y, goal_dir · wall_normal]`.

## The rollout switch: surrogate vs bicycle

`train_config::rollout` selects which differentiable integrator the training
rollout backprops through. **The coefficient net, features and loss are identical
for both — only the integrator differs.**

| `rollout` | integrator | state | gradient | use it when |
|---|---|---|---|---|
| `rollout_kind::surrogate` *(default)* | point-mass `sdf_rollout` (IPC wall force + goal spring + damping) | `(o, v)` | smooth, well-conditioned | the recommended default — refines the basin and **improves** reach |
| `rollout_kind::bicycle` | the FULL deployment kinematic-bicycle integrator (`bicycle_rollout`), differentiated | `(o, θ, sp)` | correct but non-smooth (governor branches) | you want to train against the *exact* deployment dynamics |

```cpp
train_config cfg;
cfg.rollout = rollout_kind::bicycle;   // train through the deployment integrator
cfg.lr = 1e-5f;                        // NOTE: the bicycle needs a MUCH smaller step
// cfg.veh_L / veh_delta_max / veh_a_max / ... override the bicycle vehicle params
//   (defaults = the SdfNavigator VEHICLE_DEFAULTS).
```

**Guidance.** The surrogate is the recommended default: its smooth gradient refines
the basin and measurably *improves* deployment reach (~62% → ~65% on the city
scene). The bicycle option differentiates the actual deployment integrator (no
surrogate→deployment dynamics gap), but two things make it harder: (1) its governor
branches (corner/stop/creep/reverse limits) make the loss landscape far more
sensitive, so it needs a much smaller learning rate — `~1e-5` vs the surrogate's
`2e-4`; `2e-4` collapses it; and (2) the training rollout chases the goal *directly*
because the carrot FSM that feeds sub-goals at deployment is a non-differentiable
wrapper — so on the city scene the bicycle *holds* the tuned basin rather than
beating it. Both are gradient-correct (see [gradcheck](#correctness-the-gradcheck));
prefer the surrogate unless you specifically need the deployment integrator in the
loop.

## Scene source

A `training_scene` is a static occupancy world + the vehicle/integration meta +
reachable start/goal sampling (from the largest 8-connected free component, so a
goal is always reachable from its start). Two factories:

- **`city_scene(int grid = 96)`** — the Python `STORIES["city"]` training scene,
  ported: `city_blocks(96, rows=3, cols=3, gap=9, margin=14)` rects scaled by
  `grid/96` and rasterized, with the city meta (bounds ±100, scale 0.05, rr 0.15,
  d_hat 0.35, dt 0.06, vmax 0.9). This is the *same* scene `coef_train.py` trains on.
- **`occupancy_scene(occ, rows, cols, min_x, min_y, max_x, max_y, scale, rr, d_hat,
  dt, vmax)`** — any caller-provided occupancy (0 = free). Use this to train
  **directly on the map you deploy into** — e.g. a rasterized terrain / `lsystem_forest`
  scene — rather than the Python city.

```cpp
std::vector<std::uint8_t> occ = rasterize_my_terrain(rows, cols);
training_scene scene = occupancy_scene(occ.data(), rows, cols,
                                       -400, -400, 400, 400, /*scale=*/0.02f);
```

## CPU vs CUDA (device-resident)

| entry point | where | notes |
|---|---|---|
| `coef_trainer::train(scene)` | CPU | threaded per agent; Adam + truncated BPTT on the host |
| `train_coef_mlp_cuda(scene, cfg)` | GPU | **fully device-resident** — see below |
| `loss_and_grad_cuda(...)` | GPU | per-call loss+grad (used by the CUDA-vs-CPU parity test) |

`train_coef_mlp_cuda` keeps the **field, params, Adam moments and all per-window
scratch resident on the GPU across the entire run**. Per outer step only the fresh
agent batch `(o, goal)` is uploaded; each truncated-BPTT window runs the
forward/backward kernel, then an in-place device Adam driven by a single-float D2H
of the grad-clip norm — the gradient itself never leaves the device. Only the final
trained params come back to bake the `coef_mlp`. Gate it with `train_cuda_available()`
(true iff built with CUDA *and* a device is present).

Both paths run the *same* differentiable primitives (`detail/diff_rollout.h`), so
the CUDA loss+gradient reproduce the CPU trainer's to ~1e-7 (validated).

## Exporting weights

The trained policy round-trips through the versioned `.cvcnav` format the whole
stack reads:

```cpp
coef_mlp policy = tr.to_coef_mlp();            // from_layers() under the hood
policy.save("coef_mlp.cvcnav");                // byte-identical to coef_export.write_coef_mlp
coef_mlp reloaded = coef_mlp::load("coef_mlp.cvcnav");
```

Drop it in the canonical install location `$PREFIX/share/cvc/nav/coef_mlp.cvcnav`
(or point `CVC_NAV_WEIGHTS` at it) and `sim_world` / `sim_world_cuda` pick it up via
`coef_mlp::default_weights_path()`.

## Hyperparameters

`train_config` (defaults mirror `coef_train.py` where sensible):

| field | default | meaning |
|---|---|---|
| `steps` | 400 | outer optimization steps (a fresh agent batch each) |
| `horizon` | 28 | rollout steps per outer step |
| `n` | 192 | agents per batch |
| `window` | 7 | truncated-BPTT window (detach + Adam step every `window`) |
| `hidden` | 64 | CoefMLP hidden width (≤ 64) |
| `lr` | 2e-4 | Adam step — the *refinement* rate. `1e-3` (coef_train.py's never-run default) collapses navigation; bicycle needs ~`1e-5` |
| `w_coll` | 6.0 | collision-penalty weight |
| `grad_clip` | 5.0 | global-norm gradient clip |
| `rollout` | `surrogate` | see [the switch](#the-rollout-switch-surrogate-vs-bicycle) |

## How it works

Per truncated-BPTT window, the graph is: `coef_feats(o) → CoefMLP → rollout step →
sample(o') for the collision term`, chained through the state, with the loss
`|goal − o_final| + w_coll · Σ relu(rr − phi)`. The reverse pass is hand-written
adjoints (no autograd engine): the bilinear-sample **position VJP**, the MLP
backward (Linear/SiLU/softplus), the IPC-barrier derivative, and the rollout step's
own adjoint — surrogate or the full bicycle. Adam with global-norm clipping; the
window is detached every `window` steps to bound the graph.

The differentiable primitives are `__host__ __device__` in
`cvc/nav/detail/diff_rollout.h`, so the CPU trainer and the CUDA kernel compile the
**same** adjoint source — the device backward is correct by construction.

## Correctness: the gradcheck

Correctness is **torch-independent**: a central finite-difference gradcheck is the
ground truth. If the analytic gradient matches the numeric one, the backward is
correct — no reference to any autograd. `nav_coef_train_test` asserts:

- surrogate gradient-direction FD `dir_rel ≈ 2e-4`;
- bicycle gradient-direction FD `dir_rel ≈ 2e-2` (the branchy integrator is looser;
  a standalone per-op FD pins the bicycle step adjoint to p99 ≈ 1e-2);
- the CUDA loss+gradient match the CPU trainer to ~1e-7 for **both** rollouts;
- a trained policy drives the bicycle `sim_world` (surrogate improves reach; bicycle
  holds the basin);
- the baked policy round-trips through `.cvcnav`.
