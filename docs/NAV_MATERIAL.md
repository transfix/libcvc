# cvc::nav material-aware navigation

`cvc::nav` can navigate over *terrain semantics*, not just geometry: a
per-cell **material risk** field (mud, rubble, water skirts — soft costs that
bias but never block) and a **hard hazard** mask (water, cliffs — lethal but
not physical geometry), driving two extra force terms and a feasibility
witness gate on top of the existing SDF barrier / goal-spring drive.

This is the C++ twin of GRL-SNAM's material-aware navigation (its
`docs/MATERIAL_NAV.md` is the cross-repo design record; research provenance:
`github.com/SetasAditya/material-aware-grl-snam`). The Python implementation
in `grl_snam/material.py` is the NORMATIVE reference for every BIT-tier
surface here.

- Header: [`inc/cvc/nav/material.h`](../inc/cvc/nav/material.h)
- Implementation: `src/cvc/nav/material.cpp` (+ the material coupling in
  `src/cvc/nav/drive.cpp` and `sim_world.cpp`)
- Tests: `src/cvc/tests/nav_material_test.cpp`
- Demo: [`examples/nav_material_demo.cpp`](../examples/nav_material_demo.cpp)
  (`-DCVC_BUILD_NAV_EXAMPLE=ON`)

## The executed field

Per drive substep, sampled at the vehicle's current position:

    F_soft = -lam_soft_eff * grad r~          lam_soft_eff = lam_soft * gate
    db     = -sigmoid(k_sharp * (d_hat_m - phi_m))
    F_hard = -lam_hard * db * grad phi
    F      = F_barrier + F_goal + F_soft + F_hard

* `r~` — smoothed material risk in `[0, 1]`.
* `phi_m` — UNSIGNED distance in **world metres** to the nearest hard cell
  (one-sided EDT: 0 inside hazards — different from `build_sdf`'s signed
  field). Keeping it in metres end-to-end is what makes `k_sharp` (1/m) and
  `d_hat_m` (m) plain physical constants with no rescale trap.
* The material force feeds BOTH bicycle couplings: the longitudinal
  projection (`F . heading` — a hazard dead ahead brakes) **and** the
  steering bias (`tanh((F_rep + F_mat) . left)` — a lateral risk gradient
  turns the wheel). The source method integrates a point mass whose force
  bends the trajectory directly; a bicycle discards lateral force, and
  without the steer coupling the whole feature degenerates to speed
  modulation.
* `lam_hard` is **always on**; the witness gate multiplies `lam_soft` only.

## The witness gate

`witness_gate` is a frame-wise *activation witness*, never a controller: it
answers "does a feasible, progress-making direction with lower mean risk
than straight-to-goal exist right now?" and its `active` bit switches the
soft force on. Algorithm (BIT-pinned to the Python reference):

* `primitive_count` rays (16 uses a shared exact float64 direction table —
  libm `sin`/`cos` never enter the contract), walked `1..horizon_cells` in
  continuous cell coordinates, cells by `rint` (round-half-even) + clip.
* A ray is *eligible* if its endpoint is at least `progress_slack_cells`
  closer to the goal, and *feasible* if every sample stays in bounds, off
  `gate_hard`, and at clearance `>= hard_margin_m`. The sampled risk is
  recorded BEFORE the feasibility break (order-sensitive; pinned).
* `active = feasible_count > 0 && nominal >= material_trigger &&
  nominal - best >= improvement_margin` (mean-ray-risk units).
* **`gate_hard` must include occupancy** (`material hard | occ`): a ray
  through a building is not evidence of a feasible detour. `sim_world`
  composes this for you from `hard | truth` — the oracle-maps setting.

## API

```c++
#include <cvc/nav/material.h>

// derived planes (BIT-identical to MaterialGrid._derive in GRL-SNAM):
//   risk    = gaussian_blur(risk_raw, sigma) clipped [0,1]      (f32)
//   phi_m   = sqrt(edt2(hard)) * cell_w                          (metres, f32)
//   grad r~ = np.gradient(risk)   / float(cell_w * scale)  [per normalized unit]
//   grad ph = np.gradient(phi_m)  / float(cell_w)          [metres per metre]
material_planes mp = material_build(risk_raw, hard, rows, cols, cell_w, scale, /*sigma=*/1.0);
std::vector<float> stack = mp.stacked();     // one [1,6,H,W] block

material_stack ms{stack.data(), 1, rows, cols, mnx, mny, mxx, mxy, cx, cy, scale};
material_sample(ms, on, n, map_id, risk_out, phi_out, grad_r_out, grad_phi_out);

gate_params gp;                              // 16 rays, 12 cells, 1 m margin,
gate_decision g = witness_gate(              // 0.05 margin, 0.45 trigger
    mp.risk.data(), gate_hard, clear_m, rows, cols, pos_r, pos_c, goal_r, goal_c, gp);
witness_gate_batch(...);                     // n agents, SoA outputs, threaded,
                                             // byte-identical to n serial calls

material_drive md;                           // nullptr stack == plain rollout,
md.stack = &ms;                              // byte-identical (asserted)
md.lam_soft = lam_soft_cols;                 // [n], gate already multiplied in
md.lam_hard = lam_hard_cols;                 // [n], never gated
bicycle_rollout_material(fs, o, th, sp, goal, al, be, ga, n, map_id, veh, md, minclr);
drive_step_material(fs, o, th, sp, carrot, model, n, map_id, veh, md, minclr);
```

### sim_world integration (pure-C++ material-aware swarms)

```c++
sim_world w(cfg, truth, prior, coef_mlp::default_biased(), o, goal, color, n);
material_config mc;          // lam_soft 0.5, lam_hard 1.0, k 1.25/m, d_hat 12 m
mc.gate.horizon_cells = 8;   // ~ the ray length in cells for your grid
w.set_material(risk_raw, hard, mc);   // copies + derives; default off = byte-unchanged
while (running) {
  w.step();                  // per-tick: batched gate vs each agent's goal,
  w.snapshot(pos, th, sp, mode, reached);   // then the material drive
  const std::uint8_t *gate = w.material_gate_active();  // telemetry/renderer hook
}
```

`set_material` derives the planes, builds the gate surface
(`hard | truth` + its metres-EDT clearance plane — one EDT, at set time), and
turns the feature on for every subsequent `step()`. `clear_material()` turns
it back off. `mc.gate.hard_margin_m <= 0` means "2 grid cells" (the source's
margin at its own resolution).

The trailing `planes` argument (default 1) takes grouped material: pass
`[planes,rows,cols]` risk/hard stacks and each agent indexes its material plane
by the **same** `map_id` as its belief plane, so every `map_id` must be
`< planes` (it throws otherwise — the pycvc binding pre-validates this before
releasing the GIL). `material_gate_active()` is only meaningful while material is
attached; its buffer is sized inside `set_material`, so ask `has_material()`
first.

### Choosing constants

The defaults are the GRL-SNAM sim-frame values, validated behaviorally there
(`lam_soft 0.5`, `lam_hard 1.0`, `k_sharp 1.25`, `d_hat_m 12`). Two traps the
defaults already avoid, worth knowing when you retune:

* The source paper's `lam_soft = 1.5` is a *pixel-frame* value; in the
  normalized frame it saturates the steering and launches vehicles.
* `d_hat_m` must span several grid cells or the barrier is invisible until
  contact (the source's 3 m was six cells on its 0.5 m/cell BEV; on a
  ~2 m/cell grid it is *under one cell*). Keep `k_sharp * d_hat_m ~= 15`.

Scope note: the executed field is a LOCAL layer. In the source method it
always ran under a planner's waypoint scaffold, and the same holds here — a
planner-less reactive swarm gets the hazard **no-entry guarantee** but can
dead-end against a hazard squarely blocking its goal line (a potential-field
minimum). Route-level avoidance belongs to the planner: feed
`risk_weight * r~ + hard_penalty * hard` into `astar`'s per-cell cost input
(GRL-SNAM's `FogScenario` does exactly this).

## Fidelity contract

| surface | tier | notes |
|---|---|---|
| `material_build` | **BIT** vs the Python reference | pinned-op-order blur (scipy-`reflect`-equivalent, symmetric padding, sequential tap/normalization sums, f64 through both passes, one f32 store), EDT metres chain, f32 gradients of the f32-stored planes |
| `witness_gate` (+batch) | **BIT** | float64 end-to-end, embedded direction table, `sqrt(x*x+y*y)` never `hypot` (CPython's hypot is not libm's), batch == serial bytes at any thread count |
| `material_sample` | FLOAT (rtol 1e-5) | the proven `sdf_sample` op chain; bit-exact vs torch on x86-64 in practice |
| `*_material` rollouts | FLOAT (rtol 1e-4 / atol 1e-5) | the `drive_step` tier; null material delegates byte-identically |

Build discipline: `material.cpp` is compiled with an explicit
`-ffp-contract=off` (`/fp:precise` on MSVC) — the blur's f64 accumulation
with exp-derived weights WOULD contract to FMA on aarch64 and flip the
f32-stored planes against cross-platform goldens. Do not remove the flag or
"tidy" any arithmetic in that TU; op order is the contract. The
cross-language sweep lives in GRL-SNAM `tests/test_material_parity.py`.

## pycvc surface

Field/gate kernels: `nav_material_build`, `nav_witness_gate`,
`nav_witness_gate_batch`, `nav_material_sample`, `nav_bicycle_rollout_material`,
`nav_drive_step_material`.

`sim_world` material (grouped planes): `nav_sim_world_set_material`,
`nav_sim_world_clear_material`, `nav_sim_world_material_gate_active`.

Learned model + training: `nav_matnet_forward` (the `.cvcnm` coefficient net),
`nav_integrate_surrogate_material` (the obstacle-list rollout), and the trainer
handle `nav_material_trainer_create` / `_step` / `_loss` / `_save` /
`_cuda_active`, plus the handle-free probes `nav_material_cuda_available` and
`nav_material_cuda_max_horizon`.

All NEW symbols (nothing re-signatured), so GRL-SNAM's capability flags
(`HAS_MATERIAL`, `HAS_MATERIAL_DRIVE`, `HAS_MATNET`,
`HAS_MATERIAL_ROLLOUT_INTEGRATOR`, `HAS_MATERIAL_TRAINER`) degrade cleanly
against older pycvc builds. GRL-SNAM keeps pure Python as the feature default and
opts into these via `GRL_SNAM_MATERIAL_BACKEND=native`.

The trainer's `use_cuda` is a *request*, not a guarantee: each heavy op falls back
to its host twin when the build has no CUDA, no device is present, or the batch
exceeds a kernel's bound (the device rollout VJP caps `max(H)` at
`material_rollout_cuda_max_horizon()`). `nav_material_trainer_cuda_active`
reports what a handle will actually use. The three `*_cuda_available()` probes
are defined on every build — the `.cu` supplies the real ones and the matching
`.cpp` a `false` stub — so a binding can call them without a `CVC_USING_CUDA`
guard.

Binding gotcha, earned the hard way: a C++ exception thrown while the GIL is
released (`Py_BEGIN_ALLOW_THREADS`) leaves the GIL unrestored and segfaults —
any throwing kernel precondition must be pre-validated in the binding before
the release.

## Deferred

**CUDA material *inference* drive** — a `drive.cu` material twin
(`drive_step_material` / `bicycle_rollout_material` on device) and
`sim_world_cuda` material planes. Neither exists: `drive.cu`'s device path has no
`material_stack`, no 6-channel sample and no `lam_soft`/`lam_hard` force terms.
The device drive work that did land (`bicycle_rollout_cuda`, the device-resident
`sim_world_cuda`, the fused grip feature) is the geometry/vehicle drive plus the
`mu` friction raster, which `drive.h` keeps deliberately separate from the
material planes — it is not progress on this item.

Note this is *inference*. The material **training** stack is on the device: the
model forward/backward (`coef_energy_net.cu`), the surrogate rollout forward/VJP
(`material_rollout.cu`) and a device-resident Adam (`material_train.cu`), routed
by `material_loss_and_grad(..., use_cuda=true)`. `L_multi`
(`multi_start_penalty`, `geom_rollout.cpp`) is the one training op with no device
twin and stays on the host regardless.

Formerly deferred, now shipped: the `nav_sim_world_set_material` pycvc binding
and grouped material planes (`M > 1`) — `witness_gate_batch` takes a `map_id`
against `[M,H,W]` stacks and `sim_world::set_material(..., planes)` builds
per-plane surfaces. What remains on the grouped-plane surface is Python-side:
`nav_witness_gate_batch`, `nav_material_sample` and `nav_material_build` are
still single-plane in the binding, and no GRL-SNAM wrapper attaches material to a
`NativeSimWorld` yet.
