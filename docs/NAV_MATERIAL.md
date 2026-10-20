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
- Demo: [`src/cvcGL/examples/nav_material_demo.cpp`](../src/cvcGL/examples/nav_material_demo.cpp)
  (built with `-DCVC_BUILD_CVCGL=ON -DCVC_BUILD_EXAMPLES=ON`)
- Belief (observed) material for range-limited sensing, a spec not yet built:
  [`roadmap/NAV-BELIEF-MATERIAL-ROADMAP.md`](roadmap/NAV-BELIEF-MATERIAL-ROADMAP.md)

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

### Deploying a learned grip/risk/lam policy

The example above drives with `coef_mlp::default_biased()` — the fixed
`(α,β,γ)=(1,3,4)` basin, no learned steering. To deploy a **trained** policy,
hand the `sim_world` constructor (or `bicycle_rollout_*`/`drive_step_*` directly)
a `coef_mlp::load("…​.cvcnav")` instead. The `.cvcnav` header carries the
feature-layout flags, so the loaded net self-describes what the drive must feed
it:

| net | `in`/`out` | flags | drive must supply |
|---|---|---|---|
| geometry | 5 → 3 | — | nothing extra (plain `drive_step`) |
| grip | 6 → 3 | `has_mu()` | a grip `friction_field` (`veh_params.grip`) |
| grip+risk | 7 → 3 | `has_mu()`+`has_risk()` | grip field **and** a material stack |
| + learned reroute (soft only) | · → 4 | `has_lam()` | the 4th output overrides `lam_soft` (single softplus head) |
| + two-head reroute (soft **and** hard) | · → 5 | `has_lam()`+`has_lam_hard()`+`kFlagLamSigmoid` | 4th/5th outputs are `lam_soft`/`lam_hard`, each **sigmoid-bounded** `λ_max·σ(·)` (`.cvcnav` **format v2**) |

A **risk or lam** net drives ONLY through the material path
(`drive_step_material` / `drive_step_material_ext`): the risk-lookahead feature
reads the same `material_stack` the reroute force uses, so `drive_step` /
`drive_step_ext` reject a `has_risk()` net rather than feed it a short vector.
`drive_step_material` hard-errors if the net's `in_features()` disagrees with its
flags — a mismatched weights file fails loudly, never silently mis-fed. Attach
the grip field before construction (`cfg.veh.grip = &ff`) and the material stack
via `set_material`, exactly as above; then the loaded net reads mu/risk ahead and
(with a lam head) sets its own per-agent reroute weight.

**The two-head reroute (`kFlagLamSigmoid`, `.cvcnav` format v2).** The
single-softplus `has_lam()` net learns only `lam_soft` (`lam_hard` stays a fixed
barrier). A **two-head** net learns both: with `kFlagLamSigmoid` set, output
columns `≥ 3` are `λ_max·σ(raw)` rather than `softplus(raw)`, so the 4th output is
`lam_soft = lam_soft_max·σ(·)` and the 5th is `lam_hard = lam_hard_max·σ(·)` — the
paper's sigmoid-bounded two-head reroute (`grl_snam` `material_nav.py`). Sigmoid
bounding is what lets a head cleanly *suppress* its channel (`λ → 0`) as readily
as engage it; the ceilings default to `lam_soft_max = 5`, `lam_hard_max = 10`
(`coef_mlp.h`) and are stored in the `.cvcnav` v2 trailer, so the runtime
reconstructs the exact map. A v2 blob **hard-fails on a pre-v2 host** (the loader
requires format v2 for `kFlagLamSigmoid`); the whole round-trip — widen to two
heads, serialize, reload, drive — is pinned by `nav_material_deploy_test`.

To steer on the learned grip/risk policy **while** an external force (e.g. a
downstream layer's own steering push) also acts, use the fused `drive_step_material_ext` /
`bicycle_rollout_material_ext` — `sim_world::step()` selects it automatically when
both a material stack and an `ext_force` are attached (otherwise material would
take priority and drop the ext force). The whole round-trip — a widened net
serialized to `.cvcnav`, reloaded, and driven (incl. the fused path and the
learned-lam reroute) — is pinned by `nav_material_deploy_test`.

> **Boundary — `cvc::nav` is force-agnostic.** The `ext_force` port knows only
> "a force field sampled at a pose"; it has no vocabulary for what the force
> *means*. The two-head material soft/hard channel documented here is the
> reusable template — a precomputed risk/hazard field driving `−λ_s∇r̃ −
> λ_h b′(φ)∇φ`. A downstream layer can re-source that template from a different
> field entirely and inject it through the same seam without any change here; its
> domain-specific logic stays in that layer, never in this core (as with the
> domain-neutral base statistics and their extension seam — see `NAV_STATS.md`).

### Training with the external force

The fused deploy path drives the material policy *and* an external force together.
Training has to see the same thing, or the material heads learn a world without the
force they will face. `material_ext` (`material.h`) puts that force in every material
training rollout:

- the surrogate (`integrate_surrogate_material`);
- the bicycle (`integrate_bicycle_material`);
- the dynamic vehicle (`integrate_dynamic_material`);
- their VJPs;
- the trainer, through `material_batch::ext`.

**How the force joins** is how `drive_step_material_ext` adds `ext_force`:
- It is added last to the bicycle's and the dynamic vehicle's force sum,
  `F = F_bar + F_goal + F_mat + F_ext`.
- With `steer` on, it is also added to their steering bias,
  `(F_rep + F_mat + F_ext) . left`.
- The surrogate is a point mass: the force joins its `ftot`, and `steer` has nothing to act on.
- The bicycle divides its force by `mass` and the deployed drive does not, so the two agree at
  `mass = 1`, the default.
- A null channel is the rollout without one, bit for bit. The adds are conditional, as in the
  deployed drive.

**Sources** (exactly one):
- **A force patch per agent:** `K` two-channel patches (`fx`, `fy`) on the rollout patch's
  grid, centred on the start and sampled bilinearly at the current position.
  - A schedule, `patch_of_step`, can switch patches per step, for example sources switching
    on and off.
  - J = dF/do is the patch's own gradient.
  - This is the form for a force that depends only on the agent's own position (sources
    fixed in the scene). It is an array, so it reaches pycvc.
- **Per-step values** `F`, with an optional `J`.
- **A C++ callback** `(user, b, s, ox, oy, F, J)`, sampled every `hold` steps and held between,
  as the diff rollout's `bike_ext_fn`.

**The VJPs** chain `J^T dL/dF_ext` into the position adjoint. A held window's sum goes in at the
step it was sampled. Given `g_fext`, they also return dL/dF_ext per step. `L_multi`'s geometry
rollouts leave the force out, as they leave out the material forces. With a force, the surrogate
trains on the CPU: there is no device twin of the channel yet.

Pinned by `nav_material_ext_test` and the `...WithAnExternalForce` cases of
`nav_material_train_test`:

| check | measured |
|---|---|
| Null or zero force | the rollout without one, bit for bit, forward and reverse, all three integrators |
| Coefficient gradient through a scheduled force patch | worst relative 2.2e-3 / 1.1e-2 / 1.7e-2 (surrogate / bicycle / dynamic) |
| Coefficient gradient through a callback held for 2 steps | 9.2e-4 / 1.6e-2 (surrogate / bicycle) |
| dL/dF_ext per step against central differences | 2.6e-4 / 2.5e-5 / 3.0e-5 |
| A callback giving the patch's own values and J | the patch, bit for bit |
| The full loss-to-weights gradient with the force | directional 2.2e-4 (surrogate), 7.0e-4 (bicycle) |

`tests/data/material_ext_golden.json` (`NavMaterialExt.Golden`) holds GRL-SNAM's torch twins to
the three integrators with the force on: the inputs, outputs, coefficient gradients and dL/dF_ext.

### Belief (observed) material

For range-limited sensing, the material planes can be built from what a belief plane has seen
rather than from the whole map (spec: [`roadmap/NAV-BELIEF-MATERIAL-ROADMAP.md`](roadmap/NAV-BELIEF-MATERIAL-ROADMAP.md)).
`inc/cvc/nav/belief_material.h`:

* `belief_material_inputs(risk_raw, hard, seen, …, p, risk_out, hard_out)` keeps the truth where
  `seen` and writes `p.unknown_risk` / `p.unknown_hard` elsewhere (defaults 0 / 0: an unseen cell
  exerts no force of its own). `material_build` then runs unchanged on the result.
* `belief_gate_surface(hard_b, occ, seen, …, p, gate_hard_out, clear_m_out)` is the gate's
  surface: `hard_b | occ | (p.unknown_gate_hard & !seen)` and its metres clearance, the chain
  `set_material` uses. With `unknown_gate_hard` (the default) a ray through unseen ground never
  certifies a detour. `witness_gate` runs unchanged on it.
* The seen mask is the sense ray-cast's field of view. At the default range it is `ever_seen`;
  `sense_batch` also keeps a shorter material-range mask on request
  (`belief_planes::material_seen` / `material_changed`, `sense_agents::material_range_m`).
* `seen_disc` is the optional range-disc rule (`seen_rule::disc`): no occlusion, kept to reproduce
  a controller trained on a disc mask.

On a `sim_world`, `set_material_belief(risk_raw, hard, mc, bp, material_every = 1)` replaces
`set_material`: the truth is one world plane, and one material + gate plane is kept per belief
plane from what that plane has seen. Its gate surface uses the plane's own planning raster
(`belief_occ(m)`), not truth. A plane's planes are re-derived on a sense tick on which its mask
grew, and its gate surface also when `belief_occ(m)` changed, at most once every `material_every`
sense ticks. It throws under `freeze_sense` (nothing is ever seen). Read surface: `material_seen(m)`,
`material_plane_data(m)`, `material_gate_hard(m)`, `material_clear_m(m)` and `material_version(m)`.
Memory is about 36 bytes per cell per belief plane.

The golden `src/cvc/tests/data/belief_material_golden.json` (`NavBeliefMaterial.Golden`) holds
GRL-SNAM's numpy twin to all of it, bit for bit: per case, tick and plane, the SHA-256 of every
raster, and the gate's outputs.

Training on belief planes (B3):

* The model's risk patch has 3 channels, `[r~, hard, seen]`. `material_patch_belief` crops it
  from the belief planes around an agent: P x P samples over `extent_m`, r~ bilinear, hard and
  seen from the nearest cell, and the pinned values wherever the nearest cell is unseen or off
  the map. It is BIT, op order pinned in the header.
* A 3-channel net is a `.cvcnm` **version 2** file: the version-1 layout plus a belief block
  after `eps` (`<IIfIIff>`: risk_channels, seen_rule, unknown_risk, unknown_hard,
  unknown_gate_hard, material_range_m, patch_extent_m). A 2-channel net is still written as
  version 1, byte for byte, and libcvc 3.7.0 and earlier refuse a version-2 file rather than read
  its encoder with 2-channel strides. The loader now checks every tensor's shape against the
  hyperparams and keeps the meta trailer through `serialize()`. `risk_channels()` and
  `belief_header()` report what a file holds.
* `material_batch::gate_active` (`[B]` u8, null = ungated) is the witness gate in training: an
  agent's `lam_soft` enters every rollout times its bit; `L_lreg` reads the head's own value.
* The CPU and CUDA forward, backward and trainer take `C = risk_channels()` channels.

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
| `belief_material_inputs`, `belief_gate_surface`, `seen_disc`, `material_patch_belief`, `sense_batch`'s material mask | **BIT** | a select, an OR plus the `set_material` clearance chain, the numpy-linspace disc (`belief_material.cpp` is built with `-ffp-contract=off`) |
| `material_sample` | FLOAT (rtol 1e-5) | the proven `sdf_sample` op chain; bit-exact vs torch on x86-64 in practice |
| `*_material` rollouts | FLOAT (rtol 1e-4 / atol 1e-5) | the `drive_step` tier; null material delegates byte-identically |

Build discipline: `material.cpp` is compiled with an explicit
`-ffp-contract=off` (`/fp:precise` on MSVC) — the blur's f64 accumulation
with exp-derived weights WOULD contract to FMA on aarch64 and flip the
f32-stored planes against cross-platform goldens. Do not remove the flag or
"tidy" any arithmetic in that TU; op order is the contract. The
cross-language sweep lives in GRL-SNAM `tests/test_material_parity.py`.

## pycvc surface

Belief material: `nav_belief_material_inputs`, `nav_belief_gate_surface`, `nav_seen_disc`,
`nav_sense_batch_material` (the sense plus the material mask), `nav_material_patch_belief` (the
model's crop). The trainer's step and loss take a trailing `gate_active`, and
`nav_material_trainer_risk_channels` reports a handle's channels (and marks a pycvc that takes the
gate). `nav_matnet_forward` and the trainer take `(B,C,P,P)` risk patches.

Field/gate kernels: `nav_material_build`, `nav_witness_gate`,
`nav_witness_gate_batch`, `nav_material_sample`, `nav_bicycle_rollout_material`,
`nav_drive_step_material`.

`sim_world` belief material: `nav_sim_world_set_material_belief`, `nav_sim_world_material_seen`.

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

`L_multi`'s start sampling rides on five trailing `nav_material_trainer_create`
args — `ms_count`, `frac_lo`, `frac_hi`, `seed`, `ms_semi_implicit` — whose
defaults (one start at `frac = 0.9`, explicit Euler) are the pre-sampling port,
byte for byte. GRL-SNAM #113 made its own `multi_start_penalty` draw
`frac ~ U(0.8, 0.98)` per start and per sample (the old fixed `0.9` made all
`ms_count` rollouts identical); passing that range turns the same draw on here.
`multi_start_fracs` reproduces `torch.rand(B, generator=manual_seed(seed))` bit
for bit, and the trainer's step `k` draws with `seed + k` while `_loss` scores the
base seed. `ms_semi_implicit` selects GRL-SNAM's integrator (velocity first);
the default explicit order is the material fork's, which `train_material.py`
imports. These are trailing args on an existing symbol, so an older pycvc rejects
them with a `TypeError` rather than through a capability flag.

The external force in training (`material_ext`) is optional trailing arguments on
`nav_integrate_surrogate_material`, `nav_integrate_bicycle_material`, its `_vjp`, and the trainer's
`_step` and `_loss`:
- `ext_patch` (B,K,2,Hp,Wp) f32 with `ext_patch_of_step` (B,steps) i32;
- or `ext_F` (B,steps,2) f32 with `ext_J` (B,steps,4) f32;
- and `ext_steer`.

The bicycle VJP returns `g_fext` (B,steps,2) as a sixth value with `want_g_fext=1`. An older pycvc
rejects these arguments with a `TypeError`. They are checked in the binding, with the GIL held.

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
