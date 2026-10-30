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
- [The dynamic vehicle: `dyn_bike_step` (P8a)](#the-dynamic-vehicle-dyn_bike_step-p8a)
- [The four-wheel vehicle: `dyn4_plant` (P8d)](#the-four-wheel-vehicle-dyn4_plant-p8d)
- [An external force in the differentiable rollout](#an-external-force-in-the-differentiable-rollout)
- [Energy in the loss: the power model (P8c)](#energy-in-the-loss-the-power-model-p8c)
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

The `nav_train_demo` example (in `examples/nav/`, built with
`-DCVC_BUILD_CVCGL=ON -DCVC_BUILD_EXAMPLES=ON`) does exactly
this end to end and picks the CUDA path automatically when available.

**Selecting checkpoints on the physics plant.** The training loss only approximates deployment,
so `train()` can end on the checkpoint that scores best on something closer to it:

```cpp
#include <cvc/physics/nav_eval.h>

cvc::physics::plant_reach_options val;     // reach on Jolt RoboRacers in the scene
val.seeds.clear();                         // validation seeds, not the evaluation's: 32 of them
for (unsigned s = 200; s < 232; ++s)
  val.seeds.push_back(s);
cfg.checkpoint_every = 30;
coef_trainer tr(cfg);
tr.train(scene, false, [&](const coef_mlp &p, int step) {
  return cvc::physics::reach_on_plant(p, scene, cvc::physics::vehicle_desc::roboracer(), 0.035, val);
});
// tr.checkpoints(): {step, fitness} for step 0 (untrained) and every 30; tr.selected_step()
```

- **When it scores.** The untrained policy (step 0), every `checkpoint_every` steps and after the
  last. `train()` ends on the best, so it never ends below where it started by that score; ties
  keep the earlier.
- **`reach_on_plant`** (`cvc/physics/nav_eval.h`, needs Jolt): the fraction of `agents` vehicles
  that reach their goals in `ticks`, averaged over seeds. The scene's blocks become static boxes,
  read at the vehicle's scale (`wheelbase` normalized units = its wheelbase). Each seed's world
  is single-threaded and deterministic, so a score repeats exactly. Seeds run in parallel
  (`plant_reach_options::threads`, 0 = the hardware's threads) with the same per-seed results:
  8 seeds score in about the time of one.
- **But it is chaotic.** Changing the scale by 4e-9 (passing `0.035f` instead of `0.035`) moves a
  seed's reach by up to 0.1, a per-seed SD of about 0.03. A mean over k validation seeds carries
  about 3/sqrt(k) points of that alone. Use 32 when selecting among many checkpoints: with 8,
  the best of 31 was mostly luck.

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
| `rollout_kind::dynamic` | the bicycle's controller driving the dynamic single-track vehicle (`dyn_bike_step`, below) | `(o, θ, vx)` + `(vy, r, δ, a)` | the bicycle's branches plus the tires' grip | you want the policy to learn what a real vehicle can do (P8b); CPU only |

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

## The dynamic vehicle: `dyn_bike_step` (P8a)

`inc/cvc/nav/detail/dyn_bike.h` is a differentiable dynamic single-track vehicle beside
`bike_step` (the vehicle-physics plan's P8a; P8b adds it to the trainers as a third rollout).
`bike_step` is now `bike_control` (the trained controller with every governor, emitting
`(delta_cmd, a_cmd)`) followed by `bike_plant` (the kinematic integration). Its outputs are bit
for bit what they were. `dyn_bike_step` runs the same `bike_control` into `diff::dyn_plant`:

- **State:** the rear axle's `(x, y)` and heading, the body velocity `(vx, vy)` at the centre of
  gravity and the yaw rate `r`, plus the road-wheel steering angle and the drive and brake force
  per unit mass behind their actuators.
- **Actuators:** first-order lags, exact for the step, with the steering's slew limited smoothly
  (`R tanh(d / R)`).
- **Tires:** Pacejka lateral forces `D sin(C atan(B alpha))`. Each axle's drive or brake force
  saturates smoothly at `mu Fz`, and the cornering peak `D` is the friction ellipse's remainder,
  longitudinal force first.
- **Loads:** quasi-static longitudinal load transfer from the actuator's force.
- **Longitudinal:** drive, drag, rolling resistance and grade (the grade is an input, so H2
  trains).
- **Low speed:** below `v_blend`, a sigmoid blends `(vy, r)` to the kinematic bicycle's.
- **Left out on purpose:** wheel spin, suspension and lateral load transfer.

Numerically:

- `(vy, r)` advance by implicit Euler, solved with three Newton-like steps. Each tire is
  linearized as its secant through its own root in lateral velocity, `-F / (w - w0)`, which is
  positive at any slip and speed. So stiff tires are stable at nav's 0.06 s step, and a sliding
  tire never pushes its slip past zero, even at a crawl at full lock.
- The speed is predicted from the longitudinal forces, then corrected with the lateral forces
  the implicit step applied. In the stiff limit those are constraint forces, which "slip times
  stiffness" would get badly wrong.
- The plant is written once over a scalar type. Its adjoint, `dyn_plant_vjp`, runs it on
  forward-mode duals (doc 19's suggestion), and `dyn_bike_step_bwd` hands the commands'
  gradients to `bike_control_bwd`. Everything is `CVC_HD`.

Parameters are SI and come from the spec sheet: `cvc::nav::dyn_bike_si`, converted by length
exponent with `dyn_bike_params_of(si, frame)`. `cvc::physics::nav_plant::dyn_bike_si_of(desc)`
derives them from a `vehicle_desc`, so both plants read one vehicle:

- mass, and a yaw inertia from the chassis box;
- the axles relative to the centre of gravity;
- Pacejka C from the tire's sliding fraction, and B fitted to how Jolt's tires corner: B C =
  `kJoltTireStiffness` (1.12) / the peak slip angle (#753);
- the drive and brake splits, drag and rolling resistance (none for a direct drive, whose driver
  model's trim makes up for them);
- the steering's rate, and the drive's lag (0 for a direct drive, 0.1 s through a converter).

`front_stiffness` (1) scales the front's B. A road vehicle's load sensitivity and steering
compliance would put it below 1. Jolt's tires have neither, and the fit to Jolt puts front and
rear alike.

Gates (`nav_dyn_bike_test`; the Humvee from its `vehicle_desc`, in metres):

| Gate | Result |
|---|---|
| Plant gradient vs central differences (double, 6 regimes from cruise to past the peak, backing up and creeping) | worst relative error 6e-6 |
| Whole step through the controller, 6-step rollout, float (the bicycle's gradcheck) | dir_rel 6e-4 (limit 5e-2) |
| Kinematic limit: `bike_step`'s own commands through the plant over 200 ticks, with stiff tires, no lag, mu g = 40 against the controller's 1.5, the mass at the rear axle and the drive and brakes there | within 0.011 of its path (0.07 rr; at most 0.015 over 40 sets of routes, gated at 0.2 rr). Under the controller the median agent stays within 0.0003 and ends within 0.11% as far from the goal (gated at 0.1 rr and 2%). The controller amplifies small differences along some routes, so the worst agent is reported, not gated: 0 to 11 of 32 agents part by more than 0.2 rr, depending on the routes, which differ by standard library |
| Understeer that grows with lateral g (15 m/s, steady turns) | 0.0003 rad at 0.07 g to 0.0029 rad at 0.53 g, growing faster toward the limit |
| Oversteer when braking in a turn (0.4 g, then 0.4 g of braking) | the path tightens 6%, 0.0176 to 0.0187 /m |
| Steady lateral acceleration never above mu g | 0.97 mu g at mu 0.4, 0.95 mu g at mu 0.9 |
| Stopping distance within 10% of v^2 / (2 mu g) | -4% to -6% at mu 0.5 and 0.9, from 10 and 20 m/s |

The kinematic limit is exact only for the vehicle the kinematic bicycle assumes. A real body
differs physically from `bike_step`'s `v' = a`:

- it puts kinetic energy into yaw and sideways motion as it turns;
- a steered front wheel's brake force acts along the wheel, slowing the body by about
  `F / cos(delta)` in a turn.

So the same commands drive a body braked 60% in front up to 1.7 apart over the route, open loop.
Under the controller it ends 1.122 from the goal against 1.111.

`DynBike.Golden` pins the plant's values (`tests/data/dyn_bike_golden.json`; rewrite with
`CVC_DYN_BIKE_GOLDEN_WRITE=1`). That is the parity gate: GRL-SNAM's torch twin,
`grl_snam.dyn_bike.dyn_plant` (CVC-Lab/GRL-SNAM#121), reproduces every case's next state and
VJP to 1e-9, its gradient from autograd rather than duals. A double plant is double
throughout (its lag factors and floors too), so the two can agree to rounding.

**Training through it** (`rollout_kind::dynamic`, P8b):

```cpp
train_config cfg;
cfg.rollout = rollout_kind::dynamic;
cfg.lr = 5e-5f;
// The trainer's scene has no metres. Its vehicle (veh_L 0.035, vmax 0.9) reads as a
// RoboRacer at about 0.108 normalized units per metre, where vmax is 8.3 m/s.
cfg.dyn = cvc::physics::nav_plant::dyn_bike_params_at(vehicle_desc::roboracer(), cfg.veh_L);
```

- `coef_trainer::loss_and_grad` carries the vehicle's further state `(vy, r, delta, a)` through
  `x` / `x_out` ([n*4]; null is at rest), and `train()` keeps it across windows.
- Without a vehicle whose wheelbase is `veh_L`, a dynamic run throws.
- The CUDA trainer refuses it for now.

Measured on the city (`nav_coef_train_test`):

- the gradcheck's dir_rel is 2.3e-2, against the bicycle's 8.5e-4 and the limit of 5e-2. The
  gradient is under half the bicycle's (5e-4 against 1.2e-3): grip limits pass less;
- the horizon loss falls (1.4125 to 1.3988 in 60 steps at 2e-4);
- a policy trained at 5e-5 still drives the bicycle sim_world: 71.9% reach.

An lr sweep (150 steps on `city_scene(96)`; horizon loss, then the bicycle sim's reach):

| lr | bicycle | dynamic |
|---|---|---|
| 1e-5 | 1.3986 -> 1.3963, 71.5% | 1.4632 -> 1.4628, 71.5% |
| 5e-5 | -> 1.3925, 70.3% | -> 1.4585, 71.9% |
| 2e-4 | -> 1.3768, 72.3% | -> 1.3504, 68.4% |
| 1e-3 | -> 1.2240, 13.3% | -> 1.2420, 10.5% |

At a given rate the dynamic rollout learns more than the bicycle. Its reach here is measured on
the kinematic sim, so it is not the verdict. P8b's gate is on the Jolt plant: a policy trained
on the dynamic model must beat the point-mass-trained one over 20 or more paired seeds.

**The material trainer too**: `material_loss_config::integrator = material_integrator::dynamic`.

- `material_loss_config::dynamic` is the vehicle; `bicycle` stays the controller's limits.
- `integrate_dynamic_material` (`material.h`) drives the same pieces into `dyn_plant` in place of
  the kinematic update: the force law with its material forces, pure pursuit, the governors and
  `d_cap`. Arc length and cumulative risk follow the dynamic path.
- Its VJP joins the plant's dual-number VJP to the bicycle's hand-written controller reverse. The
  reverse's new controller-only mode adds `0` in the kinematic case, so the bicycle's sums are
  unchanged.
- `L_vel` compares the rear axle's world velocity.
- `L_multi` runs on the dynamic vehicle too (`multi_start_penalty_dynamic`).

Measured (`nav_material_bike_rollout_test`, `nav_material_train_test`):

- The rollout's VJP checks against finite differences on 40 coefficients: worst 1.5e-2 short,
  4.0e-2 without reverse. The directional check is 3e-6 at a step of 1e-3; at 2e-3 the step
  crosses a controller branch, as the bicycle's does.
- `L_multi` on the dynamic vehicle: worst 4.8e-4.
- The full loss-to-weights chain: dir_rel 7.0e-4 over 64 weights, the same as on the bicycle.
- It is thread-deterministic, and the controller drives the dynamic vehicle to its goals.

**P8b's gate, on the Jolt plant** (`NavPlantConvergence.DynamicTrainedPolicyAgainstPointMassOnJolt`,
a heavy run; `P8B_GATE_LR` and `P8B_GATE_STEPS` override its training):

- **The comparison.** Two policies trained alike on the city, one on the point-mass surrogate and
  one on the dynamic vehicle (a RoboRacer at about 0.108 per metre).
- **The run.** Each drives Jolt RoboRacers through the same city, its blocks as 1.5 m static
  boxes. That is 20 paired seeds of 48 agents for 400 ticks.
- **The statistic.** The mean per-seed reach difference, with a paired-bootstrap 95% CI.

| Training | Point-mass-trained reach | Dynamic-trained reach | Dynamic - point mass, 95% CI | The untrained seed |
|---|---|---|---|---|
| lr 5e-5, 150 steps | 50.6% | 51.0% | +0.4 [-1.8, +2.8] | 51.7% |
| lr 2e-4, 300 steps | 35.0% | 44.0% | +9.0 [+6.7, +11.4] | 51.7% |

**Read it carefully.**

- **At 2e-4 the gate's wording is met.** Trained alike, the dynamic-trained policy beats the
  point-mass-trained one on the real vehicle, outside the CI.
- **But training that hard hurts both.** Each falls below the hand-tuned seed it refines. The
  dynamic model makes training hurt less; it does not make the policy better than the seed.
- **At the stable 5e-5 there is no difference.** Neither policy moves far from the seed.
- **What is left.** The training loss (distance to the goal after a window, plus a clearance
  margin) is not the deployment reach, which the carrot FSM shapes. The next step selects
  checkpoints on Jolt reach (doc 31's recorded-fitness seam).

**Checkpoints selected on Jolt reach** (`NavPlantConvergence.CheckpointSelectionOnJoltReach`, a
heavy run; `P8B_SELECT_LR`, `_STEPS`, `_EVERY` and `_VAL` override it):

- **The setup.** The same two trainings at lr 2e-4 for 300 steps. Each scores its untrained
  policy and every 30th step by `reach_on_plant` on validation seeds (200 on, never the
  evaluation's), and ends on the best (above).
- **The evaluation.** The selected policies and the seed on the gate's 20 held-out seeds.

| Validation | Point-mass: selected step, held-out reach | Dynamic: selected step, held-out reach | The seed | Dynamic - point mass, 95% CI | Dynamic - seed, 95% CI |
|---|---|---|---|---|---|
| 4 seeds (scale `0.035f`) | 120, 49.4% | 30, 53.1% | 51.9% | +3.8 [+0.9, +6.7] | +1.3 [-0.6, +3.3] |
| 8 seeds (scale `0.035`) | 120, 50.1% | 60, 52.7% | 51.7% | +2.6 [+0.3, +4.7] | +1.0 [-1.2, +3.4] |

The validation curves (8 seeds), from the untrained policy's 0.479:

- **Point mass:** about flat to step 120 (0.500), then down steadily to 0.33.
- **Dynamic:** up to step 60 (0.505), down to 0.39 by step 120, and back to 0.45 by step 300.

**Read it carefully.**

- **Selection removes the damage.** Both selected policies stay within 2.5 points of the seed.
  Without selection, at this rate they ended at 35% and 44%.
- **The gate's wording holds, now near the seed.** The dynamic-trained policy beats the
  point-mass-trained one outside the CI both times, by 2.6 to 3.8 points.
- **No trained policy beats the seed yet.** The dynamic one leads it by 1.0 to 1.3 points, but
  the CI includes 0.
- **Expect shrinkage.** The best of 11 noisy validation scores overstates its policy: the 8-seed
  gains of +2.1 and +2.6 points over the untrained policy became -1.6 and +1.0 held out.
- **The useful window is early.** The dynamic training's gains come in the first 60 steps at
  this rate. Smaller steps with denser checkpoints, a larger validation set, or a loss closer to
  reach are the next levers.

**Lower rates, denser checkpoints.** The same test, 8 validation seeds, 31 checkpoints per
training (`P8B_SELECT_LR`, `_STEPS`, `_EVERY`). `lr` is the peak: `train_config::cosine_lr`
(on by default) anneals it to `lr_min` over the run, so a shorter run anneals faster. With seeds
in parallel, each run took about 9 minutes, three at once.

| Peak lr, steps, every | Point mass: step, held-out | Dynamic: step, held-out | Dynamic - point mass | Dynamic - seed (51.7%) |
|---|---|---|---|---|
| 2e-4, 300, 30 (above) | 120, 50.1% | 60, 52.7% | +2.6 [+0.3, +4.7] | +1.0 [-1.2, +3.4] |
| 1e-4, 300, 10 | 210, 51.1% | 160, 53.5% | +2.4 [+0.3, +4.4] | +1.9 [-0.8, +4.4] |
| 5e-5, 600, 20 | 440, 48.9% | 380, 50.5% | +1.7 [-1.2, +4.5] | -1.1 [-4.0, +1.6] |
| 2e-4, 150, 5 | 70, 50.5% | 70, 51.7% | +1.1 [-0.8, +3.0] | +0.0 [-1.9, +2.1] |

- **No setting beats the seed outside the CI.** The best, 1e-4 every 10 steps, leads it by
  +1.9 [-0.8, +4.4].
- **At the lower rates the validation curves are flat.** Every checkpoint at 1e-4 and 5e-5
  scores between 0.445 and 0.523 against the untrained 0.479: no climb to sample more finely.
  At those rates the training loss barely moves Jolt reach either way.
- **Denser checkpoints raise the winner's curse.** The selected checkpoints score 3.4 to 4.4
  points above the untrained policy on validation. The best of 31 draws with 1 to 1.5 points of
  noise gives 2 to 3 points of that by itself. Held out, the selected policies land between
  -2.8 and +1.9 points from the seed.
- **The dynamic training still comes out ahead.** It beats the point-mass training in all four
  settings (+1.1 to +2.6), outside the CI in two.
- **So the binding limit is the signal, not the step size.** The next levers act on it: many
  more validation seeds (32 cost about two rounds of the parallel evaluation), or Jolt reach as
  the training signal itself rather than only the selector.

**32 validation seeds** (now the test's default; `P8B_SELECT_VAL`). A 32-seed score carries about
half a point of the plant's chaos. The 2e-4 run took about 10 minutes, the 1e-4 run 17.

| Peak lr, steps, every | Validation seeds | Point mass: step, held-out | Dynamic: step, held-out | Dynamic - point mass | Dynamic - seed (51.7%) |
|---|---|---|---|---|---|
| 2e-4, 300, 30 | 8 | 120, 50.1% | 60, 52.7% | +2.6 [+0.3, +4.7] | +1.0 [-1.2, +3.4] |
| 2e-4, 300, 30 | 32 | 120, 50.1% | 60, 52.7% | +2.6 [+0.3, +4.7] | +1.0 [-1.2, +3.4] |
| 1e-4, 300, 10 | 8 | 210, 51.1% | 160, 53.5% | +2.4 [+0.3, +4.4] | +1.9 [-0.8, +4.4] |
| 1e-4, 300, 10 | 32 | 260, 52.0% | 130, 52.3% | +0.3 [-2.2, +3.0] | +0.6 [-1.3, +2.7] |

The validation curves (32 seeds), from the untrained policy's 0.494:

- **2e-4:** point mass flat to step 120 (0.508), then down to 0.33; dynamic up to 0.509 at
  step 60, then down to about 0.42.
- **1e-4:** point mass creeps up to 0.50-0.516 and holds; dynamic rises to 0.521 at step 130,
  then falls to 0.485 by step 300.

**Read it carefully.**

- **Training's real effect is small.** With the noise down, the curves are smooth and rise only
  1.4 to 2.7 points on validation at their best. Held out, that is +0.3 to +0.6 against the seed
  at 1e-4.
- **The 8-seed 1e-4 lead was mostly luck.** Its dynamic policy led the seed by +1.9; with 32
  seeds the chosen checkpoint leads by +0.6.
- **The dynamic-over-point-mass gap is about damage.** It holds at 2e-4 (+2.6), where training
  hurts the point-mass policy more, and vanishes at 1e-4 (+0.3), where neither is hurt.
- **So the training loss moves Jolt reach by about a point at most.** Selection stops it from
  hurting, but cannot add what training does not find. The next lever is the signal: Jolt reach
  as the training objective (for example a zeroth-order fine-tune of the selected policy on
  `reach_on_plant`), or a loss closer to it.

**Zeroth-order fine-tuning on Jolt reach** (`cvc::physics::es_finetune_on_plant`,
`inc/cvc/physics/nav_es.h`):

- **The method.** Antithetic evolution strategies over the policy's flat weights
  (`coef_mlp::export_flat` / `with_flat_weights`).
  - Each iteration scores `pairs` perturbations either way on fresh city seeds, shared by the
    whole population.
  - Centred ranks weight the directions into a gradient estimate, which Adam follows.
  - The centre is validated on `reach.seeds`, and the best is kept, the start included.
  - `tail = 3` searches only the output layer's bias, which shifts the coefficient basin.
- **The tests.** `NavPlantConvergence.EsFinetuneOnJoltReach` (`P8B_ES_*` override it), with
  `CoefficientLeverageOnJoltReach` and `BasinGridOnJoltReach` alongside.
  - It and `EsEnergyAtHeldReachOnJolt` (below) are experiments: they run only with
    `CVC_NAV_ES_EXPERIMENTS=1`. On a hosted CI runner, run alone, the fine-tune took over an
    hour on Linux and 50 minutes on macOS, and the energy run 40 and 30. Their one check,
    best >= start, holds by construction. `NavPlant.EsFinetuneOnPlantMechanics` checks the
    mechanics in every build.

How much the policy moves Jolt reach (32 validation seeds; `CoefficientLeverageOnJoltReach`):

| Policy | Reach |
|---|---|
| the seed's constant (alpha, beta, gamma) = (1, 3, 4) | 50.2% |
| alpha x0.5 / x2 | 46.0% / 49.7% |
| beta x0.5 / x2 | 47.1% / 52.0% |
| gamma x0.5 / x2 | 51.3% / 47.5% |
| the dynamic-trained policy at its selected checkpoint | 52.1% |
| it + random weight noise, sigma 0.02 / 0.05 / 0.1 | 50.7-50.8% / 51.4% / 48.2-51.1% |

- **The basin has the leverage, about ±4 points.** Every random weight perturbation of the
  trained policy costs 0.7 to 4 points: it sits near a local top.
- **ES over all 4,739 weights** (40 iterations x 16 pairs, sigma 0.02, 4 seeds each) never
  beat its start on validation; held out it stays at 52.3%.
- **ES over the basin** (`tail = 3`, 30 x 8 pairs, sigma 0.2, 8 seeds each): from the trained
  policy, again no gain. From the seed, +1.0 on validation but -0.1 [-2.3, +2.1] held out. Its
  steps (about 6% in beta) move reach less than the plant's chaos.

**A grid over the basin does what ES could not** (`BasinGridOnJoltReach`): constant policies
with alpha 1, beta in {3, 4.5, 6, 9} and gamma in {2, 3, 4}, on the 32 validation seeds; the
top three then on the 20 held-out seeds against the seed.

| Basin | Validation | Held out | Against the seed (51.7%) |
|---|---|---|---|
| beta 9, gamma 4 | 52.1% | 55.4% | +3.8 [+1.9, +5.7] |
| beta 6, gamma 4 | 52.0% | 52.7% | +1.0 [-1.0, +2.9] |
| beta 4.5, gamma 2 | 51.4% | 54.2% | +2.5 [+0.3, +4.7] |

- **The first policy that beats the seed on Jolt outside the CI.** It is the seed's own form with
  a three-times stronger goal spring: (1, 9, 4).
- **It sits on a cliff's edge.** A goal spring too strong for the damping collapses reach to
  about 20%: beta 9 with gamma 3, beta 6 with gamma 2, and beta 9 with gamma 2 all did. The
  winner is one grid step from it.
- **So the coefficient policy's leverage on Jolt is mostly its basin.** The seed's basin was
  tuned on the kinematic sim. The learned, state-dependent part adds little on this vehicle and
  this city.

Not yet:
- a learned, state-dependent policy that beats the seed on Jolt outside the CI (a tuned
  constant basin does, above);
- pycvc (its `nav_train_coef_mlp` takes rollout 0/1 only, and the material trainer's binding
  turns on only the bicycle);
- a CUDA build of the plant (it is `CVC_HD`, but nothing compiles it for the device yet).

## The four-wheel vehicle: `dyn4_plant` (P8d)

`diff::dyn4_plant` (`inc/cvc/nav/detail/dyn_4w.h`) is P8a's single-track vehicle on its four
contact patches. `bike_control` commands it as it commands `dyn_plant`, so a policy trains
through per-wheel grip, the terrain under each wheel, and the load
each corner carries.

```cpp
#include <cvc/nav/dyn_4w.h>
cvc::nav::diff::dyn4_params p = cvc::physics::nav_plant::dyn4_params_at(vehicle_desc::roboracer(), cfg.veh_L);
my_ground ground;   // at(wx, wy, mu, height) at a world point, templated on the scalar
cvc::nav::diff::dyn4_bike_step(F, s, gx, gy, al, be, ga, v, p, ground, s1);
cvc::nav::diff::dyn4_bike_step_bwd(F, s, gx, gy, al, be, ga, v, p, ground, g_s1, g_s, gal, gbe, gga);
```

**State** (14 values): P8a's `(x, y, psi, vx, vy, r)`; the body's heave `z`, roll `phi` and pitch
`th` from its rest height on flat ground, with their rates; and the steering `delta` and drive
force `a` behind their actuators.

**One step**:

- **Wheels.** Front left, front right, rear left and rear right sit at (lf, ±tf) and (-lr, ±tr)
  from the centre of gravity. The front pair takes Ackermann angles for the virtual centre wheel:
  tan delta_i = L tan delta / (L - y_i tan delta).
- **Ground.** Each wheel reads its friction and terrain height from the ground sampler at its
  world position. The plant is templated on the sampler, so the terrain's gradient flows through
  the dual numbers too.
- **Loads.** Each corner's spring-damper, F = Fz0 + k (height - corner height) - c (corner rate),
  is that tire's load, floored by a softplus so a tire pushes but never pulls.
- **Tires.** Each wheel's drive or brake share saturates at mu_i Fz_i, and Pacejka gives the
  lateral force within the friction ellipse's remainder. (vy, r) advance by implicit Euler, each
  tire linearized by its secant through its root (P8a's solve, four tires).
- **Body.** (z, phi, th) advance by implicit Euler on the corners' springs and dampers, driven by
  the ground and the tire forces' moments about the centre of gravity.
- **The rest is P8a's:** the speed prediction and correction, the low-speed blend and the pose.
- **Adjoint.** As P8a: the plant is written over a scalar type, and `dyn4_plant_vjp` runs it on
  forward-mode duals.

**Parameters** from the spec sheet: `nav_plant::dyn4_si_of(vehicle_desc)`.
- P8a's numbers, plus each axle's half track.
- The chassis box's roll and pitch inertias.
- Each corner's spring and damper, from its axle's suspension frequency and damping ratio at the
  corner's static load.
- `dyn4_params_of` converts them by length exponent.
- `dyn4_params_at(desc, wheelbase)` gives the trainer's normalized vehicle, as P8a does.

**Gates** (`nav_dyn_4w_test`, the Humvee from its vehicle_desc unless noted; the tires fitted to
Jolt's since #753):

| Gate | Result |
|---|---|
| At rest on flat ground, 200 steps | stays put; the loads sum to m g |
| Single-track limit (zero track, rigid corners) against P8a's `dyn_plant`: steer, throttle, then brake from 15 m/s | positions within 0.130 m over 93 m; velocities within 0.016 m/s |
| A steady turn: the roll moment the springs carry against h x the lateral force | -5118 against -5116 N m; the outside wheels loaded |
| Steady braking at 3 m/s^2: the front axle against the quasi-static m (g lr - a h) / L | 15820 against 15820 N |
| Braking with the left wheels on mu 0.3 and the right on 0.9 | yaws toward the high-mu side (-0.89 rad in 1.8 s) |
| An 8 cm bump under the left wheels | front left peaks at 1.37 x its static load, front right 1.11 x |
| The plant's VJP against central differences (double; cruise, corner, braking in a turn, crawl, a bump) | worst relative 2.6e-9 |
| One step under the controller (a RoboRacer in the trainer's city, float) | directional relative error 4.0e-5 |
| Steady lateral acceleration (the body's, vy' + r vx), mu 0.4 / 0.9 | 0.94 / 0.89 mu g |
| Stopping from 20 m/s, mu 0.4 / 0.9 | 47.2 / 21.5 m against v^2 / (2 mu g) = 51.0 / 22.7 m |
| Against Jolt's Humvee (physics_vehicle_test's numbers) | skidpad 6.30 m/s^2 at mu 0.7 (Jolt 6.85); stopping 24.10 m from 20 m/s at mu 0.8 (Jolt 25.90) |
| Ackermann, as Jolt's gate: cot(right) - cot(left) against track / wheelbase | exact (1e-12) for delta from -0.3 to 0.45 |
| Held at delta at 3 m/s, the low-speed blend off: the rear axle's circle against L / tan(delta) | -0.03 / +0.18 / +0.62% at 0.15 / 0.30 / 0.45 rad |
| Golden for the torch twin (`tests/data/dyn4_golden.json`; the sprung and rigid Humvee on a mu split with a bump): next state and VJP | reproduced to 1e-9 |

A gradient check's starting positions should not be the scene's sampled starts. Those sit on
occupancy-cell centres, which are the SDF field's grid nodes, where the bilinear field has a
kink: a central difference there averages two cells' slopes. Jittered off the nodes, the
controller check above went from 5e-2 to 6.2e-5.

The external force reaches this plant as it reaches the others: see
[An external force in the differentiable rollout](#an-external-force-in-the-differentiable-rollout).

Not yet:
- the GRL-SNAM torch twin, held to the golden above (and to the force port's,
  `tests/data/dyn4_ext_rollout_golden.json`);
- the anti-roll bars, the unsprung masses and wheel spin.

## An external force in the differentiable rollout

The deployed drive takes a per-agent force through `ext_force` (`drive_step_ext`,
`bicycle_rollout_ext`). The differentiable rollout takes the same force, so a trainer can learn
whatever produces it through the real controller and vehicle.

**One step** (`diff::ext_in`, `detail/diff_rollout.h`):

```cpp
diff::ext_in e;
e.fx = fx; e.fy = fy;       // F_ext at this step's pose, normalized drive frame
e.dfx_dox = ...;            // optional J = dF_ext/do: the reverse adds J^T dL/dF into dL/do
e.steer = 1;                // it also steers, like ext_force::steer
diff::bike_step(F, ox, oy, th, sp, gx, gy, al, be, ga, v, ox1, oy1, th1, sp1, &e);
float g_fext[2];
diff::bike_step_bwd(F, ox, oy, th, sp, gx, gy, al, be, ga, v, g_ox1, g_oy1, g_th1, g_sp1,
                    gox, goy, gth, gsp, gal, gbe, gga, &e, g_fext);
```

- **Where it enters:** exactly where `rollout_impl` adds `ext_force`. It joins the barrier and
  goal forces in the longitudinal command and, when it steers, the repulsive force in the steering
  bias. The behind branch's barrier term stays barrier-only.
- **Null is unchanged:** a null `ext_in` is the controller as it was, bit for bit.
- **A force computed in world units:** the drive frame is o = S (w - c). A caller that writes a
  world-frame force into the drive frame unscaled passes
  F_drive = F_world. Its Jacobian still needs the chain rule: dF/do = (dF/dw) / S. Passing dF/dw
  instead is off by a factor of S.
- **The dynamic plants:** `bike_control`, `bike_control_bwd`, `dyn_bike_step` and
  `dyn_bike_step_bwd` (P8a's single track), and `dyn4_bike_step` and `dyn4_bike_step_bwd` (P8d's
  four wheels, on its ground sampler), take the same pair.

**A trajectory** (`diff::bike_rollout_ext` / `bike_rollout_ext_bwd`, `inc/cvc/nav/ext_rollout.h`;
the dynamic twins are `dyn_rollout_ext` and, for the four-wheel plant on a ground `G`,
`dyn4_rollout_ext<G>` / `dyn4_rollout_ext_bwd<G>`, whose tape holds `dyn4_state`):

- **Per-step inputs:** the goal and the coefficients may change at every step (the carrot, and
  coef_mlp's per-tick output). They are exogenous: recorded, with no derivative through the pose.
- **The force:** a callback gives F and J at a pose. It is sampled every `hold` steps and held
  between, as a harness that computes it once per frame does. `hold = 1` is the deployed drive's
  per-substep force.
- **The tape:** every step's state, and the force and J it used.
- **The reverse:** it takes dL/dstate at every step and returns dL/dF_ext at every step, dL/dcoef
  (optional) and dL/d(initial state). A hold window's dL/dF goes through J at the step where F was
  sampled.
- **Scope:** host only, one agent per call, independent agents for threads.

Measured (`nav_dyn_bike_test`, `DiffExtForce` and `DiffExtRollout`):

| Check | Result |
|---|---|
| One step against the deployed `bicycle_rollout_ext` (64 agents, with and without steering) | identical (worst 0.0) |
| A zero force against no force, forward and backward | identical |
| dL/dF_ext against central differences, one step | 5.0e-4 kinematic, 1.6e-4 dynamic; 1.3e-3 through a position-dependent force |
| A 10-step kinematic trajectory, hold 1 / 3: dL/d(initial state), dL/dF, dL/dbeta | 2.9e-4 / 3.0e-4, 2.3e-3 / 1.9e-3, 8.3e-4 / 1.3e-3 |
| The same on the dynamic twin: dL/d(initial state), dL/dF | 6.8e-4 / 6.2e-4, 9.5e-4 / 1.0e-3 |
| Four wheels (`nav_dyn_4w_test`, `Dyn4ExtForce` and `Dyn4ExtRollout`; the RoboRacer on a mu split with a bump): a zero force against no force | identical |
| Four wheels, one step: dL/dF_ext, steering / not; through a position-dependent force | 8.6e-5 / 0 (the speed command at its limit: both sides zero); 3.6e-4 |
| Four wheels, a 10-step trajectory, hold 1 / 3: dL/d(initial state), dL/dF | 5.7e-3 / 1.9e-3, 1.3e-3 / 1.4e-3 |

The finite differences are taken where they measure a slope. The starts are jittered off the
SDF's grid nodes. A case whose forward and backward differences disagree, which straddles a
controller switch, or whose central differences at eps and eps / 2 disagree, where the slope
is sharply peaked within the step, is skipped and counted, at most a quarter of them. Which cases
those are depends on the scene's draws, which differ by standard library. macOS's libc++ failed
the earlier unfiltered sums (dL/dF 0.38). Swept over 100 other draws (`CVC_EXT_CASE_SEED`),
none fails. The four-wheel checks use the same filter. Their 10-step trajectory crosses more
switches (a third of the cases may be skipped), and its initial-state gradient, which runs through
every step's plant and clamps, is held to 1e-1. Swept over `CVC_EXT_CASE_SEED` 1..100, none fails.
The worst were 1.4e-2 (one step's dL/dF), 3.4e-2 (the trajectory's dL/dF) and 6.6e-2 (its
dL/d(initial state)).

**The torch twin** is GRL-SNAM's `grl_snam/ext_rollout.py` (CVC-Lab/GRL-SNAM#122): batched,
float64 or float32, CPU or CUDA, with autograd as the reverse.
- **Its fixture:** `DiffExtRollout.Golden` writes `tests/data/ext_rollout_golden.json`, six agents'
  12-step trajectories and reverse gradients, which the twin replays.
- **Agreement:**
  - In float64, its autograd matches this float reverse to 1.9e-7 (dL/dF_ext), 4.3e-9 (dL/dcoef)
    and 1.6e-7 (dL/d(initial state)).
  - The trajectories agree to 1.6e-7.
  - So the finite-difference figures above are the differences' own error, not an adjoint gap.
- **The four-wheel fixture:** `Dyn4ExtRollout.Golden` writes `tests/data/dyn4_ext_rollout_golden.json`
  for the twin's four-wheel path (not written yet): four agents' 10-step trajectories of all 14
  state components and their reverse gradients, with the field, the plant's parameters and the
  ground (a friction split and a bump) they used. Rewrite it with `CVC_DYN4_EXT_GOLDEN_WRITE=1`.

## Energy in the loss: the power model (P8c)

The dynamic rollout can price what the vehicle draws from its fuel or battery, as the P3 ledger
counts it. A differentiable power model (`diff::power_step`, `detail/dyn_power.h`) reads the
dynamic vehicle's state at each step's end, and the trainer adds it to the loss.

**The model.** With the drive power P = m a vx (`a` the drive force per unit mass, `dyn_state::a`):

```
E = dt (k_pos P+ + k_neg P- + p_idle + c_v |vx| + c_v3 |vx|^3 + c_lat m (vx r)^2 |vx|)
```

- P+ and P- are smooth positive and negative parts, and |vx| a smooth magnitude, so E is smooth.
- `k_pos` is the source energy per joule of positive drive work: the driveline and engine together.
- `k_neg` is what braking returns: 0 for a fuel engine, which burns none on the overrun, and
  negative where it regenerates.
- `p_idle` is what the source draws at rest. On the Humvee it is mostly engine friction at
  running speed.
- `c_v` and `c_v3` carry rolling resistance and drag where the dynamic vehicle leaves them out (a
  direct drive's, whose fitted command is the achieved acceleration).
- `c_lat` is cornering: a tire's lateral force slips it sideways, losing about m a_lat² |vx| / C,
  with a_lat = vx r.

**Calibrated to the ledger** (`cvc::physics::fit_power`, `physics/plant_log.h`):

```cpp
plant_log log(sim, plant);  // each planar_sample now carries the tick's energy (J)
...
const power_fit f = fit_power(log.runs(), dt, desc);        // non-negative least squares
const power_fit h = evaluate_power(held_out, dt, desc, f.si);  // r2, total and per-run error
cfg.power = cvc::nav::dyn_power_params_of(f.si, desc.mass, frame);
```

- Every term is linear in its coefficient, so the fit is least squares on each tick's energy,
  with each coefficient's sign held.
- The fit reads the drive force the dynamic vehicle needs to make the motion the plant made: the
  measured acceleration less its own drag, rolling resistance and grade. The trainer's rollout
  carries that same force as `dyn_state::a`.
  - A first version read nav's command through the actuator's lag instead. At cruise that
    command is about zero, while Jolt's driver model holds speed with its own trim, so the
    resistances' work was invisible and the fit put the Humvee's whole draw into idle.
- `cvc plant-log --fit-power` fits every seed but the last and reports the last
  (`residual.json`'s "power"). `samples.csv` gains an `energy` column.

**Measured** (`PlantLogPowerConvergence`, Windows): each vehicle fitted on four seeds of both
scenes, 30 s each, and judged on the runs of two held-out seeds that hit no other vehicle (22 of
24 for each):

| Vehicle | Held-out tick r² | Total energy | Swerve | City | Worst run |
|---|--:|--:|--:|--:|--:|
| Humvee | 0.90 | +1.3% | +8.7% | -1.6% | 13% |
| RoboRacer | 0.97 | +1.9% | +17.7% | -0.2% | 28% |

- A run that hit another vehicle (`planar_sample::vehicle_contact`, or an impact past `kImpactG`)
  is reported apart. After the hit, the RoboRacer's two stalled with their wheels spinning, and
  the model missed them by -35% and -8%: it has no collisions and no wheel spin, so no term of its
  state sees that energy. The Humvee's two cost what the model gives (-0.4%, +0.3%).
- Which runs collide depends on the platform's floating point. Judged on all 24 runs, the
  RoboRacer's held-out r² was 0.91 on Windows and 0.60 on Linux, where two more of its city runs
  missed by about +31%.

- Fitted on one scene's two seeds, the swerve's held-out totals missed by 12% (Humvee) and 22%
  (RoboRacer). A seed drives all its agents at one speed, so two seeds cannot tell idle from speed.
- The swerve's slowest held-out seed is still over-predicted, by up to 35% a run on the RoboRacer.
  Its measured energy grows faster with speed than any of the terms.
- On the Humvee, runs at the same speed can draw very different energy, most likely from the
  gearbox: at the same road speed, a different gear means a different engine speed and friction.
  A planar model has no gear.

**In the loss** (`train_config::w_energy`, `train_config::power`; `rollout_kind::dynamic`):

- Each step adds `w_energy / window · E_t / (hdt · P_full)` per agent. P_full is the source power
  at full drive force and top speed (`diff::power_full`), so the term is the window's mean power as
  a fraction of full power: O(1), a preference like `w_coll`.
- Its adjoint lands on the step's end state, (vx, r, a)_{t+1}, before the rollout's reverse.
- `w_energy = 0` (the default) leaves the loss and its gradient bit for bit.
- It needs the dynamic rollout and a power model of `dyn`'s vehicle (its mass), or it throws.
- Gates (`nav_coef_train_test`):
  - `PowerStepMatchesFiniteDifferenceAndConverts`: the step's derivatives against central
    differences, and the SI conversion.
  - `EnergyGradcheckMatchesFiniteDifference`: the energy term's own gradient, direction error
    9.0e-4.
  - `EnergyTrainingDrawsLessEnergy`: over 60 steps from the same start the energy term falls
    0.970 → 0.937 with it, against 0.962 without it.
- **On Jolt, the gate is not met** (`NavPlantConvergence.EnergyTrainedPolicyOnJolt`, Windows).
  - The setup: the RoboRacer's power model fitted to its own ledger, then two dynamic-rollout
    trainings that differ only in `w_energy`, both scored on the P8b gate's 20 held-out city
    seeds by `cvc::physics::score_on_plant`. That is reach_on_plant's runs, plus the energy
    the agents drew and the distance their chassis drove; Wh/km is the fleet's total over its
    total.
  - The fit: k_pos 1.47, k_neg -0.66, p_idle 4.5 W, c_v 0, c_v3 0.0015, c_lat 0.018 (r² 0.955).
  - Results, with paired-bootstrap 95% CIs over the 20 seeds:

    | Training | w_energy | Reach with | Reach without | Δ reach (points) | Δ Wh/km | Metres a seed (with / without) |
    |---|--:|--:|--:|--:|--:|--:|
    | lr 5e-5, 150 steps | 3 | 51.2% | 51.1% | +0.1 [-1.6, +1.8] | -0.03 [-0.52, +0.43] | 2068 / 2110 |
    | lr 2e-4, 300 steps | 1 | 14.7% | 39.0% | -24.3 [-27.3, -21.2] | -0.23 [-0.89, +0.66] | 1095 / 2589 |
    | lr 2e-4, 300 steps | 3 | 11.0% | 39.0% | -27.9 [-31.5, -24.5] | +0.17 [-0.59, +1.12] | 940 / 2589 |
    | lr 2e-4, 300 steps | 10 | 7.6% | 39.0% | -31.4 [-34.8, -28.1] | -0.27 [-1.15, +0.66] | 563 / 2589 |
    | lr 2e-4, 300 steps | 30 | 5.8% | 39.0% | -33.1 [-36.5, -29.9] | -0.48 [-1.31, +0.56] | 306 / 2589 |

    The untrained seed reaches 51.7% at 4.02 Wh/km.
  - At the gentle rate, training barely moves the policy, as in P8b, and the term changes
    nothing.
  - Where training does move it, the term teaches the policy to stop driving, not to drive for
    less: distance falls 2-8x and reach with it. Energy per km does not fall significantly
    against the policy trained without the term.
  - Per-seed Wh/km ranges from 2.7 to 5.8 on the same policy. The plant's chaos is large next to
    any per-km effect.
  - The cause is the term's form. It prices mean power over the window, so the cheapest policy
    is the one that moves least. The gate asks for energy per distance, at the same reach.
- **Priced against progress** (`train_config::energy_ref`):
  - Each step adds `w_energy / window · softplus(E_t - energy_ref · p_t) / (hdt · P_full)`, with
    `p_t = |o_t - goal| - |o_{t+1} - goal|` the step's progress toward the goal.
  - A step pays only for the energy its progress does not cover at the reference, and the smooth
    hinge (1% of a full-power step wide) keeps the term >= 0, so it never rewards progress itself.
  - `cvc::nav::dyn_energy_ref_of(wh_per_km, frame)` converts a reference from Wh/km: 3.6 J/m,
    whose length exponent is 1.
  - Its adjoint adds the progress's gradient on both ends of the step, |o_{t+1} - goal| and
    |o_t - goal|. `PricedEnergyGradcheckMatchesFiniteDifference` checks it: direction error
    1.4e-3 at 2 Wh/km.
  - `energy_ref = 0` (the default) is the time-priced term above, bit for bit.
- **On Jolt, priced against progress: the reach loss shrinks, but the gate is still not met.**
  Same gate and seeds, lr 2e-4 and 300 steps (where training moves the policy); the policy
  trained without the term reaches 39.0% at 2.48 Wh/km over 2589 m a seed:

  | w_energy | Priced at (Wh/km of progress) | Reach with | Δ reach (points) | Δ Wh/km | Metres a seed |
  |--:|--:|--:|--:|--:|--:|
  | 3 | 4 | 24.9% | -14.1 [-17.5, -10.6] | +0.34 [-0.25, +0.93] | 1587 |
  | 3 | 8 | 31.9% | -7.1 [-10.5, -3.5] | +0.05 [-0.32, +0.42] | 1761 |
  | 3 | 16 | 34.3% | -4.7 [-7.7, -1.5] | +0.35 [-0.28, +1.04] | 1799 |
  | 10 | 8 | 29.9% | -9.1 [-12.6, -5.4] | +0.05 [-0.47, +0.68] | 1540 |

  - The policy keeps driving, and gives up a third to a fifth of the reach the time-priced term
    cost it. The higher the reference, the less it gives up.
  - It still draws no less per km on Jolt: every Δ Wh/km is between -0.05 and +0.35, and every
    CI crosses 0.
  - So what the trainer saves in the planar power model does not reach Jolt's ledger. The fit
    describes the untrained policy's driving (r² 0.955) and the gate's seeds vary 2.7-5.8 Wh/km
    on one policy. The differences the term could make are smaller than both.
  - Every trained policy here draws less per km than the untrained seed (2.5-2.8 against 4.02
    Wh/km), with or without the term. Training at this rate also costs reach (39.0% against the
    seed's 51.7%), so that saving is the slower driving the training itself brings, not the
    energy term.
- **Why: the model does not see half of what the gate's runs draw.**
  - `plant_reach_options::predict_energy` logs every gate run (`plant_log`) and gives the fitted
    model's energy over the same motion (`plant_drive_score::model_J`). `plant_drive_score::ledger`
    sums the agents' P3 ledgers, so each run says where its energy went.
  - On the gate's dense city (48 RoboRacers in the occupancy-grid city), the model under-predicts
    every policy: by 40% (without the term), 53% (priced at 8 Wh/km) and 31% (the seed). It was
    within ±1.4% on the plant-log scenes it was fitted to.
  - It does predict the direction of what the term trains. On the priced policy's own Jolt runs
    it says the policy saves -0.30 Wh/km [-0.38, -0.24]; Jolt measures +0.05. Per-run Wh/km,
    model against measured, correlates at r = 0.74 over the 60 runs, and the per-seed paired
    differences at r = 0.50.
  - The ledger shows what it misses (Wh/km of the fleet's distance; w_energy 3 at 8 Wh/km):

    | Policy | Engine | Tire slip | Brakes | Drivetrain | Aero | Rolling | Regen |
    |---|--:|--:|--:|--:|--:|--:|--:|
    | Without the term | 2.43 | 1.34 | 0.63 | 0.45 | 0.17 | 0.14 | 0.38 |
    | With it | 2.31 | 1.67 | 0.34 | 0.42 | 0.02 | 0.14 | 0.18 |
    | The untrained seed | 4.15 | 2.16 | 1.42 | 0.78 | 0.18 | 0.14 | 0.86 |

  - Tire slip is about half of the engine's work here. The model sees only the lateral part of
    it, and that only through c_lat (vx r)^2 |vx|. Longitudinal slip, and wheels pushing on
    walls and on each other, leave no trace in the planar motion it reads.
  - The trained policy did what the term asks: it brakes less (0.63 -> 0.34) and drives slower
    (aero 0.17 -> 0.02), and the model counts both. But it slips its tires more (1.34 -> 1.67),
    which the model does not count, and that cancels the saving on Jolt.
- **The tire term** (`power_params::tire_lat`, `tire_lon`; `diff::power_step_tire`):
  - `diff::dyn_tire_power` gives what the dynamic vehicle's own tires lose to slip at a state
    (vx, vy, r, delta, a), from the same loads, force split, friction ellipse and Pacejka curves
    `dyn_plant` integrates.
    - Lateral: each axle's force times the slip velocity its slip angle implies, u sin(alpha).
      Pacejka's force has alpha's sign, so the product is >= 0 and smooth everywhere. The
      geometric sideways velocity disagrees with the force's sign at a crawl, where the slip
      angles' speed floor acts, and gave a corner there.
    - Longitudinal: each axle's drive or brake force F times kappa u, kappa the slip ratio at which
      its Pacejka curve, scaled to the axle's grip, gives F.
  - The step adds `dt (tire_lat P_lat + tire_lon P_lon)`. Its gradient on (vx, vy, r, delta, a)
    comes from forward-mode duals, so the energy term's adjoint now also reaches vy and delta.
  - `fit_power` calibrates it in two stages when the runs carry the ledger's tire term
    (`planar_sample::tire`, which `plant_log` now records per tick):
    1. Non-negative least squares of each tick's ledger `tire` on P_lat and P_lon
       (`power_fit::tire_cal_lat`, `tire_cal_lon`, `tire_r2`).
    2. The calibrated sum joins the energy fit as one column, whose coefficient is what the
       source pays per tire J.
  - `fit_power(std::vector<power_runs>, ...)` fits run sets logged at different ticks together.
    `score_on_plant`'s `logs` argument returns every scored run's `plant_log`, so a model can be
    fitted on the scene it is judged on.
  - Gates:
    - `PowerStepTireMatchesFiniteDifference`: all five derivatives at 180 states.
    - `TireEnergyGradcheckMatchesFiniteDifference`: direction error 5.0e-4, and 3.1e-3 priced at
      8 Wh/km.
    - `FitCalibratesTheTireTermToTheLedger`: a known calibration and price come back to 0.1%.
- **On Jolt, with the tire term fitted on the dense city too** (8 seeds of the untrained policy
  on the gate's own city, beside the plant-log scenes):
  - The fit: k_pos 1.61, k_neg -0.76, p_idle 7.9 W, c_lat 0.0016, tire_lat 1.07, tire_lon 2.57.
    The energy r² over every tick is 0.69. The tire term explains the ledger's per-tick tire
    energy at r² 0.25 only.
  - The model now misses far less on the gate's runs: -3.9% on the seed (from -31%), -12.6% on
    the policy trained without the term (from -40%).
  - The gate is still not met (lr 2e-4, 300 steps, w_energy 3; without the term: 39.0% at
    2.48 Wh/km):

    | Priced at (Wh/km) | Reach with | Δ reach (points) | Δ Wh/km measured | Δ Wh/km the model predicts | Tire slip, without → with |
    |--:|--:|--:|--:|--:|--:|
    | 0 (time) | 11.4% | -27.6 [-31.0, -24.3] | +0.10 [-0.75, +1.18] | +0.86 [+0.72, +1.01] | 1.34 → 1.79 |
    | 8 | 29.9% | -9.1 [-12.4, -5.6] | -0.02 [-0.51, +0.46] | -0.26 [-0.36, -0.17] | 1.34 → 1.64 |
    | 16 | 34.0% | -5.0 [-8.3, -1.1] | +0.17 [-0.43, +0.87] | -0.27 [-0.36, -0.18] | 1.34 → 1.77 |

  - Every trained policy's tire slip on Jolt rises against the policy trained without the term,
    whatever the model predicts. The model's tire term, which explains a quarter of the ledger's,
    does not see the slip these policies add.
  - So the slip that grows is not the planar vehicle's. Most likely it is contact: 48 vehicles in
    a dense grid touch walls and each other, and tires scrub in contact and spin in stalls. The
    planar rollout has neither collisions nor wheel spin, so no term of its state can price them.
- **ES on Jolt's measured energy** (`es_options::fitness`, `es_energy_at_held_reach`;
  `NavPlantConvergence.EsEnergyAtHeldReachOnJolt`):
  - `es_finetune_on_plant` can now climb any score of a policy's Jolt runs (`score_on_plant`).
    Each iteration then scores the start policy on the same seeds too, and the fitness reads both.
  - `es_energy_at_held_reach(price, slack)` scores -(Wh/km) - price x max(0, points of reach
    lost against the start - slack). It needs no model of where the energy goes.
  - The run: starts from the P8b gate's dynamic-trained policy (lr 5e-5, 150 steps). 30
    iterations x 12 antithetic pairs, sigma 0.02, Adam at 0.005, 4 fresh city seeds an
    iteration; validated every 5 on 16 seeds (200..215). Price 0.1 Wh/km a point, past 1 point.
  - Results on the gate's 20 held-out seeds:

    | Searched | Best at | Reach | Wh/km | Δ reach (points) | Δ Wh/km | Tire slip (Wh/km) |
    |---|--:|--:|--:|--:|--:|--:|
    | The start | — | 51.1% | 3.95 | — | — | 2.10 |
    | Every weight | 25 | 52.7% | 3.82 | +1.6 [+0.1, +3.1] | -0.13 [-0.51, +0.21] | 1.99 |
    | The basin alone (tail 3) | 30 | 51.4% | 4.02 | +0.2 [-1.6, +1.9] | +0.07 [-0.33, +0.52] | 2.15 |

  - Over every weight, the fine-tune is the first policy to move both numbers the right way. Its
    reach gain is outside the CI, which ES on reach itself never found. Its energy saving (-3.3%,
    from tire slip 2.10 -> 1.99) is not.
  - The basin alone finds nothing.
  - Validation is noisy against the gains: the centre's validation Wh/km wanders 3.92-4.33 over
    the run, about as much as the saving it is after.
  - So the gate is still not met on energy, but this is the closest result, and the only one with
    no reach given up.
  - Next: more validation seeds (32, as P8b found) and more iterations; and fewer contacts first
    (layer 1 of the plan below), since the energy left is tire slip in contact.
- **Next: collisions, wheel spin and yielding** (Joe, 2026-10-24: "the training model needs
  collisions and wheel spin"; the vehicles should drive around each other, and get out of the
  way of an incoming vehicle's path):
  1. Yielding in deployment: `sim_world::set_yield`, intents through an `intent_channel` hook
     (perfect comms by default; the DBG case over its radios), a `yield_resolver` hook,
     priorities. In progress (nav/yield.h).
  2. Contact and wheel spin in the training model: a wheel-speed state per driven axle with
     longitudinal slip, and penalty contact with friction against walls and between agents, in
     shared multi-agent training scenes.
  3. Learned awareness: neighbour and intent features in the policy, trained against contact and
     blocking losses, on the same intents.
- Not yet in the torch twin. Like any change to the loss, it must reach `sdf_nav`'s
  `train_bicycle` before the two are compared.

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
| `w_coll` | 3.0 | the safety-for-reach dial — see [the loss](#the-loss-and-why-both-terms-are-normalized) |
| `d_safe` | 0.0 | clearance below which the penalty engages; 0 = use the scene's `d_hat` |
| `grad_clip` | 5.0 | global-norm gradient clip |
| `rollout` | `surrogate` | see [the switch](#the-rollout-switch-surrogate-vs-bicycle) |

## How it works

Per truncated-BPTT window, the graph is: `coef_feats(o) → CoefMLP → rollout step →
sample(o') for the collision term`, chained through the state, with the loss

    |goal − o_final| / region_n  +  w_coll · Σ relu(d_safe − (phi − rr)) / d_safe

described below. The reverse pass is hand-written
adjoints (no autograd engine): the bilinear-sample **position VJP**, the MLP
backward (Linear/SiLU/softplus), the IPC-barrier derivative, and the rollout step's
own adjoint — surrogate or the full bicycle. Adam with global-norm clipping; the
window is detached every `window` steps to bound the graph.

The differentiable primitives are `__host__ __device__` in
`cvc/nav/detail/diff_rollout.h`, so the CPU trainer and the CUDA kernel compile the
**same** adjoint source — the device backward is correct by construction.

## The loss, and why both terms are normalized

Both terms are O(1) so `w_coll` expresses a **preference**, not a unit
conversion. That is load-bearing rather than cosmetic. The original form —
`|goal − o| + w_coll · Σ relu(rr − phi)` — could not train this at all:

* **Units.** Goal distance spans the world (~5.5 normalized); the penalty was a
  breach depth bounded by `rr` = 0.15. ~36× apart before anything else.
* **Sparsity.** A breach depth is nonzero only for an agent *already inside*
  geometry — **0.00%** of a batch of free-space starts. Averaging divided the
  term by another ~100×.

Together that put the collision term at **0.2–0.7% of the loss**, so the
optimiser minimised goal distance alone — and since slowing costs goal distance,
it was correctly learning *not to slow down*. Balancing it would have needed
`w_coll` ≈ 3,700 on ice and ≈ 55,000 on dry: when no constant works across
conditions, the *terms* are wrong, not the weight.

So the penalty is a **margin shortfall**, nonzero for anything merely *near*
geometry (21% of random starts against 0.00%), and goal distance is divided by
the world half-extent. Measured on the torch reference over three seeds,
`w_coll` 1–3 improves on the shipped policy in reach *and* penetration, 10 is
bimodal, and 30 cuts penetration ~94% for two thirds of the reach.

This must stay in step with `sdf_nav`'s `train_bicycle`. A native trainer
descending a different objective from the torch reference is the same class of
divergence the rollout parity gates exist to prevent — and one they cannot
catch, since they hand both paths the same params and compare *traces*, not
losses.

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
