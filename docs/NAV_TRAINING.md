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
- [An external force in the differentiable rollout](#an-external-force-in-the-differentiable-rollout)
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

The `nav_train_demo` example (in `src/cvcGL/examples/`, built with
`-DCVC_BUILD_CVCGL=ON -DCVC_BUILD_EXAMPLES=ON`) does exactly
this end to end and picks the CUDA path automatically when available.

**Selecting checkpoints on the physics plant.** The training loss only approximates deployment,
so `train()` can end on the checkpoint that scores best on something closer to it:

```cpp
#include <cvc/physics/nav_eval.h>

cvc::physics::plant_reach_options val;     // reach on Jolt RoboRacers in the scene
val.seeds = {200, 201, 202, 203, 204, 205, 206, 207};  // validation seeds, not the evaluation's
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
  about 3/sqrt(k) points of that alone, so use 8 or more.

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
- Pacejka B and C from the tire's peak slip angle and sliding fraction;
- the drive and brake splits, drag and rolling resistance;
- the steering's rate.

`front_stiffness` (0.85) scales the front's B. It stands in for the load sensitivity and steering
compliance that give a road vehicle its understeer, which a single track has no other way to
show.

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

Not yet:
- a trained policy that beats the seed on Jolt outside the CI;
- pycvc (its `nav_train_coef_mlp` takes rollout 0/1 only, and the material trainer's binding
  turns on only the bicycle);
- a CUDA build of the plant (it is `CVC_HD`, but nothing compiles it for the device yet).

## An external force in the differentiable rollout

The deployed drive takes a per-agent force through `ext_force` (`drive_step_ext`,
`bicycle_rollout_ext`). The differentiable rollout takes the same force, so a trainer can learn
whatever produces it through the real controller and vehicle. cvcdbg's comm-network trainer, whose
communication force steers the agents, is the first user.

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
- **The dynamic plant:** `bike_control`, `bike_control_bwd`, `dyn_bike_step` and
  `dyn_bike_step_bwd` take the same pair.

**A trajectory** (`diff::bike_rollout_ext` / `bike_rollout_ext_bwd`, `inc/cvc/nav/ext_rollout.h`;
the dynamic twin is `dyn_rollout_ext`):

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
| dL/dF_ext against central differences, one step | 9.4e-4 kinematic, 5.7e-5 dynamic; 1.6e-2 through a position-dependent force |
| A 10-step kinematic trajectory, hold 1 / 3: dL/d(initial state), dL/dF, dL/dbeta | 1.8e-3 / 1.7e-3, 3.6e-3 / 4.8e-3, 1.6e-2 / 9.7e-3 |
| The same on the dynamic twin: dL/d(initial state), dL/dF | 1.5e-2 / 1.4e-2, 1.1e-3 / 8.3e-4 |

Not yet: the GRL-SNAM torch twin (batched, float64, run on CUDA, parity with this at 1e-9).

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
