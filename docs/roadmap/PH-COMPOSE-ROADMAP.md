# cvc::ph composition: compositional port-Hamiltonian world models (C-PHAST)

Status: Phase 1 in review. This plan covers native C++ composition of port-Hamiltonian (pH)
components in `cvc::ph`, with pycvc bindings and `cvc ph ...` subcommands. It follows the way
GRL-SNAM and LieSPHGP were brought into libcvc.

Phase 1 is two layers, so the core can be published without the rest:

1. **The core:** the library, CLI, pycvc, `docs/PH_COMPOSE.md` up to its parity section, and
   `ph_compose_test`. Its tests need no golden file and name no phast internals.
2. **Against the reference:** this plan, `src/cvc/tools/cphast_fixtures.py`, its golden file,
   `ph_compose_parity_test`, and the parity section of `docs/PH_COMPOSE.md`.

## Attribution

The method is C-PHAST, by **Shubham Bhardwaj** and **Chandrajit Bajaj** (UT Austin, CVC):
*C-PHAST: Compositional Port-Hamiltonian World Models for Structured Dynamics Transfer* (2026).
C-PHAST extends PHAST (Bhardwaj & Bajaj, arXiv:2602.17998). The authors' reference code is
CVC-Lab/phast (Apache-2.0).

The code here is written from the paper's equations. phast was run only to produce parity
fixtures; no phast source is copied. Even so, it is a port of their method, so it lands only
once the authors have agreed to its LGPL-2.1 licence (recorded on the first PR).

## What C-PHAST is

C-PHAST keeps each component's physics local and supplied wiring global. The steps:

1. **Component laws.** Each component *i* has a state `x_i`, a stored energy `H_i`, a
   lossless structure `J_i = -J_i^T`, a dissipation `R_i = R_i^T >= 0` and a port map `B_i`:
   `dx_i/dt = (J_i - R_i) grad H_i + B_i u_i`, with output `y_i = B_i^T grad H_i`. In PHAST's
   mechanical form, `x = (q, p)`, `H = V(q) + 1/2 p^T M(q)^-1 p` and `R = diag(0, D(q))`.
2. **Interconnection.** Ports are paired by a skew matrix `C = -C^T` over all coupling
   ports: `u_hat = -C y_hat`. Its blocks `C_ij` may be rectangular, and `C_ii = 0`. An
   electrical network's `C` comes from its incidence matrix `A` (bus voltages paired with
   line currents): `C = [[0, -A], [A^T, 0]]`.
3. **Composed field.**
   `dx/dt = (J_hat - B_hat C B_hat^T - R_hat) grad H_total + G_hat u_ext`, where the hats are
   block-diagonal stacks of the component pieces.
4. **Power balance (the paper's Theorem 3.1).** Internal coupling does no net work,
   `sum_i u_hat_i^T y_hat_i = -y_hat^T C y_hat = 0`, so
   `dH_total/dt = -sum_i e_i^T R_i e_i + sum_i (G_i^T e_i)^T u_i`, with `e = grad H`.
5. **Numerical transition.** A step size and a scheme turn the field into a next-state map
   `x+ = F(x, u)`. The paper's electrical scheme is a split step: a coupling half-step, each
   component's local step, then another coupling half-step, done twice per sample. Each
   component's local step does kick, damp, implicit midpoint, damp, kick. The coupling
   half-step is explicit Euler or explicit midpoint.
6. **Accounting per step.** Each step gets trapezoid books: work `W`, dissipated energy
   `E_loss`, and the residual `eps = H(x+) - H(x) - W + E_loss`. The residual is the
   integrator's own error. It is O(dt^3) per step, not zero.
7. **Query.** A query takes the actual system's history and a set of candidates (a wiring
   plus an input sequence each). It estimates the present state once, by fitting the
   history through the frozen model. It then assigns each candidate its starting state
   (existing components keep theirs; added ones get supplied states) and rolls each one out.
   The physical consequences go on an *action card*: peak currents, voltage range,
   accumulated loss. A *certified ranking* then keeps a candidate only if
   `g(a) + dg(a) <= 0` for each limit, and orders `a` before `b` only if
   `J(a) + dJ(a) < J(b) - dJ(b)`.

## Mapping to libcvc

| C-PHAST | libcvc (`cvc/ph/compose.h`) |
| --- | --- |
| component type, its `H, J, R, B` and external-input map | `ph::block_law` (virtual): `energy`, `effort`, `structure`, `port_map`, `input_map`, plus the local split maps `kick`, `damp`, `conserve` |
| linear-quadratic component (a line, a linear DGU, the LQ theorems) | `ph::linear_law`: `H = 1/2 x^T Q x`, constant `J, R, B, G` |
| PHAST mechanical form (the DGU of the electrical studies) | `ph::canonical_law`: `V(q)` = quadratic + MLP residual, diagonal `M(q)`, diagonal `D(q)`, each constant or an MLP; leapfrog or Picard implicit midpoint |
| system specification `S` (components and wiring) | `ph::composite`: `add_block`, `connect`, `connect_incidence`, `connect_skew` |
| skew pairing `C` | `composite::pairing()` (assembled; skew by construction) |
| composed field (Eq. 11) | `composite::field`, `composite::coupling_field` |
| power ledger (Thm 3.1, Eq. 10) | `composite::powers` (`stored`, `loss`, `external`, `internal`) |
| `F = Compose(S, dt)` | `ph::transition` (split scheme, Euler or midpoint coupling), `ph::step_rk4` and `ph::step_implicit_midpoint` (whole-field references) |
| step accounting (Eq. 54) | `ph::step_books` -> `cvc::nav::diff::dyn_ph_books<double>` (dH, dissipated, supplied, residual) |
| physical readout `h_obs` | `composite::readout` (each block's efforts: `(v, i)` per DGU, `i_e` per line) |
| exported trained component | `"cvc-ph-law/1"` JSON (`ph::load_law`), system `"cvc-ph-system/1"` (`ph::load_system`) |

The pieces follow libcvc's rules:

- **No process-static state.** Laws are immutable values shared by `std::shared_ptr`. A
  system and its scratch belong to the caller.
- **The logic is in the library.** `cvc ph simulate|check` and pycvc's `ph_system` are thin
  wrappers around it.
- **The energy books are libcvc's.** A step's accounting is `dyn_ph_books`, the same record
  the vehicle plants keep. Its floor, grade and limiter lines stay zero here.

## Parity with phast

phast's newest commit (2026-09-21, on `main` and `closed-loop-phast-e0`) predates the
paper's October electrical runs. It does **not** contain:

- the nonlinear DGU family (2,407 parameters);
- the explicit-midpoint coupling;
- the voltage-history state fit;
- the certified ranking.

Its microgrid is the linear Cucuzzella/Neary model, run through `CompositionalPHAST`. Each of
its substeps is Euler coupling half-step, block kick, Strang damping around the conservative
step, kick, Euler coupling half-step. That matches the paper's Euler-coupled network step
(two repetitions of `E_{d/2} o L_d o E_{d/2}`, `d = dt/2`). So parity is measured against:

- the known linear DGUs + RL lines on a chain (leapfrog core, Cayley damping), to rounding;
- learned DGU blocks (MLP potential, configuration-dependent mass, Picard implicit midpoint)
  exported from phast's torch modules, to a golden tolerance. The coupling uses the blocks'
  learned efforts, as the paper's Eq. 203 says, through phast's own `needs_grad_q/p` port
  hook. phast's stock `IncidenceCouplingPort` uses `q / C_node` even for learned laws.

The explicit-midpoint coupling has no phast counterpart. It is checked by its convergence
order and by its energy residual, which falls as dt^3.

Fixtures are produced once, outside CI, by `src/cvc/tools/cphast_fixtures.py` from a checkout
of CVC-Lab/phast. They are stored as `src/cvc/tests/data/ph_compose_golden.json`. The script
imports phast and copies none of it.

## Phases (one PR each)

1. **Composition core (this PR).** Block laws with typed ports, pairing assembly from
   explicit pairs, an incidence matrix or a dense skew rule, the composed field, the split
   transition (Euler or midpoint coupling) plus whole-field RK4 and implicit-midpoint
   references, per-step books with the balance check, readouts, and the law/system JSON
   formats. Also pycvc's `ph_system` and `cvc ph simulate|check`.
   Tests:
   - Theorem 3.1 holds to rounding at random states, for random skew pairings and
     heterogeneous blocks;
   - the split step's balance residual falls as dt^3; the whole-field implicit midpoint
     balances a linear system to solver tolerance;
   - Euler coupling is first order, midpoint second;
   - rewiring changes trajectories without changing any law;
   - a microgrid (linear DGUs, RL lines, a fifth DGU added at either of two buses) against
     a fine-step RK4 reference (`ph_compose_test`);
   - parity with phast on the fixtures above (`ph_compose_parity_test`).
2. **The DC microgrid study end to end (in review, stacked on Phase 1).**
   - DGU (Eq. 21) and line laws as presets in `cvc/ph/microgrid.h`.
   - The paper's reference nonlinear DGU (`q_C / C = 2.2 (v + v^3 / 3)`, L = 1.8 H,
     R = 1.2 ohm), through a capacitor curve on `canonical_law`.
   - The four-unit chain with lines 0.3/0.4/0.5 ohm and 0.8 H, and the added DGU and line at
     bus 0 or bus 2, assigned by `assign_state` (Eq. 53, identity reset).
   - The storage inversion (`Storage` in Appendix I.2: physical voltages and currents to charge
     and flux, by Newton, capped at 2, 30 iterations, 1e-9) as `storage_from_efforts`.
   - Writing systems out (`system_json`), and `cvc ph microgrid` / `--physical`.
   - The check:
     - phast's Python covers the linear network (Phase 1's fixtures).
     - phast has no counterpart for the nonlinear reference DGU, so the candidate study runs
       against RK4 at dt/16, which gives the same scores to six digits as dt/64.
     - With known laws, midpoint coupling is 28 to 4,200 times more accurate than Euler (voltage
       RMSE 5.1e-7 against 7.9e-5 V; peak line-current error 5.7e-8 against 2.4e-4 A). Both
       order all 16 candidate pairs as the reference does.
   - Not here: the paper's learned DGU family (2,407 parameters). It is not in phast; it would
     come with Shubham's checkpoints (Phase 4).
3. **The query.**
   - `InitializeState` as the voltage-history fit: least squares over the prior-scaled
     physical state at the window start, propagated through the frozen transition, with
     success, optimality and data-Jacobian rank reported;
   - `AssignState` (retain by identity map, add supplied states, optional reset);
   - candidate rollouts with the six accounting channels;
   - action cards (peak current, voltage range, accumulated loss);
   - the certified ranking (admission `g + dg <= 0`, strict separation of score bounds).
     Its outcome is "certified", "inconclusive" or "abstain".

   Exposed as `cvc ph query` and pycvc.
4. **Training, if needed.** Either native, as `cvc::ph::se2_train` does (Adam on
   hand-written gradients through the split step), or export from phast's PyTorch into
   `"cvc-ph-law/1"`. Which one depends on whether Shubham's trained checkpoints can be
   shared; exporting is the fallback that needs no new optimizer. Joe is asking him for the
   Go2 leg checkpoints (2026-11-14), which Phase 6 needs.
5. **Later.** The planar arm (two-link, attachment maps `B(q) = [0; (dr/dq)^T]`), and legged
   blocks on the MuJoCo plants (Go2 and Spot are in the cvcpkg Menagerie bundle; see the
   mujoco workstream). Convoy formations are not a reason for these maps: a power-preserving
   formation linkage was measured against slot chasing on point masses and lost
   ([PH-CONVOY-FORMATION.md](PH-CONVOY-FORMATION.md)).

## Limits of Phase 1

- `canonical_law` takes diagonal `M(q)` and `D(q)`. phast's Householder rank-1 mass and
  damping terms are not supported yet.
- Periodic coordinates (phast's Fourier configuration features) are not supported yet.
- Port maps `B_i` and input maps `G_i` are constant. The state-dependent attachment maps of
  the arm come with Phase 5.
- An external flow source fed to a line through its input (rather than through a bus) reaches
  the line in the local step and the bus in the coupling half-steps, and the split step lets the
  line's state creep from the geometry it stands for, even at a steady state
  ([PH-CONVOY-FORMATION.md](PH-CONVOY-FORMATION.md)). Make the source a bus, or write the system
  in its frame.
- Picard stopping is per block. phast stops a type's iteration when every instance in its
  batch has converged, so a block that converged early can take extra iterations there.
  The difference is far below the golden tolerance at the fixtures' step sizes.

## Phase 6: the C-PHAST lab (planned)

Decided by Joe (2026-11-14): after Phase 5, build a demo, gym and lab for C-PHAST, with
MuJoCo's actuation and physics in the libcvc world. The first robot is the Unitree Go2,
which lines up with the paper's Go2 push-recovery study.

**What it is**

- **Scenes in the libcvc world.**
  - The Go2 on the shared terrain, beside Jolt vehicles and water.
  - The DC microgrid as a first, physics-free scene, which needs only Phases 2 and 3.
- **C-PHAST predicts; MuJoCo is the truth.** At a decision point (a push, a slope, a
  payload, a vehicle passing), the lab:
  1. estimates the robot's state from what its sensors give;
  2. assigns and rolls out the candidates (recovery commands, gaits, an added payload or arm
     block);
  3. shows each candidate's action card and the certified ranking;
  4. executes the choice in MuJoCo, and draws the predicted future over the actual one.

  MuJoCo can compute its own energies (`mjENBL_ENERGY`). That checks C-PHAST's per-step books
  against the real plant, not only against themselves.
- **Rewiring live.** Add or remove a leg, an arm or a payload without retraining, and watch the
  futures change. This is the paper's composition claim made visible. It also gives Shubham's
  project page the examples Dr. Bajaj described (three hands, an extendable arm, legs trained
  one at a time).
- **A gym.** pycvc `reset` / `step` with MuJoCo as the plant, a registry of scenarios (push
  recovery first), and C-PHAST as a model-based planning baseline.
- **Where it runs.** Natively in Ariadne first. The browser later: cvcpkg has mujoco for wasm
  and wasm-mt, and the TPHT site already runs MuJoCo in a page.

**What it depends on**

- Phases 2 to 5 here. Phase 5 brings state-dependent attachment maps, the mechanical
  observer for position-only sensing, the learned skew coupling between legs, and contact as a
  port.
- MuJoCo in libcvc: the mujoco workstream's plant (`CVC_WITH_MUJOCO`, a `legged` plant kind).
  Under that workstream's proposed split, MuJoCo owns the robot, Jolt keeps the vehicles and
  the world, and both read one height field. One-way proxies come first; two-way contact and
  water forces after.
- Trained Go2 leg blocks: Shubham's checkpoints, exported into `"cvc-ph-law/1"` (Phase 4). If
  they cannot be shared, Phase 4 trains the blocks on MuJoCo rollouts.

**Open**

- MuJoCo is Apache-2.0, like phast. Keep it an optional, separately linked backend, and check
  shipping it with LGPL-2.1-only libcvc the way the phast port is checked.
