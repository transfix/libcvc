# Composed port-Hamiltonian systems (`cvc::ph` composition)

`cvc/ph/compose.h` joins port-Hamiltonian components through power-preserving ports and steps the
result. The method is C-PHAST, by Shubham Bhardwaj and Chandrajit Bajaj (UT Austin, CVC). The
port-Hamiltonian interconnection it builds on is classical: van der Schaft and Jeltsema,
*Port-Hamiltonian Systems Theory: An Introductory Overview* (2014). The plan and the mapping to
the C-PHAST paper are in [roadmap/PH-COMPOSE-ROADMAP.md](roadmap/PH-COMPOSE-ROADMAP.md).

## The model

A component (`block_law`) has a state `x`, an energy `H(x)`, a lossless structure `J = -J^T`, a
dissipation `R = R^T >= 0`, coupling ports `B` and external inputs `G`:

```
dx/dt = (J - R) e + B u_hat + G u,     e = grad H,     y_hat = B^T e
```

A `composite` stacks components and pairs their coupling ports by a skew matrix `C` (the
pairing), so that `u_hat = -C y_hat`. The pairing does no net work, which gives the power balance:

```
dH/dt = -e . R e  +  e . G u          (loss, then the external inputs' power)
```

Two laws are built in:

- `linear_law` is `H = 1/2 x^T Q x` with constant `J`, `R`, `B`, `G`. Use it for transmission
  lines, linear circuits and other linear-quadratic components.
- `canonical_law` is the canonical mechanical form. The state is `x = (q, p)`, with
  `H = V(q) + 1/2 p . M(q)^-1 p` and `R = diag(0, D(q))`:
  - `V` is a quadratic, optionally plus a scaled MLP;
  - `M` and `D` are diagonal: constants, or `softplus(MLP(q)) + min`.

  An electrical DGU is `n = 1`: `q` is the capacitor charge and `p` the inductor flux, so the
  efforts are the capacitor voltage and the inductor current.

## Stepping

`step(s, opts, u, x)` advances one step of `opts.dt`, with `u` held over it. The schemes:

| `scheme` | what it does |
| --- | --- |
| `split` (default) | `substeps` repetitions of: coupling half-step, each block's local step, coupling half-step. A local step is kick, damp, conserve, damp, kick. The coupling half-step is `euler` or `midpoint` (`opts.coupling`). |
| `rk4` | classical RK4 on the whole composed field (a reference) |
| `implicit_midpoint` | implicit midpoint on the whole field, by Picard iteration (a reference) |

Notes on the split step:

- Euler coupling makes the step first order; midpoint coupling makes it second order.
- `canonical_law` conserves by implicit midpoint (Picard: at most `solve.max_iter` iterations,
  stopping when no coordinate moves by `solve.tol`). With a constant mass it can use leapfrog
  instead.

`rollout(s, opts, x0, u, steps)` returns the states, each step's `step_stats` (Picard
iterations, convergence) and its books.

## The books

Every step's energy accounting is a `books` record, the same `dyn_ph_books` the vehicle plants
keep. It holds the energy change `dH`, the external inputs' work `supplied` and the loss
`dissipated` (both by the trapezoid rule over the step), and the integrator's own `residual`:

```
dH = supplied - dissipated + residual
```

The residual is not zero for any of these schemes:

- per step it is O(dt^2) with Euler coupling and O(dt^3) with midpoint coupling;
- for a quadratic `H`, the whole-field implicit midpoint closes the midpoint-rule books
  (`step_books_midpoint`) to its solver tolerance.

`check_balance` tests the power balance at a state. `check_balance_random` tests it at random
states.

## States from physical quantities

`storage_from_efforts(s, e)` finds the state whose efforts are `e`, in the readout's order: a
DGU's voltage and current, a line's current. It works block by block, by Newton with a
central-difference Jacobian, capping each update at 2 in the infinity norm, for at most 30
iterations, to a residual of 1e-9. It reports whether it converged, the iterations and the
residual.

`assign_state(now, x_now, candidate, added)` gives a candidate its start. Each candidate block
named as a block of the actual system keeps that block's state. Every other block takes its state
from `added`, by name.

## DC microgrids

`cvc/ph/microgrid.h` builds DC networks from `composite` pieces:

- `linear_dgu(C, L, R)`, `nonlinear_dgu(charge, L, R)` and `reference_dgu()`. A DGU's state is
  its capacitor charge and inductor flux; its inputs are the current injected at its bus (minus
  its load) and its source voltage; its one port is its bus voltage. The reference DGU has
  `q = 2.2 (v + v^3 / 3)` C, `L = 1.8` H and `R = 1.2` ohm.
- `rl_line(R, L)`.
- `make_microgrid(dgu, buses, lines)` names the blocks `dgu0..` and `line0..` and wires them by
  the lines' incidence (`{from, to}`: from is the sending bus).
- `physical_state(g, x)`, `peak_line_current(g, x)`, and `compare_trajectories(g, a, b)`, which
  gives voltage, DGU-current and line-current RMSE and the peak line-current error.

**The candidate study** (`ph_microgrid_test`) runs 16 episodes:

1. A four-DGU chain of reference DGUs (lines 0.3 / 0.4 / 0.5 ohm, 0.8 H) starts from a sampled
   physical state: bus voltage 0.5-1.3 V, DGU current -0.25-0.35 A, line current -0.1-0.1 A,
   loads 0.05-0.2 A, sources 0.9-1.2 V.
2. It runs for one second.
3. The same DGU law and a 0.4 ohm / 0.8 H line are added at bus 0 or at bus 2, with the added
   blocks' states supplied.
4. Each candidate runs 200 steps of 0.01 s against RK4 at dt / 16.

The laws are known, so these errors are the transition's own. Averaged over the 32 trajectories:

| coupling | voltage RMSE | DGU current RMSE | line current RMSE | peak line current error | orderings by peak |
| --- | --- | --- | --- | --- | --- |
| Euler | 7.9e-5 V | 2.0e-5 A | 2.0e-4 A | 2.4e-4 A | 16 / 16 |
| midpoint | 5.1e-7 V | 7.1e-7 A | 2.9e-7 A | 5.7e-8 A | 16 / 16 |

The worst single-step energy residual is 1.2e-5 J with Euler coupling and 1.7e-7 J with
midpoint coupling.

## Queries

`cvc/ph/query.h` compares proposed changes before they are made, with every law fixed.

1. **The present state from a measured history.** `fit_history(s, h, o)` fits the readout
   (efforts) at the start of a window, `prior_mean + prior_scale xi`, to the measured channels.
   - It maps xi to a state through `storage_from_efforts` and runs the window's inputs through
     the transition.
   - The objective is the misfit weighted by `1 / max(noise, noise_floor)`, plus `|xi|^2`. It is
     minimized by Levenberg-Marquardt from `xi = 0`, with a central-difference Jacobian and at most
     `max_evals` residual evaluations.
   - It reports success, optimality (`|J^T r|_inf`), the data Jacobian's rank, and a Laplace
     posterior. `admitted(state_dim)` requires success, optimality `<= 1e-4` and full rank.
   - `posterior_states` draws states at the query time from that posterior.
2. **Each candidate's start:** `assign_state` (above).
3. **Consequences:** `consequences(s, rollout, u)` gives each step's readout, stored energy, loss
   and external powers, and the step's work, loss and residual.
4. **Cards and the ranking.**
   - `make_card` summarizes a candidate. Each line is a `max_abs`, `min`, `max` or `final_value`
     of some readout channels, or the `total_loss`, `total_work` or `max_residual` of its books.
   - `rank_candidates(cards, bounds, limits, objective)` admits a candidate when every limit holds
     with its error bound added (`g + dg <= 0`), and rejects it when one fails with the bound
     subtracted.
   - It scores `J = sum_k w_k value_k / scale_k` with bound `dJ`, and orders `a` before `b` only
     when `J(a) + dJ(a) < J(b) - dJ(b)`.
   - The outcome is `certified` (the best admitted candidate beats every one not rejected),
     `inconclusive`, or `abstain` (nothing admitted). A certified best is within `2 max dJ` of the
     admitted set's best.
   - `sampled_bounds` gives sampled bounds: an ensemble of starts and laws, plus the transition's
     own error against RK4. They are evidence for what was sampled, not a proof.

`microgrid.h` gives a network's channels (`voltage_channels`, `dgu_current_channels`,
`line_current_channels`) and the electrical studies' prior (`microgrid_prior`: 1 +- 0.4 V,
0.1 +- 0.3 A, 0 +- 0.2 A).

**Measured** (`ph_query_test`, the four-DGU chain of reference DGUs, one second of bus voltages
sampled every 0.05 s, the truth from RK4 at dt / 16):

| forecast from the fitted state | voltage RMSE | DGU current RMSE | line current RMSE |
| --- | --- | --- | --- |
| 0.5 s | 0.29 mV | 26 mA | 22 mA |
| 2 s | 0.74 mV | 18 mA | 15 mA |
| C-PHAST's Table 8, numerical history estimate, 0.5 s | 0.36 mV | 26 mA | 25 mA |
| the same, 2 s | 1.0 mV | 18 mA | 18 mA |

The paper's networks are five-DGU chains, stars and cycles, and its DGU laws are learned, so the
match is in magnitude, not to the digit. Further results:

- A three-DGU tree is fully determined by its voltages. A symmetric cycle (equal line decay
  rates) loses one rank, a current circulating round it, and its estimate is not admitted.
- In the candidate query (a fifth DGU at bus 0 or at bus 2, limit 0.5 A on the peak line current,
  then the lowest loss), estimating from voltages alone picked the physical best in 2 of 4
  episodes, because the candidates' losses differ by less than the hidden currents' error. It
  certified none.
- Measuring the full state, it picked the best in 4 of 4 and certified 1.
- No certificate was wrong.

## Training

`cvc/ph/train.h` fits a component law's parameters natively, with gradients from the tape in
`cvc/ph/ad.h`.

- **The tape.** A reverse-mode tape owned by the caller; one tape per thread when the gradient is
  split across threads. A `var` with no tape is a constant and records nothing.
- **The law over the tape.** `canonical_t<T>` is `canonical_law`'s arithmetic written over a
  scalar type. With `double` it is the runtime law's step to rounding. With `ad::var` one backward
  sweep differentiates the loss through the potential's and the mass's input derivatives and
  through the implicit midpoint's Picard iterations.
- **The parameters.** `trainable_parameters` and `with_parameters` flatten and restore a spec's
  trainable parameters: K's upper triangle, the centre, every net's weights and biases, and the
  constant mass and damping where there is no net. `v_scale` and the floors stay fixed.

The isolated-DGU calibration follows the electrical studies:

1. `make_dgu_dataset` simulates a reference DGU law (RK4 at dt / 16) from capacitor voltages in
   [-1.5, 1.5] V and fluxes in [-1, 1] Wb, under 0.1 A in and a 1 V source. Every sample is fully
   observed: `(q, phi, V_src, V_src - R i, v, -i)`.
2. `fit_linear_dgu` fits C, L and R by least squares; `linear_dgu_spec` is that fitted-linear
   control.
3. `warm_start_dgu` builds the trained family at those values:
   - `V(q) = 1/2 q^2 / C + v_net(q)`, with v_net's last layer at zero;
   - `M(q) = softplus(m_net(q)) + 0.01`;
   - `D(q) = softplus(d_net(q)) + 0.001`, each net's last bias set to give L and R.
4. `dgu_loss` gives the constraint loss (current balance, the resistive relation and the
   capacitor-node voltage at the measured state) and the one-step loss (the six channels after
   `LocalStep(dt, 2)`), weighted `1, 0.01, 1e-8 |theta|^2`, with their gradient.
5. `train_dgu` runs Adam over random batches, optionally after constraint-only updates (the
   staged schedule).
6. `evaluate_dgu` scores a law on held-out open-loop rollouts: capacitor-voltage and
   inductor-current RMSE.

`cvc ph train-dgu <out>` runs the whole calibration and writes the trained law;
`pycvc.ph_train_dgu(...)` returns it with its scores.

## Files

### `cvc-ph-law/1`

A linear law:

```json
{"format": "cvc-ph-law/1", "kind": "linear", "n": 1,
 "Q": [1.25], "J": [0], "R": [0.3], "ports": 1, "B": [1], "inputs": 0, "G": []}
```

- `Q`, `J` and `R` are `n x n`, `B` is `n x ports` and `G` is `n x inputs`. Each may be flat
  (row-major) or nested rows.
- `J` and `R` default to zero.

A canonical law:

```json
{"format": "cvc-ph-law/1", "kind": "canonical", "n": 1,
 "potential": {"K": [[0.4545]], "center": [0], "scale": 0.1,
               "net": [{"W": [[...], ...], "b": [...], "act": "silu"}, ...]},
 "mass": {"diag": [1.8]},
 "damping": {"net": [...], "min": 0.05},
 "ports": 1, "B": [[1], [0]], "inputs": 2, "G": [[1, 0], [0, 1]],
 "scheme": "implicit_midpoint"}
```

- `potential` is `1/2 (q - center) . K (q - center) + scale * net(q) + sum_i C(q_i)`. Any part
  may be left out. `capacitor` gives `C`, a capacitor curve: the charge as an odd increasing
  polynomial of the voltage, `q = sum_j capacitor[j] v^j` (`capacitor[1] > 0`, odd entries
  `>= 0`, even entries 0). Its energy has `dC/dq = v`, and `v` is found from `q` by Newton.
  `[0, c]` is a linear capacitor `c`.
- `mass` is `diag` (positive constants) or `net` plus `min` (with `min > 0`).
- `damping` is `diag` or `net` plus `min`. Leaving it out means no damping.
- A net is a list of dense layers. `W` is `out x in`. `act` is one of `identity`, `silu`,
  `tanh`, `softplus`, `sigmoid`, and applies to that layer's output.
- `B` and `G` have `2n` rows: `q_1..q_n`, then `p_1..p_n`.
- `scheme` is `implicit_midpoint` (the default) or `leapfrog` (constant mass only).

### `cvc-ph-system/1`

```json
{"format": "cvc-ph-system/1",
 "laws": {"dgu": {...a law...}, "line": "line.json"},
 "blocks": [{"name": "dgu0", "law": "dgu"}, {"name": "dgu1", "law": "dgu"},
            {"name": "l01", "law": "line"}],
 "couplings": [
   {"type": "incidence", "buses": ["dgu0", "dgu1"], "bus_port": 0,
    "lines": ["l01"], "line_port": 0, "A": [[-1], [1]]},
   {"type": "pair", "a": "dgu0", "a_port": 0, "b": "dgu1", "b_port": 0, "c": 0.5},
   {"type": "skew", "blocks": ["dgu0", "dgu1"], "port": 0, "C": [[0, 1], [-1, 0]]}]}
```

`laws` maps a name to a law, either inline or as a path relative to the system file. Blocks are
stacked in the order given. The coupling types:

| `type` | effect |
| --- | --- |
| `pair` | `C(a.a_port, b.b_port) += c` and `C(b.b_port, a.a_port) -= c` |
| `incidence` | An electrical network. `A` is buses x lines, with -1 at a line's sending bus and +1 at its receiving bus. It gives `dq_bus += A i_line` and `dphi_line -= A^T v_bus`. |
| `skew` | A dense rule over repeated blocks, using the skew part `(C - C^T)/2` of the matrix given |

A block never pairs with itself (`C_ii = 0`).

`law_json(law)` and `system_json(s)` write these formats. Every number is written to 17 digits,
each shared law once, and the pairing as `pair` couplings, so a file read back gives the same
composite.

## Command line

```
cvc ph info grid.json
cvc ph check grid.json --samples 200
cvc ph simulate grid.json --x0 2.2,0,2.6,0,0 --u -0.1,1,-0.15,1.1 --steps 200 \
    [--dt 0.01] [--scheme split|rk4|implicit_midpoint] [--coupling midpoint|euler] \
    [--substeps 2] [--local-substeps 1] [--max-iter 10] [--tol 1e-6] [--csv out.csv]
```

```
cvc ph microgrid grid.json --dgus 4 --edges 0-1,1-2,2-3 --line-r 0.3,0.4,0.5 --line-l 0.8 \
    [--dgu reference|linear] [--dgu-c 2.2] [--dgu-l 1.8] [--dgu-r 1.2]
cvc ph simulate grid.json --physical 1,0,1.1,0,0.9,0,1,0,0,0,0 --u -0.1,1,-0.1,1,-0.1,1,-0.1,1
cvc ph train-dgu dgu.json [--updates 100000] [--constraint-first 25000] [--batch 128] [--lr 1e-4] \
    [--threads 12] [--train-runs 32] [--transitions 200] [--test-runs 16] [--test-transitions 500] \
    [--v-hidden 32,32] [--m-hidden 32,32] [--d-hidden 32] [--data-seed 1] [--init-seed 7] [--log-every N]
```

- `check` exits 1 when the pairing does work or `dH/dt` departs from `-loss + supplied`.
- `microgrid` writes a network of one DGU law on every bus. `--edges` defaults to a chain. Line
  resistances and inductances take one value or one per line; the last repeats.
- `--physical` starts from every block's efforts instead of `--x0`: a DGU's voltage and current,
  a line's current. They are mapped to the state by `storage_from_efforts`.
- `--u` takes one row of inputs, held over the rollout, or one row per step.
- `--csv` writes `t`, every coordinate (`<block>.<i>`), `H`, then the step's `supplied`,
  `dissipated` and `residual`.

## Python

```python
import pycvc
g = pycvc.ph_composite("grid.json")        # or the JSON text itself
r = g.simulate(x0, u, steps=200, dt=0.01, coupling="midpoint")
r["x"]                                      # (steps + 1, state_dim) float64
r["residual"]                               # per step
g.check(200)                                # {"rate_error", "internal_error", "skew_error", ...}
g.energy(x), g.effort(x), g.field(x, u), g.powers(x, u), g.pairing(), g.block(i)
g.storage(e)                                # {"x", "converged", "iterations", "residual"}
g.system_json()                             # the system as cvc-ph-system/1 text
pycvc.ph_microgrid_json(4, edges=[(0, 1), (1, 2), (2, 3)], line_r=[0.3, 0.4, 0.5], line_l=[0.8])
f = g.fit_history(inputs, sample_steps, channels, samples, prior_mean, prior_scale, noise=0.0)
f["x_now"], f["admitted"], f["rank"]
card = g.card(f["x_now"], u, 200, [("peak", "max_abs", [6, 7]), ("loss", "total_loss", [])])
pycvc.ph_rank([card_a, card_b], [bounds_a, bounds_b], [("peak", 0.5)], {"loss": 1.0})
# {"best", "outcome": certified | inconclusive | abstain, "feasible", "order", ...}
t = pycvc.ph_train_dgu(updates=2000, threads=8)   # {"law", "linear", "trained", "fit", "history", ...}
```

## Parity with the reference implementation

`src/cvc/tools/cphast_fixtures.py` runs CVC-Lab/phast's own torch modules in float64. The fixtures
are written to `src/cvc/tests/data/ph_compose_golden.json`:

- four linear DGUs and three RL lines on a chain;
- the same network with a fifth DGU added at bus 0 or at bus 2;
- three copies of a learned DGU law: an MLP potential, an MLP mass, an MLP damping, implicit
  midpoint.

`ph_compose_parity_test` replays them to rounding: the largest state error over 200 steps is 3.6e-15
with known laws and 6.7e-15 with the learned ones. The script's docstring lists two deliberate
departures from phast's stock microgrid factory:

- a line's dummy potential is exactly zero (in the factory it is `ln 2`);
- learned DGUs couple through their own efforts.
