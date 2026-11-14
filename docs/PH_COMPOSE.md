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

- `potential` is `1/2 (q - center) . K (q - center) + scale * net(q)`. Either part may be
  left out.
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

## Command line

```
cvc ph info grid.json
cvc ph check grid.json --samples 200
cvc ph simulate grid.json --x0 2.2,0,2.6,0,0 --u -0.1,1,-0.15,1.1 --steps 200 \
    [--dt 0.01] [--scheme split|rk4|implicit_midpoint] [--coupling midpoint|euler] \
    [--substeps 2] [--local-substeps 1] [--max-iter 10] [--tol 1e-6] [--csv out.csv]
```

- `check` exits 1 when the pairing does work or `dH/dt` departs from `-loss + supplied`.
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
