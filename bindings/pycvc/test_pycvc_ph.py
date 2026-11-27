#!/usr/bin/env python3
"""pycvc's composed port-Hamiltonian systems (cvc/ph/compose.h, pycvc_ph.i).

Checks:
  1. a two-bus DC grid (two DGUs and the RL line between them) loads from cvc-ph-system/1 text and
     reports its blocks, offsets and skew pairing;
  2. the power balance at random states: the pairing does no work and dH/dt = -loss + supplied;
  3. the efforts are the energy's gradient (central differences), and the field's power is the
     ledger's;
  4. a rollout returns its states as a (steps + 1, n) array and closes its books: per step,
     dH = supplied - dissipated + residual, with the residual small (O(dt^3) with midpoint
     coupling) and smaller than Euler coupling's;
  5. rewiring the same laws (the line to the other bus) changes the future;
  6. a DC microgrid of nonlinear DGUs: ph_microgrid_json, system_json read back, storage()
     from voltages and currents, and its books;
  7. a query: fit_history from bus voltages alone, a candidate's card, and ph_rank's certified,
     inconclusive and abstaining outcomes;
  8. a short native training run of a DGU law (ph_train_dgu);
  9. errors: a wrong-sized state, an unknown scheme, a malformed file.
Script-style, like the other pycvc tests; self-skips (exit 0) when the running pycvc predates the
bindings.
"""

import json

import numpy as np
import pycvc

if not hasattr(pycvc, "ph_composite"):
    print("pycvc lacks ph_composite — skipping")
    raise SystemExit(0)

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


DGU = {"format": "cvc-ph-law/1", "kind": "canonical", "n": 1, "potential": {"K": [[1 / 2.2]]},
       "mass": {"diag": [1.8]}, "damping": {"diag": [1.2]}, "ports": 1, "B": [[1], [0]],
       "inputs": 2, "G": [[1, 0], [0, 1]], "scheme": "leapfrog"}
LINE = {"format": "cvc-ph-law/1", "kind": "linear", "n": 1, "Q": [1 / 0.8], "R": [0.3], "ports": 1, "B": [1]}


def grid(A):
    return json.dumps({"format": "cvc-ph-system/1", "laws": {"dgu": DGU, "line": LINE},
                       "blocks": [{"name": "dgu0", "law": "dgu"}, {"name": "dgu1", "law": "dgu"},
                                  {"name": "dgu2", "law": "dgu"}, {"name": "l01", "law": "line"}],
                       "couplings": [{"type": "incidence", "buses": ["dgu0", "dgu1", "dgu2"],
                                      "lines": ["l01"], "A": A}]})


g = pycvc.ph_composite(grid([[-1], [1], [0]]))
check(g.blocks() == 4 and g.state_dim() == 7 and g.input_dim() == 6 and g.port_dim() == 4, "dims")
b = g.block(3)
check(b["name"] == "l01" and b["kind"] == "linear" and b["state_offset"] == 6, f"block 3: {b}")
C = np.array(g.pairing()).reshape(4, 4)
check(np.array_equal(C, -C.T), "the pairing is skew")
check(C[0, 3] == 1.0 and C[1, 3] == -1.0 and C[2, 3] == 0.0, f"incidence pairing: {C}")
check("pair dgu0.0 - l01.0: 1" in g.describe(), g.describe())

# 2. The power balance.
c = g.check(200, 3)
check(c["samples"] == 200 and c["rate_error"] < 1e-13 and c["internal_error"] < 1e-15 and c["skew_error"] == 0,
      f"balance: {c}")

# 3. Efforts, field and powers.
rng = np.random.default_rng(5)
x = rng.normal(size=7)
u = rng.normal(size=6)
e = np.array(g.effort(x))
h = 1e-6
fd = np.array([(g.energy(x + h * np.eye(7)[i]) - g.energy(x - h * np.eye(7)[i])) / (2 * h) for i in range(7)])
check(np.allclose(e, fd, rtol=1e-7, atol=1e-8), f"efforts {e} vs {fd}")
p = g.powers(x, u)
rate = float(e @ np.array(g.field(x, u)))
check(abs(rate - (p["external"] - p["loss"])) < 1e-12 * max(1.0, abs(rate)), f"rate {rate} vs {p}")
check(abs(p["internal"]) < 1e-15 and p["loss"] >= 0, f"powers {p}")

# 4. A rollout and its books.
x0 = [2.2 * 1.1, 1.8 * 0.1, 2.2 * 0.9, 0.0, 2.2 * 1.2, -0.05, 0.8 * 0.05]
u0 = [-0.1, 1.0, -0.15, 1.1, -0.12, 0.95]
r = g.simulate(x0, u0, steps=200, dt=0.01)
X = r["x"]
check(isinstance(X, np.ndarray) and X.shape == (201, 7) and X.dtype == np.float64, f"x: {type(X)} {X.shape}")
check(np.array_equal(X[0], np.array(x0)), "the rollout starts at x0")
dH = np.array(r["dH"])
books = np.array(r["supplied"]) - np.array(r["dissipated"]) + np.array(r["residual"])
check(np.allclose(dH, books, rtol=0, atol=1e-15), "dH = supplied - dissipated + residual")
check(np.allclose(np.diff(r["H"]), dH, rtol=0, atol=1e-14), "H follows dH")
mid = np.abs(r["residual"]).max()
eul = np.abs(g.simulate(x0, u0, steps=200, dt=0.01, coupling="euler")["residual"]).max()
check(mid < 1e-6 and mid < eul, f"residuals: midpoint {mid}, euler {eul}")
check(r["converged"] and "200 steps of 0.01 s" in r["report"], r["report"])
held = g.simulate(x0, u0 * 200, steps=200)  # one row per step, all equal
check(np.array_equal(held["x"], X), "per-step inputs equal to the held row give the same rollout")

# 5. The same laws, the line to bus 2 instead of bus 1.
g2 = pycvc.ph_composite(grid([[-1], [0], [1]]))
X2 = g2.simulate(x0, u0, steps=200)["x"]
check(np.abs(X2[-1] - X[-1]).max() > 1e-3, "rewiring changes the future")

# 6. A DC microgrid with the nonlinear reference DGU: built, written out and read back, and
#    started from a physical state (voltages and currents) through storage().
if hasattr(pycvc, "ph_microgrid_json"):
    text = pycvc.ph_microgrid_json(4, edges=[(0, 1), (1, 2), (2, 3)], line_r=[0.3, 0.4, 0.5], line_l=[0.8])
    mg = pycvc.ph_composite(text)
    check(mg.blocks() == 7 and mg.state_dim() == 11 and mg.input_dim() == 8, "microgrid dims")
    check(mg.check(100)["rate_error"] < 1e-12, "the nonlinear microgrid balances its power")
    again = pycvc.ph_composite(mg.system_json())
    check(again.pairing() == mg.pairing(), "system_json round trip: pairing")
    phys = [1.1, 0.1, 0.9, -0.05, 1.2, 0.2, 0.7, 0.0, 0.05, -0.02, 0.01]
    st = mg.storage(phys)
    check(st["converged"] and st["residual"] < 1e-9, f"storage: {st}")
    check(np.allclose(mg.effort(st["x"]), phys, rtol=0, atol=1e-9), "storage inverts the efforts")
    check(abs(again.energy(st["x"]) - mg.energy(st["x"])) == 0, "round trip: energy")
    r = mg.simulate(st["x"], [-0.1, 1.0] * 4, steps=50)
    check(r["converged"] and np.abs(r["residual"]).max() < 1e-6, f"microgrid books: {np.abs(r['residual']).max()}")
    lin = pycvc.ph_composite(pycvc.ph_microgrid_json(2, dgu="linear", dgu_c=1.5))
    check(lin.block(0)["kind"] == "canonical" and lin.state_dim() == 5, "a linear-DGU microgrid")
    try:
        pycvc.ph_microgrid_json(2, dgu="flux")
    except (RuntimeError, ValueError):
        checks += 1
    else:
        raise AssertionError("an unknown DGU kind did not raise")

# 7. A query: the state from bus voltages alone, a candidate's card, and the certified ranking.
if hasattr(pycvc.ph_composite, "fit_history") and hasattr(pycvc, "ph_rank"):
    grid3 = pycvc.ph_composite(pycvc.ph_microgrid_json(3, edges=[(0, 1), (1, 2)], line_r=[0.3, 0.5], line_l=[0.8]))
    truth_e = [1.1, 0.1, 0.9, -0.05, 1.2, 0.2, 0.05, -0.02]
    xt = grid3.storage(truth_e)["x"]
    u3 = [-0.1, 1.0, -0.12, 1.05, -0.08, 0.95]
    run = grid3.simulate(xt, u3, steps=100, dt=0.01)["x"]
    volts = [0, 2, 4]  # readout indices of the bus voltages
    steps = list(range(0, 101, 5))
    samples = [[grid3.effort(run[k])[c] for c in volts] for k in steps]
    mean = [1.0, 0.1, 1.0, 0.1, 1.0, 0.1, 0.0, 0.0]
    scale = [0.4, 0.3, 0.4, 0.3, 0.4, 0.3, 0.2, 0.2]
    fit = grid3.fit_history([u3] * 100, steps, volts, samples, mean, scale)
    check(fit["admitted"] and fit["rank"] == 8 and fit["evals"] <= 20, f"fit: {fit}")
    e_now, e_true = grid3.effort(fit["x_now"]), grid3.effort(run[100])
    check(max(abs(e_now[c] - e_true[c]) for c in volts) < 1e-3, "the fit pins the bus voltages")
    card = grid3.card(fit["x_now"], u3, 100, [("peak", "max_abs", [6, 7]), ("vmin", "min", volts),
                                             ("loss", "total_loss", [])])
    check(set(card) == {"peak", "vmin", "loss"} and card["loss"] > 0, f"card: {card}")
    r = pycvc.ph_rank([{"peak": 0.5, "loss": 1.0}, {"peak": 0.6, "loss": 2.0}, {"peak": 1.5, "loss": 0.1}],
                      [{"peak": 0.1, "loss": 0.1}] * 3, [("peak", 1.0)], {"loss": 1.0})
    check(r["best"] == 0 and r["outcome"] == "certified" and r["feasible"] == ["admitted", "admitted", "rejected"]
          and r["order"][0][1] == 1, f"ranking: {r}")
    r = pycvc.ph_rank([{"peak": 2.0, "loss": 1.0}], [{}], [("peak", 1.0)], {"loss": 1.0})
    check(r["outcome"] == "abstain" and r["best"] is None, f"abstain: {r}")

# 8. Training a DGU law natively, a short run: it beats the fitted-linear control on held-out rollouts
#    and its law reads back as a ph_composite block.
if hasattr(pycvc, "ph_train_dgu"):
    # ph_train_test's short run (8 runs of 100 transitions: fewer, and the nets extrapolate badly to
    # held-out starts outside what they saw).
    t = pycvc.ph_train_dgu(updates=400, batch=64, lr=3e-3, train_runs=8, transitions=100, test_runs=4,
                           test_transitions=200, data_seed=21, v_hidden=[16, 16], m_hidden=[8], d_hidden=[8],
                           threads=4, log_every=100)
    check(t["trained"]["finite"] and t["trained"]["voltage_rmse"] < t["linear"]["voltage_rmse"],
          f"train: linear {t['linear']}, trained {t['trained']}")
    check(len(t["history"]) == 4 and t["history"][-1]["update"] == 400, f"history: {t['history']}")
    law = json.loads(t["law"])
    check(law["kind"] == "canonical" and t["parameters"] > 100, f"law: {law.get('kind')}, {t['parameters']}")

# 9. Errors.
for bad, what in ((lambda: g.energy([1.0, 2.0]), "a wrong-sized state"),
                  (lambda: g.simulate(x0, u0, coupling="trapezoid"), "an unknown coupling"),
                  (lambda: pycvc.ph_composite('{"format": "cvc-ph-system/9"}'), "a wrong format"),
                  (lambda: pycvc.ph_composite("/nonexistent/grid.json"), "a missing file")):
    try:
        bad()
    except (RuntimeError, ValueError):
        checks += 1
    else:
        raise AssertionError(f"{what} did not raise")

print(f"pycvc ph: {checks} checks passed")
