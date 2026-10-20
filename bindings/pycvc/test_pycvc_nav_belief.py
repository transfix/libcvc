#!/usr/bin/env python3
"""pycvc belief (observed) material bindings — golden 1 through the Python surface.

Reproduces libcvc's ``src/cvc/tests/data/belief_material_golden.json`` (NavBeliefMaterial.Golden)
with only pycvc calls: ``nav_sense_batch_material`` for the sense and the material mask,
``nav_seen_disc`` for the disc rule, ``nav_composite_occupancy`` for the planning raster,
``nav_belief_material_inputs``, ``nav_material_build``, ``nav_belief_gate_surface`` and
``nav_witness_gate``. Every hash and gate output must match. Also checks the bindings' argument
validation. Script-style, like the other pycvc tests; self-skips (exit 0) when the running pycvc
predates the bindings.
"""

import hashlib
import json
import math
import os

import numpy as np
import pycvc

if not hasattr(pycvc, "nav_belief_material_inputs"):
    print("pycvc lacks nav_belief_material_inputs — skipping")
    raise SystemExit(0)

HERE = os.path.dirname(os.path.abspath(__file__))
GOLDEN = os.path.join(HERE, "..", "..", "src", "cvc", "tests", "data", "belief_material_golden.json")
g = json.load(open(GOLDEN))
R, C = g["grid"]["rows"], g["grid"]["cols"]
b = g["grid"]["bounds"]
truth = np.array(g["truth"], np.uint8).reshape(R, C)
risk_raw = np.array(g["risk_raw"], np.float32).reshape(R, C)
hard = np.array(g["hard"], np.uint8).reshape(R, C)
amap = np.array(g["agent_map"], np.int32)
N, P = len(amap), int(amap.max()) + 1
sn, oc, mb, gp = g["sense"], g["occupancy"], g["material_build"], g["gate"]
cell_w = (b[2] - b[0]) / (C - 1)


def h(a, dt):
    return hashlib.sha256(np.ascontiguousarray(np.asarray(a).astype(dt)).tobytes()).hexdigest()


def same(golden, now):
    if isinstance(golden, str):
        return (golden == "nan" and math.isnan(now)) or (
            golden in ("inf", "-inf") and now == float(golden)
        )
    return golden == now


fails = 0
for case in g["cases"]:
    p = case["params"]
    lo = np.zeros((P, R, C), np.float32)
    lv = np.zeros((P, R, C), bool)
    es = np.zeros((P, R, C), bool)
    ms = np.zeros((P, R, C), bool)
    ver = np.zeros(P, np.int32)
    rng = np.full(N, sn["range_m"])
    mrange = np.full(N, p["material_range_m"])
    for t, tk in enumerate(case["ticks"]):
        pos = np.array([[q[0], q[1]] for q in g["poses"][t]], np.float64)
        head = np.array([q[2] for q in g["poses"][t]], np.float64)
        args = (truth, pos, head, rng, np.full(N, sn["n_rays"], np.int32),
                np.full(N, sn["fov_rad"]), lo, lv, es, ver, amap, None, None)
        if p["seen_rule"] == "raycast":
            flips, changed = pycvc.nav_sense_batch_material(
                *args, ms, mrange, b[0], b[1], b[2], b[3], sn["l_occ"], sn["l_free"],
                sn["l_clamp"], 1)
            assert changed.shape == (P,) and changed.dtype == np.uint8
        else:
            pycvc.nav_sense_batch(*args, b[0], b[1], b[2], b[3], sn["l_occ"], sn["l_free"],
                                  sn["l_clamp"], 1)
            rad = p["material_range_m"] if p["material_range_m"] > 0 else sn["range_m"]
            for i in range(N):
                pycvc.nav_seen_disc(ms[amap[i]], pos[i, 0], pos[i, 1], rad, b[0], b[1], b[2], b[3])
        planes = []
        for m in range(P):
            mask = ms[m].astype(np.uint8)
            occ = pycvc.nav_composite_occupancy(lo[m], oc["policy"], oc["p_thresh"], oc["band"],
                                                None, 0.0, 0.0)
            risk_b, hard_b = pycvc.nav_belief_material_inputs(risk_raw, hard, mask,
                                                              p["unknown_risk"], p["unknown_hard"])
            six = np.asarray(pycvc.nav_material_build(risk_b, hard_b, mb["cell_w"], mb["scale"],
                                                      mb["sigma"]))
            gh, cl = pycvc.nav_belief_gate_surface(hard_b, occ, mask, cell_w,
                                                   p["unknown_gate_hard"])
            got = {"material_seen": h(mask, np.uint8), "occ": h(occ, np.uint8),
                   "risk_b": h(risk_b, np.float32), "hard_b": h(hard_b, np.uint8)}
            for k, name in enumerate(["risk", "phi_m", "grad_rx", "grad_ry", "grad_px",
                                      "grad_py"]):
                got[name] = h(six[k], np.float32)
            got["gate_hard"] = h(gh, np.uint8)
            got["clear_m"] = h(cl, np.float32)
            for k, v in got.items():
                if tk["planes"][m][k] != v:
                    fails += 1
                    print(f"MISMATCH {case['name']} t{t} plane {m} {k}")
            planes.append((six[0].astype(np.float32), gh, cl))
        s = (C - 1) / (b[2] - b[0])
        for i in range(N):
            rk, gh, cl = planes[amap[i]]
            gx, gy = g["goals"][i]
            out = np.asarray(pycvc.nav_witness_gate(
                rk, gh, cl, (pos[i, 1] - b[1]) * s, (pos[i, 0] - b[0]) * s, (gy - b[1]) * s,
                (gx - b[0]) * s, gp["horizon_cells"], gp["hard_margin_m"], gp["primitive_count"],
                gp["improvement_margin"], gp["material_trigger"], gp["progress_slack_cells"]))
            for key, val in (("active", int(out[0])), ("nominal", float(out[1])),
                             ("best", float(out[2])), ("count", int(out[3]))):
                if not same(tk["gate"][key][i], val):
                    fails += 1
                    print(f"MISMATCH {case['name']} t{t} agent {i} {key}: "
                          f"{tk['gate'][key][i]} vs {val}")

# Argument validation: a bad pin, a shape mismatch, a missing mask.
z = np.zeros((4, 4), np.float32)
u = np.zeros((4, 4), np.uint8)
for bad in ((z, u, u, 1.5, 0), (z, u, u, 0.0, 2), (z, u, np.zeros((3, 4), np.uint8), 0.0, 0)):
    try:
        pycvc.nav_belief_material_inputs(*bad)
    except RuntimeError:
        pass
    else:
        fails += 1
        print("MISMATCH nav_belief_material_inputs accepted", [np.shape(a) for a in bad[:3]],
              bad[3:])
try:
    pycvc.nav_seen_disc(np.zeros((4, 4), np.uint8), 0.0, 0.0, 1.0, 0.0, 0.0, 3.0, 3.0)
except RuntimeError:
    pass
else:
    fails += 1
    print("MISMATCH nav_seen_disc accepted a uint8 mask (it must be numpy-bool, written in place)")

if fails:
    raise SystemExit(f"{fails} mismatches")
print("belief material bindings reproduce golden 1")
