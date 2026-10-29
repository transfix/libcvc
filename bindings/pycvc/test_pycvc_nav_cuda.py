#!/usr/bin/env python3
"""pycvc GPU drive: nav_drive_step_material_cuda against the CPU drive, from Python.

Checks:
  1. with a material stack, the GPU drive moves every agent as nav_drive_step_material does
     (float-equivalent, 1e-5);
  2. telemetry=1 returns drive_telemetry as a dict of (N,) arrays, and the fixed lam columns
     are the weights it reports;
  3. a terrain alone (no material) drives: its weights are the reported lams, its grade cost the
     reported risk, and it moves the agents;
  4. a slope raster runs the governors;
  5. the risk governor caps the speed as the CPU drive does, its cap the reported v_risk;
  6. what the drive cannot apply is an error: a field of more than one plane, a material stack
     with no lam column for a net with no head for it, a malformed slope.
Script-style, like the other pycvc tests; self-skips (exit 0) when the running pycvc predates the
binding, was built without CUDA, or has no CUDA device.
"""

import math
import os

import numpy as np
import pycvc

if not hasattr(pycvc, "nav_drive_step_material_cuda"):
    print("pycvc lacks nav_drive_step_material_cuda — skipping")
    raise SystemExit(0)
if not pycvc.nav_drive_cuda_available():
    print("pycvc has no CUDA drive or no CUDA device — skipping")
    raise SystemExit(0)

WEIGHTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, "share",
                       "cvc", "nav", "coef_mlp.cvcnav")
N_CELLS, LO, HI, SCALE = 65, -100.0, 100.0, 0.05
FRAME = (LO, LO, HI, HI, 0.0, 0.0, SCALE)
VEH = (0.15, 0.35, 0.06, 0.9, 0.035, 0.6, 1.5, 1.0, 0.8, 2, 1)  # rr .. k_steer, nsub, reverse

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


def expect_raises(fn, what):
    try:
        fn()
    except Exception:  # noqa: BLE001 — the binding raises ValueError / RuntimeError
        global checks
        checks += 1
        return
    raise AssertionError(what + ": no error")


xs = np.linspace(LO, HI, N_CELLS)
X, Y = np.meshgrid(xs, xs)  # rows are y, columns x
# A circular building at (40, 10) m, radius 15 m.
dx, dy = X - 40.0, Y - 10.0
d = np.sqrt(dx * dx + dy * dy) + 1e-9
field = np.stack([(d - 15.0) * SCALE, dx / d, dy / d])[None].astype(np.float32)
# Material planes: risk in [0, 1], phi in metres, grad r~ per normalized unit, grad phi per metre.
ax, ay = X / 30.0, Y / 40.0
arg = X / 25.0 - Y / 35.0
mat = np.stack([
    0.5 + 0.4 * np.sin(ax) * np.cos(ay),
    10.0 + 7.0 * np.cos(arg),
    0.4 * np.cos(ax) * np.cos(ay) / 30.0 / SCALE,
    -0.4 * np.sin(ax) * np.sin(ay) / 40.0 / SCALE,
    -7.0 * np.sin(arg) / 25.0,
    7.0 * np.sin(arg) / 35.0,
])[None].astype(np.float32)
# The terrain: a grade-cost bump with its infeasible core.
bx, by = (X + 20.0) / 45.0, (Y - 15.0) / 45.0
g = np.exp(-(bx * bx + by * by))
terrain = np.stack([
    0.9 * g, 6.0 + 14.0 * (1.0 - g), -0.9 * g * 2.0 * bx / 45.0 / SCALE,
    -0.9 * g * 2.0 * by / 45.0 / SCALE, 14.0 * g * 2.0 * bx / 45.0, 14.0 * g * 2.0 * by / 45.0
])[None].astype(np.float32)
hx, hy = X + 30.0, Y - 30.0
z = 6.0 * np.exp(-(hx * hx + hy * hy) / 2500.0)
slope = np.stack([-2.0 * hx / 2500.0 * z, -2.0 * hy / 2500.0 * z]).astype(np.float32)

rng = np.random.default_rng(5)
pts = []
while len(pts) < 64:
    p = rng.uniform([-3.0, -2.5], [3.0, 2.5])
    if math.hypot(p[0] - 2.0, p[1] - 0.5) > 1.1:  # drivable space, outside the building
        pts.append(p)
on = np.array(pts, np.float32)
N = len(on)
th = rng.uniform(-math.pi, math.pi, N).astype(np.float32)
sp = rng.uniform(-0.1, 0.8, N).astype(np.float32)
a = rng.uniform(-math.pi, math.pi, N)
r = rng.uniform(0.2, 1.8, N)
carrot = (on + np.stack([r * np.cos(a), r * np.sin(a)], 1)).astype(np.float32)
lam_soft = np.linspace(0.4, 1.0, N, dtype=np.float32)
lam_hard = np.linspace(0.8, 2.0, N, dtype=np.float32)


def gpu(mat_field=mat, ls=lam_soft, lh=lam_hard, fld=field, **kw):
    return pycvc.nav_drive_step_material_cuda(fld, on, th, sp, carrot, WEIGHTS, mat_field, ls, lh,
                                              1.25, 12.0, *FRAME, *VEH, **kw)


# 1. the material drive, GPU against CPU.
cpu = pycvc.nav_drive_step_material(field, on, th, sp, carrot, WEIGHTS, mat, lam_soft, lam_hard,
                                    1.25, 12.0, None, *FRAME, *VEH, 1)
out = gpu()
check(len(out) == 4, "without telemetry the result is (o, th, sp, minclr)")
for c, gg, name in zip(cpu, out, ("o", "th", "sp", "minclr")):
    err = float(np.max(np.abs(c - gg)))
    check(err < 1e-5, f"GPU {name} differs from the CPU drive by {err:.3e}")
check(float(np.max(np.abs(out[0] - on))) > 1e-4, "the drive moved the agents")

# 2. telemetry.
o2, th2, sp2, mc2, tel = gpu(telemetry=1)
check(np.array_equal(o2, out[0]), "asking for telemetry does not change the drive")
keys = {"alpha", "beta", "gamma", "lam_soft", "lam_hard", "mu", "mrisk", "v_risk", "ext_fx",
        "ext_fy", "steer", "curvature", "clearance", "binding", "potential", "u_goal", "u_barrier",
        "u_soft", "u_hard", "u_terrain", "force_x", "force_y", "potential_end", "projection_work",
        "context_work"}
check(set(tel) == keys, f"telemetry keys {sorted(set(tel) ^ keys)}")
check(all(np.all(tel[k] == 0) for k in ("potential_end", "projection_work", "context_work")),
      "the tick's books are a sim_world's: the bare drive step leaves them zero")
check(all(v.shape == (N,) for v in tel.values()), "every telemetry array is (N,)")
check(tel["binding"].dtype == np.uint8, "binding is uint8")
check(np.array_equal(tel["lam_soft"], lam_soft), "a net with no lam head reports the fixed lam_soft")
check(np.array_equal(tel["lam_hard"], lam_hard), "and the fixed lam_hard")
check(bool(np.all((tel["mrisk"] >= 0.0) & (tel["mrisk"] <= 1.0))), "mrisk is a risk in [0, 1]")
check(np.allclose(tel["potential"], tel["u_goal"] + tel["u_barrier"] + tel["u_soft"] +
                  tel["u_hard"] + tel["u_terrain"], rtol=1e-5, atol=1e-6),
      "the potential is the sum of its terms")
check(np.array_equal(tel["clearance"], mc2), "clearance is the tick's min clearance")

# 3. a terrain alone: terrain risk is material risk.
plain = gpu(mat_field=None, ls=None, lh=None, telemetry=1)
ter = gpu(mat_field=None, ls=None, lh=None, terrain=terrain, ter_lam_soft=0.5, ter_lam_hard=1.0,
          telemetry=1)
check(bool(np.all(plain[4]["lam_soft"] == 0.0)), "no material and no terrain: lam_soft 0")
check(bool(np.all(ter[4]["lam_soft"] == 0.5)), "the terrain's lam_soft is the reported weight")
check(bool(np.all(ter[4]["lam_hard"] == 1.0)), "and its lam_hard")
check(float(ter[4]["mrisk"].max()) > 0.05, "the terrain's grade cost is the reported risk")
check(float(np.abs(ter[4]["u_terrain"]).max()) > 0.0, "the terrain's potential is reported")
check(float(np.max(np.abs(ter[0] - plain[0]))) > 1e-6, "the terrain's force moves the agents")

# 4. the slope governors.
sl = gpu(slope=slope, gravity=9.80665 * SCALE)
check(all(np.all(np.isfinite(x)) for x in sl), "a slope runs")

# 5. the risk governor: GPU against CPU, its cap the reported v_risk, and slower than without it.
GOV = dict(risk_governor=1, risk_vmin=0.1, risk_onset=0.3, risk_lookahead=0.6, risk_probes=3)
cpu_gov = pycvc.nav_drive_step_material(field, on, th, sp, carrot, WEIGHTS, mat, lam_soft,
                                        lam_hard, 1.25, 12.0, None, *FRAME, *VEH, 1, *GOV.values())
gov = gpu(telemetry=1, **GOV)
for c, gg, name in zip(cpu_gov, gov, ("o", "th", "sp", "minclr")):
    err = float(np.max(np.abs(c - gg)))
    check(err < 1e-5, f"governed GPU {name} differs from the CPU drive by {err:.3e}")
vr = gov[4]["v_risk"]
check(bool(np.all(gov[2] <= np.maximum(vr, sp) + 1e-6)), "no agent is sped past its v_risk")
check(bool(np.all((sp <= vr) | (gov[2] < sp))), "and one over it slows (a tick brakes only so much)")
check(bool(np.all(tel["v_risk"] == 0.0)), "v_risk is 0 with the governor off")
check(float(np.abs(gov[2]).sum()) < float(np.abs(out[2]).sum()) - 1e-3, "the governor slows them")

# 6. refusals.
expect_raises(lambda: gpu(fld=np.concatenate([field, field])), "a two-plane field")
expect_raises(lambda: gpu(ls=None), "no lam_soft column and no head for it")
expect_raises(lambda: gpu(slope=slope[:1]), "a one-plane slope")

print(f"test_pycvc_nav_cuda: {checks} checks passed")
