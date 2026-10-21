#!/usr/bin/env python3
"""Pack `cvc plant-log` runs into a LieSPHGP dataset pickle.

`cvc plant-log` (cvc::physics::write_plant_log) writes, per run directory, ``flights.npy``
(float64, (F, T, 8): x, y, psi, vx, vy, r, drive, steer at a uniform rate) and ``meta.json``.
This script turns one or more of those directories into the pickle CVC-Lab/LieSPHGP reads
(``src/models/SE3_Quadrotor/lie_ph/data.py`` and the ROV variant's loader at commit 65db994):

- a dict, pickled with the highest protocol, with
  - ``t``: ``arange(T) * dt`` (the loaders take dt as the median step of ``t``);
  - ``train_trajectories``: (F_train, T, D), observation noise added when ``--noise`` > 0;
  - ``test_trajectories``: the clean test flights; ``test_trajectories_noisy``: with noise;
  - ``settings``: the rate, the layouts, the split, the noise and each source run's meta.
- Equal-length flights at a uniform rate, zero-order-hold inputs: row k's inputs acted from
  t_k-1 to t_k, and row 0's are zero (unused), as LieSPHGP's generator writes them.
- ``--layout se3`` (the default): 20-wide rows that LieSPHGP's ROV loader reads with
  ``model.control_dim: 2``: the planar pose lifted to SE(3),
  ``[x, y, 0, vec(Rz(psi)) row-major, vx, vy, 0, 0, 0, r, drive, steer]``.
  ``--layout se2``: the 8-wide rows as written, for an SE(2) port of the model.
- Noise (``--noise SIGMA``, absolute, as the paper's generator adds it): N(0, sigma) on x, y,
  vx, vy and r, and the rotation perturbed by a yaw drawn from N(0, sigma^2); the inputs are
  left exact. Only the planar channels get noise, so the lifted rows stay planar.

The split is by seed (``--test-seeds``) or, without it, every ``--test-every``-th flight.
``--check`` re-reads the pickle and checks it against that contract.

Standard library and numpy only.
"""

from __future__ import annotations

import argparse
import json
import math
import pickle
import sys
from pathlib import Path

import numpy as np

PLANAR = ("x", "y", "psi", "vx", "vy", "r", "drive", "steer")
SE3 = (
    ["x_w", "y_w", "z_w"]
    + [f"R{i}{j}" for i in range(3) for j in range(3)]
    + ["vx_b", "vy_b", "vz_b", "wx_b", "wy_b", "wz_b", "drive", "steer"]
)


def load_runs(dirs):
    flights, seeds, metas, dt, rows = [], [], [], None, None
    for d in dirs:
        d = Path(d)
        meta = json.loads((d / "meta.json").read_text(encoding="utf-8"))
        if meta.get("format") != "cvc-plant-log-1":
            raise SystemExit(f"{d}: not a cvc-plant-log run (format {meta.get('format')!r})")
        f = np.load(d / "flights.npy", allow_pickle=False)
        if f.ndim != 3 or f.shape[2] != len(PLANAR):
            raise SystemExit(f"{d}: flights.npy has shape {f.shape}, expected (F, T, 8)")
        if dt is None:
            dt, rows = float(meta["dt"]), f.shape[1]
        elif not math.isclose(float(meta["dt"]), dt) or f.shape[1] != rows:
            raise SystemExit(
                f"{d}: dt {meta['dt']} / {f.shape[1]} rows differ from the first run's"
            )
        if len(meta["flight_seed"]) != f.shape[0]:
            raise SystemExit(f"{d}: meta.json's flight_seed does not match flights.npy")
        flights.append(f)
        seeds.extend(int(s) for s in meta["flight_seed"])
        metas.append(
            {"dir": str(d), **{k: v for k, v in meta.items() if not k.startswith("flight_")}}
        )
    if not flights:
        raise SystemExit("no runs")
    return np.concatenate(flights, 0), np.asarray(seeds), metas, dt


def lift_se3(p):
    """(F, T, 8) planar rows -> (F, T, 20) SE(3) rows (planar)."""
    F, T, _ = p.shape
    out = np.zeros((F, T, len(SE3)), np.float64)
    c, s = np.cos(p[..., 2]), np.sin(p[..., 2])
    out[..., 0], out[..., 1] = p[..., 0], p[..., 1]
    out[..., 3], out[..., 4], out[..., 6], out[..., 7] = c, -s, s, c
    out[..., 11] = 1.0
    out[..., 12], out[..., 13], out[..., 17] = p[..., 3], p[..., 4], p[..., 5]
    out[..., 18], out[..., 19] = p[..., 6], p[..., 7]
    return out


def add_noise(p, sigma, rng):
    """Absolute observation noise on the planar state (x, y, psi, vx, vy, r); inputs exact."""
    q = p.copy()
    for k in (0, 1, 2, 3, 4, 5):
        q[..., k] += rng.normal(0.0, sigma, size=q.shape[:2])
    return q


def build(planar, seeds, metas, dt, layout, sigma, noise_seed, test_seeds, test_every):
    planar = planar.copy()
    planar[:, 0, 6:8] = 0.0  # row 0's inputs: unused, zero as LieSPHGP writes them
    if test_seeds:
        test = np.isin(seeds, list(test_seeds))
    else:
        test = (np.arange(planar.shape[0]) % test_every) == test_every - 1
    if test.all() or not test.any():
        raise SystemExit(f"the split leaves {int(test.sum())} of {test.size} flights for test")
    rng = np.random.default_rng(noise_seed)
    noisy = add_noise(planar, sigma, rng) if sigma > 0 else planar.copy()
    to = lift_se3 if layout == "se3" else (lambda x: x)
    T = planar.shape[1]
    return {
        "t": np.arange(T, dtype=np.float64) * dt,
        "train_trajectories": to(noisy[~test]),
        "test_trajectories": to(planar[test]),
        "test_trajectories_noisy": to(noisy[test]),
        "train_seed": seeds[~test],
        "test_seed": seeds[test],
        "settings": {
            "source": "libcvc cvc-plant-log (cvc::physics::nav_plant on Jolt)",
            "sample_dt": dt,
            "physics_hz": 100.0,
            "state_layout": list(SE3[:18] if layout == "se3" else PLANAR[:6]),
            "control_layout": ["drive", "steer"],
            "control_dim": 2,
            "layout": layout,
            "hold": "zero-order: row k's inputs acted from t_k-1 to t_k; row 0's are zero",
            "observation_noise": {"type": "absolute", "sigma": sigma, "seed": noise_seed},
            "splits": {
                "train": int((~test).sum()),
                "test": int(test.sum()),
                "test_seeds": sorted(int(s) for s in test_seeds) if test_seeds else None,
                "test_every": None if test_seeds else test_every,
            },
            "runs": metas,
        },
    }


def check(path, layout):
    with open(path, "rb") as fh:
        d = pickle.load(fh)
    width = len(SE3) if layout == "se3" else len(PLANAR)
    for key in ("t", "train_trajectories", "test_trajectories", "test_trajectories_noisy"):
        if key not in d:
            raise SystemExit(f"check: {key} missing")
    t = d["t"]
    dt = float(np.median(np.diff(t)))
    if not np.allclose(np.diff(t), dt):
        raise SystemExit("check: t is not uniform")
    for key in ("train_trajectories", "test_trajectories", "test_trajectories_noisy"):
        a = d[key]
        if a.ndim != 3 or a.shape[1] != t.size or a.shape[2] != width or a.dtype != np.float64:
            raise SystemExit(f"check: {key} has shape {a.shape} {a.dtype}")
        if not np.isfinite(a).all():
            raise SystemExit(f"check: {key} is not finite")
        if np.any(a[:, 0, -2:] != 0.0):
            raise SystemExit(f"check: {key} row 0 inputs are not zero")
    if layout == "se3":
        R = d["test_trajectories"][..., 3:12].reshape(-1, 3, 3)
        err = np.abs(np.einsum("nij,nkj->nik", R, R) - np.eye(3)).max()
        if err > 1e-9:
            raise SystemExit(f"check: R is not orthonormal ({err:.2e})")
        # The ROV loader's contract: 18 pose and twist columns, then control_dim inputs.
        if d["test_trajectories"].shape[-1] - 18 != 2:
            raise SystemExit("check: not 2 inputs after the 18 pose/twist columns")
    return d


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("runs", nargs="+", help="cvc plant-log --out directories")
    ap.add_argument("--out", required=True, help="the pickle to write")
    ap.add_argument("--layout", choices=("se3", "se2"), default="se3")
    ap.add_argument("--noise", type=float, default=0.05, help="absolute observation noise sigma")
    ap.add_argument("--noise-seed", type=int, default=0)
    ap.add_argument("--test-seeds", default="", help="comma-separated seeds held out for test")
    ap.add_argument("--test-every", type=int, default=5, help="without --test-seeds: every k-th")
    ap.add_argument("--check", action="store_true", help="re-read and check the pickle")
    a = ap.parse_args(argv)
    planar, seeds, metas, dt = load_runs(a.runs)
    test_seeds = {int(s) for s in a.test_seeds.split(",") if s.strip()}
    d = build(planar, seeds, metas, dt, a.layout, a.noise, a.noise_seed, test_seeds, a.test_every)
    with open(a.out, "wb") as fh:
        pickle.dump(d, fh, protocol=pickle.HIGHEST_PROTOCOL)
    s = d["settings"]["splits"]
    print(
        f"wrote {a.out}: {s['train']} train / {s['test']} test flights x {d['t'].size} rows "
        f"at {1.0 / dt:.0f} Hz, layout {a.layout}, noise sigma {a.noise}"
    )
    if a.check:
        check(a.out, a.layout)
        print("check: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
