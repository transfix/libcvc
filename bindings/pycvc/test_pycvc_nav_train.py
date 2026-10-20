#!/usr/bin/env python3
"""pycvc nav material-trainer integration test — the torch-free training path, at the binding level.

Exercises the raw ``pycvc.nav_material_trainer_*`` C surface (libcvc #247) end-to-end, numpy-only,
no torch: mint a random ``.cvcnm`` (byte layout per coef_energy_net_io.cpp), create a trainer, run
Adam steps, assert the loss decreases, save, and reload the trained weights. Mirrors libcvc's
``NavCoefTrain.TrainThenSaveReloadIntegration`` (the coef_mlp twin) but for the coef_energy_net
material trainer, and lives in the binding's OWN repo so a pycvc_nav.i ABI regression (arg order,
shape/dtype validation, capsule lifetime, GIL) is caught here — not only downstream in grl-snam.

Script-style (run as ``python test_pycvc_nav_train.py``), like the other pycvc tests; self-skips
(exit 0) when the running pycvc predates the trainer binding.
"""

import os
import struct
import tempfile

import numpy as np
import pycvc

if not hasattr(pycvc, "nav_material_trainer_create"):
    print("pycvc lacks nav_material_trainer_create (pre-#247 build) — skipping")
    raise SystemExit(0)

# ── CoefEnergyNetMaterial parameter table (name, shape) — matnet_export's layout, copied verbatim
# from grl-snam tests/test_material_train_native.py (byte layout matches coef_energy_net_io.cpp). ──
_TENSORS = [
    ("goal_enc.0.weight", (64, 4)),
    ("goal_enc.0.bias", (64,)),
    ("goal_enc.2.weight", (64, 64)),
    ("goal_enc.2.bias", (64,)),
    ("obs_enc.0.weight", (128, 6)),
    ("obs_enc.0.bias", (128,)),
    ("obs_enc.2.weight", (64, 128)),
    ("obs_enc.2.bias", (64,)),
]
for _L in (0, 1):
    _p = f"fuser.layers.{_L}."
    _TENSORS += [
        (_p + "self_attn.in_proj_weight", (192, 64)),
        (_p + "self_attn.in_proj_bias", (192,)),
        (_p + "self_attn.out_proj.weight", (64, 64)),
        (_p + "self_attn.out_proj.bias", (64,)),
        (_p + "linear1.weight", (128, 64)),
        (_p + "linear1.bias", (128,)),
        (_p + "linear2.weight", (64, 128)),
        (_p + "linear2.bias", (64,)),
        (_p + "norm1.weight", (64,)),
        (_p + "norm1.bias", (64,)),
        (_p + "norm2.weight", (64,)),
        (_p + "norm2.bias", (64,)),
    ]
for _h in ("alpha_head", "beta_head", "gamma_head"):
    _TENSORS += [
        (_h + ".0.weight", (64, 64)),
        (_h + ".0.bias", (64,)),
        (_h + ".2.weight", (1, 64)),
        (_h + ".2.bias", (1,)),
    ]
_TENSORS += [
    ("risk_enc.net.0.weight", (16, 2, 3, 3)),
    ("risk_enc.net.0.bias", (16,)),
    ("risk_enc.net.2.weight", (32, 16, 3, 3)),
    ("risk_enc.net.2.bias", (32,)),
    ("risk_enc.net.4.weight", (64, 32, 3, 3)),
    ("risk_enc.net.4.bias", (64,)),
    ("risk_enc.net.8.weight", (64, 1024)),
    ("risk_enc.net.8.bias", (64,)),
]
for _h in ("lam_soft_head", "lam_hard_head", "mu_lat_head"):
    _TENSORS += [
        (_h + ".0.weight", (64, 128)),
        (_h + ".0.bias", (64,)),
        (_h + ".2.weight", (1, 64)),
        (_h + ".2.bias", (1,)),
    ]


def _write_random_cvcnm(path, patch_size, seed=0):
    rng = np.random.default_rng(seed)
    with open(path, "wb") as f:
        f.write(b"CVNM")
        f.write(struct.pack("<I", 1))
        f.write(struct.pack("<Q", 0xABCD))
        f.write(struct.pack("<IIIII", 64, 4, 2, 64, patch_size))
        f.write(struct.pack("<ffff", 5.0, 10.0, 5.0, 1e-5))
        f.write(struct.pack("<I", len(_TENSORS)))
        for name, shape in _TENSORS:
            nb = name.encode()
            f.write(struct.pack("<I", len(nb)))
            f.write(nb)
            f.write(struct.pack("<I", len(shape)))
            for d in shape:
                f.write(struct.pack("<I", d))
            n = int(np.prod(shape))
            w = rng.uniform(0.8, 1.2, n) if ("norm" in name and "weight" in name) else rng.uniform(-0.15, 0.15, n)
            f.write(w.astype("<f4").tobytes())
        f.write(struct.pack("<I", 0))  # meta_len


def _synthetic_batch(B=16, N=3, P=16, Hp=13, Wp=13, seed=7):
    rng = np.random.default_rng(seed)
    o0 = rng.uniform(-0.5, 0.5, (B, 2)).astype(np.float32)
    goal = rng.uniform(1.5, 2.5, (B, 2)).astype(np.float32)
    ang = rng.uniform(0, 6.28, (B, N))
    dist = rng.uniform(1.0, 1.4, (B, N))
    C = np.stack([o0[:, 0:1] + dist * np.cos(ang), o0[:, 1:2] + dist * np.sin(ang)], -1).astype(np.float32)
    R = rng.uniform(0.4, 0.6, (B, N)).astype(np.float32)
    mask = np.ones((B, N), np.uint8)
    mask[:, N - 1] = 0
    obs_feats = np.concatenate([C, R[..., None], rng.uniform(0.5, 1.5, (B, N, 1)), goal[:, None, :] - C], -1).astype(np.float32)
    goal_feats = np.concatenate([goal - o0, np.linalg.norm(goal - o0, axis=1, keepdims=True), np.ones((B, 1))], 1).astype(np.float32)
    risk_patch = np.stack([rng.uniform(0.1, 0.9, (B, P, P)), (rng.uniform(0, 1, (B, P, P)) > 0.7).astype(np.float32)], 1).astype(np.float32)
    rollout_patch = rng.uniform(-0.3, 0.3, (B, 6, Hp, Wp)).astype(np.float32)
    rollout_patch[:, 0] = rng.uniform(0.1, 0.9, (B, Hp, Wp))
    rollout_patch[:, 1] = rng.uniform(1.0, 4.0, (B, Hp, Wp))
    return {
        "obs_feats": obs_feats, "obs_mask": mask, "goal_feats": goal_feats, "risk_patch": risk_patch,
        "o0": o0, "v0": rng.uniform(-0.1, 0.1, (B, 2)).astype(np.float32), "goal": goal, "C": C, "R": R,
        "rollout_patch": rollout_patch, "rr": np.full(B, 0.5, np.float32), "d_hat": np.full(B, 3.0, np.float32),
        "dt": np.full(B, 0.1, np.float32), "H": np.where(np.arange(B) % 2 == 0, 2, 3).astype(np.int32),
        "o_tgt": (goal + rng.uniform(-0.3, 0.3, (B, 2))).astype(np.float32),
        "v_tgt": rng.uniform(-0.2, 0.2, (B, 2)).astype(np.float32), "gamma_o": rng.uniform(3.0, 5.0, B).astype(np.float32),
    }


# The 17 batch arrays nav_material_trainer_step/_loss expect, in order + exact dtype.
_ORDER = ("obs_feats", "obs_mask", "goal_feats", "risk_patch", "o0", "v0", "goal", "C", "R",
          "rollout_patch", "rr", "d_hat", "dt", "H", "o_tgt", "v_tgt", "gamma_o")


def _packed(batch):
    u8, i32 = {"obs_mask"}, {"H"}
    return [np.ascontiguousarray(batch[k], dtype=np.uint8 if k in u8 else np.int32 if k in i32 else np.float32)
            for k in _ORDER]


def test_trainer_reduces_loss_and_round_trips():
    with tempfile.TemporaryDirectory() as d:
        init = os.path.join(d, "init.cvcnm")
        _write_random_cvcnm(init, patch_size=16, seed=1)
        h = pycvc.nav_material_trainer_create(init)  # defaults ok
        args = _packed(_synthetic_batch())
        steps = 70
        l0 = pycvc.nav_material_trainer_step(h, *args, 1e-3, 0)  # first step's loss (pre-update)
        last = l0
        for s in range(1, steps):
            lr = 1e-4 + (1e-3 - 1e-4) * 0.5 * (1 + np.cos(np.pi * s / steps))  # cosine anneal, no torch
            last = pycvc.nav_material_trainer_step(h, *args, float(lr), 0)
        assert np.isfinite(last), "loss went non-finite"
        assert last < 0.8 * l0, f"loss did not decrease ({l0:.4f} -> {last:.4f})"

        out = os.path.join(d, "trained.cvcnm")
        pycvc.nav_material_trainer_save(h, out)
        assert os.path.getsize(out) > 0, "saved .cvcnm is empty"
        # Reload the TRAINED weights into a fresh trainer without error (the round-trip).
        pycvc.nav_material_trainer_create(out)
        print(f"  train+round-trip: loss {l0:.4f} -> {last:.4f}, saved {os.path.getsize(out)} B, reloaded OK")


def test_forward_only_loss_does_not_train():
    if not hasattr(pycvc, "nav_material_trainer_loss"):
        print("  (no nav_material_trainer_loss — skipping the forward-only invariant)")
        return
    with tempfile.TemporaryDirectory() as d:
        init = os.path.join(d, "init.cvcnm")
        _write_random_cvcnm(init, patch_size=16, seed=3)
        h = pycvc.nav_material_trainer_create(init)
        args = _packed(_synthetic_batch(seed=11))
        first = pycvc.nav_material_trainer_loss(h, *args, float("nan"), 0)
        again = pycvc.nav_material_trainer_loss(h, *args, float("nan"), 0)
        assert first == again, "forward-only loss touched the weights"  # loss() must not train
        stepped = pycvc.nav_material_trainer_step(h, *args, 0.0, 0)  # lr=0 -> weights unchanged
        assert abs(stepped - first) / (abs(first) + 1e-6) < 1e-6, "step(lr=0) changed the loss"
        print("  forward-only loss + step(lr=0) leave the weights unmoved: OK")


# nav_material_trainer_create's positional defaults up to use_cuda (grad_clip .. use_cuda), so a
# test can append the multi-start sampling args (ms_count, frac_lo, frac_hi, seed, ms_semi_implicit).
_CREATE_DEFAULTS = (5.0, 1.0, 0.5, 0.1, 5e-3, 0.01, 2.0, 0.01, 1.0, 5.0, 0.95, 0.5, 5.0, 10.0, 0.5, 1.0,
                    3.0, 5.0, 0.05, 3, 4.0, 0)


def test_sampled_multi_start_args():
    """The L_multi start-sampling args (GRL-SNAM #113): the degenerate default is the legacy single
    start, a sampled range changes the loss, validation scores the base seed, and step k draws with
    seed + k."""
    with tempfile.TemporaryDirectory() as d:
        init = os.path.join(d, "init.cvcnm")
        _write_random_cvcnm(init, patch_size=16, seed=5)
        args = _packed(_synthetic_batch(seed=13))

        def create(ms_count, frac_lo, frac_hi, seed, semi=0):
            return pycvc.nav_material_trainer_create(init, *_CREATE_DEFAULTS, ms_count, frac_lo, frac_hi,
                                                     seed, semi)

        def loss(h):
            return pycvc.nav_material_trainer_loss(h, *args, float("nan"), 0)

        legacy = loss(pycvc.nav_material_trainer_create(init))
        # A degenerate range draws nothing: any ms_count/seed is the legacy single start, exactly.
        assert loss(create(10, 0.9, 0.9, 123)) == legacy, "degenerate range moved the loss"
        sampled = create(4, 0.8, 0.98, 5)
        assert loss(sampled) != legacy, "sampled starts did not reach the loss"
        assert loss(sampled) == loss(sampled), "validation loss is not deterministic"
        assert loss(create(4, 0.8, 0.98, 5, 1)) != loss(sampled), "semi-implicit flag ignored"

        # step k samples with seed + k: at lr=0 the weights never move, so handle(seed=5)'s second
        # step scores what handle(seed=6) scores at its base seed. (step vs loss: same value, but
        # compared to a tolerance, as test_forward_only_loss_does_not_train does.)
        def close(x, y):
            return abs(x - y) / (abs(y) + 1e-6) < 1e-6

        a = create(4, 0.8, 0.98, 5)
        a0 = pycvc.nav_material_trainer_step(a, *args, 0.0, 0)
        a1 = pycvc.nav_material_trainer_step(a, *args, 0.0, 0)
        assert close(a0, loss(sampled)), "step 0 did not draw with the base seed"
        assert not close(a1, a0), "the draw did not advance between steps"
        assert close(a1, loss(create(4, 0.8, 0.98, 6))), "step 1 did not draw with seed + 1"

        try:
            create(4, 0.98, 0.8, 0)
        except Exception:  # a bad frac range is rejected at create, before any GIL-free step
            pass
        else:
            raise AssertionError("an inverted frac range was accepted")
        print(f"  multi-start sampling: legacy {legacy:.6f}, sampled {loss(sampled):.6f}, per-step seed OK")


def test_bicycle_integrator():
    """P8b: the loss taken through the kinematic bicycle instead of the point mass."""
    if not hasattr(pycvc, "nav_material_trainer_set_bicycle"):
        print("  (no nav_material_trainer_set_bicycle -- skipping the bicycle integrator)")
        return
    with tempfile.TemporaryDirectory() as d:
        init = os.path.join(d, "init.cvcnm")
        _write_random_cvcnm(init, patch_size=16, seed=7)
        h = pycvc.nav_material_trainer_create(init)
        args = _packed(_synthetic_batch())
        assert pycvc.nav_material_trainer_integrator(h) == "surrogate"
        point_mass = pycvc.nav_material_trainer_loss(h, *args, float("nan"), 0)
        # Vehicle limits for the synthetic batch's scale (positions within a few units).
        pycvc.nav_material_trainer_set_bicycle(h, 1, 0.3, 0.6, 2.0, 1.5, 0.8, 1.5, 1)
        assert pycvc.nav_material_trainer_integrator(h) == "bicycle"
        bike = pycvc.nav_material_trainer_loss(h, *args, float("nan"), 0)
        assert np.isfinite(bike) and bike != point_mass, "the bicycle did not change the loss"
        l0 = pycvc.nav_material_trainer_step(h, *args, 1e-3, 0)
        last = l0
        for _ in range(20):
            last = pycvc.nav_material_trainer_step(h, *args, 1e-3, 0)
        assert np.isfinite(last) and last < l0, f"bicycle training did not descend ({l0} -> {last})"
        try:
            pycvc.nav_material_trainer_set_bicycle(h, 1, 0.0)  # a zero wheelbase is refused
        except Exception:
            pass
        else:
            raise AssertionError("a zero wheelbase was accepted")
        pycvc.nav_material_trainer_set_bicycle(h, 0)
        assert pycvc.nav_material_trainer_integrator(h) == "surrogate"
        print(f"  bicycle integrator: point-mass loss {point_mass:.4f}, bicycle {bike:.4f}, "
              f"20 steps {l0:.4f} -> {last:.4f}")


def _takes_ext(fn, *args):
    """Whether a binding takes the trailing external-force args (material_ext): a pycvc without
    them raises TypeError on the extra argument."""
    try:
        fn(*args)
    except TypeError:
        return False
    except Exception:
        return True
    return True


def test_external_force():
    """An external force in the material training rollouts (material_ext): a zero force patch is
    the loss without one, a force patch changes it, training with it descends, malformed arrays
    are refused before the GIL-free step, and the rollout bindings reproduce libcvc's golden."""
    with tempfile.TemporaryDirectory() as d:
        init = os.path.join(d, "init.cvcnm")
        _write_random_cvcnm(init, patch_size=16, seed=9)
        h = pycvc.nav_material_trainer_create(init)
        batch = _synthetic_batch(seed=17)
        args = _packed(batch)
        B, Hp, Wp = batch["o0"].shape[0], batch["rollout_patch"].shape[2], batch["rollout_patch"].shape[3]
        if not _takes_ext(pycvc.nav_material_trainer_loss, h, *args, float("nan"), 0, None):
            print("  (pycvc lacks the external-force args -- skipping)")
            return
        base = pycvc.nav_material_trainer_loss(h, *args, float("nan"), 0)
        zero = np.zeros((B, 1, 2, Hp, Wp), np.float32)
        assert pycvc.nav_material_trainer_loss(h, *args, float("nan"), 0, zero) == base, \
            "a zero force patch moved the loss"
        yy, xx = np.meshgrid(np.linspace(0, 1, Hp), np.linspace(0, 1, Wp), indexing="ij")
        one = np.stack([0.3 * np.sin(5 * xx + 2 * yy), 0.3 * np.cos(3 * xx - 4 * yy)]).astype(np.float32)
        force = np.ascontiguousarray(np.broadcast_to(one, (B, 1, 2, Hp, Wp)))
        pushed = pycvc.nav_material_trainer_loss(h, *args, float("nan"), 0, force)
        assert np.isfinite(pushed) and pushed != base, "the force patch did not reach the loss"
        l0 = pycvc.nav_material_trainer_step(h, *args, 1e-3, 0, force)
        last = l0
        for _ in range(20):
            last = pycvc.nav_material_trainer_step(h, *args, 1e-3, 0, force)
        assert np.isfinite(last) and last < l0, f"training with the force did not descend ({l0} -> {last})"
        H = int(batch["H"].max())
        bad = [
            ("a patch off the rollout grid", dict(ext_patch=np.zeros((B, 1, 2, Hp + 1, Wp), np.float32))),
            ("a patch index past K", dict(ext_patch=force, ext_patch_of_step=np.full((B, H), 1, np.int32))),
            ("too few steps", dict(ext_F=np.zeros((B, H - 1, 2), np.float32))),
            ("J without F", dict(ext_J=np.zeros((B, H, 4), np.float32))),
        ]
        for why, kw in bad:
            try:
                pycvc.nav_material_trainer_loss(h, *args, float("nan"), 0, kw.get("ext_patch"),
                                                kw.get("ext_patch_of_step"), kw.get("ext_F"), kw.get("ext_J"))
            except (ValueError, TypeError, RuntimeError):
                pass
            else:
                raise AssertionError(f"{why} was accepted")

    # The rollout bindings against libcvc's golden (NavMaterialExt.Golden): the surrogate and the
    # bicycle with a scheduled force patch; the bicycle VJP's coefficient gradients and dL/dF_ext.
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src", "cvc", "tests",
                        "data", "material_ext_golden.json")
    if not os.path.exists(path):
        print("  (no material_ext_golden.json beside the sources -- skipping the rollout check)")
        return
    import json

    gold = json.load(open(path))
    worst = 0.0

    def close(a, b, tol=1e-5):
        a, b = np.asarray(a, np.float64).ravel(), np.asarray(b, np.float64).ravel()
        r = np.max(np.abs(a - b) / np.maximum(1.0, np.maximum(np.abs(a), np.abs(b)))) if a.size else 0.0
        assert r <= tol, f"{r:.3e} > {tol}"
        return r

    for c in gold["cases"]:
        if c["integrator"] == "dynamic":
            continue
        B, N, Hp, Wp, K, steps = c["B"], c["N"], c["Hp"], c["Wp"], c["K"], c["steps"]
        f32 = lambda k, *shape: np.ascontiguousarray(np.asarray(c[k], np.float32).reshape(*shape))  # noqa: E731
        common = [f32("goal", B, 2), f32("C", B, N, 2), f32("R", B, N),
                  np.ascontiguousarray(np.asarray(c["mask"], np.uint8).reshape(B, N)), f32("alphas", B, N),
                  f32("beta", B), f32("gamma", B), f32("lam_soft", B), f32("lam_hard", B),
                  f32("rollout_patch", B, 6, Hp, Wp), f32("rr", B), f32("d_hat", B), f32("dt", B),
                  np.ascontiguousarray(np.asarray(c["H"], np.int32))]
        p, ext_patch = c["params"], f32("ext_patch", B, K, 2, Hp, Wp)
        pos = np.ascontiguousarray(np.asarray(c["patch_of_step"], np.int32).reshape(B, steps))
        if c["integrator"] == "surrogate":
            out = pycvc.nav_integrate_surrogate_material(
                f32("o0", B, 2), f32("v0", B, 2), *common, p["margin_factor"], p["mass"], p["d_hat_sdf"],
                p["k_sharp"], 1, ext_patch, pos)
            names = ("o", "th", "min_clear", "cum_risk", "hard_count", "arc_length")
        else:
            v = c["vehicle"]
            veh = (p["margin_factor"], p["mass"], p["d_hat_sdf"], p["k_sharp"], v["L"], v["delta_max"],
                   v["a_max"], v["a_lat_max"], v["k_steer"], v["vmax"], v["allow_reverse"])
            start = [f32("o0", B, 2), f32("th0", B), f32("sp0", B)]
            out = pycvc.nav_integrate_bicycle_material(*start, *common, *veh, 1, ext_patch, pos)
            names = ("o", "th", "sp", "min_clear", "cum_risk", "hard_count", "arc_length")
            q = c["seeds"]
            seeds = [np.asarray(q["o"], np.float32).reshape(B, 2)] + [
                np.asarray(q[k], np.float32) for k in ("th", "sp", "min_clear", "cum_risk", "arc_length")]
            g = pycvc.nav_integrate_bicycle_material_vjp(*start, *common, *veh, *seeds, 1, ext_patch, pos,
                                                         None, None, 1, 1)
            assert len(g) == 6, "want_g_fext did not return g_fext"
            for k, got in zip(("alphas", "beta", "gamma", "lam_soft", "lam_hard", "fext"), g):
                worst = max(worst, close(got, c["grad"][k], 1e-4))
        for k, got in zip(names, out):
            worst = max(worst, close(got, c["out"][k]))
    print(f"  external force: zero patch = no force, loss {base:.4f} -> {pushed:.4f} with it, "
          f"20 steps {l0:.4f} -> {last:.4f}; rollouts vs golden {worst:.2e}")


if __name__ == "__main__":
    test_trainer_reduces_loss_and_round_trips()
    test_forward_only_loss_does_not_train()
    test_sampled_multi_start_args()
    test_bicycle_integrator()
    test_external_force()
    print("pycvc nav material-trainer integration: OK")
