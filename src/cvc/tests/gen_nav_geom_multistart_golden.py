#!/usr/bin/env python3
"""Regenerate the Python-parity goldens in nav_geom_rollout_grad_test.cpp.

Prints the ``// ---- BEGIN GENERATED`` ... ``END GENERATED`` block of that test:
a small fixed scene, the start fractions a seeded draw produces, and the loss and
coefficient gradients of ``multi_start_penalty`` under four configurations
({explicit, semi-implicit} x {legacy single start, sampled starts}).

The reference is GRL-SNAM's ``grl_snam/surrogate_robust.py`` (post-#113, which
added the sampled starts), evaluated in float32 with torch autograd:

* semi-implicit: ``multi_start_penalty`` exactly as GRL-SNAM ships it.
* explicit: the same function with ``integrate_surrogate_v2`` swapped for the
  material fork's update order (SetasAditya/material-aware-grl-snam,
  full_code/surrogate_robust.py: the position steps with the OLD velocity). The
  barrier stays GRL-SNAM's M10-corrected ``ipc_piecewise``, which is what
  cvc::nav uses; the fork still carries the pre-M10 derivative.

The draw is ``torch.rand(B, generator=torch.Generator().manual_seed(seed))`` per
start, which cvc::nav::multi_start_fracs reproduces bit for bit; the fractions are
printed as float bit patterns so the C++ side can check that exactly.

Usage (GRL-SNAM checkout on the path, torch installed):

    PYTHONPATH=/path/to/GRL-SNAM python gen_nav_geom_multistart_golden.py
"""

from __future__ import annotations

import struct

import torch

from grl_snam import surrogate_robust as sr

B, N = 5, 3
MARGIN = 0.5
SEED = 7
CONFIGS = [  # (name, semi_implicit, ms_count, frac_range)
    ("kLegacyExplicit", False, 1, (0.9, 0.9)),
    ("kLegacySemi", True, 1, (0.9, 0.9)),
    ("kSampledExplicit", False, 4, (0.8, 0.98)),
    ("kSampledSemi", True, 4, (0.8, 0.98)),
]
MS_H, MS_DT_MULT, TAU = 3, 4.0, 0.05


def scene():
    """Five agents, three obstacle slots (the last padded for agents 0-3).

    0-2: weak barriers and a goal beyond the nearest obstacle, so every start
         penetrates mid-rollout and the gradient is non-trivial.
    3:   already penetrating obstacle 0 at o0 (negative clearance), which is the
         only way the feasibility fallback can fire.
    4:   no valid obstacle at all (every slot padded): contributes 0.
    """
    f = torch.float32
    o0 = torch.tensor([[0.0, 0.0], [0.05, -0.08], [-0.06, 0.03], [0.0, 0.0], [0.2, 0.1]], dtype=f)
    v0 = torch.tensor([[0.1, 0.0], [0.12, 0.03], [0.05, -0.02], [0.02, 0.01], [0.1, 0.1]], dtype=f)
    goal = o0 + torch.tensor([[3.0, 0.1], [3.0, -0.15], [2.8, 0.2], [3.0, 0.0], [3.0, 3.0]], dtype=f)
    C = torch.tensor(
        [
            [[1.0, 0.02], [0.05, 1.3], [9.0, 9.0]],
            [[1.05, -0.1], [0.0, 1.25], [9.0, 9.0]],
            [[0.95, 0.05], [-0.1, -1.3], [9.0, 9.0]],
            [[0.5, 0.0], [0.0, 1.4], [9.0, 9.0]],
            [[1.0, 0.0], [0.0, 1.0], [9.0, 9.0]],
        ],
        dtype=f,
    )
    R = torch.full((B, N), 0.35, dtype=f)
    mask = torch.tensor([[1, 1, 0]] * 4 + [[0, 0, 0]], dtype=torch.bool)
    alphas = torch.tensor(
        [[0.012, 0.018, 0.5], [0.015, 0.011, 0.5], [0.019, 0.014, 0.5], [0.016, 0.013, 0.5]]
        + [[0.017, 0.012, 0.5]],
        dtype=f,
    )
    beta = torch.tensor([1.2, 1.3, 1.15, 1.25, 1.1], dtype=f)
    gamma = torch.tensor([0.25, 0.3, 0.22, 0.28, 0.2], dtype=f)
    rr = torch.full((B,), 0.5, dtype=f)
    d_hat = torch.full((B,), 3.0, dtype=f)
    dt = torch.full((B,), 0.12, dtype=f)
    H = torch.full((B,), 3, dtype=torch.long)
    return o0, v0, goal, C, R, mask, alphas, beta, gamma, rr, d_hat, dt, H


def integrate_explicit(
    o0, v0, goal, C, R, mask, alphas, beta, gamma, d_hat, dt, H,
    robot_radius=0.0, margin_factor=0.5, mass=1.0,
):  # fmt: skip
    """GRL-SNAM's integrate_surrogate_v2 with the fork's (explicit) update order."""
    B_, N_ = C.shape[:2]
    rr = sr._per_sample_radius(robot_radius, o0)
    R_eff = R + margin_factor * rr[:, None]
    o, v = o0.clone(), v0.clone()
    min_clear = torch.full((B_,), float("inf"), dtype=o.dtype, device=o.device)
    for s in range(int(H.max().item())):
        active = (s < H).to(o.dtype).unsqueeze(-1)
        F_goal = -beta.unsqueeze(-1) * (o - goal)
        diff = o.unsqueeze(1) - C
        r = torch.linalg.norm(diff, dim=-1).clamp_min(1e-9)
        n_hat = diff / r.unsqueeze(-1)
        d = r - R_eff
        d = torch.where(mask, d, torch.full_like(d, 1e6))
        _, dbdd = sr.ipc_piecewise(d, d_hat.view(-1, 1))
        F_bar = (-(alphas * dbdd).unsqueeze(-1) * n_hat).sum(dim=1)
        dmin = torch.where(mask, d, torch.full_like(d, float("inf"))).min(dim=1).values
        min_clear = torch.minimum(min_clear, dmin)
        a = (F_bar + F_goal - gamma.unsqueeze(-1) * v) / float(mass)
        o = o + active * dt.unsqueeze(-1) * v  # OLD v (the fork's order)
        v = v + active * dt.unsqueeze(-1) * a
    return o, v, min_clear


def penalty(semi, ms_count, frac_range):
    o0, v0, goal, C, R, mask, alphas, beta, gamma, rr, d_hat, dt, H = scene()
    coefs = [t.clone().requires_grad_(True) for t in (alphas, beta, gamma)]
    gen = torch.Generator().manual_seed(SEED)
    kw = dict(
        robot_radius=rr, margin_factor=MARGIN, ms_count=ms_count, ms_h=MS_H,
        ms_dt_mult=MS_DT_MULT, tau=TAU, frac_range=frac_range, generator=gen,
    )  # fmt: skip
    saved = sr.integrate_surrogate_v2
    if not semi:
        sr.integrate_surrogate_v2 = integrate_explicit
    try:
        L = sr.multi_start_penalty(o0, v0, goal, C, R, mask, *coefs, d_hat, dt, H, **kw)
    finally:
        sr.integrate_surrogate_v2 = saved
    L.backward()
    assert torch.isfinite(L) and all(torch.isfinite(c.grad).all() for c in coefs)
    return L.item(), [c.grad.reshape(-1).tolist() for c in coefs]


def bits(x):
    return struct.unpack("<I", struct.pack("<f", x))[0]


def fl(xs):
    """Round-trippable float32 literals (``3`` -> ``3.0f``: ``3f`` is not C++)."""
    out = []
    for x in xs:
        s = f"{x:.9g}"
        out.append((s if any(c in s for c in ".e") else s + ".0") + "f")
    return ", ".join(out)


def main():
    o0, v0, goal, C, R, mask, alphas, beta, gamma, rr, d_hat, dt, H = scene()
    gen = torch.Generator().manual_seed(SEED)
    lo, hi = 0.8, 0.98
    fracs = torch.cat([lo + (hi - lo) * torch.rand(B, generator=gen) for _ in range(4)]).tolist()

    out = ["// ---- BEGIN GENERATED by gen_nav_geom_multistart_golden.py (do not edit) ----"]
    out.append(f"constexpr int kPB = {B}, kPN = {N};")
    out.append(f"constexpr unsigned kPSeed = {SEED}u;")
    for name, t in [
        ("kPO0", o0), ("kPV0", v0), ("kPGoal", goal), ("kPC", C), ("kPR", R),
        ("kPAlphas", alphas), ("kPBeta", beta), ("kPGamma", gamma), ("kPRr", rr),
        ("kPDHat", d_hat), ("kPDt", dt),
    ]:  # fmt: skip
        out.append(f"const float {name}[] = {{{fl(t.reshape(-1).tolist())}}};")
    out.append(f"const std::uint8_t kPMask[] = {{{', '.join(str(int(m)) for m in mask.reshape(-1))}}};")
    out.append(f"const int kPH[] = {{{', '.join(str(int(h)) for h in H)}}};")
    out.append("// 4 starts x 5 agents, frac (0.8, 0.98), seed kPSeed: float bit patterns")
    out.append(f"const std::uint32_t kPFracBits[] = {{{', '.join(f'0x{bits(x):08x}u' for x in fracs)}}};")
    out.append("struct parity_golden {")
    out.append("  bool semi_implicit;")
    out.append("  int ms_count;")
    out.append("  double frac_lo, frac_hi;")
    out.append("  double L;")
    out.append(f"  float g_alphas[{B * N}], g_beta[{B}], g_gamma[{B}];")
    out.append("};")
    out.append("const parity_golden kPGoldens[] = {")
    for name, semi, ms_count, (flo, fhi) in CONFIGS:
        L, (ga, gb, gg) = penalty(semi, ms_count, (flo, fhi))
        out.append(f"    // {name}")
        out.append(f"    {{{'true' if semi else 'false'}, {ms_count}, {flo}, {fhi}, {L!r},")
        out.append(f"     {{{fl(ga)}}},")
        out.append(f"     {{{fl(gb)}}},")
        out.append(f"     {{{fl(gg)}}}}},")
    out.append("};")
    out.append(f"// torch {torch.__version__}")
    out.append("// ---- END GENERATED ----")
    print("\n".join(out))


if __name__ == "__main__":
    main()
