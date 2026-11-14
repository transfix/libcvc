#!/usr/bin/env python3
# Copyright 2026 The University of Texas at Austin
#
# This file is part of libcvc.
#
# libcvc is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 2.1 as published by the Free Software Foundation.

"""Parity fixtures for cvc::ph composition (src/cvc/tests/ph_compose_parity_test.cpp).

Runs CVC-Lab/phast's own torch modules (CompositionalPHAST, PHASTCore, its potentials, masses,
damping and Strang/leapfrog/implicit-midpoint integrators) in float64 and records trajectories and
energies, together with each system in the "cvc-ph-system/1" format the C++ side loads. It imports
phast from a checkout and copies none of it. Run once, outside CI:

    python cphast_fixtures.py --phast /path/to/phast --out src/cvc/tests/data/ph_compose_golden.json

Cases (all with phast's step: two substeps of Euler coupling half-step, block kick, Strang damping
around the conservative step, kick, Euler coupling half-step):

  known_chain4     four linear DGUs (C 2.2 F, L 1.8 H, R 1.2 ohm; leapfrog) on a chain, RL lines
                   0.3/0.4/0.5 ohm, 0.8 H: the paper's Appendix H.1 network with linear laws
  known_add_bus0   that network one second in, plus a fifth DGU and a 0.4 ohm / 0.8 H line to
  known_add_bus2   bus 0 or bus 2 (the paper's two candidate connections; same laws, other wiring)
  learned_chain3   three copies of one learned DGU law (quadratic + MLP potential, an MLP mass, an
                   MLP damping; implicit midpoint by Picard) on a chain, lines 0.3/0.5 ohm, 0.8 H,
                   coupled through the learned efforts (dH/dq at the buses, as the paper's Eq. 203)

Two deliberate departures from phast's stock microgrid factory (models/phast/domains/microgrid.py):

  * A line's potential is exactly zero (a non-learnable CoupledQuadraticPotential with K = 0).
    The factory uses QuadraticPotential(k=0.0), and phast's _softplus_inverse(0.0) returns 0.0,
    so its k is softplus(0) = ln 2: a spring on the line's dummy charge that feeds the line's flux
    through leapfrog. --show-line-spring prints the difference that makes.
  * Learned DGUs are coupled through their efforts (an incidence port with needs_grad_q/p, the
    hook CompositionalPHAST already provides). The stock IncidenceCouplingPort uses q / C_node
    whatever the law.

Each DGU gets its own PHASTCore over shared modules, so phast's Picard stop (taken over a type's
whole batch) is per DGU, as it is in cvc::ph.
"""

import argparse
import json
import math
import subprocess
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--phast", required=True, help="a checkout of CVC-Lab/phast")
    ap.add_argument("--out", required=True, help="the golden JSON to write")
    ap.add_argument("--steps", type=int, default=200)
    ap.add_argument("--show-line-spring", action="store_true",
                    help="also report how far phast's stock line potential moves the known chain")
    args = ap.parse_args()
    sys.path.insert(0, args.phast)

    import torch
    import torch.nn as nn
    import torch.nn.functional as F

    torch.set_default_dtype(torch.float64)

    from models.phast.architectures.explicit.model import PHASTCore
    from models.phast.compositional_v2 import CompositionalPHAST
    from models.phast.physics.damping import ConstantDamping
    from models.phast.physics.mass import NeuralMass, ScalarMass
    from models.phast.physics.potentials import (CoupledQuadraticPotential, HybridPotential,
                                                 NeuralPotential, QuadraticPotential)
    from models.phast.ports.incidence_coupling import IncidenceCouplingPort
    from models.phast.systems.defs import BlockSpec

    C_DGU, L_DGU, R_DGU = 2.2, 1.8, 1.2
    DT = 0.01
    N_LAYERS = 2

    def softplus_inv(y):
        return math.log(math.expm1(y))

    def incidence(n, edges):
        # phast's orientation (build_incidence_matrix): +1 at the edge's lower bus, -1 at the other.
        B = torch.zeros(n, len(edges))
        for k, (i, j) in enumerate(edges):
            B[i, k] = 1.0
            B[j, k] = -1.0
        return B

    def core(potential, mass, damping, integrator):
        return PHASTCore(n_dof=1, potential=potential, mass=mass, damping=damping, integrator_type=integrator,
                         n_layers=N_LAYERS, dt_init=DT / N_LAYERS, learnable_dt=False, use_riemannian=False)

    def line_core(R, L, stock=False):
        pot = QuadraticPotential(k=0.0, learnable=False) if stock else \
            CoupledQuadraticPotential(stiffness=torch.zeros(1, 1), learnable=False)
        return core(pot, ScalarMass(1, L, learnable=False), ConstantDamping(R, learnable=False), "leapfrog")

    class EffortIncidencePort(nn.Module):
        """The paper's Eq. 203 coupling through the blocks' own efforts (dH/dq, dH/dp)."""
        needs_grad_q = True
        needs_grad_p = True

        def __init__(self, B):
            super().__init__()
            self.register_buffer("B", B.clone())
            self.n, self.m = B.shape

        def compute_delta(self, q, p, ctx):
            v = ctx.grad_q[..., : self.n]
            i_line = ctx.grad_p[..., self.n:]
            dq = torch.zeros_like(q)
            dp = torch.zeros_like(p)
            dq[..., : self.n] = torch.matmul(i_line, self.B.T)
            dp[..., self.n:] = -torch.matmul(v, self.B)
            return dq, dp

    def build(dgu_cores, lines, B, port):
        types, block_map = {}, []
        for i, c in enumerate(dgu_cores):
            types[f"dgu{i}"] = c
            block_map.append(BlockSpec(type_name=f"dgu{i}", n_dof=1))
        for k, c in enumerate(lines):
            types[f"line{k}"] = c
            block_map.append(BlockSpec(type_name=f"line{k}", n_dof=1))
        return CompositionalPHAST(types=types, block_map=block_map, interconnection=port)

    def phast_step(model, x, action):
        # CompositionalPHAST.forward's substep loop, on the canonical state.
        for layer in range(model._get_n_layers()):
            dt_layer = model._get_dt(layer)
            x = model._apply_interconnection_half(x, dt_substep=dt_layer)
            x = model._step_blocks_one_layer(x, layer, action=action)
            x = model._apply_interconnection_half(x, dt_substep=dt_layer)
        return x

    def total_energy(model, x):
        n = model.total_dof
        H = 0.0
        for i, b in enumerate(model.block_map):
            s, e = model._block_slices[i]
            H += float(model.types[b.type_name].H.energy(x[..., s:e], x[..., n + s:n + e]).sum())
        return H

    def efforts(model, x):
        n = model.total_dof
        gq, gp = [], []
        for i, b in enumerate(model.block_map):
            s, e = model._block_slices[i]
            H = model.types[b.type_name].H
            gq.append(H.grad_q(x[..., s:e], x[..., n + s:n + e]))
            gp.append(H.grad_p(x[..., s:e], x[..., n + s:n + e]))
        return torch.cat(gq, -1), torch.cat(gp, -1)

    # cvc::ph's layout: (q, p) per DGU, then phi per line. phast's: [q_dgu, q_line, p_dgu, p_line].
    def to_cvc(x, n, m):
        x = x.reshape(-1)
        nd = n + m
        out = []
        for i in range(n):
            out += [float(x[i]), float(x[nd + i])]
        out += [float(x[nd + n + k]) for k in range(m)]
        return out

    def from_cvc(v, n, m):
        nd = n + m
        x = torch.zeros(1, 2 * nd)
        for i in range(n):
            x[0, i] = v[2 * i]
            x[0, nd + i] = v[2 * i + 1]
        for k in range(m):
            x[0, nd + n + k] = v[2 * n + k]
        return x

    def action_of(u, n, m):
        # u in cvc::ph's layout: (I_in, V_src) per DGU.
        nd = n + m
        a = torch.zeros(1, 2 * nd)
        for i in range(n):
            a[0, i] = u[2 * i]
            a[0, nd + i] = u[2 * i + 1]
        return a

    def layers_of(seq):
        mods = list(seq)
        out = []
        for idx, mod in enumerate(mods):
            if not isinstance(mod, nn.Linear):
                continue
            act = "identity"
            if idx + 1 < len(mods):
                nxt = mods[idx + 1]
                act = {nn.SiLU: "silu", nn.Tanh: "tanh", nn.Softplus: "softplus",
                       nn.Sigmoid: "sigmoid"}.get(type(nxt), "identity")
            out.append({"W": mod.weight.detach().tolist(), "b": mod.bias.detach().tolist(), "act": act})
        return out

    def line_law(R, L):
        return {"format": "cvc-ph-law/1", "kind": "linear", "n": 1, "Q": [1.0 / L], "R": [R],
                "ports": 1, "B": [1.0], "inputs": 0}

    def system_json(laws, blocks, buses, lines, B):
        return {"format": "cvc-ph-system/1", "laws": laws, "blocks": blocks,
                "couplings": [{"type": "incidence", "buses": buses, "bus_port": 0, "lines": lines,
                               "line_port": 0, "A": B.tolist()}]}

    def run(model, x0_cvc, u_cvc, n, m, steps):
        x = from_cvc(x0_cvc, n, m)
        a = action_of(u_cvc, n, m)
        xs, Hs = [to_cvc(x, n, m)], [total_energy(model, x)]
        for _ in range(steps):
            x = phast_step(model, x, a).detach()
            xs.append(to_cvc(x, n, m))
            Hs.append(total_energy(model, x))
        return xs, Hs

    gen = torch.Generator().manual_seed(20261114)

    def uniform(lo, hi):
        return lo + (hi - lo) * float(torch.rand(1, generator=gen))

    def physical_start(n, lines_L, C=C_DGU, L=L_DGU):
        # The paper's ranges (Appendix H.1): bus voltage [0.5, 1.3] V, DGU current [-0.25, 0.35] A,
        # line current [-0.1, 0.1] A, as charge q = C v, flux p = L i, phi = L_e i_e.
        x = []
        for _ in range(n):
            x += [C * uniform(0.5, 1.3), L * uniform(-0.25, 0.35)]
        x += [Le * uniform(-0.1, 0.1) for Le in lines_L]
        return x

    def inputs(n):
        u = []
        for _ in range(n):
            u += [-uniform(0.05, 0.2), uniform(0.9, 1.2)]  # I_in = -load current, V_src
        return u

    cases = []

    # ── known laws ──────────────────────────────────────────────────────────────────────────────
    def known_dgu():
        return core(QuadraticPotential(k=1.0 / C_DGU, learnable=False), ScalarMass(1, L_DGU, learnable=False),
                    ConstantDamping(R_DGU, learnable=False), "leapfrog")

    def known_dgu_law(c):
        return {"format": "cvc-ph-law/1", "kind": "canonical", "n": 1,
                "potential": {"K": [[float(c.potential.k)]]}, "mass": {"diag": [float(c.mass.m)]},
                "damping": {"diag": [float(c.damping.d)]}, "ports": 1, "B": [[1.0], [0.0]],
                "inputs": 2, "G": [[1.0, 0.0], [0.0, 1.0]], "scheme": "leapfrog"}

    def known_case(name, n, edges, R, L, x0, u, steps, stock_lines=False):
        B = incidence(n, edges)
        dgus = [known_dgu() for _ in range(n)]
        lines = [line_core(r, l, stock=stock_lines) for r, l in zip(R, L)]
        model = build(dgus, lines, B, IncidenceCouplingPort(B, n, len(edges), C_node=C_DGU,
                                                            L_edge=torch.tensor(L)))
        xs, Hs = run(model, x0, u, n, len(edges), steps)
        laws = {"dgu": known_dgu_law(dgus[0])}
        blocks = [{"name": f"dgu{i}", "law": "dgu"} for i in range(n)]
        for k, (r, l) in enumerate(zip(R, L)):
            laws[f"line{k}"] = line_law(float(lines[k].damping.d), float(lines[k].mass.m))
            blocks.append({"name": f"line{k}", "law": f"line{k}"})
        sysj = system_json(laws, blocks, [f"dgu{i}" for i in range(n)], [f"line{k}" for k in range(len(edges))], B)
        return {"name": name, "system": sysj, "dt": DT, "substeps": N_LAYERS, "coupling": "euler",
                "steps": steps, "x0": x0, "u": u, "x": xs, "H": Hs}

    chain = [(0, 1), (1, 2), (2, 3)]
    R4, L4 = [0.3, 0.4, 0.5], [0.8, 0.8, 0.8]
    x0 = physical_start(4, L4)
    u4 = inputs(4)
    base = known_case("known_chain4", 4, chain, R4, L4, x0, u4, args.steps)
    cases.append(base)

    if args.show_line_spring:
        stock = known_case("stock", 4, chain, R4, L4, x0, u4, args.steps, stock_lines=True)
        d = max(abs(a - b) for ra, rb in zip(stock["x"], base["x"]) for a, b in zip(ra, rb))
        print(f"phast's stock line potential (k = ln 2) moves the known chain by up to {d:.3e} "
              f"in {args.steps} steps", file=sys.stderr)

    # One second in (step 100), the fifth DGU (1.0 V, 0.1 A) and its line (0 A) are added:
    # existing storage states are kept (the paper's AssignState with the identity reset).
    now = base["x"][100]
    new_dgu = [C_DGU * 1.0, L_DGU * 0.1]
    x_add = now[: 2 * 4] + new_dgu + now[2 * 4:] + [0.0]
    u_add = u4 + [-0.1, 1.05]
    for bus in (0, 2):
        cases.append(known_case(f"known_add_bus{bus}", 5, chain + [(bus, 4)], R4 + [0.4], L4 + [0.8],
                                x_add, u_add, args.steps))

    # ── a learned law ───────────────────────────────────────────────────────────────────────────
    class NetDamping(nn.Module):
        """D(q) = softplus(net(q)) + d_min, a diagonal coefficient per DOF (phast's apply_damping)."""

        def __init__(self, hidden, d_min):
            super().__init__()
            self.net = nn.Sequential(nn.Linear(1, hidden), nn.Tanh(), nn.Linear(hidden, 1))
            self.d_min = d_min

        def forward(self, q):
            return F.softplus(self.net(q)) + self.d_min

    torch.manual_seed(7)
    pot = HybridPotential(known=CoupledQuadraticPotential(stiffness=torch.tensor([[1.0 / C_DGU]]),
                                                          center=torch.tensor([0.2]), learnable=False),
                          residual=NeuralPotential(n_dof=1, hidden_dim=16, n_layers=3), residual_scale=0.1)
    mass = NeuralMass(n_dof=1, n_terms=0, hidden_dim=16, position_dependent=True, min_diag=1e-2)
    damp = NetDamping(hidden=8, d_min=0.05)
    with torch.no_grad():
        for net in (pot.residual.net, mass.diag_net, damp.net):
            for mod in net:
                if isinstance(mod, nn.Linear):
                    mod.weight.normal_(0.0, 1.0 / math.sqrt(mod.in_features), generator=gen)
                    mod.bias.normal_(0.0, 0.3, generator=gen)
        mass.diag_net[-1].bias.fill_(softplus_inv(L_DGU - 1e-2))
        damp.net[-1].bias.fill_(softplus_inv(R_DGU - 0.05))

    def learned_dgu():
        return core(pot, mass, damp, "implicit_midpoint_hamiltonian")

    n3, chain3, R3, L3 = 3, [(0, 1), (1, 2)], [0.3, 0.5], [0.8, 0.8]
    B3 = incidence(n3, chain3)
    dgus = [learned_dgu() for _ in range(n3)]
    lines = [line_core(r, l) for r, l in zip(R3, L3)]
    model = build(dgus, lines, B3, EffortIncidencePort(B3))
    law = {"format": "cvc-ph-law/1", "kind": "canonical", "n": 1,
           "potential": {"K": pot.known.K.tolist(), "center": pot.known.center.tolist(),
                         "scale": float(pot.scale), "net": layers_of(pot.residual.net)},
           "mass": {"net": layers_of(mass.diag_net), "min": float(mass.min_diag)},
           "damping": {"net": layers_of(damp.net), "min": damp.d_min},
           "ports": 1, "B": [[1.0], [0.0]], "inputs": 2, "G": [[1.0, 0.0], [0.0, 1.0]],
           "scheme": "implicit_midpoint"}
    laws = {"dgu": law}
    blocks = [{"name": f"dgu{i}", "law": "dgu"} for i in range(n3)]
    for k in range(len(chain3)):
        laws[f"line{k}"] = line_law(float(lines[k].damping.d), float(lines[k].mass.m))
        blocks.append({"name": f"line{k}", "law": f"line{k}"})
    x0 = physical_start(n3, L3)
    u3 = inputs(n3)
    xs, Hs = run(model, x0, u3, n3, len(chain3), args.steps)
    points = []
    for _ in range(16):
        xp = physical_start(n3, L3)
        xt = from_cvc(xp, n3, len(chain3))
        gq, gp = efforts(model, xt)
        e = to_cvc(torch.cat([gq, gp], -1), n3, len(chain3))
        points.append({"x": xp, "H": total_energy(model, xt), "e": e})
    cases.append({"name": "learned_chain3",
                  "system": system_json(laws, blocks, [f"dgu{i}" for i in range(n3)],
                                        [f"line{k}" for k in range(len(chain3))], B3),
                  "dt": DT, "substeps": N_LAYERS, "coupling": "euler", "steps": args.steps,
                  "x0": x0, "u": u3, "x": xs, "H": Hs, "points": points,
                  "solve": {"max_iter": 10, "tol": 1e-6}})

    try:
        commit = subprocess.run(["git", "-C", args.phast, "rev-parse", "HEAD"], capture_output=True,
                                text=True, check=True).stdout.strip()
    except Exception:  # noqa: BLE001
        commit = "unknown"
    out = {"format": "cvc-ph-compose-golden/1",
           "provenance": {"generator": "src/cvc/tools/cphast_fixtures.py",
                          "reference": "CVC-Lab/phast (Apache-2.0), run in float64",
                          "phast_commit": commit, "torch": torch.__version__},
           "cases": cases}
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    print(f"wrote {args.out}: {len(cases)} cases, phast {commit[:12]}", file=sys.stderr)


if __name__ == "__main__":
    main()
