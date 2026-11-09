#!/usr/bin/env python3
"""pycvc's belief-material controller bindings (C1) against GRL-SNAM's golden.

The golden, src/cvc/tests/data/belief_material_controller_golden.{json,cvcnm}, is written by
GRL-SNAM's grl_snam.tools.belief_material_controller_golden (NAV-BELIEF-MATERIAL-CONTROLLER-
ROADMAP.md §6); libcvc's nav_belief_material_controller_test reproduces it in C++. This checks the
bindings GRL-SNAM's live parity test will call reproduce it too:

- nav_belief_material_contract: every accepted meta's values and to_meta, every refusal a
  ValueError (G1); nav_belief_material_contract_to_meta;
- nav_belief_material_check_cvcnm: the header check's verdicts (G2);
- nav_visible_obstacles: centres, radii, weights bit for bit (G5);
- nav_belief_material_controller_inputs: the tokens and the patch bit for bit, the goal token's
  dist within 1 ulp (G4).

Script-style, like the other pycvc tests; self-skips (exit 0) on a pycvc without the bindings.
"""

import json
import os

import numpy as np
import pycvc

if not hasattr(pycvc, "nav_belief_material_controller_inputs"):
    print("pycvc lacks nav_belief_material_controller_inputs (pre-C1 build) — skipping")
    raise SystemExit(0)

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "..", "src", "cvc", "tests", "data")
G = json.load(open(os.path.join(DATA, "belief_material_controller_golden.json")))
SCENE = json.load(open(os.path.join(DATA, "belief_material_golden.json")))
CVCNM = os.path.join(DATA, G["net"]["file"])


def f32(bits):
    return np.array(bits, np.uint32).view(np.float32)


def test_contract():
    n_ok = n_bad = 0
    for c in G["contract"]:
        if not c["accepted"]:
            try:
                pycvc.nav_belief_material_contract(c["meta"])
            except ValueError:
                n_bad += 1
                continue
            raise AssertionError(f"{c['name']}: accepted, the golden refuses it")
        got = pycvc.nav_belief_material_contract(c["meta"])
        for k, v in c["values"].items():
            assert got[k] == v, (c["name"], k, got[k], v)
        assert got["meta"] == c["to_meta"], (c["name"], got["meta"], c["to_meta"])
        n_ok += 1
    assert pycvc.nav_belief_material_contract_to_meta() == G["net"]["meta"]
    try:
        pycvc.nav_belief_material_contract_to_meta(0.0)
        raise AssertionError("a zero scale was accepted")
    except ValueError:
        pass
    print(f"  contract: {n_ok} accepted and {n_bad} refused, as the golden")


def test_header_check():
    for c in G["header_check"]:
        b = c["belief"]
        if b["seen_rule"] != "raycast":
            continue  # the binding takes a raycast belief only
        args = (CVCNM, b["unknown_risk"], b["unknown_hard"], b["unknown_gate_hard"],
                b["material_range_m"], c["patch_size"], c["patch_extent_m"])
        try:
            got = pycvc.nav_belief_material_check_cvcnm(*args)
        except ValueError as e:
            assert c["mismatches"], (c["name"], str(e))
            for field in c["mismatches"]:
                assert f"{field}=" in str(e), (c["name"], field, str(e))
            continue
        assert not c["mismatches"], (c["name"], "accepted")
        assert got["meta"] == G["net"]["meta"]
    print("  header check: the golden's verdicts")


def test_obstacles_and_tokens():
    R, C = SCENE["grid"]["rows"], SCENE["grid"]["cols"]
    b = SCENE["grid"]["bounds"]
    truth = np.array(SCENE["truth"], np.uint8).reshape(R, C)
    obs = {}
    for o in G["obstacles"]:
        lv = np.array(o["last_visible"], np.uint8).reshape(R, C)
        x, y = o["position"]
        for case in o["cases"]:
            c, r, w = pycvc.nav_visible_obstacles(lv, truth, *b, x, y, case["max_obstacles"])
            assert c.tobytes() == f32(case["centers"]).tobytes(), (o["tick"], o["agent"])
            assert r.tobytes() == f32(case["radii"]).tobytes()
            assert w.tobytes() == f32(case["weights"]).tobytes()
        first = o["cases"][0]
        obs[(o["tick"], o["agent"])] = (
            f32(first["centers"]).reshape(-1, 2), f32(first["radii"]), f32(first["weights"]))
    for t in G["obstacle_ties"]:
        ones = np.ones((t["rows"], t["cols"]), np.uint8)
        c, _, _ = pycvc.nav_visible_obstacles(ones, ones, *t["bounds"], *t["position"],
                                              t["max_obstacles"])
        assert c.tobytes() == f32(t["centers"]).tobytes()

    pins = {c["name"]: c["params"] for c in SCENE["cases"]}
    planes = {(p["case"], p["tick"], p["plane"]): p for p in G["planes"]}
    P, ext = G["net"]["patch_size"], G["net"]["patch_extent_m"]
    for tk in G["tokens"]:
        pl = planes[(tk["case"], tk["tick"], tk["plane"])]
        risk = f32(pl["risk"]).reshape(R, C)
        hard = np.array(pl["hard_b"], np.uint8).reshape(R, C)
        seen = np.array(pl["material_seen"], np.uint8).reshape(R, C)
        if tk["variant"] == "no_obstacles":
            c, r, w = np.empty((0, 2), np.float32), np.empty(0, np.float32), np.empty(0, np.float32)
        else:
            c, r, w = obs[(tk["tick"], tk["agent"])]
        pin = pins[tk["case"]]
        of, om, gf, rp = pycvc.nav_belief_material_controller_inputs(
            *tk["position"], *tk["goal"], c, r, w, risk, hard, seen, *b, ext, P,
            tk["feature_scale_m"], pin["unknown_risk"], pin["unknown_hard"])
        what = (tk["case"], tk["tick"], tk["agent"], tk["variant"])
        assert of.shape == (tk["n_obstacles"], 6) and om.all(), what
        assert of.tobytes() == f32(tk["obs_feats"]).tobytes(), what
        want = f32(tk["goal_feats"])
        assert gf[[0, 1, 3]].tobytes() == want[[0, 1, 3]].tobytes(), what
        assert abs(int(gf.view(np.int32)[2]) - int(want.view(np.int32)[2])) <= 1, what
        assert rp.shape == (3, P, P) and rp.tobytes() == f32(tk["risk_patch"]).tobytes(), what
    print(f"  obstacles and tokens: {len(G['obstacles'])} agent ticks and {len(G['tokens'])} "
          "token sets equal the golden, bit for bit")


if __name__ == "__main__":
    test_contract()
    test_header_check()
    test_obstacles_and_tokens()
    print("pycvc belief-material controller bindings: OK")
