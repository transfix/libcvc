#!/usr/bin/env python3
"""pycvc's CoPH (cvc/nav/coph_*.h, pycvc_nav_coph.i; C6 of docs/roadmap/NAV-COPH-ROADMAP.md).

Checks, against GRL-SNAM's golden (src/cvc/tests/data/coph_golden.json, the file libcvc's own
CoPH tests read; doubles are raw bits, 16 hex digits):
  1. G2, the plant: nav_coph_flow and nav_coph_balance on the 40 cases, at the file's tier;
  2. G3, the potential: nav_coph_risk, nav_coph_potential, nav_coph_grad_potential,
     nav_coph_command and nav_coph_shaping's alpha and bound on the 50 cases;
  3. G11, the allocation: nav_coph_allocate on the 60 cases;
  4. the configuration dicts: the profile, overrides, an unknown key and a refused value;
  5. one learner (NavCophAgent) driven on the plant kernel: it observes, advances, acts and
     updates, and a packet it exports is the JSON it counts, which a second learner assimilates;
  6. the fleet (NavCophFleet) on the crossing with G12's world and noise, as libcvc's
     CophFleet.TheEnergyAccountStaysAtRoundOffOverALongRun runs it: both agents reach their goals
     with no hit, each energy account closes at round-off, and packets were sent and delivered;
  7. the driver (NavCophDriver) on a sim_world: with nothing attached the run is the sim's own, bit
     for bit; with two RoboRacer-sized cars attached, both pass each other and reach their goals.
Script-style, like the other pycvc tests; self-skips (exit 0) when the running pycvc predates the
bindings.
"""

import json
import math
import struct
from pathlib import Path

import pycvc

if not hasattr(pycvc, "NavCophFleet"):
    print("pycvc lacks NavCophFleet — skipping")
    raise SystemExit(0)

GOLDEN = Path(__file__).resolve().parents[2] / "src" / "cvc" / "tests" / "data" / "coph_golden.json"
golden = json.loads(GOLDEN.read_text())

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


def num(h):
    return struct.unpack("<d", int(h, 16).to_bytes(8, "little"))[0]


def nums(a):
    return [num(h) for h in a]


def at_tier(tier, mine, gold, what):
    allowed = float(tier["rtol"]) * abs(gold) + float(tier["atol"])
    check(abs(mine - gold) <= allowed, f"{what}: {mine!r} vs the golden's {gold!r}")


# 1. G2, the plant.
plant = golden["plant"]
for c in plant["cases"]:
    x, u, d, dt = nums(c["x"]), nums(c["u"]), num(c["d"]), num(c["dt"])
    out = c["out"]
    for i, (mine, gold) in enumerate(zip(pycvc.nav_coph_flow(x, u, d, dt), nums(out["flow"]))):
        at_tier(plant["tier"], mine, gold, f"{c['name']} flow[{i}]")
    b = pycvc.nav_coph_balance(x, u, d, dt)
    for key in ("work", "heat", "kinetic_change", "residual"):
        at_tier(plant["tier"], b[key], num(out[key]), f"{c['name']} {key}")
check(len(plant["cases"]) == 40, "G2 has 40 cases")

# 2. G3, the potential, its gradient and the command.
pot = golden["potential"]
width, limit = num(pot["rbf_width"]), num(pot["limit"])
scenes = []
for s in pot["scenes"]:
    flat = nums(s["centers"])
    scenes.append(({"centers": [flat[i:i + 2] for i in range(0, len(flat), 2)],
                    "weights": nums(s["weights"]), "width": width,
                    "obstacles": [nums(o) for o in s["obstacles"]]},
                   nums(s["policy_bounds"])))
progress = 0
for c in pot["cases"]:
    scene, bounds = scenes[c["scene"]]
    config = {"rbf_width": width, "policy_bounds": bounds, "shaping_mode": c["mode"],
              "shaping_k": num(pot["shaping"]["k"]), "shaping_ell": num(pot["shaping"]["ell"]),
              "shaping_eta": num(pot["shaping"]["eta"])}
    shaping = pycvc.nav_coph_shaping(scene, config)
    check((shaping is not None) == (c["mode"] == "progress"), f"{c['name']}: shaping for its mode")
    q, v, target, policy = nums(c["q"]), nums(c["v"]), nums(c["target"]), nums(c["policy"])
    out, tier = c["out"], pot["tier"]
    at_tier(tier, pycvc.nav_coph_risk(q, scene), num(out["risk"]), f"{c['name']} risk")
    at_tier(tier, pycvc.nav_coph_potential(q, target, policy, scene, shaping), num(out["potential"]),
            f"{c['name']} potential")
    grad = pycvc.nav_coph_grad_potential(q, target, policy, scene, shaping)
    cmd = pycvc.nav_coph_command(q + v, target, policy, scene, shaping, limit=limit)
    for key, mine in (("grad", grad), ("proposed", cmd["proposed"]), ("applied", cmd["applied"])):
        for i, gold in enumerate(nums(out[key])):
            at_tier(tier, mine[i], gold, f"{c['name']} {key}[{i}]")
    if shaping is not None:
        at_tier(tier, shaping["alpha"], num(out["alpha"]), f"{c['name']} alpha")
        at_tier(tier, shaping["bound"], num(out["bound"]), f"{c['name']} bound")
        progress += 1
check(len(pot["cases"]) == 50 and progress == 25, "G3 has 50 cases, 25 of them progress")

# 3. G11, the allocation.
alloc = golden["allocation"]
for c in alloc["cases"]:
    a = pycvc.nav_coph_allocate(nums(c["u"]), num(c["heading"]), num(c["speed"]), num(c["wheelbase"]),
                                num(c["steer_max"]), num(c["accel_max"]),
                                speed_floor=num(c["speed_floor"]))
    out = c["out"]
    for key in ("accel", "steer", "kappa"):
        at_tier(alloc["tier"], a[key], num(out[key]), f"{c['name']} {key}")
    for key in ("used", "unallocated"):
        for i, gold in enumerate(nums(out[key])):
            at_tier(alloc["tier"], a[key][i], gold, f"{c['name']} {key}[{i}]")
check(len(alloc["cases"]) == 60, "G11 has 60 cases")

# 4. The configuration dicts.
profile = pycvc.nav_coph_profile()
check(profile["controller_mode"] == "routed" and profile["shaping_mode"] == "progress"
      and profile["replay_update_cap"] == 1024, f"the demos' profile: {profile}")
cfg = pycvc.nav_coph_config({"steps": 60, "policy_bounds": (0.0, 5.0)})
check(cfg["steps"] == 60 and tuple(cfg["policy_bounds"]) == (0.0, 5.0)
      and cfg["dt"] == profile["dt"], f"overrides over the profile: {cfg}")
check(pycvc.nav_coph_communication_profile()["byte_budget"] == 4000000, "the communication profile")
for bad, what in ((lambda: pycvc.nav_coph_config({"stepz": 1}), "an unknown key"),
                  (lambda: pycvc.nav_coph_config({"dt": -1.0}), "a refused value"),
                  (lambda: pycvc.NavCophFleet({"controller_mode": "legacy"}, {}, []),
                   "a mode outside the profile"),
                  (lambda: pycvc.nav_coph_flow([1, 2, 3], [0, 0], 0.5, 0.1), "a 3-vector state")):
    try:
        bad()
    except (RuntimeError, ValueError):
        checks += 1
    else:
        raise AssertionError(f"{what} did not raise")

# 5. One learner, driven on the plant kernel, and a packet between two.
DAMPING, DT, R = 0.85, 0.12, 1.25
LATTICE = [(a, b) for a in (-1.0, -0.5, 0.0, 0.5, 1.0) for b in (-1.0, -0.5, 0.0, 0.5, 1.0)
           if a * a + b * b <= 1.0]


def risk(qx, qy):
    return 0.02 + 0.5 * math.exp(-((qx - 4.0) ** 2 + (qy - 4.4) ** 2) / 0.5)


def observation(seed, tick, x):
    now = tick * DT
    return {"time": now, "sensor_center": x[:2], "sensor_radius": R,
            "readings": [{"uid": f"{seed}:{tick}:{j}", "time": now, "q": [x[0] + a * R, x[1] + b * R],
                          "value": max(0.001, risk(x[0] + a * R, x[1] + b * R))}
                         for j, (a, b) in enumerate(LATTICE)]}


def drive(agent, seed, x, ticks):
    held = None
    for tick in range(ticks):
        agent.observe(observation(seed, tick, x), [], held)
        agent.advance(x)
        if tick > 0 and tick % 8 == 0:
            r = agent.update(x)
            check(isinstance(r["accepted"], bool) and r["reason"] != "", f"an update report: {r}")
        u = agent.action(x)
        after = pycvc.nav_coph_flow(x, u, DAMPING, DT)
        held = {"uid": tick, "time": tick * DT, "x": x, "u": u, "after": after,
                "version": agent.state()["version"]}
        x = after
    return x


cfg = {"steps": 400}
a = pycvc.NavCophAgent(cfg, {}, [0.0, 4.0, 0.0, 0.0], [12.0, 4.0], "0", seed=3)
b = pycvc.NavCophAgent(cfg, {}, [12.0, 4.0, 0.0, 0.0], [0.0, 4.0], "1", seed=4)
xa = drive(a, 100, [0.0, 4.0, 0.0, 0.0], 48)
xb = drive(b, 200, [12.0, 4.0, 0.0, 0.0], 48)
check(xa[0] > 0.5, f"agent 0 moves toward its goal: {xa}")
state = a.state()
check(state["stages"] and state["version"] >= 0 and len(state["policy"]) == 2, f"its state: {state}")
meta = a.last_control()
check(meta["reason"] in ("nominal", "projected", "brake", "geometry_overflow"), f"its control: {meta}")
packet = a.export_to(xa, "1")
check(packet is not None, "agent 0 exports a packet")
body = packet.json()
check(packet.bytes() == len(body.encode()), "bytes() is the JSON's length (D9)")
parsed = json.loads(body)
check(list(parsed) == sorted(parsed) and parsed["sender"] == "0" and parsed["recipient"] == "1"
      and "geometry" in parsed, f"the packet's keys: {sorted(parsed)}")
check(packet.identity().startswith("0:") and packet.anchors() > 0, "its identity and anchors")
event = b.assimilate(packet)
check(event["recipient"] == "1" and event["packet"] == packet.identity() and event["reason"] != "",
      f"agent 1 assimilates it: {event}")
car = pycvc.NavCophAgent(cfg, {}, [0.0, 0.0, 0.0, 0.0], [4.0, 0.4], "car", network="none", seed=5)
drive(car, 300, [0.0, 0.0, 0.0, 0.0], 16)
check(car.last_control()["route_expansions"] >= 0, "a car's learner routes without a network")

# 6. The fleet on the crossing with G12's world and noise, until both agents are done.
world = golden["episode"]["world"]
fleet = pycvc.NavCophFleet({"steps": 2000}, {},
                           [{"initial": [0.0, 4.0, 0.0, 0.0], "goal": [12.0, 4.0]},
                            {"initial": [12.0, 4.0, 0.0, 0.0], "goal": [0.0, 4.0]}],
                           senders=[0, 1], seed=7)
fleet.run_crossing({"obstacles": [nums(o) for o in world["obstacles"]], "seeds": [152000, 152001],
                    "noise": lambda agent, tick, j:
                    0.003 * (((7 * tick + 13 * j + 5 * agent) % 11) - 5) / 5.0})
for i in range(2):
    e = fleet.energy(i)
    check(fleet.success(i) and not fleet.hit(i), f"agent {i} reaches its goal with no hit")
    check(e["steps"] == len(fleet.controls(i)) and e["work"] > 1.0, f"agent {i}'s account: {e}")
    check(abs(e["residual"]) <= 1e-9 * (e["work"] + e["heat"]), f"agent {i}'s residual: {e}")
    print(f"agent {i}: at its goal after {e['steps']} steps; work {e['work']:.6g}, "
          f"heat {e['heat']:.6g}, residual {e['residual']:.3g}")
kinds = {ev["kind"] for ev in fleet.events()}
check({"sent", "delivered"} <= kinds, f"packets were sent and delivered: {kinds}")
check(fleet.samples(0)[-1]["done"] and fleet.updates(0), "the samples and updates")

# 7. The driver (NavCophDriver, coph_driver.h): CoPH drives a sim_world's cars through
# override_command, as libcvc's CophDriver.TwoCarsCrossTheField: two RoboRacer-sized cars (s = 1)
# pass each other on a walled 20 m x 12 m field of 0.1 m cells.
if hasattr(pycvc, "NavCophDriver"):
    import numpy as np

    W, H, CELL, S = 20.0, 12.0, 0.1, 0.1
    rows, cols = int(H / CELL) + 1, int(W / CELL) + 1
    occ = np.zeros((rows, cols), np.uint8)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = 1
    cx, cy = W / 2, H / 2

    def field_sim():
        starts = np.array([[3.0, 5.0], [17.0, 7.0]])
        goals = np.array([[17.0, 5.4], [3.0, 6.6]])
        on = ((starts - [cx, cy]) * S).astype(np.float32)
        goal = ((goals - [cx, cy]) * S).astype(np.float32)
        sim = pycvc.nav_sim_world_create(
            occ, occ, "", on, goal, np.ones((2, 3), np.float32), rows, cols, 0.0, 0.0, W, H, cx, cy,
            S, 60.0, 120, 2 * math.pi, 0.25 * S, 0.35, 0.06, 3.0 * S, 0.33 * S, 0.42, 3.0 * S,
            0.8 * 9.80665 * S, 0.8, 6, 1, 0.3 * S, 4, 1, 2.2, -1.4, 8.0, 1, 0.5, 0.15, 4.0)
        pycvc.nav_sim_world_set_pose(sim, 0, float(on[0, 0]), float(on[0, 1]), 0.0)
        pycvc.nav_sim_world_set_pose(sim, 1, float(on[1, 0]), float(on[1, 1]), math.pi)
        return sim

    # Nothing attached: the run is the sim's own, bit for bit.
    plain, with_driver = field_sim(), field_sim()
    idle = pycvc.NavCophDriver(with_driver, {}, {}, seed=31)
    same = True
    for _ in range(60):
        pycvc.nav_sim_world_step(plain)
        idle.apply()
        pycvc.nav_sim_world_step(with_driver)
        a, b = pycvc.nav_sim_world_snapshot(plain)[:3], pycvc.nav_sim_world_snapshot(with_driver)[:3]
        same = same and all(np.array_equal(x, y) for x, y in zip(a, b))
    check(same and idle.steps() == 0 and idle.attached() == [], "a driver with nothing attached")

    sim = field_sim()
    drive = pycvc.NavCophDriver(sim, None, None, froude=1.0, seed=31)
    drive.attach(0)
    drive.attach(1)
    check(drive.attached() == [0, 1] and drive.ticks_per_step() == 2, "CoPH's cadence at s = 1")
    ticks = 0
    while ticks < 3000 and not (drive.done(0) and drive.done(1)):
        drive.apply()
        pycvc.nav_sim_world_step(sim)
        ticks += 1
    for v in range(2):
        acc = drive.account(v)
        check(drive.done(v), f"car {v} reaches its goal ({ticks} ticks)")
        check(acc["steps"] > 10 and acc["work"] > 0.0 and 0.0 < acc["allocation_residual"] < 1.0,
              f"car {v}'s account: {acc}")
        check(drive.controls(v) and drive.controls(v)[-1]["meta"]["reason"] != "", f"car {v}'s controls")
    check(any(e["kind"] == "delivered" for e in drive.events()), "packets between the cars")
    print(f"driver: both cars at their goals after {ticks} ticks ({ticks * 0.06:.1f} s), "
          f"{drive.steps()} CoPH steps")
    try:
        drive.account(2)
    except (RuntimeError, IndexError, ValueError):
        checks += 1
    else:
        raise AssertionError("an unattached vehicle's account did not raise")

print(f"pycvc nav coph: {checks} checks passed")
