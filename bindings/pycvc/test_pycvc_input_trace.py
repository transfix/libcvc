#!/usr/bin/env python3
"""pycvc's input trace (cvc/core/input_trace.h, cvc/nav/sim_world_trace.h): record and replay.

Checks:
  1. a live session on a small kinematic world, recorded from the state tree (pycvc.state_set
     between frames) and nav's verbs (nav_trace_retarget, nav_trace_add_obstacle), writes a
     cvc.input_trace/v1 file that reads back as the same lines;
  2. nav_trace_replay reproduces it headlessly: every state hash it holds matches, the end's
     among them;
  3. a Python host loop (input_trace_player quantum by quantum, the world rebuilt by
     nav_sim_world_from_trace) ends on the live world's poses, byte for byte;
  4. a perturbed trace departs, no earlier than the record perturbed;
  5. errors: a file that is not a trace, a world whose own policy is unstated.
Script-style, like the other pycvc tests; self-skips (exit 0) when the running pycvc predates the
bindings.
"""

import json
import os
import tempfile

import numpy as np
import pycvc

if not hasattr(pycvc, "input_trace_recorder") or not hasattr(pycvc, "nav_trace_replay"):
    print("pycvc lacks input_trace_recorder / nav_trace_replay — skipping")
    raise SystemExit(0)

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


def expect_raises(fn, what, needle=""):
    try:
        fn()
    except RuntimeError as e:
        if needle and needle not in str(e):
            raise AssertionError(f"{what}: raised {e!r}, not about {needle!r}")
        return
    raise AssertionError(what + ": no error raised")


BOUNDS = (-40.0, -40.0, 40.0, 40.0)  # 81 x 81 one-metre cells
SCALE = 0.05  # normalized units per metre
OCC = np.zeros((81, 81), np.uint8)
OCC[30:50, 38:43] = 1  # a wall to drive round


def make_sim():
    """Three agents round a wall, sensing on (sense_every 4)."""
    starts = np.array([[-1.5, 0.3], [-1.5, -0.4], [1.2, 1.2]], np.float32)
    goals = np.array([[1.5, 0.3], [1.5, -0.2], [-1.2, -1.0]], np.float32)
    return pycvc.nav_sim_world_create(
        OCC, OCC, "", starts, goals, np.ones((3, 3), np.float32), 81, 81, *BOUNDS,
        0.0, 0.0, SCALE, 30.0, 90, 6.283185307179586, 0.15, 0.35, 0.06, 0.4, 0.165, 0.6, 0.12,
        0.12, 0.8, 6, 1, 0.2, 4, 0, 2.2, -1.4, 8.0, 1, 0.5, 0.15, 4.0)


def snapshot(sim):
    pos, heading, speed, mode, reached = pycvc.nav_sim_world_snapshot(sim)
    return np.concatenate([pos.ravel(), heading, speed]).astype(np.float32)


# 1. record a live session
tmp = tempfile.mkdtemp(prefix="pycvc_trace_")
path = os.path.join(tmp, "session.trace")
app = pycvc.App()
clock = app.world_clock()
clock.set_config(0.01, 1.0, 100)
sim = make_sim()
pycvc.nav_sim_world_pull(sim, app, "")  # the knobs into the tree, so the header holds them
header = pycvc.nav_trace_header(sim, "py:wall81", seed=11, policies={0: ""})
check(json.loads(header.to_json())["world_id"] == "py:wall81", "the header names the world")
rec = pycvc.input_trace_recorder(app, clock)
rec.start(header, path)
check(rec.recording(), "recording")
nav = pycvc.clock_follower.for_step(clock, 0.06)
frames = [1 / 60, 1 / 30, 1 / 144, 0.05, 1 / 90]
for f in range(400):
    if f == 50:
        pycvc.state_set(app, "nav.vehicle.vmax", "5")
    if f == 120:
        check(pycvc.nav_trace_retarget(sim, rec, 2, 1.4, 1.0), "retarget agent 2")
    if f == 160:
        pycvc.nav_trace_add_obstacle(sim, rec, 10, 14, 60, 64)
    if f == 200:
        pycvc.state_set(app, "nav.vehicle.vmax", "7")
    clock.advance(frames[f % len(frames)])
    for _ in range(nav.take(clock)):
        pycvc.nav_sim_world_pull(sim, app, "")
        pycvc.nav_sim_world_step(sim, 1)
    pycvc.nav_sim_world_publish(sim, app, "")
    rec.check(pycvc.nav_trace_state_hash(sim))
rec.finish(pycvc.nav_trace_state_hash(sim))
check(rec.finished() and not rec.recording(), "finished")
live = snapshot(sim)
t = rec.trace()
kinds = [t.records[k].kind_name() for k in range(t.record_count())]
check(kinds.count("set") == 2 and kinds.count("event") == 2, f"two sets, two events: {kinds[:6]}")
sets = [t.records[k].path for k in range(t.record_count()) if t.records[k].kind_name() == "set"]
check(sets == ["nav.vehicle.vmax", "nav.vehicle.vmax"], f"only the UI's writes: {sets}")
check(t.end.present and t.end.hash == pycvc.nav_trace_state_hash(sim), "the end holds the hash")
back = pycvc.input_trace.read(path)
check(back.str() == t.str(), "the file reads back as the same lines")
check(pycvc.input_trace.parse(t.str()).str() == t.str(), "and parses back from text")
check(pycvc.input_trace.schema() == "cvc.input_trace/v1", "the schema")
info = pycvc.input_trace_describe(back)
check("py:wall81 (cvc.nav.sim_world/v1, complete)" in info, info)
h = json.loads(back.header.to_json())
check(h["seed"] == 11 and len(h["agents"]) == 3 and h["agents"][0]["policy"] == "default",
      "the header's seed and agents")
check(h["settings"]["nav.range_m"] == "30", "the settings at the start")

# 2. the headless replay
r = pycvc.nav_trace_replay(pycvc.App(), back, keep_poses=True)
check(r["matched"] and not r["diverged"], r["report"])
check(r["hash"] == t.end.hash and r["compared"] == r["agreed"] > 100, r["report"])
check(r["poses"].shape == (r["steps"], 3, 4), f"poses {r['poses'].shape}")
check(r["pose_ticks"][:3] == [6, 12, 18], "a step every 6 quanta")

# 3. a Python host loop
app2 = pycvc.App()
clock2 = app2.world_clock()
sim2 = pycvc.nav_sim_world_from_trace(back.header)
play = pycvc.input_trace_player(app2, clock2, back)
play.rewind()
check(clock2.mode_name() == "replay", "the clock is in replay mode")
f2 = pycvc.clock_follower(6, 0, clock2.tick())
applied = []
while not play.done():
    for e in play.apply_due():
        check(pycvc.nav_trace_apply_event(sim2, e), f"nav's event {e.name}")
        applied.append((e.tick, e.name))
    play.next_quantum()
    for _ in range(f2.take(clock2)):
        pycvc.nav_sim_world_pull(sim2, app2, "")
        pycvc.nav_sim_world_step(sim2, 1)
    if play.expected_hash():
        check(play.verify(pycvc.nav_trace_state_hash(sim2)), f"tick {clock2.tick()}")
check([n for _, n in applied] == ["nav.retarget", "nav.add_obstacle"], f"events {applied}")
check(play.matched(), "the host loop matches the trace")
check(np.array_equal(snapshot(sim2), live) and snapshot(sim2).tobytes() == live.tobytes(),
      "byte-identical poses")

# 4. perturbed
bent = pycvc.input_trace.parse(t.str())
at = None
for k in range(bent.record_count()):
    rk = bent.records[k]
    if rk.kind_name() == "event" and rk.name == "nav.retarget":
        rk.args = '{"agent":2,"goal":[1.4,-1.0]}'
        bent.records[k] = rk
        at = rk.tick
check(at is not None, "found the retarget")
rb = pycvc.nav_trace_replay(pycvc.App(), bent)
check(not rb["matched"] and rb["diverged"] and rb["first_divergence"] > at, rb["report"])

# 5. errors
expect_raises(lambda: pycvc.input_trace.parse("not a trace\n"), "not a trace", "line 1")
expect_raises(lambda: pycvc.nav_trace_header(make_sim(), "x"), "policy unstated",
              "the world's own policy is not stated")
expect_raises(lambda: pycvc.input_trace_recorder(None, clock), "no app")

os.remove(path)
os.rmdir(tmp)
print(f"test_pycvc_input_trace: {checks} checks passed")
