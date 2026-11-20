#!/usr/bin/env python3
"""pycvc's nav lab (cvc/nav/nav_arms.h, run_log.h, nav_panels.h; docs/NAV_LAB.md).

Checks:
  1. three arms (the default policy, libcvc's shipped trained one, the default again under
     another name) step in lockstep from one start; the run log's arrays have every column,
     (T, A, N) and (T, A);
  2. the books close: every tick's residual is 0 to float round-off;
  3. one retarget moves every arm's goal on the same tick and starts a leg;
  4. an arm runs bit for bit as its policy alone;
  5. the log streamed to a file reads back value for value, and the tables at a tick are the same;
  6. the plant ledger's items are bound (20 per vehicle); on the kinematic plant none applies;
  7. the windows land in an app's state tree, and a replay view moves its play head;
  8. errors: arms that cannot run together, a file that is not a run log;
  9. the checkpoint scrubber: a history's checkpoints deployed and cached, the report, the view.
Script-style, like the other pycvc tests; self-skips (exit 0) when the running pycvc predates the
bindings.
"""

import hashlib
import json
import math
import os
import tempfile

import numpy as np
import pycvc

if not hasattr(pycvc, "nav_arms_create"):
    print("pycvc lacks nav_arms_create — skipping")
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
    except (RuntimeError, ValueError, TypeError) as e:
        if needle and needle not in str(e):
            raise AssertionError(f"{what}: raised {e!r}, not about {needle!r}") from e
        return
    raise AssertionError(what + ": no error raised")


BOUNDS = (-40.0, -40.0, 40.0, 40.0)  # 81 x 81 one-metre cells
SCALE = 0.05
OCC = np.zeros((81, 81), np.uint8)
OCC[30:50, 38:43] = 1  # a wall to drive round
HERE = os.path.dirname(os.path.abspath(__file__))
TRAINED = os.path.normpath(os.path.join(HERE, "..", "..", "share", "cvc", "nav", "coef_mlp.cvcnav"))


def make_sim(n=1):
    starts = np.array([[-1.5, 0.3], [-1.5, -0.4], [1.2, 1.2]], np.float32)[:n]
    goals = np.array([[1.5, 0.3], [1.5, -0.2], [-1.2, -1.0]], np.float32)[:n]
    return pycvc.nav_sim_world_create(
        OCC, OCC, "", starts, goals, np.ones((n, 3), np.float32), 81, 81, *BOUNDS,
        0.0, 0.0, SCALE, 30.0, 90, 6.283185307179586, 0.15, 0.35, 0.05, 0.4, 0.165, 0.6, 0.12,
        0.12, 0.8, 6, 1, 0.2, 4, 0, 2.2, -1.4, 8.0, 1, 0.5, 0.15, 4.0)  # fmt: skip


def sha(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


lib = pycvc.nav_policy_library_create()
has_trained = os.path.exists(TRAINED)
policy = {"name": "trained", "uri": TRAINED, "sha256": sha(TRAINED)} if has_trained else None


def fleet(n=1):
    worlds = [make_sim(n) for _ in range(3)]
    if has_trained:
        for i in range(n):
            pycvc.nav_sim_world_set_policy(worlds[1], i, lib, TRAINED, policy["sha256"])
    arms = ["default", policy or "other", "again"]
    return worlds, arms


tmp = tempfile.mkdtemp(prefix="pycvc_nav_lab_")
path = os.path.join(tmp, "run.cvcrun")

# 1. three arms in lockstep
worlds, arms = fleet(2)
run = pycvc.nav_arms_create(worlds, arms, seed=3, scene="wall", world=json.dumps({"kind": "wall"}),
                            log_path=path)  # fmt: skip
check(pycvc.nav_arms_count(run) == 3 and pycvc.nav_arms_agents(run) == 2, "three arms of two")
for _ in range(80):
    pycvc.nav_arms_step(run)
log = pycvc.nav_arms_log(run)
check(pycvc.nav_arms_tick(run) == 80 and pycvc.nav_run_log_ticks(log) == 80, "80 ticks")
a = pycvc.nav_run_log_arrays(log)
check(a["x"].shape == (80, 3, 2), f"x is (T, A, N): {a['x'].shape}")
check(a["stats.path_m"].shape == (80, 3), "a Stats column is (T, A)")
check(a["t"].shape == (80,) and math.isclose(a["t"][-1], 80 * 0.05, rel_tol=1e-9), "t")
for col in ("tel.alpha", "tel.context_work", "book.residual", "plant.energy_J", "ms", "length"):
    check(col in a, f"the arrays have {col}")
head = json.loads(pycvc.nav_run_log_header(log))
check(head["schema"] == "cvc.nav.run_log/v1" and head["scene"] == "wall", "the header")
check([x["name"] for x in head["arms"]] == ["default", arms[1] if isinstance(arms[1], str)
                                            else "trained", "again"], "the arms' names")  # fmt: skip
check(np.all(a["x"][0, 0] == a["x"][0, 2]), "the default arms are the same run")
if has_trained:
    check(head["arms"][1]["policy"]["sha256"] == policy["sha256"], "the trained arm's sha256")
    # its coefficients differ; on open ground both saturate toward the goal, so they drive alike
    check(not np.array_equal(a["tel.alpha"][:, 0], a["tel.alpha"][:, 1]), "the trained arm's alpha")

# 2. the books close
res = np.abs(a["book.residual"])
check(np.all(np.isfinite(res)) and res.max() < 1e-5, f"the books close: {res.max()}")

# 3. one retarget, every arm, the same tick
pycvc.nav_arms_retarget(run, np.array([[10.0, 20.0], [-10.0, -20.0]], np.float32))
for _ in range(10):
    pycvc.nav_arms_step(run)
pycvc.nav_arms_retarget_agent(run, 1, 5.0, 5.0)
for _ in range(10):
    pycvc.nav_arms_step(run)
pycvc.nav_arms_finish(run)
legs = pycvc.nav_run_log_legs(log)
check([g["tick"] for g in legs] == [0, 80, 90], f"three legs: {[g['tick'] for g in legs]}")
check(len(legs[0]["nav_stats"]) == 3 and json.loads(legs[0]["nav_stats"][0])["per_vehicle"],
      "each arm's nav_stats when the leg ended")  # fmt: skip
a = pycvc.nav_run_log_arrays(log)
check(np.allclose(a["goal_x"][80, :, 0], 10.0, atol=1e-4), "every arm's goal moved at tick 80")
check(np.allclose(a["goal_x"][90, :, 1], 5.0, atol=1e-4), "agent 1's alone at tick 90")
check(np.allclose(a["goal_x"][90, :, 0], 10.0, atol=1e-4), "agent 0 kept its goal")
check(a["leg_goal"].shape == (3, 2, 2), "the legs' goals")
check(pycvc.nav_run_log_leg_of(log, 85) == 1 and pycvc.nav_run_log_index(log, 1.0) == 19, "index")
check(np.abs(a["book.residual"]).max() < 1e-5, "the books close across the retargets")
check(np.all(a["book.context"][80] != 0.0), "a goal's change is context work")

# 4. an arm runs as its policy alone
alone = make_sim(2)
if has_trained:
    for i in range(2):
        pycvc.nav_sim_world_set_policy(alone, i, lib, TRAINED, policy["sha256"])
pycvc.nav_sim_world_set_capture_drive_telemetry(alone, 1)
xs = []
for _ in range(80):
    pycvc.nav_sim_world_step(alone)
    xs.append(pycvc.nav_sim_world_snapshot(alone)[0][:, 0].astype(float))
check(np.array_equal(np.array(xs), a["x"][:80, 1]), "the arm, tick for tick, is its policy alone")

# 5. the log on file, value for value
back = pycvc.nav_run_log_read(path)
b = pycvc.nav_run_log_arrays(back)
for k, v in a.items():
    check(np.array_equal(v, b[k], equal_nan=True), f"{k} reads back")
check(pycvc.nav_run_log_end(back) is not None and len(pycvc.nav_run_log_end(back)) == 3, "the end")
for k in (0, 40, 85, 99):
    check(pycvc.nav_run_log_tables(log, k, 1, 2) == pycvc.nav_run_log_tables(back, k, 1, 2),
          f"the tables at tick {k}, cell for cell")  # fmt: skip
t = pycvc.nav_run_log_tables(back)
check([r["arm"] for r in t["arms.stats"]] == [x["name"] for x in head["arms"]], "a row per arm")
check(t["arms.stats"][1]["colour"] == "orange", "arm 1 is slot c1, orange")
check("/2" in t["arms.stats"][0]["reached"], "two agents: reached k/2")
check(len(t["ledger.plant"]) == 20, "the plant ledger's 20 items")
copy = os.path.join(tmp, "copy.cvcrun")
pycvc.nav_run_log_write(back, copy)
with open(path) as f1, open(copy) as f2:
    check(f1.read() == f2.read(), "written again: the same text")

# 6. the plant ledger's items
items = pycvc.nav_plant_ledger_vehicle()
check(len(items) == 20 and items[0]["key"] == "energy_J" and items[0]["unit"] == "J",
      "the vehicle items")  # fmt: skip
check({i["kind"] for i in items} == {"measure", "count", "flag"}, "their kinds")
check(all(i["comment"] for i in items) and pycvc.nav_plant_ledger_episode(), "comments, episode")
pycvc.nav_sim_world_begin_nav_stats(alone)
pycvc.nav_sim_world_step(alone)
now = pycvc.nav_sim_world_plant_ledger(alone, 0)
check(len(now) == 20 and not any(i["applies"] for i in now) and now[0]["value"] is None,
      "on the kinematic plant no item applies")  # fmt: skip
check(all(r["value"] == "n/a" for r in t["ledger.plant"]), "the window says n/a")
check("no item applies" in t["ledger.plant_note"], t["ledger.plant_note"])

# 7. the windows in a state tree, and a replay
app = pycvc.App()
pycvc.state_set(app, "doc.ledger.agent", "again")
pycvc.state_set(app, "doc.ledger.book", "context")
c = pycvc.nav_run_log_publish(app, "doc", back, -1, charts=True)
check(c == {"arm": 2, "agent": 0, "book": "context"}, f"the choice: {c}")
check(pycvc.state_get(app, "doc.arms.stats") == "3", "arms.stats holds 3 rows")
check(pycvc.state_get(app, "doc.arms.stats.2.arm") == "again", "row 2")
check(pycvc.state_get(app, "doc.ledger.compare.s") == "3", "the compare chart: a series per arm")
check(pycvc.state_get(app, "doc.arms.clearance.m") == "2", "a marker per leg after the first")
charts = pycvc.nav_run_log_charts(back, 2, 0, "context")
check(len(charts["ledger.cum"]["series"]) == 6, "the books chart")
view = pycvc.nav_replay_view_create(back, 2.0)
pycvc.state_set(app, "doc.sim.paused", "1")
f = pycvc.nav_replay_view_update(view, app, "doc", 0.1)
check(f["k"] == 39 and f["poses"].shape == (3, 2, 6), f"the replay at 2 s: {f['k']}")
check(pycvc.state_get(app, "doc.arms.time") == "t = 2.0 s, tick 40 of 100, leg 1", "arms.time")
pycvc.nav_replay_view_step(view)
check(math.isclose(pycvc.nav_replay_view_time(view), 2.05, rel_tol=1e-9), "a step")
pycvc.state_set(app, "doc.play.seek", "4.5")
pycvc.nav_replay_view_seek(view, app, "doc")
f = pycvc.nav_replay_view_update(view, app, "doc", 0.0)
check(f["leg"] == 1, "seeked into leg 2")
pycvc.nav_replay_view_restart(view, app, "doc")
check(pycvc.nav_replay_view_time(view) == 0.0, "restart")
both = pycvc.nav_run_log_stack([back, back], [0, 1], ["step 0", "step 9"])
check(json.loads(pycvc.nav_run_log_header(both))["arms"][1]["name"] == "step 9", "stacked")
p = pycvc.nav_run_log_poses(both, 1.0)
check(p.shape == (2, 2, 6) and p[1, 0, 0] == a["x"][19, 1, 0], "the poses at 1 s")

# 8. errors
expect_raises(lambda: pycvc.nav_arms_create([make_sim(1), make_sim(2)], ["a", "b"]),
              "worlds that differ", "differ")  # fmt: skip
w = make_sim(1)
expect_raises(lambda: pycvc.nav_arms_create([w, w], ["a", "b"]), "a world twice", "share")
expect_raises(lambda: pycvc.nav_arms_create([make_sim(1)], [3]), "an arm that is not a name")
with open(os.path.join(tmp, "x.txt"), "w") as f:
    f.write('{"schema":"something/v1"}\n')
expect_raises(lambda: pycvc.nav_run_log_read(os.path.join(tmp, "x.txt")), "not a run log",
              "not a run log")  # fmt: skip
expect_raises(lambda: pycvc.nav_arms_step(run), "a finished run", "finished")

# 9. the checkpoint scrubber (cvc/nav/nav_scrub.h): a history naming two checkpoints
if hasattr(pycvc, "nav_scrub_create") and has_trained:
    import shutil

    hdir = os.path.join(tmp, "train")
    os.makedirs(hdir)
    shutil.copy(TRAINED, os.path.join(hdir, "a.cvcnav"))
    shutil.copy(TRAINED, os.path.join(hdir, "b.cvcnav"))
    hist = os.path.join(hdir, "history.jsonl")
    with open(hist, "w") as f:
        f.write('{"kind":"header","schema":"cvc.nav.coef_train_history/v1"}\n'
                '{"kind":"step","step":1,"loss":2.0}\n{"kind":"step","step":2,"loss":1.5}\n'
                '{"kind":"checkpoint","step":0,"fitness":0.2,"blob":"a.cvcnav"}\n'
                '{"kind":"checkpoint","step":2,"fitness":0.4,"blob":"b.cvcnav"}\n'
                '{"kind":"checkpoint","step":3,"blob":"missing.cvcnav"}\n'
                '{"kind":"end","selected_step":2}\n')  # fmt: skip
    sc = pycvc.nav_scrub_create(hist, agents=2, ticks=60)
    info = pycvc.nav_scrub_info(sc)
    check(len(info["checkpoints"]) == 2 and info["missing"] == ["missing.cvcnav"], "two of three")
    check(info["checkpoints"][1]["selected"] and info["scenario"] == "blocks, 2 agents, seed 0",
          "the pick, the scenario")  # fmt: skip
    check(not info["checkpoints"][0]["cached"] and info["checkpoints"][1]["cached"],
          "the same blob twice: deployed once, the second from the cache")  # fmt: skip
    check(os.path.isdir(hist + ".scrub") and len(info["lines"]) == 2, "the cache, the lines")
    check(set(info["checkpoints"][0]["metrics"]) >= {"reached", "path_m", "goal_m"}, "metrics")
    rep = json.loads(pycvc.nav_scrub_report(sc))
    check(rep["schema"] == "cvc.nav.scrub/v1" and len(rep["checkpoints"]) == 2, "the report")
    check(pycvc.nav_run_log_ticks(pycvc.nav_scrub_log(sc, 1)) == 60, "a checkpoint's log")
    occ, bounds = pycvc.nav_scrub_world(sc)
    check(occ.shape == (121, 121) and occ.any() and bounds == (-60.0, -60.0, 60.0, 60.0), "world")
    app2 = pycvc.App()
    f = pycvc.nav_scrub_view_update(sc, app2, "", 0.0)
    check(f["checkpoint"] == 1 and f["poses"].shape == (2, 2, 6), "the view starts at the pick")
    check(pycvc.state_get(app2, "scrub.labels") == "step 0\nstep 2 (pick)", "the labels")
    check(pycvc.state_get(app2, "scrub.curve_name") == "step.loss", "the training curve")
    pycvc.nav_scrub_prev(sc)
    f = pycvc.nav_scrub_view_update(sc, app2, "", 0.0)
    check(f["checkpoint"] == 0 and pycvc.state_get(app2, "arms.names") == "step 0\nstep 0 ",
          "previous: step 0 beside itself")  # fmt: skip
    pycvc.state_set(app2, "scrub.pick", "1.9")
    pycvc.nav_scrub_pick_step(sc, app2, "")
    check(pycvc.nav_scrub_checkpoint(sc) == 1, "a click on a chart picks the nearest step")
    pycvc.nav_scrub_step(sc)
    pycvc.nav_scrub_restart(sc, app2, "")
    expect_raises(lambda: pycvc.nav_scrub_log(sc, 5), "no such checkpoint")
    expect_raises(lambda: pycvc.nav_scrub_create(os.path.join(tmp, "none.jsonl")), "no history")
    # the plant: the Humvee by default (the bicycle without Jolt); several, and the Plant combo
    default = "humvee" if pycvc.plant_host.available() else "kinematic"
    check(info["plant"] == default and info["plants"] == [default], "the default plant")
    two = pycvc.nav_scrub_create(hist, agents=2, ticks=60, plants=["kinematic", "humvee"])
    app3 = pycvc.App()
    f = pycvc.nav_scrub_view_update(two, app3, "", 0.0)
    check(pycvc.state_get(app3, "scrub.plant_labels") == "kinematic\nhumvee" and f["plant"] == 0,
          "two plants, the first shown")  # fmt: skip
    pycvc.state_set(app3, "scrub.plant_choice", "humvee")
    f = pycvc.nav_scrub_view_update(two, app3, "", 0.0)
    check(f["plant"] == 1 and f["checkpoint"] == 1 and pycvc.nav_scrub_info(two)["plant"] == "humvee",
          "the Plant combo: the same checkpoint on the other plant")  # fmt: skip
    check(json.loads(pycvc.nav_scrub_report(two))["scenario"]["plant"] == "humvee", "its report")
    pycvc.nav_scrub_use_plant(two, 0)
    check(pycvc.nav_scrub_info(two)["plant"] == "kinematic", "use_plant")
    expect_raises(lambda: pycvc.nav_scrub_create(hist, plants=["no such"]), "an unknown plant")

print(f"test_pycvc_nav_lab: {checks} checks passed")
