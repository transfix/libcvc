#!/usr/bin/env python3
"""pycvc_gl.nav_lab: libcvc's navigation policies side by side, and recorded runs played back.

Headless, in this process (no GL):
  1. --arms parses: default and NAME=PATH.cvcnav, with each file's sha256; bad ones say why;
  2. `arms` on the flat world, kinematic: three arms step together, the log names each policy, the
     windows' tables are in the document's tree, and --retarget-at starts a leg on one tick in
     every arm;
  3. the trained arm is the cars template's navigator on that policy alone, tick for tick;
  4. `replay` shows at a tick what the live run showed there, cell for cell, and its transport
     steps, seeks and restarts;
  5. refusals: an arm that is not there, a file that is not a run log;
  6. scrub: a history's checkpoints deployed headless, its report; a grid world's walls as boxes.
Offscreen (when GL is there): the ghosts and the windows, to a PNG.
Script-style, like the other pycvc tests; self-skips (exit 0) when the running pycvc predates the
bindings.
"""

import json
import os
import subprocess
import sys
import tempfile

import numpy as np
import pycvc

if not hasattr(pycvc, "nav_arms_create"):
    print("pycvc lacks nav_arms_create — skipping")
    raise SystemExit(0)

from pycvc_gl import cars as C  # noqa: E402
from pycvc_gl import nav_lab as L  # noqa: E402

if not C.has_cars():
    print("pycvc lacks plant_host — skipping")
    raise SystemExit(0)

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


HERE = os.path.dirname(os.path.abspath(__file__))
TRAINED = os.path.normpath(os.path.join(HERE, "..", "..", "share", "cvc", "nav", "coef_mlp.cvcnav"))
os.environ.setdefault("CVC_ARIADNE_PATH", os.path.normpath(os.path.join(HERE, "..", "..", "src",
                                                                        "cvc", "ariadne")))
tmp = tempfile.mkdtemp(prefix="pycvc_gl_nav_lab_")

# 1. the arms
arms = L.parse_arms(f"default,trained={TRAINED}", [f"twin={TRAINED}"])
check([a["name"] for a in arms] == ["default", "trained", "twin"], "the arms in order")
check(arms[0]["uri"] == "" and len(arms[1]["sha256"]) == 64, "default has no file; trained one")
for bad, why in (("nope", "an arm is"), ("x=/no/such.cvcnav", "no policy"),
                 (f"default,a={TRAINED},a={TRAINED}", "named twice"), ("", "names no arm")):
    try:
        L.parse_arms(bad)
        raise AssertionError(f"{bad!r} was taken")
    except ValueError as e:
        check(why in str(e), f"{bad!r}: {e}")
check(L.parse_point("3, -4") == (3.0, -4.0) and L.parse_point("x") is None, "parse_point")

# 2. a live arms run, headless
log_path = os.path.join(tmp, "run.cvcrun")
code = L.main(["arms", "--headless", "--frames", "120", "--world", "flat", "--plant", "kinematic",
               "--arms", f"default,trained={TRAINED}", "--arm", f"twin={TRAINED}", "--cars", "2",
               "--retarget-at", "3.0", "--log", log_path])  # fmt: skip
check(code in (0, 3), f"arms ran: exit {code}")
log = pycvc.nav_run_log_read(log_path)
head = json.loads(pycvc.nav_run_log_header(log))
check([a["name"] for a in head["arms"]] == ["default", "trained", "twin"], "the log's arms")
check(head["arms"][1]["policy"]["sha256"] == arms[1]["sha256"], "the trained arm's sha256")
check(head["plant"] == "kinematic" and head["agents"] == 2, "kinematic, two agents per arm")
check(head["world"]["run"]["world"] == "flat", "the world's keys")
check("nav.vehicle.vmax" in head["settings"], "the settings")
check(pycvc.nav_run_log_ticks(log) == 120, "a tick a frame, headless")
a = pycvc.nav_run_log_arrays(log)
legs = pycvc.nav_run_log_legs(log)
R = round(3.0 / C.DT)  # the retarget's tick: 3 s on the sim tick
check([g["tick"] for g in legs] == [0, R], f"a leg from 3 s: {[g['tick'] for g in legs]}")
check(np.all(a["goal_x"][R] == a["goal_x"][R, 0]), "every arm's goals moved on the same tick")
check(np.array_equal(a["x"][:, 1], a["x"][:, 2]), "the same policy twice: the same run")
check(np.abs(a["book.residual"]).max() < 1e-5, "the books close")

# 3. the trained arm is the template's navigator on that policy alone
world = C.flat_world(4.84)
starts, goals = L.starts_and_goals(world, 2, 0)
check(np.allclose(head["start"][0][:2], starts[0], atol=1e-3), "the arms' start is the seed's")
alone = C.Cars(world, 2, "kinematic", navigator=True, nav_weights=TRAINED, starts=starts,
               goals=goals)  # fmt: skip
xs = [alone.step_navigator().x.copy() for _ in range(R)]
check(np.array_equal(np.array(xs), a["x"][:R, 1]), "the trained arm, tick for tick, alone")

# 4. the replay shows what the live run showed
from pycvc_gl.ariadne import Host  # noqa: E402

host = Host(mode="headless", name="nav_lab", component_paths=[C.COMPONENTS])
host.load(str(L.REPLAY_DOC))
host.set("ledger.agent", "trained")
host.set("sim.paused", "1")
view = pycvc.nav_replay_view_create(log, 2.5)
f = pycvc.nav_replay_view_update(view, host.app, host.prefix, 0.0)
want = pycvc.nav_run_log_tables(log, f["k"], 0, 1)
for key in ("arms.stats", "ledger.coef", "ledger.books", "ledger.plant"):
    check(host.get_tree(key) == want[key], f"{key} at tick {f['k']}, cell for cell")
code = L.main(["replay", log_path, "--headless", "--frames", "12", "--at", "1.0"])
check(code == 0, f"replay ran: exit {code}")

# 4b. scrub: a history's checkpoints, headless and with a report
if hasattr(pycvc, "nav_scrub_create"):
    import shutil

    hdir = os.path.join(tmp, "train")
    os.makedirs(hdir)
    shutil.copy(TRAINED, os.path.join(hdir, "coef.step000000.cvcnav"))
    with open(os.path.join(hdir, "history.jsonl"), "w") as fh:
        fh.write('{"kind":"header"}\n{"kind":"step","step":1,"loss":1.0}\n'
                 '{"kind":"checkpoint","step":0,"blob":"coef.step000000.cvcnav"}\n')  # fmt: skip
    report = os.path.join(tmp, "scrub.json")
    code = L.main(["scrub", os.path.join(hdir, "history.jsonl"), "--headless", "--frames", "5",
                   "--agents", "2", "--ticks", "40", "--report", report])  # fmt: skip
    check(code == 0, f"scrub ran: exit {code}")
    check(json.load(open(report))["schema"] == "cvc.nav.scrub/v1", "the scrub report")
    check(L.main(["scrub", os.path.join(tmp, "none.jsonl"), "--headless"]) == 2, "no history")
    occ = np.zeros((5, 7), np.uint8)
    occ[2, 1:4] = 1
    w = L.grid_world(occ, (0.0, 0.0, 6.0, 4.0))
    check(len(w.boxes) == 1 and w.boxes[0][:4] == (0.5, 1.5, 3.5, 2.5), f"a run, a box: {w.boxes}")

# 5. refusals
check(L.main(["arms", "--headless", "--arms", "x=/no/such.cvcnav"]) == 2, "no such arm: exit 2")
bad = os.path.join(tmp, "bad.cvcrun")
with open(bad, "w") as fh:
    fh.write("{}\n")
check(L.main(["replay", bad, "--headless"]) == 2, "not a run log: exit 2")
check(L.main([]) == 2 and L.main(["what"]) == 2, "no command: exit 2")

# offscreen: the ghosts and the windows
png = os.path.join(tmp, "arms.png")
argv = ["arms", "--offscreen", "--frames", "40", "--world", "flat", "--arms",
        f"default,trained={TRAINED}", "--png", png]
r = subprocess.run([sys.executable, "-c", "import sys; from pycvc_gl import nav_lab; "
                    f"sys.exit(nav_lab.main({argv!r}))"],
                   capture_output=True, text=True, timeout=600, cwd=tmp)  # fmt: skip
if r.returncode == 0 and os.path.exists(png):
    check(os.path.getsize(png) > 1000, "the offscreen frame")
else:
    print(f"offscreen skipped (no GL here?): {r.returncode} {r.stderr[-400:]}")

print(f"test_pycvc_gl_nav_lab: {checks} checks passed")
