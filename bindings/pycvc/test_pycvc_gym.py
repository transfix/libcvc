"""pycvc.WorldGym: a world document driven by reset, step and observe from Python, headless.

What a training script relies on (ARIADNE_WORLDS.md section 14, L4):
  * reset(seed) spawns the agents and returns their observation as numpy arrays;
  * step(steer, accel) runs one control tick and returns (obs, reward, terminated, truncated,
    info), as gymnasium does, with a reward per agent; NaN leaves an agent to nav@1;
  * one seed and one list of commands give one world, in two gyms side by side;
  * a reward written in Python is paid and logged; its exception comes back out of step();
  * the log is numpy arrays and a CSV; the keys under gym.* follow the episode;
  * agents as dicts (archetype, name, team, a start of their own), and refusals.
"""

import math
import os
import sys
import tempfile

import numpy as np

import pycvc

if not hasattr(pycvc, "WorldGym"):
    print("SKIP test_pycvc_gym: this pycvc has no WorldGym")
    sys.exit(0)
if not pycvc.AriApp.have_yaml():
    print("SKIP test_pycvc_gym: this libcvc reads no YAML documents")
    sys.exit(0)
if "jolt" not in pycvc.physics_backends() or not pycvc.physics_backend_info("jolt")["available"]:
    print("SKIP test_pycvc_gym: no Jolt in this build")
    sys.exit(0)

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


DOC = """requires: [scene.physics]
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee, controllers: [nav@1, none] }
      racer: { vehicle: roboracer, controllers: [none] }
scene:
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [120, 120, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
"""

tmp = tempfile.mkdtemp(prefix="pycvc_gym_")
doc = os.path.join(tmp, "gym.ari")
with open(doc, "w", encoding="utf-8") as f:
    f.write(DOC)

# ── reset, observe, step ──
gym = pycvc.WorldGym(doc, count=3, max_steps=50)
check(abs(gym.dt() - 0.05) < 0.01, f"a step is the navigator's tick ({gym.dt()})")
obs = gym.reset(seed=5)
check(obs["n"] == 3 and gym.agents() == 3, "three agents")
for k in ("x", "y", "yaw", "speed", "goal_x", "goal_dist", "max_speed"):
    check(isinstance(obs[k], np.ndarray) and obs[k].shape == (3,) and obs[k].dtype == np.float64,
          f"{k} is a (3,) float64 array")
check(obs["id"].dtype == np.uint64 and list(obs["id"]) == [1, 2, 3], "ids 1..3")
check(obs["reached"].dtype == np.bool_ and obs["collided"].dtype == np.bool_, "flags are bool")
check(not obs["out"].any() and not obs["hit_vehicle"].any(), "all in the world, none touching")
check(obs["ranges"].shape == (3, 16) and obs["ranges"].dtype == np.float32, "ranges (3, 16)")
check(np.all(obs["ranges"] > 0) and np.all(obs["ranges"] <= obs["range_m"]), "ranges in (0, 60]")
check(obs["episode"] == 1 and obs["step"] == 0 and obs["time"] == 0.0, "episode 1, step 0")
check(gym.get("gym.episode") == "1" and gym.get("gym.agents") == "3", "the gym.* keys")

steer = np.array([0.0, 0.3, math.nan])
accel = np.array([2.0, 1.5, math.nan])
x0, y0, yaw0 = obs["x"].copy(), obs["y"].copy(), obs["yaw"].copy()
for _ in range(40):
    obs, reward, terminated, truncated, info = gym.step(steer, accel)
check(reward.shape == (3,), "a reward per agent")
check(isinstance(terminated, bool) and isinstance(truncated, bool), "the flags are bools")
check("return" in info and abs(info["return"] - gym.episode_return()) < 1e-12, "info's return")
check(obs["step"] == 40 and abs(obs["time"] - 40 * gym.dt()) < 1e-9, "40 steps")
check(obs["speed"][0] > 2.0, f"agent 1 under throttle ({obs['speed'][0]:.2f} m/s)")
check(obs["steer"][1] > 0.1, f"agent 2's wheels turned left ({obs['steer'][1]:.2f} rad)")
ahead = (obs["x"][0] - x0[0]) * math.cos(yaw0[0]) + (obs["y"][0] - y0[0]) * math.sin(yaw0[0])
check(ahead > 2.0, f"agent 1 drove ahead ({ahead:.2f} m)")
check(gym.get("gym.step") == "40", "gym.step follows")
# A scalar is every agent's; a wrong size is refused.
gym.step(0.0, 0.0)
try:
    gym.step([0.0, 0.0], [0.0, 0.0])
    check(False, "two commands for three agents")
except (ValueError, RuntimeError) as e:
    check("3 agents" in str(e), f"the sizes are named ({e})")
for _ in range(20):
    obs, reward, terminated, truncated, info = gym.step(0.0, 0.0)
    if truncated:
        break
check(truncated and obs["step"] == 50, "truncated at max_steps")
check(gym.get("gym.done") == "1", "gym.done")

# ── one seed, one world; another seed, another ──
def run(g, seed, n=30):
    g.reset(seed)
    o = None
    for _ in range(n):
        o = g.step([0.1, -0.2, 0.0], [1.0, 1.0, 1.5])[0]
    return o


other = pycvc.WorldGym(doc, count=3)
a = run(gym, 21)
b = run(other, 21)
check(np.array_equal(a["x"], b["x"]) and np.array_equal(a["y"], b["y"]), "two gyms, one world")
check(np.array_equal(run(gym, 21)["yaw"], a["yaw"]), "a reset with the seed again repeats it")
s1, s2 = gym.reset(21), gym.reset(22)
check(np.hypot(s1["x"][0] - s2["x"][0], s1["y"][0] - s2["y"][0]) > 1.0, "another seed, another start")

# ── a reward of your own, and the log ──
calls = []


def reward_fn(before, steer, accel, after):
    calls.append((before["step"], after["step"]))
    return after["speed"] * 0.5


gym.set_reward(reward_fn)
gym.clear_log()
obs = gym.reset(3)
obs, r, *_ = gym.step(0.0, 2.0)
check(calls == [(0, 1)], f"the reward sees before and after ({calls})")
check(np.allclose(r, obs["speed"] * 0.5), "its values are paid")


def bad(before, steer, accel, after):
    raise KeyError("no such thing")


gym.set_reward(bad)
try:
    gym.step(0.0, 0.0)
    check(False, "a reward that raises")
except KeyError as e:
    check("no such thing" in str(e), "the reward's own exception comes back")
check(gym.observe()["step"] == 2, "the step it raised in still ran")
gym.set_reward(lambda b, s, a, f: [1.0])  # the wrong count
try:
    gym.step(0.0, 0.0)
    check(False, "a reward of the wrong size")
except (ValueError, RuntimeError):
    check(True, "refused")
gym.set_reward(None)
gym.step(0.0, 0.0)

log = gym.log()
check(log["step"].dtype == np.uint64 and len(log["step"]) == 3 * 5, f"rows ({len(log['step'])})")
check(np.isnan(log["steer_cmd"][0]) and log["accel_cmd"][3] == 2.0, "the commands, as given")
csv = os.path.join(tmp, "log.csv")
check(gym.save_log(csv), "save_log")
with open(csv, encoding="utf-8") as f:
    lines = f.read().splitlines()
check(lines[0].startswith("episode,step,id,time,x,y") and len(lines) == 16, "the CSV")

# ── agents as dicts ──
g2 = pycvc.WorldGym(
    doc,
    agents=[
        {"archetype": "racer", "name": "red", "team": 1, "x": -20.0, "y": 10.0, "yaw": 0.5},
        {"archetype": "humvee", "controller": "nav@1", "metadata": {"seat": 2}},
    ],
)
o = g2.reset(0)
check(o["n"] == 2, "two agents from dicts")
check(abs(o["yaw"][0] - 0.5) < 0.05 and o["length"][0] < o["length"][1], "the racer, placed")
check(np.hypot(o["x"][0] + 20.0, o["y"][0] - 10.0) < 3.0, "at its own start")
check(g2.set_goal(int(o["id"][1]), 30.0, 30.0), "set_goal takes")
check(g2.step(math.nan, math.nan)[0]["goal_x"][1] == 30.0, "the goal, held")

for bad_doc, what in ((DOC.split("world:")[0] + "scene:" + DOC.split("scene:")[1], "no vehicles"),
                      ("windows:\n  - window: W\n", "runs no world")):
    p = os.path.join(tmp, "bad.ari")
    with open(p, "w", encoding="utf-8") as f:
        f.write(bad_doc)
    try:
        pycvc.WorldGym(p)
        check(False, f"a document with {what} loaded")
    except RuntimeError as e:
        check(what in str(e), f"refused: {what} ({e})")
try:
    pycvc.WorldGym(doc, agents=[{"archetype": "racer", "controller": "nav@1"}]).reset(0)
    check(False, "a controller the archetype does not allow")
except RuntimeError as e:
    check("does not allow" in str(e), f"the spawn's refusal ({e})")

del gym, other, g2
print(f"test_pycvc_gym: {checks} checks passed")
