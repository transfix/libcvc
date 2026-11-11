#!/usr/bin/env python3
"""pycvc's sim_world agents at run time: ids, identity and a nav@1 policy each (L0, third part).

Checks:
  1. a created world's agents have ids 1..N; nav_sim_world_add_agent returns the next id, and
     nav_sim_world_remove_agent closes the gap, keeping the others' order (nav_sim_world_ids,
     nav_sim_world_index_of); an id is never reused;
  2. an added agent drives exactly as the same agent in a world created with it;
  3. identity: name, team, two colours and metadata, read back by nav_sim_world_identity;
  4. policies: nav_sim_world_set_policy loads a .cvcnav through a policy library once per content
     hash (checked against its SHA-256 when given), agents naming it share one copy, its key is
     the agent's "policy", and resetting the agents drops it; "" is the default policy;
  5. errors: a plane out of range, an unknown id, a hash the file doesn't have.
Script-style, like the other pycvc tests; self-skips (exit 0) when the running pycvc predates the
bindings.
"""

import hashlib
import os

import numpy as np
import pycvc

if not hasattr(pycvc, "nav_sim_world_add_agent") or not hasattr(pycvc, "nav_policy_library_create"):
    print("pycvc lacks nav_sim_world_add_agent / nav_policy_library_create — skipping")
    raise SystemExit(0)

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


def expect_raises(fn, what):
    try:
        fn()
    except RuntimeError:
        return
    raise AssertionError(what + ": no error raised")


BOUNDS = (-40.0, -40.0, 40.0, 40.0)  # 81 x 81 one-metre cells
SCALE = 0.05  # normalized units per metre
OPEN = np.zeros((81, 81), np.uint8)
OPEN[30:50, 38:43] = 1  # a wall to drive round


def make_sim(starts, goals):
    """test_pycvc_nav_override.make_sim's world (sensing frozen, no crowd terms, so each agent
    drives on its own), with a wall in the middle."""
    n = len(starts)
    return pycvc.nav_sim_world_create(
        OPEN, OPEN, "", np.array(starts, np.float32).reshape(n, 2),
        np.array(goals, np.float32).reshape(n, 2), np.ones((n, 3), np.float32), 81, 81, *BOUNDS,
        0.0, 0.0, SCALE, 60.0, 240, 6.283185307179586, 0.15, 0.35, 0.06, 0.4, 0.165, 0.6, 0.12,
        0.12, 0.8, 6, 1, 0.2, 4, 1, 2.2, -1.4, 8.0, 1, 0.5, 0.15, 4.0)


def ids(sim):
    return [int(i) for i in pycvc.nav_sim_world_ids(sim)]


def step(sim, ticks):
    for _ in range(ticks):
        pycvc.nav_sim_world_step(sim, 1)


A = ([-1.5, 0.3], [1.5, 0.3])
B = ([-1.5, -0.4], [1.5, -0.2])
C = ([1.2, 1.2], [-1.2, -1.0])

# 1. ids
sim = make_sim([A[0], B[0]], [A[1], B[1]])
check(ids(sim) == [1, 2], f"created ids are 1..N, not {ids(sim)}")
nid = pycvc.nav_sim_world_add_agent(sim, *C[0], *C[1])
check(nid == 3 and ids(sim) == [1, 2, 3], f"add returns the next id ({nid}, {ids(sim)})")
check(pycvc.nav_sim_world_index_of(sim, 3) == 2, "index_of the added agent")
check(pycvc.nav_sim_world_remove_agent(sim, 1), "remove an agent")
check(not pycvc.nav_sim_world_remove_agent(sim, 1), "removing it again is False")
check(ids(sim) == [2, 3], f"the others keep their order: {ids(sim)}")
check(pycvc.nav_sim_world_index_of(sim, 1) == -1, "a removed id has no index")
check(pycvc.nav_sim_world_index_of(sim, 3) == 1, "an index past the removed one drops by one")
check(pycvc.nav_sim_world_add_agent(sim, *A[0], *A[1]) == 4, "an id is never reused")

# 2. an added agent drives as one built in
grown = make_sim([A[0]], [A[1]])
pycvc.nav_sim_world_add_agent(grown, *B[0], *B[1])
pycvc.nav_sim_world_add_agent(grown, *C[0], *C[1])
built = make_sim([A[0], B[0], C[0]], [A[1], B[1], C[1]])
step(grown, 120)
step(built, 120)
sg, sb = pycvc.nav_sim_world_snapshot(grown), pycvc.nav_sim_world_snapshot(built)
for k in range(3):
    check(np.array_equal(sg[k], sb[k]), f"snapshot field {k}: added agents drive as built-in ones")
moved = np.hypot(*(sb[0][2] - np.array(C[0]) / SCALE))
check(moved > 5.0, f"the agents really drove ({moved:.1f} m)")

# 3. identity
sim = make_sim([A[0], B[0]], [A[1], B[1]])
pycvc.nav_sim_world_set_name(sim, 0, "scout")
pycvc.nav_sim_world_set_team(sim, 0, 2)
pycvc.nav_sim_world_set_colors(sim, 0, (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
pycvc.nav_sim_world_set_metadata(sim, 0, "driver", "ada")
nid = pycvc.nav_sim_world_add_agent(sim, *C[0], *C[1], color=(0.0, 1.0, 0.0), team=5,
                                    name="hauler")
who = pycvc.nav_sim_world_identity(sim, 0)
check(who["id"] == 1 and who["name"] == "scout" and who["team"] == 2, f"identity: {who}")
check(who["color"] == (1.0, 0.0, 0.0) and who["color2"] == (0.0, 0.0, 1.0), f"colours: {who}")
check(who["metadata"] == {"driver": "ada"} and who["policy"] == "", f"metadata, policy: {who}")
added = pycvc.nav_sim_world_identity(sim, pycvc.nav_sim_world_index_of(sim, nid))
check(added["name"] == "hauler" and added["team"] == 5, f"added agent's identity: {added}")
check(added["color2"] == (0.0, 1.0, 0.0), "color2 defaults to color")
pycvc.nav_sim_world_set_metadata(sim, 0, "driver", "")
check(pycvc.nav_sim_world_identity(sim, 0)["metadata"] == {}, "an empty value removes a key")
pycvc.nav_sim_world_set_colors(sim, 0, None, (0.5, 0.5, 0.5))
check(pycvc.nav_sim_world_identity(sim, 0)["color"] == (1.0, 0.0, 0.0), "None keeps a colour")
expect_raises(lambda: pycvc.nav_sim_world_identity(sim, 9), "an index out of range")

# 4. policies
here = os.path.dirname(os.path.abspath(__file__))
weights = os.path.normpath(os.path.join(here, "..", "..", "share", "cvc", "nav", "coef_mlp.cvcnav"))
lib = pycvc.nav_policy_library_create()
if os.path.exists(weights):
    with open(weights, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    key = pycvc.nav_sim_world_set_policy(sim, 0, lib, weights, digest)
    check(key == "sha256:" + digest, f"the policy's key is its hash ({key})")
    check(pycvc.nav_sim_world_set_policy(sim, 1, lib, weights) == key, "the same file, one key")
    check(pycvc.nav_policy_library_size(lib) == 1, "loaded once")
    check(pycvc.nav_sim_world_policy_count(sim) == 2, "the world's own and the loaded one")
    check(pycvc.nav_sim_world_policy_of(sim, 1) == 1, "agent 1 drives by it")
    check(pycvc.nav_sim_world_identity(sim, 0)["policy"] == key, "identity names the policy")
    step(sim, 30)
    check(np.all(np.isfinite(pycvc.nav_sim_world_snapshot(sim)[0])), "a mixed fleet steps")
    expect_raises(lambda: pycvc.nav_sim_world_set_policy(sim, 0, lib, weights, "0" * 64),
                  "a hash the file doesn't have")
    pycvc.nav_sim_world_reset_policy(sim, 0)
    pycvc.nav_sim_world_reset_policy(sim, 1)
    check(pycvc.nav_sim_world_policy_count(sim) == 1, "an unused policy is dropped")
else:
    print(f"no shipped policy at {weights}: the .cvcnav checks are skipped")
check(pycvc.nav_sim_world_set_policy(sim, 2, lib, "") == "default", "'' is the default policy")
check(pycvc.nav_sim_world_policy_key(sim, 1) == "default", "policy 1's key")
expect_raises(lambda: pycvc.nav_sim_world_set_policy(sim, 0, lib, "https://x/p.cvcnav"),
              "a URI that isn't a path or file://")

# 5. errors
expect_raises(lambda: pycvc.nav_sim_world_add_agent(sim, 0.0, 0.0, 1.0, 1.0, plane=1),
              "a belief plane out of range")
check(not pycvc.nav_sim_world_remove_agent(sim, 999), "an unknown id is False")

# 6. agents join a world with a plant, and an agent may have a belief plane of its own
sim = make_sim([A[0], B[0]], [A[1], B[1]])
plant = pycvc.dyn_bike_plant([pycvc.vehicle_desc.humvee()])
pycvc.nav_sim_world_set_plant(sim, plant)
step(sim, 5)
nid = pycvc.nav_sim_world_add_agent(sim, *C[0], *C[1], speed=3.0)
check(plant.size() == 3, "the plant adds the new agent's vehicle")
check(pycvc.nav_sim_world_snapshot(sim)[2][2] == 0.0, "with a plant it starts at rest")
expect_raises(lambda: pycvc.nav_sim_world_add_agent(sim, *C[0], *C[1], vehicle=1),
              "a vehicle the plant doesn't have")
check(plant.size() == 3 and len(ids(sim)) == 3, "a refused add changes nothing")
step(sim, 20)
check(pycvc.nav_sim_world_remove_agent(sim, 1), "remove an agent from a plant's world")
check(plant.size() == 2, "the plant drops its vehicle")
step(sim, 10)
check(np.all(np.isfinite(pycvc.nav_sim_world_snapshot(sim)[0])), "the plant's world steps on")
pycvc.nav_sim_world_set_plant(sim, None)

oid = pycvc.nav_sim_world_add_agent(sim, *A[0], *A[1], own_plane=True)
check(pycvc.nav_sim_world_planes(sim) == 2, "own_plane adds a belief plane")
oi = pycvc.nav_sim_world_index_of(sim, oid)
check(int(pycvc.nav_sim_world_agent_planes(sim)[oi]) == 1, "the new agent is in it")
m = pycvc.nav_sim_world_add_plane(sim, copy_of=0)
check(m == 2, "add_plane returns the new plane's index")
check(np.array_equal(pycvc.nav_sim_world_belief_occ(sim, 2), pycvc.nav_sim_world_belief_occ(sim, 0)),
      "a copied plane plans as its source")
pycvc.nav_sim_world_remove_plane(sim, 2)
expect_raises(lambda: pycvc.nav_sim_world_remove_plane(sim, 0), "a plane with agents in it")
step(sim, 5)
check(pycvc.nav_sim_world_remove_agent(sim, oid), "remove the agent with its own plane")
check(pycvc.nav_sim_world_planes(sim) == 1, "its plane leaves with it")

print(f"test_pycvc_nav_agents: {checks} checks OK")
