"""pycvc.WorldEntities: a Python host lists a world's entities (its vehicles) and steers them.

Headless (pycvc.AriApp, no GL): a document whose world: block offers a Humvee that an external
controller may drive. From Python:
  * spawn() places an external entity and list() shows it, holding still with no intent;
  * set_intent(accel=) drives it ahead, set_intent(steer=) turns it, mode="off" stops it;
  * a force on a nav@1 entity's drive moves it off the path it takes without one;
  * set_intent() from another thread, while frames run, is safe, and the last one holds;
  * the refusals: no such entity, a bad mode or weight, a bad force.
pycvc.ari.Host(mode="headless").entities is the same object.
"""

import math
import os
import sys
import tempfile
import threading

import pycvc

if not hasattr(pycvc, "WorldEntities"):
    print("SKIP test_pycvc_world_entities: this pycvc has no WorldEntities")
    sys.exit(0)
if not pycvc.AriApp.have_yaml():
    print("SKIP test_pycvc_world_entities: libcvc built without yaml-cpp")
    sys.exit(0)
if "jolt" not in pycvc.physics_backends() or not pycvc.physics_backend_info("jolt")["available"]:
    print("SKIP test_pycvc_world_entities: no Jolt in this build")
    sys.exit(0)

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


def raises(fn, what):
    try:
        fn()
    except (RuntimeError, ValueError, TypeError):
        check(True, what)
        return
    raise AssertionError(f"no error: {what}")


DOC = """meta: { name: world entities test, min_libcvc: 3.7.0 }
requires: [scene.physics]
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee, controllers: [external, nav@1] }
scene:
  clock: { paused: true }
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [160, 160, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
"""

tmp = tempfile.mkdtemp(prefix="pycvc_world_entities_")
path = os.path.join(tmp, "entities.ari")
with open(path, "w", encoding="utf-8") as f:
    f.write(DOC)


def frames(rt, n):
    """n frames of world time (1/60 s each): the paused clock's quanta (1/120 s), then a drain."""
    rt.step_world(2 * n)
    rt.step(1.0 / 60.0)


def open_world(prefix):
    app = pycvc.make_app()
    rt = pycvc.AriApp(app, prefix)
    warnings = rt.load(path)
    check(not [w for w in warnings if "world:" in w], f"the document loads: {warnings}")
    rt.step(1.0 / 60.0)  # the world's grid, and the clock anchors
    return app, rt


def by_id(ents, i):
    return next(e for e in ents.list() if e["id"] == i)


app, rt = open_world("w")
ents = rt.entities()
check(ents.available(), "the headless runtime runs a world with vehicles")
check(ents.list() == [], "no entities yet")

# An external entity: it holds still with no intent.
a = ents.spawn("humvee", name="lead", at=(-40.0, 0.0, 0.0))
check(a == 1, f"spawned ({ents.why()})")
check(ents.spawn("tank") == 0 and "tank" in ents.why(), "an archetype that is not one: refused")
frames(rt, 60)
e = by_id(ents, a)
check(e["controller"] == "external" and e["name"] == "lead" and e["placed"], f"listed: {e}")
check(abs(e["speed"]) < 0.05, f"it holds still ({e['speed']})")
check(pycvc.state_get(app, "w.entities.1.command.by") == "hold", "commanded to hold")
x0, yaw0 = e["x"], e["yaw"]

# Ahead, then a turn to the left, then a stop.
check(ents.set_intent(a, accel=2.0), "set_intent")
check(ents.intent(a)["mode"] == "replace", "an accel with no mode replaces the command")
frames(rt, 180)
e = by_id(ents, a)
check(e["x"] - x0 > 4.0 and e["speed"] > 2.0, f"it drove ahead ({e['x'] - x0:.2f} m)")
check(abs(e["command_accel"] - 2.0) < 1e-3, f"at the intent's acceleration ({e['command_accel']})")
check(ents.set_intent(a, steer=0.3, accel=0.5), "a turn")
frames(rt, 120)
e = by_id(ents, a)
check(e["yaw"] > yaw0 + 0.3 and abs(e["command_steer"] - 0.3) < 1e-6, f"it turned ({e['yaw']})")
check(ents.set_intent(a, mode="off"), "off")
frames(rt, 240)
check(abs(by_id(ents, a)["speed"]) < 0.05, "it stopped")

# A force on a nav@1 entity's drive: two worlds alike, one pushed west.
_, rt2 = open_world("p")
_, rt3 = open_world("q")
plain, pushed = rt2.entities(), rt3.entities()
b_plain = plain.spawn("humvee", controller="nav@1", at=(20.0, -40.0, 90.0))
b = pushed.spawn("humvee", controller="nav@1", at=(20.0, -40.0, 90.0))
check(b_plain == 1 and b == 1, "a nav@1 entity in each")
check(pushed.set_intent(b, force=(-3.0, 0.0)), "a force west")
check(pushed.intent(b)["force"] == (-3.0, 0.0) and pushed.intent(b)["mode"] == "off",
      "a force alone leaves the command to nav@1")
frames(rt2, 240)
frames(rt3, 240)
p, q = by_id(plain, b_plain), by_id(pushed, b)
check(q["x"] < p["x"] - 0.5,
      f"the force moved it west of the plain run ({p['x']:.1f},{p['y']:.1f} vs "
      f"{q['x']:.1f},{q['y']:.1f})")
del plain, pushed, rt2, rt3

# set_intent from another thread while frames run.
stop = threading.Event()


def writer():
    i = 0
    while not stop.is_set():
        ents.set_intent(a, accel=1.0 + i % 3, force=(0.5 * (i % 4), 0.0))
        i += 1


t = threading.Thread(target=writer)
t.start()
for _ in range(30):
    frames(rt, 2)
stop.set()
t.join()
check(ents.set_intent(a, accel=2.5), "the last intent")
frames(rt, 6)
check(by_id(ents, a)["intent"]["accel"] == 2.5, "holds")

# Refusals.
check(not ents.set_intent(99, accel=1.0), "no such entity: False")
raises(lambda: ents.set_intent(a, mode="steer"), "a mode that is not one")
raises(lambda: ents.set_intent(a, mode="blend", weight=1.5), "a weight outside [0, 1]")
raises(lambda: ents.set_intent(a, force=(1.0,)), "a force that is not (fx, fy)")
raises(lambda: ents.set_intent(a, accel=float("nan")), "a steer or accel that is not finite")
check(ents.despawn(a) and ents.list() == [], "despawned")
check(not ents.set_intent(a, accel=1.0), "its intent goes with it")
del ents, rt

# The Host's entities.
import pycvc.ari as ari  # noqa: E402

with ari.Host(path, mode="headless", name="h") as host:
    host.step()
    host.step()
    check(host.entities.available(), "Host.entities")
    check(host.entities.spawn("humvee", at=(0.0, 0.0)) == 1, "spawns through the Host")

print(f"test_pycvc_world_entities: {checks} checks OK")
