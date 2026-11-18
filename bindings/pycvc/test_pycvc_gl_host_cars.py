"""pycvc_gl.cars.HostCars: the template's cars in the world the host's own document runs.

A headless host loads a world document with vehicles; HostCars spawns its cars there, drives them
with a controller's (steer, accel) as their intents and with nav@1, gives each a goal, and reads
them back as a CarsState with the world's grid as its World (rays cast on it). The same calls run
the cars in a drawn host, which shows the world and its vehicles as the document draws them.

Script-style, like the other pycvc tests; self-skips (exit 0) when the running pycvc predates the
bindings it needs, has no yaml-cpp, or has no Jolt.
"""

import os
import sys
import tempfile

import numpy as np

from pycvc_gl import cars as C

if not C.has_cars():
    print("SKIP test_pycvc_gl_host_cars: pycvc lacks the cars bindings")
    raise SystemExit(0)

import pycvc  # noqa: E402

if not hasattr(pycvc, "WorldEntities") or not hasattr(pycvc.WorldEntities, "set_goal"):
    print("SKIP test_pycvc_gl_host_cars: this pycvc's WorldEntities has no set_goal")
    raise SystemExit(0)
if "jolt" not in pycvc.physics_backends() or not pycvc.physics_backend_info("jolt")["available"]:
    print("SKIP test_pycvc_gl_host_cars: no Jolt in this build")
    raise SystemExit(0)

from pycvc_gl.ariadne import Host  # noqa: E402

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


DOC = """meta: { name: host cars test, min_libcvc: 3.7.0 }
requires: [scene.physics]
world:
  vehicles:
    archetypes:
      car: { plant: kinematic, vehicle: { preset: humvee, max_speed: 6 },
             controllers: [nav@1, external] }
scene:
  clock: { paused: true }
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [120, 120, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
    - { node: block, source: { box: { size: [10, 10, 6] } }, transform: { position: [20, 0, 3] },
        body: static }
"""

tmp = tempfile.mkdtemp(prefix="pycvc_gl_host_cars_")
path = os.path.join(tmp, "world.ari")
with open(path, "w", encoding="utf-8") as f:
    f.write(DOC)

host = Host(mode="headless", name="host_cars")
host.load(path)
frames_before = host.step(1.0 / 60.0)  # the world's grid
if not host.entities.available():
    print("SKIP test_pycvc_gl_host_cars: the document's world did not start (no Jolt?)")
    host.close()
    raise SystemExit(0)


def frames(n):
    """n frames of world time (1/60 s each): the paused clock's quanta (1/120 s), then a frame. A
    drawn host's world runs on its clock instead; lockstep keeps the test from depending on pace."""
    for _ in range(n):
        host.runtime.step_world(2)
        host.step(1.0 / 60.0)


cars = C.HostCars(host, 4, seed=3)
check(cars.n == 4 and len(cars.ids) == 4, "four cars spawned in the host's world")
check(cars.archetype == "car" and cars.driving == "kinematic", f"{cars.archetype} {cars.driving}")
check(cars.world.kind == "host" and cars.world.occ.any(), "the world is its grid, the block in it")
s = cars.state()
check(s.n == 4 and s.ranges.shape == (4, 16), f"a CarsState with rays: {s.ranges.shape}")
check(np.all(np.isfinite(s.goal)) and np.all(s.goal_dist > 0.0), "each has a goal")

# nav@1 drives them toward their goals.
d0 = s.goal_dist.copy()
for _ in range(240):
    cars.step_navigator()
    frames(1)
s = cars.state()
check(np.mean(s.goal_dist) < np.mean(d0) - 5.0, f"nav@1 drove them closer: {d0} -> {s.goal_dist}")

# A controller's command drives them instead of nav@1: a hard brake stops them, then (a negative
# acceleration from rest) backs them up.
for _ in range(180):
    cars.step(np.zeros(cars.n), np.full(cars.n, -5.0))
    frames(1)
s2 = cars.state()
check(np.all(s2.speed < 0.0), f"the command, not nav@1, drives them: backing up at {s2.speed}")
check(np.allclose(cars.last_command[1], -5.0), "the last command is the controller's")
# NaN hands them back to nav@1, as step_navigator does.
for _ in range(120):
    cars.step(np.full(cars.n, np.nan), np.full(cars.n, np.nan))
    frames(1)
check(np.all(cars.state().speed > 0.0), f"nav@1 drives them forward again: {cars.state().speed}")

# New goals, and a reset that spawns them again.
before = cars.goals
cars.new_goals()
check(not np.allclose(before, cars.goals), "new goals")
cars.reset(seed=5)
check(len(cars.ids) == 4 and len(host.entities.list()) == 4, "reset: four again, the old ones gone")

host.close()
print(f"test_pycvc_gl_host_cars: {checks} checks OK")
