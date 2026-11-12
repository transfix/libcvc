"""pycvc: the physics world, its vehicles and the navigator's plant from Python (P6 of
docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md).

Python reads the same values as C++. The checks:
  1. the cross-platform golden scene, built through pycvc, has the state hash C++ pins
     (physics_world_test's PhysicsJolt.CrossPlatformGoldenHash);
  2. with the C++ reference (CVC_PHYSICS_PARITY_EXE, physics_parity_test), a Humvee through the
     driver model and an agent on the Jolt plant end bit for bit where C++'s twins do;
  3. a vehicle's state, the snapshot JSON (C++'s own serializer) and the state tree
     (physics_binding) carry the same numbers;
  4. save and restore reproduce a run;
  5. plant selection: a sim_world drives nav's kinematic bicycle, the Jolt plant or the dynamic
     single-track vehicle (whose books close), and the plant outlives the Python names for it
     while it is set;
  6. the sim_world's and the plant's settings in the state tree, with their metadata;
  7. terrain (P5): a vehicle's limits, the infeasible cells and the entries counted;
  8. fields and free surfaces (O6 of docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md): the surfaces
     give Airy's numbers, the swell scene built through pycvc has the hash C++ pins
     (physics_field_test's PhysicsFluidSurface.CrossPlatformGoldenHash), the field time follows a
     clock, and save, restore and the state tree carry the surface.
On Windows, CVC_DLL_DIRS (os.pathsep-separated) names the directories pycvc's DLLs load from
(Python 3.8+ ignores PATH for them).
"""

from __future__ import annotations

import gc
import json
import math
import os
import subprocess
import sys
import tempfile

if sys.platform == "win32":
    for _d in os.environ.get("CVC_DLL_DIRS", "").split(os.pathsep):
        if _d and os.path.isdir(_d):
            os.add_dll_directory(_d)

import numpy as np

import pycvc

GOLDEN = 0x70B2CC0BD34EA4BE
SWELL_GOLDEN = 0xF0085B4212E1A8C0  # PhysicsFluidSurface.CrossPlatformGoldenHash
G = 9.80665

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


# --- the reference scenarios (physics_parity_test.cpp's twins, call for call) ----------------


def exact_scene():
    w = pycvc.physics_world()  # jolt, cross-platform determinism, 0.01 s
    heights = [[(r % 5) * 0.125 + (c % 3) * 0.25 for c in range(17)] for r in range(17)]
    w.add_terrain(pycvc.height_field(17, 17, -4.0, -4.0, 0.5, 0.5, heights))
    bodies = []
    for i in range(3):
        for j in range(3):
            pos = (i * 1.5 - 1.5, j * 1.5 - 1.5, 2.0 + 0.5 * (i + j))
            vel = (0.25 * (i - 1), -0.25 * (j - 1), 0.0)
            if (i + j) % 2:
                bodies.append(w.add_sphere(0.25, position=pos, linear_velocity=vel))
            else:
                bodies.append(w.add_box((0.25, 0.125, 0.25), position=pos, linear_velocity=vel))
    for s in range(240):
        w.add_force(bodies[s % len(bodies)], ((s % 7 - 3) * 2.0, (s % 5 - 2) * 4.0, 0.0))
        w.step()
    return w.state_hash()


def ground(w):
    m = w.add_material("ground", friction=0.9, restitution=0.0, rolling_resistance=0.015)
    return w.add_box((150.0, 150.0, 0.5), position=(0.0, 0.0, -0.5), motion="static", material=m)


def vehicle_scene():
    w = pycvc.physics_world()
    ground(w)
    car = w.add_vehicle(pycvc.vehicle_desc.humvee(), position=(0.0, 0.0, 1.2))
    for s in range(400):
        w.drive(car, steer=0.05 if s < 300 else 0.0, accel=1.5 if s < 300 else -3.0)
        w.step()
    st = w.vehicle_state(car)
    e = st["energy"]
    v = {
        "x": st["position"][0],
        "y": st["position"][1],
        "z": st["position"][2],
        "qw": st["rotation"][0],
        "speed": st["speed"],
        "engine_rpm": st["engine_rpm"],
        "gear": float(st["gear"]),
        "supplied": e["supplied"],
        "source": e["source"],
        "fuel_used": e["fuel_used"],
        "distance": e["distance"],
        "wheel0_load": st["wheels"][0]["load"],
    }
    return w.state_hash(), v, w, car


def make_sim(occ=None):
    """The reference's sim_world: 81 x 81 one-metre cells, scale 0.05, the built-in policy, on the
    occupancy `occ` (rows y, columns x; empty by default) as truth and prior."""
    if occ is None:
        occ = np.zeros((81, 81), np.uint8)
    on = np.array([[-1.0, 0.0]], np.float32)
    goal = np.array([[1.0, 0.0]], np.float32)
    color = np.ones((1, 3), np.float32)
    return pycvc.nav_sim_world_create(
        occ, occ, "", on, goal, color, 81, 81, -40.0, -40.0, 40.0, 40.0, 0.0, 0.0, 0.05,
        60.0, 240, 6.283185307179586, 0.15, 0.35, 0.06, 0.4, 0.165, 0.6, 0.12, 0.12, 0.8, 6, 1,
        0.2, 4, 1, 2.2, -1.4, 8.0, 1, 0.5, 0.15, 4.0)


def plant_scene():
    sim = make_sim()
    w = pycvc.physics_world()
    ground(w)
    plant = pycvc.nav_plant(w, [pycvc.vehicle_desc.humvee()])
    pycvc.nav_sim_world_set_plant(sim, plant)
    for _ in range(150):
        pycvc.nav_sim_world_step(sim)
    s = plant.state(0)
    vs = w.vehicle_state(plant.vehicle(0))
    bins = plant.drift_summary()
    v = {
        "x": s["x"],
        "y": s["y"],
        "heading": s["heading"],
        "speed": s["speed"],
        "tire_use": s["tire_use"],
        "source": vs["energy"]["source"],
        "distance": vs["energy"]["distance"],
        "bin0_ticks": float(bins[0]["ticks"]),
        "bin0_position_rms_m": bins[0]["position_rms_m"],
        "sim_tick": float(pycvc.nav_sim_world_tick(sim)),
    }
    return w.state_hash(), v


def reference():
    """physics_parity_test's values, or None without CVC_PHYSICS_PARITY_EXE."""
    exe = os.environ.get("CVC_PHYSICS_PARITY_EXE")
    if not exe:
        print("  (no CVC_PHYSICS_PARITY_EXE: the golden hash only)")
        return None
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "parity.jsonl")
        env = dict(os.environ, CVC_PHYSICS_PARITY_JSON=out)
        dlls = os.environ.get("CVC_DLL_DIRS")
        if sys.platform == "win32" and dlls:  # the reference loads the same DLLs
            env["PATH"] = dlls + os.pathsep + env.get("PATH", "")
        subprocess.run([exe, "--gtest_filter=PhysicsParity.*"], check=True, env=env,
                       stdout=subprocess.DEVNULL)
        with open(out) as f:
            return {r["scenario"]: r for r in map(json.loads, f)}


# --- the checks ---------------------------------------------------------------------------------


def test_golden_and_reference():
    info = pycvc.physics_backend_info("jolt")
    check("jolt" in pycvc.physics_backends() and info["available"], "this build has Jolt")
    h = exact_scene()
    if info["determinism_cross_platform"]:
        check(h == GOLDEN, "the golden scene's hash is C++'s pinned %016x, got %016x" % (GOLDEN, h))
    ref = reference()
    hv, vv, _, _ = vehicle_scene()
    hp, vp = plant_scene()
    print("  golden %016x; the Humvee %.3f m (hash %016x); the agent %.3f m (hash %016x)"
          % (h, vv["distance"], hv, vp["distance"], hp))
    if ref is None:
        return
    check(ref["exact"]["hash"] == "%016x" % h, "the golden scene matches C++")
    for name, hash_, vals in (("vehicle", hv, vv), ("plant", hp, vp)):
        r = ref[name]
        check(r["hash"] == "%016x" % hash_,
              "%s: Python's state hash %016x is C++'s %s" % (name, hash_, r["hash"]))
        for k, v in vals.items():
            check(r[k] == v, "%s.%s: Python %r, C++ %r" % (name, k, v, r[k]))
    print("  Python and C++ agree bit for bit: 3 state hashes, %d values"
          % (len(vv) + len(vp)))


def test_the_same_numbers_three_ways():
    _, _, w, car = vehicle_scene()
    st = w.vehicle_state(car)
    snap = json.loads(w.snapshot_json())
    chassis = next(b for b in snap["bodies"] if b["id"] == st["chassis"])
    check(tuple(chassis["position"]) == st["position"], "the JSON's chassis position")
    check(tuple(chassis["rotation"]) == st["rotation"], "the JSON's chassis rotation")
    check(snap["tick"] == w.tick() == 400, "the JSON's tick")
    check(snap["vehicles"][0]["desc"]["mass"] == pycvc.vehicle_desc.humvee().mass,
          "the JSON's vehicle description")
    app = pycvc.make_app()
    b = pycvc.physics_binding(app, w, "physics")
    b.publish_vehicle("car", car)
    b.push()
    b.flush()
    tree_speed = float(pycvc.state_get(app, "physics.vehicles.car.speed"))
    check(tree_speed == st["speed"], "the tree's speed %r is %r" % (tree_speed, st["speed"]))
    check(pycvc.state_property(app, "physics.vehicles.car.speed", "unit") == "m/s",
          "the tree says the speed's unit")
    check(pycvc.state_comment(app, "physics.vehicles.car.speed") != "", "and what it is")
    del b
    gc.collect()


def test_save_restore():
    w = pycvc.physics_world()
    ground(w)
    ball = w.add_sphere(0.5, position=(0.0, 0.0, 3.0), linear_velocity=(1.0, 0.0, 0.0))
    w.step(20)
    saved = w.save()
    w.step(80)
    end, end_state = w.state_hash(), w.state(ball)
    w.restore(saved)
    check(w.tick() == 20, "restore rewinds the tick")
    w.step(80)
    check(w.state_hash() == end and w.state(ball) == end_state, "and the re-run reproduces it")
    check(w.state(ball)["position"][2] < 1.0, "the ball fell")
    hit = w.raycast((5.0, 5.0, 10.0), (5.0, 5.0, -10.0))
    check(hit is not None and abs(hit["point"][2]) < 1e-3, "a ray down hits the ground's top")
    expect_raises(lambda: w.state(123456789), "a stale body id")
    expect_raises(lambda: w.add_box((1, 1)), "a 2-vector half extents")
    expect_raises(lambda: pycvc.physics_world(backend="havok"), "an unknown backend")


def test_forces_at_a_point():
    # A 125 kg, 1 x 0.5 x 0.25 m box floating free (no gravity, no damping), as in
    # physics_world_test.cpp's PhysicsJolt.AnOffCentreForceSpinsTheBody.
    w = pycvc.physics_world(gravity=(0.0, 0.0, 0.0))
    b = w.add_box((0.5, 0.25, 0.125), position=(3.0, 2.0, 20.0), linear_damping=0.0,
                  angular_damping=0.0)
    com = w.state(b)["position"]
    iyy = 125.0 * (1.0 + 0.0625) / 12.0
    izz = 125.0 * (1.0 + 0.25) / 12.0
    dt = w.fixed_dt()
    # Up at the +x end: r x F = (0.5, 0, 0) x (0, 0, 100) = (0, -50, 0) N m.
    w.add_force_at(b, (0.0, 0.0, 100.0), (com[0] + 0.5, com[1], com[2]))
    w.step()
    s = w.state(b)
    check(abs(s["angular_velocity"][1] - (-50.0 * dt / iyy)) < 1e-3 * 50.0 * dt / iyy,
          "an off-centre force spins the body at r x F dt / I")
    check(abs(s["linear_velocity"][2] - 100.0 * dt / 125.0) < 1e-6,
          "and moves its centre as add_force would")
    w2 = pycvc.physics_world(gravity=(0.0, 0.0, 0.0))
    b2 = w2.add_box((0.5, 0.25, 0.125), position=(3.0, 2.0, 20.0), linear_damping=0.0,
                    angular_damping=0.0)
    w2.add_torque(b2, (0.0, 0.0, 5.0))
    w2.step()
    s2 = w2.state(b2)
    check(abs(s2["angular_velocity"][2] - 5.0 * dt / izz) < 1e-3 * 5.0 * dt / izz,
          "a torque spins the body at T dt / I")
    check(s2["linear_velocity"] == (0.0, 0.0, 0.0), "without moving it")
    nan = float("nan")
    expect_raises(lambda: w.add_force_at(b, (nan, 0.0, 0.0), com), "a force that is not finite")
    expect_raises(lambda: w.add_force_at(b, (0.0, 0.0, 1.0), (0.0, float("inf"), 0.0)),
                  "a point that is not finite")
    expect_raises(lambda: w.add_torque(b, (0.0, nan, 0.0)), "a torque that is not finite")
    expect_raises(lambda: w.add_torque(123456789, (0.0, 0.0, 1.0)), "a stale body id")
    expect_raises(lambda: w.add_force_at(b, (0.0, 0.0), com), "a 2-vector force")


def test_plant_selection():
    sim = make_sim()
    check(pycvc.nav_sim_world_plant_kind(sim) == "kinematic", "nav's bicycle by default")
    w = pycvc.physics_world()
    ground(w)
    plant = pycvc.nav_plant(w, [pycvc.vehicle_desc.humvee()], latency=0.15)
    pycvc.nav_sim_world_set_plant(sim, plant)
    check(pycvc.nav_sim_world_plant_kind(sim) == "jolt", "the Jolt plant once set")
    check(plant.options()["latency"] == 0.15, "the plant's options")
    del plant, w  # the handle keeps the plant (and the plant the world) alive
    gc.collect()
    for _ in range(20):
        pycvc.nav_sim_world_step(sim)
    pos = pycvc.nav_sim_world_snapshot(sim)[0]
    check(pos[0, 0] > -20.0, "the agent moved on the plant after its Python names were gone")
    pycvc.nav_sim_world_set_plant(sim, None)
    check(pycvc.nav_sim_world_plant_kind(sim) == "kinematic", "and back to the bicycle")
    pycvc.nav_sim_world_step(sim)
    expect_raises(lambda: pycvc.nav_sim_world_set_plant(sim, 42), "a plant that is not one")


def test_the_ground_is_not_a_building():
    """nav_plant(ground_bodies=): a chassis set into the ground touches it, which nav_stats'
    building_contacts leaves out once the plant knows the ground; a pole through the nose counts."""

    def contacts(spawn_height, tell, pole=False):
        sim = make_sim()
        w = pycvc.physics_world()
        g = ground(w)
        if pole:  # 3 m ahead of the rear axle (the agent starts at x = -20 m), inside the body
            w.add_box((0.3, 0.3, 1.5), position=(-17.0, 0.0, 1.5), motion="static")
        plant = pycvc.nav_plant(w, [pycvc.vehicle_desc.humvee()], spawn_height=spawn_height,
                                ground_bodies=[g] if tell else None)
        pycvc.nav_sim_world_set_plant(sim, plant)
        pycvc.nav_sim_world_begin_nav_stats(sim)
        for _ in range(40):
            pycvc.nav_sim_world_step(sim)
        return json.loads(pycvc.nav_sim_world_nav_stats(sim))["per_vehicle"][0]["building_contacts"]

    check(contacts(0.2, False) > 0, "without the ground's id the ground counts")
    check(contacts(0.2, True) == 0, "the ground is not a building")
    check(contacts(-1.0, True, pole=True) > 0, "a pole through the nose is one")
    w = pycvc.physics_world()
    expect_raises(lambda: pycvc.nav_plant(w, [pycvc.vehicle_desc.humvee()], ground_bodies=3),
                  "ground_bodies is a sequence of ids")


def books_gap(plant, i):
    """kinetic now - kinetic at attach - the books: zero to rounding, by construction."""
    b = plant.books(i)
    return b["kinetic"] - plant.kinetic_at_attach(i) - (
        b["drive"] - b["brake"] - b["dissipated"] - b["slip"] + b["floor"] + b["grade"]
        + b["limiter"] + b["contact"] - b["contact_stored"] - b["contact_dissipated"]
        + b["contact_exchange"] + b["residual"])


def test_the_dynamic_plant():
    """A sim_world drives the dynamic single-track vehicle (dyn_bike_plant): the agent moves, its
    books close, the plant outlives its Python names while it is set, and with a world its chassis
    hits a wall the navigator does not know about."""
    sim = make_sim()
    plant = pycvc.dyn_bike_plant([pycvc.vehicle_desc.humvee()], mu=0.7)
    check(plant.kind() == "dynamic" and plant.size() == 0 and not plant.collides(),
          "a dynamic plant without a world, not yet attached")
    check(plant.options()["mu"] == 0.7, "the plant's options")
    pycvc.nav_sim_world_set_plant(sim, plant)
    check(pycvc.nav_sim_world_plant_kind(sim) == "dynamic", "the dynamic plant once set")
    check(plant.size() == 1 and plant.chassis(0) is None, "one agent, no chassis without a world")
    for _ in range(150):
        pycvc.nav_sim_world_step(sim)
    s, body, b = plant.state(0), plant.body(0), plant.books(0)
    check(s["x"] > 0.8 and abs(body["x"] / 20.0 - s["x"]) < 0.1,
          "the agent drove east to its goal (x %.3f; the model's rear axle at %.1f m)"
          % (s["x"], body["x"]))
    check(b["drive"] > 0.0 and b["dissipated"] > 0.0, "it drew drive energy and dissipated some")
    check(abs(books_gap(plant, 0)) <= 1e-9 * b["flow"], "the books close")
    check(abs(b["residual"]) / b["flow"] < 5e-3, "the integrator's residual is small")
    tick = plant.last_tick(0)
    check(set(tick) == set(b) and tick["flow"] <= b["flow"], "the last tick's books")
    expect_raises(lambda: plant.books(1), "an agent the plant does not have")
    del plant
    gc.collect()
    pycvc.nav_sim_world_step(sim)  # the handle keeps the plant alive
    pycvc.nav_sim_world_set_plant(sim, None)
    check(pycvc.nav_sim_world_plant_kind(sim) == "kinematic", "and back to the bicycle")

    # With a world: a wall across the road at x = -5 m that nav's (empty) map lacks.
    sim = make_sim()
    w = pycvc.physics_world()
    m = w.add_material("ground", friction=0.9, restitution=0.0, rolling_resistance=0.015)
    g = w.add_box((150.0, 150.0, 0.5), position=(0.0, 0.0, -0.5), motion="static", material=m)
    w.add_box((0.5, 20.0, 3.0), position=(-4.5, 0.0, 3.0), motion="static")
    plant = pycvc.dyn_bike_plant([pycvc.vehicle_desc.humvee()], world=w, ground_bodies=[g])
    pycvc.nav_sim_world_set_plant(sim, plant)
    check(plant.collides() and plant.chassis(0) is not None, "a chassis in the world")
    del w
    gc.collect()  # the plant keeps its world alive
    for _ in range(200):
        pycvc.nav_sim_world_step(sim)
    b = plant.books(0)
    check(plant.collisions(0) > 0 and b["contact"] != 0.0,
          "the chassis hit the wall (%d substeps, %.1f J)" % (plant.collisions(0), b["contact"]))
    check(plant.state(0)["x"] < -0.2, "and stopped short of it")
    check(abs(books_gap(plant, 0)) <= 1e-9 * b["flow"], "the books close with contact")
    expect_raises(lambda: pycvc.dyn_bike_plant([pycvc.vehicle_desc.humvee()], ground_bodies=[g]),
                  "ground bodies without a world")


def drive(plant, sim=None, ticks=200):
    """Puts a sim_world (make_sim's by default) on `plant` and steps it; returns each tick's model
    state (the rear axle in metres)."""
    sim = sim or make_sim()
    pycvc.nav_sim_world_set_plant(sim, plant)
    track = []
    for _ in range(ticks):
        pycvc.nav_sim_world_step(sim)
        track.append(plant.body(0))
    return track


def test_the_dynamic_plant_options():
    """dyn_bike_plant's options from Python: wheel spin (the slip line), a ground the vehicle
    climbs, the side slope (gravity across the heading, pulling it downhill) and compliant contact
    (a wall in a sim_world's signed distance), each with books that close."""
    humvee = pycvc.vehicle_desc.humvee()
    roll = pycvc.dyn_bike_plant([humvee], mu=0.3)
    o = roll.options()
    check(not (o["wheel_spin"] or o["side_slope"] or o["contact"] or o["ground"]),
          "every option off by default")
    drive(roll, ticks=100)
    check(roll.books(0)["slip"] == 0.0 and roll.contact_now(0) is None,
          "without them: no slip line, no contact")
    check(roll.wheel_speed(0, 0) == roll.wheel_speed(0, 1),
          "and the wheels roll at the speed's rate")

    spin = pycvc.dyn_bike_plant([humvee], mu=0.3, wheel_spin=True)
    drive(spin, ticks=100)
    b = spin.books(0)
    check(spin.options()["wheel_spin"] and b["slip"] > 0.0,
          "wheel spin: the tires' longitudinal slip is booked (%.1f J)" % b["slip"])
    check(spin.wheel_speed(0, 0) != spin.wheel_speed(0, 1), "each axle's wheels at their own speed")
    check(abs(books_gap(spin, 0)) <= 1e-9 * b["flow"] and abs(b["residual"]) / b["flow"] < 5e-3,
          "the books close with the slip line")
    expect_raises(lambda: spin.wheel_speed(0, 2), "an axle the single track does not have")

    # A 5% ramp up along the road (x), and a 10% slope down across it (toward -y).
    xs = np.linspace(-40.0, 40.0, 161)
    ramp = pycvc.height_field(161, 161, -40.0, -40.0, 0.5, 0.5,
                              np.tile(0.05 * xs, (161, 1)).astype(np.float32))
    hill = pycvc.height_field(161, 161, -40.0, -40.0, 0.5, 0.5,
                              np.tile(0.1 * xs[:, None], (1, 161)).astype(np.float32))
    climb = pycvc.dyn_bike_plant([humvee], ground=ramp)
    del ramp
    gc.collect()  # the plant keeps its own copy
    drive(climb, ticks=150)
    b = climb.books(0)
    check(climb.options()["ground"] and b["grade"] < 0.0,
          "a ground: climbing the ramp costs energy (grade %.0f J)" % b["grade"])
    check(abs(books_gap(climb, 0)) <= 1e-9 * b["flow"], "the books close with the grade")

    across = pycvc.dyn_bike_plant([humvee], ground=hill, side_slope=True)
    along = pycvc.dyn_bike_plant([humvee], ground=hill)
    low_across = min(s["y"] for s in drive(across, ticks=150))
    low_along = min(s["y"] for s in drive(along, ticks=150))
    b = across.books(0)
    check(low_across < low_along - 0.01,
          "the side slope pulls it downhill (lowest y %.3f m, %.3f m without)"
          % (low_across, low_along))
    check(along.books(0)["grade"] == 0.0 and b["grade"] != 0.0,
          "only the side slope books the slope across the heading")
    check(abs(books_gap(across, 0)) <= 1e-9 * b["flow"] and abs(b["residual"]) / b["flow"] < 5e-3,
          "the books close with gravity in H")

    # A wall across the road at x = 0..2 m in one sim_world's map, and a vehicle driven by
    # another whose map is empty: the navigator drives at it, at about 7.8 m/s, and the barrier
    # stops it, at the default 0.01 s substep (the implicit damper and the contact substeps,
    # contact_step_depth).
    occ = np.zeros((81, 81), np.uint8)
    occ[:, 40:42] = 1
    walled = make_sim(occ)
    hit = pycvc.dyn_bike_plant([humvee], contact=True, contact_field=walled,
                               contact_params={"friction": 0.3})
    del walled
    gc.collect()  # the plant keeps its own copy of the field
    o = hit.options()
    check(o["contact"] and o["contact_field"] and o["contact_params"]["friction"] == 0.3
          and o["contact_params"]["k"] > 0.0 and o["substep"] == 0.01
          and o["contact_step_depth"] > 0.0, "contact, its field and its settings")
    check(pycvc.dyn_bike_plant([humvee], contact=True, contact_step_depth=0.25).options()
          ["contact_step_depth"] == 0.25
          and pycvc.dyn_bike_plant([humvee], contact=True, contact_step_depth=None).options()
          ["contact_step_depth"] == o["contact_step_depth"],
          "contact_step_depth is set, and None keeps the plant's default")
    sim = make_sim()
    pycvc.nav_sim_world_set_plant(sim, hit)
    front, pressed = -1e9, None
    for _ in range(200):
        pycvc.nav_sim_world_step(sim)
        front = max(front, hit.body(0)["x"])
        c = hit.contact_now(0)
        if c["touching"] > 0 and (pressed is None or c["energy"] > pressed["energy"]):
            pressed = c
    b = hit.books(0)
    check(front < 0.0, "the wall stops it (the rear axle at most %.2f m)" % front)
    check(pressed is not None and pressed["energy"] > 0.0 and pressed["wrench"][0] < 0.0,
          "against the wall: %d points, %.1f J, %.0f N back" % (
              pressed["touching"], pressed["energy"], pressed["wrench"][0]))
    check(b["contact_dissipated"] > 0.0 and b["contact"] == 0.0
          and abs(b["limiter"]) <= 1e-6 * b["flow"],
          "the contact's damping is booked (%.0f J), the world's contact line is not, and the "
          "speed clamp takes nothing" % b["contact_dissipated"])
    check(abs(books_gap(hit, 0)) <= 1e-9 * b["flow"] and abs(b["residual"]) / b["flow"] < 5e-3,
          "the books close with the contact lines")

    w = pycvc.physics_world()
    expect_raises(lambda: pycvc.dyn_bike_plant([humvee], world=w, contact=True),
                  "compliant contact and a world")
    expect_raises(lambda: pycvc.dyn_bike_plant([humvee], contact_field=make_sim()),
                  "a contact field without contact")
    expect_raises(lambda: pycvc.dyn_bike_plant([humvee], contact=True, contact_field=w),
                  "a contact field that is not a sim_world")
    expect_raises(lambda: pycvc.dyn_bike_plant([humvee], contact=True,
                                               contact_params={"stiffness": 1.0}),
                  "a contact setting that does not exist")
    expect_raises(lambda: pycvc.dyn_bike_plant([humvee], contact=True, contact_step_depth="deep"),
                  "a contact_step_depth that is not a number")


def test_private_belief_planes():
    """nav_sim_world_create's map_id puts each agent in its own belief plane (the fog-of-war
    twin), as sim_world's constructor does."""
    occ = np.zeros((81, 81), np.uint8)
    on = np.array([[-1.0, 0.0], [1.0, 0.0]], np.float32)
    goal = np.array([[1.0, 0.0], [-1.0, 0.0]], np.float32)
    color = np.ones((2, 3), np.float32)
    args = [occ, occ, "", on, goal, color, 81, 81, -40.0, -40.0, 40.0, 40.0, 0.0, 0.0, 0.05,
            60.0, 240, 6.283185307179586, 0.15, 0.35, 0.06, 0.4, 0.165, 0.6, 0.12, 0.12, 0.8,
            6, 1, 0.2, 4, 0, 2.2, -1.4, 8.0, 1, 0.5, 0.15, 4.0]
    shared = pycvc.nav_sim_world_create(*args)
    check(pycvc.nav_sim_world_planes(shared) == 1, "one shared plane by default")
    private = pycvc.nav_sim_world_create(*args, np.array([0, 1], np.int32), 2)
    check(pycvc.nav_sim_world_planes(private) == 2, "one plane per agent with map_id")
    check(list(pycvc.nav_sim_world_agent_planes(private)) == [0, 1], "each agent in its own")
    pycvc.nav_sim_world_step(private)
    expect_raises(lambda: pycvc.nav_sim_world_create(*args, np.array([0, 2], np.int32), 2),
                  "a plane id past n_planes")


def test_the_terrain_planes():
    """nav_terrain_build is terrain_build: the planes a Python twin is checked against."""
    x = np.arange(41, dtype=np.float64) - 20.0
    z = (0.3 * x)[None, :].repeat(41, axis=0) + 0.01 * (x ** 2)[:, None]  # a tilted bowl
    t = pycvc.nav_terrain_build(z, 1.0, {"max_grade": 0.5, "tip_ratio": 0.0})
    check(t["slope"].shape == (2, 41, 41) and t["slope"].dtype == np.float32, "slope planes")
    gy, gx = np.gradient(z.astype(np.float32))
    check(np.array_equal(t["slope"][0], gx) and np.array_equal(t["slope"][1], gy),
          "the slope is np.gradient of the float32 heights, bit for bit")
    tan = np.sqrt(gx.astype(np.float64) ** 2 + gy.astype(np.float64) ** 2)
    sin_lim = 0.5 / np.sqrt(1.25)
    cost = np.minimum(1.0, tan / np.sqrt(1.0 + tan * tan) / sin_lim).astype(np.float32)
    check(np.array_equal(t["grade_cost"], cost), "the grade cost, bit for bit")
    check(np.array_equal(t["infeasible"], (tan > 0.5).astype(np.uint8)), "the infeasible cells")
    check(t["limit"] == 0.5, "the limit")
    tip = pycvc.nav_terrain_build(z, 1.0, {"max_grade": 0.5, "tip_ratio": 0.5, "tip_margin": 0.8})
    check(tip["limit"] == 0.4 and tip["infeasible"].sum() > t["infeasible"].sum(),
          "a tip ratio lowers the limit")
    expect_raises(lambda: pycvc.nav_terrain_build(z, 0.0), "cell_w 0")


def test_settings_in_the_tree():
    sim = make_sim()
    w = pycvc.physics_world()
    ground(w)
    plant = pycvc.nav_plant(w, [pycvc.vehicle_desc.humvee()])
    pycvc.nav_sim_world_set_plant(sim, plant)
    app = pycvc.make_app()
    pycvc.nav_sim_world_pull(sim, app, "scene")
    pycvc.nav_sim_world_publish(sim, app, "scene")
    plant.pull(app, "scene")
    plant.publish(app, "scene", ["lead"])
    vmax = "scene.nav.vehicle.vmax"
    check(abs(float(pycvc.state_get(app, vmax)) - 8.0) < 1e-5, "vmax seeded in m/s")
    check(pycvc.state_property(app, vmax, "unit") == "m/s", "with its unit")
    check(pycvc.state_property(app, vmax, "access") == "knob", "its access")
    check(pycvc.state_property(app, vmax, "max") == "40", "and a recommended max")
    check(pycvc.state_get(app, "scene.nav.settings.plant") == "jolt", "the plant, published")
    check(pycvc.state_comment(app, "scene.nav.tick") != "", "the tick says what it is")
    # A modulation gain set in the tree reaches the plant.
    pycvc.state_set(app, "scene.nav_plant.modulation.tire_gain", "3")
    plant.pull(app, "scene")
    check(plant.modulation()["tire_gain"] == 3.0, "the tree's tire gain reached the plant")
    check(pycvc.state_property(app, "scene.nav_plant.modulation.tire_gain", "max") == "10",
          "the gain's recommended max")
    # A top speed set in the tree caps the agent.
    pycvc.state_set(app, vmax, "5")
    pycvc.nav_sim_world_pull(sim, app, "scene")
    fastest = 0.0
    for _ in range(150):
        pycvc.nav_sim_world_step(sim)
        fastest = max(fastest, float(pycvc.nav_sim_world_snapshot(sim)[2][0]))
    check(fastest < 5.5, "the agent kept under the tree's 5 m/s (fastest %.2f)" % fastest)
    check(pycvc.state_property(app, "scene.vehicles.lead.nav.tire_use", "access") == "telemetry",
          "the plant's measured state is telemetry")


def test_terrain():
    sim = make_sim()
    humvee = pycvc.vehicle_desc.humvee()
    cfg = pycvc.nav_plant.terrain_config_of(humvee)
    check(0.6 < cfg["max_grade"] < 0.8 and abs(cfg["tip_ratio"] - 0.91 / 0.742) < 0.05,
          "the M1097A2's limits: climb %.2f, SSF %.2f" % (cfg["max_grade"], cfg["tip_ratio"]))
    x = np.arange(81, dtype=np.float64) - 40.0
    ridge = np.maximum(0.0, 6.0 - 1.2 * np.abs(x))[None, :].repeat(81, axis=0)
    ridge[61:, :] = 0.0  # its end, north of y = 20
    check(pycvc.nav_sim_world_terrain_settings(sim) is None, "no terrain yet")
    pycvc.nav_sim_world_set_terrain(sim, ridge, cfg)
    got = pycvc.nav_sim_world_terrain_settings(sim)
    check(got["max_grade"] == cfg["max_grade"] and got["tip_ratio"] == cfg["tip_ratio"],
          "the terrain has the vehicle's limits")
    check(pycvc.nav_sim_world_infeasible_entries(sim) == [0], "no entries yet")
    app = pycvc.make_app()
    pycvc.nav_sim_world_publish(sim, app, "scene")
    blocked = int(pycvc.state_get(app, "scene.nav.terrain.infeasible_cells"))
    check(blocked > 0, "the ridge's flanks are infeasible")
    pycvc.nav_sim_world_set_terrain_config(sim, {"max_grade": 1.5, "tip_ratio": 0.0})
    pycvc.nav_sim_world_publish(sim, app, "scene")
    check(int(pycvc.state_get(app, "scene.nav.terrain.infeasible_cells")) < blocked // 4,
          "a vehicle that climbs 1.5 finds them feasible")
    expect_raises(lambda: pycvc.nav_sim_world_set_terrain(sim, np.zeros((3, 3)), cfg),
                  "heights the wrong shape")
    expect_raises(lambda: pycvc.nav_sim_world_set_terrain_config(sim, {"grade": 1.0}),
                  "a field terrain_config lacks")


# --- fields and free surfaces (field.h, fluid_surface.h; OCEAN-AND-WEATHER-ROADMAP.md O6) ----

_volume_app = None


def uniform(value, lo, hi):
    """A 2x2x2 volume of `value` over the box [lo, hi], as physics_field_test's uniform()."""
    global _volume_app
    if _volume_app is None:
        _volume_app = pycvc.make_app()
    v = pycvc.volume(_volume_app)
    v.set_float_grid([value] * 8, 2, 2, 2, lo[0], lo[1], lo[2], hi[0], hi[1], hi[2])
    return v


WATER = ((-5.0, -5.0, -5.0), (5.0, 5.0, 1.0))  # 10 m across, from z = -5 to a box top of 1


def swell(amplitude, period, depth=0.0):
    return pycvc.wave_train_surface(level=0.0, depth=depth, trains=[
        {"amplitude": amplitude, "period_s": period, "dir_rad": 0.3, "phase": 0.7}])


def swell_scene(surface, t0=3.0, damping=1.0e4):
    """physics_field_test's swell scene, call for call: a slab at density 500 and a ball at 300
    in water whose top is `surface`. Returns the world, the field and the slab."""
    w = pycvc.physics_world()
    sea = w.add_field(uniform(1000.0, *WATER), kind="fluid", damping=damping, surface=surface)
    slab = w.add_box((0.5, 0.5, 0.2), position=(0.0, 0.0, 0.0), density=500.0)
    w.add_sphere(0.25, position=(1.5, -1.0, 0.5), density=300.0)
    w.set_field_time(t0)
    return w, sea, slab


def test_the_free_surface():
    still = pycvc.still_surface(1.25, current=(0.5, -0.25, 0.0))
    check(still.level(3.0, -4.0) == 1.25 and still.height(3.0, -4.0, 17.0) == 1.25,
          "still water is flat")
    check(still.velocity((1.0, 2.0, 0.0), 5.0) == (0.5, -0.25, 0.0), "and carries its current")
    check(still.digest() == pycvc.still_surface(1.25, current=(0.5, -0.25, 0.0)).digest(),
          "a surface's digest is its parameters'")
    check(still.digest() != pycvc.still_surface(1.25).digest(), "and follows them")
    # Deep water: k = omega^2 / g, eta = A cos(k.x - omega t + phase), and a particle at the
    # still level goes round a circle at A omega, smaller as e^(kz) below.
    deep = swell(0.4, 6.0)
    om = 2.0 * math.pi / 6.0
    k = om * om / G
    check(abs(deep.wavenumber(0) - k) < 1e-12 and abs(deep.omega(0) - om) < 1e-15,
          "deep water: k = omega^2 / g")
    check(deep.train_count() == 1 and deep.train(0) == {"amplitude": 0.4, "period_s": 6.0,
                                                        "dir_rad": 0.3, "phase": 0.7},
          "the train reads back")
    x, y = 2.0, -1.0
    for t in (0.0, 1.3, 4.7):
        eta = 0.4 * math.cos(k * (x * math.cos(0.3) + y * math.sin(0.3)) - om * t + 0.7)
        check(abs(deep.height(x, y, t) - eta) < 1e-12, "eta at t = %g" % t)
        check(abs(math.hypot(*deep.velocity((x, y, 0.0), t)) - 0.4 * om) < 1e-9,
              "the orbital speed at the level, t = %g" % t)
        check(abs(math.hypot(*deep.velocity((x, y, -3.0), t)) - 0.4 * om * math.exp(-3.0 * k))
              < 1e-9, "and 3 m down, t = %g" % t)
    # Finite depth: the dispersion relation, and a bottom that does not move up or down.
    shallow = swell(0.3, 8.0, depth=6.0)
    ks, ws = shallow.wavenumber(0), 2.0 * math.pi / 8.0
    check(abs(G * ks * math.tanh(ks * 6.0) - ws * ws) < 1e-12, "omega^2 = g k tanh(k d)")
    check(abs(shallow.velocity((1.0, 1.0, -6.0), 2.0)[2]) < 1e-12, "the bottom stays put")
    check(shallow.digest() != swell(0.3, 8.0).digest(), "the depth is in the digest")
    expect_raises(lambda: pycvc.wave_train_surface(trains=[{"period_s": 0.0}]), "a period of 0")
    expect_raises(lambda: pycvc.wave_train_surface(trains=[{"height": 0.1}]),
                  "a key a train lacks")
    expect_raises(lambda: pycvc.wave_train_surface(trains=[(0.1, 4.0)]),
                  "a train that is not a dict")
    expect_raises(lambda: pycvc.still_surface(float("nan")), "a level that is not a number")
    expect_raises(lambda: deep.train(1), "a train it lacks")


def test_floating_on_the_sea():
    # The swell scene through pycvc is C++'s, bit for bit. Its surface was a temporary: the
    # field keeps it alive.
    w, sea, slab = swell_scene(swell(0.2, 4.0))
    gc.collect()
    w.step(300)
    print("  swell scene %016x" % w.state_hash())
    if pycvc.physics_backend_info("jolt")["determinism_cross_platform"]:
        check(w.state_hash() == SWELL_GOLDEN, "the swell scene's hash is C++'s pinned %016x, "
              "got %016x" % (SWELL_GOLDEN, w.state_hash()))
    load = w.field_load_on(slab)
    check(load["force"][2] > 0.0 and abs(load["immersed_volume"] - 0.2) < 0.03,
          "the sea holds the slab up, about half under (%.4f m^3)" % load["immersed_volume"])
    check(w.field_count() == 1, "one field")
    # On calm water the slab floats at half draft where the surface is, not at the box's top.
    calm = pycvc.physics_world()
    calm.add_field(uniform(1000.0, *WATER), damping=1.0e4, surface=pycvc.still_surface(-1.5))
    b = calm.add_box((0.5, 0.5, 0.2), position=(0.0, 0.0, -1.5), density=500.0)
    calm.step(600)
    z = calm.state(b)["position"][2]
    check(abs(-1.5 - (z - 0.2) - 0.2) < 0.002, "half draft at the level (z %.4f)" % z)
    expect_raises(lambda: calm.add_field(uniform(1e5, *WATER), kind="medium",
                                         surface=pycvc.still_surface(0.0)),
                  "a medium with a free surface")
    expect_raises(lambda: calm.add_field(uniform(1000.0, *WATER), kind="lava"), "a kind")
    expect_raises(lambda: calm.add_field(uniform(1000.0, *WATER), velocity=(1.0, 2.0, 3.0)),
                  "a velocity that is not volumes")
    # The surface is in the state: another amplitude or field time hashes differently.
    a = swell_scene(swell(0.2, 4.0))[0]
    check(a.state_hash() != swell_scene(swell(0.21, 4.0))[0].state_hash(), "the digest")
    check(a.state_hash() != swell_scene(swell(0.2, 4.0), t0=4.5)[0].state_hash(),
          "the field time")
    # The field time follows a clock the world joins late; save and restore carry it.
    clock = pycvc.world_clock(0.01)
    clock.advance_ticks(12345)  # 123.45 s before the world joins
    check(a.follow(clock) == 0 and abs(a.field_time() - clock.t()) < 1e-9,
          "follow() anchors the field time at the clock's")
    clock.advance_ticks(40)
    check(a.follow(clock) == 40 and abs(a.field_time() - clock.t()) < 1e-9,
          "and keeps it there")
    check(abs(a.time() - 0.4) < 1e-12, "time() stays the world's own")
    saved, t_saved = a.save(), a.field_time()
    a.step(100)
    again = swell_scene(swell(0.2, 4.0), t0=99.0)[0]
    again.restore(saved)
    check(abs(again.field_time() - t_saved) < 1e-12, "restore brings back the field time")
    again.step(100)
    check(again.state_hash() == a.state_hash(), "and the run")
    expect_raises(lambda: swell_scene(swell(0.25, 4.0))[0].restore(saved),
                  "a world with another surface")
    expect_raises(lambda: swell_scene(None)[0].restore(saved), "a world with no surface")
    expect_raises(lambda: a.set_field_time(float("nan")), "a field time that is not a number")
    # A published field's knobs come from the tree, and set_field keeps its surface: after the
    # tree doubles the damping, the run is a world built with that damping.
    w, sea, _ = swell_scene(swell(0.2, 4.0))
    app = pycvc.make_app()
    bind = pycvc.physics_binding(app, w, "physics")
    bind.publish_field("sea", sea)
    pycvc.state_set(app, "physics.fields.sea.damping", "20000")
    bind.pull()
    w.step(100)
    ref = swell_scene(swell(0.2, 4.0), damping=2.0e4)[0]
    ref.step(100)
    check(w.state_hash() == ref.state_hash(), "the tree's damping, the same surface")
    bind.push()
    bind.flush()
    check(pycvc.state_get(app, "physics.fields.sea.kind") == "fluid", "the field's telemetry")
    # set_field replaces the whole field, surface included; remove_field takes it away.
    w.set_field(sea, uniform(1000.0, *WATER), damping=1.0e4)
    check(w.state_hash() != ref.state_hash(), "a field set without its surface")
    w.remove_field(sea)
    check(w.field_count() == 0, "removed")
    expect_raises(lambda: w.remove_field(sea), "a stale field id")
    del bind
    gc.collect()


def main():
    for t in (test_golden_and_reference, test_the_same_numbers_three_ways, test_save_restore,
              test_forces_at_a_point,
              test_plant_selection, test_the_ground_is_not_a_building, test_the_dynamic_plant,
              test_the_dynamic_plant_options,
              test_private_belief_planes,
              test_settings_in_the_tree,
              test_terrain, test_the_terrain_planes,
              test_the_free_surface, test_floating_on_the_sea):
        t()
        print("ok  " + t.__name__)
    print("test_pycvc_physics: %d checks passed" % checks)


if __name__ == "__main__":
    main()
