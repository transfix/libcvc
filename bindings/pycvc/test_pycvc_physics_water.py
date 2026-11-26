"""pycvc: the water physics' knobs from Python (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md: C5, C6,
the wading of land vehicles, E5/O14's wind and O12's boats).

Python reads the same values as C++. The checks:
  1. the wake (field_wake, C5): a dropped crate's ring rocks a neighbour afloat at rest, which
     stays still without a wake; the wake's scene built through pycvc has the hash C++ pins
     (physics_field_test's PhysicsFieldWake.CrossPlatformGoldenHash); its readouts, the water at a
     point with the waves in it, a coast (shore_grid_mask, C6) and the refusals;
  2. wading (vehicle_desc.wading): a Humvee fords 0.5 m and its engine floods as its front axle
     meets 0.9 m, past its 0.76 m fording depth; with the deep-water kit (fording_depth 1.52, set
     through the physics binding's tuning.wading knobs) it drives on through;
  3. the wind (wind_flow, physics_world.set_wind as Ariadne's scene wind keys): its digest and
     samples are C++'s pinned ones, and it pushes a light body downwind, gusts included;
  4. boats (hull_mesh, loft_hull, boat_desc, dyn_boat_desc, dyn_boat_plant): the tactical boat is
     its model's hull, floats at its draft, and nav drives it to a goal with its books balanced; a
     boat built field by field is the launch preset bit for bit; plant_host reads the boats.
On Windows, CVC_DLL_DIRS (os.pathsep-separated) names the directories pycvc's DLLs load from
(Python 3.8+ ignores PATH for them).
"""

from __future__ import annotations

import gc
import math
import os
import struct
import sys

if sys.platform == "win32":
    for _d in os.environ.get("CVC_DLL_DIRS", "").split(os.pathsep):
        if _d and os.path.isdir(_d):
            os.add_dll_directory(_d)

import numpy as np

import pycvc

WAKE_GOLDEN = 0xC72E26B4EF0F764A  # PhysicsFieldWake.CrossPlatformGoldenHash
WIND_DIGEST = 0xC7EC24178041550E  # PhysicsField.TheSceneWindIsTheSameOnEveryPlatform
WIND_SAMPLES = 0x3EC1C28CA0601404

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


JOLT = pycvc.physics_backend_info("jolt")
CROSS = JOLT["available"] and JOLT["determinism_cross_platform"]

_volume_app = None


def uniform(value, lo, hi):
    """A 2x2x2 volume of `value` over the box [lo, hi], as the C++ tests' uniform()."""
    global _volume_app
    if _volume_app is None:
        _volume_app = pycvc.make_app()
    v = pycvc.volume(_volume_app)
    v.set_float_grid([value] * 8, 2, 2, 2, lo[0], lo[1], lo[2], hi[0], hi[1], hi[2])
    return v


def tilt(q):
    w, x, y, z = q
    return math.acos(min(1.0, 1.0 - 2.0 * (x * x + y * y)))


# --- 1. the wake (field.h's field_wake; C5, C6) ----------------------------------------------


def wake_water(w, wake=True, surface=None, shore=None):
    """physics_field_test's wake_water: calm water 200 m across, inertial and damped as a water
    node's, with a wake of 64 cells of 1 m."""
    return w.add_field(
        uniform(1000.0, (-100.0, -100.0, -8.0), (100.0, 100.0, 4.0)), damping=1.0e4,
        inertial=True, surface=surface if surface is not None else pycvc.still_surface(0.0),
        wake=pycvc.field_wake(cells=64, cell=1.0, shore=shore) if wake else None)


def crate(w, x, y, z):
    """A crate 1.6 x 1.6 x 0.8 m at density 500: afloat at rest with its centre at z = 0."""
    return w.add_box((0.8, 0.8, 0.4), position=(x, y, z), density=500.0, linear_damping=0.0)


def drop_beside(wake, apart=5.0, steps=900):
    """A crate dropped from 1.2 m beside one afloat at rest `apart` m off: the resting one's
    largest heave, drift and tilt, the most sources, the largest wave 2 m off the drop, the hash."""
    w = pycvc.physics_world()
    sea = wake_water(w, wake)
    crate(w, 0.0, 0.0, 1.2)
    resting = crate(w, apart, 0.0, 0.0)
    p0 = w.state(resting)["position"]
    out = {"heave": 0.0, "drift": 0.0, "tilt": 0.0, "sources": 0, "ring": 0.0}
    for _ in range(steps):
        w.step()
        s = w.state(resting)
        p = s["position"]
        out["heave"] = max(out["heave"], abs(p[2] - p0[2]))
        out["drift"] = max(out["drift"], math.hypot(p[0] - p0[0], p[1] - p0[1]))
        out["tilt"] = max(out["tilt"], tilt(s["rotation"]))
        out["ring"] = max(out["ring"], abs(w.field_wake_height(sea, 0.0, 2.0)))
        out["sources"] = max(out["sources"], len(w.field_wake_info(sea)["sources"]))
    out["hash"] = w.state_hash()
    out["info"] = w.field_wake_info(sea)
    return out


def test_the_wake():
    if not JOLT["available"]:
        print("  Jolt is not available: the wake's runs skipped")
        return
    # One body's waves push another (ADroppedCratesRingRocksItsNeighbour).
    off, on = drop_beside(False), drop_beside(True)
    print("  the neighbour 5 m off over 9 s: without a wake heave %.5f m, drift %.5f m; with one "
          "heave %.4f m, drift %.4f m, tilt %.4f rad (the ring %.3f m high 2 m off the drop)"
          % (off["heave"], off["drift"], on["heave"], on["drift"], on["tilt"], on["ring"]))
    check(off["heave"] < 1e-3 and off["drift"] < 1e-3, "without a wake the neighbour stays still")
    check(on["heave"] > 5e-3 and on["heave"] > 10.0 * off["heave"], "the ring lifts it")
    check(on["hash"] != off["hash"], "the wake is in the state")
    check(on["sources"] == 2, "both crates make waves (%d)" % on["sources"])
    check(on["ring"] > 0.01, "the drop makes a ring")
    check(on["info"]["placed"] and on["info"]["cells"] == 64 and on["info"]["cell"] == 1.0,
          "the window is down: %r" % on["info"])
    check(off["info"] == {"placed": False, "x0": 0.0, "y0": 0.0, "cell": 0.0, "cells": 0,
                          "windows": 0, "sources": [], "outside": 0}, "no wake, no window")

    # The wake's golden scene, call for call: a swell, two crates and a hull pushed through them.
    w = pycvc.physics_world()
    sea = wake_water(w, surface=pycvc.wave_train_surface(level=0.0, depth=20.0, trains=[
        {"amplitude": 0.2, "period_s": 6.0, "dir_rad": 0.4, "phase": 0.0},
        {"amplitude": 0.05, "period_s": 2.5, "dir_rad": 1.2, "phase": 1.0}]))
    crate(w, 0.0, 0.0, 1.2)
    crate(w, 4.0, 1.0, 0.0)
    boat = w.add_box((2.0, 0.75, 0.3), position=(-8.0, -6.0, 0.0), density=400.0,
                     linear_damping=0.0, allow_sleeping=False,
                     hull=pycvc.box_hull((2.0, 0.75, 0.3)))
    for _ in range(400):
        w.add_force(boat, (1500.0, 300.0, 0.0))
        w.step()
    print("  wake scene %016x, %d sources" % (w.state_hash(),
                                              len(w.field_wake_info(sea)["sources"])))
    check(len(w.field_wake_info(sea)["sources"]) == 3, "three bodies make waves")
    if CROSS:
        check(w.state_hash() == WAKE_GOLDEN, "the wake scene's hash is C++'s pinned %016x, got "
              "%016x" % (WAKE_GOLDEN, w.state_hash()))
    # The water at a point has the waves in it: the top is the surface plus all the wakes' height.
    x, y, z = 1.0, 3.0, -0.5
    sample = w.water_at((x, y, z))
    top = sample["depth"] + z
    swell = w.field_wake_height(sea, x, y)
    check(sample["wet"] and sample["density"] == 1000.0, "wet water at the point")
    t = w.field_time()
    surface = pycvc.wave_train_surface(level=0.0, depth=20.0, trains=[
        {"amplitude": 0.2, "period_s": 6.0, "dir_rad": 0.4, "phase": 0.0},
        {"amplitude": 0.05, "period_s": 2.5, "dir_rad": 1.2, "phase": 1.0}])
    check(abs(top - (surface.height(x, y, t) + swell)) < 1e-6 and swell != 0.0,
          "the top is the swell plus the waves (%.6f vs %.6f + %.6f)"
          % (top, surface.height(x, y, t), swell))
    bx, by, bz = w.state(boat)["position"]
    mine, all_ = w.water_at((bx, by, bz - 0.25), body=boat), w.water_at((bx, by, bz - 0.25))
    check(mine["wet"] and mine["depth"] != all_["depth"],
          "the water as the hull feels it leaves its own waves out")
    # The wake as given reads back; set_field with the same wake keeps its waves.
    check(w.field_wake_of(sea) == pycvc.field_wake(cells=64, cell=1.0), "the wake reads back")
    check(w.field_wake_of(sea).sources == 8 and w.field_wake_of(sea).shore is None,
          "with field.h's defaults")

    # Refusals (MalformedWakesAreRefused).
    r = pycvc.physics_world(backend="null")
    for bad in (dict(cells=48), dict(cells=512), dict(cells=64, cell=0.0),
                dict(cells=64, sources=0), dict(cells=64, sources=17),
                dict(cells=64, period=11.0)):
        expect_raises(lambda: r.add_field(
            uniform(1000.0, (-100.0, -100.0, -8.0), (100.0, 100.0, 4.0)), damping=1e4,
            surface=pycvc.still_surface(0.0), wake=pycvc.field_wake(**bad)), "wake %r" % bad)
    expect_raises(lambda: r.add_field(uniform(1000.0, (-5.0, -5.0, -5.0), (5.0, 5.0, 0.0)),
                                      wake=pycvc.field_wake(cells=64)),
                  "a wake on a flat-topped box")
    expect_raises(lambda: r.add_field(uniform(1.2, (-5.0, -5.0, 0.0), (5.0, 5.0, 5.0)),
                                      kind="flow", wake=pycvc.field_wake(cells=64)),
                  "a wake on a flow")

    # A coast (C6): a shore grid with land west of x = -10 (a steep shore), the mask 0 on land and
    # 1 in open water, folded into the wake by its digest.
    n = 129
    xs = -64.0 + np.arange(n)
    dist = np.tile(xs + 10.0, (n, 1)).astype(np.float32)  # + offshore, east of x = -10
    depth = np.where(dist > 0.0, np.minimum(8.0, 0.5 + dist), -1.0).astype(np.float32)
    shore = pycvc.shore_grid_mask(depth, dist, min_x=-64.0, min_y=-64.0, cell=1.0,
                                  slope=np.full((n, n), 1.0, np.float32))
    check(shore.rows() == n and shore.cols() == n, "the grid's size")
    check(shore.mask(-30.0, 0.0) == 0.0 and shore.mask(30.0, 0.0) == 1.0,
          "0 on land, 1 offshore")
    check(abs(shore.mask(-9.0, 0.0) - 1.0) < 1e-9, "a steep shore is a wall up to the waterline")
    gentle = pycvc.shore_grid_mask(depth, dist, min_x=-64.0, min_y=-64.0, cell=1.0)
    check(0.5 <= gentle.mask(-9.5, 0.0) < 1.0, "a gentle one a beach")
    check(gentle.digest() != shore.digest(), "the slope is in the digest")
    coast = pycvc.physics_world()
    sea = wake_water(coast, shore=shore)
    check(coast.field_wake_of(sea).shore_digest() == shore.digest(), "the wake keeps its shore")
    check(coast.field_wake_of(sea) != pycvc.field_wake(cells=64, cell=1.0), "a shore is another wake")
    expect_raises(lambda: pycvc.shore_grid_mask(depth, dist[:10]), "planes of two sizes")
    expect_raises(lambda: pycvc.shore_grid_mask(depth, dist, cell=0.0), "a cell of 0")


# --- 2. wading (vehicle.h's wading_desc) ------------------------------------------------------


def ford(fording=None, steps=8000):
    """PastItsFordingDepthTheEngineFloodsAndAKitFordsDeeper's ford: 0.5 m of water from x = 0,
    0.9 m from x = 40, a stock Humvee from x = -40 at about 3 m/s; `fording` set through the
    physics binding's tuning.wading.fording_depth before it drives. Returns where its front axle
    was when the engine flooded (None: never), where it stopped after, where it ended, its rpm,
    the fuel it burnt after flooding and what the binding published."""
    w = pycvc.physics_world()
    m = w.add_material("ground", friction=0.9)
    w.add_box((1500.0, 1500.0, 0.5), position=(0.0, 0.0, -0.5), motion="static", material=m)
    w.add_field(uniform(1000.0, (0.0, -50.0, -1.0), (40.0, 50.0, 0.5)))
    w.add_field(uniform(1000.0, (40.0, -50.0, -1.0), (140.0, 50.0, 0.9)))
    d = pycvc.vehicle_desc.humvee()
    front = d.axle(0)["x"]
    car = w.add_vehicle(d, position=(-40.0, 0.0, 1.2))
    app = pycvc.make_app()
    bind = pycvc.physics_binding(app, w, "physics")
    bind.publish_vehicle("car", car)
    bind.pull()
    seeded = float(pycvc.state_get(app, "physics.vehicles.car.tuning.wading.fording_depth"))
    if fording is not None:
        pycvc.state_set(app, "physics.vehicles.car.tuning.wading.fording_depth", str(fording))
        bind.pull()
    w.drive(car)
    w.step(200)
    out = {"seeded": seeded, "fording": w.vehicle_desc_of(car).wading.fording_depth,
           "flooded_at": None, "stopped_at": None, "water_depth": 0.0, "water_force": 0.0}
    fuel_at_flood = 0.0
    for _ in range(steps):
        s = w.vehicle_state(car)
        w.drive(car, accel=1.0 if s["speed"] < 3.0 else 0.0)
        w.step()
        n = w.vehicle_state(car)
        out["water_depth"] = max(out["water_depth"], n["wheels"][0]["water_depth"])
        out["water_force"] = max(out["water_force"], math.hypot(*n["water_force"]))
        if n["engine_flooded"] and out["flooded_at"] is None:
            out["flooded_at"] = n["position"][0] + front
            fuel_at_flood = n["energy"]["fuel_used"]
        if out["flooded_at"] is not None and abs(n["speed"]) < 0.05 and out["stopped_at"] is None:
            out["stopped_at"] = n["position"][0]
        if n["position"][0] > 100.0:
            break
    s = w.vehicle_state(car)
    out["end"] = s["position"][0]
    out["rpm"] = s["engine_rpm"]
    out["fuel_after"] = s["energy"]["fuel_used"] - fuel_at_flood
    bind.push()
    bind.flush()
    out["published"] = pycvc.state_get(app, "physics.vehicles.car.engine_flooded")
    out["w"], out["car"] = w, car
    del bind
    gc.collect()
    return out


def test_wading():
    d = pycvc.vehicle_desc.humvee()
    g = d.wading
    check(g.enabled and g.fording_depth == 0.76 and g.displacement == 0.12, "the Humvee wades")
    check(g.as_dict()["wheel_displacement"] == 0.8 and g.as_dict()["underbody_depth"] == 0.16,
          "its knobs as a dict")
    d.wading.fording_depth = 1.52
    check(d.wading.fording_depth == 1.52, "an attribute of the description")
    check(pycvc.wading_desc().enabled is False and pycvc.wading_desc().displacement == 0.15,
          "a generic road vehicle's defaults, off")
    check(pycvc.vehicle_desc.roboracer().wading.enabled, "the RoboRacer wades too")
    if not JOLT["available"]:
        print("  Jolt is not available: the wading runs skipped")
        return
    stock = ford()
    print("  stock: flooded with its front axle at x = %s m, stopped at %s m, %.0f rpm; water "
          "%.2f m at the front wheel, %.0f N on the wheels"
          % (stock["flooded_at"], stock["stopped_at"], stock["rpm"], stock["water_depth"],
             stock["water_force"]))
    check(stock["seeded"] == 0.76, "the knob is seeded with the Humvee's fording depth")
    check(stock["flooded_at"] is not None and abs(stock["flooded_at"] - 40.0) < 1.0,
          "the engine floods as the front axle reaches the deep water")
    check(stock["stopped_at"] is not None and stock["stopped_at"] < 60.0, "and it coasts to a stop")
    check(abs(stock["end"] - stock["stopped_at"]) < 0.5, "where it stays")
    check(stock["rpm"] < 1.0 and stock["fuel_after"] == 0.0, "a flooded engine burns nothing")
    check(stock["water_depth"] > 0.5 and stock["water_force"] > 100.0,
          "the wheels feel the water")
    check(float(stock["published"]) == 1.0, "the binding publishes engine_flooded")
    kit = ford(fording=1.52)
    print("  with the kit: reached x = %.2f m" % kit["end"])
    check(kit["fording"] == 1.52, "the tree fits the deep-water kit")
    check(kit["end"] > 100.0 and kit["flooded_at"] is None, "it fords 0.9 m and never floods")
    check(float(kit["published"]) == 0.0, "its engine runs")
    # set_vehicle tunes wading's knobs on a running vehicle; enabled and displacement are fixed.
    w, car = kit["w"], kit["car"]
    t = w.vehicle_desc_of(car)
    t.wading.fording_depth = 0.5
    w.set_vehicle(car, t)
    check(w.vehicle_desc_of(car).wading.fording_depth == 0.5, "a new fording depth")
    t.wading.enabled = False
    expect_raises(lambda: w.set_vehicle(car, t), "wading switched off on a running vehicle")
    t = w.vehicle_desc_of(car)
    t.wading.displacement = 0.5
    expect_raises(lambda: w.set_vehicle(car, t), "a new displacement on a running vehicle")
    bad = pycvc.vehicle_desc.humvee()
    bad.wading.displacement = 1.5
    expect_raises(lambda: w.add_vehicle(bad, position=(0.0, 20.0, 1.2)), "a displacement over 1")
    bad = pycvc.vehicle_desc.humvee()
    bad.wading.fording_depth = -0.1
    expect_raises(lambda: w.add_vehicle(bad, position=(0.0, 20.0, 1.2)), "a negative fording depth")


# --- 3. the wind (flow_velocity.h; Ariadne's scene wind keys) ---------------------------------


def light_ball(w):
    return w.add_sphere(0.5, position=(0.0, 0.0, 10.0), density=20.0, linear_damping=0.0,
                        angular_damping=0.0)


def test_the_wind():
    # The wind is C++'s, bit for bit (TheSceneWindIsTheSameOnEveryPlatform).
    f = pycvc.wind_flow(speed_mps=9.0, dir_rad=0.6, gustiness=0.3, base_z=2.0)
    check(f.digest() == WIND_DIGEST, "wind_flow's digest is C++'s %016x, got %016x"
          % (WIND_DIGEST, f.digest()))
    h = 1469598103934665603
    for k in range(5):
        for i in range(40):
            v = f.velocity((-300.0 + 17.0 * i, 120.0 - 9.0 * i, 2.0 + (i % 7)), 11.0 * k)
            for c in v:
                for b in struct.pack("<d", c):
                    h = ((h ^ b) * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    check(h == WIND_SAMPLES, "its samples hash to C++'s %016x, got %016x" % (WIND_SAMPLES, h))
    check(f.wind() == {"speed_mps": 9.0, "dir_rad": 0.6, "gustiness": 0.3} and f.base_z() == 2.0,
          "it reads back")
    check(f.velocity((0.0, 0.0, 1.0), 0.0) == (0.0, 0.0, 0.0) and
          abs(f.profile(12.0) - 1.0) < 1e-12 and f.base() == 2.0,
          "none under its base, the reference 10 m over it")
    # The scene's keys: dir in degrees, the wind_flow they make.
    s = pycvc.scene_wind_flow(9.0, dir=30.0, gustiness=0.25)
    check(s.digest() == pycvc.wind_flow(9.0, 30.0 * math.pi / 180.0, 0.25).digest(),
          "scene_wind_flow is the wind the keys make")
    check(s.digest() == pycvc.scene_wind_flow(9.0, 30.0).digest(), "gustiness 0.25 by default")
    gusty = pycvc.wind_flow(speed_mps=9.0, gustiness=0.5)
    steady = pycvc.wind_flow(speed_mps=9.0)
    p = (40.0, -25.0, 10.0)
    check(gusty.velocity(p, 0.0) != gusty.velocity(p, 30.0), "the gusts change in time")
    check(steady.velocity(p, 0.0) == steady.velocity(p, 30.0) and
          max(abs(a - b) for a, b in zip(steady.velocity(p, 0.0), (9.0, 0.0, 0.0))) < 1e-12,
          "a steady wind does not")
    expect_raises(lambda: pycvc.wind_flow(speed_mps=-1.0), "a negative speed")
    expect_raises(lambda: pycvc.wind_flow(speed_mps=float("nan")), "a speed that is not a number")

    # set_wind pushes a light ball downwind (no gravity: the air alone moves it); still air does
    # not.
    calm = pycvc.physics_world(gravity=(0.0, 0.0, 0.0))
    ball0 = light_ball(calm)
    calm.step(200)
    w = pycvc.physics_world(gravity=(0.0, 0.0, 0.0))
    ball = light_ball(w)
    fid = w.set_wind(12.0, dir=90.0, gustiness=0.0)  # blowing toward +y
    check(w.field_count() == 1 and w.wind_field() == fid, "the air is a field")
    check(w.wind().wind()["speed_mps"] == 12.0, "set_wind's air reads back")
    w.step()
    load = w.field_load_on(ball)["force"]
    mass = 20.0 * 4.0 / 3.0 * math.pi * 0.125
    ideal = 0.5 * 1.225 * 1.0 * math.pi * 0.25 * 12.0 * 12.0  # 1/2 rho C_d A u^2, at rest
    print("  12 m/s on a 0.5 m ball at rest: %.1f N along the wind (1/2 rho A u^2 = %.1f N), "
          "%.2f N across" % (load[1], ideal, load[0]))
    check(0.6 * ideal < load[1] < 1.4 * ideal and abs(load[0]) < 1e-6 * ideal,
          "the drag along the wind")
    w.step(199)
    v = w.state(ball)["linear_velocity"]
    check(v[1] > 2.0 and abs(v[0]) < 1e-6 * v[1], "the ball is blown along +y (%r)" % (v,))
    check(calm.state(ball0)["linear_velocity"] == (0.0, 0.0, 0.0), "still air leaves it at rest")
    check(v[1] < 12.0, "no faster than the wind")
    # The same air added by hand is the same world.
    same = pycvc.physics_world(gravity=(0.0, 0.0, 0.0))
    light_ball(same)
    same.add_field(None, kind="flow", wind=pycvc.scene_wind_flow(12.0, dir=90.0, gustiness=0.0))
    same.step(200)
    check(same.state_hash() == w.state_hash(), "add_field(None, kind='flow', wind=...) is set_wind")
    # A new wind in place; None takes it away.
    check(w.set_wind(5.0, dir=0.0) == fid and w.field_count() == 1, "changed in place")
    check(w.field_wind(fid).digest() == pycvc.scene_wind_flow(5.0, 0.0).digest(),
          "the field holds the new wind")
    w.set_wind(None)
    check(w.field_count() == 0 and w.wind() is None and w.wind_field() is None, "and taken away")
    # Gusts push harder at times: a gusty wind is in the hash, and a world with another refuses it.
    a = pycvc.physics_world(backend="null")
    a.set_wind(9.0, dir=17.0, gustiness=0.25)
    b = pycvc.physics_world(backend="null")
    b.set_wind(9.0, dir=17.0, gustiness=0.25)
    x = pycvc.physics_world(backend="null")
    x.set_wind(12.0, dir=17.0, gustiness=0.25)
    check(a.state_hash() == b.state_hash() != x.state_hash(), "the wind is in the hash")
    b.restore(a.save())
    expect_raises(lambda: x.restore(a.save()), "a world with another wind")
    expect_raises(lambda: a.add_field(None), "a fluid with no volume")
    expect_raises(lambda: a.add_field(uniform(1000.0, (-1.0, -1.0, -1.0), (1.0, 1.0, 0.0)),
                                      wind=steady), "a wind on a fluid")
    expect_raises(lambda: a.add_wind(None), "add_wind with no wind")
    if JOLT["available"]:
        # A gusty wind: the ball's push changes as the gusts pass.
        g = pycvc.physics_world(gravity=(0.0, 0.0, 0.0))
        gb = light_ball(g)
        g.set_wind(12.0, dir=90.0, gustiness=0.5)
        pushes = []
        for _ in range(20):
            g.set_state(gb, position=(0.0, 0.0, 10.0), linear_velocity=(0.0, 0.0, 0.0))
            g.step(25)
            pushes.append(g.field_load_on(gb)["force"][1])
        check(max(pushes) > 1.1 * min(pushes) > 0.0,
              "the gusts push harder at times (%.1f .. %.1f N)" % (min(pushes), max(pushes)))
    # A MuJoCo robot takes the same air (mujoco_robot.set_wind; test_pycvc_mujoco drives one).
    check(hasattr(pycvc.mujoco_robot, "set_wind") and hasattr(pycvc.mujoco_robot, "wind_force"),
          "mujoco_robot.set_wind")


# --- 4. boats (hull.h, boat.h, dyn_boat_plant.h; O12) -----------------------------------------


def boat_water(w, depth=10.0):
    """Still water up to z = 0, 1200 m square (physics_dyn_boat_plant_test's water())."""
    return w.add_field(uniform(1000.0, (-600.0, -600.0, -depth), (600.0, 600.0, 0.0)),
                       damping=1.0e4)


S = 0.05  # normalized units per metre


def open_sim(start, goal):
    """A sim_world on open water 200 m square, nav's trained metric (plant_host's make_sim)."""
    occ = np.zeros((201, 201), np.uint8)
    return pycvc.nav_sim_world_create(
        occ, occ, "", np.asarray([start], np.float32) * S, np.asarray([goal], np.float32) * S,
        np.ones((1, 3), np.float32), 201, 201, -100.0, -100.0, 100.0, 100.0, 0.0, 0.0, S, 60.0,
        120, 2 * math.pi, 1.2 * S, 3.0 * S, 0.05, 24.6 * S, 3.3 * S, 0.457, 2.0 * S, 3.0 * S, 0.8,
        5, 1, 6.0 * S, 4, 1, 2.2, -1.4, 8.0, 1, 0.5, 0.15, 4.0)


def test_hulls():
    box = pycvc.box_hull((2.0, 1.0, 0.5))
    check(box.vertex_count() == 8 and box.triangle_count() == 12, "a box's hull")
    check(box.bounds() == ((-2.0, -1.0, -0.5), (2.0, 1.0, 0.5)), "about the origin")
    # A lofted box (a flat bottom 2 m wide, 1 m high, 4 m long) floats as the box does, its origin
    # at its keel.
    lofted = pycvc.loft_hull([{"x": -2.0, "section": [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]},
                              (2.0, [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])])
    check(lofted.bounds() == ((-2.0, -1.0, 0.0), (2.0, 1.0, 1.0)), "the lofted box's extent")
    a, b = pycvc.boat_desc(), pycvc.boat_desc()
    a.set_hull_mesh(box)
    b.set_hull_mesh(lofted)
    fa = pycvc.dyn_boat_plant.float_height(a, 2000.0, 1000.0)
    fb = pycvc.dyn_boat_plant.float_height(b, 2000.0, 1000.0)
    check(abs(fa - 0.25) < 1e-6 and abs(fb + 0.25) < 1e-6,
          "2 t in a 4 x 2 m box draws 0.25 m (%.6f, %.6f)" % (fa, fb))
    check(pycvc.dyn_boat_plant.draft(a, 2000.0) >= 0.25 - 1e-6, "its draft")
    expect_raises(lambda: pycvc.dyn_boat_plant.float_height(a, 9000.0, 1000.0), "too heavy")
    expect_raises(lambda: pycvc.dyn_boat_plant.float_height(pycvc.boat_desc(), 900.0, 1000.0),
                  "no hull_mesh")
    # A hull from its own triangles; a mesh is copied in and out of a description.
    m = pycvc.hull_mesh(box.vertices(), box.triangles(), lift_coefficient=0.3)
    check(m.vertices() == box.vertices() and m.triangles() == box.triangles() and
          m.lift_coefficient == 0.3 and m.slam_coefficient == 0.5, "hull_mesh round trip")
    a.set_hull_mesh(m)
    m.lift_coefficient = 0.9
    check(a.hull_mesh().lift_coefficient == 0.3, "the description keeps its own copy")
    a.hull_mesh().lift_coefficient = 0.9
    check(a.hull_mesh().lift_coefficient == 0.3, "and gives a copy out")
    cut = a.hull_mesh().waterline_box(0.0)
    check(cut is not None and abs(cut["x"]) < 1e-9 and abs(cut["half_beam"] - 1.0) < 1e-9 and
          abs(cut["half_length"] - 2.0) < 0.06, "a box's water cut-out is its own box: %r" % cut)
    check(a.hull_mesh().waterline_box(5.0) is None, "none above it")
    a.set_hull_mesh(None)
    check(a.hull_mesh() is None and not a.hull()["has_mesh"], "and cleared")
    expect_raises(lambda: pycvc.hull_mesh([(0, 0, 0)], [(0, 1, 2)]), "a triangle past the vertices")
    expect_raises(lambda: pycvc.loft_hull([(0.0, [(0.0, 0.0), (1.0, 1.0)])]), "one station")
    expect_raises(lambda: pycvc.loft_hull([(1.0, [(0.0, 0.0), (1.0, 1.0)]),
                                           (0.0, [(0.0, 0.0), (1.0, 1.0)])]), "x decreasing")
    expect_raises(lambda: pycvc.loft_hull([{"x": 0.0}]), "a station without a section")


def test_the_tactical_boat():
    # TheTacticalBoatIsItsModelsHull: its lofted V hull, its lift, its legs as its rudder.
    t = pycvc.dyn_boat_desc.tactical()
    m = t.boat.hull_mesh()
    check(m.vertex_count() == 242 and m.triangle_count() == 480 and m.lift_coefficient == 0.56,
          "its model's hull (%d vertices, %d triangles)" % (m.vertex_count(), m.triangle_count()))
    hull = t.boat.hull()
    check(hull["shape"] == "convex_hull" and hull["mass"] == 2800.0, "2800 kg, a convex hull")
    check(t.length_overall == 7.8 and t.max_speed == 10.2 and t.turn_radius == 32.3,
          "what nav plans with")
    check(t.boat.rudder()["max_angle"] == t.boat.propeller()["outboard_max_angle"] == 0.3 and
          t.boat.rudder()["position"][0] == t.boat.propeller()["position"][0], "legs as its rudder")
    lo, hi = m.bounds()
    check(abs(hi[0] - lo[0] - 6.85) < 1e-9 and abs(hi[1] - lo[1] - 2.3296) < 1e-3,
          "6.85 m long, 2.33 m in beam")
    fh = pycvc.dyn_boat_plant.float_height(t.boat, hull["mass"], 1000.0)
    waterline = -fh
    print("  tactical: waterline %.4f m over its keel" % (waterline - lo[2]))
    check(abs(waterline - lo[2] - 0.488) < 0.002, "it floats 0.49 m over its keel")
    cut = m.waterline_box(waterline)
    check(cut is not None and 0.3 < cut["half_beam"] < 0.5 * (hi[1] - lo[1]) and
          cut["x"] + cut["half_length"] < hi[0], "its water cut-out inside the hull")
    check(pycvc.dyn_boat_desc.presets() == ("launch", "jonboat", "tactical"), "the presets")
    check(pycvc.dyn_boat_desc.preset("tactical").boat.hull_mesh().vertices() == m.vertices(),
          "a preset by name")
    expect_raises(lambda: pycvc.dyn_boat_desc.preset("yacht"), "an unknown preset")
    if not JOLT["available"]:
        print("  Jolt is not available: the boats' runs skipped")
        return

    # It floats at its draft: put on the water at rest it stays there, upright.
    w = pycvc.physics_world()
    boat_water(w)
    b = w.add_boat(t.boat, position=(0.0, 0.0, fh))
    check(w.boat_count() == 1, "one boat")
    w.step(300)
    s = w.boat_state(b)
    print("  afloat 3 s: z %.4f m (float height %.4f), tilt %.4f rad" %
          (s["position"][2], fh, tilt(s["rotation"])))
    check(abs(s["position"][2] - fh) < 0.02, "it floats at its draft")
    # Its centre of mass is its collision shape's centroid, not over the hull's centre of
    # buoyancy, so it trims a little (about 1.3 degrees).
    check(tilt(s["rotation"]) < 0.05 and abs(s["speed"]) < 0.05, "upright and at rest")
    check(w.boat_desc_of(b).hull()["mass"] == 2800.0 and
          w.boat_desc_of(b).hull_mesh().vertex_count() == 242, "its description reads back")
    expect_raises(lambda: w.remove_body(s["hull"]), "its hull is the boat's")
    # The helm: full ahead, rudder to port turns it left.
    w.set_boat_controls(b, throttle=1.0, rudder=0.5)
    w.step(500)
    s = w.boat_state(b)
    check(s["propeller_wet"] and s["thrust"] > 0.0 and s["speed"] > 3.0,
          "full ahead (%.2f m/s, %.0f N)" % (s["speed"], s["thrust"]))
    check(s["yaw_rate"] > 0.0 and s["command"] == {"throttle": 1.0, "rudder": 0.5},
          "rudder to port turns it left")
    expect_raises(lambda: w.set_boat_controls(b, throttle=2.0), "a throttle past 1")
    w.remove_boat(b)
    check(w.boat_count() == 0, "removed")

    # Nav drives it to a goal 120 m east (NavDrivesTheBoatsToTheirGoals), its books balanced.
    w = pycvc.physics_world()
    boat_water(w)
    sim = open_sim((-60.0, 0.0), (60.0, 0.0))
    plant = pycvc.dyn_boat_plant([t], w)
    pycvc.nav_sim_world_set_plant(sim, plant)
    check(pycvc.nav_sim_world_plant_kind(sim) == "dynamic" and plant.kind() == "dynamicboat",
          "the sim drives boats")
    check(plant.size() == 1 and plant.substeps() == 5, "one boat, five world steps a tick")
    hull0 = w.state(plant.hull(0))["position"]
    check(abs(hull0[0] + 60.0) < 1e-3 and abs(hull0[2] - fh) < 1e-6,
          "it starts at its agent's start, at its draft")
    reached = False
    ticks = 0
    for ticks in range(1, 2001):
        pycvc.nav_sim_world_step(sim)
        if pycvc.nav_sim_world_snapshot(sim)[4][0]:
            reached = True
            break
    pos = pycvc.nav_sim_world_snapshot(sim)[0][0]  # world metres
    books = plant.books(0)
    gap = books["kinetic"] - plant.kinetic_at_attach(0) - (
        books["drive"] - books["brake"] - books["rudder"] - books["damping"] + books["water"] +
        books["gravity"] + books["residual"])
    print("  nav: reached %s after %.1f s at (%.1f, %.1f); top speed %.2f m/s; residual %.2e of "
          "the flow" % (reached, 0.05 * ticks, pos[0], pos[1], plant.boat_state(0)["speed"],
                        abs(books["residual"]) / books["flow"]))
    check(reached, "nav drives the tactical boat to its goal")
    check(abs(gap) < 1e-9 * books["flow"], "its books close")
    check(abs(books["residual"]) / books["flow"] < 1e-3, "with a small residual")
    check(plant.wheelbase(0) > 6.0 and plant.max_steer(0) > 0.0, "the bicycle nav plans with")
    check(plant.desc(0).max_speed == 10.2, "its description")
    plant.hold_controls(0, throttle=-1.0, rudder=0.0)
    check(plant.held(0), "a helm of its own")
    for _ in range(40):
        pycvc.nav_sim_world_step(sim)
    check(plant.boat_state(0)["throttle"] < -0.5, "astern on the held helm")
    plant.release_controls(0)
    check(not plant.held(0), "released")
    expect_raises(lambda: plant.state(1), "an agent it does not have")
    # The plant keeps the world alive; the sim keeps the plant.
    del plant, w
    gc.collect()
    pycvc.nav_sim_world_step(sim)
    pycvc.nav_sim_world_set_plant(sim, None)
    check(pycvc.nav_sim_world_plant_kind(sim) == "kinematic", "back on the bicycle")


def test_a_boat_field_by_field():
    """A boat built from the launch preset's own numbers is the launch, bit for bit."""
    launch = pycvc.dyn_boat_desc.launch().boat
    h = launch.hull()
    mesh = launch.hull_mesh()
    d = pycvc.boat_desc()
    d.set_hull(half_extents=h["half_extents"], mass=h["mass"], density=h["density"],
               linear_damping=h["linear_damping"], angular_damping=h["angular_damping"],
               allow_sleeping=h["allow_sleeping"], material=h["material"])
    d.set_hull_mesh(pycvc.hull_mesh(
        mesh.vertices(), mesh.triangles(), length=mesh.length,
        kinematic_viscosity=mesh.kinematic_viscosity, suction_coefficient=mesh.suction_coefficient,
        slam_coefficient=mesh.slam_coefficient, lift_coefficient=mesh.lift_coefficient))
    d.set_propeller(launch.propeller())
    d.set_rudder(launch.rudder())
    d.actuator_time_constant = launch.actuator_time_constant
    check(d.hull() == h and d.propeller() == launch.propeller() and d.rudder() == launch.rudder(),
          "the same numbers")
    check(abs(pycvc.dyn_boat_plant.float_height(d, 900.0, 1000.0) - 0.15) < 1e-3,
          "900 kg in 4 x 1.5 m draws 0.15 m")
    d.set_propeller({"max_thrust": 2500.0})
    check(d.propeller()["max_thrust"] == 2500.0 and
          d.propeller()["position"] == launch.propeller()["position"], "a key it lacks keeps")
    d.set_propeller({"max_thrust": launch.propeller()["max_thrust"]})
    expect_raises(lambda: d.set_propeller({"thrust": 1.0}), "a key a propeller lacks")
    expect_raises(lambda: d.set_rudder({"area": "big"}), "a rudder area that is not a number")
    expect_raises(lambda: d.set_hull(half_extents=(1, 1, 1), radius=1.0), "two shapes")
    if not JOLT["available"]:
        return
    hashes = []
    for desc in (launch, d):
        w = pycvc.physics_world()
        boat_water(w)
        b = w.add_boat(desc, position=(0.0, 0.0, 0.15))
        w.set_boat_controls(b, throttle=1.0, rudder=0.3)
        w.step(300)
        hashes.append(w.state_hash())
    check(hashes[0] == hashes[1], "the same run (%016x, %016x)" % tuple(hashes))
    bigger = pycvc.boat_desc()
    bigger.set_hull(radius=0.8, mass=300.0)
    check(bigger.hull()["shape"] == "sphere" and bigger.hull()["radius"] == 0.8, "a sphere hull")


def test_the_plant_host_reads_the_boats():
    if not (pycvc.plant_host.available() and JOLT["available"]):
        print("  Jolt is not available: plant_host's boats skipped")
        return
    sim = open_sim((-20.0, 0.0), (20.0, 0.0))
    host = pycvc.plant_host()
    host.set_water(pycvc.make_app(), level=0.0, depth=10.0, half_size=150.0)
    check(host.boat_state(0) is None and host.boat_books(0) is None, "nothing to read off boats")
    check(host.use(sim, "dynamicboat:tactical") == "dynamicboat:tactical" and
          host.kind() == "dynamicboat", "the agents on tactical boats")
    for _ in range(100):
        pycvc.nav_sim_world_step(sim)
    s = host.boat_state(0)
    check(s is not None and s["speed"] > 0.5 and s["propeller_wet"],
          "the host reads its boat (%.2f m/s)" % s["speed"])
    check(host.boat_books(0)["drive"] > 0.0, "and its books")
    expect_raises(lambda: host.boat_state(3), "an agent it does not have")
    host.use(sim, "kinematic")
    check(host.boat_state(0) is None, "off boats again")


def main():
    for t in (test_the_wake, test_wading, test_the_wind, test_hulls, test_the_tactical_boat,
              test_a_boat_field_by_field, test_the_plant_host_reads_the_boats):
        t()
        print("ok  " + t.__name__)
    print("test_pycvc_physics_water: %d checks passed" % checks)


if __name__ == "__main__":
    main()
