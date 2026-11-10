#!/usr/bin/env python3
"""pycvc_gl.cars: N Ackermann cars that a controller of yours drives, and its Ariadne template.

Headless, in this process (no GL):
  1. the worlds: a flat square, a generated world (pycvc.generate_world) and a city from the data
     store (a city bundle in a temp directory) each give an occupancy grid, a signed distance and a
     terrain height;
  2. Cars on nav's bicycle: a straight run is the bicycle's, commands are held to the limits, a NaN
     leaves a car to the navigator, ranges see the wall, a car in another counts as touching;
  3. the settings live in the state tree: settings() lists libcvc's keys with their units and
     ranges, nav.vehicle.vmax set there caps the speed, and (with Jolt) nav_plant.vehicle switches
     the plant live, the cars keeping their poses;
  4. the navigator baseline drives and reports its coefficients;
  5. rollout, local_raster and CarsLog (npz and csv);
  6. the template, run headless: it logs states, commands, coefficients, rasters and the effective
     settings; it drives on a generated world and on a city loaded by --uri; --min-arrivals and a
     controller that raises set the exit code; --copy writes cars.py and cars.ari;
  7. the pieces another host reuses: cars.ari's panels come from cars_panels.ari by cvc://, a
     state from a host's own arrays (state_from) goes in a CarsLog, the rasters and pictures, the
     coefficient plots, Cars at given starts and goals, add_args and apply_settings, save_log's
     meta and exit_code.
Script-style, like the other pycvc tests; self-skips (exit 0) when the running pycvc predates the
bindings pycvc_gl.cars needs.
"""

import json
import math
import os
import shutil
import sys
import tempfile

import numpy as np

from pycvc_gl import cars as C

if not C.has_cars():
    print("pycvc lacks nav_sim_world_override / plant_host — skipping")
    raise SystemExit(0)

import pycvc  # noqa: E402

from pycvc_gl import cars_template as T  # noqa: E402

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


TMP = tempfile.mkdtemp(prefix="pycvc_gl_cars_")


def city_bundle(path, half=100.0, n=41):
    """A city bundle with a gentle slope and no buildings: terrain.json alone."""
    os.makedirs(path, exist_ok=True)
    grid = [[0.02 * (r - n // 2) for _ in range(n)] for r in range(n)]
    with open(os.path.join(path, "terrain.json"), "w") as f:
        json.dump({"bounds": {"min_x": -half, "min_y": -half, "max_x": half, "max_y": half},
                   "rows": n, "cols": n, "grid": grid}, f)  # fmt: skip
    return path


try:
    # 1. the worlds
    flat = C.flat_world(4.84)
    check(
        flat.occ[0].all() and not flat.occ[flat.occ.shape[0] // 2, flat.occ.shape[1] // 2],
        "the flat square: walled, open inside",
    )
    check(flat.sdf.max() > 50.0 and flat.sdf[0, 0] <= 0.0, "its signed distance")
    gen = C.world_from_generated(C.generate({"seed": 3, "half": 150, "buildings": 10, "trees": 0,
                                             "rocks": 0, "amp": 10}, app=pycvc.make_app()))  # fmt: skip
    check(
        gen.kind == "generated" and gen.info["seed"] == 3 and gen.geometry is not None,
        "a generated world",
    )
    check(0.0 < gen.occ.mean() < 0.9 and not math.isnan(gen.z(0.0, 0.0)), "its grid and terrain")
    app = pycvc.make_app()
    pycvc.data_load(app, city_bundle(os.path.join(TMP, "town")), decoder="city_bundle", name="city",
                    prefix="doc")  # fmt: skip
    town = C.world_from_data(app, "doc.data.city")
    check(
        town.kind == "data" and town.bounds == (-100.0, -100.0, 100.0, 100.0),
        "a city from the store",
    )
    rise = town.z(0.0, -50.0) - town.z(0.0, 50.0)  # terrain.json's row 0 is the north edge
    check(abs(rise - 0.4) < 0.1, f"its terrain (a {rise:.2f} m rise over 100 m)")

    # 2. Cars on nav's bicycle
    cars = C.Cars(flat, 2, "kinematic", seed=1)
    s = cars.state()
    x0, y0, h0 = s.x.copy(), s.y.copy(), s.heading.copy()
    for _ in range(20):
        s = cars.step([0.0, 0.0], [1.0, 1.0])
    check(np.allclose(s.speed, 1.0, atol=1e-4), f"1 m/s^2 for 1 s: 1 m/s ({s.speed})")
    check(np.allclose(np.hypot(s.x - x0, s.y - y0), 0.5, atol=0.02), "0.5 m covered")
    check(np.allclose(s.heading, h0, atol=1e-6), "straight")
    s = cars.step([5.0, 5.0], [100.0, 100.0])
    check(
        np.allclose(cars.last_command[0], s.max_steer)
        and np.allclose(cars.last_command[1], s.max_accel),
        "held to the lock and the acceleration",
    )
    cars.step([np.nan, 0.0], [np.nan, 0.0])
    check(np.isnan(cars.last_command[0][0]), "a NaN leaves the car to the navigator")
    ahead = s.ranges[:, np.argmin(np.abs(s.ray_angles))]
    check((ahead <= s.range_max).all() and (ahead > 0).all(), "the ranges")
    near = C.collisions(flat, np.array([0.0, 1.0]), np.array([0.0, 0.5]), np.zeros(2),
                        np.full(2, 4.84), np.full(2, 2.18), np.full(2, 3.3))  # fmt: skip
    check(near.all(), "two cars on top of each other touch")

    # 3. the settings, live in the state tree
    app = pycvc.make_app()
    cars = C.Cars(flat, 2, "kinematic", seed=2, app=app, prefix="cars")
    rows = {d["path"]: d for d in C.settings(app, "cars")}
    check(
        rows["nav.vehicle.vmax"].get("unit") == "m/s" and "max" in rows["nav.vehicle.vmax"],
        "settings() lists libcvc's keys with their units and ranges",
    )
    check(rows["nav_plant.vehicle"]["value"] == "kinematic", "the plant is a setting too")
    pycvc.state_set(app, "cars.nav.vehicle.vmax", "3")
    cars.pull()
    s = cars.state()
    for _ in range(100):
        s = cars.step([0.0, 0.0], [2.0, 2.0])
    check(np.allclose(s.speed, 3.0, atol=0.05), f"nav.vehicle.vmax caps the speed ({s.speed})")
    if pycvc.plant_host.available():
        pos = s.pos.copy()
        pycvc.state_set(app, "cars.nav_plant.vehicle", "roboracer")
        cars.pull()
        s = cars.step([0.0, 0.0], [0.0, 0.0])
        check(s.driving == "roboracer" and s.length.max() < 1.0, "switched to RoboRacers, live")
        check(np.allclose(s.pos, pos, atol=0.3), "the cars keep their poses")
        for _ in range(30):
            s = cars.step([0.0, 0.0], [-1.5, -1.5])
        check((s.speed < -0.05).all(), f"a negative acceleration from rest backs up ({s.speed})")
    else:
        print("Jolt is not available — skipping the live plant switch")

    # 4. the navigator baseline
    nav = C.Cars(flat, 3, "kinematic", seed=4, navigator=True)
    d0 = nav.state().goal_dist.copy()
    for _ in range(200):
        s = nav.step_navigator()
    check((s.goal_dist < d0).sum() >= 2, "the navigator drives toward the goals")
    check(
        {"alpha", "beta", "potential"} <= set(s.nav),
        f"and reports its coefficients ({sorted(s.nav)})",
    )

    # 5. rollout, rasters, the log
    cars = C.Cars(flat, 3, "kinematic", seed=5)
    s = cars.state()
    p = dict(T.PARAMS)
    paths = C.rollout(T.controller, s, p, flat, steps=20, every=2)
    check(paths.shape == (3, 11, 2) and np.allclose(paths[:, 0], s.pos), "rollout's paths")
    r = C.local_raster(flat, s.x[0], s.y[0], s.heading[0], 60.0, 32)
    check(r["occ"].shape == (32, 32) and r["sdf"].dtype.kind == "f", "local_raster")
    check(C.colorize(r["sdf"], occ=r["occ"]).shape == (32, 32, 3), "colorize")
    log = C.CarsLog()
    for _ in range(5):
        steer, accel, info = T.controller(s, p)
        s = cars.step(steer, accel)
        log.record(s, cars.last_command)
        log.coeffs(info["coeffs"], s.n)
    log.raster(s.t, sdf=r["sdf"])
    log.meta["note"] = "test"
    z = np.load(log.save(os.path.join(TMP, "log.npz")))
    check(z["x"].shape == (5, 3) and z["coef_target speed"].shape == (5, 3), "the log's arrays")
    check(
        z["raster_sdf"].shape == (1, 32, 32) and json.loads(str(z["meta"]))["note"] == "test",
        "its rasters and meta",
    )
    rows = open(log.save(os.path.join(TMP, "log.csv"))).read().splitlines()
    check(rows[0].startswith("tick,t,car,x,y") and len(rows) == 1 + 5 * 3, "the csv")

    # 6. the template, headless
    out = os.path.join(TMP, "run.npz")
    rc = C.run(T.controller, T.PARAMS, cost=T.cost,
               argv=["--headless", "--frames", "40", "--world", "flat", "--log", out,
                     "--raster-every", "10", "--set", "ctl.speed=0.5"])  # fmt: skip
    check(rc == 0, f"a headless run (exit {rc})")
    z = np.load(out)
    meta = json.loads(str(z["meta"]))
    check(z["x"].shape == (40, 6) and z["raster_sdf"].shape[1:] == (96, 96), "states and rasters")
    check(
        z["raster_plan"].shape[1:3] == (6, 31) and "coef_heading error" in z.files,
        "predicted paths and coefficients",
    )
    check(
        meta["ctl"]["speed"] == "0.5"
        and "nav.vehicle.vmax" in meta["settings"]
        and meta["run"]["world"] == "flat",
        "the effective settings",
    )
    out = os.path.join(TMP, "gen.npz")
    rc = C.run(T.controller, T.PARAMS, argv=["--headless", "--frames", "10", "--world", "generated",
                                             "--set", "world.half=150", "--set", "world.trees=0",
                                             "--set", "world.seed=9", "--log", out])  # fmt: skip
    meta = json.loads(str(np.load(out)["meta"]))
    check(
        rc == 0
        and meta["world"]["source"] == "generated"
        and meta["world"]["seed"] == 9
        and meta["world"]["half_m"] == 150.0,
        "a generated world, its params logged",
    )
    out = os.path.join(TMP, "town.npz")
    rc = C.run(T.controller, T.PARAMS, argv=["--headless", "--frames", "10", "--uri",
                                             os.path.join(TMP, "town"), "--log", out])  # fmt: skip
    meta = json.loads(str(np.load(out)["meta"]))
    check(rc == 0 and meta["world"]["source"] == "data", "a city loaded by --uri")
    rc = C.run(T.controller, T.PARAMS, argv=["--headless", "--frames", "5", "--world", "flat",
                                             "--min-arrivals", "999"])  # fmt: skip
    check(rc == 3, f"--min-arrivals unmet: exit 3 ({rc})")

    def broken(s, p):
        raise ValueError("a controller bug")

    rc = C.run(broken, T.PARAMS, argv=["--headless", "--frames", "5", "--world", "flat"])
    check(rc == 1, f"a controller that raises: exit 1 ({rc})")
    rc = C.run(T.controller, T.PARAMS, argv=["--headless", "--frames", "20", "--world", "flat",
                                             "--controller", "navigator", "--log", out])  # fmt: skip
    check(rc == 0 and "coef_alpha" in np.load(out).files, "the navigator's coefficients logged")
    mine = os.path.join(TMP, "mine")
    check(C.run(T.controller, T.PARAMS, argv=["--copy", mine]) == 0, "--copy")
    check(
        os.path.isfile(os.path.join(mine, "cars.py"))
        and os.path.isfile(os.path.join(mine, "cars.ari")),
        "it writes cars.py and cars.ari",
    )
    check(C.run(T.controller, T.PARAMS, argv=["--copy", mine]) == 1, "and refuses to overwrite")

    # 7. the pieces for another host: the panels as units, a state from its own arrays, the
    #    pictures, the flags and settings, the log and the exit code.
    import argparse

    from pycvc_gl.ariadne import Host

    host = Host(mode="headless", name="pieces", component_paths=[C.COMPONENTS])
    warns = host.load(str(C.template_files()[1]))
    check(not [w for w in warns if "unit" in w or "import" in w],
          f"cars.ari's panels come from cars_panels.ari ({warns})")  # fmt: skip
    flat = C.flat_world(4.84)
    s = C.state_from(flat, [0.0, 2.0], [0.0, 0.0], [0.0, 0.0], [1.0, 2.0], [[50.0, 0.0], [0.0, 0.5]],
                     t=1.5, tick=30, driving="mine", nav={"alpha": [0.1, 0.2]})  # fmt: skip
    check(s.n == 2 and s.ranges.shape == (2, 16) and s.world == "flat" and s.t == 1.5,
          "state_from: a state from a host's own arrays")  # fmt: skip
    check(s.collided.all() and not s.reached[0] and s.reached[1],
          "its contacts and arrivals, worked out on the world")  # fmt: skip
    check(np.allclose(s.vx, [1.0, 2.0]) and np.allclose(s.length, 4.84), "its defaults")
    log = C.CarsLog()
    log.record(s)
    log.coeffs(s.nav, s.n)
    check(len(log) == 1 and log.arrays()["coef_alpha"].shape == (1, 2), "and CarsLog takes it")
    belief = np.zeros_like(flat.occ)
    r = C.raster_arrays(flat, s, 0, 40.0, 32, belief=belief, cost=lambda X, Y: X + Y)
    check(r["sdf"].shape == (32, 32) and r["belief"].max() == 0 and r["cost"].dtype == np.float32,
          "raster_arrays: distance, belief and cost around a car")  # fmt: skip
    rp = C.raster_arrays(flat, s, 0, 40.0, 32, belief=np.full(flat.occ.shape, 0.5, np.float32))
    check(rp["belief"].dtype == np.float32 and np.allclose(rp["belief"], 0.5),
          "a belief as a probability stays one")  # fmt: skip
    pics = C.raster_pictures(r, s, 0, 40.0, path=[(5.0, 0.0), (10.0, 0.0)])
    check(sorted(pics) == ["raster.cost", "raster.sdf"] and pics["raster.sdf"].shape == (32, 32, 3),
          "raster_pictures: the panel's two pictures")  # fmt: skip
    hist = {}
    C.plot_coefficients(host, hist, {"alpha": np.array([0.1, 0.2]), "beta": np.array([1.0, 2.0])}, 1)
    check(host.get("plot.coef0_name") == "alpha" and host.get("plot.coef2_name") == ""
          and hist["beta"] == [2.0], "plot_coefficients: the chosen car's coefficients")  # fmt: skip
    cars = C.Cars(flat, 2, "kinematic", starts=[[0.0, 0.0], [0.0, 20.0]],
                  goals=[[30.0, 0.0], [0.0, 60.0]], app=host.app, prefix=host.prefix)  # fmt: skip
    st = cars.state()
    check(np.allclose(st.pos, [[0.0, 0.0], [0.0, 20.0]], atol=1e-3)
          and np.allclose(st.heading, [0.0, math.pi / 2], atol=1e-3),
          "Cars(starts, goals): placed, facing their goals")  # fmt: skip
    p = C.add_args(argparse.ArgumentParser())
    params = os.path.join(TMP, "params.json")
    with open(params, "w") as f:
        json.dump({"nav.vehicle.vmax": 9, "view.car": 1}, f)
    a = p.parse_args(["--set", "nav.vehicle.vmax=7", "--world", "flat", "--params", params,
                      "--log", "x.npz", "--min-arrivals", "2"])  # fmt: skip
    done = C.apply_settings(host, a, {"run.cars": 3, "nav_plant.vehicle": None})
    check(host.get("nav.vehicle.vmax") == "7" and host.get("view.car") == "1"
          and host.get("run.world") == "flat" and host.get("run.cars") == "3"
          and "nav_plant.vehicle" not in done, "add_args and apply_settings: --set over --params")  # fmt: skip
    out = C.save_log(log, host, os.path.join(TMP, "pieces.npz"), who="pieces")
    meta = json.loads(str(np.load(out)["meta"]))
    check("nav.vehicle.vmax" in meta["settings"] and meta["who"] == "pieces" and "world_keys" in meta,
          "save_log: every setting in the log's meta")  # fmt: skip
    check(C.exit_code(0, error="boom") == 1 and C.exit_code(0, arrivals=1, min_arrivals=2) == 3
          and C.exit_code(4, arrivals=2, min_arrivals=2) == 4, "exit_code: 1, 3 or the document's")  # fmt: skip
    host.close()
finally:
    shutil.rmtree(TMP, ignore_errors=True)

print(f"pycvc_gl.cars: {checks} checks passed")
sys.exit(0)
