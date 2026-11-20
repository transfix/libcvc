"""pycvc_gl.nav_lab: libcvc's navigation policies side by side, and recorded runs played back.

    python -m pycvc_gl.nav_lab arms --arms default,trained=run/coef.step000200.cvcnav
    python -m pycvc_gl.nav_lab arms --arms default,a=A.cvcnav,b=B.cvcnav --world flat --cars 3 \\
        --headless --frames 600 --log run.cvcrun
    python -m pycvc_gl.nav_lab replay run.cvcrun
    python -m pycvc_gl.nav_lab replay run.cvcrun --at 12 --offscreen --frames 1 --png at12.png
    python -m pycvc_gl.nav_lab scrub run/history.jsonl --agents 8 --ticks 900

**arms** runs several policies side by side: one libcvc sim_world per arm, each on the plant
``nav_plant.vehicle`` names (``--plant``: kinematic, humvee, roboracer, dynamic:humvee, ...), all
from the same starts to the same goals on the same world with the same seed, stepped tick for tick
together (``cvc::nav::arms_run``). Each arm is a ghost in its own colour. A Retarget (the button, or
``--retarget-at``) moves every arm's goals on the same tick and starts a leg. The Stats and Ledger
windows (libcvc's ``nav_lab_stats`` and ``nav_lab_ledger`` components) show what each arm did, from the run's log at its
last tick; ``--log`` writes that log (``cvc.nav.run_log/v1``) as the run goes.

An arm is ``default`` (libcvc's built-in policy: constant coefficients, alpha 1, beta 3, gamma 4) or
``NAME=PATH.cvcnav`` (a policy coef_trainer saved, a checkpoint, or GRL-SNAM's exported one).
``--scale`` is the frame the policies were trained in (normalized units per metre).

**replay** plays a run log back: it rebuilds the recorded world from the log's world keys, draws
each arm's ghost and track where it was at the play head, and writes the Stats and Ledger windows as
the live run showed them at that tick, through the same library code, so they are the same cells.
The transport is libcvc's sim_transport (Paused, Speed, Step, Restart), with a Position slider that
scrubs; a click on a chart seeks there. Headless, nothing is drawn and no world is built.

**scrub** watches a policy learn: every checkpoint a training history names (cvc::nav's coef_trainer
``history_path``, or GRL-SNAM's) deployed on one scenario (the built-in blocks, or ``--scenario
TRACE``), cached beside the history; the view shows checkpoint k beside step 0 (cvc::nav's
``scrub_view``): the deployed metric against the step, the training curve, the table, the two
runs' Stats and Ledger at the play head, and their ghosts on the scenario's grid. Each checkpoint is
deployed on each ``--plant`` (default the Jolt Humvee, which stops against the buildings, and nav's
kinematic bicycle, which drives through them; the Humvee alone, or the bicycle without Jolt), and
the Scrub window's Plant combo switches between them.

The windows and the replay's play head are cvc::nav's (cvc/nav/nav_panels.h): this module builds the
worlds, draws the ghosts and wires the events. docs/NAV_LAB.md has the whole model. The shipped
component resolves through ``cvc://``: from an installed libcvc's share, or with
``CVC_ARIADNE_PATH`` pointing at a checkout's ``src/cvc/ariadne``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import pycvc

from pycvc_gl import cars as C

HERE = Path(__file__).resolve().parent
ARMS_DOC = HERE / "nav_lab_arms.ari"
REPLAY_DOC = HERE / "nav_lab_replay.ari"
SCRUB_DOC = HERE / "nav_lab_scrub.ari"
#: Ariadne chart:'s palette (light), slots c0..c7: arm j draws in slot j everywhere.
SLOTS = (0x2A78D6, 0xEB6834, 0x1BAF7A, 0xEDA100, 0xE87BA4, 0x008300, 0x6250D6, 0xE34948)
#: How often a live run's charts are rewritten (frames).
CHART_EVERY = 6


def slot_rgb(j: int) -> tuple[float, float, float]:
    """Arm ``j``'s colour as (r, g, b) in [0, 1]."""
    c = SLOTS[j % len(SLOTS)]
    return ((c >> 16) / 255.0, ((c >> 8) & 0xFF) / 255.0, (c & 0xFF) / 255.0)


def _refuse(who, msg) -> int:
    print(f"nav_lab {who}: {msg}", file=sys.stderr)
    return 2


def sha256(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# ── arms ───────────────────────────────────────────────────────────────────────────────────────


def parse_arms(text: str | None, extra=()) -> list[dict]:
    """The arms ``--arms`` (a comma list) and each ``--arm NAME=PATH`` name, in order: each a dict
    ``{name, uri, sha256}``. ``default`` is libcvc's built-in policy. Raises ValueError for an
    unknown bare name, a file that is not there, a repeated name or more arms than colours."""
    items = [t.strip() for t in (text or "").split(",") if t.strip()]
    items += [a.strip() for a in extra if a.strip()]
    out: list[dict] = []
    for item in items:
        name, sep, path = item.partition("=")
        name = name.strip()
        if not sep:
            if name != "default":
                raise ValueError(f"no arm {item!r}: an arm is 'default' or NAME=PATH.cvcnav")
            arm = {"name": "default", "uri": "", "sha256": ""}
        else:
            p = Path(path.strip()).expanduser()
            if not p.is_file():
                raise ValueError(f"the arm {name!r}: no policy at {p}")
            arm = {"name": name, "uri": str(p.resolve()), "sha256": sha256(p)}
        if any(o["name"] == arm["name"] for o in out):
            raise ValueError(f"the arm {arm['name']!r} is named twice")
        out.append(arm)
    if not out:
        raise ValueError("--arms names no arm")
    if len(out) > len(SLOTS):
        raise ValueError(f"at most {len(SLOTS)} arms (one colour each), not {len(out)}")
    return out


def make_fleet(world, plant, arms, starts, goals, *, seed=0, scale=C.SCALE, app=None, prefix=""):
    """One ``pycvc_gl.cars.Cars`` per arm (a libcvc sim_world on ``plant``), every one from
    ``starts`` to ``goals`` ((n, 2) world metres) on ``world``, each agent driven by its arm's
    policy, loaded once through one policy library. With ``app`` the settings come from its state
    tree, as the template's cars read them."""
    library = pycvc.nav_policy_library_create()
    fleet = []
    n = len(starts)
    for a in arms:
        cars = C.Cars(world, n, plant, navigator=True, scale=scale, seed=seed, app=app,
                      prefix=prefix, starts=starts, goals=goals)  # fmt: skip
        for i in range(n):
            pycvc.nav_sim_world_set_policy(cars.sim, i, library, a["uri"], a["sha256"])
        fleet.append(cars)
    return fleet


def starts_and_goals(world, n, seed):
    """``n`` free starts, and goals at least 0.3 of the world's span away, from ``seed``."""
    rng = np.random.default_rng(seed)
    clear = 0.8 * C._BICYCLE["length"]
    starts = world.free_points(rng, n, clear, 2.5 * C._BICYCLE["length"])
    goals = world.free_points(rng, n, clear, 4.0 * C._BICYCLE["length"], avoid=starts,
                              avoid_dist=0.3 * world.span)  # fmt: skip
    return np.asarray(starts, float), np.asarray(goals, float)


def world_keys(host) -> dict:
    """What rebuilds the world: ``run`` (the world's kind, its uri, the part kept, the crop) and,
    for a generated world, ``world`` (the generator's keys)."""
    run = {k: host.get(f"run.{k}") for k in ("world", "uri", "data_path", "crop")}
    out = {"run": {k: v for k, v in run.items() if v not in (None, "")}}
    if run.get("world") == "generated":
        out["world"] = {k: host.get(f"world.{k}") for k in C.WORLD_KEYS
                        if host.get(f"world.{k}") not in (None, "")}  # fmt: skip
    return out


def settings_of(host) -> dict:
    return {d["path"]: d["value"] for d in C.settings(host.app, host.prefix)}


def parse_point(text):
    """``"x, y"`` -> (x, y), or None for an empty or malformed one."""
    parts = [p for p in (text or "").replace(";", ",").split(",") if p.strip()]
    if len(parts) != 2:
        return None
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        return None


def check_goal(world, x: float, y: float, radius: float) -> str:
    """Why ``(x, y)`` is not a goal on ``world`` for a footprint of ``radius`` (m), or "": off the
    world, inside a building, or too tight."""
    mnx, mny, mxx, mxy = world.bounds
    if not (mnx <= x <= mxx and mny <= y <= mxy):
        return "off the world"
    r, c = world.cells(x, y)
    d = float(world.sdf[r, c])
    if d <= 0.0:
        return "inside a building"
    if d < radius:
        return f"too tight ({d:.1f} m clear, the footprint needs {radius:.1f} m)"
    return ""


# ── the ghosts ─────────────────────────────────────────────────────────────────────────────────


class Ghosts:
    """Each arm's agents drawn where they are: a box their size in the arm's colour, see-through,
    a tag above, a track, and each goal's post. Live and replay both draw with it."""

    def __init__(self, host, world, arms: int, agents: int):
        self.host, self.world = host, world
        self.arms, self.agents = arms, agents
        self.nodes: list[str] = []
        self.tracks = [[[] for _ in range(agents)] for _ in range(arms)]
        self.sized = None
        self.camera = None

    def _draw(self, sizes):
        host = self.host
        for name in self.nodes:
            if host.scene.hasGraphics(name):
                host.scene.removeGraphics(name)
        self.nodes = []
        for j in range(self.arms):
            col = slot_rgb(j)
            for i in range(self.agents):
                length, width, _ = sizes[j][i]
                name = f"arm{j}_{i}"
                host.lab.add_mesh(name, *C._car_mesh(length, width, 0.4 * length), color=col)
                gn = host.scene.geometry_node(name)
                gn.setOpacity(0.6)
                host.lab.add_mesh(f"{name}_tag", *C._box(-0.35, -0.35, 0.35, 0.35, 0, 0.5),
                                  color=col)  # fmt: skip
                self.nodes += [name, f"{name}_tag"]
        u = float(max(s[0] for arm in sizes for s in arm))
        for i in range(self.agents):
            host.lab.add_mesh(f"goal_{i}", *C._box(-0.08 * u, -0.08 * u, 0.08 * u, 0.08 * u, 0,
                                                   2 * u), color=(0.95, 0.80, 0.10))  # fmt: skip
            self.nodes.append(f"goal_{i}")
        self.sized = sizes

    def update(self, poses, goals, follow=0, track_upto=None):
        """``poses`` (A, N, 6): x, y, heading, length, width, wheelbase; ``goals`` (N, 2);
        ``track_upto`` replaces the tracks with recorded ones ((A, N) lists of (x, y))."""
        import pycvc_gl
        from pycvc_gl.vehicle import seat_transform

        host, w = self.host, self.world
        sampler = None if w.height is None else w.z
        sizes = [[(round(float(p[3]), 3), round(float(p[4]), 3), float(p[5])) for p in arm]
                 for arm in poses]  # fmt: skip
        if self.sized is None or [[s[:2] for s in a] for a in sizes] != [
            [s[:2] for s in a] for a in self.sized
        ]:
            self._draw(sizes)
        centers = []
        for j in range(self.arms):
            for i in range(self.agents):
                x, y, h, _, _, wb = (float(v) for v in poses[j][i])
                cx, cy = x + 0.5 * wb * math.cos(h), y + 0.5 * wb * math.sin(h)
                centers.append((cx, cy))
                host.scene.getGraphics(f"arm{j}_{i}").setTransform(seat_transform(cx, cy, h,
                                                                                  sampler))
                host.scene.getGraphics(f"arm{j}_{i}_tag").setTransform(
                    seat_transform(cx, cy, h, sampler, lift=2.3 + 0.6 * j)
                )
                if track_upto is None and host.frame_count % 3 == 0:
                    self.tracks[j][i] = (self.tracks[j][i] + [(x, y)])[-600:]
        if track_upto is not None:
            self.tracks = track_upto
        for i, (gx, gy) in enumerate(goals):
            host.scene.getGraphics(f"goal_{i}").setTransform(
                [1, 0, 0, gx, 0, 1, 0, gy, 0, 0, 1, w.z(gx, gy), 0, 0, 0, 1]
            )
        if host.frame_count % 6 == 0 or track_upto is not None:
            for j in range(self.arms):
                for i in range(self.agents):
                    name, pts = f"arm{j}_{i}_track", self.tracks[j][i]
                    if host.scene.hasGraphics(name):
                        host.scene.removeGraphics(name)
                    if len(pts) >= 2:
                        host.lab.add_path(name, [(x, y, w.z(x, y) + 0.3) for x, y in pts],
                                          slot_rgb(j))  # fmt: skip
                        g = host.scene.geometry_node(name)
                        g.setRenderMode(pycvc_gl.GeometryRenderMode_LINES)
                        g.setLineWidth(2.5)
        cx, cy = centers[min(max(follow, 0), self.arms - 1) * self.agents]
        mode = host.get("view.camera", "follow")
        if mode == "follow":
            if self.camera != "follow":
                u = float(max(s[0] for a in sizes for s in a))
                # high behind the arm, looking down at it: over the buildings of a city street
                host.set_many({"back": 5.0 * u, "height": 7.0 * u, "look_ahead": 1.0 * u,
                               "look_up": 0.0, "cam_tau": 0.15, "pos_tau": 0.1},
                              prefix="viewers.main.camera.track")  # fmt: skip
            host.camera.setMode(pycvc_gl.CameraController.Mode_Track)
            host.camera.feedTrackTarget(cx, cy, w.z(cx, cy))
        elif self.camera != mode:
            mnx, mny, mxx, mxy = w.bounds
            host.camera.setMode(pycvc_gl.CameraController.Mode_Orbit)
            host.frame_bounds((mnx, mny, 0.0), (mxx, mxy, 40.0))
        self.camera = mode


def _follow(host, names) -> int:
    v = host.get("view.arm", "")
    return names.index(v) if v in names else 0


# ── the commands ───────────────────────────────────────────────────────────────────────────────


def _host(args, doc, name):
    from pycvc_gl.ariadne import Host

    host = Host(mode=args.mode, name=name, width=args.width, height=args.height,
                component_paths=[C.COMPONENTS])  # fmt: skip
    pycvc.data_allow_network(host.app, ["https://cvcpkg.org"])
    host.load(str(doc))
    return host


def run_arms(argv) -> int:
    from pycvc_gl.ariadne import main_args

    ns = main_args(argv)
    p = argparse.ArgumentParser(prog="python -m pycvc_gl.nav_lab arms")
    p.add_argument("--arms", default="default", help="a comma list: default, NAME=PATH.cvcnav")
    p.add_argument("--arm", action="append", default=[], help="one more arm, NAME=PATH.cvcnav")
    p.add_argument("--plant", default=None, help="nav_plant.vehicle (default the document's)")
    p.add_argument("--cars", type=int, default=1, help="agents per arm (default 1)")
    p.add_argument("--seed", type=int, default=0, help="the starts and goals")
    p.add_argument("--scale", type=float, default=C.SCALE,
                   help="the policies' frame, normalized units per metre (default 0.05)")
    p.add_argument("--retarget-at", type=float, action="append", default=[],
                   help="retarget every arm at this time (s; to arms.goal, else a drawn point)")
    C.add_args(p)
    args = p.parse_args(ns.rest)
    for k, v in vars(ns).items():
        if k != "rest":
            setattr(args, k, v)
    try:
        arms = parse_arms(args.arms, args.arm)
    except ValueError as e:
        return _refuse("arms", e)
    host = _host(args, ARMS_DOC, "nav_lab")
    C.apply_settings(host, args, {"nav_plant.vehicle": args.plant})
    plant = host.get("nav_plant.vehicle", "kinematic") or "kinematic"
    kind = host.get("run.world", "flat") or "flat"
    keys = world_keys(host)
    world = C.make_world(host, kind)
    drawn = host.mode != "headless"
    if drawn:
        C.show_world(host, kind, str(ARMS_DOC))
        C.draw_world(host, {"world": world})
    starts, goals = starts_and_goals(world, max(1, args.cars), args.seed)
    fleet = make_fleet(world, plant, arms, starts, goals, seed=args.seed, scale=args.scale,
                       app=host.app, prefix=host.prefix)  # fmt: skip
    run = pycvc.nav_arms_create([c.sim for c in fleet], arms, seed=args.seed, scene=world.kind,
                                plant=fleet[0].driving, world=json.dumps(keys),
                                settings=json.dumps(settings_of(host)),
                                meta=json.dumps({"scale": args.scale, "host": "pycvc_gl.nav_lab"}),
                                log_path=args.log or "")  # fmt: skip
    log = pycvc.nav_arms_log(run)
    names = [a["name"] for a in arms]
    host.set_many({"on": "1", "live": "1", "status": "", "names": "\n".join(names)},
                  prefix="arms")  # fmt: skip
    for key in ("view.arm", "ledger.agent"):
        if host.get(key, "") not in names:
            host.set(key, names[0])
    host.set("demo.title", f"{len(arms)} arms on {plant}: {', '.join(names)}")
    rng = np.random.default_rng(args.seed + 1)
    radius = 0.5 * float(fleet[0].width[0])
    st = {"pending": None, "due": sorted(args.retarget_at), "error": None, "frame": 0,
          "acc": 0.0}  # fmt: skip
    ghosts = Ghosts(host, world, len(arms), len(starts)) if drawn else None

    def retarget(to=None):
        """Every arm's goals move before the next tick: agent 0's to ``to``, else fresh ones."""
        n = len(starts)
        g = np.asarray(fleet[0].goals, float).copy()
        if to is not None:
            why = check_goal(world, to[0], to[1], radius)
            if why:
                host.set("arms.status", f"not there: {why}")
                return
            g[0] = to
        else:
            pos = np.stack([fleet[0].state().x, fleet[0].state().y], 1)
            g = world.free_points(rng, n, 0.8 * C._BICYCLE["length"], 4.0 * C._BICYCLE["length"],
                                  avoid=pos, avoid_dist=0.3 * world.span)  # fmt: skip
        for cars in fleet:
            cars.goals = np.asarray(g, float).copy()
        pycvc.nav_arms_retarget(run, np.asarray(g, np.float32))
        legs = len(pycvc.nav_run_log_legs(log))
        host.set("arms.status", f"leg {legs}: every arm to new goals from tick "
                                f"{pycvc.nav_arms_tick(run)}")  # fmt: skip

    host.on("arms.retarget", lambda: st.update(pending=parse_point(host.get("arms.goal")) or "draw"))
    host.on("arms.seek", lambda: None)  # a live run has no play head to move
    host.on("sim.step", lambda: st.update(step=True))
    host.on("sim.restart", lambda: host.set("arms.status", "a live run does not restart"))

    @host.on_frame
    def tick(dt):
        try:
            if st["pending"] is not None:  # between ticks: every arm's goals move at once
                to, st["pending"] = st["pending"], None
                retarget(None if to == "draw" else to)
            for cars in fleet:
                cars.pull()
            n = 0  # ticks this frame: one a frame headless, else as the clock and Speed say
            if host.get_bool("sim.paused"):
                n = int(st.pop("step", False))
            elif not drawn:
                n = 1
            else:
                st["acc"] += dt * max(host.get_float("sim.speed", 1.0), 0.0)
                n = min(int(st["acc"] / C.DT), 8)
                st["acc"] = 0.0 if n == 8 else st["acc"] - n * C.DT
            for _ in range(n):
                if st["due"] and pycvc.nav_arms_tick(run) * C.DT >= st["due"][0] - 1e-9:
                    st["due"].pop(0)
                    retarget(parse_point(host.get("arms.goal")))
                for j, cars in enumerate(fleet):
                    pycvc.nav_arms_set_size(run, j, *(np.asarray(v, np.float32) for v in
                                                      (cars.length, cars.width, cars.wheelbase)))
                pycvc.nav_arms_step(run)
                for cars in fleet:
                    cars.stepped()
            fleet[0].publish()
            charts = hasattr(host, "set_plot") and (st["frame"] % CHART_EVERY == 0)
            pycvc.nav_run_log_publish(host.app, host.prefix, log, -1, charts=charts)
            k = pycvc.nav_run_log_ticks(log)
            legs = len(pycvc.nav_run_log_legs(log))
            host.set("arms.time", f"t = {k * C.DT:.1f} s, tick {k}, leg {legs}")
            st["frame"] += 1
            if ghosts is not None and k:
                poses = pycvc.nav_run_log_poses(log, k * C.DT)
                ghosts.update(poses, fleet[0].goals, _follow(host, names))
        except Exception as e:  # noqa: BLE001 - a failure ends the run with its reason
            import traceback

            traceback.print_exc()
            st["error"] = str(e)
            host.request_close()

    host.run(fps=30, frames=args.frames or None)
    pycvc.nav_arms_finish(run)
    if args.png and drawn:
        host.snapshot(args.png)
    if args.log:
        print(f"nav_lab arms: {pycvc.nav_run_log_ticks(log)} ticks of {len(arms)} arms to "
              f"{args.log}")  # fmt: skip
    rows = pycvc.nav_run_log_tables(log)["arms.stats"]
    arrivals = 0
    for r in rows:
        print(f"nav_lab arms: {r['arm']:>12}: clearance {r['clear_min']} / {r['clear_mean']} m, "
              f"{r['pen']} penetrating, {r['path']} m in {r['elapsed']} s, {r['ms']} ms/tick, "
              f"reached {r['reached']}")  # fmt: skip
        arrivals += r["reached"] == "yes" or (r["reached"].split("/")[0].isdigit()
                                              and int(r["reached"].split("/")[0]) > 0)
    code = host.exit_code()
    host.close()
    return C.exit_code(code, error=st["error"], arrivals=arrivals,
                       min_arrivals=args.min_arrivals, who="nav_lab arms")  # fmt: skip


def run_replay(argv) -> int:
    from pycvc_gl.ariadne import main_args

    ns = main_args(argv)
    p = argparse.ArgumentParser(prog="python -m pycvc_gl.nav_lab replay")
    p.add_argument("log", help="a run log (cvc.nav.run_log/v1)")
    p.add_argument("--at", type=float, default=0.0, help="start the play head here (s)")
    p.add_argument("--loop", action="store_true", help="play it again from the start at the end")
    p.add_argument("--paused", action="store_true", help="start paused (at --at)")
    p.add_argument("--camera", choices=("follow", "orbit"), default=None)
    args = p.parse_args(ns.rest)
    for k, v in vars(ns).items():
        if k != "rest":
            setattr(args, k, v)
    try:
        log = pycvc.nav_run_log_read(args.log)
    except RuntimeError as e:
        return _refuse("replay", e)
    head = json.loads(pycvc.nav_run_log_header(log))
    if not pycvc.nav_run_log_ticks(log) or not head.get("arms"):
        return _refuse("replay", f"{args.log}: the run has no ticks")
    host = _host(args, REPLAY_DOC, "nav_lab")
    keys = head.get("world") or {}
    host.set_many({k: v for k, v in (keys.get("run") or {}).items() if v not in (None, "")},
                  prefix="run")  # fmt: skip
    host.set_many({k: v for k, v in (keys.get("world") or {}).items() if v not in (None, "")},
                  prefix="world")  # fmt: skip
    names = [a["name"] for a in head["arms"]]
    host.set_many({"names": "\n".join(names), "status": "", "on": "1", "live": "0"}, prefix="arms")
    host.set("view.arm", names[0])
    host.set("ledger.agent", names[0])
    if args.loop:
        host.set("play.loop", "1")
    if args.paused:
        host.set("sim.paused", "1")
    if args.camera:
        host.set("view.camera", args.camera)
    host.set("replay.title", f"{head.get('scene', '?')} on {head.get('plant', '?')}: "
                             f"{len(names)} arms, {', '.join(names)}; seed {head.get('seed')}")  # fmt: skip
    ticks = pycvc.nav_run_log_ticks(log)
    host.set("replay.made", f"libcvc {head.get('libcvc') or '?'}; {ticks} ticks of "
                            f"{head.get('dt', 0):g} s, {len(pycvc.nav_run_log_legs(log))} legs")  # fmt: skip
    view = pycvc.nav_replay_view_create(log, args.at)
    host.on("sim.step", lambda: pycvc.nav_replay_view_step(view))
    host.on("sim.restart", lambda: pycvc.nav_replay_view_restart(view, host.app, host.prefix))
    host.on("arms.seek", lambda: pycvc.nav_replay_view_seek(view, host.app, host.prefix))
    host.on("arms.retarget", lambda: None)
    drawn = host.mode != "headless"
    ghosts = None
    a = pycvc.nav_run_log_arrays(log) if drawn else None
    if drawn:
        kind = (keys.get("run") or {}).get("world", "flat")
        world = C.make_world(host, kind)
        C.show_world(host, kind, str(REPLAY_DOC))
        C.draw_world(host, {"world": world})
        ghosts = Ghosts(host, world, len(names), int(head.get("agents", 1)))
    charts = hasattr(host, "set_plot")
    every = max(1, ticks // 600)

    @host.on_frame
    def frame(dt):
        f = pycvc.nav_replay_view_update(view, host.app, host.prefix, dt, charts=charts)
        if ghosts is not None:
            k = f["k"]
            tracks = [[list(zip(a["x"][: k + 1 : every, j, i].tolist(),
                                a["y"][: k + 1 : every, j, i].tolist(), strict=True))
                       for i in range(a["x"].shape[2])] for j in range(a["x"].shape[1])]  # fmt: skip
            goals = np.stack([a["goal_x"][k, 0], a["goal_y"][k, 0]], 1)
            ghosts.update(f["poses"], goals, _follow(host, names), track_upto=tracks)

    host.run(fps=30, frames=args.frames or None)
    if args.png and drawn:
        host.snapshot(args.png)
    code = host.exit_code()
    host.close()
    return code


def grid_world(occ, bounds, kind="grid", height=1.5) -> C.World:
    """A ``pycvc_gl.cars.World`` from an occupancy grid over ``bounds``, its blocked cells drawn
    as boxes ``height`` m tall (each row's runs merged), on flat ground."""
    occ = np.ascontiguousarray(np.asarray(occ, np.uint8))
    rows, cols = occ.shape
    mnx, mny, mxx, mxy = bounds
    cx, cy = (mxx - mnx) / (cols - 1), (mxy - mny) / (rows - 1)
    boxes = []
    for r in range(rows):
        c = 0
        while c < cols:
            if not occ[r, c]:
                c += 1
                continue
            c0 = c
            while c < cols and occ[r, c]:
                c += 1
            boxes.append((mnx + (c0 - 0.5) * cx, mny + (r - 0.5) * cy, mnx + (c - 0.5) * cx,
                          mny + (r + 0.5) * cy, height))  # fmt: skip
    w = C.World("flat", occ, tuple(float(b) for b in bounds), None, None, {"source": kind}, boxes)
    return w


def run_scrub(argv) -> int:
    from pycvc_gl.ariadne import main_args

    ns = main_args(argv)
    p = argparse.ArgumentParser(prog="python -m pycvc_gl.nav_lab scrub")
    p.add_argument("history", help="a training history (JSON lines) whose checkpoints name a "
                                   ".cvcnav blob: coef_trainer's history_path, or GRL-SNAM's")  # fmt: skip
    p.add_argument("--run", type=int, default=-1, help="which run in the file (default the last)")
    p.add_argument("--scenario", default="", help="an input trace whose world to deploy on "
                                                  "(default: the built-in blocks)")  # fmt: skip
    p.add_argument("--agents", type=int, default=4, help="the built-in scenario's agents")
    p.add_argument("--seed", type=int, default=0, help="the built-in scenario's seed")
    p.add_argument("--ticks", type=int, default=600, help="each checkpoint's ticks (0.05 s each)")
    p.add_argument("--no-cache", action="store_true", help="deploy every checkpoint again")
    p.add_argument("--report", default=None, help="write every checkpoint's metrics (JSON)")
    p.add_argument("--at", type=int, default=None, help="start at checkpoint k (default the pick)")
    p.add_argument("--plant", action="append", default=None,
                   help="what the agents drive, more than once for several (pycvc.plant_host's "
                        "names; default humvee and kinematic, or kinematic without Jolt)")  # fmt: skip
    args = p.parse_args(ns.rest)
    for k, v in vars(ns).items():
        if k != "rest":
            setattr(args, k, v)
    plants = list(dict.fromkeys(args.plant or (["humvee", "kinematic"]
                                               if pycvc.plant_host.available() else ["kinematic"])))  # fmt: skip
    try:
        s = pycvc.nav_scrub_create(args.history, run=args.run, scenario=args.scenario,
                                   agents=args.agents, seed=args.seed, ticks=args.ticks,
                                   use_cache=not args.no_cache,
                                   k=-1 if args.at is None else args.at, plants=plants)  # fmt: skip
    except RuntimeError as e:
        return _refuse("scrub", e)
    info = pycvc.nav_scrub_info(s)
    for line in info["lines"]:
        print(line)
    if info["missing"]:
        print(f"scrub: {len(info['missing'])} saved policies are not there (first: "
              f"{info['missing'][0]})", file=sys.stderr)  # fmt: skip
    if args.report:
        Path(args.report).write_text(pycvc.nav_scrub_report(s))
        print(f"scrub: {len(info['checkpoints'])} checkpoints on {info['plant']} to {args.report}")
    host = _host(args, SCRUB_DOC, "nav_lab")
    host.on("scrub.prev", lambda: pycvc.nav_scrub_prev(s))
    host.on("scrub.next", lambda: pycvc.nav_scrub_next(s))
    host.on("scrub.pick_step", lambda: pycvc.nav_scrub_pick_step(s, host.app, host.prefix))
    host.on("sim.step", lambda: pycvc.nav_scrub_step(s))
    host.on("sim.restart", lambda: pycvc.nav_scrub_restart(s, host.app, host.prefix))
    host.on("arms.seek", lambda: pycvc.nav_scrub_seek(s, host.app, host.prefix))
    host.on("arms.retarget", lambda: None)
    drawn = host.mode != "headless"
    ghosts, world = None, None
    if drawn:
        occ, bounds = pycvc.nav_scrub_world(s)
        world = grid_world(occ, bounds)
        C.show_world(host, "flat", str(SCRUB_DOC))
        C.draw_world(host, {"world": world})
    charts = hasattr(host, "set_plot")
    st = {"shown": None, "log": None, "a": None}

    @host.on_frame
    def frame(dt):
        f = pycvc.nav_scrub_view_update(s, host.app, host.prefix, dt, charts=charts)
        if world is None:
            return
        nonlocal ghosts
        if st["shown"] != (f["checkpoint"], f["plant"]):  # the pair's recorded tracks
            st["shown"] = (f["checkpoint"], f["plant"])
            a0 = pycvc.nav_run_log_arrays(pycvc.nav_scrub_log(s, 0))
            ak = pycvc.nav_run_log_arrays(pycvc.nav_scrub_log(s, f["checkpoint"]))
            st["a"] = (a0, ak)
            if ghosts is None:
                ghosts = Ghosts(host, world, 2, a0["x"].shape[2])
        k = f["k"]
        every = max(1, len(st["a"][0]["t"]) // 600)
        tracks = [[list(zip(a["x"][: k + 1 : every, 0, i].tolist(),
                            a["y"][: k + 1 : every, 0, i].tolist(), strict=True))
                   for i in range(a["x"].shape[2])] for a in st["a"]]  # fmt: skip
        a0 = st["a"][0]
        goals = np.stack([a0["goal_x"][k, 0], a0["goal_y"][k, 0]], 1)
        ghosts.update(f["poses"], goals, 1, track_upto=tracks)

    host.run(fps=30, frames=args.frames or None)
    if args.png and drawn:
        host.snapshot(args.png)
    code = host.exit_code()
    host.close()
    return code


COMMANDS = {"arms": run_arms, "replay": run_replay, "scrub": run_scrub}


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] not in COMMANDS:
        print("usage: python -m pycvc_gl.nav_lab {arms|replay|scrub} ...  (docs/NAV_LAB.md)",
              file=sys.stderr)  # fmt: skip
        return 2
    return COMMANDS[argv[0]](argv[1:])


if __name__ == "__main__":
    sys.exit(main())
