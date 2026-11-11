"""pycvc_gl.ariadne.Host: an Ariadne document run by a Python program.

Headless (no GL; runs in this process):
  * data in: set / set_many / set_tree / set_table / set_series / publish_image (kept headless);
  * data out: get / get_float / get_int / get_bool / get_tree / get_series, and watch callbacks;
  * the document calls Python: on(), function(), async_function(), with values both ways;
  * errors: an on() handler's exception is raised from step(); a function's is the document's
    PythonError, and Host(strict=True) raises the warning it leaves as an AriadneError;
  * re-entrancy: step() inside a callback raises, load() inside one is deferred;
  * run(until=...), settle(), (quit code) -> exit_code(), close() twice, the context manager.

Offscreen (GL; in a child process, skipped without a GL context unless CVC_REQUIRE_RENDER=1):
  * a published picture and a live stream show up in the rendered frame (frame_rgb);
  * snapshot() writes a PNG; host.lab adds a mesh to the document's scene;
  * the four examples in examples/ariadne/python run offscreen and do what they say.
"""

import importlib.util
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
EXAMPLES = os.path.normpath(os.path.join(HERE, "..", "..", "examples", "ariadne", "python"))
_ASSERT_RC = 3  # the child's exit code for a real failure (not a missing GL)

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


DOC = """meta:
  name: Host test
  min_libcvc: 3.4.0
init: |
  (begin
    (state-set "count" "0")
    (state-set "flag" "true"))
windows:
  - window: Main
    id: main
    pos: [0, 0]
    size: [520, 460]
    children:
      - button: Ping
        on: ping
      - text: count
        bind: count
      - image: Picture
        src: pic
        size: 200
      - type: stream_view
        stream: streams.cam
        size: 200
customs:
  - widget: stream_view
"""


def write_doc(tmp, text=DOC, name="doc.ari"):
    path = os.path.join(tmp, name)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    return path


def headless(tmp):
    from pycvc_gl.ariadne import AriadneError, Host

    doc = write_doc(tmp)
    host = Host(mode="headless", name="h")
    check(host.prefix == "h", "the prefix is the host's name")

    # ── data in, data out ──
    host.set("a.int", 7)
    host.set("a.float", 0.1)
    host.set("a.bool", True)
    host.set("a.text", "hi")
    host["a.none"] = None
    check(host.get("a.int") == "7" and host.get_int("a.int") == 7, "an int")
    check(host.get("a.float") == "0.1" and host.get_float("a.float") == 0.1, "a float, round trip")
    check(host.get("a.bool") == "1" and host.get_bool("a.bool"), "a bool as a widget writes it")
    check(host["a.text"] == "hi" and "a.text" in host and "a.nope" not in host, "text, in")
    check(host.get("a.nope", "dflt") == "dflt", "a missing key's default")
    import numpy as np

    host.set("a.np", np.float32(1.5))
    check(host.get_float("a.np") == 1.5, "a numpy scalar")
    host.set_many({"x": 1, "y": 2}, prefix="pos")
    check(host.get_tree("pos") == {"x": "1", "y": "2"}, "set_many under a prefix")
    host.set_tree("t", {"rows": [{"n": 1}, {"n": 2}], "name": "z"})
    check(host.get("t.rows") == "2", "a list's node holds its length (what repeat: counts)")
    check(host.get_tree("t") == {"rows": [{"n": "1"}, {"n": "2"}], "name": "z"}, "get_tree")
    host.set_tree("t", [3])
    check(host.get_tree("t") == ["3"], "set_tree replaces the whole subtree")
    host.set_table("tab", [[1, "a"], [2, "b"]], columns=["k", "v"])
    check(host.get_tree("tab") == [{"k": "1", "v": "a"}, {"k": "2", "v": "b"}], "set_table rows")
    check(host.get_tree("tab_columns") == ["k", "v"], "set_table keeps the columns")
    host.set_series("s", np.array([1.0, 2.5, -3.0]), limit=2)
    check(host.get("s") == "2.5,-3", "set_series: comma text, the last `limit`")
    check(list(host.get_series("s")) == [2.5, -3.0], "get_series")
    host.set("s2", np.arange(3.0))
    check(host.get("s2") == "0,1,2", "a 1-D array is a series")
    for bad in ({"a.b": 1}, {"": 1}, {" a": 1}):
        try:
            host.set_tree("bad", bad)
        except ValueError:
            checks_ok = True
        else:
            checks_ok = False
        check(checks_ok, f"set_tree refuses the key {list(bad)[0]!r}")
    host.publish_image("pic", np.zeros((4, 6, 3), np.float32) + 0.5)
    img = host.images["pic"]
    check(img.shape == (4, 6, 4) and img.dtype == np.uint8 and img[0, 0, 0] == 128,
          "publish_image: float RGB to uint8 RGBA, kept headless")

    # ── the document calls Python ──
    events, watched = [], []

    @host.on("ping")
    def ping():
        events.append("ping")
        host.set("count", host.get_int("count") + 1)

    @host.function("add")
    def add(a, b):
        return np.float64(a) + b  # a numpy value goes back as a number

    @host.function("stats")
    def stats(xs):
        return {"n": len(xs), "mean": float(np.mean(xs)), "arr": np.arange(3)}

    @host.async_function("later")
    def later(x):
        time.sleep(0.05)
        return x + 1

    @host.watch("count")
    def on_count(v):
        watched.append(v)

    host.load(doc)
    check(host.get_bool("flag"), "init: wrote \"true\", which reads as true")
    host.run_program('(state-set "sum" (add 2 3.5))')
    host.run_program('(state-set-tree "st" (stats (list 1 2 3)))')
    host.run_program('(state-set "l" (later 41))')
    check(host.settle(), "settles once the async call is back")
    check(host.get("sum") == "5.5", "a function's value")
    check(host.get_tree("st") == {"n": "3", "mean": "2", "arr": ["0", "1", "2"]},
          "a dict with a numpy array back (the DSL writes 2.0 as 2)")
    check(host.get("l") == "42", "an async function's value")
    host.emit("ping")  # as the Ping button raises it
    host.step()
    check(events == ["ping"] and host.get("count") == "1", "on(): the handler ran")
    # init: wrote "0" (a change from no node at all), then the handler wrote "1".
    check(watched == ["0", "1"], f"watch: called once per change, with the value ({watched})")
    host.step()
    check(watched == ["0", "1"], "watch: not again without a change")

    # ── errors ──
    @host.on("bad")
    def bad():
        raise KeyError("from a handler")

    host.emit("bad")
    try:
        host.step()
    except KeyError:
        ok = True
    else:
        ok = False
    check(ok, "an on() handler's exception is raised from step()")

    @host.function("oops")
    def oops():
        raise ValueError("from a function")

    host.run_program('(try (oops) (catch (PythonError e) (state-set "caught" "1")))')
    host.settle()
    check(host.get("caught") == "1", "a function's exception is a PythonError a try catches")
    host.strict = True
    host.run_program("(oops)")
    try:
        host.settle()
    except AriadneError as e:
        ok = isinstance(e.__cause__, ValueError) and "from a function" in str(e)
    else:
        ok = False
    check(ok, "strict: an uncaught one is an AriadneError, the Python exception its cause")
    host.strict = False
    host.warnings.clear()
    host.run_program("(oops)")
    host.settle()
    check(any("from a function" in w for w in host.warnings), "not strict: it is a warning")

    # ── re-entrancy ──
    nested = []

    @host.on_frame
    def nest(dt):
        if not nested:
            try:
                host.step()
            except RuntimeError as e:
                nested.append(str(e))
            nested.append(host.load(doc))  # deferred to the frame's end
    host.step()
    check(nested and "inside a frame" in nested[0] and nested[1] == [], "step() in a frame raises")
    check(host.document == doc, "load() in a frame ran once the frame ended")
    host._frame_hooks.remove(nest)

    # ── run, quit, close ──
    host.run_program("(quit 4)")
    n = host.run(frames=100)
    check(n == 4 and host.should_close(), "(quit 4): run() returns 4")
    host.close()
    host.close()
    check(host.exit_code() == 4, "the exit code survives close()")
    try:
        host.step()
    except RuntimeError:
        ok = True
    else:
        ok = False
    check(ok, "a closed host refuses step()")
    with Host(mode="headless", name="ctx") as h2:
        h2.load(doc)
        h2.run(frames=3)
    check(h2.should_close(), "the context manager closes")


def run_example(name, argv):
    """Run examples/ariadne/python/<name>.py's main(argv); the document's state at close, and rc."""
    from pycvc_gl.ariadne import Host

    spec = importlib.util.spec_from_file_location(f"ex_{name}",
                                                  os.path.join(EXAMPLES, f"{name}.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    captured = {}
    real_close = Host.close

    def keep(self, _real=real_close):  # read the document's state before it closes
        if not self._closed:
            captured.update({k: self.get(k) for k in ("clicks", "stats", "spread.n",
                                                       "stats.trips", "stats.time",
                                                       "stats.plant", "stats.world")})
        _real(self)

    Host.close = keep
    try:
        captured["rc"] = mod.main(argv)
    finally:
        Host.close = real_close
    return captured


def vehicles(tmp):
    """pycvc_gl.vehicle: VehicleModel's normalization, seat_transform and body_transform."""
    import math

    import pycvc
    from pycvc_gl.vehicle import VehicleModel, body_transform, seat_transform

    m = seat_transform(3.0, 4.0, math.pi / 2)
    check(abs(m[0]) < 1e-12 and abs(m[4] - 1.0) < 1e-12 and m[3] == 3.0 and m[7] == 4.0,
          "seat_transform: heading 90 degrees turns +X to +Y, on flat ground at z 0")
    m = seat_transform(0.0, 0.0, 0.0, lambda x, y: 0.5 * x, lift=0.0)
    check(m[8] > 0.4 and abs(m[11]) < 1e-12, "seat_transform: pitched up a 0.5 slope")
    q = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))  # yaw 90 degrees
    m = body_transform((1.0, 2.0, 3.0), q, ground_z=0.25)
    check(abs(m[0]) < 1e-12 and abs(m[4] - 1.0) < 1e-12 and m[11] == 0.25,
          "body_transform: the quaternion's yaw, the ground's z")

    # A 10 m long box along y: a tall cabin at -y (more vertices), a low hood at +y.
    verts = []
    for y0, y1, h in ((-5, 0, 3.0), (0, 5, 1.5)):
        for z in (0.0, h):
            verts += [(-2, y0, z), (2, y0, z), (2, y1, z), (-2, y1, z)]
    verts += [(-2, -2.5, 3.0), (2, -2.5, 3.0), (-2, -2.5, 0.0), (2, -2.5, 0.0)]
    tris = []
    for b in (0, 8):
        for f in ((4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7),
                  (2, 7, 6), (3, 0, 4), (3, 4, 7)):
            tris.append(tuple(b + k for k in f))
    tris += [(16, 17, 4), (18, 19, 0)]
    obj = os.path.join(tmp, "car.obj")
    with open(obj, "w") as f:
        f.writelines(f"v {x} {y} {z}\n" for x, y, z in verts)
        f.writelines(f"f {a + 1} {b + 1} {c + 1}\n" for a, b, c in tris)
    try:
        car = VehicleModel(pycvc.make_app(), obj, 4.72)
    except Exception as e:  # noqa: BLE001 - a pycvc built without its model loader
        print("  skip VehicleModel:", e)
        return
    import numpy as np

    v = np.asarray(car.geometry.vertices())
    check(abs(v[:, 0].max() - v[:, 0].min() - 4.72) < 1e-6, "scaled to 4.72 m along +X")
    check(abs(v[:, 2].min()) < 1e-9 and abs(v[:, 1].mean()) < 1.0, "base at z 0, centred")
    check(v[v[:, 0] > 0.5, 2].max() < v[v[:, 0] < -0.5, 2].max(), "the low hood is the nose (+X)")
    check(abs(car.size[0] - 4.72) < 1e-6 and car.size[2] < car.size[1], "size: length, width, height")


def gl_child(tmp):
    """Runs in the child: everything that needs a GL context."""
    import numpy as np

    from pycvc_gl.ariadne import Host

    doc = write_doc(tmp)
    # 1024 x 768: a view whose shorter side is under 700 px starts with the windows hidden (§5.14)
    host = Host(mode="offscreen", name="g", width=1024, height=768)
    print("WINDOW_OK", flush=True)  # a GL context exists past here: a failure is a real bug
    try:
        stream = host.open_stream("cam", 64, 48)
        host.load(doc)
        pic = np.zeros((32, 32, 3), np.uint8)
        pic[...] = (255, 0, 255)  # magenta
        host.publish_image("pic", pic)
        green = np.zeros((48, 64, 4), np.uint8)
        green[...] = (0, 255, 0, 255)
        for _ in range(6):
            stream.publish(green.copy(), 0.0)
            host.step()
        rgb = host.frame_rgb()
        check(rgb.shape == (768, 1024, 3), "frame_rgb's shape")
        mag = np.all(rgb == (255, 0, 255), axis=2).sum()
        grn = ((rgb[..., 1] > 200) & (rgb[..., 0] < 60) & (rgb[..., 2] < 60)).sum()
        check(mag > 2000, f"the published picture is in the frame ({mag} magenta pixels)")
        check(grn > 2000, f"the stream's frames are in the frame ({grn} green pixels)")
        png = os.path.join(tmp, "frame.png")
        host.snapshot(png)
        check(os.path.getsize(png) > 0, "snapshot() wrote a PNG")
        host.lab.add_mesh("tri", [0, 0, 0, 1, 0, 0, 0, 1, 0], [0, 1, 2], color=(1, 0, 0))
        check(host.scene.hasGraphics("tri"), "host.lab adds to the document's scene")
        host.close()

        # The examples, offscreen, as a user runs them (nav on its block city: no bundle here).
        for name, frames, extra, expect in (
                ("hello", 10, [], ("clicks", "2")),
                ("data_feed", 20, [], ("stats", "2")),
                ("form", 10, [], ("spread.n", "400"))):
            captured = run_example(name, ["--offscreen", "--frames", str(frames), "--png",
                                          os.path.join(tmp, f"{name}.png")] + extra)
            check(captured["rc"] == 0 and os.path.getsize(os.path.join(tmp, f"{name}.png")) > 0,
                  f"example {name} ran offscreen")
            check(captured[expect[0]] == expect[1], f"example {name}: {expect} ({captured})")
        for plant in ("kinematic", "jolt"):  # nav: the fleet, the routes, the readouts, drawn
            png = os.path.join(tmp, f"nav_{plant}.png")
            got = run_example("nav", ["--offscreen", "--frames", "60", "--png", png, "--no-bundle",
                                      "--plant", plant])
            check(got["rc"] == 0 and os.path.getsize(png) > 0, f"nav ({plant}) ran offscreen")
            check(float(got["stats.time"]) > 0, f"example nav ({plant}) stepped ({got})")
            check(got["stats.plant"] in (plant, "kinematic"), f"example nav's plant ({got})")
    except AssertionError as e:
        print("RENDER_FAIL:", e, flush=True)
        sys.exit(_ASSERT_RC)
    print(f"RENDER_OK {checks}", flush=True)


def main():
    tmp = tempfile.mkdtemp(prefix="pycvc_gl_ariadne_host_")
    if os.environ.get("_CVC_ARI_HOST_CHILD") == "1":
        gl_child(tmp)
        return
    headless(tmp)
    vehicles(tmp)
    try:  # nav.ari requires gl.scene, so nav.py says so instead of running headless
        run_example("nav", ["--headless", "--frames", "1", "--no-bundle"])
        check(False, "nav.py --headless should refuse")
    except SystemExit as e:
        check("gl.scene" in str(e.code), f"nav.py --headless names gl.scene ({e.code!r})")
    print(f"  ok: headless, {checks} checks")
    env = dict(os.environ, _CVC_ARI_HOST_CHILD="1")
    p = subprocess.run([sys.executable, os.path.abspath(__file__)], env=env, capture_output=True,
                       text=True, timeout=900)
    sys.stdout.write(p.stdout)
    if "RENDER_OK" in p.stdout:
        print("pycvc_gl.ariadne Host test: OK")
    elif "WINDOW_OK" in p.stdout or p.returncode == _ASSERT_RC:
        print("pycvc_gl.ariadne Host test: FAILED (rc=%d)\n%s" % (p.returncode, p.stderr))
        sys.exit(1)
    elif os.environ.get("CVC_REQUIRE_RENDER", "0") not in ("", "0"):
        print("pycvc_gl.ariadne Host test: FAILED, no usable GL context under CVC_REQUIRE_RENDER "
              "(child rc=%d)\n%s" % (p.returncode, p.stderr))
        sys.exit(1)
    else:
        print("SKIP the offscreen half: no usable GL context (child rc=%d)" % p.returncode)
        print("pycvc_gl.ariadne Host test: OK (headless only)")


if __name__ == "__main__":
    main()
