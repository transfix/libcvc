"""pycvc.AriApp and pycvc.host_function: an Ariadne document's programs calling Python, with no GL.

What a Python host relies on, checked headless (no window, no GL, so it runs in every lane that
builds pycvc):
  * a host function gets the call's arguments as Python values and its value goes back;
  * its exception raises PythonError in the document, which a try catches; one that is not caught
    stops the program, and the host hears why;
  * an async function runs on a pool thread while frames keep running (the frame does not hold the
    GIL), parks its program until it returns, and posts to a done channel;
  * on() handlers, run_program's completion, (quit code), re-entrancy (a frame from inside a call
    raises) and wait_idle with a call in flight.
"""

import os
import sys
import tempfile
import threading
import time

import pycvc

if not hasattr(pycvc, "AriApp"):
    print("SKIP test_pycvc_ariadne: this pycvc has no AriApp")
    sys.exit(0)

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


DOC = """meta:
  name: pycvc_ariadne test
  min_libcvc: 3.4.0
init: |
  (state-set "status" "ready")
windows:
  - window: W
    children:
      - text: status
        bind: status
      - button: Ping
        on: ping
"""

tmp = tempfile.mkdtemp(prefix="pycvc_ariadne_")
doc = os.path.join(tmp, "doc.ari")
with open(doc, "w", encoding="utf-8") as f:
    f.write(DOC)


def make(prefix="t", limits=True):
    app = pycvc.make_app()
    if limits:  # a Python call may take its time (docs/ARIADNE.md 2.2's caps)
        pycvc.state_set(app, "ariadne.limits.action_max_time", "30")
        pycvc.state_set(app, "ariadne.limits.action_max_steps", "0")
    return app, pycvc.AriApp(app, prefix)


def run(rt, script, frames=200, until=None):
    """Queue `script`, then run frames until it ends; its `done` text."""
    out = []
    rt.run_program(script, out.append)
    for _ in range(frames):
        rt.step(1.0 / 60.0)
        if out and (until is None or until()):
            break
        time.sleep(0.001)
    return out[0] if out else None


# ── a synchronous function: values in and out ────────────────────────────────────────────────────
app, rt = make()
seen = []


def echo(*args):
    seen.append(args)
    return {"n": len(args), "first": args[0] if args else None, "list": [1, 2.5, "x"]}


rt.register_host_function(pycvc.host_function("echo", echo))
check(list(rt.load(doc)) == [], "the document loads cleanly")
check(rt.has_function("echo"), "has? sees a registered function")
how = run(rt, '(state-set-tree "r" (echo 7 2.5 "s" #t (list 1 2) (dict "a" 1) nil))')
check(how == "", f"the program finished ({how!r})")
args = seen[0]
check(args[0] == 7 and isinstance(args[0], int), "an int arrives as int")
check(args[1] == 2.5 and args[2] == "s" and args[3] is True, "float, str and bool arrive")
check(args[4] == [1, 2] and args[5] == {"a": 1} and args[6] is None, "list, dict and nil arrive")
check(pycvc.state_get(app, "t.r.n") == "7", "the returned dict is written back")
check(pycvc.state_get(app, "t.r.list.1") == "2.5", "with its nested list")
check(pycvc.state_get(app, "t.status") == "ready", "init: ran, under the prefix")

# A Python exception: PythonError in the document, which a try catches.
rt.register_host_function(pycvc.host_function("boom", lambda: 1 / 0))
how = run(rt, '(try (boom) (catch (PythonError e) (state-set "caught" (get-attr e "python_type"))))')
check(how == "", f"the try caught it ({how!r})")
check(pycvc.state_get(app, "t.caught") == "ZeroDivisionError", "its python_type names it")
how = run(rt, "(boom)")
check(how.startswith("failed: PythonError") and "division by zero" in how,
      f"uncaught, it fails: {how!r}")

# Re-entrancy: a frame from inside a call raises, as a PythonError in the document.
rt.register_host_function(pycvc.host_function("reenter", lambda: rt.step(0.0)))
how = run(rt, "(reenter)")
check(how.startswith("failed") and "inside a frame" in how, f"a nested frame is refused: {how!r}")
rt.step(0.0)  # and the runtime still runs frames afterwards

# on(): a bare event's handler, on the frame's thread.
pinged = []
rt.on("ping", lambda: pinged.append(threading.get_ident()))
rt.run_program("(begin)", None)  # a program that does nothing finishes too
rt.step(0.0)
check(rt.idle(), "idle once nothing runs")

# ── an async function: off the frame's thread, frames keep running ──────────────────────────────
main_thread = threading.get_ident()
info = {}


def slow(x, delay):
    info["thread"] = threading.get_ident()
    time.sleep(delay)
    return x * 2


rt.register_host_function(pycvc.host_function("slow", slow, is_async=True,
                                              done_channel="slow.done"))
frames = [0]
out = []
rt.run_program('(state-set "v" (slow 21 0.3))', out.append)
t0 = time.time()
while not out and time.time() - t0 < 20:
    rt.step(1.0 / 60.0)
    frames[0] += 1
    time.sleep(0.002)
check(out == [""], f"the async program finished ({out!r})")
check(pycvc.state_get(app, "t.v") == "42", "with the value")
check(info["thread"] != main_thread, "the function ran on a pool thread")
check(frames[0] > 20, f"frames kept running while it did ({frames[0]})")

# Its future form, and the done channel the value was posted to.
how = run(rt, '(state-set "w" (await (slow-async 5 0.01)))')
check(how == "" and pycvc.state_get(app, "t.w") == "10", "(slow-async ...) is a future")
how = run(rt, '(state-set "posted" (msg-recv "slow.done"))')
check(how == "" and pycvc.state_get(app, "t.posted") in ("42", "10"), "the done channel got it")

# with_call: the call first, then the arguments.
calls = []


def progress(call, n):
    calls.append(type(call).__name__)
    return n + 1


rt.register_host_function(pycvc.host_function("progress", progress, is_async=True,
                                              with_call=True))
how = run(rt, '(state-set "p" (progress 1))')
check(how == "" and pycvc.state_get(app, "t.p") == "2", "with_call passes the call first")
check(calls == ["async_call"], f"... a pycvc.async_call ({calls})")

# ── wait_idle with a call in flight, then teardown ───────────────────────────────────────────────
rt.run_program('(state-set "late" (slow 1 0.3))', None)
rt.step(0.0)
rt.step(0.0)
t0 = time.time()
rt.wait_idle()  # releases the GIL: the call's Python finishes
check(0.05 < time.time() - t0 < 10, "wait_idle waited for the call in flight")
del rt
del app

# ── (quit code) and the caps a Python host raises ───────────────────────────────────────────────
app, rt = make("q")
rt.register_quit()
rt.load(doc)
run(rt, "(quit 3)")
rt.step(0.0)
check(rt.should_close() and rt.exit_code() == 3, "(quit 3) closes with code 3")
del rt

app, rt = make("caps", limits=False)  # Ariadne's own 0.1 s cap
rt.register_host_function(pycvc.host_function("slow", slow, is_async=True))
rt.load(doc)
how = run(rt, '(slow 1 0.4)', frames=2000)
check(how is not None and "did not complete" in how, f"the default caps stop a slow call: {how!r}")
rt.wait_idle()
del rt

# ── sim.step / sim.restart: the built-in Step and Restart ──────────────────────────────────────
world_doc = os.path.join(tmp, "world.ari")
with open(world_doc, "w", encoding="utf-8") as f:
    f.write("scene:\n  clock: { paused: true }\n  physics: {}\n  nodes:\n"
            "    - node: crate\n      source: { box: { size: [1, 1, 1] } }\n"
            "      transform: { position: [0, 0, 4] }\n      body: { mass: 10 }\n")
app, rt = make("w")
rt.load(world_doc)
rt.step(1.0 / 60.0)  # the world anchors on the paused clock
rt.step_world(12)
rt.step(1.0 / 60.0)
check(pycvc.state_get(app, "w.sim.step") == "12", "step_world bumps the scene clock's step key")
ticks = pycvc.state_get(app, "w.physics.stats.tick")
check(ticks not in ("", "0"), f"the paused world ran the requested steps ({ticks!r})")
check(rt.restart_world(), "restart_world rebuilds the world")
check(pycvc.state_get(app, "w.sim.restarts") == "1", "and counts the restart")
rt.step(1.0 / 60.0)
rt.step_world(12)
rt.step(1.0 / 60.0)
check(pycvc.state_get(app, "w.physics.stats.tick") == ticks, "a restarted world steps as before")
check(rt.grid("crate", 1.0) is None, "a node with a body of its own makes no world to grid")
del rt

# ── AriApp.grid: the navigator's grid of a generated world the headless runtime runs ────────────
gen_doc = os.path.join(tmp, "generated.ari")
with open(gen_doc, "w", encoding="utf-8") as f:
    f.write("scene:\n  physics: {}\n  nodes:\n    - node: land\n      type: generated_world\n"
            "      seed: 5\n      half: 150\n      trees: 4\n      rocks: 2\n      buildings: 4\n"
            "      body: {}\n")
app, rt = make("g")
rt.load(gen_doc)
g = rt.grid("land", cell=5.0)
check(g is not None and g.cell() == 5.0, "AriApp.grid: the generated world's grid")
if g is not None:
    occ = g.occupancy()
    check(occ.shape == (g.rows(), g.cols()) and occ[0].all(), "walled, (rows, cols)")
    check(rt.grid("land", 5.0, half=50.0).rows() == 21, "half crops round the terrain's centre")
    pts = g.free_points(3, 2, 4.0, 10.0)
    check(not any(g.occupied(x, y) for x, y in pts), "free points on open ground")
check(rt.grid("nosuch", 5.0) is None, "no such node: None")
del rt

# ── host_function's own checks ──────────────────────────────────────────────────────────────────
for bad, kw in ((42, {}), (len, {"with_call": True}), (len, {"done_channel": "x"})):
    try:
        pycvc.host_function("f", bad, **kw)
    except Exception:  # noqa: BLE001 - which C++ exception it maps to is not the point
        checks += 1
    else:
        raise AssertionError(f"host_function accepted {bad!r} {kw}")
f = pycvc.host_function("g", len, is_async=True, done_channel="c")
check(f.name == "g" and f.is_async() and f.done_channel == "c", "a HostFunction says what it is")

print(f"pycvc AriApp / host_function: {checks} checks passed")
