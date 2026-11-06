"""The GIL in both directions for a Python Ariadne host: no deadlock, no starvation, no re-entry.

Each case runs in its own process with a timeout, so a deadlock fails the test instead of hanging
it. The GL cases (pycvc_gl.AriRuntime) skip without a GL context unless CVC_REQUIRE_RENDER=1; the
headless ones (pycvc.AriApp) always run.

  run_async      AriRuntime.run(), the C++ loop, while an async Python function runs on a pool
                 thread: run() releases the GIL, so the call finishes and the document quits.
                 (Before, run() held the GIL and the call waited for it forever.)
  run_old_verb   the same through register_async_verb, the no-argument form.
  reentry        a handler that calls frame(), render(), step() or run() gets RuntimeError, and
                 the runtime keeps running frames afterwards.
  thread         another Python thread writes state while frames run: it progresses during them.
  pool_writes    an async function writes state from its pool thread.
  close_inflight Host.close() with an async call in flight waits for it, then tears down.
  drop_inflight  letting go of everything (no close()) with an async call in flight: the runtime's
                 and the app's destructors release the GIL, so the call finishes. (Before, `del
                 app` deadlocked.)
"""

import os
import subprocess
import sys
import tempfile
import threading
import time

DOC = """meta:
  name: GIL test
  min_libcvc: 3.4.0
init: |
  (state-set "status" "ready")
windows:
  - window: W
    children:
      - text: status
        bind: status
"""
SKIP_RC = 77  # a GL case with no GL context


def doc_path():
    d = tempfile.mkdtemp(prefix="pycvc_gl_ariadne_gil_")
    p = os.path.join(d, "doc.ari")
    with open(p, "w", encoding="utf-8") as f:
        f.write(DOC)
    return p


def caps(app):
    import pycvc

    pycvc.state_set(app, "ariadne.limits.action_max_time", "30")
    pycvc.state_set(app, "ariadne.limits.action_max_steps", "0")


def gl_runtime():
    """An offscreen AriRuntime, or exit SKIP_RC when there is no GL context (in this process)."""
    import pycvc
    import pycvc_gl

    app = pycvc.make_app()
    caps(app)
    sg = pycvc_gl.SceneGraph(app, "gil")
    try:
        view = pycvc_gl.SceneRenderer(sg, 320, 240, True, "main")
    except Exception:  # noqa: BLE001 - no GL here
        sys.exit(SKIP_RC)
    cam = pycvc_gl.CameraController(view)
    ui = pycvc_gl.ImGuiOverlay(view)
    ui.attachCamera(cam)
    ui.setVisible(True)
    rt = pycvc_gl.AriRuntime(view, cam, ui)
    rt.register_quit()
    return app, (sg, view, cam, ui), rt


def slow(x=1, delay=0.3):
    time.sleep(delay)
    return x * 2


def case_run_async():
    import pycvc

    app, keep, rt = gl_runtime()
    rt.register_host_function(pycvc.host_function("slow", slow, is_async=True))
    rt.load(doc_path())
    rt.run_program('(begin (state-set "v" (slow 21)) (quit 0))')
    rt.run(240.0)  # returns once the document quits
    assert pycvc.state_get(app, "gil.v") == "42", "the async value arrived"


def case_run_old_verb():
    import pycvc

    app, keep, rt = gl_runtime()
    rt.register_async_verb("slow", lambda: str(slow(5, 0.2)))
    rt.load(doc_path())
    rt.run_program('(begin (state-set "v" (slow)) (quit 0))')
    rt.run(240.0)
    assert pycvc.state_get(app, "gil.v") == "10", "the verb's value arrived"


def case_reentry():
    import pycvc

    app, keep, rt = gl_runtime()
    errors = []

    def handler():
        for name, call in (("frame", lambda: rt.frame(0.0)), ("render", rt.render),
                           ("step", lambda: rt.step(0.0)), ("run", lambda: rt.run(60.0))):
            try:
                call()
            except RuntimeError as e:
                errors.append((name, "inside a frame" in str(e)))
            else:
                errors.append((name, False))

    rt.on("nest", handler)
    rt.register_host_function(pycvc.host_function("nest", handler))
    rt.load(doc_path())
    rt.run_program("(nest)")  # from a function the document calls
    for _ in range(3):
        rt.step(1.0 / 60.0)
    assert [e[0] for e in errors] == ["frame", "render", "step", "run"], errors
    assert all(ok for _, ok in errors), f"each one refused: {errors}"
    rt.step(1.0 / 60.0)  # and the runtime still runs frames


def headless_runtime():
    import pycvc

    app = pycvc.make_app()
    caps(app)
    rt = pycvc.AriApp(app, "gil")
    rt.register_quit()
    return app, rt


def case_thread():
    import pycvc

    app, rt = headless_runtime()
    rt.load(doc_path())
    writes = [0]
    stop = threading.Event()

    def writer():
        while not stop.is_set():
            pycvc.state_set(app, "gil.from_thread", str(writes[0]))
            writes[0] += 1

    t = threading.Thread(target=writer)
    t.start()
    t0 = time.time()
    frames = 0
    while time.time() - t0 < 0.5:
        rt.step(0.0)
        frames += 1
    stop.set()
    t.join()
    assert frames > 10 and writes[0] > 10, f"both ran: {frames} frames, {writes[0]} writes"
    assert pycvc.state_get(app, "gil.from_thread") == str(writes[0] - 1), "the last write landed"


def case_pool_writes():
    import pycvc

    app, rt = headless_runtime()

    def work(n):
        for i in range(int(n)):
            pycvc.state_set(app, f"gil.pool.{i}", str(i))  # state writes from the pool thread
        return threading.get_ident()

    rt.register_host_function(pycvc.host_function("work", work, is_async=True))
    rt.load(doc_path())
    rt.run_program('(state-set "tid" (work 50))', None)
    t0 = time.time()
    while not pycvc.state_has(app, "gil.tid") and time.time() - t0 < 20:
        rt.step(0.0)
        time.sleep(0.001)
    assert pycvc.state_get(app, "gil.pool.49") == "49", "the pool thread's writes landed"
    assert pycvc.state_get(app, "gil.tid") != str(threading.get_ident()), "on a pool thread"


def case_close_inflight():
    from pycvc_gl.ariadne import Host

    host = Host(mode="headless", name="gil")
    host.async_function("slow", slow)
    host.load(doc_path())
    host.run_program("(slow 1 0.5)")
    host.step()
    host.step()
    t0 = time.time()
    host.close()
    took = time.time() - t0
    assert 0.2 < took < 20, f"close() waited for the call in flight ({took:.2f} s)"


def case_drop_inflight():
    import gc

    import pycvc

    app, rt = headless_runtime()
    rt.register_host_function(pycvc.host_function("slow", slow, is_async=True))
    rt.load(doc_path())
    rt.run_program("(slow 1 0.5)", None)
    rt.step(0.0)
    rt.step(0.0)
    time.sleep(0.05)  # the call is in Python's time.sleep now
    del rt
    del app  # ~app waits for the call: without the GIL
    gc.collect()


def case_drop_inflight_gl():
    import gc

    import pycvc

    app, keep, rt = gl_runtime()
    rt.register_host_function(pycvc.host_function("slow", slow, is_async=True))
    rt.load(doc_path())
    rt.run_program("(slow 1 0.5)")
    rt.step(0.0)
    rt.step(0.0)
    time.sleep(0.05)
    del rt  # ~AriRuntime, without the GIL
    keep[1].close()
    del keep
    del app
    gc.collect()


CASES = {name[5:]: fn for name, fn in globals().items() if name.startswith("case_")}
GL_CASES = {"run_async", "run_old_verb", "reentry", "drop_inflight_gl"}


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "--case":
        CASES[sys.argv[2]]()
        print("CASE_OK", flush=True)
        return
    require = os.environ.get("CVC_REQUIRE_RENDER", "0") not in ("", "0")
    failed = []
    for name in CASES:
        t0 = time.time()
        try:
            p = subprocess.run([sys.executable, os.path.abspath(__file__), "--case", name],
                               capture_output=True, text=True, timeout=180)
        except subprocess.TimeoutExpired:
            print(f"  FAIL {name}: no answer in 180 s (a deadlock)")
            failed.append(name)
            continue
        took = time.time() - t0
        if p.returncode == 0 and "CASE_OK" in p.stdout:
            print(f"  ok   {name} ({took:.1f} s)")
        elif p.returncode == SKIP_RC and name in GL_CASES and not require:
            print(f"  SKIP {name}: no GL context")
        else:
            print(f"  FAIL {name} (rc={p.returncode})\n{p.stdout}\n{p.stderr}")
            failed.append(name)
    if failed:
        print("pycvc_gl.ariadne GIL test: FAILED:", ", ".join(failed))
        sys.exit(1)
    print("pycvc_gl.ariadne GIL test: OK")


if __name__ == "__main__":
    main()
