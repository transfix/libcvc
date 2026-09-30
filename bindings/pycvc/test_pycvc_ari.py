"""pycvc_gl.AriRuntime — a full Ariadne (.ari) application from Python: open a
window, load a document, run its frame loop, and register a Python host verb.

Rendering needs a GL context. A headless box with no usable GL (llvmpipe/Xvfb)
raises a FATAL X error inside the render-window construction that aborts the
process — not a catchable Python exception — so the render body runs in a CHILD
process and the parent decides:
  * child rendered OK        -> pass (exit 0)
  * child hit a real assert  -> fail (exit 1)   (child exits 3)
  * child aborted / no GL    -> SKIP (exit 0)   (any other non-zero)
So it is a no-op locally (no GL) and a real render check in CI (Xvfb + llvmpipe).
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
_ASSERT_RC = 3  # child's exit code for a genuine assertion failure (a real bug, not a missing GL)


def _child():
    # Runs in the subprocess; a fatal X error here aborts THIS process, not the parent.
    import pycvc
    import pycvc_gl

    app = pycvc.App()
    sg = pycvc_gl.SceneGraph(app, "demo")
    view = pycvc_gl.SceneRenderer(sg, 320, 240, True, "main")  # offscreen; may abort with no GL
    print("WINDOW_OK", flush=True)  # GL context exists past here — a later failure is a real bug
    cam = pycvc_gl.CameraController(view)
    ui = pycvc_gl.ImGuiOverlay(view)
    ui.attachCamera(cam)
    ui.setVisible(True)  # load-bearing: the widget draw is skipped when the overlay is hidden

    ari = pycvc_gl.AriRuntime(view, cam, ui)
    try:
        pinged = {"n": 0}
        ari.on("ping", lambda: pinged.__setitem__("n", pinged["n"] + 1))
        warnings = ari.load(os.path.join(HERE, "demo.ari"))
        for w in warnings:
            print("  warn:", w)
        for _ in range(3):
            ari.step(1.0 / 60.0)  # frame(dt) + render() via the overlay draw callback
        out = os.path.join(HERE, "ari_demo_frame.png")
        view.writePNG(out)
        assert os.path.exists(out) and os.path.getsize(out) > 0, "no frame written"
        os.remove(out)
    except AssertionError as e:
        print("RENDER_FAIL:", e)
        sys.exit(_ASSERT_RC)
    print("RENDER_OK")


def main():
    if os.environ.get("_CVC_ARI_CHILD") == "1":
        _child()
        return
    env = dict(os.environ, _CVC_ARI_CHILD="1")
    p = subprocess.run([sys.executable, os.path.abspath(__file__)], env=env,
                       capture_output=True, text=True)
    sys.stdout.write(p.stdout)
    if "RENDER_OK" in p.stdout:
        print("pycvc AriRuntime test: OK")
    elif "WINDOW_OK" in p.stdout or p.returncode == _ASSERT_RC or "RENDER_FAIL" in p.stdout:
        # A GL context WAS created (or a real assertion tripped), so a subsequent failure is a genuine
        # bug — do NOT mask it as a skip.
        print("pycvc AriRuntime test: FAILED (rc=%d)\n%s" % (p.returncode, p.stderr))
        sys.exit(1)
    else:
        # No usable GL context (the render-window construction aborted) — skip, like the native
        # cvcGL render tests do on a headless box.
        print("SKIP test_pycvc_ari: no usable GL context (child rc=%d)" % p.returncode)


if __name__ == "__main__":
    main()
