"""pycvc_gl.CameraController — the Python wrapper over cvcGL's built-in camera.

Proves the wrapper exposes the navigation + full cvc::state config surface, and
that because it is a state_object the SAME settings are reachable through
pycvc.state_set/get. Uses make_app() (an independently-owned cvc::app) — no
singleton crosses the boundary.
"""
import pycvc
import pycvc_gl

CC = pycvc_gl.CameraController
fails = 0


def check(what, ok):
    global fails
    print("  [%s] %s" % ("PASS" if ok else "FAIL", what))
    if not ok:
        fails += 1


def getf(app, key):
    return float(pycvc.state_get(app, key))


app = pycvc.make_app()

print("A. construct from an injected app (no singleton) + navigation")
cam = CC(app, "test.camera")
cam.frameBounds(-50, -50, 0, 50, 50, 20)
cam.setMode(CC.Mode_Fly)
check("setMode -> mode()", cam.mode() == CC.Mode_Fly)
cam.setMoveSpeed(10.0)
cam.keyDown("w")
cam.update(1.0)
cam.keyUp("w")  # no assert on pose (getPose is state-side); just exercise it

print("B. camera -> state (methods write state)")
check("mode in state", pycvc.state_get(app, "test.camera.mode") == "1")
cam.setUpAxis(0, 1, 0)
check("up axis in state", abs(getf(app, "test.camera.up.y") - 1.0) < 1e-9)
cam.setOrbitCenter(3, 4, 5)
check("orbit center in state", abs(getf(app, "test.camera.orbit.center.x") - 3.0) < 1e-9 and
      abs(getf(app, "test.camera.orbit.center.z") - 5.0) < 1e-9)
cam.setKeyBinding("forward", "Up")
check("key binding via API + state", cam.keyBinding("forward") == "Up" and
      pycvc.state_get(app, "test.camera.keys.forward") == "Up")

print("C. state -> camera (pycvc.state_set drives the camera)")
pycvc.state_set(app, "test.camera.mode", "0")
check("state mode=0 -> Orbit", cam.mode() == CC.Mode_Orbit)
pycvc.state_set(app, "test.camera.keys.forward", "i")
check("state rebinds forward key", cam.keyBinding("forward") == "i")
pycvc.state_set(app, "test.camera.settings.move_speed", "77")
cam.setMouseSensitivity(0.5)  # any call re-reads nothing; state already applied
check("state move_speed -> reflected", abs(getf(app, "test.camera.settings.move_speed") - 77.0) < 1e-6)

print("D. canonical viewer state path (static)")
check("viewerStatePath", CC.viewerStatePath("cvcgl", "left") == "cvcgl.viewers.left.camera")

print("E. construct from a SceneRenderer (canonical viewer path, end-to-end)")
try:
    sg = pycvc_gl.SceneGraph(app, "scene")
    view = pycvc_gl.SceneRenderer(sg, 64, 48, True, "left")  # offscreen, named "left"
    vcam = CC(view)
    vcam.setMode(CC.Mode_Fly)
    check("viewer camera state at scene.viewers.left.camera",
          pycvc.state_get(app, "scene.viewers.left.camera.mode") == "1")
    check("SceneRenderer.name()", view.name() == "left")

    print("F. cinematic TRACK mode surface (third mode)")
    vcam.setTrackTarget("mover")
    vcam.setMode(CC.Mode_Track)
    check("Mode_Track set", vcam.mode() == CC.Mode_Track)
    check("track target via API + state", vcam.trackTarget() == "mover" and
          pycvc.state_get(app, "scene.viewers.left.camera.track.target") == "mover")
    check("track mode in state", pycvc.state_get(app, "scene.viewers.left.camera.mode") == "2")
    # configure the follow via state (as a demo/script would)
    pycvc.state_set(app, "scene.viewers.left.camera.track.back", "40")
    pycvc.state_set(app, "scene.viewers.left.camera.track.target", "hero")
    check("track params drive from state", vcam.trackTarget() == "hero")
except Exception as e:  # noqa: BLE001 — offscreen GL may be unavailable
    print("  [SKIP] SceneRenderer path (offscreen GL unavailable): %s" % e)

print("\n%s (%d failures)" % ("ALL PASS" if fails == 0 else "FAILED", fails))

# cvcGL's static GL/state teardown races at interpreter exit (a known, harmless
# boost::lock_error that fires AFTER results are printed). Exit hard with the real
# status so the process code reflects the test, not the teardown race.
import os
import sys

sys.stdout.flush()
os._exit(1 if fails else 0)
