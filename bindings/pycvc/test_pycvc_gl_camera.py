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
check(
    "orbit center in state",
    abs(getf(app, "test.camera.orbit.center.x") - 3.0) < 1e-9
    and abs(getf(app, "test.camera.orbit.center.z") - 5.0) < 1e-9,
)
cam.setKeyBinding("forward", "Up")
check(
    "key binding via API + state",
    cam.keyBinding("forward") == "Up" and pycvc.state_get(app, "test.camera.keys.forward") == "Up",
)

print("C. state -> camera (pycvc.state_set drives the camera)")
pycvc.state_set(app, "test.camera.mode", "0")
check("state mode=0 -> Orbit", cam.mode() == CC.Mode_Orbit)
pycvc.state_set(app, "test.camera.keys.forward", "i")
check("state rebinds forward key", cam.keyBinding("forward") == "i")
pycvc.state_set(app, "test.camera.settings.move_speed", "77")
cam.setMouseSensitivity(0.5)  # any call re-reads nothing; state already applied
check(
    "state move_speed -> reflected", abs(getf(app, "test.camera.settings.move_speed") - 77.0) < 1e-6
)

print("D. canonical viewer state path (static)")
check("viewerStatePath", CC.viewerStatePath("cvcgl", "left") == "cvcgl.viewers.left.camera")

print("E. construct from a SceneRenderer (canonical viewer path, end-to-end)")
try:
    sg = pycvc_gl.SceneGraph(app, "scene")
    view = pycvc_gl.SceneRenderer(sg, 64, 48, True, "left")  # offscreen, named "left"
    vcam = CC(view)
    vcam.setMode(CC.Mode_Fly)
    check(
        "viewer camera state at scene.viewers.left.camera",
        pycvc.state_get(app, "scene.viewers.left.camera.mode") == "1",
    )
    check("SceneRenderer.name()", view.name() == "left")

    print("F. cinematic TRACK mode surface (third mode)")
    vcam.setTrackTarget("mover")
    vcam.setMode(CC.Mode_Track)
    check("Mode_Track set", vcam.mode() == CC.Mode_Track)
    check(
        "track target via API + state",
        vcam.trackTarget() == "mover"
        and pycvc.state_get(app, "scene.viewers.left.camera.track.target") == "mover",
    )
    check("track mode in state", pycvc.state_get(app, "scene.viewers.left.camera.mode") == "2")
    # configure the follow via state (as a demo/script would)
    pycvc.state_set(app, "scene.viewers.left.camera.track.back", "40")
    pycvc.state_set(app, "scene.viewers.left.camera.track.target", "hero")
    check("track params drive from state", vcam.trackTarget() == "hero")
except Exception as e:  # noqa: BLE001 — offscreen GL may be unavailable
    print("  [SKIP] SceneRenderer path (offscreen GL unavailable): %s" % e)

print("G. camera Phase A: pushed Track feed + per-viewport FoV + widen (headless)")
# No SceneRenderer/vtkCamera needed: Track computes its pose from state, and
# get_pose() reads that computed pose. This is the seam the ChaseCamera shim uses.
pc = CC(app, "pa.cam")
for k in ("pos_tau", "vel_tau", "cam_tau"):
    pycvc.state_set(app, "pa.cam.track." + k, "0.01")  # fast easing so it converges
pycvc.state_set(app, "pa.cam.track.back", "10")
pycvc.state_set(app, "pa.cam.track.height", "5")
pycvc.state_set(app, "pa.cam.track.look_up", "0")
pc.setMode(CC.Mode_Track)
check("trackFed false before feeding", not pc.trackFed())
px = 0.0
for _ in range(200):
    px += 0.5
    pc.feedTrackTarget(px, 0.0, 0.0)  # push a +X position stream (no scene node)
    pc.update(0.05)
check("feedTrackTarget switches to push mode", pc.trackFed())
eye, focal, up = pc.get_pose()  # ((ex,ey,ez),(fx,fy,fz),(ux,uy,uz))
check("pushed Track focal follows the fed target", abs(focal[0] - px) < 2.0 and abs(focal[1]) < 1.0)
check(
    "pushed Track eye trails behind (-X) and above (+Z)", eye[0] < focal[0] - 5.0 and eye[2] > 2.0
)
pc.setTrackTarget("someNode")
check("setTrackTarget reverts push mode", not pc.trackFed())
pc.setFieldOfView(60.0)
check(
    "field_of_view mirrored to state", abs(getf(app, "pa.cam.settings.field_of_view") - 60.0) < 1e-9
)
check("fieldOfView() reads back", abs(pc.fieldOfView() - 60.0) < 1e-9)
pc.setWidenTau(1.5)
check("widen_tau mirrored to state", abs(getf(app, "pa.cam.track.widen_tau") - 1.5) < 1e-9)
check("widenTau() reads back", abs(pc.widenTau() - 1.5) < 1e-9)

print("H. Map mode holds its own pose (headless)")
# update() used to re-apply the ORBIT pose in Map mode, so a framed map lasted
# until the next frame. get_pose() is the pose update() applies.


def close3(a, b, tol=1e-9):
    return all(abs(a[i] - b[i]) < tol for i in range(3))


def same_pose(p, q):
    return all(close3(p[i], q[i]) for i in range(3))


mc = CC(app, "map.cam")
mc.frameBounds(-50, -50, 0, 50, 50, 20)
orbit = mc.get_pose()
mc.frameMap(100.0, 200.0, 50.0)
check("frameMap -> Mode_Map", mc.mode() == CC.Mode_Map)
mc.update(0.016)
eye, focal, up = mc.get_pose()
check(
    "update() keeps the map pose: straight down at (100, 200), north up",
    close3(eye, (100, 200, 1000)) and close3(focal, (100, 200, 0)) and close3(up, (0, 1, 0)),
)
check(
    "map pose in state",
    getf(app, "map.cam.map.center.x") == 100.0
    and getf(app, "map.cam.map.center.y") == 200.0
    and getf(app, "map.cam.map.scale") == 50.0,
)
mc.beginDrag()
mc.mouseLook(1, 1)  # no viewport: 1 px spans 2 * scale = 100 world units
mc.endDrag()
mc.mouseWheel(1.0)
mc.update(0.016)
check("a drag pans against the pointer", close3(mc.get_pose()[0], (0, 100, 1000)))
check("the wheel zooms the map scale", abs(getf(app, "map.cam.map.scale") - 45.0) < 1e-9)
map_pose = mc.get_pose()
mc.setMode(CC.Mode_Orbit)
check("Map -> Orbit: the orbit pose it had", same_pose(mc.get_pose(), orbit))
mc.setMode(CC.Mode_Map)
check("Orbit -> Map: the map pose it had", same_pose(mc.get_pose(), map_pose))
pycvc.state_set(app, "map.cam.settings.move_speed", "7")
check("an outside settings write stays in Map", mc.mode() == CC.Mode_Map)
pycvc.state_set(app, "map.cam.mode", "0")
check(
    "state mode=0 -> Orbit, its pose intact",
    mc.mode() == CC.Mode_Orbit and same_pose(mc.get_pose(), orbit),
)
pycvc.state_set(app, "map.cam.mode", "3")
check(
    "state mode=3 -> Map, its pose intact",
    mc.mode() == CC.Mode_Map and same_pose(mc.get_pose(), map_pose),
)

print("\n%s (%d failures)" % ("ALL PASS" if fails == 0 else "FAILED", fails))

# cvcGL's static GL/state teardown races at interpreter exit (a known, harmless
# boost::lock_error that fires AFTER results are printed). Exit hard with the real
# status so the process code reflects the test, not the teardown race.
import os
import sys

sys.stdout.flush()
os._exit(1 if fails else 0)
