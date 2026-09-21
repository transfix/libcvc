"""pycvc_gl.ViewportManager / Viewport — the Python wrapper over cvcGL's
picture-in-picture surface (one window / one GL context, N cameras).

Proves volrover3 / grl-snam (both Python) can build multi-viewport + minimap
layouts and drive per-viewport cameras from Python, identically to C++: N
viewports over separate scenes AND an alternate-view mirror, the input router
(viewportAt hit-test, focus-follows-click into cvc::state, per-viewport camera),
region tuples, and frameRGB bytes. Uses make_app() (no singleton crosses the
boundary).

Offscreen construction needs a GL driver; where none is available the whole test
SKIPs cleanly rather than failing.
"""

import os
import sys

import pycvc
import pycvc_gl

VM = pycvc_gl.ViewportManager
fails = 0


def check(what, ok):
    global fails
    print("  [%s] %s" % ("PASS" if ok else "FAIL", what))
    if not ok:
        fails += 1


def _tri(app):
    g = pycvc.geometry(app)
    g.add_vertices([-50, -50, 0, 50, -50, 0, 0, 50, 0])
    g.add_triangle(0, 1, 2)
    g.set_colors([0.8, 0.8, 0.8] * 3)
    return g


app = pycvc.make_app()
main_scene = pycvc_gl.SceneGraph(app, "main_scene")
inset_scene = pycvc_gl.SceneGraph(app, "inset_scene")
main_scene.addGraphics("ground", _tri(app))
inset_scene.addGraphics("ground", _tri(app))

W, H = 320, 240
print("A. construct a ViewportManager (offscreen) with an auto primary")
try:
    vm = VM(main_scene, W, H, True, "main")
except Exception as e:  # no GL context available in this environment
    print("SKIP: ViewportManager needs a GL driver — %s" % e)
    sys.stdout.flush()
    os._exit(0)

check("frame size", vm.frameWidth() == W and vm.frameHeight() == H)
check("offscreen has no window to close", not vm.windowClosed())
check("primary auto-created", vm.hasViewport("main"))
check("primary().name()", vm.primary().name() == "main")
check("one viewport so far", len(list(vm.viewportNames())) == 1)

print("B. add an inset over a SECOND scene, and a mirror of the primary")
inset = vm.addSceneViewport("inset", inset_scene, (0.62, 0.05, 0.97, 0.42), 1)
mini = vm.addMirrorViewport("mini", "main", (0.62, 0.58, 0.97, 0.95), 2)
vm.render()
check("three viewports", sorted(vm.viewportNames()) == ["inset", "main", "mini"])
check("inset is not a mirror", not inset.isMirror())
check("mini is a mirror", mini.isMirror())
check("mini mirrors the main scene", mini.scene().getStatePrefix() == "main_scene")
check(
    "region() round-trips as a 4-tuple",
    tuple(round(v, 3) for v in inset.region()) == (0.62, 0.05, 0.97, 0.42),
)

print("C. viewportAt: layer / visibility / inputEnabled / gutter")
in_primary = (40, 120)  # primary-only
in_inset = (254, 56)  # inside the inset (layer 1, over primary)
in_mini = (254, 187)  # inside the mirror (layer 2, over primary)
check("primary-only point -> main", vm.viewportAt(*in_primary).name() == "main")
check("inset point -> inset (higher layer)", vm.viewportAt(*in_inset).name() == "inset")
check("mirror point -> mini", vm.viewportAt(*in_mini).name() == "mini")
check("gutter -> None", vm.viewportAt(4000, 4000) is None)
inset.setVisible(False)
check("hidden viewport skipped", vm.viewportAt(*in_inset).name() == "main")
inset.setVisible(True)
mini.setInputEnabled(False)
check("input-disabled viewport falls through", vm.viewportAt(*in_mini).name() == "main")
mini.setInputEnabled(True)

print("D. focus-follows-click writes active_viewport to cvc::state")
vm.routeMouseButton(VM.MouseButton_Left, True, in_inset[0], in_inset[1])
vm.routeMouseButton(VM.MouseButton_Left, False, in_inset[0], in_inset[1])
check("active viewport is the clicked one", vm.activeViewport().name() == "inset")
check("active_viewport in state", pycvc.state_get(app, "main_scene.active_viewport") == "inset")

print("E. per-viewport camera is fully reachable from Python")
cam = inset.camera()
cam.setMode(pycvc_gl.CameraController.Mode_Fly)
check("viewport camera setMode", cam.mode() == pycvc_gl.CameraController.Mode_Fly)
# the inset camera's state lives under the inset scene's viewer tree
check(
    "per-viewport camera state",
    pycvc.state_get(app, "inset_scene.viewers.inset.camera.mode") == "1",
)

print("F. input dispatch + a full-window frame come back cleanly")
vm.routeMouseWheel(in_primary[0], in_primary[1], 1.0)  # zoom the primary
vm.routeKey("w", True)  # to the active (inset) viewport
vm.updateCameras(0.05)
vm.routeKey("w", False)
frame = vm.frameRGB()
check("frameRGB returns bytes", isinstance(frame, bytes))
check("frameRGB is W*H*3", len(frame) == W * H * 3)

print("\n%s (%d failures)" % ("ALL PASS" if fails == 0 else "FAILED", fails))
# Match the other pycvc_gl tests: hard-exit past cvcGL's harmless static-teardown
# race so the process code reflects the test result.
sys.stdout.flush()
os._exit(1 if fails else 0)
