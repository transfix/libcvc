"""pycvc_gl: an object that borrows another by C++ reference keeps it alive through the cycle
collector.

SceneRenderer, ViewportManager, StageLighting, CameraController, ScreenTextHud, ImGuiOverlay and
AriRuntime hold a scene, a viewer, a camera or an overlay by reference, and their destructors use
it: they detach from the scene, remove their lights, text actor or window observers, or reset the
overlay's draw callback. Each proxy keeps what it borrows alive by stashing it. When the stash and
the borrower are garbage in one collection (at interpreter exit: module globals and anything in a
reference cycle), the collector may clear the borrowed proxy first, which releases its C++ object
under the borrower. So the stash is the borrowed proxy's untracked `this` (pycvc._keepalive), not
the proxy.

Each case checks the stash, then collects the borrowed objects and their borrowers in one
reference cycle, the borrowed ones created first so the collector meets their proxies first. A
freed scene's memory usually still reads back, so with the proxy as the stash the collection
crashes only now and then; the stash checks fail every time. Also: a scene added to a
ViewportManager lives as long as the manager, not as long as the Viewport proxy addSceneViewport
returned (on that proxy, dropping it freed the scene under the manager).

StageLighting is headless; the rest need an offscreen GL context and skip without one.
Runs under pytest or as a plain script (ctest: pycvc_gl_keepalive).
"""

import gc
import os
import sys

import pycvc
import pycvc_gl

app = pycvc.make_app()
REGION = (0.5, 0.5, 1.0, 1.0)


def _scene(name):
    g = pycvc.geometry(app)
    g.add_vertices([-50, -50, 0, 50, -50, 0, 0, 50, 0])
    g.add_triangle(0, 1, 2)
    sg = pycvc_gl.SceneGraph(app, name)
    sg.addGraphics("ground", g)
    return sg


def _viewer(sg):
    """An offscreen SceneRenderer over sg, or None (printing why) without a GL context."""
    try:
        return pycvc_gl.SceneRenderer(sg, 64, 48, True, "keepalive")
    except Exception as e:  # noqa: BLE001 -- no offscreen GL context here
        print("  skip: no offscreen GL (%s)" % e)
        return None


def _holds(borrower, attr, borrowed):
    """borrower.<attr> is borrowed's owning SwigPyObject, which the collector never clears."""
    stash = getattr(borrower, attr)
    assert stash is borrowed.this and not gc.is_tracked(stash), (type(borrower).__name__, attr)


def _collected(build):
    """build() makes what is borrowed, then its borrowers (checking their stashes), and returns
    them oldest first, or None to skip. Drop them inside one reference cycle and collect it, the
    way an interpreter exit meets them. Three rounds, so freed memory gets reused."""
    for _ in range(3):
        objs = build()
        if objs is None:
            return False
        cycle = {"objs": objs}
        cycle["self"] = cycle
        del objs, cycle
        gc.collect()
    return True


def test_stage_lighting():
    def build():
        sg = _scene("ka_stage")
        rig = pycvc_gl.StageLighting(sg)
        rig.apply()
        _holds(rig, "_pycvc_keepalive", sg)
        return [sg, rig]

    _collected(build)
    print("  ok: StageLighting keeps its scene")


def test_scene_renderer():
    def build():
        sg = _scene("ka_renderer")
        view = _viewer(sg)
        if view is None:
            return None
        view.render()
        _holds(view, "_pycvc_scene", sg)
        return [sg, view]

    if _collected(build):
        print("  ok: SceneRenderer keeps its scene")


def test_viewport_manager():
    def build():
        sg, inset = _scene("ka_main"), _scene("ka_inset")
        try:
            vm = pycvc_gl.ViewportManager(sg, 64, 48, True, "main")
        except Exception as e:  # noqa: BLE001 -- no offscreen GL context here
            print("  skip: no offscreen GL (%s)" % e)
            return None
        vp = vm.addSceneViewport("inset", inset, REGION, 1)
        vm.render()
        _holds(vm, "_pycvc_scene", sg)
        assert len(vm._pycvc_scenes) == 1
        assert vm._pycvc_scenes[0] is inset.this and not gc.is_tracked(vm._pycvc_scenes[0])
        assert vp._pycvc_keepalive is vm  # a Viewport is a reference into its manager
        return [sg, inset, vm, vp]

    if _collected(build):
        print("  ok: ViewportManager keeps its main scene and every added one")


def test_added_scene_lives_as_long_as_the_manager():
    sg = _scene("ka_main2")
    try:
        vm = pycvc_gl.ViewportManager(sg, 64, 48, True, "main")
    except Exception as e:  # noqa: BLE001 -- no offscreen GL context here
        print("  skip: no offscreen GL (%s)" % e)
        return
    # Neither the scene nor the Viewport proxy is kept: the manager alone holds the scene.
    vm.addSceneViewport("inset", _scene("ka_inset2"), REGION, 1)
    gc.collect()
    vm.render()
    assert vm.viewport("inset").scene().getGraphics("ground") is not None
    print("  ok: a scene added to a ViewportManager lives as long as the manager")


def test_a_reference_proxy_is_stashed_whole():
    """A proxy that does not own its C++ object -- Viewport.camera(), a reference into the
    Viewport -- is stashed as itself: its own stash of the Viewport (and the Viewport's of its
    manager) is what keeps the camera valid, and `.this` alone would drop that chain."""
    sg = _scene("ka_ref")
    try:
        vm = pycvc_gl.ViewportManager(sg, 64, 48, True, "main")
    except Exception as e:  # noqa: BLE001 -- no offscreen GL context here
        print("  skip: no offscreen GL (%s)" % e)
        return
    cam = vm.primary().camera()
    assert not cam.thisown and pycvc._keepalive(cam) is cam
    assert cam._pycvc_keepalive._pycvc_keepalive is vm
    assert sg.thisown and pycvc._keepalive(sg) is sg.this
    assert pycvc._keepalive(None) is None
    print("  ok: _keepalive stashes an owning proxy's `this`, a reference proxy whole")


def test_camera_controller_and_hud():
    def build():
        sg = _scene("ka_camera")
        view = _viewer(sg)
        if view is None:
            return None
        cam = pycvc_gl.CameraController(view)
        hud = pycvc_gl.ScreenTextHud(view, "caption")
        low = pycvc_gl.ScreenTextHud(app, "scene.viewers.keepalive.hud.low", view)
        headless = pycvc_gl.ScreenTextHud(app, "scene.viewers.keepalive.hud.headless", None)
        hud.setText("hello")
        low.setText("low")
        view.render()
        _holds(cam, "_pycvc_keepalive", view)
        _holds(hud, "_pycvc_keepalive", view)
        _holds(low, "_pycvc_keepalive", app)
        _holds(low, "_pycvc_viewer", view)  # the low-level ctor's viewer, kept too
        assert not hasattr(headless, "_pycvc_viewer")
        return [sg, view, cam, hud, low, headless]

    if _collected(build):
        print("  ok: CameraController / ScreenTextHud keep their viewer (or app)")


def test_imgui_overlay_and_ari_runtime():
    if not hasattr(pycvc_gl, "ImGuiOverlay"):
        print("  skip: pycvc_gl built without ImGui")
        return

    def build():
        sg = _scene("ka_imgui")
        view = _viewer(sg)
        if view is None:
            return None
        cam = pycvc_gl.CameraController(view)
        ui = pycvc_gl.ImGuiOverlay(view)
        ui.attachCamera(cam)
        _holds(ui, "_pycvc_keepalive", view)
        _holds(ui, "_pycvc_camera", cam)
        objs = [sg, view, cam, ui]
        if hasattr(pycvc_gl, "AriRuntime"):
            ari = pycvc_gl.AriRuntime(view, cam, ui)
            _holds(ari, "_pycvc_view", view)
            _holds(ari, "_pycvc_cam", cam)
            _holds(ari, "_pycvc_overlay", ui)
            objs.append(ari)
        return objs

    if _collected(build):
        print("  ok: ImGuiOverlay keeps its viewer and camera; AriRuntime its view, camera, overlay")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            print(name)
            fn()
    print("pycvc_gl keep-alive tests: OK")
    # Like the other pycvc_gl scripts (see test_pycvc_gl_imgui.py): exit past cvcGL's static
    # teardown at interpreter exit, with the real status.
    sys.stdout.flush()
    os._exit(0)
