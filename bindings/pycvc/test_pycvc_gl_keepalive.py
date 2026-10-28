"""pycvc_gl: an object that borrows another by C++ reference keeps it alive through the cycle
collector.

SceneRenderer, ViewportManager, StageLighting, CameraController, ScreenTextHud, ImGuiOverlay and
AriRuntime hold a scene, a viewer, a camera or an overlay by reference, and their destructors use
it: they detach from the scene, remove their lights, text actor or window observers, or reset the
overlay's draw callback. Each proxy keeps what it borrows alive by stashing it. When the stash and
the borrower are garbage in one collection (at interpreter exit: module globals and anything in a
reference cycle), the collector may clear the borrowed proxy first, which releases its C++ object
under the borrower. So the stash is the borrowed proxy's untracked `this` (pycvc._keepalive), not
the proxy. When the borrowed object keeps objects of its own, it is a tuple of those and then
`this`: the borrowed proxy can go first, and its own stashes with it.

Each case checks the stash, then collects the borrowed objects and their borrowers in one
reference cycle, the borrowed ones created first so the collector meets their proxies first. A
freed scene's memory usually still reads back, so with the proxy as the stash the collection
crashes only now and then; the stash checks fail every time. Also: a scene added to a
ViewportManager lives as long as the manager, not as long as the Viewport proxy addSceneViewport
returned (on that proxy, dropping it freed the scene under the manager).

StageLighting is headless; the rest need an offscreen GL context and skip without one
(fail, with CVC_REQUIRE_RENDER=1).
Runs under pytest or as a plain script (ctest: pycvc_gl_keepalive).
"""

import gc
import os

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


def _no_gl(e):
    """Skip (printing why) without a GL context; under CVC_REQUIRE_RENDER, fail instead."""
    if os.environ.get("CVC_REQUIRE_RENDER", "0") not in ("", "0"):
        raise AssertionError("no offscreen GL under CVC_REQUIRE_RENDER") from e
    print("  skip: no offscreen GL (%s)" % e)


def _viewer(sg):
    """An offscreen SceneRenderer over sg, or None without a GL context (see _no_gl)."""
    try:
        return pycvc_gl.SceneRenderer(sg, 64, 48, True, "keepalive")
    except Exception as e:  # noqa: BLE001 -- no offscreen GL context here
        _no_gl(e)
        return None


def _uncleared(stash):
    """Nothing in stash is anything the collector can clear: tuples of untracked objects."""
    if isinstance(stash, tuple):
        return all(_uncleared(x) for x in stash)
    return not gc.is_tracked(stash)


def _kept_by(obj):
    return tuple(v for k, v in vars(obj).items() if k.startswith("_pycvc_"))


def _is_keepalive_of(stash, borrowed):
    """stash is pycvc._keepalive(borrowed): its owning SwigPyObject, after borrowed's own stashes
    when it has any, and nothing in it is anything the collector can clear."""
    kept = _kept_by(borrowed)
    want = kept + (borrowed.this,) if kept else borrowed.this
    if isinstance(want, tuple):
        same = isinstance(stash, tuple) and len(stash) == len(want)
        same = same and all(a is b for a, b in zip(stash, want))
    else:
        same = stash is want
    return same and _uncleared(stash)


def _holds(borrower, attr, borrowed):
    assert _is_keepalive_of(getattr(borrower, attr), borrowed), (type(borrower).__name__, attr)


def _flatten(stash):
    if isinstance(stash, tuple):
        return [y for x in stash for y in _flatten(x)]
    return [stash]


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
            _no_gl(e)
            return None
        vp = vm.addSceneViewport("inset", inset, REGION, 1)
        vm.render()
        _holds(vm, "_pycvc_scene", sg)
        assert isinstance(vm._pycvc_scenes, tuple) and len(vm._pycvc_scenes) == 1
        assert _is_keepalive_of(vm._pycvc_scenes[0], inset)
        assert vp._pycvc_keepalive is vm  # a Viewport is a reference into its manager
        return [sg, inset, vm, vp]

    if _collected(build):
        print("  ok: ViewportManager keeps its main scene and every added one")


def test_added_scene_lives_as_long_as_the_manager():
    sg = _scene("ka_main2")
    try:
        vm = pycvc_gl.ViewportManager(sg, 64, 48, True, "main")
    except Exception as e:  # noqa: BLE001 -- no offscreen GL context here
        _no_gl(e)
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
        _no_gl(e)
        return
    cam = vm.primary().camera()
    assert not cam.thisown and pycvc._keepalive(cam) is cam
    assert cam._pycvc_keepalive._pycvc_keepalive is vm
    assert pycvc._keepalive(app) is app.this  # keeps nothing of its own
    assert sg.thisown and _is_keepalive_of(pycvc._keepalive(sg), sg)
    assert any(x is app.this for x in pycvc._keepalive(sg))  # the scene keeps its app
    assert pycvc._keepalive(None) is None
    print("  ok: _keepalive stashes an owning proxy's `this`, a reference proxy whole")


def test_a_borrower_keeps_what_the_borrowed_object_keeps():
    """CameraController(view) keeps the SceneRenderer, which keeps its scene, which keeps its app.
    The renderer's `this` alone would outlive the renderer's proxy and with it that proxy's stash
    of the scene: dropping the proxies oldest first freed the scene, then its app, under the live
    renderer (a segfault at exit on Linux, in ~ViewportManager's scene->setRenderer)."""
    own_app = pycvc.make_app()
    sg = pycvc_gl.SceneGraph(own_app, "ka_chain")
    view = _viewer(sg)
    if view is None:
        return
    cam = pycvc_gl.CameraController(view)
    kept = _flatten(cam._pycvc_keepalive)
    for name, obj in (("renderer", view), ("scene", sg), ("app", own_app)):
        assert any(x is obj.this for x in kept), "the camera does not keep the " + name
    assert kept[-1] is view.this  # released first: the renderer goes before its scene
    del own_app, sg, view  # proxies oldest first: only the camera's stash keeps the chain
    gc.collect()
    cam.setMode(pycvc_gl.CameraController.Mode_Fly)
    del cam
    gc.collect()
    print("  ok: a borrower keeps what the borrowed object keeps")


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
