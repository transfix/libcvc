"""pycvc_gl streaming overlays -- StreamingGeometryNode / RibbonNode /
DrapedLinkNode / HeightFieldTexture from Python, plus
SceneGraph.post_event_coalesced.

Headless parts check the wrapped surface: the SceneGraph factories, typed
downcasts, point data from numpy float32/float64 arrays and plain lists, the
capacity contract (generic writes past capacity raise; a ribbon grows), the
CPU twin of the draping shader, and coalesced events. With an offscreen GL
context it also renders and checks the upload accounting (an append = one
48-byte sub-upload; a visible-range move or a link move = none). The render
part skips (exit 0) without offscreen GL unless CVC_REQUIRE_RENDER=1.
Synthetic data only.
"""

import os
import sys
import threading

import pycvc

app = pycvc.make_app()
import pycvc_gl

try:
    import numpy as np
except ImportError:  # pragma: no cover -- numpy is a pycvc build dependency
    np = None

FAILS = []


def check(name, ok, detail=""):
    print("  [%s] %s%s" % ("PASS" if ok else "FAIL", name, (" -- " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


BOUNDS = [-10.0, -10.0, -1.0, 110.0, 10.0, 1.0]


def test_factories_and_types():
    print("factories + typed downcasts")
    sg = pycvc_gl.SceneGraph(app, "pystream")
    rib = sg.add_ribbon("track", 64, 0.5, BOUNDS)
    check("add_ribbon returns a RibbonNode", isinstance(rib, pycvc_gl.RibbonNode))
    check("getGraphics downcasts to the RibbonNode",
          isinstance(sg.getGraphics("track"), pycvc_gl.RibbonNode))
    check("ribbon_node / streaming_node accessors",
          sg.ribbon_node("track") is not None and sg.streaming_node("track") is not None
          and sg.draped_link_node("track") is None)
    grp = sg.add_group("overlays")
    child = sg.add_ribbon("spine", 32, 0.5, BOUNDS, "classic", "overlays")
    check("add_ribbon under a parent, forced classic mapper",
          child.mapper_kind_str() == "classic" and "spine" in grp.child_names())
    low = sg.add_ribbon("low", 32, 0.5, BOUNDS, "lowmem")
    check("forced low-memory mapper", low.mapper_kind_str() == "lowmem")
    gen = sg.add_streaming_geometry("quad", 4, [0, 1, 2, 0, 2, 3], [0, 0, -1, 1, 1, 1])
    check("add_streaming_geometry", isinstance(gen, pycvc_gl.StreamingGeometryNode)
          and gen.capacityPoints() == 4 and gen.triangleCount() == 2)
    check("reserved bounds round-trip",
          list(gen.get_reserved_bounds()) == [0.0, 0.0, -1.0, 1.0, 1.0, 1.0])
    gen.set_reserved_bounds(-1, -1, -1, 2, 2, 2)
    check("set_reserved_bounds", list(gen.get_reserved_bounds()) == [-1.0, -1.0, -1.0, 2.0, 2.0, 2.0])
    try:
        sg.add_streaming_geometry("bad", 2, [0, 1, 2], [0, 0, 0, 1, 1, 1])
        check("a triangle past the capacity raises", False)
    except RuntimeError:
        check("a triangle past the capacity raises", True)
    try:
        sg.add_ribbon("bad2", 4, 0.5, BOUNDS, "vulkan")
        check("an unknown mapper name raises", False)
    except RuntimeError:
        check("an unknown mapper name raises", True)
    if np is not None:
        for dt in (np.uint32, np.int64, np.uint16, np.int32):
            g = sg.add_streaming_geometry("np_" + dt.__name__, 4,
                                          np.array([0, 1, 2, 0, 2, 3], dtype=dt), [0, 0, -1, 1, 1, 1])
            check("numpy %s triangle indices" % dt.__name__, g.triangleCount() == 2)
        g = sg.add_streaming_geometry("np_scalars", 4, [np.int64(0), np.uint8(1), 2],
                                      [0, 0, -1, 1, 1, 1])
        check("a list of numpy integer scalars", g.triangleCount() == 1)
        for bad, why in ((np.array([0.0, 1.0, 2.0]), "float indices"),
                         (np.array([0, -1, 2], dtype=np.int32), "a negative index"),
                         ([0, 1, np.float64(2.0)], "a float in a list")):
            try:
                sg.add_streaming_geometry("np_bad", 4, bad, [0, 0, -1, 1, 1, 1])
                check(why + " raises", False)
            except RuntimeError:
                check(why + " raises", True)
    check("streaming overlays are not pickable by default",
          not rib.pickable() and not gen.pickable())
    rib.setPickable(True)
    check("setPickable opts in", rib.pickable())


def test_points_and_capacity():
    print("point data + capacity")
    sg = pycvc_gl.SceneGraph(app, "pypoints")
    gen = sg.add_streaming_geometry("quad", 4, [0, 1, 2, 0, 2, 3], [0, 0, -1, 1, 1, 1])
    gen.writePoints(0, [0, 0, 0, 1, 0, 0])  # a plain list
    if np is not None:
        gen.writePoints(2, np.array([1, 1, 0, 0, 1, 0], dtype=np.float32))
        gen.writePoints(2, np.array([1, 1, 0, 0, 1, 0], dtype=np.float64))
    st = gen.stream_stats()
    check("writes counted (list + numpy float32/64)",
          st["writes"] == (3 if np is not None else 1) and st["applies"] == st["writes"],
          str(st))
    try:
        gen.writePoints(3, [0, 0, 0, 1, 1, 1])
        check("a write past the capacity raises", False)
    except RuntimeError:
        check("a write past the capacity raises", True)
    check("... and stages nothing", gen.stream_stats()["writes"] == st["writes"])
    try:
        gen.writePoints(0, [0, 0])
        check("a partial xyz triple raises", False)
    except RuntimeError:
        check("a partial xyz triple raises", True)
    gen.setDrawRange(1)  # from triangle 1 to the end
    gen.setDrawRange(0, 1)
    gen.setUniform("cvcTint", 0.5)
    gen.setUniform("cvcTint3", 0.1, 0.2, 0.3)
    check("draw ranges + uniforms accepted", True)

    rib = sg.add_ribbon("track", 4, 0.5, BOUNDS)
    for k in range(10):
        rib.append(float(k), 0.0, 0.0)
    check("a ribbon GROWS past its capacity", rib.centerCount() == 10 and rib.centerCapacity() == 16,
          "capacity %d" % rib.centerCapacity())
    check("arc length", abs(rib.arcLength() - 9.0) < 1e-9
          and abs(rib.centerAtArcLength(2.5) - 2.5) < 1e-9)
    v = rib.center_vertices(5)
    check("centre vertices: half width either side",
          abs(v[1] - 0.5) < 1e-6 and abs(v[4] + 0.5) < 1e-6, str(v))
    route = [float(c) for k in range(5) for c in (k * 2.0, 1.0, 0.0)]
    rib.assign(route)
    check("assign a replan", rib.centerCount() == 5)
    if np is not None:
        rib.assign(np.zeros(3 * 7, dtype=np.float32))
        check("assign from numpy", rib.centerCount() == 7)
    rib.setVisibleCenters(1.5)
    rib.setVisibleCenters(1.5, 3.25)
    rib.clearCenters()
    check("visible window + clear", rib.centerCount() == 0)

    far = sg.add_ribbon("far", 8, 1.0, BOUNDS)
    for k in range(4):
        far.append(200.0 * k, 0.0, 0.0)
    b = far.get_reserved_bounds()
    check("a track that leaves its box grows the box (never culled)",
          b[3] >= 600.0 + 2.0 and b[0] <= BOUNDS[0], str(list(b)))


def test_height_field():
    print("HeightFieldTexture + DrapedLinkNode")
    hf = pycvc_gl.HeightFieldTexture(4, 3, 10.0, 20.0, 2.0, 5.0)
    hf.set_heights([float(i + 10 * j) for j in range(3) for i in range(4)])
    check("sample is bilinear", abs(hf.sample(11.0, 22.5) - 5.5) < 1e-9)
    check("extent", list(hf.get_extent()) == [10.0, 20.0, 0.0, 16.0, 30.0, 23.0])
    hf.update_rows(1, [100.0] * 4)
    check("update_rows", abs(hf.sample(12.0, 25.0) - 100.0) < 1e-9)
    try:
        hf.update_rows(0, [1.0] * 5)
        check("a ragged row raises", False)
    except RuntimeError:
        check("a ragged row raises", True)
    sg = pycvc_gl.SceneGraph(app, "pylink")
    link = sg.add_draped_link("link", hf, 8)
    check("add_draped_link", isinstance(link, pycvc_gl.DrapedLinkNode) and link.stations() == 8
          and isinstance(sg.getGraphics("link"), pycvc_gl.DrapedLinkNode))
    link.setStyle(1.0, 0.5, 0.2, 0.6, 1.0, 1.0)
    link.setEndpoints(10.0, 25.0, 16.0, 25.0)
    x, y, z = link.center_at(0.5)
    check("center_at drapes onto the field (+ lift)",
          abs(x - 13.0) < 1e-6 and abs(y - 25.0) < 1e-6 and abs(z - (hf.sample(x, y) + 0.5)) < 1e-4,
          "%g %g %g" % (x, y, z))
    check("the link shares the height field", link.heightField().nx() == 4)


def test_coalesced():
    print("SceneGraph.post_event_coalesced")
    sg = pycvc_gl.SceneGraph(app, "pycoalesce")
    seen = []
    key = object()

    def post_all():
        for v in range(5):
            sg.post_event_coalesced(key, lambda v=v: seen.append(v))

    t = threading.Thread(target=post_all)
    t.start()
    t.join()
    sg.processEvents()
    check("one call per key per drain, the latest", seen == [4], str(seen))
    sg.processEvents()
    check("nothing left over", seen == [4])

    # Distinct keys never coalesce, even short-lived ones whose memory Python
    # would otherwise reuse for the next key before the drain.
    seen.clear()
    for v in range(3):
        sg.post_event_coalesced(object(), lambda v=v: seen.append(v))
    sg.processEvents()
    check("three fresh keys = three callbacks", sorted(seen) == [0, 1, 2], str(seen))

    # A node key means the node: every lookup returns a new proxy, yet they are
    # one key -- and a node key does not evict the node's own streaming apply.
    seen.clear()
    rib = sg.add_ribbon("r", 8, 1.0, BOUNDS)
    applies0 = rib.stream_stats()["applies"]

    def post_nodes():
        rib.append(1.0, 2.0, 0.0)
        for v in range(3):
            sg.post_event_coalesced(sg.ribbon_node("r"), lambda v=v: seen.append(v))

    t = threading.Thread(target=post_nodes)
    t.start()
    t.join()
    sg.processEvents()
    check("proxies of one node are one key (the latest wins)", seen == [2], str(seen))
    check("... and the ribbon's own apply still ran",
          rib.stream_stats()["applies"] == applies0 + 1 and rib.centerCount() == 1,
          str(rib.stream_stats()))


def test_render():
    print("offscreen render: upload accounting")
    sg = pycvc_gl.SceneGraph(app, "pyrender")
    sg.setDiagnosticChromeVisible(False)
    rib = sg.add_ribbon("track", 256, 1.0, [-10, -10, -1, 110, 10, 1])
    rib.setColor(1.0, 0.1, 0.1)
    rib.setAmbient(1.0)
    rib.setDiffuse(0.0)
    hf = pycvc_gl.HeightFieldTexture(16, 16, -10.0, -10.0, 8.0, 8.0)
    link = sg.add_draped_link("link", hf, 12)
    link.setEndpoints(0.0, 0.0, 50.0, 0.0)
    try:
        view = pycvc_gl.SceneRenderer(sg, 96, 64, True)
        view.setCamera(50, 0, 300, 50, 0, 0, 0, 1, 0, 30.0, 1.0, 2000.0)
        view.render()
    except Exception as e:  # noqa: BLE001 -- no offscreen GL here
        if os.environ.get("CVC_REQUIRE_RENDER", "0") not in ("", "0"):
            check("offscreen GL available (CVC_REQUIRE_RENDER)", False, str(e))
        else:
            print("  [SKIP] offscreen GL unavailable: %s" % e)
        return
    frame = view.frameRGB()
    lit = sum(1 for i in range(0, len(frame), 3) if max(frame[i], frame[i + 1], frame[i + 2]) > 40)
    if lit == 0:
        if os.environ.get("CVC_REQUIRE_RENDER", "0") not in ("", "0"):
            check("this build rasterises (CVC_REQUIRE_RENDER)", False)
        else:
            print("  [SKIP] this build did not rasterise")
        return
    s0 = rib.stream_stats()
    for k in range(5):
        rib.append(10.0 * k, 0.0, 0.0)
        view.render()
    s1 = rib.stream_stats()
    check("5 appends = 5 sub-uploads, 4 x 48 B + 24 B",
          s1["uploads"] - s0["uploads"] == 5 and s1["upload_bytes"] - s0["upload_bytes"] == 216
          and s1["full_uploads"] == s0["full_uploads"], str(s1))
    for f in range(5):
        rib.setVisibleCenters(0.5 * f)
        link.setEndpoints(0.0, float(f), 50.0, float(f))
        view.render()
    s2 = rib.stream_stats()
    l2 = link.stream_stats()
    check("visible-range moves: no upload", s2["uploads"] == s1["uploads"])
    check("link moves: no upload (template written once)",
          l2["uploads"] == 0 and l2["full_uploads"] == 1, str(l2))
    hs = hf.upload_stats()
    check("height field uploaded once", hs["full_uploads"] == 1, str(hs))
    view.close()


test_factories_and_types()
test_points_and_capacity()
test_height_field()
test_coalesced()
test_render()

if FAILS:
    print("FAIL: test_pycvc_gl_streaming (%d failed)" % len(FAILS))
    sys.exit(1)
print("PASS: test_pycvc_gl_streaming")
