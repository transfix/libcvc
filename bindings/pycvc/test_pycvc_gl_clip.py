"""pycvc_gl GraphicsNode.setClipChildren -- a parent clips its children to its
own box, from Python.

Headless part: the wrapped getter/setter. With an offscreen GL context it also
renders a box-outline parent with three flat-coloured children, measured by
lit area per hue: one fully inside (red: all of it drawn), one straddling the
+x face (green: about half), one above the +z face (blue: none) -- unclipped
first, so the checks can fail, then clipped, off again, and on again -- and a
fourth child added (sg.add_child_geometry) while clipping is on is cut at the
-x face like the others. The C++ side, with exact pixel samples on both mapper
families and every way a parent's box can change, is cvcgl_clip_children.
The render part skips (exit 0) without offscreen GL unless
CVC_REQUIRE_RENDER=1. Synthetic data only.
"""

import os
import sys

import pycvc

app = pycvc.make_app()
import pycvc_gl

FAILS = []


def check(name, ok, detail=""):
    print("  [%s] %s%s" % ("PASS" if ok else "FAIL", name, (" -- " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


C = 500.0  # box centre in x and y: well away from the origin
BOX = (C - 50, C - 50, -10.0, C + 50, C + 50, 10.0)


def box_outline():
    """The parent: the box's 12 edges as lines (its extents ARE the clip box)."""
    x0, y0, z0, x1, y1, z1 = BOX
    g = pycvc.geometry(app)
    g.add_vertices([x0, y0, z0, x1, y0, z0, x1, y1, z0, x0, y1, z0,
                    x0, y0, z1, x1, y0, z1, x1, y1, z1, x0, y1, z1])
    g.add_lines([0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7])
    return g


def quad(x0, y0, x1, y1, z):
    g = pycvc.geometry(app)
    g.add_vertices([x0, y0, z, x1, y0, z, x1, y1, z, x0, y1, z])
    g.add_triangles([0, 1, 2, 0, 2, 3])
    return g


def scene(name):
    sg = pycvc_gl.SceneGraph(app, name)
    sg.setDiagnosticChromeVisible(False)
    box = sg.addGraphics("box", box_outline())
    sg.geometry_node("box").setColor(1.0, 1.0, 1.0)
    kids = [("inside", quad(C - 40, C + 10, C - 10, C + 40, 0.0), (1.0, 0.1, 0.1)),
            ("straddle", quad(C + 20, C - 40, C + 80, C - 10, 0.0), (0.1, 1.0, 0.1)),
            ("above", quad(C - 40, C - 40, C - 10, C - 10, 30.0), (0.1, 0.1, 1.0))]
    nodes = {}
    for name_, geom, rgb in kids:
        n = sg.add_child_geometry("box", name_, geom)
        n.setColor(*rgb)
        n.setAmbient(1.0)
        n.setDiffuse(0.0)
        n.setSpecular(0.0)
        nodes[name_] = n
    sg._clip_kids = nodes  # the child nodes, for the per-node plane checks
    return sg, box


def test_wrapped_surface():
    print("wrapped surface")
    sg, box = scene("pyclip_surface")
    check("clipping is off by default", not box.getClipChildren())
    box.setClipChildren(True)
    check("setClipChildren(True)", box.getClipChildren())
    box.setClipChildren(False)
    check("setClipChildren(False)", not box.getClipChildren())
    # Per-node planes: px,py,pz,nx,ny,nz each, in the node's local frame.
    kid = sg._clip_kids["inside"]
    kid.set_clip_planes([C, 0, 0, 1, 0, 0])
    check("set_clip_planes / get_clip_planes round-trip",
          list(kid.get_clip_planes()) == [C, 0.0, 0.0, 1.0, 0.0, 0.0], str(list(kid.get_clip_planes())))
    check("... handed to its renderer", kid.applied_clip_plane_count() == 1 and kid.clipPlaneCount() == 1)
    box.set_clip_box(*BOX)
    check("a parent's set_clip_box reaches the child, nearest (its own) first",
          kid.clipPlaneCount() == 7 and kid.applied_clip_plane_count() == 6 and kid.maxClipPlanes() == 6)
    box.set_clip_planes([])
    kid.set_clip_planes([])
    check("[] clears them", kid.applied_clip_plane_count() == 0 and list(box.get_clip_planes()) == [])
    try:
        kid.set_clip_planes([1, 2, 3])
        check("set_clip_planes needs 6 numbers per plane", False)
    except Exception:  # noqa: BLE001 -- SWIG maps std::invalid_argument to ValueError
        check("set_clip_planes needs 6 numbers per plane", True)


def hue_areas(frame):
    """Pixels of each clearly dominant channel (r, g, b)."""
    n = [0, 0, 0]
    for i in range(0, len(frame), 3):
        p = frame[i:i + 3]
        for ch in range(3):
            o1, o2 = p[(ch + 1) % 3], p[(ch + 2) % 3]
            if p[ch] > 80 and p[ch] > o1 + 30 and p[ch] > o2 + 30:
                n[ch] += 1
    return n


def test_render():
    print("offscreen render: inside / straddling / outside")
    sg, box = scene("pyclip_render")
    try:
        view = pycvc_gl.SceneRenderer(sg, 160, 160, True)
        view.setBackground(0.0, 0.0, 0.0)
        # Far and narrow: the quads (all parallel to the image plane) project
        # with one uniform scale, so area ratios are the world-space ones.
        view.setCamera(C, C, 2000, C, C, 0, 0, 1, 0, 8.0, 100.0, 5000.0)
        view.render()
    except Exception as e:  # noqa: BLE001 -- no offscreen GL here
        if os.environ.get("CVC_REQUIRE_RENDER", "0") not in ("", "0"):
            check("offscreen GL available (CVC_REQUIRE_RENDER)", False, str(e))
        else:
            print("  [SKIP] offscreen GL unavailable: %s" % e)
        return
    full = hue_areas(view.frameRGB())
    if min(full) == 0:
        if os.environ.get("CVC_REQUIRE_RENDER", "0") not in ("", "0"):
            check("this build rasterises (CVC_REQUIRE_RENDER)", False, str(full))
        else:
            print("  [SKIP] this build did not rasterise")
        return
    check("unclipped: all three children drawn", min(full) > 100, str(full))

    def ratios(state):
        got = hue_areas(view.frameRGB())
        r = [g / float(f) for g, f in zip(got, full)]
        print("  %s: areas %s of %s" % (state, got, full))
        return r

    box.setClipChildren(True)
    r = ratios("clipped")
    check("clipped: the inside child is drawn whole", r[0] > 0.95, "%.3f" % r[0])
    check("clipped: the straddling child is cut at the face (about half)",
          0.4 < r[1] < 0.6, "%.3f" % r[1])
    check("clipped: the child above the box is gone", r[2] == 0.0, "%.3f" % r[2])

    box.setClipChildren(False)
    r = ratios("clipping off again")
    check("clipping off again: all three whole", min(r) > 0.95, str(["%.3f" % x for x in r]))

    box.setClipChildren(True)
    r = ratios("clipping on again")
    check("clipping on again: clipped as before",
          r[0] > 0.95 and 0.4 < r[1] < 0.6 and r[2] == 0.0, str(["%.3f" % x for x in r]))

    # Added while clipping is on: a red band across the -x face (half inside).
    late = sg.add_child_geometry("box", "late", quad(C - 80, C - 5, C - 20, C + 5, 0.0))
    late.setColor(1.0, 0.1, 0.1)
    late.setAmbient(1.0)
    late.setDiffuse(0.0)
    late.setSpecular(0.0)
    red_on = hue_areas(view.frameRGB())[0] - full[0]  # less the inside child (drawn whole)
    box.setClipChildren(False)
    red_off = hue_areas(view.frameRGB())[0] - full[0]
    ratio = red_on / float(red_off) if red_off > 0 else -1.0
    print("  added while clipping: late child %d of %d px" % (red_on, red_off))
    check("a child added while clipping is on is cut at the face (about half)",
          red_off > 50 and 0.4 < ratio < 0.6, "%.3f" % ratio)

    # A node's OWN plane: the straddling child keeps x <= C + 50 (normal -x),
    # with the parent no longer clipping -- about half again.
    straddle = sg._clip_kids["straddle"]
    straddle.set_clip_planes([C + 50, 0, 0, -1, 0, 0])
    g_own = hue_areas(view.frameRGB())[1]
    straddle.set_clip_planes([])
    g_none = hue_areas(view.frameRGB())[1]
    print("  own plane: straddling child %d of %d px" % (g_own, g_none))
    check("a node's own clip plane cuts it (about half), and [] restores it",
          g_none > 100 and 0.4 < g_own / float(g_none) < 0.6, "%d / %d" % (g_own, g_none))
    view.close()


test_wrapped_surface()
test_render()

if FAILS:
    print("FAIL: test_pycvc_gl_clip (%d failed)" % len(FAILS))
    sys.exit(1)
print("PASS: test_pycvc_gl_clip")
