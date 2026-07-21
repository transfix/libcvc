"""pycvc_gl smoke test — Python builds a live 3D scene via cvcGL.

Proves the generic binding chain end-to-end: geometry/volume built in the
`pycvc` module cross into the `pycvc_gl` module's Scene (SWIG %import) and
drive the cvcGL scene graph, with no Qt and no domain knowledge.
"""

import pycvc
import pycvc_gl


def test_build_scene():
    scene = pycvc_gl.Scene()

    g = pycvc.Geometry()
    g.add_vertices([0, 0, 0, 10, 0, 0, 0, 10, 0])
    g.add_triangle(0, 1, 2)
    scene.add_geometry("tri", g)

    v = pycvc.Volume()
    v.set_float_grid([float(i) for i in range(27)], 3, 3, 3, 0, 0, 0, 1, 1, 1)
    scene.add_volume("field", v)

    scene.pump()  # re-pumpable, no Qt loop
    assert scene.has("tri")
    assert scene.has("field")
    assert scene.num_graphics() == 2


if __name__ == "__main__":
    test_build_scene()
    print("pycvc_gl smoke test: OK")
