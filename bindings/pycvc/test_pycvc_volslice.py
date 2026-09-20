"""pycvc cvc::volslice value-type tests — the headless slice-geometry compute
path + the render_settings surface, driven from Python.

compute_slices(local_to_clip, box, params) is deterministic and GL-free, so it
runs fully here: this pins the fan-geometry read-out accessors (added so a Python
offline/twin/regression path can consume the vertices it computes, not just count
them) against the same contract the C++ volslice_test.cpp checks, plus the
render_settings/box3d/slice_params value round-trips. The scene node itself is
covered in test_pycvc_gl_world.py.
"""

import math

import pycvc


def test_value_types_construct_and_round_trip():
    box = pycvc.box3d()  # default cube [-0.5, 0.5]^3
    assert math.isclose(box.min.x, -0.5) and math.isclose(box.max.z, 0.5)

    p = pycvc.slice_params()
    assert math.isclose(p.quality, 0.5) and p.max_planes == 1000
    p.quality = 0.25
    p.max_planes = 200
    assert math.isclose(p.quality, 0.25) and p.max_planes == 200

    # render_settings is renamed (volren already defines a render_settings).
    rs = pycvc.volslice_render_settings()
    rs.filter = pycvc.interpolation_nearest  # flat enum-class int constant
    rs.opacity_correction = True
    rs.window_min, rs.window_max = -6.0, 6.0
    rs.slices.max_planes = 300
    assert rs.filter == pycvc.interpolation_nearest
    assert rs.opacity_correction is True
    assert rs.slices.max_planes == 300
    assert rs.tf.point_count() == 0  # the shared volren transfer_function
    print("  ok: box3d/slice_params/volslice_render_settings construct + round-trip")


def test_compute_slices_and_fan_accessors():
    # Identity local->clip: view normal is +z (non-degenerate), so the unit cube
    # is sliced along z — the exact case volslice_test.cpp uses.
    geo = pycvc.compute_slices(pycvc.mat4(), pycvc.box3d(), pycvc.slice_params())
    nplanes = geo.planes()
    nverts = geo.vertices()
    assert nplanes > 10, "default quality should yield many slice planes"

    pos = list(geo.get_positions())
    tex = list(geo.get_texcoords())
    assert len(pos) == 3 * nverts and len(tex) == 3 * nverts
    # texcoords are positions shifted by +0.5 for the [-0.5,0.5] cube -> [0,1]
    # (the same invariant volslice_test.cpp asserts); proves the float->double copy.
    for i in range(len(pos)):
        assert math.isclose(tex[i], pos[i] + 0.5, abs_tol=1e-5)

    offs = list(geo.fan_offsets())
    cnts = list(geo.fan_counts())
    assert len(offs) == nplanes and len(cnts) == nplanes
    assert sum(cnts) == nverts, "fan counts must sum to the vertex total"
    assert all(3 <= c <= 6 for c in cnts), "clipped-cube fans have 3..6 vertices"
    print("  ok: compute_slices fan accessors — positions/texcoords/offsets/counts consistent")


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("pycvc volslice value-type tests: OK")
