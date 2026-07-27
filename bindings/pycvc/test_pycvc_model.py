"""Phase-6 pycvc.load_model + pycvc.model/material binding tests (core, no VTK).

Covers the native mesh/model loader wrapped from cvc::model_file_io:
pycvc.load_model(path) → pycvc.model, its meshes/materials sequences, per-mesh
geometry (vertices/triangles/UVs), merged() flatten, and the material surface
(base_color/emissive tuples, metallic/roughness, base_color_texture_path, and the
loaded base_color_texture() as a pycvc.image).

Mirrors the C++ model_test.cpp OBJ+MTL+PNG fixture: the PNG is authored through
pycvc.image.from_numpy + .save(), so a build with no image delegate (ImageMagick
off) skips the texture-specific assertions, and a build without the Assimp handler
skips the loader tests — exactly as the C++ test GTEST_SKIP()s.
"""

import os
import tempfile

import numpy as np
import pycvc

# Authored material / texture constants (match model_test.cpp's write_obj_fixture).
_KD = (0.8, 0.2, 0.1)
_TEX_RGBA = (200, 100, 50, 255)
_TEX_W = _TEX_H = 4


def _write_obj_fixture():
    """Write a tiny OBJ + MTL + 4x4 PNG into a temp dir; return (obj_path,
    texture_ok). texture_ok is False when no image handler could write the PNG."""
    d = tempfile.mkdtemp(prefix="pycvc_model_")
    obj = os.path.join(d, "cvc_model_test_mesh.obj")
    mtl = os.path.join(d, "cvc_model_test_mesh.mtl")
    png = os.path.join(d, "cvc_model_test_tex.png")

    # 4x4 RGBA texture, uniform color, via pycvc.image (needs an image handler).
    texture_ok = False
    try:
        px = np.empty((_TEX_H, _TEX_W, 4), dtype=np.uint8)
        px[:, :, 0] = _TEX_RGBA[0]
        px[:, :, 1] = _TEX_RGBA[1]
        px[:, :, 2] = _TEX_RGBA[2]
        px[:, :, 3] = _TEX_RGBA[3]
        pycvc.image.from_numpy(px).save(png)
        texture_ok = True
    except Exception:  # noqa: BLE001 - no image delegate compiled in
        texture_ok = False

    with open(mtl, "w") as om:
        om.write("newmtl cvc_mat\n")
        om.write("Kd 0.8 0.2 0.1\n")
        om.write("d 1.0\n")
        om.write("map_Kd cvc_model_test_tex.png\n")
    with open(obj, "w") as oo:
        oo.write("mtllib cvc_model_test_mesh.mtl\n")
        oo.write("v 0 0 0\n")
        oo.write("v 1 0 0\n")
        oo.write("v 0 1 0\n")
        oo.write("vt 0 0\n")
        oo.write("vt 1 0\n")
        oo.write("vt 0 1\n")
        oo.write("usemtl cvc_mat\n")
        oo.write("f 1/1 2/2 3/3\n")
    return obj, texture_ok


def _write_tetra_obj_fixture():
    """Write a NON-planar tetrahedron OBJ with a known, non-zero-volume bbox
    (x∈[0,2], y∈[0,3], z∈[0,4]) so extents() has concrete values to assert — a
    flat triangle's bbox collapses to the null/identity box (see the C++
    ExtentsUnion test, which uses non-planar geometry for the same reason)."""
    d = tempfile.mkdtemp(prefix="pycvc_model_tetra_")
    obj = os.path.join(d, "tetra.obj")
    with open(obj, "w") as oo:
        oo.write("v 0 0 0\n")
        oo.write("v 2 0 0\n")
        oo.write("v 0 3 0\n")
        oo.write("v 0 0 4\n")
        oo.write("f 1 2 3\n")
        oo.write("f 1 2 4\n")
        oo.write("f 1 3 4\n")
        oo.write("f 2 3 4\n")
    return obj


def _load_or_skip(path):
    """load_model(path) or None (printing a skip) when no handler is built."""
    try:
        return pycvc.load_model(path)
    except Exception as exc:  # noqa: BLE001
        msg = str(exc).lower()
        if "handler" in msg or "unsupported" in msg:
            print("  skip: no model file handler built (Assimp off)")
            return None
        raise


def _uv_present(uvs, u, v):
    return bool(np.any(np.all(np.isclose(uvs, [u, v], atol=1e-5), axis=1)))


# ── value-type: no handler / bad extension raises ───────────────────────


def test_bad_extension_raises():
    try:
        pycvc.load_model("/no/such/file.qwerty")
    except Exception:  # noqa: BLE001 - the expected cvc::exception -> RuntimeError
        pass
    else:
        raise AssertionError("load_model must raise on an unsupported extension")


# ── loader: OBJ with UVs + material + texture ───────────────────────────


def test_load_obj_mesh_and_merged():
    obj, _ = _write_obj_fixture()
    m = _load_or_skip(obj)
    if m is None:
        return
    assert not m.empty()
    assert m.num_meshes() >= 1

    mesh = m.meshes[0]
    g = mesh.geometry()
    assert g.num_vertices() == 3
    assert g.num_triangles() == 1

    # UVs present and matching the authored (unflipped) coordinates.
    uvs = g.uvs()
    assert uvs.shape == (3, 2)
    assert _uv_present(uvs, 0, 0)
    assert _uv_present(uvs, 1, 0)
    assert _uv_present(uvs, 0, 1)

    # merged() flattens to a single geometry with the same counts (one mesh here).
    merged = m.merged()
    assert merged.num_vertices() == 3
    assert merged.num_triangles() == 1


def test_load_obj_material():
    obj, texture_ok = _write_obj_fixture()
    m = _load_or_skip(obj)
    if m is None:
        return

    mesh = m.meshes[0]
    assert mesh.material >= 0
    assert mesh.material < len(m.materials)
    mat = m.materials[mesh.material]

    # Kd -> base_color rgb, d=1 -> alpha (a plain 4-tuple).
    bc = mat.base_color()
    assert len(bc) == 4
    assert np.allclose(bc[:3], _KD, atol=1e-3)
    assert abs(bc[3] - 1.0) < 1e-3
    # emissive default (0,0,0), and the scalar PBR fields marshal as floats.
    assert len(mat.emissive()) == 3
    assert isinstance(mat.metallic, float)
    assert isinstance(mat.roughness, float)

    assert mat.base_color_texture_path == "cvc_model_test_tex.png"

    if not texture_ok:
        print("  skip: texture assertions (no image delegate to author the PNG)")
        return
    assert mat.has_base_color_texture()
    tex = mat.base_color_texture()
    assert not tex.empty()
    assert tex.width() == _TEX_W and tex.height() == _TEX_H
    assert tex.channels() == 4
    # Decoded pixels round-trip (PNG is lossless; uniform color -> flip-invariant).
    px = tex.numpy()
    assert px.shape == (_TEX_H, _TEX_W, 4)
    assert tuple(int(v) for v in px[0, 0]) == _TEX_RGBA


def test_extents_values():
    # A non-planar tetra (bbox x∈[0,2], y∈[0,3], z∈[0,4]) so extents() has real
    # values — this proves the (minx,miny,minz,maxx,maxy,maxz) marshaling ORDER,
    # which an all-zero flat-triangle bbox could not distinguish from a broken
    # (swapped/transposed/default) binding.
    m = _load_or_skip(_write_tetra_obj_fixture())
    if m is None:
        return
    e = m.extents()
    assert len(e) == 6
    assert all(np.isfinite(v) for v in e)
    minx, miny, minz, maxx, maxy, maxz = e
    assert np.isclose(minx, 0.0) and np.isclose(miny, 0.0) and np.isclose(minz, 0.0)
    assert np.isclose(maxx, 2.0) and np.isclose(maxy, 3.0) and np.isclose(maxz, 4.0)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("pycvc model tests: OK")
