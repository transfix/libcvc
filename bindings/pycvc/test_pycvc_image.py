"""Phase-5 pycvc.image + geometry UV/tangent binding tests (core module, no VTK).

Covers the cvc::image value type wrapped in the `pycvc` module: construction with
the flat pixel_format/data_type enum aliases, the queries, the manipulation
helpers, save/load round-trip (skipped when no image handler is built), and —
the point of the phase — the ZERO-COPY image.numpy() view (an in-place numpy
edit writes the same buffer the image owns). Plus geometry.set_uvs()/uvs() and
set_tangents()/tangents(), mirroring the vertices()/vertex_colors() contract.
"""

import gc
import os
import tempfile

import numpy as np
import pycvc

app = pycvc.make_app()


# ── image: construction + enums ─────────────────────────────────────


def test_image_construct_and_enums():
    # The flat enum aliases: pycvc.image.RGBA / .u8 etc.
    im = pycvc.image(8, 4, pycvc.image.RGBA, pycvc.image.u8)
    assert im.width() == 8 and im.height() == 4
    assert im.channels() == 4
    assert not im.empty()
    # format defaults (RGBA, u8) when omitted
    im2 = pycvc.image(2, 2)
    assert im2.channels() == 4
    # channels track the format
    assert pycvc.image(1, 1, pycvc.image.GRAY, pycvc.image.u8).channels() == 1
    assert pycvc.image(1, 1, pycvc.image.RGB, pycvc.image.u8).channels() == 3
    # a default image is empty
    assert pycvc.image().empty()


# ── image: zero-copy numpy() view ───────────────────────────────────


def test_image_numpy_shape_dtype():
    im = pycvc.image(6, 3, pycvc.image.RGBA, pycvc.image.u8)
    a = im.numpy()
    assert a.shape == (3, 6, 4)  # (H, W, C)
    assert a.dtype == np.uint8
    assert a.flags.writeable is True
    assert a.base is not None  # the owning capsule pins the buffer


def test_image_numpy_is_zero_copy():
    im = pycvc.image(4, 4, pycvc.image.RGBA, pycvc.image.u8)
    a = im.numpy()
    b = im.numpy()
    # two views of the SAME underlying buffer (storage() is non-detaching)
    a[1, 2, 0] = 200
    a[1, 2, 3] = 255
    assert b[1, 2, 0] == 200 and b[1, 2, 3] == 255


def test_image_numpy_view_outlives_image():
    def make():
        im = pycvc.image(4, 4, pycvc.image.RGBA, pycvc.image.u8)
        v = im.numpy()
        v[0, 0, 0] = 42
        return v

    v = make()  # the image object is now unreferenced
    gc.collect()
    assert v.shape == (4, 4, 4)
    assert v[0, 0, 0] == 42  # buffer still valid


def test_image_numpy_f32():
    im = pycvc.image(2, 2, pycvc.image.GRAY, pycvc.image.f32)
    a = im.numpy()
    assert a.shape == (2, 2, 1)
    assert a.dtype == np.float32


# ── image: from_numpy (copy constructor) ────────────────────────────


def test_image_from_numpy_round_trip():
    src = np.arange(3 * 5 * 4, dtype=np.uint8).reshape(5, 3, 4)  # (H,W,C)
    im = pycvc.image.from_numpy(src)
    assert im.width() == 3 and im.height() == 5 and im.channels() == 4
    out = im.numpy()
    assert out.shape == (5, 3, 4)
    assert np.array_equal(out, src)
    # it copied: mutating the source must not touch the image
    src[0, 0, 0] = 99
    assert im.numpy()[0, 0, 0] == 0


def test_image_from_numpy_gray_2d():
    src = (np.arange(4 * 6, dtype=np.uint8) % 255).reshape(4, 6)  # (H,W), 1 channel
    im = pycvc.image.from_numpy(src)
    assert im.channels() == 1 and im.width() == 6 and im.height() == 4
    assert np.array_equal(im.numpy()[..., 0], src)


# ── image: manipulation helpers ─────────────────────────────────────


def test_image_manipulation():
    im = pycvc.image.from_numpy(
        np.arange(4 * 4 * 4, dtype=np.uint8).reshape(4, 4, 4)
    )
    up = im.resized(8, 2)
    assert up.width() == 8 and up.height() == 2 and up.channels() == 4
    rgb = im.converted(pycvc.image.RGB, pycvc.image.u8)
    assert rgb.channels() == 3
    flip = im.flipped_vertical()
    assert flip.width() == 4 and flip.height() == 4
    # flipping twice is the identity
    assert np.array_equal(flip.flipped_vertical().numpy(), im.numpy())


# ── image: save / load round-trip (skips if no handler is built) ────


def test_image_save_load_png_round_trip():
    im = pycvc.image.from_numpy(
        (np.arange(5 * 3 * 4, dtype=np.uint8) * 3 % 255).reshape(3, 5, 4)
    )
    path = os.path.join(tempfile.gettempdir(), "pycvc_image_test.png")
    try:
        im.save(path)
    except Exception as exc:  # noqa: BLE001 - no image handler compiled in
        if "no handler" in str(exc).lower():
            print("  skip: no image file handler built (ImageMagick off)")
            return
        raise
    back = pycvc.image.load(path)
    assert back.width() == 5 and back.height() == 3 and back.channels() == 4
    # PNG is lossless -> exact pixel match
    assert np.array_equal(back.numpy(), im.numpy())
    os.remove(path)


# ── geometry: set_uvs / uvs() ───────────────────────────────────────


def test_geometry_uvs_round_trip_and_zero_copy():
    g = pycvc.geometry(app)
    g.add_vertices([-1, -1, 0, 1, -1, 0, 1, 1, 0, -1, 1, 0])  # 4 verts
    g.set_uvs([0, 0, 1, 0, 1, 1, 0, 1])
    u = g.uvs()
    assert u.shape == (4, 2) and u.dtype == np.float64
    assert np.allclose(u[2], [1.0, 1.0])
    # zero-copy: a numpy edit writes the geometry's uv container in place
    u[0, 0] = 0.25
    assert g.uvs()[0, 0] == 0.25


def test_geometry_uvs_length_validation():
    g = pycvc.geometry(app)
    g.add_vertices([0, 0, 0, 1, 0, 0])  # 2 verts -> needs 4 uv values
    try:
        g.set_uvs([0, 0, 1])  # wrong length
    except Exception:
        pass
    else:
        raise AssertionError("set_uvs must reject a wrong-length list")


def test_geometry_tangents_round_trip():
    g = pycvc.geometry(app)
    g.add_vertices([0, 0, 0, 1, 0, 0, 0, 1, 0])  # 3 verts
    g.set_tangents([1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, -1])  # 3 * 4
    t = g.tangents()
    assert t.shape == (3, 4)
    assert t[2, 3] == -1.0  # handedness
    t[0, 3] = 5.0
    assert g.tangents()[0, 3] == 5.0


def test_empty_geometry_uv_view():
    g = pycvc.geometry(app)
    u = g.uvs()
    assert u.shape == (0, 2) and u.size == 0


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("pycvc image + geometry-uv tests: OK")
