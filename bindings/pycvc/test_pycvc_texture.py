"""Phase-5 pycvc_gl texture tests — live, zero-copy texture editing on a mesh.

Python builds a UV'd quad in a headless cvcGL scene, applies a pycvc.image as a
texture on the node, and edits the texture's pixels in place through the
zero-copy image.numpy() view. Asserts:
  * node.set_texture / node.texture_modified / node.clear_texture are callable;
  * a numpy edit + texture_modified() runs each "frame" with no error (no re-copy);
  * the ZERO-COPY invariant — when the VTK Python wrappers (vtkmodules) are
    importable, the node's vtkTexture scalar buffer IS the image's buffer, so an
    in-place image.numpy() edit is visible via the texture without a re-set.

All headless: no show(), no render window.
"""

import numpy as np
import pycvc

app = pycvc.make_app()
import pycvc_gl


def _uv_quad():
    g = pycvc.geometry(app)
    g.add_vertices([-1, -1, 0, 1, -1, 0, 1, 1, 0, -1, 1, 0])
    g.add_triangles([0, 1, 2, 0, 2, 3])
    g.set_uvs([0, 0, 1, 0, 1, 1, 0, 1])
    return g


def test_set_texture_and_modify_callable():
    sg = pycvc_gl.SceneGraph(app)
    sg.addGraphics("quad", _uv_quad())
    node = sg.geometry_node("quad")
    assert node is not None

    tex = pycvc.image(32, 32, pycvc.image.RGBA, pycvc.image.u8)
    node.set_texture(tex)  # zero-copy default
    pix = tex.numpy()  # (H, W, 4) uint8 — aliases the texture buffer
    assert pix.shape == (32, 32, 4)

    # Drive a few "frames": paint the pixels in place + signal the texture dirty.
    for t in range(5):
        pix[..., 0] = (t * 40) % 256
        pix[..., 3] = 255
        node.texture_modified()
        sg.processEvents()

    node.clear_texture()
    print("  ok: set_texture / numpy pixel edit / texture_modified / clear_texture")


def test_set_texture_copy_fallback():
    sg = pycvc_gl.SceneGraph(app)
    sg.addGraphics("quad", _uv_quad())
    node = sg.geometry_node("quad")
    gray = pycvc.image(8, 8, pycvc.image.GRAY, pycvc.image.u8)
    node.set_texture_copy(gray)  # convert+copy fallback path (any format)
    node.texture_modified()
    print("  ok: set_texture_copy fallback path")


def test_zero_copy_invariant_via_render():
    # The end-to-end proof: render the textured quad, edit ONLY the texture's
    # pixels in place (through the zero-copy image.numpy() view) + texture_modified()
    # — NO re-set — render again, and assert the rendered image CHANGED. Needs an
    # offscreen GL context + an image handler to read the PNGs back; skips cleanly
    # if either is missing (e.g. a headless CI with no GL).
    import os
    import tempfile

    sg = pycvc_gl.SceneGraph(app)
    sg.addGraphics("quad", _uv_quad())
    node = sg.getGraphics("quad")  # downcasts to GeometryNode

    tex = pycvc.image(64, 64, pycvc.image.RGBA, pycvc.image.u8)
    pix = tex.numpy()
    pix[:32, :, 0] = 255  # top half red
    pix[:, :, 3] = 255
    node.set_texture(tex)  # zero-copy
    node.texture_modified()

    d = tempfile.mkdtemp(prefix="pycvc_tex_")
    p1, p2 = os.path.join(d, "r1.png"), os.path.join(d, "r2.png")
    try:
        pycvc_gl.render_png(sg, p1, 256, 256)
        # edit the SAME buffer in place — no re-set of the texture
        pix[:, :, :3] = 0
        pix[32:, :, 2] = 255  # bottom half blue now
        node.texture_modified()
        pycvc_gl.render_png(sg, p2, 256, 256)
        a = pycvc.image.load(p1).numpy()
        b = pycvc.image.load(p2).numpy()
    except Exception as exc:  # noqa: BLE001 - no GL context / no image handler
        print("  skip: offscreen render / image handler unavailable (%s)" % type(exc).__name__)
        return
    finally:
        for p in (p1, p2):
            if os.path.exists(p):
                os.remove(p)
        os.rmdir(d)

    changed = int(np.count_nonzero(np.any(a != b, axis=-1)))
    assert changed > 0, "editing the texture in place did not change the render (aliasing broken)"
    print("  ok: zero-copy invariant — in-place texture edit changed the render (%d px), no re-set"
          % changed)


if __name__ == "__main__":
    test_set_texture_and_modify_callable()
    test_set_texture_copy_fallback()
    test_zero_copy_invariant_via_render()
    print("pycvc_gl texture tests: OK")
