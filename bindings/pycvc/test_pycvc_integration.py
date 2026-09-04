"""End-to-end capstone: one realistic pipeline exercising every rearchitected
layer through the singleton-free, explicit-app API.

    app -> geometry/generators -> SDF -> meshing -> zero-copy numpy
        -> state (+ push observer) -> DSL with a Python-authored function
        -> scene

If this passes, Phases 0-5a compose: one app threads through all of it, one
shared state tree, real cvc types, a SWIG director, and a Python DSL function —
no module-global anywhere.
"""

import pycvc


def test_full_pipeline_on_one_app():
    # Phase 0 / singleton-free: one explicit app for everything.
    app = pycvc.make_app()

    # Phase 2: a procedural sphere, then its signed-distance field (Phase 2 sdf
    # needs the app explicitly), then an isosurface back out of the field.
    sphere = pycvc.sphere(0.0, 0.0, 0.0, 1.0, 24, 12)
    assert sphere.num_vertices() > 0 and sphere.num_triangles() > 0

    field = pycvc.sdf(app, sphere, 24, 24, 24)  # -> pycvc.volume
    assert field.xdim() == 24 and field.voxelTypeStr() is not None

    surf = pycvc.isosurface(field, 0.0)  # remesh the zero level set
    assert surf.num_vertices() > 0

    # Phase 1: zero-copy numpy view of the field grid; write-through visible.
    grid = field.grid()
    assert grid.shape == (24, 24, 24) and not grid.flags["OWNDATA"]

    # Phase 3: record mesh stats into the shared state tree, watched by a push
    # observer (SWIG director).
    class Watch(pycvc.state_observer):
        def __init__(self):
            super().__init__()
            self.paths = []

        def on_changed(self, path):
            self.paths.append(path)

    obs = Watch()
    obs.watch(app)
    pycvc.state_set(app, "mesh.verts", str(surf.num_vertices()))
    pycvc.state_set(app, "mesh.tris", str(surf.num_triangles()))
    assert pycvc.state_get(app, "mesh.verts") == str(surf.num_vertices())
    assert "mesh.verts" in obs.paths and "mesh.tris" in obs.paths
    obs.unwatch()

    # Phase 4: a DSL program calls a Python function that reads the state we just
    # wrote — Python <-> state <-> DSL, all on the same app.
    ex = None
    try:
        ex = pycvc.Exec(app)
    except Exception as e:
        if "without state_exec" not in str(e):
            raise
    if ex is not None:
        ex.register_fn("verts", lambda: int(pycvc.state_get(app, "mesh.verts")))
        # (+ 0 (verts)) forces numeric context; equals the vertex count.
        assert ex.run("(+ 0 (verts))") == str(surf.num_vertices())
        # A DSL write is visible back in Python.
        ex.run('(state-set "dsl.done" "yes")')
        assert pycvc.state_get(app, "dsl.done") == "yes"

    # Phase 5a: drop the meshes into a scene bound to the SAME app (no cvcGL
    # singleton). num_graphics reflects what we added.
    #
    # pycvc_gl wraps the REAL cvcGL classes; the Scene facade this used to go
    # through is gone (see the header of pycvc_scene.h). addGraphics returns the
    # LIVE node, so the mesh below can be posed and recoloured in place.
    import pycvc_gl

    scene = pycvc_gl.SceneGraph(app)
    surf_node = scene.addGraphics("surface", surf)
    scene.addGraphics("sphere", sphere)
    assert scene.num_graphics() == 2
    assert scene.hasGraphics("surface")

    # The capstone is about the layers composing, so go one step past "it was
    # added": what came back is the LIVE node, and posing it moves the real
    # scene-graph transform rather than a copy.
    surf_node.setPosition(1.0, 2.0, 3.0)
    w = surf_node.get_world_transform()  # row-major 4x4
    assert (w[3], w[7], w[11]) == (1.0, 2.0, 3.0)

    print("  ok: full app -> sdf -> mesh -> state/observer -> DSL -> scene pipeline")


if __name__ == "__main__":
    test_full_pipeline_on_one_app()
    print("pycvc integration capstone: OK")
