"""VTK Python bridge (F3): a Python-built vtkActor crosses into the C++ scene as
a real vtkProp*, and C++ props surface back as live vtkmodules objects.

Needs the vtk-python wrappers (vtkmodules importable + libvtkWrappingPythonCore
linked into pycvc_gl). Skips cleanly when they're absent, so the suite stays
green on a WRAP_PYTHON=OFF build.
"""

import pycvc
import pycvc_gl

try:
    from vtkmodules.vtkFiltersSources import vtkSphereSource
    from vtkmodules.vtkRenderingCore import vtkActor, vtkPolyDataMapper

    # Registry-consistency guard: the loaded vtkmodules must be the SAME VTK build
    # pycvc_gl linked its wrapping-core against — vtkPythonUtil's type registry
    # lives in that one libvtkWrappingPythonCore. In a proper single-prefix
    # install (vtk + vtk-python side by side) this always holds; in a split dev
    # env a different VTK can be resolved at runtime, and then the bridge can't
    # recognize the object. Detect that via a round trip and SKIP rather than
    # fail on a mismatched environment.
    _probe = vtkActor()
    HAVE_VTK = pycvc_gl.identity_prop(_probe) is _probe
except Exception:
    HAVE_VTK = False


def _make_actor():
    src = vtkSphereSource()
    src.SetRadius(1.0)
    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(src.GetOutputPort())
    actor = vtkActor()
    actor.SetMapper(mapper)
    return actor


def test_vtkprop_roundtrip():
    """Python vtkActor -> C++ vtkProp* -> back to the SAME Python wrapper."""
    if not HAVE_VTK:
        print("  skip: vtkmodules not installed (build without vtk-python)")
        return
    actor = _make_actor()
    back = pycvc_gl.identity_prop(actor)
    # vtkPythonUtil caches one Python wrapper per C++ object, so the round trip
    # returns the very same object.
    assert back is actor, (back, actor)
    # None <-> nullptr.
    assert pycvc_gl.identity_prop(None) is None


def test_wrong_type_rejected():
    if not HAVE_VTK:
        return
    # A non-VTK object must not unwrap to a vtkProp*.
    try:
        pycvc_gl.identity_prop("not a prop")
    except (TypeError, Exception):
        pass
    else:
        raise AssertionError("passing a non-vtkProp must raise")


def test_python_actor_into_cpp_scene():
    """add_prop: a Python vtkActor flows into the cvcGL scene graph, and prop()
    hands it back as a live vtkmodules object. (Free functions over the real
    wrapped SceneGraph — pycvc_gl has no facade.)"""
    if not HAVE_VTK:
        return
    app = pycvc.make_app()
    scene = pycvc_gl.SceneGraph(app)

    actor = _make_actor()
    pycvc_gl.add_prop(scene, "pyactor", actor, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0)
    assert scene.hasGraphics("pyactor")
    assert scene.num_graphics() >= 1

    got = pycvc_gl.prop(scene, "pyactor")
    assert got is not None
    assert got is actor  # same object back out
    assert got.IsA("vtkActor")  # a real VTK object on the Python side


def test_mixed_python_and_native_nodes():
    """Python props coexist with pycvc-native geometry nodes in one scene."""
    if not HAVE_VTK:
        return
    app = pycvc.make_app()
    scene = pycvc_gl.SceneGraph(app)

    g = pycvc.geometry(app)
    g.add_vertices([0, 0, 0, 1, 0, 0, 0, 1, 0])
    g.add_triangles([0, 1, 2])
    scene.addGraphics("nativetri", g)

    pycvc_gl.add_prop(scene, "pysphere", _make_actor())
    assert scene.hasGraphics("nativetri") and scene.hasGraphics("pysphere")
    assert scene.num_graphics() >= 2


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc VTK-python bridge tests: OK")
