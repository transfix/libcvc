// pycvc_host.cpp — minimal CPython-wasm launcher that embeds the STATIC pycvc /
// pycvc_gl SWIG extension archives into a wasm main module.
//
// On Emscripten there is no dlopen of shared Python extensions, so the SWIG
// wrappers are built as static archives (bindings/pycvc/CMakeLists.txt:
// TYPE STATIC on EMSCRIPTEN) and their PyInit__<mod> functions are registered as
// builtins here, before Py_Initialize, via PyImport_AppendInittab — the
// embed-time mechanism from docs/roadmap/static-single-binary-python.md §2.1/§5.
// The pure-Python proxies (pycvc.py, pycvc_gl/__init__.py, pymod_gl/*) live in
// MEMFS on sys.path; `import pycvc` / `import pycvc_gl` then resolve the C ext to
// the builtin and the .py proxy layer on top.
//
// Built by recipes/pycvc-gl-cp312/build-wasm.sh, which links this against the
// static libpython3.12.a + the static cvc/cvcGL/VTK closure + libpycvc.a +
// libpycvc_gl.a (force-loaded) and emits pycvc_host.{wasm,js}.

#include <Python.h>
#include <cstdio>
#include <cstdlib>

// SWIG emits these as extern "C" PyMODINIT_FUNC (visibility default), so they
// survive in the static archive and are referenced here (which force-loads the
// objects). %module pycvc -> _pycvc ; %module pycvc_gl -> _pycvc_gl.
extern "C" PyObject *PyInit__pycvc(void);
extern "C" PyObject *PyInit__pycvc_gl(void);

// numpy's wasm C-extensions ship as relocatable WASM objects (not side modules),
// so on Emscripten they link statically into this binary and register as builtins
// under their full dotted import names (CPython's BuiltinImporter checks the
// inittab before the filesystem .so). The set of extensions is enumerated by the
// recipe (build-wasm.{sh,ps1}) into a generated pycvc_register_numpy_inittab()
// that AppendInittab's each one — `import numpy` eagerly pulls the core plus
// linalg/fft/etc., so all must be registered.
#ifdef PYCVC_EMBED_NUMPY
extern "C" int pycvc_register_numpy_inittab(void); // generated; 0 = success
#endif

// VTK's static Python build (vtk-python-cp312 wasm) aggregates EVERY wrapped
// module into ONE builtin: _vtkmodules_static.a exports a single
// PyInit__vtkmodules_static (verified via llvm-nm — there is no bulk `_load()`
// void function). Register it like any other builtin below; vtkmodules/__init__.py
// (shipped in vtk-python's _vtk.zip, which the recipe/link puts on sys.path) does
// `import _vtkmodules_static`, which in turn registers every vtkmodules_vtkXXX
// submodule so `import vtkmodules.vtkCommonCore` etc. resolve. WITHOUT this the
// cvc-typed scene surface still works fully; marshaling live Python vtkmodules
// objects <-> C++ vtkProp*/vtkRenderer* (the BRIDGE) needs it. The recipe/link
// defines -DPYCVC_EMBED_VTKPYTHON when vtk-python is in the closure.
#ifdef PYCVC_EMBED_VTKPYTHON
extern "C" PyObject *PyInit__vtkmodules_static(void);
#endif

// Default smoke script when no entry script is passed: prove the modules import
// and the scene surface is live. The recipe's node smoke greps for PYCVC_WASM_OK.
static const char *kDefaultScript =
    "import sys\n"
    "print('pycvc-wasm host:', sys.version.split()[0])\n"
    "import numpy as np\n"
    "print('numpy', np.__version__, '- arange(5).sum() =', int(np.arange(5).sum()))\n"
    "import pycvc\n"
    "print('import pycvc OK ->', pycvc.__name__)\n"
    "import pycvc_gl\n"
    "print('import pycvc_gl OK ->', pycvc_gl.__name__)\n"
    // The bound C++ classes are live + type-safe; constructing a scene needs a
    // cvc::app + a GL context, out of scope for a headless node smoke.
    "print('pycvc_gl.SceneGraph  ->', pycvc_gl.SceneGraph.__name__)\n"
    "print('pycvc_gl.Node        ->', getattr(pycvc_gl, 'Node', '(n/a)'))\n"
    // The VTK-Python bridge: prove live vtkmodules objects coexist in-interpreter
    // with pycvc_gl (BRIDGE=ON links libvtkWrappingPythonCore for the marshaling).
    "try:\n"
    "    import vtkmodules.vtkCommonCore as _vcc\n"
    "    print('vtkmodules OK      -> vtkObject', _vcc.vtkObject().GetClassName())\n"
    "    import vtkmodules.vtkFiltersSources as _vfs\n"
    "    _s = _vfs.vtkSphereSource(); _s.Update()\n"
    "    print('live VTK pipeline  -> sphere points', _s.GetOutput().GetNumberOfPoints())\n"
    "    import vtkmodules.vtkIONetCDF as _vnc\n"
    "    print('IONetCDF (cvc.6)   ->', _vnc.vtkNetCDFReader().GetClassName())\n"
    "    print('PYCVC_VTK_BRIDGE_OK')\n"
    "except Exception as _e:\n"
    "    print('vtk bridge not embedded:', type(_e).__name__, _e)\n"
    "print('PYCVC_WASM_OK')\n";

int main(int argc, char **argv) {
  if (PyImport_AppendInittab("_pycvc", PyInit__pycvc) != 0) {
    fprintf(stderr, "pycvc_host: failed to register _pycvc inittab\n");
    return 1;
  }
  // pycvc_gl is a PACKAGE: its __init__.py does `from . import _pycvc_gl`, so the
  // builtin must register under the dotted submodule name, not top-level. (pycvc
  // core is a plain module — pycvc.py does `import _pycvc` — so it stays flat.)
  if (PyImport_AppendInittab("pycvc_gl._pycvc_gl", PyInit__pycvc_gl) != 0) {
    fprintf(stderr, "pycvc_host: failed to register pycvc_gl._pycvc_gl inittab\n");
    return 1;
  }
#ifdef PYCVC_EMBED_NUMPY
  if (pycvc_register_numpy_inittab() != 0) {
    fprintf(stderr, "pycvc_host: numpy inittab registration failed\n");
    return 1;
  }
#endif
#ifdef PYCVC_EMBED_VTKPYTHON
  // Register the aggregated vtkmodules builtin (before Py_Initialize) so
  // `import vtkmodules.*` resolves in-interpreter and the bridge can hand back
  // live vtk objects (BRIDGE=ON).
  if (PyImport_AppendInittab("_vtkmodules_static", PyInit__vtkmodules_static) != 0) {
    fprintf(stderr, "pycvc_host: failed to register _vtkmodules_static inittab\n");
    return 1;
  }
#endif

  Py_Initialize();

  int rc;
  if (argc > 1) {
    // Run an entry script supplied on argv (preloaded into MEMFS).
    FILE *f = fopen(argv[1], "r");
    if (!f) {
      fprintf(stderr, "pycvc_host: cannot open entry script %s\n", argv[1]);
      Py_FinalizeEx();
      return 2;
    }
    rc = PyRun_SimpleFile(f, argv[1]);
    fclose(f);
  } else {
    rc = PyRun_SimpleString(kDefaultScript);
  }

  if (Py_FinalizeEx() < 0)
    rc = rc ? rc : 120;
  return rc == 0 ? 0 : 3;
}
