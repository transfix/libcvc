# pycvc-gl ↔ VTK Python wrapper integration

How the `pycvc_gl` SWIG module links against VTK's Python wrappers so that
Python-defined scene nodes can hand **real `vtkProp*` / `vtkActor` objects**
across the C++↔Python boundary, and how that build is packaged with cvcpkg.

This is the consumer-side companion to the `vtk-python` cvcpkg recipe
(`libcvc-deps/recipes/vtk-python`).

---

## 1. What the `vtk-python` package provides

The existing `vtk` cvcpkg package builds with `-DVTK_WRAP_PYTHON=OFF`: it ships
the C++ libraries (`libvtk*-9.5.so`), headers (`include/vtk-9.5/`), and the
monolithic VTK CMake config (`lib/cmake/vtk-9.5/`), but **no** Python artifacts.

The new **`vtk-python`** package rebuilds the *same pinned VTK 9.5 source* with
`-DVTK_WRAP_PYTHON=ON` and packages **only** the Python-specific outputs, which
install side by side with `vtk` in one prefix (zero file overlap):

| Artifact | Path (Linux) | Purpose |
|---|---|---|
| Importable package | `lib/python3.11/site-packages/vtkmodules/` (+ `vtk.py`) | `import vtkmodules` / `from vtkmodules.vtkRenderingCore import vtkActor` |
| Wrapping core lib | `lib/libvtkWrappingPythonCore-9.5.so` | defines `vtkPythonUtil::*` — the C++↔Python object bridge |
| Interpreter lib | `lib/libvtkPythonInterpreter-9.5.so` | embedded-interpreter helpers (optional for pycvc-gl) |
| Bridge headers | `include/vtk-9.5/vtkPythonUtil.h`, `PyVTK*.h`, `vtkWrappingPythonCoreModule.h`, `vtkSmartPyObject.h` | `#include "vtkPythonUtil.h"` |

Because the bridge headers land in `include/vtk-9.5/` — the **same** dir the `vtk`
package's `VTK::CommonCore` already puts on a consumer's include path — no extra
include directory is needed; `#include "vtkPythonUtil.h"` just works once
`vtk-python` is installed.

### ABI / registry coupling (the whole reason this is one shared build)

`vtkPythonUtil` keeps an internal map from C++ VTK class → Python wrapper type.
That registry lives in `libvtkWrappingPythonCore`. For
`GetPointerFromObject` / `GetObjectFromPointer` to recognize an object, the code
producing the object and the code consuming it **must link the same
`libvtkWrappingPythonCore` and import the same `vtkmodules`** — i.e. the exact
same `vtk-python` build. cvcpkg guarantees this: one `vtk-python` bundle is
installed into the prefix and both `pycvc_gl` and the host (VolRover) link that
single `.so` and import that single `vtkmodules`.

Consequences:
- **Python-ABI-specific**: `vtk-python` is `cp311` (matches `python311` /
  `numpy-cp311` / `pycvc`). A host on 3.12/3.13 needs a parallel `vtk-python`.
- **Version lockstep**: `vtk-python` must be the same VTK version + toolchain as
  `vtk`; rebuild/bump both together.

---

## 2. cvcpkg dependency wiring (`pycvc-gl` recipe)

Add `vtk-python` to `libcvc/cvcpkg/recipes/pycvc-gl/recipe.yaml` alongside the
existing `vtk` entry, in **both** `depends.build` and `depends.runtime`:

```yaml
depends:
  build:
    - name: libcvc
    - name: cvcgl
    - name: python311
    - name: numpy-cp311
    - name: vtk
    - name: vtk-python      # NEW — vtkPythonUtil.h + libvtkWrappingPythonCore
  runtime:
    - name: pycvc
    - name: libcvc
    - name: cvcgl
    - name: python311
    - name: numpy-cp311
    - name: vtk
    - name: vtk-python      # NEW — vtkmodules import + wrapping-core .so at load
```

The `vtk-python` package depends on `vtk` (`=9.5.0`) + `python311`, so those come
in transitively; listing `vtk` explicitly stays fine.

---

## 3. CMake changes (`bindings/pycvc/CMakeLists.txt`, `pycvc_gl` block)

Keep the existing `find_package(VTK COMPONENTS ...)` for the C++ side — **do not**
try to request `WrappingPythonCore` as a VTK component. VTK's CMake config is
monolithic and regenerated per build; the `vtk` package (WRAP_PYTHON=OFF) does
not enumerate the Python modules, so `find_package(VTK COMPONENTS
WrappingPythonCore)` would fail. Instead link the wrapping-core lib directly
(it is a plain shared library from the `vtk-python` package):

```cmake
find_package(VTK REQUIRED COMPONENTS RenderingCore RenderingOpenGL2 IOImage InteractionStyle)
target_link_libraries(pycvc_gl PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS pycvc_gl MODULES ${VTK_LIBRARIES})

# NEW: link VTK's Python wrapping core (defines vtkPythonUtil::*).
# vtkPythonUtil.h is already on the include path via VTK::CommonCore.
find_library(VTK_WRAPPING_PYTHONCORE
  NAMES vtkWrappingPythonCore-9.5 vtkWrappingPythonCore
  HINTS "${VTK_PREFIX_PATH}/lib" ${CMAKE_PREFIX_PATH}
  PATH_SUFFIXES lib)
if(NOT VTK_WRAPPING_PYTHONCORE)
  message(FATAL_ERROR
    "libvtkWrappingPythonCore not found — install the `vtk-python` cvcpkg package")
endif()
target_link_libraries(pycvc_gl PRIVATE ${VTK_WRAPPING_PYTHONCORE} Python3::Module)
```

`Python3::Module` is already linked; `RenderingCore` (already a requested
component) supplies `vtkProp` / `vtkActor`. No new include dirs are required.

---

## 4. SWIG `%typemap` for `vtkProp*` (both directions)

Put the include in the module's `%{ ... %}` block (in `pycvc_gl.i`, or a shared
`vtk_python.i` you `%include`):

```swig
%{
#include "vtkPythonUtil.h"     // from the vtk-python package
#include "vtkProp.h"
#include "vtkActor.h"
%}
```

### 4a. C++ → Python (`out`): return a live VTK wrapper object

`vtkPythonUtil::GetObjectFromPointer(vtkObjectBase*)` returns a **new reference**
to the Python wrapper for the object (creating or reusing it). A `vtkProp*`
upcasts to `vtkObjectBase*` implicitly. It returns a new ref to `Py_None` for a
null pointer, so it is null-safe. SWIG takes ownership of `$result`, so the new
reference is exactly right — no extra `Py_INCREF`.

```swig
%typemap(out) vtkProp* {
  // GetObjectFromPointer returns a NEW reference (Py_None if $1 is null).
  $result = vtkPythonUtil::GetObjectFromPointer($1);
  if (!$result) SWIG_fail;   // only on wrapper-creation failure
}
```

### 4b. Python → C++ (`in`): unwrap a Python VTK object to `vtkProp*`

`vtkPythonUtil::GetPointerFromObject(PyObject*, const char* classname)` checks
that the Python object wraps a VTK object convertible to `classname`, returns the
correctly-offset `void*` (cast to `vtkProp*`), and on mismatch returns `nullptr`
**and sets a Python `TypeError`**.

```swig
%typemap(in) vtkProp* {
  if ($input == Py_None) {
    $1 = nullptr;                                  // allow None -> nullptr
  } else {
    void* _p = vtkPythonUtil::GetPointerFromObject($input, "vtkProp");
    if (!_p) {                                     // sets a Python error itself
      SWIG_fail;
    }
    $1 = reinterpret_cast<vtkProp*>(_p);
  }
}

// Reject silently-wrong Python types passed positionally.
%typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) vtkProp* {
  $1 = ($input == Py_None) ||
       (vtkPythonUtil::GetPointerFromObject($input, "vtkProp") != nullptr);
  if (!$1) PyErr_Clear();   // typecheck must not leave an error set
}
```

**Lifetime note (important).** `GetPointerFromObject` returns a **borrowed**
pointer whose lifetime is the Python wrapper's. If the C++ side *retains* the
`vtkProp*` (e.g. stores it in the scene graph past the call), it must take a
reference — `prop->Register(nullptr)` and `UnRegister(nullptr)` on drop, or store
it in a `vtkSmartPointer<vtkProp>`. Otherwise a Python GC of the wrapper can
delete the object out from under the scene. For the covariant subtype
(`vtkActor*` etc.) use the same pattern with `"vtkActor"` as the classname, or
apply the `vtkProp*` typemaps to the subtypes:

```swig
%apply vtkProp* { vtkActor*, vtkVolume*, vtkImageActor* };
```

### 4c. Where these plug into the Scene facade

`pycvc_scene.h` currently exposes `add_geometry` / `add_volume` (string + pycvc
type). For the rearchitecture, add facade methods that take/return `vtkProp*`
(e.g. `void add_prop(const std::string&, vtkProp*)` and
`vtkProp* prop(const std::string&) const`) so a Python-defined node's
`vtkActor` flows straight into cvcGL's `SceneGraph`, and cvcGL-created props
surface back to Python as live `vtkmodules` objects. The typemaps above make
those signatures Just Work from SWIG.

---

## 5. Verifying the shared build

Once `vtk-python` + `pycvc-gl` are installed into one prefix:

```bash
python3.11 -c "import vtkmodules; from vtkmodules.vtkRenderingCore import vtkActor; print('ok', vtkActor)"
python3.11 -c "import pycvc, pycvc_gl; s = pycvc_gl.Scene(); print('scene ok')"
# round-trip: a vtkActor created in Python, handed to the scene, read back,
# must be the SAME object identity (proves one shared vtkPythonUtil registry).
```

`test_pycvc_gl.py` is the place to add the `vtkProp*` round-trip assertion.

---

## 6. Open issues

- **Build time**: `vtk-python` is a full VTK rebuild (long) — VTK wraps its
  modules in-tree; there is no way to wrap an already-installed VTK.
- **Python-ABI matrix**: cp311 only today. Each interpreter needs its own
  `vtk-python` (and matching `pycvc` / `numpy` column).
- **Platforms**: linux + windows (the pycvc-gl set). No macOS VTK bundle yet.
- **RenderingCore must be linked** by pycvc_gl for the `vtkProp` / `vtkActor`
  type symbols — it already is (existing `find_package(VTK COMPONENTS ...)`).
