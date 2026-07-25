// pycvc_gl.i — SWIG module for the cvcGL scene graph (Scene facade).
// %import pulls in pycvc's Geometry/Volume types without re-wrapping them.
%module pycvc_gl

%{
#include <cvc/core/exception.h>  // the %import'd %exception block catches cvc::exception
#include <stdexcept>
#include "pycvc_scene.h"
// VTK Python bridge: vtkPythonUtil translates C++ vtkProp* <-> live Python
// vtkmodules objects. From the vtk-python cvcpkg package (vtkPythonUtil.h lands
// in include/vtk-9.5/, already on VTK::CommonCore's include path).
#include "vtkPythonUtil.h"
#include "vtkProp.h"
#include "vtkActor.h"
%}

// pycvc.i's %exception (applied via %import) references SWIG_exception, so
// exception.i must be included here too.
%include <exception.i>
%include <std_string.i>
%include <std_vector.i>
%import "pycvc.i"

// ── vtkProp* <-> Python VTK object typemaps (the F3 "full bridge") ──────
// out: return a live vtkmodules wrapper for a C++ prop (new ref; Py_None if null).
%typemap(out) vtkProp* {
  $result = vtkPythonUtil::GetObjectFromPointer($1);
  if (!$result) SWIG_fail;
}
// in: unwrap a Python vtkProp/vtkActor to a C++ vtkProp* (None -> nullptr).
%typemap(in) vtkProp* {
  if ($input == Py_None) {
    $1 = nullptr;
  } else {
    void* _p = vtkPythonUtil::GetPointerFromObject($input, "vtkProp");
    if (!_p) SWIG_fail;  // GetPointerFromObject sets a Python TypeError itself
    $1 = reinterpret_cast<vtkProp*>(_p);
  }
}
%typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) vtkProp* {
  $1 = ($input == Py_None) ||
       (vtkPythonUtil::GetPointerFromObject($input, "vtkProp") != nullptr);
  if (!$1) PyErr_Clear();  // typecheck must not leave an error set
}
// The scene stores props in vtkSmartPointer (Register/UnRegister), so the
// borrowed pointer from GetPointerFromObject is safe to retain. Cover subtypes.
%apply vtkProp* { vtkActor*, vtkVolume*, vtkImageActor* };

%include "pycvc_scene.h"

// ── Live-scene bridge: adopt an embedding host's SceneGraph ─────────────────
// An embedding host (e.g. volrover3) hands its LIVE app + SceneGraph across as
// two PyCapsules — "cvc.app" and "cvc.scenegraph", each holding a heap
// shared_ptr COPY. scene_from_capsule extracts both raw and builds a Scene that
// ADOPTS the existing SceneGraph (no fresh make_shared), so add_geometry/
// add_volume mutate the RUNNING scene and appear in the host's window. Both
// handles cross as raw shared_ptr through the capsule — NOT through SWIG's
// cross-module type table — so this needs no SWIG type sharing with the host and
// no SWIG-runtime-version coupling (mirrors pycvc.app_from_capsule).
%newobject scene_from_capsule;  // Python owns the returned Scene
%inline %{
namespace pycvc {
Scene *scene_from_capsule(PyObject *app_cap, PyObject *scene_cap) {
  if (!app_cap || !PyCapsule_CheckExact(app_cap))
    throw std::invalid_argument("pycvc_gl.scene_from_capsule: app arg is not a PyCapsule");
  if (!scene_cap || !PyCapsule_CheckExact(scene_cap))
    throw std::invalid_argument("pycvc_gl.scene_from_capsule: scene arg is not a PyCapsule");
  void *ap = PyCapsule_GetPointer(app_cap, "cvc.app");
  if (!ap)
    throw std::invalid_argument("pycvc_gl.scene_from_capsule: app capsule is not named \"cvc.app\"");
  void *sp = PyCapsule_GetPointer(scene_cap, "cvc.scenegraph");
  if (!sp)
    throw std::invalid_argument(
        "pycvc_gl.scene_from_capsule: scene capsule is not named \"cvc.scenegraph\"");
  std::shared_ptr<cvc::app> app = *static_cast<std::shared_ptr<cvc::app> *>(ap);
  std::shared_ptr<SceneGraph> sg = *static_cast<std::shared_ptr<SceneGraph> *>(sp);
  return new Scene(app, sg); // adopt-existing ctor
}
} // namespace pycvc
%}

// Round-trip proof of the bridge, independent of the scene graph: hand a Python
// vtkProp in and get the same object back out — exercises both typemaps.
%inline %{
static vtkProp* identity_prop(vtkProp* p) { return p; }
%}
