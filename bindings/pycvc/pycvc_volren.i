// pycvc_volren.i — cvc::volren software raycaster: value types + headless raycaster.
//
// A SUB-interface %include'd into pycvc.i (NOT its own %module): the raycaster is
// genuinely headless (renders a cvc::volume to a cvc::image on CPU/CUDA, no VTK,
// no scene graph), so its value types and front-end join the core `pycvc` module,
// which already wraps cvc::app / cvc::volume / cvc::image / cvc::bounding_box /
// cvc::world_units. cvc::gl::VolRenNode (the drop-in scene node, needs the
// GeometryNode/VTK base) is bound separately in pycvc_gl.i, which %imports these.
//
// The #1 marshalling hazard is std::array members: settings/camera/shadow use
// std::array<float,3> (colours), std::array<double,3> (directions/points/eye) and
// std::array<double,16> (mat4.m). Without the tuple typemaps below SWIG silently
// makes each an opaque proxy (no warning). The typemaps MUST precede the header
// %includes that use them. Collections (isosurfaces/lights/cut_planes/points) are
// exposed through add_*/count/at %extend helpers rather than raw std::vector
// members, which sidesteps the "%template must precede the member" ordering trap.

%{
#include <cvc/volren/volren.h>
%}

// ── std::array<float,3> (colours) ──
%typemap(out) std::array<float, 3> {
  $result = Py_BuildValue("(fff)", $1[0], $1[1], $1[2]);
}
%typemap(varout) std::array<float, 3> {
  $result = Py_BuildValue("(fff)", $1[0], $1[1], $1[2]);
}
%typemap(in) std::array<float, 3> (std::array<float, 3> tmp) {
  if (!PySequence_Check($input) || PySequence_Size($input) != 3) {
    PyErr_SetString(PyExc_ValueError, "expected a 3-sequence");
    SWIG_fail;
  }
  for (Py_ssize_t i = 0; i < 3; ++i) {
    PyObject *o = PySequence_GetItem($input, i);
    tmp[i] = (float)PyFloat_AsDouble(o);
    Py_XDECREF(o);
    if (PyErr_Occurred())
      SWIG_fail;
  }
  $1 = tmp;
}
%typemap(varin) std::array<float, 3> {
  if (!PySequence_Check($input) || PySequence_Size($input) != 3) {
    PyErr_SetString(PyExc_ValueError, "expected a 3-sequence");
    SWIG_fail;
  }
  for (Py_ssize_t i = 0; i < 3; ++i) {
    PyObject *o = PySequence_GetItem($input, i);
    $1[i] = (float)PyFloat_AsDouble(o);
    Py_XDECREF(o);
    if (PyErr_Occurred())
      SWIG_fail;
  }
}

// ── std::array<double,3> (directions / points / eye / focal / up) ──
%typemap(out) std::array<double, 3> {
  $result = Py_BuildValue("(ddd)", $1[0], $1[1], $1[2]);
}
%typemap(varout) std::array<double, 3> {
  $result = Py_BuildValue("(ddd)", $1[0], $1[1], $1[2]);
}
%typemap(in) std::array<double, 3> (std::array<double, 3> tmp) {
  if (!PySequence_Check($input) || PySequence_Size($input) != 3) {
    PyErr_SetString(PyExc_ValueError, "expected a 3-sequence");
    SWIG_fail;
  }
  for (Py_ssize_t i = 0; i < 3; ++i) {
    PyObject *o = PySequence_GetItem($input, i);
    tmp[i] = PyFloat_AsDouble(o);
    Py_XDECREF(o);
    if (PyErr_Occurred())
      SWIG_fail;
  }
  $1 = tmp;
}
%typemap(varin) std::array<double, 3> {
  if (!PySequence_Check($input) || PySequence_Size($input) != 3) {
    PyErr_SetString(PyExc_ValueError, "expected a 3-sequence");
    SWIG_fail;
  }
  for (Py_ssize_t i = 0; i < 3; ++i) {
    PyObject *o = PySequence_GetItem($input, i);
    $1[i] = PyFloat_AsDouble(o);
    Py_XDECREF(o);
    if (PyErr_Occurred())
      SWIG_fail;
  }
}

// ── std::array<double,16> (mat4.m, row-major 4x4) ──
%typemap(varout) std::array<double, 16> {
  $result = PyList_New(16);
  for (Py_ssize_t i = 0; i < 16; ++i)
    PyList_SetItem($result, i, PyFloat_FromDouble($1[i]));
}
%typemap(varin) std::array<double, 16> {
  if (!PySequence_Check($input) || PySequence_Size($input) != 16) {
    PyErr_SetString(PyExc_ValueError, "expected a 16-sequence (row-major 4x4)");
    SWIG_fail;
  }
  for (Py_ssize_t i = 0; i < 16; ++i) {
    PyObject *o = PySequence_GetItem($input, i);
    $1[i] = PyFloat_AsDouble(o);
    Py_XDECREF(o);
    if (PyErr_Occurred())
      SWIG_fail;
  }
}

// ── value headers, dependency order ──
%ignore cvc::volren::mat4::from_row_major; // double[16] arg; set .m (16-list) instead
%include "cvc/volren/types.h"

%ignore cvc::volren::transfer_function::points; // vector<transfer_point>&; use point_at/count
%include "cvc/volren/transfer_function.h"
%extend cvc::volren::transfer_function {
  std::size_t point_count() const { return $self->points().size(); }
  cvc::volren::transfer_point point_at(std::size_t i) const { return $self->points().at(i); }
}

%ignore cvc::volren::shadow_settings::lights;    // vector<int>; per-light subset, deferred
%ignore cvc::volren::shadow_view::project;       // int&/double& out-params
%include "cvc/volren/shadow.h"

// isosurfaces / lights / cut_planes are exposed via add_*/count/at below.
%ignore cvc::volren::volume_settings::isosurfaces;
%ignore cvc::volren::render_settings::lights;
%ignore cvc::volren::render_settings::cut_planes;
%include "cvc/volren/settings.h"
%extend cvc::volren::volume_settings {
  void add_isosurface(const cvc::volren::isosurface &s) { $self->isosurfaces.push_back(s); }
  std::size_t isosurface_count() const { return $self->isosurfaces.size(); }
  cvc::volren::isosurface isosurface_at(std::size_t i) const { return $self->isosurfaces.at(i); }
  void clear_isosurfaces() { $self->isosurfaces.clear(); }
}
%extend cvc::volren::render_settings {
  void add_light(const cvc::volren::light &l) { $self->lights.push_back(l); }
  std::size_t light_count() const { return $self->lights.size(); }
  cvc::volren::light light_at(std::size_t i) const { return $self->lights.at(i); }
  void clear_lights() { $self->lights.clear(); }
  void add_cut_plane(const cvc::volren::cut_plane &c) { $self->cut_planes.push_back(c); }
  std::size_t cut_plane_count() const { return $self->cut_planes.size(); }
  void clear_cut_planes() { $self->cut_planes.clear(); }
}

%ignore cvc::volren::camera::from_pose; // double[3] args; set eye/focal/up (3-tuples)
%include "cvc/volren/camera.h"

// ── raycaster (headless front-end) ──
%ignore cvc::volren::raycaster::raycaster(cvc::app &);      // app& ctor -> shared_ptr factory
%ignore cvc::volren::raycaster::scene_bounds;               // opaque bbox -> 6-tuple
%ignore cvc::volren::raycaster::set_thread_pool;            // raw thread_pool*
%ignore cvc::volren::raycaster::view() const;               // keep the mutable overload
%ignore cvc::volren::raycaster::settings() const;
%ignore cvc::volren::raycaster::volume_config(std::size_t) const;
%include "cvc/volren/raycaster.h"
%pythonappend cvc::volren::raycaster::raycaster(std::shared_ptr<cvc::app>) %{
    self._pycvc_app = app
%}
%extend cvc::volren::raycaster {
  // Construct against an injected app (Python holds shared_ptr<app>, not app&).
  raycaster(std::shared_ptr<cvc::app> app) {
    if (!app)
      throw std::invalid_argument("pycvc_volren.raycaster: null app handle");
    return new cvc::volren::raycaster(*app);
  }
  // scene_bounds() as a (minx..maxz) 6-tuple (the bounding_box return is opaque).
  std::vector<double> scene_bounds_bbox() const {
    cvc::bounding_box b = $self->scene_bounds();
    return {b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz};
  }
}
