// pycvc_gl.i — SWIG module for the cvcGL scene graph.
//
// DIRECT WRAP (mirrors pycvc.i): SWIG wraps the REAL cvcGL classes — SceneGraph
// and the SceneNode -> GraphicsNode -> {GeometryNode, VolumeNode} hierarchy —
// held by std::shared_ptr, exactly as C++ owns them. There is NO facade layer:
// Python calls the actual object methods. sg.addGraphics(name, geom) returns the
// LIVE node; node.setPosition(x,y,z) / node.setColor(r,g,b) / node.setTransform(m)
// mutate it IN PLACE — animation moves coordinates, it never destroys+recreates.
//
// %feature("director") on the node types (added below) lets Python SUBCLASS a
// scene node and have C++ call the Python overrides — e.g. a Python getProp()
// returning a vtkmodules-built vtkProp, which the vtkProp* typemaps marshal.
//
// The heavy cvcGL headers are %include'd and curated with %ignore for the
// VTK-typed / boost::signals2 / std::any / templated members that don't marshal;
// a small set of %extend methods adds the ergonomic surface (vector transform,
// injected-app factory ctor, typed node accessors, node count). %import pulls
// pycvc's app / geometry / volume types across without re-wrapping them.
%module(directors="1", dirprot="1") pycvc_gl

// Windows: register <prefix>/bin (cvc.dll, cvcGL.dll + closure) before the
// `import _pycvc_gl` in the generated proxy — Python 3.8+ ignores PATH for
// extension-module deps. pycvc_gl is a PACKAGE, so its __init__.py sits at
// <prefix>/Lib/site-packages/pycvc_gl/, i.e. <prefix>/bin is ../../../bin (one
// level deeper than pycvc.py). No-op off Windows (POSIX uses RPATH).
%pythonbegin %{
import os as _os, sys as _sys
if _sys.platform == "win32":
    try:
        _cvc_bin = _os.path.normpath(
            _os.path.join(_os.path.dirname(__file__), "..", "..", "..", "bin"))
        if _os.path.isdir(_cvc_bin):
            _os.add_dll_directory(_cvc_bin)
    except (OSError, AttributeError, NameError):
        pass
%}

%{
#include <cvc/core/exception.h> // the %import'd %exception block catches cvc::exception
#include <any>
#include <functional>
#include <stdexcept>
#include <cvc/gl/SceneNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/VolRenNode.h>
#include <cvc/gl/LightNode.h>  // scene light rig (addLight)
#include <cvc/gl/GridNode.h>   // the built-in reference grid (getGridNode)
#include <cvc/gl/AxisNode.h>   // the built-in world axis (getAxisNode)
#include <cvc/gl/VolSliceNode.h> // cvc::volslice view-aligned slice renderer node
#include <cvc/volren/volren.h> // volume_settings/render_settings etc. VolRenNode takes
#include <cvc/gl/NullGraphicNode.h> // the concrete empty node behind add_child_group
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/CameraController.h>
#include <cvc/image/image.h> // GeometryNode::setTexture(const cvc::image&) — image %import'd from pycvc.i
#include "pycvc_scene.h"
// VTK Python bridge: vtkPythonUtil translates C++ vtkProp* <-> live Python
// vtkmodules objects. From the vtk-python cvcpkg package (vtkPythonUtil.h lands
// in include/vtk-9.5/, already on VTK::CommonCore's include path).
#include "vtkPythonUtil.h"
#include "vtkMatrix4x4.h"
#include "vtkProp.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
%}

// pycvc.i's %exception (applied via %import) references SWIG_exception, so
// exception.i must be included here too.
%include <exception.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_shared_ptr.i>
%import "pycvc.i"

// Registering VTK's Python types is what makes renderer()/renderWindow()
// return live vtkmodules objects rather than tripping the guard in the out
// typemap. Best effort: a build without the VTK Python modules still imports,
// it just cannot hand back a scriptable renderer.
%pythoncode %{
try:  # noqa: SIM105
    import vtkmodules.vtkRenderingCore as _vtk_core  # noqa: F401
    import vtkmodules.vtkRenderingOpenGL2 as _vtk_gl  # noqa: F401
except Exception:  # pragma: no cover -- VTK python bindings are optional
    pass
%}

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

// ── the same bridge for the RENDERER and its window ─────────────────────────
// Without these, SceneRenderer::renderer() came back as an opaque SwigPyObject
// and every reason to reach for it failed: AddLight, AddActor2D for a HUD,
// GradientBackgroundOn, a second camera pass. A scene you cannot light or
// annotate from Python is only half-scriptable, so this is the difference
// between "there is a renderer" and "you can use it".
//
// GetObjectFromPointer works for any vtkObjectBase; only the class NAME used
// on the way in differs, which is why these cannot simply %apply the vtkProp
// typemaps (GetPointerFromObject would type-check against "vtkProp").
%define %CVC_VTK_BRIDGE(TYPE, NAME)
%typemap(out) TYPE* {
  $result = vtkPythonUtil::GetObjectFromPointer($1);
  // GetObjectFromPointer returns Py_None for a NULL pointer -- and also when
  // VTK's Python type registry has no wrapper for the class, which happens if
  // the corresponding vtkmodules package was never imported. Silently handing
  // back None there is the worst outcome: the caller sees a renderer that is
  // not None-checked and fails one line later with a confusing AttributeError.
  if ($1 && (!$result || $result == Py_None)) {
    Py_XDECREF($result);
    PyErr_SetString(PyExc_RuntimeError,
                    "pycvc_gl: VTK Python types are not registered; "
                    "import vtkmodules.vtkRenderingOpenGL2 before using this");
    SWIG_fail;
  }
  if (!$result) SWIG_fail;
}
%typemap(in) TYPE* {
  if ($input == Py_None) {
    $1 = nullptr;
  } else {
    void* _p = vtkPythonUtil::GetPointerFromObject($input, NAME);
    if (!_p) SWIG_fail;  // GetPointerFromObject sets a Python TypeError itself
    $1 = reinterpret_cast<TYPE*>(_p);
  }
}
%typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) TYPE* {
  $1 = ($input == Py_None) ||
       (vtkPythonUtil::GetPointerFromObject($input, NAME) != nullptr);
  if (!$1) PyErr_Clear();  // typecheck must not leave an error set
}
%enddef

%CVC_VTK_BRIDGE(vtkRenderer, "vtkRenderer")
%CVC_VTK_BRIDGE(vtkRenderWindow, "vtkRenderWindow")

// ── PyCallable -> std::function<void()> ─────────────────────────────────────
// A Python callable crosses as a C++ std::function so Python functions can be
// used for scene callbacks (SceneGraph::postEvent, on_graphics_changed, ...).
// A shared_ptr holder owns one reference and DECREFs it (under the GIL) when the
// last copy of the std::function is destroyed; the call site re-acquires the GIL
// and reports (does not swallow into C++) any Python exception.
%typemap(in) std::function<void()> {
  if (!PyCallable_Check($input))
    SWIG_exception_fail(SWIG_TypeError, "expected a callable for std::function<void()>");
  Py_INCREF($input);
  std::shared_ptr<PyObject> _cb($input, [](PyObject *p) {
    PyGILState_STATE g = PyGILState_Ensure();
    Py_DECREF(p);
    PyGILState_Release(g);
  });
  $1 = [_cb]() {
    PyGILState_STATE g = PyGILState_Ensure();
    PyObject *r = PyObject_CallObject(_cb.get(), nullptr);
    if (!r)
      PyErr_Print();
    else
      Py_DECREF(r);
    PyGILState_Release(g);
  };
}
%typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) std::function<void()> {
  $1 = PyCallable_Check($input) ? 1 : 0;
}
// directorout: when a Python-defined node's getProp() returns a vtkmodules
// object, unwrap it to the C++ vtkProp* the scene renders (None -> nullptr).
// This is what makes a Python scene node's Python-built actor flow into C++.
%typemap(directorout) vtkProp* {
  if ($input == Py_None) {
    $result = nullptr;
  } else {
    void* _p = vtkPythonUtil::GetPointerFromObject($input, "vtkProp");
    if (!_p) {
      PyErr_Clear();
      throw Swig::DirectorMethodException("getProp() must return a vtkProp (or None)");
    }
    $result = reinterpret_cast<vtkProp*>(_p);
  }
}

// ── shared_ptr the whole node hierarchy (base classes FIRST) ────────────────
// Every cvcGL node is created and passed as std::shared_ptr (see
// GraphicsNode::addGraphicsChild / SceneGraph::getGraphics). Declaring the
// hierarchy shared_ptr-managed makes getGraphics()/addGraphics() return a proxy
// that CO-OWNS the live node, so Python can hold and mutate it safely.
%shared_ptr(cvc::gl::SceneNode)
%shared_ptr(cvc::gl::GraphicsNode)
%shared_ptr(cvc::gl::GeometryNode)
%shared_ptr(cvc::gl::VolumeNode)
%shared_ptr(cvc::gl::VolRenNode)
%shared_ptr(cvc::gl::LightNode)
%shared_ptr(cvc::gl::GridNode)
%shared_ptr(cvc::gl::AxisNode)
%shared_ptr(cvc::gl::VolSliceNode)
%shared_ptr(cvc::gl::SceneGraph)

// ── directors: Python-defined scene node types ──────────────────────────────
// With directors on the node classes, Python can SUBCLASS a scene node and have
// C++ call the Python overrides — e.g. override getProp() to return a
// vtkmodules-built vtkProp (marshalled by the directorout typemap above), so a
// pure-Python node renders in the C++ scene. dirprot (module flag) exposes the
// protected virtuals (getProp / handleStateChanged / applyTransformToVTK) so
// they are overridable. The concrete leaves carry C++ impls of the pure virtuals
// (getProp / getBoundingBox), so a Python subclass need only override what it
// wants to customize.
%feature("director") cvc::gl::GraphicsNode;
%feature("director") cvc::gl::GeometryNode;
%feature("director") cvc::gl::VolumeNode;
%feature("director") cvc::gl::VolRenNode;
%feature("director") cvc::gl::VolSliceNode;

// A Python-CONSTRUCTED node (a director subclass built as MyNode(app, path,
// name)) must keep its app alive too — its ~SceneNode touches the app's state
// tree by raw reference. args[0] is the app the node ctor takes. (Nodes obtained
// from a SceneGraph get this via the SceneGraph appends below instead.)
%pythonappend cvc::gl::GraphicsNode::GraphicsNode %{
    if args: self._pycvc_app = args[0]
%}
%pythonappend cvc::gl::GeometryNode::GeometryNode %{
    if args: self._pycvc_app = args[0]
%}
%pythonappend cvc::gl::VolumeNode::VolumeNode %{
    if args: self._pycvc_app = args[0]
%}
%pythonappend cvc::gl::VolRenNode::VolRenNode %{
    if args: self._pycvc_app = args[0]
%}
%pythonappend cvc::gl::LightNode::LightNode %{
    if args: self._pycvc_app = args[0]
%}
%pythonappend cvc::gl::GridNode::GridNode %{
    if args: self._pycvc_app = args[0]
%}
%pythonappend cvc::gl::AxisNode::AxisNode %{
    if args: self._pycvc_app = args[0]
%}
%pythonappend cvc::gl::VolSliceNode::VolSliceNode %{
    if args: self._pycvc_app = args[0]
%}

// ── SceneNode (abstract base): trim VTK / threading internals ───────────────
%ignore cvc::gl::SceneNode::addToRenderer;
%ignore cvc::gl::SceneNode::removeFromRenderer;
%ignore cvc::gl::SceneNode::runOnMainThread;
%ignore cvc::gl::SceneNode::setSceneGraph;
%ignore cvc::gl::SceneNode::getSceneGraph;
%include "cvc/gl/SceneNode.h"

// ── GraphicsNode: keep transform / material / label; ignore VTK/any/templates ─
%ignore cvc::gl::GraphicsNode::setTransform(vtkMatrix4x4 *);
%ignore cvc::gl::GraphicsNode::setTransform(const double[16]); // replaced by the vector<double> %extend
%ignore cvc::gl::GraphicsNode::getTransform;
%ignore cvc::gl::GraphicsNode::getWorldTransform;
%ignore cvc::gl::GraphicsNode::getClipPlanes;
%ignore cvc::gl::GraphicsNode::setMetadata;
%ignore cvc::gl::GraphicsNode::getMetadata;
%ignore cvc::gl::GraphicsNode::hasMetadata;
%ignore cvc::gl::GraphicsNode::getAllMetadata;
%ignore cvc::gl::GraphicsNode::getGraphicsChildren;      // vector<shared_ptr<...>> return
// bounding_box (generic_bounding_box<double>) is opaque here; its greedy
// templated converting ctor mis-binds SWIG's SwigValueWrapper on a by-value
// return, so every bounding_box-returning method is ignored (as in pycvc.i).
%ignore cvc::gl::GraphicsNode::getBoundingBox;
%ignore cvc::gl::GraphicsNode::getCombinedBoundingBox;
%ignore cvc::gl::GraphicsNode::getWorldBoundingBox;         // opaque bbox return -> 6-tuple below
%ignore cvc::gl::GraphicsNode::getCombinedWorldBoundingBox; // opaque bbox return -> 6-tuple below
%ignore cvc::gl::GraphicsNode::localToWorld;                // double[3] in/out -> vector below
%ignore cvc::gl::GraphicsNode::worldToLocal;                // double[3] in/out -> vector below
%ignore cvc::gl::GraphicsNode::localPointToReal;            // double[3] in -> (x,y,z,unit) tuple
%ignore cvc::gl::GraphicsNode::realDimensions;              // coordinate return -> tuple below
%ignore cvc::gl::GraphicsNode::getBBoxColor;             // out-ref params
%ignore cvc::gl::GraphicsNode::getLabelColor;
%ignore cvc::gl::GraphicsNode::getExtentLabelColor;
%ignore cvc::gl::GraphicsNode::addToRenderer;
%ignore cvc::gl::GraphicsNode::removeFromRenderer;
%ignore cvc::gl::GraphicsNode::addGraphicsChild;         // templates + shared_ptr overload
%ignore cvc::gl::GraphicsNode::createChild;
%ignore cvc::gl::GraphicsNode::transformChanged;         // public boost::signals2::signal member
%extend cvc::gl::GraphicsNode {
  // Row-major 4x4 transform from a 16-element list (the vtkMatrix4x4 overload is
  // ignored; this is the Python-friendly path). Full rotate/scale/translate.
  void setTransform(const std::vector<double>& m) {
    if (m.size() != 16)
      throw std::invalid_argument("setTransform: need 16 doubles (row-major 4x4)");
    $self->setTransform(m.data());
  }
  // Read this node's local transform as a 16-element row-major list (the
  // vtkMatrix4x4 return is ignored; this marshals cleanly).
  std::vector<double> get_transform() {
    std::vector<double> out(16, 0.0);
    out[0] = out[5] = out[10] = out[15] = 1.0;
    if (vtkMatrix4x4* m = $self->getTransform())
      for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) out[i * 4 + j] = m->GetElement(i, j);
    return out;
  }
  // The accumulated world transform (this node * all parents), row-major 16.
  std::vector<double> get_world_transform() {
    std::vector<double> out(16, 0.0);
    out[0] = out[5] = out[10] = out[15] = 1.0;
    if (auto m = $self->getWorldTransform())
      for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) out[i * 4 + j] = m->GetElement(i, j);
    return out;
  }
  // (minx, miny, minz, maxx, maxy, maxz) — the opaque bounding_box returns are
  // ignored; these expose them as plain 6-tuples.
  std::vector<double> get_bounding_box() {
    cvc::bounding_box b = $self->getBoundingBox();
    return {b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz};
  }
  std::vector<double> get_combined_bounding_box() {
    cvc::bounding_box b = $self->getCombinedBoundingBox();
    return {b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz};
  }
  // Point transforms between this node's LOCAL frame and WORLD space (the
  // double[3] overloads are ignored). Take/return a [x,y,z] list.
  std::vector<double> local_to_world(const std::vector<double>& p) {
    if (p.size() != 3)
      throw std::invalid_argument("local_to_world: need [x, y, z]");
    double in[3] = {p[0], p[1], p[2]}, out[3];
    $self->localToWorld(in, out);
    return {out[0], out[1], out[2]};
  }
  std::vector<double> world_to_local(const std::vector<double>& p) {
    if (p.size() != 3)
      throw std::invalid_argument("world_to_local: need [x, y, z]");
    double in[3] = {p[0], p[1], p[2]}, out[3];
    $self->worldToLocal(in, out);
    return {out[0], out[1], out[2]};
  }
  // World-space AABB of this node (and node+subtree) as (minx..maxz) 6-tuples,
  // reliable through the whole chain of local transforms.
  std::vector<double> get_world_bounding_box() {
    cvc::bounding_box b = $self->getWorldBoundingBox();
    return {b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz};
  }
  std::vector<double> get_combined_world_bounding_box() {
    cvc::bounding_box b = $self->getCombinedWorldBoundingBox();
    return {b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz};
  }
  // A point in this node's LOCAL frame as a real-world coordinate in the given
  // world_units regime: an (x, y, z, unit) tuple (e.g. (3.2, 0.5, -1.0, "km")).
  PyObject* local_point_to_real(const std::vector<double>& p, const cvc::world_units& u) {
    if (p.size() != 3)
      throw std::invalid_argument("local_point_to_real: need [x, y, z]");
    double in[3] = {p[0], p[1], p[2]};
    cvc::world_units::coordinate c = $self->localPointToReal(in, u);
    return Py_BuildValue("(ddds)", c.x, c.y, c.z, c.unit.c_str());
  }
  // This node's real-world size in the regime: an (x, y, z, unit) tuple, taken
  // through the whole chain of local transforms (with descendants by default).
  PyObject* real_dimensions(const cvc::world_units& u, bool include_children = true) {
    cvc::world_units::coordinate c = $self->realDimensions(u, include_children);
    return Py_BuildValue("(ddds)", c.x, c.y, c.z, c.unit.c_str());
  }
  // Names of this node's direct children (traverse via SceneGraph.getGraphics).
  std::vector<std::string> child_names() {
    std::vector<std::string> out;
    for (auto& c : $self->getGraphicsChildren())
      if (c) out.push_back(c->getName());
    return out;
  }
  // Per-node metadata backed by std::any: bool / int / float / str round-trip by
  // type_info; other types raise. (The raw std::any accessors are ignored.)
  void set_metadata(const std::string& key, PyObject* value) {
    if (PyBool_Check(value))
      $self->setMetadata(key, std::any(value == Py_True));
    else if (PyLong_Check(value))
      $self->setMetadata(key, std::any(static_cast<long>(PyLong_AsLong(value))));
    else if (PyFloat_Check(value))
      $self->setMetadata(key, std::any(PyFloat_AsDouble(value)));
    else if (PyUnicode_Check(value))
      $self->setMetadata(key, std::any(std::string(PyUnicode_AsUTF8(value))));
    else
      throw std::invalid_argument("set_metadata: value must be bool, int, float, or str");
  }
  PyObject* get_metadata(const std::string& key) {
    if (!$self->hasMetadata(key))
      Py_RETURN_NONE;
    std::any a = $self->getMetadata(key);
    const std::type_info& t = a.type();
    if (t == typeid(bool))
      return PyBool_FromLong(std::any_cast<bool>(a));
    if (t == typeid(long))
      return PyLong_FromLong(std::any_cast<long>(a));
    if (t == typeid(double))
      return PyFloat_FromDouble(std::any_cast<double>(a));
    if (t == typeid(std::string))
      return PyUnicode_FromString(std::any_cast<std::string>(a).c_str());
    Py_RETURN_NONE; // unknown stored type
  }
  bool has_metadata(const std::string& key) { return $self->hasMetadata(key); }
}
%include "cvc/gl/GraphicsNode.h"

// ── GeometryNode: setGeometry (in-place data), material + render-mode setters ─
// enum class + scalar setters + cvc::geometry (%import'd) marshal cleanly; only
// the opaque-by-value bbox override needs ignoring. setTexture / clearTexture /
// texture_modified auto-wrap (cvc::image is %import'd from pycvc.i); the snake
// aliases below match the pycvc image/texture demo surface.
%ignore cvc::gl::GeometryNode::getBoundingBox;
// Replace the std::vector<double> updateVertices with a numpy-direct one (below) so
// the per-frame deform path reads the buffer directly instead of via .tolist().
%ignore cvc::gl::GeometryNode::updateVertices(const std::vector<double> &);
%extend cvc::gl::GeometryNode {
  // Zero-copy texture (default): the vtkTexture aliases img's RGBA8 buffer, so a
  // later img.numpy() pixel edit + texture_modified() shows live with no re-copy.
  void set_texture(const cvc::image& img) { $self->setTexture(img, /*zeroCopy=*/true); }
  // Convert-flip-and-copy fallback (any format; the texture owns its own copy).
  void set_texture_copy(const cvc::image& img) { $self->setTexture(img, /*zeroCopy=*/false); }
  void clear_texture() { $self->clearTexture(); }

  // numpy-direct vertex update: accept a C-contiguous float64 buffer (e.g. a numpy
  // array's .ravel()) and blit it in place — skipping the ~N Python-float
  // allocations that .tolist() would force on every frame of a deforming mesh.
  // The std::vector<double> overload is %ignored for Python; this is updateVertices.
  void updateVertices(PyObject *buf) {
    Py_buffer view;
    if (PyObject_GetBuffer(buf, &view, PyBUF_C_CONTIGUOUS | PyBUF_FORMAT) != 0) {
      PyErr_Clear();
      throw std::invalid_argument(
          "updateVertices: expected a C-contiguous float64 buffer (e.g. a numpy array's .ravel())");
    }
    const char *fmt = view.format ? view.format : "";
    bool is_double = fmt[0] == 'd' || ((fmt[0] == '<' || fmt[0] == '=') && fmt[1] == 'd');
    if (!is_double) {
      PyBuffer_Release(&view);
      throw std::invalid_argument("updateVertices: expected float64 (double) data");
    }
    const double *data = static_cast<const double *>(view.buf);
    std::vector<double> xyz(data, data + view.len / sizeof(double));
    PyBuffer_Release(&view);
    $self->updateVertices(xyz);
  }
}
%include "cvc/gl/GeometryNode.h"

// ── VolumeNode: transfer function (vector<double>) + rendering props ────────
%ignore cvc::gl::VolumeNode::addToRenderer;
%ignore cvc::gl::VolumeNode::getBoundingBox;
%include "cvc/gl/VolumeNode.h"

// ── VolRenNode: the cvc::volren software raycaster as a scene node ──────────
// Derives from GeometryNode (already shared_ptr'd/director'd above); the volren
// value types it takes (volume_settings/render_settings) come from pycvc_volren.i
// via the %import of pycvc.i. addVolume/tick/config drive the raycast; the mesh
// API it inherits from GeometryNode is a documented WART here (corrupts the quad)
// so it is hidden.
%ignore cvc::gl::VolRenNode::getBoundingBox;         // opaque bbox -> 6-tuple below
%ignore cvc::gl::VolRenNode::volumePointToReal;      // world_units::coordinate -> tuple below
%ignore cvc::gl::VolRenNode::volumeRealDimensions;   // world_units::coordinate -> tuple below
%ignore cvc::gl::VolRenNode::setGeometry;            // inherited mesh API: meaningless here
%ignore cvc::gl::VolRenNode::updateVertices;
%ignore cvc::gl::VolRenNode::updateColors;
%ignore cvc::gl::VolRenNode::setRenderMode;
%include "cvc/gl/VolRenNode.h"
%extend cvc::gl::VolRenNode {
  // Real-world coordinate/size of a rendered volume (world_units::coordinate ->
  // (x, y, z, unit) tuple), composing the per-volume model_transform, this node's
  // world transform, and the regime.
  PyObject *volume_point_to_real(std::size_t index, double ox, double oy, double oz,
                                 const cvc::world_units &u) {
    cvc::world_units::coordinate c = $self->volumePointToReal(index, ox, oy, oz, u);
    return Py_BuildValue("(ddds)", c.x, c.y, c.z, c.unit.c_str());
  }
  PyObject *volume_real_dimensions(std::size_t index, const cvc::world_units &u) {
    cvc::world_units::coordinate c = $self->volumeRealDimensions(index, u);
    return Py_BuildValue("(ddds)", c.x, c.y, c.z, c.unit.c_str());
  }
  // The node's (all volumes') box in its local frame as a (minx..maxz) 6-tuple.
  std::vector<double> get_bounding_box() {
    cvc::bounding_box b = $self->getBoundingBox();
    return {b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz};
  }
}

// ── LightNode: a scene light (SceneGraph::addLight returns one) ─────────────
// A GraphicsNode with no visual prop of its own; it drives the renderer's light
// set. The Kind enum + the double& out-params (target/color/worldPosition) are
// re-exposed as a string and (x,y,z) tuples, mirroring the world_units/volren
// idiom. getProp()/getBoundingBox() are trivial overrides with no Python value.
%ignore cvc::gl::LightNode::Kind;              // nested enum -> string set_kind/kind_str
%ignore cvc::gl::LightNode::setKind;           // takes Kind
%ignore cvc::gl::LightNode::kind;              // returns Kind
%ignore cvc::gl::LightNode::target;            // double& out-params -> get_target tuple
%ignore cvc::gl::LightNode::color;             // double& out-params -> get_color tuple
%ignore cvc::gl::LightNode::worldPosition;     // double& out-params -> world_position tuple
%ignore cvc::gl::LightNode::getProp;           // vtkProp* (always nullptr here)
%ignore cvc::gl::LightNode::getBoundingBox;    // opaque bbox (empty here)
%include "cvc/gl/LightNode.h"
%extend cvc::gl::LightNode {
  // Kind as a string: "spot" | "directional" | "fill".
  void set_kind(const std::string &k) {
    if (k == "spot")
      $self->setKind(cvc::gl::LightNode::Kind::Spot);
    else if (k == "directional")
      $self->setKind(cvc::gl::LightNode::Kind::Directional);
    else if (k == "fill")
      $self->setKind(cvc::gl::LightNode::Kind::Fill);
    else
      throw std::invalid_argument("LightNode.set_kind: expected 'spot'|'directional'|'fill'");
  }
  std::string kind_str() const {
    switch ($self->kind()) {
    case cvc::gl::LightNode::Kind::Spot:
      return "spot";
    case cvc::gl::LightNode::Kind::Directional:
      return "directional";
    case cvc::gl::LightNode::Kind::Fill:
      return "fill";
    }
    return "spot";
  }
  PyObject *get_target() const {
    double x, y, z;
    $self->target(x, y, z);
    return Py_BuildValue("(ddd)", x, y, z);
  }
  PyObject *get_color() const {
    double r, g, b;
    $self->color(r, g, b);
    return Py_BuildValue("(ddd)", r, g, b);
  }
  // World-space position of the light, resolved through the transform chain.
  PyObject *world_position() const {
    double x, y, z;
    $self->worldPosition(x, y, z);
    return Py_BuildValue("(ddd)", x, y, z);
  }
}

// ── GridNode: the reference grid (SceneGraph::getGridNode) ──────────────────
// Bounds + per-plane colours/visibility + tick config. bounding_box in/out is
// opaque, so setBounds/bounds are re-exposed as 6-tuples; the double&/int& colour
// and division out-params become (r,g,b)/(x,y,z) tuples. VTK renderer hooks hide.
%ignore cvc::gl::GridNode::setBounds;             // opaque bbox -> set_bounds(6 doubles)
%ignore cvc::gl::GridNode::bounds;                // opaque bbox -> get_bounds tuple
%ignore cvc::gl::GridNode::getBoundingBox;        // opaque bbox
%ignore cvc::gl::GridNode::addToRenderer;         // vtkRenderer*
%ignore cvc::gl::GridNode::removeFromRenderer;    // vtkRenderer*
%ignore cvc::gl::GridNode::getYZPlaneColor;       // double& out-params -> tuple
%ignore cvc::gl::GridNode::getXZPlaneColor;
%ignore cvc::gl::GridNode::getXYPlaneColor;
%ignore cvc::gl::GridNode::getGridDivisions;      // int& out-params -> tuple
%ignore cvc::gl::GridNode::getTickIntervals;      // int& out-params -> tuple
%ignore cvc::gl::GridNode::getTickLabelColor;     // double& out-params -> tuple
%include "cvc/gl/GridNode.h"
%extend cvc::gl::GridNode {
  void set_bounds(double minx, double miny, double minz, double maxx, double maxy, double maxz) {
    $self->setBounds(cvc::bounding_box(minx, miny, minz, maxx, maxy, maxz));
  }
  PyObject *get_bounds() const {
    const cvc::bounding_box &b = $self->bounds();
    return Py_BuildValue("(dddddd)", b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz);
  }
  PyObject *get_yz_plane_color() const {
    double r, g, b;
    $self->getYZPlaneColor(r, g, b);
    return Py_BuildValue("(ddd)", r, g, b);
  }
  PyObject *get_xz_plane_color() const {
    double r, g, b;
    $self->getXZPlaneColor(r, g, b);
    return Py_BuildValue("(ddd)", r, g, b);
  }
  PyObject *get_xy_plane_color() const {
    double r, g, b;
    $self->getXYPlaneColor(r, g, b);
    return Py_BuildValue("(ddd)", r, g, b);
  }
  PyObject *get_grid_divisions() const {
    int x, y, z;
    $self->getGridDivisions(x, y, z);
    return Py_BuildValue("(iii)", x, y, z);
  }
  PyObject *get_tick_intervals() const {
    int x, y, z;
    $self->getTickIntervals(x, y, z);
    return Py_BuildValue("(iii)", x, y, z);
  }
  PyObject *get_tick_label_color() const {
    double r, g, b;
    $self->getTickLabelColor(r, g, b);
    return Py_BuildValue("(ddd)", r, g, b);
  }
}

// ── AxisNode: the world axis gizmo (SceneGraph::getAxisNode) ────────────────
%ignore cvc::gl::AxisNode::getBoundingBox; // opaque bbox
%include "cvc/gl/AxisNode.h"

// ── VolSliceNode: the cvc::volslice view-aligned slice renderer as a node ───
// Derives from GeometryNode (shared_ptr'd/director'd above). Its config type
// cvc::volslice::render_settings is wrapped (renamed volslice_render_settings)
// in pycvc_volslice.i and reaches here via the %import of pycvc.i, so
// config()/setConfig() marshal directly — no %extend, like VolumeNode's props.
// The inherited GeometryNode mesh API is a WART here (it drives per-frame slice
// fans itself), so it is hidden, exactly as for VolRenNode. tick()/planesRendered
// are KEPT but upload GL textures — a live context is needed to call tick(), so
// the Python tests assert their presence, not a live tick (see test note).
%ignore cvc::gl::VolSliceNode::getBoundingBox;      // opaque bbox -> 6-tuple below
%ignore cvc::gl::VolSliceNode::addToRenderer;       // vtkRenderer* (toggles OIT)
%ignore cvc::gl::VolSliceNode::depthSortSliceProps; // static; vtkRenderer* + vector<VolSliceNode*>
%ignore cvc::gl::VolSliceNode::setGeometry;         // inherited mesh WART: corrupts slices
%ignore cvc::gl::VolSliceNode::updateVertices;      // inherited mesh WART
%ignore cvc::gl::VolSliceNode::updateColors;        // inherited mesh WART
%ignore cvc::gl::VolSliceNode::setRenderMode;       // inherited mesh WART
%include "cvc/gl/VolSliceNode.h"
%extend cvc::gl::VolSliceNode {
  // The node's box in its local frame as a (minx..maxz) 6-tuple (the
  // bounding_box return is opaque — the SwigValueWrapper mis-bind).
  std::vector<double> get_bounding_box() {
    cvc::bounding_box b = $self->getBoundingBox();
    return {b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz};
  }
}

// ── SceneGraph: the top-level graph. App injected explicitly (no singleton). ─
%ignore cvc::gl::SceneGraph::SceneGraph(const std::string &);           // process-wide singleton ctor
%ignore cvc::gl::SceneGraph::SceneGraph(cvc::app &, const std::string &); // re-exposed via shared_ptr factory
%ignore cvc::gl::SceneGraph::setRenderer;
// SceneGraph::postEvent(std::function<void()>) is NOT ignored — the callable
// typemap above marshals a Python function to the std::function, so Python can
// post work onto the scene's owner thread.
// getGridNode()/getAxisNode() are re-exposed: GridNode/AxisNode are now wrapped
// (shared_ptr) above, so the built-in reference grid + world axis are reachable
// and mutable from Python (set_bounds / colours / divisions / axis length).
%ignore cvc::gl::SceneGraph::getAllGraphics;
%ignore cvc::gl::SceneGraph::getAllGraphicsOfType;
%ignore cvc::gl::SceneGraph::getAllVolumeGraphics;
%ignore cvc::gl::SceneGraph::getAllGeometryGraphics;
%ignore cvc::gl::SceneGraph::updateTransferFunction;
%ignore cvc::gl::SceneGraph::updateGrid;
%ignore cvc::gl::SceneGraph::computeGraphicsBounds;
%ignore cvc::gl::SceneGraph::computeVolumeBounds;
%ignore cvc::gl::SceneGraph::graphicsChanged; // public boost::signals2::signal member
// Keep the app alive for at least as long as the SceneGraph proxy: SceneGraph
// holds the app by RAW reference (cvc::app& m_ctx), so without this the app
// could be torn down first (at GC / interpreter shutdown) and ~SceneGraph would
// lock a destroyed state mutex (boost::lock_error). Stash it on the proxy — the
// same keep-alive pycvc.i uses for volume/geometry. This %feature MUST precede
// the %extend ctor it targets. The generated __init__ is `def __init__(self,
// *args)` and the sole ctor is the injected-app factory, so the app is args[0].
// (In the adopt case the host owns app+scene for the whole session, so the raw
// scene_from_capsule proxy needs no stash; grl_snam_lab.Lab holds the app too.)
%pythonappend cvc::gl::SceneGraph::SceneGraph %{
    if args:
        self._pycvc_app = args[0]
%}
// Propagate that keep-alive to every RETURNED node proxy: a node's C++ teardown
// (~SceneNode) touches the app's state tree by raw reference, so a node proxy
// that outlives the app (possible at interpreter shutdown, when a script holds a
// node global) would lock a destroyed state mutex. Stashing the app on each node
// makes the app outlive every node proxy. `val` is SWIG's result local; it is
// None when a lookup misses or a typed cast fails. (Adopted scenes have no
// _pycvc_app — the host owns app+scene+nodes for the whole session, so None is
// correct there.) These %feature lines MUST precede the wrapping below.
// getGraphics()/addGraphics(name, geom) are typed shared_ptr<GraphicsNode> at
// the C++ boundary, so SWIG hands Python the BASE proxy — a GeometryNode's
// set_texture() / a VolumeNode's setVolume() are invisible on it. Downcast the
// result to its concrete node type via the typed accessors so scripts can do
// sg.getGraphics(name).set_texture(img) / .setVolume(vol) directly (as the
// pycvc image/texture + SDF demos do), not only through geometry_node()/
// volume_node(). args[0] is the node name for both wrapped methods.
%pythoncode %{
def _typed_node(sg, name):
    n = sg.geometry_node(name)
    if n is None:
        n = sg.volume_node(name)
    if n is None:
        n = sg.volren_node(name)
    if n is None:
        n = sg.light_node(name)
    return n
%}
%pythonappend cvc::gl::SceneGraph::getGraphics %{
    if val is not None:
        _t = _typed_node(self, name)
        if _t is not None: val = _t
        val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::getGraphicsRoot %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::addGraphics %{
    if val is not None:
        _t = _typed_node(self, args[0])
        if _t is not None: val = _t
        val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::geometry_node %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
// The child-adders return live node proxies too, so they need the same app
// keep-alive as getGraphics/addGraphics: a node proxy that outlives the app
// would lock a destroyed state mutex in ~SceneNode.
%pythonappend cvc::gl::SceneGraph::add_child_group %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::add_group %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::add_child_geometry %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::add_child_volume %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::add_child_volren %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::add_volren %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::volren_node %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::volume_node %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
// Light / grid / axis return live node proxies — same app keep-alive.
%pythonappend cvc::gl::SceneGraph::addLight %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::light_node %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::getGridNode %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::getAxisNode %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
// VolSlice factories return live node proxies — same app keep-alive.
%pythonappend cvc::gl::SceneGraph::add_child_volslice %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::add_volslice %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend cvc::gl::SceneGraph::volslice_node %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%extend cvc::gl::SceneGraph {
  // NOTE: swig parses the DECLARATIONS below in cvc::gl scope (so a bare
  // `GeometryNode` return type resolves), but emits each BODY verbatim as a
  // free function at global scope. Bodies therefore have to name cvcGL types
  // fully qualified — unqualified ones broke the wrapper's compile when these
  // classes moved into cvc::gl, with no swig diagnostic at generation time.
  // Standalone factory: build a scene under an EXPLICIT injected app (no
  // singleton), mirroring pycvc.volume(app). `app` must outlive the scene.
  SceneGraph(std::shared_ptr<cvc::app> app, const std::string& prefix = "cvcgl") {
    if (!app)
      throw std::invalid_argument("pycvc_gl.SceneGraph: null app handle");
    return new cvc::gl::SceneGraph(*app, prefix);
  }
  // Node count (getAllGraphics() is ignored for the Python surface but callable
  // here in C++).
  std::size_t num_graphics() const { return $self->getAllGraphics().size(); }
  // Typed accessors: return the CONCRETE node so its type-specific setters
  // (GeometryNode::setColor, VolumeNode::setTransferFunction) are visible from
  // Python — getGraphics() alone yields the GraphicsNode base. Null if `name` is
  // absent or not of that type.
  std::shared_ptr<GeometryNode> geometry_node(const std::string& name) {
    return std::dynamic_pointer_cast<cvc::gl::GeometryNode>($self->getGraphics(name));
  }
  std::shared_ptr<VolumeNode> volume_node(const std::string& name) {
    return std::dynamic_pointer_cast<cvc::gl::VolumeNode>($self->getGraphics(name));
  }
  // Insert a caller-constructed node (e.g. a Python DIRECTOR subclass of
  // GraphicsNode/GeometryNode) into the render tree under `name`. This is the
  // hook that lets a Python-defined scene type join the scene: C++ then calls
  // the node's getProp()/getBoundingBox() overrides. addGraphicsChild wires it
  // into the render tree; registerGraphics exposes it in the flat name lookup.
  void add_node(const std::string& name, std::shared_ptr<GraphicsNode> node) {
    if (!node)
      throw std::invalid_argument("pycvc_gl.SceneGraph.add_node: null node");
    node->setName(name);
    $self->getGraphicsRoot()->addGraphicsChild(node);
    $self->registerGraphics(name, node);
  }
  // Names of all registered graphics nodes (getAllGraphics is ignored for the
  // Python surface; this exposes its keys).
  std::vector<std::string> graphics_names() const {
    std::vector<std::string> out;
    for (auto& kv : $self->getAllGraphics()) out.push_back(kv.first);
    return out;
  }
  // Combined world bounds of all graphics / all volumes as (minx..maxz) — the
  // opaque bounding_box returns are ignored; these expose them as 6-tuples.
  std::vector<double> compute_graphics_bounds() const {
    cvc::bounding_box b = $self->computeGraphicsBounds();
    return {b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz};
  }
  std::vector<double> compute_volume_bounds() const {
    cvc::bounding_box b = $self->computeVolumeBounds();
    return {b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz};
  }
  // Resize the world grid/box to (minx..maxz) (updateGrid takes an opaque bbox).
  void update_grid(double minx, double miny, double minz, double maxx, double maxy, double maxz) {
    $self->updateGrid(cvc::bounding_box(minx, miny, minz, maxx, maxy, maxz));
  }
  // Add an EMPTY grouping node as a child of `parent` — a pure transform node.
  //
  // Without this, every node in a Python-built hierarchy had to carry geometry,
  // because add_child_geometry/add_child_volume were the only parenting
  // primitives on offer (GraphicsNode::addGraphicsChild is a template, so SWIG
  // cannot wrap it). That forced callers to flatten any multi-step local
  // transform into a single matrix in Python instead of letting the graph
  // compose it — see the lsystem_tree volrover3 example, where a whole turtle
  // path per module gets collapsed for exactly this reason.
  //
  // NullGraphicNode is the concrete "no visual data" GraphicsNode (GraphicsNode
  // itself is abstract — getProp() is pure virtual). Its bounds default to
  // syncing with its children, so a group reports the extent of what it holds.
  std::shared_ptr<GraphicsNode> add_child_group(const std::string& parent,
                                                const std::string& name) {
    auto p = $self->getGraphics(parent);
    if (!p)
      throw std::invalid_argument("add_child_group: no parent node named '" + parent + "'");
    auto child = p->addGraphicsChild<cvc::gl::NullGraphicNode>(name);
    $self->registerGraphics(name, child);
    return child;
  }
  // The same, at the top of the graph.
  std::shared_ptr<GraphicsNode> add_group(const std::string& name) {
    auto child = $self->getGraphicsRoot()->addGraphicsChild<cvc::gl::NullGraphicNode>(name);
    $self->registerGraphics(name, child);
    return child;
  }
  // Add a geometry / volume as a CHILD of `parent` (inherits its transform), and
  // register it so getGraphics(name) finds it.
  std::shared_ptr<GeometryNode> add_child_geometry(const std::string& parent,
                                                   const std::string& name,
                                                   const cvc::geometry& g) {
    auto p = $self->getGraphics(parent);
    if (!p)
      throw std::invalid_argument("add_child_geometry: no parent node named '" + parent + "'");
    auto child = p->addGraphicsChild<cvc::gl::GeometryNode>(name);
    child->setGeometry(g);
    $self->registerGraphics(name, child);
    return child;
  }
  std::shared_ptr<VolumeNode> add_child_volume(const std::string& parent, const std::string& name,
                                               const cvc::volume& v) {
    auto p = $self->getGraphics(parent);
    if (!p)
      throw std::invalid_argument("add_child_volume: no parent node named '" + parent + "'");
    auto child = p->addGraphicsChild<cvc::gl::VolumeNode>(name);
    child->setData(v);
    $self->registerGraphics(name, child);
    return child;
  }
  // A cvc::volren software-raycast volume node as a child of `parent` / at the
  // root. addGraphicsChild<T> is a template (unwrappable), so this clones the
  // add_child_geometry factory. Fill it in Python: n = sg.add_volren("vol");
  // n.addVolume(vol, vs); ... n.tick().
  std::shared_ptr<cvc::gl::VolRenNode> add_child_volren(const std::string& parent,
                                                        const std::string& name) {
    auto p = $self->getGraphics(parent);
    if (!p)
      throw std::invalid_argument("add_child_volren: no parent node named '" + parent + "'");
    auto child = p->addGraphicsChild<cvc::gl::VolRenNode>(name);
    $self->registerGraphics(name, child);
    return child;
  }
  std::shared_ptr<cvc::gl::VolRenNode> add_volren(const std::string& name) {
    auto child = $self->getGraphicsRoot()->addGraphicsChild<cvc::gl::VolRenNode>(name);
    $self->registerGraphics(name, child);
    return child;
  }
  // Typed downcast (like geometry_node/volume_node): the concrete VolRenNode so
  // its addVolume/tick/real-units methods are visible. Null if absent/wrong type.
  std::shared_ptr<cvc::gl::VolRenNode> volren_node(const std::string& name) {
    return std::dynamic_pointer_cast<cvc::gl::VolRenNode>($self->getGraphics(name));
  }
  // Typed downcast for a light added via addLight(name): the concrete LightNode
  // so its kind/target/color/intensity setters are visible. Null if absent/wrong.
  std::shared_ptr<cvc::gl::LightNode> light_node(const std::string& name) {
    return std::dynamic_pointer_cast<cvc::gl::LightNode>($self->getGraphics(name));
  }
  // A cvc::volslice view-aligned slice-renderer node as a child of `parent` / at
  // the root. addGraphicsChild<T> is a template (unwrappable), so this clones the
  // add_volren factory. Fill it in Python: n = sg.add_volslice("slice");
  // n.setVolume(vol); n.setConfig(rs); ... (n.tick() needs a live GL context).
  std::shared_ptr<cvc::gl::VolSliceNode> add_child_volslice(const std::string& parent,
                                                            const std::string& name) {
    auto p = $self->getGraphics(parent);
    if (!p)
      throw std::invalid_argument("add_child_volslice: no parent node named '" + parent + "'");
    auto child = p->addGraphicsChild<cvc::gl::VolSliceNode>(name);
    $self->registerGraphics(name, child);
    return child;
  }
  std::shared_ptr<cvc::gl::VolSliceNode> add_volslice(const std::string& name) {
    auto child = $self->getGraphicsRoot()->addGraphicsChild<cvc::gl::VolSliceNode>(name);
    $self->registerGraphics(name, child);
    return child;
  }
  // Typed downcast (like volren_node): the concrete VolSliceNode so its
  // setVolume/config/tick methods are visible. Null if absent/wrong type.
  std::shared_ptr<cvc::gl::VolSliceNode> volslice_node(const std::string& name) {
    return std::dynamic_pointer_cast<cvc::gl::VolSliceNode>($self->getGraphics(name));
  }
  // Connect a Python callable to the scene's graphics-changed signal (fires when
  // a node is added or removed) — Python functions as scene callbacks.
  void on_graphics_changed(std::function<void()> cb) { $self->graphicsChanged.connect(cb); }
}
%include "cvc/gl/SceneGraph.h"

// ── Standalone render helpers + Python vtkProp bridge (free functions) ──────
// Wrapped as module-level pycvc_gl.render_png(sg,...) / show / add_prop / prop,
// plus the pycvc_gl.scene_renderer class.
// Declared after SceneGraph so its SceneGraph& params resolve to the wrapped type.

%include "pycvc_scene.h"

// ── SceneRenderer: a render target that stays open across frames ────────────
// Lives in cvcGL (inc/cvc/gl/SceneRenderer.h), not here, so volrover3 and any
// other C++ consumer gets it too and Python is only a wrapper over it.
//
// frameRGB() returns raw framebuffer pixels, not text. The default
// std::vector<unsigned char> wrapper would hand back a list of ints (slow and
// enormous), so map it to bytes — the form an encoder actually wants. Must
// precede the %include.
%typemap(out) std::vector<unsigned char> cvc::gl::SceneRenderer::frameRGB {
  $result = PyBytes_FromStringAndSize(reinterpret_cast<const char *>($1.data()),
                                      static_cast<Py_ssize_t>($1.size()));
}
// pickWorld has a double[3] OUT param SWIG can't express; re-exposed as pick_world
// below (returns an (x,y,z) tuple or None).
%ignore cvc::gl::SceneRenderer::pickWorld;
%include "cvc/gl/SceneRenderer.h"

%extend cvc::gl::SceneRenderer {
  // Cast a pick ray through a display pixel (VTK display coords: pixels from the
  // LOWER-left, matching frameRGB()'s row order) and return the first world-space
  // hit as an (x, y, z) tuple, or None on a miss. Pass the tuple to a node's
  // world_to_local / a world_units.world_point_to_real for a real-world readout.
  PyObject* pick_world(double display_x, double display_y) {
    double w[3];
    if (!$self->pickWorld(display_x, display_y, w))
      Py_RETURN_NONE;
    return Py_BuildValue("(ddd)", w[0], w[1], w[2]);
  }
}

// ── CameraController: built-in orbit + Quake-fly navigation, fully cvc::state ──
// Python constructs it from a wrapped SceneRenderer — CameraController(view) — so
// no raw vtk handles cross the boundary; the (app, path) ctor covers headless use.
// Being a state_object, every setting is ALSO reachable via
// pycvc.state_set(app, "<scene prefix>.viewers.<name>.camera.<key>", ...) — the
// wrapper is a convenience over the same reactive state. No singleton: the app is
// taken from the injected viewer/scene (view.scene().appContext()).
//
// The raw vtk-pointer wiring (setCamera/setRenderer/setRenderWindow/attach) is done
// by the SceneRenderer ctor, so it is hidden from Python (avoids marshalling live
// vtk handles). getPose/getUpAxis use C-array out-params SWIG can't express well;
// read the pose/up from state ("pose.eye.*", "up.*") instead.
%ignore cvc::gl::CameraController::setCamera;
%ignore cvc::gl::CameraController::setRenderer;
%ignore cvc::gl::CameraController::setRenderWindow;
%ignore cvc::gl::CameraController::attach;
%ignore cvc::gl::CameraController::setScene; // raw SceneGraph* — the viewer ctor sets it
%ignore cvc::gl::CameraController::getPose;
%ignore cvc::gl::CameraController::getUpAxis;
// Keep the injected viewer/app alive: ~state_object touches that app's state tree.
%pythonappend cvc::gl::CameraController::CameraController %{
    if args: self._pycvc_keepalive = args[0]
%}
%include "cvc/gl/CameraController.h"

// ── Live-scene bridge: adopt an embedding host's SceneGraph ─────────────────
// An embedding host (e.g. volrover3) hands its LIVE scene across as a PyCapsule
// named "cvc.scenegraph" holding a heap shared_ptr<SceneGraph> COPY. This adopts
// it and returns the REAL SceneGraph (co-owned) so add/getGraphics/setPosition
// mutate the RUNNING scene and appear in the host's window. The handle crosses as
// a raw shared_ptr through the capsule — NOT through SWIG's cross-module type
// table — so this needs no SWIG type sharing with the host and no SWIG-runtime
// coupling (mirrors pycvc.app_from_capsule). `app_cap` is accepted for symmetry
// with the host's two-capsule delivery and validated when present; the scene
// already carries its own app (cvc::app& m_ctx), so it isn't needed to adopt.
%inline %{
namespace pycvc {
std::shared_ptr<cvc::gl::SceneGraph> scene_from_capsule(PyObject *app_cap, PyObject *scene_cap) {
  if (!scene_cap || !PyCapsule_CheckExact(scene_cap))
    throw std::invalid_argument("pycvc_gl.scene_from_capsule: scene arg is not a PyCapsule");
  void *sp = PyCapsule_GetPointer(scene_cap, "cvc.scenegraph");
  if (!sp)
    throw std::invalid_argument(
        "pycvc_gl.scene_from_capsule: scene capsule is not named \"cvc.scenegraph\"");
  if (app_cap && app_cap != Py_None) {
    if (!PyCapsule_CheckExact(app_cap) || !PyCapsule_GetPointer(app_cap, "cvc.app")) {
      PyErr_Clear();
      throw std::invalid_argument(
          "pycvc_gl.scene_from_capsule: app arg is not a \"cvc.app\" PyCapsule");
    }
  }
  return *static_cast<std::shared_ptr<cvc::gl::SceneGraph> *>(sp);
}
} // namespace pycvc
%}

// Round-trip proof of the bridge, independent of the scene graph: hand a Python
// vtkProp in and get the same object back out — exercises both typemaps.
%inline %{
static vtkProp* identity_prop(vtkProp* p) { return p; }
%}

// Director proof, independent of the scene graph: invoke a node's public virtual
// update() FROM C++ through a base-class handle. If Python subclassed the node
// and overrode update(), C++ dispatches to the Python override (cross-language
// polymorphism) — the definition of a working director.
%inline %{
namespace pycvc {
void poke_update(const std::shared_ptr<cvc::gl::GraphicsNode> &node) {
  if (node)
    node->update();
}
} // namespace pycvc
%}
