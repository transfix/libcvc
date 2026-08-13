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

%{
#include <cvc/core/exception.h> // the %import'd %exception block catches cvc::exception
#include <any>
#include <functional>
#include <stdexcept>
#include <cvc/gl/SceneNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
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
%shared_ptr(SceneNode)
%shared_ptr(GraphicsNode)
%shared_ptr(GeometryNode)
%shared_ptr(VolumeNode)
%shared_ptr(SceneGraph)

// ── directors: Python-defined scene node types ──────────────────────────────
// With directors on the node classes, Python can SUBCLASS a scene node and have
// C++ call the Python overrides — e.g. override getProp() to return a
// vtkmodules-built vtkProp (marshalled by the directorout typemap above), so a
// pure-Python node renders in the C++ scene. dirprot (module flag) exposes the
// protected virtuals (getProp / handleStateChanged / applyTransformToVTK) so
// they are overridable. The concrete leaves carry C++ impls of the pure virtuals
// (getProp / getBoundingBox), so a Python subclass need only override what it
// wants to customize.
%feature("director") GraphicsNode;
%feature("director") GeometryNode;
%feature("director") VolumeNode;

// A Python-CONSTRUCTED node (a director subclass built as MyNode(app, path,
// name)) must keep its app alive too — its ~SceneNode touches the app's state
// tree by raw reference. args[0] is the app the node ctor takes. (Nodes obtained
// from a SceneGraph get this via the SceneGraph appends below instead.)
%pythonappend GraphicsNode::GraphicsNode %{
    if args: self._pycvc_app = args[0]
%}
%pythonappend GeometryNode::GeometryNode %{
    if args: self._pycvc_app = args[0]
%}
%pythonappend VolumeNode::VolumeNode %{
    if args: self._pycvc_app = args[0]
%}

// ── SceneNode (abstract base): trim VTK / threading internals ───────────────
%ignore SceneNode::addToRenderer;
%ignore SceneNode::removeFromRenderer;
%ignore SceneNode::runOnMainThread;
%ignore SceneNode::setSceneGraph;
%ignore SceneNode::getSceneGraph;
%include "cvc/gl/SceneNode.h"

// ── GraphicsNode: keep transform / material / label; ignore VTK/any/templates ─
%ignore GraphicsNode::setTransform(vtkMatrix4x4 *);
%ignore GraphicsNode::setTransform(const double[16]); // replaced by the vector<double> %extend
%ignore GraphicsNode::getTransform;
%ignore GraphicsNode::getWorldTransform;
%ignore GraphicsNode::getClipPlanes;
%ignore GraphicsNode::setMetadata;
%ignore GraphicsNode::getMetadata;
%ignore GraphicsNode::hasMetadata;
%ignore GraphicsNode::getAllMetadata;
%ignore GraphicsNode::getGraphicsChildren;      // vector<shared_ptr<...>> return
// bounding_box (generic_bounding_box<double>) is opaque here; its greedy
// templated converting ctor mis-binds SWIG's SwigValueWrapper on a by-value
// return, so every bounding_box-returning method is ignored (as in pycvc.i).
%ignore GraphicsNode::getBoundingBox;
%ignore GraphicsNode::getCombinedBoundingBox;
%ignore GraphicsNode::getBBoxColor;             // out-ref params
%ignore GraphicsNode::getLabelColor;
%ignore GraphicsNode::getExtentLabelColor;
%ignore GraphicsNode::addToRenderer;
%ignore GraphicsNode::removeFromRenderer;
%ignore GraphicsNode::addGraphicsChild;         // templates + shared_ptr overload
%ignore GraphicsNode::createChild;
%ignore GraphicsNode::transformChanged;         // public boost::signals2::signal member
%extend GraphicsNode {
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
%ignore GeometryNode::getBoundingBox;
%extend GeometryNode {
  // Zero-copy texture (default): the vtkTexture aliases img's RGBA8 buffer, so a
  // later img.numpy() pixel edit + texture_modified() shows live with no re-copy.
  void set_texture(const cvc::image& img) { $self->setTexture(img, /*zeroCopy=*/true); }
  // Convert-flip-and-copy fallback (any format; the texture owns its own copy).
  void set_texture_copy(const cvc::image& img) { $self->setTexture(img, /*zeroCopy=*/false); }
  void clear_texture() { $self->clearTexture(); }
}
%include "cvc/gl/GeometryNode.h"

// ── VolumeNode: transfer function (vector<double>) + rendering props ────────
%ignore VolumeNode::addToRenderer;
%ignore VolumeNode::getBoundingBox;
%include "cvc/gl/VolumeNode.h"

// ── SceneGraph: the top-level graph. App injected explicitly (no singleton). ─
%ignore SceneGraph::SceneGraph(const std::string &);           // process-wide singleton ctor
%ignore SceneGraph::SceneGraph(cvc::app &, const std::string &); // re-exposed via shared_ptr factory
%ignore SceneGraph::setRenderer;
// SceneGraph::postEvent(std::function<void()>) is NOT ignored — the callable
// typemap above marshals a Python function to the std::function, so Python can
// post work onto the scene's owner thread.
%ignore SceneGraph::getGridNode;
%ignore SceneGraph::getAllGraphics;
%ignore SceneGraph::getAllGraphicsOfType;
%ignore SceneGraph::getAllVolumeGraphics;
%ignore SceneGraph::getAllGeometryGraphics;
%ignore SceneGraph::updateTransferFunction;
%ignore SceneGraph::updateGrid;
%ignore SceneGraph::computeGraphicsBounds;
%ignore SceneGraph::computeVolumeBounds;
%ignore SceneGraph::graphicsChanged; // public boost::signals2::signal member
// Keep the app alive for at least as long as the SceneGraph proxy: SceneGraph
// holds the app by RAW reference (cvc::app& m_ctx), so without this the app
// could be torn down first (at GC / interpreter shutdown) and ~SceneGraph would
// lock a destroyed state mutex (boost::lock_error). Stash it on the proxy — the
// same keep-alive pycvc.i uses for volume/geometry. This %feature MUST precede
// the %extend ctor it targets. The generated __init__ is `def __init__(self,
// *args)` and the sole ctor is the injected-app factory, so the app is args[0].
// (In the adopt case the host owns app+scene for the whole session, so the raw
// scene_from_capsule proxy needs no stash; grl_snam_lab.Lab holds the app too.)
%pythonappend SceneGraph::SceneGraph %{
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
    return n
%}
%pythonappend SceneGraph::getGraphics %{
    if val is not None:
        _t = _typed_node(self, name)
        if _t is not None: val = _t
        val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend SceneGraph::getGraphicsRoot %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend SceneGraph::addGraphics %{
    if val is not None:
        _t = _typed_node(self, args[0])
        if _t is not None: val = _t
        val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend SceneGraph::geometry_node %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend SceneGraph::volume_node %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%extend SceneGraph {
  // Standalone factory: build a scene under an EXPLICIT injected app (no
  // singleton), mirroring pycvc.volume(app). `app` must outlive the scene.
  SceneGraph(std::shared_ptr<cvc::app> app, const std::string& prefix = "cvcgl") {
    if (!app)
      throw std::invalid_argument("pycvc_gl.SceneGraph: null app handle");
    return new SceneGraph(*app, prefix);
  }
  // Node count (getAllGraphics() is ignored for the Python surface but callable
  // here in C++).
  std::size_t num_graphics() const { return $self->getAllGraphics().size(); }
  // Typed accessors: return the CONCRETE node so its type-specific setters
  // (GeometryNode::setColor, VolumeNode::setTransferFunction) are visible from
  // Python — getGraphics() alone yields the GraphicsNode base. Null if `name` is
  // absent or not of that type.
  std::shared_ptr<GeometryNode> geometry_node(const std::string& name) {
    return std::dynamic_pointer_cast<GeometryNode>($self->getGraphics(name));
  }
  std::shared_ptr<VolumeNode> volume_node(const std::string& name) {
    return std::dynamic_pointer_cast<VolumeNode>($self->getGraphics(name));
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
  // Add a geometry / volume as a CHILD of `parent` (inherits its transform), and
  // register it so getGraphics(name) finds it.
  std::shared_ptr<GeometryNode> add_child_geometry(const std::string& parent,
                                                   const std::string& name,
                                                   const cvc::geometry& g) {
    auto p = $self->getGraphics(parent);
    if (!p)
      throw std::invalid_argument("add_child_geometry: no parent node named '" + parent + "'");
    auto child = p->addGraphicsChild<GeometryNode>(name);
    child->setGeometry(g);
    $self->registerGraphics(name, child);
    return child;
  }
  std::shared_ptr<VolumeNode> add_child_volume(const std::string& parent, const std::string& name,
                                               const cvc::volume& v) {
    auto p = $self->getGraphics(parent);
    if (!p)
      throw std::invalid_argument("add_child_volume: no parent node named '" + parent + "'");
    auto child = p->addGraphicsChild<VolumeNode>(name);
    child->setData(v);
    $self->registerGraphics(name, child);
    return child;
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
%typemap(out) std::vector<unsigned char> SceneRenderer::frameRGB {
  $result = PyBytes_FromStringAndSize(reinterpret_cast<const char *>($1.data()),
                                      static_cast<Py_ssize_t>($1.size()));
}
%include "cvc/gl/SceneRenderer.h"

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
std::shared_ptr<SceneGraph> scene_from_capsule(PyObject *app_cap, PyObject *scene_cap) {
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
  return *static_cast<std::shared_ptr<SceneGraph> *>(sp);
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
void poke_update(const std::shared_ptr<GraphicsNode> &node) {
  if (node)
    node->update();
}
} // namespace pycvc
%}
