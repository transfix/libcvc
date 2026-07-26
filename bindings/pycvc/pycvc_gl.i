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
#include <stdexcept>
#include <cvc/gl/SceneNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/SceneGraph.h>
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
%include <std_shared_ptr.i>
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
%extend GraphicsNode {
  // Row-major 4x4 transform from a 16-element list (the vtkMatrix4x4 overload is
  // ignored; this is the Python-friendly path). Full rotate/scale/translate.
  void setTransform(const std::vector<double>& m) {
    if (m.size() != 16)
      throw std::invalid_argument("setTransform: need 16 doubles (row-major 4x4)");
    $self->setTransform(m.data());
  }
}
%include "cvc/gl/GraphicsNode.h"

// ── GeometryNode: setGeometry (in-place data), material + render-mode setters ─
// enum class + scalar setters + cvc::geometry (%import'd) marshal cleanly; only
// the opaque-by-value bbox override needs ignoring.
%ignore GeometryNode::getBoundingBox;
%include "cvc/gl/GeometryNode.h"

// ── VolumeNode: transfer function (vector<double>) + rendering props ────────
%ignore VolumeNode::addToRenderer;
%ignore VolumeNode::getBoundingBox;
%include "cvc/gl/VolumeNode.h"

// ── SceneGraph: the top-level graph. App injected explicitly (no singleton). ─
%ignore SceneGraph::SceneGraph(const std::string &);           // process-wide singleton ctor
%ignore SceneGraph::SceneGraph(cvc::app &, const std::string &); // re-exposed via shared_ptr factory
%ignore SceneGraph::setRenderer;
%ignore SceneGraph::postEvent;
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
%pythonappend SceneGraph::getGraphics %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend SceneGraph::getGraphicsRoot %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
%}
%pythonappend SceneGraph::addGraphics %{
    if val is not None: val._pycvc_app = getattr(self, "_pycvc_app", None)
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
}
%include "cvc/gl/SceneGraph.h"

// ── Standalone render helpers + Python vtkProp bridge (free functions) ──────
// Wrapped as module-level pycvc_gl.render_png(sg,...) / show / add_prop / prop.
// Declared after SceneGraph so its SceneGraph& params resolve to the wrapped type.
%include "pycvc_scene.h"

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
