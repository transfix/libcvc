// pycvc_scene.h — SWIG-safe facade over cvcGL's SceneGraph, so Python can
// build a live 3D scene: add pycvc.geometry / pycvc.volume (the directly-
// wrapped cvc value types) as nodes and pump the reactive scene graph.
// Generic — no domain knowledge.
#pragma once

#include <cstddef>
#include <memory>
#include <string>

class SceneGraph; // cvcGL (global namespace)
class vtkProp;    // VTK (global namespace) — bridged via vtkPythonUtil typemaps

namespace cvc {
class app;
class geometry;
class volume;
} // namespace cvc

namespace pycvc {

class Scene {
public:
  // Bound to an EXPLICIT app (no cvcGL context() singleton): the SceneGraph
  // lives on this app's state tree under `state_prefix`, so the scene shares the
  // host's app/context like everything else in pycvc.
  Scene(const std::shared_ptr<cvc::app> &app, const std::string &state_prefix = "cvcgl");
#ifndef SWIG
  // Adopt an EXISTING SceneGraph (e.g. an embedding host's LIVE scene) instead of
  // constructing a fresh one, so add_geometry/add_volume mutate the RUNNING scene
  // and render in the host's own window. Co-owns both the app and the scene (the
  // host keeps its own refs, so this shared ownership never double-frees). Hidden
  // from SWIG: Python constructs this via pycvc_gl.scene_from_capsule(app_cap,
  // scene_cap), not this ctor (SceneGraph is not a SWIG-wrapped type here).
  Scene(const std::shared_ptr<cvc::app> &app, const std::shared_ptr<SceneGraph> &existing);
#endif
  ~Scene();

  // Add a mesh/polyline or a scalar-field volume as a named node. The
  // arguments are the directly-wrapped cvc types (SWIG %import's them from
  // pycvc.i), so pycvc.geometry / pycvc.volume cross into the scene as-is.
  void add_geometry(const std::string &name, const cvc::geometry &g);
  void add_volume(const std::string &name, const cvc::volume &v);

  // ── VTK Python bridge (Goal 2) ─────────────────────────────────────
  // Add a Python-built VTK prop (e.g. a vtkActor created with vtkmodules) as a
  // named scene node, with an explicit bounding box. The vtkProp* in-typemap
  // (vtkPythonUtil::GetPointerFromObject) unwraps the live Python VTK object
  // into a C++ vtkProp* that cvcGL renders directly — Python graphics flow
  // straight into the C++ scene.
  void add_prop(const std::string &name, vtkProp *prop, double minx = -1.0, double miny = -1.0,
                double minz = -1.0, double maxx = 1.0, double maxy = 1.0, double maxz = 1.0);
  // Return a node's vtkProp back to Python as a live vtkmodules object (via the
  // out-typemap). Null if `name` is not a prop node.
  vtkProp *prop(const std::string &name) const;

  // Re-pumpable event pump: apply queued scene mutations (no Qt loop).
  void pump();

  std::size_t num_graphics() const;
  bool has(const std::string &name) const;

  // ── Display (VTK render window, no Qt) ─────────────────────────────
  // Open an interactive window and block until closed (needs a display).
  void show(const std::string &title = "cvc lab", int width = 1024, int height = 768);
  // Offscreen render of one frame to a PNG (needs a GL context).
  void render_png(const std::string &path, int width = 1024, int height = 768);

private:
  // Co-own the app so it outlives the SceneGraph (which references the app's
  // state tree by raw reference). Declared BEFORE sg_ so sg_ is destroyed first.
  std::shared_ptr<cvc::app> app_;
  std::shared_ptr<SceneGraph> sg_;
};

} // namespace pycvc
