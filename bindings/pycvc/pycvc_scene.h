// pycvc_scene.h — SWIG-safe facade over cvcGL's SceneGraph, so Python can
// build a live 3D scene: add pycvc.Geometry / pycvc.Volume as nodes and
// pump the reactive scene graph. Generic — no domain knowledge.
#pragma once

#include <cstddef>
#include <memory>
#include <string>

class SceneGraph; // cvcGL (global namespace)

namespace pycvc {

class Geometry;
class Volume;

class Scene {
public:
  Scene();
  ~Scene();

  // Add a mesh/polyline or a scalar-field volume as a named node.
  void add_geometry(const std::string &name, const Geometry &g);
  void add_volume(const std::string &name, const Volume &v);

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
  std::shared_ptr<SceneGraph> sg_;
};

} // namespace pycvc
