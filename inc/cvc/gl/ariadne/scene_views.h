// scene_views.h — the `viewport` widget: a second view of the document's scene inside one of its
// windows, with its own camera, drawing all of the scene or only the nodes whose `tags:` it shows
// (roadmap §9.5, docs/ARIADNE.md §5.18).
//
//     scene:
//       nodes:
//         - { node: city, type: geometry, ... }                 # untagged: in every view
//         - { node: agents, type: geometry, tags: [sim], ... }
//         - { node: labels, type: geometry, tags: [annotation], ... }
//     windows:
//       - window: Overview
//         children:
//           - type: viewport
//             view: overview          # its camera is <prefix>.viewers.overview.camera
//             show: [sim]             # the untagged nodes and those tagged sim
//             camera: map             # orbit (default) | fly | map | track (with track: NODE)
//             override:               # restyled in this view only, by node id, tag or "*"
//               city: { drawstyle: wireframe, opacity: 0.4 }
//             width: 320
//             height: 240
//
// Each view is a mirror viewport of the main one (ViewportManager::addMirrorViewport): it draws
// the main renderer's props, the same geometry, from its own camera, less the props of the tagged
// nodes it leaves out, and with a twin in place of each actor it restyles: its own vtkActor and
// vtkProperty over the original's mapper, so no geometry is copied (Viewport::setMirrorFilter).
// The widget reserves its rect in the window,
// cuts it out of the window's background (ImGuiBackend::cut_window_background) and puts the
// viewport there, so the view sits where the widget is, scrolls and moves with its window, and is
// hidden on any frame the widget is not drawn (a closed or collapsed window, another tab).
// Dragging in it orbits its camera (the middle or right button pans), the wheel zooms, and a
// double-click puts the camera back where azimuth: / elevation: / zoom: start it.
//
// A view's camera is its own unless it names one: `camera: <id>` draws from a camera the scene
// declares (`scene: cameras:`), so every viewport bound to it moves together, and `camera: main`
// draws from the main view's. A view receives the scene's shadows (the main renderer's maps,
// SceneGraph::makeShadowReceiverPass) unless `shadows: false`.
//
// AriRuntime owns one and registers the widget, so every AriRuntime host (arigl, pycvc_gl) has it.
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

namespace cvc {
namespace gl {
class SceneRenderer;
class ImGuiBackend;
class CameraController;
namespace ariadne {

struct RealizedScene; // scene_realize.h

class SceneViews {
public:
  // `realized` is the document's realized scene now (null without one): its nodes and its named
  // cameras. `main` drives the main view's camera, for a viewport bound to `camera: main` (null:
  // such a viewport keeps a camera of its own). `view`, `backend` and `main` must outlive this.
  SceneViews(SceneRenderer &view, ImGuiBackend &backend,
             std::function<const RealizedScene *()> realized, CameraController *main = nullptr);
  ~SceneViews(); // removes its viewports
  SceneViews(const SceneViews &) = delete;
  SceneViews &operator=(const SceneViews &) = delete;

  // Register `type: viewport` with the backend.
  void register_widget();

  // After the frame's widget walk (inside the render, before the renderers draw): hide each view
  // no widget drew this frame.
  void end_walk();

  // Once per frame before the render: work out what each view leaves out of the scene, frame a new
  // view's camera on what it shows, and advance the views' cameras by dt. With no widget walk
  // since the last call (the host hid the UI), every view is hidden.
  void update(double dt);

  // Remove every view and named camera (a new document).
  void clear();

  // The views that exist, by name, for tests and hosts.
  std::size_t size() const { return views_.size(); }
  bool has_view(const std::string &name) const { return views_.count(name) != 0; }

private:
  struct View;
  struct Camera; // a named camera (scene: cameras:) and its controller
  View *view_for(const std::string &name);
  Camera *named_camera(const std::string &name);
  void bind_camera(View &v, const std::string &name); // "" its own, "main", or a named camera
  CameraController &controller(View &v);              // what a drag in v moves

  SceneRenderer *view_;
  ImGuiBackend *backend_;
  std::function<const RealizedScene *()> realized_;
  CameraController *main_;
  std::map<std::string, std::unique_ptr<View>> views_;
  std::map<std::string, std::unique_ptr<Camera>> cameras_;
  bool walked_ = false; // end_walk() ran since the last update()
};

} // namespace ariadne
} // namespace gl
} // namespace cvc
