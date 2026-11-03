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
//             width: 320
//             height: 240
//
// Each view is a mirror viewport of the main one (ViewportManager::addMirrorViewport): it draws
// the main renderer's props, the same geometry, from its own camera, less the props of the tagged
// nodes it leaves out (Viewport::setMirrorFilter). The widget reserves its rect in the window,
// cuts it out of the window's background (ImGuiBackend::cut_window_background) and puts the
// viewport there, so the view sits where the widget is, scrolls and moves with its window, and is
// hidden on any frame the widget is not drawn (a closed or collapsed window, another tab).
// Dragging in it orbits its camera (the middle or right button pans) and the wheel zooms.
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
namespace ariadne {

struct RealizedScene; // scene_realize.h

class SceneViews {
public:
  // `realized` is the document's realized scene now (null without one): its tagged nodes. `view`
  // and `backend` must outlive this.
  SceneViews(SceneRenderer &view, ImGuiBackend &backend,
             std::function<const RealizedScene *()> realized);
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

  // Remove every view (a new document).
  void clear();

  // The views that exist, by name, for tests and hosts.
  std::size_t size() const { return views_.size(); }
  bool has_view(const std::string &name) const { return views_.count(name) != 0; }

private:
  struct View;
  View *view_for(const std::string &name);

  SceneRenderer *view_;
  ImGuiBackend *backend_;
  std::function<const RealizedScene *()> realized_;
  std::map<std::string, std::unique_ptr<View>> views_;
  bool walked_ = false; // end_walk() ran since the last update()
};

} // namespace ariadne
} // namespace gl
} // namespace cvc
