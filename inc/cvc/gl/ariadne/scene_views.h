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
// An override entry may also move, turn or scale what it covers (`transform:`, in the frame of the
// node it names) and start over from the node's own look (`mode: replace`). A volslice node, whose
// slices face the main camera, draws slices swept for a view's own camera (VolSliceNode::viewProp);
// a volren node, raycast for the main view and composited into its pixels, is in no view. A
// `type: volume` node takes an opacity override.
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
// SceneGraph::makeShadowReceiverPass), bakes its own (`shadows: own`), or none (`shadows: false`).
//
// `scene: <id>` shows one of the document's `scenes:` instead of its `scene:`: the view mirrors
// that scene's own viewport (SceneSlot::source), so it draws that scene's props, lights and
// background. A `main_view:` on the document's scene takes the groups it leaves out of the main
// renderer for the length of its draw (main_view_begin / main_view_end), so the views still have
// them.
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
class SceneGraph;
namespace ariadne {

struct RealizedScene; // scene_realize.h

// A scene a viewport can show: the document's main one ("") or a `scenes:` entry, as the host has
// realized it.
struct SceneSlot {
  SceneGraph *sg = nullptr;                // its graph
  const RealizedScene *realized = nullptr; // its nodes and named cameras (null: none realized)
  std::string source;                      // the viewport its views mirror
  bool anchor = false; // `source` is the scene's own, never shown: the views draw a pixel of it
                       // under one of them so its shadow maps are baked
};

class SceneViews {
public:
  // `scenes` finds the scene a viewport names ("" the main one): false when there is none.
  // `main` drives the main view's camera, for a viewport bound to `camera: main` (null: such a
  // viewport keeps a camera of its own). `view`, `backend` and `main` must outlive this.
  SceneViews(SceneRenderer &view, ImGuiBackend &backend,
             std::function<bool(const std::string &scene, SceneSlot &out)> scenes,
             CameraController *main = nullptr);
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
  // The scene a viewport would show for `scene` ("" the main one), as the constructor's lookup
  // finds it. False when there is none.
  bool scene(const std::string &scene, SceneSlot &out) const {
    return scenes_ && scenes_(scene, out);
  }

private:
  struct View;
  struct Camera; // a named camera (scene: cameras:) and its controller
  View *view_for(const std::string &name, const std::string &scene);
  Camera *named_camera(const std::string &scene, const std::string &name);
  void main_view_begin(); // the main renderer's StartEvent: take out what main_view: hides
  void main_view_end();   // ... and its EndEvent: put it back
  void bind_camera(View &v, const std::string &name); // "" its own, "main", or a named camera
  CameraController &controller(View &v);              // what a drag in v moves

  SceneRenderer *view_;
  ImGuiBackend *backend_;
  std::function<bool(const std::string &, SceneSlot &)> scenes_;
  CameraController *main_;
  std::map<std::string, std::unique_ptr<View>> views_;
  std::map<std::string, std::unique_ptr<Camera>> cameras_; // by "<scene>|<name>"
  std::vector<void *> main_hidden_;  // vtkProp*: what main_view: leaves out of the main view
  std::vector<void *> main_removed_; // ... taken out for this render, to put back
  unsigned long main_start_tag_ = 0, main_end_tag_ = 0; // the main renderer's observers
  std::vector<std::string> anchors_on_; // the scenes: viewports drawn (a pixel) this frame
  const void *main_framed_ = nullptr;   // the RealizedScene the main camera was framed for
  bool walked_ = false;                 // end_walk() ran since the last update()
};

} // namespace ariadne
} // namespace gl
} // namespace cvc
