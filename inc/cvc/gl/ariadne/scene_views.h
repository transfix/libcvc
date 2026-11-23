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
//                                     # | slice (with axis: x | y | z or { bind: KEY })
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
// The view draws into a texture of its own, the widget's size (Viewport::setTextureSize), and the
// widget draws that texture as an image in its window, so the window stacks, clips and scrolls the
// view as it does any of its content: a window in front covers it, and it covers a window behind.
// It is hidden on any frame the widget is not drawn (a closed or collapsed window, another tab).
// Dragging in it orbits its camera (the middle or right button pans), the wheel zooms, and a
// double-click puts the camera back where azimuth: / elevation: / zoom: start it. In mode: slice
// (CameraController::Mode::Slice) it looks straight down one axis: drags pan, the wheel zooms
// about the pointer, it is framed on the volume_slice nodes on its axis, and it leaves out those
// on the other two; a pick meets the plane of its slice exactly and writes the voxel (i, j, k),
// its value and the slice's axis. `pointer: <key>` writes the same keys (phase "hover") as the
// pointer moves over the view with no button down, so a probe can follow it (in an orbit view each
// move costs a cell pick).
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
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {
struct CustomEdit;  // backend.h: what a custom widget hands back
struct SceneCamera; // scene.h: a camera's mode, start and viewpoint
} // namespace ariadne
namespace gl {
class SceneRenderer;
class ImGuiBackend;
class CameraController;
class SceneGraph;
namespace ariadne {

struct RealizedScene; // scene_realize.h

// Frame `cam` on the bounds `b` (VTK's order: min x, max x, min y, max y, min z, max z) as the
// scene camera `s` says (§5.18, Cameras): its mode and where its orbit starts, or its authored
// viewpoint, with its lens and up axis. What a viewport does with its own camera and a named one,
// and what AriRuntime does with the main view's when the scene's `cameras:` has one named `main`.
void frame_scene_camera(CameraController &cam, const double b[6],
                        const ::cvc::ariadne::SceneCamera &s, SceneGraph &sg);

// A scene a viewport can show: the document's main one ("") or a `scenes:` entry, as the host has
// realized it.
struct SceneSlot {
  SceneGraph *sg = nullptr;                // its graph
  const RealizedScene *realized = nullptr; // its nodes and named cameras (null: none realized)
  std::string source;                      // the viewport its views mirror
  bool anchor = false; // `source` is the scene's own, never shown: while one of its views shows it
                       // draws a pixel into a texture of its own, so its shadow maps are baked
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
  // What a pick in a view hit: a world point on what it draws, and the scene node that drew it.
  struct PickHit {
    bool hit = false;
    double world[3] = {0.0, 0.0, 0.0};
    std::string node; // the realized node's id ("" when the prop is no node's)
    // On a volume_slice node: the voxel the point lies in and its value. In a slice view the
    // pick meets the plane of the slice on the view's axis exactly; `world` is that point even
    // past the volume's edge (then hit and voxel are false), so a drag keeps reporting.
    bool plane = false; // the point lies on a slice's plane
    bool voxel = false;
    long long ijk[3] = {0, 0, 0};
    double value = 0.0;
    int axis = -1; // with plane: the axis the slice looks down (0 x, 1 y, 2 z)
  };
  // Pick in view `view` at (x, y) in VTK display coordinates of the window (framebuffer pixels from
  // the lower left; the view's texture is shown at its region), against what the view drew last
  // frame: what an `input: pick` viewport's left button does, for tests and hosts. False for a
  // miss or no such view.
  bool pick(const std::string &view, double x, double y, PickHit &hit) const;

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
  // A view's frame: what it leaves out and restyles, its shadows, its camera framed once it has
  // something to show and moved by dt. update() runs it for every view; the widget runs it for a
  // view made in this frame's walk, so its first frame shows it.
  void prepare(View &v, double dt);
  // A named camera framed on its whole scene, the first time a shown view draws from it; `whole`
  // is that scene's bounds (false when it has none).
  bool frame_named(Camera &c, double whole[6]);
  // Where the keys of the viewport being drawn (pick:, pointer:, axis: { bind }) resolve, for a
  // view of `sg`: the scope its widget sits in, as its bind: (Backend::custom_scope).
  std::string key_scope(const SceneGraph &sg) const;
  bool pick_at(View &v, double x, double y, PickHit &hit) const;
  // An input: pick view's pick at the pointer, in `phase` (down, drag, up): written under its
  // pick key, and its on_pick raised through `edit`.
  void pick(View &v, const char *phase, ::cvc::ariadne::CustomEdit &edit);
  // A pointer: view's pick at the pointer as it moves with no button down: written under its
  // pointer key with phase "hover" (no on_pick).
  void pointer(View &v);
  // The keys a pick at the pointer writes (x, y, z, hit, node, i, j, k, value, axis, button,
  // mods, phase, view, seq), `seq` counted by `seq`.
  std::vector<std::pair<std::string, std::string>> pick_fields(View &v, const char *phase,
                                                               long &seq) const;

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
