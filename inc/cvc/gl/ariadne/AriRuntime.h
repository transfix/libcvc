// AriRuntime — run a full Ariadne (.ari) application over cvcGL from a host program with
// ONE object. It is the thin GL composer over the backend-neutral
// cvc::ariadne::AppRuntime: an ImGui-over-VTK backend (cvc::gl::ariadne::ImGuiBackend), a
// cvcGL SceneAdapter (realize_scene / tick_scene over the SceneRenderer's SceneGraph), and
// an AppRuntime that owns all the neutral bundling (loader, init:/on_* lanes, host verbs,
// the per-frame drain + scene-service loop). The GL-driving bits — VTK input pump, camera
// integration, and the draw — stay here; everything else lives in AppRuntime.
//
// The public API is unchanged from the pre-split class, so pycvc_gl, demos, and the CI
// render test consume it exactly as before. libpython-free (std::function, never
// PyObject*). Input is VTK's (widget on_click/on_hover + camera nav arrive through the VTK
// interactor -> ImGui); document-level on_key/on_pointer residents need a producer fed via
// post_key/post_pointer (a follow-up seam), never SDL. Loading a YAML .ari needs the yaml
// build.
#pragma once

#include <cvc/ariadne/app_runtime.h>     // cvc::ariadne::AppRuntime, SceneAdapter
#include <cvc/gl/ariadne/ImGuiBackend.h> // cvc::gl::ImGuiBackend
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace gl {
class SceneRenderer;
class CameraController;
class ImGuiOverlay;
namespace ariadne {

struct RealizedScene; // scene_realize.h

class AriRuntime {
public:
  // App + state prefix are taken from the view's scene, so every layer (Runtime, run_init,
  // realize_scene) binds against the SAME prefix — a widget `bind:` and a scene `visible:` on one
  // path can never desync. The overlay must be VISIBLE for widgets to render (ImGuiOverlay hides
  // its draw when !visible); this ctor leaves the caller's setVisible() choice alone but the class
  // documents the requirement. `view`, `cam`, `ui` must outlive the AriRuntime.
  AriRuntime(SceneRenderer &view, CameraController &cam, ImGuiOverlay &ui);
  ~AriRuntime();
  AriRuntime(const AriRuntime &) = delete;
  AriRuntime &operator=(const AriRuntime &) = delete;

  // --- setup (call BEFORE load()) ---
  void add_component_path(const std::string &dir);
  void on(const std::string &event, std::function<void()> handler);
  void register_verb(const std::string &name, std::function<void()> fn);
  // An async function: (name) parks the program while `work` runs on the app's background task
  // pool and returns its string (AppRuntime::register_async_verb). With a done_channel the
  // result is also posted there, for documents that wait with msg-recv.
  void register_async_verb(const std::string &name, std::function<std::string()> work);
  void register_async_verb(const std::string &name, const std::string &done_channel,
                           std::function<std::string()> work);

  // --- document ---
  std::vector<std::string> load(const std::string &path);
  void set_root(cvc::ariadne::Widget root);

  // --- per-frame ---
  // Host work for one frame (NO draw): pump VTK input, advance the camera, then AppRuntime::drain()
  // (queued actions + scheduler slice + scene visibility/tick). Call once per frame, BEFORE
  // render().
  void frame(double dt_seconds);
  // The draw: view.render(), which fires the widget-tree walk (rt.render()) through the overlay
  // callback. NEVER call the Runtime's render() directly.
  void render();
  void step(double dt_seconds); // frame(dt) then render()
  bool should_close() const;    // a host `quit` OR the window closed
  void run(double fps = 120.0); // optional blocking host loop until should_close()
  void request_close();

  // --- input seam (document-level on_key/on_pointer residents; VTK-sourced, never SDL) ---
  void post_key(bool down, const std::string &key, int mods = 0, bool repeat = false);
  void post_pointer(int kind, double x, double y, double dx, double dy, int button, int clicks);
  void post_input(const cvc::ariadne::InputEvent &e);
  // Route a signal to the document's on_signal: (AppRuntime::post_signal). False = it declares
  // none, so the host applies the default.
  bool post_signal(const std::string &signal, const std::string &source = "host");
  // A host step frame() runs after the VTK input pump and before the camera update and the drain:
  // where a host feeds its own input (touch gestures, SDL events, trapped OS signals) so the
  // document sees it this same frame.
  void set_input_pump(std::function<void()> pump);
  // Advance the scene clock by frame()'s dt rather than by wall time, so captured frames are
  // reproducible. Off by default.
  void set_fixed_clock(bool fixed);

  // --- the parts a host composes with ---
  cvc::ariadne::AppRuntime &app_runtime(); // e.g. set_document_args before load()
  cvc::gl::ImGuiBackend &backend();        // e.g. register_widget for a custom ImGui widget
  // The last load()'s realized scene (its lighting rigs, ...), or null when it had none.
  const RealizedScene *realized() const;

  // --- diagnostics / capabilities ---
  std::vector<std::string> take_warnings();
  bool reload_if_changed();
  static bool have_yaml();

private:
  SceneRenderer *view_;
  CameraController *cam_;
  ImGuiOverlay *overlay_;
  cvc::app &app_;
  cvc::gl::ImGuiBackend backend_; // DECLARED BEFORE app_rt_ => the Runtime it points at dies first
  std::unique_ptr<cvc::ariadne::SceneAdapter> scene_; // the GL scene seam; before app_rt_
  cvc::ariadne::AppRuntime app_rt_; // owns the neutral bundling; holds scene_.get()
  std::function<void()> inputPump_;
  bool fixedClock_ = false;
};

} // namespace ariadne
} // namespace gl
} // namespace cvc
