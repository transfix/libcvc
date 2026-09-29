// AriRuntime — run a full Ariadne (.ari) application from a host program with ONE object.
//
// It bundles the wiring src/cvcGL/examples/ariadne_hello.cpp does by hand: a cvc::ariadne::Runtime
// over a SceneGraph's state prefix, the cvc::gl::ariadne::ImGuiBackend that renders the widget tree
// as ImGui over VTK, the .ari loader, scene realization, and the per-frame drain/render/tick loop.
// The host supplies the window (SceneRenderer), camera (CameraController) and overlay
// (ImGuiOverlay) and drives the frame loop; AriRuntime owns the Runtime/backend/realized-scene and
// the sequencing.
//
// This class is libpython-free (it takes std::function, never PyObject*) so native demos, the
// static wasm build, and the pycvc_gl binding all use it. Input is VTK's (widget on_click/on_hover
// + camera nav arrive through the VTK interactor -> ImGui, zero plumbing); document-level
// on_key/on_pointer residents need a producer fed via post_key/post_pointer (a follow-up seam),
// never SDL.
//
// The program lanes (init:/on_tick/on_key/on_pointer, register_verb*, the scheduler pump) are
// no-ops without CVC_STATE_EXEC; loading a YAML .ari needs the yaml build.
// have_state_exec()/have_yaml() report which are live.
#pragma once

#include <cstdint>
#include <cvc/ariadne/ariadne.h>          // cvc::ariadne::Runtime
#include <cvc/gl/ariadne/ImGuiBackend.h>  // cvc::gl::ImGuiBackend
#include <cvc/gl/ariadne/scene_realize.h> // RealizedScene
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace gl {
class SceneRenderer;
class CameraController;
class ImGuiOverlay;
namespace ariadne {

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

  // A search dir for cvc:// component imports; forwarded to register_cvc_uri_handler on load().
  void add_component_path(const std::string &dir);

  // Bind a host action `event` (a widget `event: quit` / menu_action) to a C++ handler, run on the
  // host thread inside frame()'s drain(), never mid-walk.
  void on(const std::string &event, std::function<void()> handler);

  // Register a host verb the .ari program can call. `register_verb` is SYNCHRONOUS (fn runs inline
  // on the host thread inside drain(), no args/return marshalling — a pure side-effecting hook).
  // `register_async_verb` OFFLOADS `work` to the app compute pool and posts its string result to
  // `done_channel`; the .ari program parks on (msg-recv "done_channel") and resumes under drain()'s
  // slice budget — the nav_compute pattern. Both are no-ops without state_exec.
  void register_verb(const std::string &name, std::function<void()> fn);
  void register_async_verb(const std::string &name, const std::string &done_channel,
                           std::function<std::string()> work);

  // --- document ---

  // Load + mount a .ari document (widgets and/or a scene), running its init:/on_* scripts and
  // realizing its scene under the state prefix. Returns non-fatal warnings; THROWS
  // std::runtime_error(lr.error) on a fatal load (min_libcvc gate, parse error, no yaml support).
  // A widgets-only document skips the scene entirely.
  std::vector<std::string> load(const std::string &path);

  // Mount a programmatically-built widget tree instead of loading a file (the P0 path). Applied at
  // the next frame boundary.
  void set_root(cvc::ariadne::Widget root);

  // --- per-frame ---

  // Host work for one frame (NO draw): pump VTK input, advance the camera, drain queued Ariadne
  // actions + the scheduler slice, mirror bound scene visibility, service volume nodes. Call once
  // per frame, BEFORE render().
  void frame(double dt_seconds);
  // The draw: view.render(), which fires the widget-tree walk (rt.render()) through the overlay
  // callback. NEVER call rt.render() directly.
  void render();
  // Convenience: frame(dt) then render().
  void step(double dt_seconds);
  // view.windowClosed() OR a host `quit` handler set the flag.
  bool should_close() const;
  // Optional blocking host loop at ~fps until should_close(); yields the GIL between frames when
  // the caller is Python (the pycvc_gl typemap wraps this).
  void run(double fps = 120.0);

  // A `quit` event marks should_close(); a convenience for the common menu action.
  void request_close();

  // --- input seam (document-level on_key/on_pointer residents; VTK-sourced, never SDL) ---
  void post_key(bool down, const std::string &key, int mods = 0, bool repeat = false);
  void post_pointer(int kind, double x, double y, double dx, double dy, int button, int clicks);

  // --- diagnostics / capabilities ---
  std::vector<std::string> take_warnings(); // reactive read-lane diagnostics from render()
  bool reload_if_changed();                 // §12.5 hot reload if a watched source changed
  static bool have_state_exec();            // program lanes + verbs live?
  static bool have_yaml();                  // can load a YAML .ari?

private:
  void ensure_intrinsics(); // lazily install the register_action_intrinsics provider

  SceneRenderer *view_;
  CameraController *cam_;
  ImGuiOverlay *overlay_;
  cvc::app &app_;
  cvc::gl::ImGuiBackend
      backend_; // DECLARED BEFORE rt_ => destroyed AFTER rt_ (backend outlives Runtime)
  cvc::ariadne::Runtime rt_;
  RealizedScene realized_; // owns StageLighting rigs; must outlive the render loop
  std::vector<std::string> componentPaths_;
  std::vector<std::string> sources_;
  std::map<std::string, std::int64_t> stamps_;
  bool quit_ = false;
  bool haveScene_ = false;
  bool intrinsicsRegistered_ = false;
  // Verbs registered before the intrinsics provider is installed (installed once, lazily).
  std::vector<std::pair<std::string, std::function<void()>>> syncVerbs_;
  std::vector<std::tuple<std::string, std::string, std::function<std::string()>>> asyncVerbs_;
};

} // namespace ariadne
} // namespace gl
} // namespace cvc
