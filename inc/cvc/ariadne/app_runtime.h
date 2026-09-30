// AppRuntime — run a full Ariadne (.ari) application from one object, backend-neutral.
//
// This is the bundling ariadne_hello.cpp does by hand — a cvc::ariadne::Runtime over a
// state prefix, the .ari loader, the init:/on_* program lanes, host verb registration,
// and the per-frame drain/scene-service loop — with ZERO renderer knowledge. The widget
// surface is a Backend (set_backend); the optional 3D scene is a SceneAdapter passed to
// the ctor. Give it a null SceneAdapter and it runs every non-scene lane unchanged, so a
// terminal / no-scene backend reuses all of this; a scene-bearing document then loads
// with a warning and no realized scene.
//
// The GL host cvc::gl::ariadne::AriRuntime is a thin composer over this: an ImGuiBackend
// + a cvcGL SceneAdapter + an AppRuntime. A terminal host would compose an FtxuiBackend
// + no adapter over the same AppRuntime.
//
// libpython-free (std::function, never PyObject*). Loading a YAML .ari needs the yaml
// build; have_yaml() reports whether it is live.
#pragma once

#include <cstdint>
#include <cvc/ariadne/ariadne.h>       // cvc::ariadne::Runtime, Widget, run_init, ...
#include <cvc/ariadne/scene_adapter.h> // cvc::ariadne::SceneAdapter (optional)
#include <functional>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace cvc {
class app;
namespace ariadne {

class Backend;

class AppRuntime {
public:
  // Build a Runtime over `app`'s state at `prefix`. `scene` is the optional 3D-scene
  // adapter (nullptr = no scene: a scene-bearing document warns and drops its scene).
  // `scene`, once given, must outlive this AppRuntime.
  AppRuntime(cvc::app &app, std::string prefix, SceneAdapter *scene = nullptr);
  ~AppRuntime();
  AppRuntime(const AppRuntime &) = delete;
  AppRuntime &operator=(const AppRuntime &) = delete;

  // --- setup (call BEFORE load()) ---

  // Parameterize over the widget backend (forwards to the Runtime). The backend must
  // outlive this AppRuntime. Call before load()/drain().
  void set_backend(Backend &backend);

  // The underlying Runtime — an escape hatch for backend attachment (a backend's own
  // install/wire step) and advanced hosts. Do not call rt.render() directly.
  Runtime &runtime();

  // A search dir for cvc:// component imports; forwarded to register_cvc_uri_handler on load().
  void add_component_path(const std::string &dir);

  // Bind a host action `event` to a C++ handler, run inside drain(), never mid-walk.
  void on(const std::string &event, std::function<void()> handler);

  // Register a host verb the .ari program can call. register_verb is SYNCHRONOUS (fn runs
  // inline on the drain thread, no args/return). register_async_verb OFFLOADS `work` to the
  // app compute pool and posts its string result to `done_channel`; the program parks on
  // (msg-recv "done_channel") and resumes under drain()'s slice budget (the nav_compute
  // pattern).
  void register_verb(const std::string &name, std::function<void()> fn);
  void register_async_verb(const std::string &name, const std::string &done_channel,
                           std::function<std::string()> work);

  // --- document ---

  // Load + mount a .ari document, running its init:/on_* scripts and (via the SceneAdapter,
  // if any) realizing its scene under the state prefix. Returns non-fatal warnings; THROWS
  // std::runtime_error on a fatal load (min_libcvc gate, parse error, no yaml support).
  std::vector<std::string> load(const std::string &path);

  // Mount a programmatically-built widget tree instead of a file. Applied next frame.
  void set_root(Widget root);

  // --- per-frame (NEUTRAL core; no draw, no dt) ---

  // Run queued Ariadne actions on this thread + a scheduler slice, then (if a SceneAdapter
  // is set) mirror bound scene visibility and service the scene. The host does input pump /
  // camera / draw around this. Call once per frame.
  void drain();

  bool should_close() const;
  void request_close();

  // --- input seam (document-level on_key/on_pointer residents) ---
  void post_key(bool down, const std::string &key, int mods = 0, bool repeat = false);
  void post_pointer(int kind, double x, double y, double dx, double dy, int button, int clicks);

  // --- diagnostics / capabilities ---
  std::vector<std::string> take_warnings(); // reactive read-lane diagnostics from the last render
  bool reload_if_changed();                 // §12.5 hot reload if a watched source changed
  static bool have_yaml();                  // can load a YAML .ari?

private:
  void ensure_intrinsics(); // lazily install the register_action_intrinsics provider

  cvc::app &app_;
  std::string prefix_;
  SceneAdapter *scene_; // optional; null = no scene (terminal / no-scene backend)
  Runtime rt_;
  std::vector<std::string> componentPaths_;
  std::vector<std::string> sources_;
  std::map<std::string, std::int64_t> stamps_;
  bool quit_ = false;
  bool intrinsicsRegistered_ = false;
  std::vector<std::pair<std::string, std::function<void()>>> syncVerbs_;
  std::vector<std::tuple<std::string, std::string, std::function<std::string()>>> asyncVerbs_;
};

} // namespace ariadne
} // namespace cvc
