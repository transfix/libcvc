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
#include <cvc/ariadne/net_host.h>      // NetHostConfig
#include <cvc/ariadne/scene_adapter.h> // cvc::ariadne::SceneAdapter (optional)
#include <functional>
#include <map>
#include <memory>
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

  // Menus the host puts in every document it mounts: a gallery's Demos menu, say, whose
  // menu_action events the host handles with on(). Where the list holds document_menus(), the
  // document's own menus go there; otherwise the host's go first, ahead of the document's. A
  // document without a menubar gets one. Applied by the next load() or set_root(); empty (the
  // default) leaves documents as written.
  void set_host_menus(std::vector<Widget> menus);

  // The place among set_host_menus() for the document's own menus. Given a label, it is one menu
  // of that name holding them as submenus, so the menu bar stays the same from one document to the
  // next (the menu is greyed out for a document without menus). Without one, the document's menus
  // stand in the bar as written, and a host menu named like one of them adds its items to it.
  static Widget document_menus(std::string label = std::string());

  // Key bindings of the host's own (keys.h), beside each document's `keys:`: arigl's camera keys.
  // Once a host has called this (even with none), or for a document with keys, the document gets
  // a Keys window (id ariadne_keys, closed until tree.ariadne_keys.open is set -- a host's
  // menu_toggle on that key opens it) listing them all, each with a Change button: click it, then
  // press the new key (Escape keeps the old one). Applied by the next load() or set_root().
  void set_host_keys(std::vector<KeyBinding> keys);

  // Register a host verb the .ari program can call. register_verb is SYNCHRONOUS (fn runs
  // inline on the drain thread, no args/return).
  //
  // register_async_verb makes `name` an async function on the app's async host
  // (docs/STATE_EXEC_EXCEPTIONS.md 3):
  // - (name) parks the calling program while `work` runs on the app's background task pool,
  //   then returns `work`'s string;
  // - an exception `work` throws raises at the call;
  // - (name-async) returns a future for (await f);
  // - the work is charged to the calling action's budget (ariadne.limits.action_max_steps and
  //   action_max_time).
  // With a `done_channel`, the result is also posted there. That keeps documents that wait with
  // (msg-recv "done_channel") working: the old nav_compute pattern.
  void register_verb(const std::string &name, std::function<void()> fn);
  void register_async_verb(const std::string &name, std::function<std::string()> work);
  void register_async_verb(const std::string &name, const std::string &done_channel,
                           std::function<std::string()> work);

  // The standard way out: `on: quit` (a menu item's event) and the verb (quit [code]) both
  // request_close(code). code is an integer 0..255 (default 0); anything else raises in the
  // calling program (TypeError, ValueError) and nothing closes. The first request's code is the
  // one exit_code() keeps. The host returns exit_code() as its exit status.
  void register_quit();

  // Let the document use the network, as a host's --net flags say (net_host.h: the allowlist, the
  // credential store, the http verbs and http(s) URIs). Returns "" or what is wrong, and then
  // nothing is turned on. Off unless called; this AppRuntime turns it off again when it dies.
  std::string enable_net(const NetHostConfig &config);

  // The document's own command line: everything after its path (doc_args.h). load() publishes
  // it as the read-only <prefix>.argv before init: runs and, when the document declares an args:
  // block, parses it into <prefix>.args.<name>. `invocation` starts the usage text (e.g. "ari run
  // doc.ari"; empty = the document path). With `validate` false (a `check`), argv is not checked
  // against the block and declared arguments take their defaults.
  void set_document_args(std::vector<std::string> argv, std::string invocation = std::string(),
                         bool validate = true);

  // --- document ---

  // Load + mount a .ari document, running its init:/on_* scripts and (via the SceneAdapter,
  // if any) realizing its scene under the state prefix. Returns non-fatal warnings; THROWS
  // std::runtime_error on a fatal load (min_libcvc gate, parse error, no yaml support), and
  // document_args_error when its arguments do not match its args: block or ask for --help.
  std::vector<std::string> load(const std::string &path);

  // Whether the last load()ed document declares an args: block.
  bool has_args_block() const;

  // The last load()ed document's tests: block, name -> program in document order (`ari test`
  // runs each with runtime().run_program).
  const std::vector<std::pair<std::string, std::string>> &tests() const;

  // Mount a programmatically-built widget tree instead of a file. Applied next frame.
  void set_root(Widget root);

  // --- per-frame (NEUTRAL core; no draw, no dt) ---

  // Run queued Ariadne actions on this thread + a scheduler slice, then (if a SceneAdapter
  // is set) mirror bound scene visibility and service the scene. The host does input pump /
  // camera / draw around this. Call once per frame.
  void drain();

  bool should_close() const;
  void request_close();
  // Close with an exit status: the first request's code stands (request_close() is code 0).
  void request_close(int code);
  // The status a (quit code) asked for; 0 until something asks to close.
  int exit_code() const;

  // Whether the document is idle: no async call (a request) in flight, no program step waiting
  // to run (a resident still parsing a reply between two requests is not idle) and no picture it
  // draws still loading (§5.11).
  bool idle() const;
  // For a snapshot of a document that uses the network: run `step` (a drain, or a host's whole
  // frame) until the document has been idle for three steps running, or `max_seconds` pass, so
  // the frame shows what its requests brought back. ari and arigl call it when the network is on.
  void settle(const std::function<void()> &step, double max_seconds = 10.0);
  // settle(), then draw a frame with `render`; while that frame drew a picture still loading (one
  // whose URL a reply has just put in state, so no frame had asked for it yet), settle and draw
  // again. All within `max_seconds`. A snapshot's last frame, with its requests and pictures in.
  void settle_and_render(const std::function<void()> &step, const std::function<void()> &render,
                         double max_seconds = 10.0);

  // --- input seam (document-level on_key/on_pointer residents) ---
  void post_key(bool down, const std::string &key, int mods = 0, bool repeat = false);
  void post_pointer(int kind, double x, double y, double dx, double dy, int button, int clicks);
  // Route a signal to the document's on_signal: program. True = the document handles it; false =
  // it declared none, so the host applies the default (for SIGINT: close). See
  // Runtime::post_signal.
  bool post_signal(const std::string &signal, const std::string &source = "host");

  // --- diagnostics / capabilities ---
  std::vector<std::string> take_warnings(); // reactive read-lane diagnostics since the last call
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
  int exitCode_ = 0; // the first close request's code
  bool intrinsicsRegistered_ = false;
  bool netEnabled_ = false; // enable_net turned the network on: the destructor turns it off
  // The host verbs, shared with the program-lane provider so a verb registered after the first
  // is still bound when the next program env is built. Emptied by the destructor: the provider
  // lives on in the process-wide registry, and must not reach this AppRuntime's handlers after it.
  struct Verbs {
    std::vector<std::pair<std::string, std::function<void()>>> sync;
    std::vector<std::tuple<std::string, std::string, std::function<std::string()>>> async;
    std::function<void(int)> quit; // register_quit: (quit [code])
  };
  std::shared_ptr<Verbs> verbs_;
  std::vector<std::string> pendingWarnings_; // read-lane diagnostics drain() collected
  std::vector<Widget> hostMenus_;            // set_host_menus: put in every mounted tree
  std::vector<KeyBinding> hostKeys_;         // set_host_keys: bound, and listed, in every one
  bool keysWindow_ = false;                  // set_host_keys was called: always a Keys window
  // The keys' defaults at keys.<id>, before init: (which may set its own). A new document's own
  // replace whatever an earlier document left at the same keys.<id> (they share the prefix), ""
  // for one with no key; a reload keeps the user's keys, and the host's are only seeded.
  void seed_keys(const std::vector<KeyBinding> &own, bool new_document);
  // Bind them, and the host's, and add the Keys window.
  void add_keys(Widget &root, std::vector<KeyBinding> keys);
  std::string docPath_; // the document load() last loaded, to tell a reload from a new one
  std::vector<std::string> docArgv_;         // the document's command line (set_document_args)
  std::string invocation_;
  bool validateArgs_ = true;
  bool hasArgsBlock_ = false;
  std::vector<std::pair<std::string, std::string>> tests_; // the document's tests: block
};

} // namespace ariadne
} // namespace cvc
