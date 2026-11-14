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
#include <cvc/ariadne/data.h>          // DataStore
#include <cvc/ariadne/net_host.h>      // NetHostConfig
#include <cvc/ariadne/scene_adapter.h> // cvc::ariadne::SceneAdapter (optional)
#include <cvc/ariadne/scene_world.h>   // SceneWorld: a scene's world with no adapter
#include <cvc/state/state_exec/async_call.h> // async_marshal (register_async_function)
#include <cvc/state/state_exec/types.h>      // native_fn (register_function)
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace cvc {
class app;
namespace ariadne {

class Backend;
struct LoadResult;  // the loaded document (loader.h)
class ScriptEngine; // a scripting host's language engine (script_engine.h)
class ScriptLane;

class AppRuntime {
public:
  // Build a Runtime over `app`'s state at `prefix`. `scene` is the optional 3D-scene
  // adapter (nullptr = no scene: a scene-bearing document warns and drops its scene).
  // `scene`, once given, must outlive this AppRuntime.
  AppRuntime(cvc::app &app, std::string prefix, SceneAdapter *scene = nullptr);
  // Host thread, outside a pump (Runtime::~Runtime).
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
  // The app it runs in.
  cvc::app &app() const { return app_; }
  // The state prefix the document binds under.
  const std::string &prefix() const { return prefix_; }

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

  // The host's UI size (keys.h), offered at the top of the Controls window as "UI size": arigl's
  // ImGuiOverlay scale, which a phone may want larger or smaller. A host that calls this gets the
  // window. Applied by the next load() or set_root().
  void set_host_ui_size(UiSizeControl ui);

  // Key bindings of the host's own (keys.h), beside each document's `keys:`: arigl's camera keys.
  // Once a host has called this (even with none), or for a document with keys, the document gets
  // a Controls window (id ariadne_keys, closed until tree.ariadne_keys.open is set -- a host's
  // menu_toggle on that key opens it) listing them all, each with a Change button: click it, then
  // press the new key (Escape keeps the old one). Applied by the next load() or set_root().
  void set_host_keys(std::vector<KeyBinding> keys);
  // The pointer settings of the host's camera (keys.h), shown above the keys in the Controls
  // window: the look (drag or mouse look), its sensitivity and the inverted axes. A host that
  // calls this gets the window too. Applied by the next load() or set_root().
  void set_host_mouse(MouseControls mouse);

  // Register a host verb the .ari program can call. register_verb is SYNCHRONOUS (fn runs
  // inline on the drain thread, no args/return).
  //
  // Every registration below is made on the host thread, outside a frame (not from a verb or a
  // function while one runs). Register before load(), or before the first frame: one registered
  // once the document's programs run binds in an isolated document the first time nothing of it
  // runs, at the latest at its next load (until then a host_only one is a NameError there; see
  // Runtime::isolated_group), as in group 0 it binds at once.
  //
  // register_async_verb makes `name` an async function on the calling program's async host (the
  // app's, or an isolated document's group's) (docs/STATE_EXEC_EXCEPTIONS.md 3):
  // - (name) parks the calling program while `work` runs on the app's background task pool,
  //   then returns `work`'s string;
  // - an exception `work` throws raises at the call;
  // - (name-async) returns a future for (await f);
  // - the work is charged to the calling action's budget (ariadne.limits.action_max_steps and
  //   action_max_time).
  // With a `done_channel`, the result is also posted there, to the calling program's group (group
  // 0 from init:). That keeps documents that wait with (msg-recv "done_channel") working: the old
  // nav_compute pattern.
  void register_verb(const std::string &name, std::function<void()> fn);
  void register_async_verb(const std::string &name, std::function<std::string()> work);
  void register_async_verb(const std::string &name, const std::string &done_channel,
                           std::function<std::string()> work);

  // Host functions with arguments and a value, the way a language binding registers its
  // callables (pycvc: pycvc.host_function). register_function's `fn` runs inline on the drain
  // thread: (name a b) passes the call's arguments and returns fn's value; an exception it throws
  // raises at the call. register_async_function's `marshal` runs at the call, on the scheduler's
  // thread, and returns the kernel the app's background task pool runs while the program parks,
  // as register_async_verb's work does; (name-async ...) returns a future. With a `done_channel`,
  // the result is also posted there. Re-registering a name replaces it.
  void register_function(const std::string &name, cvc::state_exec::native_fn fn);
  void register_async_function(const std::string &name, cvc::state_exec::async_marshal marshal,
                               const std::string &done_channel = std::string());
  // An async function whose work runs by itself (docs/STATE_EXEC_EXCEPTIONS.md 3.12): `marshal`
  // runs at the call, on the scheduler's thread, and returns the work's start, which begins it and
  // returns; the work settles the call when it ends, and the program parks until then. A
  // scripting engine's activation is such work (ScriptLane::export_activation). (name-async ...)
  // returns a future. With `blocking`, a call from a program whose host can't park it (init:'s,
  // which runs on a scheduler of its own and makes its calls inline) is `blocking`'s instead,
  // made in the call: plain native code, not a kernel, so it may run processes. With `sliced`, a
  // call from a program whose host parks (group 0's, or an isolated document's group's) is
  // `sliced`'s instead: the work done as the program's own (a data job, data_job.h), with
  // (name-async ...) still the parked work's future. Re-registering a name replaces it.
  void register_async_work_function(const std::string &name,
                                    cvc::state_exec::async_work_marshal marshal,
                                    cvc::state_exec::native_fn blocking = nullptr,
                                    cvc::state_exec::native_fn sliced = nullptr);

  // A host extension's part in loading a document. `hook` runs in every load() (a reload too),
  // after the document is parsed and before its requires: is checked and its init: runs, with the
  // parsed document (a custom block a host registered is in lr.extras) and its path. A scripting
  // host runs the document's python: block here and registers the functions it exports, so
  // requires: and the load's scan of what the programs call both see them. Throw to fail the load;
  // push onto lr.warnings to warn. Hooks run in the order added.
  using LoadHook = std::function<void(LoadResult &lr, const std::string &path)>;
  void add_load_hook(LoadHook hook);
  // Run the document blocks of `engine`'s language (script_engine.h): every load() hands the
  // document's block to the engine's load_block, with this runtime's lane for it, after the parse
  // and before the load hooks, requires: and init:. From the second load on, each engine that ran a
  // block is unloaded first, and its lane's hooks dropped. A scripting host does this for each
  // runtime it lends its engine to (python::Engine::attach). An engine of a language already used
  // replaces that one. Returns the engine's lane.
  ScriptLane &use_script_engine(std::shared_ptr<ScriptEngine> engine);
  // The lane of the engine this runtime uses for `language`, or null.
  ScriptLane *script_lane(const std::string &language) const;
  // For ScriptLane: a lane's key, pointer or signal hooks changed. The runtime hands the lanes the
  // kinds of event their hooks take now (Runtime::set_event_tap), and no others.
  void script_hooks_changed();
  // Work a host extension does every frame: `hook` runs at the end of every drain(), on its
  // thread, after the document's programs and the scene service. A scripting host runs a Python
  // program's per-frame work here (pycvc.ari.Host's events, on_frame hooks and watches), so
  // it runs whichever loop drives the frames. Hooks run in the order added.
  void add_frame_hook(std::function<void()> hook);

  // Run `script` as one of the document's action programs (Runtime::run_program): submitted in the
  // next drain(), and `done` (may be null) is called from a later drain() with "" when it finished,
  // else why not.
  void run_program(std::string script, std::function<void(const std::string &)> done = nullptr);

  // The standard way out: `on: quit` (a menu item's event) and the verb (quit [code]) both
  // request_close(code). code is an integer 0..255 (default 0); anything else raises in the
  // calling program (TypeError, ValueError) and nothing closes. The first request's code is the
  // one exit_code() keeps. The host returns exit_code() as its exit status.
  void register_quit();

  // Let the document use the network, as a host's --net flags say (net_host.h: the allowlist, the
  // credential store, the http verbs and http(s) URIs). Returns "" or what is wrong, and then
  // nothing is turned on. Off unless called; this AppRuntime turns it off again when it dies.
  std::string enable_net(const NetHostConfig &config);

  // The app's data store (data.h), which load() binds the document's data: block in (each entry a
  // link at <prefix>.data.<name>) and the (data-*) verbs reach. A host reads the same datasets
  // through it: wait("<prefix>.data.city"), then the objects on the nodes.
  std::shared_ptr<DataStore> data_store() const { return data_; }

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
  // document_args_error when its arguments do not match its args: block or ask for --help. It
  // decides where the document's programs run (Runtime::isolated_group). Host thread, outside a
  // pump.
  std::vector<std::string> load(const std::string &path);

  // Whether the last load()ed document declares an args: block.
  bool has_args_block() const;

  // The last load()ed document's tests: block, name -> program in document order (`ari test`
  // runs each with runtime().run_program).
  const std::vector<std::pair<std::string, std::string>> &tests() const;

  // Mount a programmatically-built widget tree instead of a file. Applied next frame. Host thread,
  // outside a pump (Runtime::set_root).
  void set_root(Widget root);

  // --- per-frame (NEUTRAL core; no draw, no dt) ---

  // Run queued Ariadne actions on this thread + a scheduler slice, then (if a SceneAdapter
  // is set) mirror bound scene visibility and service the scene. The host does input pump /
  // camera / draw around this. Call once per frame.
  void drain();
  // drain() in two halves around app.exec_pump_frame() (Runtime::drain_begin/drain_end): the
  // scene work, warnings and frame hooks come after drain_end(). drain() is both.
  void drain_begin();
  void drain_end();
  // Declare that a subscriber this host connected at or under `prefix` must run on the host thread
  // (STATE_EXEC_THREADED.md 4.8). A document whose prefix is at, above or under a declared one is
  // never pumped on a worker (a write emits childChanged on every ancestor). Call before load().
  void declare_thread_affine_prefix(const std::string &prefix);
  // Wait, without cancelling anything, until no async kernel of this document is queued or
  // running: on the app's host, and on its own group's when the document is isolated. Host thread,
  // outside a frame.
  void wait_idle();

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
  // Hear each runtime warning as it is produced, while the document runs: a program that failed,
  // a resident (on_tick:, on_key: ...) stopped or given up, a broken expression. drain() calls
  // `listener` once per new warning, on the drain's thread; take_warnings() still returns every
  // one, so a host can both report them live and summarize them at exit. Null stops it.
  void set_warning_listener(std::function<void(const std::string &)> listener);
  // A host extension's warning (a scripting engine's), reported as the runtime's own are: to the
  // listener now, and in the next take_warnings().
  void add_warning(std::string warning);
  bool reload_if_changed();                 // §12.5 hot reload if a watched source changed
  static bool have_yaml();                  // can load a YAML .ari?

  // The document's scene world when this runtime runs it itself: a host with no SceneAdapter (ari,
  // ari test, pycvc's headless AriApp, arix) still runs a scene's physics and bodies, stepped on
  // app.world_clock() by drain() and publishing `physics.stats.*` (scene_world.h). Null when an
  // adapter realizes the scene (it runs the same world beside its drawing) or the scene has none.
  SceneWorld *scene_world() const { return world_.get(); }

  // The built-in sim.step and sim.restart (sim_transport.ari's Step and Restart buttons, and any
  // document's bare `on: sim.step` / `on: sim.restart`), which the constructor registers. A host's
  // own on("sim.step", ...) or on("sim.restart", ...) replaces them.
  //
  // step_world(n): n more quanta on the scene's clock, through its step key (<prefix>.sim.step by
  // default; world_clock_binding), so it works the same whether this runtime drives the clock or an
  // adapter does, paused or not. Nothing happens for a document with no scene.
  void step_world(std::uint64_t n = 1);
  // restart_world(): the world clock back to tick 0 (world_clock::reset), and the scene's world
  // rebuilt as the document built it (SceneWorld::restart, here or through the adapter), so a
  // restarted world steps as a fresh one. Then it adds one to the restart counter beside the step
  // key (<prefix>.sim.restarts), which a program or a host watches to reset its own state.
  // Problems warn (take_warnings). False when there was no world to rebuild (the clock and the
  // counter still restart).
  bool restart_world();

private:
  void ensure_intrinsics(); // lazily install the register_action_intrinsics provider

  cvc::app &app_;
  std::string prefix_;
  SceneAdapter *scene_; // optional; null = no scene (terminal / no-scene backend)
  // With no adapter: the scene's world, and the clock it runs on (scene_world()).
  std::unique_ptr<SceneWorld> world_;
  std::unique_ptr<SceneClockDriver> worldClock_;
  // The scene clock's step key and the restart counter beside it (step_world, restart_world);
  // empty for a document with no scene.
  std::string stepKey_, restartsKey_;
  // use_script_engine: each engine, its lane, and whether it has run a block since its last unload.
  // Before rt_, so the lanes outlive the document's processes: an activation is cancelled while
  // its engine and lane are still there.
  struct EngineUse {
    std::shared_ptr<ScriptEngine> engine;
    std::unique_ptr<ScriptLane> lane;
    bool loaded = false;
  };
  std::vector<EngineUse> engines_;
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
  // lives on in the app's list, and must not reach this AppRuntime's handlers after it.
  struct Verbs {
    std::vector<std::pair<std::string, std::function<void()>>> sync;
    std::vector<std::tuple<std::string, std::string, std::function<std::string()>>> async;
    std::function<void(int)> quit; // register_quit: (quit [code])
    // The (data-*) verbs: the store, and the document's prefix and directory (a relative URI's
    // base), as the last load() set them.
    std::weak_ptr<DataStore> data;
    std::string prefix;
    std::string doc_dir;
    // register_function / register_async_function, by name (a later registration replaces).
    std::map<std::string, cvc::state_exec::native_fn> functions;
    std::map<std::string, std::pair<std::string, cvc::state_exec::async_marshal>> async_functions;
    struct AsyncWork {
      cvc::state_exec::async_work_marshal marshal;
      cvc::state_exec::native_fn blocking; // init:'s form
      cvc::state_exec::native_fn sliced;   // a parking host's (name ...), when there is one
    };
    std::map<std::string, AsyncWork> async_work_functions;
    // Held by every read and write of the fields above: the providers' bodies read them, which a
    // spawned child in an `any` group runs on the pumping worker, and the registrations, a load
    // and the destructor write them (c-m5). A body copies what it binds under it, then binds
    // unlocked. Nothing a host gave (a callable) is destroyed under it, since its destructor may
    // take a lock of its own (pycvc's takes the GIL): what goes is moved out first.
    std::mutex mutex;
  };
  std::shared_ptr<Verbs> verbs_;
  std::shared_ptr<DataStore> data_; // DataStore::of(app_): kept alive while this runtime is
  std::vector<std::string> pendingWarnings_; // read-lane diagnostics drain() collected
  std::vector<std::string> threadAffinePrefixes_;            // declare_thread_affine_prefix
  std::function<void(const std::string &)> warningListener_; // set_warning_listener
  void collect_warnings(); // the Runtime's new warnings -> pendingWarnings_, each to the listener
  std::vector<Widget> hostMenus_;            // set_host_menus: put in every mounted tree
  UiSizeControl hostUiSize_;                 // set_host_ui_size: the Controls window's first row
  // The Controls window's Display rows (hostUiSize_), ahead of everything else in it.
  void add_ui_size(Widget &root) const;
  std::vector<KeyBinding> hostKeys_;         // set_host_keys: bound, and listed, in every one
  bool keysWindow_ = false;                  // set_host_keys was called: always a Keys window
  MouseControls hostMouse_;                  // set_host_mouse: the window's Mouse rows
  // The keys' defaults at keys.<id>, before init: (which may set its own). A new document's own
  // replace whatever an earlier document left at the same keys.<id> (they share the prefix), ""
  // for one with no key; a reload keeps the user's keys, and the host's are only seeded.
  void seed_keys(const std::vector<KeyBinding> &own, bool new_document);
  // Bind them, and the host's, and add the Controls window.
  void add_keys(Widget &root, std::vector<KeyBinding> keys);
  std::string docPath_; // the document load() last loaded, to tell a reload from a new one
  std::vector<std::string> docArgv_;         // the document's command line (set_document_args)
  std::string invocation_;
  bool validateArgs_ = true;
  bool hasArgsBlock_ = false;
  std::vector<std::pair<std::string, std::string>> tests_; // the document's tests: block
  std::vector<LoadHook> loadHooks_;                        // add_load_hook, in order
  std::vector<std::function<void()>> frameHooks_;          // add_frame_hook, in order
  // load()'s part for them: unload what the last document loaded, then hand each its block.
  void run_script_blocks(LoadResult &lr, const std::string &path);
};

} // namespace ariadne
} // namespace cvc
