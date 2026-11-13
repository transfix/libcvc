#ifndef CVC_ARIADNE_H
#define CVC_ARIADNE_H

// Ariadne — the context-agnostic UI & scene DSL core (cvc::ariadne, pure libcvc).
//
// Runtime is the backend-neutral host for a retained widget tree (widget.h). It
// owns the tree, the reconcile commit boundary, and ALL cvc::state binding; it
// renders by WALKING the tree once and driving a pluggable Backend (backend.h) —
// it reads each bound cvc::state path, hands the value to a Backend primitive to
// draw, and writes back only when the primitive reports the edit committed.
// Nothing here knows about ImGui, VTK, or a terminal: cvc::gl::ImGuiBackend is
// one Backend; a terminal backend is another (roadmap §16).
//
// A backend decides WHEN render() runs: a host-pumped backend (ImGui-over-VTK)
// calls render() from the host's render pass (see ImGuiBackend::install); a
// framework-owned backend calls it from its own loop. Either way, actions raised
// by buttons / menu items are QUEUED as named events and run OFF the render walk
// by Runtime::drain() (the P0 form of the deferred intent buffer, roadmap
// §7.2/§4.7) — nothing side-effectful runs inside the mid-render walk.
//
// This is the P0 slice; the full contract (dynamic-DOM reconcile, expressions,
// scene binding, validation) is docs/roadmap/CVCGL-UI-DSL-ROADMAP.md.

#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/input.h> // InputEvent — Runtime::post_input feeds on_key/on_pointer (§4.6)
#include <cvc/ariadne/keys.h>  // KeyBinding — set_key_bindings
#include <cvc/ariadne/requires.h> // HostFacts — what this host provides (requires:, has?)
#include <cvc/ariadne/widget.h>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {
class Backend;
}
namespace state_exec {
// Runtime::document_scope() return type; full definition in <cvc/state/state_exec/intrinsics.h>.
// `class` (not struct) to match the real declaration — a tag mismatch is an LNK2019 on MSVC.
class document_scope;
class exec_group;          // Runtime::isolated_group(); <cvc/state/state_exec/exec_group.h>
struct environment;        // types.h
struct intrinsics_context; // intrinsics.h
struct value_tag;          // types.h: value_t
} // namespace state_exec
} // namespace cvc

namespace cvc {
namespace ariadne {

// The caps a program of the document runs under, read from the tree as each one starts.
enum class ProgramLane {
  // An on: action's (§7.4): ariadne.limits.action_max_steps (5000), action_max_time (0.1 s) and
  // action_max_memory (4 MiB), and the with-lock default ariadne.exec.lock_timeout.
  action,
  // A resident's (on_tick:, on_key: ...; §13.8): no total, ariadne.limits.resident_max_step_time
  // for each step, and no with-lock default.
  resident,
  // The init: lane's: 10 million steps and 5 s, for work a load does (a guest's ari_init), and
  // the action's with-lock default.
  init
};

// A program a host extension runs as one of the document's own (Runtime::start_process): a
// scripting engine's activation is one (script_engine.h).
struct LaneProcess {
  std::string program; // the DSL program the process runs
  std::string name;    // as (ps) shows it
  ProgramLane lane = ProgramLane::action;
  std::string scope; // its chroot: "" = the document's prefix
  // Caps of its own, which only ever lower the lane's: 0 leaves the lane's.
  uint64_t max_steps = 0;
  double max_time = 0.0;
  uint64_t max_memory = 0;
  int priority = 0; // the scheduler's nice value, as a program's set-priority sets it
  // Natives of its own, bound in a scope of their own over the lane's environment. `ictx` is the
  // process's intrinsics context, which lives as long as the process.
  std::function<void(const std::shared_ptr<cvc::state_exec::environment> &env,
                     cvc::state_exec::intrinsics_context &ictx)>
      extend;
  // Once its process has ended, from the drain() whose pump saw it end: "" for a normal finish,
  // else why not, as run_program's done says it. Never called if the Runtime goes first. Without
  // one, a process that fails or is killed is warned about as an action is, by its name.
  std::function<void(const std::string &)> done;
};

class Runtime {
public:
  // `prefix` is the cvc::state prefix that relative bind paths splice onto
  // (e.g. a SceneGraph's getStatePrefix(), or a "ui.docs.<doc>" subtree). A bind
  // path with a leading '/' is resolved app-root-absolute instead. Empty prefix
  // = binds are used as-is (app-root-relative).
  explicit Runtime(cvc::app &app, std::string prefix = std::string());
  // Tears the document down: closes what it opened, kills and reaps its processes, and destroys
  // its group (waiting for that group's kernels). Host thread, outside a pump: while a pump_all of
  // the app's pool holds its group it is refused, from any thread (logged, then aborted).
  ~Runtime();

  Runtime(const Runtime &) = delete;
  Runtime &operator=(const Runtime &) = delete;

  // Choose the surface. Must be set before render(); the Runtime does not own
  // the Backend (it must outlive the Runtime).
  void set_backend(Backend *backend);

  // §5.11: the directory a picture's relative file:// URI resolves against (the document's own;
  // AppRuntime::load sets it). Empty: the working directory.
  void set_media_base(std::string dir);
  // §5.11: whether a picture by URI the document draws is still loading.
  bool media_busy() const;
  // §5.11: whether the last frame drew a picture that was still loading (a placeholder), so a
  // later frame shows more once media_busy() is false: a snapshot waits, then draws again.
  bool media_stale() const;

  // Set/replace the widget tree. The swap is applied at the next frame boundary
  // (the top of render(), before any node is emitted), never mid-walk — the P0
  // form of the §11.5.1 reconcile commit boundary. Host thread, between frames and outside a
  // pump. A new tree is a new document: where its programs run is decided again (isolated_group).
  void set_root(Widget root);

  // Register a host handler for an action event name (a Button / MenuItemAction
  // `on:`). The handler runs on the host thread inside drain(), never in the walk.
  void on(std::string event, std::function<void()> handler);

  // §7.1 resident on:tick handler — register a state_exec PROGRAM that runs ONCE PER FRAME.
  // Unlike a fire-once action `on:`, this submits ONE long-lived, owner-tagged process on the
  // app-wide scheduler that PARKS between frames (it never re-submits per frame); each drain()
  // posts it a tick to run the program once. Pass a LoadResult::on_tick_script here after loading.
  // "" clears it (kills and reaps the resident). The resident is reaped with the Runtime
  // (kill_owner on teardown). Host thread, outside a pump; like the other lanes' programs and the
  // channel policy, it is read by the isolation rules, so the placement is decided again.
  void set_tick_program(std::string script);

  // §4.6 document-level input handlers — register a state_exec PROGRAM run per input event. Like
  // on:tick, each is ONE long-lived resident (owner-tagged, reaped with the Runtime); it parks and
  // is woken by post_input with the event bound as `event`, which the body reads via
  // (get-attr event "key"/"kind"/"x"/"y"/"button"/"mods"/"dx"/"dy"/"clicks"/"repeat").
  // set_key_program handles keyboard events (key_down/key_up); set_pointer_program handles mouse
  // events (mouse_move/mouse_button_*/mouse_wheel). Pass LoadResult::on_key_script /
  // on_pointer_script.
  // "" clears it. (Widget-level on_click/on_hover are a separate path.) Host thread, outside a
  // pump, as set_tick_program.
  void set_key_program(std::string script);
  void set_pointer_program(std::string script);

  // Rebindable key actions (keys.h). A key down post_input gets that matches one of a binding's
  // keys -- read from its state key each time, so a rebinding takes effect at once -- runs the
  // binding's `on` in the next drain(), as a button's on: runs. While keys.capture names a binding
  // (the Keys window's Change button), the next key down is that binding's new key instead
  // (Escape keeps the old one), and on_key does not see it. Replaces the last set; {} clears them.
  void set_key_bindings(std::vector<KeyBinding> bindings);
  // The Keys window is waiting for a key: a host that would use keys itself first (a terminal's
  // focus keys) hands them to post_input instead.
  bool key_capture_pending() const;

  // The next tree is a different document (AppRuntime::load): each window's first frame from the
  // next render() on ignores the geometry persisted at tree.<id>.geometry -- the last document's,
  // where it used the same id -- and the backend lays it out afresh (Backend::reset_layout).
  void forget_layout();

  // Document-level signal handler — a state_exec PROGRAM run per signal the host routes here (the
  // .ari on_signal: block; pass LoadResult::on_signal_script). Like on_key it is ONE resident,
  // woken with `event` bound to {kind: "signal", signal: <name>, source: <who raised it>}; names
  // follow POSIX ("SIGINT" for a terminal's Ctrl-C, "SIGQUIT" for Ctrl-\, "SIGTERM", "SIGHUP").
  // Declaring it takes the signal over, like a POSIX handler: the host skips the default action
  // (exit) and the program decides — e.g. calls the host's quit verb, or asks first. "" clears it.
  void set_signal_program(std::string script);
  bool has_signal_program() const;
  // Deliver `signal` to the on_signal program, and to the tap that takes signals
  // (set_event_tap). Returns true when one of them will handle it, false when neither is set (the
  // host then applies the default). `source` says where it came from (e.g. "terminal" for a Ctrl-C
  // keypress, "os" for a delivered signal). Host thread, outside a pump (it may start the
  // program's resident; debug builds check it), like post_input; the program runs in the next
  // drain(). A host that catches an OS signal on another thread hands it over.
  bool post_signal(const std::string &signal, const std::string &source = "host");

  // §12 channel enforcement (runtime backstop): install the document's channel policy so a program
  // `on:`/resident that sends or receives on an UNDECLARED channel is refused at run time — the
  // completeness layer for DYNAMIC channel names the load-time lint (loader) can't see. `declared`
  // is the document's declared channel names, `global` its declared app-root-globals (the
  // '/'-escape allowlist); `strict` enforces (throw, caught fail-safe) — pass false for warn/off
  // (the load lint already surfaced those), which clears enforcement. The host feeds these from a
  // LoadResult (channels: + lint:); pass plain vectors so this header stays independent of the
  // loader. Calling with `strict=false` and empty lists (the default) disables runtime enforcement.
  // Set once after load, before render/drain; it applies to every action + resident.
  void set_channel_policy(std::vector<std::string> declared, std::vector<std::string> global,
                          bool strict, bool quiet);

  // Feed one input event to the runtime — the host calls this each frame for every event from its
  // input source (e.g. cvc::gl::SdlInput), BEFORE drain(), so the event is delivered to the
  // on_key/on_pointer residents this same frame (drain() drains the ingress + pumps). Host thread,
  // outside a pump: it may start the residents, then posts to the document's scheduler's ingress
  // (debug builds check it). A no-op if no matching handler is registered.
  void post_input(const InputEvent &ev);

  // A listener of the document's input events and signals besides its on_key:/on_pointer:/
  // on_signal: programs: the scripting engines' hooks, for AppRuntime (script_engine.h). Each
  // event post_input or post_signal takes is handed to `tap` as the dict those programs read as
  // `event` ("key", "pointer" or "signal" its kind), in the order they came: in the next drain(),
  // before its pump, on the drain's thread. `kinds` says which it takes (tap_key | tap_pointer |
  // tap_signal). A signal it takes is handled, as one an on_signal: program takes is: post_signal
  // returns true. 0 stops it. Set it on the drain's thread, the host thread, where post_input and
  // post_signal run too.
  enum : unsigned { tap_key = 1u, tap_pointer = 2u, tap_signal = 4u };
  using EventTap =
      std::function<void(const std::string &kind, const cvc::state_exec::value_tag &event)>;
  void set_event_tap(unsigned kinds, EventTap tap);

  // Run `script` as an action program (the on: lane: the full env, writes chrooted to the
  // document's prefix), submitted in the next drain(). When its own process has ended, a later
  // drain() calls `done` with "" for a normal finish, else why it did not finish ("failed: " an
  // uncaught exception and where it was raised, "did not complete: " a limit or a kill). Its
  // ending is reported only to `done`, never as a warning. `ari test` runs a tests: case this way.
  // Host thread, outside a pump.
  void run_program(std::string script, std::function<void(const std::string &)> done);

  // Start `p` now as a process of the document's program lane on the document's scheduler (its
  // isolated group's, else app.exec_scheduler()), as an action or a resident is: in an environment
  // made like theirs (the intrinsics, the host's natives and functions, writes chrooted to its
  // scope, the document's channel policy and resource scope, the app's lock manager), after the
  // document's functions: when it runs in the document's own scope, owner-tagged with the document
  // so it is reaped with it, and under p.lane's caps. The next pump runs it: the same drain's, when
  // started from inside a drain() before its pump. A process with natives of its own (p.extend)
  // runs host code inline, so it declares require_host_thread first, and it is refused while the
  // document still runs in an isolated `any` group (see require_host_thread). Returns the pid;
  // throws (a syntax error in p.program, that refusal) when it can't start. Host thread, outside a
  // pump or from a step of the document's own group 0 or host-affinity group (an exported call,
  // an on: hook), never a worker.
  int start_process(LaneProcess p);

  // Pump app.exec_scheduler() one bounded slice, and the document's isolated group if it has one
  // (through the app's pool), then sweep this document's processes, as drain() does after its
  // handlers and residents, and nothing more: for a host extension that has to wait for a
  // process of its own outside the frame loop (ScriptLane::run). Each call starts the next app
  // frame, as a drain's pump does. Host thread, never from inside a pump or a step of either.
  void pump();

  // Render one frame: apply any pending tree swap, then walk the tree driving the
  // Backend and binding cvc::state. Called by the active backend (from the host
  // render pass, or the backend's own loop). A pure reader of state that only
  // ENQUEUES action intents — it never runs a handler or reloads a document.
  void render();

  // The frame clock that animations read rather than state (a spinner, Kind::Spinner): seconds
  // since this Runtime was made, read from the steady clock at the top of each render(), so it
  // moves with the host's frames whatever the document's programs are doing. A host that renders
  // frames with no real time between them (a snapshot) pins it instead: set_frame_clock(t) holds t
  // from the next render() on; nullopt goes back to the steady clock.
  void set_frame_clock(std::optional<double> seconds);
  // The clock's reading for the frame render() last drew.
  double frame_clock() const;

  // The time a frame's read-lane expressions (visible_when, enabled_when, computed binds) may take
  // together, from the next render() on (default 0.030 s). Past it the rest skip that frame: each
  // keeps the value it had last frame (a predicate, a computed text, combo options, a repeat
  // count), or its fail-safe one where it never had one (hidden, disabled, empty, 0), so a slow
  // frame doesn't flicker the widgets at the end of a long list. A skip is not the document's
  // fault, so it is never a reactive warning.
  void set_reactive_frame_budget(double seconds);
  // How many read-lane evaluations the frame budget has skipped since this Runtime was made: a
  // diagnostic (a host may log it), never a warning.
  std::uint64_t reactive_skip_count() const;
  // How many read-lane expressions have run since this Runtime was made. A result reused from the
  // cache (§2.1: nothing it read was written since) is not counted, so a frame that wrote nothing
  // adds none.
  std::uint64_t reactive_eval_count() const;

  // Run every queued action event's handler, then clear the queue. Call once per
  // frame in the host loop, BEFORE your sim tick and OUTSIDE render().
  //
  // drain() is also where the app's state_exec scheduler (app.exec_scheduler()) advances: it pumps
  // it a bounded slice every drain once this Runtime has submitted to it (an `on:` program, a
  // resident), and otherwise whenever any process is live on it, whoever submitted it (e.g. a
  // scene's transfer-function overlay tasks, cvc/tf/overlay_tasks.h). An app's Runtimes share one
  // frame boundary: with each drained once a frame, in any order, an (await …) resumes once a
  // frame, not once per Runtime. While nothing has used the scheduler it is neither pumped nor
  // built. A host with no Runtime pumps it itself (cvc::pump_exec_frame, cvc/core/async_lane.h).
  // An isolated document's group (isolated_group) is pumped there too, through the app's pool
  // (app.exec_pool().pump_all), once a drain.
  void drain();
  // drain() in two halves, for a host that pumps once a frame for all its Runtimes (threaded
  // Stage 1, STATE_EXEC_THREADED.md 4.12): drain_begin() on each Runtime (events, key bindings,
  // actions submitted, ticks, input residents), then app.exec_pump_frame(), then drain_end() on
  // each (the sweep). drain_end() pumps the scheduler itself unless exec_pump_frame() ran since
  // this Runtime's drain_begin(). drain() is drain_begin() then drain_end(). Host thread only.
  void drain_begin();
  void drain_end();

  // ---- isolated documents (threaded Stage 1, S1-Q6) ----
  // The group this document's programs run in when ariadne.exec.isolate_documents isolated it
  // (pumped on a worker, or on the host thread when its affinity is host); null when they run in
  // group 0, the app scheduler. Decided at load (AppRuntime::load asks at its end), and again,
  // before the next program starts or the next frame is drawn, after a set_root, a lane's program,
  // the channel policy, the key bindings or a thread-affine prefix change. A host verb or provider
  // registered later (refresh_program_names, register_action_intrinsics) has it decided again the
  // first time nothing of the document runs (before its first frame, or a frame with no resident,
  // child or action left), at the latest at its next restart (a load, set_root, a lane's program);
  // until then a host_only name registered late is a NameError in an isolated `any` group (logged
  // once). When the answer changes the document
  // moves: its actions, spawned children and residents are killed and reaped where they ran, a
  // run_program caller is told "did not complete: the document moved to another group", the
  // residents start again in the new place, and the streams the document owns post there from then
  // on; an old group is unregistered at once and destroyed once its calls have ended, so a move
  // does not wait for them. A move out of a group a pump_all holds is refused (logged, then
  // aborted). Host thread, outside a pump.
  cvc::state_exec::exec_group *isolated_group() const;
  // Why the document stays in group 0: "" when it is isolated, else the first rule it failed
  // (logged when it changes, with the setting on).
  std::string isolation_reason() const;
  // A subscriber the host connected at or under `prefix` must run on the host thread
  // (AppRuntime::declare_thread_affine_prefix forwards here). An isolated document gets a
  // host-affinity group when a declared prefix is at, above or under its own: a write under the
  // document emits childChanged on every ancestor, so a subscriber above it runs on the writing
  // thread too. Before the document is loaded.
  void declare_thread_affine_prefix(const std::string &prefix);
  // The document's programs run host code inline that must stay on the host thread: a scripting
  // engine's activations (AppRuntime::use_script_engine declares it; start_process too, for a
  // process with natives of its own). An activation's natives run a slice of its language in the
  // calling step, charged to its process, under a frame budget the language's activations share.
  // An isolated document then gets a host-affinity group, as a host_only provider gives it. `why`
  // is kept for the first declaration. Like a late registration: decided again the first time
  // nothing of the document runs, at the latest at its next restart; before load() it takes effect
  // at the load.
  void require_host_thread(const std::string &why);
  // The document's programs this Runtime does not hold, which run elsewhere: AppRuntime::load
  // passes its init: (run_init) and its tests: (run_program). The isolation rules read them with
  // the document's own (S1-Q6). Each call replaces the last; before set_root.
  void set_document_programs(std::vector<std::string> programs);
  // The group the document's programs run in now, as isolated_group() last decided, without
  // deciding again or moving anything (AppRuntime::idle and wait_idle). Null in group 0.
  cvc::state_exec::exec_group *current_group() const;
  // The kernels still running in the groups a move retired (each goes at a drain once its count
  // is 0), and a wait until they have all ended: AppRuntime::idle and wait_idle count them too,
  // so a host does not let go of what an async function uses while one still runs. Host thread;
  // neither decides nor destroys anything.
  std::size_t retired_calls_in_flight() const;
  void wait_retired_calls();

  // Drain the §4 read-lane diagnostics accumulated during render() — one message per
  // distinct failing predicate (a parse error, a runtime error, or a step / time cap overrun;
  // not a per-frame budget skip, see set_reactive_frame_budget).
  // De-duplicated for the Runtime's lifetime, so a broken predicate warns ONCE however
  // many frames it renders. Empty when everything evaluated cleanly. The host may log
  // these (e.g. after the first frame); ignoring them is safe — the walk already
  // degraded fail-safe. Not part of render()'s hot path beyond a moved-out vector.
  std::vector<std::string> take_reactive_warnings();

  // ---- what this host provides (requires:, has?; cvc/ariadne/requires.h) ----
  // Whether a program could call `name` here: it resolves in the env this Runtime's programs run
  // in (the builtins, the stdlib, the state intrinsics and every register_action_intrinsics
  // provider, host verbs included), or it is a special form.
  bool has_function(const std::string &name) const;
  // Every name a program can reference here (functions and constants), sorted. Special forms are
  // not included.
  std::vector<std::string> program_names() const;
  // Whether a scene adapter realizes this document's scene: (the gl.scene capability). AppRuntime
  // sets it from its SceneAdapter.
  void set_scene_capable(bool on);
  // Whether that adapter's renderer runs the GLSL a document gives its scene (the gl.scene.shaders
  // capability; SceneAdapter::runs_shaders). AppRuntime sets it with set_scene_capable().
  void set_scene_shaders(bool on);
  // Whether this runtime runs a relay's server document (the relay capability). ServerDocument
  // sets it.
  void set_relay_document(bool on);
  // The facts the capability tokens read: the backend's Capabilities (all false without a
  // backend), set_scene_capable(), set_scene_shaders() and set_relay_document().
  HostFacts host_facts() const;
  // What (has? "name") answers: a capability token that holds here, a function a program can
  // call (has_function), or a registered widget type. Programs and reactive expressions both have
  // it, e.g. `visible: (has? "gl.scene")`.
  bool has(const std::string &name) const;
  // Forget the cached program names, so has_function() / has() see verbs registered since they
  // were last asked. AppRuntime::register_verb calls it; a new register_action_intrinsics provider
  // is noticed without it. Like a new provider, it has the placement decided again while nothing
  // of the document runs, else at its next restart (isolated_group).
  void refresh_program_names();

  // A function the read lane's expressions may call (visible_when, a computed bind:, §2.1) beside
  // its allowlist: a scripting engine's pure export (ScriptLane::export_pure). `fn` gets the
  // call's arguments and the read lane's environment, whose state readers note what the
  // expression reads, and runs inline in the expression's eval, under its step and time caps. It
  // must be pure: an expression's result is kept, and not run again, until a path it read through
  // those readers is written. A later registration of the name replaces the earlier one and drops
  // the kept results.
  using ReadFunction = std::function<cvc::state_exec::value_tag(
      std::span<const cvc::state_exec::value_tag> args, cvc::state_exec::environment &env)>;
  void register_read_function(const std::string &name, ReadFunction fn);

  // This document's per-resource scope (the se::document_scope installed on every action/resident
  // intrinsics_context as ictx.document). A host that contributes SCENE-BOUND intrinsics — whose
  // verb must reach live host objects a neutral register_action_intrinsics provider never sees (the
  // SceneGraph, a per-frame sink) — writes a NON-OWNING handle to those objects here, so the verb
  // reaches THIS document's host via ictx.document rather than capturing it in the app's provider,
  // which every document of the app shares. That routes correctly per document (two documents each
  // resolve their own handle) and cannot dangle across documents. Used by cvcGL's (gl-bind-stream)
  // verb (stream_verbs.h). Lives on the Impl, torn down with the Runtime; call between construction
  // and the first drain().
  cvc::state_exec::document_scope &document_scope();

private:
  struct Impl;
  std::unique_ptr<Impl> m_;
};

// The app frame its Runtimes' pumps are in: how many frame boundaries they have started (0 before
// the first pump). An app's Runtimes share it, so it moves once a frame however many of them drain.
std::uint64_t exec_frame(cvc::app &app);

// --- extensibility: custom widget types --------------------------------------
//
// Context handed to a custom-widget emit fn. It lets the fn COMPOSE built-in widgets
// (the core still owns ALL cvc::state binding), read live state (so the widget's
// structure can depend on state), and raise action events. It exposes no Backend and
// no toolkit — a custom widget is therefore backend-NEUTRAL: it works on ImGui, a
// terminal, wasm, and every future backend because it drives the same primitives the
// built-ins do.
struct WidgetEmitContext {
  // Emit a built-in child widget through the normal walk (binding handled by the
  // core) — e.g. a labelled vec3 editor = a Text + three slider_float children.
  std::function<void(const Widget &)> emit;
  // The current raw-string value at a bind path (resolved against the document prefix,
  // exactly as a bound widget resolves it); "<unset>" if the path has no value.
  std::function<std::string(const std::string &bind)> read;
  // Raise a named action event (drained on the host thread, like a Button's `on:`).
  std::function<void(const std::string &event)> fire;
};

// A custom widget type's emit fn: render the widget `w` (config in `w.props`, a
// cvc::ariadne::Value; bind `w.bind`; nested `w.children`) by composing built-in
// widgets through `ctx`. Runs each frame during the walk (immediate mode).
using WidgetEmitFn = std::function<void(const Widget &w, const WidgetEmitContext &ctx)>;

// Register (or replace) the emit fn for a custom widget `type` — a type the built-ins
// don't handle, which the loader carries as a Kind::Custom widget (custom_type +
// props). An unregistered custom type draws a labelled placeholder. Process-global and
// thread-safe; register before render(). Mirrors the register_scene_node_type /
// register_ari_block registries.
void register_widget_type(const std::string &type, WidgetEmitFn emit);

// Mark `type` as a KNOWN widget type that a BACKEND renders natively (e.g. ImGuiBackend's
// register_widget — a raw-ImGui widget like `stream_view`), so the loader does not warn that it is
// unregistered and has_widget_type() reports it present. No compositional emit fn is involved.
void register_known_widget_type(const std::string &type);

// Whether a custom widget `type` is registered — either a compositional emit fn
// (register_widget_type) or a backend-native widget (register_known_widget_type).
bool has_widget_type(const std::string &type);

// --- extensibility: the `init:` block (a state_exec script run on load) -------
//
// Run the .ari `init:` script (LoadResult::init_script) ONCE, scoped to `prefix` — so
// `(state-set "demo.n" "5")` writes `<prefix>.demo.n`, the SAME key a widget
// `bind: demo.n` resolves to (a state_exec chroot on the shared "." separator). Run it
// at load, BEFORE the first render(), so init values win and widget/scene read_or_seed
// defaults only fill keys init left unset. Returns true on success (or an empty
// script); false with a message appended to `errors` on a parse/runtime error. Never
// throws. (The loader stays app-free and only captures the script; this is the
// host/Runtime-side seam that has the app.)
// `has`, when set, binds (has? "name") for the script (AppRuntime passes Runtime::has).
bool run_init(cvc::app &app, const std::string &prefix, const std::string &script,
              std::vector<std::string> *errors = nullptr,
              const std::function<bool(const std::string &)> &has = {});

// ---------------------------------------------------------------------------
// Host-contributed state_exec intrinsics for the PROGRAM lanes (init: and a program `on:` action).
// A host binds native functions — e.g. nav verbs `(nav-step)` / `(nav-arrived)` — that a .ari
// program may call, so a program can invoke a host capability AND compute over live host state
// inline
// (`(if (> (nav-arrived) 5) …)`). The provider is invoked when each program-lane environment is
// built, AFTER the standard register_intrinsics, so it registers its fns via
// cvc::state_exec::builtins::register_fn(env, name, fn) on the given env. `ictx` is that lane's
// context (its chrooted state root / scheduler). Each app has its own providers; register at
// setup, before load_*.
//
// NOTE the boundaries: this is ONLY the full-env program lanes (init:/action on:), never the
// per-frame reactive READ lane (visible_when/computed binds — kept default-deny +
// side-effect-free). A program on: runs synchronously on the host thread inside Runtime::drain(),
// so an intrinsic that mutates async host state (a worker-thread sim) should ENQUEUE the mutation,
// not do it inline.
// ---------------------------------------------------------------------------
} // namespace ariadne
namespace state_exec {
struct environment; // NB: struct, not class — MSVC encodes the class/struct tag in the mangled
struct intrinsics_context; // name (types.h/intrinsics.h declare both as struct); a mismatch =
                           // LNK2019 on Windows (Itanium ABI ignores the tag, so it hides on
                           // Linux/macOS).
enum class provider_affinity; // intrinsics.h: any, host_proxied, host_only
} // namespace state_exec
namespace ariadne {

using ActionIntrinsicProvider = std::function<void(
    std::shared_ptr<cvc::state_exec::environment> env, cvc::state_exec::intrinsics_context &ictx)>;

// Register a provider of host intrinsics for `app`'s program lanes. Providers run in order. The
// list is the app's own (VOLROVER-LIBRARY-PORTS-ROADMAP.md 1.3): another app's documents never
// get them, and they go with the app. Without a key it appends. With a non-empty key it replaces
// the provider registered under that key, and its kind, in its place, so a host that registers the
// same stateless provider each time it starts (a cvcGL runtime) keeps one. The forms without a kind
// register the provider host_only.
void register_action_intrinsics(cvc::app &app, ActionIntrinsicProvider provider);
void register_action_intrinsics(cvc::app &app, const std::string &key,
                                ActionIntrinsicProvider provider);
// The same, with the kind of every native the provider registers (STATE_EXEC_THREADED.md 4.9): an
// isolated document's group installs them by it. A provider installed in an `any` group must also
// have a registration body safe to run on a worker, since a spawned child runs it there. A
// host_proxied provider binds synchronous natives only (an async one would run its kernel inline on
// the caller, uncharged). What a provider binds that is not a native is made fresh for each env, or
// is immutable: two groups' programs must never share a mutable value.
void register_action_intrinsics(cvc::app &app, ActionIntrinsicProvider provider,
                                cvc::state_exec::provider_affinity affinity);
void register_action_intrinsics(cvc::app &app, const std::string &key,
                                ActionIntrinsicProvider provider,
                                cvc::state_exec::provider_affinity affinity);
// Remove all of `app`'s providers (a host tears down before its captured context dies; also for
// tests).
void clear_action_intrinsics(cvc::app &app);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_H
