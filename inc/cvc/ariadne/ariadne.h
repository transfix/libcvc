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

#include <cvc/ariadne/input.h> // InputEvent — Runtime::post_input feeds on_key/on_pointer (§4.6)
#include <cvc/ariadne/widget.h>
#include <functional>
#include <memory>
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
} // namespace state_exec
} // namespace cvc

namespace cvc {
namespace ariadne {

class Runtime {
public:
  // `prefix` is the cvc::state prefix that relative bind paths splice onto
  // (e.g. a SceneGraph's getStatePrefix(), or a "ui.docs.<doc>" subtree). A bind
  // path with a leading '/' is resolved app-root-absolute instead. Empty prefix
  // = binds are used as-is (app-root-relative).
  explicit Runtime(cvc::app &app, std::string prefix = std::string());
  ~Runtime();

  Runtime(const Runtime &) = delete;
  Runtime &operator=(const Runtime &) = delete;

  // Choose the surface. Must be set before render(); the Runtime does not own
  // the Backend (it must outlive the Runtime).
  void set_backend(Backend *backend);

  // Set/replace the widget tree. The swap is applied at the next frame boundary
  // (the top of render(), before any node is emitted), never mid-walk — the P0
  // form of the §11.5.1 reconcile commit boundary. Safe to call from the host
  // thread between frames.
  void set_root(Widget root);

  // Register a host handler for an action event name (a Button / MenuItemAction
  // `on:`). The handler runs on the host thread inside drain(), never in the walk.
  void on(std::string event, std::function<void()> handler);

  // §7.1 resident on:tick handler — register a state_exec PROGRAM that runs ONCE PER FRAME.
  // Unlike a fire-once action `on:`, this submits ONE long-lived, owner-tagged process on the
  // app-wide scheduler that PARKS between frames (it never re-submits per frame); each drain()
  // posts it a tick to run the program once. Pass a LoadResult::on_tick_script here after loading.
  // "" clears it (kills the resident). The resident is reaped with the Runtime (kill_owner on
  // teardown).
  void set_tick_program(std::string script);

  // §4.6 document-level input handlers — register a state_exec PROGRAM run per input event. Like
  // on:tick, each is ONE long-lived resident (owner-tagged, reaped with the Runtime); it parks and
  // is woken by post_input with the event bound as `event`, which the body reads via
  // (get-attr event "key"/"kind"/"x"/"y"/"button"/"mods"/"dx"/"dy"/"clicks"/"repeat").
  // set_key_program handles keyboard events (key_down/key_up); set_pointer_program handles mouse
  // events (mouse_move/mouse_button_*/mouse_wheel). Pass LoadResult::on_key_script /
  // on_pointer_script.
  // "" clears it. (Widget-level on_click/on_hover are a separate path.)
  void set_key_program(std::string script);
  void set_pointer_program(std::string script);

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
  // on_key/on_pointer residents this same frame (drain() drains the ingress + pumps). Thread-safe
  // (routes through the scheduler's MPSC ingress). A no-op if no matching handler is registered.
  void post_input(const InputEvent &ev);

  // Render one frame: apply any pending tree swap, then walk the tree driving the
  // Backend and binding cvc::state. Called by the active backend (from the host
  // render pass, or the backend's own loop). A pure reader of state that only
  // ENQUEUES action intents — it never runs a handler or reloads a document.
  void render();

  // Run every queued action event's handler, then clear the queue. Call once per
  // frame in the host loop, BEFORE your sim tick and OUTSIDE render().
  void drain();

  // Drain the §4 read-lane diagnostics accumulated during render() — one message per
  // distinct failing predicate (a parse error, a runtime error, or a per-frame budget
  // overrun).
  // De-duplicated for the Runtime's lifetime, so a broken predicate warns ONCE however
  // many frames it renders. Empty when everything evaluated cleanly. The host may log
  // these (e.g. after the first frame); ignoring them is safe — the walk already
  // degraded fail-safe. Not part of render()'s hot path beyond a moved-out vector.
  std::vector<std::string> take_reactive_warnings();

  // This document's per-resource scope (the se::document_scope installed on every action/resident
  // intrinsics_context as ictx.document). A host that contributes SCENE-BOUND intrinsics — whose
  // verb must reach live host objects a neutral register_action_intrinsics provider never sees (the
  // SceneGraph, a per-frame sink) — writes a NON-OWNING handle to those objects here, so the verb
  // reaches THIS document's host via ictx.document rather than capturing it in the process-global
  // provider. That routes correctly per document (two documents each resolve their own handle) and
  // cannot dangle across documents. Used by cvcGL's (gl-bind-stream) verb (stream_verbs.h). Lives
  // on the Impl, torn down with the Runtime; call between construction and the first drain().
  cvc::state_exec::document_scope &document_scope();

private:
  struct Impl;
  std::unique_ptr<Impl> m_;
};

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

// Whether a custom widget `type` has a registered emit fn (test/introspection).
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
bool run_init(cvc::app &app, const std::string &prefix, const std::string &script,
              std::vector<std::string> *errors = nullptr);

// ---------------------------------------------------------------------------
// Host-contributed state_exec intrinsics for the PROGRAM lanes (init: and a program `on:` action).
// A host binds native functions — e.g. nav verbs `(nav-step)` / `(nav-arrived)` — that a .ari
// program may call, so a program can invoke a host capability AND compute over live host state
// inline
// (`(if (> (nav-arrived) 5) …)`). The provider is invoked when each program-lane environment is
// built, AFTER the standard register_intrinsics, so it registers its fns via
// cvc::state_exec::builtins::register_fn(env, name, fn) on the given env. `ictx` is that lane's
// context (its chrooted state root / scheduler). Process-global; register at setup, before load_*.
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
} // namespace state_exec
namespace ariadne {

using ActionIntrinsicProvider = std::function<void(
    std::shared_ptr<cvc::state_exec::environment> env, cvc::state_exec::intrinsics_context &ictx)>;

// Register a provider of host intrinsics for the program lanes (append; providers run in order).
void register_action_intrinsics(ActionIntrinsicProvider provider);
// Remove all registered providers (a host tears down before its captured context dies; also for
// tests).
void clear_action_intrinsics();

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_H
