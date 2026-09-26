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

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <cvc/ariadne/widget.h>

namespace cvc {
class app;
namespace ariadne {
class Backend;
}
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

  // Render one frame: apply any pending tree swap, then walk the tree driving the
  // Backend and binding cvc::state. Called by the active backend (from the host
  // render pass, or the backend's own loop). A pure reader of state that only
  // ENQUEUES action intents — it never runs a handler or reloads a document.
  void render();

  // Run every queued action event's handler, then clear the queue. Call once per
  // frame in the host loop, BEFORE your sim tick and OUTSIDE render().
  void drain();

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
// script); false with a message appended to `errors` on a parse/runtime error, or when
// this build lacks state_exec. Never throws. (The loader stays app-free and only
// captures the script; this is the host/Runtime-side seam that has the app.)
bool run_init(cvc::app &app, const std::string &prefix, const std::string &script,
              std::vector<std::string> *errors = nullptr);

// Whether this build has state_exec (CVC_STATE_EXEC). When false, run_init cannot
// execute a non-empty init: script (it reports an error instead).
bool have_state_exec();

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_H
