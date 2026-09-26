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

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_H
