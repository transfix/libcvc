#ifndef CVC_GL_ARIADNE_H
#define CVC_GL_ARIADNE_H

// Ariadne — the cvcGL UI & scene DSL (cvc::gl::ariadne).
//
// Runtime is the loader/host for a retained widget tree (widget.h): it installs
// a single per-frame draw callback on an ImGuiOverlay that WALKS the tree and
// re-emits Dear ImGui immediate-mode calls, binding widgets to cvc::state via
// the cvc::gl::ui:: helpers. Actions raised by buttons / menu items are QUEUED
// as named events and run OFF the draw callback by Runtime::drain(), the P0 form
// of the deferred intent buffer in the roadmap (§7.2/§4.7) — nothing
// side-effectful runs inside the mid-render walk.
//
// Threading / context: the walk runs inside VTK's render pass on the render
// thread (ImGuiOverlay's draw callback). Ariadne is compiled INTO cvcGL, so it
// shares cvcGL's Dear ImGui context — no SetCurrentContext dance is needed for
// its own ImGui::* calls (that dance is only for a SEPARATE binary calling raw
// ImGui::* — see ImGuiOverlay::imguiContext()).
//
// This is the P0 slice; the full contract (dynamic-DOM reconcile, expressions,
// scene binding, validation) is docs/roadmap/CVCGL-UI-DSL-ROADMAP.md.

#include <functional>
#include <memory>
#include <string>

#include <cvc/gl/ariadne/widget.h>

namespace cvc {
class app;
}
namespace cvc {
namespace gl {
class ImGuiOverlay;
}
} // namespace cvc

namespace cvc {
namespace gl {
namespace ariadne {

class Runtime {
public:
  // `prefix` is the cvc::state prefix that relative bind paths splice onto
  // (e.g. a SceneGraph's getStatePrefix(), or a "ui.docs.<doc>" subtree). A bind
  // path with a leading '/' is resolved app-root-absolute instead. Empty prefix
  // = binds are used as-is (app-root-relative).
  Runtime(cvc::app &app, ImGuiOverlay &overlay, std::string prefix = std::string());
  ~Runtime();

  Runtime(const Runtime &) = delete;
  Runtime &operator=(const Runtime &) = delete;

  // Set/replace the widget tree. The swap is applied at the next frame boundary
  // (before the walk), never mid-walk — the P0 form of the §11.5.1 reconcile
  // commit boundary. Safe to call from the host thread between frames.
  void set_root(Widget root);

  // Install the per-frame walk as the overlay's draw callback. Call once after
  // set_root(); replaces any existing draw callback on the overlay.
  void install();

  // Register a host handler for an action event name (a Button / MenuItemAction
  // `on:`). The handler runs on the host thread inside drain(), never in the walk.
  void on(std::string event, std::function<void()> handler);

  // Run every queued action event's handler, then clear the queue. Call once per
  // frame in the host loop, BEFORE your sim tick and OUTSIDE the draw callback.
  void drain();

private:
  struct Impl;
  std::unique_ptr<Impl> m_;
};

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_H
