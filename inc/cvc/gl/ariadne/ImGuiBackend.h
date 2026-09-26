#ifndef CVC_GL_ARIADNE_IMGUIBACKEND_H
#define CVC_GL_ARIADNE_IMGUIBACKEND_H

// ImGuiBackend — the reference Ariadne backend: renders a cvc::ariadne Widget
// tree as Dear ImGui, over cvcGL's VTK canvas (roadmap §16.1). This is the ONE
// place ImGui/VTK is touched; the Ariadne core (cvc::ariadne) stays pure libcvc.
//
// It is a guest-mode backend: VTK owns the vtkRenderWindow + the single GL
// context, and ImGui draws inside VTK's RenderEvent framebuffer via ImGuiOverlay
// (owns_loop == false — the host render pass drives Runtime::render()). Use
// install() to wire an overlay's per-frame draw callback to a Runtime.
//
// It degrades to safe no-ops when libcvc is built without CVC_ENABLE_IMGUI
// (every draw becomes an inert stub, mirroring the cvc::gl::ui:: layer).

#include <string>
#include <vector>

#include <cvc/ariadne/backend.h>

namespace cvc {
namespace ariadne {
class Runtime;
}
namespace gl {
class ImGuiOverlay;

class ImGuiBackend : public cvc::ariadne::Backend {
public:
  ImGuiBackend() = default;

  cvc::ariadne::Capabilities capabilities() const override;

  bool begin_main_menu_bar() override;
  void end_main_menu_bar() override;
  bool begin_menu(const char *label) override;
  void end_menu() override;
  bool begin_window(const char *title, const char *id) override;
  void end_window() override;
  void push_id(const char *id) override;
  void pop_id() override;

  void text_line(const char *text) override;
  void text_value(const char *label, const std::string &value) override;
  void separator() override;
  bool button(const char *label) override;
  bool menu_item_action(const char *label) override;

  cvc::ariadne::BoolEdit menu_item_toggle(const char *label, bool current) override;
  cvc::ariadne::BoolEdit checkbox(const char *label, bool current) override;
  cvc::ariadne::IntEdit slider_int(const char *label, const char *key, int current, int lo,
                                   int hi) override;
  cvc::ariadne::DoubleEdit slider_double(const char *label, const char *key, double current,
                                         double lo, double hi, const char *fmt) override;
  cvc::ariadne::IndexEdit combo(const char *label, int current_index,
                                const std::vector<std::string> &options) override;

  // Convenience: install `rt`'s per-frame walk as `overlay`'s draw callback, so
  // VTK drives Runtime::render() once per rendered frame. Call after
  // rt.set_backend(this). `rt` must outlive the overlay's callback.
  void install(cvc::ariadne::Runtime &rt, ImGuiOverlay &overlay);
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_IMGUIBACKEND_H
