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

#include <cvc/ariadne/backend.h>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

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
  bool begin_window(const char *title, const char *id, const cvc::ariadne::Size &size,
                    float border) override;
  void end_window() override;
  bool begin_grid(const cvc::ariadne::Layout &layout, const char *id) override;
  void grid_next_cell() override;
  void end_grid() override;
  void begin_disabled() override; // §4: ImGui::BeginDisabled (greyed, non-interactive)
  void end_disabled() override;
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

  // Novel-primitive custom widgets (§16.1b): a host registers a raw-ImGui draw fn for
  // a custom_type; custom_widget() dispatches to it. The draw fn gets the bound value
  // as a string (value-in) and the Widget (props/label) and returns the edit; the
  // Ariadne core owns the cvc::state read/write. An unregistered type -> {handled:
  // false} -> the core draws a placeholder. Use this for widgets the compositional
  // register_widget_type can't express (a colour wheel, a shader canvas).
  using CustomDrawFn = std::function<cvc::ariadne::CustomEdit(const std::string &current,
                                                              const cvc::ariadne::Widget &w)>;
  void register_widget(std::string custom_type, CustomDrawFn draw);
  cvc::ariadne::CustomEdit custom_widget(const char *type, const std::string &current,
                                         const cvc::ariadne::Widget &w) override;

  // Convenience: install `rt`'s per-frame walk as `overlay`'s draw callback, so
  // VTK drives Runtime::render() once per rendered frame. Call after
  // rt.set_backend(this). `rt` must outlive the overlay's callback.
  void install(cvc::ariadne::Runtime &rt, ImGuiOverlay &overlay);

private:
  // ImGui style vars pushed per open window (WindowBorderSize, §3.0.3b), popped
  // in end_window. A stack so window nesting is correct.
  std::vector<int> m_windowStylePushes;

  // Per open grid (§3.0.3b). A grid is realized either as an ImGui table (columns
  // + optional per-row min-heights) or, when resizable with row tracks, as a
  // vertical split-pane stack with draggable seams (row splitter). Held on a
  // stack so nested grids compose. No ImGui types here (backend-header-clean).
  struct GridState {
    bool split = false; // split-pane mode vs table mode
    int cols = 1;
    int cell = 0;                 // cells emitted so far
    std::vector<float> row_px;    // resolved row heights (px; 0 = auto/fit)
    std::vector<float *> pane_h;  // split mode: pointers to each pane's live height (ImGui storage)
    float split_long_axis = 0.0f; // split mode: the seam's cross length
    int color_pushes = 0;         // table border-colour style pushes to pop
  };
  std::vector<GridState> m_grids;

  // Host-registered raw-ImGui draw fns for custom widget types (register_widget).
  std::unordered_map<std::string, CustomDrawFn> m_customWidgets;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_IMGUIBACKEND_H
