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
#include <cvc/image/image.h> // set_image publishes a cvc::image for the Kind::Image raster viewer
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
  void set_tooltip(const char *text) override; // §4: hover tooltip on the last item
  void push_id(const char *id) override;
  void pop_id() override;

  void text_line(const char *text) override;
  void text_value(const char *label, const std::string &value) override;
  void separator() override;
  bool button(const char *label) override;
  bool menu_item_action(const char *label) override;
  bool item_clicked() override; // §4.6 widget on_click: ImGui::IsItemClicked() on the last item

  cvc::ariadne::BoolEdit menu_item_toggle(const char *label, bool current) override;
  cvc::ariadne::BoolEdit checkbox(const char *label, bool current) override;
  cvc::ariadne::IntEdit slider_int(const char *label, const char *key, int current, int lo,
                                   int hi) override;
  cvc::ariadne::DoubleEdit slider_double(const char *label, const char *key, double current,
                                         double lo, double hi, const char *fmt) override;
  cvc::ariadne::IndexEdit combo(const char *label, int current_index,
                                const std::vector<std::string> &options) override;
  cvc::ariadne::ColorEdit color(const char *label, const float rgb[3]) override; // §G7 RGB picker

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

  // Publish/update a named image the Kind::Image raster viewer displays (a nav grid colorized to a
  // cvc::image, updated each frame). The image is copied (COW) and uploaded to a GL texture lazily
  // inside draw_image (on the render thread, where the GL context is current). Call it from the
  // host loop; draw_image references it by the name a `- image:` widget's src/bind resolves to.
  void set_image(const std::string &name, const cvc::image &img);
  bool draw_image(const char *name, float width) override; // §raster viewer (ImGui::Image over GL)

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

  // Host-published images (Kind::Image). Each keeps its source cvc::image + a lazily-created GL
  // texture; `dirty` forces a re-upload after set_image. Textures are process-lifetime (freed at
  // exit) — deleting them would need a live GL context the destructor cannot assume.
  struct PublishedImage {
    cvc::image img;
    unsigned int tex = 0; // GL texture id (0 = not yet created)
    bool dirty = true;    // needs (re-)upload
  };
  std::unordered_map<std::string, PublishedImage> m_images;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_IMGUIBACKEND_H
