// ImGuiBackend — the reference Ariadne backend (see the header). Renders a
// cvc::ariadne Widget tree as Dear ImGui over cvcGL's VTK canvas.
//
// Structure and discrete widgets go through the cvc::gl::ui:: pointer-free subset
// (which already degrades to no-ops without CVC_ENABLE_IMGUI). The two things ui::
// does not expose in value-in/edit-out form — the commit-on-release slider (a
// per-widget edit cache + the IsItemDeactivatedAfterEdit edge) and the value-form
// combo (BeginCombo) — touch raw ImGui directly, guarded by CVC_ENABLE_IMGUI with
// inert stubs otherwise. The commit policy is faithful to cvc::gl::ui::Slider*:
// a drag edits a cache every frame and reports `committed` only on release, so
// the core writes cvc::state once per drag, not once per frame.

#include <cvc/gl/ariadne/ImGuiBackend.h>

#include <cvc/ariadne/ariadne.h>
#include <cvc/gl/ImGuiBinding.h> // cvc::gl::ui::*
#include <cvc/gl/ImGuiOverlay.h>

#include <cfloat>
#include <string>
#include <vector>

#ifdef CVC_ENABLE_IMGUI
#define IMGUI_DEFINE_MATH_OPERATORS // must precede imgui.h (imgui_internal asserts it)
#include <imgui.h>
#include <imgui_internal.h> // ImHashStr: a stable per-key slot in the window's storage
#endif

namespace cvc {
namespace gl {

// NOTE: no `namespace ui = cvc::gl::ui;` alias — we are already inside `cvc::gl`, so `ui::`
// resolves to the nested `cvc::gl::ui` namespace directly. Aliasing a name that already names
// a namespace in the same scope is a redefinition that Clang rejects (macOS CI), though GCC
// tolerates it. `ariadne` below IS a real alias: there is no `cvc::gl::ariadne` in this TU.
namespace ariadne = cvc::ariadne;

#ifdef CVC_ENABLE_IMGUI
namespace {
// Per-key edit cache for continuous widgets, held in ImGui's current-window
// storage (so it lives and dies with the window, and the same key in two windows
// keeps two independent caches). While the widget is NOT active it follows the
// incoming value, so an external write (script / config / replicated peer) moves
// the slider; while active it holds the in-progress edit. Salted so a key can
// never collide with one of ImGui's own storage keys. Mirrors cvc::gl::ui.
constexpr ImGuiID kCacheSalt = 0x63766367; // 'cvcg'

float *cache_float(const char *key, float seed, bool active) {
  ImGuiStorage *store = ImGui::GetStateStorage();
  const ImGuiID id = ImHashStr(key, 0, kCacheSalt);
  float *slot = store->GetFloatRef(id, seed);
  if (!active)
    *slot = seed;
  return slot;
}
int *cache_int(const char *key, int seed, bool active) {
  ImGuiStorage *store = ImGui::GetStateStorage();
  const ImGuiID id = ImHashStr(key, 0, kCacheSalt);
  int *slot = store->GetIntRef(id, seed);
  if (!active)
    *slot = seed;
  return slot;
}

// Resolve a §3.0.3b track to pixels: px verbatim, percent as a fraction of the
// parent extent, auto -> 0 (fit / default).
float resolve_track(const ariadne::Track &t, float parent) {
  if (t.unit == ariadne::Unit::Px)
    return t.value;
  if (t.unit == ariadne::Unit::Percent)
    return parent * t.value / 100.0f;
  return 0.0f;
}

// A horizontal drag seam between two stacked panes (the classic ImGui splitter
// idiom, §3.0.3b "rows are a manual splitter"): dragging adjusts *a and *b.
bool row_splitter(float thickness, float *a, float *b, float min_a, float min_b, float long_axis) {
  ImGuiContext &g = *GImGui;
  ImGuiWindow *win = g.CurrentWindow;
  const ImGuiID id = win->GetID("##cvcg.rowsplit");
  ImRect bb;
  bb.Min = win->DC.CursorPos;
  bb.Max = ImVec2(bb.Min.x + long_axis, bb.Min.y + thickness);
  return ImGui::SplitterBehavior(bb, id, ImGuiAxis_Y, a, b, min_a, min_b, 0.0f);
}
} // namespace
#endif

ariadne::Capabilities ImGuiBackend::capabilities() const {
  ariadne::Capabilities c;
  c.windows = true;
  c.menubar = true;
  c.mouse = true;
  c.keyboard = true;
  c.color = true;
  c.glsl = true;       // cvcGL can render scene-node shaders / a shader_canvas
  c.view_embed = true; // render-to-texture views (roadmap §9.6)
  c.owns_loop = false; // guest mode: VTK's render pass drives Runtime::render()
  return c;
}

bool ImGuiBackend::begin_main_menu_bar() { return ui::BeginMainMenuBar(); }
void ImGuiBackend::end_main_menu_bar() { ui::EndMainMenuBar(); }
bool ImGuiBackend::begin_menu(const char *label) { return ui::BeginMenu(label); }
void ImGuiBackend::end_menu() { ui::EndMenu(); }

bool ImGuiBackend::begin_window(const char *title, const char *id, const ariadne::Size &size,
                                float border) {
  int pushes = 0;
#ifdef CVC_ENABLE_IMGUI
  // Resolve the §3.0.3b size against the main viewport's WORK area (the menu-aware
  // content rect), per axis: px verbatim, percent as a fraction of the work size,
  // auto -> 0 (let ImGui fit). min/max become window size constraints.
  const ImGuiViewport *vp = ImGui::GetMainViewport();
  const ImVec2 avail = vp->WorkSize;
  auto px = [](const ariadne::Extent &e, float parent) -> float {
    if (e.unit == ariadne::Unit::Px)
      return e.value;
    if (e.unit == ariadne::Unit::Percent)
      return parent * e.value / 100.0f;
    return 0.0f; // Auto
  };
  if (size.w.is_set() || size.h.is_set())
    ImGui::SetNextWindowSize(ImVec2(px(size.w, avail.x), px(size.h, avail.y)),
                             ImGuiCond_FirstUseEver);
  if (size.min_w.is_set() || size.min_h.is_set() || size.max_w.is_set() || size.max_h.is_set()) {
    const ImVec2 mn(size.min_w.is_set() ? px(size.min_w, avail.x) : 0.0f,
                    size.min_h.is_set() ? px(size.min_h, avail.y) : 0.0f);
    const ImVec2 mx(size.max_w.is_set() ? px(size.max_w, avail.x) : FLT_MAX,
                    size.max_h.is_set() ? px(size.max_h, avail.y) : FLT_MAX);
    ImGui::SetNextWindowSizeConstraints(mn, mx);
  }
  if (border >= 0.0f) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, border);
    ++pushes;
  }
#else
  (void)size;
  (void)border;
#endif
  // Stable ImGui identity via the "Visible title###stable.id" trick (roadmap §3.3).
  std::string name = title;
  name += "###";
  name += id;
  const bool vis = ui::Begin(name.c_str());
  m_windowStylePushes.push_back(pushes);
  return vis;
}

void ImGuiBackend::end_window() {
  ui::End(); // unconditional per ImGui's contract
  if (!m_windowStylePushes.empty()) {
    const int n = m_windowStylePushes.back();
    m_windowStylePushes.pop_back();
#ifdef CVC_ENABLE_IMGUI
    if (n)
      ImGui::PopStyleVar(n);
#else
    (void)n;
#endif
  }
}

bool ImGuiBackend::begin_grid(const ariadne::Layout &layout, const char *id) {
#ifdef CVC_ENABLE_IMGUI
  GridState gs;
  gs.cols = layout.col_widths.empty() ? 1 : static_cast<int>(layout.col_widths.size());
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const float availH = avail.y > 0.0f ? avail.y : ImGui::GetMainViewport()->WorkSize.y;
  const float availW = avail.x > 0.0f ? avail.x : ImGui::GetMainViewport()->WorkSize.x;
  for (const ariadne::Track &t : layout.row_heights)
    gs.row_px.push_back(resolve_track(t, availH));

  // Resizable rows → a vertical split-pane stack with draggable seams (§3.0.3b).
  // Pane heights persist across frames in ImGui window storage, seeded from the
  // row tracks (cross-reload persistence to tree.<id> is the §11.4 follow-up).
  if (layout.is_row_split()) {
    gs.split = true;
    gs.split_long_axis = availW;
    ImGuiStorage *store = ImGui::GetStateStorage();
    const std::string base = std::string("cvcg.split.") + (id ? id : "");
    for (std::size_t i = 0; i < gs.row_px.size(); ++i) {
      const ImGuiID key = ImHashStr((base + "." + std::to_string(i)).c_str(), 0, kCacheSalt);
      gs.pane_h.push_back(store->GetFloatRef(key, gs.row_px[i] > 0.0f ? gs.row_px[i] : 80.0f));
    }
    m_grids.push_back(std::move(gs));
    return true; // the panes themselves open in grid_next_cell
  }

  // Table mode: columns sized by col_widths, optional per-row min-heights.
  ImGuiTableFlags flags = ImGuiTableFlags_SizingStretchProp;
  if (layout.resizable)
    flags |= ImGuiTableFlags_Resizable; // native column drag-resize (§3.0.3b)
  switch (layout.borders) {
  case ariadne::BorderShow::Inner:
    flags |= ImGuiTableFlags_BordersInner;
    break;
  case ariadne::BorderShow::Outer:
    flags |= ImGuiTableFlags_BordersOuter;
    break;
  case ariadne::BorderShow::All:
    flags |= ImGuiTableFlags_Borders;
    break;
  default:
    break;
  }
  if (layout.has_border_color) {
    const ImVec4 c(layout.border_color[0], layout.border_color[1], layout.border_color[2],
                   layout.border_color[3]);
    ImGui::PushStyleColor(ImGuiCol_TableBorderLight, c);
    ImGui::PushStyleColor(ImGuiCol_TableBorderStrong, c);
    gs.color_pushes = 2;
  }
  std::string tid = "###grid.";
  tid += (id ? id : "");
  const bool open = ImGui::BeginTable(tid.c_str(), gs.cols, flags);
  if (open) {
    for (int i = 0; i < gs.cols; ++i) {
      ImGuiTableColumnFlags cf = ImGuiTableColumnFlags_WidthStretch;
      float w = 1.0f;
      if (i < static_cast<int>(layout.col_widths.size())) {
        const ariadne::Track &t = layout.col_widths[i];
        if (t.unit == ariadne::Unit::Px) {
          cf = ImGuiTableColumnFlags_WidthFixed;
          w = t.value;
        } else if (t.unit == ariadne::Unit::Percent) {
          cf = ImGuiTableColumnFlags_WidthStretch;
          w = t.value;
        } else {
          cf = ImGuiTableColumnFlags_WidthFixed;
          w = 0.0f; // Auto → fit content
        }
      }
      ImGui::TableSetupColumn("", cf, w);
    }
    m_grids.push_back(std::move(gs));
  } else if (gs.color_pushes) {
    ImGui::PopStyleColor(gs.color_pushes); // no EndTable when BeginTable() is false
  }
  return open;
#else
  (void)layout;
  (void)id;
  return true; // stub: emit the children as a plain vertical flow
#endif
}

void ImGuiBackend::grid_next_cell() {
#ifdef CVC_ENABLE_IMGUI
  if (m_grids.empty())
    return;
  GridState &g = m_grids.back();
  if (g.split) {
    const int i = g.cell;
    if (i > 0) {
      ImGui::EndChild(); // close the previous pane
      if (i - 1 < static_cast<int>(g.pane_h.size()) && i < static_cast<int>(g.pane_h.size()))
        row_splitter(4.0f, g.pane_h[i - 1], g.pane_h[i], 24.0f, 24.0f, g.split_long_axis);
    }
    const float h = (i < static_cast<int>(g.pane_h.size())) ? *g.pane_h[i] : 0.0f;
    const std::string pid = "##cvcg.pane" + std::to_string(i);
    ImGui::BeginChild(pid.c_str(), ImVec2(0.0f, h), ImGuiChildFlags_Borders);
    ++g.cell;
    return;
  }
  // Table mode: at each new row, honour the row's min-height.
  const int col = g.cell % g.cols;
  const int row = g.cell / g.cols;
  if (col == 0) {
    const float h = (row < static_cast<int>(g.row_px.size())) ? g.row_px[row] : 0.0f;
    ImGui::TableNextRow(ImGuiTableRowFlags_None, h);
  }
  ImGui::TableSetColumnIndex(col);
  ++g.cell;
#endif
}

void ImGuiBackend::end_grid() {
#ifdef CVC_ENABLE_IMGUI
  if (m_grids.empty())
    return;
  const GridState g = m_grids.back();
  m_grids.pop_back();
  if (g.split) {
    if (g.cell > 0)
      ImGui::EndChild(); // close the last pane
  } else {
    ImGui::EndTable();
    if (g.color_pushes)
      ImGui::PopStyleColor(g.color_pushes);
  }
#endif
}
void ImGuiBackend::push_id(const char *id) { ui::PushId(id); }
void ImGuiBackend::pop_id() { ui::PopId(); }

void ImGuiBackend::text_line(const char *text) { ui::TextLine(text); }
void ImGuiBackend::text_value(const char *label, const std::string &value) {
  std::string line = label;
  line += ": ";
  line += value;
  ui::TextLine(line.c_str());
}
void ImGuiBackend::separator() { ui::Separator(); }
bool ImGuiBackend::button(const char *label) { return ui::Button(label); }
bool ImGuiBackend::menu_item_action(const char *label) { return ui::MenuItemClicked(label); }

ariadne::BoolEdit ImGuiBackend::menu_item_toggle(const char *label, bool current) {
  const bool next = ui::MenuItemToggle(label, current); // discrete: commit immediately
  const bool changed = next != current;
  return {changed, changed, next};
}

ariadne::BoolEdit ImGuiBackend::checkbox(const char *label, bool current) {
  const bool next = ui::CheckboxValue(label, current); // discrete: commit immediately
  const bool changed = next != current;
  return {changed, changed, next};
}

ariadne::IntEdit ImGuiBackend::slider_int(const char *label, const char *key, int current, int lo,
                                          int hi) {
#ifdef CVC_ENABLE_IMGUI
  int &v = *cache_int(key, current, ImGui::IsAnyItemActive());
  const bool changed = ImGui::SliderInt(label, &v, lo, hi);
  const bool committed = ImGui::IsItemDeactivatedAfterEdit(); // one write, on release
  return {changed, committed, v};
#else
  (void)label;
  (void)key;
  (void)lo;
  (void)hi;
  return {false, false, current};
#endif
}

ariadne::DoubleEdit ImGuiBackend::slider_double(const char *label, const char *key, double current,
                                                double lo, double hi, const char *fmt) {
#ifdef CVC_ENABLE_IMGUI
  float &v = *cache_float(key, static_cast<float>(current), ImGui::IsAnyItemActive());
  const bool changed =
      ImGui::SliderFloat(label, &v, static_cast<float>(lo), static_cast<float>(hi), fmt);
  const bool committed = ImGui::IsItemDeactivatedAfterEdit();
  return {changed, committed, static_cast<double>(v)};
#else
  (void)label;
  (void)key;
  (void)lo;
  (void)hi;
  (void)fmt;
  return {false, false, current};
#endif
}

ariadne::IndexEdit ImGuiBackend::combo(const char *label, int current_index,
                                       const std::vector<std::string> &options) {
  ariadne::IndexEdit e;
  e.index = current_index;
  if (options.empty())
    return e;
#ifdef CVC_ENABLE_IMGUI
  const int idx =
      (current_index >= 0 && current_index < static_cast<int>(options.size())) ? current_index : 0;
  if (ImGui::BeginCombo(label, options[idx].c_str())) {
    for (std::size_t i = 0; i < options.size(); ++i) {
      const bool sel = (static_cast<int>(i) == idx);
      if (ImGui::Selectable(options[i].c_str(), sel)) { // discrete: commit on pick
        e.index = static_cast<int>(i);
        e.changed = true;
        e.committed = true;
      }
      if (sel)
        ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  }
#else
  (void)label;
#endif
  return e;
}

void ImGuiBackend::register_widget(std::string custom_type, CustomDrawFn draw) {
  if (draw)
    m_customWidgets[std::move(custom_type)] = std::move(draw);
}

ariadne::CustomEdit ImGuiBackend::custom_widget(const char *type, const std::string &current,
                                                const cvc::ariadne::Widget &w) {
  // Dispatch to the host's registered raw-ImGui draw fn; unknown type -> not handled,
  // so the core draws a placeholder. The draw fn (in the host TU) does the ImGui work.
  const auto it = m_customWidgets.find(type ? type : "");
  if (it == m_customWidgets.end() || !it->second)
    return {}; // handled == false
  cvc::ariadne::CustomEdit e = it->second(current, w);
  e.handled = true; // it was drawn by a registered handler, regardless of edit state
  return e;
}

void ImGuiBackend::install(cvc::ariadne::Runtime &rt, ImGuiOverlay &overlay) {
  cvc::ariadne::Runtime *r = &rt;
  overlay.setDrawCallback([r] { r->render(); });
}

} // namespace gl
} // namespace cvc
