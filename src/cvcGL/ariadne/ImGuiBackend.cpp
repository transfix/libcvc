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

#include <cfloat>
#include <cvc/ariadne/ariadne.h>
#include <cvc/gl/ImGuiBinding.h> // cvc::gl::ui::*
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <string>
#include <vector>

#ifdef CVC_ENABLE_IMGUI
#define IMGUI_DEFINE_MATH_OPERATORS // must precede imgui.h (imgui_internal asserts it)
#include <cstdint>
#include <imgui.h>
#include <imgui_internal.h> // ImHashStr: a stable per-key slot in the window's storage
#include <vtk_glad.h>       // GL texture entry points (the loader ImGuiOverlay uses)
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
  // §11.4: seed the window from a persisted tree.<id>.geometry when the core supplied one. A FREE
  // window uses ImGuiCond_FirstUseEver (apply-once: the seed takes only on the window's first
  // appearance — a genuinely new window, or one whose imgui.ini state was cleared — and the user's
  // drag then owns it; the core re-passes the seed each frame, harmless under FirstUseEver, §16.2).
  // A seeded size wins over the authored size below (skip that seeding when a seed carries a size).
  const bool seededSize = m_haveSeed && m_pendingSeed.has_size;
  if (m_haveSeed) {
    if (m_pendingSeed.has_pos)
      ImGui::SetNextWindowPos(ImVec2(m_pendingSeed.x, m_pendingSeed.y), ImGuiCond_FirstUseEver);
    if (m_pendingSeed.has_size)
      ImGui::SetNextWindowSize(ImVec2(m_pendingSeed.w, m_pendingSeed.h), ImGuiCond_FirstUseEver);
    m_haveSeed = false;
  }
  if (!seededSize && (size.w.is_set() || size.h.is_set()))
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
  m_haveSeed = false; // consume the seed even in the inert (no-ImGui) build
#endif
  // Stable ImGui identity via the "Visible title###stable.id" trick (roadmap §3.3).
  std::string name = title;
  name += "###";
  name += id;
  const bool vis = ui::Begin(name.c_str());
  m_windowStylePushes.push_back(pushes);
  // §11.4: snapshot this window's current geometry for the core to persist. Position is always
  // valid; size only when NOT collapsed (a collapsed window reports just its title-bar height,
  // which must not clobber the persisted content size — the core back-fills the size from the
  // seed). A build without ImGui reports nothing (has_pos/has_size stay false).
  m_lastWindowGeom = ariadne::WindowGeom{};
#ifdef CVC_ENABLE_IMGUI
  const ImVec2 wp = ImGui::GetWindowPos();
  m_lastWindowGeom.has_pos = true;
  m_lastWindowGeom.x = wp.x;
  m_lastWindowGeom.y = wp.y;
  if (!ImGui::IsWindowCollapsed()) {
    const ImVec2 ws = ImGui::GetWindowSize();
    m_lastWindowGeom.has_size = true;
    m_lastWindowGeom.w = ws.x;
    m_lastWindowGeom.h = ws.y;
  }
#endif
  return vis;
}

void ImGuiBackend::seed_window_geometry(const ariadne::WindowGeom &g) {
  m_pendingSeed = g;
  m_haveSeed = true;
}

ariadne::WindowGeom ImGuiBackend::window_geometry() const { return m_lastWindowGeom; }

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
  const bool haveTrackSeed = m_haveTracks; // §11.4: consumed by this grid whatever its mode
  m_haveTracks = false;
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
      // §11.4: a persisted track size (seed_grid_tracks) is the pane's create-time default, so it
      // wins when the ImGui-storage slot is (re)created after a reload; a live slot keeps the
      // user's drag (GetFloatRef ignores the default when the slot exists — apply-once). No / short
      // seed → the authored row track (or an 80px floor).
      float def = gs.row_px[i] > 0.0f ? gs.row_px[i] : 80.0f;
      if (haveTrackSeed && i < m_pendingTracks.size() && m_pendingTracks[i] > 0.0f)
        def = m_pendingTracks[i];
      gs.pane_h.push_back(store->GetFloatRef(key, def));
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
  m_haveTracks =
      false;   // consume the seed even in the inert build (parity with begin_window's #else)
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
  // §11.4: snapshot the (possibly dragged) pane sizes for the core to persist; clear it for a
  // non-split grid so a stale split's sizes are never reported against it.
  m_lastGridTracks.clear();
  if (g.split) {
    for (float *p : g.pane_h)
      if (p)
        m_lastGridTracks.push_back(*p);
    if (g.cell > 0)
      ImGui::EndChild(); // close the last pane
  } else {
    ImGui::EndTable();
    if (g.color_pushes)
      ImGui::PopStyleColor(g.color_pushes);
  }
#endif
}

void ImGuiBackend::seed_grid_tracks(const std::vector<float> &sizes) {
  m_pendingTracks = sizes;
  m_haveTracks = true;
}

std::vector<float> ImGuiBackend::grid_tracks() const { return m_lastGridTracks; }
void ImGuiBackend::begin_disabled() {
#ifdef CVC_ENABLE_IMGUI
  ImGui::BeginDisabled(true); // §4 enabled_when/disabled_when: grey + non-interactive
#endif
}
void ImGuiBackend::end_disabled() {
#ifdef CVC_ENABLE_IMGUI
  ImGui::EndDisabled();
#endif
}
void ImGuiBackend::set_tooltip(const char *text) {
#ifdef CVC_ENABLE_IMGUI
  // AllowWhenDisabled: the core emits the tooltip while still inside a begin_disabled() scope
  // (enabled_when/disabled_when), and a bare IsItemHovered() returns false for disabled items —
  // yet a disabled control is exactly where "why is this greyed out?" tooltip matters most.
  if (text && *text && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    ImGui::SetTooltip("%s", text);
#else
  (void)text;
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
// §4.6 widget on_click: was the last-submitted item clicked (left button, released over it)? Works
// for any item ImGui submitted this frame — the walk queries it right after drawing the widget.
// Guarded like the other raw-ImGui calls: without imgui the overlay is inert, so no widget clicks.
bool ImGuiBackend::item_clicked() {
#ifdef CVC_ENABLE_IMGUI
  return ImGui::IsItemClicked();
#else
  return false;
#endif
}
// §4.6 widget on_hover: is the pointer over the last-submitted item this frame? Plain
// IsItemHovered() (NOT the AllowWhenDisabled variant the tooltip path uses) so a disabled widget
// reports no hover and its on_hover never fires. Continuous — true every frame the pointer is over
// the item.
bool ImGuiBackend::item_hovered() {
#ifdef CVC_ENABLE_IMGUI
  return ImGui::IsItemHovered();
#else
  return false;
#endif
}
// §4.6 widget on_drag: is the last item being actively dragged this frame? IsItemActive() is true
// while the item holds the mouse capture (press-and-hold begun on it); gating on IsMouseDragging()
// restricts firing to frames where the pointer is actually moving past the drag threshold, so a
// plain press without motion does not fire on_drag (that is on_click's job).
bool ImGuiBackend::item_dragged() {
#ifdef CVC_ENABLE_IMGUI
  return ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left);
#else
  return false;
#endif
}
// §4.6 on_drag_start / on_drag_end: the activate/deactivate edges of the last item —
// IsItemActivated fires the frame a press/drag begins on it, IsItemDeactivated the frame it
// releases.
bool ImGuiBackend::item_drag_started() {
#ifdef CVC_ENABLE_IMGUI
  return ImGui::IsItemActivated();
#else
  return false;
#endif
}
bool ImGuiBackend::item_drag_ended() {
#ifdef CVC_ENABLE_IMGUI
  return ImGui::IsItemDeactivated();
#else
  return false;
#endif
}
// §4.6 event scope: the last item's pointer state for a widget handler's `event` dict. x/y are the
// pointer in the item's content-rect-local pixels (GetMousePos - GetItemRectMin), dx/dy the left-
// button drag delta this gesture, button the 1-based button (left=1) or 0 if none is down.
bool ImGuiBackend::item_pointer(float &x, float &y, float &dx, float &dy, int &button) {
#ifdef CVC_ENABLE_IMGUI
  const ImVec2 mouse = ImGui::GetMousePos();
  const ImVec2 origin = ImGui::GetItemRectMin();
  x = mouse.x - origin.x;
  y = mouse.y - origin.y;
  const ImVec2 drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
  dx = drag.x;
  dy = drag.y;
  button = ImGui::IsMouseDown(ImGuiMouseButton_Left)     ? 1
           : ImGui::IsMouseDown(ImGuiMouseButton_Right)  ? 2
           : ImGui::IsMouseDown(ImGuiMouseButton_Middle) ? 3
                                                         : 0;
  return true;
#else
  x = y = dx = dy = 0.0f;
  button = 0;
  return false;
#endif
}

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

ariadne::ColorEdit ImGuiBackend::color(const char *label, const float rgb[3]) {
  ariadne::ColorEdit e;
  e.rgb[0] = rgb[0];
  e.rgb[1] = rgb[1];
  e.rgb[2] = rgb[2];
#ifdef CVC_ENABLE_IMGUI
  e.drawn = true;
  // ColorEdit3 edits e.rgb in place and returns true on any change this frame. Commit immediately
  // (like a discrete widget) — the core writes each change, so the picker reads its own value back
  // next frame with no per-widget edit cache (a colour edit is infrequent, not a continuous drag).
  e.changed = ImGui::ColorEdit3(label, e.rgb);
  e.committed = e.changed;
#else
  (void)label;
#endif
  return e;
}

void ImGuiBackend::register_widget(std::string custom_type, CustomDrawFn draw) {
  if (draw)
    m_customWidgets[std::move(custom_type)] = std::move(draw);
}

void ImGuiBackend::set_image(const std::string &name, const cvc::image &img) {
  PublishedImage &p = m_images[name];
  p.img = img; // copy-on-write; no pixel copy until data() detaches (it won't — we read const)
  p.dirty = true;
}

bool ImGuiBackend::draw_image(const char *name, float width) {
#ifdef CVC_ENABLE_IMGUI
  const auto it = m_images.find(name ? name : "");
  if (it == m_images.end())
    return false;
  PublishedImage &p = it->second;
  if (p.img.width() <= 0 || p.img.height() <= 0)
    return false;
  // Upload as RGBA8. `const` so data() reads the shared buffer without a COW detach.
  const cvc::image rgba =
      (p.img.format() == cvc::image::pixel_format::RGBA)
          ? p.img
          : p.img.converted(cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  const int W = rgba.width(), H = rgba.height();
  if (p.tex == 0) {
    glGenTextures(1, &p.tex);
    glBindTexture(GL_TEXTURE_2D, p.tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // crisp grid cells
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    p.dirty = true;
  }
  glBindTexture(GL_TEXTURE_2D, p.tex);
  if (p.dirty) {
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, W, H, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
    p.dirty = false;
  }
  const float h = width * static_cast<float>(H) / static_cast<float>(W);
  ImGui::Image(static_cast<ImTextureID>(static_cast<std::intptr_t>(p.tex)), ImVec2(width, h));
  return true;
#else
  (void)name;
  (void)width;
  return false;
#endif
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
