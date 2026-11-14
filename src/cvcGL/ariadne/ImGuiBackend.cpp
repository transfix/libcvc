// ImGuiBackend — the reference Ariadne backend (see the header). Renders a
// cvc::ariadne Widget tree as Dear ImGui over cvcGL's VTK canvas.
//
// Structure and discrete widgets go through the cvc::gl::ui:: pointer-free subset
// (which already degrades to no-ops without CVC_ENABLE_IMGUI). The two things ui::
// does not expose in value-in/edit-out form — the commit-on-release slider (a
// per-widget edit cache + the IsItemDeactivatedAfterEdit edge) and the value-form
// combo (BeginCombo) — touch raw ImGui directly, guarded by CVC_ENABLE_IMGUI with
// inert stubs otherwise. A drag edits a cache every frame, reports `changed` on
// each frame the value moves and `committed` on release; the core writes each
// change, or only the release for a `commit: release` slider.

#include <algorithm> // std::max
#include <cfloat>
#include <cmath> // fmod: the arc's turn
#include <cstring>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/rich_text.h> // §5.20 layout_rich_text
#include <cvc/gl/Clipboard.h>    // set_clipboard
#include <cvc/gl/ImGuiBinding.h> // cvc::gl::ui::*
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/TouchScroll.h> // keepTouchDrag: a widget with on_drag: keeps its drag
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <memory>
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

constexpr float kFlowGap = 10.0f;   // between flowing windows, and from the viewport's edges
constexpr float kFitFloor = 120.0f; // a fitting window's smallest limit, however near the edge

// A document's px at the UI's size. ImGuiOverlay scales the whole UI by the UI size and the
// display's pixel ratio (style.FontScaleMain x FontScaleDpi), so a window's size:, a column's
// width or a picture's scales with it: a 360 px window holds the same widgets at any UI size and on
// a phone's dense screen. What the core persists (§11.4 geometry, tracks, columns) is in the same
// logical px, so it holds through a change of UI size too.
float ui_px(float v) {
  const ImGuiStyle &s = ImGui::GetStyle();
  return v * s.FontScaleMain * s.FontScaleDpi;
}

// A phone's view: under 500 CSS px on its short side, either way up (today's phones are 360-430; a
// tablet or a laptop's window is well over). CSS px are the display's pixels over its pixel ratio.
constexpr float kPhoneView = 500.0f;

bool phone_view() {
  const ImVec2 d = ImGui::GetIO().DisplaySize;
  const float css = ImMin(d.x, d.y) / ImMax(ImGui::GetStyle().FontScaleDpi, 0.01f);
  return css > 1.0f && css < kPhoneView;
}

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

// InputText writing straight into a std::string — what imgui_stdlib's std::string overload does,
// inlined so this TU need not depend on that optional vendored file. CallbackResize grows the
// string to fit as the user types; the buffer handed to ImGui is the string's own storage.
struct InputTextStr {
  std::string *str;
};
int input_text_resize(ImGuiInputTextCallbackData *data) {
  if (data->EventFlag == ImGuiInputTextFlags_CallbackResize) {
    std::string *s = static_cast<InputTextStr *>(data->UserData)->str;
    s->resize(static_cast<std::size_t>(data->BufTextLen));
    data->Buf = s->data();
  }
  return 0;
}

// Resolve a §3.0.3b track to pixels: px verbatim, percent as a fraction of the
// parent extent, auto -> 0 (fit / default).
float resolve_track(const ariadne::Track &t, float parent) {
  if (t.unit == ariadne::Unit::Px)
    return ui_px(t.value);
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

#ifdef CVC_ENABLE_IMGUI
// A text view's layout and scroll, per key.
struct ImGuiBackend::TextViewState {
  std::string text;                                       // what the layout is of
  std::vector<std::pair<std::size_t, std::size_t>> lines; // each line's [begin, end) in text
  std::vector<float> tops;                                // each line's top; back() = the total
  float wrap_w = -2.0f;                                   // the layout's wrap width (<0: none)
  float scroll = 0.0f;
  bool at_bottom = true;
  // Where the view is being taken (FLT_MAX: the bottom; < 0: nowhere), asked of ImGui each frame
  // until the scroll gets there: ImGui clamps a scroll to the content it measured the frame
  // before, so a text that just grew needs a frame or two. Starts at the bottom.
  float target = FLT_MAX;
  int target_frames = 0;
  bool armed = true;     // reaching the top reports it (re-armed once it leaves the top)
  bool reported = false; // a view whose text fits has reported the room above it
  double fired_at = -1.0;
  float below = 0.0f; // what the window drew after the view last frame
  float end_y = 0.0f; // the window's cursor right after the view

  // Split the text into lines and measure each, wrapped at WRAP_W (<0: one row each).
  void layout(float wrap) {
    wrap_w = wrap;
    lines.clear();
    tops.assign(1, 0.0f);
    const float lh = ImGui::GetTextLineHeightWithSpacing();
    const float gap = lh - ImGui::GetTextLineHeight();
    for (std::size_t i = 0; i <= text.size();) {
      const std::size_t j = std::min(text.find('\n', i), text.size());
      if (j == text.size() && i == j && !lines.empty())
        break; // the text's last newline ends the last line
      lines.emplace_back(i, j);
      float h = lh;
      if (wrap > 0.0f && j > i)
        h = ImGui::CalcTextSize(text.data() + i, text.data() + j, false, wrap).y + gap;
      tops.push_back(tops.back() + h);
      i = j + 1;
    }
  }
};
#endif

ariadne::Capabilities ImGuiBackend::capabilities() const {
  ariadne::Capabilities c;
  c.windows = true;
  c.menubar = true;
  c.mouse = true;
  c.keyboard = true;
  c.color = true;
  c.glsl = true;       // the UI can draw GLSL (a shader_canvas; the scene's are the adapter's)
  c.view_embed = true; // render-to-texture views (roadmap §9.6)
  c.owns_loop = false; // guest mode: VTK's render pass drives Runtime::render()
  c.bitmap = true;     // image widgets draw as GL textures
  c.item_groups = true;
  c.clipboard = cvc::gl::clipboard::canCopy();
  c.fullscreen = static_cast<bool>(m_setFullscreen);
  c.fullscreen_view = true;
  c.maximize = true;
  c.badge = true;
  c.rich_text = true;
  return c;
}

bool ImGuiBackend::begin_main_menu_bar() { return ui::BeginMainMenuBar(); }
void ImGuiBackend::end_main_menu_bar() { ui::EndMainMenuBar(); }
bool ImGuiBackend::begin_menu(const char *label) { return ui::BeginMenu(label); }
void ImGuiBackend::end_menu() { ui::EndMenu(); }

bool ImGuiBackend::next_window_maximize(bool *maximized) {
  m_maximize = maximized;
  return true;
}

#ifdef CVC_ENABLE_IMGUI
namespace {
// §5.9 the square button in a window's title bar, left of its close button (and of a collapse arrow
// on the right): it flips *maximized. An item over the title bar, clipped to it, as ImGui's own
// close button is, so a press on it is the button's and doesn't start a move. Restoring, it is
// two squares. The window's last item (its title bar) is left as it was.
void maximize_button(bool *maximized, bool has_close) {
  ImGuiWindow *win = ImGui::GetCurrentWindow();
  if (!win || (win->Flags & ImGuiWindowFlags_NoTitleBar))
    return;
  ImGuiContext &g = *GImGui;
  const ImGuiStyle &st = g.Style;
  const float sz = g.FontSize;
  const ImRect bar = win->TitleBarRect();
  float right = bar.Max.x - st.FramePadding.x;
  if (has_close)
    right -= sz + st.ItemInnerSpacing.x;
  if (st.WindowMenuButtonPosition == ImGuiDir_Right && !(win->Flags & ImGuiWindowFlags_NoCollapse))
    right -= sz + st.ItemInnerSpacing.x;
  const ImRect bb(ImVec2(right - sz, bar.Min.y + st.FramePadding.y),
                  ImVec2(right, bar.Min.y + st.FramePadding.y + sz));
  const ImGuiLastItemData last = g.LastItemData;
  ImGui::PushClipRect(bar.Min, bar.Max, false);
  const ImGuiID id = win->GetID("#MAXIMIZE");
  if (ImGui::ItemAdd(bb, id, nullptr, ImGuiItemFlags_NoNav)) {
    const bool on = *maximized;
    bool hovered = false, held = false;
    if (ImGui::ButtonBehavior(bb, id, &hovered, &held))
      *maximized = !on;
    if (hovered || held)
      win->DrawList->AddRectFilled(
          bb.Min, bb.Max, ImGui::GetColorU32(held ? ImGuiCol_ButtonActive : ImGuiCol_ButtonHovered),
          st.FrameRounding);
    const ImU32 col = ImGui::GetColorU32(ImGuiCol_Text);
    const ImVec2 c = bb.GetCenter();
    const float r = ImMax(2.0f, sz * 0.27f), t = ImMax(1.0f, sz / 16.0f);
    if (on) { // restore: a square behind another
      const float o = r * 0.45f;
      win->DrawList->AddRect(ImVec2(c.x - r + o, c.y - r - o), ImVec2(c.x + r + o, c.y + r - o),
                             col, 0.0f, 0, t);
      win->DrawList->AddRectFilled(ImVec2(c.x - r - o, c.y - r + o),
                                   ImVec2(c.x + r - o, c.y + r + o),
                                   ImGui::GetColorU32(ImGuiCol_TitleBgActive));
      win->DrawList->AddRect(ImVec2(c.x - r - o, c.y - r + o), ImVec2(c.x + r - o, c.y + r + o),
                             col, 0.0f, 0, t);
    } else {
      win->DrawList->AddRect(ImVec2(c.x - r, c.y - r), ImVec2(c.x + r, c.y + r), col, 0.0f, 0, t);
    }
  }
  ImGui::PopClipRect();
  g.LastItemData = last;
}
} // namespace
#endif

void ImGuiBackend::focus_next_window() {
#ifdef CVC_ENABLE_IMGUI
  ImGui::SetNextWindowFocus(); // applies to the next ImGui::Begin — i.e. the next begin_window
#endif
}

#ifdef CVC_ENABLE_IMGUI
struct ImGuiBackend::WindowCuts {
  std::string name;    // the ImGui window's name ("title###id")
  bool paints = false; // ImGui's background is clear this frame and end_window paints it
  ImDrawList *dl = nullptr;
  ImDrawListSplitter split; // channel 0 the background, 1 the window's content
  std::vector<ImRect> holes;
};
#else
struct ImGuiBackend::WindowCuts {};
#endif

bool ImGuiBackend::begin_window(const char *title, const char *id, const ariadne::Size &size,
                                float border, bool *p_open) {
  int pushes = 0;
  bool *const maximize = m_maximize; // §5.9 this window's tree.<id>.maximized, when it may be
  m_maximize = nullptr;
  // Stable ImGui identity via the "Visible title###stable.id" trick (roadmap §3.3).
  std::string name = title;
  name += "###";
  name += id;
  bool vis = false;
#ifdef CVC_ENABLE_IMGUI
  // Resolve the §3.0.3b size against the main viewport's WORK area (the menu-aware
  // content rect), per axis: px verbatim, percent as a fraction of the work size,
  // auto -> 0 (let ImGui fit). min/max become window size constraints. The work area as built so
  // far this frame, which counts a main menu bar drawn before the windows: the viewport's WorkPos
  // lags a frame, so a document's first frame would put its windows under its menu bar.
  const ImRect work = static_cast<ImGuiViewportP *>(ImGui::GetMainViewport())->GetBuildWorkRect();
  const ImVec2 avail = work.GetSize();
  const float gap = ui_px(kFlowGap), fitFloor = ui_px(kFitFloor);
  auto px = [](const ariadne::Extent &e, float parent) -> float {
    if (e.unit == ariadne::Unit::Px)
      return ui_px(e.value);
    if (e.unit == ariadne::Unit::Percent)
      return parent * e.value / 100.0f;
    return 0.0f; // Auto
  };
  // A window with no authored size fits its widgets every frame, so it grows and shrinks with what
  // it shows, up to the viewport's edge, past which it scrolls. One with a size keeps it (the user
  // may resize it) and scrolls, both ways, when its widgets need more.
  const bool fits = !size.w.is_set() && !size.h.is_set();
  // After reset_layout() a window's first draw is a new document's: ImGui's window of the same id
  // (the last document's) counts as none, so the authored place and size apply again.
  const bool relaid = m_relayout && m_relaid.insert(id).second;
  ImGuiWindow *const existing = relaid ? nullptr : ImGui::FindWindowByName(name.c_str());
  const ImGuiCond once = relaid ? ImGuiCond_Always : ImGuiCond_FirstUseEver;
  const ariadne::WindowGeom seedGeom = m_pendingSeed; // read below, then consumed
  const bool hadSeed = m_haveSeed;
  if (relaid)
    ImGui::SetNextWindowCollapsed(false, ImGuiCond_Always);
  // §11.4: seed the window from a persisted tree.<id>.geometry when the core supplied one. A FREE
  // window uses ImGuiCond_FirstUseEver (apply-once: the seed takes only on the window's first
  // appearance — a genuinely new window, or one whose imgui.ini state was cleared — and the user's
  // drag then owns it; the core re-passes the seed each frame, harmless under FirstUseEver, §16.2).
  // A seeded size wins over the authored size below (skip that seeding when a seed carries a size).
  // A fitting window takes no size from it: the size the core persisted after the window's first,
  // still unmeasured frame would otherwise stop the fit and leave the window a few pixels square.
  // An authored `pos:` is in the work area, below a main menu bar; a persisted one is ImGui's own.
  const bool seededPos = m_haveSeed && m_pendingSeed.has_pos;
  const bool seededSize = m_haveSeed && m_pendingSeed.has_size && !fits;
  // A window opens inside the view, which on a phone is narrower than most documents were laid out
  // for: no larger than the work area, and placed so that all of it shows (a window that fits its
  // content, not yet measured, so that its smallest fit does, and again the next frame, measured).
  // The user may drag it anywhere after. One moved in keeps heading for where it would have been
  // (m_home), so a view that grows again returns it; a window that showed in full when the view
  // gets smaller moves in too (below).
  ImVec2 openSize(0.0f, 0.0f); // 0 on an axis: fit it
  if (seededSize)
    openSize = ImVec2(ui_px(m_pendingSeed.w), ui_px(m_pendingSeed.h));
  else if (!fits)
    openSize = ImVec2(px(size.w, avail.x), px(size.h, avail.y));
  const ImVec2 homeSize = openSize.x > 0.0f && openSize.y > 0.0f ? openSize : ImVec2(0.0f, 0.0f);
  const bool tooBig = openSize.x > avail.x || openSize.y > avail.y;
  openSize = ImMin(openSize, avail);
  if (m_haveSeed) {
    if (m_pendingSeed.has_pos) {
      const ImVec2 origin = m_pendingSeed.authored ? work.Min : ImVec2(0.0f, 0.0f);
      const ImVec2 shows(openSize.x > 0.0f ? openSize.x : ImMin(fitFloor, avail.x),
                         openSize.y > 0.0f ? openSize.y : ImMin(fitFloor, avail.y));
      const ImVec2 at = origin + ImVec2(ui_px(m_pendingSeed.x), ui_px(m_pendingSeed.y));
      ImVec2 in = ImMax(work.Min, ImMin(at, work.Max - shows));
      // On a phone, a document laid out for a wider screen would have all its windows moved in
      // onto the same few hundred pixels, one over another. There a window whose authored place or
      // size does not fit opens collapsed instead, its title bar under those of the others that
      // did, from the top of the view: a list to open one at a time. Expanded, it opens from its
      // title down, as wide as the view and no taller than the room below.
      const bool stack = !existing && m_pendingSeed.authored && phone_view() &&
                         (tooBig || in.x != at.x || in.y != at.y);
      if (stack) {
        const float title = ImGui::GetFrameHeight();
        in = ImVec2(work.Min.x,
                    ImMin(work.Min.y + static_cast<float>(m_stacked) * title, work.Max.y - title));
        ++m_stacked;
        ImGui::SetNextWindowCollapsed(true, once);
        if (!fits)
          openSize = ImVec2(avail.x, ImMin(openSize.y, work.Max.y - in.y));
      }
      ImGui::SetNextWindowPos(in, once);
      if (fits && !existing && !stack)
        m_placedUnmeasured.insert(id);
      // Where it opens: moved in or cut down to fit, or not yet measured, it keeps its home. One
      // stacked on a phone does not: its place is in the list, and a home would draw it back out.
      if (!existing) {
        m_home.erase(id);
        if (!stack && (fits || in.x != at.x || in.y != at.y || homeSize.x > openSize.x ||
                       homeSize.y > openSize.y))
          m_home[id] = Home{at.x, at.y, homeSize.x, homeSize.y, in.x, in.y, openSize.x, openSize.y};
      }
    }
    m_haveSeed = false;
  }
  if (seededSize || !fits)
    ImGui::SetNextWindowSize(openSize, once);

  // A new window with no place of its own joins the flow down the left of the viewport (m_flow);
  // one the user has dragged since it was put there leaves it.
  if (ImGui::GetFrameCount() != m_flowFrame) {
    m_flowFrame = ImGui::GetFrameCount();
    m_flowX = work.Min.x + gap;
    m_flowY = work.Min.y + gap;
    m_flowColumnW = 0.0f;
    m_flowObstacles.clear();
    // The first frame of a new document has nothing in its way: what was drawn last frame was
    // the last document's.
    const bool fresh = m_resetFrame == ImGui::GetFrameCount();
    std::unordered_set<ImGuiID> flowing;
    for (const auto &kv : m_flow)
      flowing.insert(kv.second.window);
    for (const ImGuiWindow *w : GImGui->Windows) {
      if (fresh || !w->WasActive || w->Hidden || w->IsFallbackWindow ||
          (w->Flags &
           (ImGuiWindowFlags_ChildWindow | ImGuiWindowFlags_Popup | ImGuiWindowFlags_Tooltip)))
        continue;
      // ImGui's own windows ("##MainMenuBar", Ctrl+Tab's "##NavWindowingOverlay") are not in
      // the way; a window with no title is "###<id>".
      if (std::strncmp(w->Name, "##", 2) == 0 && std::strncmp(w->Name, "###", 3) != 0)
        continue;
      if (flowing.count(w->ID))
        continue; // a flowing window: the flow places it
      m_flowObstacles.push_back(
          FlowRect{w->Pos.x, w->Pos.y, w->Pos.x + w->Size.x, w->Pos.y + w->Size.y});
    }
  }
  auto spot = m_flow.find(id);
  if (!existing && !seededPos)
    spot = m_flow.emplace(id, FlowSpot{}).first;
  else if (spot != m_flow.end() && existing &&
           (existing->Pos.x != spot->second.x || existing->Pos.y != spot->second.y)) {
    m_flow.erase(spot);
    spot = m_flow.end();
  }
  ImVec2 at = existing ? existing->Pos : work.Min + ImVec2(ui_px(60.0f), ui_px(60.0f));
  if (existing && m_placedUnmeasured.erase(id)) {
    const ImVec2 want = ImMin(ImGui::CalcWindowNextAutoFitSize(existing), avail);
    const ImVec2 in = ImMax(work.Min, ImMin(existing->Pos, work.Max - want));
    if (in.x != at.x || in.y != at.y) {
      ImGui::SetNextWindowPos(in, ImGuiCond_Always);
      at = in;
    }
  }
  // A window in the flow is placed again every frame anyway; the others keep inside the view.
  if (existing && spot == m_flow.end()) {
    auto home = m_home.find(id);
    // The view got smaller since this window was last drawn (a browser window made smaller, a
    // phone turned on its side, fullscreen left): a window that showed in full there makes where
    // it stands its home, and moves in. One the user left partly outside stays where it is.
    if (auto last = m_lastView.find(id); home == m_home.end() && last != m_lastView.end()) {
      const ImRect was(last->second.x0, last->second.y0, last->second.x1, last->second.y1);
      const ImRect shown(existing->Pos, existing->Pos + existing->Size); // collapsed: title bar
      ImRect wasRoom = was;
      wasRoom.Expand(1.0f); // a window moved in to the very edge
      if ((work.Min.x > was.Min.x || work.Min.y > was.Min.y || work.Max.x < was.Max.x ||
           work.Max.y < was.Max.y) &&
          wasRoom.Contains(shown) && !work.Contains(shown)) {
        const ImVec2 sz = fits ? ImVec2(0.0f, 0.0f) : existing->SizeFull;
        home = m_home
                   .emplace(id, Home{existing->Pos.x, existing->Pos.y, sz.x, sz.y, existing->Pos.x,
                                     existing->Pos.y, existing->SizeFull.x, existing->SizeFull.y})
                   .first;
      }
    }
    // A window with a home heads for it, as far as the view allows: in when the view gets
    // smaller, back out when it grows, until it is home. Moved or resized by the user since it
    // was put there, it stays where they put it.
    if (home != m_home.end()) {
      Home &h = home->second;
      const bool sized = h.w > 0.0f && h.h > 0.0f;
      const auto off = [](float a, float b) { return std::fabs(a - b) > 0.5f; };
      if (off(existing->Pos.x, h.placedX) || off(existing->Pos.y, h.placedY) ||
          (sized &&
           (off(existing->SizeFull.x, h.placedW) || off(existing->SizeFull.y, h.placedH)))) {
        m_home.erase(home);
      } else {
        const ImVec2 full = sized ? ImMin(ImVec2(h.w, h.h), avail) : existing->SizeFull;
        const ImVec2 want = existing->Collapsed ? ImMin(existing->Size, avail)
                            : sized             ? full
                                    : ImMin(ImGui::CalcWindowNextAutoFitSize(existing), avail);
        const ImVec2 to = ImMax(work.Min, ImMin(ImVec2(h.x, h.y), work.Max - want));
        if (sized && (full.x != existing->SizeFull.x || full.y != existing->SizeFull.y))
          ImGui::SetNextWindowSize(full, ImGuiCond_Always);
        if (to.x != existing->Pos.x || to.y != existing->Pos.y)
          ImGui::SetNextWindowPos(to, ImGuiCond_Always);
        at = to;
        h.placedX = to.x;
        h.placedY = to.y;
        h.placedW = full.x;
        h.placedH = full.y;
        if (to.x == h.x && to.y == h.y && (!sized || (full.x == h.w && full.y == h.h)))
          m_home.erase(home); // home again
      }
    }
  }
  m_lastView[id] = FlowRect{work.Min.x, work.Min.y, work.Max.x, work.Max.y};
  if (spot != m_flow.end()) {
    // The room it wants: all its widgets, for a window that fits them; else its size. (Nothing
    // yet for a window not drawn before -- the next frame places it again, measured.)
    const ImVec2 want = !existing                      ? ImVec2(0.0f, 0.0f)
                        : fits && !existing->Collapsed ? ImGui::CalcWindowNextAutoFitSize(existing)
                                                       : existing->Size;
    const float top = work.Min.y + gap, bottom = work.Max.y;
    const float left = work.Min.x + gap, right = work.Max.x;
    const auto new_column = [&] {
      m_flowX += ImMax(m_flowColumnW, want.x) + gap;
      m_flowY = top;
      m_flowColumnW = 0.0f;
    };
    if (m_flowY > top && m_flowY + want.y > bottom) // it would run off the bottom: a new column
      new_column();
    // Below any window it would cover (a bounded walk: a crowded screen ends in an overlap).
    for (int tries = 0; tries < 16 && m_flowX < right; ++tries) {
      const ImRect r(m_flowX, m_flowY, m_flowX + ImMax(want.x, 1.0f),
                     m_flowY + ImMax(want.y, 1.0f));
      const FlowRect *hit = nullptr;
      for (const FlowRect &o : m_flowObstacles)
        if (r.Overlaps(ImRect(o.x0, o.y0, o.x1, o.y1)))
          hit = &o;
      if (!hit)
        break;
      m_flowY = hit->y1 + gap;
      if (m_flowY + want.y > bottom)
        new_column();
    }
    // Never past the right edge: a column that would start beyond it overlaps the last instead.
    at = ImTrunc(ImVec2(ImMax(left, ImMin(m_flowX, right - want.x - gap)), m_flowY));
    ImGui::SetNextWindowPos(at, ImGuiCond_Always);
    spot->second = FlowSpot{at.x, at.y, ImHashStr(name.c_str())};
    m_flowY = at.y + ImMin(want.y, ImMax(bottom - at.y, fitFloor)) + gap;
    m_flowColumnW = ImMax(m_flowColumnW, want.x);
  }

  ImVec2 mn(size.min_w.is_set() ? px(size.min_w, avail.x) : 0.0f,
            size.min_h.is_set() ? px(size.min_h, avail.y) : 0.0f);
  ImVec2 mx(size.max_w.is_set() ? px(size.max_w, avail.x) : FLT_MAX,
            size.max_h.is_set() ? px(size.max_h, avail.y) : FLT_MAX);
  if (fits) { // no further than the viewport's far edges from where the window stands
    mx.x = ImMin(mx.x, ImMax(work.Max.x - at.x, fitFloor));
    mx.y = ImMin(mx.y, ImMax(work.Max.y - at.y, fitFloor));
  }
  // §5.9 maximized: the window fills the work area (under a main menu bar), neither moved nor
  // resized there; the place and size it had before come back when it is no longer.
  const bool maximized = maximize && *maximize;
  if (maximized) {
    if (!m_unmaximized.count(id)) {
      Unmaximized u;
      if (existing) {
        u = Unmaximized{existing->Pos.x, existing->Pos.y, existing->SizeFull.x,
                        existing->SizeFull.y, !fits};
      } else if (hadSeed && seedGeom.has_pos) { // maximized from the start: what is persisted
        const ImVec2 origin = seedGeom.authored ? work.Min : ImVec2(0.0f, 0.0f);
        u.x = origin.x + ui_px(seedGeom.x);
        u.y = origin.y + ui_px(seedGeom.y);
        u.sized = seedGeom.has_size && !fits;
        u.w = ui_px(seedGeom.w);
        u.h = ui_px(seedGeom.h);
      } else {
        u.x = work.Min.x + gap;
        u.y = work.Min.y + gap;
      }
      m_unmaximized[id] = u;
    }
    ImGui::SetNextWindowPos(work.Min, ImGuiCond_Always);
    ImGui::SetNextWindowSize(work.GetSize(), ImGuiCond_Always);
  } else if (auto back = m_unmaximized.find(id); back != m_unmaximized.end()) {
    ImGui::SetNextWindowPos(ImVec2(back->second.x, back->second.y), ImGuiCond_Always);
    if (back->second.sized)
      ImGui::SetNextWindowSize(ImVec2(back->second.w, back->second.h), ImGuiCond_Always);
    m_unmaximized.erase(back);
  }
  if (!maximized && (fits || size.min_w.is_set() || size.min_h.is_set() || size.max_w.is_set() ||
                     size.max_h.is_set()))
    ImGui::SetNextWindowSizeConstraints(mn, mx);
  if (border >= 0.0f) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, border);
    ++pushes;
  }
  const ImGuiWindowFlags flags = ImGuiWindowFlags_HorizontalScrollbar |
                                 (maximized ? ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize
                                            : (fits ? ImGuiWindowFlags_AlwaysAutoResize : 0));
  // A window that cut a hole in its background last frame (a viewport widget) paints its own in
  // end_window, around this frame's cuts: ImGui's is made clear, and the content goes on a channel
  // above the one that background is drawn on.
  auto cuts = std::make_shared<WindowCuts>();
  cuts->name = name;
  cuts->paints = m_cutWindows.count(name) != 0;
  if (cuts->paints)
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
  vis = ImGui::Begin(name.c_str(), p_open, flags);
  if (maximize)
    maximize_button(maximize, p_open != nullptr);
  if (cuts->paints) {
    ImGui::PopStyleColor();
    cuts->dl = ImGui::GetWindowDrawList();
    cuts->split.Split(cuts->dl, 2);
    cuts->split.SetCurrentChannel(cuts->dl, 1);
  }
  m_windowCuts.push_back(std::move(cuts));
#else
  (void)size;
  (void)border;
  m_haveSeed = false; // consume the seed even in the inert (no-ImGui) build
  vis = ui::Begin(name.c_str(), p_open);
#endif
  m_windowStylePushes.push_back(pushes);
  // §11.4: snapshot this window's current geometry for the core to persist. Position is always
  // valid; size only when NOT collapsed (a collapsed window reports just its title-bar height,
  // which must not clobber the persisted content size — the core back-fills the size from the
  // seed). A build without ImGui reports nothing (has_pos/has_size stay false). In logical px
  // (ui_px), as the seed is.
  m_lastWindowGeom = ariadne::WindowGeom{};
#ifdef CVC_ENABLE_IMGUI
  const float unit = ui_px(1.0f);
  const ImVec2 wp = ImGui::GetWindowPos();
  m_lastWindowGeom.has_pos = true;
  m_lastWindowGeom.x = wp.x / unit;
  m_lastWindowGeom.y = wp.y / unit;
  if (!ImGui::IsWindowCollapsed()) {
    const ImVec2 ws = ImGui::GetWindowSize();
    m_lastWindowGeom.has_size = true;
    m_lastWindowGeom.w = ws.x / unit;
    m_lastWindowGeom.h = ws.y / unit;
  }
#endif
  return vis;
}

void ImGuiBackend::reset_layout() {
  m_flow.clear();
  m_placedUnmeasured.clear();
  m_lastView.clear();
  m_home.clear();
  m_stacked = 0;
  m_relaid.clear();
  m_relaidGrids.clear();
  m_relayout = true;
  m_resetFrame = ImGui::GetFrameCount();
}

void ImGuiBackend::seed_window_geometry(const ariadne::WindowGeom &g) {
  m_pendingSeed = g;
  m_haveSeed = true;
}

ariadne::WindowGeom ImGuiBackend::window_geometry() const { return m_lastWindowGeom; }

void ImGuiBackend::end_window() {
#ifdef CVC_ENABLE_IMGUI
  // A text view of height 0 fills the window less what follows it: what that is, for next frame.
  if (m_lastTextView && m_lastTextViewWindow == ImGui::GetCurrentWindow())
    m_lastTextView->below = ImMax(0.0f, ImGui::GetCursorPosY() - m_lastTextView->end_y);
#endif
  m_lastTextView.reset();
  m_lastTextViewWindow = nullptr;
#ifdef CVC_ENABLE_IMGUI
  if (!m_windowCuts.empty()) {
    std::shared_ptr<WindowCuts> cuts = std::move(m_windowCuts.back());
    m_windowCuts.pop_back();
    if (cuts->paints)
      paint_window_background(*cuts);
    if (cuts->holes.empty())
      m_cutWindows.erase(cuts->name);
    else
      m_cutWindows.insert(cuts->name);
  }
#endif
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
  const bool haveColumnSeed = m_haveColumns;
  m_haveTracks = m_haveColumns = false;
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
  // After reset_layout() a grid's first draw is a new document's: its authored sizes, not the
  // ones the last document's grid of the same id was dragged to.
  const bool relaid = m_relayout && m_relaidGrids.insert(id ? id : "").second;
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
      float def = gs.row_px[i] > 0.0f ? gs.row_px[i] : ui_px(80.0f);
      if (haveTrackSeed && i < m_pendingTracks.size() && m_pendingTracks[i] > 0.0f)
        def = ui_px(m_pendingTracks[i]);
      if (relaid)
        store->SetFloat(key, def);
      else
        store->GetFloatRef(key, def); // create/seed the slot apply-once; the returned pointer is
                                      // NOT cached (a later iteration's insert can realloc the
                                      // storage) — we keep only the key and re-resolve it below at
                                      // use time.
      gs.pane_keys.push_back(key);
    }
    m_grids.push_back(std::move(gs));
    return true; // the panes themselves open in grid_next_cell
  }

  // A horizontal row with no col_widths: its cells side by side, one per child, as FTXUI lays it
  // out. A table needs its column count before the first cell, which only the walk knows, so the
  // cells flow on one line instead (each a group, so a cell's widgets stack inside it). Borders and
  // resizable seams need columns: such a row keeps the table, one column per col_widths entry.
  if (layout.kind == ariadne::LayoutKind::Horizontal && layout.col_widths.empty() &&
      !layout.resizable && layout.borders == ariadne::BorderShow::Off) {
    gs.flow = true;
    m_grids.push_back(std::move(gs));
    return true;
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
  if (relaid) // its columns' widths again from TableSetupColumn below
    if (ImGuiTable *old = ImGui::TableFindByID(ImGui::GetID(tid.c_str())))
      ImGui::TableResetSettings(old);
  const bool open = ImGui::BeginTable(tid.c_str(), gs.cols, flags);
  if (open) {
    gs.resizable = layout.resizable;
    for (int i = 0; i < gs.cols; ++i) {
      ImGuiTableColumnFlags cf = ImGuiTableColumnFlags_WidthStretch;
      float w = 1.0f;
      if (i < static_cast<int>(layout.col_widths.size())) {
        const ariadne::Track &t = layout.col_widths[i];
        if (t.unit == ariadne::Unit::Px) {
          cf = ImGuiTableColumnFlags_WidthFixed;
          w = ui_px(t.value);
        } else if (t.unit == ariadne::Unit::Percent) {
          cf = ImGuiTableColumnFlags_WidthStretch;
          w = t.value;
        } else {
          cf = ImGuiTableColumnFlags_WidthFixed;
          w = 0.0f; // Auto → fit content
        }
      }
      // §11.4: a persisted width (seed_grid_columns, px) is the column's create-time init value,
      // which ImGui applies only when it creates the table, so a live drag wins. A fixed column
      // takes it as its width; a stretch column as its weight, so the stretch columns keep their
      // persisted proportions at any table width.
      if (layout.resizable && haveColumnSeed && i < static_cast<int>(m_pendingColumns.size()) &&
          m_pendingColumns[static_cast<std::size_t>(i)] > 0.0f)
        w = ui_px(m_pendingColumns[static_cast<std::size_t>(i)]);
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
  m_haveTracks = m_haveColumns =
      false;   // consume the seed even in the inert build (parity with begin_window's #else)
  return true; // stub: emit the children as a plain vertical flow
#endif
}

void ImGuiBackend::grid_next_cell() {
#ifdef CVC_ENABLE_IMGUI
  if (m_grids.empty())
    return;
  GridState &g = m_grids.back();
  if (g.flow) {
    if (g.cell > 0) {
      ImGui::EndGroup();
      ImGui::SameLine();
    }
    ImGui::BeginGroup();
    ++g.cell;
    return;
  }
  if (g.split) {
    const int i = g.cell;
    const int n = static_cast<int>(g.pane_keys.size());
    if (i > 0)
      ImGui::EndChild(); // close the previous pane -> back to the parent window
    // Now in the parent window, whose StateStorage holds the pane-height slots seeded in
    // begin_grid. Re-resolve pointers by key HERE (never cached across begin_grid's insert loop):
    // every slot already exists, so these GetFloatRef calls cannot insert/realloc, and the two the
    // splitter needs stay valid together.
    ImGuiStorage *store = ImGui::GetStateStorage();
    if (i > 0 && i - 1 < n && i < n) {
      float *a = store->GetFloatRef(g.pane_keys[i - 1]);
      float *b = store->GetFloatRef(g.pane_keys[i]);
      row_splitter(4.0f, a, b, 24.0f, 24.0f, g.split_long_axis);
    }
    const float h = (i < n) ? store->GetFloat(g.pane_keys[i]) : 0.0f;
    const std::string pid = "##cvcg.pane" + std::to_string(i);
    // A pane is as wide as its window, unless the window fits its widgets: then the window takes
    // its width from the panes', which fit their own widgets.
    const bool fit = (ImGui::GetCurrentWindow()->Flags & ImGuiWindowFlags_AlwaysAutoResize) != 0;
    ImGui::BeginChild(pid.c_str(), ImVec2(0.0f, h),
                      ImGuiChildFlags_Borders | (fit ? ImGuiChildFlags_AutoResizeX : 0));
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
  m_lastGridColumns.clear();
  if (g.flow) {
    if (g.cell > 0)
      ImGui::EndGroup();
  } else if (g.split) {
    if (g.cell > 0)
      ImGui::EndChild(); // close the last pane FIRST -> back to the parent window, whose
                         // StateStorage holds the pane slots we snapshot by key next.
    ImGuiStorage *store = ImGui::GetStateStorage();
    for (unsigned int k : g.pane_keys)
      m_lastGridTracks.push_back(store->GetFloat(k) / ui_px(1.0f)); // logical px, as seeded
  } else {
    // §11.4: a resizable table's column widths (the width each column was given this frame, the
    // value TableSetupColumn takes back), read before EndTable while the table is current. Not
    // before its layout is locked (its first row), when the widths are not computed yet.
    const ImGuiTable *t = ImGui::GetCurrentTable();
    if (g.resizable && t && t->IsLayoutLocked)
      for (int i = 0; i < t->ColumnsCount; ++i)
        m_lastGridColumns.push_back(t->Columns[i].WidthGiven / ui_px(1.0f));
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

void ImGuiBackend::seed_grid_columns(const std::vector<float> &widths) {
  m_pendingColumns = widths;
  m_haveColumns = true;
}

std::vector<float> ImGuiBackend::grid_columns() const { return m_lastGridColumns; }
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
void ImGuiBackend::item_badge(const char *text) {
#ifdef CVC_ENABLE_IMGUI
  // Over the item just drawn (a picture), on the window's draw list, so the picture stays the item:
  // the picture dimmed, the text centred on it, a third of its height (within one and four lines
  // of text, and narrower than the picture).
  const ImVec2 a = ImGui::GetItemRectMin(), z = ImGui::GetItemRectMax();
  const float w = z.x - a.x, h = z.y - a.y;
  if (!text || !*text || !(w > 0.0f) || !(h > 0.0f))
    return;
  ImDrawList *dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(a, z, IM_COL32(0, 0, 0, 140));
  ImFont *font = ImGui::GetFont();
  const float line = ImGui::GetFontSize();
  float size = std::clamp(h / 3.0f, line, 4.0f * line);
  ImVec2 ts = font->CalcTextSizeA(size, FLT_MAX, 0.0f, text);
  if (ts.x > 0.9f * w) {
    size = std::max(0.5f * line, size * 0.9f * w / ts.x);
    ts = font->CalcTextSizeA(size, FLT_MAX, 0.0f, text);
  }
  dl->AddText(font, size, ImVec2(a.x + (w - ts.x) * 0.5f, a.y + (h - ts.y) * 0.5f),
              IM_COL32(255, 255, 255, 255), text);
#else
  (void)text;
#endif
}

void ImGuiBackend::push_id(const char *id) { ui::PushId(id); }
void ImGuiBackend::pop_id() { ui::PopId(); }

void ImGuiBackend::text_line(const char *text) { ui::TextLine(text); }
void ImGuiBackend::text_value(const char *label, const std::string &value) {
  // A non-empty label prefixes "label: value"; an empty label shows just the value (e.g. a roster
  // name bound with no caption).
  std::string line = (label && *label) ? (std::string(label) + ": " + value) : value;
  ui::TextLine(line.c_str());
}
void ImGuiBackend::text_line_colored(const char *text, const float rgb[3]) {
  ui::TextLineColored(text, rgb[0], rgb[1], rgb[2]);
}
void ImGuiBackend::text_value_colored(const char *label, const std::string &value,
                                      const float rgb[3]) {
  std::string line = (label && *label) ? (std::string(label) + ": " + value) : value;
  ui::TextLineColored(line.c_str(), rgb[0], rgb[1], rgb[2]);
}
bool ImGuiBackend::text_runs(const std::vector<cvc::ariadne::TextRun> &runs, const float *rgb) {
#ifdef CVC_ENABLE_IMGUI
  // Each run right after the last (SameLine with no spacing), a picture as tall as a line of text
  // at its own shape; a picture not published yet shows its text.
  const float lh = ImGui::GetTextLineHeight();
  if (rgb)
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(rgb[0], rgb[1], rgb[2], 1.0f));
  ImGui::BeginGroup();
  bool first = true;
  for (const cvc::ariadne::TextRun &r : runs) {
    if (!first)
      ImGui::SameLine(0.0f, 0.0f);
    first = false;
    int W = 0, H = 0;
    const unsigned int tex = r.image.empty() ? 0 : image_texture(r.image.c_str(), W, H);
    if (tex != 0 && W > 0 && H > 0) {
      ImGui::Image(static_cast<ImTextureID>(static_cast<std::intptr_t>(tex)),
                   ImVec2(lh * static_cast<float>(W) / static_cast<float>(H), lh));
      if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
        ImGui::SetTooltip("%s", r.text.c_str()); // its :shortcode:
    } else {
      ImGui::TextUnformatted(r.text.data(), r.text.data() + r.text.size());
    }
  }
  ImGui::EndGroup();
  if (rgb)
    ImGui::PopStyleColor();
  return true;
#else
  (void)runs;
  (void)rgb;
  return false;
#endif
}
// §5.20 a rich text, laid out by layout_rich_text in px at the width its line has (in a window that
// fits its content, as wide as its longest line up to the view's edge), size: bounding it. The
// paragraph is one group, so it is the last item afterwards. Each piece of a link is an item of
// its own, submitted before the group closes, so it takes the pointer before the paragraph or a
// clickable card around it; the first piece of each is a stop for keyboard navigation. Styles are
// drawn with the one UI font: bold twice, a pixel apart; italic slanted (its glyphs' vertices
// sheared); code on a FrameBg box; a link in TextLink's colour, underlined.
cvc::ariadne::RichTextEvent ImGuiBackend::rich_text(const std::vector<ariadne::TextRun> &runs,
                                                    const ariadne::RichTextOptions &o) {
#ifdef CVC_ENABLE_IMGUI
  ariadne::RichTextEvent ev;
  ImGuiWindow *win = ImGui::GetCurrentWindow();
  ImFont *font = ImGui::GetFont();
  const float fs = ImGui::GetFontSize();
  const float lh = ImGui::GetTextLineHeight();
  const float bold_dx = std::max(1.0f, std::floor(fs / 14.0f));
  const float pad = std::max(1.0f, std::floor(fs * 0.15f)); // code's box, past its text
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  const ImGuiStyle &style = ImGui::GetStyle();

  // The width: the line's; in a window whose width fits its content (no size:, or an auto width
  // while ImGui measures it), the room to the view's right edge, not a table cell's; size: N at
  // most.
  float width = ImGui::GetContentRegionAvail().x;
  const bool fitting =
      (win->Flags & ImGuiWindowFlags_AlwaysAutoResize) != 0 || win->AutoFitFramesX > 0;
  if (fitting && !ImGui::GetCurrentTable()) {
    const ImGuiViewport *vp = ImGui::GetMainViewport();
    width = vp->WorkPos.x + vp->WorkSize.x - origin.x - style.WindowPadding.x - style.ScrollbarSize;
  }
  if (o.max_width > 0.0f)
    width = std::min(width, ui_px(o.max_width));
  width = std::max(width, fs); // a line holds at least a glyph or two
  const auto measure = [&](const std::string &t, unsigned st) {
    float w = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, t.data(), t.data() + t.size()).x;
    if (st & ariadne::text_style::bold)
      w += bold_dx;
    return w;
  };
  const auto picture = [&](const ariadne::TextRun &r) {
    int W = 0, H = 0;
    if (image_texture(r.image.c_str(), W, H) != 0 && W > 0 && H > 0)
      return lh * static_cast<float>(W) / static_cast<float>(H);
    return measure(r.text, r.style);
  };
  const std::string ellipsis = font->IsGlyphInFont(0x2026) ? "\xE2\x80\xA6" : "...";
  const ariadne::RichLayout layout = ariadne::layout_rich_text(
      runs, o.wrap ? width : 0.0f, o.max_lines, measure, picture, ellipsis);

  // A stable id for this paragraph: its place among the paragraphs under the same id stack.
  if (m_richFrame != ImGui::GetFrameCount()) {
    m_richFrame = ImGui::GetFrameCount();
    m_richSeen.clear();
  }
  const ImGuiID key = ImGui::GetID("##ari_rich");
  const int nth = m_richSeen[key]++;
  const ImGuiID base = ImHashData(&nth, sizeof nth, key);

  // The links: each piece an item, the first one a stop for keyboard navigation. Pressed is a
  // tap: released over it, the pointer not dragged far (a finger scrolling the feed opens none).
  const int links = std::max(0, o.links);
  std::vector<int> link_state(static_cast<std::size_t>(links), 0); // 1 hovered, 2 held
  std::vector<bool> link_first(static_cast<std::size_t>(links), true);
  const float slop = ImGui::GetIO().MouseDragThreshold;
  const bool tap = !ImGui::GetIO().MouseReleased[ImGuiMouseButton_Left] ||
                   ImGui::GetIO().MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] <= slop * slop;
  ImGui::BeginGroup();
  int piece_no = 0;
  for (std::size_t li = 0; li < layout.lines.size(); ++li) {
    const float y = origin.y + lh * static_cast<float>(li);
    for (const ariadne::RichPiece &p : layout.lines[li].pieces) {
      ++piece_no;
      const ariadne::TextRun *run = p.run < runs.size() ? &runs[p.run] : nullptr;
      const ImRect bb(ImVec2(origin.x + p.x, y), ImVec2(origin.x + p.x + p.w, y + lh));
      if (run && run->link >= 0 && run->link < links) {
        const std::size_t k = static_cast<std::size_t>(run->link);
        const ImGuiID id = ImHashData(&piece_no, sizeof piece_no, base);
        const bool first = link_first[k];
        link_first[k] = false;
        if (!ImGui::ItemAdd(bb, id, nullptr, first ? 0 : ImGuiItemFlags_NoNav))
          continue;
        bool hovered = false, held = false;
        const bool pressed = ImGui::ButtonBehavior(bb, id, &hovered, &held);
        if (pressed && tap && ev.clicked < 0)
          ev.clicked = run->link;
        if (hovered || held) {
          link_state[k] = std::max(link_state[k], held ? 2 : 1);
          ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
          if (ev.hovered < 0)
            ev.hovered = run->link;
        }
        if (first)
          ImGui::RenderNavCursor(bb, id);
        if (k < o.link_tips.size() && !o.link_tips[k].empty() &&
            ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
          ImGui::SetTooltip("%s", o.link_tips[k].c_str());
      } else if (p.image && run) {
        // A picture that isn't a link: its :shortcode: on hover, as in a plain line (text_runs).
        if (ImGui::ItemAdd(bb, 0) && ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
          ImGui::SetTooltip("%s", run->text.c_str());
      }
    }
  }

  // Draw: the code boxes first (under their text), then the text and pictures.
  ImDrawList *dl = ImGui::GetWindowDrawList();
  const ImU32 text_col = o.rgb ? ImGui::GetColorU32(ImVec4(o.rgb[0], o.rgb[1], o.rgb[2], 1.0f))
                               : ImGui::GetColorU32(ImGuiCol_Text);
  const ImU32 link_col = ImGui::GetColorU32(ImGuiCol_TextLink);
  const ImU32 code_bg = ImGui::GetColorU32(ImGuiCol_FrameBg);
  const float ascent = ImGui::GetFontBaked() ? ImGui::GetFontBaked()->Ascent : lh * 0.8f;
  for (std::size_t li = 0; li < layout.lines.size(); ++li) {
    const float y = origin.y + lh * static_cast<float>(li);
    for (const ariadne::RichPiece &p : layout.lines[li].pieces) {
      const ariadne::TextRun *run = p.run < runs.size() ? &runs[p.run] : nullptr;
      if (run && (run->style & ariadne::text_style::code) && !p.image)
        dl->AddRectFilled(ImVec2(origin.x + p.x - pad, y),
                          ImVec2(origin.x + p.x + p.w + pad, y + lh), code_bg, pad);
    }
  }
  for (std::size_t li = 0; li < layout.lines.size(); ++li) {
    const float y = origin.y + lh * static_cast<float>(li);
    for (const ariadne::RichPiece &p : layout.lines[li].pieces) {
      const ariadne::TextRun *run = p.run < runs.size() ? &runs[p.run] : nullptr;
      const ImVec2 at(origin.x + p.x, y);
      const int link = run && run->link >= 0 && run->link < links ? run->link : -1;
      const int st = link >= 0 ? link_state[static_cast<std::size_t>(link)] : 0;
      if (p.image && run) {
        int W = 0, H = 0;
        const unsigned int tex = image_texture(run->image.c_str(), W, H);
        if (tex != 0 && W > 0 && H > 0) {
          dl->AddImage(static_cast<ImTextureID>(static_cast<std::intptr_t>(tex)), at,
                       ImVec2(at.x + p.w, y + lh));
          continue;
        }
      }
      const unsigned s = run ? run->style : 0u;
      const ImU32 col = link >= 0 ? link_col : text_col;
      const char *b = p.text.data(), *e = p.text.data() + p.text.size();
      const int vtx0 = dl->VtxBuffer.Size;
      dl->AddText(font, fs, at, col, b, e);
      if (s & ariadne::text_style::bold)
        dl->AddText(font, fs, ImVec2(at.x + bold_dx, at.y), col, b, e);
      if (s & ariadne::text_style::italic) { // lean the glyphs right about the baseline
        const float base_y = at.y + ascent;
        for (int v = vtx0; v < dl->VtxBuffer.Size; ++v)
          dl->VtxBuffer[v].pos.x += (base_y - dl->VtxBuffer[v].pos.y) * 0.2f;
      }
      if (link >= 0) {
        const float uy = std::floor(at.y + ascent + std::max(1.0f, fs / 12.0f));
        dl->AddLine(ImVec2(at.x, uy), ImVec2(at.x + p.w, uy), col,
                    st > 0 ? std::max(1.5f, fs / 10.0f) : 1.0f);
      }
    }
  }
  // The paragraph's own extent, so the group (and the window) take its room.
  ImGui::SetCursorScreenPos(origin);
  ImGui::Dummy(ImVec2(std::max(1.0f, layout.width), lh * static_cast<float>(layout.lines.size())));
  ImGui::EndGroup();
  return ev;
#else
  (void)runs;
  (void)o;
  return {};
#endif
}
void ImGuiBackend::separator() { ui::Separator(); }
void ImGuiBackend::progress_bar(const char *label, float fraction, const std::string &overlay) {
#ifdef CVC_ENABLE_IMGUI
  // The bar fills the line but leaves the label's width, as the sliders do; the label (up to any
  // "##") follows it.
  const char *end = label ? ImGui::FindRenderedTextEnd(label) : label;
  const bool labelled = label && end != label;
  const float width = labelled ? ImGui::CalcItemWidth() : -FLT_MIN;
  ImGui::ProgressBar(fraction, ImVec2(width, 0.f), overlay.empty() ? nullptr : overlay.c_str());
  if (labelled) {
    ImGui::SameLine(0.f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::TextUnformatted(label, end);
  }
#else
  Backend::progress_bar(label, fraction, overlay);
#endif
}
void ImGuiBackend::plot_lines(const char *label, const std::vector<float> &values, double lo,
                              double hi, const std::string &overlay, double height) {
#ifdef CVC_ENABLE_IMGUI
  // Laid out as the progress bar is: the graph fills the line but leaves the label's width. ImGui
  // shows the value under the mouse; FLT_MAX for a range fits it to the values.
  const char *end = label ? ImGui::FindRenderedTextEnd(label) : label;
  const bool labelled = label && end != label;
  const float width = labelled ? ImGui::CalcItemWidth() : -FLT_MIN;
  const float a = lo < hi ? static_cast<float>(lo) : FLT_MAX;
  const float z = lo < hi ? static_cast<float>(hi) : FLT_MAX;
  ImGui::PushID(label ? label : "plot");
  ImGui::PlotLines("##plot", values.empty() ? nullptr : values.data(),
                   static_cast<int>(values.size()), 0, overlay.empty() ? nullptr : overlay.c_str(),
                   a, z, ImVec2(width, ui_px(static_cast<float>(height))));
  ImGui::PopID();
  if (labelled) {
    ImGui::SameLine(0.f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::TextUnformatted(label, end);
  }
#else
  Backend::plot_lines(label, values, lo, hi, overlay, height);
#endif
}
void ImGuiBackend::spinner(const char *label, double seconds, bool ascii) {
#ifdef CVC_ENABLE_IMGUI
  // An arc drawn rather than a glyph, so it turns whatever the font holds (the browser build's is
  // ASCII): three quarters of a circle a text line high, one turn a second, in the text colour
  // (dimmed by a disabled scope like any text). The label follows on the same line.
  (void)ascii;
  const float h = ImGui::GetTextLineHeight();
  const ImVec2 at = ImGui::GetCursorScreenPos();
  ImGui::Dummy(ImVec2(h, h));
  const float r = 0.5f * h - 1.5f;
  if (r > 1.f) {
    const ImVec2 c(at.x + 0.5f * h, at.y + 0.5f * h);
    const double turns = seconds > 0.0 && std::isfinite(seconds) ? std::fmod(seconds, 1.0) : 0.0;
    const float a0 = static_cast<float>(turns * 2.0 * IM_PI);
    ImDrawList *dl = ImGui::GetWindowDrawList();
    dl->PathArcTo(c, r, a0, a0 + 1.5f * IM_PI, 24);
    dl->PathStroke(ImGui::GetColorU32(ImGuiCol_Text), ImDrawFlags_None, std::max(1.5f, 0.15f * h));
  }
  if (label && *label) {
    ImGui::SameLine(0.f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::TextUnformatted(label);
  }
#else
  Backend::spinner(label, seconds, ascii);
#endif
}
bool ImGuiBackend::button(const char *label) { return ui::Button(label); }
bool ImGuiBackend::menu_item_action(const char *label) { return ui::MenuItemClicked(label); }
// §4.6 widget on_click: was the last-submitted item clicked (left button, released over it)? Works
// for any item ImGui submitted this frame — the walk queries it right after drawing the widget.
// Guarded like the other raw-ImGui calls: without imgui the overlay is inert, so no widget clicks.
bool ImGuiBackend::item_clicked() {
#ifdef CVC_ENABLE_IMGUI
  if (m_groupHitId != 0 && ImGui::GetItemID() == m_groupHitId)
    return m_groupHit; // an item group: its button's tap (end_item_group)
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
  // An item group: the pointer is over it, whether or not a widget inside it is held.
  if (m_groupHitId != 0 && ImGui::GetItemID() == m_groupHitId)
    return ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
  return ImGui::IsItemHovered();
#else
  return false;
#endif
}
// §5.13 an autoplaying video plays while its picture shows: not scrolled out of its window's view.
bool ImGuiBackend::item_visible() {
#ifdef CVC_ENABLE_IMGUI
  return ImGui::IsItemVisible();
#else
  return true;
#endif
}
// §5.11 the width left on the line, in the document's px. A window that fits its content has the
// width its widest row makes (last frame's), so a picture that takes all of it keeps it as it is;
// `at_least` keeps a window holding little else from fitting the picture to nothing.
float ImGuiBackend::available_width(float at_least) const {
#ifdef CVC_ENABLE_IMGUI
  const float scale = ui_px(1.0f);
  const float avail = ImGui::GetContentRegionAvail().x / (scale > 0.0f ? scale : 1.0f);
  const bool fit = (ImGui::GetCurrentWindow()->Flags & ImGuiWindowFlags_AlwaysAutoResize) != 0;
  return fit ? ImMax(avail, at_least) : ImMax(avail, 1.0f);
#else
  (void)at_least;
  return 0.0f;
#endif
}
// §4.6 widget on_drag: is the last item being actively dragged this frame? IsItemActive() is true
// while the item holds the mouse capture (press-and-hold begun on it); gating on IsMouseDragging()
// restricts firing to frames where the pointer is actually moving past the drag threshold, so a
// plain press without motion does not fire on_drag (that is on_click's job). The core asks only
// for a widget whose document handles its drag, so a finger's drag on it is never a scroll
// (keepTouchDrag), here and for its on_drag_start / on_drag_end below.
bool ImGuiBackend::item_dragged() {
#ifdef CVC_ENABLE_IMGUI
  cvc::gl::keepTouchDrag();
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
  cvc::gl::keepTouchDrag();
  return ImGui::IsItemActivated();
#else
  return false;
#endif
}
bool ImGuiBackend::item_drag_ended() {
#ifdef CVC_ENABLE_IMGUI
  cvc::gl::keepTouchDrag();
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

#ifdef CVC_ENABLE_IMGUI
// Keep a slider / input / combo / colour widget's TRAILING label from clipping in a narrow window.
// ImGui lays a labeled widget out as [frame of CalcItemWidth()][ItemInnerSpacing][label]; the
// default item width (~65% of the content region) leaves only the remainder for the label, so a
// long label ("Bake interval (frames)") runs past the right edge of a slim panel. When that would
// happen, shrink JUST the next item so [frame][spacing][label] fits the content region — floored so
// the frame stays grabbable. When the label already fits at the default width (a roomy panel /
// short label), ImGui's default is left untouched, so this never narrows a slider that was fine.
//
// In a table cell the widget always fills the cell instead. The default (~65% of the cell) feeds
// back into an auto-width column, which ImGui re-fits to its content each frame: the column shrank
// to 65% of itself every frame until the 48 px floor snapped it back, so the slider never held a
// width long enough to be dragged.
//
// In a row of widgets side by side (a player's Play, seek slider and Mute) the widget takes at most
// the 16 font sizes ImGui gives an auto-resizing window's items, so the cells after it keep their
// room: filling the line, as in a table cell, pushed them out of the window.
void reserve_label_width(const char *label, bool in_row) {
  const ImGuiStyle &style = ImGui::GetStyle();
  const float labelW = ImGui::CalcTextSize(label, nullptr, /*hide_text_after_double_hash=*/true).x;
  const ImGuiWindow *win = ImGui::GetCurrentWindow();
  if (in_row) {
    const float room = (win->Flags & ImGuiWindowFlags_AlwaysAutoResize)
                           ? FLT_MAX
                           : ImGui::GetContentRegionAvail().x;
    const float fit = ImMin(room, 16.0f * ImGui::GetFontSize()) - style.ItemInnerSpacing.x - labelW;
    ImGui::SetNextItemWidth(ImMax(fit, 48.0f));
    return;
  }
  // A window that fits its widgets grows to the label instead -- and shrinking the frame to the
  // window's width there would only shrink the window with it, frame after frame. It grows only as
  // far as the view's edge, though, which on a phone is near: ImGui gives such a window's widgets
  // 16 lines' width, most of a phone's at its UI size. There the frame leaves the label room up to
  // the widest the window can be -- bounded by the view, not the window, so it stays put frame
  // after frame. ImGui fits a window no wider than the view less its safe-area padding each side.
  if (win->Flags & ImGuiWindowFlags_AlwaysAutoResize) {
    const ImGuiViewport *vp = ImGui::GetMainViewport();
    const float widest = ImMin(vp->WorkPos.x + vp->WorkSize.x - win->Pos.x,
                               vp->WorkSize.x - 2.0f * style.DisplaySafeAreaPadding.x);
    const float most = widest - (win->DC.CursorPos.x - win->Pos.x) - style.WindowPadding.x -
                       (win->ScrollbarY ? style.ScrollbarSize : 0.0f);
    const float fit = most - style.ItemInnerSpacing.x - labelW;
    if (fit < ImGui::CalcItemWidth())
      ImGui::SetNextItemWidth(ImMax(fit, 48.0f));
    return;
  }
  const float avail = ImGui::GetContentRegionAvail().x;
  const float fit = avail - style.ItemInnerSpacing.x - labelW;
  if (ImGui::GetCurrentTable() != nullptr || fit < ImGui::CalcItemWidth())
    ImGui::SetNextItemWidth(ImMax(fit, 48.0f)); // floor: a still-usable slider/drag handle
}
#endif

ariadne::IntEdit ImGuiBackend::slider_int(const char *label, const char *key, int current, int lo,
                                          int hi) {
#ifdef CVC_ENABLE_IMGUI
  int &v = *cache_int(key, current, ImGui::IsAnyItemActive());
  reserve_label_width(label, in_flow_row());
  const bool changed = ImGui::SliderInt(label, &v, lo, hi);
  const bool committed = ImGui::IsItemDeactivatedAfterEdit(); // the drag ended
  return {changed, committed, v};
#else
  (void)label;
  (void)key;
  (void)lo;
  (void)hi;
  return {false, false, current};
#endif
}

namespace {
ariadne::DoubleEdit imgui_slider_double(const char *label, const char *key, double current,
                                        double lo, double hi, const char *fmt, bool log,
                                        bool in_row) {
#ifdef CVC_ENABLE_IMGUI
  float &v = *cache_float(key, static_cast<float>(current), ImGui::IsAnyItemActive());
  reserve_label_width(label, in_row);
  const bool changed =
      ImGui::SliderFloat(label, &v, static_cast<float>(lo), static_cast<float>(hi), fmt,
                         log ? ImGuiSliderFlags_Logarithmic : ImGuiSliderFlags_None);
  const bool committed = ImGui::IsItemDeactivatedAfterEdit();
  return {changed, committed, static_cast<double>(v)};
#else
  (void)label;
  (void)key;
  (void)lo;
  (void)hi;
  (void)fmt;
  (void)log;
  (void)in_row;
  return {false, false, current};
#endif
}
} // namespace

ariadne::DoubleEdit ImGuiBackend::slider_double(const char *label, const char *key, double current,
                                                double lo, double hi, const char *fmt) {
  return imgui_slider_double(label, key, current, lo, hi, fmt, false, in_flow_row());
}

ariadne::DoubleEdit ImGuiBackend::slider_double_log(const char *label, const char *key,
                                                    double current, double lo, double hi,
                                                    const char *fmt) {
  return imgui_slider_double(label, key, current, lo, hi, fmt, true, in_flow_row());
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
  reserve_label_width(label, in_flow_row());
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

// The caption first (the part before any "##" id suffix), then one radio button per option, on
// the caption's line for a row. Each option's id is scoped by the label, so two radios may share
// option names.
ariadne::IndexEdit ImGuiBackend::radio(const char *label, int current_index,
                                       const std::vector<std::string> &options, bool row) {
  ariadne::IndexEdit e;
  e.index = current_index;
  if (options.empty())
    return e;
#ifdef CVC_ENABLE_IMGUI
  const char *hash = std::strstr(label, "##");
  const std::string caption = hash ? std::string(label, hash) : std::string(label);
  ImGui::PushID(label);
  if (!caption.empty())
    ImGui::TextUnformatted(caption.c_str());
  for (std::size_t i = 0; i < options.size(); ++i) {
    if (row && (i > 0 || !caption.empty()))
      ImGui::SameLine();
    ImGui::PushID(static_cast<int>(i));
    if (ImGui::RadioButton(options[i].c_str(), static_cast<int>(i) == current_index)) {
      e.index = static_cast<int>(i); // discrete: commit on pick
      e.changed = true;
      e.committed = true;
    }
    ImGui::PopID();
  }
  ImGui::PopID();
#else
  (void)label;
  (void)row;
#endif
  return e;
}

void ImGuiBackend::focus_next_widget() {
#ifdef CVC_ENABLE_IMGUI
  ImGui::SetKeyboardFocusHere(); // the next widget submitted
#endif
}

ariadne::StringEdit ImGuiBackend::input_text(const char *label, const char *key,
                                             const std::string &current, bool masked) {
#ifdef CVC_ENABLE_IMGUI
  // Hold the in-progress text across frames keyed by `key`; follow external writes (script / config
  // / replicated peer) while no field is being edited, exactly like cache_int/cache_float.
  std::string &buf = m_textEdits[key];
  if (!ImGui::IsAnyItemActive())
    buf = current;
  InputTextStr ud{&buf};
  reserve_label_width(label, in_flow_row());
  // EnterReturnsTrue commits on Enter (a chat box's send); IsItemDeactivatedAfterEdit commits on
  // focus-leave; CallbackResize lets the std::string grow past its initial capacity as typed.
  // Password: ImGui draws a mask and refuses to copy the text out.
  const bool entered =
      ImGui::InputText(label, buf.data(), buf.capacity() + 1,
                       ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CallbackResize |
                           (masked ? ImGuiInputTextFlags_Password : 0),
                       input_text_resize, &ud);
  const bool committed = entered || ImGui::IsItemDeactivatedAfterEdit();
  return {buf != current, committed, buf};
#else
  (void)label;
  (void)key;
  (void)masked;
  return {false, false, current};
#endif
}

#ifdef CVC_ENABLE_IMGUI
namespace {
// S cut to at most MAX bytes, at a UTF-8 character boundary.
void truncate_utf8(std::string &s, std::size_t max) {
  if (s.size() <= max)
    return;
  std::size_t n = max;
  while (n > 0 && (static_cast<unsigned char>(s[n]) & 0xC0) == 0x80)
    --n;
  s.resize(n);
}
} // namespace

#endif

ariadne::MessageEdit ImGuiBackend::message_input(const char *label, const char *key,
                                                 const std::string &current,
                                                 const ariadne::MessageInputOptions &o) {
  ariadne::MessageEdit out;
#ifdef CVC_ENABLE_IMGUI
  MessageField &f = m_messageFields[key];
  // Live (the core writes each change): a draft in state the field did not report is the
  // document's own (a completion), which it takes even while it is being edited.
  const bool external = o.live && current != f.reported && current != f.text;
  if ((!ImGui::IsAnyItemActive() && !f.refocus) || external)
    f.text = current; // follow the draft in state while nothing is being edited
  const ImGuiStyle &st = ImGui::GetStyle();
  const char *end = label ? ImGui::FindRenderedTextEnd(label) : label;
  const bool labelled = label && end != label;
  // A send button sits on the field's right, so the field leaves it room.
  const float send_w = o.send.empty() ? 0.0f
                                      : ImGui::CalcTextSize(o.send.c_str()).x +
                                            2.0f * st.FramePadding.x + st.ItemInnerSpacing.x;
  const float width = labelled        ? ImGui::CalcItemWidth() - send_w
                      : send_w > 0.0f ? ImMax(ImGui::GetContentRegionAvail().x - send_w, 1.0f)
                                      : -FLT_MIN;
  ImGui::PushID(label ? label : "message");
  if (external) // the field being edited shows the new text, the cursor after it
    if (ImGuiInputTextState *s = ImGui::GetInputTextState(ImGui::GetID("##message")))
      s->ReloadUserBufAndMoveToEnd();
  if (f.refocus) {
    ImGui::SetKeyboardFocusHere();
    f.refocus = false;
  }
  InputTextStr ud{&f.text};
  bool entered = false;
  if (o.multiline) {
    // As tall as the wrapped text, from one line to max_lines, then it scrolls.
    const float inner = (width > 0.0f ? width : ImGui::GetContentRegionAvail().x) -
                        2.0f * st.FramePadding.x - st.ScrollbarSize;
    const float lh = ImGui::GetTextLineHeight();
    float th = f.text.empty() ? lh : ImGui::CalcTextSize(f.text.c_str(), nullptr, false, inner).y;
    if (!f.text.empty() && f.text.back() == '\n')
      th += lh; // the caret's line after a trailing newline
    const float rows = ImClamp(std::floor(th / lh + 0.5f), 1.0f,
                               static_cast<float>(o.max_lines < 1 ? 1 : o.max_lines));
    entered = ImGui::InputTextMultiline(
        "##message", f.text.data(), f.text.capacity() + 1,
        ImVec2(width, rows * lh + 2.0f * st.FramePadding.y),
        ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CtrlEnterForNewLine |
            ImGuiInputTextFlags_WordWrap | ImGuiInputTextFlags_NoHorizontalScroll |
            ImGuiInputTextFlags_CallbackResize,
        input_text_resize, &ud);
  } else {
    ImGui::SetNextItemWidth(width);
    entered =
        ImGui::InputText("##message", f.text.data(), f.text.capacity() + 1,
                         ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CallbackResize,
                         input_text_resize, &ud);
  }
  const bool active = ImGui::IsItemActive();
  const bool left = ImGui::IsItemDeactivatedAfterEdit();
  if (f.text.empty() && !active && !o.hint.empty())
    ImGui::GetWindowDrawList()->AddText(ImVec2(ImGui::GetItemRectMin().x + st.FramePadding.x,
                                               ImGui::GetItemRectMin().y + st.FramePadding.y),
                                        ImGui::GetColorU32(ImGuiCol_TextDisabled), o.hint.c_str());
  if (!o.send.empty()) { // as Enter: it sends, and the focus goes back to the field
    ImGui::SameLine(0.0f, st.ItemInnerSpacing.x);
    if (ImGui::Button(o.send.c_str()))
      entered = true;
  }
  ImGui::PopID();
  if (labelled) {
    ImGui::SameLine(0.f, st.ItemInnerSpacing.x);
    ImGui::TextUnformatted(label, end);
  }
  if (o.max_length > 0)
    truncate_utf8(f.text, static_cast<std::size_t>(o.max_length));
  if (entered) {
    // Sent, unless there is nothing but blanks; either way the focus comes back next frame.
    if (f.text.find_first_not_of(" \t\r\n") != std::string::npos) {
      out.submitted = true;
      out.value = f.text;
      f.text.clear();
    }
    f.refocus = true;
  } else {
    out.committed = left;
    out.value = f.text;
  }
  out.changed = f.text != current;
  f.reported = out.submitted ? std::string() : f.text;
#else
  (void)label;
  (void)key;
  (void)o;
  out.value = current;
#endif
  return out;
}

ariadne::TextViewEvent ImGuiBackend::text_view(const char *label, const char *key,
                                               const std::string &text,
                                               const ariadne::TextViewOptions &o) {
  ariadne::TextViewEvent ev;
#ifdef CVC_ENABLE_IMGUI
  std::shared_ptr<TextViewState> &sp = m_textViews[key];
  if (!sp)
    sp = std::make_shared<TextViewState>();
  TextViewState &v = *sp;
  const ImGuiStyle &st = ImGui::GetStyle();
  ImGuiWindow *host = ImGui::GetCurrentWindow();
  const bool fit = (host->Flags & ImGuiWindowFlags_AlwaysAutoResize) != 0;
  const float lh = ImGui::GetTextLineHeightWithSpacing();
  // A window that fits its content has no space to fill: 12 lines, 28 font sizes wide.
  const float w = fit ? 28.0f * ImGui::GetFontSize() : ImGui::GetContentRegionAvail().x;
  float h = o.height > 0.0f ? o.height
            : fit           ? 12.0f * lh + 2.0f * st.WindowPadding.y
                            : ImGui::GetContentRegionAvail().y - v.below;
  h = ImMax(h, 3.0f * lh + 2.0f * st.WindowPadding.y);
  const float wrap_w =
      o.wrap ? ImMax(w - 2.0f * st.WindowPadding.x - st.ScrollbarSize, 16.0f) : -1.0f;

  // The layout, redone when the text or the width changes. Lines added above (an older page)
  // move the scroll by their height, so what is in view stays put; another text altogether (a
  // channel switch) starts at its bottom.
  if (text != v.text) {
    const float old_total = v.tops.empty() ? 0.0f : v.tops.back();
    const bool grew = !v.text.empty() && text.size() > v.text.size();
    const bool prepended =
        grew && text.compare(text.size() - v.text.size(), v.text.size(), v.text) == 0;
    const bool appended = grew && !prepended && text.compare(0, v.text.size(), v.text) == 0;
    v.text = text;
    v.layout(wrap_w);
    v.reported = false;
    if (prepended) // still on its way to the bottom: it goes on there
      v.target = v.target == FLT_MAX
                     ? FLT_MAX
                     : (v.target >= 0.0f ? v.target : v.scroll) + (v.tops.back() - old_total);
    else if (!appended || (o.follow && v.at_bottom))
      v.target = FLT_MAX;
    v.target_frames = 0;
  } else if (wrap_w != v.wrap_w) {
    v.layout(wrap_w);
    if (v.at_bottom)
      v.target = FLT_MAX;
  }
  const float total = v.tops.back();
  const float view_h = h - 2.0f * st.WindowPadding.y;
  const float max_scroll = ImMax(0.0f, total - view_h);
  if (v.target >= 0.0f)
    ImGui::SetNextWindowScroll(ImVec2(-1.0f, ImMin(v.target, max_scroll)));

  bool wheel_up_at_top = false;
  ImGui::BeginChild(label && *label ? label : "##text_view", ImVec2(fit ? w : 0.0f, h),
                    ImGuiChildFlags_Borders);
  {
    const float scroll = ImGui::GetScrollY();
    const std::size_t n = v.lines.size();
    // The first line in view, then each one down to the bottom of the view.
    std::size_t i = static_cast<std::size_t>(
        std::upper_bound(v.tops.begin(), v.tops.begin() + static_cast<std::ptrdiff_t>(n), scroll) -
        v.tops.begin());
    i = i > 0 ? i - 1 : 0;
    if (wrap_w > 0.0f)
      ImGui::PushTextWrapPos(0.0f);
    for (; i < n && v.tops[i] < scroll + view_h; ++i) {
      ImGui::SetCursorPos(ImVec2(st.WindowPadding.x, st.WindowPadding.y + v.tops[i]));
      const auto &ln = v.lines[i];
      ImGui::TextUnformatted(v.text.data() + ln.first, v.text.data() + ln.second);
    }
    if (wrap_w > 0.0f)
      ImGui::PopTextWrapPos();
    ImGui::SetCursorPos(ImVec2(st.WindowPadding.x, st.WindowPadding.y + total));
    ImGui::Dummy(ImVec2(0.0f, 0.0f)); // the content's height, so the scroll range is the text's
    v.scroll = ImGui::GetScrollY();
    v.at_bottom = v.scroll >= max_scroll - 1.0f;
    // The target is reached (or given up on, so the user's scroll is never fought for long).
    if (v.target >= 0.0f &&
        (std::fabs(v.scroll - ImMin(v.target, max_scroll)) < 1.0f || ++v.target_frames > 8))
      v.target = -1.0f;
    wheel_up_at_top =
        ImGui::IsWindowHovered() && ImGui::GetIO().MouseWheel > 0.0f && v.scroll <= 0.5f;
    if (ImGui::BeginPopupContextWindow("##text_view_menu")) {
      if (ImGui::MenuItem("Copy"))
        ImGui::SetClipboardText(v.text.c_str());
      ImGui::EndPopup();
    }
  }
  ImGui::EndChild();
  v.end_y = ImGui::GetCursorPosY();
  m_lastTextView = sp;
  m_lastTextViewWindow = host;

  // The top: reached by scrolling (once, until the view leaves it), the wheel turned up there
  // (at most twice a second), or a text that does not fill the view (once per text, an empty one
  // aside: there is room above it for more).
  // Nothing is reported while the view is still being taken somewhere.
  const double now = ImGui::GetTime();
  const bool at_top = v.scroll <= 0.5f;
  if (total <= view_h + 0.5f) {
    if (!v.reported && !v.text.empty()) {
      ev.at_top = true;
      v.reported = true;
    }
  } else if (v.target >= 0.0f) {
    v.armed = !at_top;
  } else if (at_top && v.armed) {
    ev.at_top = true;
    v.armed = false;
  } else if (!at_top) {
    v.armed = true;
  }
  if (wheel_up_at_top && v.target < 0.0f && now - v.fired_at >= 0.5)
    ev.at_top = true;
  if (ev.at_top)
    v.fired_at = now;
#else
  (void)label;
  (void)key;
  (void)text;
  (void)o;
#endif
  return ev;
}

ariadne::ColorEdit ImGuiBackend::color(const char *label, const float rgb[3]) {
  ariadne::ColorEdit e;
  e.rgb[0] = rgb[0];
  e.rgb[1] = rgb[1];
  e.rgb[2] = rgb[2];
#ifdef CVC_ENABLE_IMGUI
  e.drawn = true;
  reserve_label_width(label, in_flow_row());
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

void ImGuiBackend::register_widget(cvc::app &app, std::string custom_type, CustomDrawFn draw) {
  if (draw) {
    // Tell the core loader this type is handled (natively, by this backend) so it does not warn at
    // load that the type is unregistered and fall back to a placeholder — has_widget_type() reports
    // it present and the draw routes here via custom_widget(). (e.g. the stream_view peer-video
    // tile.)
    cvc::ariadne::register_known_widget_type(app, custom_type);
    m_customWidgets[std::move(custom_type)] = std::move(draw);
  }
}

void ImGuiBackend::set_image(const std::string &name, const cvc::image &img) {
  PublishedImage &p = m_images[name];
  p.img = img; // copy-on-write; no pixel copy until data() detaches (it won't — we read const)
  p.dirty = true;
}

void ImGuiBackend::drop_image(const std::string &name) {
  const auto it = m_images.find(name);
  if (it == m_images.end())
    return;
#ifdef CVC_ENABLE_IMGUI
  if (it->second.tex != 0)
    glDeleteTextures(1, &it->second.tex);
#endif
  m_images.erase(it);
}

#ifdef CVC_ENABLE_IMGUI
unsigned int ImGuiBackend::image_texture(const char *name, int &W, int &H) {
  const auto it = m_images.find(name ? name : "");
  if (it == m_images.end())
    return 0;
  PublishedImage &p = it->second;
  if (p.img.width() <= 0 || p.img.height() <= 0)
    return 0;
  // Upload as RGBA8. `const` so data() reads the shared buffer without a COW detach.
  const cvc::image rgba =
      (p.img.format() == cvc::image::pixel_format::RGBA)
          ? p.img
          : p.img.converted(cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  W = rgba.width();
  H = rgba.height();
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
  return p.tex;
}
#endif

bool ImGuiBackend::draw_image(const char *name, float width) {
#ifdef CVC_ENABLE_IMGUI
  int W = 0, H = 0;
  const unsigned int tex = image_texture(name, W, H);
  if (tex == 0)
    return false;
  width = ui_px(width); // a document's px (the image widget's size:, a stream view's)
  const float h = width * static_cast<float>(H) / static_cast<float>(W);
  ImGui::Image(static_cast<ImTextureID>(static_cast<std::intptr_t>(tex)), ImVec2(width, h));
  return true;
#else
  (void)name;
  (void)width;
  return false;
#endif
}

#ifdef CVC_ENABLE_IMGUI
void ImGuiBackend::cut_window_background(float x0, float y0, float x1, float y1) {
  if (m_windowCuts.empty() || !(x1 > x0) || !(y1 > y0))
    return;
  m_windowCuts.back()->holes.emplace_back(x0, y0, x1, y1);
}

// The window's background below its title bar, less its holes: for each band between the holes'
// top and bottom edges, the stretches of it no hole covers.
void ImGuiBackend::paint_window_background(WindowCuts &cuts) {
  ImGuiWindow *win = ImGui::GetCurrentWindow();
  ImDrawList *dl = cuts.dl;
  cuts.split.SetCurrentChannel(dl, 0);
  const ImRect bg(win->Pos.x, win->InnerRect.Min.y, win->Pos.x + win->Size.x,
                  win->Pos.y + win->Size.y);
  if (bg.Max.y > bg.Min.y && !win->Collapsed) {
    const ImU32 col = ImGui::GetColorU32(ImGuiCol_WindowBg);
    std::vector<ImRect> holes;
    std::vector<float> ys{bg.Min.y, bg.Max.y};
    for (ImRect h : cuts.holes) {
      h.ClipWithFull(bg);
      if (h.Max.x > h.Min.x && h.Max.y > h.Min.y) {
        holes.push_back(h);
        ys.push_back(h.Min.y);
        ys.push_back(h.Max.y);
      }
    }
    std::sort(ys.begin(), ys.end());
    ys.erase(std::unique(ys.begin(), ys.end()), ys.end());
    dl->PushClipRect(bg.Min, bg.Max, false);
    for (std::size_t i = 0; i + 1 < ys.size(); ++i) {
      const float ya = ys[i], yb = ys[i + 1];
      std::vector<std::pair<float, float>> covered;
      for (const ImRect &h : holes)
        if (h.Min.y < yb && h.Max.y > ya)
          covered.emplace_back(h.Min.x, h.Max.x);
      std::sort(covered.begin(), covered.end());
      float x = bg.Min.x;
      for (const auto &c : covered) {
        if (c.first > x)
          dl->AddRectFilled(ImVec2(x, ya), ImVec2(c.first, yb), col);
        x = std::max(x, c.second);
      }
      if (bg.Max.x > x)
        dl->AddRectFilled(ImVec2(x, ya), ImVec2(bg.Max.x, yb), col);
    }
    dl->PopClipRect();
  }
  cuts.split.Merge(dl);
}

struct ImGuiBackend::BackdropFrame {
  cvc::ariadne::Backdrop bd;
  ImDrawList *dl = nullptr;
  ImDrawListSplitter split;
  ImVec2 start;
  float avail_w = 0.0f, avail_h = 0.0f;
};
struct ImGuiBackend::ItemGroupFrame {
  std::string id;
  cvc::ariadne::ItemGroupOptions opts;
  ImDrawList *dl = nullptr;
  ImDrawListSplitter split;
  float right = 0.0f; // the right edge of the room it started with
};
#else
void ImGuiBackend::cut_window_background(float, float, float, float) {}
struct ImGuiBackend::BackdropFrame {};
struct ImGuiBackend::ItemGroupFrame {};
#endif

bool ImGuiBackend::set_clipboard(cvc::app &app, const std::string &text) {
  return cvc::gl::clipboard::set(app, text);
}

void ImGuiBackend::set_fullscreen_handler(std::function<bool(bool)> set, std::function<bool()> is) {
  m_setFullscreen = std::move(set);
  m_isFullscreen = std::move(is);
}
bool ImGuiBackend::set_fullscreen(bool on) { return m_setFullscreen && m_setFullscreen(on); }
bool ImGuiBackend::is_fullscreen() const { return m_isFullscreen && m_isFullscreen(); }

cvc::ariadne::FullscreenView ImGuiBackend::begin_fullscreen_view() {
  cvc::ariadne::FullscreenView v;
#ifdef CVC_ENABLE_IMGUI
  const ImGuiViewport *vp = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(vp->Pos);
  ImGui::SetNextWindowSize(vp->Size);
  ImGui::SetNextWindowFocus(); // over everything, and the keys are its
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::PushStyleColor(ImGuiCol_WindowBg, IM_COL32(0, 0, 0, 255));
  ImGui::Begin("##ari_fullscreen", nullptr,
               ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollWithMouse);
  ImGui::PopStyleColor();
  ImGui::PopStyleVar(2);
  m_inFullscreenView = true;
  const float scale = ui_px(1.0f) > 0.0f ? ui_px(1.0f) : 1.0f;
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  v.shown = true;
  v.w = avail.x / scale;
  v.h = avail.y / scale;
  v.controls_h = ImGui::GetFrameHeightWithSpacing() / scale;
  v.exit = ImGui::IsKeyPressed(ImGuiKey_Escape, false);
#endif
  return v;
}

void ImGuiBackend::end_fullscreen_view() {
#ifdef CVC_ENABLE_IMGUI
  if (m_inFullscreenView)
    ImGui::End();
  m_inFullscreenView = false;
#endif
}

void ImGuiBackend::begin_item_group(const char *id, const cvc::ariadne::ItemGroupOptions &opts) {
#ifdef CVC_ENABLE_IMGUI
  auto f = std::make_shared<ItemGroupFrame>();
  f->id = std::string("##ari_group:") + (id ? id : "");
  f->opts = opts;
  f->dl = ImGui::GetWindowDrawList();
  // The content on channel 1, so the highlight, drawn on channel 0 once the area is known, is
  // under it.
  f->split.Split(f->dl, 2);
  f->split.SetCurrentChannel(f->dl, 1);
  f->right = ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
  ImGui::BeginGroup();
  m_itemGroups.push_back(std::move(f));
#else
  (void)id;
  (void)opts;
#endif
}

void ImGuiBackend::end_item_group() {
#ifdef CVC_ENABLE_IMGUI
  if (m_itemGroups.empty())
    return;
  std::shared_ptr<ItemGroupFrame> f = std::move(m_itemGroups.back());
  m_itemGroups.pop_back();
  ImGui::EndGroup();
  // As wide as the room it had (a feed's card: the whole row is the post), at least its content.
  const ImVec2 mn = ImGui::GetItemRectMin();
  const ImVec2 mx(std::max(ImGui::GetItemRectMax().x, f->right), ImGui::GetItemRectMax().y);
  // The group's item: a button over all of it, submitted after what it holds, so a button, a
  // checkbox or a field in it (which took the hover first) keeps its own clicks. It takes the
  // place EndGroup gave the group, so the layout after it is the same. A click is a tap: released
  // over it, the pointer not dragged far (a finger scrolling the feed opens nothing). Keyboard
  // navigation reaches it when it is focusable.
  ImGui::SetCursorScreenPos(mn);
  const ImGuiButtonFlags flags =
      f->opts.focusable ? ImGuiButtonFlags_EnableNav : ImGuiButtonFlags_None;
  const bool pressed = ImGui::InvisibleButton(
      f->id.c_str(), ImVec2(std::max(1.0f, mx.x - mn.x), std::max(1.0f, mx.y - mn.y)), flags);
  const float slop = ImGui::GetIO().MouseDragThreshold;
  m_groupHitId = ImGui::GetItemID();
  m_groupHit =
      pressed && (!ImGui::GetIO().MouseReleased[ImGuiMouseButton_Left] ||
                  ImGui::GetIO().MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] <= slop * slop);
  f->split.SetCurrentChannel(f->dl, 0);
  if (f->opts.highlight && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem)) {
    const ImGuiCol col = ImGui::IsItemActive() ? ImGuiCol_HeaderActive : ImGuiCol_HeaderHovered;
    f->dl->AddRectFilled(mn, mx, ImGui::GetColorU32(col), ImGui::GetStyle().FrameRounding);
  }
  f->split.Merge(f->dl);
#endif
}

void ImGuiBackend::begin_backdrop(const cvc::ariadne::Backdrop &bd) {
#ifdef CVC_ENABLE_IMGUI
  auto f = std::make_shared<BackdropFrame>();
  f->bd = bd;
  f->dl = ImGui::GetWindowDrawList();
  f->start = ImGui::GetCursorScreenPos();
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  f->avail_w = avail.x;
  f->avail_h = avail.y;
  // The content on channel 1, so the picture, drawn last on channel 0, ends up under it.
  f->split.Split(f->dl, 2);
  f->split.SetCurrentChannel(f->dl, 1);
  ImGui::BeginGroup();
  m_backdrops.push_back(std::move(f));
#else
  (void)bd;
#endif
}

void ImGuiBackend::end_backdrop() {
#ifdef CVC_ENABLE_IMGUI
  if (m_backdrops.empty())
    return;
  std::shared_ptr<BackdropFrame> f = std::move(m_backdrops.back());
  m_backdrops.pop_back();
  const cvc::ariadne::Backdrop &bd = f->bd;
  // A group's area: at least its authored size (px at the UI's size, or percent of the space it
  // started in), and the full width available where it set none, as a banner is.
  const auto extent = [](const cvc::ariadne::Extent &e, float avail) {
    if (e.unit == cvc::ariadne::Unit::Px)
      return ui_px(e.value);
    if (e.unit == cvc::ariadne::Unit::Percent)
      return avail * e.value / 100.0f;
    return 0.0f;
  };
  const float want_w = bd.size.w.is_set() ? extent(bd.size.w, f->avail_w) : f->avail_w;
  const float want_h = extent(bd.size.h, f->avail_h);
  const ImVec2 now = ImGui::GetCursorScreenPos();
  if (!bd.window && want_h > now.y - f->start.y)
    ImGui::Dummy(ImVec2(0.0f, want_h - (now.y - f->start.y)));
  ImGui::EndGroup();
  ImVec2 mn = ImGui::GetItemRectMin(), mx = ImGui::GetItemRectMax();
  if (bd.window) {
    const ImRect inner = ImGui::GetCurrentWindow()->InnerRect;
    mn = inner.Min;
    mx = inner.Max;
  } else {
    mx.x = std::max(mx.x, mn.x + want_w);
  }
  f->split.SetCurrentChannel(f->dl, 0);
  int W = 0, H = 0;
  const unsigned int tex = bd.image.empty() ? 0 : image_texture(bd.image.c_str(), W, H);
  if (tex != 0 && mx.x > mn.x && mx.y > mn.y) {
    const float rw = mx.x - mn.x, rh = mx.y - mn.y;
    ImVec2 p0 = mn, p1 = mx, uv0(0.0f, 0.0f), uv1(1.0f, 1.0f);
    if (bd.fit == cvc::ariadne::ImageFit::Cover) { // the middle of the picture, the area filled
      const float k = std::max(rw / W, rh / H);
      const float u = std::min(1.0f, rw / (W * k)), v = std::min(1.0f, rh / (H * k));
      uv0 = ImVec2((1.0f - u) * 0.5f, (1.0f - v) * 0.5f);
      uv1 = ImVec2(uv0.x + u, uv0.y + v);
    } else if (bd.fit == cvc::ariadne::ImageFit::Contain) { // all of the picture, centred
      const float k = std::min(rw / W, rh / H);
      const ImVec2 size(W * k, H * k);
      p0 = ImVec2(mn.x + (rw - size.x) * 0.5f, mn.y + (rh - size.y) * 0.5f);
      p1 = ImVec2(p0.x + size.x, p0.y + size.y);
    }
    const int alpha = std::clamp(static_cast<int>(bd.opacity * 255.0f + 0.5f), 0, 255);
    f->dl->PushClipRect(mn, mx, true);
    f->dl->AddImage(static_cast<ImTextureID>(static_cast<std::intptr_t>(tex)), p0, p1, uv0, uv1,
                    IM_COL32(255, 255, 255, alpha));
    f->dl->PopClipRect();
  }
  f->split.Merge(f->dl);
#endif
}

bool ImGuiBackend::draw_image_fit(const char *name, float box_w, float box_h) {
#ifdef CVC_ENABLE_IMGUI
  int W = 0, H = 0;
  const unsigned int tex = image_texture(name, W, H);
  if (tex == 0 || !(box_w > 0.0f) || !(box_h > 0.0f))
    return false;
  box_w = ui_px(box_w);
  box_h = ui_px(box_h);
  const float s = std::min(box_w / static_cast<float>(W), box_h / static_cast<float>(H));
  const ImVec2 size(static_cast<float>(W) * s, static_cast<float>(H) * s);
  // The box (black bars where the picture leaves it), the picture centred in it, then the box's
  // space in the layout.
  const ImVec2 p0 = ImGui::GetCursorScreenPos();
  const ImVec2 at(p0.x + 0.5f * (box_w - size.x), p0.y + 0.5f * (box_h - size.y));
  ImDrawList *dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(p0, ImVec2(p0.x + box_w, p0.y + box_h), IM_COL32(0, 0, 0, 255));
  dl->AddImage(static_cast<ImTextureID>(static_cast<std::intptr_t>(tex)), at,
               ImVec2(at.x + size.x, at.y + size.y));
  ImGui::Dummy(ImVec2(box_w, box_h));
  return true;
#else
  (void)name;
  (void)box_w;
  (void)box_h;
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

void ImGuiBackend::setImGuiContext(ImGuiContext *ctx) {
#ifdef CVC_ENABLE_IMGUI
  ImGui::SetCurrentContext(ctx);
#else
  (void)ctx;
#endif
}

} // namespace gl
} // namespace cvc
