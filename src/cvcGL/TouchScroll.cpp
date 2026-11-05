// TouchScroll — scrolling with a finger, and the glide after it (see the header).

#include <algorithm>
#include <cmath>
#include <cvc/gl/TouchScroll.h>

#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#include <imgui_internal.h> // ImGuiWindow (Scroll, ScrollMax), ActiveId, StateStorage
#endif

namespace cvc {
namespace gl {

// ---- KineticScroll -----------------------------------------------------------
void KineticScroll::fling(float speed) { m_speed = std::isfinite(speed) ? speed : 0.0f; }
void KineticScroll::stop() { m_speed = 0.0f; }

float KineticScroll::step(float dt, float friction, float stopSpeed) {
  if (m_speed == 0.0f || !(dt > 0.0f))
    return 0.0f;
  // The exact distance under v(t) = v0 * exp(-f t), so the glide's length does not depend on the
  // frame rate: v0 * (1 - exp(-f dt)) / f, and v0 * dt with no friction.
  const float keep = std::exp(-friction * dt);
  const float d = friction > 0.0f ? m_speed * (1.0f - keep) / friction : m_speed * dt;
  m_speed *= keep;
  if (std::fabs(m_speed) < stopSpeed)
    m_speed = 0.0f;
  return d;
}

// ---- VelocityTracker ---------------------------------------------------------
void VelocityTracker::add(double t, float x, float y) {
  m_samples[m_count % kSamples] = {t, x, y};
  ++m_count;
}

void VelocityTracker::velocity(double now, float window, float &vx, float &vy) const {
  vx = vy = 0.0f;
  const int held = std::min(m_count, kSamples);
  if (held < 2)
    return;
  const auto at = [&](int back) -> const Sample & { // 0 = the newest
    return m_samples[(m_count - 1 - back) % kSamples];
  };
  const Sample &newest = at(0);
  if (newest.t < now - window) // nothing in the window: the finger has been still
    return;
  int oldest = 0;
  while (oldest + 1 < held && at(oldest + 1).t >= now - window)
    ++oldest;
  if (oldest == 0) // one sample in the window: measure from the one before it
    oldest = 1;
  const Sample &o = at(oldest);
  const double dt = newest.t - o.t;
  if (!(dt > 1e-6))
    return;
  vx = static_cast<float>((newest.x - o.x) / dt);
  vy = static_cast<float>((newest.y - o.y) / dt);
}

// ---- TouchScroll -------------------------------------------------------------
void TouchScroll::setSettings(Settings s) {
  if (!std::isfinite(s.friction))
    s.friction = Settings{}.friction;
  s.friction = std::clamp(s.friction, 0.0f, kMaxFriction);
  m_settings = s;
}

#ifdef CVC_ENABLE_IMGUI
namespace {

// In logical px (times the display's pixel ratio, style.FontScaleDpi) and seconds.
constexpr float kVelocityWindow = 0.1f; // the finger's speed: over its last 100 ms
constexpr float kStopSpeed = 20.0f;     // a glide slower than this has stopped
constexpr float kMaxSpeed = 8000.0f;    // a fling's cap (one jittery frame is not a fling)
constexpr float kCatchSpeed = 50.0f;    // a press on content gliding faster stops it, not a tap

// The pointer while a finger's drag scrolls: active, so no widget under the finger takes it.
ImGuiID scrollId() {
  static const ImGuiID id = ImHashStr("##cvcgl_touch_scroll");
  return id;
}
// Where keepTouchDrag notes its item, in the item's window's storage.
ImGuiID keepDragKey() {
  static const ImGuiID key = ImHashStr("cvcgl.keep_touch_drag");
  return key;
}

float dpiOf(const ImGuiContext &g) {
  return g.Style.FontScaleDpi > 0.0f ? g.Style.FontScaleDpi : 1.0f;
}

// The innermost window from `w` out through its parents (child windows only) that scrolls along
// `axis` with the pointer: the window the mouse wheel would scroll.
ImGuiWindow *scrollable(ImGuiWindow *w, int axis) {
  for (; w; w = (w->Flags & ImGuiWindowFlags_ChildWindow) ? w->ParentWindow : nullptr)
    if (!(w->Flags & (ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoMouseInputs)) &&
        w->ScrollMax[axis] > 0.0f)
      return w;
  return nullptr;
}

bool within(ImGuiWindow *w, ImGuiID target) {
  for (; w; w = (w->Flags & ImGuiWindowFlags_ChildWindow) ? w->ParentWindow : nullptr)
    if (w->ID == target)
      return true;
  return false;
}

// May a drag that began as `g`'s active widget become a scroll? Not one that takes the drag itself.
bool dragIsFree(ImGuiContext &g, ImGuiID activeAtPress) {
  const ImGuiID id = g.ActiveId;
  if (id == 0)
    return true;
  if (g.MovingWindow) // a title bar: the drag moves the window
    return false;
  ImGuiWindow *aw = g.ActiveIdWindow;
  if (aw && id == aw->MoveId) // a window's empty space (a finger moves it by its title only)
    return true;
  if (id == activeAtPress) // the press went on with what was active: a field being edited
    return false;
  switch (ImGui::GetMouseCursor()) { // a resize grip or border, a table's column border
  case ImGuiMouseCursor_ResizeAll:
  case ImGuiMouseCursor_ResizeNS:
  case ImGuiMouseCursor_ResizeEW:
  case ImGuiMouseCursor_ResizeNESW:
  case ImGuiMouseCursor_ResizeNWSE:
    return false;
  default:
    break;
  }
  if (aw) {
    if (id == ImGui::GetWindowScrollbarID(aw, ImGuiAxis_X) ||
        id == ImGui::GetWindowScrollbarID(aw, ImGuiAxis_Y))
      return false;
    for (int n = 0; n < 4; ++n)
      if (id == ImGui::GetWindowResizeCornerID(aw, n))
        return false;
    for (int dir = 0; dir < 4; ++dir)
      if (id == ImGui::GetWindowResizeBorderID(aw, static_cast<ImGuiDir>(dir)))
        return false;
    if (static_cast<ImGuiID>(aw->StateStorage.GetInt(keepDragKey(), 0)) == id)
      return false;
  }
  // A field this press made active is scrolled past, as on a phone: it was not being edited.
  if (g.InputTextState.ID == id)
    return true;
  // A slider or a drag widget declares the arrows it reads (ActiveIdUsingNavDirMask); a button,
  // a selectable, a checkbox declares none, and its press is let go.
  return g.ActiveIdUsingNavDirMask == 0;
}

// Asks `w` for `scroll` along `axis` (ImGui applies it at the window's Begin, this frame).
void ask(ImGuiWindow *w, int axis, float scroll) {
  if (axis)
    ImGui::SetScrollY(w, scroll);
  else
    ImGui::SetScrollX(w, scroll);
}

} // namespace

void TouchScroll::take(unsigned window, int axis) {
  ImGuiContext &g = *GImGui;
  ImGuiWindow *w = ImGui::FindWindowByID(window);
  if (!w)
    return;
  // The widget pressed lets go: a button does not fire, and nothing under the finger takes the
  // pointer while the content moves.
  ImGui::SetActiveID(scrollId(), w);
  m_target = window;
  m_axis = axis;
  m_anchorPointer = axis ? g.IO.MousePos.y : g.IO.MousePos.x;
  m_anchorScroll = m_applied = w->Scroll[axis];
  m_phase = Phase::Dragging;
}

void TouchScroll::beginFrame() {
  ImGuiContext *ctx = ImGui::GetCurrentContext();
  if (!ctx)
    return;
  ImGuiContext &g = *ctx;
  ImGuiIO &io = g.IO;
  if (!m_settings.enabled) {
    reset();
    return;
  }
  const float dpi = dpiOf(g);
  const bool valid = ImGui::IsMousePosValid(&io.MousePos);

  if (io.MouseClicked[0]) {
    // A press on content gliding at speed stops it, and is the content's, not a tap on what the
    // glide brought under the finger: it may drag it on, or fling it again.
    const bool caught = m_phase == Phase::Gliding &&
                        std::fabs(m_glide.speed()) > kCatchSpeed * dpi &&
                        within(g.HoveredWindow, m_target);
    m_glide.stop();
    m_tracker.reset();
    if (valid)
      m_tracker.add(g.Time, io.MousePos.x, io.MousePos.y);
    m_pressWindow = g.HoveredWindow ? g.HoveredWindow->ID : 0;
    m_activeAtPress = g.ActiveId;
    m_decided = false;
    m_phase = Phase::Pressed;
    if (caught) {
      m_decided = true;
      take(m_target, m_axis);
    }
    return;
  }

  switch (m_phase) {
  case Phase::Idle:
    break;
  case Phase::Pressed:
    if (!io.MouseDown[0])
      m_phase = Phase::Idle;
    else if (valid)
      m_tracker.add(g.Time, io.MousePos.x, io.MousePos.y);
    break;
  case Phase::Dragging: {
    ImGuiWindow *w = ImGui::FindWindowByID(m_target);
    if (g.ActiveId != scrollId() || !w || !w->WasActive) { // the window went, or took the pointer
      if (g.ActiveId == scrollId())
        ImGui::ClearActiveID();
      m_phase = Phase::Idle;
      break;
    }
    if (io.MouseDown[0]) {
      ImGui::KeepAliveID(scrollId());
      if (valid) {
        m_tracker.add(g.Time, io.MousePos.x, io.MousePos.y);
        const float p = m_axis ? io.MousePos.y : io.MousePos.x;
        m_applied = std::clamp(m_anchorScroll - (p - m_anchorPointer), 0.0f, w->ScrollMax[m_axis]);
        ask(w, m_axis, m_applied);
      }
      break;
    }
    // Lifted: the content glides on at the finger's speed (a finger moving up scrolls it down).
    ImGui::ClearActiveID();
    if (valid)
      m_tracker.add(g.Time, io.MousePos.x, io.MousePos.y);
    float vx = 0.0f, vy = 0.0f;
    m_tracker.velocity(g.Time, kVelocityWindow, vx, vy);
    const float cap = kMaxSpeed * dpi;
    const float v = std::clamp(-(m_axis ? vy : vx), -cap, cap);
    m_scroll = m_applied;
    if (std::fabs(v) < kStopSpeed * dpi) {
      m_phase = Phase::Idle;
      break;
    }
    m_glide.fling(v);
    m_phase = Phase::Gliding;
    break;
  }
  case Phase::Gliding: {
    ImGuiWindow *w = ImGui::FindWindowByID(m_target);
    // Over when its window goes, or another hand moves its scroll (the wheel, its scroll bar, a
    // text view following new lines): ImGui applied ours rounded, so within a pixel is ours.
    if (!w || !w->WasActive || w->Collapsed || std::fabs(w->Scroll[m_axis] - m_applied) > 1.5f) {
      m_glide.stop();
      m_phase = Phase::Idle;
      break;
    }
    m_scroll += m_glide.step(io.DeltaTime, m_settings.friction, kStopSpeed * dpi);
    const float hi = w->ScrollMax[m_axis];
    if (m_scroll <= 0.0f || m_scroll >= hi) { // an end: the glide stops there
      m_scroll = std::clamp(m_scroll, 0.0f, hi);
      m_glide.stop();
    }
    m_applied = m_scroll;
    ask(w, m_axis, m_applied);
    if (!m_glide.gliding())
      m_phase = Phase::Idle;
    break;
  }
  }
}

void TouchScroll::endFrame() {
  ImGuiContext *ctx = ImGui::GetCurrentContext();
  if (!ctx || !m_settings.enabled || m_phase != Phase::Pressed || m_decided)
    return;
  ImGuiContext &g = *ctx;
  const ImGuiIO &io = g.IO;
  if (!io.MouseDown[0] || !ImGui::IsMousePosValid(&io.MousePos))
    return;
  const ImVec2 d(io.MousePos.x - io.MouseClickedPos[0].x, io.MousePos.y - io.MouseClickedPos[0].y);
  const float th = io.MouseDragThreshold;
  if (d.x * d.x + d.y * d.y < th * th) // still a tap
    return;
  // Judged once: from here the drag is a scroll, or the widget's for good.
  m_decided = true;
  ImGuiWindow *pressed = ImGui::FindWindowByID(m_pressWindow);
  if (!pressed || !dragIsFree(g, m_activeAtPress))
    return;
  // A title or menu bar is the window's own: it moves by it, and its menus open from it.
  ImGuiWindow *root = pressed->RootWindow;
  if (root && !(root->Flags & ImGuiWindowFlags_NoTitleBar) &&
      root->TitleBarRect().Contains(io.MouseClickedPos[0]))
    return;
  if (root && (root->Flags & ImGuiWindowFlags_MenuBar) &&
      root->MenuBarRect().Contains(io.MouseClickedPos[0]))
    return;
  const int axis = std::fabs(d.y) >= std::fabs(d.x) ? 1 : 0;
  if (ImGuiWindow *w = scrollable(pressed, axis))
    take(w->ID, axis);
}

void TouchScroll::reset() {
  ImGuiContext *ctx = ImGui::GetCurrentContext();
  if (m_phase == Phase::Dragging && ctx && ctx->ActiveId == scrollId())
    ImGui::ClearActiveID();
  m_glide.stop();
  m_phase = Phase::Idle;
  m_decided = false;
}

void keepTouchDrag() {
  ImGuiContext *ctx = ImGui::GetCurrentContext();
  if (!ctx || !ctx->CurrentWindow || !ImGui::IsItemActive())
    return;
  ctx->CurrentWindow->StateStorage.SetInt(keepDragKey(), static_cast<int>(ctx->LastItemData.ID));
}

#else // !CVC_ENABLE_IMGUI

void TouchScroll::take(unsigned, int) {}
void TouchScroll::beginFrame() {}
void TouchScroll::endFrame() {}
void TouchScroll::reset() {}
void keepTouchDrag() {}

#endif

} // namespace gl
} // namespace cvc
