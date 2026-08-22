// ImGuiOverlay — Dear ImGui inside a cvcGL viewer (see the header).
//
// The hook pattern (render on the window's Start/Render events, input via an
// interceptor on the current interactor style) follows Jaswant Panchumarti's
// vtkDearImGuiInjector (BSD-3-Clause); this is a cvcGL-native reimplementation
// against SceneRenderer, not a vendored copy.
//
// Without CVC_ENABLE_IMGUI the whole class compiles to inert stubs, so consumers
// never need an #ifdef.

#include <cvc/gl/ImGuiOverlay.h>

#ifdef CVC_ENABLE_IMGUI

#include <cvc/gl/CameraController.h>
#include <cvc/gl/Clipboard.h>
#include <cvc/gl/SceneRenderer.h>
#include <imgui.h>
// The GL loader must match VTK's context: WebGL2/GLES3 in the browser build.
#if defined(__EMSCRIPTEN__) && !defined(IMGUI_IMPL_OPENGL_ES3)
#define IMGUI_IMPL_OPENGL_ES3
#endif
#include <backends/imgui_impl_opengl3.h>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif
#include <cstdio>
#include <cstdlib>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkNew.h>
#include <vtkOpenGLFramebufferObject.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLState.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkTimerLog.h>
#include <vtkWeakPointer.h>
#include <vtk_glad.h> // GL_DRAW_FRAMEBUFFER

namespace cvc {
namespace gl {

namespace {
// Every pointer/wheel event we route through ImGui before VTK's style sees it.
const unsigned long kMouseEvents[] = {vtkCommand::MouseMoveEvent,
                                      vtkCommand::LeftButtonPressEvent,
                                      vtkCommand::LeftButtonReleaseEvent,
                                      vtkCommand::MiddleButtonPressEvent,
                                      vtkCommand::MiddleButtonReleaseEvent,
                                      vtkCommand::RightButtonPressEvent,
                                      vtkCommand::RightButtonReleaseEvent,
                                      vtkCommand::MouseWheelForwardEvent,
                                      vtkCommand::MouseWheelBackwardEvent,
                                      vtkCommand::EnterEvent,
                                      vtkCommand::LeaveEvent};
const unsigned long kKeyEvents[] = {vtkCommand::KeyPressEvent, vtkCommand::KeyReleaseEvent,
                                    vtkCommand::CharEvent};

// Is this a touch device? In the browser, ask the browser. Natively we assume
// not: a desktop touchscreen still has a mouse, and doubling the UI for someone
// with a mouse would be obnoxious. Callers can always override with setUiScale.
bool detectTouchDevice() {
#ifdef __EMSCRIPTEN__
  return emscripten_run_script_int("(('ontouchstart' in window) || "
                                   "(navigator.maxTouchPoints > 0)) ? 1 : 0") != 0;
#else
  return false;
#endif
}

// A big, round, translucent show/hide button pinned to the bottom-right corner.
//
// ImGui has no round button, so this is an InvisibleButton (whose hit area is
// its bounding square — a feature for fingers) with a circle drawn over it. It
// lives in its own window because that is what makes ImGui claim the pointer
// over it; a foreground draw-list would paint fine but let clicks fall through
// to the camera. Zero window padding keeps the claimed rect exactly the size of
// the button, so it steals as little of the scene as possible.
bool floatingCircleToggle(const char *id, bool *open, float diameter, const char *glyph) {
  const ImGuiViewport *vp = ImGui::GetMainViewport();
  const float margin = ImGui::GetFontSize() * 0.75f; // scales with the UI
  ImGui::SetNextWindowPos(
      ImVec2(vp->WorkPos.x + vp->WorkSize.x - margin, vp->WorkPos.y + vp->WorkSize.y - margin),
      ImGuiCond_Always, ImVec2(1.0f, 1.0f)); // pivot: the window's own corner
  ImGui::SetNextWindowBgAlpha(0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  const ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
      ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
      ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;
  bool clicked = false;
  if (ImGui::Begin(id, nullptr, flags)) {
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##hit", ImVec2(diameter, diameter));
    clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);
    const bool hovered = ImGui::IsItemHovered(), held = ImGui::IsItemActive();
    const ImVec2 c(p0.x + diameter * 0.5f, p0.y + diameter * 0.5f);
    const float r = diameter * 0.5f;
    ImDrawList *dl = ImGui::GetWindowDrawList();
    // GetColorU32 with an alpha multiplier respects style.Alpha; a raw IM_COL32
    // would not. Translucent so the scene still reads through the button.
    const ImU32 fill = ImGui::GetColorU32(held      ? ImGuiCol_ButtonActive
                                          : hovered ? ImGuiCol_ButtonHovered
                                                    : ImGuiCol_Button,
                                          held      ? 0.85f
                                          : hovered ? 0.70f
                                                    : 0.40f);
    dl->AddCircleFilled(c, r, fill, 0);
    dl->AddCircle(c, r, ImGui::GetColorU32(ImGuiCol_Border, 0.55f), 0,
                  diameter * 0.03f > 1.0f ? diameter * 0.03f : 1.0f);
    const ImVec2 ts = ImGui::CalcTextSize(glyph);
    dl->AddText(ImVec2(c.x - ts.x * 0.5f, c.y - ts.y * 0.5f),
                ImGui::GetColorU32(ImGuiCol_Text, 0.9f), glyph);
  }
  ImGui::End();
  ImGui::PopStyleVar();
  if (clicked && open)
    *open = !*open;
  return clicked;
}
} // namespace

struct ImGuiOverlay::Impl {
  vtkWeakPointer<vtkRenderWindow> window;
  vtkWeakPointer<vtkRenderWindowInteractor> interactor;
  vtkSmartPointer<vtkCallbackCommand> beginCmd, renderCmd, interceptCmd;
  std::function<void()> draw;
  ImGuiContext *ctx = nullptr;
  bool ready = false;   // ImGui context created
  bool glReady = false; // GL backend initialized (deferred to first render)
  bool visible = true;  // caller intent
  CameraController *cam = nullptr;
  bool hadKeyboard = false; // for the rising edge of WantCaptureKeyboard
  ImGuiStyle pristineStyle; // the unscaled style, captured once
  bool haveStyle = false;
  float appliedScale = -1.0f; // guard: applyScale is idempotent per value
  float uiScale = 1.0f;
  bool touchMode = false;
  bool panelsOpen = true; // the floating toggle drives this
  bool showToggle = true; // draw the floating show/hide button
  bool sizedOnce = false; // first frame with a real DisplaySize
  bool frameOpen = false;
  double lastTime = 0.0;

  // ---- render hooks -------------------------------------------------------
  // The GL backend can only be initialized once a context is CURRENT, which VTK
  // does not guarantee until it starts rendering — so it is done lazily on the
  // first StartEvent (in the ctor it fails with "Failed to initialize OpenGL
  // loader"), and the overlay stays inert until then.
  bool ensureBackend() {
    if (glReady)
      return true;
    if (!window)
      return false;
    window->MakeCurrent();
    if (!window->IsCurrent())
      return false;
    glReady = ImGui_ImplOpenGL3_Init();
    return glReady;
  }

  // StartEvent: begin an ImGui frame and run the user's UI code.
  void beginFrame() {
    if (!ready || !visible || !window || !ensureBackend())
      return;
    int *sz = window->GetSize();
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(static_cast<float>(sz[0]), static_cast<float>(sz[1]));
    // ImGui requires a positive frame delta (it asserts on <= 0 and produces no
    // usable frame otherwise). VTK gives us no clock, so keep our own.
    const double now = vtkTimerLog::GetUniversalTime();
    io.DeltaTime = (lastTime > 0.0) ? static_cast<float>(now - lastTime) : (1.0f / 60.0f);
    if (io.DeltaTime <= 0.0f)
      io.DeltaTime = 1.0f / 60.0f;
    lastTime = now;
    ImGui_ImplOpenGL3_NewFrame();
    ImGui::NewFrame();
    frameOpen = true;
    // On a SMALL viewport (a phone), start with the panels hidden. Finger-sized
    // widgets on a 400px-wide screen would otherwise bury the scene the moment
    // it loads — the UI has to be reachable, not permanently in the way. The
    // toggle button is always there to bring it back. Decided once, on the
    // first frame that has a real size, so a later resize does not fight the
    // user's own choice.
    if (!sizedOnce && io.DisplaySize.x > 1.0f && io.DisplaySize.y > 1.0f) {
      sizedOnce = true;
      const float shortSide =
          io.DisplaySize.x < io.DisplaySize.y ? io.DisplaySize.x : io.DisplaySize.y;
      if (shortSide < 700.0f)
        panelsOpen = false;
    }
    // The toggle is drawn FIRST so it is always reachable, even when the panels
    // it hides would have covered this corner.
    if (showToggle) {
      const float d = 34.0f * (appliedScale > 0.0f ? appliedScale : 1.0f);
      floatingCircleToggle("##cvcgl_ui_toggle", &panelsOpen, d, panelsOpen ? "X" : "=");
    }
    if (draw && panelsOpen)
      draw();
  }

  // RenderEvent: VTK has drawn the scene — composite the ImGui draw lists on
  // top, inside the same render, so offscreen captures include the UI.
  void renderFrame() {
    if (!frameOpen)
      return;
    ImGui::Render();
    // Draw into VTK's OWN render framebuffer: at RenderEvent the scene has been
    // drawn there but not yet resolved/blitted out, so compositing here lands on
    // top in the window AND in offscreen captures. Without this the draw goes to
    // whatever happens to be bound and silently disappears. On wasm the same
    // chain ends at the WebGL default framebuffer (FrameBlitMode BlitToCurrent),
    // so this path is identical in the browser.
    auto *oglWin = vtkOpenGLRenderWindow::SafeDownCast(window);
    vtkOpenGLState *ostate = oglWin ? oglWin->GetState() : nullptr;
    if (ostate)
      ostate->PushFramebufferBindings();
    if (oglWin && oglWin->GetRenderFramebuffer())
      oglWin->GetRenderFramebuffer()->Bind(GL_DRAW_FRAMEBUFFER);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    if (ostate)
      ostate->PopFramebufferBindings();
    frameOpen = false;
  }

  // ---- scaling -------------------------------------------------------------
  // Re-derive the whole style from the pristine copy at a new scale.
  // ScaleAllSizes() multiplies IN PLACE (and truncates to integers), so calling
  // it repeatedly compounds: five calls at 2x give FramePadding 128 instead of
  // 8. Always start from the unscaled style.
  void applyScale(float scale) {
    if (!haveStyle || scale == appliedScale)
      return;
    appliedScale = scale;
    ImGuiStyle &style = ImGui::GetStyle();
    style = pristineStyle;
    style.ScaleAllSizes(scale);
    // ScaleAllSizes cannot grow this one: it defaults to (0,0) and 0 * scale is
    // still 0. It is the slop around a widget's hit rect, which is exactly what
    // a fingertip needs.
    if (scale > 1.0f)
      style.TouchExtraPadding = ImVec2(4.0f * scale, 4.0f * scale);
    // Fonts are dynamic in 1.92 (re-baked at the final size), so this is both
    // crisp and safe to change at runtime. io.FontGlobalScale is OBSOLETE and
    // asserts against this field — do not use it.
    style.FontScaleMain = scale;
    // Fingers wobble; the 6px default drag threshold turns a tap into a drag.
    ImGui::GetIO().MouseDragThreshold = scale > 1.0f ? 10.0f : 6.0f;
  }

  // ---- input --------------------------------------------------------------
  // Feed ImGui the interactor's pointer state, then decide whether VTK's style
  // is allowed to see this event. Returning true ABORTS the style's handling.
  bool intercept(unsigned long eid) {
    if (!ready || !visible || !interactor)
      return false;
    // Quake pointer capture: the cursor is warped to the centre each frame, so
    // absolute positions are meaningless — hand everything to the camera and let
    // ImGui idle until capture is released.
    if (cam && cam->pointerCapture())
      return false;
    ImGuiIO &io = ImGui::GetIO();
    // Tag the source BEFORE the events it applies to: AddMouseSourceEvent is not
    // an event, it is a latch stamped onto subsequent mouse events, and it is
    // initialised once at context creation rather than per frame. Left unset,
    // every event stays tagged with whatever was latched last.
    io.AddMouseSourceEvent(touchMode ? ImGuiMouseSource_TouchScreen : ImGuiMouseSource_Mouse);
    int x = 0, y = 0;
    interactor->GetEventPosition(x, y);
    int *sz = interactor->GetSize();
    // VTK's origin is bottom-left, ImGui's is top-left.
    io.AddMousePosEvent(static_cast<float>(x), static_cast<float>(sz[1] - y));

    switch (eid) {
    case vtkCommand::LeftButtonPressEvent:
      io.AddMouseButtonEvent(0, true);
      break;
    case vtkCommand::LeftButtonReleaseEvent:
      io.AddMouseButtonEvent(0, false);
      // A finger that lifts does not hover anywhere, but a mouse does — so
      // without this the last touched widget stays hovered FOREVER, which keeps
      // io.WantCaptureMouse true, which makes intercept() abort every event from
      // here on: one tap and the camera is locked out permanently. Clearing the
      // position is what the Android backend does for the same reason.
      //
      // This relies on io.ConfigInputTrickleEventQueue (default true): with
      // trickling off, the clear collapses into the same frame as the release,
      // the release lands on a non-hovered widget, and the tap is silently lost.
      // Never turn trickling off here.
      if (touchMode)
        io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
      break;
    case vtkCommand::LeaveEvent:
      // The pointer left the window — same reasoning, for the desktop build.
      io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
      break;
    case vtkCommand::RightButtonPressEvent:
      io.AddMouseButtonEvent(1, true);
      break;
    case vtkCommand::RightButtonReleaseEvent:
      io.AddMouseButtonEvent(1, false);
      break;
    case vtkCommand::MiddleButtonPressEvent:
      io.AddMouseButtonEvent(2, true);
      break;
    case vtkCommand::MiddleButtonReleaseEvent:
      io.AddMouseButtonEvent(2, false);
      break;
    case vtkCommand::MouseWheelForwardEvent:
      io.AddMouseWheelEvent(0.0f, 1.0f);
      break;
    case vtkCommand::MouseWheelBackwardEvent:
      io.AddMouseWheelEvent(0.0f, -1.0f);
      break;
    default:
      break;
    }

    const bool keyEvent = (eid == vtkCommand::KeyPressEvent || eid == vtkCommand::KeyReleaseEvent ||
                           eid == vtkCommand::CharEvent);
    // Rising edge of "UI owns the keyboard": drop the camera's held keys so a
    // key released while the UI has focus can't leave it drifting.
    const bool wantsKeys = io.WantCaptureKeyboard;
    if (cam && wantsKeys && !hadKeyboard)
      cam->releaseHeldKeys();
    hadKeyboard = wantsKeys;
    // Swallow the event only when ImGui actually wants it — otherwise the
    // CameraController must keep receiving everything it always did.
    return keyEvent ? io.WantCaptureKeyboard : io.WantCaptureMouse;
  }

  static void onBegin(vtkObject *, unsigned long, void *cd, void *) {
    static_cast<Impl *>(cd)->beginFrame();
  }
  static void onRender(vtkObject *, unsigned long, void *cd, void *) {
    static_cast<Impl *>(cd)->renderFrame();
  }
  static void onIntercept(vtkObject *caller, unsigned long eid, void *cd, void *) {
    auto *self = static_cast<Impl *>(cd);
    (void)caller;
    // Abort => lower-priority observers (the interactor style, hence
    // CameraController) never see this event. Only when ImGui wants it.
    if (auto *cmd = self->interceptCmd.Get())
      cmd->SetAbortFlagOnExecute(self->intercept(eid) ? 1 : 0);
  }
};

ImGuiOverlay::ImGuiOverlay(SceneRenderer &viewer) : m_impl(new Impl) {
  m_impl->window = viewer.renderWindow();
  if (!m_impl->window)
    return;
  m_impl->interactor = m_impl->window->GetInteractor();

  IMGUI_CHECKVERSION();
  m_impl->ctx = ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.IniFilename = nullptr; // don't litter imgui.ini next to the binary
  ImGui::StyleColorsDark();

  // Be explicit about the font. AddFontDefault() is a HEURISTIC that picks the
  // 13px bitmap ProggyClean below an expected size of 15 and the scalable
  // ProggyForever at or above it — and it runs on the first frame, before any
  // scale we set later. Left implicit we silently get the bitmap face, which is
  // sharp at 13px and ugly at every other size. Ask for the vector face and give
  // it a logical base size; final pixels = FontSizeBase * FontScaleMain, and
  // ImGui 1.92 re-bakes glyphs at that exact size (no stretching, so scaling up
  // stays crisp).
  io.Fonts->AddFontDefaultVector();
  ImGui::GetStyle().FontSizeBase = 16.0f;

  // Remember the unscaled style: ScaleAllSizes multiplies IN PLACE and rounds,
  // so it must never be applied twice. applyUiScale() always re-derives from
  // this pristine copy rather than scaling the live style again.
  m_impl->pristineStyle = ImGui::GetStyle();
  m_impl->haveStyle = true;

  // Touch devices get a bigger UI by default — the widget sizes that suit a
  // mouse are unusable with a finger.
  m_impl->touchMode = detectTouchDevice();
  m_impl->uiScale = m_impl->touchMode ? 2.0f : 1.0f;
  m_impl->applyScale(m_impl->uiScale);

  // Copy/paste. There is no ImGui platform backend under us (VTK owns the
  // window), so nothing would supply these except on Windows, where imgui core
  // already implements the Win32 clipboard — cvc::gl::clipboard forwards to it
  // there and implements the rest (X11 selections, NSPasteboard, and in the
  // browser writeText for copy + the page's paste events for read).
  ImGuiPlatformIO &pio = ImGui::GetPlatformIO();
  pio.Platform_SetClipboardTextFn = [](ImGuiContext *, const char *text) {
    cvc::gl::clipboard::set(text ? text : "");
  };
  pio.Platform_GetClipboardTextFn = [](ImGuiContext *) -> const char * {
    // ImGui reads this immediately, so hand back storage that outlives the call
    // but is replaced on the next paste.
    static std::string buf;
    buf = cvc::gl::clipboard::get();
    return buf.c_str();
  };

  m_impl->ready = true; // GL backend is initialized lazily (see ensureBackend)

  m_impl->beginCmd = vtkSmartPointer<vtkCallbackCommand>::New();
  m_impl->beginCmd->SetCallback(&Impl::onBegin);
  m_impl->beginCmd->SetClientData(m_impl.get());
  m_impl->window->AddObserver(vtkCommand::StartEvent, m_impl->beginCmd);

  m_impl->renderCmd = vtkSmartPointer<vtkCallbackCommand>::New();
  m_impl->renderCmd->SetCallback(&Impl::onRender);
  m_impl->renderCmd->SetClientData(m_impl.get());
  // RenderEvent: fired after the renderers have drawn but BEFORE VTK copies the
  // result frame, so the scene's framebuffer is still bound and the UI composites
  // on top — natively and in offscreen captures. (EndEvent is too late: the
  // result frame has already been copied out.)
  m_impl->window->AddObserver(vtkCommand::RenderEvent, m_impl->renderCmd);

  // Observe the INTERACTOR at priority 1.0 — NEVER the interactor style.
  // vtkInteractorStyle::ProcessEvents is
  //     if (HandleObservers && HasObserver(evt)) InvokeEvent(evt) else OnMouseMove()
  // so merely REGISTERING an observer on the style diverts every event into the
  // observer branch forever: OnMouseMove/OnLeftButtonDown are never called again
  // and orbit/pan/zoom/fly die the moment an overlay is constructed. Observing
  // the interactor leaves the style untouched; the camera is suppressed only by
  // the abort flag, and only when ImGui actually wants the input.
  if (m_impl->interactor) {
    m_impl->interceptCmd = vtkSmartPointer<vtkCallbackCommand>::New();
    m_impl->interceptCmd->SetCallback(&Impl::onIntercept);
    m_impl->interceptCmd->SetClientData(m_impl.get());
    for (unsigned long e : kMouseEvents)
      m_impl->interactor->AddObserver(e, m_impl->interceptCmd, 1.0);
    for (unsigned long e : kKeyEvents)
      m_impl->interactor->AddObserver(e, m_impl->interceptCmd, 1.0);
  }
}

ImGuiOverlay::~ImGuiOverlay() {
  if (m_impl->window) {
    if (m_impl->beginCmd)
      m_impl->window->RemoveObservers(vtkCommand::StartEvent, m_impl->beginCmd);
    if (m_impl->renderCmd)
      m_impl->window->RemoveObservers(vtkCommand::RenderEvent, m_impl->renderCmd);
  }
  if (m_impl->interactor && m_impl->interceptCmd) {
    for (unsigned long e : kMouseEvents)
      m_impl->interactor->RemoveObservers(e, m_impl->interceptCmd);
    for (unsigned long e : kKeyEvents)
      m_impl->interactor->RemoveObservers(e, m_impl->interceptCmd);
  }
  if (m_impl->glReady)
    ImGui_ImplOpenGL3_Shutdown();
  if (m_impl->ctx)
    ImGui::DestroyContext(m_impl->ctx);
}

void ImGuiOverlay::setDrawCallback(std::function<void()> draw) { m_impl->draw = std::move(draw); }

void ImGuiOverlay::setUiScale(float scale) {
  if (scale < 0.25f)
    scale = 0.25f;
  if (scale > 4.0f)
    scale = 4.0f;
  m_impl->uiScale = scale;
  m_impl->applyScale(scale);
}
float ImGuiOverlay::uiScale() const { return m_impl->uiScale; }

void ImGuiOverlay::setTouchMode(bool on) {
  m_impl->touchMode = on;
  setUiScale(on ? 2.0f : 1.0f);
}
bool ImGuiOverlay::touchMode() const { return m_impl->touchMode; }

void ImGuiOverlay::setToggleButtonEnabled(bool on) { m_impl->showToggle = on; }
bool ImGuiOverlay::toggleButtonEnabled() const { return m_impl->showToggle; }
void ImGuiOverlay::setPanelsOpen(bool on) { m_impl->panelsOpen = on; }
bool ImGuiOverlay::panelsOpen() const { return m_impl->panelsOpen; }
void ImGuiOverlay::attachCamera(CameraController &cam) { m_impl->cam = &cam; }
void ImGuiOverlay::setVisible(bool on) { m_impl->visible = on; }
bool ImGuiOverlay::visible() const { return m_impl->visible; }
bool ImGuiOverlay::enabled() const { return m_impl->ready; }
bool ImGuiOverlay::wantsMouse() const {
  return m_impl->ready && m_impl->visible && ImGui::GetIO().WantCaptureMouse;
}
bool ImGuiOverlay::wantsKeyboard() const {
  return m_impl->ready && m_impl->visible && ImGui::GetIO().WantCaptureKeyboard;
}

} // namespace gl
} // namespace cvc

#else // !CVC_ENABLE_IMGUI — inert stubs so consumers need no #ifdef

namespace cvc {
namespace gl {

struct ImGuiOverlay::Impl {};

ImGuiOverlay::ImGuiOverlay(SceneRenderer &) : m_impl(new Impl) {}
ImGuiOverlay::~ImGuiOverlay() = default;
void ImGuiOverlay::setDrawCallback(std::function<void()>) {}
void ImGuiOverlay::setVisible(bool) {}
bool ImGuiOverlay::visible() const { return false; }
bool ImGuiOverlay::enabled() const { return false; }
void ImGuiOverlay::setUiScale(float) {}
float ImGuiOverlay::uiScale() const { return 1.0f; }
void ImGuiOverlay::setTouchMode(bool) {}
bool ImGuiOverlay::touchMode() const { return false; }
void ImGuiOverlay::setToggleButtonEnabled(bool) {}
bool ImGuiOverlay::toggleButtonEnabled() const { return false; }
void ImGuiOverlay::setPanelsOpen(bool) {}
bool ImGuiOverlay::panelsOpen() const { return true; }
bool ImGuiOverlay::wantsMouse() const { return false; }
bool ImGuiOverlay::wantsKeyboard() const { return false; }

} // namespace gl
} // namespace cvc

#endif // CVC_ENABLE_IMGUI
