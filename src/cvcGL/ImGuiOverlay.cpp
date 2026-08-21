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
#include <cvc/gl/SceneRenderer.h>
#include <imgui.h>
// The GL loader must match VTK's context: WebGL2/GLES3 in the browser build.
#if defined(__EMSCRIPTEN__) && !defined(IMGUI_IMPL_OPENGL_ES3)
#define IMGUI_IMPL_OPENGL_ES3
#endif
#include <backends/imgui_impl_opengl3.h>
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
    if (draw)
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
bool ImGuiOverlay::wantsMouse() const { return false; }
bool ImGuiOverlay::wantsKeyboard() const { return false; }

} // namespace gl
} // namespace cvc

#endif // CVC_ENABLE_IMGUI
