// SdlWindow -- cvcGL's onscreen window on SDL3 (internal to cvcGL).
//
// SDL3 creates the window and its OpenGL context, VTK renders into that context, and SDL's events
// drive VTK's interactor, so every host sees one input path on every platform: the camera,
// ImGuiOverlay and anything else observing the interactor get VTK events as before, and SdlInput
// gets the same events in SDL's own names. ViewportManager uses these two classes for an onscreen
// window when cvcGL is built with SDL3 (CVC_ENABLE_SDL); offscreen windows stay VTK's.
//
// VTK's own SDL2 pair (vtkSDL2OpenGLRenderWindow / vtkSDL2RenderWindowInteractor) is the model;
// VTK has no SDL3 classes.
#ifndef CVC_GL_SDL_WINDOW_H
#define CVC_GL_SDL_WINDOW_H

#ifdef CVC_ENABLE_SDL

#include <SDL3/SDL.h>
#include <functional>
#include <map>
#include <stack>
#include <string>
#include <utility>
#include <vector>
#include <vtkOpenGLRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

namespace cvc {
namespace gl {

// $CVCGL_SDL_WINDOW=0: VTK's own onscreen window even where SDL3 is built in (a fallback while
// this path is new). Always false in the browser, which keeps VTK's canvas window for now.
bool sdl_window_enabled();

class SdlGLRenderWindow : public vtkOpenGLRenderWindow {
public:
  static SdlGLRenderWindow *New();
  vtkTypeMacro(SdlGLRenderWindow, vtkOpenGLRenderWindow);

  // Makes the SDL window and its GL context (at the first render, or here), then VTK's GL state.
  // If SDL cannot make them (no display, no GL), the window stays empty: sdlWindow() is null,
  // nothing renders, and lastError() says why (ViewportManager then uses VTK's window instead).
  void Initialize() override;
  void Finalize() override;
  void Start() override;
  void End() override;
  void Render() override;
  void Frame() override; // VTK's blit, then SDL_GL_SwapWindow

  void MakeCurrent() override;
  void ReleaseCurrent() override;
  bool IsCurrent() override;
  void PushContext() override;
  void PopContext() override;
  bool SetSwapControl(int interval) override;
  int SupportsOpenGL() override { return 1; }
  vtkTypeBool IsDirect() override { return 1; }

  // Size is the drawable's, in pixels, as VTK's own windows have it: where a window point is
  // several pixels (macOS, Wayland), the window is that many times smaller in points.
  void SetSize(int width, int height) override;
  void SetSize(int a[2]) override { this->SetSize(a[0], a[1]); }
  void SetPosition(int x, int y) override;
  void SetPosition(int a[2]) override { this->SetPosition(a[0], a[1]); }
  int *GetPosition() VTK_SIZEHINT(2) override;
  int *GetScreenSize() VTK_SIZEHINT(2) override;
  void SetWindowName(const char *name) override;
  void SetFullScreen(vtkTypeBool on) override;
  void HideCursor() override;
  void ShowCursor() override;
  void SetCursorPosition(int x, int y) override; // VTK's coordinates: pixels, y up
  void SetCurrentCursor(int shape) override;
  void SetShowWindow(bool show) override;

  // The SDL_Window (no native handle: hosts that wrapped one now ask SDL).
  void *GetGenericWindowId() override { return window_; }
  void *GetGenericDisplayId() override { return nullptr; }
  void *GetGenericParentId() override { return nullptr; }
  void *GetGenericContext() override { return context_; }
  void *GetGenericDrawable() override { return window_; }
  void SetWindowId(void *) override {}
  void SetDisplayId(void *) override {}
  void SetParentId(void *) override {}
  void SetNextWindowId(void *) override {}
  void SetWindowInfo(const char *) override {}
  void SetParentInfo(const char *) override {}
  void SetNextWindowInfo(const char *) override {}
  void WindowRemap() override;
  vtkTypeBool GetEventPending() override { return 0; }

  SDL_Window *sdlWindow() const { return window_; }
  // Why the window could not be made (SDL's error), or empty.
  const std::string &lastError() const { return error_; }
  // The window's size changed (its interactor saw it): VTK's size follows the drawable's.
  void syncSizeFromWindow();
  // Window point -> VTK pixel (x right, y down; the interactor flips y).
  void toPixels(float x, float y, int &px, int &py) const;
  // How many of these windows are open in the process: while any is, its interactor owns SDL's
  // event queue, and SdlInput never drains it itself.
  static int openWindows();

protected:
  SdlGLRenderWindow();
  ~SdlGLRenderWindow() override;
  void CreateAWindow() override;
  void DestroyWindow() override;

private:
  SdlGLRenderWindow(const SdlGLRenderWindow &) = delete;
  void operator=(const SdlGLRenderWindow &) = delete;

  SDL_Window *window_ = nullptr;
  SDL_GLContext context_ = nullptr;
  bool video_ = false;
  bool resizing_ = false; // SetSize from the window's own resize: no SDL_SetWindowSize back
  bool started_ = false;  // Start() bound VTK's framebuffer, so End() pops it
  std::string error_;
  std::map<int, SDL_Cursor *> cursors_;
  std::stack<std::pair<SDL_Window *, SDL_GLContext>> contexts_; // PushContext's
  int screen_[2] = {0, 0};
};

class SdlRenderWindowInteractor : public vtkRenderWindowInteractor {
public:
  static SdlRenderWindowInteractor *New();
  vtkTypeMacro(SdlRenderWindowInteractor, vtkRenderWindowInteractor);

  void Initialize() override;
  // Drains SDL's queue into VTK events (and to the sinks below); never blocks. SDL has one queue
  // per process: an event of another cvcGL window is kept for that window's interactor, and
  // delivered when it next processes events.
  void ProcessEvents() override;
  void TerminateApp() override { this->Done = true; }

  // Every SDL event of this window, before it becomes VTK's: SdlInput subscribes here, so it reads
  // the window's events without draining SDL's queue itself. Returns an id for removeSink().
  int addSink(std::function<void(const SDL_Event &)> sink);
  void removeSink(int id);

protected:
  SdlRenderWindowInteractor();
  ~SdlRenderWindowInteractor() override;
  void StartEventLoop() override;
  int InternalCreateTimer(int timerId, int timerType, unsigned long duration) override;
  int InternalDestroyTimer(int platformTimerId) override;

private:
  SdlRenderWindowInteractor(const SdlRenderWindowInteractor &) = delete;
  void operator=(const SdlRenderWindowInteractor &) = delete;

  struct Kept { // an event another interactor took off the queue for this one
    SDL_Event event;
    std::string text; // a text event's text, which SDL frees at its next poll
  };

  void claimWindow();                // routes this window's events here
  void deliver(const SDL_Event &e);  // to this interactor, or kept for the window's own
  void keep(const SDL_Event &e);     // ... kept
  void dispatch(const SDL_Event &e); // to the sinks, then as VTK events
  SdlGLRenderWindow *sdlWindow() const;

  std::vector<std::pair<int, std::function<void(const SDL_Event &)>>> sinks_;
  int nextSink_ = 1;
  std::vector<Kept> kept_;
  float wheelY_ = 0.0f, wheelX_ = 0.0f; // fractional wheel steps (trackpads) carried over
  std::map<int, SDL_TimerID> timers_;   // VTK platform timer id -> SDL timer
};

} // namespace gl
} // namespace cvc

#endif // CVC_ENABLE_SDL
#endif // CVC_GL_SDL_WINDOW_H
