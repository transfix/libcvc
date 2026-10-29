// SdlWindow -- see SdlWindow.h.
#include "SdlWindow.h"

#ifdef CVC_ENABLE_SDL

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <set>
#include <string>
#include <vtkCommand.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>

namespace cvc {
namespace gl {

bool sdl_window_enabled() {
#ifdef __EMSCRIPTEN__
  return false; // the browser keeps VTK's canvas window for now
#else
  const char *v = std::getenv("CVCGL_SDL_WINDOW");
  return !(v && std::string(v) == "0");
#endif
}

namespace {

int g_openWindows = 0;

// A hint the app has not set, set to what VTK's own windows do.
void hint_default(const char *name, const char *value) {
  if (!SDL_GetHint(name))
    SDL_SetHint(name, value);
}

} // namespace

// ── the window ──────────────────────────────────────────────────────────────────────────────────

vtkStandardNewMacro(SdlGLRenderWindow);

int SdlGLRenderWindow::openWindows() { return g_openWindows; }

SdlGLRenderWindow::SdlGLRenderWindow() {
  this->SetWindowName("cvcGL");
  this->SetStencilCapable(1);
  this->Position[0] = this->Position[1] = -1; // SDL places it
}

SdlGLRenderWindow::~SdlGLRenderWindow() {
  this->Finalize();
  vtkRenderer *ren = nullptr;
  vtkCollectionSimpleIterator rit;
  this->Renderers->InitTraversal(rit);
  while ((ren = this->Renderers->GetNextRenderer(rit)))
    ren->SetRenderWindow(nullptr);
}

void SdlGLRenderWindow::CreateAWindow() {
  if (window_)
    return;
  error_.clear();
  if (!video_) {
    // SDL's video is this window's, and it leaves the process as VTK's windows do unless the app
    // said otherwise: the screen still sleeps, Ctrl+C still ends the process, X11's compositor
    // stays on, a click that focuses the window also acts, and on Windows the process keeps the
    // DPI awareness it has. (SDL 3 still reads SDL_WINDOWS_DPI_AWARENESS, though SDL_hints.h no
    // longer names it; with none of its values, "permonitorv2" and so on, it leaves it alone.)
    hint_default(SDL_HINT_VIDEO_ALLOW_SCREENSAVER, "1");
    hint_default(SDL_HINT_NO_SIGNAL_HANDLERS, "1");
    hint_default(SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR, "0");
    hint_default(SDL_HINT_MOUSE_FOCUS_CLICKTHROUGH, "1");
    hint_default("SDL_WINDOWS_DPI_AWARENESS", "");
    if (!SDL_InitSubSystem(SDL_INIT_VIDEO)) {
      error_ = std::string("SDL_InitSubSystem(VIDEO): ") + SDL_GetError();
      return;
    }
    video_ = true;
  }
  // VTK's OpenGL2 backend: a 3.2 core context (ES 3.0 in the browser).
  SDL_GL_ResetAttributes();
#ifdef __EMSCRIPTEN__
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#else
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
#ifdef __APPLE__
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
#endif
#endif
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, this->DoubleBuffer ? 1 : 0);
  SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, this->StencilCapable ? 8 : 0);

  // Made hidden, sized, then shown: no flash at the wrong size on a high-density display.
  const int w = this->Size[0] > 0 ? this->Size[0] : 300;
  const int h = this->Size[1] > 0 ? this->Size[1] : 300;
  const SDL_WindowFlags flags =
      SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY | SDL_WINDOW_HIDDEN;
  window_ = SDL_CreateWindow(this->WindowName ? this->WindowName : "cvcGL", w, h, flags);
  if (!window_) {
    error_ = std::string("SDL_CreateWindow: ") + SDL_GetError();
    return;
  }
  context_ = SDL_GL_CreateContext(window_);
  if (!context_) {
    error_ = std::string("SDL_GL_CreateContext: ") + SDL_GetError();
    SDL_DestroyWindow(window_);
    window_ = nullptr;
    return;
  }
  ++g_openWindows;
  SDL_GL_MakeCurrent(window_, context_);
  this->SetOpenGLSymbolLoader(
      [](void *, const char *name) -> VTKOpenGLAPIProc {
        return reinterpret_cast<VTKOpenGLAPIProc>(SDL_GL_GetProcAddress(name));
      },
      nullptr);
  // VTK's size is in pixels; SDL's window size is in points, several pixels each on macOS and
  // Wayland's scaled outputs.
  const float d = SDL_GetWindowPixelDensity(window_);
  if (d > 1.0f)
    SDL_SetWindowSize(window_, std::max(1, static_cast<int>(std::lround(w / d))),
                      std::max(1, static_cast<int>(std::lround(h / d))));
  if (this->Position[0] >= 0 && this->Position[1] >= 0)
    SDL_SetWindowPosition(window_, this->Position[0], this->Position[1]);
  if (this->FullScreen)
    SDL_SetWindowFullscreen(window_, true);
  if (this->ShowWindow)
    SDL_ShowWindow(window_);
  SDL_StartTextInput(window_); // typed characters, for ImGui's text fields (CharEvent)
  this->Mapped = 1;
  this->syncSizeFromWindow();
}

void SdlGLRenderWindow::DestroyWindow() {
  for (auto &kv : cursors_)
    SDL_DestroyCursor(kv.second);
  cursors_.clear();
  if (context_) {
    SDL_GL_DestroyContext(context_);
    context_ = nullptr;
  }
  if (window_) {
    SDL_StopTextInput(window_);
    SDL_DestroyWindow(window_);
    window_ = nullptr;
    --g_openWindows;
  }
  this->Mapped = 0;
  if (video_) {
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
    video_ = false;
  }
}

void SdlGLRenderWindow::Initialize() {
  if (!window_)
    this->CreateAWindow();
  if (window_ && !this->Initialized) {
    this->MakeCurrent();
    this->OpenGLInit(); // loads GL through SDL_GL_GetProcAddress, then VTK's GL state
    if (!this->Initialized && error_.empty())
      error_ = "VTK found no OpenGL 3.2 in SDL's context";
  }
}

void SdlGLRenderWindow::Finalize() {
  if (window_ && context_ && this->OwnContext && this->Initialized) {
    this->MakeCurrent();
    this->ReleaseGraphicsResources(this);
  }
  this->DestroyWindow(); // and SDL's video, which a window that failed to open still holds
}

// Without a window and its GL (SDL could not make them) VTK must not touch GL at all: Start, End,
// Render and Frame do nothing.
void SdlGLRenderWindow::Start() {
  started_ = false;
  this->Initialize();
  if (!window_ || !this->Initialized)
    return;
  this->Superclass::Start();
  started_ = true;
}

void SdlGLRenderWindow::End() {
  if (!started_)
    return;
  started_ = false;
  this->Superclass::End();
}

void SdlGLRenderWindow::Render() {
  this->Initialize();
  if (!window_ || !this->Initialized)
    return;
  this->Superclass::Render();
}

void SdlGLRenderWindow::Frame() {
  if (!window_ || !this->Initialized)
    return;
  this->MakeCurrent();
  this->Superclass::Frame();
  if (!this->AbortRender && this->DoubleBuffer && this->SwapBuffers)
    SDL_GL_SwapWindow(window_);
}

void SdlGLRenderWindow::MakeCurrent() {
  if (window_ && context_)
    SDL_GL_MakeCurrent(window_, context_);
}

void SdlGLRenderWindow::ReleaseCurrent() {
  if (window_ && context_)
    SDL_GL_MakeCurrent(window_, nullptr);
}

bool SdlGLRenderWindow::IsCurrent() { return context_ && SDL_GL_GetCurrentContext() == context_; }

void SdlGLRenderWindow::PushContext() {
  SDL_GLContext current = SDL_GL_GetCurrentContext();
  contexts_.emplace(SDL_GL_GetCurrentWindow(), current);
  if (current != context_)
    this->MakeCurrent();
}

void SdlGLRenderWindow::PopContext() {
  if (contexts_.empty())
    return;
  const auto prior = contexts_.top();
  contexts_.pop();
  if (prior.second != SDL_GL_GetCurrentContext())
    SDL_GL_MakeCurrent(prior.first, prior.second);
}

bool SdlGLRenderWindow::SetSwapControl(int interval) {
  return context_ && SDL_GL_SetSwapInterval(interval);
}

void SdlGLRenderWindow::SetSize(int width, int height) {
  if (this->Size[0] == width && this->Size[1] == height)
    return;
  this->Superclass::SetSize(width, height);
  if (this->Interactor)
    this->Interactor->UpdateSize(width, height); // its events flip y by the new height
  if (window_ && !resizing_) {
    const float d = SDL_GetWindowPixelDensity(window_);
    const float k = d > 0.0f ? d : 1.0f;
    SDL_SetWindowSize(window_, std::max(1, static_cast<int>(std::lround(width / k))),
                      std::max(1, static_cast<int>(std::lround(height / k))));
  }
}

void SdlGLRenderWindow::syncSizeFromWindow() {
  if (!window_)
    return;
  int w = 0, h = 0;
  SDL_GetWindowSizeInPixels(window_, &w, &h);
  if (w < 1 || h < 1)
    return;
  resizing_ = true;
  this->SetSize(w, h);
  resizing_ = false;
}

void SdlGLRenderWindow::toPixels(float x, float y, int &px, int &py) const {
  int lw = 0, lh = 0, pw = 0, ph = 0;
  if (window_) {
    SDL_GetWindowSize(window_, &lw, &lh);
    SDL_GetWindowSizeInPixels(window_, &pw, &ph);
  }
  const double sx = lw > 0 ? double(pw) / lw : 1.0, sy = lh > 0 ? double(ph) / lh : 1.0;
  px = static_cast<int>(std::floor(x * sx));
  py = static_cast<int>(std::floor(y * sy));
}

void SdlGLRenderWindow::SetPosition(int x, int y) {
  if (this->Position[0] == x && this->Position[1] == y)
    return;
  this->Superclass::SetPosition(x, y);
  if (window_)
    SDL_SetWindowPosition(window_, x, y);
}

int *SdlGLRenderWindow::GetPosition() {
  if (window_)
    SDL_GetWindowPosition(window_, &this->Position[0], &this->Position[1]);
  return this->Position;
}

int *SdlGLRenderWindow::GetScreenSize() {
  const SDL_DisplayID display =
      window_ ? SDL_GetDisplayForWindow(window_) : SDL_GetPrimaryDisplay();
  if (const SDL_DisplayMode *mode = display ? SDL_GetCurrentDisplayMode(display) : nullptr) {
    const float d = mode->pixel_density > 0.0f ? mode->pixel_density : 1.0f; // in pixels
    screen_[0] = static_cast<int>(std::lround(mode->w * d));
    screen_[1] = static_cast<int>(std::lround(mode->h * d));
  }
  return screen_;
}

void SdlGLRenderWindow::SetWindowName(const char *name) {
  this->Superclass::SetWindowName(name);
  if (window_ && name)
    SDL_SetWindowTitle(window_, name);
}

void SdlGLRenderWindow::SetFullScreen(vtkTypeBool on) {
  if (this->FullScreen == on)
    return;
  this->FullScreen = on;
  if (window_)
    SDL_SetWindowFullscreen(window_, on != 0); // the desktop's mode, borderless
  this->Modified();
}

void SdlGLRenderWindow::HideCursor() { SDL_HideCursor(); }
void SdlGLRenderWindow::ShowCursor() { SDL_ShowCursor(); }

void SdlGLRenderWindow::SetCursorPosition(int x, int y) {
  if (!window_)
    return;
  int lw = 0, lh = 0, pw = 0, ph = 0;
  SDL_GetWindowSize(window_, &lw, &lh);
  SDL_GetWindowSizeInPixels(window_, &pw, &ph);
  const float sx = pw > 0 ? float(lw) / pw : 1.0f, sy = ph > 0 ? float(lh) / ph : 1.0f;
  // The motion SDL has already queued is from where the pointer was: drop it, so the next move is
  // measured from here (the camera's mouse-look recenters every frame).
  SDL_FlushEvent(SDL_EVENT_MOUSE_MOTION);
  SDL_WarpMouseInWindow(window_, x * sx, (ph - 1 - y) * sy);
}

void SdlGLRenderWindow::SetCurrentCursor(int shape) {
  this->Superclass::SetCurrentCursor(shape);
  static const std::map<int, SDL_SystemCursor> kCursor = {
      {VTK_CURSOR_DEFAULT, SDL_SYSTEM_CURSOR_DEFAULT},
      {VTK_CURSOR_ARROW, SDL_SYSTEM_CURSOR_DEFAULT},
      {VTK_CURSOR_SIZENE, SDL_SYSTEM_CURSOR_NE_RESIZE},
      {VTK_CURSOR_SIZENW, SDL_SYSTEM_CURSOR_NW_RESIZE},
      {VTK_CURSOR_SIZESW, SDL_SYSTEM_CURSOR_SW_RESIZE},
      {VTK_CURSOR_SIZESE, SDL_SYSTEM_CURSOR_SE_RESIZE},
      {VTK_CURSOR_SIZENS, SDL_SYSTEM_CURSOR_NS_RESIZE},
      {VTK_CURSOR_SIZEWE, SDL_SYSTEM_CURSOR_EW_RESIZE},
      {VTK_CURSOR_SIZEALL, SDL_SYSTEM_CURSOR_MOVE},
      {VTK_CURSOR_HAND, SDL_SYSTEM_CURSOR_POINTER},
      {VTK_CURSOR_CROSSHAIR, SDL_SYSTEM_CURSOR_CROSSHAIR}};
  if (!window_)
    return;
  const auto it = kCursor.find(shape);
  const SDL_SystemCursor sys = it == kCursor.end() ? SDL_SYSTEM_CURSOR_DEFAULT : it->second;
  SDL_Cursor *&c = cursors_[sys];
  if (!c)
    c = SDL_CreateSystemCursor(sys);
  if (c)
    SDL_SetCursor(c);
}

void SdlGLRenderWindow::SetShowWindow(bool show) {
  if (show == this->ShowWindow)
    return;
  if (window_) {
    if (show)
      SDL_ShowWindow(window_);
    else
      SDL_HideWindow(window_);
  }
  this->Superclass::SetShowWindow(show);
}

void SdlGLRenderWindow::WindowRemap() {
  this->Finalize();
  this->Initialize();
}

// ── the interactor ──────────────────────────────────────────────────────────────────────────────

vtkStandardNewMacro(SdlRenderWindowInteractor);

namespace {

// SDL has one event queue per process, so whichever interactor drains it hands each event to the
// interactor of the window it names, and a timer's tick to the interactor that made the timer.
// VTK's timer ids are unique in the process.
std::map<SDL_WindowID, SdlRenderWindowInteractor *> &window_routes() {
  static std::map<SDL_WindowID, SdlRenderWindowInteractor *> routes;
  return routes;
}
std::map<int, SdlRenderWindowInteractor *> &timer_routes() {
  static std::map<int, SdlRenderWindowInteractor *> routes;
  return routes;
}

// One registered event type carries every interactor's timer ticks.
Uint32 timer_event() {
  static const Uint32 type = SDL_RegisterEvents(1);
  return type;
}

// Runs on SDL's timer thread: posts the tick to the queue. The timer's id and kind ride in the
// callback's pointer, so nothing is shared with the interactor.
Uint32 SDLCALL on_timer(void *tick, SDL_TimerID, Uint32 interval) {
  const std::intptr_t t = reinterpret_cast<std::intptr_t>(tick);
  SDL_Event e{};
  e.type = timer_event();
  e.user.code = static_cast<Sint32>(t >> 1);
  SDL_PushEvent(&e);
  return (t & 1) ? interval : 0;
}

// A printable key's X11 keysym: VTK's windows name keys as X11 does (vtkWin32RenderWindowInteractor
// and vtkCocoaGLView share its table), and ImGuiOverlay, the camera and SdlInput read them.
std::string x11_printable(unsigned c) {
  if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
    return std::string(1, static_cast<char>(c));
  static const std::map<unsigned, const char *> kPunctuation = {
      {' ', "space"},      {'!', "exclam"},       {'"', "quotedbl"},    {'#', "numbersign"},
      {'$', "dollar"},     {'%', "percent"},      {'&', "ampersand"},   {'\'', "apostrophe"},
      {'(', "parenleft"},  {')', "parenright"},   {'*', "asterisk"},    {'+', "plus"},
      {',', "comma"},      {'-', "minus"},        {'.', "period"},      {'/', "slash"},
      {':', "colon"},      {';', "semicolon"},    {'<', "less"},        {'=', "equal"},
      {'>', "greater"},    {'?', "question"},     {'@', "at"},          {'[', "bracketleft"},
      {'\\', "backslash"}, {']', "bracketright"}, {'^', "asciicircum"}, {'_', "underscore"},
      {'`', "grave"},      {'{', "braceleft"},    {'|', "bar"},         {'}', "braceright"},
      {'~', "asciitilde"}};
  const auto it = kPunctuation.find(c);
  return it == kPunctuation.end() ? std::string() : it->second;
}

// SDL's key as VTK names it. The keysym is the key's own, without Shift (SDL's key events carry
// the unshifted key), so a key goes up under the name it went down with: "a", "1", "comma".
std::string vtk_keysym(SDL_Keycode k) {
  switch (k) {
  case SDLK_RETURN:
    return "Return";
  case SDLK_ESCAPE:
    return "Escape";
  case SDLK_BACKSPACE:
    return "BackSpace";
  case SDLK_TAB:
    return "Tab";
  case SDLK_DELETE:
    return "Delete";
  case SDLK_INSERT:
    return "Insert";
  case SDLK_LEFT:
    return "Left";
  case SDLK_RIGHT:
    return "Right";
  case SDLK_UP:
    return "Up";
  case SDLK_DOWN:
    return "Down";
  case SDLK_HOME:
    return "Home";
  case SDLK_END:
    return "End";
  case SDLK_PAGEUP:
    return "Prior";
  case SDLK_PAGEDOWN:
    return "Next";
  case SDLK_LSHIFT:
    return "Shift_L";
  case SDLK_RSHIFT:
    return "Shift_R";
  case SDLK_LCTRL:
    return "Control_L";
  case SDLK_RCTRL:
    return "Control_R";
  case SDLK_LALT:
    return "Alt_L";
  case SDLK_RALT:
    return "Alt_R";
  case SDLK_LGUI:
    return "Super_L";
  case SDLK_RGUI:
    return "Super_R";
  case SDLK_CAPSLOCK:
    return "Caps_Lock";
  case SDLK_NUMLOCKCLEAR:
    return "Num_Lock";
  case SDLK_SCROLLLOCK:
    return "Scroll_Lock";
  case SDLK_PRINTSCREEN:
    return "Print";
  case SDLK_PAUSE:
    return "Pause";
  case SDLK_APPLICATION:
  case SDLK_MENU:
    return "Menu";
  case SDLK_KP_ENTER:
    return "KP_Enter";
  case SDLK_KP_PLUS:
    return "KP_Add";
  case SDLK_KP_MINUS:
    return "KP_Subtract";
  case SDLK_KP_MULTIPLY:
    return "KP_Multiply";
  case SDLK_KP_DIVIDE:
    return "KP_Divide";
  case SDLK_KP_PERIOD:
    return "KP_Decimal";
  case SDLK_KP_EQUALS:
    return "KP_Equal";
  case SDLK_KP_0:
    return "KP_0";
  default:
    break;
  }
  if (k >= SDLK_KP_1 && k <= SDLK_KP_9)
    return "KP_" + std::to_string(1 + (k - SDLK_KP_1));
  if (k >= SDLK_F1 && k <= SDLK_F12)
    return "F" + std::to_string(1 + (k - SDLK_F1));
  if (k >= SDLK_F13 && k <= SDLK_F24)
    return "F" + std::to_string(13 + (k - SDLK_F13));
  const std::string printable = x11_printable(k);
  if (!printable.empty())
    return printable;
  const char *name = SDL_GetKeyName(k);
  return name && *name ? name : "None";
}

// The character a key press types, as VTK's KeyCode has it: with Shift, Caps Lock and AltGr ('A',
// '!'), the control character of Return, Escape, Tab, BackSpace and Delete, else 0.
char vtk_keycode(const SDL_KeyboardEvent &k) {
  switch (k.key) {
  case SDLK_RETURN:
  case SDLK_KP_ENTER:
    return '\r';
  case SDLK_ESCAPE:
    return 27;
  case SDLK_TAB:
    return '\t';
  case SDLK_BACKSPACE:
    return 8;
  case SDLK_DELETE:
    return 127;
  default:
    break;
  }
  SDL_Keycode c = k.key;
  if (k.scancode != SDL_SCANCODE_UNKNOWN) {
    const SDL_Keymod typing =
        static_cast<SDL_Keymod>(k.mod & (SDL_KMOD_SHIFT | SDL_KMOD_CAPS | SDL_KMOD_MODE));
    const SDL_Keycode typed = SDL_GetKeyFromScancode(k.scancode, typing, false);
    if (typed != SDLK_UNKNOWN)
      c = typed;
  } else if ((k.mod & SDL_KMOD_SHIFT) && c >= 'a' && c <= 'z') { // a pushed event: no scancode
    c = c - 'a' + 'A';
  }
  return c >= 32 && c < 256 && c != 127 ? static_cast<char>(c) : 0;
}

// The next code point of UTF-8 text (0 at its end); an invalid byte is skipped.
std::uint32_t next_codepoint(const char *&p) {
  while (*p) {
    const unsigned char b = static_cast<unsigned char>(*p++);
    if (b < 0x80)
      return b;
    int more = b >= 0xF0 ? 3 : b >= 0xE0 ? 2 : b >= 0xC0 ? 1 : -1;
    if (more < 0)
      continue; // a stray continuation byte
    std::uint32_t cp = b & (0x3F >> more);
    for (; more > 0 && (static_cast<unsigned char>(*p) & 0xC0) == 0x80; --more)
      cp = (cp << 6) | (static_cast<unsigned char>(*p++) & 0x3F);
    if (more == 0)
      return cp;
  }
  return 0;
}

} // namespace

SdlRenderWindowInteractor::SdlRenderWindowInteractor() = default;

SdlRenderWindowInteractor::~SdlRenderWindowInteractor() {
  while (!timers_.empty())
    this->InternalDestroyTimer(timers_.begin()->first);
  auto &routes = window_routes();
  for (auto it = routes.begin(); it != routes.end();)
    it = it->second == this ? routes.erase(it) : std::next(it);
}

SdlGLRenderWindow *SdlRenderWindowInteractor::sdlWindow() const {
  return SdlGLRenderWindow::SafeDownCast(this->RenderWindow);
}

void SdlRenderWindowInteractor::Initialize() {
  if (!this->RenderWindow) {
    vtkErrorMacro("SdlRenderWindowInteractor: no render window");
    return;
  }
  if (this->Initialized)
    return;
  this->Initialized = 1;
  this->RenderWindow->Start(); // makes the SDL window and its GL context
  this->RenderWindow->End();
  const int *size = this->RenderWindow->GetSize();
  this->Size[0] = this->EventSize[0] = size[0];
  this->Size[1] = this->EventSize[1] = size[1];
  this->claimWindow();
  this->Enable();
}

void SdlRenderWindowInteractor::claimWindow() {
  const SdlGLRenderWindow *win = sdlWindow();
  if (win && win->sdlWindow())
    window_routes()[SDL_GetWindowID(win->sdlWindow())] = this;
}

int SdlRenderWindowInteractor::addSink(std::function<void(const SDL_Event &)> sink) {
  sinks_.emplace_back(nextSink_, std::move(sink));
  return nextSink_++;
}

void SdlRenderWindowInteractor::removeSink(int id) {
  for (auto it = sinks_.begin(); it != sinks_.end(); ++it)
    if (it->first == id) {
      sinks_.erase(it);
      return;
    }
}

void SdlRenderWindowInteractor::ProcessEvents() {
  this->claimWindow(); // a remapped window has a new id
  if (!kept_.empty()) {
    std::vector<Kept> kept;
    kept.swap(kept_);
    for (Kept &k : kept) {
      if (k.event.type == SDL_EVENT_TEXT_INPUT)
        k.event.text.text = k.text.c_str();
      dispatch(k.event);
    }
  }
  SDL_Event e;
  while (SDL_PollEvent(&e))
    deliver(e);
}

void SdlRenderWindowInteractor::StartEventLoop() {
  while (!this->Done) {
    this->ProcessEvents();
    SDL_Event e;
    if (!this->Done && SDL_WaitEventTimeout(&e, 10))
      deliver(e);
  }
}

void SdlRenderWindowInteractor::keep(const SDL_Event &e) {
  Kept k{e, std::string()};
  if (e.type == SDL_EVENT_TEXT_INPUT) {
    k.text = e.text.text ? e.text.text : "";
    k.event.text.text = nullptr;
  }
  kept_.push_back(std::move(k));
}

void SdlRenderWindowInteractor::deliver(const SDL_Event &e) {
  SdlRenderWindowInteractor *to = this;
  if (e.type == timer_event()) {
    const auto it = timer_routes().find(e.user.code);
    if (it == timer_routes().end())
      return; // its timer is gone
    to = it->second;
  } else if (e.type == SDL_EVENT_QUIT) { // every window's
    std::set<SdlRenderWindowInteractor *> others;
    for (const auto &kv : window_routes())
      if (kv.second != this)
        others.insert(kv.second);
    for (SdlRenderWindowInteractor *other : others)
      other->keep(e);
  } else if (SDL_Window *w = SDL_GetWindowFromEvent(&e)) {
    const auto it = window_routes().find(SDL_GetWindowID(w));
    if (it != window_routes().end())
      to = it->second;
  }
  if (to == this)
    dispatch(e);
  else
    to->keep(e);
}

void SdlRenderWindowInteractor::dispatch(const SDL_Event &e) {
  for (const auto &kv : sinks_)
    kv.second(e);
  if (!this->Enabled)
    return;
  SdlGLRenderWindow *win = sdlWindow();
  const SDL_Keymod mods =
      (e.type == SDL_EVENT_KEY_DOWN || e.type == SDL_EVENT_KEY_UP) ? e.key.mod : SDL_GetModState();
  const int ctrl = (mods & SDL_KMOD_CTRL) ? 1 : 0, shift = (mods & SDL_KMOD_SHIFT) ? 1 : 0;
  this->SetAltKey((mods & SDL_KMOD_ALT) ? 1 : 0);
  const auto pointer = [&](float x, float y, int repeat = 0) {
    int px = 0, py = 0;
    if (win)
      win->toPixels(x, y, px, py);
    this->SetEventInformationFlipY(px, py, ctrl, shift, 0, repeat);
  };

  switch (e.type) {
  case SDL_EVENT_QUIT:
  case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
    this->ExitCallback(); // TerminateApp() (Done) unless the host observes ExitEvent
    break;
  case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
    if (win) {
      win->syncSizeFromWindow();
      const int *size = win->GetSize();
      this->UpdateSize(size[0], size[1]);
      this->SetEventSize(size[0], size[1]);
      this->InvokeEvent(vtkCommand::ConfigureEvent, nullptr);
    }
    break;
  case SDL_EVENT_WINDOW_EXPOSED:
    this->InvokeEvent(vtkCommand::ExposeEvent, nullptr);
    break;
  case SDL_EVENT_WINDOW_MOUSE_ENTER:
    this->InvokeEvent(vtkCommand::EnterEvent, nullptr);
    break;
  case SDL_EVENT_WINDOW_MOUSE_LEAVE:
    this->InvokeEvent(vtkCommand::LeaveEvent, nullptr);
    break;
  case SDL_EVENT_MOUSE_MOTION:
    pointer(e.motion.x, e.motion.y);
    this->InvokeEvent(vtkCommand::MouseMoveEvent, nullptr);
    break;
  case SDL_EVENT_MOUSE_BUTTON_DOWN:
  case SDL_EVENT_MOUSE_BUTTON_UP: {
    const bool down = e.type == SDL_EVENT_MOUSE_BUTTON_DOWN;
    pointer(e.button.x, e.button.y, down && e.button.clicks > 1 ? 1 : 0); // a double click repeats
    unsigned long ev = 0;
    switch (e.button.button) {
    case SDL_BUTTON_LEFT:
      ev = down ? vtkCommand::LeftButtonPressEvent : vtkCommand::LeftButtonReleaseEvent;
      break;
    case SDL_BUTTON_MIDDLE:
      ev = down ? vtkCommand::MiddleButtonPressEvent : vtkCommand::MiddleButtonReleaseEvent;
      break;
    case SDL_BUTTON_RIGHT:
      ev = down ? vtkCommand::RightButtonPressEvent : vtkCommand::RightButtonReleaseEvent;
      break;
    case SDL_BUTTON_X1:
      ev = down ? vtkCommand::FourthButtonPressEvent : vtkCommand::FourthButtonReleaseEvent;
      break;
    case SDL_BUTTON_X2:
      ev = down ? vtkCommand::FifthButtonPressEvent : vtkCommand::FifthButtonReleaseEvent;
      break;
    default:
      break;
    }
    if (ev)
      this->InvokeEvent(ev, nullptr);
    break;
  }
  case SDL_EVENT_MOUSE_WHEEL: {
    // A notch is one step; a trackpad's fractions add up to one.
    const float flip = e.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -1.0f : 1.0f;
    wheelY_ += flip * e.wheel.y;
    wheelX_ += flip * e.wheel.x;
    pointer(e.wheel.mouse_x, e.wheel.mouse_y);
    for (; wheelY_ >= 1.0f; wheelY_ -= 1.0f)
      this->InvokeEvent(vtkCommand::MouseWheelForwardEvent, nullptr);
    for (; wheelY_ <= -1.0f; wheelY_ += 1.0f)
      this->InvokeEvent(vtkCommand::MouseWheelBackwardEvent, nullptr);
    for (; wheelX_ >= 1.0f; wheelX_ -= 1.0f)
      this->InvokeEvent(vtkCommand::MouseWheelRightEvent, nullptr);
    for (; wheelX_ <= -1.0f; wheelX_ += 1.0f)
      this->InvokeEvent(vtkCommand::MouseWheelLeftEvent, nullptr);
    break;
  }
  case SDL_EVENT_KEY_DOWN:
  case SDL_EVENT_KEY_UP: {
    const std::string sym = vtk_keysym(e.key.key);
    this->SetKeyEventInformation(ctrl, shift, vtk_keycode(e.key), e.key.repeat ? 1 : 0,
                                 sym.c_str());
    this->InvokeEvent(e.type == SDL_EVENT_KEY_DOWN ? vtkCommand::KeyPressEvent
                                                   : vtkCommand::KeyReleaseEvent,
                      nullptr);
    break;
  }
  case SDL_EVENT_TEXT_INPUT: {
    // The typed text, a character at a time, as VTK's CharEvent: its KeyCode is a char, so Latin-1
    // (what ImGuiOverlay feeds ImGui as a code point); the keysym stays the key's, as in VTK.
    const char *p = e.text.text ? e.text.text : "";
    for (std::uint32_t cp = next_codepoint(p); cp; cp = next_codepoint(p)) {
      if (cp > 0xFF)
        continue;
      this->SetKeyEventInformation(ctrl, shift, static_cast<char>(cp), 0, nullptr);
      this->InvokeEvent(vtkCommand::CharEvent, nullptr);
    }
    break;
  }
  default:
    if (e.type == timer_event()) {
      int vtkId = this->GetVTKTimerId(e.user.code);
      if (vtkId != 0)
        this->InvokeEvent(vtkCommand::TimerEvent, &vtkId);
    }
    break;
  }
}

int SdlRenderWindowInteractor::InternalCreateTimer(int timerId, int timerType,
                                                   unsigned long duration) {
  if (!timer_event())
    return 0;
  const std::intptr_t tick =
      (static_cast<std::intptr_t>(timerId) << 1) | (timerType == RepeatingTimer ? 1 : 0);
  const SDL_TimerID id = SDL_AddTimer(static_cast<Uint32>(duration > 0 ? duration : 1), on_timer,
                                      reinterpret_cast<void *>(tick));
  if (!id)
    return 0;
  timers_[timerId] = id;
  timer_routes()[timerId] = this;
  return timerId;
}

int SdlRenderWindowInteractor::InternalDestroyTimer(int platformTimerId) {
  const auto it = timers_.find(platformTimerId);
  if (it == timers_.end())
    return 0;
  SDL_RemoveTimer(it->second); // false for a one-shot that already fired: nothing to remove
  timers_.erase(it);
  timer_routes().erase(platformTimerId);
  return 1;
}

} // namespace gl
} // namespace cvc

#endif // CVC_ENABLE_SDL
