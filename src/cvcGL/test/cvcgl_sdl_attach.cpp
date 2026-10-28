// cvcGL's onscreen window on SDL3 (SdlWindow.h), on a real window: SDL makes the window and its GL
// context and VTK renders into it; an SDL event reaches VTK's interactor as a VTK event (a key with
// its X11 keysym and the character it types, a click at its pixel with y up, typed text as a
// CharEvent) and reaches SdlInput too, in SDL's names; a resize sets VTK's size; VTK's timers tick;
// with two windows open, each window's events reach its own interactor, whichever drains SDL's
// queue; and an SdlInput that is not attached leaves the window's events alone. On Windows a key
// message sent to SDL's window goes the OS's way, through SDL's window procedure, to both. With
// $CVCGL_SDL_WINDOW=0 the window is VTK's own, and SdlInput reads VTK's key events instead (checked
// on Windows, where a key message can be sent to VTK's window). SKIPs (rc 0) without SDL or where
// SDL opens no window.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/SdlInput.h>
#include <string>
#include <vector>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#ifdef CVC_ENABLE_SDL
#include <SDL3/SDL.h>
#endif
#ifdef _WIN32
#include <windows.h>
#endif

namespace {

int failures = 0;

void chk(bool ok, const std::string &what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
  if (!ok)
    ++failures;
}

// What VTK's interactor saw: the event, its keysym and key code / position / timer.
struct Seen {
  std::vector<std::string> log;
  static void cb(vtkObject *caller, unsigned long eid, void *self, void *data) {
    auto *iren = static_cast<vtkRenderWindowInteractor *>(caller);
    std::string e;
    if (eid == vtkCommand::KeyPressEvent) {
      e = std::string("key:") + (iren->GetKeySym() ? iren->GetKeySym() : "");
      const char c = iren->GetKeyCode();
      if (c >= 32 && c < 127)
        e += std::string(":") + c;
    } else if (eid == vtkCommand::CharEvent)
      e = std::string("char:") + iren->GetKeyCode();
    else if (eid == vtkCommand::LeftButtonPressEvent)
      e = "press:" + std::to_string(iren->GetEventPosition()[0]) + "," +
          std::to_string(iren->GetEventPosition()[1]);
    else if (eid == vtkCommand::TimerEvent)
      e = "timer:" + std::to_string(data ? *static_cast<int *>(data) : 0);
    else
      return;
    static_cast<Seen *>(self)->log.push_back(e);
  }
  bool has(const std::string &s) const {
    for (const std::string &l : log)
      if (l == s)
        return true;
    return false;
  }
  vtkSmartPointer<vtkCallbackCommand> observe(vtkRenderWindowInteractor *iren) {
    auto cb = vtkSmartPointer<vtkCallbackCommand>::New();
    cb->SetClientData(this);
    cb->SetCallback(&Seen::cb);
    for (unsigned long e : {vtkCommand::KeyPressEvent, vtkCommand::CharEvent,
                            vtkCommand::LeftButtonPressEvent, vtkCommand::TimerEvent})
      iren->AddObserver(e, cb);
    return cb;
  }
};

void setenv_(const char *k, const char *v) {
#ifdef _WIN32
  _putenv_s(k, v);
#else
  setenv(k, v, 1);
#endif
}

} // namespace

int main() {
#ifndef CVC_ENABLE_SDL
  std::printf("SKIP: cvcGL built without SDL\n");
  return 0;
#else
  cvc::app app;
  cvc::gl::SceneGraph sg(app, "sdl_attach");
  cvc::gl::SceneRenderer view(sg, 240, 160, /*offscreen=*/false, "main");
  view.render(); // the window and its GL context exist from the first render
  vtkRenderWindow *win = view.renderWindow();
  vtkRenderWindowInteractor *iren = win ? win->GetInteractor() : nullptr;
  SDL_Window *sw = win ? static_cast<SDL_Window *>(win->GetGenericWindowId()) : nullptr;
  if (!win || !iren || !sw || std::string(win->GetClassName()) != "SdlGLRenderWindow") {
    std::printf("SKIP: no SDL window here (%s)\n", win ? win->GetClassName() : "none");
    return 0;
  }
  chk(win->GetSize()[0] > 0 && win->GetSize()[1] > 0, "VTK renders into SDL's window");

  Seen seen;
  auto cb = seen.observe(iren);

  const SDL_WindowID id = SDL_GetWindowID(sw);
  const auto key = [](SDL_WindowID to, SDL_Keycode k, bool down, SDL_Keymod mod = SDL_KMOD_NONE) {
    SDL_Event e{};
    e.type = down ? SDL_EVENT_KEY_DOWN : SDL_EVENT_KEY_UP;
    e.key.windowID = to;
    e.key.key = k;
    e.key.mod = mod;
    e.key.down = down;
    SDL_PushEvent(&e);
  };

  // An SdlInput that is not attached leaves SDL's queue to the window.
  {
    cvc::gl::SdlInput loose;
    loose.init();
    key(id, SDLK_HOME, true);
    chk(loose.poll().empty(), "an SdlInput not attached takes nothing off the window's queue");
    view.processUIEvents();
    chk(seen.has("key:Home"), "and the window's interactor still gets the key");
  }

  cvc::gl::SdlInput in;
  chk(in.attach(win) && in.attached(), "SdlInput takes the window's events");

  key(id, SDLK_LEFT, true);
  key(id, SDLK_LEFT, false);
  key(id, SDLK_PAGEUP, true);
  key(id, SDLK_COMMA, true);
  key(id, SDLK_A, true, SDL_KMOD_LSHIFT);
  SDL_Event text{};
  text.type = SDL_EVENT_TEXT_INPUT;
  text.text.windowID = id;
  text.text.text = "x\xc3\xa9"; // "xé" -- a literal: it outlives the queue
  SDL_PushEvent(&text);
  int lw = 0, lh = 0, pw = 0, ph = 0;
  SDL_GetWindowSize(sw, &lw, &lh);
  SDL_GetWindowSizeInPixels(sw, &pw, &ph);
  SDL_Event click{};
  click.type = SDL_EVENT_MOUSE_BUTTON_DOWN;
  click.button.windowID = id;
  click.button.button = SDL_BUTTON_LEFT;
  click.button.down = true;
  click.button.clicks = 1;
  click.button.x = 40.0f;
  click.button.y = 30.0f;
  SDL_PushEvent(&click);
  view.processUIEvents(); // the interactor drains SDL's queue

  chk(seen.has("key:Left") && seen.has("key:Prior"),
      "VTK's interactor gets the keys by their X11 keysyms (Left, Prior)");
  chk(seen.has("key:comma:,") && seen.has("key:a:A"),
      "a printable key is its unshifted keysym, with the character it types (comma ',', a 'A')");
  chk(seen.has("char:x") && seen.has(std::string("char:") + static_cast<char>(0xE9)),
      "typed text reaches VTK as CharEvents, Latin-1 (ImGui's text fields)");
  const int px = lw > 0 ? 40 * pw / lw : 40, py = lh > 0 ? 30 * ph / lh : 30;
  const std::string press = "press:" + std::to_string(px) + "," + std::to_string(ph - 1 - py);
  chk(seen.has(press), "a click reaches VTK at its pixel, y up (" + press + ")");

  bool left_down = false, left_up = false, page_up = false, click_seen = false;
  for (const cvc::gl::InputEvent &e : in.poll()) {
    left_down |= e.kind == cvc::gl::InputEvent::Kind::KeyDown && e.key == "Left";
    left_up |= e.kind == cvc::gl::InputEvent::Kind::KeyUp && e.key == "Left";
    page_up |= e.kind == cvc::gl::InputEvent::Kind::KeyDown && e.key == "PageUp";
    click_seen |= e.kind == cvc::gl::InputEvent::Kind::MouseButtonDown && e.button == 1;
  }
  chk(left_down && left_up && page_up, "SdlInput gets the same keys in SDL's names (PageUp)");
  chk(click_seen, "and the click");

  SDL_SetWindowSize(sw, 320, 200);
  SDL_SyncWindow(sw);
  view.processUIEvents();
  SDL_GetWindowSizeInPixels(sw, &pw, &ph);
  chk(win->GetSize()[0] == pw && win->GetSize()[1] == ph && iren->GetSize()[1] == ph,
      "a resize sets VTK's size to the window's pixels (" + std::to_string(pw) + "x" +
          std::to_string(ph) + ")");

  // VTK's timers run on SDL's.
  const int timer = iren->CreateOneShotTimer(5);
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!seen.has("timer:" + std::to_string(timer)) && std::chrono::steady_clock::now() < until)
    view.processUIEvents();
  chk(timer != 0 && seen.has("timer:" + std::to_string(timer)), "a VTK timer ticks");
  iren->DestroyTimer(timer);

  // A second SDL window: SDL has one queue, and each window's events reach its own interactor,
  // whichever interactor drains it.
  {
    cvc::gl::SceneGraph sg2(app, "sdl_attach_two");
    cvc::gl::SceneRenderer view2(sg2, 200, 120, /*offscreen=*/false, "second");
    view2.render();
    vtkRenderWindow *win2 = view2.renderWindow();
    vtkRenderWindowInteractor *iren2 = win2 ? win2->GetInteractor() : nullptr;
    SDL_Window *sw2 = win2 ? static_cast<SDL_Window *>(win2->GetGenericWindowId()) : nullptr;
    chk(iren2 && sw2 && std::string(win2->GetClassName()) == "SdlGLRenderWindow",
        "a second window is SDL's too");
    if (iren2 && sw2) {
      Seen seen2;
      auto cb2 = seen2.observe(iren2);
      seen.log.clear();
      key(SDL_GetWindowID(sw2), SDLK_END, true);
      key(id, SDLK_INSERT, true);
      view.processUIEvents(); // the first window's interactor drains both
      chk(seen.has("key:Insert") && !seen.has("key:End"), "the first window gets only its key");
      view2.processUIEvents();
      chk(seen2.has("key:End") && !seen2.has("key:Insert"),
          "the second window gets its own, kept for it");
    }
  }

#ifdef _WIN32
  // A key the OS sends the window goes through SDL's window procedure to both.
  {
    const HWND hwnd = static_cast<HWND>(SDL_GetPointerProperty(
        SDL_GetWindowProperties(sw), SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr));
    seen.log.clear();
    in.poll();
    const LPARAM down = 1 | (0x4D << 16) | (1 << 24); // VK_RIGHT: scan code 0x4D, extended
    SendMessageW(hwnd, WM_KEYDOWN, VK_RIGHT, down);
    SendMessageW(hwnd, WM_KEYUP, VK_RIGHT, down | (LPARAM(1) << 30) | (LPARAM(1) << 31));
    view.processUIEvents();
    bool sdl_right = false;
    for (const cvc::gl::InputEvent &e : in.poll())
      sdl_right |= e.kind == cvc::gl::InputEvent::Kind::KeyDown && e.key == "Right";
    chk(hwnd && seen.has("key:Right") && sdl_right,
        "a key message to SDL's window reaches VTK (Right) and SdlInput");
  }
#endif

  in.shutdown();
  seen.log.clear();
  key(id, SDLK_RIGHT, true);
  view.processUIEvents();
  chk(seen.has("key:Right") && in.poll().empty(), "after shutdown() VTK alone gets keys");

  // VTK's own window, as without SDL: SdlInput reads VTK's key events.
  setenv_("CVCGL_SDL_WINDOW", "0");
  {
    cvc::gl::SceneGraph sg2(app, "sdl_attach_vtk");
    cvc::gl::SceneRenderer vtkView(sg2, 240, 160, /*offscreen=*/false, "vtk");
    vtkView.render();
    vtkRenderWindow *vw = vtkView.renderWindow();
    chk(vw && vw->GetInteractor() && std::string(vw->GetClassName()) != "SdlGLRenderWindow",
        std::string("with $CVCGL_SDL_WINDOW=0 the window is VTK's (") +
            (vw ? vw->GetClassName() : "none") + ")");
#ifdef _WIN32
    // A held key repeats.
    const HWND hwnd = vw ? static_cast<HWND>(vw->GetGenericWindowId()) : nullptr;
    cvc::gl::SdlInput vk;
    chk(hwnd && vk.attach(vw), "and SdlInput takes VTK's keys");
    const LPARAM leftDown = 1 | (0x4B << 16) | (1 << 24);
    SendMessageW(hwnd, WM_KEYDOWN, VK_LEFT, leftDown);
    SendMessageW(hwnd, WM_KEYDOWN, VK_LEFT, leftDown | (LPARAM(1) << 30));
    SendMessageW(hwnd, WM_KEYUP, VK_LEFT, leftDown | (LPARAM(1) << 30) | (LPARAM(1) << 31));
    std::string seq;
    for (const cvc::gl::InputEvent &e : vk.poll())
      seq += std::string(e.kind == cvc::gl::InputEvent::Kind::KeyDown ? "down" : "up") + ":" +
             e.key + (e.repeat ? "(repeat)" : "") + " ";
    chk(seq == "down:Left down:Left(repeat) up:Left ", "a held key repeats, then goes up: " + seq);
#endif
  }
  setenv_("CVCGL_SDL_WINDOW", "");

  std::printf("cvcgl_sdl_attach: %s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}
