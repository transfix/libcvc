// SdlInput — SDL3-backed input source. SDL is confined here; the header is SDL-free.
// Compiled always (the file glob picks it up); the SDL body is behind CVC_ENABLE_SDL, so a build
// without SDL yields an inert source (init() false, poll() empty) — the VTK-interactor path as
// today.

#include <cvc/gl/SdlInput.h>

#ifdef CVC_ENABLE_SDL
#include <SDL3/SDL.h>
#include <cstdint>
#include <cstdlib>
#include <vtkRenderWindow.h>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif
#endif

namespace cvc {
namespace gl {

#ifdef CVC_ENABLE_SDL
namespace {

InputEvent key_event(const SDL_Event &e) {
  InputEvent ie;
  ie.kind = (e.type == SDL_EVENT_KEY_DOWN) ? InputEvent::Kind::KeyDown : InputEvent::Kind::KeyUp;
  const char *name = SDL_GetKeyName(e.key.key); // "A", "Escape", "Space", "Left", …
  ie.key = name ? name : "";
  ie.mods = static_cast<unsigned>(e.key.mod);
  ie.repeat = e.key.repeat;
  return ie;
}

#ifdef __EMSCRIPTEN__
// In the browser SDL calls preventDefault() on every key event it queues, which would swallow the
// keypress the browser makes from a keydown -- the one VTK turns into typed characters for ImGui.
// A key event a filter drops is not prevented, so the filter keeps the keys for poll() itself.
bool SDLCALL keep_keys(void *keys, SDL_Event *e) {
  if (e->type != SDL_EVENT_KEY_DOWN && e->type != SDL_EVENT_KEY_UP)
    return true;
  static_cast<std::vector<InputEvent> *>(keys)->push_back(key_event(*e));
  return false;
}
#endif

} // namespace
#endif

bool SdlInput::init() {
#ifdef CVC_ENABLE_SDL
  if (inited_)
    return true;
  // Events subsystem only: no window/video, so this works headless (and a windowed host that
  // already initialized SDL_INIT_VIDEO just shares the refcounted subsystem). SDL3 returns bool.
  if (!SDL_InitSubSystem(SDL_INIT_EVENTS))
    return false;
  inited_ = true;
  return true;
#else
  return false;
#endif
}

bool SdlInput::available() const {
#ifdef CVC_ENABLE_SDL
  return inited_;
#else
  return false;
#endif
}

bool SdlInput::attach(vtkRenderWindow *host) {
#ifdef CVC_ENABLE_SDL
  if (window_)
    return true;
  if (!host || !init())
    return false;
#if defined(__APPLE__)
  (void)host;
  return false; // VTK's Cocoa window hands out its NSView; SDL wraps an NSWindow. Not yet.
#else
#ifndef __EMSCRIPTEN__
  void *native = host->GetGenericWindowId();
  if (!native)
    return false; // VTK makes its window at the first render: try again on a later frame
#endif
  if (!video_) {
    if (!SDL_InitSubSystem(SDL_INIT_VIDEO))
      return false;
    video_ = true;
  }
  const SDL_PropertiesID props = SDL_CreateProperties();
#if defined(__EMSCRIPTEN__)
  // SDL sizes the canvas its window is made on, and zeroes it when the window goes, which would
  // fight VTK over its canvas. So SDL's window is a hidden canvas of its own, and only its keyboard
  // listeners go on VTK's canvas (Module.canvas), where the keys arrive.
  char *keys = static_cast<char *>(MAIN_THREAD_EM_ASM_PTR({
    if (!document.getElementById('cvcgl-sdl-input')) {
      const c = document.createElement('canvas');
      c.id = 'cvcgl-sdl-input';
      c.width = c.height = 1;
      c.style.cssText =
          'position:fixed;left:0;top:0;width:1px;height:1px;visibility:hidden;pointer-events:none';
      document.body.appendChild(c);
    }
    const id = Module['canvas'] && Module['canvas'].id ? Module['canvas'].id : 'canvas';
    return stringToNewUTF8('#' + id);
  }));
  SDL_SetStringProperty(props, SDL_PROP_WINDOW_CREATE_EMSCRIPTEN_CANVAS_ID_STRING,
                        "#cvcgl-sdl-input");
  SDL_SetStringProperty(props, SDL_PROP_WINDOW_CREATE_EMSCRIPTEN_KEYBOARD_ELEMENT_STRING, keys);
  SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_WIDTH_NUMBER, 1);
  SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_HEIGHT_NUMBER, 1);
  std::free(keys);
#elif defined(_WIN32)
  // SDL subclasses the window's procedure and passes every message on to VTK's.
  SDL_SetPointerProperty(props, SDL_PROP_WINDOW_CREATE_WIN32_HWND_POINTER, native);
#else
  SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_X11_WINDOW_NUMBER,
                        static_cast<Sint64>(reinterpret_cast<std::uintptr_t>(native)));
#endif
  window_ = SDL_CreateWindowWithProperties(props);
  SDL_DestroyProperties(props);
#ifdef __EMSCRIPTEN__
  if (window_)
    SDL_SetEventFilter(keep_keys, &keys_);
#endif
  return window_ != nullptr;
#endif
#else
  (void)host;
  return false;
#endif
}

bool SdlInput::attached() const { return window_ != nullptr; }

std::vector<InputEvent> SdlInput::poll() {
  std::vector<InputEvent> out;
#ifdef CVC_ENABLE_SDL
  if (!inited_)
    return out;
  SDL_PumpEvents(); // the browser's keys reach keys_ through keep_keys as they are pumped
  out.swap(keys_);
  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    switch (e.type) {
    case SDL_EVENT_KEY_DOWN:
    case SDL_EVENT_KEY_UP:
      out.push_back(key_event(e));
      break;
    case SDL_EVENT_MOUSE_MOTION: {
      InputEvent ie;
      ie.kind = InputEvent::Kind::MouseMove;
      ie.x = e.motion.x;
      ie.y = e.motion.y;
      ie.dx = e.motion.xrel;
      ie.dy = e.motion.yrel;
      out.push_back(ie);
      break;
    }
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
    case SDL_EVENT_MOUSE_BUTTON_UP: {
      InputEvent ie;
      ie.kind = (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN) ? InputEvent::Kind::MouseButtonDown
                                                        : InputEvent::Kind::MouseButtonUp;
      ie.x = e.button.x;
      ie.y = e.button.y;
      ie.button = e.button.button;
      ie.clicks = e.button.clicks;
      out.push_back(ie);
      break;
    }
    case SDL_EVENT_MOUSE_WHEEL: {
      InputEvent ie;
      ie.kind = InputEvent::Kind::MouseWheel;
      ie.x = e.wheel.mouse_x;
      ie.y = e.wheel.mouse_y;
      ie.dx = e.wheel.x;
      ie.dy = e.wheel.y;
      out.push_back(ie);
      break;
    }
    default:
      break; // quit / window / text / gamepad-etc. — the host owns those (v1 = keyboard + mouse)
    }
  }
#endif
  return out;
}

void SdlInput::shutdown() {
#ifdef CVC_ENABLE_SDL
  if (window_) {
#ifdef __EMSCRIPTEN__
    SDL_SetEventFilter(nullptr, nullptr);
#endif
    // A wrapped window is the host's: SDL gives its window procedure back and leaves it standing.
    SDL_DestroyWindow(static_cast<SDL_Window *>(window_));
    window_ = nullptr;
  }
  if (video_) {
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
    video_ = false;
  }
  if (inited_) {
    SDL_QuitSubSystem(SDL_INIT_EVENTS);
    inited_ = false;
  }
#endif
}

SdlInput::~SdlInput() { shutdown(); }

} // namespace gl
} // namespace cvc
