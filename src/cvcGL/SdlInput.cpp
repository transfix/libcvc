// SdlInput — SDL3-backed input source. SDL is confined here; the header is SDL-free.
// Compiled always (the file glob picks it up); the SDL body is behind CVC_ENABLE_SDL, so a build
// without SDL yields an inert source (init() false, poll() empty) — the VTK-interactor path as
// today.

#include <cvc/gl/SdlInput.h>

#ifdef CVC_ENABLE_SDL
#include <SDL3/SDL.h>
#endif

namespace cvc {
namespace gl {

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

std::vector<InputEvent> SdlInput::poll() {
  std::vector<InputEvent> out;
#ifdef CVC_ENABLE_SDL
  if (!inited_)
    return out;
  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    switch (e.type) {
    case SDL_EVENT_KEY_DOWN:
    case SDL_EVENT_KEY_UP: {
      InputEvent ie;
      ie.kind =
          (e.type == SDL_EVENT_KEY_DOWN) ? InputEvent::Kind::KeyDown : InputEvent::Kind::KeyUp;
      const char *name = SDL_GetKeyName(e.key.key); // "A", "Escape", "Space", "Left", …
      ie.key = name ? name : "";
      ie.mods = static_cast<unsigned>(e.key.mod);
      ie.repeat = e.key.repeat;
      out.push_back(std::move(ie));
      break;
    }
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
  if (inited_) {
    SDL_QuitSubSystem(SDL_INIT_EVENTS);
    inited_ = false;
  }
#endif
}

SdlInput::~SdlInput() { shutdown(); }

} // namespace gl
} // namespace cvc
