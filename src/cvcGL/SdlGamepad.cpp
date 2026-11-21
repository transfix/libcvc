// SdlGamepad — the first SDL gamepad, read by its inputs' names (SdlGamepad.h). SDL is confined
// here; without CVC_ENABLE_SDL there is never a gamepad.

#include <algorithm>
#include <cvc/gl/SdlGamepad.h>
#include <string_view>
#include <utility>
#ifdef CVC_ENABLE_SDL
#include <SDL3/SDL.h>
#endif

namespace cvc {
namespace gl {

#ifdef CVC_ENABLE_SDL
namespace {

// Updates between looks for a gamepad while none is open (about a second at 60 Hz): SDL's list
// is cheap, not free.
constexpr int kScanEvery = 60;

constexpr std::pair<std::string_view, SDL_GamepadButton> kButtons[] = {
    {"A", SDL_GAMEPAD_BUTTON_SOUTH},
    {"B", SDL_GAMEPAD_BUTTON_EAST},
    {"X", SDL_GAMEPAD_BUTTON_WEST},
    {"Y", SDL_GAMEPAD_BUTTON_NORTH},
    {"LB", SDL_GAMEPAD_BUTTON_LEFT_SHOULDER},
    {"RB", SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER},
    {"Back", SDL_GAMEPAD_BUTTON_BACK},
    {"Start", SDL_GAMEPAD_BUTTON_START},
    {"Guide", SDL_GAMEPAD_BUTTON_GUIDE},
    {"LeftStick", SDL_GAMEPAD_BUTTON_LEFT_STICK},
    {"RightStick", SDL_GAMEPAD_BUTTON_RIGHT_STICK},
    {"DPadUp", SDL_GAMEPAD_BUTTON_DPAD_UP},
    {"DPadDown", SDL_GAMEPAD_BUTTON_DPAD_DOWN},
    {"DPadLeft", SDL_GAMEPAD_BUTTON_DPAD_LEFT},
    {"DPadRight", SDL_GAMEPAD_BUTTON_DPAD_RIGHT},
};

constexpr std::pair<std::string_view, SDL_GamepadAxis> kAxes[] = {
    {"LeftX", SDL_GAMEPAD_AXIS_LEFTX},         {"LeftY", SDL_GAMEPAD_AXIS_LEFTY},
    {"RightX", SDL_GAMEPAD_AXIS_RIGHTX},       {"RightY", SDL_GAMEPAD_AXIS_RIGHTY},
    {"LT", SDL_GAMEPAD_AXIS_LEFT_TRIGGER},     {"RT", SDL_GAMEPAD_AXIS_RIGHT_TRIGGER},
};

SDL_Gamepad *as_pad(void *p) { return static_cast<SDL_Gamepad *>(p); }

} // namespace
#endif

SdlGamepad::~SdlGamepad() {
#ifdef CVC_ENABLE_SDL
  if (pad_)
    SDL_CloseGamepad(as_pad(pad_));
  pad_ = nullptr;
  if (inited_)
    SDL_QuitSubSystem(SDL_INIT_GAMEPAD);
  inited_ = false;
#endif
}

bool SdlGamepad::update() {
#ifdef CVC_ENABLE_SDL
  if (failed_)
    return false;
  if (!inited_) {
    if (!SDL_InitSubSystem(SDL_INIT_GAMEPAD)) {
      failed_ = true; // no joystick support here: never ask again
      return false;
    }
    inited_ = true;
  }
  // The window's event loop pumps SDL's gamepads; a host without SDL's window has none, so pump
  // them here as well (twice in a frame is harmless).
  SDL_UpdateGamepads();
  if (pad_ && !SDL_GamepadConnected(as_pad(pad_))) {
    SDL_CloseGamepad(as_pad(pad_));
    pad_ = nullptr;
    scan_ = 0;
  }
  if (!pad_ && --scan_ <= 0) {
    scan_ = kScanEvery;
    if (SDL_HasGamepad()) {
      int count = 0;
      SDL_JoystickID *ids = SDL_GetGamepads(&count);
      if (ids && count > 0)
        pad_ = SDL_OpenGamepad(ids[0]);
      SDL_free(ids);
    }
  }
  return pad_ != nullptr;
#else
  return false;
#endif
}

std::string SdlGamepad::name() const {
#ifdef CVC_ENABLE_SDL
  if (pad_) {
    const char *n = SDL_GetGamepadName(as_pad(pad_));
    return n ? n : "";
  }
#endif
  return {};
}

double SdlGamepad::input(const std::string &name) const {
#ifdef CVC_ENABLE_SDL
  if (!pad_)
    return 0.0;
  for (const auto &[n, b] : kButtons)
    if (n == name)
      return SDL_GetGamepadButton(as_pad(pad_), b) ? 1.0 : 0.0;
  for (const auto &[n, a] : kAxes)
    if (n == name)
      return std::clamp(SDL_GetGamepadAxis(as_pad(pad_), a) / 32767.0, -1.0, 1.0);
#else
  (void)name;
#endif
  return 0.0;
}

} // namespace gl
} // namespace cvc
