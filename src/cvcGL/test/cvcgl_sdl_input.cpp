// SdlInput translation: push synthetic SDL events onto the queue and confirm SdlInput::poll()
// returns the normalized InputEvents. Headless — the SDL EVENTS subsystem needs no window, and
// SDL_PushEvent/SDL_PollEvent round-trip without a display. SKIPs (rc 0) in a build without SDL
// (CVC_ENABLE_SDL off) or if the events subsystem is unavailable.

#include <cstdio>
#include <cvc/gl/SdlInput.h>

#ifdef CVC_ENABLE_SDL
#include <SDL3/SDL.h>
#endif

int main() {
#ifndef CVC_ENABLE_SDL
  std::printf("SKIP: cvcGL built without SDL (CVC_ENABLE_SDL off)\n");
  return 0;
#else
  cvc::gl::SdlInput in;
  if (!in.init()) {
    std::printf("SKIP: SDL events subsystem unavailable (%s)\n", SDL_GetError());
    return 0;
  }

  // Synthetic key-down 'A'.
  SDL_Event ke;
  SDL_zero(ke);
  ke.type = SDL_EVENT_KEY_DOWN;
  ke.key.key = SDLK_A;
  ke.key.down = true;
  if (!SDL_PushEvent(&ke)) {
    std::printf("SKIP: SDL_PushEvent failed (%s)\n", SDL_GetError());
    return 0;
  }
  // Synthetic mouse move to (12, 34).
  SDL_Event me;
  SDL_zero(me);
  me.type = SDL_EVENT_MOUSE_MOTION;
  me.motion.x = 12.0f;
  me.motion.y = 34.0f;
  SDL_PushEvent(&me);
  // Synthetic left-button click.
  SDL_Event be;
  SDL_zero(be);
  be.type = SDL_EVENT_MOUSE_BUTTON_DOWN;
  be.button.button = 1;
  be.button.clicks = 1;
  be.button.x = 5.0f;
  be.button.y = 6.0f;
  SDL_PushEvent(&be);

  const std::vector<cvc::gl::InputEvent> evs = in.poll();

  using Kind = cvc::gl::InputEvent::Kind;
  bool sawKey = false, sawMove = false, sawButton = false;
  for (const auto &e : evs) {
    if (e.kind == Kind::KeyDown && e.key == "A")
      sawKey = true;
    if (e.kind == Kind::MouseMove && e.x == 12.0f && e.y == 34.0f)
      sawMove = true;
    if (e.kind == Kind::MouseButtonDown && e.button == 1 && e.clicks == 1)
      sawButton = true;
  }
  int rc = 0;
  if (!sawKey) {
    std::printf("FAIL: key-down 'A' not translated\n");
    rc = 1;
  }
  if (!sawMove) {
    std::printf("FAIL: mouse-move (12,34) not translated\n");
    rc = 1;
  }
  if (!sawButton) {
    std::printf("FAIL: mouse-button-down (1) not translated\n");
    rc = 1;
  }
  if (rc == 0)
    std::printf("OK: SDL input translated (%zu event(s))\n", evs.size());
  return rc;
#endif
}
