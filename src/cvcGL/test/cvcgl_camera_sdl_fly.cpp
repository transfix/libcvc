// The camera driven the way arigl drives it: SDL's events, on a real window, through VTK's
// interactor to the CameraController (and past the ImGuiOverlay, which takes only what ImGui
// wants). Tab goes to Fly; W held over frames flies forward; the mouse turns the view; Tab goes
// back to Orbit; the wheel glides the orbit distance to its goal. Without SDL, or where SDL opens
// no window, it skips: exit 77.
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <string>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#ifdef CVC_ENABLE_SDL
#include <SDL3/SDL.h>
#endif

namespace {

int failures = 0;

void chk(bool ok, const std::string &what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
  if (!ok)
    ++failures;
}

} // namespace

int main() {
#ifndef CVC_ENABLE_SDL
  std::printf("SKIP: cvcGL built without SDL\n");
  return 77;
#else
  cvc::app app;
  cvc::gl::SceneGraph sg(app, "camera_sdl_fly");
  // Over the overlay's phone size (its short side 500 px), so its desktop layout takes no press.
  cvc::gl::SceneRenderer view(sg, 800, 600, /*offscreen=*/false, "main");
  cvc::gl::CameraController cam(view);
  cam.setMode(cvc::gl::CameraController::Mode::Orbit);
  cvc::gl::ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  ui.setVisible(true);
  view.render();
  vtkRenderWindow *win = view.renderWindow();
  SDL_Window *sw = win ? static_cast<SDL_Window *>(win->GetGenericWindowId()) : nullptr;
  if (!win || !sw || std::string(win->GetClassName()) != "SdlGLRenderWindow") {
    std::printf("SKIP: no SDL window here (%s)\n", win ? win->GetClassName() : "none");
    return 77;
  }
  cam.frameBounds(-100, -100, 0, 100, 100, 50);
  const SDL_WindowID id = SDL_GetWindowID(sw);
  const auto key = [id](SDL_Keycode k, bool down) {
    SDL_Event e{};
    e.type = down ? SDL_EVENT_KEY_DOWN : SDL_EVENT_KEY_UP;
    e.key.windowID = id;
    e.key.key = k;
    e.key.down = down;
    SDL_PushEvent(&e);
  };
  const auto frame = [&](double dt) {
    view.processUIEvents();
    cam.update(dt);
    view.render();
  };
  double e0[3], f0[3], u[3], e1[3], f1[3];
  for (int i = 0; i < 3; ++i)
    frame(1.0 / 60.0);

  key(SDLK_TAB, true);
  key(SDLK_TAB, false);
  frame(1.0 / 60.0);
  chk(cam.mode() == cvc::gl::CameraController::Mode::Fly, "Tab through SDL: Fly");

  cam.getPose(e0, f0, u);
  key(SDLK_W, true);
  for (int i = 0; i < 30; ++i)
    frame(1.0 / 60.0);
  key(SDLK_W, false);
  frame(1.0 / 60.0);
  cam.getPose(e1, f1, u);
  double fwd[3] = {f0[0] - e0[0], f0[1] - e0[1], f0[2] - e0[2]};
  const double fl = std::sqrt(fwd[0] * fwd[0] + fwd[1] * fwd[1] + fwd[2] * fwd[2]);
  const double moved =
      ((e1[0] - e0[0]) * fwd[0] + (e1[1] - e0[1]) * fwd[1] + (e1[2] - e0[2]) * fwd[2]) / fl;
  std::printf("     W held 0.5 s: moved %.2f along the view\n", moved);
  chk(moved > 1.0, "W held through SDL flies forward");

  cam.getPose(e0, f0, u);
  for (int i = 0; i < 10; ++i) {
    SDL_Event m{};
    m.type = SDL_EVENT_MOUSE_MOTION;
    m.motion.windowID = id;
    m.motion.x = 160.0f + 8.0f * (i + 1);
    m.motion.y = 120.0f;
    m.motion.xrel = 8.0f;
    SDL_PushEvent(&m);
    frame(1.0 / 60.0);
  }
  cam.getPose(e1, f1, u);
  const double turned =
      std::fabs((f1[0] - e1[0]) - (f0[0] - e0[0])) + std::fabs((f1[1] - e1[1]) - (f0[1] - e0[1]));
  std::printf("     mouse moved 80 px: view direction changed by %.3f\n", turned);
  chk(turned > 1e-3, "the mouse through SDL turns the view in Fly");

  // Without the pointer held (a browser that has not granted its lock), a bare move of the cursor
  // turns nothing; a drag does.
  cam.setPointerCapture(false);
  cam.getPose(e0, f0, u);
  const auto motion = [&](float x, float xrel) {
    SDL_Event m{};
    m.type = SDL_EVENT_MOUSE_MOTION;
    m.motion.windowID = id;
    m.motion.x = x;
    m.motion.y = 120.0f;
    m.motion.xrel = xrel;
    SDL_PushEvent(&m);
    frame(1.0 / 60.0);
  };
  for (int i = 0; i < 5; ++i)
    motion(100.0f + 8.0f * i, 8.0f);
  cam.getPose(e1, f1, u);
  const double bare =
      std::fabs((f1[0] - e1[0]) - (f0[0] - e0[0])) + std::fabs((f1[1] - e1[1]) - (f0[1] - e0[1]));
  chk(bare < 1e-9, "Fly without the pointer held: a bare move turns nothing");
  {
    SDL_Event b{};
    b.type = SDL_EVENT_MOUSE_BUTTON_DOWN;
    b.button.windowID = id;
    b.button.button = SDL_BUTTON_LEFT;
    b.button.down = true;
    b.button.clicks = 1;
    b.button.x = 140.0f;
    b.button.y = 120.0f;
    SDL_PushEvent(&b);
    frame(1.0 / 60.0);
    for (int i = 0; i < 5; ++i)
      motion(148.0f + 8.0f * i, 8.0f);
    b.type = SDL_EVENT_MOUSE_BUTTON_UP;
    b.button.down = false;
    b.button.x = 180.0f;
    SDL_PushEvent(&b);
    frame(1.0 / 60.0);
  }
  cam.getPose(e0, f0, u);
  const double dragged =
      std::fabs((f0[0] - e0[0]) - (f1[0] - e1[0])) + std::fabs((f0[1] - e0[1]) - (f1[1] - e1[1]));
  chk(dragged > 1e-3, "and a drag turns the view");

  key(SDLK_TAB, true);
  key(SDLK_TAB, false);
  frame(1.0 / 60.0);
  chk(cam.mode() == cvc::gl::CameraController::Mode::Orbit, "Tab again: Orbit");

  // The wheel glides: two notches set the goal 0.81 of the distance, a frame goes part of the way,
  // and the glide ends there.
  const auto dist = [&] {
    double e[3], f[3], uu[3];
    cam.getPose(e, f, uu);
    return std::sqrt((e[0] - f[0]) * (e[0] - f[0]) + (e[1] - f[1]) * (e[1] - f[1]) +
                     (e[2] - f[2]) * (e[2] - f[2]));
  };
  const double d0 = dist();
  for (int i = 0; i < 2; ++i) {
    SDL_Event w{};
    w.type = SDL_EVENT_MOUSE_WHEEL;
    w.wheel.windowID = id;
    w.wheel.y = 1.0f;
    w.wheel.mouse_x = 160.0f;
    w.wheel.mouse_y = 120.0f;
    SDL_PushEvent(&w);
  }
  frame(1.0 / 60.0);
  const double d1 = dist();
  for (int i = 0; i < 120; ++i)
    frame(1.0 / 60.0);
  const double d2 = dist();
  std::printf("     wheel x2: %.3f -> %.3f after a frame -> %.3f (goal %.3f)\n", d0, d1, d2,
              0.81 * d0);
  chk(d1 < d0 && d1 > 0.81 * d0 + 1e-6, "the wheel's zoom goes part of the way in a frame");
  chk(std::fabs(d2 - 0.81 * d0) < 1e-6 * d0 && !cam.zooming(), "and ends at its goal");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}
