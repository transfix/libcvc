// cvc::gl::FpsHud — two-way cvc::state binding, headless (no window, no
// renderer: only the state surface and config members are exercised).
//   * seeded defaults land in the state tree under the canonical path;
//   * writing state changes the HUD (enabled, toggle key, update_hz);
//   * driving the HUD writes state back (setEnabled/setToggleKey);
//   * key-sym normalization matches CameraController ("F" == "f").
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/gl/FpsHud.h>
#include <string>

using cvc::gl::FpsHud;

static int failures = 0;
static void check(const char *what, bool ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    ++failures;
}

int main() {
  // Own an explicit cvc::app and inject it — no global/singleton context.
  cvc::app app;
  cvc::app &ctx = app;
  cvc::state &root = cvc::state::instance(ctx);

  check("canonical viewer state path",
        FpsHud::viewerStatePath("forest", "main") == "forest.viewers.main.hud");

  FpsHud hud(ctx, "test.hud");

  // Seeded defaults.
  check("default enabled", hud.enabled());
  check("seeded enabled=1", root("test.hud.enabled").value<int>() == 1);
  check("seeded toggle key 'f'", root("test.hud.keys.toggle").value() == "f");
  check("seeded update_hz", root("test.hud.update_hz").value<double>() == 2.0);

  // state -> HUD
  root("test.hud.enabled").value(0);
  check("state write disables", !hud.enabled());
  root("test.hud.keys.toggle").value(std::string("h"));
  check("state write rebinds toggle key", hud.toggleKey() == "h");

  // HUD -> state
  hud.setEnabled(true);
  check("setEnabled mirrors to state", root("test.hud.enabled").value<int>() == 1);
  hud.setToggleKey("G"); // upper-case key syms normalize like the camera's
  check("setToggleKey normalizes + mirrors", root("test.hud.keys.toggle").value() == "g");

  hud.setUpdateHz(4.0);
  check("setUpdateHz mirrors to state", root("test.hud.update_hz").value<double>() == 4.0);

  printf("%s\n", failures == 0 ? "ALL PASS" : "FAILURES");
  return failures == 0 ? 0 : 1;
}
