// The overlay's panels start hidden on a phone's view and open on anything larger, a laptop's
// browser canvas included. The rule is decided at the overlay's first frame, so each case is a new
// offscreen window of the size and pixel ratio it names, rendered a few times. Skips (rc 0) where
// the overlay has no GL to draw with, unless CVC_REQUIRE_RENDER=1.
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <string>

namespace {

int fails = 0;
void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

struct View {
  int w, h;    // device pixels, as the browser sizes VTK's canvas
  float ratio; // device pixels per logical (CSS) pixel
  const char *what;
  bool open; // the panels after the first frame
};

// The panels after the first frames of a w x h window at this pixel ratio: 1 open, 0 hidden, -1
// where the overlay could not draw. A few renders, as a host makes: the overlay's GL backend starts
// at the first, so ImGui's first frame is the second.
int panels_after_first_frames(int w, int h, float ratio) {
  cvc::app app;
  cvc::gl::SceneGraph sg(app, "panels");
  cvc::gl::SceneRenderer view(sg, w, h, /*offscreen=*/true, "panels");
  cvc::gl::ImGuiOverlay ui(view);
  ui.setPixelRatio(ratio);
  ui.setVisible(true);
  for (int frame = 0; frame < 3; ++frame)
    view.render();
  if (!ui.enabled())
    return -1;
  return ui.panelsOpen() ? 1 : 0;
}

} // namespace

int main() {
  const View views[] = {
      {1348, 577, 1.0f, "a 1366 x 768 laptop's browser canvas (1348 x 577)", true},
      {1582, 729, 1.0f, "a 1600 x 900 browser window's canvas (1582 x 729)", true},
      {1024, 768, 1.0f, "arigl's own window (1024 x 768)", true},
      {1920, 810, 1.25f, "a browser canvas on a 125% display (1536 x 648 logical)", true},
      {1170, 2400, 3.0f, "a phone upright (390 x 800 CSS px)", false},
      {2532, 1050, 3.0f, "a phone on its side (844 x 350 CSS px)", false},
  };
  for (const View &v : views) {
    const int open = panels_after_first_frames(v.w, v.h, v.ratio);
    if (open < 0) {
      const char *require = std::getenv("CVC_REQUIRE_RENDER");
      if (require && *require && std::string(require) != "0") {
        chk(false, "CVC_REQUIRE_RENDER is set, but the overlay could not draw");
        break;
      }
      std::printf("cvcgl_imgui_panels_start: SKIP (the overlay could not draw)\n");
      return 0;
    }
    chk((open == 1) == v.open,
        std::string(v.what) + (v.open ? ": the panels start open" : ": the panels start hidden"));
  }
  std::printf("cvcgl_imgui_panels_start: %s\n", fails ? "FAILED" : "PASSED");
  return fails ? 1 : 0;
}
