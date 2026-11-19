// Computed colours (docs/ARIADNE.md §5.26) in ImGui's pixels, the way a host runs a document:
// AriRuntime + ImGuiBackend over an offscreen SceneRenderer. Each check counts the frame's pixels
// of one colour:
//   - three grid cells filled from a palette held in state (`background: { color: { bind:
//     palette.{i} } }` under repeat:), each its whole cell; a palette entry changed in state
//     recolours its cell on the next frame;
//   - a text red past a threshold (`color: "(if ...)"`), and in the default colour under it;
//   - a button coloured from a key, and a radio coloured per option.
// Skips (rc 0) where nothing rasterises, unless CVC_REQUIRE_RENDER=1.
#include "streaming_test_util.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using cvcgl_test::check;

namespace {

constexpr int kWidth = 640, kHeight = 480;
constexpr double kDt = 1.0 / 30.0;

const char *kDoc = R"ari(meta:
  name: computed colours test
  min_libcvc: 4.0.0

windows:
  - window: Cells
    id: cells
    pos: [20, 20]
    size: [420, 120]
    layout: { kind: grid, col_widths: [120, 120, 120] }
    children:
      - type: group
        id: cell
        repeat: "3"
        background:
          color: { bind: "palette.{i}" }
        children:
          - text: "cell {i}"
          - text: " "
          - text: " "

  - window: Rows
    id: rows
    pos: [20, 200]
    size: [420, 200]
    children:
      - text: Temp
        bind: temp
        color: "(if (> (float (state-get \"temp\")) 80) \"1,0,0\" \"\")"
      - button: Go
        on: go
        color: { bind: go.color }
      - radio_row: Pick
        bind: pick
        options: [orange, violet]
        color: "#ff8000;#8000ff"
)ari";

// Pixels within `tol` of (r, g, b) in an RGB frame.
long count(const std::vector<unsigned char> &rgb, int r, int g, int b, int tol) {
  long n = 0;
  for (std::size_t i = 0; i + 2 < rgb.size(); i += 3)
    if (std::abs(rgb[i] - r) <= tol && std::abs(rgb[i + 1] - g) <= tol &&
        std::abs(rgb[i + 2] - b) <= tol)
      ++n;
  return n;
}

// Within the box around every pixel near (r, g, b) (a filled cell), the pixels whose channels are
// all below `dark` (dark text) or all above `light` (light text).
struct InBox {
  long dark = 0, light = 0;
};
InBox in_box(const std::vector<unsigned char> &rgb, int w, int r, int g, int b) {
  const int h = static_cast<int>(rgb.size() / 3) / w;
  int x0 = w, y0 = h, x1 = -1, y1 = -1;
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const unsigned char *p = &rgb[(static_cast<std::size_t>(y) * w + x) * 3];
      if (std::abs(p[0] - r) <= 2 && std::abs(p[1] - g) <= 2 && std::abs(p[2] - b) <= 2) {
        x0 = std::min(x0, x);
        y0 = std::min(y0, y);
        x1 = std::max(x1, x);
        y1 = std::max(y1, y);
      }
    }
  InBox n;
  for (int y = y0; y <= y1; ++y)
    for (int x = x0; x <= x1; ++x) {
      const unsigned char *p = &rgb[(static_cast<std::size_t>(y) * w + x) * 3];
      if (p[0] < 60 && p[1] < 60 && p[2] < 60)
        ++n.dark;
      else if (p[0] > 200 && p[1] > 200 && p[2] > 200)
        ++n.light;
    }
  return n;
}

} // namespace

int main() {
  const bool require = std::getenv("CVC_REQUIRE_RENDER") != nullptr;
  const fs::path dir = fs::temp_directory_path() / "cvcgl_imgui_colors";
  fs::create_directories(dir);
  const fs::path doc = dir / "colors.ari";
  std::ofstream(doc) << kDoc;

  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::SceneGraph sg(app, "colors");
  sg.setDiagnosticChromeVisible(false);
  cvc::gl::SceneRenderer view(sg, kWidth, kHeight, /*offscreen=*/true, "main");
  view.setBackground(0.0, 0.0, 0.0);
  cvc::gl::CameraController cam(view);
  cvc::gl::ImGuiOverlay ui(view);
  if (!ui.enabled()) {
    std::printf("SKIP: no ImGui overlay\n");
    return require ? 1 : 0;
  }
  ui.attachCamera(cam);
  ui.setVisible(true);
  ui.setTouchMode(false);
  ui.setUiScale(1.0f);
  ui.setToggleButtonEnabled(false);
  ui.setPanelsOpen(true);
  ui.setFixedDeltaTime(static_cast<float>(kDt));

  cvc::gl::ariadne::AriRuntime rt(view, cam, ui);
  cvc::state &st = cvc::state::instance(app);
  const auto key = [&](const std::string &k) -> cvc::state & { return st(rt.prefix() + "." + k); };
  key("palette.0").value("1,0,1"); // magenta
  key("palette.1").value("#00ffff");
  key("palette.2").value("1 1 0"); // yellow
  key("temp").value("50");
  key("go.color").value("0,1,0");
  for (const std::string &w : rt.load(doc.string()))
    check(false, "the document loads without warnings", w);
  for (int f = 0; f < 6; ++f)
    rt.step(kDt);
  if (!ui.enabled()) { // ImGui's GL backend could not start: nothing rasterises here
    std::printf("SKIP: nothing rasterises\n");
    return require ? 1 : 0;
  }
  const auto frame = [&] {
    rt.step(kDt);
    return view.frameRGB();
  };

  std::vector<unsigned char> px = frame();
  if (const char *png = std::getenv("CVC_COLORS_PNG")) // the frame, to look at
    view.writePNG(png);
  const long magenta = count(px, 255, 0, 255, 2), cyan = count(px, 0, 255, 255, 2),
             yellow = count(px, 255, 255, 0, 2);
  std::printf("  cells: magenta %ld cyan %ld yellow %ld\n", magenta, cyan, yellow);
  // A cell is about 120 x 60 px: its fill covers the whole cell, not only its text.
  check(magenta > 3000 && cyan > 3000 && yellow > 3000, "each grid cell is filled from its key",
        std::to_string(magenta) + " " + std::to_string(cyan) + " " + std::to_string(yellow));
  // The text on a fill is black or white, whichever reads on it.
  const InBox on_yellow = in_box(px, view.frameWidth(), 255, 255, 0);
  const InBox on_magenta = in_box(px, view.frameWidth(), 255, 0, 255);
  check(on_yellow.dark > 15 && on_yellow.light == 0, "the light cell's text is black",
        std::to_string(on_yellow.dark) + " " + std::to_string(on_yellow.light));
  check(on_magenta.light > 15 && on_magenta.dark == 0, "the dark cell's text is white",
        std::to_string(on_magenta.light) + " " + std::to_string(on_magenta.dark));
  const long red_cool = count(px, 255, 0, 0, 40);
  check(red_cool < 3, "under the threshold the text is in the default colour",
        std::to_string(red_cool));
  const long green = count(px, 0, 255, 0, 40);
  check(green > 15, "the button's text is in its key's colour", std::to_string(green));
  const long orange = count(px, 255, 128, 0, 40), violet = count(px, 128, 0, 255, 40);
  check(orange > 15 && violet > 15, "each radio option is in its own colour",
        std::to_string(orange) + " " + std::to_string(violet));

  // Past the threshold the text turns red; a palette entry changed recolours its cell.
  key("temp").value("95");
  key("palette.0").value("0,0,1");
  frame();
  px = frame();
  const long red_hot = count(px, 255, 0, 0, 40);
  check(red_hot > 15, "past the threshold the text is red", std::to_string(red_hot));
  const long blue = count(px, 0, 0, 255, 2), magenta_after = count(px, 255, 0, 255, 2);
  check(blue > 3000 && magenta_after < 3, "the changed palette entry recolours its cell",
        std::to_string(blue) + " " + std::to_string(magenta_after));
  check(rt.app_runtime().runtime().take_reactive_warnings().empty(), "no reactive warnings");
  return cvcgl_test::finish("cvcgl_imgui_colors");
}
