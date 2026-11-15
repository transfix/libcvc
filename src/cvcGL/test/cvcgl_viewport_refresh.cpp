// cvcgl_viewport_refresh — Viewport::setRefreshHz: a throttled inset draws its scene at most hz
// times a second and shows its last pixels in between (docs/ARIADNE.md, "Refresh rate").
//
// What is pinned here, with and without a multisampled window:
//   * at 0 Hz (the default) the inset draws in every render;
//   * at 2 Hz a burst of renders draws it no more often than the rate allows, while the base view
//     under it draws every time;
//   * the renders in between show its last pixels, byte for byte, not the base view under it and
//     not a picture of what changed since (a new background shows only once it draws);
//   * invalidate(), a new place on screen, being shown again and the time passing each draw it;
//   * a mirror is not re-synced while it shows its last pixels, and is on the render it draws;
//   * a translucent inset draws every render, whatever the rate; 0 Hz draws every render again;
//   * removing a throttled viewport leaves the window rendering.
//
// Offscreen; skips where nothing rasterises unless CVC_REQUIRE_RENDER=1.

#include "streaming_test_util.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/Viewport.h>
#include <cvc/gl/ViewportManager.h>
#include <string>
#include <thread>
#include <vector>
#include <vtkPropCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

using cvc::gl::SceneGraph;
using cvc::gl::Viewport;
using cvc::gl::ViewportManager;
using cvcgl_test::check;

namespace {

constexpr int kW = 320, kH = 240;

void addQuad(SceneGraph &sg, const std::string &name, double h, double z) {
  cvc::geometry g;
  const double xs[4] = {-h, h, h, -h};
  const double ys[4] = {-h, -h, h, h};
  for (int i = 0; i < 4; ++i) {
    cvc::geometry::point_t p;
    p[0] = xs[i];
    p[1] = ys[i];
    p[2] = z;
    g.points().push_back(p);
  }
  for (const auto &tri : {std::array<int, 3>{0, 1, 2}, std::array<int, 3>{0, 2, 3}}) {
    cvc::geometry::tri_t t;
    t[0] = tri[0];
    t[1] = tri[1];
    t[2] = tri[2];
    g.tris().push_back(t);
  }
  sg.addGraphics(name, g);
}

struct Rect {
  int x0, y0, x1, y1; // pixels from the lower left, as frameRGB's rows
};

// The pixels of a normalized region, inset by 3 px so no edge pixel is read.
Rect rectOf(const double r[4]) {
  return {static_cast<int>(std::ceil(r[0] * kW)) + 3, static_cast<int>(std::ceil(r[1] * kH)) + 3,
          static_cast<int>(std::floor(r[2] * kW)) - 3, static_cast<int>(std::floor(r[3] * kH)) - 3};
}

struct Rgb {
  double r = 0, g = 0, b = 0;
};

Rgb mean(const std::vector<unsigned char> &f, const Rect &q) {
  Rgb c;
  long n = 0;
  for (int y = q.y0; y < q.y1; ++y)
    for (int x = q.x0; x < q.x1; ++x) {
      const size_t i = (static_cast<size_t>(y) * kW + x) * 3;
      c.r += f[i];
      c.g += f[i + 1];
      c.b += f[i + 2];
      ++n;
    }
  if (n) {
    c.r /= 255.0 * n;
    c.g /= 255.0 * n;
    c.b /= 255.0 * n;
  }
  return c;
}

bool same(const std::vector<unsigned char> &a, const std::vector<unsigned char> &b, const Rect &q) {
  if (a.size() != b.size())
    return false;
  for (int y = q.y0; y < q.y1; ++y)
    for (int x = q.x0; x < q.x1; ++x)
      for (int c = 0; c < 3; ++c) {
        const size_t i = (static_cast<size_t>(y) * kW + x) * 3 + c;
        if (a[i] != b[i])
          return false;
      }
  return true;
}

std::string rgb(const Rgb &c) {
  char b[64];
  std::snprintf(b, sizeof b, "(%.2f, %.2f, %.2f)", c.r, c.g, c.b);
  return b;
}

long props(Viewport &v) { return v.renderer()->GetViewProps()->GetNumberOfItems(); }

void run(cvc::app &app, int samples) {
  const std::string tag = " [" + std::to_string(samples) + " samples]";
  SceneGraph scene(app, "refresh_" + std::to_string(samples));
  scene.setDiagnosticChromeVisible(false);
  addQuad(scene, "ground", 50.0, 0.0);
  ViewportManager vm(scene, kW, kH, /*offscreen=*/true);
  vm.renderWindow()->SetMultiSamples(samples);
  vm.primary().setBackground(0.70, 0.10, 0.10); // a red base under the inset
  const double at[4] = {0.55, 0.55, 0.95, 0.95};
  Viewport &mini = vm.addMirrorViewport("mini", "main", at, /*layer=*/1);
  mini.setBackground(0.10, 0.10, 0.80); // a blue inset
  vm.render();
  vm.primary().renderer()->ResetCamera();
  mini.renderer()->ResetCamera();
  const Rect in = rectOf(at);
  std::printf("  window multisamples %d%s\n", vm.renderWindow()->GetMultiSamples(), tag.c_str());

  // 0 Hz: every render draws it.
  long d = mini.draws();
  for (int i = 0; i < 5; ++i)
    vm.frameRGB();
  check(mini.draws() == d + 5, "at 0 Hz the inset draws in every render" + tag,
        std::to_string(mini.draws() - d));

  // 2 Hz: the first render draws it and keeps its pixels; a burst after shows them.
  mini.setRefreshHz(2.0);
  check(mini.refreshHz() == 2.0, "the rate reads back" + tag);
  d = mini.draws();
  const std::vector<unsigned char> drawn = vm.frameRGB();
  check(mini.draws() == d + 1, "setting a rate draws it at the next render" + tag);
  const Rgb blue = mean(drawn, in);
  check(blue.b > blue.r, "the inset drew blue over the red base" + tag, rgb(blue));
  d = mini.draws();
  const long base = vm.primary().draws();
  const auto t0 = std::chrono::steady_clock::now();
  bool kept = true;
  for (int i = 0; i < 10; ++i) {
    const long before = mini.draws();
    const std::vector<unsigned char> f = vm.frameRGB();
    if (mini.draws() == before)
      kept = kept && same(f, drawn, in);
  }
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  const long burst = mini.draws() - d;
  check(burst <= 1 + static_cast<long>(secs * 2.0),
        "a burst draws it no more than 2 Hz allows" + tag,
        std::to_string(burst) + " draws in " + std::to_string(secs) + " s");
  check(burst < 10, "and fewer times than it rendered" + tag, std::to_string(burst));
  check(vm.primary().draws() - base == 10, "while the base view drew every time" + tag);
  check(kept, "the renders in between show its last pixels, byte for byte" + tag);

  // What changed since shows only once it draws.
  mini.setBackground(0.10, 0.80, 0.10); // green
  d = mini.draws();
  std::vector<unsigned char> f = vm.frameRGB();
  if (mini.draws() == d)
    check(same(f, drawn, in), "a new background does not show before it draws" + tag);
  mini.invalidate();
  d = mini.draws();
  f = vm.frameRGB();
  const Rgb green = mean(f, in);
  check(mini.draws() == d + 1 && green.g > green.b, "invalidate() draws it, green now" + tag,
        rgb(green));

  // A new place on screen draws it there; the old place shows the base.
  const double moved[4] = {0.05, 0.55, 0.45, 0.95};
  mini.setRegion(moved[0], moved[1], moved[2], moved[3]);
  d = mini.draws();
  f = vm.frameRGB();
  const Rgb there = mean(f, rectOf(moved)), left = mean(f, in);
  check(mini.draws() == d + 1 && there.g > there.r, "a move draws it at its new place" + tag,
        rgb(there));
  check(left.r > left.g, "and its old place shows the base" + tag, rgb(left));

  // The time passing draws it.
  d = mini.draws();
  vm.frameRGB();
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  vm.frameRGB();
  check(mini.draws() >= d + 1, "half a second later it draws again" + tag);

  // Hidden it draws nothing and shows nothing; shown again it draws.
  mini.setVisible(false);
  d = mini.draws();
  f = vm.frameRGB();
  const Rgb hidden = mean(f, rectOf(moved));
  check(mini.draws() == d && hidden.r > hidden.g, "hidden, the base shows where it was" + tag,
        rgb(hidden));
  mini.setVisible(true);
  vm.frameRGB();
  check(mini.draws() == d + 1, "shown again, it draws" + tag);

  // A mirror is not re-synced while it shows its last pixels, and is when it draws.
  const long n0 = props(mini);
  addQuad(scene, "marker", 10.0, 1.0);
  d = mini.draws();
  vm.frameRGB();
  if (mini.draws() == d)
    check(props(mini) == n0, "showing its last pixels, the mirror is not re-synced" + tag,
          std::to_string(props(mini)) + " props, were " + std::to_string(n0));
  mini.invalidate();
  vm.frameRGB();
  check(props(mini) == n0 + 1, "drawing, it picks up the new node" + tag,
        std::to_string(props(mini)));

  // Translucent: every render draws it, whatever the rate.
  mini.setBackground(0.10, 0.80, 0.10, /*opaque=*/false);
  vm.frameRGB();
  d = mini.draws();
  for (int i = 0; i < 3; ++i)
    vm.frameRGB();
  check(mini.draws() == d + 3, "a translucent inset draws every render" + tag);
  mini.setBackground(0.10, 0.80, 0.10, /*opaque=*/true);

  // 0 Hz again: every render.
  mini.setRefreshHz(0.0);
  d = mini.draws();
  for (int i = 0; i < 3; ++i)
    vm.frameRGB();
  check(mini.draws() == d + 3, "back at 0 Hz it draws every render" + tag);

  // Removed while throttled: the window keeps rendering.
  mini.setRefreshHz(2.0);
  vm.frameRGB();
  vm.frameRGB();
  vm.removeViewport("mini");
  f = vm.frameRGB();
  const Rgb gone = mean(f, rectOf(moved));
  check(gone.r > gone.g, "removed, the base shows where it was" + tag, rgb(gone));
}

} // namespace

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  if (!cvcgl_test::renderAvailable(app))
    return cvcgl_test::finish("cvcgl_viewport_refresh");
  run(app, 0);
  run(app, 4);
  return cvcgl_test::finish("cvcgl_viewport_refresh");
}
