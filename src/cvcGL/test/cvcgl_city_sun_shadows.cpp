// A low afternoon sun's shadows over a city: StageLighting's key over a 3 km ground, the way
// cvcdbg's demo3 lights Austin (stage radius = the footprint's half-diagonal, the key 2.2 radii
// out), with one 50 m building at the centre and a 2 m box (a vehicle) 200 m east of it.
//
//   A. The whole city in one map: the ground in the building's shadow reads darker than the same
//      ground in the sun, and the ground beside the shadow does not. (VTK's exponential lookup
//      darkened it by a fifth of the key at this scale, so it read as lit.)
//   B. The map focused on the vehicle (SceneGraph::setShadowFocus): its shadow reads on the
//      ground beside it, the sunward ground is lit, and the building's shadow, now outside the
//      map, is gone rather than cut off.
//
// Renders for real; skips where nothing rasterises (fatal under CVC_REQUIRE_RENDER=1).
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/StageLighting.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkRenderer.h>

using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

static int fails = 0;
static void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

// A flat n x n grid of quads over [-h, h]^2 at z = 0.
static cvc::geometry ground(double h, int n) {
  cvc::geometry g;
  for (int j = 0; j <= n; ++j)
    for (int i = 0; i <= n; ++i)
      g.points().push_back({-h + 2.0 * h * i / n, -h + 2.0 * h * j / n, 0.0});
  using I = cvc::geometry::index_t;
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const I a = static_cast<I>(j * (n + 1) + i), b = a + 1, c = a + (n + 1), d = c + 1;
      g.tris().push_back({a, b, d});
      g.tris().push_back({a, d, c});
    }
  return g;
}

// An axis-aligned box [x0, x1] x [y0, y1] x [0, h], outward-wound.
static cvc::geometry box(double x0, double y0, double x1, double y1, double h) {
  cvc::geometry g;
  const double xs[2] = {x0, x1}, ys[2] = {y0, y1}, zs[2] = {0.0, h};
  for (int k = 0; k < 2; ++k)
    for (int j = 0; j < 2; ++j)
      for (int i = 0; i < 2; ++i)
        g.points().push_back({xs[i], ys[j], zs[k]});
  using I = cvc::geometry::index_t;
  const I f[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4},
                     {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
  for (const auto &q : f) {
    g.tris().push_back({q[0], q[1], q[2]});
    g.tris().push_back({q[0], q[2], q[3]});
  }
  return g;
}

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  const bool require =
      std::getenv("CVC_REQUIRE_RENDER") && std::string(std::getenv("CVC_REQUIRE_RENDER")) == "1";
  cvc::app app;
  SceneGraph sg(app, "city");
  sg.setGridVisible(false); // the diagnostic grid and axes would cast over the samples
  sg.setAxisVisible(false);
  const double half = 1500.0;
  auto g = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("ground", ground(half, 60)));
  auto b = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("building", box(-20.0, -20.0, 20.0, 20.0, 50.0)));
  auto v = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("vehicle", box(200.0, 0.0, 204.6, 2.2, 2.0)));
  for (auto &n : {g, b, v})
    if (n) {
      n->setUseSingleColor(true);
      n->setColor(0.5, 0.5, 0.5); // the sunlit ground well under full brightness
      n->setAmbient(0.3);
      n->setDiffuse(0.7);
    }

  // demo3's rig: the stage the footprint, the key low in the west-southwest.
  const double az = -60.0, el = 28.0;
  cvc::gl::StageLighting rig(sg);
  rig.setStage(0.0, 0.0, 25.0, std::sqrt(2.0) * half);
  rig.applyPreset(cvc::gl::StageLighting::Preset::ThreePoint);
  rig.setKey(1.3, az, el, 62.0);
  rig.setFill(0.3);
  rig.setEnvironment(0.5);
  rig.setBack(0.0);
  rig.setWash(0.0, 0, 1.0);
  rig.apply();

  SceneRenderer view(sg, 640, 640, true);
  if (!sg.setShadowsEnabled(true)) {
    std::printf("  SKIP  no shadow passes here\n");
    return require ? 1 : 0;
  }
  sg.setShadowResolution(4096);

  // The sun's horizontal heading (toward where its light goes), as StageLighting places the key:
  // at (sin az, -cos az) * cos el from the stage, so the light travels the other way.
  const double kDeg = 3.14159265358979323846 / 180.0;
  const double dx = -std::sin(az * kDeg), dy = std::cos(az * kDeg);
  const double run = 50.0 / std::tan(el * kDeg); // the building's shadow length on the ground

  // Looking straight down from 600 m at the building and the shadow it throws.
  auto look = [&](double cx, double cy, double h) {
    view.setCamera(cx, cy, h, cx, cy, 0.0, 0.0, 1.0, 0.0, 40.0);
    for (int i = 0; i < 3; ++i) {
      sg.processEvents();
      view.render();
    }
  };
  // CVCGL_SHADOW_DUMP=<dir>: each view's frame as a PPM there, to look at.
  auto dump = [&](const char *name) {
    const char *dir = std::getenv("CVCGL_SHADOW_DUMP");
    if (!dir)
      return;
    const std::vector<unsigned char> px = view.frameRGB();
    const int w = view.frameWidth(), h = view.frameHeight();
    if (px.size() != static_cast<std::size_t>(w) * h * 3)
      return;
    if (std::FILE *f = std::fopen((std::string(dir) + "/" + name + ".ppm").c_str(), "wb")) {
      std::fprintf(f, "P6\n%d %d\n255\n", w, h);
      for (int y = h - 1; y >= 0; --y) // frameRGB is bottom-up
        std::fwrite(px.data() + static_cast<std::size_t>(y) * w * 3, 1,
                    static_cast<std::size_t>(w) * 3, f);
      std::fclose(f);
    }
  };
  auto luma = [&](double x, double y) {
    const std::vector<unsigned char> px = view.frameRGB();
    const int w = view.frameWidth(), h = view.frameHeight();
    view.renderer()->SetWorldPoint(x, y, 0.0, 1.0);
    view.renderer()->WorldToDisplay();
    double d[3];
    view.renderer()->GetDisplayPoint(d);
    double s = 0.0;
    int n = 0;
    for (int yy = static_cast<int>(d[1]) - 2; yy <= static_cast<int>(d[1]) + 2; ++yy)
      for (int xx = static_cast<int>(d[0]) - 2; xx <= static_cast<int>(d[0]) + 2; ++xx)
        if (xx >= 0 && yy >= 0 && xx < w && yy < h &&
            px.size() == static_cast<std::size_t>(w) * h * 3) {
          const std::size_t o = (static_cast<std::size_t>(yy) * w + xx) * 3;
          s += 0.299 * px[o] + 0.587 * px[o + 1] + 0.114 * px[o + 2];
          ++n;
        }
    return n ? s / n : -1.0;
  };

  // A. One map over the whole city. The samples: in the building's shadow, on the sunward side
  // clear of the building's image, and beside the shadow.
  look(0.5 * run * dx, 0.5 * run * dy, 600.0);
  dump("a_city");
  const double shadeX = 0.6 * run * dx, shadeY = 0.6 * run * dy;
  const double shade = luma(shadeX, shadeY);
  const double sun = luma(-1.2 * run * dx, -1.2 * run * dy);
  const double side = luma(shadeX + 80.0 * dy, shadeY - 80.0 * dx);
  std::printf("  A building: shadow %.1f  sunward %.1f  beside the shadow %.1f\n", shade, sun,
              side);
  if (sun <= 1.0) {
    std::printf("  SKIP  nothing rasterised\n");
    return require ? 1 : 0;
  }
  chk(shade < 0.75 * sun,
      "A: the building's shadow on the ground is darker than the sunlit ground");
  chk(std::fabs(side - sun) < 0.06 * sun,
      "A: the ground beside the shadow is lit like the sunward side");

  // B. The map on the vehicle: 2 m tall, its shadow 3.8 m long beside it.
  const double vx = 202.3, vy = 1.1, vrun = 2.0 / std::tan(el * kDeg);
  sg.setShadowFocus(vx, vy, 1.0, 60.0);
  look(vx, vy, 40.0);
  dump("b_vehicle");
  const double vshade = luma(vx + (1.5 + 0.5 * vrun) * dx, vy + (1.5 + 0.5 * vrun) * dy);
  const double vsun = luma(vx - (3.0 + vrun) * dx, vy - (3.0 + vrun) * dy);
  std::printf("  B vehicle: shadow %.1f  sunward %.1f\n", vshade, vsun);
  chk(vshade < 0.75 * vsun,
      "B: a 2 m vehicle's shadow on the ground is darker than the sunlit ground");
  look(0.5 * run * dx, 0.5 * run * dy, 600.0);
  const double outside = luma(shadeX, shadeY);
  std::printf("  B building, outside the map: %.1f (sunward %.1f)\n", outside,
              luma(-1.2 * run * dx, -1.2 * run * dy));
  chk(std::fabs(outside - sun) < 0.06 * sun,
      "B: outside the focused map the building's shadow is gone, the ground lit");
  sg.clearShadowFocus();
  look(0.5 * run * dx, 0.5 * run * dy, 600.0);
  chk(luma(shadeX, shadeY) < 0.75 * sun, "B: clearing the focus brings the whole-city map back");

  std::printf("%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
  return fails ? 1 : 0;
}
