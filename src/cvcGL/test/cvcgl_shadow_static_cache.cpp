// The shadow bake's static casters' cache (SceneGraph::setShadowStaticCache): while only a few
// casters move, a bake draws just those, over a copy of maps the static casters were drawn into.
//
// cvcdbg's demo3 (Humvees on a city) missed refreshes in Firefox on every frame a vehicle moved:
// each such frame re-baked the shadow map, which re-drew every building and the terrain, and that
// was nearly all of the frame's render. Baking only every Nth frame instead left a moving
// vehicle's shadow behind it, and the vehicle drove into its own stale shadow.
//
// Two scenes alike in every way, one with the cache on and one with it off (every change re-bakes
// every caster). Each frame both move the same things; each frame's pictures must be the same.
// Pins:
//   A. a vehicle drives through a building's shadow: every frame the cached scene's picture is
//      the reference's, byte for byte -- so its shadow follows it in the frame it moves, and the
//      building still shades it -- and the picture is not the previous frame's (the shadow did
//      move); after the first bakes every bake draws only the vehicle, over only the texels it
//      covers and covered (shadowBakeStats);
//   B. a static caster moving, going and coming back, a light turning, the focus moving, a
//      caster joining, the moving one going and coming back: the same pictures; a whole bake
//      (the static maps re-made) for each change to the static casters, the light, the focus or
//      the bounds, and the moving casters alone for the rest;
//   C. moving casters that stop: once most have held still a while, a whole bake makes them
//      static, and the one still moving is drawn alone;
//   D. the cache off: every bake is whole, nothing is cached;
//   E. interval 2: the vehicle's bakes stride as before, cached or not.
// On Apple's GL (frames a few bytes apart from identical GL calls with shadows on, see
// cvcgl_shadow_casters) the pictures must agree within a small step instead. Renders for real and
// skips where nothing rasterises (fatal under CVC_REQUIRE_RENDER=1).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>
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

static cvc::geometry box(double x0, double y0, double z0, double x1, double y1, double z1) {
  cvc::geometry g;
  const double v[8][3] = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
                          {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
  for (const auto &p : v)
    g.points().push_back({p[0], p[1], p[2]});
  const int f[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                        {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
  for (const auto &t : f)
    g.tris().push_back({static_cast<unsigned int>(t[0]), static_cast<unsigned int>(t[1]),
                        static_cast<unsigned int>(t[2])});
  return g;
}

// A flat, upward-facing ground at height z in step x step tiles (as cvcgl_shadow_casters').
static cvc::geometry tiledGround(double x0, double y0, double x1, double y1, double z,
                                 double step) {
  cvc::geometry g;
  const int nx = static_cast<int>(std::lround((x1 - x0) / step));
  const int ny = static_cast<int>(std::lround((y1 - y0) / step));
  for (int j = 0; j <= ny; ++j)
    for (int i = 0; i <= nx; ++i)
      g.points().push_back({x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny, z});
  auto at = [nx](int i, int j) { return static_cast<unsigned int>(j * (nx + 1) + i); };
  for (int j = 0; j < ny; ++j)
    for (int i = 0; i < nx; ++i) {
      g.tris().push_back({at(i, j), at(i + 1, j), at(i + 1, j + 1)});
      g.tris().push_back({at(i, j), at(i + 1, j + 1), at(i, j + 1)});
    }
  return g;
}

static bool canRasterise(cvc::app &app) {
  SceneGraph sg(app, "cachecontrol");
  sg.setDiagnosticChromeVisible(false);
  auto g = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("control");
  g->setGeometry(box(-1, -1, -1, 1, 1, 1));
  SceneRenderer sr(sg, 32, 32, /*offscreen=*/true);
  sr.setBackground(0.0, 0.0, 0.0);
  sr.setCamera(4, 4, 4, 0, 0, 0, 0, 0, 1, 40.0, 0.5, 100.0);
  for (unsigned char c : sr.frameRGB())
    if (c > 20)
      return true;
  return false;
}

static std::string glRendererName(vtkRenderWindow *w) {
  auto *gl = vtkOpenGLRenderWindow::SafeDownCast(w);
  const char *caps = gl ? gl->ReportCapabilities() : nullptr;
  const std::string all = caps ? caps : "";
  const std::string key = "renderer string:";
  const std::size_t at = all.find(key);
  if (at == std::string::npos)
    return "unknown";
  std::size_t b = all.find_first_not_of(' ', at + key.size());
  const std::size_t e = all.find('\n', at);
  if (b == std::string::npos || b > e)
    b = e;
  return all.substr(b, e == std::string::npos ? std::string::npos : e - b);
}

// How two frames differ: bytes, and the largest step. bytes is -1 for frames of different sizes.
struct Diff {
  long bytes = -1;
  int step = 0;
};
static Diff diff(const std::vector<unsigned char> &a, const std::vector<unsigned char> &b) {
  Diff d;
  if (a.size() != b.size() || a.empty())
    return d;
  d.bytes = 0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const int s = std::abs(static_cast<int>(a[i]) - static_cast<int>(b[i]));
    if (s) {
      ++d.bytes;
      d.step = std::max(d.step, s);
    }
  }
  return d;
}

constexpr int kW = 128, kH = 128;

// Mean luma of the ground square [x0, x1] x [y0, y1] (world units at z = 0) in a frame of the
// top-down view: the camera is 30 up over the origin with a 30 degree view, +y up the frame.
static double lumaAt(const std::vector<unsigned char> &rgb, double x0, double y0, double x1,
                     double y1) {
  const double half = 30.0 * std::tan(15.0 * 3.14159265358979323846 / 180.0);
  auto px = [&](double x) { return static_cast<int>((x + half) / (2.0 * half) * kW); };
  auto py = [&](double y) { return static_cast<int>((y + half) / (2.0 * half) * kH); };
  double sum = 0.0;
  long n = 0;
  for (int y = std::max(0, py(y0)); y < std::min(kH, py(y1)); ++y)
    for (int x = std::max(0, px(x0)); x < std::min(kW, px(x1)); ++x) {
      const std::size_t o = (static_cast<std::size_t>(y) * kW + x) * 3;
      sum += 0.299 * rgb[o] + 0.587 * rgb[o + 1] + 0.114 * rgb[o + 2];
      ++n;
    }
  return n ? sum / n : -1.0;
}

// One of the two scenes: a white tiled ground, two buildings, a vehicle and four parked cars, all
// casting, under a sun from +x (so shadows fall toward -x) and a fill, seen from straight above.
struct World {
  SceneGraph sg;
  std::shared_ptr<GeometryNode> ground, tower, block, vehicle;
  std::vector<std::shared_ptr<GeometryNode>> cars;
  std::unique_ptr<SceneRenderer> view;
  int sun = -1;

  World(cvc::app &app, const std::string &name, bool cache) : sg(app, name) {
    sg.setDiagnosticChromeVisible(false);
    ground = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("ground");
    ground->setGeometry(tiledGround(-40, -40, 40, 40, 0.0, 2.0));
    ground->setColor(1.0, 1.0, 1.0);
    // The tower's shadow lies over the vehicle's road (y in [2, 4]) for x in [-2, 4]: at the
    // vehicle's roof (z 1.6), x in [-0.4, 4].
    tower = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("tower");
    tower->setGeometry(box(4, 1, 0, 6, 5, 6));
    block = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("block");
    block->setGeometry(box(-6, -7, 0, -3, -4, 3));
    // Off the ground by a tenth, so no face of it lies in the ground's plane (a tie in depth
    // could fall either way in two scenes that draw in different orders).
    vehicle = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("vehicle");
    vehicle->setGeometry(box(-1, 2, 0.1, 1, 4, 1.6));
    vehicle->setColor(0.9, 0.6, 0.3);
    for (int i = 0; i < 4; ++i) {
      auto c = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("car" + std::to_string(i));
      c->setGeometry(box(-7 + 3.5 * i, -2.5, 0.1, -5.5 + 3.5 * i, -1.5, 1.2));
      cars.push_back(c);
    }
    sg.processEvents();
    view = std::make_unique<SceneRenderer>(sg, kW, kH, /*offscreen=*/true, "main");
    view->setBackground(0.0, 0.0, 1.0);
    view->setCamera(0, 0, 30, 0, 0, 0, 0, 1, 0, 30.0, 1.0, 200.0);
    sun = sg.addDirectionalLight(90.0, 45.0); // from +x
    // A dim fill from above that casts nothing, so what lies in shadow still shows: a vehicle
    // moving inside the tower's shadow changes the picture.
    sg.addFillLight(0, 0, 40, 0, 0, 0, 1.0, 1.0, 1.0, 0.3);
    sg.setShadowStaticCache(cache);
    sg.setShadowsEnabled(true);
    sg.setShadowResolution(1024);
    sg.setShadowUpdateInterval(1);
  }
  std::vector<unsigned char> frame() {
    sg.processEvents();
    return view->frameRGB(); // renders the frame, then reads it back
  }
};

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  if (!canRasterise(app)) {
    const char *require = std::getenv("CVC_REQUIRE_RENDER");
    if (require && *require && std::string(require) != "0") {
      std::printf("CVC_REQUIRE_RENDER is set, but this build did not rasterise\n");
      return 1;
    }
    std::printf("skipped: this build did not rasterise\n");
    return 0;
  }

  World cached(app, "cached", true), ref(app, "reference", false);
  if (!cached.sg.shadowsEnabled() || !ref.sg.shadowsEnabled()) {
    std::printf("skipped: shadows unavailable\n");
    return 0;
  }
  const std::string renderer = glRendererName(cached.view->renderer()->GetRenderWindow());
#ifdef __APPLE__
  const bool appleGL = renderer.find("Apple") != std::string::npos;
#else
  const bool appleGL = false;
#endif
  std::printf("GL renderer: %s%s\n", renderer.c_str(),
              appleGL ? " (Apple's GL: a step of a few levels allowed)" : "");
  const auto same = [&](const Diff &d) { return d.bytes == 0 || (appleGL && d.step <= 2); };
  // Both scenes' next frames, which must be the same picture.
  long worst = 0;
  int worstStep = 0;
  const auto both = [&](std::vector<unsigned char> *out = nullptr) {
    const std::vector<unsigned char> a = cached.frame(), b = ref.frame();
    const Diff d = diff(a, b);
    worst = std::max(worst, d.bytes < 0 ? 1L << 30 : d.bytes);
    worstStep = std::max(worstStep, d.step);
    if (out)
      *out = a;
    return same(d);
  };
  for (int i = 0; i < 3; ++i)
    both();

  std::printf("A. a vehicle drives through a building's shadow\n");
  int sameFrames = 0, movedFrames = 0, darkFrames = 0, shadedRoof = 0;
  const int kSteps = 38;
  std::vector<unsigned char> prev, now;
  both(&prev);
  cvc::gl::SceneGraph::shadow_bake_stats s0 = cached.sg.shadowBakeStats();
  // Lit ground no shadow ever reaches (a strip along the top of the view), and the vehicle's roof
  // lit, out in the open.
  const double open = lumaAt(prev, -7.5, 6.0, 7.5, 7.5);
  double roofLit = -1.0;
  for (int i = 1; i <= kSteps; ++i) {
    const double x = -7.0 + 0.25 * i; // -6.75 .. 2.5: into the tower's shadow, not into the tower
    cached.vehicle->setPosition(x, 0.0, 0.0);
    ref.vehicle->setPosition(x, 0.0, 0.0);
    sameFrames += both(&now) ? 1 : 0;
    if (diff(now, prev).bytes > 0)
      ++movedFrames;
    else
      std::printf("  frame %d (x %.2f): the picture of the frame before\n", i, x);
    // Its shadow, toward -x of it (x - 2.6 .. x - 1 on the ground), in the frame it moved to: where
    // that is in view and out of the tower's shadow (x < -2 there).
    if (x - 2.2 > -7.8 && x - 1.2 < -2.2) {
      const double dark = lumaAt(now, x - 2.2, 2.4, x - 1.2, 3.6);
      darkFrames += (dark >= 0.0 && dark < 0.6 * open) ? 1 : 0;
    } else {
      ++darkFrames;
    }
    // Its roof (x - 1 .. x + 1, at 1.6): lit in the open, and in the tower's shadow once all of
    // the sample is past x = -0.4.
    const double roof = lumaAt(now, x - 0.5, 2.6, x + 0.5, 3.4);
    if (i == 12) // x = -4: the open road
      roofLit = roof;
    if (x - 0.5 > -0.2)
      shadedRoof += (roof >= 0.0 && roofLit > 0.0 && roof < 0.6 * roofLit) ? 1 : 0;
    else
      ++shadedRoof;
    if (i == 3)
      s0 = cached.sg.shadowBakeStats(); // past the first bakes: the vehicle sorted moving
    prev = now;
  }
  std::printf("  open ground luma %.1f, the vehicle's roof lit %.1f\n", open, roofLit);
  const cvc::gl::SceneGraph::shadow_bake_stats s1 = cached.sg.shadowBakeStats();
  chk(sameFrames == kSteps, std::to_string(sameFrames) + " of " + std::to_string(kSteps) +
                                " frames the reference's picture (worst " + std::to_string(worst) +
                                " bytes apart, step " + std::to_string(worstStep) + ")");
  chk(movedFrames == kSteps, "and each frame's picture is new: " + std::to_string(movedFrames) +
                                 " of " + std::to_string(kSteps) + " differ from the frame before");
  chk(darkFrames == kSteps, "its shadow is on the ground beside it in the frame it moved to (" +
                                std::to_string(darkFrames) + " of " + std::to_string(kSteps) + ")");
  chk(shadedRoof == kSteps, "and the tower's shadow is on its roof under it (" +
                                std::to_string(shadedRoof) + " of " + std::to_string(kSteps) + ")");
  chk(s1.bakes - s0.bakes == static_cast<std::uint64_t>(kSteps - 3) && s1.whole == s0.whole,
      "after the first bakes, one bake a frame and none whole: " +
          std::to_string(s1.bakes - s0.bakes) + " bakes, " + std::to_string(s1.whole - s0.whole) +
          " whole");
  chk(s1.map_share > 0.0 && s1.map_share < 0.02,
      "and only where it is and was: " + std::to_string(100.0 * s1.map_share) +
          "% of the map copied and drawn into");
  chk(s1.moving_casters == 1 && s1.static_casters >= 7,
      "the vehicle drawn over the static casters, the 7 or more (" +
          std::to_string(s1.moving_casters) + " moving, " + std::to_string(s1.static_casters) +
          " static)");
  const cvc::gl::SceneGraph::shadow_bake_stats r1 = ref.sg.shadowBakeStats();
  chk(r1.whole == r1.bakes && r1.static_casters == 0 && r1.moving_casters == 0,
      "the reference bakes whole, caching nothing (" + std::to_string(r1.whole) + " of " +
          std::to_string(r1.bakes) + ")");

  std::printf("B. the static casters' maps re-made\n");
  const auto step = [&](const std::string &what, bool wholeBake, auto &&change) {
    change(cached);
    change(ref);
    const cvc::gl::SceneGraph::shadow_bake_stats b = cached.sg.shadowBakeStats();
    const bool ok = both();
    const cvc::gl::SceneGraph::shadow_bake_stats a = cached.sg.shadowBakeStats();
    chk(ok && a.bakes == b.bakes + 1 && (a.whole == b.whole + 1) == wholeBake,
        what + ": the reference's picture, " + (wholeBake ? "a whole bake" : "the moving casters") +
            " (" + std::to_string(a.bakes - b.bakes) + " bakes, " +
            std::to_string(a.whole - b.whole) + " whole)");
  };
  step("a building moved", true, [](World &w) { w.block->setPosition(0.5, 0.0, 0.0); });
  step("the vehicle again", false, [](World &w) { w.vehicle->setPosition(2.5, 0.0, 0.0); });
  step("a building hidden", true, [](World &w) { w.tower->setVisible(false); });
  // Back in view it reaches above everything else, past the bounds the light's camera was
  // fitted to without it: a whole bake.
  step("and shown", true, [](World &w) { w.tower->setVisible(true); });
  step("the vehicle hidden", false, [](World &w) { w.vehicle->setVisible(false); });
  step("and shown", false, [](World &w) { w.vehicle->setVisible(true); });
  step("a car hidden", true, [](World &w) { w.cars[0]->setVisible(false); });
  // Back in view it is new to the casters and inside the bounds: drawn with the moving ones,
  // over the static maps as they are.
  step("and shown", false, [](World &w) { w.cars[0]->setVisible(true); });
  step("the sun turned", true, [](World &w) { w.sg.setLightDirection(w.sun, 80.0, 50.0); });
  step("the vehicle again", false, [](World &w) { w.vehicle->setPosition(2.0, 0.0, 0.0); });
  step("a caster added", false, [](World &w) {
    auto c = w.sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("late");
    c->setGeometry(box(-2, -6, 0.1, 0, -5, 2));
    w.cars.push_back(c);
  });
  step("it moves with the vehicle", false, [](World &w) {
    w.vehicle->setPosition(1.5, 0.0, 0.0);
    w.cars.back()->setPosition(0.5, 0.0, 0.0);
  });
  step("a shadow focus", true, [](World &w) { w.sg.setShadowFocus(0.0, 0.0, 0.0, 12.0); });
  step("the vehicle under it", false, [](World &w) { w.vehicle->setPosition(1.0, 0.0, 0.0); });
  step("the focus moved", true, [](World &w) { w.sg.setShadowFocus(2.0, 1.0, 0.0, 12.0); });
  step("focus off", true, [](World &w) { w.sg.clearShadowFocus(); });

  std::printf("C. moving casters that stop\n");
  // The vehicle and the four cars all drive for a while, then the cars park and the vehicle goes
  // on: once the cars have held still long enough, a whole bake makes them static.
  for (int i = 0; i < 6; ++i) {
    for (World *w : {&cached, &ref}) {
      w->vehicle->setPosition(1.0 - 0.1 * i, 0.0, 0.0);
      for (std::size_t k = 0; k < w->cars.size(); ++k)
        w->cars[k]->setPosition(0.1 * i, 0.05 * static_cast<double>(k), 0.0);
    }
    both();
  }
  const std::size_t driving = cached.sg.shadowBakeStats().moving_casters;
  int parkedWhole = -1, sameC = 0;
  const cvc::gl::SceneGraph::shadow_bake_stats c0 = cached.sg.shadowBakeStats();
  for (int i = 0; i < 80; ++i) {
    const cvc::gl::SceneGraph::shadow_bake_stats b = cached.sg.shadowBakeStats();
    cached.vehicle->setPosition(0.4 - 0.05 * i, 0.0, 0.0);
    ref.vehicle->setPosition(0.4 - 0.05 * i, 0.0, 0.0);
    sameC += both() ? 1 : 0;
    if (parkedWhole < 0 && cached.sg.shadowBakeStats().whole > b.whole)
      parkedWhole = i;
  }
  const cvc::gl::SceneGraph::shadow_bake_stats c1 = cached.sg.shadowBakeStats();
  chk(driving >= 6,
      "6 casters or more drawn moving while they all drive (" + std::to_string(driving) + ")");
  chk(parkedWhole >= 50 && parkedWhole < 70 && c1.whole == c0.whole + 1,
      "one whole bake once the parked ones have held still a while (frame " +
          std::to_string(parkedWhole) + ", " + std::to_string(c1.whole - c0.whole) + " whole)");
  chk(c1.moving_casters == 1,
      "then the vehicle is drawn alone (" + std::to_string(c1.moving_casters) + " moving)");
  chk(sameC == 80, "every frame the reference's picture (" + std::to_string(sameC) + " of 80)");

  std::printf("D. the cache off\n");
  cached.sg.setShadowStaticCache(false);
  const cvc::gl::SceneGraph::shadow_bake_stats d0 = cached.sg.shadowBakeStats();
  int sameD = 0;
  for (int i = 0; i < 5; ++i) {
    cached.vehicle->setPosition(-3.0 + 0.3 * i, 0.0, 0.0);
    ref.vehicle->setPosition(-3.0 + 0.3 * i, 0.0, 0.0);
    sameD += both() ? 1 : 0;
  }
  const cvc::gl::SceneGraph::shadow_bake_stats d1 = cached.sg.shadowBakeStats();
  chk(sameD == 5 && d1.whole - d0.whole == 5 && d1.moving_casters == 0 && d1.static_casters == 0,
      "every bake whole, nothing cached, the same pictures (" +
          std::to_string(d1.whole - d0.whole) + " whole of " + std::to_string(d1.bakes - d0.bakes) +
          ")");
  cached.sg.setShadowStaticCache(true);

  std::printf("E. interval 2\n");
  for (World *w : {&cached, &ref})
    w->sg.setShadowUpdateInterval(2);
  for (int i = 0; i < 4; ++i) { // the cache back on: re-sorted, then the vehicle alone
    cached.vehicle->setPosition(-1.5 + 0.2 * i, 0.0, 0.0);
    ref.vehicle->setPosition(-1.5 + 0.2 * i, 0.0, 0.0);
    both();
  }
  const cvc::gl::SceneGraph::shadow_bake_stats e0 = cached.sg.shadowBakeStats(),
                                               f0 = ref.sg.shadowBakeStats();
  int sameE = 0;
  for (int i = 0; i < 12; ++i) {
    cached.vehicle->setPosition(-0.7 + 0.2 * i, 0.0, 0.0);
    ref.vehicle->setPosition(-0.7 + 0.2 * i, 0.0, 0.0);
    sameE += both() ? 1 : 0;
  }
  const cvc::gl::SceneGraph::shadow_bake_stats e1 = cached.sg.shadowBakeStats(),
                                               f1 = ref.sg.shadowBakeStats();
  chk(e1.bakes - e0.bakes == 6 && f1.bakes - f0.bakes == 6 && e1.whole == e0.whole,
      "12 frames, 6 bakes each (" + std::to_string(e1.bakes - e0.bakes) + " cached, " +
          std::to_string(f1.bakes - f0.bakes) + " reference), none whole cached");
  chk(sameE == 12,
      "the same pictures, stale frames included (" + std::to_string(sameE) + " of 12)");

  std::printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
