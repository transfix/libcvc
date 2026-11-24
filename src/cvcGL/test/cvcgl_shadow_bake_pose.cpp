// A caster that moved since the last shadow bake is shaded as it stood at that bake.
//
// With an update interval n > 1 the baker bakes every nth frame and the frames between sample the
// last maps (StridedShadowBaker). A caster that had moved since -- a crate bobbing on the sea, a
// boat -- was looked up at its new pose against a map of its old one: where it had moved behind
// its own baked surface it read as shadowed, and the patch cleared at the next bake, so a moving
// body flashed at the bake cadence (terrain_lab at update_interval 4: a crate's brightness jumped
// back up every 4th frame and dimmed over the three between). FilteredShadowMapPass now looks a
// moved caster's shadow up where it stood at the bake, so its own surface agrees with the map.
//
// Each pin compares a frame that does not bake, the maps one pose behind, with a frame that bakes
// at the same pose:
//   A. a caster lowered 1.2 units: its sunlit top is as bright as under fresh maps;
//   B. the caster tilted 15 degrees, lowered and moved away from the sun: the same;
//   C. beside it, a caster that did not move and a non-caster that did draw the same either way
//      (the shadow pass's per-draw transform is the moved caster's alone).
// Renders for real; skips where nothing rasterises (fatal under CVC_REQUIRE_RENDER=1).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <vtkProp.h>

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

// A flat ground at height z in step x step tiles (as cvcgl_shadow_casters' tiledGround).
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

// Exposes the node's actor, so a frame can be made to bake (a touched caster is a changed one).
class Node : public GeometryNode {
public:
  Node(cvc::app &a, const std::string &path, const std::string &name)
      : GeometryNode(a, path, name) {}
  vtkProp *actor() { return getProp(); }
};

// The square of pixels within r of (cx, cy), rows bottom-up as frameRGB returns them.
struct Patch {
  int cx, cy, r;
};

static double meanLuma(const std::vector<unsigned char> &rgb, int w, const Patch &p) {
  double sum = 0.0;
  long n = 0;
  for (int y = p.cy - p.r; y <= p.cy + p.r; ++y)
    for (int x = p.cx - p.r; x <= p.cx + p.r; ++x) {
      const std::size_t o = (static_cast<std::size_t>(y) * w + x) * 3;
      sum += 0.299 * rgb[o] + 0.587 * rgb[o + 1] + 0.114 * rgb[o + 2];
      ++n;
    }
  return n ? sum / n : 0.0;
}

// The largest byte step between two frames over a patch.
static int maxStep(const std::vector<unsigned char> &a, const std::vector<unsigned char> &b, int w,
                   const Patch &p) {
  int worst = 0;
  for (int y = p.cy - p.r; y <= p.cy + p.r; ++y)
    for (int x = p.cx - p.r; x <= p.cx + p.r; ++x)
      for (int c = 0; c < 3; ++c) {
        const std::size_t o = (static_cast<std::size_t>(y) * w + x) * 3 + c;
        worst = std::max(worst, std::abs(int(a[o]) - int(b[o])));
      }
  return worst;
}

static bool canRasterise(cvc::app &app) {
  SceneGraph sg(app, "bakeposecontrol");
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

int main() {
  cvc::app app;
  std::printf("cvcgl_shadow_bake_pose\n");
  if (!canRasterise(app)) {
    const char *require = std::getenv("CVC_REQUIRE_RENDER");
    if (require && *require && std::string(require) != "0") {
      std::printf("CVC_REQUIRE_RENDER is set, but this build did not rasterise\n");
      return 1;
    }
    std::printf("skipped: this build did not rasterise\n");
    return 0;
  }

  // Seen from straight above: a white ground; the mover, a 6 x 6 x 6 caster floating at the
  // centre; a still caster to its north and a non-caster to its south. A sun from +x at 45
  // degrees throws their shadows west.
  SceneGraph sg(app, "bakepose");
  sg.setDiagnosticChromeVisible(false);
  auto ground = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("ground");
  ground->setGeometry(tiledGround(-30, -30, 30, 30, 0.0, 2.0));
  ground->setColor(1.0, 1.0, 1.0);
  auto mover = sg.getGraphicsRoot()->addGraphicsChild<Node>("mover");
  mover->setGeometry(box(-3, -3, 0, 3, 3, 6));
  mover->setColor(0.85, 0.65, 0.45);
  auto still = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("still");
  still->setGeometry(box(-1.5, -1.5, 0, 1.5, 1.5, 3));
  still->setPosition(0.0, 7.0, 0.0);
  auto ghost = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("ghost");
  ghost->setGeometry(box(-1.5, -1.5, 0, 1.5, 1.5, 3));
  ghost->setCastsShadow(false);
  sg.processEvents();

  const int W = 128, H = 128;
  SceneRenderer view(sg, W, H, /*offscreen=*/true, "main");
  view.setBackground(0.0, 0.0, 1.0);
  view.setCamera(0, 0, 40, 0, 0, 0, 0, 1, 0, 30.0, 1.0, 200.0);
  sg.addDirectionalLight(90.0, 45.0); // from +x
  sg.setShadowUpdateInterval(4);      // before shadows go on: changing it rebuilds the passes
  sg.setShadowResolution(1024);
  if (!sg.setShadowsEnabled(true)) {
    std::printf("skipped: shadows unavailable\n");
    return 0;
  }

  // One frame; whether it baked.
  auto frame = [&](std::vector<unsigned char> &rgb) {
    sg.processEvents();
    const std::uint64_t before = sg.shadowBakes();
    rgb = view.frameRGB();
    return sg.shadowBakes() != before && rgb.size() == static_cast<std::size_t>(W) * H * 3;
  };
  // A frame that bakes: the mover touched (a changed caster) and the bake forced.
  auto baked = [&](std::vector<unsigned char> &rgb) {
    mover->actor()->Modified();
    sg.invalidateShadowBake();
    return frame(rgb);
  };
  // At `base`, a frame that bakes; then at `moved`, a frame that does not (stale) and one that
  // does (fresh). A due frame bakes even so; then again, the counter moved on.
  auto pair = [&](const std::function<void()> &base, const std::function<void()> &moved,
                  std::vector<unsigned char> &stale, std::vector<unsigned char> &fresh) {
    std::vector<unsigned char> scratch;
    for (int attempt = 0; attempt < 4; ++attempt) {
      base();
      if (!baked(scratch))
        continue;
      moved();
      if (frame(stale))
        continue;
      return baked(fresh);
    }
    return false;
  };

  for (int i = 0; i < 4; ++i) { // settle the first bakes
    std::vector<unsigned char> scratch;
    frame(scratch);
  }

  // The mover's top at the centre (z 10, 30 below the eye: about 8 px a unit, the top +-24 px);
  // the still caster's and the non-caster's tops 7 units north and south (about 6.5 px a unit).
  const Patch top{W / 2, H / 2, 8}, northTop{W / 2, H / 2 + 45, 4};
  const Patch litGround{W / 2 + 56, H / 2, 4};        // x +7 on the ground: east of everything
  const Patch shadowGround{W / 2 - 64 + 6, H / 2, 4}; // under the mover's shadow, west

  auto atBase = [&]() {
    mover->setPosition(0.0, 0.0, 4.0);
    mover->setRotation(0.0, 0.0, 0.0);
    ghost->setPosition(0.0, -7.0, 0.0);
  };

  std::printf("A. lowered between bakes\n");
  std::vector<unsigned char> stale, fresh;
  const bool gotA = pair(atBase, [&]() { mover->setPosition(0.0, 0.0, 2.8); }, stale, fresh);
  chk(gotA, "a frame that did not bake after the move, and one that did");
  if (gotA) {
    const double s = meanLuma(stale, W, top), f = meanLuma(fresh, W, top);
    const double lit = meanLuma(fresh, W, litGround), dark = meanLuma(fresh, W, shadowGround);
    std::printf("  top: %.1f stale, %.1f fresh; ground: %.1f lit, %.1f in its shadow\n", s, f, lit,
                dark);
    chk(lit - dark > 15.0, "the sun shadows the ground visibly (else nothing here could show)");
    chk(f - dark > 15.0, "under fresh maps its top is sunlit");
    chk(std::fabs(s - f) < 2.0, "under the maps of its old pose, as bright (" + std::to_string(s) +
                                    " vs " + std::to_string(f) + ")");
  }

  std::printf("B. tilted, lowered and moved away from the sun between bakes\n");
  // About y through its base: its top tips toward the sun (+x) and drops 0.2 at its middle, a
  // further 0.6 with the move. Its middle ends up about a unit east, still inside `top`.
  const bool gotB = pair(
      atBase,
      [&]() {
        mover->setPosition(-0.5, 0.0, 3.4);
        mover->setRotation(0.0, 15.0, 0.0);
      },
      stale, fresh);
  chk(gotB, "a frame that did not bake after the move, and one that did");
  if (gotB) {
    const double s = meanLuma(stale, W, top), f = meanLuma(fresh, W, top);
    std::printf("  top: %.1f stale, %.1f fresh\n", s, f);
    chk(std::fabs(s - f) < 2.0, "under the maps of its old pose, as bright (" + std::to_string(s) +
                                    " vs " + std::to_string(f) + ")");
  }

  std::printf("C. the others draw the same\n");
  const bool gotC = pair(
      atBase,
      [&]() {
        mover->setPosition(0.0, 0.0, 2.8);
        ghost->setPosition(0.5, -7.0, 0.0);
      },
      stale, fresh);
  chk(gotC, "a frame that did not bake after the move, and one that did");
  if (gotC) {
    const Patch southTop{W / 2 + 3, H / 2 - 45, 4}; // the non-caster, moved half a unit east
    const int n = maxStep(stale, fresh, W, northTop), s = maxStep(stale, fresh, W, southTop);
    chk(n <= 1, "the caster that did not move: largest step " + std::to_string(n));
    chk(s <= 1, "the non-caster that moved: largest step " + std::to_string(s));
  }

  std::printf("%s\n", fails ? "FAILED" : "all passed");
  return fails ? 1 : 0;
}
