// lsystem_forest_ari — the cvcGL lsystem_forest demo, ported to Ariadne. The island, sky-gradient,
// lighting rig and shadows are authored declaratively in lsystem_forest.ari on the reusable scene
// DSL (a procedural `heightfield` terrain with height-band colours); the procedural L-system TREES
// are a custom scene-node type this host registers (`type: forest_trees`), the one piece that is
// not expressible declaratively. The trees are planted on the terrain by sampling the realized
// heightfield mesh, so the .ari owns the island and the host owns only the generators.
//
// Everything below the forest_trees registration is the stock cvcGL Ariadne runner (cf.
// ariadne_hello / ariadne_stream): load the doc, realize the scene, frame the camera, render — a
// window, or a one-frame offscreen PNG for verification.
//
//   lsystem_forest_ari lsystem_forest.ari --component-path src/cvc/ariadne/components
//   lsystem_forest_ari lsystem_forest.ari --offscreen --png forest.png --component-path …

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/volume/bounding_box.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkRenderer.h> // the scene's background gradient reaches the renderer directly (§16.1)

using cvc::gl::CameraController;
using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
using cvc::gl::ImGuiBackend;
using cvc::gl::ImGuiOverlay;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::TouchGestures;
namespace ari = cvc::ariadne;

namespace {

// ─────────────────────────── tiny 3-vector + an L-system tree ───────────────────────────

struct V3 {
  double x = 0, y = 0, z = 0;
};
V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double len(V3 a) { return std::sqrt(dot(a, a)); }
V3 norm(V3 a) {
  double l = len(a);
  return l > 1e-12 ? a * (1.0 / l) : V3{0, 0, 1};
}

// A deterministic little RNG so a given seed always grows the same forest.
struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed) : s(seed ? seed : 0x9e3779b97f4a7c15ULL) {}
  uint64_t next() {
    s ^= s << 13;
    s ^= s >> 7;
    s ^= s << 17;
    return s;
  }
  double uniform() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
  double range(double a, double b) { return a + (b - a) * uniform(); }
};

const V3 C_WOOD_LIGHT{0.655, 0.490, 0.239};
const V3 C_WOOD_DARK{0.361, 0.251, 0.200};
const V3 C_NEEDLE{0.137, 0.557, 0.137};

// Append a tapered cylinder (p0->p1, radii r0->r1) as a RING-sided tube with per-vertex colour.
void add_cylinder(cvc::geometry &g, V3 p0, V3 p1, double r0, double r1, int ring, V3 col) {
  V3 axis = norm(p1 - p0);
  // An orthonormal basis (u,v) perpendicular to the axis.
  V3 ref = std::fabs(axis.z) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0};
  V3 u = norm(cross(axis, ref));
  V3 v = norm(cross(axis, u));
  const size_t base = g.points().size();
  for (int k = 0; k < ring; ++k) {
    const double a = 2.0 * M_PI * k / ring;
    const V3 dir = u * std::cos(a) + v * std::sin(a);
    const V3 b0 = p0 + dir * r0, b1 = p1 + dir * r1;
    g.points().push_back({b0.x, b0.y, b0.z});
    g.points().push_back({b1.x, b1.y, b1.z});
    g.normals().push_back({dir.x, dir.y, dir.z});
    g.normals().push_back({dir.x, dir.y, dir.z});
    g.colors().push_back({col.x, col.y, col.z});
    g.colors().push_back({col.x, col.y, col.z});
  }
  for (int k = 0; k < ring; ++k) {
    const unsigned a0 = static_cast<unsigned>(base + 2 * k);
    const unsigned a1 = a0 + 1;
    const unsigned b0 = static_cast<unsigned>(base + 2 * ((k + 1) % ring));
    const unsigned b1 = b0 + 1;
    g.tris().push_back({a0, b0, b1});
    g.tris().push_back({a0, b1, a1});
  }
}

// Grow one recursive tree into `g` at `basePt`: a tapered trunk that splits into `branches` child
// limbs at each of `levels`, tips coloured as needles. Compact, not the full grammar —
// recognizable.
void grow_tree(cvc::geometry &g, Rng &rng, V3 basePt, V3 dir, double length, double radius,
               int level, int levels, int branches) {
  const V3 tip = basePt + dir * length;
  const double tipR = radius * 0.62;
  const bool leaf = (level >= levels);
  // Wood darkens toward the trunk, lightens toward the twigs; the final twigs read as needles.
  V3 col = leaf
               ? C_NEEDLE
               : C_WOOD_DARK + (C_WOOD_LIGHT - C_WOOD_DARK) * (double(level) / std::max(1, levels));
  add_cylinder(g, basePt, tip, radius, tipR, level == 0 ? 7 : 5, col);
  if (leaf)
    return;
  // Spawn child limbs around the tip, tilted out from the parent direction.
  V3 ref = std::fabs(dir.z) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0};
  V3 u = norm(cross(dir, ref));
  V3 v = norm(cross(dir, u));
  const int n = branches + (rng.uniform() < 0.4 ? 1 : 0);
  for (int b = 0; b < n; ++b) {
    const double az = 2.0 * M_PI * (b + rng.range(-0.2, 0.2)) / n;
    const double tilt = rng.range(0.45, 0.80); // radians out from the parent axis
    const V3 out = u * std::cos(az) + v * std::sin(az);
    const V3 cdir = norm(dir * std::cos(tilt) + out * std::sin(tilt));
    grow_tree(g, rng, tip, cdir, length * rng.range(0.62, 0.78), radius * 0.58, level + 1, levels,
              branches);
  }
}

// A height sampler over an already-realized heightfield GeometryNode: the mesh is a regular
// row-major res×res grid (make_heightfield), so nearest-grid-cell lookup gives the terrain height
// at (x,y).
struct TerrainHeights {
  const cvc::geometry *geom = nullptr;
  int res = 0;
  double minx = 0, miny = 0, step = 1;
  bool ok() const { return geom && res > 1; }
  void build(const cvc::geometry *g) {
    geom = g;
    if (!g || g->const_points().empty())
      return;
    const size_t n = g->const_points().size();
    res = static_cast<int>(std::lround(std::sqrt(double(n))));
    if (res < 2 || size_t(res) * res != n) {
      res = 0;
      return;
    }
    double maxx = -1e30, maxy = -1e30;
    minx = miny = 1e30;
    for (const auto &p : g->const_points()) {
      minx = std::min(minx, p[0]);
      maxx = std::max(maxx, p[0]);
      miny = std::min(miny, p[1]);
      maxy = std::max(maxy, p[1]);
    }
    step = (maxx - minx) / (res - 1);
  }
  double at(double x, double y) const {
    if (!ok())
      return 0.0;
    int i = int(std::lround((x - minx) / step));
    int j = int(std::lround((y - miny) / step));
    i = std::max(0, std::min(res - 1, i));
    j = std::max(0, std::min(res - 1, j));
    return geom->const_points()[size_t(j) * res + i][2];
  }
};

// The `type: forest_trees` realizer: scatter `count` L-system trees on dry land of the named ground
// node, merged into one mesh. props: count, seed, ground (node id), sea_level, scale, radius, span,
// levels, branches.
std::shared_ptr<GraphicsNode> realize_forest_trees(SceneGraph &sg, const ari::SceneNode &n,
                                                   GraphicsNode *parent,
                                                   cvc::gl::ariadne::RealizedScene &,
                                                   std::vector<std::string> *warnings) {
  const auto warn = [&](const std::string &m) {
    if (warnings)
      warnings->push_back("ari: forest_trees '" + n.id + "': " + m);
  };
  const int count = static_cast<int>(n.props.num("count", 60.0));
  const uint64_t seed = static_cast<uint64_t>(n.props.num("seed", 1337.0));
  const std::string ground = n.props.str("ground").empty() ? "terrain" : n.props.str("ground");
  const double seaLevel = n.props.num("sea_level", 0.5);
  const double scale = n.props.num("scale", 1.0);
  const double baseLen = n.props.num("length", 6.0) * scale;
  const double baseRad = n.props.num("radius", 0.7) * scale;
  const double span = n.props.num("span", 100.0); // half-extent to scatter across
  const int levels = static_cast<int>(n.props.num("levels", 4.0));
  const int branches = static_cast<int>(n.props.num("branches", 3.0));

  TerrainHeights heights;
  if (auto gnode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics(ground)))
    heights.build(gnode->getGeometry());
  if (!heights.ok())
    warn("ground node '" + ground + "' is not a realized heightfield — planting on a flat plane");

  cvc::geometry forest;
  Rng rng(seed);
  int planted = 0;
  for (int attempt = 0; attempt < count * 6 && planted < count; ++attempt) {
    const double x = rng.range(-span, span);
    const double y = rng.range(-span, span);
    if (std::sqrt(x * x + y * y) > span)
      continue;
    const double z = heights.at(x, y);
    if (z < seaLevel + 0.5)
      continue; // only dry land above the waterline
    grow_tree(forest, rng, V3{x, y, z}, V3{0, 0, 1}, baseLen * rng.range(0.8, 1.25), baseRad, 0,
              levels, branches);
    ++planted;
  }
  if (forest.points().empty()) {
    warn("no trees planted (no dry land above sea_level in the scatter span)");
    return nullptr;
  }

  std::shared_ptr<GraphicsNode> gn =
      parent ? parent->createChild<GeometryNode>(n.id, forest) : sg.addGraphics(n.id, forest);
  if (auto geo = std::dynamic_pointer_cast<GeometryNode>(gn))
    geo->setUseSingleColor(false); // per-vertex wood/needle colour
  std::printf("[lsystem_forest_ari] planted %d trees (%zu triangles)\n", planted,
              forest.tris().size());
  return gn;
}

} // namespace

int main(int argc, char **argv) {
  std::string docPath, png;
  bool offscreen = false;
  long frames = 0;
  int width = 1024, height = 768;
  std::vector<std::string> componentPaths;
  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    auto next = [&](const char *d) { return (i + 1 < argc) ? argv[++i] : d; };
    if (!std::strcmp(a, "--offscreen"))
      offscreen = true;
    else if (!std::strcmp(a, "--png")) {
      png = next("");
      offscreen = true;
    } else if (!std::strcmp(a, "--frames"))
      frames = std::atol(next("0"));
    else if (!std::strcmp(a, "--width"))
      width = std::atoi(next("1024"));
    else if (!std::strcmp(a, "--height"))
      height = std::atoi(next("768"));
    else if (!std::strcmp(a, "--component-path"))
      componentPaths.push_back(next(""));
    else if (a[0] != '-' && docPath.empty())
      docPath = a;
  }
  cvc::ariadne::register_cvc_uri_handler(componentPaths);
  const bool capturing = offscreen || !png.empty();
  if (capturing && frames <= 0)
    frames = 2;

  cvc::app app;
  SceneGraph sg(app, "forest");
  SceneRenderer view(sg, width, height, offscreen, "main");
  CameraController cam(view);
  cam.setMode(CameraController::Mode::Orbit);
  TouchGestures touch(view, cam);
  ImGuiOverlay ui(view);
  ui.attachCamera(cam);

  ari::Runtime rt(app, sg.getStatePrefix());
  ImGuiBackend backend;
  rt.set_backend(&backend);
  backend.install(rt, ui);
  bool quit = false;
  rt.on("quit", [&] { quit = true; });

  // The forest's one non-declarative piece: the procedural L-system trees.
  cvc::gl::ariadne::register_scene_node_type("forest_trees", realize_forest_trees);

  if (docPath.empty()) {
    std::printf("[lsystem_forest_ari] FATAL: give a .ari document (e.g. lsystem_forest.ari)\n");
    return 1;
  }
  ari::LoadResult lr = ari::load_file(docPath.c_str());
  if (!lr.ok) {
    std::printf("[lsystem_forest_ari] load failed: %s\n", lr.error.c_str());
    return 1;
  }
  for (const std::string &w : lr.warnings)
    std::printf("[lsystem_forest_ari]   %s\n", w.c_str());

  std::vector<std::string> init_errs;
  ari::run_init(app, sg.getStatePrefix(), lr.init_script, &init_errs);
  rt.set_tick_program(lr.on_tick_script);
  rt.set_root(std::move(lr.root));

  cvc::gl::ariadne::RealizedScene realized;
  if (lr.scene.any()) {
    std::vector<std::string> sw;
    realized = cvc::gl::ariadne::realize_scene(sg, lr.scene, sg.getStatePrefix(), &sw);
    for (const std::string &w : sw)
      std::printf("[lsystem_forest_ari]   %s\n", w.c_str());
    std::printf("[lsystem_forest_ari] scene: %zu node(s)\n", realized.created.size());
    const cvc::bounding_box bb = sg.computeGraphicsBounds();
    if (!bb.isNull())
      cam.frameBounds(bb.minx, bb.miny, bb.minz, bb.maxx, bb.maxy, bb.maxz);
    // Apply the scene's background (a VIEW property): a solid colour, or a top→bottom gradient
    // reached through the renderer (SceneRenderer exposes only a flat setBackground).
    if (lr.scene.has_background) {
      const auto &t = lr.scene.background_top;
      const auto &b = lr.scene.background_bottom;
      if (lr.scene.background_gradient) {
        if (vtkRenderer *r = view.renderer()) {
          r->SetGradientBackground(true);
          r->SetBackground(b[0], b[1], b[2]);  // bottom
          r->SetBackground2(t[0], t[1], t[2]); // top
        }
      } else {
        view.setBackground(t[0], t[1], t[2]);
      }
    }
  }

  auto frame_body = [&](double dt) {
    view.processUIEvents();
    touch.update();
    cam.update(dt);
    rt.drain();
    ari::sync_scene_visibility(app, realized.visibility);
    cvc::gl::ariadne::tick_scene(realized, view.renderer());
  };

  if (capturing) {
    for (long f = 0; f < frames; ++f) {
      frame_body(1.0 / 60.0);
      if (f + 1 == frames && !png.empty())
        view.writePNG(png.c_str());
      else
        view.render();
    }
    std::printf("[lsystem_forest_ari] captured %ld frame(s)%s%s\n", frames,
                png.empty() ? "" : " -> ", png.c_str());
  } else {
    std::puts("[lsystem_forest_ari] running — close the window or Sim > Quit to exit.");
    while (!view.windowClosed() && !quit) {
      frame_body(1.0 / 120.0);
      view.render();
    }
  }
  return 0;
}
