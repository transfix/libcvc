// material: { opacity } on an Ariadne scene node (docs/ARIADNE.md §5.4, "See-through nodes"), from
// a loaded document through realize_scene and tick_scene. Checked:
//   1. a literal opacity sets the mesh's vtkProperty and draws it in the translucent pass; with
//      opaque: true beside it the opacity wins (the pass is not forced opaque), as a viewport
//      override:'s opacity does; opaque: alone still forces the opaque pass;
//   2. a bound opacity starts at its default, follows its key each tick (clamped to 0..1, a
//      non-number falling back to the default), and gives the forced-opaque pass back at 1;
//   3. every rung of a LOD ladder takes it, the coarser rungs built after realize included;
//   4. what a source: { data } node draws takes it, and so does its redraw from a new dataset;
//   5. the pixels: a half see-through red tile over a blue ground reads as both in the main view,
//      with opaque: true as without it, while a solid one reads red.
// The pixels skip (rc 0) where nothing rasterises, unless CVC_REQUIRE_RENDER=1.
#include "streaming_test_util.h"

#include <cmath>
#include <cstdio>
#include <cvc/ariadne/bind.h> // resolve_bind
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h> // a deferred LOD build
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/LodGraphicsNode.h>
#include <cvc/gl/LodPyramidCache.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkProperty.h>

using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvcgl_test::check;
namespace ari = cvc::gl::ariadne;
namespace fs = std::filesystem;

namespace {

cvc::ariadne::LoadResult load(const std::string &yaml) {
  cvc::app app; // the built-in registries only
  return cvc::ariadne::load_string(app, yaml);
}

vtkActor *actor_of(const std::shared_ptr<GraphicsNode> &n) {
  auto g = std::dynamic_pointer_cast<GeometryNode>(n);
  return g ? vtkActor::SafeDownCast(g->prop()) : nullptr;
}

double opacity_of(vtkActor *a) { return a ? a->GetProperty()->GetOpacity() : -1.0; }

// Drawn see-through at `opacity`: in the translucent pass, not forced opaque.
bool see_through(vtkActor *a, double opacity) {
  return a && std::fabs(opacity_of(a) - opacity) < 1e-6 && !a->GetForceOpaque() &&
         a->HasTranslucentPolygonalGeometry() && !a->HasOpaqueGeometry();
}

// Drawn solid, forced into the opaque pass or not.
bool solid(vtkActor *a, bool forced) {
  return a && opacity_of(a) == 1.0 && bool(a->GetForceOpaque()) == forced && a->HasOpaqueGeometry();
}

void set_key(cvc::app &app, const std::string &prefix, const std::string &key,
             const std::string &v) {
  cvc::state::instance(app)(cvc::ariadne::resolve_bind(prefix, key)).value(v);
}

void tick(ari::RealizedScene &out, SceneGraph &sg) {
  ari::tick_scene(out, nullptr, 0.0);
  sg.processEvents();
}

const char *kScene = R"(
meta: { min_libcvc: "0.0.0" }
scene:
  chrome: false
  nodes:
    - node: ground
      source: { plane: { size: 30 } }
      material: { color: [0, 0, 1], ambient: 1, diffuse: 0, specular: 0, opaque: true }
    - node: half
      source: { plane: { size: 5 } }
      transform: { position: [-7, 0, 2] }
      material: { color: [1, 0, 0], ambient: 1, diffuse: 0, specular: 0, opacity: 0.5 }
    - node: half_opaque
      source: { plane: { size: 5 } }
      transform: { position: [0, 0, 2] }
      material: { color: [1, 0, 0], ambient: 1, diffuse: 0, specular: 0, opacity: 0.5,
                  opaque: true }
    - node: full
      source: { plane: { size: 5 } }
      transform: { position: [7, 0, 2] }
      material: { color: [1, 0, 0], ambient: 1, diffuse: 0, specular: 0 }
    - node: veil
      source: { box: 1 }
      transform: { position: [0, 9, 2] }
      material: { opacity: { bind: veil.alpha, default: 0.25 }, opaque: true }
)";

void test_literal_and_bound(cvc::app &app) {
  std::printf("== material.opacity: literal and bound, on a mesh ==\n");
  cvc::ariadne::LoadResult r = load(kScene);
  check(r.ok, "the document loads", r.error);
  if (!r.ok)
    return;
  SceneGraph sg(app, "opa");
  sg.setDiagnosticChromeVisible(false);
  SceneRenderer sr(sg, 240, 120, /*offscreen=*/true, "main");
  const std::string prefix = "opa";
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, prefix, &warnings);
  sg.processEvents();
  check(warnings.empty(), "no warnings", warnings.empty() ? "" : warnings.front());

  vtkActor *ground = actor_of(sg.getGraphics("ground"));
  vtkActor *half = actor_of(sg.getGraphics("half"));
  vtkActor *halfOpaque = actor_of(sg.getGraphics("half_opaque"));
  vtkActor *full = actor_of(sg.getGraphics("full"));
  vtkActor *veil = actor_of(sg.getGraphics("veil"));
  check(see_through(half, 0.5), "opacity: 0.5 -> the mesh draws see-through at 0.5",
        "opacity " + std::to_string(opacity_of(half)));
  check(see_through(halfOpaque, 0.5), "opacity: 0.5 with opaque: true -> still see-through");
  check(solid(ground, /*forced=*/true), "opaque: alone still forces the opaque pass");
  check(solid(full, /*forced=*/false), "no opacity: the mesh stays solid");
  check(out.opacity.size() == 1, "one bound opacity (veil)");
  check(see_through(veil, 0.25), "a bound opacity starts at its default while the key is unset",
        "opacity " + std::to_string(opacity_of(veil)));

  set_key(app, prefix, "veil.alpha", "1");
  tick(out, sg);
  check(solid(veil, /*forced=*/true), "the key at 1 -> solid, and opaque: forces the pass again");
  set_key(app, prefix, "veil.alpha", "0.6");
  tick(out, sg);
  check(see_through(veil, 0.6), "the key at 0.6 -> see-through at 0.6 on the next tick");
  set_key(app, prefix, "veil.alpha", "7");
  tick(out, sg);
  check(solid(veil, /*forced=*/true), "the key past 1 is clamped to 1");
  set_key(app, prefix, "veil.alpha", "-3");
  tick(out, sg);
  check(see_through(veil, 0.0), "the key below 0 is clamped to 0");
  set_key(app, prefix, "veil.alpha", "no");
  tick(out, sg);
  check(see_through(veil, 0.25), "a key that is not a number -> the default");

  // 5. The pixels, in the main view, from above.
  if (!cvcgl_test::renderAvailable(app))
    return;
  sr.setBackground(0, 0, 0);
  cvcgl_test::topView(sr, 0, 0, 12);
  const cvcgl_test::Frame f = cvcgl_test::grab(sr);
  auto at = [&](double x, double y) {
    const std::array<double, 2> p = cvcgl_test::toPx(sr, x, y, 2.0);
    return f.at(int(p[0]), int(p[1]));
  };
  auto rgb = [](const unsigned char *p) {
    return std::to_string(p[0]) + "," + std::to_string(p[1]) + "," + std::to_string(p[2]);
  };
  const unsigned char *g = at(-7, -8), *h = at(-7, 0), *ho = at(0, 0), *s = at(7, 0);
  check(g[2] > 200 && g[0] < 40, "the ground reads blue", rgb(g));
  check(h[0] > 70 && h[2] > 70, "the half see-through tile reads red over blue", rgb(h));
  check(ho[0] > 70 && ho[2] > 70, "... with opaque: true too", rgb(ho));
  check(s[0] > 200 && s[2] < 40, "the solid tile reads red", rgb(s));
}

void test_lod_rungs(cvc::app &app) {
  std::printf("== material.opacity on a LOD ladder's rungs ==\n");
  cvc::ariadne::LoadResult r = load(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: bunny
      source: { file: x.bunny }
      material: { color: [1, 0.4, 0.2], opacity: { bind: bunny.alpha, default: 0.3 } }
)");
  check(r.ok, "the document loads", r.error);
  if (!r.ok)
    return;
  cvc::gl::LodPyramidCache cache(cvc::lod::pyramid_params(),
                                 cvc::gl::LodPyramidCache::default_max_build_tris(),
                                 cvc::lane_mode::deferred);
  ari::RealizeOptions opts;
  opts.lod = &cache; // a mesh file of 4,096 triangles or more is a ladder: the bunny's 69,473
  SceneGraph sg(app, "opalod");
  const std::string prefix = "opalod";
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, prefix, nullptr, opts);
  sg.processEvents();
  auto lod = std::dynamic_pointer_cast<cvc::gl::LodGraphicsNode>(sg.getGraphics("bunny"));
  check(lod != nullptr && lod->rungCount() == 1, "the bunny is a LOD ladder, rung 0 at first");
  if (!lod)
    return;
  auto rung = [&](int k) { return vtkActor::SafeDownCast(lod->rung(k)->prop()); };
  check(see_through(rung(0), 0.3), "rung 0 starts at the default");
  set_key(app, prefix, "bunny.alpha", "0.7");
  tick(out, sg);
  check(see_through(rung(0), 0.7), "rung 0 follows the key");
  check(cache.poll() == 1, "the coarser rungs build");
  check(lod->rungCount() > 1, "the ladder has coarser rungs now",
        std::to_string(lod->rungCount()) + " rungs");
  tick(out, sg);
  bool all = true;
  for (int k = 0; k < lod->rungCount(); ++k)
    all = all && see_through(rung(k), 0.7);
  check(all, "every rung, the ones built after realize included, takes the bound opacity");
}

struct TempDir {
  fs::path dir;
  TempDir() {
    std::random_device rd;
    dir = fs::temp_directory_path() /
          ("cvcgl-opacity-" + std::to_string(rd()) + std::to_string(rd()));
    fs::create_directories(dir);
  }
  ~TempDir() {
    std::error_code ec;
    fs::remove_all(dir, ec);
  }
  std::string put(const std::string &name, const std::string &bytes) const {
    std::ofstream(dir / name, std::ios::binary) << bytes;
    return (dir / name).generic_string();
  }
};

void test_data_node(cvc::app &app) {
  std::printf("== material.opacity on what a source: { data } node draws ==\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  TempDir t;
  t.put("tri.off", "OFF\n3 1 0\n0 0 0\n1 0 0\n0 1 0\n3 0 1 2\n");
  const std::string other =
      t.put("quad.off", "OFF\n4 1 0\n0 0 0\n2 0 0\n2 2 0\n0 2 0\n4 0 1 2 3\n");
  cvc::ariadne::LoadResult r = load(R"(
meta: { min_libcvc: "0.0.0" }
data:
  tri: tri.off
scene:
  nodes:
    - node: zone
      source: { data: tri }
      material: { opacity: { bind: zone.alpha, default: 0.4 }, opaque: true }
)");
  check(r.ok, "the document loads", r.error);
  if (!r.ok)
    return;
  SceneGraph sg(app);
  const std::string prefix = sg.getStatePrefix();
  auto store = cvc::ariadne::DataStore::of(app);
  for (cvc::ariadne::DataSpec d : r.data) {
    d.base = t.dir.generic_string();
    store->bind(prefix, d);
  }
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, prefix, nullptr);
  const std::string path = prefix + ".data.tri";
  check(store->wait(path, 30.0), "the dataset loads");
  tick(out, sg); // draws it, then its opacity
  auto zone = sg.getGraphics("zone");
  auto drawn = [&]() -> std::shared_ptr<GraphicsNode> {
    if (!zone)
      return nullptr;
    const auto kids = zone->getGraphicsChildren();
    return kids.size() == 1 ? kids[0] : nullptr;
  };
  std::shared_ptr<GraphicsNode> first = drawn();
  check(first != nullptr, "the node drew its mesh");
  check(see_through(actor_of(first), 0.4), "the drawn mesh is see-through at the default",
        "opacity " + std::to_string(opacity_of(actor_of(first))));
  set_key(app, prefix, "zone.alpha", "0.8");
  tick(out, sg);
  check(see_through(actor_of(first), 0.8), "and follows the key");

  cvc::ariadne::DataSpec quad;
  quad.name = "tri";
  quad.uri = other;
  store->bind(prefix, quad);
  check(store->wait(path, 30.0), "the replacement loads");
  tick(out, sg);
  std::shared_ptr<GraphicsNode> second = drawn();
  check(second != nullptr && second != first, "the node drew the new dataset");
  check(see_through(actor_of(second), 0.8), "the redrawn mesh takes the bound opacity",
        "opacity " + std::to_string(opacity_of(actor_of(second))));
}

} // namespace

int main() {
  cvcgl_test::installErrorCounter();
  cvc::app app;
  test_literal_and_bound(app);
  test_lod_rungs(app);
  test_data_node(app);
  check(cvcgl_test::ErrorCounter::errors() == 0, "no VTK errors");
  return cvcgl_test::finish("cvcgl_ariadne_opacity");
}
