// terrain_lab — a pure-C++ cvcGL demo that RENDERS what cvc::world produces.
//
// The forest/coast demos hand-author their terrain (a fixed island + a hardcoded
// tree grammar). Terrain Lab instead drives the real headless generator:
//   cvc::world::world_model::generate(params) -> heightfield + surface registry +
//   scattered L-system props; cvc::world::raster() -> the occupancy/material/height
//   grid the DBG training + gym consumers see. This demo turns that same output into
//   geometry so we can SEE and TUNE it: the terrain mesh is coloured by the raster's
//   per-cell MATERIAL (or risk / occupancy / height), the props are grown from the
//   built-in L-system recipes and tessellated in place, water fills every cell that
//   sits below the sea level (streams, ponds, coast), and a drifting cloud slab sits
//   overhead — all navigable with the built-in CameraController.
//
// The ImGui panel re-generates the world live: seed, per-species counts, tree
// generations, relief amplitude, sea level, ontology and a preset — so you can tune
// the generator and watch the scene rebuild. This is the visual QA surface for the
// L-system terrain source (dbg-technical/21 §5, LSYSTEM-LABORATORY-ROADMAP.md).
//
// Run (onscreen, navigable):   terrain_lab
//   Tab toggles orbit/fly; WASD + mouse to fly; Esc releases the pointer.
// Verify (offscreen, headless): terrain_lab --offscreen --frames 30 --png out.png
// Cinematic capture:            terrain_lab --capture fly --frames 900 --out frames

#define _USE_MATH_DEFINES
#include <algorithm>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/FpsHud.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/ImGuiBinding.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/gl/TouchGestures.h>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/image/image.h>
#ifdef __EMSCRIPTEN__
#include <cvc/gl/state_publisher.h>
#include <emscripten.h>
#endif
#include <cvc/lsys/derive.h>
#include <cvc/lsys/grammar.h>
#include <cvc/lsys/interp.h>
#include <cvc/lsys/recipes.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/volume.h>
#include <cvc/world/grid.h>
#include <cvc/world/heightfield.h>
#include <cvc/world/raster.h>
#include <cvc/world/scatter.h>
#include <cvc/world/surface.h>
#include <deque>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>
#include <vtkRenderer.h>

using cvc::gl::CameraController;
using cvc::gl::GeometryNode;
using cvc::gl::GeometryRenderMode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::VolumeNode;
namespace lsys = cvc::lsys;
namespace world = cvc::world;
using idx_t = cvc::geometry::index_t;

namespace {

// ── small vector maths (Z-up, metres) ───────────────────────────────────────
struct Vec3d {
  double x = 0, y = 0, z = 0;
};
Vec3d operator+(Vec3d a, Vec3d b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3d operator-(Vec3d a, Vec3d b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3d operator*(Vec3d a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double vlen(Vec3d a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
Vec3d vnorm(Vec3d a) {
  double l = vlen(a);
  return l > 1e-12 ? Vec3d{a.x / l, a.y / l, a.z / l} : Vec3d{0, 0, 1};
}
Vec3d vcross(Vec3d a, Vec3d b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
Vec3d fromLsys(const lsys::vec3 &v) { return {v.x, v.y, v.z}; }
struct Apron {                // a paved plaza hugging one building's footprint (rotated by yaw)
  double x = 0, y = 0, z = 0; // centre + base height
  double hx = 0, hy = 0;      // footprint half-extents (+ sidewalk margin)
  double yaw = 0;             // building rotation (radians)
};
// Rotate about +Z by yaw (radians).
Vec3d rotZ(Vec3d p, double c, double s) { return {p.x * c - p.y * s, p.x * s + p.y * c, p.z}; }

// ── the world scale + shared state the sea/sky/sun read ──────────────────────
// world_model owns the heightfield; a pointer to the live one lets the water/shadow
// code sample the same terrain the mesh was built from, across regenerations.
double gHalf = 600.0;   // world spans [-gHalf, gHalf]^2 (metres)
double gSeaLevel = 0.0; // water surface / shoreline height
const world::heightfield *gHF = nullptr;

double terrainH(double x, double y) { return gHF ? gHF->sample(x, y) : 0.0; }

// ── generation parameters (the ImGui panel edits these) ──────────────────────
struct GenParams {
  std::uint64_t seed = 7;
  int trees = 500;
  int rocks = 90;
  int buildings = 110;
  int tree_gen = 5;
  double amp_m = 44.0;
  // Water SURFACE height. The heightfield floor sits at 0, so terrain rises in
  // [0, amp]; anything below this line floods — valleys become streams/ponds and
  // the low rim becomes coast. Default ~a third of the relief, so water reads.
  double water_level_m = 13.0;
  int preset = 0; // 0 standard, 1 forest, 2 urban, 3 sparse, 4 island
  int ontology = 0;
};
const char *const kPresetNames[] = {"standard", "forest", "urban", "sparse", "island"};
const char *const kOntologyNames[] = {"merged_default", "soft_vegetation", "strict_water_mud"};
const char *const kColorModeNames[] = {"material", "risk", "occupancy", "height"};

world::world_params toWorldParams(const GenParams &gp) {
  world::world_params wp;
  wp.min_x = wp.min_y = -gHalf;
  wp.max_x = wp.max_y = gHalf;
  wp.seed = gp.seed;
  wp.ontology = kOntologyNames[std::max(0, std::min(2, gp.ontology))];
  // Relief scaled for a large world: long base wavelength, deep enough that low
  // ground drops below the sea to carve streams/ponds.
  wp.hf.seed = gp.seed;
  wp.hf.amp_m = gp.amp_m;
  wp.hf.sea_level_m = 0.0; // terrain floor at 0; water floods above it (see water_level_m)
  // More variance: a shorter base wavelength + more octaves + higher gain give
  // steeper, more broken relief, so the generator classifies more gravel / scree /
  // bare rock (dirt) on the slopes instead of an all-grass plain.
  wp.hf.base_wavelength_m = std::max(50.0, gHalf * 0.30);
  wp.hf.octaves = 8;
  wp.hf.gain = 0.55;
  wp.hf.lacunarity = 2.1;
  wp.hf.warp_m = 22.0;
  wp.hf.warp_wavelength_m = gHalf * 0.6;
  // The "island" preset shapes the relief (a central dome dropping below the water
  // line at the rim); every preset's object mix lives in the count sliders (see
  // presetFill), so counts here always come straight from the panel.
  if (gp.preset == 4) {
    wp.hf.island = true;
    wp.hf.island_peak_m = std::max(gp.amp_m, 40.0);
    wp.hf.island_radius_m = gHalf * 0.72;
    wp.hf.shelf_m = -std::max(6.0, gp.water_level_m + 4.0);
  }
  world::scatter_params &s = wp.sc;
  // The default L-system set for Terrain Lab: favour the full-crowned oak, a good
  // share of conical pine, then shrubs and a few slender birches — a mix that reads
  // as varied woodland rather than a field of poles. Rocks are boulder clusters;
  // buildings split concrete offices and brick houses (concrete/brick materials).
  s.trees = {{"oak_sympodial", 0.46},
             {"pine_monopodial", 0.30},
             {"shrub_bush", 0.16},
             {"birch_slender", 0.08}};
  s.rocks = {{"boulder_cluster", 1.0}};
  s.buildings = {{"office_block", 0.5}, {"brick_house", 0.5}};
  s.tree_count = gp.trees;
  s.rock_count = gp.rocks;
  s.building_count = gp.buildings;
  s.tree_gen = gp.tree_gen;
  s.min_ground_m = gp.water_level_m + 1.0; // keep props on dry land, above the waterline
  return wp;
}

// Fill the count/amp fields with a sensible mix for the chosen preset (the panel's
// quick-set: pick a preset, get a good starting point, then fine-tune the sliders).
void presetFill(GenParams &gp) {
  switch (gp.preset) {
  case 1:
    gp.trees = 1600;
    gp.rocks = 220;
    gp.buildings = 8;
    gp.amp_m = 26.0;
    break; // forest
  case 2:
    gp.trees = 320;
    gp.rocks = 40;
    gp.buildings = 340;
    gp.amp_m = 10.0;
    break; // urban
  case 3:
    gp.trees = 260;
    gp.rocks = 140;
    gp.buildings = 70;
    gp.amp_m = 22.0;
    break; // sparse
  case 4:
    gp.trees = 900;
    gp.rocks = 220;
    gp.buildings = 60;
    gp.amp_m = 46.0;
    break; // island
  default:
    gp.trees = 500;
    gp.rocks = 90;
    gp.buildings = 110;
    gp.amp_m = 36.0;
    break; // standard
  }
}

// ── raster -> terrain mesh ───────────────────────────────────────────────────
constexpr int TERRAIN_N = 289; // render grid resolution (cell = 2*gHalf/(N-1))

// Colour for one raster cell under the current colour mode.
cvc::geometry::color_t cellColor(const world::raster_out &ro, const world::surface_registry &reg,
                                 std::size_t i, int mode, float hmin, float hmax) {
  auto heat = [](double t) -> cvc::geometry::color_t { // blue -> green -> yellow -> red
    t = std::min(1.0, std::max(0.0, t));
    double r = std::min(1.0, std::max(0.0, 1.5 - std::fabs(4.0 * t - 3.0)));
    double g = std::min(1.0, std::max(0.0, 1.5 - std::fabs(4.0 * t - 2.0)));
    double b = std::min(1.0, std::max(0.0, 1.5 - std::fabs(4.0 * t - 1.0)));
    return {r, g, b};
  };
  if (mode == 1)
    return heat(ro.risk_raw[i]);
  if (mode == 2)
    return ro.occupancy[i] ? cvc::geometry::color_t{0.85, 0.12, 0.12}
                           : cvc::geometry::color_t{0.20, 0.22, 0.26};
  if (mode == 3) {
    double t = (hmax > hmin) ? (ro.height[i] - hmin) / (hmax - hmin) : 0.0;
    return heat(t);
  }
  const world::surface_class &sc = reg[ro.klass[i]];
  return {sc.albedo[0], sc.albedo[1], sc.albedo[2]};
}

// Build the terrain mesh straight from the raster: a vertex per cell, coloured by
// the current colour mode, real heights. Row 0 == min_y (grid_spec convention).
// The terrain mesh: real heights + UVs (world -> [0,1]) so the surface colour comes
// from a baked texture (materials + dirt + roads + plazas), not per-vertex colour.
cvc::geometry buildTerrain(cvc::app &app, const world::raster_out &ro, const world::grid_spec &g) {
  cvc::geometry geo(app);
  auto &pts = geo.points();
  auto &uvs = geo.uvs();
  pts.reserve(ro.klass.size());
  uvs.reserve(ro.klass.size());
  for (int r = 0; r < g.rows; ++r)
    for (int c = 0; c < g.cols; ++c) {
      std::size_t i = std::size_t(r) * g.cols + c;
      pts.push_back({g.world_x(c), g.world_y(r), double(ro.height[i])});
      uvs.push_back({double(c) / (g.cols - 1), double(r) / (g.rows - 1)});
    }
  auto &tris = geo.tris();
  tris.reserve(std::size_t(g.rows - 1) * (g.cols - 1) * 2);
  for (int r = 0; r < g.rows - 1; ++r)
    for (int c = 0; c < g.cols - 1; ++c) {
      idx_t v = idx_t(r) * g.cols + c;
      idx_t vr = v + 1, vd = v + g.cols, vrd = vd + 1;
      tris.push_back({v, vr, vd});
      tris.push_back({vr, vrd, vd});
    }
  return geo;
}

// ── prop tessellation (L-system structure -> merged geometry) ────────────────
void appendCylinder(cvc::geometry &g, Vec3d a, Vec3d b, double r0, double r1,
                    const cvc::geometry::color_t &col, int sides = 6) {
  Vec3d dir = b - a;
  double len = vlen(dir);
  if (len < 1e-5)
    return;
  dir = dir * (1.0 / len);
  Vec3d up = std::fabs(dir.z) < 0.9 ? Vec3d{0, 0, 1} : Vec3d{1, 0, 0};
  Vec3d u = vnorm(vcross(dir, up)), v = vcross(dir, u);
  idx_t base = idx_t(g.points().size());
  auto &P = g.points();
  auto &C = g.colors();
  for (int ring = 0; ring < 2; ++ring) {
    Vec3d ctr = ring == 0 ? a : b;
    double rr = ring == 0 ? r0 : r1;
    for (int s = 0; s < sides; ++s) {
      double th = s * 2.0 * M_PI / sides;
      Vec3d p = ctr + (u * (std::cos(th) * rr)) + (v * (std::sin(th) * rr));
      P.push_back({p.x, p.y, p.z});
      C.push_back(col);
    }
  }
  auto &T = g.tris();
  for (int s = 0; s < sides; ++s) {
    idx_t a0 = base + s, a1 = base + (s + 1) % sides;
    idx_t b0 = base + sides + s, b1 = base + sides + (s + 1) % sides;
    T.push_back({a0, a1, b1});
    T.push_back({a0, b1, b0});
  }
}

void appendBox(cvc::geometry &g, const lsys::obox &o, const cvc::geometry::color_t &col,
               double yawC, double yawS, double sx, double sy, double sz, Vec3d origin) {
  Vec3d ax[3] = {rotZ(fromLsys(o.axis[0]), yawC, yawS), rotZ(fromLsys(o.axis[1]), yawC, yawS),
                 rotZ(fromLsys(o.axis[2]), yawC, yawS)};
  Vec3d ctr = rotZ(Vec3d{o.center.x * sx, o.center.y * sy, o.center.z * sz}, yawC, yawS) + origin;
  double hx = o.half.x * sx, hy = o.half.y * sy, hz = o.half.z * sz;
  idx_t base = idx_t(g.points().size());
  auto &P = g.points();
  auto &C = g.colors();
  for (int sx = -1; sx <= 1; sx += 2)
    for (int sy = -1; sy <= 1; sy += 2)
      for (int sz = -1; sz <= 1; sz += 2) {
        Vec3d p = ctr + (ax[0] * (hx * sx)) + (ax[1] * (hy * sy)) + (ax[2] * (hz * sz));
        P.push_back({p.x, p.y, p.z});
        C.push_back(col);
      }
  // corner index = ((sx+1)/2)<<2 | ((sy+1)/2)<<1 | ((sz+1)/2)
  auto co = [&](int ix, int iy, int iz) { return base + (ix << 2 | iy << 1 | iz); };
  auto quad = [&](idx_t a, idx_t b, idx_t c, idx_t d) {
    g.tris().push_back({a, b, c});
    g.tris().push_back({a, c, d});
  };
  quad(co(0, 0, 0), co(0, 1, 0), co(0, 1, 1), co(0, 0, 1)); // -x
  quad(co(1, 0, 0), co(1, 0, 1), co(1, 1, 1), co(1, 1, 0)); // +x
  quad(co(0, 0, 0), co(0, 0, 1), co(1, 0, 1), co(1, 0, 0)); // -y
  quad(co(0, 1, 0), co(1, 1, 0), co(1, 1, 1), co(0, 1, 1)); // +y
  quad(co(0, 0, 0), co(1, 0, 0), co(1, 1, 0), co(0, 1, 0)); // -z
  quad(co(0, 0, 1), co(0, 1, 1), co(1, 1, 1), co(1, 0, 1)); // +z
}

// A leaf marker (the L-system's terminal foliage primitive) rendered as a small
// SOLID octahedron — real 3-D geometry, so the tree's own leaves fill out the
// canopy from every angle. Slightly flattened in z so a cluster reads as a leafy
// mass rather than a spiky ball.
void appendLeaf(cvc::geometry &g, Vec3d pos, Vec3d dir, double size,
                const cvc::geometry::color_t &col) {
  (void)dir;
  double r = 0.5 * size, rz = 0.38 * size;
  idx_t b = idx_t(g.points().size());
  Vec3d v[6] = {pos + Vec3d{r, 0, 0}, pos - Vec3d{r, 0, 0},  pos + Vec3d{0, r, 0},
                pos - Vec3d{0, r, 0}, pos + Vec3d{0, 0, rz}, pos - Vec3d{0, 0, rz}};
  for (const Vec3d &p : v) {
    g.points().push_back({p.x, p.y, p.z});
    g.colors().push_back(col);
  }
  const int f[8][3] = {{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4},
                       {2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5}};
  for (auto &t : f)
    g.tris().push_back({b + idx_t(t[0]), b + idx_t(t[1]), b + idx_t(t[2])});
}

cvc::geometry::color_t roleColor(const world::surface_registry &reg, lsys::role r) {
  const world::surface_class &sc = reg[reg.class_for_role(r)];
  cvc::geometry::color_t c{sc.albedo[0], sc.albedo[1], sc.albedo[2]};
  // Canopy foliage: brighten so it reads as green crown against the grass rather
  // than a dark speck (the registry's bush_cover albedo is a muted ground green).
  if (r == lsys::role::foliage)
    c = {0.19, 0.47, 0.16};
  return c;
}

// A small pool of derived+interpreted variants per recipe, so instances of one
// species are not identical. Keyed by recipe name.
struct PropLib {
  std::map<std::string, std::vector<lsys::structure>> variants;
  int nvar = 5;

  void build(const world::world_params &wp) {
    variants.clear();
    std::map<std::string, int> gen; // recipe -> generation count
    for (const auto &sw : wp.sc.trees)
      gen[sw.recipe] = wp.sc.tree_gen;
    for (const auto &sw : wp.sc.rocks)
      gen[sw.recipe] = wp.sc.rock_gen;
    for (const auto &sw : wp.sc.buildings)
      gen[sw.recipe] = wp.sc.building_gen;
    // The defaults() mix is used when a species list is empty; cover it too by
    // deriving whatever recipe a prop actually references (filled lazily below).
    seed_ = wp.seed;
    gen_ = gen;
  }
  const std::vector<lsys::structure> &get(const std::string &recipe) {
    auto it = variants.find(recipe);
    if (it != variants.end())
      return it->second;
    std::vector<lsys::structure> v;
    if (lsys::has_recipe(recipe)) {
      lsys::ruleset rs = lsys::load_recipe(recipe);
      int g = 3;
      auto gi = gen_.find(recipe);
      if (gi != gen_.end())
        g = gi->second;
      for (int k = 0; k < nvar; ++k) {
        lsys::derive_options opt;
        opt.master_seed = seed_ + std::uint64_t(k) * 0x9E3779B97F4A7C15ull;
        opt.generations = g;
        lsys::derive_result dr = lsys::derive(rs, opt);
        v.push_back(lsys::interpret(rs, dr.w));
      }
    }
    return variants.emplace(recipe, std::move(v)).first->second;
  }

private:
  std::uint64_t seed_ = 0;
  std::map<std::string, int> gen_;
};

// Tessellate every placed prop into two merged meshes: solid (trunks/branches +
// rock/building boxes, per-vertex material colour) and foliage (leaf quads).
// Render the scattered TREES and ROCKS (buildings are handled by buildBuildings).
void buildProps(cvc::app &app, const world::world_model &wm, const world::surface_registry &reg,
                PropLib &lib, cvc::geometry &solid, cvc::geometry &foliage) {
  const bool dbg = std::getenv("CVC_TL_DEBUG") != nullptr;
  const double kStem = 1.7, kLeaf = 2.2;
  std::set<std::string> buildingRecipes;
  for (const auto &sw : wm.params().sc.buildings)
    buildingRecipes.insert(sw.recipe);
  const auto &props = wm.props();
  std::map<std::string, int> dumped;
  std::size_t i = 0;
  for (const world::placed_prop &pp : props) {
    if (buildingRecipes.count(pp.recipe)) { // buildings have their own massing generator
      ++i;
      continue;
    }
    const std::vector<lsys::structure> &vs = lib.get(pp.recipe);
    if (vs.empty()) {
      ++i;
      continue;
    }
    const lsys::structure &st = vs[i % vs.size()];
    if (dbg && dumped[pp.recipe]++ == 0)
      std::fprintf(stderr, "  recipe %-16s segs=%zu leaves=%zu boxes=%zu bounds z[%.1f..%.1f]\n",
                   pp.recipe.c_str(), st.segments.size(), st.leaves.size(), st.boxes.size(),
                   st.lo.z, st.hi.z);
    double yaw = pp.yaw_deg * M_PI / 180.0, yc = std::cos(yaw), ys = std::sin(yaw);
    double sc = pp.scale;
    Vec3d org{pp.x, pp.y, pp.z};
    auto place = [&](const lsys::vec3 &p) { return rotZ(fromLsys(p) * sc, yc, ys) + org; };
    for (const lsys::segment &s : st.segments)
      appendCylinder(solid, place(s.a), place(s.b), std::max(0.06, s.r0 * sc * kStem),
                     std::max(0.04, s.r1 * sc * kStem), roleColor(reg, s.rl), s.level >= 2 ? 5 : 6);
    for (const lsys::obox &o : st.boxes)
      appendBox(solid, o, roleColor(reg, o.rl), yc, ys, sc, sc, sc, org);
    for (const lsys::leaf &lf : st.leaves)
      appendLeaf(foliage, place(lf.pos), rotZ(fromLsys(lf.dir), yc, ys),
                 std::max(1.2, lf.size * sc * kLeaf), roleColor(reg, lf.rl));
    ++i;
  }
}

// ── buildings: a per-building massing "sub-grammar" that lays out SOLID full-height
// blocks in a varied footprint (tower / slab / L / T / podium+tower), on the world's
// building sites but greedily de-conflicted so they never intersect ──────────────
std::uint64_t splitmix(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}
struct BRng {
  std::uint64_t s;
  double next() {
    s = splitmix(s);
    return double(s >> 11) * (1.0 / 9007199254740992.0);
  }
  double range(double a, double b) { return a + (b - a) * next(); }
  int pick(int n) { return int(next() * n) % n; }
};
lsys::obox groundBox(double cx, double cy, double w, double d, double h, lsys::role rl) {
  lsys::obox b;
  b.center = {cx, cy, 0.5 * h};
  b.half = {0.5 * w, 0.5 * d, 0.5 * h};
  b.axis[0] = {1, 0, 0};
  b.axis[1] = {0, 1, 0};
  b.axis[2] = {0, 0, 1};
  b.rl = rl;
  return b;
}
struct Massing {
  std::vector<lsys::obox> boxes;
  double radius = 0; // XY bounding radius (de-overlap + apron)
};
Massing genMassing(BRng &r) {
  Massing m;
  const lsys::role mats[4] = {lsys::role::wall_concrete, lsys::role::wall_brick, lsys::role::glass,
                              lsys::role::metal};
  lsys::role rl = mats[r.pick(4)];
  int shape = r.pick(5);
  double w = r.range(12, 22), d = r.range(10, 18);
  auto add = [&](double cx, double cy, double bw, double bd, double bh) {
    m.boxes.push_back(groundBox(cx, cy, bw, bd, bh, rl));
  };
  switch (shape) {
  case 0:
    add(0, 0, w, d, r.range(40, 78));
    break; // tower
  case 1:
    add(0, 0, r.range(30, 46), r.range(11, 16), r.range(22, 40));
    break;  // slab
  case 2: { // L
    double h = r.range(20, 40);
    add(-0.25 * w, 0, 0.5 * w, d, h);
    add(0.25 * w, -0.25 * d, 0.5 * w, 0.5 * d, h);
    break;
  }
  case 3: { // T
    double h = r.range(20, 42);
    add(0, 0.2 * d, w * 1.3, 0.45 * d, h);
    add(0, -0.25 * d, 0.4 * w, d, h);
    break;
  }
  default: { // podium + tower
    double pw = r.range(26, 40), pd = r.range(18, 28);
    add(0, 0, pw, pd, r.range(10, 16));
    add(r.range(-0.15, 0.15) * pw, r.range(-0.15, 0.15) * pd, r.range(12, 18), r.range(10, 16),
        r.range(38, 66));
    break;
  }
  }
  for (const lsys::obox &b : m.boxes) {
    double ex = std::fabs(b.center.x) + b.half.x, ey = std::fabs(b.center.y) + b.half.y;
    m.radius = std::max(m.radius, std::sqrt(ex * ex + ey * ey));
  }
  return m;
}
void buildBuildings(cvc::app &app, const world::world_model &wm, const world::surface_registry &reg,
                    std::uint64_t seed, cvc::geometry &solid, std::vector<Apron> &aprons) {
  (void)app;
  std::set<std::string> buildingRecipes;
  for (const auto &sw : wm.params().sc.buildings)
    buildingRecipes.insert(sw.recipe);
  std::vector<const world::placed_prop *> sites;
  for (const world::placed_prop &pp : wm.props())
    if (buildingRecipes.count(pp.recipe))
      sites.push_back(&pp);
  std::vector<Vec3d> placed; // accepted (x, y, radius) for de-overlap
  std::size_t idx = 0;
  for (const world::placed_prop *pp : sites) {
    BRng r{splitmix(seed ^ (std::uint64_t(idx + 1) * 0x100000001B3ull))};
    ++idx;
    Massing m = genMassing(r);
    bool ok = true;
    for (const Vec3d &q : placed) {
      double dx = pp->x - q.x, dy = pp->y - q.y, gap = m.radius + q.z + 8.0;
      if (dx * dx + dy * dy < gap * gap) {
        ok = false;
        break;
      }
    }
    if (!ok)
      continue;
    placed.push_back({pp->x, pp->y, m.radius});
    double yaw = pp->yaw_deg * M_PI / 180.0, yc = std::cos(yaw), ys = std::sin(yaw);
    Vec3d org{pp->x, pp->y, pp->z};
    for (const lsys::obox &b : m.boxes)
      appendBox(solid, b, roleColor(reg, b.rl), yc, ys, 1.0, 1.0, 1.0, org);
    // Plaza hugs the footprint rectangle (+ a sidewalk margin), at the building's yaw.
    double halfX = 0, halfY = 0;
    for (const lsys::obox &b : m.boxes) {
      halfX = std::max(halfX, std::fabs(b.center.x) + b.half.x);
      halfY = std::max(halfY, std::fabs(b.center.y) + b.half.y);
    }
    aprons.push_back({pp->x, pp->y, pp->z, halfX + 6.0, halfY + 6.0, yaw});
  }
}

// ── streets + terrain texture: roads and plazas are BAKED INTO the terrain's
// texture (not draped quads, which z-fight and shimmer). The mesh carries UVs; the
// texture holds the final surface colour — per-cell material (+ dirt/grass variance
// in material mode), a paved plaza hugging each building footprint, and an asphalt
// street network connecting the plazas. ─────────────────────────────────────────
struct Street {
  double x0, y0, x1, y1;
};

// Connect each building plaza to its two nearest neighbours, skipping any span
// whose midpoint terrain would sit underwater.
std::vector<Street> computeStreets(const std::vector<Apron> &aprons, const world::heightfield &hf,
                                   double waterLevel) {
  std::vector<Street> out;
  const int n = int(aprons.size());
  const double maxLen = gHalf * 0.7;
  std::set<std::pair<int, int>> edges;
  for (int i = 0; i < n; ++i) {
    int best[2] = {-1, -1};
    double bd[2] = {1e30, 1e30};
    for (int j = 0; j < n; ++j) {
      if (j == i)
        continue;
      double dx = aprons[j].x - aprons[i].x, dy = aprons[j].y - aprons[i].y, d = dx * dx + dy * dy;
      if (d < bd[0]) {
        bd[1] = bd[0];
        best[1] = best[0];
        bd[0] = d;
        best[0] = j;
      } else if (d < bd[1]) {
        bd[1] = d;
        best[1] = j;
      }
    }
    for (int k = 0; k < 2; ++k)
      if (best[k] >= 0)
        edges.insert({std::min(i, best[k]), std::max(i, best[k])});
  }
  for (const auto &e : edges) {
    const Apron &A = aprons[e.first], &B = aprons[e.second];
    double dx = B.x - A.x, dy = B.y - A.y, L = std::sqrt(dx * dx + dy * dy);
    if (L < 1.0 || L > maxLen)
      continue;
    if (hf.sample(0.5 * (A.x + B.x), 0.5 * (A.y + B.y)) < waterLevel)
      continue;
    out.push_back({A.x, A.y, B.x, B.y});
  }
  return out;
}

// A cheap self-contained 2-D value-noise fBm (the cloud fBm is defined later).
double gnoise2(double x, double y) {
  auto hsh = [](int a, int b) {
    unsigned u = unsigned(a * 374761393 + b * 668265263);
    u = (u ^ (u >> 13)) * 1274126177u;
    return ((u ^ (u >> 16)) & 0xffffffu) / double(0x1000000);
  };
  int xi = int(std::floor(x)), yi = int(std::floor(y));
  double fx = x - xi, fy = y - yi;
  fx = fx * fx * (3 - 2 * fx);
  fy = fy * fy * (3 - 2 * fy);
  double a = hsh(xi, yi), b = hsh(xi + 1, yi), c = hsh(xi, yi + 1), d = hsh(xi + 1, yi + 1);
  return (a + (b - a) * fx) * (1 - fy) + (c + (d - c) * fx) * fy;
}
double gfbm2(double x, double y) {
  double f = 0, amp = 0.5, tot = 0, fr = 1;
  for (int i = 0; i < 4; ++i) {
    f += amp * gnoise2(x * fr, y * fr);
    tot += amp;
    amp *= 0.5;
    fr *= 2.03;
  }
  return f / tot;
}

// Bake the terrain surface into an RGB texture (TEX x TEX). Row 0 == min_y (v=0),
// matching the mesh UVs. In material mode grass is broken up with dirt / dry-grass
// patches and the roads + plazas are painted on top; QA colour modes stay raw.
constexpr int TEX = 1280;
std::vector<unsigned char> paintTerrain(int mode, const world::raster_out &ro,
                                        const world::grid_spec &g,
                                        const world::surface_registry &reg,
                                        const std::vector<Apron> &aprons,
                                        const std::vector<Street> &streets) {
  const int T = TEX;
  std::vector<unsigned char> img(std::size_t(T) * T * 3, 0);
  float hmin = 1e30f, hmax = -1e30f;
  for (float h : ro.height) {
    hmin = std::min(hmin, h);
    hmax = std::max(hmax, h);
  }
  const double spanx = g.max_x - g.min_x, spany = g.max_y - g.min_y;
  const cvc::geometry::color_t dirt = {0.42, 0.34, 0.23}, dryGrass = {0.45, 0.47, 0.24};
  auto put = [&](int tx, int ty, const cvc::geometry::color_t &c) {
    std::size_t o = (std::size_t(ty) * T + tx) * 3;
    img[o] = (unsigned char)std::min(255.0, std::max(0.0, c[0] * 255.0));
    img[o + 1] = (unsigned char)std::min(255.0, std::max(0.0, c[1] * 255.0));
    img[o + 2] = (unsigned char)std::min(255.0, std::max(0.0, c[2] * 255.0));
  };
  for (int ty = 0; ty < T; ++ty) {
    double v = double(ty) / (T - 1), wy = g.min_y + v * spany;
    int r = std::min(g.rows - 1, std::max(0, int(std::lround(v * (g.rows - 1)))));
    for (int tx = 0; tx < T; ++tx) {
      double u = double(tx) / (T - 1), wx = g.min_x + u * spanx;
      int c = std::min(g.cols - 1, std::max(0, int(std::lround(u * (g.cols - 1)))));
      std::size_t i = std::size_t(r) * g.cols + c;
      cvc::geometry::color_t col = cellColor(ro, reg, i, mode, hmin, hmax);
      if (mode == 0) {
        std::uint16_t k = ro.klass[i];
        if (k == 6 || k == 7 || k == 8) { // grass / tall_grass / bush_cover
          double n1 = gfbm2(wx * 0.020, wy * 0.020), n2 = gfbm2(wx * 0.006 + 11, wy * 0.006 + 7);
          double dfac = std::min(1.0, std::max(0.0, (n1 - 0.52) / 0.30));
          double gfac = std::min(1.0, std::max(0.0, (n2 - 0.55) / 0.30));
          for (int q = 0; q < 3; ++q)
            col[q] = col[q] * (1 - dfac) + dirt[q] * dfac;
          for (int q = 0; q < 3; ++q)
            col[q] = col[q] * (1 - 0.5 * gfac) + dryGrass[q] * 0.5 * gfac;
        }
      }
      put(tx, ty, col);
    }
  }
  if (mode != 0)
    return img; // QA modes: raw raster only
  const cvc::geometry::color_t road = {0.20, 0.20, 0.23}, curb = {0.31, 0.31, 0.34};
  const double roadW = 8.0;
  double sx = (T - 1) / spanx, sy = (T - 1) / spany;
  auto W2Tx = [&](double x) { return (x - g.min_x) * sx; };
  auto W2Ty = [&](double y) { return (y - g.min_y) * sy; };
  double rTexX = 0.5 * roadW * sx;
  for (const Street &s : streets) {
    double ax = W2Tx(s.x0), ay = W2Ty(s.y0), bx = W2Tx(s.x1), by = W2Ty(s.y1);
    double dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
    int x0 = std::max(0, int(std::floor(std::min(ax, bx) - rTexX - 2)));
    int x1 = std::min(T - 1, int(std::ceil(std::max(ax, bx) + rTexX + 2)));
    int y0 = std::max(0, int(std::floor(std::min(ay, by) - rTexX - 2)));
    int y1 = std::min(T - 1, int(std::ceil(std::max(ay, by) + rTexX + 2)));
    for (int ty = y0; ty <= y1; ++ty)
      for (int tx = x0; tx <= x1; ++tx) {
        double t = L2 > 1e-9 ? ((tx - ax) * dx + (ty - ay) * dy) / L2 : 0.0;
        t = std::min(1.0, std::max(0.0, t));
        double px = ax + t * dx, py = ay + t * dy;
        double dd = std::sqrt((tx - px) * (tx - px) + (ty - py) * (ty - py));
        if (dd <= rTexX)
          put(tx, ty, road);
        else if (dd <= rTexX + 1.5)
          put(tx, ty, curb);
      }
  }
  const cvc::geometry::color_t pave = {0.44, 0.44, 0.46};
  for (const Apron &a : aprons) {
    double cx = W2Tx(a.x), cy = W2Ty(a.y);
    double hxT = a.hx * sx, hyT = a.hy * sy, R = std::sqrt(hxT * hxT + hyT * hyT);
    double c = std::cos(a.yaw), s = std::sin(a.yaw);
    int x0 = std::max(0, int(std::floor(cx - R))), x1 = std::min(T - 1, int(std::ceil(cx + R)));
    int y0 = std::max(0, int(std::floor(cy - R))), y1 = std::min(T - 1, int(std::ceil(cy + R)));
    for (int ty = y0; ty <= y1; ++ty)
      for (int tx = x0; tx <= x1; ++tx) {
        double dx = tx - cx, dy = ty - cy;
        double lx = dx * c + dy * s, ly = -dx * s + dy * c;
        if (std::fabs(lx) <= hxT && std::fabs(ly) <= hyT)
          put(tx, ty, pave);
      }
  }
  return img;
}

// ── the water: a volume of depth under a gently travelling wave, filling every
// cell that sits below the sea level (streams, ponds, coast) ──────────────────
int gSeaN = 160, gSeaNz = 22;
double gSeaFloor = -30.0, gSeaTop = 5.0;

double seaSurface(double x, double y, double t) {
  struct Wave {
    double hx, hy, len, omega, amp;
  };
  const double L = std::max(30.0, gHalf * 0.25);
  const Wave W[] = {{0.86, 0.51, L, 0.42, 1.00},
                    {-0.30, 0.95, L * 0.64, 0.73, 0.5},
                    {0.42, 0.91, L * 1.2, 0.31, 0.55}};
  double h = 0.0, peak = 0.0;
  for (const Wave &w : W) {
    double k = 2.0 * M_PI / w.len;
    double s = 0.5 + 0.5 * std::sin(k * (w.hx * x + w.hy * y) - w.omega * t);
    h += w.amp * std::pow(s, 2.2);
    peak = std::max(peak, w.amp);
  }
  double crest = std::min(1.0, h / (peak > 1e-9 ? peak : 1.0));
  return gSeaLevel + 0.9 * (crest - 0.7);
}

std::vector<float> seaField(double t) {
  std::vector<float> f(std::size_t(gSeaN) * gSeaN * gSeaNz, 0.0f);
  for (int k = 0; k < gSeaNz; ++k) {
    double z = gSeaFloor + (gSeaTop - gSeaFloor) * k / (gSeaNz - 1);
    for (int j = 0; j < gSeaN; ++j) {
      double y = -gHalf + 2.0 * gHalf * j / (gSeaN - 1);
      for (int i = 0; i < gSeaN; ++i) {
        double x = -gHalf + 2.0 * gHalf * i / (gSeaN - 1);
        double surf = seaSurface(x, y, t);
        double below = surf - z, above = z - terrainH(x, y);
        double depth = (below > 0.0 && above > 0.0) ? std::min(1.0, below / 5.0) : 0.0;
        f[std::size_t(k) * gSeaN * gSeaN + j * gSeaN + i] = float(depth);
      }
    }
  }
  return f;
}
void seaTransfer(std::vector<double> &color, std::vector<double> &opacity, double t) {
  double k = 0.0110 + 0.0016 * std::sin(t * 0.9);
  color = {0.00, 0.40, 0.74, 0.72, 0.25, 0.13, 0.52, 0.64,
           0.60, 0.05, 0.24, 0.44, 1.00, 0.02, 0.10, 0.24};
  opacity = {0.00, 0.0, 0.14, k * 0.5, 0.55, k, 1.00, k * 2.2};
}
cvc::volume seaVolume(cvc::app &app, const std::vector<float> &field) {
  return cvc::volume(app, reinterpret_cast<const unsigned char *>(field.data()),
                     cvc::dimension(gSeaN, gSeaN, gSeaNz), cvc::Float,
                     cvc::bounding_box(-gHalf, -gHalf, gSeaFloor, gHalf, gHalf, gSeaTop));
}

// ── the sky: a drifting cloud slab (fixed 60^3 grid, world-scaled extent) ─────
constexpr int SKY_N = 60, SKY_NZ = 26;
double gSkyHalf = 640.0, gSkyBase = 220.0, gSkyTop = 360.0;
constexpr int CLOUD_MAPS = 2, CLOUD_DEPTH = 6;
constexpr double CLOUD_TURN = 32.0, CLOUD_STEP0 = 8.1, CLOUD_STEP_DECAY = 0.9;
constexpr double CLOUD_PUFF0 = 8.8, CLOUD_PUFF_DECAY = 0.88;
constexpr double CLOUD_FLOOR = 0.10;
const char *CLOUD_AXIOM = "[A][+++++A][-----A][++++++++++A][----------A][+++++++++++++++A]";
const char *cloudRule(char c) {
  switch (c) {
  case 'A':
    return "FF[+<B]^F[-<C]<F[+<C]vFA";
  case 'B':
    return "F[+<F]F<[-<F]vB";
  case 'C':
    return "^<F[+<F][-<F]^<FC";
  default:
    return nullptr;
  }
}
inline std::size_t skyIdx(int z, int y, int x) { return (std::size_t(z) * SKY_N + y) * SKY_N + x; }
double percentileSorted(const std::vector<float> &a, double q) {
  if (a.empty())
    return 0.0;
  double rank = (q / 100.0) * (a.size() - 1);
  std::size_t lo = std::size_t(std::floor(rank));
  if (lo + 1 >= a.size())
    return a.back();
  return a[lo] + (rank - lo) * (a[lo + 1] - a[lo]);
}
double vhash3(int x, int y, int z) {
  unsigned h = unsigned(x * 374761393 + y * 668265263 + z * 1274126177);
  h = (h ^ (h >> 13)) * 1274126177u;
  return ((h ^ (h >> 16)) & 0xffffffu) / double(0x1000000);
}
double vnoise3(double x, double y, double z) {
  int xi = int(std::floor(x)), yi = int(std::floor(y)), zi = int(std::floor(z));
  double fx = x - xi, fy = y - yi, fz = z - zi;
  auto sm = [](double t) { return t * t * (3.0 - 2.0 * t); };
  fx = sm(fx);
  fy = sm(fy);
  fz = sm(fz);
  auto L = [](double a, double b, double t) { return a + (b - a) * t; };
  double x00 = L(vhash3(xi, yi, zi), vhash3(xi + 1, yi, zi), fx);
  double x10 = L(vhash3(xi, yi + 1, zi), vhash3(xi + 1, yi + 1, zi), fx);
  double x01 = L(vhash3(xi, yi, zi + 1), vhash3(xi + 1, yi, zi + 1), fx);
  double x11 = L(vhash3(xi, yi + 1, zi + 1), vhash3(xi + 1, yi + 1, zi + 1), fx);
  return L(L(x00, x10, fy), L(x01, x11, fy), fz);
}
double fbm3(double x, double y, double z, int octaves) {
  double f = 0.0, amp = 0.5, tot = 0.0, fr = 1.0;
  for (int i = 0; i < octaves; ++i) {
    f += amp * vnoise3(x * fr, y * fr, z * fr);
    tot += amp;
    amp *= 0.5;
    fr *= 2.02;
  }
  return f / tot;
}
std::vector<float> walkClouds(std::mt19937 &rng) {
  std::uniform_real_distribution<double> U(0.0, 1.0);
  auto uni = [&](double a, double b) { return a + (b - a) * U(rng); };
  const int N = SKY_N, NZ = SKY_NZ;
  std::vector<float> field(std::size_t(NZ) * N * N, 0.0f);
  const double zscale = (double(N) / NZ) * ((gSkyTop - gSkyBase) / (2.0 * gSkyHalf));
  double x = uni(0.25, 0.75) * N, y = uni(0.3, 0.7) * N, z = NZ * 0.42;
  double head = uni(0.0, 360.0);
  double step = CLOUD_STEP0, puff = CLOUD_PUFF0, climb = 0.0;
  int depth = 0;
  struct St {
    double x, y, z, head, step, puff, climb;
    int depth;
  };
  std::vector<St> stack;
  std::deque<char> todo(CLOUD_AXIOM, CLOUD_AXIOM + std::strlen(CLOUD_AXIOM));
  int guard = 0;
  while (!todo.empty() && guard < 4000) {
    ++guard;
    char c = todo.front();
    todo.pop_front();
    if (c == 'F') {
      x = std::fmod(x + step * std::cos(head * M_PI / 180.0), double(N));
      if (x < 0)
        x += N;
      y = std::min(std::max(y + step * std::sin(head * M_PI / 180.0), 0.0), double(N - 1));
      z = std::min(std::max(z + climb, 1.0), double(NZ - 2));
      const double p2 = 2.0 * puff * puff;
      for (int gz = 0; gz < NZ; ++gz) {
        double dz = (gz - z) * zscale, dz2 = dz * dz;
        for (int gy = 0; gy < N; ++gy) {
          double dy = gy - y, dyz2 = dy * dy + dz2;
          for (int gx = 0; gx < N; ++gx) {
            double dx = std::fabs(gx - x);
            dx = std::min(dx, double(N) - dx);
            field[skyIdx(gz, gy, gx)] += float(std::exp(-(dx * dx + dyz2) / p2));
          }
        }
      }
    } else if (c == '+')
      head += CLOUD_TURN;
    else if (c == '-')
      head -= CLOUD_TURN;
    else if (c == '<')
      puff *= CLOUD_PUFF_DECAY;
    else if (c == '^')
      climb += 0.55;
    else if (c == 'v')
      climb -= 0.45;
    else if (c == '[')
      stack.push_back({x, y, z, head, step, puff, climb, depth});
    else if (c == ']') {
      if (!stack.empty()) {
        St s = stack.back();
        stack.pop_back();
        x = s.x;
        y = s.y;
        z = s.z;
        head = s.head;
        step = s.step;
        puff = s.puff;
        climb = s.climb;
        depth = s.depth;
      }
    } else if (cloudRule(c) && depth < CLOUD_DEPTH) {
      ++depth;
      step *= CLOUD_STEP_DECAY;
      const char *r = cloudRule(c);
      todo.insert(todo.begin(), r, r + std::strlen(r));
    }
  }
  std::vector<float> sorted(field);
  std::sort(sorted.begin(), sorted.end());
  double m = percentileSorted(sorted, 99.9);
  if (m > 0)
    for (float &v : field)
      v = std::min(1.0f, std::max(0.0f, v / float(m)));
  std::vector<double> w(N), zf(NZ);
  for (int i = 0; i < N; ++i)
    w[i] = std::sqrt(0.5 - 0.5 * std::cos(2.0 * M_PI * i / (N - 1)));
  for (int k = 0; k < NZ; ++k)
    zf[k] = std::sin(0.12 + (M_PI - 0.24) * k / (NZ - 1));
  for (int gz = 0; gz < NZ; ++gz)
    for (int gy = 0; gy < N; ++gy)
      for (int gx = 0; gx < N; ++gx) {
        float &v = field[skyIdx(gz, gy, gx)];
        v *= float(std::min(w[gy], w[gx]) * zf[gz]);
        if (v > 0.0f) {
          double d = fbm3(gx * 0.30, gy * 0.30, gz * 0.62, 5);
          v = float(std::min(1.0, std::max(0.0, v * (0.32 + 1.5 * d))));
        }
      }
  return field;
}
struct SkyModel {
  std::vector<std::vector<float>> maps;
  double norm = 1.0;
  std::vector<float> raw(double shift, double morph) const {
    const int N = SKY_N, NZ = SKY_NZ;
    long mi = long(std::floor(morph));
    int i = int(((mi % CLOUD_MAPS) + CLOUD_MAPS) % CLOUD_MAPS);
    int j = (i + 1) % CLOUD_MAPS;
    double u = morph - std::floor(morph);
    u = u * u * (3.0 - 2.0 * u);
    long ks = long(std::floor(shift));
    double f = shift - ks;
    int k0 = int(((ks % N) + N) % N), k1 = (k0 + 1) % N;
    const std::vector<float> &A = maps[i], &B = maps[j];
    std::vector<float> out(std::size_t(NZ) * N * N);
    for (int z = 0; z < NZ; ++z)
      for (int y = 0; y < N; ++y)
        for (int x = 0; x < N; ++x) {
          int sx0 = ((x - k0) % N + N) % N, sx1 = ((x - k1) % N + N) % N;
          auto mix = [&](int xx) {
            return (1.0 - u) * A[skyIdx(z, y, xx)] + u * B[skyIdx(z, y, xx)];
          };
          double vol = (1.0 - f) * mix(sx0) + f * mix(sx1);
          double lump = std::min(1.0, std::max(0.0, (vol - CLOUD_FLOOR) / (1.0 - CLOUD_FLOOR)));
          out[skyIdx(z, y, x)] = float(lump * lump);
        }
    return out;
  }
  std::vector<float> field(double shift, double morph) const {
    std::vector<float> r = raw(shift, morph);
    double inv = 1.0 / norm;
    for (float &v : r)
      v = float(v * inv);
    return r;
  }
};
SkyModel buildSky() {
  SkyModel sky;
  std::mt19937 rng(20u);
  for (int m = 0; m < CLOUD_MAPS; ++m)
    sky.maps.push_back(walkClouds(rng));
  double norm = 0.0;
  for (int c = 0; c < SKY_N; c += 8)
    for (double mo : {0.0, 0.5, 1.0}) {
      std::vector<float> r = sky.raw(double(c), mo);
      for (float v : r)
        norm = std::max(norm, double(v));
    }
  sky.norm = norm > 0 ? norm : 1.0;
  return sky;
}
cvc::volume skyVolume(cvc::app &app, const std::vector<float> &field) {
  return cvc::volume(
      app, reinterpret_cast<const unsigned char *>(field.data()),
      cvc::dimension(SKY_N, SKY_N, SKY_NZ), cvc::Float,
      cvc::bounding_box(-gSkyHalf, -gSkyHalf, gSkyBase, gSkyHalf, gSkyHalf, gSkyTop));
}
void skyTransfer(std::vector<double> &color, std::vector<double> &opacity) {
  color = {0.0, 0.80, 0.85, 0.93, 0.45, 0.94, 0.96, 0.98, 1.0, 1.00, 1.00, 1.00};
  opacity = {0.0, 0.0, 0.22, 0.0, 0.55, 0.26, 1.0, 0.54};
}

// ── the sun (a flat-lit disc + halo, camera-relative, at infinity) ───────────
double SUN_AZ = -52.0, SUN_EL = 36.0;
Vec3d sunDir(double azDeg, double elDeg) {
  double az = azDeg * M_PI / 180.0, el = elDeg * M_PI / 180.0;
  return {std::cos(el) * std::sin(az), -std::cos(el) * std::cos(az), std::sin(el)};
}
cvc::geometry discGeom(cvc::app &app, Vec3d c, Vec3d normal, double radius, int seg = 48) {
  Vec3d n = vnorm(normal);
  Vec3d up = std::fabs(n.z) < 0.9 ? Vec3d{0, 0, 1} : Vec3d{1, 0, 0};
  Vec3d u = vnorm(vcross(n, up)), v = vcross(n, u);
  cvc::geometry g(app);
  g.points().push_back({c.x, c.y, c.z});
  for (int i = 0; i < seg; ++i) {
    double th = i * 2.0 * M_PI / seg;
    Vec3d p = c + (u * (radius * std::cos(th))) + (v * (radius * std::sin(th)));
    g.points().push_back({p.x, p.y, p.z});
  }
  for (int i = 0; i < seg; ++i)
    g.tris().push_back({0, idx_t(1 + i), idx_t(1 + (i + 1) % seg)});
  return g;
}

} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  bool offscreen = false, noShadows = false, verbose = false, no_ui = false, show_ui = false;
  int frames = 0, width = 1280, height = 800;
  double fps = 30.0, half = 600.0;
  unsigned long seed = 7;
  std::string png, captureStr = "none", outDir = "frames", preset = "standard";
  po::options_description desc("terrain_lab — a pure-C++ cvcGL viewer for cvc::world\nOptions");
  desc.add_options()("help,h", "show this help and exit")                                     //
      ("offscreen", po::bool_switch(&offscreen), "render offscreen (no window)")              //
      ("no-shadows", po::bool_switch(&noShadows), "disable prop shadows")                     //
      ("verbose,v", po::bool_switch(&verbose), "show cvcGL debug logging")                    //
      ("no-ui", po::bool_switch(&no_ui), "hide the ImGui overlay")                            //
      ("show-ui", po::bool_switch(&show_ui), "force the ImGui overlay on (even offscreen)")   //
      ("frames", po::value<int>(&frames)->default_value(0), "stop after N frames (0 = live)") //
      ("png", po::value<std::string>(&png)->default_value(""), "write final frame to PNG")    //
      ("capture", po::value<std::string>(&captureStr)->default_value("none"),                 //
       "cinematic capture: none | orbit | fly (forces --offscreen)")                          //
      ("width", po::value<int>(&width)->default_value(1280), "render width")                  //
      ("height", po::value<int>(&height)->default_value(800), "render height")                //
      ("fps", po::value<double>(&fps)->default_value(30.0), "capture fps (synthetic clock)")  //
      ("out", po::value<std::string>(&outDir)->default_value("frames"), "capture output dir") //
      ("half", po::value<double>(&half)->default_value(600.0), "world half-extent (metres)")  //
      ("seed", po::value<unsigned long>(&seed)->default_value(7), "generation seed")          //
      ("preset", po::value<std::string>(&preset)->default_value("standard"),
       "standard | forest | urban | sparse | island");
  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (const std::exception &e) {
    std::cerr << "error: " << e.what() << "\n\n" << desc << "\n";
    return 2;
  }
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  enum class Capture { None, Orbit, Fly } capture = Capture::None;
  if (captureStr == "orbit")
    capture = Capture::Orbit;
  else if (captureStr == "fly")
    capture = Capture::Fly;
  else if (captureStr != "none") {
    std::cerr << "unknown --capture '" << captureStr << "'\n";
    return 2;
  }
  const bool capturing = capture != Capture::None;
  if (capturing) {
    offscreen = true;
    std::error_code ec;
    std::filesystem::create_directories(outDir, ec);
  }

  gHalf = half;
  // Water/cloud extents follow the world size.
  gSeaN = std::max(80, std::min(160, int(std::lround(2.0 * gHalf / 8.0))));
  gSkyHalf = gHalf * 1.06;

  GenParams gp;
  gp.seed = seed;
  for (int i = 0; i < 5; ++i)
    if (preset == kPresetNames[i])
      gp.preset = i;

  cvc::app app;
  app.properties("system.log_verbosity", verbose ? "6" : "2");
  SceneGraph sg(app, "terrain_lab");
  const world::surface_registry *reg =
      &world::surface_registry::variant(kOntologyNames[gp.ontology]);

  int colorMode = 0; // material
  PropLib lib;

  // Build (or rebuild) the whole world into the scene. Called once up front and on
  // every ImGui "Regenerate". Keeps the same node names so nodes are reused.
  world::world_model wm = world::world_model::generate(toWorldParams(gp));
  world::grid_spec grid;
  world::raster_out ro;
  auto rasterize = [&]() {
    grid.rows = grid.cols = TERRAIN_N;
    grid.min_x = grid.min_y = -gHalf;
    grid.max_x = grid.max_y = gHalf;
    world::raster(wm, grid, ro);
  };

  std::vector<Apron> gAprons; // current buildings' plazas (for the baked texture)
  std::vector<Street> gStreets;
  // Paint the terrain texture (material + dirt + roads + plazas) for the current
  // colour mode and hand it to the terrain node — on rebuild and on colour switch.
  auto applyTexture = [&]() {
    if (!sg.hasGraphics("terrain"))
      return;
    std::vector<unsigned char> rgb = paintTerrain(colorMode, ro, grid, *reg, gAprons, gStreets);
    cvc::image img(TEX, TEX, cvc::image::pixel_format::RGB, cvc::image::data_type::u8, rgb.data());
    std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("terrain"))->setTexture(img, false);
  };

  auto rebuild = [&](bool reframe) {
    world::world_params wp = toWorldParams(gp);
    reg = &world::surface_registry::variant(wp.ontology);
    gSeaLevel = gp.water_level_m;
    wm = world::world_model::generate(wp);
    gHF = &wm.hf();
    rasterize();
    // sea vertical extent from the actual relief
    float hmin = 1e30f, hmax = -1e30f;
    for (float h : ro.height) {
      hmin = std::min(hmin, h);
      hmax = std::max(hmax, h);
    }
    gSeaFloor = std::min(double(hmin) - 2.0, gSeaLevel - 6.0);
    gSeaTop = gSeaLevel + 4.0;
    gSkyBase = double(hmax) + gHalf * 0.35 + 60.0;
    gSkyTop = gSkyBase + gHalf * 0.30 + 80.0;

    lib.build(wp);
    cvc::geometry terrain = buildTerrain(app, ro, grid);
    cvc::geometry solid(app), foliage(app);
    gAprons.clear();
    buildProps(app, wm, *reg, lib, solid, foliage);         // trees + rocks
    buildBuildings(app, wm, *reg, gp.seed, solid, gAprons); // solid, non-overlapping edifices
    gStreets = computeStreets(gAprons, wm.hf(), gSeaLevel); // baked into the terrain texture

    if (sg.hasGraphics("terrain"))
      std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("terrain"))->setGeometry(terrain);
    else {
      auto t = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("terrain", terrain));
      t->setUseSingleColor(true); // surface colour comes from the baked texture
      t->setColor(1.0, 1.0, 1.0);
      t->setAmbient(0.5);
      t->setDiffuse(0.95);
    }
    applyTexture(); // material + dirt + roads + plazas, per the colour mode

    if (sg.hasGraphics("props"))
      std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("props"))->setGeometry(solid);
    else {
      auto p = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("props", solid));
      p->setUseSingleColor(false);
      p->setAmbient(0.45);
      p->setDiffuse(0.85);
    }
    if (foliage.points().size()) {
      if (sg.hasGraphics("foliage"))
        std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("foliage"))->setGeometry(foliage);
      else {
        auto fol = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("foliage", foliage));
        fol->setUseSingleColor(false);
        fol->setAmbient(0.6);
        fol->setDiffuse(0.7);
      }
    } else if (sg.hasGraphics("foliage"))
      sg.removeGraphics("foliage");

    // water volume
    if (sg.hasGraphics("water"))
      std::dynamic_pointer_cast<VolumeNode>(sg.getGraphics("water"))
          ->setVolume(seaVolume(app, seaField(0.0)));
    else {
      auto w = sg.addGraphics("water", seaVolume(app, seaField(0.0)));
      std::vector<double> col, op;
      seaTransfer(col, op, 0.0);
      w->setTransferFunction(col, op);
      w->setShading(true);
      w->setAmbient(0.18);
      w->setDiffuse(0.72);
      w->setSpecular(0.85);
      w->setSpecularPower(70.0);
    }
    std::printf("terrain_lab: seed=%llu preset=%s trees=%d rocks=%d buildings=%d props=%zu "
                "occupied=%.2f%%\n",
                (unsigned long long)gp.seed, kPresetNames[gp.preset], gp.trees, gp.rocks,
                gp.buildings, wm.props().size(),
                100.0 * double(ro.occupied_count()) / std::max<std::size_t>(1, ro.klass.size()));
    (void)reframe;
  };

  // Recolour the terrain (colour-mode switch) — repaint the texture, no regeneration.
  auto recolor = [&]() { applyTexture(); };

  gHF = &wm.hf();
  gSeaLevel = gp.water_level_m;
  rebuild(true);

  // Stage lighting (a rig aimed at the world, not a sky).
  cvc::gl::StageLighting rig(sg);
  rig.setStage(0.0, 0.0, 0.0, gHalf);
  rig.applyPreset(cvc::gl::StageLighting::Preset::ThreePoint);
  rig.setKey(1.15, SUN_AZ, SUN_EL, gHalf * 0.3);
  rig.setWarmth(0.4);

  SceneRenderer view(sg, width, height, offscreen, "main");
  view.renderer()->GradientBackgroundOn();
  view.renderer()->SetBackground(0.66, 0.71, 0.74);
  view.renderer()->SetBackground2(0.23, 0.44, 0.80);

  const bool shadows = !noShadows && sg.setShadowsEnabled(true);
  if (shadows) {
    sg.setShadowResolution(2048);
    sg.setShadowUpdateInterval(capturing ? 1 : 8);
  }

  CameraController cam(view);
  cvc::bounding_box b = sg.computeGraphicsBounds();
  cam.frameBounds(b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz);
  cvc::gl::FpsHud hud(view);
  cvc::gl::TouchGestures touch(view, cam);

  bool wantRegen = false;

#ifdef CVC_ENABLE_IMGUI
  cvc::gl::ImGuiOverlay ui(view);
  ImGui::SetCurrentContext(ui.imguiContext());
  ui.attachCamera(cam);
  ui.setVisible(show_ui || (!no_ui && !capturing && !offscreen));
  bool uiScene = false, uiLighting = false;
  ui.setDrawCallback([&] {
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("Scene")) {
        cvc::gl::ui::SceneMenuItems(sg, &uiScene, &uiLighting);
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }
    ImGui::SetNextWindowPos(ImVec2(12, 34), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(320, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("Terrain Lab");
    ImGui::TextWrapped("Live view of cvc::world - the L-system terrain source.");
    ImGui::Separator();

    int seedI = int(gp.seed);
    if (ImGui::InputInt("seed", &seedI)) {
      gp.seed = std::uint64_t(std::max(0, seedI));
    }
    if (ImGui::Combo("preset", &gp.preset, kPresetNames, IM_ARRAYSIZE(kPresetNames))) {
      presetFill(gp); // quick-set the sliders for this preset
      wantRegen = true;
    }
    ImGui::Combo("ontology", &gp.ontology, kOntologyNames, IM_ARRAYSIZE(kOntologyNames));
    ImGui::SliderInt("trees", &gp.trees, 0, 3000);
    ImGui::SliderInt("rocks", &gp.rocks, 0, 600);
    ImGui::SliderInt("buildings", &gp.buildings, 0, 600);
    ImGui::SliderInt("tree gens", &gp.tree_gen, 2, 8);
    { // amp / sea sliders (double -> float bridge)
      float amp = float(gp.amp_m), sea = float(gp.water_level_m);
      if (ImGui::SliderFloat("relief amp m", &amp, 2.0f, 80.0f))
        gp.amp_m = amp;
      if (ImGui::SliderFloat("water level m", &sea, 0.0f, 40.0f))
        gp.water_level_m = sea;
    }
    if (ImGui::Button("Regenerate"))
      wantRegen = true;
    ImGui::SameLine();
    if (ImGui::Button("Randomize")) {
      gp.seed = gp.seed * 6364136223846793005ull + 1442695040888963407ull;
      wantRegen = true;
    }
    ImGui::Separator();
    if (ImGui::Combo("colour by", &colorMode, kColorModeNames, IM_ARRAYSIZE(kColorModeNames)))
      recolor();
    ImGui::Text("%zu props  |  %.1f%% occupied", wm.props().size(),
                100.0 * double(ro.occupied_count()) / std::max<std::size_t>(1, ro.klass.size()));
    ImGui::End();

    cvc::gl::ui::ScenePanel(sg, &uiScene);
    cvc::gl::ui::StageLightingPanel(rig, &uiLighting);
  });
#endif
  if (offscreen)
    hud.setEnabled(false);

  // The sun billboard (added after framing so it doesn't inflate the bounds).
  Vec3d sd = sunDir(SUN_AZ, SUN_EL);
  Vec3d face{-sd.x, -sd.y, -sd.z};
  double sunR = gHalf * 0.10, sunRef = gHalf * 4.0, sunCap = gHalf * 1.1;
  sg.addGraphics("sun", discGeom(app, {0, 0, 0}, face, sunR));
  {
    auto disc = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("sun"));
    disc->setColor(1.0, 0.97, 0.88);
    disc->setAmbient(1.0);
    disc->setDiffuse(0.0);
    disc->setOpacity(0.99);
  }
  sg.addGraphics("sun_halo", discGeom(app, {0, 0, 0}, face, sunR * 3.2));
  {
    auto halo = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("sun_halo"));
    halo->setColor(1.0, 0.90, 0.72);
    halo->setAmbient(1.0);
    halo->setDiffuse(0.0);
    halo->setOpacity(0.22);
  }
  auto sunNode = sg.getGraphics("sun");
  auto sunHalo = sg.getGraphics("sun_halo");
  auto placeSky = [&](const Vec3d &eye) {
    double eDotS = eye.x * sd.x + eye.y * sd.y + eye.z * sd.z;
    double dd = std::min(sunRef, sunCap - eDotS);
    bool vis = dd > 1.0;
    sunNode->setVisible(vis);
    sunHalo->setVisible(vis);
    if (!vis)
      return;
    double s = dd / sunRef;
    sunNode->setScale(s, s, s);
    sunHalo->setScale(s, s, s);
    Vec3d p = eye + sd * dd;
    sunNode->setPosition(p.x, p.y, p.z);
    sunHalo->setPosition(p.x, p.y, p.z);
  };

  // The sky slab (added after framing too — it is scenery, not a framing target).
  SkyModel sky = buildSky();
  auto skyNode = sg.addGraphics("sky", skyVolume(app, sky.field(0.0, 0.0)));
  skyNode->setShading(false);
  skyNode->setAmbient(0.95);
  skyNode->setDiffuse(0.35);
  skyNode->setSpecular(0.0);
  skyNode->setVolumetricScattering(0.0);
  {
    std::vector<double> col, op;
    skyTransfer(col, op);
    skyNode->setTransferFunction(col, op);
  }

  sg.setDiagnosticChromeVisible(false);
  sg.processEvents();

  std::printf("terrain_lab: %s, world %.0f m, %s. Tab=orbit/fly, WASD+mouse=fly.\n",
              offscreen ? "offscreen" : "onscreen", 2 * gHalf,
              shadows ? "shadows on" : "no shadows");

  const std::vector<Vec3d> flyEye = {{-gHalf * 1.5, -gHalf * 1.3, gHalf * 0.4},
                                     {-gHalf * 0.6, gHalf * 0.2, gHalf * 0.35},
                                     {gHalf * 0.5, gHalf * 0.6, gHalf * 0.5},
                                     {gHalf * 1.1, -gHalf * 0.3, gHalf * 0.7}};
  const std::vector<Vec3d> flyTgt(4, {0, 0, gHalf * 0.1});
  auto crInterp = [](const std::vector<Vec3d> &p, double s) {
    int m = int(p.size());
    double x = s * (m - 1);
    int i = std::min(int(std::floor(x)), m - 2);
    if (i < 0)
      i = 0;
    double u = x - i, u2 = u * u, u3 = u2 * u;
    auto seg = [&](double a, double b, double c, double d) {
      return 0.5 * (2 * b + (-a + c) * u + (2 * a - 5 * b + 4 * c - d) * u2 +
                    (-a + 3 * b - 3 * c + d) * u3);
    };
    const Vec3d &p0 = p[std::max(i - 1, 0)], &p1 = p[i], &p2 = p[i + 1],
                &p3 = p[std::min(i + 2, m - 1)];
    return Vec3d{seg(p0.x, p1.x, p2.x, p3.x), seg(p0.y, p1.y, p2.y, p3.y),
                 seg(p0.z, p1.z, p2.z, p3.z)};
  };
  const std::string azPath =
      CameraController::viewerStatePath("terrain_lab", "main") + ".orbit.azimuth";
  double orbitAz0 = 0.0;
  if (capture == Capture::Orbit) {
    try {
      orbitAz0 = cvc::state::instance(app)(azPath).value<double>();
    } catch (...) {
    }
  }

  auto start = std::chrono::steady_clock::now();
  double last = 0.0;
  long frame = 0;
  int n = 0;
  const int SEA_STRIDE = 4, CLOUD_STRIDE = 8;
  while (!view.windowClosed()) {
    double t, dt;
    if (capturing) {
      t = frame / fps;
      dt = 1.0 / fps;
    } else {
      t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      dt = t - last;
      last = t;
    }
    view.processUIEvents();
    touch.update();
    if (wantRegen) {
      wantRegen = false;
      rebuild(false);
    }
    const bool seaDue = capturing || frame % SEA_STRIDE == 0;
    const bool cloudDue = capturing || frame % CLOUD_STRIDE == 1;
    if (seaDue && sg.hasGraphics("water")) {
      auto w = std::dynamic_pointer_cast<VolumeNode>(sg.getGraphics("water"));
      w->updateScalars(seaField(t));
      std::vector<double> col, op;
      seaTransfer(col, op, t);
      w->setTransferFunction(col, op);
    }
    if (cloudDue) {
      double shift = t * (gHalf * 0.010) * SKY_N / (2.0 * gSkyHalf);
      double morph = t / 60.0 * CLOUD_MAPS;
      skyNode->updateScalars(sky.field(shift, morph));
    }
    if (capture == Capture::Fly) {
      double s = frames > 1 ? double(frame) / (frames - 1) : 0.0;
      s = s * s * (3.0 - 2.0 * s);
      Vec3d e = crInterp(flyEye, s), tg = crInterp(flyTgt, s);
      view.setCamera(e.x, e.y, e.z, tg.x, tg.y, tg.z, 0, 0, 1, 42.0, 1.0, gHalf * 12.0);
      placeSky(e);
    } else {
      if (capture == Capture::Orbit && frames > 0)
        cvc::state::instance(app)(azPath).value(orbitAz0 + 360.0 * double(frame) / frames);
      cam.update(dt);
      double ep[3], fp[3], upv[3];
      cam.getPose(ep, fp, upv);
      placeSky({ep[0], ep[1], ep[2]});
      view.renderer()->ResetCameraClippingRange();
    }
    if (capturing) {
      char path[1024];
      std::snprintf(path, sizeof path, "%s/frame_%05ld.png", outDir.c_str(), frame);
      view.writePNG(path);
    } else {
      view.render();
    }
#ifdef __EMSCRIPTEN__
#ifndef __EMSCRIPTEN_PTHREADS__
    sg.publisher().flush();
#endif
    emscripten_sleep(0);
#endif
    ++frame;
    if (frames > 0 && ++n >= frames)
      break;
  }
  if (!png.empty())
    view.writePNG(png);
  hud.detach();
  cam.detach();
  return 0;
}
