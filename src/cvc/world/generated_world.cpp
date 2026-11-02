/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// generated_world.cpp — terrain_lab's world (see generated_world.h). Moved out of
// examples/cvcgl/terrain_lab.cpp so the native lab, the generated_world Ariadne node type and a
// headless consumer build the same world.

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <algorithm>
#include <cmath>
#include <cvc/lsys/derive.h>
#include <cvc/lsys/forest.h> // the preserved demo tree generators (grow_pine / grow_branchy)
#include <cvc/lsys/grammar.h>
#include <cvc/lsys/interp.h>
#include <cvc/lsys/recipes.h>
#include <cvc/lsys/rng.h> // hashed per-tree draws (uni/irand/cell_id)
#include <cvc/world/generated_world.h>
#include <cvc/world/heightfield.h>
#include <cvc/world/road_mesh.h>
#include <cvc/world/road_raster.h>
#include <cvc/world/roads.h>
#include <cvc/world/scatter.h>
#include <cvc/world/sites.h>
#include <cvc/world/surface.h>
#include <map>
#include <set>
#include <utility>

namespace cvc {
namespace world {
namespace {

namespace lsys = cvc::lsys;
using idx_t = cvc::geometry::index_t;

const char *const kPresetNames[] = {"standard", "forest", "urban", "sparse", "island"};
const char *const kOntologyNames[] = {"merged_default", "soft_vegetation", "strict_water_mud"};

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
// Rotate about +Z by yaw (radians).
Vec3d rotZ(Vec3d p, double c, double s) { return {p.x * c - p.y * s, p.x * s + p.y * c, p.z}; }

// ── raster cell colour ───────────────────────────────────────────────────────
cvc::geometry::color_t heat(double t) { // blue -> green -> yellow -> red over [0, 1]
  t = std::min(1.0, std::max(0.0, t));
  double r = std::min(1.0, std::max(0.0, 1.5 - std::fabs(4.0 * t - 3.0)));
  double g = std::min(1.0, std::max(0.0, 1.5 - std::fabs(4.0 * t - 2.0)));
  double b = std::min(1.0, std::max(0.0, 1.5 - std::fabs(4.0 * t - 1.0)));
  return {r, g, b};
}

cvc::geometry::color_t cellColor(const raster_out &ro, const surface_registry &reg, std::size_t i,
                                 int mode, float hmin, float hmax) {
  if (mode == 1)
    return heat(ro.risk_raw[i]);
  if (mode == 2)
    return ro.occupancy[i] ? cvc::geometry::color_t{0.85, 0.12, 0.12}
                           : cvc::geometry::color_t{0.20, 0.22, 0.26};
  if (mode == 3) {
    double t = (hmax > hmin) ? (ro.height[i] - hmin) / (hmax - hmin) : 0.0;
    return heat(t);
  }
  const surface_class &sc = reg[ro.klass[i]];
  return {sc.albedo[0], sc.albedo[1], sc.albedo[2]};
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

// A leaf marker (the L-system's terminal foliage primitive) as a small SOLID octahedron, slightly
// flattened in z so a cluster reads as a leafy mass rather than a spiky ball.
void appendLeaf(cvc::geometry &g, Vec3d pos, double size, const cvc::geometry::color_t &col) {
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

cvc::geometry::color_t roleColor(const surface_registry &reg, lsys::role r) {
  const surface_class &sc = reg[reg.class_for_role(r)];
  cvc::geometry::color_t c{sc.albedo[0], sc.albedo[1], sc.albedo[2]};
  // Canopy foliage: brighter, so it reads as green crown against the grass rather than a dark
  // speck (the registry's bush_cover albedo is a muted ground green).
  if (r == lsys::role::foliage)
    c = {0.19, 0.47, 0.16};
  return c;
}

// A small pool of derived+interpreted variants per recipe, so instances of one species are not
// identical. Keyed by recipe name.
struct PropLib {
  std::map<std::string, std::vector<lsys::structure>> variants;
  int nvar = 5;

  void build(const world_params &wp) {
    variants.clear();
    std::map<std::string, int> gen; // recipe -> generation count
    for (const auto &sw : wp.sc.trees)
      gen[sw.recipe] = wp.sc.tree_gen;
    for (const auto &sw : wp.sc.rocks)
      gen[sw.recipe] = wp.sc.rock_gen;
    for (const auto &sw : wp.sc.buildings)
      gen[sw.recipe] = wp.sc.building_gen;
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

// The two PRESERVED demo tree generators, invoked by reserved recipe name — the exact look from
// lsystem_forest. grow_pine fills the solid (trunk/branches) AND a needle (LINES) mesh;
// grow_branchy fills only the solid. Per-tree size / maturity / length jitter mirror grow_forest()
// so the shapes match, keyed by a STABLE cell id from the tree's position. Returns false for any
// non-reserved recipe.
// grow_pine's size is the island's, not metres: a size-1 pine stands 24, 42, 57 and 72 m at
// maturity 1..4. This scale puts the world's pines at 3..31 m (young to mature), beside the
// branchy broadleafs (about 8..28 m) and buildings at their own heights.
constexpr double kPineSize = 0.35;

bool growNativeTree(const std::string &recipe, const placed_prop &pp, std::uint64_t seed,
                    cvc::geometry &solid, cvc::geometry &needle) {
  using namespace cvc::lsys;
  static const int MATURITY[7] = {1, 2, 2, 3, 3, 3, 4};
  const std::uint64_t el =
      cell_id(std::int32_t(std::lround(pp.x)), std::int32_t(std::lround(pp.y)));
  if (recipe == "native_pine") {
    const double size = (0.55 + 0.4 * uni(seed, stream::size, el, 0)) * kPineSize * pp.scale;
    const int maturity = MATURITY[irand(seed, stream::maturity, el, 0, 0, 6)];
    grow_pine(solid, needle, pp.x, pp.y, pp.z, size, maturity);
    return true;
  }
  if (recipe == "native_branchy") {
    const double lenJitter = uni(seed, stream::size, el, 1, 0.8, 1.25);
    const std::uint64_t treeSeed = detail::splitmix(seed ^ path_id(el, 0xB2A4u));
    grow_branchy(solid, treeSeed, pp.x, pp.y, pp.z, 6.0 * pp.scale * lenJitter, 0.7 * pp.scale, 4,
                 3);
    return true;
  }
  return false;
}

// Tessellate the scattered TREES and ROCKS into merged meshes (buildings are the city's).
void buildProps(const world_model &wm, const surface_registry &reg, PropLib &lib,
                cvc::geometry &solid, cvc::geometry &foliage, cvc::geometry &needle) {
  const double kStem = 1.7, kLeaf = 2.2;
  std::set<std::string> buildingRecipes;
  for (const auto &sw : wm.params().sc.buildings)
    buildingRecipes.insert(sw.recipe);
  std::size_t i = 0;
  for (const placed_prop &pp : wm.props()) {
    if (buildingRecipes.count(pp.recipe) ||
        growNativeTree(pp.recipe, pp, wm.params().seed, solid, needle)) {
      ++i;
      continue;
    }
    const std::vector<lsys::structure> &vs = lib.get(pp.recipe);
    if (vs.empty()) {
      ++i;
      continue;
    }
    const lsys::structure &st = vs[i % vs.size()];
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
      appendLeaf(foliage, place(lf.pos), std::max(1.2, lf.size * sc * kLeaf),
                 roleColor(reg, lf.rl));
    ++i;
  }
}

// ── box massing: a per-building "sub-grammar" of SOLID full-height blocks in a varied footprint
// (tower / slab / L / T / podium+tower), on the scatter's building sites, greedily de-conflicted
// so they never intersect ─────────────────────────────────────────────────────────────────────
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
void buildBoxBuildings(const world_model &wm, const surface_registry &reg, std::uint64_t seed,
                       cvc::geometry &solid, std::vector<generated_world::apron> &aprons,
                       std::size_t &count) {
  std::set<std::string> buildingRecipes;
  for (const auto &sw : wm.params().sc.buildings)
    buildingRecipes.insert(sw.recipe);
  std::vector<const placed_prop *> sites;
  for (const placed_prop &pp : wm.props())
    if (buildingRecipes.count(pp.recipe))
      sites.push_back(&pp);
  std::vector<Vec3d> placed; // accepted (x, y, radius) for de-overlap
  std::size_t idx = 0;
  for (const placed_prop *pp : sites) {
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
    ++count;
    double yaw = pp->yaw_deg * M_PI / 180.0, yc = std::cos(yaw), ys = std::sin(yaw);
    Vec3d org{pp->x, pp->y, pp->z};
    for (const lsys::obox &b : m.boxes)
      appendBox(solid, b, roleColor(reg, b.rl), yc, ys, 1.0, 1.0, 1.0, org);
    // The plaza hugs the building's footprint AABB (+ a sidewalk margin), CENTRED on it:
    // asymmetric shapes (L/T/podium) sit off the massing origin.
    double minX = 1e30, maxX = -1e30, minY = 1e30, maxY = -1e30;
    for (const lsys::obox &b : m.boxes) {
      minX = std::min(minX, b.center.x - b.half.x);
      maxX = std::max(maxX, b.center.x + b.half.x);
      minY = std::min(minY, b.center.y - b.half.y);
      maxY = std::max(maxY, b.center.y + b.half.y);
    }
    double lcx = 0.5 * (minX + maxX), lcy = 0.5 * (minY + maxY);
    double hx = 0.5 * (maxX - minX) + 6.0, hy = 0.5 * (maxY - minY) + 6.0;
    aprons.push_back(
        {pp->x + lcx * yc - lcy * ys, pp->y + lcx * ys + lcy * yc, pp->z, hx, hy, yaw});
  }
}

// Connect each building plaza to its two nearest neighbours, skipping any span whose midpoint
// terrain would sit underwater.
std::vector<generated_world::street>
computeStreets(const std::vector<generated_world::apron> &aprons, const heightfield &hf,
               double half, double waterLevel) {
  std::vector<generated_world::street> out;
  const int n = int(aprons.size());
  const double maxLen = half * 0.7;
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
    const generated_world::apron &A = aprons[e.first], &B = aprons[e.second];
    double dx = B.x - A.x, dy = B.y - A.y, L = std::sqrt(dx * dx + dy * dy);
    if (L < 1.0 || L > maxLen)
      continue;
    if (hf.sample(0.5 * (A.x + B.x), 0.5 * (A.y + B.y)) < waterLevel)
      continue;
    out.push_back({A.x, A.y, B.x, B.y});
  }
  return out;
}

// ── the roads-first city ─────────────────────────────────────────────────────
// Road-generation parameters scaled to the world size. Moderate branching and roomy blocks make a
// readable street grid; short segment steps keep each ribbon on the terrain.
road_params cityRoadParams(std::uint64_t seed, double half) {
  road_params rp;
  rp.half = half;
  rp.seed = seed;
  rp.branch_chance = 0.14;
  rp.terrain_weight = 0.35; // bend toward contours, but keep a readable grid
  rp.block_size = std::max(110.0, half * 0.20);
  rp.local_block = std::max(48.0, half * 0.08);
  rp.segment_len = std::max(6.0, half * 0.012);
  rp.core_radius = half * 0.6;
  rp.arterial_width = 8.0; // thin carriageways
  rp.local_width = 4.5;
  return rp;
}

// Drop road segments whose midpoint is under the water line, so streets stay on dry land.
void pruneUnderwaterRoads(road_network &net, const heightfield &hf, double waterLevel) {
  std::vector<road_segment> keep;
  keep.reserve(net.segments.size());
  for (const road_segment &s : net.segments) {
    if (s.a < 0 || s.b < 0 || s.a >= int(net.nodes.size()) || s.b >= int(net.nodes.size()))
      continue;
    const double mx = 0.5 * (net.nodes[s.a].x + net.nodes[s.b].x);
    const double my = 0.5 * (net.nodes[s.a].y + net.nodes[s.b].y);
    if (hf.sample(mx, my) > waterLevel + 0.5)
      keep.push_back(s);
  }
  net.segments.swap(keep);
}

// Stamp the library's archetypes along the roads, on dry-land lots. Returns how many.
std::size_t buildCityBuildings(const building_library &lib, const road_network &net,
                               const heightfield &hf, std::uint64_t seed, double waterLevel,
                               cvc::geometry &out) {
  // Lots the size of a city's (16 m of frontage, 28 m deep): the archetypes keep their real size
  // (true_scale), so a lot takes a building that fits it rather than a shrunken one, and the
  // buildings stand at their own heights beside the trees.
  site_params sp;
  sp.seed = seed;
  sp.frontage = 16.0;
  sp.spacing = 3.0;
  sp.lot_depth = 28.0;
  sp.setback = 4.0;
  sp.junction_clear = 10.0;
  std::vector<lot> lots = layout_lots(net, sp);
  lots.erase(std::remove_if(lots.begin(), lots.end(),
                            [&](const lot &L) { return hf.sample(L.x, L.y) <= waterLevel + 1.0; }),
             lots.end());
  fit_params fp;
  fp.seed = seed;
  fp.true_scale = true;
  const std::vector<building_instance> inst =
      fit_buildings(lib, lots, fp, [&hf](double x, double y) { return hf.sample(x, y); });
  cvc::geometry b = stamp_buildings(lib, inst);
  if (b.const_points().empty())
    return 0;
  if (b.const_normals().size() != b.const_points().size())
    b.compute_normals();
  // The archetypes carry no material colour: one concrete tone, per vertex.
  b.colors().assign(b.const_points().size(), cvc::geometry::color_t{{0.64, 0.62, 0.59}});
  out.merge(b);
  return inst.size();
}

// A cheap self-contained 2-D value-noise fBm (the ground's dirt / dry-grass breakup).
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

// The travelling wave's surface height at (x, y) and time t.
double seaSurface(double half, double level, double x, double y, double t) {
  struct Wave {
    double hx, hy, len, omega, amp;
  };
  const double L = std::max(30.0, half * 0.25);
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
  return level + 0.9 * (crest - 0.7);
}

} // namespace

const char *world_preset_name(int preset) {
  return kPresetNames[(preset >= 0 && preset < 5) ? preset : 0];
}
const char *world_ontology_name(int ontology) {
  return kOntologyNames[std::max(0, std::min(2, ontology))];
}
int world_preset_from_name(const std::string &n) {
  for (int i = 0; i < 5; ++i)
    if (n == kPresetNames[i])
      return i;
  return 0;
}

void apply_world_preset(generated_world_params &p) {
  switch (p.preset) {
  case 1: // forest
    p.trees = 1600;
    p.rocks = 220;
    p.buildings = 8;
    p.amp_m = 26.0;
    break;
  case 2: // urban
    p.trees = 320;
    p.rocks = 40;
    p.buildings = 340;
    p.amp_m = 10.0;
    break;
  case 3: // sparse
    p.trees = 260;
    p.rocks = 140;
    p.buildings = 70;
    p.amp_m = 22.0;
    break;
  case 4: // island
    p.trees = 900;
    p.rocks = 220;
    p.buildings = 60;
    p.amp_m = 46.0;
    break;
  default: // standard
    p.trees = 500;
    p.rocks = 90;
    p.buildings = 110;
    p.amp_m = 36.0;
    break;
  }
}

world_params to_world_params(const generated_world_params &gp) {
  const double half = std::max(50.0, gp.half_m);
  world_params wp;
  wp.min_x = wp.min_y = -half;
  wp.max_x = wp.max_y = half;
  wp.seed = gp.seed;
  wp.ontology = world_ontology_name(gp.ontology);
  // Relief scaled for a large world: long base wavelength, deep enough that low ground drops below
  // the water to carve streams and ponds. A shorter base wavelength, more octaves and a higher
  // gain give steeper, more broken relief, so the slopes classify as gravel / scree / bare rock
  // rather than an all-grass plain.
  wp.hf.seed = gp.seed;
  wp.hf.amp_m = gp.amp_m;
  wp.hf.sea_level_m = 0.0; // terrain floor at 0; water floods above it (water_level_m)
  wp.hf.base_wavelength_m = std::max(50.0, half * 0.30);
  wp.hf.octaves = 8;
  wp.hf.gain = 0.55;
  wp.hf.lacunarity = 2.1;
  wp.hf.warp_m = 22.0;
  wp.hf.warp_wavelength_m = half * 0.6;
  // The island preset shapes the relief (a central dome dropping below the water line at the rim);
  // every preset's object mix is in the counts (apply_world_preset).
  if (gp.preset == int(world_preset::island)) {
    wp.hf.island = true;
    wp.hf.island_peak_m = std::max(gp.amp_m, 40.0);
    wp.hf.island_radius_m = half * 0.72;
    wp.hf.shelf_m = -std::max(6.0, gp.water_level_m + 4.0);
  }
  scatter_params &s = wp.sc;
  // Trees are the two PRESERVED demo generators (reserved recipe names, see growNativeTree), an
  // even mix: the conical pine and the recursive broadleaf, the exact look from lsystem_forest.
  s.trees = {{"native_pine", 0.5}, {"native_branchy", 0.5}};
  s.rocks = {{"boulder_cluster", 1.0}};
  s.buildings = {{"office_block", 0.5}, {"brick_house", 0.5}};
  s.tree_count = gp.trees;
  s.rock_count = gp.rocks;
  s.building_count = gp.buildings;
  s.tree_gen = gp.tree_gen;
  s.min_ground_m = gp.water_level_m + 1.0; // props on dry land, above the waterline
  return wp;
}

double generated_world::height(double x, double y) const {
  if (!terrain)
    return 0.0;
  x = std::max(terrain->x0(), std::min(terrain->max_x(), x));
  y = std::max(terrain->y0(), std::min(terrain->max_y(), y));
  const double h = terrain->height(x, y);
  return std::isnan(h) ? 0.0 : h;
}

world_geometry generated_world::geometry() const {
  world_geometry g(model ? model->registry() : surface_registry::builtin());
  if (terrain) {
    geometry_item t;
    t.name = "terrain";
    t.geometry = terrain;
    g.add(std::move(t));
  }
  const surface_registry &reg = g.surfaces();
  auto add_mesh = [&](const char *name, const cvc::geometry &m, std::uint16_t surface,
                      std::uint8_t roles) {
    if (m.const_points().empty())
      return;
    geometry_item it;
    it.name = name;
    it.geometry = std::make_shared<const cvc::geometry>(m);
    it.surface = surface;
    it.roles = roles;
    g.add(std::move(it));
  };
  std::uint16_t asphalt = 0;
  reg.find("asphalt", asphalt);
  add_mesh("buildings", buildings, reg.class_for_role(lsys::role::wall_concrete), role_all);
  add_mesh("roads", roads, asphalt, role_render);
  add_mesh("props", props, 0, role_render);
  add_mesh("foliage", foliage, 0, role_render);
  return g;
}

generated_world generate_world(const generated_world_params &p, const building_library *library) {
  generated_world w;
  w.params = p;
  const double half = std::max(50.0, p.half_m);
  w.params.half_m = half;
  const world_params wp = to_world_params(w.params);
  w.model = std::make_shared<const world_model>(world_model::generate(wp));
  const world_model &wm = *w.model;
  const heightfield &hf = wm.hf();
  const surface_registry &reg = wm.registry();
  const double water = p.water_level_m;

  // The terrain mesh: one sample per cell_m (capped), and a moderate raster decoupled from it, so a
  // huge terrain does not blow up the per-cell raster work (which includes an O(cells x segments)
  // asphalt stamp).
  const int terrainN = std::max(
      129, std::min(std::max(129, p.max_terrain_n), int(std::lround(2.0 * half / p.cell_m)) + 1));
  const int rasterN = std::max(2, std::min(terrainN, p.raster_n));
  w.grid.rows = w.grid.cols = rasterN;
  w.grid.min_x = w.grid.min_y = -half;
  w.grid.max_x = w.grid.max_y = half;
  raster(wm, w.grid, w.raster);
  float hmin = 1e30f, hmax = -1e30f;
  for (float h : w.raster.height) {
    hmin = std::min(hmin, h);
    hmax = std::max(hmax, h);
  }
  w.height_min = hmin;
  w.height_max = hmax;

  std::vector<float> heights(std::size_t(terrainN) * terrainN);
  const double step = 2.0 * half / (terrainN - 1);
  for (int r = 0; r < terrainN; ++r)
    for (int c = 0; c < terrainN; ++c)
      heights[std::size_t(r) * terrainN + c] = float(hf.sample(-half + c * step, -half + r * step));
  w.terrain = std::make_shared<cvc::height_field>(cvc::height_field::from_bounds(
      terrainN, terrainN, -half, -half, half, half, std::move(heights)));
  // Each terrain cell takes the material of the raster cell under its centre.
  {
    std::vector<cvc::geometry::face_material_t> mats(std::size_t(terrainN - 1) * (terrainN - 1));
    for (int r = 0; r + 1 < terrainN; ++r)
      for (int c = 0; c + 1 < terrainN; ++c) {
        const double u = (c + 0.5) / (terrainN - 1), v = (r + 0.5) / (terrainN - 1);
        const int rc = std::min(rasterN - 1, int(u * rasterN));
        const int rr = std::min(rasterN - 1, int(v * rasterN));
        mats[std::size_t(r) * (terrainN - 1) + c] =
            cvc::geometry::face_material_t(w.raster.klass[std::size_t(rr) * rasterN + rc]);
      }
    w.terrain->set_cell_materials(std::move(mats));
  }

  PropLib lib;
  lib.build(wp);
  buildProps(wm, reg, lib, w.props, w.foliage, w.needles);

  // The city. With a library: roads FIRST (a terrain-following street network), the archetypes
  // stamped along the road-side lots, asphalt burned into the raster, and the extruded road
  // surface. Without: box massing on the scatter's sites, with streets painted between plazas.
  const auto road_h = [&hf](double x, double y) { return hf.sample(x, y); };
  if (library && !library->empty()) {
    road_network net = generate_roads(cityRoadParams(p.seed, half), road_h);
    pruneUnderwaterRoads(net, hf, water);
    w.building_count = buildCityBuildings(*library, net, hf, p.seed, water, w.buildings);
    w.archetypes = true;
    rasterize_roads(net, w.grid, reg, w.raster, 1.0);
    road_mesh_params rmp;
    rmp.lift = 0.35; // clearly above the terrain mesh, so the ribbons don't z-fight it
    w.roads = extrude_roads(net, rmp, road_h);
  } else {
    buildBoxBuildings(wm, reg, p.seed, w.buildings, w.aprons, w.building_count);
    w.streets = computeStreets(w.aprons, hf, half, water);
  }
  return w;
}

std::vector<unsigned char> paint_surface(const generated_world &w, surface_color mode, int size) {
  const int T = std::max(2, size);
  const raster_out &ro = w.raster;
  const grid_spec &g = w.grid;
  const surface_registry &reg = w.model ? w.model->registry() : surface_registry::builtin();
  const int m = int(mode);
  std::vector<unsigned char> img(std::size_t(T) * T * 3, 0);
  if (ro.klass.empty())
    return img;
  const float hmin = float(w.height_min), hmax = float(w.height_max);
  const double spanx = g.max_x - g.min_x, spany = g.max_y - g.min_y;
  const cvc::geometry::color_t dirt = {0.42, 0.34, 0.23}, dryGrass = {0.45, 0.47, 0.24};
  auto put = [&](int tx, int ty, const cvc::geometry::color_t &c) {
    std::size_t o = (std::size_t(ty) * T + tx) * 3;
    img[o] = (unsigned char)std::min(255.0, std::max(0.0, c[0] * 255.0));
    img[o + 1] = (unsigned char)std::min(255.0, std::max(0.0, c[1] * 255.0));
    img[o + 2] = (unsigned char)std::min(255.0, std::max(0.0, c[2] * 255.0));
  };
  // The raster is coarse (a few metres a cell) and the texture fine, so a texel does not take its
  // nearest cell's colour, which paints the grid. Material mode blends the four cells around it,
  // at a position pushed about a cell by low-frequency noise (domain warping), with the blend's
  // fractions jittered by finer noise and eased: class boundaries wander and fray like real ground
  // cover instead of following the grid, and a lone cell is a soft patch, not a square. Asphalt
  // is not warped, so the road edges stay straight. Risk and height, continuous fields, are
  // interpolated; occupancy, a yes/no QA view, keeps its cells.
  const int maxc = g.cols - 1, maxr = g.rows - 1;
  const double cellx = spanx / std::max(1, maxc), celly = spany / std::max(1, maxr);
  const double warpAmp = 0.9 * std::max(cellx, celly),
               warpFreq = 1.0 / (3.0 * std::max(cellx, celly));
  const double jitterFreq = 1.0 / std::max(0.5, 0.35 * std::max(cellx, celly));
  std::uint16_t asphalt = 0;
  const bool hasAsphalt = reg.find("asphalt", asphalt);
  auto isGrass = [](std::uint16_t k) { return k == 6 || k == 7 || k == 8; }; // grass/tall/bush
  auto ease = [](double t) { return t * t * (3.0 - 2.0 * t); };
  auto cell = [&](int r, int c) { return std::size_t(r) * g.cols + c; };
  // Each texel is a pure function of its position, so the rows paint in parallel (the same
  // bytes on any thread count).
#ifdef CVC_USING_OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (int ty = 0; ty < T; ++ty) {
    double v = double(ty) / (T - 1), wy = g.min_y + v * spany;
    for (int tx = 0; tx < T; ++tx) {
      double u = double(tx) / (T - 1), wx = g.min_x + u * spanx;
      double fx = u * maxc, fy = v * maxr;
      if (m == 2) { // occupancy: its cells, as they are
        const int c = std::min(maxc, std::max(0, int(std::lround(fx))));
        const int r = std::min(maxr, std::max(0, int(std::lround(fy))));
        put(tx, ty, cellColor(ro, reg, cell(r, c), m, hmin, hmax));
        continue;
      }
      int c0 = std::min(std::max(0, maxc - 1), int(std::floor(fx)));
      int r0 = std::min(std::max(0, maxr - 1), int(std::floor(fy)));
      bool road = false;
      if (m == 0 && hasAsphalt)
        for (int dr = 0; dr < 2 && !road; ++dr)
          for (int dc = 0; dc < 2 && !road; ++dc)
            road = ro.klass[cell(std::min(maxr, r0 + dr), std::min(maxc, c0 + dc))] == asphalt;
      double fu = fx - c0, fv = fy - r0;
      if (m == 0 && !road) {
        const double qx =
            wx + warpAmp * (2.0 * gfbm2(wx * warpFreq + 3.1, wy * warpFreq + 7.7) - 1.0);
        const double qy =
            wy + warpAmp * (2.0 * gfbm2(wx * warpFreq + 13.9, wy * warpFreq + 1.3) - 1.0);
        fx = std::min(double(maxc), std::max(0.0, (qx - g.min_x) / cellx));
        fy = std::min(double(maxr), std::max(0.0, (qy - g.min_y) / celly));
        c0 = std::min(std::max(0, maxc - 1), int(std::floor(fx)));
        r0 = std::min(std::max(0, maxr - 1), int(std::floor(fy)));
        fu = fx - c0;
        fv = fy - r0;
        fu = ease(std::min(
            1.0, std::max(0.0, fu + 0.6 * (gnoise2(wx * jitterFreq, wy * jitterFreq) - 0.5))));
        fv = ease(std::min(
            1.0, std::max(0.0, fv + 0.6 * (gnoise2(wx * jitterFreq + 5.3, wy * jitterFreq + 9.1) -
                                           0.5))));
      }
      const int c1 = std::min(maxc, c0 + 1), r1 = std::min(maxr, r0 + 1);
      const std::size_t idx[4] = {cell(r0, c0), cell(r0, c1), cell(r1, c0), cell(r1, c1)};
      const double wt[4] = {(1 - fu) * (1 - fv), fu * (1 - fv), (1 - fu) * fv, fu * fv};
      if (m != 0) { // risk / height: the field interpolated, then coloured
        double s = 0.0;
        for (int k = 0; k < 4; ++k)
          s += wt[k] * (m == 1 ? ro.risk_raw[idx[k]] : ro.height[idx[k]]);
        put(tx, ty, heat(m == 1 ? s : (hmax > hmin ? (s - hmin) / (hmax - hmin) : 0.0)));
        continue;
      }
      cvc::geometry::color_t col{0.0, 0.0, 0.0};
      double grass = 0.0;
      for (int k = 0; k < 4; ++k) {
        const surface_class &sc = reg[ro.klass[idx[k]]];
        for (int q = 0; q < 3; ++q)
          col[q] += wt[k] * sc.albedo[q];
        if (isGrass(ro.klass[idx[k]]))
          grass += wt[k];
      }
      if (grass > 0.0) { // grass broken up by dirt and dry patches, as much as the texel is grass
        double n1 = gfbm2(wx * 0.020, wy * 0.020), n2 = gfbm2(wx * 0.006 + 11, wy * 0.006 + 7);
        double dfac = grass * std::min(1.0, std::max(0.0, (n1 - 0.52) / 0.30));
        double gfac = grass * std::min(1.0, std::max(0.0, (n2 - 0.55) / 0.30));
        for (int q = 0; q < 3; ++q)
          col[q] = col[q] * (1 - dfac) + dirt[q] * dfac;
        for (int q = 0; q < 3; ++q)
          col[q] = col[q] * (1 - 0.5 * gfac) + dryGrass[q] * 0.5 * gfac;
      }
      put(tx, ty, col);
    }
  }
  if (m != 0)
    return img; // QA modes: raw raster only
  const cvc::geometry::color_t road = {0.20, 0.20, 0.23}, curb = {0.31, 0.31, 0.34};
  const double roadW = 8.0;
  double sx = (T - 1) / spanx, sy = (T - 1) / spany;
  auto W2Tx = [&](double x) { return (x - g.min_x) * sx; };
  auto W2Ty = [&](double y) { return (y - g.min_y) * sy; };
  double rTexX = 0.5 * roadW * sx;
  for (const generated_world::street &s : w.streets) {
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
  for (const generated_world::apron &a : w.aprons) {
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

double water_floor(const generated_world &w) {
  return std::min(w.height_min - 2.0, w.params.water_level_m - 6.0);
}
double water_top(const generated_world &w) { return w.params.water_level_m + 4.0; }
int water_resolution(const generated_world &w) {
  return std::max(80, std::min(320, int(std::lround(2.0 * w.params.half_m / 8.0))));
}

std::vector<float> water_field(const generated_world &w, double t, int n, int nz) {
  n = std::max(2, n);
  nz = std::max(2, nz);
  const double half = w.params.half_m, level = w.params.water_level_m;
  const double z0 = water_floor(w), z1 = water_top(w);
  const heightfield *hf = w.model ? &w.model->hf() : nullptr;
  // The ground and the wave's surface per column, then the depth down each column.
  std::vector<float> f(std::size_t(n) * n * nz, 0.0f);
  std::vector<double> ground(std::size_t(n) * n), surf(std::size_t(n) * n);
  for (int j = 0; j < n; ++j) {
    const double y = -half + 2.0 * half * j / (n - 1);
    for (int i = 0; i < n; ++i) {
      const double x = -half + 2.0 * half * i / (n - 1);
      ground[std::size_t(j) * n + i] = hf ? hf->sample(x, y) : 0.0;
      surf[std::size_t(j) * n + i] = seaSurface(half, level, x, y, t);
    }
  }
  for (int k = 0; k < nz; ++k) {
    const double z = z0 + (z1 - z0) * k / (nz - 1);
    for (std::size_t c = 0; c < std::size_t(n) * n; ++c) {
      const double below = surf[c] - z, above = z - ground[c];
      const double depth = (below > 0.0 && above > 0.0) ? std::min(1.0, below / 5.0) : 0.0;
      f[std::size_t(k) * n * n + c] = float(depth);
    }
  }
  return f;
}

void water_transfer(double t, std::vector<double> &color, std::vector<double> &opacity) {
  double k = 0.0110 + 0.0016 * std::sin(t * 0.9);
  color = {0.00, 0.40, 0.74, 0.72, 0.25, 0.13, 0.52, 0.64,
           0.60, 0.05, 0.24, 0.44, 1.00, 0.02, 0.10, 0.24};
  opacity = {0.00, 0.0, 0.14, k * 0.5, 0.55, k, 1.00, k * 2.2};
}

} // namespace world
} // namespace cvc
