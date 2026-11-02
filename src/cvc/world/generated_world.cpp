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
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
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

// Tessellate the scattered TREES and ROCKS into merged meshes (buildings are the city's), leaving
// out any the city has taken the ground of (`keep` false: a road, a building).
void buildProps(const world_model &wm, const surface_registry &reg, PropLib &lib,
                const std::function<bool(double, double)> &keep, cvc::geometry &solid,
                cvc::geometry &foliage, cvc::geometry &needle) {
  const double kStem = 1.7, kLeaf = 2.2;
  std::set<std::string> buildingRecipes;
  for (const auto &sw : wm.params().sc.buildings)
    buildingRecipes.insert(sw.recipe);
  std::size_t i = 0;
  for (const placed_prop &pp : wm.props()) {
    if (buildingRecipes.count(pp.recipe) || (keep && !keep(pp.x, pp.y)) ||
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

// ── the city: districts, roads first, lots along them, a building per lot ────
// A district core: an elliptical bump of urbanity (1 at its centre) on dry, gentle ground.
struct District {
  double x = 0, y = 0, r_major = 1, r_minor = 1, c = 1, s = 0, strength = 1;
};

// The urbanity field, in [0, 1]: overlapping districts reinforce without passing 1.
struct Districts {
  std::vector<District> d;
  double operator()(double x, double y) const {
    double keep = 1.0;
    for (const District &k : d) {
      const double dx = x - k.x, dy = y - k.y;
      const double a = (dx * k.c + dy * k.s) / k.r_major, b = (-dx * k.s + dy * k.c) / k.r_minor;
      keep *= 1.0 - k.strength * std::exp(-0.5 * (a * a + b * b));
    }
    return 1.0 - keep;
  }
};

double slopeDeg(const terrain_source &hf, double x, double y) {
  const double e = 4.0;
  const double gx = (hf.sample(x + e, y) - hf.sample(x - e, y)) / (2.0 * e);
  const double gy = (hf.sample(x, y + e) - hf.sample(x, y - e)) / (2.0 * e);
  return std::atan(std::hypot(gx, gy)) * 180.0 / M_PI;
}

// About one district per 600 m of world side. Each centre is the best of a few dozen hashed
// candidates: dry, flat (scored by exp(-(slope / 9 deg)^2)), with dry, gentle ground around it,
// off the edges and away from the centres already placed, with a slight pull to the middle of the
// world.
Districts pickDistricts(const terrain_source &hf, double half, double water, std::uint64_t seed,
                        const road_mask_fn &blocked) {
  using lsys::stream;
  using lsys::uni;
  Districts out;
  const int n = std::max(1, std::min(6, int(std::lround(2.0 * half / 600.0))));
  const double edge = 0.15 * half, apart = 0.9 * half / std::sqrt(double(n));
  for (int k = 0; k < n; ++k) {
    double best = 0.0, bx = 0.0, by = 0.0;
    for (std::uint32_t i = 0; i < 48; ++i) {
      const std::uint64_t el = 0xD157u + std::uint64_t(k) * 1000u + i;
      const double x = uni(seed, stream::terrain, el, 0, -half + edge, half - edge);
      const double y = uni(seed, stream::terrain, el, 1, -half + edge, half - edge);
      if (hf.sample(x, y) < water + 3.0 || (blocked && blocked(x, y)))
        continue;
      // How much of the ground around it a town could use: dry, and gentle enough for streets
      // (a flat flood plain by the shore scores low).
      const double ring = 0.2 * half;
      int usable = 0, total = 0;
      for (int a = 0; a < 8; ++a)
        for (double rr : {0.0, 0.5, 1.0}) {
          if (rr == 0.0 && a > 0)
            continue;
          const double qx = x + rr * ring * std::cos(a * M_PI / 4.0);
          const double qy = y + rr * ring * std::sin(a * M_PI / 4.0);
          ++total;
          usable += hf.sample(qx, qy) > water + 2.0 && slopeDeg(hf, qx, qy) < 16.0 &&
                            !(blocked && blocked(qx, qy))
                        ? 1
                        : 0;
        }
      const double slope = slopeDeg(hf, x, y);
      double sep = 1.0;
      for (const District &o : out.d) {
        const double f = std::min(1.0, std::hypot(x - o.x, y - o.y) / apart);
        sep *= f * f;
      }
      const double score = std::exp(-(slope / 9.0) * (slope / 9.0)) * (double(usable) / total) *
                           sep * (1.0 - 0.3 * std::hypot(x, y) / half);
      if (score > best) {
        best = score;
        bx = x;
        by = y;
      }
    }
    if (best <= 0.0)
      continue; // no dry ground left for another core
    District D;
    D.x = bx;
    D.y = by;
    const std::uint64_t el = 0xD15Cu + std::uint64_t(k);
    D.r_major = half * uni(seed, stream::terrain, el, 0, 0.28, 0.42);
    D.r_minor = D.r_major * uni(seed, stream::terrain, el, 1, 0.55, 0.95);
    const double ang = uni(seed, stream::terrain, el, 2, 0.0, M_PI);
    D.c = std::cos(ang);
    D.s = std::sin(ang);
    D.strength = k == 0 ? 1.0 : uni(seed, stream::terrain, el, 3, 0.7, 1.0);
    out.d.push_back(D);
  }
  return out;
}

// Road-generation parameters scaled to the world size: arterials crossing at each district
// centre, collectors a block apart off them, local streets packed tighter in the cores and fading
// out at the districts' edges; grades held to 35 % (steered along the contours: the
// relief is hilly, its median slope about 33 %) and no road into the water.
road_params cityRoadParams(std::uint64_t seed, double half, const Districts &dist) {
  road_params rp;
  rp.half = half;
  rp.seed = seed;
  rp.branch_chance = 0.6;   // once a block is run: a little irregularity in the blocks
  rp.terrain_weight = 0.35; // bend toward contours, but keep a readable grid
  rp.block_size = std::max(110.0, half * 0.20);
  rp.local_block = std::max(56.0, half * 0.09); // room for two lots back to back
  rp.segment_len = std::max(6.0, half * 0.012);
  // Snap within less than a step, so the arms leaving a centre (a step apart) stay separate roads.
  rp.snap_dist = std::min(10.0, 0.8 * rp.segment_len);
  rp.min_segment = 0.5 * rp.segment_len;
  rp.core_radius = half * 0.35;
  rp.arterial_width = 9.0;
  rp.collector_width = 6.5;
  rp.local_width = 5.0;
  rp.max_grade = 0.35;
  rp.steer_max_deg = 80.0;
  rp.sparse_spacing = 0.55;
  rp.sparse_branch = 0.45;
  rp.local_min_urbanity = 0.06;
  rp.parallel_gap = 0.5;
  for (const District &d : dist.d)
    rp.centres.push_back(road_centre{d.x, d.y});
  return rp;
}

// The "districts" city: district cores on dry, gentle ground, and a street network grown from them
// (graded, kept off blocked ground, dead-end stubs trimmed).
city_streets districtStreets(const city_context &c) {
  const generated_world_params &p = c.params;
  const terrain_source &ground = c.terrain;
  auto dist = std::make_shared<const Districts>(
      pickDistricts(ground, p.half_m, p.water_level_m, p.seed, c.blocked));
  city_streets out;
  out.roads = generate_roads(
      cityRoadParams(p.seed, p.half_m, *dist),
      [&ground](double x, double y) { return ground.sample(x, y); }, c.blocked,
      [dist](double x, double y) { return (*dist)(x, y); });
  prune_roads(out.roads, 30.0);
  out.urbanity = [dist](double x, double y) { return (*dist)(x, y); };
  out.districts = dist->d.size();
  return out;
}

// A building's footprint, for keeping trees out of it: centre, axes (c, s) and half extents.
struct Footprint {
  double x, y, c, s, hx, hy;
};
struct FootprintIndex {
  double cell = 32.0;
  std::map<std::pair<int, int>, std::vector<Footprint>> cells;
  std::vector<Footprint> all; // in the order added
  void add(const Footprint &f) {
    all.push_back(f);
    const double r = std::hypot(f.hx, f.hy);
    for (int i = int(std::floor((f.x - r) / cell)); i <= int(std::floor((f.x + r) / cell)); ++i)
      for (int j = int(std::floor((f.y - r) / cell)); j <= int(std::floor((f.y + r) / cell)); ++j)
        cells[{i, j}].push_back(f);
  }
  bool contains(double x, double y) const {
    auto it = cells.find({int(std::floor(x / cell)), int(std::floor(y / cell))});
    if (it == cells.end())
      return false;
    for (const Footprint &f : it->second) {
      const double dx = x - f.x, dy = y - f.y;
      if (std::fabs(dx * f.c + dy * f.s) <= f.hx && std::fabs(-dx * f.s + dy * f.c) <= f.hy)
        return true;
    }
    return false;
  }
};

// The buildings into the raster: every cell whose sample point is under a footprint (the
// footprints carry a 1 m tree margin, taken off here), and the cell under each centre, so a
// building smaller than a cell on a coarse raster still blocks it. They take the concrete wall
// class, occupied and hard.
void stampBuildings(const FootprintIndex &fps, const grid_spec &g, const surface_registry &reg,
                    raster_out &out) {
  const std::uint16_t wall = reg.class_for_role(lsys::role::wall_concrete);
  const surface_class &sc = reg[wall];
  const double cw = g.cell_w(), ch = g.cell_h();
  auto mark = [&](int r, int c) {
    const std::size_t i = std::size_t(r) * g.cols + c;
    out.klass[i] = wall;
    out.layer_owner[i] = 1;
    out.risk_raw[i] = sc.rho;
    out.hard[i] = sc.hard ? 1 : 0;
    out.occupancy[i] = 1;
  };
  for (const Footprint &f : fps.all) {
    const double hx = std::max(0.0, f.hx - 1.0), hy = std::max(0.0, f.hy - 1.0);
    const double rad = std::hypot(hx, hy);
    const int c0 = std::max(0, int(std::floor((f.x - rad - g.min_x) / cw)));
    const int c1 = std::min(g.cols - 1, int(std::ceil((f.x + rad - g.min_x) / cw)));
    const int r0 = std::max(0, int(std::floor((f.y - rad - g.min_y) / ch)));
    const int r1 = std::min(g.rows - 1, int(std::ceil((f.y + rad - g.min_y) / ch)));
    for (int r = r0; r <= r1; ++r)
      for (int c = c0; c <= c1; ++c) {
        const double dx = g.world_x(c) - f.x, dy = g.world_y(r) - f.y;
        if (std::fabs(dx * f.c + dy * f.s) <= hx && std::fabs(-dx * f.s + dy * f.c) <= hy)
          mark(r, c);
      }
    const int cc = int(std::lround((f.x - g.min_x) / cw)),
              rc = int(std::lround((f.y - g.min_y) / ch));
    if (cc >= 0 && cc < g.cols && rc >= 0 && rc < g.rows)
      mark(rc, cc);
  }
}

// A flat-shaded convex polygon (its own vertices, so the faces keep crisp edges).
void appendFace(cvc::geometry &g, std::initializer_list<Vec3d> poly,
                const cvc::geometry::color_t &col) {
  const idx_t b = idx_t(g.points().size());
  for (const Vec3d &p : poly) {
    g.points().push_back({p.x, p.y, p.z});
    g.colors().push_back(col);
  }
  for (idx_t i = 1; i + 1 < idx_t(poly.size()); ++i)
    g.tris().push_back({b, b + i, b + i + 1});
}

// A building piece in a lot's frame: a box over [cx +- hx] x [cy +- hy] x [z0, z1], with an
// optional gable roof (ridge along x, `rise` high) instead of a flat top. `to` maps lot-frame
// points to the world.
template <class To>
void appendMass(cvc::geometry &g, const To &to, double cx, double cy, double hx, double hy,
                double z0, double z1, double rise, const cvc::geometry::color_t &wall,
                const cvc::geometry::color_t &roof) {
  const Vec3d a0 = to(cx - hx, cy - hy, z0), b0 = to(cx + hx, cy - hy, z0),
              c0 = to(cx + hx, cy + hy, z0), d0 = to(cx - hx, cy + hy, z0);
  const Vec3d a1 = to(cx - hx, cy - hy, z1), b1 = to(cx + hx, cy - hy, z1),
              c1 = to(cx + hx, cy + hy, z1), d1 = to(cx - hx, cy + hy, z1);
  appendFace(g, {a0, b0, b1, a1}, wall); // -y
  appendFace(g, {c0, d0, d1, c1}, wall); // +y
  appendFace(g, {b0, c0, c1, b1}, wall); // +x
  appendFace(g, {d0, a0, a1, d1}, wall); // -x
  if (rise <= 0.0) {
    appendFace(g, {a1, b1, c1, d1}, roof);
    return;
  }
  const Vec3d r0 = to(cx - hx, cy, z1 + rise), r1 = to(cx + hx, cy, z1 + rise);
  appendFace(g, {a1, b1, r1, r0}, roof); // the -y slope
  appendFace(g, {c1, d1, r0, r1}, roof); // the +y slope
  appendFace(g, {b1, c1, r1}, wall);     // the gable ends
  appendFace(g, {d1, a1, r0}, wall);
}

// Procedural massing for one lot. The footprint fills the frontage (to shared walls in a core,
// with side yards outside one) and sits at the front of the lot; its height is lognormal around a
// storey count that rises with urbanity, a little taller for a bigger footprint, with a rare tall
// tail (a podium and a tower) in the cores. Low houses outside the cores get a gable roof. The
// walls go down to the lot's lowest ground, so no corner floats on a slope. Returns the footprint.
Footprint massLot(const lot &L, std::uint64_t seed, const surface_registry &reg,
                  cvc::geometry &out) {
  using lsys::stream;
  const std::uint64_t el =
      lsys::cell_id(std::int32_t(std::lround(L.x)), std::int32_t(std::lround(L.y)));
  auto U = [&](std::uint32_t d) { return lsys::uni(seed, stream::floorplan, el, d); };
  const double u = L.urbanity;
  const double side_gap = 1.6 * (1.0 - u);
  double w = std::max(5.0, L.frontage - 2.0 * side_gap);
  if (u < 0.6)
    w *= 0.72 + 0.28 * U(0);
  const double ratio = std::exp(std::log(0.6) + std::log(1.6 / 0.6) * U(1));
  const double d = std::max(5.0, std::min(L.depth - 2.0 - 5.0 * (1.0 - u), w * ratio));
  const double area = w * d;
  double h = (4.5 + 8.0 * u) * std::exp(0.15 * std::log(area / 150.0) +
                                        lsys::nrand(seed, stream::floorplan, el, 3, 0.0, 0.28));
  const bool tall = U(8) < 0.08 * u * u && area > 200.0;
  if (tall)
    h = std::max(h, std::min(90.0, 18.0 / std::sqrt(std::max(1e-3, U(9)))));
  h = std::max(3.0, std::min(90.0, h));

  // Materials: glass, metal and concrete in the cores; concrete, brick and plaster outside.
  const lsys::role core[3] = {lsys::role::glass, lsys::role::wall_concrete, lsys::role::metal};
  const lsys::role town[3] = {lsys::role::wall_brick, lsys::role::wall_concrete,
                              lsys::role::wall_brick};
  const lsys::role rl = (U(4) < u ? core : town)[int(U(5) * 3.0) % 3];
  cvc::geometry::color_t wall = roleColor(reg, rl);
  const double tint = 0.9 + 0.2 * U(6);
  for (double &ch : wall)
    ch = std::min(1.0, ch * tint);
  const cvc::geometry::color_t roofFlat = {0.36, 0.36, 0.37}, roofTile = {0.46, 0.24, 0.18};

  const double yc = std::cos(L.yaw), ys = std::sin(L.yaw);
  auto to = [&](double lx, double ly, double z) {
    return Vec3d{L.x + lx * yc - ly * ys, L.y + lx * ys + ly * yc, z};
  };
  const double cy = 0.5 * L.depth - 0.5 * d; // the front wall on the setback line
  const double z0 = L.ground_min - 0.5, top = L.ground_max + h;
  if (tall) {
    const double podium = L.ground_max + 5.0 + 4.0 * U(10);
    appendMass(out, to, 0.0, cy, 0.5 * w, 0.5 * d, z0, podium, 0.0, wall, roofFlat);
    appendMass(out, to, 0.0, cy, 0.34 * w, 0.34 * d, podium, top, 0.0, wall, roofFlat);
  } else {
    const bool gable = h < 9.0 && u < 0.5;
    appendMass(out, to, 0.0, cy, 0.5 * w, 0.5 * d, z0, top, gable ? 0.35 * std::min(w, d) : 0.0,
               wall, gable ? roofTile : roofFlat);
  }
  const Vec3d c = to(0.0, cy, 0.0);
  return Footprint{c.x, c.y, yc, ys, 0.5 * w + 1.0, 0.5 * d + 1.0};
}

// Build the city on the network's lots: the most built-up streets first (the urbanity at the lot,
// plus noise shared by a whole street side, so a side is built out or not), up to `cap`
// buildings. With a library the lots take the archetypes at their real size; without, procedural
// massing. Core lots get a paved forecourt to the kerb. Returns how many buildings.
std::size_t buildCity(const building_library *lib, const road_network &net,
                      const terrain_source &hf, const road_density_fn &urbanity,
                      const surface_registry &reg, std::uint64_t seed, double waterLevel, int cap,
                      cvc::geometry &out, std::vector<generated_world::apron> &aprons,
                      FootprintIndex &fps) {
  using lsys::stream;
  site_params sp;
  sp.seed = seed;
  sp.frontage = 15.0;
  sp.frontage_jitter = 0.35;
  sp.spacing = 3.0;
  sp.lot_depth = 30.0;
  sp.setback = 5.0;
  sp.junction_clear = 10.0;
  sp.block_depth = true;
  sp.min_depth = 9.0;
  sp.max_relief = 10.0;
  sp.min_ground = waterLevel + 1.0;
  sp.core_setback = 0.7;
  sp.road_margin = 0.5; // under the cores' 1.5 m setback
  sp.core_spacing = 1.0;
  sp.sparse_fill = 0.35;
  const auto h = [&hf](double x, double y) { return hf.sample(x, y); };
  std::vector<lot> lots = layout_lots(net, sp, h, urbanity);
  // A generator's own masks overrule the slope and water checks: no lot off its buildable ground
  // or on a channel or cliff.
  const bool buildable = hf.has_mask(terrain_mask::buildable),
             channel = hf.has_mask(terrain_mask::channel), cliff = hf.has_mask(terrain_mask::cliff);
  if (buildable || channel || cliff)
    lots.erase(
        std::remove_if(lots.begin(), lots.end(),
                       [&](const lot &L) {
                         return (buildable && hf.mask(terrain_mask::buildable, L.x, L.y) < 0.5) ||
                                (channel && hf.mask(terrain_mask::channel, L.x, L.y) > 0.5) ||
                                (cliff && hf.mask(terrain_mask::cliff, L.x, L.y) > 0.5);
                       }),
        lots.end());

  auto gumbel = [&](std::uint64_t el, std::uint32_t d) {
    return -std::log(-std::log(std::max(1e-12, lsys::uni(seed, stream::floorplan, el, d))));
  };
  std::vector<std::pair<double, std::size_t>> order;
  order.reserve(lots.size());
  for (std::size_t i = 0; i < lots.size(); ++i) {
    const lot &L = lots[i];
    const std::uint64_t sideEl = 0x51DEu + std::uint64_t(L.seg) * 2u + (L.side > 0 ? 1u : 0u);
    const std::uint64_t lotEl =
        lsys::cell_id(std::int32_t(std::lround(L.x)), std::int32_t(std::lround(L.y)));
    order.push_back({L.urbanity + 0.25 * gumbel(sideEl, 0) + 0.05 * gumbel(lotEl, 1), i});
  }
  std::stable_sort(order.begin(), order.end(),
                   [](const auto &a, const auto &b) { return a.first > b.first; });
  std::vector<lot> ordered;
  ordered.reserve(order.size());
  for (const auto &o : order)
    ordered.push_back(lots[o.second]);

  const std::size_t limit = std::size_t(std::max(0, cap));
  auto setbackOf = [&](const lot &L) { return sp.setback * (1.0 - sp.core_setback * L.urbanity); };
  auto forecourt = [&](const lot &L) {
    if (L.urbanity < 0.55)
      return;
    const double sb = setbackOf(L), yc = std::cos(L.yaw), ys = std::sin(L.yaw);
    const double ly = 0.5 * sb; // reach the kerb: the lot shifted toward the road by half a setback
    aprons.push_back({L.x - ly * ys, L.y + ly * yc, L.ground_min, 0.5 * L.frontage,
                      0.5 * L.depth + 0.5 * sb, L.yaw});
  };

  std::size_t count = 0;
  if (lib && !lib->empty()) {
    fit_params fp;
    fp.seed = seed;
    fp.true_scale = true;
    std::vector<building_instance> inst = fit_buildings(*lib, ordered, fp, h);
    if (inst.size() > limit)
      inst.resize(limit);
    cvc::geometry b = stamp_buildings(*lib, inst);
    if (b.const_points().empty())
      return 0;
    if (b.const_normals().size() != b.const_points().size())
      b.compute_normals();
    // The archetypes carry no material colour: one concrete tone, per vertex.
    b.colors().assign(b.const_points().size(), cvc::geometry::color_t{{0.64, 0.62, 0.59}});
    out.merge(b);
    for (const building_instance &bi : inst) {
      const building_archetype &a = lib->archetypes[bi.archetype];
      fps.add(Footprint{bi.x, bi.y, std::cos(bi.yaw), std::sin(bi.yaw),
                        a.footprint_x * bi.scale + 1.0, a.footprint_y * bi.scale + 1.0});
    }
    // fit_buildings keeps the lots' order, skipping a lot nothing fits: match them up by position.
    std::size_t j = 0;
    for (const lot &L : ordered) {
      if (j >= inst.size())
        break;
      if (std::fabs(inst[j].x - L.x) < 1e-9 && std::fabs(inst[j].y - L.y) < 1e-9) {
        forecourt(L);
        ++j;
      }
    }
    count = inst.size();
  } else {
    for (const lot &L : ordered) {
      if (count >= limit)
        break;
      fps.add(massLot(L, seed, reg, out));
      forecourt(L);
      ++count;
    }
  }
  return count;
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

heightfield_params fbm_terrain_params(const generated_world_params &gp) {
  const double half = std::max(50.0, gp.half_m);
  // Relief scaled for a large world: long base wavelength, deep enough that low ground drops below
  // the water to carve streams and ponds. A shorter base wavelength, more octaves and a higher
  // gain give steeper, more broken relief, so the slopes classify as gravel / scree / bare rock
  // rather than an all-grass plain.
  heightfield_params hp;
  hp.seed = gp.seed;
  hp.amp_m = gp.amp_m;
  hp.sea_level_m = 0.0; // terrain floor at 0; water floods above it (water_level_m)
  hp.base_wavelength_m = std::max(50.0, half * 0.30);
  hp.octaves = 8;
  hp.gain = 0.55;
  hp.lacunarity = 2.1;
  hp.warp_m = 22.0;
  hp.warp_wavelength_m = half * 0.6;
  // The island preset shapes the relief (a central dome dropping below the water line at the rim);
  // every preset's object mix is in the counts (apply_world_preset).
  if (gp.preset == int(world_preset::island)) {
    hp.island = true;
    hp.island_peak_m = std::max(gp.amp_m, 40.0);
    hp.island_radius_m = half * 0.72;
    hp.shelf_m = -std::max(6.0, gp.water_level_m + 4.0);
  }
  return hp;
}

namespace {

template <class G> struct Registry {
  explicit Registry(std::vector<std::pair<std::string, G>> builtins)
      : entries(std::move(builtins)) {}
  std::mutex m;
  std::vector<std::pair<std::string, G>> entries; // the built-ins first, then registration order
  void put(const std::string &name, G g) {
    std::lock_guard<std::mutex> lk(m);
    for (auto &e : entries)
      if (e.first == name) {
        e.second = std::move(g);
        return;
      }
    entries.emplace_back(name, std::move(g));
  }
  G get(const std::string &name, const char *what) {
    std::lock_guard<std::mutex> lk(m);
    for (const auto &e : entries)
      if (e.first == name)
        return e.second;
    std::string known;
    for (const auto &e : entries)
      known += (known.empty() ? "" : ", ") + e.first;
    throw std::invalid_argument(std::string("generate_world: no ") + what + " generator '" + name +
                                "' (known: " + known + ")");
  }
  std::vector<std::string> names() {
    std::lock_guard<std::mutex> lk(m);
    std::vector<std::string> out;
    for (const auto &e : entries)
      out.push_back(e.first);
    return out;
  }
};

Registry<terrain_generator> &terrainRegistry() {
  const terrain_generator fbm = [](const generated_world_params &p) {
    return std::shared_ptr<const terrain_source>(
        std::make_shared<const heightfield>(fbm_terrain_params(p)));
  };
  const terrain_generator dem = [](const generated_world_params &p) {
    if (p.terrain_file.empty())
      throw std::invalid_argument("generate_world: the dem terrain needs a terrain_file (.npy)");
    return std::shared_ptr<const terrain_source>(std::make_shared<const raster_heightfield>(
        load_dem(p.terrain_file, std::max(50.0, p.half_m), p.amp_m)));
  };
  static Registry<terrain_generator> r({{"fbm", fbm}, {"dem", dem}});
  return r;
}

Registry<city_generator> &cityRegistry() {
  static Registry<city_generator> r(
      {{"districts", city_generator(districtStreets)},
       {"none", [](const city_context &) { return city_streets{}; }}});
  return r;
}

} // namespace

void register_terrain_generator(const std::string &name, terrain_generator g) {
  terrainRegistry().put(name, std::move(g));
}
std::vector<std::string> terrain_generator_names() { return terrainRegistry().names(); }
std::shared_ptr<const terrain_source> make_terrain(const generated_world_params &p) {
  std::shared_ptr<const terrain_source> t = terrainRegistry().get(p.terrain, "terrain")(p);
  if (!t)
    throw std::invalid_argument("generate_world: terrain generator '" + p.terrain +
                                "' made no terrain");
  return t;
}
void register_city_generator(const std::string &name, city_generator g) {
  cityRegistry().put(name, std::move(g));
}
std::vector<std::string> city_generator_names() { return cityRegistry().names(); }

world_params to_world_params(const generated_world_params &gp) {
  const double half = std::max(50.0, gp.half_m);
  world_params wp;
  wp.min_x = wp.min_y = -half;
  wp.max_x = wp.max_y = half;
  wp.seed = gp.seed;
  wp.ontology = world_ontology_name(gp.ontology);
  wp.hf = fbm_terrain_params(gp);
  wp.terrain = make_terrain(gp);
  scatter_params &s = wp.sc;
  // Trees are the two PRESERVED demo generators (reserved recipe names, see growNativeTree), an
  // even mix: the conical pine and the recursive broadleaf, the exact look from lsystem_forest.
  s.trees = {{"native_pine", 0.5}, {"native_branchy", 0.5}};
  s.rocks = {{"boulder_cluster", 1.0}};
  s.buildings = {{"office_block", 0.5}, {"brick_house", 0.5}};
  s.tree_count = gp.trees;
  s.rock_count = gp.rocks;
  s.building_count = 0; // the city lays out its own buildings, along its roads
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
  // Both generators are looked up first, so an unknown name fails before any work.
  const city_generator city = cityRegistry().get(p.city, "city");
  const world_params wp = to_world_params(w.params);
  w.model = std::make_shared<const world_model>(world_model::generate(wp));
  const world_model &wm = *w.model;
  const terrain_source &hf = wm.terrain();
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
  // Each sample is a pure function of its position: the rows fill in parallel.
#ifdef CVC_USING_OPENMP
#pragma omp parallel for schedule(static)
#endif
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
        // The raster's samples are corner-inclusive: the nearest is at u x (rasterN - 1).
        const int rc = std::min(rasterN - 1, int(std::lround(u * (rasterN - 1))));
        const int rr = std::min(rasterN - 1, int(std::lround(v * (rasterN - 1))));
        mats[std::size_t(r) * (terrainN - 1) + c] =
            cvc::geometry::face_material_t(w.raster.klass[std::size_t(rr) * rasterN + rc]);
      }
    w.terrain->set_cell_materials(std::move(mats));
  }

  // The city, roads FIRST: the city generator's streets (by default district cores on dry, gentle
  // ground and a graded network grown from them); lots along the streets, filled from the most
  // urban out with the library's archetypes or procedural massing; asphalt burned into the raster
  // and the extruded road surface. Then the trees and rocks, off the roads and buildings.
  const auto road_h = [&hf](double x, double y) { return hf.sample(x, y); };
  const bool channel = hf.has_mask(terrain_mask::channel), cliff = hf.has_mask(terrain_mask::cliff);
  const city_context ctx{w.params, hf, [&hf, water, channel, cliff](double x, double y) {
                           return hf.sample(x, y) < water + 1.5 ||
                                  (channel && hf.mask(terrain_mask::channel, x, y) > 0.5) ||
                                  (cliff && hf.mask(terrain_mask::cliff, x, y) > 0.5);
                         }};
  city_streets streets = city(ctx);
  const road_network &net = streets.roads;
  FootprintIndex fps;
  w.building_count = buildCity(library, net, hf, streets.urbanity, reg, p.seed, water, p.buildings,
                               w.buildings, w.aprons, fps);
  w.archetypes = library && !library->empty();
  w.road_length = 0.0;
  for (const road_segment &s : net.segments)
    w.road_length +=
        std::hypot(net.nodes[s.b].x - net.nodes[s.a].x, net.nodes[s.b].y - net.nodes[s.a].y);
  w.districts = streets.districts;
  rasterize_roads(net, w.grid, reg, w.raster, 1.0);
  stampBuildings(fps, w.grid, reg, w.raster); // after the asphalt: a building always wins
  road_mesh_params rmp;
  rmp.lift = 0.35; // clearly above the terrain mesh, so the ribbons don't z-fight it
  w.roads = extrude_roads(net, rmp, road_h);

  PropLib lib;
  lib.build(wp);
  buildProps(
      wm, reg, lib,
      [&](double x, double y) { return !net.on_or_near_road(x, y, 1.5) && !fps.contains(x, y); },
      w.props, w.foliage, w.needles);
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
  double sx = (T - 1) / spanx, sy = (T - 1) / spany;
  auto W2Tx = [&](double x) { return (x - g.min_x) * sx; };
  auto W2Ty = [&](double y) { return (y - g.min_y) * sy; };
  const cvc::geometry::color_t pave = {0.52, 0.51, 0.49};
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
  const terrain_source *hf = w.model ? &w.model->terrain() : nullptr;
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
