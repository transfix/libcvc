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

#include <cmath>
#include <cvc/lsys/derive.h>
#include <cvc/lsys/interp.h>
#include <cvc/lsys/recipes.h>
#include <cvc/lsys/rng.h>
#include <cvc/world/scatter.h>
#include <map>

namespace cvc {
namespace world {

namespace {

using cvc::lsys::stream;

// Reduce an interpreted structure (at unit scale, origin) to ground footprints.
std::vector<footprint> footprints_of(const cvc::lsys::structure &s, const surface_registry &reg) {
  using role = cvc::lsys::role;
  std::vector<footprint> out;

  // Trunk disc: near-ground woody segments -> one occupied disc.
  double bx = 0, by = 0, br = 0;
  int nb = 0;
  for (const cvc::lsys::segment &sg : s.segments) {
    if (sg.rl == role::trunk || sg.rl == role::branch || sg.rl == role::wood_solid) {
      double z = std::min(sg.a.z, sg.b.z);
      if (z < 2.0) {
        bx += sg.a.x;
        by += sg.a.y;
        ++nb;
        br = std::max(br, double(sg.r0));
      }
    }
  }
  if (nb > 0) {
    footprint f;
    f.k = footprint::disc;
    f.cx = bx / nb;
    f.cy = by / nb;
    f.radius = std::max(0.15, br);
    f.klass = reg.class_for_role(role::trunk);
    f.occupied = reg[f.klass].hard;
    out.push_back(f);
  }

  // Canopy disc: from leaves (soft foliage, not occupied).
  if (!s.leaves.empty()) {
    double cx = 0, cy = 0;
    for (const cvc::lsys::leaf &l : s.leaves) {
      cx += l.pos.x;
      cy += l.pos.y;
    }
    cx /= double(s.leaves.size());
    cy /= double(s.leaves.size());
    double rad = 0;
    for (const cvc::lsys::leaf &l : s.leaves)
      rad = std::max(rad, std::hypot(l.pos.x - cx, l.pos.y - cy) + 0.5 * l.size);
    footprint f;
    f.k = footprint::disc;
    f.cx = cx;
    f.cy = cy;
    f.radius = std::max(0.5, rad);
    f.klass = reg.class_for_role(role::foliage);
    f.occupied = reg[f.klass].hard;
    out.push_back(f);
  }

  // Boxes -> oriented-box footprints (buildings, rock chunks).
  for (const cvc::lsys::obox &b : s.boxes) {
    footprint f;
    f.k = footprint::obox;
    f.cx = b.center.x;
    f.cy = b.center.y;
    double a0x = b.axis[0].x, a0y = b.axis[0].y;
    double n = std::hypot(a0x, a0y);
    if (n < 1e-9) {
      a0x = 1;
      a0y = 0;
      n = 1;
    }
    f.ax = a0x / n;
    f.ay = a0y / n;
    // Half-extents projected to the ground plane (axes are near-in-plane for
    // buildings; tilted rock chunks lose their vertical component here, which is
    // the correct footprint).
    f.hx = std::max(0.1, b.half.x * std::hypot(b.axis[0].x, b.axis[0].y));
    f.hy = std::max(0.1, b.half.y * std::hypot(b.axis[1].x, b.axis[1].y));
    f.klass = reg.class_for_role(b.rl);
    f.occupied = reg[f.klass].hard;
    out.push_back(f);
  }

  // Paint stamps -> ground material discs.
  for (const cvc::lsys::paint2d &p : s.paints) {
    footprint f;
    f.k = footprint::disc;
    f.cx = p.a.x;
    f.cy = p.a.y;
    f.radius = std::max(0.2, p.radius);
    f.klass = reg.class_for_role(p.rl);
    f.occupied = reg[f.klass].hard;
    out.push_back(f);
  }
  return out;
}

footprint transform(footprint f, double px, double py, double scale, double yaw_deg) {
  const double a = yaw_deg * 0.017453292519943295;
  const double c = std::cos(a), s = std::sin(a);
  auto rot = [&](double x, double y, double &ox, double &oy) {
    ox = x * c - y * s;
    oy = x * s + y * c;
  };
  double ncx, ncy;
  rot(f.cx * scale, f.cy * scale, ncx, ncy);
  f.cx = px + ncx;
  f.cy = py + ncy;
  f.radius *= scale;
  if (f.k == footprint::obox) {
    double nax, nay;
    rot(f.ax, f.ay, nax, nay);
    f.ax = nax;
    f.ay = nay;
    f.hx *= scale;
    f.hy *= scale;
  }
  return f;
}

const std::string &pick(const std::vector<species_weight> &sp, double u) {
  double tot = 0;
  for (const species_weight &w : sp)
    tot += w.weight > 0 ? w.weight : 0;
  double x = u * (tot > 0 ? tot : 1.0), acc = 0;
  for (const species_weight &w : sp) {
    acc += w.weight > 0 ? w.weight : 0;
    if (x < acc)
      return w.recipe;
  }
  return sp.back().recipe;
}

// Cache local footprints per recipe (recipes are deterministic, so one
// derivation per recipe suffices; per-instance variety comes from scale/yaw).
struct recipe_cache {
  const surface_registry &reg;
  int gen;
  std::map<std::string, std::vector<footprint>> local;
  const std::vector<footprint> &get(const std::string &name) {
    auto it = local.find(name);
    if (it != local.end())
      return it->second;
    cvc::lsys::ruleset rs = cvc::lsys::load_recipe(name);
    cvc::lsys::derive_options o;
    o.generations = gen;
    o.master_seed = 0;
    cvc::lsys::structure s = cvc::lsys::interpret(rs, cvc::lsys::derive(rs, o).w);
    return local.emplace(name, footprints_of(s, reg)).first->second;
  }
};

void place_category(std::vector<placed_prop> &out, const std::vector<species_weight> &species,
                    int count, int gen, stream strm, double min_x, double min_y, double max_x,
                    double max_y, const scatter_params &sp, const heightfield &hf,
                    const surface_registry &reg, std::uint64_t seed) {
  if (species.empty() || count <= 0)
    return;
  const int gridN = std::max(1, int(std::lround(std::sqrt(double(count)))));
  const double cw = (max_x - min_x) / gridN;
  const double ch = (max_y - min_y) / gridN;
  recipe_cache cache{reg, gen, {}};

  for (int gy = 0; gy < gridN; ++gy)
    for (int gx = 0; gx < gridN; ++gx) {
      // Fold the category into the element so the species/size/phase draws (which
      // use their own fixed streams) do not correlate a tree with a rock in the
      // same grid cell. Still a pure function of (cell, category) -> insertion-stable.
      const std::uint64_t el =
          cvc::lsys::path_id(cvc::lsys::cell_id(gx, gy), static_cast<std::uint32_t>(strm));
      const double jx = (cvc::lsys::uni(seed, strm, el, 0) - 0.5) * sp.jitter * cw;
      const double jy = (cvc::lsys::uni(seed, strm, el, 1) - 0.5) * sp.jitter * ch;
      const double x = min_x + (gx + 0.5) * cw + jx;
      const double y = min_y + (gy + 0.5) * ch + jy;
      const double h = hf.sample(x, y);
      if (h < sp.min_ground_m)
        continue; // water / shore — skip (insertion-stable: per-cell decision)
      const std::string &recipe = pick(species, cvc::lsys::uni(seed, stream::species, el, 2));
      const double scale = cvc::lsys::uni(seed, stream::size, el, 3, sp.size_min, sp.size_max);
      const double yaw = cvc::lsys::uni(seed, stream::phase, el, 4, 0.0, 360.0);

      placed_prop pp;
      pp.x = x;
      pp.y = y;
      pp.z = h;
      pp.scale = scale;
      pp.yaw_deg = yaw;
      pp.recipe = recipe;
      for (const footprint &lf : cache.get(recipe))
        pp.footprints.push_back(transform(lf, x, y, scale, yaw));
      out.push_back(std::move(pp));
    }
}

} // namespace

scatter_params scatter_params::defaults() {
  scatter_params p;
  p.trees = {{"pine_monopodial", 1.0},
             {"oak_sympodial", 1.0},
             {"birch_slender", 0.8},
             {"shrub_bush", 0.6}};
  p.rocks = {{"boulder_cluster", 1.0}};
  p.buildings = {{"office_block", 1.0}, {"brick_house", 1.2}};
  return p;
}

std::vector<placed_prop> scatter(double min_x, double min_y, double max_x, double max_y,
                                 const scatter_params &sp, const heightfield &hf,
                                 const surface_registry &reg, std::uint64_t seed) {
  std::vector<placed_prop> out;
  // Distinct streams keep the three categories independent and insertion-stable.
  place_category(out, sp.trees, sp.tree_count, sp.tree_gen, stream::placement, min_x, min_y, max_x,
                 max_y, sp, hf, reg, seed);
  place_category(out, sp.rocks, sp.rock_count, sp.rock_gen, stream::rock, min_x, min_y, max_x,
                 max_y, sp, hf, reg, seed);
  place_category(out, sp.buildings, sp.building_count, sp.building_gen, stream::building, min_x,
                 min_y, max_x, max_y, sp, hf, reg, seed);
  return out;
}

} // namespace world
} // namespace cvc
