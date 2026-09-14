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

#include <algorithm>
#include <cmath>
#include <cvc/world/raster.h>

namespace cvc {
namespace world {

world_model world_model::generate(const world_params &wp) {
  world_model wm(wp);
  wm.reg_ = &surface_registry::variant(wp.ontology);
  // The heightfield takes the world seed.
  wm.hf_ = heightfield([&] {
    heightfield_params hp = wp.hf;
    hp.seed = wp.seed ^ hp.seed;
    return hp;
  }());
  wm.props_ = scatter(wp.min_x, wp.min_y, wp.max_x, wp.max_y, wp.sc, wm.hf_, *wm.reg_, wp.seed);
  return wm;
}

namespace {

// Layer-0 classification over (height, slope), thresholds scaled to the terrain
// amplitude so a rolling ±8 m scene and an island both classify sensibly.
std::uint16_t derived_class(double h, double slope_deg, double sea, double amp) {
  const double a = amp > 1e-3 ? amp : 1.0;
  if (h < sea - 0.5)
    return 16; // water_deep (hard)
  if (h < sea)
    return 15; // water_shallow
  if (std::fabs(h - sea) < 0.6)
    return 5; // sand (shore band)
  if (slope_deg > 42.0)
    return 17; // cliff_rock (hard)
  if (slope_deg > 30.0)
    return 10; // scree
  if (h > sea + 1.6 * a)
    return 9; // bare_rock (high ground)
  if (slope_deg > 18.0)
    return 4; // gravel
  return 6;   // grass
}

// Is a world point inside a footprint?
bool covers(const footprint &f, double x, double y) {
  if (f.k == footprint::disc) {
    double dx = x - f.cx, dy = y - f.cy;
    return dx * dx + dy * dy <= f.radius * f.radius;
  }
  // Oriented box: project onto axis0 and its perpendicular.
  double dx = x - f.cx, dy = y - f.cy;
  double p0 = dx * f.ax + dy * f.ay;
  double p1 = -dx * f.ay + dy * f.ax;
  return std::fabs(p0) <= f.hx && std::fabs(p1) <= f.hy;
}

// World-space AABB of a footprint (metres), for cell-range clipping.
void aabb(const footprint &f, double &x0, double &y0, double &x1, double &y1) {
  if (f.k == footprint::disc) {
    x0 = f.cx - f.radius;
    x1 = f.cx + f.radius;
    y0 = f.cy - f.radius;
    y1 = f.cy + f.radius;
  } else {
    double ex = std::fabs(f.ax) * f.hx + std::fabs(f.ay) * f.hy;
    double ey = std::fabs(f.ay) * f.hx + std::fabs(f.ax) * f.hy;
    x0 = f.cx - ex;
    x1 = f.cx + ex;
    y0 = f.cy - ey;
    y1 = f.cy + ey;
  }
}

} // namespace

void raster(const world_model &wm, const grid_spec &g, raster_out &out) {
  const int rows = g.rows, cols = g.cols;
  const std::size_t n = std::size_t(rows) * std::size_t(cols);
  const surface_registry &reg = wm.registry();
  const heightfield &hf = wm.hf();

  out.rows = rows;
  out.cols = cols;
  out.klass.assign(n, 0);
  out.risk_raw.assign(n, 0.0f);
  out.hard.assign(n, 0);
  out.occupancy.assign(n, 0);
  out.height.assign(n, 0.0f);
  out.layer_owner.assign(n, 0);

  const double sea = hf.params().sea_level_m;
  const double amp = hf.params().island ? hf.params().island_peak_m : hf.params().amp_m;

  // ── Layer 0: derived predicates (each cell independent -> thread-invariant) ──
  for (int r = 0; r < rows; ++r) {
    const double y = g.world_y(r); // row 0 == min_y
    for (int c = 0; c < cols; ++c) {
      const double x = g.world_x(c);
      const double h = hf.sample(x, y);
      const double sl = hf.slope_deg(x, y);
      const std::size_t i = std::size_t(r) * cols + c;
      out.height[i] = float(h);
      out.klass[i] = derived_class(h, sl, sea, amp);
    }
  }

  // ── Layer 1: grammar paint (prop footprints). Two passes so occupied
  //    footprints (trunks, walls, rock) win over soft ones (canopy, ground). ──
  const double cw = g.cell_w(), ch = g.cell_h();
  auto stamp = [&](bool occupied_pass) {
    for (const placed_prop &pp : wm.props()) {
      for (const footprint &f : pp.footprints) {
        if (f.occupied != occupied_pass)
          continue;
        double x0, y0, x1, y1;
        aabb(f, x0, y0, x1, y1);
        int c0 = std::max(0, int(std::floor((x0 - g.min_x) / cw)));
        int c1 = std::min(cols - 1, int(std::ceil((x1 - g.min_x) / cw)));
        int r0 = std::max(0, int(std::floor((y0 - g.min_y) / ch)));
        int r1 = std::min(rows - 1, int(std::ceil((y1 - g.min_y) / ch)));
        for (int r = r0; r <= r1; ++r) {
          const double y = g.world_y(r);
          for (int c = c0; c <= c1; ++c) {
            const double x = g.world_x(c);
            if (!covers(f, x, y))
              continue;
            const std::size_t i = std::size_t(r) * cols + c;
            out.klass[i] = f.klass;
            out.layer_owner[i] = 1;
            if (f.occupied)
              out.occupancy[i] = 1;
          }
        }
      }
    }
  };
  stamp(false); // soft footprints first
  stamp(true);  // occupied footprints override

  // ── Derive risk_raw / hard from the final class map; enforce hard ⊆ occ. ──
  for (std::size_t i = 0; i < n; ++i) {
    const surface_class &sc = reg[out.klass[i]];
    out.risk_raw[i] = sc.rho;
    out.hard[i] = sc.hard ? 1 : 0;
    if (sc.hard)
      out.occupancy[i] = 1;
  }
}

} // namespace world
} // namespace cvc
