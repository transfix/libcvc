/*
  Copyright 2026 The University of Texas at Austin

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

// gust_front.h — a storm cell's gust front, the wind its outflow adds at a point
// (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md F4), in one place: the weather sums it over its cells
// (cvc::world::weather::gust_front, the physics' air), the sea's ripples read it where each wave
// is (cvc::ocean::sea_forcing::fronts), and the shaders evaluate its GLSL twin per vertex
// (cvc/gl/StormFrontGLSL.h: the water and the trees).
//
//   cvc::algo::front_cell c{x, y, r, vx, vy, out, {1.5, 0.9, 0.6}};
//   double fx = 0, fy = 0;
//   cvc::algo::add_gust_front(c, px, py, fx, fy);       // m/s along x and y
//
// The cell blows out of its centre: its outflow rises as q (2 - q) from nothing at the centre to
// `out` at the front's edge, then dies away as (1 - u^2)^2 over the next `width` of the edge's
// distance, so the wind is smooth and exactly 0 past its reach. The edge is `ahead` radii ahead of
// the cell along its motion and `behind` radii behind it, between them by the cosine off the
// motion (a still cell's is round), and the outflow is weaker behind by their ratio.
//
// Header-only and standard library only, so each library compiles the same arithmetic: + - * /
// and sqrt, correctly rounded, so every platform gives the same bits.

#ifndef CVC_ALGO_GUST_FRONT_H
#define CVC_ALGO_GUST_FRONT_H

#include <algorithm>
#include <cmath>

namespace cvc {
namespace algo {

struct front_cell {
  double x = 0.0, y = 0.0;   // its centre (m)
  double r = 0.0;            // its radius (m)
  double vx = 0.0, vy = 0.0; // its motion (any unit: only the way it moves counts)
  double out = 0.0;          // its outflow at the front's edge (m/s): the front's peak x strength
  double ahead = 1.5, behind = 0.9, width = 0.6; // the front's shape (each above 0)
};

// The cell's outflow at (x, y), added to (vx, vy) (m/s).
inline void add_gust_front(const front_cell &c, double x, double y, double &vx, double &vy) {
  if (!(c.out > 0.0) || !(c.r > 0.0))
    return;
  const double most = std::max(c.ahead, c.behind);
  const double dx = x - c.x, dy = y - c.y, d2 = dx * dx + dy * dy;
  const double reach = c.r * most * (1.0 + c.width);
  if (!(d2 > 0.0) || d2 >= reach * reach)
    return;
  const double d = std::sqrt(d2);
  // The front's edge this way, `ahead` of the radius along the cell's motion, `behind` against it,
  // between them by the cosine of the angle off the motion.
  const double m = std::sqrt(c.vx * c.vx + c.vy * c.vy);
  const double along = m > 0.0 ? (dx * c.vx + dy * c.vy) / (d * m) : 0.0;
  const double share = 0.5 * (c.ahead + c.behind) + 0.5 * (c.ahead - c.behind) * along;
  // Out from the centre: rising to the edge (q (2 - q), flat there), then dying away over `width`
  // of the edge's distance ((1 - u^2)^2, flat at both ends).
  const double q = d / (c.r * share);
  double shape = q * (2.0 - q);
  if (q > 1.0) {
    const double u = (q - 1.0) / c.width;
    if (u >= 1.0)
      return;
    shape = (1.0 - u * u) * (1.0 - u * u);
  }
  const double out = c.out * shape * (share / most);
  vx += out * dx / d;
  vy += out * dy / d;
}

// How much the wind at (x, y) is of the wind at the reference point, with the fronts in both:
// |U + front(x, y)| / |U + front at the reference|, `ref` that reference speed (m/s; read as at
// least 0.5), clamped to [0, 4]. U = (ux, uy) is the mean wind the fronts add to. The sea's
// ripples and the drawn roughness scale by it (F4): a gust front where each wave is.
template <class Cells>
double front_wind_ratio(const Cells &cells, double ux, double uy, double ref, double x, double y) {
  double vx = ux, vy = uy;
  for (const front_cell &c : cells)
    add_gust_front(c, x, y, vx, vy);
  return std::clamp(std::sqrt(vx * vx + vy * vy) / std::max(ref, 0.5), 0.0, 4.0);
}

} // namespace algo
} // namespace cvc

#endif // CVC_ALGO_GUST_FRONT_H
