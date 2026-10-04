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

// road_mesh.cpp — see road_mesh.h. GL-free; no RNG (a pure function of the network).

#include <cmath>
#include <cstdint>
#include <cvc/world/road_mesh.h>
#include <vector>

namespace cvc {
namespace world {
namespace {

struct builder {
  cvc::geometry &g;
  std::uint64_t base() const { return g.const_points().size(); }
  void vert(double x, double y, double z, const double rgb[3]) {
    g.points().push_back(cvc::geometry::point_t{{x, y, z}});
    g.normals().push_back(cvc::geometry::normal_t{{0.0, 0.0, 1.0}}); // roads lie ~flat
    g.colors().push_back(cvc::geometry::color_t{{rgb[0], rgb[1], rgb[2]}});
  }
  void tri(std::uint64_t a, std::uint64_t b, std::uint64_t c) {
    g.tris().push_back(cvc::geometry::tri_t{{a, b, c}});
  }
};

} // namespace

cvc::geometry extrude_roads(const road_network &net, const road_mesh_params &p,
                            const road_height_fn &height) {
  cvc::geometry g;
  if (net.segments.empty())
    return g;
  builder b{g};
  const int N = int(net.nodes.size());
  auto z_at = [&](double x, double y) { return (height ? height(x, y) : 0.0) + p.lift; };

  // One quad ribbon per segment: the segment swept perpendicular by its half-width.
  for (const road_segment &s : net.segments) {
    if (s.a < 0 || s.b < 0 || s.a >= N || s.b >= N)
      continue;
    const road_node &na = net.nodes[s.a];
    const road_node &nb = net.nodes[s.b];
    double dx = nb.x - na.x, dy = nb.y - na.y;
    const double len = std::hypot(dx, dy);
    if (len < 1e-6)
      continue;
    dx /= len;
    dy /= len;
    const double nx = -dy, ny = dx; // perpendicular
    const double hw = 0.5 * s.width;
    const double *rgb = s.arterial ? p.arterial_rgb : p.local_rgb;

    const double ax0 = na.x + nx * hw, ay0 = na.y + ny * hw;
    const double ax1 = na.x - nx * hw, ay1 = na.y - ny * hw;
    const double bx0 = nb.x + nx * hw, by0 = nb.y + ny * hw;
    const double bx1 = nb.x - nx * hw, by1 = nb.y - ny * hw;

    const std::uint64_t v = b.base();
    b.vert(ax0, ay0, z_at(ax0, ay0), rgb); // v+0
    b.vert(ax1, ay1, z_at(ax1, ay1), rgb); // v+1
    b.vert(bx0, by0, z_at(bx0, by0), rgb); // v+2
    b.vert(bx1, by1, z_at(bx1, by1), rgb); // v+3
    b.tri(v + 0, v + 1, v + 2);
    b.tri(v + 2, v + 1, v + 3);
  }

  // A rounded pad at every junction (degree >= 2), sized to the widest road meeting there, so the
  // ribbons join cleanly. Also record the pad colour (arterial if any incident road is arterial).
  if (p.intersections && p.pad_segments >= 3) {
    std::vector<double> node_hw(N, 0.0);
    std::vector<char> node_art(N, 0);
    std::vector<int> node_deg(N, 0);
    for (const road_segment &s : net.segments) {
      if (s.a < 0 || s.b < 0 || s.a >= N || s.b >= N)
        continue;
      const double hw = 0.5 * s.width;
      for (int e : {s.a, s.b}) {
        node_deg[e]++;
        if (hw > node_hw[e])
          node_hw[e] = hw;
        if (s.arterial)
          node_art[e] = 1;
      }
    }
    const double TWO_PI = 6.283185307179586476925286766559;
    for (int i = 0; i < N; ++i) {
      if (node_deg[i] < 2 || node_hw[i] <= 0.0)
        continue;
      const road_node &nd = net.nodes[i];
      const double *rgb = node_art[i] ? p.arterial_rgb : p.local_rgb;
      const double r = node_hw[i];
      const std::uint64_t c = b.base();
      b.vert(nd.x, nd.y, z_at(nd.x, nd.y), rgb); // centre
      for (int k = 0; k < p.pad_segments; ++k) {
        const double a = TWO_PI * double(k) / double(p.pad_segments);
        const double x = nd.x + r * std::cos(a), y = nd.y + r * std::sin(a);
        b.vert(x, y, z_at(x, y), rgb);
      }
      for (int k = 0; k < p.pad_segments; ++k) {
        const std::uint64_t r0 = c + 1 + k;
        const std::uint64_t r1 = c + 1 + (k + 1) % p.pad_segments;
        b.tri(c, r0, r1);
      }
    }
  }
  return g;
}

} // namespace world
} // namespace cvc
