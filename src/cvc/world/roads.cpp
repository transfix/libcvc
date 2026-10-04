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

// roads.cpp — the tensor-field street network (see roads.h). Deterministic (hashed RNG,
// stream::road; no <random>) and GL-free. Proximity is brute-force over the segment list — the
// network is capped at a few thousand segments, so the O(N^2) accept loop is well under a second; a
// spatial hash is a follow-up optimisation, not a correctness requirement.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/lsys/rng.h>
#include <cvc/world/roads.h>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

namespace cvc {
namespace world {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct V2 {
  double x = 0, y = 0;
};
V2 operator+(V2 a, V2 b) { return {a.x + b.x, a.y + b.y}; }
V2 operator-(V2 a, V2 b) { return {a.x - b.x, a.y - b.y}; }
V2 operator*(V2 a, double s) { return {a.x * s, a.y * s}; }
double dot(V2 a, V2 b) { return a.x * b.x + a.y * b.y; }
double len(V2 a) { return std::sqrt(dot(a, a)); }
double cross(V2 a, V2 b) { return a.x * b.y - a.y * b.x; }

// Closest point on segment [a,b] to p: returns the parameter t in [0,1] and the distance.
double point_seg(V2 p, V2 a, V2 b, double &t) {
  const V2 ab = b - a;
  const double L2 = dot(ab, ab);
  t = L2 > 1e-12 ? std::max(0.0, std::min(1.0, dot(p - a, ab) / L2)) : 0.0;
  const V2 c = a + ab * t;
  return len(p - c);
}

// Proper segment-segment intersection (interiors only). Returns true + the point and the parameters
// ta (on p1p2), tb (on p3p4) when the two segments cross strictly between their endpoints.
bool seg_seg(V2 p1, V2 p2, V2 p3, V2 p4, V2 &out, double &ta, double &tb) {
  const V2 r = p2 - p1, s = p4 - p3;
  const double d = cross(r, s);
  if (std::fabs(d) < 1e-12)
    return false; // parallel / collinear
  ta = cross(p3 - p1, s) / d;
  tb = cross(p3 - p1, r) / d;
  const double e = 1e-6;
  if (ta <= e || ta >= 1.0 - e || tb <= e || tb >= 1.0 - e)
    return false; // touches at an endpoint or outside -> not an interior crossing
  out = p1 + r * ta;
  return true;
}

// The tensor field's major direction at (x,y): a grid direction blended with the terrain contour
// (perpendicular to the height gradient), weighted by local slope.
V2 field_major(const road_params &p, const road_height_fn &h, double x, double y) {
  const double ga = p.grid_angle_deg * kPi / 180.0;
  const V2 g{std::cos(ga), std::sin(ga)};
  V2 t = g;
  double w = 0.0;
  if (h) {
    const double e = 1.0;
    const double hx = (h(x + e, y) - h(x - e, y)) / (2.0 * e);
    const double hy = (h(x, y + e) - h(x, y - e)) / (2.0 * e);
    const double gmag = std::sqrt(hx * hx + hy * hy);
    if (gmag > 1e-6) {
      t = V2{-hy / gmag, hx / gmag}; // contour = perpendicular to the gradient
      const double slope_deg = std::atan(gmag) * 180.0 / kPi;
      w = p.terrain_weight * std::min(1.0, slope_deg / std::max(1e-3, p.terrain_slope_ref_deg));
    }
  }
  if (dot(t, g) < 0) // a road direction is unoriented (mod 180) — align hemispheres before blending
    t = t * -1.0;
  V2 m = g * (1.0 - w) + t * w;
  const double L = len(m);
  return L > 1e-9 ? V2{m.x / L, m.y / L} : g;
}
V2 perp(V2 d) { return {-d.y, d.x}; }

// A candidate segment to grow: from an existing node, in a direction, for a length.
struct Cand {
  double t = 0;  // priority (smaller first)
  int from = -1; // start node index
  V2 dir{1, 0};  // unit growth direction
  double length = 0;
  bool arterial = false;
};
struct CandCmp {
  bool operator()(const Cand &a, const Cand &b) const { return a.t > b.t; } // min-heap on t
};

} // namespace

double road_network::distance_to_road(double x, double y, int *seg, double *t) const {
  double best = 1e30;
  int bi = -1;
  double bt = 0;
  const V2 p{x, y};
  for (std::size_t i = 0; i < segments.size(); ++i) {
    double tt;
    const double d = point_seg(p, V2{nodes[segments[i].a].x, nodes[segments[i].a].y},
                               V2{nodes[segments[i].b].x, nodes[segments[i].b].y}, tt);
    if (d < best) {
      best = d;
      bi = int(i);
      bt = tt;
    }
  }
  if (seg)
    *seg = bi;
  if (t)
    *t = bt;
  return best;
}

bool road_network::on_or_near_road(double x, double y, double margin) const {
  int si = -1;
  double tt = 0;
  const double d = distance_to_road(x, y, &si, &tt);
  if (si < 0)
    return false;
  return d <= segments[si].width * 0.5 + margin;
}

void road_field_dir(const road_params &p, const road_height_fn &height, double x, double y,
                    double &dx, double &dy) {
  const V2 d = field_major(p, height, x, y);
  dx = d.x;
  dy = d.y;
}

road_network generate_roads(const road_params &p, const road_height_fn &height) {
  road_network net;
  net.half = p.half;
  if (p.half <= 0 || p.segment_len <= 0)
    return net;

  std::uint32_t draw = 0; // deterministic draw counter (seed+counter => reproducible network)
  auto uni = [&](double lo, double hi) {
    return cvc::lsys::uni(p.seed, cvc::lsys::stream::road, 0, draw++, lo, hi);
  };

  // Add a node (deduped against existing nodes within a tiny epsilon is unnecessary — callers
  // snap).
  auto add_node = [&](V2 pt) {
    net.nodes.push_back(road_node{pt.x, pt.y, 0});
    return int(net.nodes.size()) - 1;
  };
  auto node_pos = [&](int i) { return V2{net.nodes[i].x, net.nodes[i].y}; };
  auto connected = [&](int a, int b) {
    for (const road_segment &s : net.segments)
      if ((s.a == a && s.b == b) || (s.a == b && s.b == a))
        return true;
    return false;
  };

  // Try to accept a candidate: apply the local constraints (crossing cut, node/segment snap,
  // near-miss extend) and, if valid, append the segment. Returns the end node index, or -1.
  auto accept = [&](const Cand &c) -> int {
    const V2 a = node_pos(c.from);
    V2 end = a + c.dir * c.length;
    // Clip to the square region.
    end.x = std::max(-p.half, std::min(p.half, end.x));
    end.y = std::max(-p.half, std::min(p.half, end.y));
    if (len(end - a) < p.min_segment)
      return -1;

    // 1) Crossing: if a-end strictly crosses an existing segment, cut at the nearest crossing and
    //    split that segment there (forms a 4-way / T).
    int cut_seg = -1;
    double cut_ta = 2.0;
    V2 cut_pt{};
    for (std::size_t i = 0; i < net.segments.size(); ++i) {
      V2 ip;
      double ta, tb;
      if (seg_seg(a, end, node_pos(net.segments[i].a), node_pos(net.segments[i].b), ip, ta, tb)) {
        if (ta < cut_ta) {
          cut_ta = ta;
          cut_seg = int(i);
          cut_pt = ip;
          (void)tb;
        }
      }
    }
    if (cut_seg >= 0)
      end = cut_pt;
    if (len(end - a) < p.min_segment)
      return -1;

    int b = -1;
    if (cut_seg >= 0) {
      // split the crossed segment at the crossing point into a shared junction node
      road_segment &old = net.segments[cut_seg];
      const int oa = old.a, ob = old.b;
      const double ow = old.width;
      const bool oart = old.arterial;
      b = add_node(end);
      old.b = b;                                             // reuse the slot as oa->b
      net.segments.push_back(road_segment{b, ob, ow, oart}); // b->ob
      net.nodes[b].degree += 2;
    } else {
      // 2) snap the free end to the nearest existing node within snap_dist
      int best_node = -1;
      double best_nd = p.snap_dist;
      for (int i = 0; i < int(net.nodes.size()); ++i) {
        if (i == c.from)
          continue;
        const double d = len(end - node_pos(i));
        if (d < best_nd) {
          best_nd = d;
          best_node = i;
        }
      }
      if (best_node >= 0) {
        b = best_node;
      } else {
        // 3) else snap/extend onto the nearest existing segment within snap_dist (T-junction)
        int best_seg = -1;
        double best_sd = p.snap_dist, best_t = 0;
        for (std::size_t i = 0; i < net.segments.size(); ++i) {
          double tt;
          const double d =
              point_seg(end, node_pos(net.segments[i].a), node_pos(net.segments[i].b), tt);
          if (d < best_sd && tt > 0.05 && tt < 0.95) {
            best_sd = d;
            best_seg = int(i);
            best_t = tt;
          }
        }
        if (best_seg >= 0) {
          road_segment &old = net.segments[best_seg];
          const int oa = old.a, ob = old.b;
          const double ow = old.width;
          const bool oart = old.arterial;
          const V2 sp = node_pos(oa) + (node_pos(ob) - node_pos(oa)) * best_t;
          b = add_node(sp);
          old.b = b;
          net.segments.push_back(road_segment{b, ob, ow, oart});
          net.nodes[b].degree += 2;
        } else {
          b = add_node(end); // 4) a fresh endpoint
        }
      }
    }
    if (b == c.from || len(node_pos(b) - a) < p.min_segment || connected(c.from, b))
      return -1;

    const double width = c.arterial ? p.arterial_width : p.local_width;
    net.segments.push_back(road_segment{c.from, b, width, c.arterial});
    net.nodes[c.from].degree += 1;
    net.nodes[b].degree += 1;
    return b;
  };

  std::priority_queue<Cand, std::vector<Cand>, CandCmp> Q;
  // Seed the two principal arterials through the centre (major + minor), each way.
  const int center = add_node(V2{0, 0});
  {
    const V2 mj = field_major(p, height, 0, 0);
    const V2 mn = perp(mj);
    for (V2 d : {mj, mj * -1.0, mn, mn * -1.0})
      Q.push(Cand{0.0, center, d, p.segment_len, /*arterial=*/true});
  }

  while (!Q.empty() && int(net.segments.size()) < p.max_segments) {
    const Cand c = Q.top();
    Q.pop();
    const int b = accept(c);
    if (b < 0)
      continue;
    const V2 bp = node_pos(b);
    // priority grows with distance from the core (so the dense centre is laid first)
    const double core_t = len(bp) / std::max(1.0, p.core_radius);

    // Continue straight: re-sample the field at b so the road follows the terrain.
    {
      V2 d = field_major(p, height, bp.x, bp.y);
      if (dot(d, c.dir) < 0)
        d = d * -1.0; // keep heading the same way
      Q.push(Cand{c.t + 1.0 + core_t, b, d, p.segment_len, c.arterial});
    }
    // Branch perpendicular (a local street), spaced by block_size — skip if a parallel road is
    // already within ~half a block of where the branch would land.
    const double spacing = c.arterial ? p.block_size : p.local_block;
    if (uni(0.0, 1.0) < p.branch_chance) {
      const V2 mn = perp(field_major(p, height, bp.x, bp.y));
      for (V2 d : {mn, mn * -1.0}) {
        const V2 probe = bp + d * (spacing * 0.9);
        if (std::fabs(probe.x) > p.half || std::fabs(probe.y) > p.half)
          continue;
        if (net.distance_to_road(probe.x, probe.y) < spacing * 0.55)
          continue; // keep blocks from collapsing together
        // branches off an arterial are local streets; local branches stay local
        Q.push(Cand{c.t + 2.0 + core_t, b, d, p.segment_len, /*arterial=*/false});
      }
    }
  }
  return net;
}

std::string roads_to_svg(const road_network &net) {
  const double H = net.half, S = 2.0 * H;
  std::ostringstream o;
  o << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"" << -H << " " << -H << " " << S << " "
    << S << "\">\n";
  o << "<rect x=\"" << -H << "\" y=\"" << -H << "\" width=\"" << S << "\" height=\"" << S
    << "\" fill=\"#e8e6df\"/>\n";
  for (const road_segment &s : net.segments) {
    const road_node &a = net.nodes[s.a], &b = net.nodes[s.b];
    o << "<line x1=\"" << a.x << "\" y1=\"" << a.y << "\" x2=\"" << b.x << "\" y2=\"" << b.y
      << "\" stroke=\"" << (s.arterial ? "#555" : "#999") << "\" stroke-width=\"" << s.width
      << "\" stroke-linecap=\"round\"/>\n";
  }
  for (const road_node &n : net.nodes)
    if (n.degree > 2)
      o << "<circle cx=\"" << n.x << "\" cy=\"" << n.y << "\" r=\"2.5\" fill=\"#c33\"/>\n";
  o << "</svg>\n";
  return o.str();
}

} // namespace world
} // namespace cvc
