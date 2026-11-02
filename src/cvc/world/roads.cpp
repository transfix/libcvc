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
// stream::road; no <random>) and GL-free. Proximity queries — both the per-candidate local
// constraints during generation and the public distance_to_road — go through a uniform-grid index
// (struct Grid), so a city-scale network builds in ~O(segments) rather than O(segments^2). The grid
// is a candidate filter whose results are byte-identical to a brute-force scan (ascending candidate
// order gives the same minimum and the same lowest-index tie-break), so the network is unchanged.

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

V2 rotate(V2 d, double rad) {
  const double c = std::cos(rad), s = std::sin(rad);
  return {d.x * c - d.y * s, d.x * s + d.y * c};
}

// A candidate segment to grow: from an existing node, in a direction, for a length.
struct Cand {
  double t = 0;  // priority (smaller first)
  int from = -1; // start node index
  V2 dir{1, 0};  // unit growth direction
  double length = 0;
  road_class kind = road_class::local;
  double run = 0; // how far the road has run since it last branched
};
struct CandCmp {
  bool operator()(const Cand &a, const Cand &b) const { return a.t > b.t; } // min-heap on t
};

// A uniform-grid spatial index of item ids (segment or node indices) over the square
// [-half,half]^2. It is a candidate FILTER, not an approximation: a query returns a de-duplicated,
// ASCENDING-sorted superset of the ids that could match, and the caller computes the exact test on
// each — so results are byte-identical to a brute-force scan (same minimum, same lowest-index
// tie-break). An item is registered in every cell its bounding box touches, so a long segment is
// found from any nearby cell.
struct Grid {
  double minx = 0, miny = 0, inv_cell = 1.0; // cells per metre
  int cols = 1, rows = 1;
  std::vector<std::vector<int>> cells;

  void init(double half, double target_cell) {
    const int n =
        std::max(1, std::min(1024, int(std::lround(2.0 * half / std::max(1.0, target_cell)))));
    minx = miny = -half;
    cols = rows = n;
    inv_cell = (n > 0 ? double(n) : 1.0) / (2.0 * half);
    cells.assign(std::size_t(cols) * rows, {});
  }
  int cx(double x) const {
    int c = int((x - minx) * inv_cell);
    return c < 0 ? 0 : (c >= cols ? cols - 1 : c);
  }
  int cy(double y) const {
    int r = int((y - miny) * inv_cell);
    return r < 0 ? 0 : (r >= rows ? rows - 1 : r);
  }
  void add_segment(int id, double x0, double y0, double x1, double y1) {
    const int c0 = cx(std::min(x0, x1)), c1 = cx(std::max(x0, x1));
    const int r0 = cy(std::min(y0, y1)), r1 = cy(std::max(y0, y1));
    for (int r = r0; r <= r1; ++r)
      for (int c = c0; c <= c1; ++c)
        cells[std::size_t(r) * cols + c].push_back(id);
  }
  void add_point(int id, double x, double y) {
    cells[std::size_t(cy(y)) * cols + cx(x)].push_back(id);
  }
  // Gather the de-duplicated, ascending ids whose cell overlaps [x-r,x+r] x [y-r,y+r] into `buf`.
  void gather(double x, double y, double r, std::vector<int> &buf) const {
    buf.clear();
    const int c0 = cx(x - r), c1 = cx(x + r), r0 = cy(y - r), r1 = cy(y + r);
    for (int rr = r0; rr <= r1; ++rr)
      for (int cc = c0; cc <= c1; ++cc) {
        const std::vector<int> &v = cells[std::size_t(rr) * cols + cc];
        buf.insert(buf.end(), v.begin(), v.end());
      }
    std::sort(buf.begin(), buf.end());
    buf.erase(std::unique(buf.begin(), buf.end()), buf.end());
  }
};

// The lazy proximity index cached on a road_network for its public distance/near queries.
struct road_index {
  Grid g;
  explicit road_index(const road_network &net) {
    // Cell ~ a fraction of the region, bounded, so a typical query touches a handful of cells.
    g.init(net.half, std::max(6.0, net.half / 128.0));
    for (std::size_t i = 0; i < net.segments.size(); ++i) {
      const road_segment &s = net.segments[i];
      if (s.a < 0 || s.b < 0 || s.a >= int(net.nodes.size()) || s.b >= int(net.nodes.size()))
        continue;
      g.add_segment(int(i), net.nodes[s.a].x, net.nodes[s.a].y, net.nodes[s.b].x, net.nodes[s.b].y);
    }
  }
};

} // namespace

double road_network::distance_to_road(double x, double y, int *seg, double *t) const {
  if (segments.empty()) {
    if (seg)
      *seg = -1;
    if (t)
      *t = 0;
    return 1e30;
  }
  // Build / refresh the cached grid index when the segment count changes.
  if (idx_nseg_ != segments.size()) {
    idx_cache_ = std::make_shared<road_index>(*this);
    idx_nseg_ = segments.size();
  }
  const Grid &g = static_cast<const road_index *>(idx_cache_.get())->g;
  const double cell = g.inv_cell > 0 ? 1.0 / g.inv_cell : (2.0 * half);
  const V2 p{x, y};
  double best = 1e30;
  int bi = -1;
  double bt = 0;
  auto consider = [&](int i) {
    const road_segment &s = segments[i];
    double tt;
    const double d =
        point_seg(p, V2{nodes[s.a].x, nodes[s.a].y}, V2{nodes[s.b].x, nodes[s.b].y}, tt);
    if (d < best) {
      best = d;
      bi = i;
      bt = tt;
    }
  };
  // Fast path: scan a local window. If the nearest road found is within the window radius, it is
  // the global nearest (nothing outside the window can be closer). The window (a few cells, at
  // least ~30 m) comfortably covers the road-adjacent queries a site layout / scatter makes.
  const double R0 = std::max(cell * 3.0, 30.0);
  std::vector<int> buf;
  g.gather(x, y, R0, buf);
  for (int i : buf)
    consider(i);
  if (bi < 0 || best > R0) {
    // Slow path (rare: the query is far from every road) — an exact brute scan. Still O(segments),
    // not O(segments^2), and the ascending order matches the fast path's lowest-index tie-break.
    best = 1e30;
    bi = -1;
    for (std::size_t i = 0; i < segments.size(); ++i)
      consider(int(i));
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

road_network generate_roads(const road_params &p, const road_height_fn &height,
                            const road_mask_fn &blocked, const road_density_fn &density) {
  road_network net;
  net.half = p.half;
  if (p.half <= 0 || p.segment_len <= 0)
    return net;
  const bool graded = p.max_grade > 0.0 && height;
  auto clip = [&](V2 q) {
    return V2{std::max(-p.half, std::min(p.half, q.x)), std::max(-p.half, std::min(p.half, q.y))};
  };
  auto grade = [&](V2 a, V2 b) {
    const double run = len(b - a);
    return graded && run > 1e-9 ? std::fabs(height(b.x, b.y) - height(a.x, a.y)) / run : 0.0;
  };
  // Does the straight piece a-b cross the no-go mask? Sampled about every 3 m.
  auto crosses_mask = [&](V2 a, V2 b) {
    if (!blocked)
      return false;
    const int n = std::max(2, int(std::ceil(len(b - a) / 3.0)));
    for (int i = 1; i <= n; ++i) {
      const V2 q = a + (b - a) * (double(i) / n);
      if (blocked(q.x, q.y))
        return true;
    }
    return false;
  };
  // Within the grade limit (with a little slack for the snapped end) and off the mask?
  auto passable = [&](V2 a, V2 b, double slack) {
    return (!graded || grade(a, b) <= p.max_grade * slack) && !crosses_mask(a, b);
  };
  const auto width_of = [&](road_class k) {
    return k == road_class::arterial    ? p.arterial_width
           : k == road_class::collector ? p.collector_width
                                        : p.local_width;
  };
  auto urbanity = [&](V2 q) {
    return density ? std::max(0.0, std::min(1.0, density(q.x, q.y))) : 1.0;
  };
  // The spacing of local streets at q: local_block in a core, wider on sparse ground.
  auto local_spacing = [&](V2 q) {
    return p.local_block / (p.sparse_spacing + (1.0 - p.sparse_spacing) * urbanity(q));
  };
  std::vector<road_centre> centres = p.centres;
  if (centres.empty())
    centres.push_back(road_centre{});

  std::uint32_t draw = 0; // deterministic draw counter (seed+counter => reproducible network)
  auto uni = [&](double lo, double hi) {
    return cvc::lsys::uni(p.seed, cvc::lsys::stream::road, 0, draw++, lo, hi);
  };

  // Incremental spatial indices over nodes and segments, so the per-candidate proximity tests
  // (crossing / node snap / segment snap / branch probe / duplicate check) query a handful of
  // nearby cells instead of scanning the whole network — O(sum of footprints) rather than
  // O(segments^2). The indices are candidate FILTERS: each query returns an ascending,
  // de-duplicated superset and the exact test is computed per candidate, so the network produced is
  // byte-identical to the brute version. A split shortens a segment in place; its (now
  // over-inclusive) index cell stays — safe, since the exact endpoints are re-read on every test.
  const double gcell = std::max(6.0, std::max(p.snap_dist, p.segment_len));
  Grid sg, ng;
  sg.init(p.half, gcell);
  ng.init(p.half, gcell);
  std::vector<int> buf; // reused query scratch

  auto add_node = [&](V2 pt) {
    const int id = int(net.nodes.size());
    net.nodes.push_back(road_node{pt.x, pt.y, 0});
    ng.add_point(id, pt.x, pt.y);
    return id;
  };
  auto node_pos = [&](int i) { return V2{net.nodes[i].x, net.nodes[i].y}; };
  auto push_seg = [&](int a, int b, double w, road_class k) {
    const int id = int(net.segments.size());
    net.segments.push_back(road_segment{a, b, w, k == road_class::arterial, k});
    sg.add_segment(id, net.nodes[a].x, net.nodes[a].y, net.nodes[b].x, net.nodes[b].y);
    return id;
  };
  auto connected = [&](int a, int b) {
    const V2 pa = node_pos(a), pb = node_pos(b);
    sg.gather(0.5 * (pa.x + pb.x), 0.5 * (pa.y + pb.y), 0.5 * len(pb - pa) + gcell, buf);
    for (int i : buf) {
      const road_segment &s = net.segments[i];
      if ((s.a == a && s.b == b) || (s.a == b && s.b == a))
        return true;
    }
    return false;
  };

  // Try to accept a candidate: apply the local constraints (crossing cut, node/segment snap,
  // near-miss extend) and, if valid, append the segment. Returns the end node index, or -1.
  auto accept = [&](Cand &c) -> int {
    const V2 a = node_pos(c.from);
    V2 end = clip(a + c.dir * c.length);
    if (len(end - a) < p.min_segment)
      return -1;
    // Too steep, or across the mask: turn toward the contour in growing steps, either way, taking
    // the gentler side when both fit; none fits -> drop the candidate.
    if ((graded || blocked) && !passable(a, end, 1.0)) {
      const double step = std::max(1.0, p.steer_step_deg) * kPi / 180.0;
      const int turns = int(std::floor(p.steer_max_deg / std::max(1.0, p.steer_step_deg)));
      const V2 dir0 = c.dir;
      bool found = false;
      for (int k = 1; k <= turns && !found; ++k) {
        double best_g = 1e30;
        for (double sgn : {1.0, -1.0}) {
          const V2 d = rotate(dir0, sgn * k * step);
          const V2 e = clip(a + d * c.length);
          if (len(e - a) < p.min_segment || !passable(a, e, 1.0))
            continue;
          const double g = grade(a, e);
          if (g < best_g) {
            best_g = g;
            c.dir = d;
            end = e;
            found = true;
          }
        }
      }
      if (!found)
        return -1;
    }
    // Running alongside another road (not this one's own last piece) too closely: stop here.
    if (p.parallel_gap > 0.0 && c.kind != road_class::arterial) {
      const V2 mid = (a + end) * 0.5, dir = (end - a) * (1.0 / std::max(1e-9, len(end - a)));
      const double gap = p.parallel_gap * local_spacing(mid);
      sg.gather(mid.x, mid.y, gap, buf);
      for (int i : buf) {
        const road_segment &s = net.segments[i];
        if (s.a == c.from || s.b == c.from)
          continue;
        const V2 sa = node_pos(s.a), sb = node_pos(s.b);
        const double sl = len(sb - sa);
        double tt;
        if (sl > 1e-9 && std::fabs(dot(sb - sa, dir)) / sl > 0.906 && // within 25 degrees
            point_seg(mid, sa, sb, tt) < gap && tt > 0.0 && tt < 1.0)
          return -1;
      }
    }

    // 1) Crossing: if a-end strictly crosses an existing segment, cut at the nearest crossing and
    //    split that segment there (forms a 4-way / T).
    int cut_seg = -1;
    double cut_ta = 2.0;
    V2 cut_pt{};
    sg.gather(0.5 * (a.x + end.x), 0.5 * (a.y + end.y), 0.5 * len(end - a) + gcell, buf);
    for (int i : buf) {
      V2 ip;
      double ta, tb;
      if (seg_seg(a, end, node_pos(net.segments[i].a), node_pos(net.segments[i].b), ip, ta, tb)) {
        if (ta < cut_ta) {
          cut_ta = ta;
          cut_seg = i;
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
      const int ob = net.segments[cut_seg].b;
      const double ow = net.segments[cut_seg].width;
      const road_class ok = net.segments[cut_seg].kind;
      b = add_node(end);
      net.segments[cut_seg].b = b; // reuse the slot as oa->b (its index cell is now over-inclusive)
      push_seg(b, ob, ow, ok);     // b->ob
      net.nodes[b].degree += 2;
    } else {
      // 2) snap the free end to the nearest existing node within snap_dist
      int best_node = -1;
      double best_nd = p.snap_dist;
      ng.gather(end.x, end.y, p.snap_dist, buf);
      for (int i : buf) {
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
        sg.gather(end.x, end.y, p.snap_dist, buf);
        for (int i : buf) {
          double tt;
          const double d =
              point_seg(end, node_pos(net.segments[i].a), node_pos(net.segments[i].b), tt);
          if (d < best_sd && tt > 0.05 && tt < 0.95) {
            best_sd = d;
            best_seg = i;
            best_t = tt;
          }
        }
        if (best_seg >= 0) {
          const int oa = net.segments[best_seg].a, ob = net.segments[best_seg].b;
          const double ow = net.segments[best_seg].width;
          const road_class ok = net.segments[best_seg].kind;
          const V2 sp = node_pos(oa) + (node_pos(ob) - node_pos(oa)) * best_t;
          b = add_node(sp);
          net.segments[best_seg].b = b;
          push_seg(b, ob, ow, ok);
          net.nodes[b].degree += 2;
        } else {
          b = add_node(end); // 4) a fresh endpoint
        }
      }
    }
    if (b == c.from || len(node_pos(b) - a) < p.min_segment || connected(c.from, b))
      return -1;
    // The snap moved the end: hold the piece actually built to the limit too (some slack, since a
    // snap is at most snap_dist). A node the split above made is left in place, unused.
    if ((graded || blocked) && !passable(a, node_pos(b), 1.5))
      return -1;

    push_seg(c.from, b, width_of(c.kind), c.kind);
    net.nodes[c.from].degree += 1;
    net.nodes[b].degree += 1;
    return b;
  };

  std::priority_queue<Cand, std::vector<Cand>, CandCmp> Q;
  // Seed the two principal arterials through each centre (major + minor), each way. A centre on
  // the mask seeds nothing.
  for (const road_centre &rc : centres) {
    if (blocked && blocked(rc.x, rc.y))
      continue;
    const int center = add_node(clip(V2{rc.x, rc.y}));
    const V2 mj = field_major(p, height, rc.x, rc.y);
    const V2 mn = perp(mj);
    for (V2 d : {mj, mj * -1.0, mn, mn * -1.0})
      Q.push(Cand{0.0, center, d, p.segment_len, road_class::arterial});
  }
  const road_class branch_kind =
      p.collector_width > 0.0 ? road_class::collector : road_class::local;

  while (!Q.empty() && int(net.segments.size()) < p.max_segments) {
    Cand c = Q.top();
    Q.pop();
    const int b = accept(c);
    if (b < 0)
      continue;
    const V2 bp = node_pos(b);
    // priority grows with distance from the nearest core (so the dense centres are laid first)
    double core_d = 1e30;
    for (const road_centre &rc : centres)
      core_d = std::min(core_d, len(bp - V2{rc.x, rc.y}));
    const double core_t = core_d / std::max(1.0, p.core_radius);
    const double u = urbanity(bp);

    // The field at b, along whichever of its two directions (major or minor) the road is already
    // on: a minor-direction road would otherwise turn onto the major direction after its first
    // step.
    const V2 mj = field_major(p, height, bp.x, bp.y);
    V2 along = std::fabs(dot(mj, c.dir)) >= std::fabs(dot(perp(mj), c.dir)) ? mj : perp(mj);
    if (dot(along, c.dir) < 0)
      along = along * -1.0; // keep heading the same way
    const double run = c.run + len(bp - node_pos(c.from));

    // Branch perpendicular, once the road has run a block since its last branch: block_size off
    // an arterial (a collector, or a local street without a hierarchy), the local spacing
    // otherwise (a local street) — and skip a side if a road is already within ~half a block of
    // where the branch would land. Away from a core the local streets thin out (sparse_spacing,
    // sparse_branch) and stop (local_min_urbanity).
    const bool off_arterial = c.kind == road_class::arterial;
    const double spacing = off_arterial ? p.block_size : local_spacing(bp);
    const road_class child = off_arterial ? branch_kind : road_class::local;
    const double chance = p.branch_chance * (p.sparse_branch + (1.0 - p.sparse_branch) * u);
    const double draw = uni(0.0, 1.0);
    bool branched = false;
    if (run >= spacing && draw < chance &&
        (child != road_class::local || u >= p.local_min_urbanity)) {
      const V2 mn = perp(along);
      for (V2 d : {mn, mn * -1.0}) {
        const V2 probe = bp + d * (spacing * 0.9);
        if (std::fabs(probe.x) > p.half || std::fabs(probe.y) > p.half)
          continue;
        // Is a road already within ~half a block of the branch target? (same test as
        // distance_to_road(probe) < spacing*0.55, via the incremental index.)
        bool near = false;
        sg.gather(probe.x, probe.y, spacing * 0.55, buf);
        for (int i : buf) {
          double tt;
          if (point_seg(probe, node_pos(net.segments[i].a), node_pos(net.segments[i].b), tt) <
              spacing * 0.55) {
            near = true;
            break;
          }
        }
        if (near)
          continue; // keep blocks from collapsing together
        Q.push(Cand{c.t + 2.0 + core_t, b, d, p.segment_len, child});
        branched = true;
      }
    }
    // Continue straight, the field re-sampled so the road follows the terrain. A local street
    // stops where the district fades out (local_min_urbanity).
    if (c.kind != road_class::local || u >= p.local_min_urbanity)
      Q.push(Cand{c.t + 1.0 + core_t, b, along, p.segment_len, c.kind, branched ? 0.0 : run});
  }
  return net;
}

std::size_t prune_roads(road_network &net, double min_stub) {
  const int N = int(net.nodes.size());
  std::vector<char> gone(net.segments.size(), 0);
  std::vector<std::vector<int>> adj(N); // node -> live segment ids
  auto rebuild = [&]() {
    for (auto &v : adj)
      v.clear();
    for (std::size_t i = 0; i < net.segments.size(); ++i) {
      const road_segment &s = net.segments[i];
      if (gone[i] || s.a < 0 || s.b < 0 || s.a >= N || s.b >= N) {
        gone[i] = 1;
        continue;
      }
      adj[s.a].push_back(int(i));
      adj[s.b].push_back(int(i));
    }
  };
  auto seg_len = [&](int i) {
    const road_segment &s = net.segments[i];
    return std::hypot(net.nodes[s.b].x - net.nodes[s.a].x, net.nodes[s.b].y - net.nodes[s.a].y);
  };
  std::size_t removed = 0;
  bool changed = min_stub > 0.0;
  rebuild();
  while (changed) {
    changed = false;
    for (int n0 = 0; n0 < N; ++n0) {
      if (adj[n0].size() != 1)
        continue;
      // Walk from the dead end through degree-2 nodes to the first junction (or the other end).
      std::vector<int> chain;
      double length = 0.0;
      int cur = n0, seg = adj[n0][0];
      while (true) {
        chain.push_back(seg);
        length += seg_len(seg);
        const road_segment &s = net.segments[seg];
        cur = s.a == cur ? s.b : s.a;
        if (adj[cur].size() != 2 || length >= min_stub)
          break;
        seg = adj[cur][0] == seg ? adj[cur][1] : adj[cur][0];
      }
      if (length >= min_stub)
        continue;
      for (int i : chain)
        gone[i] = 1;
      removed += chain.size();
      changed = true;
      rebuild();
    }
  }
  std::vector<road_segment> keep;
  keep.reserve(net.segments.size() - removed);
  for (std::size_t i = 0; i < net.segments.size(); ++i)
    if (!gone[i])
      keep.push_back(net.segments[i]);
  net.segments.swap(keep);
  for (road_node &n : net.nodes)
    n.degree = 0;
  for (const road_segment &s : net.segments) {
    ++net.nodes[s.a].degree;
    ++net.nodes[s.b].degree;
  }
  return removed;
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
