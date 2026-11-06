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
#include <cvc/world/road_grade.h>
#include <functional>
#include <utility>

namespace cvc {
namespace world {

namespace {

double segLength(const road_network &net, const road_segment &s) {
  return std::hypot(net.nodes[s.b].x - net.nodes[s.a].x, net.nodes[s.b].y - net.nodes[s.a].y);
}

// Distance from (x, y) to segment s's centreline, and the parameter t of the closest point.
double toSegment(const road_network &net, const road_segment &s, double x, double y, double &t) {
  const double ax = net.nodes[s.a].x, ay = net.nodes[s.a].y;
  const double vx = net.nodes[s.b].x - ax, vy = net.nodes[s.b].y - ay;
  const double l2 = vx * vx + vy * vy;
  t = l2 > 1e-12 ? std::clamp(((x - ax) * vx + (y - ay) * vy) / l2, 0.0, 1.0) : 0.0;
  return std::hypot(ax + vx * t - x, ay + vy * t - y);
}

double smoothstep(double u) { return u * u * (3.0 - 2.0 * u); }

// The most two roads' heights may differ where their level corridors are `gap` metres apart:
// the side slope over the gap, and none where the corridors touch.
double allowedStep(const road_grade_params &p, double gap) {
  return std::max(0.0, gap) * p.side_slope;
}

// A node held to a road it does not lie on: the point at t along segment (a, b), and the most
// their heights may differ.
struct Link {
  int i, a, b;
  double t, lim;
};

// Hold every segment to its climb (`rise`, metres end to end) and every link to its step: a pair
// too far apart in height is drawn together, never below a floor. Sweeps until nothing moves (or
// a sweep budget runs out).
void limitGrade(const road_network &net, const std::vector<double> &rise,
                const std::vector<Link> &links, const std::vector<double> &floor,
                std::vector<double> &z) {
  for (int sweep = 0; sweep < 400; ++sweep) {
    double moved = 0.0;
    for (std::size_t si = 0; si < net.segments.size(); ++si) {
      const road_segment &s = net.segments[si];
      const double lim = rise[si];
      if (lim < 0.0)
        continue;
      const double dz = z[s.b] - z[s.a];
      const double excess = std::abs(dz) - lim;
      if (excess <= 1e-6)
        continue;
      const int hi = dz > 0 ? s.b : s.a, lo = dz > 0 ? s.a : s.b;
      // Split the excess: the high end down (no lower than its floor), the low end up.
      const double down = std::min(0.5 * excess, z[hi] - floor[hi]);
      z[hi] -= down;
      z[lo] += excess - down;
      moved = std::max(moved, excess);
    }
    for (const Link &k : links) {
      const double zp = z[k.a] + (z[k.b] - z[k.a]) * k.t;
      const double dz = z[k.i] - zp;
      const double excess = std::abs(dz) - k.lim;
      if (excess <= 1e-6)
        continue;
      // The node moves half the way and the point on the road the other half, its two ends sharing
      // that by where the point lies (the least change that moves it as far); a move down stops
      // at the floors.
      const double s = dz > 0 ? -1.0 : 1.0; // the node's direction
      const double wa = 1.0 - k.t, wb = k.t, norm = wa * wa + wb * wb;
      double mi = 0.5 * excess, mr = excess - mi;
      if (s < 0)
        mi = std::min(mi, z[k.i] - floor[k.i]);
      else if (norm > 0.0) // the road's ends go down
        mr = std::min({mr, wa > 0.0 ? (z[k.a] - floor[k.a]) * norm / wa : mr,
                       wb > 0.0 ? (z[k.b] - floor[k.b]) * norm / wb : mr});
      z[k.i] += s * mi;
      if (norm > 0.0) {
        z[k.a] -= s * mr * wa / norm;
        z[k.b] -= s * mr * wb / norm;
      }
      moved = std::max(moved, excess);
    }
    if (moved < 1e-3)
      return;
  }
}

} // namespace

// A uniform grid of the network's segments, for the roads near a point.
struct road_segment_grid {
  double x0 = 0, y0 = 0, cell = 1;
  int nx = 0, ny = 0;
  std::vector<std::vector<int>> cells;

  road_segment_grid(const road_network &net, double c) : cell(std::max(1.0, c)) {
    if (net.segments.empty())
      return;
    double lo[2] = {1e300, 1e300}, hi[2] = {-1e300, -1e300};
    for (const road_node &n : net.nodes) {
      lo[0] = std::min(lo[0], n.x);
      lo[1] = std::min(lo[1], n.y);
      hi[0] = std::max(hi[0], n.x);
      hi[1] = std::max(hi[1], n.y);
    }
    x0 = lo[0] - cell;
    y0 = lo[1] - cell;
    nx = int((hi[0] - x0) / cell) + 2;
    ny = int((hi[1] - y0) / cell) + 2;
    cells.assign(std::size_t(nx) * ny, {});
    for (std::size_t i = 0; i < net.segments.size(); ++i) {
      const road_node &a = net.nodes[net.segments[i].a], &b = net.nodes[net.segments[i].b];
      const int c0 = col(std::min(a.x, b.x)), c1 = col(std::max(a.x, b.x));
      const int r0 = row(std::min(a.y, b.y)), r1 = row(std::max(a.y, b.y));
      for (int r = r0; r <= r1; ++r)
        for (int c = c0; c <= c1; ++c)
          cells[std::size_t(r) * nx + c].push_back(int(i));
    }
  }
  int col(double x) const { return std::clamp(int((x - x0) / cell), 0, nx - 1); }
  int row(double y) const { return std::clamp(int((y - y0) / cell), 0, ny - 1); }
  // The segments in the cells within r of (x, y), each once, into `out`.
  void near(double x, double y, double r, std::vector<int> &out) const {
    out.clear();
    if (cells.empty())
      return;
    for (int rr = row(y - r); rr <= row(y + r); ++rr)
      for (int cc = col(x - r); cc <= col(x + r); ++cc) {
        const std::vector<int> &v = cells[std::size_t(rr) * nx + cc];
        out.insert(out.end(), v.begin(), v.end());
      }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
  }
};

std::vector<double> grade_road_profile(const road_network &net, const terrain_source &ground,
                                       const road_grade_params &p) {
  const std::size_t n = net.nodes.size();
  std::vector<double> z0(n), floor(n), z(n);
  const double lowest = std::isfinite(p.water_level) ? p.water_level + p.water_clearance
                                                     : -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    z0[i] = ground.sample(net.nodes[i].x, net.nodes[i].y);
    floor[i] = lowest;
    z[i] = std::max(z0[i], lowest);
  }
  // Each node's neighbours along the network, weighted by closeness (1 / length), so a node
  // between a short and a long segment follows the near one: the profile's second difference.
  struct Arc {
    int to;
    double w;
  };
  std::vector<std::vector<Arc>> adj(n);
  std::vector<double> nodeHalf(n, 0.0); // the widest road at each node, halved
  double widest = 0.0;
  for (const road_segment &s : net.segments) {
    const double w = 1.0 / std::max(0.5, segLength(net, s));
    adj[std::size_t(s.a)].push_back({s.b, w});
    adj[std::size_t(s.b)].push_back({s.a, w});
    nodeHalf[std::size_t(s.a)] = std::max(nodeHalf[std::size_t(s.a)], 0.5 * s.width);
    nodeHalf[std::size_t(s.b)] = std::max(nodeHalf[std::size_t(s.b)], 0.5 * s.width);
    widest = std::max(widest, s.width);
  }
  // Each segment's climb: max_grade over its length, junction_grade where it meets a junction
  // (three or more roads), so a crossing is near level and the roads leaving it agree there.
  std::vector<double> rise(net.segments.size(), -1.0);
  for (std::size_t si = 0; si < net.segments.size(); ++si) {
    const road_segment &s = net.segments[si];
    const bool junction = adj[std::size_t(s.a)].size() >= 3 || adj[std::size_t(s.b)].size() >= 3;
    const double g =
        junction && p.junction_grade > 0.0 ? std::min(p.max_grade, p.junction_grade) : p.max_grade;
    if (g > 0.0)
      rise[si] = g * segLength(net, s);
  }
  // The roads near each node that it is not on: where their level corridors (nearly) touch, their
  // heights must agree to within the side slope over the gap, or one road's surface would tilt
  // toward the other's. Further apart, the ground between them blends (graded_terrain::sample).
  // A road the node reaches along the network within a few times the distance is its own road (or
  // one met at a junction just by), which the grade limit already ties to it.
  std::vector<Link> links;
  const double close = 2.0 * p.min_shoulder;
  const double reach = widest + 2.0 * p.flat_margin + close;
  const double cutoff = 4.0 * reach;
  const road_segment_grid grid(net, reach);
  std::vector<int> cand;
  std::vector<double> path(n, std::numeric_limits<double>::infinity());
  std::vector<int> touched;
  for (std::size_t i = 0; i < n; ++i) {
    if (adj[i].empty())
      continue;
    grid.near(net.nodes[i].x, net.nodes[i].y, reach, cand);
    if (cand.empty())
      continue;
    // Network distances from i, out to the cutoff.
    for (int v : touched)
      path[std::size_t(v)] = std::numeric_limits<double>::infinity();
    touched.clear();
    std::vector<std::pair<double, int>> heap{{0.0, int(i)}};
    path[i] = 0.0;
    touched.push_back(int(i));
    while (!heap.empty()) {
      std::pop_heap(heap.begin(), heap.end(), std::greater<>());
      const auto [du, u] = heap.back();
      heap.pop_back();
      if (du > path[std::size_t(u)])
        continue;
      for (const Arc &a : adj[std::size_t(u)]) {
        const double dv = du + 1.0 / a.w;
        if (dv > cutoff || dv >= path[std::size_t(a.to)])
          continue;
        if (std::isinf(path[std::size_t(a.to)]))
          touched.push_back(a.to);
        path[std::size_t(a.to)] = dv;
        heap.push_back({dv, a.to});
        std::push_heap(heap.begin(), heap.end(), std::greater<>());
      }
    }
    for (int si : cand) {
      const road_segment &s = net.segments[std::size_t(si)];
      if (s.a == int(i) || s.b == int(i))
        continue;
      double t = 0.0;
      const double d = toSegment(net, s, net.nodes[i].x, net.nodes[i].y, t);
      const double gap = d - (nodeHalf[i] + p.flat_margin) - (0.5 * s.width + p.flat_margin);
      const double along = std::min(path[std::size_t(s.a)], path[std::size_t(s.b)]);
      if (gap < close && along > 3.0 * d + reach)
        links.push_back({int(i), s.a, s.b, t, allowedStep(p, gap)});
    }
  }
  std::vector<double> next(n);
  for (int pass = 0; pass < p.smooth_passes; ++pass) {
    for (std::size_t i = 0; i < n; ++i) {
      if (adj[i].empty()) {
        next[i] = z[i];
        continue;
      }
      double sum = 0.0, wsum = 0.0;
      for (const Arc &a : adj[i]) {
        sum += a.w * z[std::size_t(a.to)];
        wsum += a.w;
      }
      const double relaxed = z[i] + p.smooth * (sum / wsum - z[i]);
      const double held = relaxed + p.hold * (std::max(z0[i], floor[i]) - relaxed);
      next[i] = std::max(floor[i], held);
    }
    z.swap(next);
    limitGrade(net, rise, links, floor, z);
  }
  limitGrade(net, rise, links, floor, z);
  return z;
}

graded_terrain::graded_terrain(std::shared_ptr<const terrain_source> ground, road_network net,
                               std::vector<double> node_z, const road_grade_params &p)
    : ground_(std::move(ground)), net_(std::move(net)), z_(std::move(node_z)), p_(p) {
  double widest = 0.0;
  for (const road_segment &s : net_.segments)
    widest = std::max(widest, s.width);
  reach_ = 0.5 * widest + p_.flat_margin + p_.max_shoulder;
  grid_ = std::make_shared<const road_segment_grid>(net_, reach_);
  // The runs: segments joined through a node only two roads use are one road.
  std::vector<int> deg(net_.nodes.size(), 0), first(net_.nodes.size(), -1);
  for (const road_segment &s : net_.segments) {
    ++deg[std::size_t(s.a)];
    ++deg[std::size_t(s.b)];
  }
  run_.resize(net_.segments.size());
  for (std::size_t i = 0; i < run_.size(); ++i)
    run_[i] = int(i);
  auto find = [&](int i) {
    while (run_[std::size_t(i)] != i)
      i = run_[std::size_t(i)] = run_[std::size_t(run_[std::size_t(i)])];
    return i;
  };
  for (std::size_t i = 0; i < net_.segments.size(); ++i)
    for (int v : {net_.segments[i].a, net_.segments[i].b}) {
      if (deg[std::size_t(v)] != 2)
        continue;
      if (first[std::size_t(v)] < 0)
        first[std::size_t(v)] = int(i);
      else
        run_[std::size_t(find(int(i)))] = find(first[std::size_t(v)]);
    }
  for (std::size_t i = 0; i < run_.size(); ++i)
    run_[i] = find(int(i));
}

double graded_terrain::road_height(int seg, double t) const {
  const road_segment &s = net_.segments[std::size_t(seg)];
  return z_[std::size_t(s.a)] + (z_[std::size_t(s.b)] - z_[std::size_t(s.a)]) * t;
}

double graded_terrain::sample(double x, double y) const {
  const double h0 = ground_->sample(x, y);
  if (net_.empty())
    return h0;
  // Each road (run of segments) within reach pulls the ground toward its surface by its influence:
  // 1 on the road and its level margin, easing to 0 across its cut or bank. A road's surface here
  // is its nearest segment's, blended with any other of its segments within kBend of as near (the
  // inside of a bend), so it is continuous round the bend and exact along a straight. The ground
  // moves toward the roads' surfaces (weighted by influence, the strongest dominating) by the
  // strongest influence, so it is continuous where two roads' reaches meet, and a road's own
  // surface on the road.
  constexpr double kBend = 0.25;
  struct Near {
    int run;
    double d, zr, flat;
  };
  thread_local std::vector<int> cand;
  thread_local std::vector<Near> near;
  grid_->near(x, y, reach_, cand);
  near.clear();
  for (int si : cand) {
    const road_segment &s = net_.segments[std::size_t(si)];
    double t = 0.0;
    const double d = toSegment(net_, s, x, y, t);
    const double flat = 0.5 * s.width + p_.flat_margin;
    if (d < flat + p_.max_shoulder)
      near.push_back({run_[std::size_t(si)], d, road_height(si, t), flat});
  }
  double strongest = 0.0, sum = 0.0, wsum = 0.0;
  for (std::size_t i = 0; i < near.size(); ++i) {
    // Each run once, at its first entry: its nearest distance, then its surface.
    bool seen = false;
    for (std::size_t j = 0; j < i && !seen; ++j)
      seen = near[j].run == near[i].run;
    if (seen)
      continue;
    double dmin = near[i].d, flat = near[i].flat;
    for (std::size_t j = i + 1; j < near.size(); ++j)
      if (near[j].run == near[i].run && near[j].d < dmin) {
        dmin = near[j].d;
        flat = near[j].flat;
      }
    double zs = 0.0, ws = 0.0;
    for (std::size_t j = i; j < near.size(); ++j)
      if (near[j].run == near[i].run) {
        const double f = 1.0 - (near[j].d - dmin) / kBend;
        if (f > 0.0) {
          zs += f * f * near[j].zr;
          ws += f * f;
        }
      }
    const double zr = zs / ws;
    double a = 1.0;
    if (dmin > flat) {
      const double shoulder =
          std::min(p_.max_shoulder,
                   std::max(p_.min_shoulder, std::abs(h0 - zr) / std::max(1e-3, p_.side_slope)));
      const double u = (dmin - flat) / shoulder;
      if (u >= 1.0)
        continue;
      a = 1.0 - smoothstep(u);
    }
    const double w = std::pow(a, 8.0); // the strongest road dominates the blend of surfaces
    strongest = std::max(strongest, a);
    sum += w * zr;
    wsum += w;
  }
  if (strongest <= 0.0 || wsum <= 0.0)
    return h0;
  return h0 + strongest * (sum / wsum - h0);
}

bool graded_terrain::graded_at(double x, double y) const {
  if (net_.empty())
    return false;
  thread_local std::vector<int> cand;
  grid_->near(x, y, reach_, cand);
  for (int si : cand) {
    double t = 0.0;
    const road_segment &s = net_.segments[std::size_t(si)];
    if (toSegment(net_, s, x, y, t) < 0.5 * s.width + p_.flat_margin + p_.max_shoulder)
      return true;
  }
  return false;
}

} // namespace world
} // namespace cvc
