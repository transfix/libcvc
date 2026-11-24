/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// CoPH-Stagewise's learner (C3 of docs/roadmap/NAV-COPH-ROADMAP.md, §4.3); see coph_learner.h.
// Each function names the GRL-SNAM (grl_snam.coph) function it follows. Built with
// -ffp-contract=off, so its arithmetic is the Python's, operation for operation.

#include <algorithm>
#include <cmath>
#include <cvc/algo/det_math.h>
#include <cvc/nav/coph_exchange.h>
#include <cvc/nav/coph_learner.h>
#include <cvc/nav/detail/coph_math.h>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <utility>

namespace cvc {
namespace nav {

namespace {

using detail::coph_cholesky;
using detail::coph_cholesky_solve;
using detail::coph_features;
using detail::coph_slab_distance;

constexpr double kAgentRadius = 0.12;
constexpr double kSupportRadius = 0.65;
constexpr double kGoalRadius = 0.25;
constexpr double kCollisionCost = 40.0;
constexpr int kRoutedMultiplier = 7; // replay_budget.multiplier for routed control
constexpr long long kExternalReserve = 12;
constexpr double kInf = std::numeric_limits<double>::infinity();

double sq(double x) { return x * x; }
double norm2(double x, double y) { return std::sqrt(x * x + y * y); }
double clip(double x, double lo, double hi) { return std::min(std::max(x, lo), hi); }

template <class T> T tsq(const T &x) { return x * x; }

template <class T> T tsqrt(const T &x);
template <> double tsqrt<double>(const double &x) { return std::sqrt(x); }
template <> coph_dual2 tsqrt<coph_dual2>(const coph_dual2 &x) {
  coph_dual2 r;
  r.v = std::sqrt(x.v);
  for (int i = 0; i < 2; ++i)
    r.d[i] = x.d[i] / (2.0 * r.v);
  return r;
}

template <class T> T tnorm(const T &x, const T &y) { return tsqrt<T>(x * x + y * y); }

template <class T> coph_state_of<double> values(const coph_state_of<T> &x) {
  return {coph_value(x[0]), coph_value(x[1]), coph_value(x[2]), coph_value(x[3])};
}
template <class T> coph_vec2 values(const coph_vec2_of<T> &x) {
  return {coph_value(x[0]), coph_value(x[1])};
}

// ---------------------------------------------------------------------------
// Shapes (shapes.py, maze_geometry.py, terrain_warning.py)
// ---------------------------------------------------------------------------

struct segment_ends {
  coph_vec2 p, s; // the slab's core segment, (x, y) -/+ (hx, hy)
};
segment_ends ends(const coph_shape &o) {
  return {{o.x - o.hx, o.y - o.hy}, {o.x + o.hx, o.y + o.hy}};
}

// shapes._point_segment: the distance from p to the segment a-b.
double point_segment(const coph_vec2 &p, const coph_vec2 &a, const coph_vec2 &b) {
  const double d0 = b[0] - a[0], d1 = b[1] - a[1];
  const double t =
      clip(((p[0] - a[0]) * d0 + (p[1] - a[1]) * d1) / ((d0 * d0 + d1 * d1) + 1e-20), 0.0, 1.0);
  return norm2((a[0] + t * d0) - p[0], (a[1] + t * d1) - p[1]);
}

// shapes._crosses: whether the segments a-b and p-s cross properly.
bool crosses(const coph_vec2 &a, const coph_vec2 &b, const coph_vec2 &p, const coph_vec2 &s) {
  auto side = [](const coph_vec2 &o, const coph_vec2 &x, const coph_vec2 &y) {
    return (x[0] - o[0]) * (y[1] - o[1]) - (x[1] - o[1]) * (y[0] - o[0]);
  };
  return side(p, s, a) * side(p, s, b) < 0 && side(a, b, p) * side(a, b, s) < 0;
}

// shapes.slab_sweep: the distance from the motion a-b to the slab's surface.
double slab_sweep(const coph_vec2 &a, const coph_vec2 &b, const coph_shape &o) {
  const segment_ends e = ends(o);
  if (crosses(a, b, e.p, e.s))
    return -o.r;
  return std::min({point_segment(a, e.p, e.s), point_segment(b, e.p, e.s), point_segment(e.p, a, b),
                   point_segment(e.s, a, b)}) -
         o.r;
}

// A disk's distance from the motion a-b to its centre (the closest point of the segment).
double disk_sweep(const coph_vec2 &a, const coph_vec2 &b, const coph_shape &o) {
  const double d0 = b[0] - a[0], d1 = b[1] - a[1];
  const double t =
      clip(((o.x - a[0]) * d0 + (o.y - a[1]) * d1) / ((d0 * d0 + d1 * d1) + 1e-20), 0.0, 1.0);
  return norm2((a[0] + t * d0) - o.x, (a[1] + t * d1) - o.y);
}

// maze_geometry.obstacle_clear: the motion a-b stays more than margin from every shape.
bool obstacle_clear(const coph_vec2 &a, const coph_vec2 &b, const std::vector<coph_shape> &shapes,
                    double margin = 0.17) {
  for (const coph_shape &o : shapes) {
    if (o.slab) {
      if (slab_sweep(a, b, o) <= margin)
        return false;
      continue;
    }
    if (disk_sweep(a, b, o) <= o.r + margin)
      return false;
  }
  return true;
}

// maze_geometry.distance_to_network.
double distance_to_network(const coph_vec2 &p, const coph_network &net) {
  double best = kInf;
  for (const auto &e : net.edges) {
    const coph_vec2 &a = net.nodes[e[0]];
    const double d0 = net.nodes[e[1]][0] - a[0], d1 = net.nodes[e[1]][1] - a[1];
    const double x0 = p[0] - a[0], x1 = p[1] - a[1];
    const double t = clip((x0 * d0 + x1 * d1) / (d0 * d0 + d1 * d1), 0.0, 1.0);
    best = std::min(best, norm2(x0 - t * d0, x1 - t * d1));
  }
  return best;
}

// maze_geometry.corridor_clear: points every 0.04 m along a-b, all within the corridor less the
// margin (the distance is 1-Lipschitz; half the spacing pads it).
bool corridor_clear(const coph_vec2 &a, const coph_vec2 &b, const coph_network &net,
                    double margin = 0.17) {
  const double d0 = b[0] - a[0], d1 = b[1] - a[1];
  const double len = norm2(d0, d1);
  const int n = std::max(2, static_cast<int>(std::ceil(len / 0.04)) + 1);
  const double step = 1.0 / (n - 1); // numpy.linspace(0, 1, n)
  double worst = 0.0;
  for (int i = 0; i < n; ++i) {
    const double t = i == n - 1 ? 1.0 : i * step;
    worst = std::max(worst, distance_to_network({a[0] + d0 * t, a[1] + d1 * t}, net));
  }
  const double padding = len / (2 * (n - 1));
  return worst + padding <= (net.corridor_radius - margin) + 1e-10;
}

// terrain_warning.dense: points every `spacing` along the polyline, ending at its last point.
std::vector<coph_vec2> dense(const std::vector<coph_vec2> &path, double spacing) {
  std::vector<coph_vec2> out;
  for (std::size_t k = 0; k + 1 < path.size(); ++k) {
    const coph_vec2 &a = path[k], &b = path[k + 1];
    const double d0 = b[0] - a[0], d1 = b[1] - a[1];
    const int n = std::max(2, static_cast<int>(norm2(d0, d1) / spacing) + 1);
    const double step = 1.0 / n; // numpy.linspace(0, 1, n, endpoint=False)
    for (int i = 0; i < n; ++i) {
      const double t = i * step;
      out.push_back({a[0] + d0 * t, a[1] + d1 * t});
    }
  }
  out.push_back(path.back());
  return out;
}

// terrain_warning.terrain_values: `unknown` where no anchor supports a point, else the largest
// supported first-order estimate, clipped to [0, 20].
double terrain_value(const coph_vec2 &p, const std::vector<coph_anchor> &anchors, double unknown) {
  double best = -kInf;
  bool supported = false;
  for (const coph_anchor &a : anchors) {
    const double dx = p[0] - a.x, dy = p[1] - a.y;
    if (norm2(dx, dy) > kSupportRadius)
      continue;
    supported = true;
    best = std::max(best, a.value + (a.gx * dx + a.gy * dy));
  }
  return supported ? clip(best, 0.0, 20.0) : unknown;
}

// The priced length of a polyline: sum over its pieces of length (1 + 4 weight risk(midpoint)).
double priced_length(const std::vector<coph_vec2> &points, const coph_route_context &ctx) {
  double total = 0.0;
  for (std::size_t k = 0; k + 1 < points.size(); ++k) {
    const coph_vec2 mid{(points[k][0] + points[k + 1][0]) / 2,
                        (points[k][1] + points[k + 1][1]) / 2};
    const double risk = terrain_value(mid, ctx.terrain_anchors, ctx.unknown_risk);
    total += norm2(points[k + 1][0] - points[k][0], points[k + 1][1] - points[k][1]) *
             (1 + 4 * ctx.risk_weight * risk);
  }
  return total;
}

// Whether the motion a-b collides with a stage's obstacles (the replay's swept test).
bool collides(const coph_vec2 &a, const coph_vec2 &b, const std::vector<coph_shape> &obstacles) {
  bool hit = false;
  for (const coph_shape &o : obstacles)
    hit = hit || (o.slab ? slab_sweep(a, b, o) <= kAgentRadius
                         : disk_sweep(a, b, o) <= o.r + kAgentRadius);
  return hit;
}

std::vector<double> sorted_dampings(const coph_config &cfg, double damping) {
  std::set<double> ds{cfg.damping_bounds.first, cfg.damping_bounds.second, damping};
  return {ds.begin(), ds.end()};
}

// ---------------------------------------------------------------------------
// The CBF projection (navigation.project) and the guard
// ---------------------------------------------------------------------------

template <class T> struct disk_constraint {
  coph_vec2_of<T> center;
  double radius;
};
template <class T> struct line_constraint {
  coph_vec2_of<T> a; // a . u >= b
  T b;
};

template <class T> T dot(const coph_vec2_of<T> &a, const coph_vec2_of<T> &b) {
  return a[0] * b[0] + a[1] * b[1];
}

template <class T> T tmax(const T &a, const T &b) { return coph_value(a) >= coph_value(b) ? a : b; }

template <class T>
bool satisfies(const coph_vec2_of<T> &p, const std::vector<disk_constraint<T>> &disks,
               const std::vector<line_constraint<T>> &lines) {
  const coph_vec2 pv = values<T>(p);
  if (!std::isfinite(pv[0]) || !std::isfinite(pv[1]))
    return false;
  for (const auto &c : disks) {
    const coph_vec2 cv = values<T>(c.center);
    if (norm2(pv[0] - cv[0], pv[1] - cv[1]) - c.radius > 1e-8)
      return false;
  }
  for (const auto &l : lines) {
    const coph_vec2 av = values<T>(l.a);
    if (coph_value(l.b) - (av[0] * pv[0] + av[1] * pv[1]) > 1e-8)
      return false;
  }
  return true;
}

// navigation.project: the closest point to raw in the intersection of the force disk, the
// velocity-reachability disks at each damping, and the obstacles' robust CBF halfplanes. With
// fast, raw itself when it already satisfies them all (maze_graph). nullopt when no candidate is
// feasible; count is the number of candidates tried.
template <class T> struct projection {
  std::optional<coph_vec2_of<T>> point;
  int count = 0;
};

template <class T>
projection<T> project(const coph_vec2_of<T> &raw, const coph_state_of<T> &x,
                      const std::vector<coph_shape> &obstacles, double damping,
                      const coph_config &cfg, bool fast) {
  const coph_vec2_of<T> q{x[0], x[1]}, v{x[2], x[3]};
  std::vector<disk_constraint<T>> disks{{{T(0.0), T(0.0)}, cfg.force_limit}};
  for (double d : sorted_dampings(cfg, damping)) {
    const double a = algo::det::exp(-d * cfg.dt);
    const double b = -algo::det::expm1(-d * cfg.dt) / d;
    disks.push_back({{(-a / b) * v[0], (-a / b) * v[1]}, 1.0 / b});
  }
  std::vector<line_constraint<T>> lines;
  for (const coph_shape &o : obstacles) {
    coph_vec2_of<T> z;
    T curvature;
    if (o.slab) {
      // shapes.slab_offset: q minus its nearest core point, and whether that point is interior.
      const segment_ends e = ends(o);
      const double d0 = e.s[0] - e.p[0], d1 = e.s[1] - e.p[1];
      const T raw_t = ((q[0] - e.p[0]) * d0 + (q[1] - e.p[1]) * d1) / (d0 * d0 + d1 * d1);
      const double tv = coph_value(raw_t);
      const T t = tv < 0.0 ? T(0.0) : (tv > 1.0 ? T(1.0) : raw_t);
      z = {q[0] - (e.p[0] + t * d0), q[1] - (e.p[1] + t * d1)};
      const bool interior = tv > 0.0 && tv < 1.0;
      if (interior) {
        T zz = dot<T>(z, z);
        if (coph_value(zz) < 1e-20)
          zz = T(1e-20);
        curvature = tsq<T>(dot<T>(z, v)) / zz;
      } else {
        curvature = dot<T>(v, v);
      }
    } else {
      z = {q[0] - o.x, q[1] - o.y};
      curvature = dot<T>(v, v);
    }
    const T h = dot<T>(z, z) - sq((o.r + kAgentRadius) + cfg.safety_margin);
    const T rv = dot<T>(z, v);
    const T worst = tmax<T>(2 * cfg.damping_bounds.first * rv, 2 * cfg.damping_bounds.second * rv);
    const T bound = ((worst - 2.0 * curvature) - 8.0 * rv) - 4.0 * h;
    lines.push_back({{2.0 * z[0], 2.0 * z[1]}, bound});
  }
  if (fast && satisfies<T>(raw, disks, lines))
    return {raw, 1};
  std::vector<coph_vec2_of<T>> candidates{raw};
  for (const auto &c : disks) {
    const coph_vec2_of<T> z{raw[0] - c.center[0], raw[1] - c.center[1]};
    T len = tnorm<T>(z[0], z[1]);
    if (coph_value(len) < 1e-12)
      len = T(1e-12);
    candidates.push_back(
        {c.center[0] + c.radius * z[0] / len, c.center[1] + c.radius * z[1] / len});
  }
  for (const auto &l : lines) {
    const T aa = dot<T>(l.a, l.a);
    if (coph_value(aa) < 1e-14)
      continue;
    const T slack = (l.b - dot<T>(l.a, raw)) / aa;
    candidates.push_back({raw[0] + slack * l.a[0], raw[1] + slack * l.a[1]});
    const T root = tsqrt<T>(aa);
    const coph_vec2_of<T> perp{-l.a[1] / root, l.a[0] / root};
    for (const auto &c : disks) {
      const T t = (l.b - dot<T>(l.a, c.center)) / aa;
      const coph_vec2_of<T> base{c.center[0] + t * l.a[0], c.center[1] + t * l.a[1]};
      const T disc = c.radius * c.radius - t * t * aa;
      if (coph_value(disc) >= 0) {
        const T off = tsqrt<T>(coph_value(disc) < 1e-20 ? T(1e-20) : disc);
        candidates.push_back({base[0] + off * perp[0], base[1] + off * perp[1]});
        candidates.push_back({base[0] - off * perp[0], base[1] - off * perp[1]});
      }
    }
  }
  for (std::size_t i = 0; i < lines.size(); ++i)
    for (std::size_t j = i + 1; j < lines.size(); ++j) {
      const auto &l1 = lines[i], &l2 = lines[j];
      const T det = l1.a[0] * l2.a[1] - l1.a[1] * l2.a[0];
      if (std::fabs(coph_value(det)) > 1e-10)
        candidates.push_back(
            {(l1.b * l2.a[1] - l1.a[1] * l2.b) / det, (l1.a[0] * l2.b - l1.b * l2.a[0]) / det});
    }
  for (std::size_t i = 0; i < disks.size(); ++i)
    for (std::size_t j = i + 1; j < disks.size(); ++j) {
      const auto &a = disks[i], &b = disks[j];
      const coph_vec2_of<T> z{b.center[0] - a.center[0], b.center[1] - a.center[1]};
      const T len = tnorm<T>(z[0], z[1]);
      if (coph_value(len) < 1e-12)
        continue;
      const T t = (a.radius * a.radius - b.radius * b.radius + len * len) / (2.0 * len);
      const T h = a.radius * a.radius - t * t;
      if (coph_value(h) >= 0) {
        const coph_vec2_of<T> unit{z[0] / len, z[1] / len};
        const coph_vec2_of<T> perp{-unit[1], unit[0]};
        const coph_vec2_of<T> base{a.center[0] + t * unit[0], a.center[1] + t * unit[1]};
        const T off = tsqrt<T>(coph_value(h) < 1e-20 ? T(1e-20) : h);
        candidates.push_back({base[0] + off * perp[0], base[1] + off * perp[1]});
        candidates.push_back({base[0] - off * perp[0], base[1] - off * perp[1]});
      }
    }
  const coph_vec2 rv = values<T>(raw);
  int best = -1;
  double best_d = kInf;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (!satisfies<T>(candidates[i], disks, lines))
      continue;
    const coph_vec2 cv = values<T>(candidates[i]);
    const double d = sq(cv[0] - rv[0]) + sq(cv[1] - rv[1]);
    if (d < best_d) {
      best_d = d;
      best = static_cast<int>(i);
    }
  }
  const int count = static_cast<int>(candidates.size());
  if (best < 0)
    return {std::nullopt, count};
  return {candidates[best], count};
}

// Whether the atlas has a surveyed network (the maze router, and the corridor in the guard); a
// car's learner has none.
bool maze(const coph_atlas &atlas) { return atlas.network && !atlas.network->nodes.empty(); }

// guarded_control's guard: the step under u, at each damping, stays in the corridor (on a network,
// with room to stop), clear of every shape in the geometry store, and under 1 m/s.
bool guard(const coph_state &x, const coph_vec2 &u, double damping, const coph_atlas &atlas,
           int &guard_steps) {
  const coph_config &cfg = *atlas.cfg;
  bool valid = true;
  for (double d : sorted_dampings(cfg, damping)) {
    const coph_state y = coph_flow<double>(x, u, d, cfg.dt);
    ++guard_steps;
    const coph_vec2 a{x[0], x[1]}, b{y[0], y[1]}, vel{y[2], y[3]};
    const double stopping = (vel[0] * vel[0] + vel[1] * vel[1]) / (2 * cfg.force_limit);
    const double speed = norm2(vel[0], vel[1]);
    if (maze(atlas)) {
      valid = corridor_clear(a, b, *atlas.network) && valid;
      const coph_vec2 endpoint{b[0] + vel[0] / (speed + 1e-20) * stopping,
                               b[1] + vel[1] / (speed + 1e-20) * stopping};
      valid = corridor_clear(b, endpoint, *atlas.network) && valid;
    }
    const double clear = kAgentRadius + cfg.safety_margin;
    for (const coph_shape &o : atlas.geometry) {
      if (o.slab) {
        valid = slab_sweep(a, b, o) >= clear - 1e-9 && valid;
        valid = coph_slab_distance(b, o) >= (clear + stopping) - 1e-9 && valid;
        continue;
      }
      valid = disk_sweep(a, b, o) >= (o.r + kAgentRadius + cfg.safety_margin) - 1e-9 && valid;
      valid = norm2(b[0] - o.x, b[1] - o.y) >=
                  ((o.r + kAgentRadius + cfg.safety_margin) + stopping) - 1e-9 &&
              valid;
    }
    valid = speed <= 1.0 + 1e-8 && valid;
  }
  return valid;
}

// FrozenStages.potential_at / energy: the stage's potential at q with the given target, the
// geometry store's obstacles and the profile's progress shaping.
template <class T>
double stage_energy(const coph_atlas &atlas, const coph_vec2_of<T> &q, int index,
                    const coph_vec2_of<T> &policy, const coph_vec2 &target) {
  const coph_frozen_stage &s = atlas.stages[index];
  coph_field_view f;
  f.centers = s.centers;
  f.weights = s.weights;
  f.width = atlas.cfg->rbf_width;
  f.obstacles = atlas.geometry;
  const coph_vec2 qv = values<T>(q), pv = values<T>(policy);
  return coph_potential<double>(qv, target, pv, f, coph_shaping_for(f, *atlas.cfg));
}

coph_vec2 energy_target(const coph_atlas &atlas, const coph_route_context &ctx) {
  return ctx.target ? *ctx.target : atlas.goal;
}

// The risk of the active stage's frozen field at q.
template <class T> T stage_risk(const coph_atlas &atlas, const coph_vec2_of<T> &q, int index) {
  const coph_frozen_stage &s = atlas.stages[index];
  coph_field_view f;
  f.centers = s.centers;
  f.weights = s.weights;
  f.width = atlas.cfg->rbf_width;
  return coph_risk<T>(q, f);
}

// A context as an entry's: the update freezes the navigation context's reported geometry and
// terrain anchors into every stage's entry context (MazeLearner.update, TerrainLearner.update).
coph_route_context entry_context(const coph_learner_state &s) {
  coph_route_context ctx;
  ctx.reported_geometry = s.reported_geometry;
  ctx.terrain_anchors = s.terrain_anchors;
  return ctx;
}

struct plan_lookup {
  const coph_replay_plan &plan;
  const coph_config &cfg;
  int horizon(std::int64_t id) const { return plan.horizon(id, cfg); }
};

// retained_replay on scalar T (double, or coph_dual2 for the gradient).
template <class T>
coph_replay_result replay_on(coph_learner_state &s, const coph_atlas &atlas,
                             const coph_replay_plan &plan, const coph_vec2 &policy_value,
                             double damping) {
  const coph_config &cfg = s.cfg;
  coph_vec2_of<T> policy{T(policy_value[0]), T(policy_value[1])};
  if constexpr (!std::is_same_v<T, double>) {
    policy[0] = T::variable(policy_value[0], 0);
    policy[1] = T::variable(policy_value[1], 1);
  }
  coph_replay_result out;
  const long long start = s.counterfactual_steps;
  std::vector<T> costs;
  for (int i = 0; i < static_cast<int>(atlas.stages.size()); ++i) {
    const coph_frozen_stage &st = atlas.stages[i];
    coph_state_of<T> x{T(st.entry[0]), T(st.entry[1]), T(st.entry[2]), T(st.entry[3])};
    int index = i;
    T cost(0.0);
    std::vector<coph_state> path{st.entry};
    coph_route_context ctx = maze(atlas) ? entry_context(s) : st.entry_navigation;
    if (!st.supported({st.entry[0], st.entry[1]}, cfg)) {
      out.eligibility.push_back({st.id, false, "unsupported_entry"});
      continue;
    }
    out.eligibility.push_back({st.id, true, "supported"});
    try {
      const int horizon = plan.horizon(st.id, cfg);
      for (int step = 0; step < horizon; ++step) {
        const coph_vec2 qv{coph_value(x[0]), coph_value(x[1])};
        const int next = atlas.advance(qv, index);
        if (next != index) {
          const coph_vec2_of<T> q{x[0], x[1]};
          const coph_vec2 target = energy_target(atlas, ctx);
          const double jump = stage_energy<T>(atlas, q, next, policy, target) -
                              stage_energy<T>(atlas, q, index, policy, target);
          out.events.push_back({st.id, step, atlas.stages[index].id, atlas.stages[next].id, jump});
        }
        index = next;
        const coph_frozen_stage &active = atlas.stages[index];
        coph_control_of<T> c = coph_control<T>(atlas, x, index, policy, damping, ctx, step);
        s.counterfactual_steps += c.meta.guard_steps;
        out.controls.push_back({st.id, step, c.meta});
        if (c.meta.reason == "nominal" || c.meta.reason == "projected")
          out.depends_on_policy = true;
        if (c.meta.infeasible)
          throw coph_domain_exit("safety_infeasible");
        const coph_state_of<T> after = coph_flow<T>(x, c.applied, T(damping), cfg.dt);
        ++s.counterfactual_steps;
        const coph_vec2 a = qv, b{coph_value(after[0]), coph_value(after[1])};
        if (!active.supported(b, cfg))
          throw coph_domain_exit("unsupported_motion");
        const coph_vec2_of<T> qa{after[0], after[1]};
        cost = cost + cfg.dt * ((1.0 + 4.0 * ctx.risk_weight * stage_risk<T>(atlas, qa, index)) +
                                0.03 * (c.applied[0] * c.applied[0] + c.applied[1] * c.applied[1]));
        const bool hit = collides(a, b, active.obstacles);
        x = after;
        path.push_back(values<T>(x));
        if (hit || norm2(b[0] - atlas.goal[0], b[1] - atlas.goal[1]) < kGoalRadius) {
          if (hit)
            cost = cost + kCollisionCost;
          break;
        }
      }
    } catch (const coph_domain_exit &e) {
      out.eligibility.back().reason = e.what();
      out.steps = s.counterfactual_steps - start;
      out.replayed = false;
      out.paths.clear();
      return out;
    }
    costs.push_back(cost);
    out.paths.push_back({st.id, std::move(path)});
  }
  out.steps = s.counterfactual_steps - start;
  if (costs.empty())
    return out;
  T total(0.0);
  for (const T &c : costs)
    total = total + c;
  const T mean = total / static_cast<double>(costs.size());
  out.replayed = true;
  out.cost = coph_value(mean);
  if constexpr (!std::is_same_v<T, double>)
    out.gradient = {mean.d[0], mean.d[1]};
  return out;
}

// The kept stages' transitions, oldest stage first, held out (uid % 5 == 0) or not.
std::vector<const coph_transition *> transitions(const coph_learner_state &s, bool validation) {
  std::vector<const coph_transition *> out;
  for (const coph_stage &st : s.stages)
    for (const coph_transition &t : st.transitions)
      if ((t.uid % 5 == 0) == validation)
        out.push_back(&t);
  return out;
}

template <class T>
T plant_loss(const std::vector<const coph_transition *> &records, const T &d, double dt) {
  if (records.empty())
    return T(0.0);
  T se(0.0), ee(0.0);
  for (const coph_transition *t : records) {
    const coph_state_of<T> x{T(t->x[0]), T(t->x[1]), T(t->x[2]), T(t->x[3])};
    const coph_vec2_of<T> u{T(t->u[0]), T(t->u[1])};
    const coph_state_of<T> pred = coph_flow<T>(x, u, d, dt);
    for (int k = 0; k < 4; ++k)
      se = se + tsq<T>(pred[k] - t->after[k]);
    const double v00 = t->x[2], v01 = t->x[3], v10 = t->after[2], v11 = t->after[3];
    const double delta = 0.5 * ((v10 * v10 + v11 * v11) - (v00 * v00 + v01 * v01));
    const T power = (t->u[0] * (v00 + v10) / 2 + t->u[1] * (v01 + v11) / 2) -
                    d * ((v00 * v00 + v01 * v01) + (v10 * v10 + v11 * v11)) / 2.0;
    ee = ee + tsq<T>(delta - dt * power);
  }
  const double n = static_cast<double>(records.size());
  return se / (4.0 * n) + 0.05 * (ee / n);
}

} // namespace

// ---------------------------------------------------------------------------
// The network and the state
// ---------------------------------------------------------------------------

coph_network coph_crossing_network() {
  coph_network n;
  n.nodes = {{0.0, 0.0}, {0.0, 4.0}, {0.0, 7.5}, {12.0, 0.0}, {12.0, 4.0}, {12.0, 7.5}};
  n.edges = {{0, 1}, {1, 2}, {1, 4}, {2, 5}, {0, 3}, {3, 4}, {4, 5}};
  return n;
}

bool coph_corridor_clear(const coph_vec2 &a, const coph_vec2 &b, const coph_network &network,
                         double margin) {
  return corridor_clear(a, b, network, margin);
}

double coph_slab_sweep(const coph_vec2 &a, const coph_vec2 &b, const coph_shape &slab) {
  return slab_sweep(a, b, slab);
}

bool coph_is_validation(const std::string &uid) {
  const std::size_t colon = uid.rfind(':');
  const std::string last = colon == std::string::npos ? uid : uid.substr(colon + 1);
  return std::stoll(last) % 5 == 0;
}

// ---------------------------------------------------------------------------
// G5
// ---------------------------------------------------------------------------

std::vector<coph_stage_fit> coph_field_fit(const coph_learner_state &s) {
  const coph_config &cfg = s.cfg;
  std::vector<coph_stage_fit> out;
  for (const coph_stage &st : s.stages) {
    const int m = static_cast<int>(st.centers.size());
    coph_stage_fit f;
    f.normal.assign(static_cast<std::size_t>(m) * m, 0.0);
    f.rhs.assign(m, 0.0);
    for (const coph_reading &r : st.readings) {
      if (coph_is_validation(r.uid))
        continue;
      const std::vector<double> phi = coph_features(r.q, st.centers, cfg.rbf_width);
      const double y = algo::det::log(algo::det::expm1(4 * std::max(r.value, 0.001)));
      f.targets.push_back(y);
      for (int i = 0; i < m; ++i) {
        for (int k = 0; k < m; ++k)
          f.normal[i * m + k] += phi[i] * phi[k];
        f.rhs[i] += phi[i] * y;
      }
      ++f.rows;
    }
    for (int i = 0; i < m; ++i) {
      f.normal[i * m + i] += cfg.fit_ridge;
      f.rhs[i] += cfg.fit_ridge * st.weights[i];
    }
    std::vector<double> l = f.normal;
    coph_cholesky(l, m, "coph_field_fit: the normal matrix is not positive definite");
    f.weights = coph_cholesky_solve(l, m, f.rhs);
    f.covariance.assign(static_cast<std::size_t>(m) * m, 0.0);
    for (int j = 0; j < m; ++j) {
      std::vector<double> e(m, 0.0);
      e[j] = 1.0;
      const std::vector<double> col = coph_cholesky_solve(l, m, e);
      for (int i = 0; i < m; ++i)
        f.covariance[i * m + j] = col[i];
    }
    out.push_back(std::move(f));
  }
  return out;
}

double coph_field_validation(const coph_learner_state &s,
                             const std::vector<std::vector<double>> &weights) {
  double total = 0.0;
  long long n = 0;
  for (std::size_t k = 0; k < s.stages.size(); ++k) {
    const coph_stage &st = s.stages[k];
    coph_field_view f;
    f.centers = st.centers;
    f.weights = weights[k];
    f.width = s.cfg.rbf_width;
    for (const coph_reading &r : st.readings) {
      if (!coph_is_validation(r.uid))
        continue;
      total += sq(coph_risk<double>(r.q, f) - r.value);
      ++n;
    }
  }
  return n ? total / static_cast<double>(n) : 0.0;
}

// ---------------------------------------------------------------------------
// G6
// ---------------------------------------------------------------------------

double coph_plant_loss(const coph_learner_state &s, double damping, bool validation) {
  return plant_loss<double>(transitions(s, validation), damping, s.cfg.dt);
}

coph_damping_fit coph_fit_damping(const coph_learner_state &s) {
  const std::vector<const coph_transition *> records = transitions(s, false);
  coph_damping_fit out;
  out.transitions = static_cast<int>(records.size());
  out.result = s.damping;
  if (records.size() < 3)
    return out;
  double d = s.damping;
  for (int k = 0; k < s.cfg.solver_steps; ++k) {
    coph_hyper h;
    h.v.v = d;
    h.v.d[0] = 1.0;
    h.d[0].v = 1.0;
    const coph_hyper loss = plant_loss<coph_hyper>(records, h, s.cfg.dt);
    coph_damping_iterate it;
    it.damping = d;
    it.loss = loss.v.v;
    it.gradient = loss.v.d[0];
    it.curvature = loss.d[0].d[0];
    it.step = d - it.gradient / (std::fabs(it.curvature) + 1e-5);
    it.next = clip(it.step, 0.05, 3.0);
    out.iterates.push_back(it);
    d = it.next;
  }
  out.result = d;
  return out;
}

double coph_reconcile_damping(double result, double committed, const coph_config &cfg) {
  return clip(result, std::max(cfg.damping_bounds.first, committed - cfg.damping_trust),
              std::min(cfg.damping_bounds.second, committed + cfg.damping_trust));
}

// ---------------------------------------------------------------------------
// The frozen stages and the maze router
// ---------------------------------------------------------------------------

bool coph_frozen_stage::supported(const coph_vec2 &q, const coph_config &cfg) const {
  if (!(std::fabs(q[0] - center[0]) <= cfg.stage_width / 2 + 1e-10 &&
        std::fabs(q[1] - center[1]) <= cfg.stage_height / 2 + 1e-10) ||
      support.empty())
    return false;
  double best = kInf;
  for (const coph_vec2 &p : support)
    best = std::min(best, norm2(p[0] - q[0], p[1] - q[1]));
  return best <= kSupportRadius;
}

int coph_atlas::advance(const coph_vec2 &q, int index) const {
  const coph_frozen_stage &old = stages[index];
  if (!old.supported(q, *cfg))
    throw coph_domain_exit("unsupported_state");
  const double hx = cfg->stage_width * (1 - cfg->overlap) / 2;
  const double hy = cfg->stage_height * (1 - cfg->overlap) / 2;
  const bool due = std::fabs(q[0] - old.center[0]) > hx || std::fabs(q[1] - old.center[1]) > hy;
  if (!due)
    return index;
  if (index + 1 >= static_cast<int>(stages.size()) || stages[index + 1].id != old.id + 1)
    throw coph_domain_exit("unavailable_stage");
  if (!stages[index + 1].supported(q, *cfg))
    throw coph_domain_exit("unsupported_overlap");
  return index + 1;
}

coph_atlas coph_freeze(const coph_learner_state &s,
                       const std::vector<std::vector<double>> &weights) {
  coph_atlas atlas;
  atlas.cfg = &s.cfg;
  atlas.network = &s.network;
  atlas.goal = s.goal;
  atlas.geometry = s.geometry;
  atlas.overflow = s.geometry_overflow;
  for (std::size_t k = 0; k < s.stages.size(); ++k) {
    const coph_stage &st = s.stages[k];
    coph_frozen_stage f;
    f.id = st.id;
    f.center = st.center;
    f.entry = st.entry;
    f.centers = st.centers;
    f.weights = weights[k];
    f.obstacles = st.obstacles;
    for (const coph_reading &r : st.readings) {
      if (coph_is_validation(r.uid))
        continue;
      if (std::find(f.support.begin(), f.support.end(), r.q) == f.support.end())
        f.support.push_back(r.q);
    }
    for (const auto &e : s.entry_navigation)
      if (e.first == st.id)
        f.entry_navigation = e.second;
    for (const coph_footprint &fp : st.footprints)
      if (std::find(atlas.footprints.begin(), atlas.footprints.end(), fp) == atlas.footprints.end())
        atlas.footprints.push_back(fp);
    atlas.stages.push_back(std::move(f));
  }
  return atlas;
}

coph_route coph_maze_route(const coph_vec2 &q, const coph_vec2 &goal, const coph_network &net,
                           coph_route_context &ctx, int /*step*/) {
  const std::vector<coph_shape> &obstacles = ctx.reported_geometry;
  const std::size_t n = net.nodes.size();
  // graph_costs: each open edge, priced by its dense midpoints' risk.
  std::vector<std::vector<std::pair<int, double>>> graph(n);
  coph_route out;
  for (std::size_t ei = 0; ei < net.edges.size(); ++ei) {
    const int i = net.edges[ei][0], j = net.edges[ei][1];
    if (!obstacle_clear(net.nodes[i], net.nodes[j], obstacles)) {
      out.blocked.push_back(static_cast<int>(ei));
      continue;
    }
    const double cost = priced_length(dense({net.nodes[i], net.nodes[j]}, 0.3), ctx);
    graph[i].push_back({j, cost});
    graph[j].push_back({i, cost});
  }
  // The virtual start connects along the occupied corridor, never through a closure or a wall.
  std::vector<int> order; // first insertion order, as a Python dict's
  std::map<int, double> connections;
  for (const auto &e : net.edges) {
    const coph_vec2 &a = net.nodes[e[0]], &b = net.nodes[e[1]];
    const double d0 = b[0] - a[0], d1 = b[1] - a[1];
    const double t =
        clip(((q[0] - a[0]) * d0 + (q[1] - a[1]) * d1) / (d0 * d0 + d1 * d1), 0.0, 1.0);
    if (norm2(q[0] - (a[0] + t * d0), q[1] - (a[1] + t * d1)) > 0.7)
      continue;
    for (int k : {e[0], e[1]}) {
      if (!corridor_clear(q, net.nodes[k], net) || !obstacle_clear(q, net.nodes[k], obstacles))
        continue;
      if (!connections.count(k))
        order.push_back(k);
      connections[k] = priced_length(dense({q, net.nodes[k]}, 0.15), ctx);
    }
  }
  // Dijkstra to the node nearest the goal.
  std::map<int, double> distances;
  std::map<int, int> parents;
  using item = std::pair<double, int>;
  std::priority_queue<item, std::vector<item>, std::greater<item>> queue;
  for (int k : order) {
    distances[k] = connections[k];
    parents[k] = -1;
    queue.push({connections[k], k});
  }
  int target_node = 0;
  double nearest = kInf;
  for (std::size_t k = 0; k < n; ++k) {
    const double d = norm2(net.nodes[k][0] - goal[0], net.nodes[k][1] - goal[1]);
    if (d < nearest) {
      nearest = d;
      target_node = static_cast<int>(k);
    }
  }
  while (!queue.empty()) {
    const auto [cost, i] = queue.top();
    queue.pop();
    if (cost > distances[i] + 1e-12)
      continue;
    if (i == target_node)
      break;
    for (const auto &[j, edge] : graph[i]) {
      const double proposed = cost + edge;
      const auto it = distances.find(j);
      if (proposed < (it == distances.end() ? kInf : it->second) - 1e-12) {
        distances[j] = proposed;
        parents[j] = i;
        queue.push({proposed, j});
      }
    }
  }
  if (!distances.count(target_node)) {
    ctx.target = q;
    ctx.blocked_edges = out.blocked;
    out.target = q;
    out.no_route = true;
    out.changed = true;
    out.reason = "maze_no_route";
    return out;
  }
  std::vector<int> ids{target_node};
  while (parents[ids.back()] != -1)
    ids.push_back(parents[ids.back()]);
  std::reverse(ids.begin(), ids.end());
  std::vector<coph_vec2> waypoints{q};
  for (int k : ids)
    waypoints.push_back(net.nodes[k]);
  const std::vector<coph_vec2> path = dense(waypoints, 0.15);
  coph_vec2 target = path.back();
  for (std::size_t k = 1; k < path.size(); ++k) {
    const coph_vec2 &p = path[k];
    if (!corridor_clear(q, p, net) || !obstacle_clear(q, p, obstacles))
      break;
    target = p;
    if (norm2(p[0] - q[0], p[1] - q[1]) >= kSupportRadius)
      break;
  }
  const coph_vec2 &tn = net.nodes[target_node];
  if (norm2(q[0] - tn[0], q[1] - tn[1]) < 1 && corridor_clear(q, tn, net) &&
      obstacle_clear(q, tn, obstacles))
    target = tn;
  // A fixed 0.1 m grid keeps the state derivatives away from grid switches (numpy.round: halves
  // to even, as nearbyint under the default rounding).
  target = {std::nearbyint(target[0] / 0.1) * 0.1, std::nearbyint(target[1] / 0.1) * 0.1};
  out.nodes = ids;
  if (!corridor_clear(q, target, net) || !obstacle_clear(q, target, obstacles)) {
    out.target = q;
    out.no_route = true;
    out.changed = true;
    out.reason = "unsafe_connector";
    return out;
  }
  out.changed = ids != ctx.route_nodes || out.blocked != ctx.blocked_edges;
  ctx.target = target;
  ctx.route_nodes = ids;
  ctx.blocked_edges = out.blocked;
  out.target = target;
  out.reason = "maze_graph";
  return out;
}

// navigation.edge_clear.
bool coph_edge_clear(const coph_vec2 &a, const coph_vec2 &b, const std::vector<coph_shape> &shapes,
                     const std::vector<coph_footprint> &footprints, double margin) {
  const double v0 = b[0] - a[0], v1 = b[1] - a[1], vv = v0 * v0 + v1 * v1;
  for (const coph_shape &o : shapes) {
    const double t = clip(((o.x - a[0]) * v0 + (o.y - a[1]) * v1) / (vv + 1e-20), 0.0, 1.0);
    if (norm2(a[0] + t * v0 - o.x, a[1] + t * v1 - o.y) <= o.r + margin + 1e-9)
      return false;
  }
  if (footprints.empty())
    return false;
  if (vv < 1e-18) {
    for (const coph_footprint &f : footprints)
      if (sq(f.x - a[0]) + sq(f.y - a[1]) <= f.radius * f.radius)
        return true;
    return false;
  }
  std::vector<std::pair<double, double>> intervals;
  for (const coph_footprint &f : footprints) {
    const double dx = f.x - a[0], dy = f.y - a[1];
    const double t = (dx * v0 + dy * v1) / vv;
    const double perp = (dx * dx + dy * dy) - t * t * vv;
    if (!(perp <= f.radius * f.radius))
      continue;
    const double d = std::sqrt(std::max(0.0, f.radius * f.radius - perp) / vv);
    intervals.push_back({std::max(0.0, t - d), std::min(1.0, t + d)});
  }
  std::sort(intervals.begin(), intervals.end());
  double end = 0.0;
  for (const auto &[lo, hi] : intervals) {
    if (hi < 0 || lo > 1)
      continue;
    if (lo > end + 1e-9)
      return false;
    end = std::max(end, hi);
    if (end >= 1 - 1e-9)
      return true;
  }
  return false;
}

namespace {

const coph_shape &disk_only(const coph_shape &o) {
  if (o.slab)
    throw std::invalid_argument("coph: the profile's router takes disks only");
  return o;
}

double apart(const coph_vec2 &a, const coph_vec2 &b) { return norm2(a[0] - b[0], a[1] - b[1]); }

// navigation.route's plan: A* on the 0.1 m grid. Sets path, reason and expansions.
void profile_plan(const coph_vec2 &q, const coph_atlas &atlas, std::vector<coph_vec2> &path,
                  std::string &reason, int &expansions) {
  const coph_config &cfg = *atlas.cfg;
  const coph_vec2 &goal = atlas.goal;
  const std::vector<coph_shape> &obstacles = atlas.geometry;
  const double margin = kAgentRadius + cfg.safety_margin;
  const double spacing = 0.1;
  double lo0 = kInf, lo1 = kInf, hi0 = -kInf, hi1 = -kInf;
  for (const coph_frozen_stage &st : atlas.stages) {
    lo0 = std::min(lo0, st.center[0]);
    lo1 = std::min(lo1, st.center[1]);
    hi0 = std::max(hi0, st.center[0]);
    hi1 = std::max(hi1, st.center[1]);
  }
  lo0 -= cfg.stage_width / 2;
  lo1 -= cfg.stage_height / 2;
  hi0 += cfg.stage_width / 2;
  hi1 += cfg.stage_height / 2;
  const long long l0 = static_cast<long long>(std::ceil(lo0 / spacing));
  const long long l1 = static_cast<long long>(std::ceil(lo1 / spacing));
  const long long u0 = static_cast<long long>(std::floor(hi0 / spacing));
  const long long u1 = static_cast<long long>(std::floor(hi1 / spacing));
  // The free grid points, in order of (i, j).
  std::vector<std::array<long long, 2>> nodes;
  std::vector<coph_vec2> points;
  for (long long i = l0; i <= u0; ++i)
    for (long long j = l1; j <= u1; ++j) {
      const coph_vec2 p{static_cast<double>(i) * spacing, static_cast<double>(j) * spacing};
      bool free = false;
      for (const coph_footprint &f : atlas.footprints) {
        const double r = std::max(0.0, f.radius - 0.075);
        free = free || sq(p[0] - f.x) + sq(p[1] - f.y) <= r * r;
      }
      for (const coph_shape &shape : obstacles) {
        const coph_shape &o = disk_only(shape);
        free = free &&
               sq(p[0] - o.x) + sq(p[1] - o.y) > sq(o.r + kAgentRadius + cfg.safety_margin + 0.075);
      }
      if (free) {
        nodes.push_back({i, j});
        points.push_back(p);
      }
    }
  const int n = static_cast<int>(points.size());
  if (n == 0) {
    path.clear();
    reason = "no_observed_free_cells";
    return;
  }
  // Each point's risk from the newest stage whose box and support hold it; 1 where none does.
  std::vector<double> values(n, 1.0);
  std::vector<bool> unassigned(n, true);
  for (auto it = atlas.stages.rbegin(); it != atlas.stages.rend(); ++it) {
    const coph_frozen_stage &st = *it;
    if (st.support.empty())
      continue;
    coph_field_view f;
    f.centers = st.centers;
    f.weights = st.weights;
    f.width = cfg.rbf_width;
    std::vector<int> ids;
    for (int k = 0; k < n; ++k) {
      if (!unassigned[k] || !(std::fabs(points[k][0] - st.center[0]) <= cfg.stage_width / 2) ||
          !(std::fabs(points[k][1] - st.center[1]) <= cfg.stage_height / 2))
        continue;
      double nearest = kInf;
      for (const coph_vec2 &sp : st.support)
        nearest = std::min(nearest, sq(points[k][0] - sp[0]) + sq(points[k][1] - sp[1]));
      if (nearest <= kSupportRadius * kSupportRadius)
        ids.push_back(k);
    }
    for (int k : ids) {
      values[k] = coph_risk<double>(points[k], f);
      unassigned[k] = false;
    }
  }
  const long long w1 = u1 - l1 + 1;
  std::vector<int> lookup(static_cast<std::size_t>((u0 - l0 + 1) * w1), -1);
  auto slot = [&](long long i, long long j) -> int {
    if (i < l0 || i > u0 || j < l1 || j > u1)
      return -1;
    return lookup[static_cast<std::size_t>((i - l0) * w1 + (j - l1))];
  };
  for (int k = 0; k < n; ++k)
    lookup[static_cast<std::size_t>((nodes[k][0] - l0) * w1 + (nodes[k][1] - l1))] = k;
  // Python's key (|p - to|, (i, j)).
  auto closer = [&](const coph_vec2 &to, int a, int b) {
    const double da = apart(points[a], to), db = apart(points[b], to);
    return da != db ? da < db : nodes[a] < nodes[b];
  };
  std::vector<int> starts(n);
  for (int k = 0; k < n; ++k)
    starts[k] = k;
  std::sort(starts.begin(), starts.end(), [&](int a, int b) { return closer(q, a, b); });
  int source = -1;
  for (int k = 0; k < std::min(n, 16); ++k)
    if (coph_edge_clear(q, points[starts[k]], obstacles, atlas.footprints, margin)) {
      source = starts[k];
      break;
    }
  if (source < 0) {
    path.clear();
    reason = "no_start_connector";
    return;
  }
  int destination = 0;
  for (int k = 1; k < n; ++k)
    if (closer(goal, k, destination))
      destination = k;
  // The graph: each node's edges in the order of the offsets.
  static constexpr int kOffsets[8][2] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                                         {0, 1},   {1, -1}, {1, 0},  {1, 1}};
  std::vector<std::vector<std::pair<int, double>>> adjacency(n);
  for (const auto &off : kOffsets) {
    const double d0 = static_cast<double>(off[0]) * spacing;
    const double d1 = static_cast<double>(off[1]) * spacing;
    const double dd = d0 * d0 + d1 * d1;
    const double length = std::sqrt(dd);
    for (int k = 0; k < n; ++k) {
      const int j = slot(nodes[k][0] + off[0], nodes[k][1] + off[1]);
      if (j < 0)
        continue;
      if (off[0] && off[1] &&
          (slot(nodes[k][0] + off[0], nodes[k][1]) < 0 ||
           slot(nodes[k][0], nodes[k][1] + off[1]) < 0))
        continue;
      bool keep = true;
      for (const coph_shape &o : obstacles) {
        const double t =
            clip(((o.x - points[k][0]) * d0 + (o.y - points[k][1]) * d1) / dd, 0.0, 1.0);
        keep = keep && norm2(points[k][0] + t * d0 - o.x, points[k][1] + t * d1 - o.y) >
                           o.r + kAgentRadius + cfg.safety_margin;
      }
      if (keep)
        adjacency[k].push_back({j, length * (1.0 + 4.0 * (values[k] + values[j]) / 2.0)});
    }
  }
  std::vector<double> goal_distances(n), heuristic(n);
  for (int k = 0; k < n; ++k) {
    goal_distances[k] = apart(points[k], goal);
    heuristic[k] = apart(points[k], points[destination]);
  }
  std::vector<bool> nearest(n, false);
  int nearest_left = 0;
  for (int k = 0; k < n; ++k)
    if (goal_distances[k] <= goal_distances[destination] + 1e-12) {
      nearest[k] = true;
      ++nearest_left;
    }
  std::vector<double> distances(n, kInf);
  std::vector<int> parents(n, -1);
  std::vector<bool> visited(n, false);
  std::vector<int> order; // the visited nodes
  using entry = std::pair<double, int>;
  std::priority_queue<entry, std::vector<entry>, std::greater<entry>> queue;
  distances[source] = 0.0;
  queue.push({apart(points[source], points[destination]), source});
  expansions = 0;
  while (!queue.empty() && expansions < cfg.route_expansion_cap) {
    const int i = queue.top().second;
    queue.pop();
    if (visited[i])
      continue;
    visited[i] = true;
    order.push_back(i);
    ++expansions;
    if (nearest[i] && --nearest_left == 0)
      break;
    for (const auto &[j, edge_cost] : adjacency[i]) {
      const double cost = distances[i] + edge_cost;
      if (cost < distances[j] - 1e-12) {
        distances[j] = cost;
        parents[j] = i;
        queue.push({cost + heuristic[j], j});
      }
    }
  }
  if (order.empty()) {
    path.clear();
    reason = "no_route";
    return;
  }
  double closest = kInf;
  for (int i : order)
    closest = std::min(closest, goal_distances[i]);
  int dest = -1;
  for (int i : order)
    if (goal_distances[i] <= closest + 1e-12 &&
        (dest < 0 || distances[i] < distances[dest] ||
         (distances[i] == distances[dest] && nodes[i] < nodes[dest])))
      dest = i;
  std::vector<int> ids{dest};
  while (ids.back() != source)
    ids.push_back(parents[ids.back()]);
  path.clear();
  for (auto it = ids.rbegin(); it != ids.rend(); ++it)
    path.push_back(points[*it]);
  if (coph_edge_clear(path.back(), goal, obstacles, atlas.footprints, margin) &&
      apart(points[dest], goal) < 0.2)
    path.push_back(goal);
  reason = apart(path.back(), goal) < 0.01 ? "goal_route" : "frontier_route";
}

} // namespace

coph_route coph_profile_route(const coph_vec2 &q, const coph_atlas &atlas, coph_route_context &ctx,
                              int step) {
  const coph_config &cfg = *atlas.cfg;
  const coph_vec2 &goal = atlas.goal;
  const std::vector<coph_shape> &obstacles = atlas.geometry;
  const double margin = kAgentRadius + cfg.safety_margin;
  std::vector<coph_vec2> path = ctx.path;
  const coph_vec2 old = ctx.target ? *ctx.target : goal;
  coph_route out;
  if (cfg.shaping_mode == "progress") {
    std::vector<std::array<double, 3>> history = ctx.progress_history;
    if (!history.empty() && step <= history.back()[0])
      history.clear();
    history.push_back({static_cast<double>(step), q[0], q[1]});
    if (history.size() > 17)
      history.erase(history.begin(), history.end() - 17);
    if (history.size() == 17 && step - history[0][0] == 16) {
      const coph_vec2 prior{history[0][1], history[0][2]};
      const double progress = apart(prior, old) - apart(q, old);
      out.stagnation = apart(q, prior) < 0.05 && progress < 0.05 && apart(q, goal) > 0.25 &&
                       step - ctx.last_stagnation >= cfg.update_every;
    }
    ctx.progress_history = history;
    if (out.stagnation)
      ctx.last_stagnation = step;
  }
  int cursor = ctx.cursor;
  const int size = static_cast<int>(path.size());
  if (!path.empty())
    while (cursor < size - 1 && apart(q, path[cursor]) < 0.2)
      ++cursor;
  double from_path = 1e9;
  for (const coph_vec2 &p : path)
    from_path = std::min(from_path, apart(q, p));
  bool stale =
      out.stagnation || path.empty() || step - ctx.planned_at >= cfg.update_every ||
      from_path > 0.25 ||
      !coph_edge_clear(q, path.empty() ? q : path[cursor], obstacles, atlas.footprints, margin);
  // A newly sensed disk invalidates any remaining route edge at once.
  if (!path.empty() && !obstacles.empty()) {
    std::vector<coph_vec2> pts{q};
    pts.insert(pts.end(), path.begin() + cursor, path.end());
    for (const coph_shape &shape : obstacles) {
      const coph_shape &o = disk_only(shape);
      bool hit = false;
      for (std::size_t k = 0; k + 1 < pts.size() && !hit; ++k) {
        const double d0 = pts[k + 1][0] - pts[k][0], d1 = pts[k + 1][1] - pts[k][1];
        const double t =
            clip(((o.x - pts[k][0]) * d0 + (o.y - pts[k][1]) * d1) / ((d0 * d0 + d1 * d1) + 1e-20),
                 0.0, 1.0);
        hit = norm2(pts[k][0] + t * d0 - o.x, pts[k][1] + t * d1 - o.y) <=
              o.r + kAgentRadius + cfg.safety_margin;
      }
      if (hit) {
        stale = true;
        break;
      }
    }
  }
  std::string reason = "cached";
  if (stale) {
    profile_plan(q, atlas, path, reason, out.expansions);
    cursor = 0;
    for (int k = 0; k < static_cast<int>(path.size()); ++k) {
      if (apart(path[k], q) <= 1.0 &&
          coph_edge_clear(q, path[k], obstacles, atlas.footprints, margin))
        cursor = k;
      else if (k > cursor)
        break;
    }
    ctx.path = path;
    ctx.cursor = cursor;
    ctx.planned_at = step;
  } else if (!path.empty() && cursor != ctx.cursor) {
    for (int k = cursor; k < static_cast<int>(path.size()); ++k) {
      if (apart(path[k], q) <= 1.0 &&
          coph_edge_clear(q, path[k], obstacles, atlas.footprints, margin))
        cursor = k;
      else
        break;
    }
  }
  coph_vec2 target = path.empty() ? q : path[cursor];
  if (cfg.shaping_mode == "progress" && apart(q, goal) <= 1.0 &&
      coph_edge_clear(q, goal, obstacles, atlas.footprints, margin)) {
    target = goal;
    path = {goal};
    cursor = 0;
    reason = "direct_goal";
    ctx.path = path;
  }
  if (!path.empty() && cursor == static_cast<int>(path.size()) - 1 && apart(q, target) < 0.2 &&
      apart(q, goal) > 0.25)
    ctx.planned_at = -10000;
  ctx.target = target;
  ctx.cursor = cursor;
  out.target = target;
  out.reason = reason;
  out.changed = stale;
  out.no_route = path.empty();
  out.path = path;
  return out;
}

template <class T>
coph_control_of<T> coph_control(const coph_atlas &atlas, const coph_state_of<T> &x, int index,
                                const coph_vec2_of<T> &policy, double damping,
                                coph_route_context &ctx, int step) {
  const coph_config &cfg = *atlas.cfg;
  const coph_frozen_stage &s = atlas.stages[index];
  coph_control_of<T> out;
  const coph_vec2 q{coph_value(x[0]), coph_value(x[1])};
  const bool on_network = maze(atlas);
  const coph_route route = on_network ? coph_maze_route(q, atlas.goal, *atlas.network, ctx, step)
                                      : coph_profile_route(q, atlas, ctx, step);
  out.meta.route_reason = route.reason;
  out.meta.route_nodes = route.nodes;
  out.meta.waypoint = route.target;
  out.meta.route_changed = route.changed;
  out.meta.route_expansions = route.expansions;
  out.meta.route = route.path;
  out.meta.stagnation = route.stagnation;
  coph_field_view f;
  f.centers = s.centers;
  f.weights = s.weights;
  f.width = cfg.rbf_width;
  f.obstacles = atlas.geometry;
  const coph_command_of<T> cmd =
      coph_command<T>(x, route.target, policy, f, coph_shaping_for(f, cfg), cfg.force_limit);
  const coph_vec2_of<T> &nominal = cmd.applied;
  out.applied = nominal;
  out.meta.proposed = values<T>(cmd.proposed);
  out.meta.reason = "nominal";
  projection<T> candidate;
  if (!atlas.overflow && !route.no_route)
    candidate = project<T>(cmd.proposed, x, atlas.geometry, damping, cfg, on_network);
  out.meta.projection_candidates = candidate.count;
  const coph_state xv = values<T>(x);
  if (candidate.point &&
      guard(xv, values<T>(*candidate.point), damping, atlas, out.meta.guard_steps)) {
    out.applied = *candidate.point;
    const coph_vec2 a = values<T>(out.applied), b = values<T>(nominal);
    out.meta.reason = norm2(a[0] - b[0], a[1] - b[1]) > 1e-9 ? "projected" : "nominal";
  } else {
    const T speed = tnorm<T>(x[2], x[3]);
    const T floor = coph_value(speed) < 1e-12 ? T(1e-12) : speed;
    T scale = speed / (cfg.force_limit * cfg.dt);
    if (coph_value(scale) > 1.0)
      scale = T(1.0);
    out.applied = {-cfg.force_limit * x[2] / floor * scale,
                   -cfg.force_limit * x[3] / floor * scale};
    const bool ok = guard(xv, values<T>(out.applied), damping, atlas, out.meta.guard_steps);
    out.meta.reason = atlas.overflow ? "geometry_overflow" : "brake";
    out.meta.infeasible = !ok;
  }
  out.meta.applied = values<T>(out.applied);
  return out;
}

template coph_control_of<double> coph_control<double>(const coph_atlas &,
                                                      const coph_state_of<double> &, int,
                                                      const coph_vec2_of<double> &, double,
                                                      coph_route_context &, int);
template coph_control_of<coph_dual2> coph_control<coph_dual2>(const coph_atlas &,
                                                              const coph_state_of<coph_dual2> &,
                                                              int, const coph_vec2_of<coph_dual2> &,
                                                              double, coph_route_context &, int);

// ---------------------------------------------------------------------------
// G7
// ---------------------------------------------------------------------------

int coph_replay_plan::horizon(std::int64_t stage, const coph_config &cfg) const {
  for (const coph_plan_entry &e : entries)
    if (e.stage == stage)
      return e.horizon;
  return cfg.replay_steps;
}

long long coph_remaining(const coph_learner_state &s, const coph_replay_plan &plan) {
  const long long cap = s.cfg.replay_update_cap.value_or(0);
  return std::max(0LL, cap - (s.counterfactual_steps - plan.budget_start) - plan.external_reserve);
}

long long coph_batch_steps(const coph_atlas &atlas, const coph_replay_plan &plan) {
  long long total = 0;
  for (const coph_frozen_stage &st : atlas.stages)
    if (st.supported({st.entry[0], st.entry[1]}, *atlas.cfg))
      total += plan.horizon(st.id, *atlas.cfg);
  return kRoutedMultiplier * total;
}

coph_replay_plan coph_prepare(coph_learner_state &s, const coph_atlas &atlas, double damping,
                              long long external_reserve) {
  const coph_config &cfg = s.cfg;
  coph_replay_plan plan;
  plan.budget_start = s.counterfactual_steps;
  plan.external_reserve = external_reserve;
  const int n = static_cast<int>(atlas.stages.size());
  for (int i = 0; i < n; ++i) {
    const coph_frozen_stage &st = atlas.stages[i];
    coph_plan_entry rec;
    rec.stage = st.id;
    rec.horizon = cfg.replay_steps;
    rec.reason = "fixed";
    rec.eligible = st.supported({st.entry[0], st.entry[1]}, cfg);
    plan.entries.push_back(rec);
    coph_plan_entry &r = plan.entries.back();
    if (!r.eligible) {
      r.reason = "unsupported_entry";
      continue;
    }
    if (cfg.replay_horizon_mode == "fixed")
      continue;
    if (i + 1 >= n || atlas.stages[i + 1].id != st.id + 1) {
      r.reason = "no_known_successor";
      continue;
    }
    long long reserved = 0;
    for (const coph_frozen_stage &t : atlas.stages)
      if (t.supported({t.entry[0], t.entry[1]}, cfg))
        reserved += plan.horizon(t.id, cfg);
    const long long reserve =
        (cfg.boundary_max_steps + 2 * (reserved - cfg.replay_steps + cfg.boundary_max_steps)) *
        kRoutedMultiplier;
    if (coph_remaining(s, plan) < reserve) {
      r.reason = "pilot_budget";
      continue;
    }
    coph_state x = st.entry;
    int index = i;
    std::optional<int> target;
    coph_route_context ctx = maze(atlas) ? entry_context(s) : st.entry_navigation;
    try {
      bool finished = false;
      for (int step = 0; step < cfg.boundary_max_steps; ++step) {
        const int next = atlas.advance({x[0], x[1]}, index);
        if (next != index) {
          plan.entries.back().handoffs.push_back(
              {step, atlas.stages[index].id, atlas.stages[next].id});
          if (!target) {
            target = std::max(cfg.replay_steps, step + cfg.boundary_follow_steps);
            if (*target > cfg.boundary_max_steps)
              throw coph_domain_exit("post_boundary_cap");
          }
        }
        index = next;
        const coph_control_of<double> c =
            coph_control<double>(atlas, x, index, s.policy, damping, ctx, step);
        plan.entries.back().pilot_steps += c.meta.guard_steps;
        s.counterfactual_steps += c.meta.guard_steps;
        plan.pilot_steps += c.meta.guard_steps;
        if (c.meta.infeasible)
          throw coph_domain_exit("pilot_safety_infeasible");
        const coph_state after = coph_flow<double>(x, c.applied, damping, cfg.dt);
        ++plan.entries.back().pilot_steps;
        ++s.counterfactual_steps;
        ++plan.pilot_steps;
        if (!atlas.stages[index].supported({after[0], after[1]}, cfg))
          throw coph_domain_exit("pilot_domain_exit");
        const coph_vec2 a{x[0], x[1]}, b{after[0], after[1]};
        const bool hit = collides(a, b, atlas.stages[index].obstacles);
        x = after;
        const bool terminal = hit || norm2(b[0] - s.goal[0], b[1] - s.goal[1]) < kGoalRadius;
        if (target && (step + 1 >= *target || terminal)) {
          plan.entries.back().horizon = *target;
          plan.entries.back().reason = "supported_boundary";
          finished = true;
          break;
        }
        if (terminal) {
          plan.entries.back().reason = "termination_before_handoff";
          finished = true;
          break;
        }
      }
      if (!finished)
        plan.entries.back().reason = "handoff_not_reached";
    } catch (const coph_domain_exit &e) {
      plan.entries.back().reason = e.what();
    }
  }
  plan.baseline_reserved = coph_remaining(s, plan) >= 2 * coph_batch_steps(atlas, plan);
  return plan;
}

coph_replay_result coph_replay(coph_learner_state &s, const coph_atlas &atlas,
                               const coph_replay_plan &plan, const coph_vec2 &policy,
                               double damping, bool gradient) {
  return gradient ? replay_on<coph_dual2>(s, atlas, plan, policy, damping)
                  : replay_on<double>(s, atlas, plan, policy, damping);
}

// ---------------------------------------------------------------------------
// G8
// ---------------------------------------------------------------------------

coph_update_report coph_update(coph_learner_state &s, const coph_state &x) {
  const coph_config &cfg = s.cfg;
  coph_update_report rep;
  coph_expire(s); // CooperativeLearner.update
  rep.fit = coph_field_fit(s);
  // CooperativeLearner._field_fit: with packets held, each stage takes its Gauss-Newton step.
  if (!s.exchange.received.empty())
    rep.message_fit = coph_fit_messages(s, rep.fit);
  std::vector<std::vector<double>> weights, committed;
  for (std::size_t k = 0; k < s.stages.size(); ++k) {
    weights.push_back(rep.message_fit.empty() ? rep.fit[k].weights : rep.message_fit[k].weights);
    committed.push_back(s.stages[k].weights);
  }
  rep.damping_fit = coph_fit_damping(s);
  rep.field_validation_before = coph_field_validation(s, committed);
  const double d = coph_reconcile_damping(rep.damping_fit.result, s.damping, cfg);
  rep.damping_proposal = d;
  coph_vec2 policy = s.policy;
  const coph_atlas atlas = coph_freeze(s, weights);
  rep.plan = coph_prepare(s, atlas, d, kExternalReserve);

  // Learner.replay: a batch only while the budget holds one.
  auto replay = [&](const coph_vec2 &p, bool gradient) -> std::optional<coph_replay_result> {
    if (coph_remaining(s, rep.plan) < coph_batch_steps(atlas, rep.plan)) {
      rep.exhausted = true;
      return std::nullopt;
    }
    return coph_replay(s, atlas, rep.plan, p, d, gradient);
  };
  std::optional<coph_replay_result> baseline;
  if (rep.plan.baseline_reserved)
    baseline = replay(policy, false);
  else
    rep.exhausted = true;
  if (baseline && baseline->replayed) {
    rep.replay_before = baseline->cost;
    // The Python replays again with the policy requiring a gradient; one dual pass gives both,
    // and its steps are counted again, as the second replay's.
    const std::optional<coph_replay_result> second = replay(policy, true);
    coph_vec2 g{0.0, 0.0};
    if (second && second->replayed && second->depends_on_policy)
      g = second->gradient;
    rep.policy_independent = !(second && second->depends_on_policy);
    rep.gradient = g;
    const coph_vec2 unconstrained{policy[0] - 0.2 * g[0], policy[1] - 0.2 * g[1]};
    rep.unconstrained = unconstrained;
    coph_vec2 bounded;
    for (int k = 0; k < 2; ++k) {
      const double lo = std::max(cfg.policy_bounds.first, policy[k] - cfg.policy_trust);
      const double hi = std::min(cfg.policy_bounds.second, policy[k] + cfg.policy_trust);
      bounded[k] = std::max(lo, std::min(hi, unconstrained[k]));
    }
    rep.bounded = bounded;
    const coph_vec2 start = policy;
    for (int power = 0; power < cfg.solver_steps; ++power) {
      if (coph_remaining(s, rep.plan) < coph_batch_steps(atlas, rep.plan)) {
        rep.exhausted = true;
        break;
      }
      const double f = std::ldexp(1.0, -power); // 0.5 ** power
      const coph_vec2 candidate{start[0] + (bounded[0] - start[0]) * f,
                                start[1] + (bounded[1] - start[1]) * f};
      const std::optional<coph_replay_result> r = replay(candidate, false);
      coph_update_attempt a;
      a.power = power;
      a.policy = candidate;
      const bool replayed = r && r->replayed && std::isfinite(r->cost);
      if (replayed)
        a.cost = r->cost;
      a.accepted = replayed && r->cost <= *rep.replay_before + 1e-10;
      a.reason = a.accepted ? "accepted" : (replayed ? "replay_cost" : "unsupported_or_nonfinite");
      rep.attempts.push_back(a);
      if (a.accepted) {
        policy = candidate;
        break;
      }
    }
  }
  // A baseline that left the domain, or found the budget spent, makes no policy step.
  rep.field_validation_after = coph_field_validation(s, weights);
  rep.plant_validation_before = coph_plant_loss(s, s.damping, true);
  rep.plant_validation_after = coph_plant_loss(s, d, true);
  bool finite = std::isfinite(policy[0]) && std::isfinite(policy[1]) && std::isfinite(d);
  for (const auto &w : weights)
    for (double v : w)
      finite = finite && std::isfinite(v);
  rep.finite = finite;
  rep.feasible = cfg.damping_bounds.first <= d && d <= cfg.damping_bounds.second &&
                 policy[0] >= 0 && policy[1] >= 0 && policy[0] <= cfg.policy_bounds.second &&
                 policy[1] <= cfg.policy_bounds.second;
  rep.valid = rep.field_validation_after <= rep.field_validation_before + cfg.validation_slack &&
              rep.plant_validation_after <= rep.plant_validation_before + cfg.validation_slack;
  rep.accepted = rep.finite && rep.feasible && rep.valid;
  rep.reason =
      rep.accepted ? "accepted" : (!rep.finite ? "nonfinite" : "validation_or_feasibility");

  // Learner.shaping_energy on the newest stage: the goal as the target, the geometry store's
  // obstacles, the committed weights and policy.
  auto shaping_energy = [&]() {
    const coph_stage &st = s.stages.back();
    coph_field_view f;
    f.centers = st.centers;
    f.weights = st.weights;
    f.width = cfg.rbf_width;
    f.obstacles = s.geometry;
    return coph_potential<double>({x[0], x[1]}, s.goal, s.policy, f, coph_shaping_for(f, cfg));
  };
  const double old = shaping_energy();
  if (rep.accepted) {
    ++s.version;
    for (std::size_t k = 0; k < s.stages.size(); ++k) {
      s.stages[k].weights = weights[k];
      s.stages[k].version = s.version;
    }
    s.policy = policy;
    s.damping = d;
  }
  rep.shaping_jump = shaping_energy() - old;
  rep.version_after = s.version;
  rep.policy_after = s.policy;
  rep.damping_after = s.damping;
  for (const auto &r : s.exchange.received)
    rep.messages.push_back(r.second.identity);
  return rep;
}

} // namespace nav
} // namespace cvc
