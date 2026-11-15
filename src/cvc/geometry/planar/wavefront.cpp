/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::planar::wavefront_offset (inc/cvc/geometry/planar.h): the straight skeleton's offset,
// which ContourTiler took from CGAL's GPL Straight Skeleton package, from mitred Clipper2 offsets.
//
// The straight skeleton's offset at distance d is the wavefront: every edge moves along its normal
// at unit speed, and each vertex moves along its angle's bisector so that it stays on both of its
// edges (the unlimited mitre). Two kinds of event change the wavefront's shape:
//  - an edge event: an edge shrinks to nothing, and its neighbours become adjacent;
//  - a split event: a vertex reaches another edge, and the wavefront splits (shrinking) or merges
//    (growing) there.
// Between events a single mitred offset pass is exactly the wavefront: every edge moved by d,
// consecutive edges meeting at their mitre. Past an edge event it is not: a single pass still
// bounds the region by the vanished edge's offset line, where the skeleton mitres the neighbours
// (the finding of the VolumeRover ports' P0c A/B, tools/cgal_ab). So this offsets in steps:
//  1. clean the current polygons (no collinear vertices, no edge shorter than kMinEdge: the stub an
//     edge event leaves after rounding, whose direction is noise);
//  2. find the time of the wavefront's next event (each edge's collapse from its endpoints'
//     velocities; each reflex vertex's arrival on another edge, within that edge's moving extent);
//  3. offset by that much (at least kMinStep, and no more than what is left) in one mitred pass,
//     whose clean-up resolves the event.
// The steps run on a grid kScale (2^20) times finer than the caller's, still exact in a double
// and well inside Clipper2's range, so rounding at each step adds no visible error; the result is
// rounded to the caller's grid once, at the end. Each step costs O(r n) for the event search (r
// reflex vertices, n vertices) plus the Clipper2 pass; the number of steps is at most the number of
// events, and at most kMaxStepsPerVertex * n + 64 before the rest is done in one pass.

#include "planar_clipper.h"
#include "planar_internal.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace cvc {
namespace planar {

namespace {

// The internal grid: kScale fine units to the caller's unit. Coordinates stay within 2^51, exact in
// a double (the event search) and inside Clipper2's +-2^61.
constexpr double kScale = 1048576.0; // 2^20
// A step is at least 1/16 of the caller's unit (Clipper2 returns its input for |delta| < 0.5 fine
// units).
constexpr double kMinStep = kScale / 16;
// Shorter edges (fine units: about 6e-5 of the caller's unit) are merged into a vertex before each
// step: the stub an edge event leaves after rounding.
constexpr double kMinEdge = 64.0;
// Unlimited mitres, as the skeleton's (a vertex moves at most this many times the distance).
constexpr double kMiterLimit = 1e6;
constexpr std::size_t kMaxStepsPerVertex = 4;

struct vec {
  double x = 0, y = 0;
};
vec operator+(vec a, vec b) { return {a.x + b.x, a.y + b.y}; }
vec operator-(vec a, vec b) { return {a.x - b.x, a.y - b.y}; }
vec operator*(double s, vec a) { return {s * a.x, s * a.y}; }
double dot(vec a, vec b) { return a.x * b.x + a.y * b.y; }
double cross(vec a, vec b) { return a.x * b.y - a.y * b.x; }
vec of(const point &p) { return {double(p.x), double(p.y)}; }

// The ring with every edge shorter than kMinEdge merged into its midpoint.
ring merge_short_edges(ring r) {
  r = clean(r);
  bool changed = true;
  while (changed && r.size() >= 3) {
    changed = false;
    for (std::size_t i = 0; i < r.size(); ++i) {
      const std::size_t j = (i + 1) % r.size();
      const double dx = double(r[j].x - r[i].x), dy = double(r[j].y - r[i].y);
      if (dx * dx + dy * dy < kMinEdge * kMinEdge) {
        r[i] = point(coord(std::llround((double(r[i].x) + double(r[j].x)) / 2)),
                     coord(std::llround((double(r[i].y) + double(r[j].y)) / 2)));
        r.erase(r.begin() + std::ptrdiff_t(j));
        r = clean(r);
        changed = true;
        break;
      }
    }
  }
  return r;
}

polygon_set prepare(const polygon_set &s) {
  polygon_set out;
  for (const polygon &p : s) {
    polygon q;
    q.outer = merge_short_edges(p.outer);
    if (q.outer.size() < 3)
      continue;
    for (const ring &h : p.holes) {
      ring c = merge_short_edges(h);
      if (c.size() >= 3)
        q.holes.push_back(std::move(c));
    }
    out.push_back(normalize(q));
  }
  return out;
}

// A wavefront vertex: its position, its velocity, and its outgoing edge's direction and length.
struct wvertex {
  vec p, v, u;
  double length = 0;
  bool reflex = false; // moves ahead of its edges: may reach another edge
};
struct wedge {
  vec q0, q1, w0, w1, u, m; // endpoints, their velocities, direction, motion normal
};

// The time until the wavefront of the (normalised) set, moving outward (sign +1) or inward (-1),
// next changes shape; infinity if it never does.
double next_event(const polygon_set &s, int sign) {
  std::vector<std::vector<wvertex>> rings;
  for (const polygon &p : s) {
    auto add = [&](const ring &r) {
      const std::size_t n = r.size();
      std::vector<wvertex> vs(n);
      std::vector<vec> m(n);
      for (std::size_t i = 0; i < n; ++i) {
        const vec d = of(r[(i + 1) % n]) - of(r[i]);
        const double len = std::sqrt(dot(d, d));
        vs[i].p = of(r[i]);
        vs[i].u = (1 / len) * d;
        vs[i].length = len;
        // Rings keep the region on their left; outward is the right-hand normal.
        m[i] = sign > 0 ? vec{vs[i].u.y, -vs[i].u.x} : vec{-vs[i].u.y, vs[i].u.x};
      }
      for (std::size_t i = 0; i < n; ++i) {
        const std::size_t h = (i + n - 1) % n;
        // v . m[h] = v . m[i] = 1: the vertex stays on both moving edges.
        const double denom = std::max(1 + dot(m[h], m[i]), 1e-12);
        vs[i].v = (1 / denom) * (m[h] + m[i]);
        // A left turn is a convex vertex of the region, which runs ahead when growing; a right
        // turn a reflex one, which runs ahead when shrinking.
        vs[i].reflex = sign * cross(vs[h].u, vs[i].u) > 0;
      }
      rings.push_back(std::move(vs));
    };
    add(p.outer);
    for (const ring &h : p.holes)
      add(h);
  }
  double best = std::numeric_limits<double>::infinity();
  std::vector<wedge> edges;
  for (const auto &vs : rings) {
    const std::size_t n = vs.size();
    for (std::size_t i = 0; i < n; ++i) {
      const wvertex &a = vs[i], &b = vs[(i + 1) % n];
      // Edge event: the edge's length changes at (v_b - v_a) . u.
      const double rate = dot(b.v - a.v, a.u);
      if (rate < 0)
        best = std::min(best, a.length / -rate);
      const vec m = sign > 0 ? vec{a.u.y, -a.u.x} : vec{-a.u.y, a.u.x};
      edges.push_back({a.p, b.p, a.v, b.v, a.u, m});
    }
  }
  // Split events: a reflex vertex ahead of an edge's front, closing on it, and landing within the
  // edge's extent at that time.
  std::size_t base = 0;
  for (const auto &vs : rings) {
    const std::size_t n = vs.size();
    for (std::size_t i = 0; i < n; ++i) {
      const wvertex &a = vs[i];
      if (!a.reflex)
        continue;
      const std::size_t in = base + (i + n - 1) % n, out = base + i;
      for (std::size_t k = 0; k < edges.size(); ++k) {
        if (k == in || k == out)
          continue;
        const wedge &e = edges[k];
        const double gap = dot(e.m, a.p - e.q0);
        const double closing = 1 - dot(e.m, a.v);
        if (gap <= 0 || closing <= 0)
          continue;
        const double t = gap / closing;
        if (t >= best)
          continue;
        const vec h = a.p + t * a.v;
        const vec q0 = e.q0 + t * e.w0, q1 = e.q1 + t * e.w1, d = q1 - q0;
        if (dot(d, e.u) <= 0)
          continue; // the edge has collapsed by then
        const double lambda = dot(h - q0, d) / dot(d, d);
        if (lambda >= 0 && lambda <= 1)
          best = t;
      }
    }
    base += n;
  }
  return best;
}

// Every vertex times `s`, rounded (scaling up by a power of two is exact).
polygon_set scaled(const polygon_set &in, double s) {
  polygon_set out = in;
  auto scale = [s](ring &r) {
    for (point &p : r)
      p = point(coord(std::llround(double(p.x) * s)), coord(std::llround(double(p.y) * s)));
  };
  for (polygon &p : out) {
    scale(p.outer);
    for (ring &h : p.holes)
      scale(h);
  }
  return out;
}

std::size_t vertex_count(const polygon_set &s) {
  std::size_t n = 0;
  for (const polygon &p : s) {
    n += p.outer.size();
    for (const ring &h : p.holes)
      n += h.size();
  }
  return n;
}

} // namespace

polygon_set wavefront_offset(const polygon_set &s, double delta) {
  detail::check_range("planar::wavefront_offset", s);
  if (!std::isfinite(delta))
    throw planar_error("planar::wavefront_offset: the distance is not finite");
  // The set as non-overlapping polygons.
  Clipper2Lib::PolyTree64 tree;
  Clipper2Lib::BooleanOp(Clipper2Lib::ClipType::Union, Clipper2Lib::FillRule::NonZero,
                         detail::to_paths(s), {}, tree);
  polygon_set cur = scaled(detail::from_tree(tree), kScale);
  const int sign = delta > 0 ? 1 : -1;
  double remaining = std::fabs(delta) * kScale;
  const std::size_t max_steps = kMaxStepsPerVertex * vertex_count(cur) + 64;
  std::size_t steps = 0;
  while (remaining >= kMinStep && !cur.empty()) {
    cur = prepare(cur);
    if (cur.empty())
      break;
    double step = remaining;
    if (++steps <= max_steps && remaining > kMinStep) {
      step = std::min(remaining, std::max(next_event(cur, sign), kMinStep));
      if (remaining - step < kMinStep)
        step = remaining; // what would be left is too small for a pass of its own
    }
    cur = detail::mitred_offset(detail::to_paths(cur), sign * step, kMiterLimit);
    remaining -= step;
  }
  // Back to the caller's grid; rounding may pinch thin parts, so resolve again.
  Clipper2Lib::PolyTree64 out;
  Clipper2Lib::BooleanOp(Clipper2Lib::ClipType::Union, Clipper2Lib::FillRule::NonZero,
                         detail::to_paths(scaled(cur, 1 / kScale)), {}, out);
  polygon_set result = detail::from_tree(out);
  detail::check_result_range("planar::wavefront_offset", result);
  return result;
}

} // namespace planar
} // namespace cvc
