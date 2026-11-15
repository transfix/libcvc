/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::planar::medial_axis (inc/cvc/geometry/planar.h): a polygon's medial axis from
// Boost.Polygon's Voronoi diagram of its edges (Boost Software Licence; Boost is already libcvc's
// dependency). It stands where ContourTiler had CGAL's GPL straight skeleton: no permissive C++
// straight skeleton exists (the ports plan, §1.2).
//
// Boost.Polygon builds the Voronoi diagram of segments exactly from their int32 endpoints (the
// grid's range). Each polygon edge gives three sites: its two endpoints and its inside. The medial
// axis is the diagram's edges inside the polygon, without the "secondary" ones between an edge and
// its own endpoint (they run along the edge's normal at a reflex vertex):
//  - an edge with a segment site lies on one side of that segment's line, the inside when it is to
//    the segment's left (rings keep the region on their left);
//  - an edge between two endpoint sites lies inside when they are reflex vertices, whose cells
//    are wedges inside the polygon (a convex vertex's cell lies outside).
// A point-segment edge is a parabolic arc, sampled until no chord strays more than the tolerance.

#include "planar_internal.h"

#include <algorithm>
#include <boost/polygon/point_concept.hpp>
#include <boost/polygon/point_data.hpp>
#include <boost/polygon/segment_concept.hpp>
#include <boost/polygon/segment_data.hpp>
#include <boost/polygon/voronoi.hpp>
#include <cmath>
#include <cstddef>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cvc {
namespace planar {

namespace {

namespace bp = boost::polygon;
using bp_point = bp::point_data<int>;
using bp_segment = bp::segment_data<int>;
using vd_t = bp::voronoi_diagram<double>;

struct vec {
  double x = 0, y = 0;
};
vec sub(const vec &a, const vec &b) { return {a.x - b.x, a.y - b.y}; }
double dot(const vec &a, const vec &b) { return a.x * b.x + a.y * b.y; }
vec of(const point &p) { return {double(p.x), double(p.y)}; }
vec of(const bp_point &p) { return {double(p.x()), double(p.y())}; }

double dist_to_segment(const vec &q, const vec &a, const vec &b) {
  const vec d = sub(b, a), w = sub(q, a);
  const double l2 = dot(d, d);
  double t = l2 > 0 ? dot(w, d) / l2 : 0;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(q.x - (a.x + t * d.x), q.y - (a.y + t * d.y));
}

struct builder {
  const std::vector<bp_segment> &segs;
  const std::map<point, bool> &reflex; // polygon vertex -> is it reflex?
  double tolerance;
  medial_axis_graph out;
  std::unordered_map<const vd_t::vertex_type *, int> index;

  // The endpoint a point site stands for.
  bp_point site_point(const vd_t::cell_type &c) const {
    const bp_segment &s = segs[c.source_index()];
    return c.source_category() == bp::SOURCE_CATEGORY_SEGMENT_START_POINT ? s.low() : s.high();
  }

  // Distance from q to the cell's site.
  double clearance(const vd_t::cell_type &c, const vec &q) const {
    if (c.contains_segment()) {
      const bp_segment &s = segs[c.source_index()];
      return dist_to_segment(q, of(s.low()), of(s.high()));
    }
    const vec p = of(site_point(c));
    return std::hypot(q.x - p.x, q.y - p.y);
  }

  int add_vertex(const vec &p, double r) {
    out.vertices.push_back(pointd(p.x, p.y));
    out.radius.push_back(r);
    return int(out.vertices.size()) - 1;
  }

  int vertex_of(const vd_t::vertex_type *v, const vd_t::cell_type &c) {
    auto it = index.find(v);
    if (it != index.end())
      return it->second;
    const vec p{v->x(), v->y()};
    const int i = add_vertex(p, clearance(c, p));
    index.emplace(v, i);
    return i;
  }

  bool inside(const vd_t::edge_type &e) const {
    const vd_t::cell_type &c1 = *e.cell(), &c2 = *e.twin()->cell();
    const vec m{(e.vertex0()->x() + e.vertex1()->x()) / 2,
                (e.vertex0()->y() + e.vertex1()->y()) / 2};
    for (const vd_t::cell_type *c : {&c1, &c2})
      if (c->contains_segment()) {
        const bp_segment &s = segs[c->source_index()];
        return orientation(pointd(double(s.low().x()), double(s.low().y())),
                           pointd(double(s.high().x()), double(s.high().y())),
                           pointd(m.x, m.y)) > 0;
      }
    const bp_point p = site_point(c1);
    const auto it = reflex.find(point(p.x(), p.y()));
    return it != reflex.end() && it->second;
  }

  // The parabola of points equidistant from the focus and the directrix's line, sampled from a to
  // b (both on it) until no chord strays more than the tolerance; the points between a and b in
  // order, with their clearances.
  void sample(const vec &focus, const vec &d0, const vec &d1, const vec &a, const vec &b,
              std::vector<std::pair<vec, double>> &pts) const {
    // Local frame: x along the directrix, y towards the focus.
    const double len = std::hypot(d1.x - d0.x, d1.y - d0.y);
    if (!(len > 0))
      return;
    vec u{(d1.x - d0.x) / len, (d1.y - d0.y) / len}, n{-u.y, u.x};
    if (dot(sub(focus, d0), n) < 0)
      n = {-n.x, -n.y};
    const double fx = dot(sub(focus, d0), u), fy = dot(sub(focus, d0), n);
    if (!(fy > 1e-9))
      return; // the focus on the line: the arc is straight
    auto y_at = [&](double x) { return ((x - fx) * (x - fx) + fy * fy) / (2 * fy); };
    auto world = [&](double x) {
      const double y = y_at(x);
      return std::make_pair(vec{d0.x + x * u.x + y * n.x, d0.y + x * u.y + y * n.y}, y);
    };
    const double xa = dot(sub(a, d0), u), xb = dot(sub(b, d0), u);
    // Recursive halving, depth-limited; the vertical gap bounds the chord's distance to the arc.
    struct span {
      double x0, x1;
      int depth;
    };
    std::vector<span> stack = {{xa, xb, 0}};
    std::vector<double> xs;
    while (!stack.empty()) {
      const span s = stack.back();
      stack.pop_back();
      const double xm = (s.x0 + s.x1) / 2;
      const double gap = std::fabs(y_at(xm) - (y_at(s.x0) + y_at(s.x1)) / 2);
      if (gap > tolerance && s.depth < 24) {
        stack.push_back({xm, s.x1, s.depth + 1}); // the far half after the near one
        stack.push_back({s.x0, xm, s.depth + 1});
      } else {
        xs.push_back(s.x1);
      }
    }
    xs.pop_back(); // the last is b itself
    for (double x : xs)
      pts.push_back(world(x));
  }

  void add_edge(const vd_t::edge_type &e) {
    const vd_t::cell_type &c1 = *e.cell(), &c2 = *e.twin()->cell();
    const int a = vertex_of(e.vertex0(), c1), b = vertex_of(e.vertex1(), c1);
    std::vector<std::pair<vec, double>> mid;
    if (e.is_curved()) {
      const vd_t::cell_type &pc = c1.contains_point() ? c1 : c2;
      const vd_t::cell_type &sc = c1.contains_point() ? c2 : c1;
      const bp_segment &s = segs[sc.source_index()];
      sample(of(site_point(pc)), of(s.low()), of(s.high()), {e.vertex0()->x(), e.vertex0()->y()},
             {e.vertex1()->x(), e.vertex1()->y()}, mid);
    }
    int prev = a;
    for (const auto &m : mid) {
      const int k = add_vertex(m.first, m.second);
      out.edges.push_back({prev, k});
      prev = k;
    }
    out.edges.push_back({prev, b});
  }

  // Chains of degree-2 vertices between vertices of any other degree; then the cycles left.
  void chain() {
    const std::size_t nv = out.vertices.size();
    std::vector<std::vector<std::pair<int, int>>> adj(nv); // (neighbour, edge)
    for (int i = 0; i < int(out.edges.size()); ++i) {
      adj[std::size_t(out.edges[i][0])].push_back({out.edges[i][1], i});
      adj[std::size_t(out.edges[i][1])].push_back({out.edges[i][0], i});
    }
    std::vector<char> used(out.edges.size(), 0);
    auto walk = [&](int start, int first_edge) {
      std::vector<int> line = {start};
      int v = start, e = first_edge;
      while (true) {
        used[std::size_t(e)] = 1;
        const int w = out.edges[std::size_t(e)][0] == v ? out.edges[std::size_t(e)][1]
                                                        : out.edges[std::size_t(e)][0];
        line.push_back(w);
        if (adj[std::size_t(w)].size() != 2 || w == start)
          break;
        const auto &n = adj[std::size_t(w)];
        const int next = used[std::size_t(n[0].second)] ? n[1].second : n[0].second;
        if (used[std::size_t(next)])
          break;
        v = w;
        e = next;
      }
      out.polylines.push_back(std::move(line));
    };
    for (int v = 0; v < int(nv); ++v)
      if (adj[std::size_t(v)].size() != 2)
        for (const auto &n : adj[std::size_t(v)])
          if (!used[std::size_t(n.second)])
            walk(v, n.second);
    for (int e = 0; e < int(out.edges.size()); ++e)
      if (!used[std::size_t(e)])
        walk(out.edges[std::size_t(e)][0], e);
  }
};

} // namespace

medial_axis_graph medial_axis(const polygon &p, const medial_axis_params &params) {
  detail::check_range("planar::medial_axis", p);
  if (!(params.tolerance > 0))
    throw planar_error("planar::medial_axis: the tolerance must be positive");
  if (!is_simple(p))
    throw planar_error("planar::medial_axis: the polygon is not simple (an edge crosses or touches "
                       "another, a vertex repeats, or a hole meets another ring)");
  // Collinear vertices go (their sites' cells are degenerate); rings keep the region on their left.
  const polygon n = normalize(p);
  std::vector<bp_segment> segs;
  std::map<point, bool> reflex;
  auto add = [&](const ring &r) {
    const std::size_t k = r.size();
    for (std::size_t i = 0; i < k; ++i) {
      const point &a = r[i], &b = r[(i + 1) % k];
      segs.emplace_back(bp_point(int(a.x), int(a.y)), bp_point(int(b.x), int(b.y)));
      reflex[a] = orientation(r[(i + k - 1) % k], a, b) < 0;
    }
  };
  if (n.outer.size() >= 3) {
    add(n.outer);
    for (const ring &h : n.holes)
      add(h);
  }
  builder b{segs, reflex, params.tolerance, {}, {}};
  if (segs.empty())
    return b.out;
  vd_t vd;
  bp::construct_voronoi(segs.begin(), segs.end(), &vd);
  for (const vd_t::edge_type &e : vd.edges()) {
    if (e.color() != 0)
      continue;
    e.color(1);
    e.twin()->color(1);
    if (!e.is_primary() || !e.is_finite() || !b.inside(e))
      continue;
    b.add_edge(e);
  }
  b.chain();
  return b.out;
}

} // namespace planar
} // namespace cvc
