/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::planar's triangulations (inc/cvc/geometry/planar.h) on CDT 2.0 (artem-ogre/CDT, MPL-2.0,
// cvcpkg's cdt), replacing CGAL's GPL Constrained_Delaunay_triangulation_2 and
// Delaunay_triangulation_2, and the Hertel-Mehlhorn convex partition replacing CGAL's
// approx_convex_partition_2. CDT's predicates are William Lenthe's robust ones; grid coordinates
// are exact as doubles. CDT's randomised steps use a generator local to each call, seeded with 0,
// so a triangulation is the same on every run.

#include "planar_internal.h"

#include <CDT/CDT.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <map>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace planar {

namespace {

using cdt_t = CDT::Triangulation<double>;

// neighbors[t][i]: the triangle across the edge opposite vertex i, from the triangles alone.
std::vector<std::array<int, 3>> neighbours_of(const std::vector<std::array<int, 3>> &tris) {
  std::map<std::pair<int, int>, std::pair<int, int>> by_edge; // sorted edge -> (triangle, slot)
  std::vector<std::array<int, 3>> nb(tris.size(), {-1, -1, -1});
  for (int t = 0; t < int(tris.size()); ++t)
    for (int i = 0; i < 3; ++i) {
      const int a = tris[t][(i + 1) % 3], b = tris[t][(i + 2) % 3];
      const std::pair<int, int> key(std::min(a, b), std::max(a, b));
      auto it = by_edge.find(key);
      if (it == by_edge.end()) {
        by_edge.emplace(key, std::make_pair(t, i));
      } else {
        nb[t][i] = it->second.first;
        nb[it->second.first][it->second.second] = t;
      }
    }
  return nb;
}

// CDT's triangles as indices into `index_of` (CDT's vertex -> ours), counterclockwise.
void take_triangles(const cdt_t &cdt, const std::vector<int> &index_of,
                    const std::vector<point> &pts, triangulation &out) {
  out.triangles.reserve(cdt.triangles.size());
  for (const CDT::Triangle &t : cdt.triangles) {
    std::array<int, 3> tri = {index_of[t.vertices[0]], index_of[t.vertices[1]],
                              index_of[t.vertices[2]]};
    const int o = orientation(pts[tri[0]], pts[tri[1]], pts[tri[2]]);
    if (o == 0)
      continue; // a flat triangle of collinear input
    if (o < 0)
      std::swap(tri[1], tri[2]);
    out.triangles.push_back(tri);
  }
  out.neighbors = neighbours_of(out.triangles);
}

// The distinct points, in first-appearance order, and each input point's index among them.
void dedupe(const std::vector<point> &pts, std::vector<int> &first_of,
            std::vector<int> &unique_to_input, std::vector<int> &input_to_unique) {
  std::vector<int> order(pts.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return pts[a] < pts[b]; });
  first_of.assign(pts.size(), -1);
  for (std::size_t k = 0; k < order.size(); ++k) {
    const int i = order[k];
    first_of[i] = (k > 0 && pts[order[k - 1]] == pts[i]) ? first_of[order[k - 1]] : i;
  }
  input_to_unique.assign(pts.size(), -1);
  unique_to_input.clear();
  for (int i = 0; i < int(pts.size()); ++i)
    if (first_of[i] == i) {
      input_to_unique[i] = int(unique_to_input.size());
      unique_to_input.push_back(i);
    }
  for (int i = 0; i < int(pts.size()); ++i)
    input_to_unique[i] = input_to_unique[first_of[i]];
}

std::vector<CDT::V2d<double>> cdt_points(const std::vector<point> &pts,
                                         const std::vector<int> &which) {
  std::vector<CDT::V2d<double>> v;
  v.reserve(which.size());
  for (int i : which)
    v.push_back(CDT::V2d<double>(double(pts[i].x), double(pts[i].y)));
  return v;
}

triangulation run(const char *fn, const std::vector<point> &pts,
                  const std::vector<std::array<int, 2>> &edges, bool erase_outside) {
  for (const point &p : pts)
    detail::check_range(fn, p);
  for (const auto &e : edges)
    if (e[0] < 0 || e[1] < 0 || e[0] >= int(pts.size()) || e[1] >= int(pts.size()))
      throw planar_error(std::string(fn) + ": edge (" + std::to_string(e[0]) + ", " +
                         std::to_string(e[1]) + ") names a point that does not exist");
  triangulation out;
  out.vertices = pts;
  std::vector<int> first_of, unique_to_input, input_to_unique;
  dedupe(pts, first_of, unique_to_input, input_to_unique);
  if (unique_to_input.size() < 3)
    return out;
  cdt_t cdt(CDT::VertexInsertionOrder::Auto, CDT::IntersectingConstraintEdges::NotAllowed, 0.0);
  try {
    cdt.insertVertices(cdt_points(pts, unique_to_input));
    if (!edges.empty()) {
      std::vector<CDT::Edge> ce;
      ce.reserve(edges.size());
      for (const auto &e : edges) {
        const int a = input_to_unique[e[0]], b = input_to_unique[e[1]];
        if (a != b)
          ce.emplace_back(CDT::VertInd(a), CDT::VertInd(b));
      }
      cdt.insertEdges(ce);
    }
    if (erase_outside)
      cdt.eraseOuterTrianglesAndHoles();
    else
      cdt.eraseSuperTriangle();
  } catch (const CDT::IntersectingConstraintsError &) {
    throw planar_error(std::string(fn) + ": two constraint edges cross");
  } catch (const CDT::Error &e) {
    throw planar_error(std::string(fn) + ": " + e.what());
  }
  take_triangles(cdt, unique_to_input, pts, out);
  return out;
}

void append(triangulation &to, const triangulation &from) {
  const int base = int(to.vertices.size()), tbase = int(to.triangles.size());
  to.vertices.insert(to.vertices.end(), from.vertices.begin(), from.vertices.end());
  for (const auto &t : from.triangles)
    to.triangles.push_back({t[0] + base, t[1] + base, t[2] + base});
  for (const auto &n : from.neighbors)
    to.neighbors.push_back(
        {n[0] < 0 ? -1 : n[0] + tbase, n[1] < 0 ? -1 : n[1] + tbase, n[2] < 0 ? -1 : n[2] + tbase});
}

} // namespace

triangulation delaunay(const std::vector<point> &points) {
  return run("planar::delaunay", points, {}, false);
}

triangulation constrained_delaunay(const std::vector<point> &points,
                                   const std::vector<std::array<int, 2>> &edges) {
  return run("planar::constrained_delaunay", points, edges, false);
}

triangulation triangulate(const polygon &p) {
  detail::check_range("planar::triangulate", p);
  if (!is_simple(p))
    throw planar_error("planar::triangulate: the polygon is not simple (an edge crosses or touches "
                       "another, a vertex repeats, or a hole meets another ring)");
  std::vector<point> pts;
  std::vector<std::array<int, 2>> edges;
  auto add = [&](const ring &r) {
    const int base = int(pts.size()), n = int(r.size());
    pts.insert(pts.end(), r.begin(), r.end());
    for (int i = 0; i < n; ++i)
      edges.push_back({base + i, base + (i + 1) % n});
  };
  add(p.outer);
  for (const ring &h : p.holes)
    add(h);
  return run("planar::triangulate", pts, edges, true);
}

triangulation triangulate(const polygon_set &s) {
  triangulation out;
  for (const polygon &p : s)
    append(out, triangulate(p));
  return out;
}

std::vector<ring> convex_partition(const polygon &p) {
  const triangulation t = triangulate(p);
  const std::vector<point> &v = t.vertices;
  // Each piece a counterclockwise cycle of vertex indices; piece_of[] a union-find over triangles.
  std::vector<std::vector<int>> piece(t.triangles.size());
  std::vector<int> parent(t.triangles.size());
  for (std::size_t i = 0; i < t.triangles.size(); ++i) {
    piece[i] = {t.triangles[i][0], t.triangles[i][1], t.triangles[i][2]};
    parent[i] = int(i);
  }
  auto find = [&](int i) {
    while (parent[i] != i)
      i = parent[i] = parent[parent[i]];
    return i;
  };
  // Hertel-Mehlhorn: drop each diagonal whose removal leaves both its ends convex.
  for (int a = 0; a < int(t.triangles.size()); ++a)
    for (int k = 0; k < 3; ++k) {
      const int b = t.neighbors[a][k];
      if (b < a) // each diagonal once; -1 is a boundary edge
        continue;
      const int pa = find(a), pb = find(b);
      if (pa == pb)
        continue;
      // The diagonal u -> w runs counterclockwise in piece pa, so w -> u in piece pb.
      const int u = t.triangles[a][(k + 1) % 3], w = t.triangles[a][(k + 2) % 3];
      const std::vector<int> &P = piece[pa], &Q = piece[pb];
      const int np = int(P.size()), nq = int(Q.size());
      const int iu = int(std::find(P.begin(), P.end(), u) - P.begin());
      const int jw = int(std::find(Q.begin(), Q.end(), w) - Q.begin());
      if (iu == np || jw == nq || P[(iu + 1) % np] != w || Q[(jw + 1) % nq] != u)
        continue; // not a shared edge of the two pieces as they now are
      // At u: P's vertex before u, then Q's after u. At w: Q's before w, then P's after w.
      const point &u_prev = v[P[(iu + np - 1) % np]], &u_next = v[Q[(jw + 2) % nq]];
      const point &w_prev = v[Q[(jw + nq - 1) % nq]], &w_next = v[P[(iu + 2) % np]];
      if (orientation(u_prev, v[u], u_next) < 0 || orientation(w_prev, v[w], w_next) < 0)
        continue;
      // Merged: P from w round to u, then Q from u's successor round to w's predecessor.
      std::vector<int> m;
      m.reserve(std::size_t(np + nq - 2));
      for (int s = 0; s < np; ++s)
        m.push_back(P[(iu + 1 + s) % np]); // w ... u
      for (int s = 2; s < nq; ++s)
        m.push_back(Q[(jw + s) % nq]); // after u ... before w
      piece[pa] = std::move(m);
      piece[pb].clear();
      parent[pb] = pa;
    }
  std::vector<ring> out;
  for (std::size_t i = 0; i < piece.size(); ++i) {
    if (find(int(i)) != int(i) || piece[i].empty())
      continue;
    ring r;
    for (int k : piece[i])
      r.push_back(v[k]);
    out.push_back(std::move(r));
  }
  return out;
}

} // namespace planar
} // namespace cvc
