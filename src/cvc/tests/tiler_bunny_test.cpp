/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The contour tiler end to end, on the Stanford bunny (docs/roadmap/VOLROVER-LIBRARY-PORTS-
// ROADMAP.md §3). The steps:
//   1. the bunny's signed distance field (cvc::sdf, SDF_V2);
//   2. its z slices, from the ortho slicer (cvc::extract_slice);
//   3. each slice's zero isocontours (marching squares, below), as a cvc::contour_stack;
//   4. a surface through them (cvc::tile_contours);
//   5. that surface against the bunny mesh it came from: the distance from each side to the
//      other (the Hausdorff distance), in slice spacings.
//
// The bunny is scaled to span 100 units first: the tiler's tolerances are absolute (tiler.h), set
// for coordinates of order 1 to 10^4, and the built-in bunny spans about 0.15.
//
// Needs CVC_ENABLE_SDF (it is built only then) and cvc::planar (the tiler; it skips without it).
// Nothing here needs libigl: the distances come from the small search below.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/geometry/contours.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/geometry/tiler.h>
#include <cvc/utility/algorithm.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_slice.h>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

// The loops where a slice crosses `iso`, as x0 y0 x1 y1 ... in the slice plane's coordinates
// (right, up): marching squares on the grid of voxel centres. A value below `iso` is inside.
// - Each crossing is interpolated linearly along its grid edge. The two cells on either side of
//   that edge share it, so the segments join into closed loops when no inside value touches the
//   slice's border.
// - A saddle cell (inside at two opposite corners) is split by the mean of its four values: a mean
//   inside joins the two inside corners.
// - A value equal to `iso` counts as outside, a hair above it, so no crossing falls on a grid
//   point.
std::vector<std::vector<double>> isocontours(const cvc::volume_slice_data &s, double iso,
                                             double right0, double dright, double up0, double dup) {
  const std::int64_t w = static_cast<std::int64_t>(s.width);
  const std::int64_t h = static_cast<std::int64_t>(s.height);
  std::vector<double> v(s.values);
  double span = 0;
  for (double x : v)
    span = std::max(span, std::abs(x - iso));
  for (double &x : v)
    if (x == iso)
      x = iso + 1e-9 * std::max(span, 1.0);
  auto at = [&](std::int64_t c, std::int64_t r) { return v[r * w + c]; };
  auto inside = [&](std::int64_t c, std::int64_t r) { return at(c, r) < iso; };

  // A crossing's id: 2 * (r * w + c) for the edge from (c, r) to (c + 1, r), + 1 for the edge from
  // (c, r) to (c, r + 1).
  std::unordered_map<std::int64_t, std::array<double, 2>> point;
  auto crossing = [&](std::int64_t c0, std::int64_t r0, std::int64_t c1, std::int64_t r1) {
    const std::int64_t id = 2 * (r0 * w + c0) + (r1 != r0 ? 1 : 0);
    if (!point.count(id)) {
      const double a = at(c0, r0), b = at(c1, r1), t = (iso - a) / (b - a);
      point[id] = {right0 + (c0 + t * (c1 - c0)) * dright, up0 + (r0 + t * (r1 - r0)) * dup};
    }
    return id;
  };

  std::unordered_map<std::int64_t, std::vector<std::int64_t>> next; // crossing -> its neighbours
  auto segment = [&](std::int64_t a, std::int64_t b) {
    next[a].push_back(b);
    next[b].push_back(a);
  };
  for (std::int64_t r = 0; r + 1 < h; ++r)
    for (std::int64_t c = 0; c + 1 < w; ++c) {
      const bool i00 = inside(c, r), i10 = inside(c + 1, r), i11 = inside(c + 1, r + 1),
                 i01 = inside(c, r + 1);
      const int code = (i00 ? 1 : 0) | (i10 ? 2 : 0) | (i11 ? 4 : 0) | (i01 ? 8 : 0);
      if (code == 0 || code == 15)
        continue;
      auto bottom = [&] { return crossing(c, r, c + 1, r); };
      auto right = [&] { return crossing(c + 1, r, c + 1, r + 1); };
      auto top = [&] { return crossing(c, r + 1, c + 1, r + 1); };
      auto left = [&] { return crossing(c, r, c, r + 1); };
      const bool centre = (at(c, r) + at(c + 1, r) + at(c + 1, r + 1) + at(c, r + 1)) / 4 < iso;
      switch (code) {
      case 1:
      case 14:
        segment(left(), bottom());
        break;
      case 2:
      case 13:
        segment(bottom(), right());
        break;
      case 3:
      case 12:
        segment(left(), right());
        break;
      case 4:
      case 11:
        segment(right(), top());
        break;
      case 6:
      case 9:
        segment(bottom(), top());
        break;
      case 7:
      case 8:
        segment(left(), top());
        break;
      case 5: // inside at (c, r) and (c + 1, r + 1)
        if (centre) {
          segment(left(), top());
          segment(bottom(), right());
        } else {
          segment(left(), bottom());
          segment(right(), top());
        }
        break;
      case 10: // inside at (c + 1, r) and (c, r + 1)
        if (centre) {
          segment(left(), bottom());
          segment(right(), top());
        } else {
          segment(left(), top());
          segment(bottom(), right());
        }
        break;
      }
    }

  std::vector<std::vector<double>> loops;
  std::map<std::int64_t, bool> used; // ordered, so the loops come out in one order everywhere
  for (const auto &kv : next)
    used[kv.first] = false;
  for (auto &[start, done] : used) {
    if (done)
      continue;
    std::vector<double> xy;
    std::int64_t prev = -1, cur = start;
    do {
      used[cur] = true;
      xy.push_back(point[cur][0]);
      xy.push_back(point[cur][1]);
      const std::vector<std::int64_t> &n = next[cur];
      EXPECT_EQ(n.size(), 2u) << "a crossing with " << n.size() << " neighbours: the loop is open";
      if (n.size() != 2)
        break;
      const std::int64_t to = n[0] != prev ? n[0] : n[1];
      prev = cur;
      cur = to;
    } while (cur != start);
    if (xy.size() >= 6)
      loops.push_back(std::move(xy));
  }
  return loops;
}

using P3 = std::array<double, 3>;
P3 sub(const P3 &a, const P3 &b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
double dot(const P3 &a, const P3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// The distance from p to the triangle abc (Ericson's closest point, as tiler_test's).
double point_triangle(const P3 &p, const P3 &a, const P3 &b, const P3 &c) {
  auto at = [](const P3 &o, const P3 &u, double s, const P3 &v, double t) {
    return P3{o[0] + s * u[0] + t * v[0], o[1] + s * u[1] + t * v[1], o[2] + s * u[2] + t * v[2]};
  };
  const P3 ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
  const double d1 = dot(ab, ap), d2 = dot(ac, ap);
  P3 q;
  if (d1 <= 0 && d2 <= 0) {
    q = a;
  } else {
    const P3 bp = sub(p, b), cp = sub(p, c);
    const double d3 = dot(ab, bp), d4 = dot(ac, bp), d5 = dot(ab, cp), d6 = dot(ac, cp);
    const double vc = d1 * d4 - d3 * d2, vb = d5 * d2 - d1 * d6, va = d3 * d6 - d5 * d4;
    if (d3 >= 0 && d4 <= d3)
      q = b;
    else if (vc <= 0 && d1 >= 0 && d3 <= 0)
      q = at(a, ab, d1 / (d1 - d3), ac, 0);
    else if (d6 >= 0 && d5 <= d6)
      q = c;
    else if (vb <= 0 && d2 >= 0 && d6 <= 0)
      q = at(a, ab, 0, ac, d2 / (d2 - d6));
    else if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
      q = at(b, sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6)), ac, 0);
    else {
      const double den = 1 / (va + vb + vc);
      q = at(a, ab, vb * den, ac, vc * den);
    }
  }
  const P3 d = sub(p, q);
  return std::sqrt(dot(d, d));
}

// A surface's triangles bucketed in cubes of side `cell` (each in every cube its bounding box
// touches), for the distance from a point to the surface: the cubes are searched in rings around
// the point's until the nearest triangle found is nearer than any cube not yet searched.
class surface_grid {
public:
  surface_grid(const cvc::geometry &g, double cell) : _cell(cell) {
    for (const auto &p : g.points())
      _p.push_back({p[0], p[1], p[2]});
    for (const auto &t : g.tris())
      _t.push_back({std::size_t(t[0]), std::size_t(t[1]), std::size_t(t[2])});
    _lo = _p.empty() ? P3{0, 0, 0} : _p[0];
    P3 hi = _lo;
    for (const P3 &p : _p)
      for (int a = 0; a < 3; ++a) {
        _lo[a] = std::min(_lo[a], p[a]);
        hi[a] = std::max(hi[a], p[a]);
      }
    for (int a = 0; a < 3; ++a)
      _n[a] = std::max<std::int64_t>(1, std::int64_t((hi[a] - _lo[a]) / cell) + 1);
    _cells.resize(std::size_t(_n[0] * _n[1] * _n[2]));
    for (std::size_t i = 0; i < _t.size(); ++i) {
      std::int64_t c0[3], c1[3];
      for (int a = 0; a < 3; ++a) {
        const double lo = std::min({_p[_t[i][0]][a], _p[_t[i][1]][a], _p[_t[i][2]][a]});
        const double up = std::max({_p[_t[i][0]][a], _p[_t[i][1]][a], _p[_t[i][2]][a]});
        c0[a] = clamp(a, std::int64_t((lo - _lo[a]) / cell));
        c1[a] = clamp(a, std::int64_t((up - _lo[a]) / cell));
      }
      for (std::int64_t z = c0[2]; z <= c1[2]; ++z)
        for (std::int64_t y = c0[1]; y <= c1[1]; ++y)
          for (std::int64_t x = c0[0]; x <= c1[0]; ++x)
            _cells[std::size_t((z * _n[1] + y) * _n[0] + x)].push_back(i);
    }
  }

  double distance(const P3 &p) const {
    std::int64_t c[3];
    for (int a = 0; a < 3; ++a)
      c[a] = clamp(a, std::int64_t(std::floor((p[a] - _lo[a]) / _cell)));
    // How far p lies outside the grid: every cube is at least that far.
    double outside = 0;
    for (int a = 0; a < 3; ++a) {
      const double lo = _lo[a], up = _lo[a] + _n[a] * _cell;
      outside = std::max(outside, p[a] < lo ? lo - p[a] : (p[a] > up ? p[a] - up : 0.0));
    }
    double best = std::numeric_limits<double>::infinity();
    const std::int64_t rings = std::max({_n[0], _n[1], _n[2]});
    for (std::int64_t r = 0; r <= rings; ++r) {
      for (std::int64_t z = c[2] - r; z <= c[2] + r; ++z)
        for (std::int64_t y = c[1] - r; y <= c[1] + r; ++y)
          for (std::int64_t x = c[0] - r; x <= c[0] + r; ++x) {
            if (std::max({std::abs(x - c[0]), std::abs(y - c[1]), std::abs(z - c[2])}) != r)
              continue; // the ring's shell only
            if (x < 0 || y < 0 || z < 0 || x >= _n[0] || y >= _n[1] || z >= _n[2])
              continue;
            for (std::size_t i : _cells[std::size_t((z * _n[1] + y) * _n[0] + x)])
              best = std::min(best, point_triangle(p, _p[_t[i][0]], _p[_t[i][1]], _p[_t[i][2]]));
          }
      // Every cube not yet searched is at least r cubes away from p's.
      if (best <= outside + r * _cell)
        break;
    }
    return best;
  }

private:
  std::int64_t clamp(int a, std::int64_t i) const {
    return std::clamp<std::int64_t>(i, 0, _n[a] - 1);
  }

  double _cell;
  std::vector<P3> _p;
  std::vector<std::array<std::size_t, 3>> _t;
  P3 _lo{};
  std::int64_t _n[3] = {1, 1, 1};
  std::vector<std::vector<std::size_t>> _cells;
};

// The distance from `from`'s points (its vertices, and each triangle's centroid) to the surface
// `to`: the largest, the mean, and the largest over the points with y at least `y_cut`.
struct one_side {
  double max = 0, mean = 0, max_above = 0;
};
one_side distance_to(const cvc::geometry &from, const cvc::geometry &to, double cell,
                     double y_cut) {
  const surface_grid grid(to, cell);
  std::vector<P3> q;
  for (const auto &p : from.points())
    q.push_back({p[0], p[1], p[2]});
  for (const auto &t : from.tris()) {
    P3 c;
    for (int a = 0; a < 3; ++a)
      c[a] = (from.points()[t[0]][a] + from.points()[t[1]][a] + from.points()[t[2]][a]) / 3;
    q.push_back(c);
  }
  one_side d;
  for (const P3 &p : q) {
    const double s = grid.distance(p);
    d.max = std::max(d.max, s);
    if (p[1] >= y_cut)
      d.max_above = std::max(d.max_above, s);
    d.mean += s;
  }
  d.mean /= double(q.size());
  return d;
}

// About 13 s in Release (mostly the SDF; the tiling about 2.5 s since transfix/libcvc#1355), so it
// runs on every PR. Its name stays clear of ci.yml's heavy suites (STRESS_REGEX), and its ctest
// timeout is 900 s for Debug jobs (tests/CMakeLists.txt).
TEST(TilerBunny, TheTiledSlicesOfItsSdfAreTheBunny) {
#ifndef CVC_WITH_PLANAR
  GTEST_SKIP() << "libcvc was built without cvc::planar (the tiler)";
#endif
  cvc::app app;

  // The bunny, scaled to span 100 units, its lowest corner at the origin (y is up: the base is at
  // y = 0, the ears' tips near y = 99).
  cvc::geometry bunny = cvc::read_geometry(app, "test.bunny");
  ASSERT_GT(bunny.num_tris(), 0u);
  const cvc::point_t lo = bunny.min_point(), hi = bunny.max_point();
  const double extent = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});
  const double scale = 100.0 / extent;
  for (auto &p : bunny.points())
    for (int a = 0; a < 3; ++a)
      p[a] = (p[a] - lo[a]) * scale;
  const cvc::point_t blo = bunny.min_point();

  // Its signed distance on an n^3 grid of spacing h, with `pad` voxels of outside around the
  // bunny's 100 units, so every isocontour closes inside its slice. SDF_V1, because the bunny
  // (Stanford's bun_zipper) has holes in its base: SDF_V2's sign leaks through them, and the
  // inside reaches the grid's border. At n = 48 SDF_V1 gives spurious contours (167 in place of
  // 95); n = 72 is clean.
  const int n = 72, pad = 3;
  const double h = 100.0 / (n - 1 - 2 * pad);
  const double x0 = blo[0] - pad * h, y0 = blo[1] - pad * h, z0 = blo[2] - pad * h;
  const double span = (n - 1) * h;
  const cvc::bounding_box box(x0, y0, z0, x0 + span, y0 + span, z0 + span);
  const cvc::volume sdf = cvc::sdf(app, bunny, cvc::dimension(n, n, n), box, cvc::SDF_V1);
  const cvc::slice_grid grid = cvc::grid_of(sdf);
  ASSERT_NEAR(grid.spacing(0), h, 1e-9 * h);
  ASSERT_NEAR(grid.spacing(2), h, 1e-9 * h);

  // The ortho slicer's z slices; each slice's zero isocontours are polygons of the stack's one
  // component.
  cvc::contour_stack stack;
  stack.z_origin = grid.lo(2);
  stack.z_spacing = grid.spacing(2);
  std::size_t loops = 0, max_loops = 0;
  for (std::uint64_t k = 0; k < grid.count(cvc::slice_axis::z); ++k) {
    const cvc::volume_slice_data s = cvc::extract_slice(sdf, cvc::slice_axis::z, k);
    const auto contours =
        isocontours(s, 0.0, grid.lo(0), grid.spacing(0), grid.lo(1), grid.spacing(1));
    for (const auto &xy : contours)
      stack.add(static_cast<int>(k), "bunny", xy);
    loops += contours.size();
    max_loops = std::max(max_loops, contours.size());
  }
  ASSERT_GT(stack.slices.size(), 40u) << "the bunny should cross most slices";
  // The bunny's slices are contiguous: a missing slice would close the surface there.
  EXPECT_EQ(std::size_t(stack.slices.rbegin()->first - stack.slices.begin()->first + 1),
            stack.slices.size());

  // No curation: the contours are level sets of one field, so no two of a slice cross, and
  // curating them doubles the tiling's time for the same surface.
  cvc::tile_params params;
  params.curation_delta = 0;
  const cvc::tile_result r = cvc::tile_contours(stack, params);
  ASSERT_EQ(r.components.count("bunny"), 1u);
  const cvc::geometry &tiled = r.components.at("bunny");
  ASSERT_GT(tiled.num_tris(), 0u);

  // Both ways. The base band is the bottom 3 h, where the bunny's holes are: the tiled surface is
  // closed and caps them, and a cap's middle is far from the hole's rim.
  const double base = blo[1] + 3 * h;
  const one_side to_bunny = distance_to(tiled, bunny, 2 * h, base);
  const one_side to_tiled = distance_to(bunny, tiled, 2 * h, base);
  const double hausdorff = std::max(to_bunny.max, to_tiled.max);
  std::cout << "[ bunny ] " << stack.slices.size() << " slices, " << loops << " contours (at most "
            << max_loops << " in a slice), " << stack.vertex_count() << " contour vertices; "
            << tiled.num_tris() << " triangles, " << r.warnings.size() << " warnings\n"
            << "[ bunny ] in slice spacings (h = " << h << "): tiled to bunny max "
            << to_bunny.max / h << " (above the base band " << to_bunny.max_above / h << "), mean "
            << to_bunny.mean / h << "; bunny to tiled max " << to_tiled.max / h << ", mean "
            << to_tiled.mean / h << "; Hausdorff " << hausdorff / h << "\n";
  for (const std::string &w : r.warnings)
    std::cout << "[ bunny ] tiler warning: " << w.substr(0, 160) << "\n";

  // Tolerances in slice spacings h, about 1.4 times what Windows (MSVC, Release) measured:
  // - every point of the bunny lies within 2.5 h of the tiled surface (measured 1.78 h, near a
  //   hole's rim in the base);
  // - above the base band, every point of the tiled surface lies within 2 h of the bunny
  //   (measured 1.38 h);
  // - the Hausdorff distance, the base's hole caps included, is under 5 h (measured 3.54 h);
  // - on average, either surface lies within 0.4 h of the other (measured 0.28 h and 0.25 h).
  EXPECT_LT(to_tiled.max, 2.5 * h);
  EXPECT_LT(to_bunny.max_above, 2 * h);
  EXPECT_LT(hausdorff, 5 * h);
  EXPECT_LT(to_bunny.mean, 0.4 * h);
  EXPECT_LT(to_tiled.mean, 0.4 * h);
}

} // namespace
