/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::planar (cvc/geometry/planar.h), the ContourTiler port's planar geometry
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §3, P3a): T2, each piece against brute force.
//  - the grid: snapping, exact world positions, the range;
//  - the predicates against exact integer arithmetic, and at the edge of the range, where doubles
//    are not exact;
//  - rings and polygons: simplicity, location (against winding angles), clean-up, nesting;
//  - Booleans: every sample point classified as the operation says, and exact areas on
//    orthogonal polygons;
//  - offsets: a convex polygon's mitred offset against half-plane clipping, a rounded offset
//    against point distances, and the wavefront offset against the straight skeleton's known shape
//    past an edge event;
//  - Minkowski sums;
//  - triangulations: empty circumcircles (hidden ones only behind a constraint), exact areas,
//    the neighbour table;
//  - convex partitions: convex pieces that tile the polygon, within Hertel-Mehlhorn's bound;
//  - Sibson coordinates against Voronoi cells built by half-plane clipping;
//  - the medial axis: known shapes, and on random polygons every vertex's clearance, its two
//    nearest boundary points and the graph's topology, by brute force;
//  - two threads giving what each gives alone (T6).
// Runs natively and under node (the ports lane). Without Clipper2, CDT and Shewchuk's predicates
// (CVC_WITH_PLANAR undefined) it is one skipped test.

#include <gtest/gtest.h>

#ifndef CVC_WITH_PLANAR

TEST(Planar, NotBuilt) {
  GTEST_SKIP() << "libcvc was built without cvc::planar (Clipper2, CDT and Shewchuk's predicates)";
}

#else

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/geometry/planar.h>
#include <functional>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace cvc::planar;

const double kPi = 3.14159265358979323846;

// --- brute-force references -------------------------------------------------------------------

// Exact orientation for |coordinates| <= 2^20: the cross product fits an int64.
int brute_orientation(const point &a, const point &b, const point &c) {
  const std::int64_t v = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  return (v > 0) - (v < 0);
}

// Exact in-circle determinant for |coordinates| <= 2^10.
int brute_in_circle(const point &a, const point &b, const point &c, const point &d) {
  const std::int64_t adx = a.x - d.x, ady = a.y - d.y, bdx = b.x - d.x, bdy = b.y - d.y,
                     cdx = c.x - d.x, cdy = c.y - d.y;
  const std::int64_t al = adx * adx + ady * ady, bl = bdx * bdx + bdy * bdy,
                     cl = cdx * cdx + cdy * cdy;
  const std::int64_t det =
      adx * (bdy * cl - bl * cdy) - ady * (bdx * cl - bl * cdx) + al * (bdx * cdy - bdy * cdx);
  return (det > 0) - (det < 0);
}

// Do two closed segments meet? Solved with integer fractions (Cramer's rule), not orientations.
bool brute_segments_meet(point a, point b, point c, point d) {
  // A point on a segment: on its line (a zero cross product) and in its box.
  auto on = [](const point &p, const point &s0, const point &s1) {
    return (s1.x - s0.x) * (p.y - s0.y) - (s1.y - s0.y) * (p.x - s0.x) == 0 &&
           std::min(s0.x, s1.x) <= p.x && p.x <= std::max(s0.x, s1.x) &&
           std::min(s0.y, s1.y) <= p.y && p.y <= std::max(s0.y, s1.y);
  };
  if (a == b)
    return on(a, c, d);
  if (c == d)
    return on(c, a, b);
  const std::int64_t rx = b.x - a.x, ry = b.y - a.y, sx = d.x - c.x, sy = d.y - c.y;
  const std::int64_t den = rx * sy - ry * sx;
  const std::int64_t qx = c.x - a.x, qy = c.y - a.y;
  if (den != 0) {
    // a + t r = c + u s: t = (q x s) / den, u = (q x r) / den, both in [0, 1].
    std::int64_t tn = qx * sy - qy * sx, un = qx * ry - qy * rx, dd = den;
    if (dd < 0) {
      dd = -dd;
      tn = -tn;
      un = -un;
    }
    return tn >= 0 && tn <= dd && un >= 0 && un <= dd;
  }
  if (qx * ry - qy * rx != 0)
    return false; // parallel, on different lines
  // Collinear: the projections onto the longer axis overlap.
  auto proj = [&](const point &p) {
    return (std::llabs(rx) + std::llabs(sx) >= std::llabs(ry) + std::llabs(sy)) ? p.x : p.y;
  };
  std::int64_t a0 = proj(a), a1 = proj(b), c0 = proj(c), c1 = proj(d);
  if (a0 > a1)
    std::swap(a0, a1);
  if (c0 > c1)
    std::swap(c0, c1);
  return std::max(a0, c0) <= std::min(a1, c1);
}

// The winding number by summing angles: independent of the crossing rule locate() uses.
int brute_winding(const pointd &q, const ring &r) {
  double total = 0;
  for (std::size_t i = 0; i < r.size(); ++i) {
    const point &a = r[i], &b = r[(i + 1) % r.size()];
    const double t0 = std::atan2(double(a.y) - q.y, double(a.x) - q.x);
    const double t1 = std::atan2(double(b.y) - q.y, double(b.x) - q.x);
    double d = t1 - t0;
    while (d > kPi)
      d -= 2 * kPi;
    while (d < -kPi)
      d += 2 * kPi;
    total += d;
  }
  return int(std::lround(total / (2 * kPi)));
}

double seg_dist(const pointd &q, const point &a, const point &b) {
  const double ax = double(a.x), ay = double(a.y), dx = double(b.x) - ax, dy = double(b.y) - ay;
  const double l2 = dx * dx + dy * dy;
  double t = l2 > 0 ? ((q.x - ax) * dx + (q.y - ay) * dy) / l2 : 0;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(q.x - (ax + t * dx), q.y - (ay + t * dy));
}

double boundary_dist(const pointd &q, const ring &r) {
  double d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < r.size(); ++i)
    d = std::min(d, seg_dist(q, r[i], r[(i + 1) % r.size()]));
  return d;
}

double boundary_dist(const pointd &q, const polygon_set &s) {
  double d = std::numeric_limits<double>::infinity();
  for (const polygon &p : s) {
    d = std::min(d, boundary_dist(q, p.outer));
    for (const ring &h : p.holes)
      d = std::min(d, boundary_dist(q, h));
  }
  return d;
}

bool brute_inside(const pointd &q, const polygon_set &s) {
  for (const polygon &p : s) {
    if (brute_winding(q, p.outer) == 0)
      continue;
    bool in_hole = false;
    for (const ring &h : p.holes)
      in_hole = in_hole || brute_winding(q, h) != 0;
    if (!in_hole)
      return true;
  }
  return false;
}

// Sutherland-Hodgman: the convex polygon (doubles, counterclockwise) cut by the half-plane left of
// the directed line through a and b.
std::vector<pointd> clip_left(const std::vector<pointd> &poly, pointd a, pointd b) {
  std::vector<pointd> out;
  auto side = [&](const pointd &p) {
    return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
  };
  for (std::size_t i = 0; i < poly.size(); ++i) {
    const pointd &p = poly[i], &q = poly[(i + 1) % poly.size()];
    const double sp = side(p), sq = side(q);
    if (sp >= 0)
      out.push_back(p);
    if ((sp >= 0) != (sq >= 0)) {
      const double t = sp / (sp - sq);
      out.push_back(pointd(p.x + t * (q.x - p.x), p.y + t * (q.y - p.y)));
    }
  }
  return out;
}

double area_of(const std::vector<pointd> &poly) {
  double s = 0;
  for (std::size_t i = 0; i < poly.size(); ++i) {
    const pointd &p = poly[i], &q = poly[(i + 1) % poly.size()];
    s += p.x * q.y - q.x * p.y;
  }
  return s / 2;
}

// --- inputs -------------------------------------------------------------------------------------

// A star-shaped ring (counterclockwise): k vertices at increasing angles, random radii.
ring star(std::mt19937 &g, double cx, double cy, double rad, int k, double rmin = 0.35) {
  std::uniform_real_distribution<double> jitter(0, 0.8), radius(rmin, 1.0);
  ring r;
  for (int i = 0; i < k; ++i) {
    const double t = 2 * kPi * (i + jitter(g)) / k, q = radius(g) * rad;
    r.emplace_back(coord(std::llround(cx + q * std::cos(t))),
                   coord(std::llround(cy + q * std::sin(t))));
  }
  return clean(r);
}

// A convex ring (counterclockwise): k points on a circle.
ring convex(std::mt19937 &g, double cx, double cy, double rad, int k) {
  std::uniform_real_distribution<double> jitter(0, 0.8);
  ring r;
  for (int i = 0; i < k; ++i) {
    const double t = 2 * kPi * (i + jitter(g)) / k;
    r.emplace_back(coord(std::llround(cx + rad * std::cos(t))),
                   coord(std::llround(cy + rad * std::sin(t))));
  }
  return clean(r);
}

// An orthogonal "skyline" ring (counterclockwise): every intersection with another is a grid point.
ring skyline(std::mt19937 &g, coord x0, coord width, coord y0, coord hmax, int n, bool transpose) {
  std::vector<coord> xs = {x0, x0 + width};
  std::uniform_int_distribution<coord> xd(x0 + 1, x0 + width - 1), hd(hmax / 5, hmax);
  while (int(xs.size()) < n + 1) {
    const coord x = xd(g);
    if (std::find(xs.begin(), xs.end(), x) == xs.end())
      xs.push_back(x);
  }
  std::sort(xs.begin(), xs.end());
  std::vector<coord> h(n);
  for (int i = 0; i < n; ++i)
    do
      h[i] = hd(g);
    while (i > 0 && h[i] == h[i - 1]);
  ring p = {{xs[0], y0}, {xs[n], y0}};
  for (int i = n - 1; i >= 0; --i) {
    p.emplace_back(xs[i + 1], y0 + h[i]);
    p.emplace_back(xs[i], y0 + h[i]);
  }
  if (transpose) {
    for (point &v : p)
      std::swap(v.x, v.y);
    std::reverse(p.begin(), p.end());
  }
  return clean(p);
}

ring square(coord x0, coord y0, coord s) {
  return {{x0, y0}, {x0 + s, y0}, {x0 + s, y0 + s}, {x0, y0 + s}};
}

polygon_set set_of(const ring &r) { return {polygon(r)}; }

// --- the grid -----------------------------------------------------------------------------------

TEST(PlanarGrid, FitPutsTheBoxWithinTheExtentWithAPowerOfTwoUnit) {
  struct box {
    double xmin, ymin, xmax, ymax;
  };
  for (const box &b : {box{0, 0, 1000, 10}, box{-3.25, 7.5, -1.0, 9.0},
                       box{1e6, 1e6, 1e6 + 1e-3, 1e6}, box{-12345.678, -1e-9, 98765.4321, 3.0}}) {
    const grid g = grid::fit(b.xmin, b.ymin, b.xmax, b.ymax);
    int e = 0;
    EXPECT_EQ(std::frexp(g.unit, &e), 0.5) << "unit " << g.unit << " is not a power of two";
    EXPECT_EQ(std::fmod(g.origin_x, g.unit), 0.0);
    for (double x : {b.xmin, b.xmax})
      for (double y : {b.ymin, b.ymax}) {
        const point p = g.snap(x, y);
        EXPECT_LE(std::llabs(p.x), fit_extent + 1);
        EXPECT_LE(std::llabs(p.y), fit_extent + 1);
        const pointd w = g.to_world(p);
        EXPECT_LE(std::fabs(w.x - x), g.unit / 2 * (1 + 1e-12));
        EXPECT_LE(std::fabs(w.y - y), g.unit / 2 * (1 + 1e-12));
        EXPECT_EQ(g.snap(w), p) << "a grid point's world position snaps back to it";
      }
    // The unit is the smallest that fits: half of it would not.
    const double half = std::max(b.xmax - b.xmin, b.ymax - b.ymin) / 2;
    EXPECT_GT(half / (g.unit / 2), double(fit_extent));
  }
}

TEST(PlanarGrid, WorldPositionsOfGridPointsAreExact) {
  const grid g = grid::fit(-0.37, 2.0, 15.1, 3.0);
  std::mt19937 rng(7);
  std::uniform_int_distribution<coord> c(-fit_extent, fit_extent);
  for (int i = 0; i < 1000; ++i) {
    const point p(c(rng), c(rng));
    const pointd w = g.to_world(p);
    // Exact: subtracting the origin and dividing by a power of two gives the integer back.
    EXPECT_EQ((w.x - g.origin_x) / g.unit, double(p.x));
    EXPECT_EQ((w.y - g.origin_y) / g.unit, double(p.y));
  }
}

TEST(PlanarGrid, ABoxOfOnePointGetsUnitOne) {
  const grid g = grid::fit(3.5, -2.0, 3.5, -2.0);
  EXPECT_EQ(g.unit, 1.0);
  // The origin is the box's centre rounded to a unit: (4, -2), so the point is half a unit off it.
  EXPECT_EQ(g.origin_x, 4.0);
  EXPECT_EQ(g.origin_y, -2.0);
  EXPECT_EQ(g.snap(3.5, -2.0), point(-1, 0));
  EXPECT_EQ(g.snap(4.0, -2.0), point(0, 0));
}

TEST(PlanarGrid, BadBoxesAndPointsOffTheRangeThrow) {
  EXPECT_THROW(grid::fit(0, 0, std::nan(""), 1), planar_error);
  EXPECT_THROW(grid::fit(0, 0, std::numeric_limits<double>::infinity(), 1), planar_error);
  EXPECT_THROW(grid::fit(1, 0, 0, 1), planar_error);
  const grid g = grid::fit(0, 0, 1, 1);
  EXPECT_THROW(g.snap(1e6, 0), planar_error); // 2^30 units per half-unit box: far off the range
  EXPECT_THROW(g.snap(std::nan(""), 0), planar_error);
  try {
    g.snap(1e6, 0);
  } catch (const planar_error &e) {
    EXPECT_NE(std::string(e.what()).find("outside the grid's range"), std::string::npos)
        << e.what();
  }
}

// --- predicates ---------------------------------------------------------------------------------

TEST(PlanarPredicates, OrientationMatchesExactIntegers) {
  std::mt19937 rng(11);
  std::uniform_int_distribution<coord> c(-(1 << 20), 1 << 20), s(-4, 4);
  for (int i = 0; i < 200000; ++i) {
    point a, b, cc;
    if (i % 2) {
      a = {c(rng), c(rng)};
      b = {c(rng), c(rng)};
      cc = {c(rng), c(rng)};
    } else { // small: many collinear triples
      a = {s(rng), s(rng)};
      b = {s(rng), s(rng)};
      cc = {s(rng), s(rng)};
    }
    ASSERT_EQ(orientation(a, b, cc), brute_orientation(a, b, cc)) << i;
    ASSERT_EQ(orientation(pointd(double(a.x), double(a.y)), pointd(double(b.x), double(b.y)),
                          pointd(double(cc.x), double(cc.y))),
              brute_orientation(a, b, cc));
  }
}

TEST(PlanarPredicates, OrientationIsExactAtTheEdgeOfTheRange) {
  // Collinear points spanning the whole range, and the same nudged by one unit: the cross products
  // are near 2^63, far past what a double holds exactly.
  const point d(65537, 65521); // direction
  const point a(-coord_max + 5, -coord_max + 7);
  for (coord k : {coord(1), coord(1000), coord(32767)}) {
    for (coord j : {coord(-1), coord(3), coord(16000)}) {
      const point b(a.x + k * d.x, a.y + k * d.y);
      const point c(a.x + (k + j) * d.x, a.y + (k + j) * d.y);
      if (!(std::llabs(b.x) <= coord_max && std::llabs(c.x) <= coord_max &&
            std::llabs(b.y) <= coord_max && std::llabs(c.y) <= coord_max))
        continue;
      EXPECT_EQ(orientation(a, b, c), 0);
      // cross(b - a, c + (0, 1) - a) = k d.x: positive.
      EXPECT_EQ(orientation(a, b, point(c.x, c.y + 1)), 1);
      EXPECT_EQ(orientation(a, b, point(c.x, c.y - 1)), -1);
      // cross(c + (1, 0) - a, b - a) = k d.y: positive.
      EXPECT_EQ(orientation(a, point(c.x + 1, c.y), b), 1);
    }
  }
}

TEST(PlanarPredicates, InCircleMatchesExactIntegers) {
  std::mt19937 rng(13);
  std::uniform_int_distribution<coord> c(-(1 << 10), 1 << 10), s(-3, 3);
  for (int i = 0; i < 200000; ++i) {
    std::uniform_int_distribution<coord> &d = (i % 2) ? c : s;
    const point a(d(rng), d(rng)), b(d(rng), d(rng)), cc(d(rng), d(rng)), dd(d(rng), d(rng));
    ASSERT_EQ(in_circle(a, b, cc, dd), brute_in_circle(a, b, cc, dd)) << i;
  }
  // Cocircular at the edge of the range: the four corners of a big square.
  const coord m = coord_max - 1;
  EXPECT_EQ(in_circle(point(-m, -m), point(m, -m), point(m, m), point(-m, m)), 0);
  EXPECT_EQ(in_circle(point(-m, -m), point(m, -m), point(m, m), point(-m + 1, m)), 1);
  EXPECT_EQ(in_circle(point(-m, -m), point(m, -m), point(m, m), point(-m, m + 1)), -1);
}

TEST(PlanarPredicates, SegmentIntersectionMatchesIntegerFractions) {
  std::mt19937 rng(17);
  std::uniform_int_distribution<coord> c(0, 6);
  for (int i = 0; i < 200000; ++i) {
    const point a(c(rng), c(rng)), b(c(rng), c(rng)), cc(c(rng), c(rng)), d(c(rng), c(rng));
    ASSERT_EQ(segments_intersect(a, b, cc, d), brute_segments_meet(a, b, cc, d))
        << "(" << a.x << "," << a.y << ")-(" << b.x << "," << b.y << ") vs (" << cc.x << "," << cc.y
        << ")-(" << d.x << "," << d.y << ")";
  }
}

// --- rings and polygons -------------------------------------------------------------------------

TEST(PlanarRings, AreaOrientationAndPerimeter) {
  ring r = square(0, 0, 10);
  EXPECT_EQ(signed_area(r), 100.0);
  EXPECT_EQ(orientation(r), 1);
  EXPECT_EQ(perimeter(r), 40.0);
  std::reverse(r.begin(), r.end());
  EXPECT_EQ(signed_area(r), -100.0);
  EXPECT_EQ(orientation(r), -1);
  const polygon p(square(0, 0, 10), {square(2, 2, 3)});
  EXPECT_EQ(area(p), 91.0);
  // Areas near the edge of the range stay accurate.
  const coord m = coord_max;
  EXPECT_DOUBLE_EQ(signed_area(ring{{-m, -m}, {m, -m}, {m, m}, {-m, m}}),
                   4.0 * double(m) * double(m));
}

TEST(PlanarRings, Simplicity) {
  EXPECT_TRUE(is_simple(square(0, 0, 10)));
  EXPECT_TRUE(is_simple(ring{{0, 0}, {5, 0}, {10, 0}, {10, 10}, {0, 10}}));   // a collinear vertex
  EXPECT_FALSE(is_simple(ring{{0, 0}, {10, 10}, {10, 0}, {0, 10}}));          // a bow tie
  EXPECT_FALSE(is_simple(ring{{0, 0}, {10, 0}, {10, 10}, {10, 0}, {0, 10}})); // repeated vertex
  EXPECT_FALSE(is_simple(ring{{0, 0}, {10, 0}, {20, 0}, {15, 0}, {15, 10}})); // a spike
  EXPECT_FALSE(is_simple(ring{{0, 0}, {10, 0}, {5, 0}}));                     // no area
  EXPECT_FALSE(is_simple(ring{{0, 0}, {10, 0}, {10, 10}, {5, 0}, {0, 10}}));  // a vertex on an edge
  EXPECT_FALSE(is_simple(ring{{0, 0}, {10, 0}}));
  std::mt19937 rng(19);
  for (int i = 0; i < 200; ++i)
    EXPECT_TRUE(is_simple(star(rng, 0, 0, 1e6, 5 + i % 60)));
  const polygon ok(square(0, 0, 10), {square(2, 2, 3), square(6, 6, 2)});
  EXPECT_TRUE(is_simple(ok));
  const polygon touching(square(0, 0, 10), {square(0, 2, 3)}); // a hole on the outer boundary
  EXPECT_FALSE(is_simple(touching));
}

TEST(PlanarRings, LocationMatchesWindingAngles) {
  std::mt19937 rng(23);
  for (int k = 0; k < 40; ++k) {
    const ring r = star(rng, 0, 0, 1000, 5 + k);
    std::uniform_int_distribution<coord> c(-1100, 1100);
    for (int i = 0; i < 500; ++i) {
      const point q(c(rng), c(rng));
      const location l = locate(q, r);
      bool on = false;
      for (std::size_t j = 0; j < r.size(); ++j)
        on = on || brute_segments_meet(q, q, r[j], r[(j + 1) % r.size()]);
      if (on) {
        EXPECT_EQ(l, location::boundary);
        continue;
      }
      const bool inside = brute_winding(pointd(double(q.x), double(q.y)), r) != 0;
      ASSERT_EQ(l, inside ? location::inside : location::outside);
    }
    for (const point &v : r)
      EXPECT_EQ(locate(v, r), location::boundary);
  }
  const polygon p(square(0, 0, 10), {square(2, 2, 3)});
  EXPECT_EQ(locate(point(1, 1), p), location::inside);
  EXPECT_EQ(locate(point(3, 3), p), location::outside);
  EXPECT_EQ(locate(point(2, 3), p), location::boundary);
  EXPECT_EQ(locate(point(11, 3), p), location::outside);
}

TEST(PlanarRings, CleanDropsRepeatsCollinearVerticesAndSpikes) {
  EXPECT_EQ(clean(ring{{0, 0}, {0, 0}, {5, 0}, {10, 0}, {10, 10}, {10, 10}, {0, 10}, {0, 0}}),
            (ring{{0, 0}, {10, 0}, {10, 10}, {0, 10}}));
  // A spike out of the top edge: it and the vertex it leaves behind go.
  const ring c = clean(ring{{0, 0}, {10, 0}, {10, 10}, {5, 10}, {5, 20}, {5, 10}, {0, 10}});
  EXPECT_EQ(c.size(), 4u);
  EXPECT_EQ(signed_area(c), 100.0);
  EXPECT_LT(clean(ring{{0, 0}, {5, 0}, {10, 0}}).size(), 3u);
  const polygon n = normalize(polygon(ring{{0, 10}, {10, 10}, {10, 0}, {0, 0}}, {square(2, 2, 3)}));
  EXPECT_EQ(orientation(n.outer), 1);
  EXPECT_EQ(orientation(n.holes[0]), -1);
}

TEST(PlanarRings, NestArrangesByDepth) {
  // Two squares side by side; the first with a hole holding an island holding a hole.
  const std::vector<ring> rings = {square(4, 4, 2),
                                   square(0, 0, 10),
                                   square(20, 0, 5),
                                   square(2, 2, 6),
                                   square(5, 5, 0) /* degenerate */,
                                   ring{{3, 3}, {7, 3}, {7, 7}, {3, 7}}};
  // rings[4] is a point and is dropped; rings[3] (2..8) holds rings[5] (3..7) holds rings[0]
  // (4..6).
  const polygon_set s = nest(rings);
  ASSERT_EQ(s.size(), 3u);
  // Outer rings in input order: rings[1], rings[2], rings[5].
  EXPECT_EQ(area(s[0]), 100.0 - 36.0);
  EXPECT_EQ(area(s[1]), 25.0);
  EXPECT_EQ(area(s[2]), 16.0 - 4.0);
  for (const polygon &p : s) {
    EXPECT_EQ(orientation(p.outer), 1);
    for (const ring &h : p.holes)
      EXPECT_EQ(orientation(h), -1);
  }
  EXPECT_THROW(nest({square(0, 0, 10), square(5, 5, 10)}), planar_error);  // crossing
  EXPECT_THROW(nest({square(0, 0, 10), square(10, 0, 10)}), planar_error); // touching
  EXPECT_THROW(nest({ring{{0, 0}, {10, 10}, {10, 0}, {0, 10}}}), planar_error);
}

TEST(PlanarRings, CoordinatesOffTheRangeThrow) {
  const ring big = {{0, 0}, {coord_max + 1, 0}, {0, 10}};
  EXPECT_THROW(nest({big}), planar_error);
  EXPECT_THROW(unite(set_of(big), {}), planar_error);
  EXPECT_THROW(offset(set_of(big), 1), planar_error);
  EXPECT_THROW(wavefront_offset(set_of(big), -1), planar_error);
  // An offset that would leave the range.
  const coord m = coord_max - 10;
  EXPECT_THROW(offset(set_of(ring{{-m, -m}, {m, -m}, {m, m}, {-m, m}}), 100), planar_error);
  EXPECT_NO_THROW(offset(set_of(ring{{-m, -m}, {m, -m}, {m, m}, {-m, m}}), -100));
}

// --- Booleans -----------------------------------------------------------------------------------

using op_fn = std::function<polygon_set(const polygon_set &, const polygon_set &)>;
struct op_case {
  const char *name;
  op_fn f;
  std::function<bool(bool, bool)> rule;
};

std::vector<op_case> ops() {
  return {{"unite", unite, [](bool a, bool b) { return a || b; }},
          {"intersect", intersect, [](bool a, bool b) { return a && b; }},
          {"subtract", subtract, [](bool a, bool b) { return a && !b; }},
          {"exclusive_or", exclusive_or, [](bool a, bool b) { return a != b; }}};
}

// Every sample point not within `margin` of a boundary is in the result exactly when the operation
// says it should be.
void check_samples(const polygon_set &a, const polygon_set &b, const op_case &op, std::mt19937 &rng,
                   double lo, double hi, double margin) {
  const polygon_set r = op.f(a, b);
  std::uniform_real_distribution<double> c(lo, hi);
  int checked = 0;
  for (int i = 0; i < 2000; ++i) {
    const point q(coord(std::llround(c(rng))), coord(std::llround(c(rng))));
    const pointd qd(double(q.x), double(q.y));
    if (boundary_dist(qd, a) < margin || boundary_dist(qd, b) < margin ||
        boundary_dist(qd, r) < margin)
      continue;
    const bool want = op.rule(brute_inside(qd, a), brute_inside(qd, b));
    ASSERT_EQ(locate(q, r) == location::inside, want)
        << op.name << " at (" << q.x << ", " << q.y << ")";
    ++checked;
  }
  EXPECT_GT(checked, 1000) << op.name;
  for (const polygon &p : r) {
    EXPECT_EQ(orientation(p.outer), 1) << op.name;
    for (const ring &h : p.holes)
      EXPECT_EQ(orientation(h), -1) << op.name;
  }
}

TEST(PlanarBooleans, StarsClassifyEverySamplePoint) {
  std::mt19937 rng(29);
  for (int k = 0; k < 6; ++k) {
    const polygon_set a = set_of(star(rng, 0, 0, 1e5, 12 + 6 * k));
    const polygon_set b = set_of(star(rng, 3e4, 2e4, 9e4, 10 + 5 * k));
    for (const op_case &op : ops())
      check_samples(a, b, op, rng, -1.2e5, 1.4e5, 2.0);
  }
}

TEST(PlanarBooleans, PolygonsWithHolesClassifyEverySamplePoint) {
  std::mt19937 rng(31);
  const polygon_set a = {
      polygon(square(0, 0, 1000), {square(200, 200, 300), square(600, 100, 200)})};
  const polygon_set b = {polygon(square(400, 400, 1000), {square(500, 500, 200)}),
                         polygon(square(-300, 700, 200))};
  for (const op_case &op : ops())
    check_samples(a, b, op, rng, -400, 1500, 1.0);
}

TEST(PlanarBooleans, OrthogonalAreasAreExact) {
  std::mt19937 rng(37);
  for (int k = 0; k < 8; ++k) {
    const polygon_set a = set_of(skyline(rng, 0, 1000000, 0, 800000, 10 + 3 * k, false));
    const polygon_set b = set_of(skyline(rng, 100000, 900000, 50000, 900000, 8 + 2 * k, true));
    const double aa = area(a), ab = area(b);
    const double u = area(unite(a, b)), i = area(intersect(a, b));
    EXPECT_EQ(u + i, aa + ab);
    EXPECT_EQ(area(subtract(a, b)), aa - i);
    EXPECT_EQ(area(subtract(b, a)), ab - i);
    EXPECT_EQ(area(exclusive_or(a, b)), u - i);
  }
}

TEST(PlanarBooleans, ResolveAppliesTheFillRule) {
  // A bow tie: two triangles meeting at a point.
  const polygon_set bow = resolve({ring{{0, 0}, {10, 10}, {10, 0}, {0, 10}}});
  EXPECT_EQ(area(bow), 50.0);
  // Two overlapping squares: non-zero fills the overlap, even-odd leaves it out.
  const std::vector<ring> two = {square(0, 0, 10), square(5, 5, 10)};
  EXPECT_EQ(area(resolve(two, fill_rule::non_zero)), 175.0);
  EXPECT_EQ(area(resolve(two, fill_rule::even_odd)), 150.0);
  // A ring inside another, same orientation: positive counts it twice; non-zero once.
  const std::vector<ring> nested = {square(0, 0, 10), square(2, 2, 3)};
  EXPECT_EQ(area(resolve(nested, fill_rule::non_zero)), 100.0);
  EXPECT_EQ(area(resolve(nested, fill_rule::even_odd)), 91.0);
  EXPECT_EQ(area(resolve(nested, fill_rule::negative)), 0.0);
}

// --- offsets ------------------------------------------------------------------------------------

// A convex polygon's mitred offset: the polygon (grown, or not) cut by each edge's line moved by
// delta. Growing starts from a box around the grown polygon.
double brute_convex_offset_area(const ring &r, double delta) {
  std::vector<pointd> poly;
  if (delta < 0) {
    for (const point &v : r)
      poly.push_back(pointd(double(v.x), double(v.y)));
  } else {
    const double big = 1e12;
    poly = {pointd(-big, -big), pointd(big, -big), pointd(big, big), pointd(-big, big)};
  }
  for (std::size_t i = 0; i < r.size(); ++i) {
    const point &a = r[i], &b = r[(i + 1) % r.size()];
    const double dx = double(b.x - a.x), dy = double(b.y - a.y), l = std::hypot(dx, dy);
    // The left normal points inside a counterclockwise ring; move the line outward by delta.
    const double nx = -dy / l, ny = dx / l;
    const pointd a2(double(a.x) - delta * nx, double(a.y) - delta * ny);
    const pointd b2(double(b.x) - delta * nx, double(b.y) - delta * ny);
    poly = clip_left(poly, a2, b2);
    if (poly.size() < 3)
      return 0;
  }
  return area_of(poly);
}

TEST(PlanarOffsets, ConvexMitredOffsetsMatchHalfPlaneClipping) {
  std::mt19937 rng(41);
  offset_params mitre;
  mitre.miter_limit = 1e6;
  for (int k = 0; k < 20; ++k) {
    const ring r = convex(rng, 0, 0, 1e6, 6 + k);
    for (double d : {-4e5, -1e5, -1e3, 1e3, 1e5, 4e5}) {
      const double want = brute_convex_offset_area(r, d);
      const double tol =
          perimeter(r) + 2 * std::fabs(d) * double(r.size()); // a unit per unit of boundary
      EXPECT_NEAR(area(offset(set_of(r), d, mitre)), want, tol) << "k=" << k << " d=" << d;
      EXPECT_NEAR(area(wavefront_offset(set_of(r), d)), want, tol) << "k=" << k << " d=" << d;
    }
  }
}

TEST(PlanarOffsets, RoundOffsetsAreDistanceSets) {
  std::mt19937 rng(43);
  offset_params round;
  round.join = join_type::round;
  round.arc_tolerance = 0.25;
  for (int k = 0; k < 6; ++k) {
    const ring r = star(rng, 0, 0, 10000, 10 + 4 * k);
    const polygon_set s = set_of(r);
    for (double d : {800.0, -600.0}) {
      const polygon_set o = offset(s, d, round);
      std::uniform_real_distribution<double> c(-11500, 11500);
      int checked = 0;
      for (int i = 0; i < 3000; ++i) {
        const point q(coord(std::llround(c(rng))), coord(std::llround(c(rng))));
        const pointd qd(double(q.x), double(q.y));
        const bool in = brute_winding(qd, r) != 0;
        const double dist = boundary_dist(qd, r);
        // Signed distance to the ring: negative inside. Inside the offset iff it is below d.
        const double sd = in ? -dist : dist;
        if (std::fabs(sd - d) < 2.0)
          continue;
        ASSERT_EQ(locate(q, o) == location::inside, sd < d)
            << "d=" << d << " q=(" << q.x << "," << q.y << ")";
        ++checked;
      }
      EXPECT_GT(checked, 2000);
    }
  }
}

TEST(PlanarOffsets, WavefrontEqualsOnePassBeforeTheFirstEvent) {
  std::mt19937 rng(47);
  offset_params mitre;
  mitre.miter_limit = 1e6;
  for (int k = 0; k < 10; ++k) {
    // Few, long edges and a shallow offset: no edge collapses or meets another.
    const ring r = star(rng, 0, 0, 1e6, 8 + k, 0.7);
    for (double d : {-1e4, 1e4}) {
      const double a1 = area(offset(set_of(r), d, mitre)),
                   a2 = area(wavefront_offset(set_of(r), d));
      EXPECT_NEAR(a1, a2, perimeter(r)) << "k=" << k << " d=" << d;
    }
  }
}

// Past an edge event the straight skeleton drops the vanished edge and mitres its neighbours. So a
// polygon whose short edge collapses at t1 has, at any d > t1, the offset of the same polygon with
// that edge replaced by its neighbours' meeting point, which has no event up to d. (Here each
// collapse leaves a convex join, where a single mitred pass happens to agree too; where a collapse
// leaves a reflex join, the single pass bounds the region by the vanished edge's line instead of
// the mitre: tools/cgal_ab's INFO lines, against CGAL's straight skeleton.)
struct event_case {
  const char *name;
  ring original, without_edge;
  double d;
};

std::vector<event_case> event_cases() {
  std::vector<event_case> cs;
  // A big square with a narrowing tab on top: walls from (400, 1000) and (600, 1000) leaning in by
  // 0.5 per unit of height to (450, 1100) and (550, 1100), the tab's top edge 100 long. Shrinking,
  // the top edge's corners (116.6 degrees) each close at cot(58.3 degrees) = 0.618, so it vanishes
  // at d = 100 / 1.236 = 80.9. The walls meet at (500, 1200), the apex of the replacement, whose
  // tab is cut off by the square's top edge only at d = 200 / (2.236 - 1) = 161.8.
  cs.push_back({"shrinking tab",
                {{0, 0},
                 {1000, 0},
                 {1000, 1000},
                 {600, 1000},
                 {550, 1100},
                 {450, 1100},
                 {400, 1000},
                 {0, 1000}},
                {{0, 0}, {1000, 0}, {1000, 1000}, {600, 1000}, {500, 1200}, {400, 1000}, {0, 1000}},
                -120});
  // A notch into the top of a square, narrowing downward: walls from (300, 1000) and (700, 1000) to
  // (450, 700) and (550, 700), its bottom edge 100 long between two reflex corners. Growing, the
  // bottom edge vanishes at d = 80.9, as above; the walls meet at (500, 600), and the replacement's
  // notch fills only at d = 400 / 1.236 = 323.
  cs.push_back({"growing notch",
                {{0, 0},
                 {1000, 0},
                 {1000, 1000},
                 {700, 1000},
                 {550, 700},
                 {450, 700},
                 {300, 1000},
                 {0, 1000}},
                {{0, 0}, {1000, 0}, {1000, 1000}, {700, 1000}, {500, 600}, {300, 1000}, {0, 1000}},
                120});
  // Scaled up, so rounding is negligible.
  for (event_case &c : cs) {
    for (point &p : c.original)
      p = point(p.x * 1000, p.y * 1000);
    for (point &p : c.without_edge)
      p = point(p.x * 1000, p.y * 1000);
    c.d *= 1000;
  }
  return cs;
}

TEST(PlanarOffsets, WavefrontFollowsTheSkeletonPastAnEdgeEvent) {
  offset_params mitre;
  mitre.miter_limit = 1e6;
  for (const event_case &c : event_cases()) {
    const polygon_set want = offset(set_of(c.without_edge), c.d, mitre);
    const polygon_set got = wavefront_offset(set_of(c.original), c.d);
    const double tol = perimeter(c.original) * 2.0;
    EXPECT_NEAR(area(got), area(want), tol) << c.name;
    EXPECT_LT(area(exclusive_or(got, want)), tol)
        << c.name << ": the wavefront is the skeleton's shape";
  }
}

TEST(PlanarOffsets, WavefrontStepsCompose) {
  // The skeleton's offsets compose: d1 then d2 is d1 + d2 (up to rounding at each step).
  std::mt19937 rng(53);
  for (int k = 0; k < 6; ++k) {
    const ring r = star(rng, 0, 0, 1e6, 16 + 8 * k);
    for (double d : {-2e5, 1.5e5}) {
      const polygon_set once = wavefront_offset(set_of(r), d);
      const polygon_set twice = wavefront_offset(wavefront_offset(set_of(r), d / 2), d / 2);
      EXPECT_NEAR(area(once), area(twice), 4 * perimeter(r)) << "k=" << k << " d=" << d;
    }
  }
}

TEST(PlanarOffsets, HolesMoveTheOtherWay) {
  const polygon_set s = {polygon(square(0, 0, 1000), {square(400, 400, 200)})};
  offset_params mitre;
  // Growing by 50 shrinks the hole to 100 x 100.
  EXPECT_EQ(area(offset(s, 50, mitre)), 1100.0 * 1100.0 - 100.0 * 100.0);
  EXPECT_EQ(area(wavefront_offset(s, 50)), 1100.0 * 1100.0 - 100.0 * 100.0);
  // Shrinking by 50 grows it to 300 x 300.
  EXPECT_EQ(area(offset(s, -50, mitre)), 900.0 * 900.0 - 300.0 * 300.0);
  EXPECT_EQ(area(wavefront_offset(s, -50)), 900.0 * 900.0 - 300.0 * 300.0);
  // Growing by 100 or more fills it.
  EXPECT_EQ(area(wavefront_offset(s, 150)), 1300.0 * 1300.0);
  // Shrinking a 1000 x 200 rectangle by 100 leaves nothing.
  EXPECT_TRUE(
      wavefront_offset(set_of(ring{{0, 0}, {1000, 0}, {1000, 200}, {0, 200}}), -100).empty());
  // A shrink that splits a dumbbell in two.
  const ring dumbbell = {{0, 0},      {400, 0},   {400, 180}, {600, 180}, {600, 0},   {1000, 0},
                         {1000, 400}, {600, 400}, {600, 220}, {400, 220}, {400, 400}, {0, 400}};
  const polygon_set split = wavefront_offset(set_of(dumbbell), -50);
  ASSERT_EQ(split.size(), 2u);
  EXPECT_EQ(area(split), 2 * 300.0 * 300.0);
}

TEST(PlanarOffsets, BadParametersThrow) {
  const polygon_set s = set_of(square(0, 0, 10));
  EXPECT_THROW(offset(s, std::nan("")), planar_error);
  EXPECT_THROW(wavefront_offset(s, std::numeric_limits<double>::infinity()), planar_error);
  offset_params p;
  p.miter_limit = 0;
  EXPECT_THROW(offset(s, 1, p), planar_error);
}

// --- Minkowski sums -----------------------------------------------------------------------------

TEST(PlanarMinkowski, ConvexSumsHaveTheMixedArea) {
  // Squares: (a + b)^2. A square and a triangle: area(A) + area(B) + perimeter-weighted mixed area,
  // here checked through the vertices' sums' convex hull, computed by brute force.
  EXPECT_EQ(area(minkowski_sum(square(0, 0, 10), square(5, 5, 3))), 169.0);
  std::mt19937 rng(59);
  for (int k = 0; k < 10; ++k) {
    const ring a = convex(rng, 0, 0, 1e5, 5 + k), b = convex(rng, 1e4, -2e4, 3e4, 3 + k);
    std::vector<point> sums;
    for (const point &p : a)
      for (const point &q : b)
        sums.emplace_back(p.x + q.x, p.y + q.y);
    // Convex hull (Andrew's monotone chain) of the vertex sums.
    std::sort(sums.begin(), sums.end());
    sums.erase(std::unique(sums.begin(), sums.end()), sums.end());
    std::vector<point> hull(2 * sums.size());
    std::size_t h = 0;
    for (std::size_t i = 0; i < sums.size(); ++i) {
      while (h >= 2 && brute_orientation(hull[h - 2], hull[h - 1], sums[i]) <= 0)
        --h;
      hull[h++] = sums[i];
    }
    for (std::size_t i = sums.size() - 1, t = h + 1; i-- > 0;) {
      while (h >= t && brute_orientation(hull[h - 2], hull[h - 1], sums[i]) <= 0)
        --h;
      hull[h++] = sums[i];
    }
    hull.resize(h - 1);
    const polygon_set m = minkowski_sum(a, b);
    ASSERT_EQ(m.size(), 1u);
    EXPECT_TRUE(m[0].holes.empty());
    EXPECT_EQ(area(m), signed_area(hull)) << k;
  }
}

TEST(PlanarMinkowski, AnLShapeAndASquare) {
  // An L of three unit-10 squares, swept by a 4 x 4 square: an L of outer size 24 with its inner
  // corner filled to (14, 14).
  const ring l = {{0, 0}, {20, 0}, {20, 10}, {10, 10}, {10, 20}, {0, 20}};
  const polygon_set m = minkowski_sum(l, square(0, 0, 4));
  EXPECT_EQ(area(m), 24.0 * 24.0 - 10.0 * 10.0);
  EXPECT_EQ(locate(point(12, 12), m), location::inside);
  EXPECT_EQ(locate(point(15, 15), m), location::outside);
}

// --- triangulations -----------------------------------------------------------------------------

// Twice a triangle's area, exactly, for |coordinates| <= 2^20.
std::int64_t twice_area(const point &a, const point &b, const point &c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// Twice the convex hull's area (Andrew's monotone chain), exactly.
std::int64_t twice_hull_area(std::vector<point> p) {
  std::sort(p.begin(), p.end());
  p.erase(std::unique(p.begin(), p.end()), p.end());
  if (p.size() < 3)
    return 0;
  std::vector<point> h(2 * p.size());
  std::size_t k = 0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    while (k >= 2 && brute_orientation(h[k - 2], h[k - 1], p[i]) <= 0)
      --k;
    h[k++] = p[i];
  }
  for (std::size_t i = p.size() - 1, t = k + 1; i-- > 0;) {
    while (k >= t && brute_orientation(h[k - 2], h[k - 1], p[i]) <= 0)
      --k;
    h[k++] = p[i];
  }
  h.resize(k - 1);
  std::int64_t s = 0;
  for (std::size_t i = 1; i + 1 < h.size(); ++i)
    s += twice_area(h[0], h[i], h[i + 1]);
  return s;
}

std::vector<point> random_points(std::mt19937 &rng, int n, coord range) {
  std::uniform_int_distribution<coord> c(-range, range);
  std::vector<point> p(static_cast<std::size_t>(n));
  for (point &q : p)
    q = point(c(rng), c(rng));
  return p;
}

// Every triangle counterclockwise, the neighbour table consistent (each neighbour shares the
// opposite edge and points back), and no edge in more than two triangles.
void check_structure(const triangulation &t) {
  ASSERT_EQ(t.neighbors.size(), t.triangles.size());
  for (std::size_t i = 0; i < t.triangles.size(); ++i) {
    const auto &tri = t.triangles[i];
    ASSERT_EQ(orientation(t.vertices[tri[0]], t.vertices[tri[1]], t.vertices[tri[2]]), 1);
    for (int k = 0; k < 3; ++k) {
      const int n = t.neighbors[i][k];
      if (n < 0)
        continue;
      const int a = tri[(k + 1) % 3], b = tri[(k + 2) % 3];
      const auto &o = t.triangles[std::size_t(n)];
      int shared = 0, back = -1;
      for (int j = 0; j < 3; ++j) {
        shared += (o[j] == a || o[j] == b);
        if (o[j] != a && o[j] != b)
          back = j;
      }
      ASSERT_EQ(shared, 2);
      ASSERT_EQ(t.neighbors[std::size_t(n)][back], int(i));
    }
  }
}

TEST(PlanarTriangulation, DelaunayHasEmptyCircumcirclesAndTilesTheHull) {
  std::mt19937 rng(71);
  for (int k = 0; k < 6; ++k) {
    const std::vector<point> p = random_points(rng, 50 + 90 * k, 10000);
    const triangulation t = delaunay(p);
    check_structure(t);
    std::int64_t sum = 0;
    for (const auto &tri : t.triangles) {
      sum += twice_area(p[tri[0]], p[tri[1]], p[tri[2]]);
      for (const point &q : p)
        ASSERT_LE(in_circle(p[tri[0]], p[tri[1]], p[tri[2]], q), 0)
            << "a point inside a circumcircle";
    }
    EXPECT_EQ(sum, twice_hull_area(p));
  }
}

TEST(PlanarTriangulation, DelaunayDegenerateInputs) {
  // A grid: every unit square's corners are cocircular. 2 triangles per square, none flat.
  std::vector<point> g;
  for (coord y = 0; y < 12; ++y)
    for (coord x = 0; x < 15; ++x)
      g.emplace_back(x * 1000, y * 1000);
  const triangulation tg = delaunay(g);
  check_structure(tg);
  EXPECT_EQ(tg.triangles.size(), std::size_t(2 * 11 * 14));
  for (const auto &tri : tg.triangles)
    for (const point &q : g)
      ASSERT_LE(in_circle(g[tri[0]], g[tri[1]], g[tri[2]], q), 0);
  // Collinear points, fewer than three, and duplicates.
  EXPECT_TRUE(delaunay({{0, 0}, {5, 5}, {10, 10}, {-3, -3}}).triangles.empty());
  EXPECT_TRUE(delaunay({{0, 0}, {5, 5}}).triangles.empty());
  EXPECT_TRUE(delaunay({}).triangles.empty());
  const std::vector<point> dup = {{0, 0}, {10, 0}, {0, 0}, {0, 10}, {10, 0}, {10, 10}};
  const triangulation td = delaunay(dup);
  EXPECT_EQ(td.vertices.size(), dup.size());
  EXPECT_EQ(td.triangles.size(), 2u);
  for (const auto &tri : td.triangles)
    for (int v : tri)
      EXPECT_TRUE(v == 0 || v == 1 || v == 3 || v == 5) << "a duplicate's index: " << v;
  // The edge of the range.
  const coord m = coord_max;
  const triangulation big = delaunay({{-m, -m}, {m, -m}, {m, m}, {-m, m}, {0, 1}});
  EXPECT_EQ(big.triangles.size(), 4u);
  EXPECT_THROW(delaunay({{0, 0}, {m + 1, 0}, {0, 1}}), planar_error);
}

// Does the open segment (p, q) cross the closed segment [a, b] properly (not at an endpoint)?
bool crosses(const pointd &p, const pointd &q, const pointd &a, const pointd &b) {
  return orientation(p, q, a) * orientation(p, q, b) < 0 &&
         orientation(a, b, p) * orientation(a, b, q) < 0;
}

TEST(PlanarTriangulation, ConstrainedDelaunayKeepsTheEdgesAndHidesOnlyBehindThem) {
  std::mt19937 rng(73);
  std::vector<point> p = random_points(rng, 400, 10000);
  const ring s = star(rng, 0, 0, 8000, 40);
  std::vector<std::array<int, 2>> edges;
  const int first = int(p.size());
  for (std::size_t i = 0; i < s.size(); ++i) {
    p.push_back(s[i]);
    edges.push_back({first + int(i), first + int((i + 1) % s.size())});
  }
  const triangulation t = constrained_delaunay(p, edges);
  check_structure(t);
  std::map<std::pair<int, int>, int> has;
  for (const auto &tri : t.triangles)
    for (int k = 0; k < 3; ++k)
      ++has[{std::min(tri[k], tri[(k + 1) % 3]), std::max(tri[k], tri[(k + 1) % 3])}];
  // Every constraint is an edge, unless a point on it split it.
  for (const auto &e : edges) {
    const std::pair<int, int> key(std::min(e[0], e[1]), std::max(e[0], e[1]));
    bool split = false;
    for (const point &q : p)
      split = split || (q != p[e[0]] && q != p[e[1]] && on_segment(q, p[e[0]], p[e[1]]));
    if (!split)
      EXPECT_TRUE(has.count(key)) << "constraint " << e[0] << "-" << e[1] << " is missing";
  }
  // A point inside a triangle's circumcircle is hidden from the triangle by a constraint.
  std::int64_t sum = 0;
  for (const auto &tri : t.triangles) {
    sum += twice_area(p[tri[0]], p[tri[1]], p[tri[2]]);
    const pointd c((double(p[tri[0]].x) + double(p[tri[1]].x) + double(p[tri[2]].x)) / 3,
                   (double(p[tri[0]].y) + double(p[tri[1]].y) + double(p[tri[2]].y)) / 3);
    for (std::size_t i = 0; i < p.size(); ++i) {
      if (in_circle(p[tri[0]], p[tri[1]], p[tri[2]], p[i]) <= 0)
        continue;
      bool hidden = false;
      const pointd q(double(p[i].x), double(p[i].y));
      for (const auto &e : edges)
        hidden = hidden || crosses(q, c, pointd(double(p[e[0]].x), double(p[e[0]].y)),
                                   pointd(double(p[e[1]].x), double(p[e[1]].y)));
      ASSERT_TRUE(hidden) << "point " << i << " is inside a circumcircle and visible";
    }
  }
  EXPECT_EQ(sum, twice_hull_area(p));
}

TEST(PlanarTriangulation, ConstraintsThroughPointsSplitAndCrossingOnesThrow) {
  const std::vector<point> p = {{0, 0}, {10, 0}, {5, 0}, {5, 5}, {5, -5}};
  const triangulation t = constrained_delaunay(p, {{0, 1}});
  std::set<std::pair<int, int>> es;
  for (const auto &tri : t.triangles)
    for (int k = 0; k < 3; ++k)
      es.insert({std::min(tri[k], tri[(k + 1) % 3]), std::max(tri[k], tri[(k + 1) % 3])});
  EXPECT_TRUE(es.count({0, 2}));
  EXPECT_TRUE(es.count({1, 2}));
  EXPECT_THROW(constrained_delaunay({{0, 0}, {10, 10}, {10, 0}, {0, 10}}, {{0, 1}, {2, 3}}),
               planar_error);
  EXPECT_THROW(constrained_delaunay(p, {{0, 7}}), planar_error);
}

TEST(PlanarTriangulation, PolygonsTriangulateExactlyTheirInside) {
  std::mt19937 rng(79);
  for (int k = 0; k < 10; ++k) {
    const ring r = star(rng, 0, 0, 10000, 6 + 7 * k);
    const triangulation t = triangulate(polygon(r));
    check_structure(t);
    EXPECT_EQ(t.triangles.size(), r.size() - 2);
    std::int64_t sum = 0;
    for (const auto &tri : t.triangles)
      sum += twice_area(t.vertices[tri[0]], t.vertices[tri[1]], t.vertices[tri[2]]);
    EXPECT_EQ(double(sum), 2 * signed_area(r));
  }
  // Holes: n + 2h - 2 triangles, none inside a hole, the area the polygon's.
  const polygon ph(square(0, 0, 1000),
                   {square(100, 100, 200), ring{{600, 600}, {800, 700}, {650, 850}}});
  const triangulation th = triangulate(ph);
  check_structure(th);
  EXPECT_EQ(th.triangles.size(), 4u + 4u + 3u + 2 * 2 - 2);
  double sum = 0;
  for (const auto &tri : th.triangles) {
    const point &a = th.vertices[tri[0]], &b = th.vertices[tri[1]], &c = th.vertices[tri[2]];
    sum += double(twice_area(a, b, c)) / 2;
    const pointd centre((double(a.x) + double(b.x) + double(c.x)) / 3,
                        (double(a.y) + double(b.y) + double(c.y)) / 3);
    EXPECT_TRUE(brute_inside(centre, {ph}));
  }
  EXPECT_EQ(sum, area(ph));
  // Collinear vertices are kept: a square with its edges' midpoints.
  const ring mids = {{0, 0},     {50, 0},   {100, 0}, {100, 50},
                     {100, 100}, {50, 100}, {0, 100}, {0, 50}};
  const triangulation tm = triangulate(polygon(mids));
  EXPECT_EQ(tm.triangles.size(), 6u);
  std::set<int> used;
  for (const auto &tri : tm.triangles)
    used.insert(tri.begin(), tri.end());
  EXPECT_EQ(used.size(), 8u);
  // A set: the polygons one after another.
  const triangulation ts =
      triangulate(polygon_set{polygon(square(0, 0, 10)), polygon(square(20, 0, 10))});
  EXPECT_EQ(ts.vertices.size(), 8u);
  EXPECT_EQ(ts.triangles.size(), 4u);
  EXPECT_THROW(triangulate(polygon(ring{{0, 0}, {10, 10}, {10, 0}, {0, 10}})), planar_error);
}

// --- convex partitions
// ----------------------------------------------------------------------------

int reflex_count(const ring &r) {
  int n = 0;
  for (std::size_t i = 0; i < r.size(); ++i)
    n += orientation(r[(i + r.size() - 1) % r.size()], r[i], r[(i + 1) % r.size()]) < 0;
  return n;
}

void check_partition(const polygon &p, const std::vector<ring> &pieces, std::mt19937 &rng) {
  double sum = 0;
  for (const ring &q : pieces) {
    ASSERT_GE(q.size(), 3u);
    for (std::size_t i = 0; i < q.size(); ++i)
      ASSERT_GE(orientation(q[(i + q.size() - 1) % q.size()], q[i], q[(i + 1) % q.size()]), 0)
          << "a reflex corner in a piece";
    ASSERT_GT(signed_area(q), 0);
    sum += signed_area(q);
  }
  EXPECT_DOUBLE_EQ(sum, area(p));
  // A sample point inside the polygon is inside exactly one piece; outside, in none.
  const polygon n = normalize(p);
  coord xmin = n.outer[0].x, xmax = xmin, ymin = n.outer[0].y, ymax = ymin;
  for (const point &v : n.outer) {
    xmin = std::min(xmin, v.x), xmax = std::max(xmax, v.x);
    ymin = std::min(ymin, v.y), ymax = std::max(ymax, v.y);
  }
  std::uniform_int_distribution<coord> cx(xmin, xmax), cy(ymin, ymax);
  for (int i = 0; i < 500; ++i) {
    const point q(cx(rng), cy(rng));
    const location l = locate(q, p);
    if (l == location::boundary)
      continue;
    int inside = 0, boundary = 0;
    for (const ring &r : pieces) {
      const location lr = locate(q, r);
      inside += lr == location::inside;
      boundary += lr == location::boundary;
    }
    if (l == location::outside)
      ASSERT_EQ(inside + boundary, 0);
    else if (boundary == 0)
      ASSERT_EQ(inside, 1);
  }
}

TEST(PlanarPartition, HertelMehlhornGivesConvexPiecesThatTileThePolygon) {
  std::mt19937 rng(83);
  for (int k = 0; k < 12; ++k) {
    const ring r = star(rng, 0, 0, 10000, 8 + 6 * k);
    const std::vector<ring> pieces = convex_partition(polygon(r));
    check_partition(polygon(r), pieces, rng);
    EXPECT_LE(int(pieces.size()), 2 * reflex_count(r) + 1);
  }
  const ring l = {{0, 0}, {20, 0}, {20, 10}, {10, 10}, {10, 20}, {0, 20}};
  EXPECT_EQ(convex_partition(polygon(l)).size(), 2u);
  std::mt19937 rng2(89);
  EXPECT_EQ(convex_partition(polygon(convex(rng2, 0, 0, 1000, 20))).size(), 1u);
  const polygon ph(square(0, 0, 1000), {square(300, 300, 400)});
  const std::vector<ring> pieces = convex_partition(ph);
  check_partition(ph, pieces, rng2);
  EXPECT_GE(pieces.size(), 4u); // a square ring needs at least four convex pieces
}

// --- Sibson coordinates
// ---------------------------------------------------------------------------

// The part of `poly` nearer site i than every other site: cut by each bisector.
std::vector<pointd> clip_to_cell(std::vector<pointd> poly, const std::vector<pointd> &sites,
                                 std::size_t i) {
  const pointd s = sites[i];
  for (std::size_t j = 0; j < sites.size() && poly.size() >= 3; ++j) {
    if (j == i)
      continue;
    const pointd t = sites[j];
    // The bisector through the midpoint, directed so that s is on its left.
    const pointd m((s.x + t.x) / 2, (s.y + t.y) / 2), d(t.x - s.x, t.y - s.y);
    poly = clip_left(poly, pointd(m.x + d.y, m.y - d.x), pointd(m.x - d.y, m.y + d.x));
  }
  return poly;
}

TEST(PlanarSibson, CoordinatesMatchVoronoiCellsByHalfPlaneClipping) {
  std::mt19937 rng(97);
  const std::vector<point> p = random_points(rng, 40, 1000);
  const triangulation dt = delaunay(p);
  std::vector<pointd> sites;
  for (const point &v : p)
    sites.push_back(pointd(double(v.x), double(v.y)));
  std::uniform_real_distribution<double> c(-600, 600);
  const double big = 1e6;
  const std::vector<pointd> box = {pointd(-big, -big), pointd(big, -big), pointd(big, big),
                                   pointd(-big, big)};
  int checked = 0;
  for (int k = 0; k < 60; ++k) {
    const pointd q(c(rng), c(rng));
    const std::vector<natural_neighbor> w = sibson_coordinates(dt, q);
    if (w.empty())
      continue; // outside the hull
    std::vector<pointd> with_q = sites;
    with_q.push_back(q);
    const std::vector<pointd> qcell = clip_to_cell(box, with_q, with_q.size() - 1);
    const double qa = area_of(qcell);
    double sum = 0, rx = 0, ry = 0;
    std::vector<double> want(p.size(), 0.0);
    for (std::size_t i = 0; i < p.size(); ++i)
      want[i] = area_of(clip_to_cell(qcell, sites, i)) / qa;
    for (const natural_neighbor &n : w) {
      EXPECT_GT(n.weight, 0);
      EXPECT_NEAR(n.weight, want[std::size_t(n.vertex)], 1e-7) << "vertex " << n.vertex;
      want[std::size_t(n.vertex)] = 0;
      sum += n.weight;
      rx += n.weight * sites[std::size_t(n.vertex)].x;
      ry += n.weight * sites[std::size_t(n.vertex)].y;
    }
    for (double rest : want)
      EXPECT_NEAR(rest, 0.0, 1e-9) << "a natural neighbour left out";
    EXPECT_NEAR(sum, 1.0, 1e-12);
    EXPECT_NEAR(rx, q.x, 1e-6);
    EXPECT_NEAR(ry, q.y, 1e-6);
    ++checked;
  }
  EXPECT_GT(checked, 30);
}

TEST(PlanarSibson, InterpolationReproducesLinearFunctions) {
  std::mt19937 rng(101);
  const std::vector<point> p = random_points(rng, 300, 100000);
  const triangulation dt = delaunay(p);
  std::vector<double> f;
  for (const point &v : p)
    f.push_back(3.0 * double(v.x) - 2.0 * double(v.y) + 7.0);
  std::uniform_real_distribution<double> c(-90000, 90000);
  for (int k = 0; k < 200; ++k) {
    const pointd q(c(rng), c(rng));
    const double z = sibson_interpolate(dt, f, q);
    if (std::isnan(z))
      continue;
    EXPECT_NEAR(z, 3.0 * q.x - 2.0 * q.y + 7.0, 1e-5 * 100000);
  }
}

TEST(PlanarSibson, VerticesHullEdgesAndOutside) {
  const std::vector<point> p = {{0, 0}, {100, 0}, {100, 100}, {0, 100}, {40, 60}};
  const triangulation dt = delaunay(p);
  const std::vector<natural_neighbor> at = sibson_coordinates(dt, pointd(40, 60));
  ASSERT_EQ(at.size(), 1u);
  EXPECT_EQ(at[0].vertex, 4);
  EXPECT_EQ(at[0].weight, 1.0);
  // On the hull: linear along the edge from (0, 0) to (100, 0).
  const std::vector<natural_neighbor> edge = sibson_coordinates(dt, pointd(25, 0));
  ASSERT_EQ(edge.size(), 2u);
  for (const natural_neighbor &n : edge)
    EXPECT_NEAR(n.weight, n.vertex == 0 ? 0.75 : 0.25, 1e-12);
  EXPECT_TRUE(sibson_coordinates(dt, pointd(150, 50)).empty());
  const std::vector<double> f = {1, 2, 3, 4, 5};
  EXPECT_TRUE(std::isnan(sibson_interpolate(dt, f, pointd(-1, 50))));
  EXPECT_EQ(sibson_interpolate(dt, f, pointd(40, 60)), 5.0);
  EXPECT_THROW(sibson_interpolate(dt, {1, 2}, pointd(10, 10)), planar_error);
}

// --- the medial axis
// ------------------------------------------------------------------------------

// Every segment of a polygon's boundary.
std::vector<std::pair<point, point>> boundary_segments(const polygon &p) {
  std::vector<std::pair<point, point>> s;
  auto add = [&](const ring &r) {
    for (std::size_t i = 0; i < r.size(); ++i)
      s.push_back({r[i], r[(i + 1) % r.size()]});
  };
  add(p.outer);
  for (const ring &h : p.holes)
    add(h);
  return s;
}

// The nearest point on [a, b] to q.
pointd nearest_on(const pointd &q, const point &a, const point &b) {
  const double ax = double(a.x), ay = double(a.y), dx = double(b.x) - ax, dy = double(b.y) - ay;
  const double l2 = dx * dx + dy * dy;
  double t = l2 > 0 ? ((q.x - ax) * dx + (q.y - ay) * dy) / l2 : 0;
  t = std::max(0.0, std::min(1.0, t));
  return pointd(ax + t * dx, ay + t * dy);
}

// Brute force: the distance from q to the boundary, and how many distinct boundary points are
// that near (within `eps`).
std::pair<double, int> nearest_points(const pointd &q, const polygon &p, double eps) {
  const auto segs = boundary_segments(p);
  double d = std::numeric_limits<double>::infinity();
  for (const auto &s : segs) {
    const pointd n = nearest_on(q, s.first, s.second);
    d = std::min(d, std::hypot(q.x - n.x, q.y - n.y));
  }
  std::vector<pointd> near;
  for (const auto &s : segs) {
    const pointd n = nearest_on(q, s.first, s.second);
    if (std::hypot(q.x - n.x, q.y - n.y) > d + eps)
      continue;
    bool seen = false;
    for (const pointd &m : near)
      seen = seen || std::hypot(m.x - n.x, m.y - n.y) <= 4 * eps;
    if (!seen)
      near.push_back(n);
  }
  return {d, int(near.size())};
}

// Connected components of the graph, by union-find.
int components(const medial_axis_graph &g) {
  std::vector<int> parent(g.vertices.size());
  for (std::size_t i = 0; i < parent.size(); ++i)
    parent[i] = int(i);
  std::function<int(int)> find = [&](int i) {
    return parent[std::size_t(i)] == i ? i : parent[std::size_t(i)] = find(parent[std::size_t(i)]);
  };
  for (const auto &e : g.edges)
    parent[std::size_t(find(e[0]))] = find(e[1]);
  int n = 0;
  for (std::size_t i = 0; i < parent.size(); ++i)
    n += find(int(i)) == int(i);
  return n;
}

// What every medial axis satisfies, checked by brute force: each vertex inside the polygon (or at a
// convex corner), its radius its distance to the boundary, at least two nearest boundary points
// (away from the corners), one connected piece with a loop per hole, and polylines that use every
// edge once.
void check_medial_axis(const polygon &p, const medial_axis_graph &g, double tol) {
  ASSERT_EQ(g.radius.size(), g.vertices.size());
  ASSERT_FALSE(g.edges.empty());
  const polygon_set ps = {p};
  for (std::size_t i = 0; i < g.vertices.size(); ++i) {
    const pointd &v = g.vertices[i];
    const auto near = nearest_points(v, p, tol + 1e-6 * (1 + g.radius[i]));
    ASSERT_NEAR(g.radius[i], near.first, tol + 1e-7 * (1 + near.first)) << "vertex " << i;
    if (near.first < 1e-6)
      continue; // a convex corner, on the boundary
    ASSERT_TRUE(brute_inside(v, ps)) << "vertex " << i << " (" << v.x << ", " << v.y << ")";
    ASSERT_GE(near.second, 2) << "vertex " << i << " (" << v.x << ", " << v.y
                              << ") has one nearest point";
  }
  EXPECT_EQ(components(g), 1);
  EXPECT_EQ(int(g.edges.size()) - int(g.vertices.size()) + 1, int(p.holes.size()))
      << "a loop around each hole, and no other";
  std::vector<int> uses(g.edges.size(), 0);
  std::map<std::pair<int, int>, int> edge_of;
  for (int i = 0; i < int(g.edges.size()); ++i)
    edge_of[{std::min(g.edges[i][0], g.edges[i][1]), std::max(g.edges[i][0], g.edges[i][1])}] = i;
  for (const auto &line : g.polylines)
    for (std::size_t k = 0; k + 1 < line.size(); ++k) {
      const auto it =
          edge_of.find({std::min(line[k], line[k + 1]), std::max(line[k], line[k + 1])});
      ASSERT_NE(it, edge_of.end());
      ++uses[std::size_t(it->second)];
    }
  for (int u : uses)
    EXPECT_EQ(u, 1);
}

int find_vertex(const medial_axis_graph &g, double x, double y) {
  for (std::size_t i = 0; i < g.vertices.size(); ++i)
    if (std::fabs(g.vertices[i].x - x) < 1e-6 && std::fabs(g.vertices[i].y - y) < 1e-6)
      return int(i);
  return -1;
}

TEST(PlanarMedialAxis, ARectangleHasItsKnownAxis) {
  // 400 x 100: the middle segment from (50, 50) to (350, 50) and the four corners' bisectors.
  const polygon p(ring{{0, 0}, {400, 0}, {400, 100}, {0, 100}});
  const medial_axis_graph g = medial_axis(p);
  check_medial_axis(p, g, 1e-6);
  EXPECT_EQ(g.vertices.size(), 6u);
  EXPECT_EQ(g.edges.size(), 5u);
  EXPECT_EQ(g.polylines.size(), 5u);
  const int a = find_vertex(g, 50, 50), b = find_vertex(g, 350, 50);
  ASSERT_GE(a, 0);
  ASSERT_GE(b, 0);
  EXPECT_DOUBLE_EQ(g.radius[std::size_t(a)], 50.0);
  for (const auto &c :
       std::vector<std::pair<double, double>>{{0, 0}, {400, 0}, {400, 100}, {0, 100}}) {
    const int k = find_vertex(g, c.first, c.second);
    ASSERT_GE(k, 0);
    EXPECT_EQ(g.radius[std::size_t(k)], 0.0);
  }
}

TEST(PlanarMedialAxis, ARegularPolygonsSpokesMeetAtTheCentre) {
  ring r;
  const int k = 12;
  for (int i = 0; i < k; ++i)
    r.emplace_back(coord(std::llround(100000 * std::cos(2 * kPi * i / k))),
                   coord(std::llround(100000 * std::sin(2 * kPi * i / k))));
  const polygon p(r);
  const medial_axis_graph g = medial_axis(p);
  check_medial_axis(p, g, 1e-6);
  // Rounding the corners to the grid splits the centre slightly; every spoke still ends at a
  // corner.
  int ends = 0;
  std::vector<int> degree(g.vertices.size(), 0);
  for (const auto &e : g.edges)
    ++degree[std::size_t(e[0])], ++degree[std::size_t(e[1])];
  for (int d : degree)
    ends += d == 1;
  EXPECT_EQ(ends, k);
  double rmax = 0;
  for (double rad : g.radius)
    rmax = std::max(rmax, rad);
  EXPECT_NEAR(rmax, 100000 * std::cos(kPi / k), 2.0); // the inradius
}

TEST(PlanarMedialAxis, AReflexVertexGivesAParabolicArc) {
  // An L: the reflex vertex (10, 10) faces edges, so the axis curves round it, sampled within the
  // tolerance, and never touches it.
  const polygon p(ring{{0, 0}, {2000, 0}, {2000, 1000}, {1000, 1000}, {1000, 2000}, {0, 2000}});
  for (double tol : {0.5, 5.0}) {
    medial_axis_params mp;
    mp.tolerance = tol;
    const medial_axis_graph g = medial_axis(p, mp);
    check_medial_axis(p, g, tol);
    double nearest_to_reflex = std::numeric_limits<double>::infinity();
    for (const pointd &v : g.vertices)
      nearest_to_reflex = std::min(nearest_to_reflex, std::hypot(v.x - 1000, v.y - 1000));
    EXPECT_GT(nearest_to_reflex, 100.0);
  }
  // A finer tolerance samples the arcs more finely.
  medial_axis_params fine, coarse;
  fine.tolerance = 0.5;
  coarse.tolerance = 50;
  EXPECT_GT(medial_axis(p, fine).vertices.size(), medial_axis(p, coarse).vertices.size());
}

TEST(PlanarMedialAxis, RandomPolygonsAndHolesSatisfyTheDefinition) {
  std::mt19937 rng(103);
  for (int k = 0; k < 8; ++k) {
    const polygon p(star(rng, 0, 0, 100000, 8 + 5 * k));
    check_medial_axis(p, medial_axis(p), 0.5);
  }
  // Holes: a loop around each.
  polygon ph(square(0, 0, 100000), {square(20000, 20000, 20000), square(60000, 55000, 25000)});
  std::reverse(ph.holes[0].begin(), ph.holes[0].end());
  check_medial_axis(ph, medial_axis(ph), 0.5);
  std::mt19937 rng2(107);
  polygon pr(star(rng2, 0, 0, 100000, 40, 0.8), {convex(rng2, 0, 0, 30000, 12)});
  check_medial_axis(pr, medial_axis(pr), 0.5);
}

TEST(PlanarMedialAxis, BadInputThrows) {
  EXPECT_THROW(medial_axis(polygon(ring{{0, 0}, {10, 10}, {10, 0}, {0, 10}})), planar_error);
  EXPECT_THROW(medial_axis(polygon(ring{{0, 0}, {coord_max + 1, 0}, {0, 10}})), planar_error);
  medial_axis_params bad;
  bad.tolerance = 0;
  EXPECT_THROW(medial_axis(polygon(square(0, 0, 10)), bad), planar_error);
  // The edge of the range is fine: Boost.Polygon's input is int32.
  const coord m = coord_max;
  const medial_axis_graph g = medial_axis(polygon(ring{{-m, -m}, {m, -m}, {m, m}, {-m, m}}));
  EXPECT_EQ(g.edges.size(), 4u);
}

// --- no singletons (T6) -------------------------------------------------------------------------

TEST(PlanarThreads, TwoThreadsGiveWhatEachGivesAlone) {
  auto work = [](unsigned seed) {
    std::mt19937 rng(seed);
    std::vector<double> out;
    for (int k = 0; k < 20; ++k) {
      const polygon_set a = set_of(star(rng, 0, 0, 1e6, 30 + k));
      const polygon_set b = set_of(star(rng, 2e5, 1e5, 8e5, 25 + k));
      out.push_back(area(unite(a, b)));
      out.push_back(area(intersect(a, b)));
      out.push_back(area(wavefront_offset(a, -1e5)));
      out.push_back(area(offset(b, 5e4)));
      const triangulation t = delaunay(a[0].outer);
      out.push_back(double(t.triangles.size()));
      out.push_back(double(convex_partition(a[0]).size()));
      for (const natural_neighbor &n : sibson_coordinates(t, pointd(1000.5, -2000.25)))
        out.push_back(n.weight);
      const medial_axis_graph g = medial_axis(a[0]);
      out.push_back(double(g.vertices.size()));
      for (double r : g.radius)
        out.push_back(r);
    }
    return out;
  };
  const std::vector<double> alone1 = work(61), alone2 = work(67);
  std::vector<double> t1, t2;
  std::thread a([&] { t1 = work(61); });
  std::thread b([&] { t2 = work(67); });
  a.join();
  b.join();
  EXPECT_EQ(t1, alone1);
  EXPECT_EQ(t2, alone2);
  EXPECT_EQ(work(67), alone2); // and in the other order
}

} // namespace

#endif // CVC_WITH_PLANAR
