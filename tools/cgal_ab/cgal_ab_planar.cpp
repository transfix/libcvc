// cgal_ab_planar.cpp -- row T16 for cvc::planar (inc/cvc/geometry/planar.h), the ContourTiler
// port's planar geometry (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §3, P3a): each operation
// against the CGAL call ContourTiler makes today, on the same inputs, for results and time. See
// README.md. cgal_ab_permissive.cpp checks the libraries on their own; this checks cvc::planar's
// functions, which are what the tiler will call.
//
//   A: CGAL (GPL-3+), as ContourTiler calls it      B: cvc::planar
//   Polygon_set_2 join / intersection /             unite / intersect / subtract / exclusive_or
//     difference / symmetric_difference (Epeck)
//   create_{interior,exterior}_skeleton_and_        wavefront_offset (asserted), and offset with
//     offset_polygons_with_holes_2                    an unlimited mitre (INFO past an edge event)
//   minkowski_sum_2                                 minkowski_sum
//
// Results are compared by area, by the counts of outer rings and holes, and by the area of the
// symmetric difference between the two results (CGAL's exact vertices rounded to the grid): within
// a grid unit per unit of boundary, which is what rounding new vertices to the grid allows.

#include "cgal_ab.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>
#include <CGAL/minkowski_sum_2.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/geometry/planar.h>
#include <iterator>
#include <string>
#include <vector>

namespace cgal_ab {
namespace {

namespace pl = cvc::planar;
using Epick = CGAL::Exact_predicates_inexact_constructions_kernel;
using Epeck = CGAL::Exact_predicates_exact_constructions_kernel;
using Poly = CGAL::Polygon_2<Epeck>;
using Pwh = CGAL::Polygon_with_holes_2<Epeck>;

template <typename... Args> std::string fmt(const char *f, Args... args) {
  char buf[512];
  std::snprintf(buf, sizeof buf, f, args...);
  return buf;
}

const double kPi = 3.14159265358979323846;

// --- conversions --------------------------------------------------------------------------------

Poly to_cgal(const pl::ring &r) {
  Poly p;
  for (const pl::point &v : r)
    p.push_back(Epeck::Point_2(double(v.x), double(v.y)));
  return p;
}

CGAL::Polygon_2<Epick> to_epick(const pl::ring &r) {
  CGAL::Polygon_2<Epick> p;
  for (const pl::point &v : r)
    p.push_back(Epick::Point_2(double(v.x), double(v.y)));
  return p;
}

Pwh to_cgal(const pl::polygon &p) {
  const pl::polygon n = pl::normalize(p);
  Pwh out(to_cgal(n.outer));
  for (const pl::ring &h : n.holes)
    out.add_hole(to_cgal(h));
  return out;
}

pl::ring of_cgal(const Poly &p) {
  pl::ring r;
  for (auto v = p.vertices_begin(); v != p.vertices_end(); ++v)
    r.emplace_back(pl::coord(std::llround(CGAL::to_double(v->x()))),
                   pl::coord(std::llround(CGAL::to_double(v->y()))));
  return r;
}

// CGAL's result, its exact vertices rounded to the grid.
pl::polygon_set of_cgal(const std::vector<Pwh> &ps) {
  pl::polygon_set out;
  for (const Pwh &p : ps) {
    pl::polygon q(of_cgal(p.outer_boundary()));
    for (auto h = p.holes_begin(); h != p.holes_end(); ++h)
      q.holes.push_back(of_cgal(*h));
    out.push_back(pl::normalize(q));
  }
  return out;
}

// --- comparisons --------------------------------------------------------------------------------

struct summary {
  double area = 0, perimeter = 0;
  int outers = 0, holes = 0;
};

double perimeter_of(const Poly &p) {
  double s = 0;
  for (auto e = p.edges_begin(); e != p.edges_end(); ++e)
    s += std::sqrt(CGAL::to_double(e->squared_length()));
  return s;
}

summary of(const std::vector<Pwh> &ps) {
  summary s;
  Epeck::FT a = 0;
  for (const Pwh &p : ps) {
    ++s.outers;
    a += p.outer_boundary().area();
    s.perimeter += perimeter_of(p.outer_boundary());
    for (auto h = p.holes_begin(); h != p.holes_end(); ++h) {
      ++s.holes;
      a += h->area(); // clockwise: negative
      s.perimeter += perimeter_of(*h);
    }
  }
  s.area = CGAL::to_double(a);
  return s;
}

summary of(const pl::polygon_set &ps) {
  summary s;
  for (const pl::polygon &p : ps) {
    ++s.outers;
    s.holes += int(p.holes.size());
    s.perimeter += pl::perimeter(p);
  }
  s.area = pl::area(ps);
  return s;
}

struct verdict {
  bool ok;
  std::string detail;
};

// Same area (exactly, or within a unit per unit of boundary), same counts (unless `counts` is
// false), and a symmetric difference within the tolerance.
verdict compare(const std::vector<Pwh> &a, const pl::polygon_set &b, bool exact, bool counts) {
  const summary sa = of(a), sb = of(b);
  const double tol = exact ? 0.0 : sa.perimeter;
  const double xor_area = pl::area(pl::exclusive_or(of_cgal(a), b));
  // Rounding CGAL's own vertices for the comparison moves its boundary by up to half a unit.
  const double xor_tol = sa.perimeter;
  const bool same_counts = sa.outers == sb.outers && sa.holes == sb.holes;
  const bool ok =
      std::fabs(sa.area - sb.area) <= tol && (same_counts || !counts) && xor_area <= xor_tol;
  return {ok, fmt("area cgal %.1f, planar %.1f (diff %.3g, %s %.3g); %d/%d outer, %d/%d holes; "
                  "symmetric difference %.3g (tol %.3g)",
                  sa.area, sb.area, sb.area - sa.area, exact ? "exact, tol" : "tol", tol, sa.outers,
                  sb.outers, sa.holes, sb.holes, xor_area, xor_tol)};
}

// --- inputs -------------------------------------------------------------------------------------

pl::ring star(rng &r, double cx, double cy, double rad, int k, double rmin = 0.35) {
  pl::ring p;
  for (int i = 0; i < k; ++i) {
    const double t = 2 * kPi * (i + r.uniform(0, 0.8)) / k, q = r.uniform(rmin, 1.0) * rad;
    p.emplace_back(pl::coord(std::llround(cx + q * std::cos(t))),
                   pl::coord(std::llround(cy + q * std::sin(t))));
  }
  return pl::clean(p);
}

pl::ring convex(rng &r, double cx, double cy, double rad, int k) {
  pl::ring p;
  for (int i = 0; i < k; ++i) {
    const double t = 2 * kPi * (i + r.uniform(0, 0.8)) / k;
    p.emplace_back(pl::coord(std::llround(cx + rad * std::cos(t))),
                   pl::coord(std::llround(cy + rad * std::sin(t))));
  }
  return pl::clean(p);
}

pl::ring skyline(rng &r, pl::coord x0, pl::coord width, pl::coord y0, pl::coord hmax, int n,
                 bool transpose) {
  std::vector<pl::coord> xs = {x0, x0 + width};
  while (int(xs.size()) < n + 1) {
    const pl::coord x = x0 + 1 + pl::coord(r.next() % std::uint64_t(width - 1));
    if (std::find(xs.begin(), xs.end(), x) == xs.end())
      xs.push_back(x);
  }
  std::sort(xs.begin(), xs.end());
  std::vector<pl::coord> h(n);
  for (int i = 0; i < n; ++i)
    do
      h[i] = hmax / 5 + pl::coord(r.next() % std::uint64_t(hmax - hmax / 5));
    while (i > 0 && h[i] == h[i - 1]);
  pl::ring p = {{xs[0], y0}, {xs[n], y0}};
  for (int i = n - 1; i >= 0; --i) {
    p.emplace_back(xs[i + 1], y0 + h[i]);
    p.emplace_back(xs[i], y0 + h[i]);
  }
  if (transpose) {
    for (pl::point &v : p)
      std::swap(v.x, v.y);
    std::reverse(p.begin(), p.end());
  }
  return pl::clean(p);
}

// A contour as ContourTiler sees one: a closed curve sampled densely (many short edges), wobbly,
// with a concave bay, on a grid fitted to its box.
pl::ring contour(rng &r, double rad, int k) {
  pl::ring p;
  const double phase = r.uniform(0, 2 * kPi), bay = r.uniform(0.3, 0.6);
  for (int i = 0; i < k; ++i) {
    const double t = 2 * kPi * i / k;
    const double q = rad * (1 + 0.15 * std::sin(3 * t + phase) + 0.05 * std::sin(11 * t) -
                            bay * std::exp(-12 * (1 - std::cos(t - phase))));
    p.emplace_back(pl::coord(std::llround(q * std::cos(t))),
                   pl::coord(std::llround(q * std::sin(t))));
  }
  return pl::clean(p);
}

// --- Booleans -----------------------------------------------------------------------------------

enum class op { join, intersect, difference, xor_ };
const char *op_name(op o) {
  switch (o) {
  case op::join:
    return "union";
  case op::intersect:
    return "intersection";
  case op::difference:
    return "difference";
  default:
    return "symmetric difference";
  }
}

void ab_boolean(const std::string &name, const pl::polygon_set &p, const pl::polygon_set &q, op o,
                bool exact, double &ta, double &tb) {
  std::vector<Pwh> a;
  pl::polygon_set b;
  ta += ms_of([&] {
    CGAL::Polygon_set_2<Epeck> s;
    for (const pl::polygon &x : p)
      s.join(to_cgal(x));
    CGAL::Polygon_set_2<Epeck> t;
    for (const pl::polygon &x : q)
      t.join(to_cgal(x));
    switch (o) {
    case op::join:
      s.join(t);
      break;
    case op::intersect:
      s.intersection(t);
      break;
    case op::difference:
      s.difference(t);
      break;
    default:
      s.symmetric_difference(t);
      break;
    }
    s.polygons_with_holes(std::back_inserter(a));
  });
  tb += ms_of([&] {
    switch (o) {
    case op::join:
      b = pl::unite(p, q);
      break;
    case op::intersect:
      b = pl::intersect(p, q);
      break;
    case op::difference:
      b = pl::subtract(p, q);
      break;
    default:
      b = pl::exclusive_or(p, q);
      break;
    }
  });
  // A symmetric difference's pieces touch where the boundaries cross: CGAL keeps pieces touching at
  // a point as one polygon with holes, Clipper2 as separate polygons. So there only the area and
  // shape are compared.
  const verdict v = compare(a, b, exact, o != op::xor_);
  report(v.ok, std::string("planar ") + op_name(o) + " " + name, v.detail);
}

void ab_booleans(rng &r) {
  double ta = 0, tb = 0;
  const op ops[] = {op::join, op::intersect, op::difference, op::xor_};
  for (int i = 0; i < 3; ++i) {
    const pl::polygon_set p = {pl::polygon(skyline(r, 0, 1000000, 0, 800000, 12 + 4 * i, false))};
    const pl::polygon_set q = {
        pl::polygon(skyline(r, 100000, 900000, 50000, 900000, 10 + 3 * i, true))};
    for (op o : ops)
      ab_boolean(fmt("skylines #%d", i), p, q, o, true, ta, tb);
  }
  for (int i = 0; i < 3; ++i) {
    const pl::polygon_set p = {pl::polygon(star(r, 0, 0, 1e6, 24 + 8 * i))};
    const pl::polygon_set q = {pl::polygon(star(r, 3e5, 2e5, 9e5, 20 + 8 * i))};
    for (op o : ops)
      ab_boolean(fmt("stars #%d", i), p, q, o, false, ta, tb);
  }
  {
    // Polygons with holes, several per operand: an annulus-like contour pair and islands.
    const pl::ring outer = contour(r, 1e6, 400);
    pl::ring hole = contour(r, 4e5, 200);
    const pl::polygon_set p = {pl::polygon(outer, {hole}),
                               pl::polygon(convex(r, 2.2e6, 0, 3e5, 40))};
    const pl::polygon_set q = {
        pl::polygon(star(r, 5e5, 3e5, 1.1e6, 60), {convex(r, 6e5, 3e5, 2e5, 24)}),
        pl::polygon(convex(r, 2.3e6, 1e5, 2.5e5, 30))};
    for (op o : ops)
      ab_boolean("polygons with holes", p, q, o, false, ta, tb);
  }
  {
    // Two dense contours (ContourTiler's input: adjacent components on one slice).
    const pl::polygon_set p = {pl::polygon(contour(r, 1e6, 1500))};
    pl::ring c = contour(r, 9e5, 1200);
    for (pl::point &v : c)
      v = pl::point(v.x + 6e5, v.y + 1e5);
    const pl::polygon_set q = {pl::polygon(c)};
    for (op o : ops)
      ab_boolean("dense contours", p, q, o, false, ta, tb);
  }
  report_time("planar booleans (32 cases)", "cgal", ta, "planar", tb);
}

// --- offsets ------------------------------------------------------------------------------------

std::vector<Pwh> cgal_offset(const pl::ring &p, double d) {
  std::vector<Pwh> a;
  const CGAL::Polygon_2<Epick> pk = to_epick(pl::normalize(pl::polygon(p)).outer);
  const auto out =
      d > 0 ? CGAL::create_exterior_skeleton_and_offset_polygons_with_holes_2(d, pk, Epeck())
            : CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(-d, pk, Epeck());
  for (const auto &q : out)
    a.push_back(*q);
  return a;
}

int g_one_pass_differs = 0, g_one_pass_cases = 0;

// The straight skeleton's offset against wavefront_offset (asserted at every depth) and against a
// single mitred pass (asserted only where `one_pass_agrees`: convex polygons, and offsets before
// the first edge event; INFO otherwise).
void ab_offset(const std::string &name, const pl::ring &p, double d, bool one_pass_agrees,
               double &ta, double &tb, double &tc) {
  std::vector<Pwh> a;
  pl::polygon_set b, c;
  ta += ms_of([&] { a = cgal_offset(p, d); });
  tb += ms_of([&] { b = pl::wavefront_offset({pl::polygon(p)}, d); });
  pl::offset_params mitre;
  mitre.miter_limit = 1e6;
  tc += ms_of([&] { c = pl::offset({pl::polygon(p)}, d, mitre); });
  const verdict vb = compare(a, b, false, true);
  report(vb.ok, fmt("planar wavefront_offset %s d=%g", name.c_str(), d), vb.detail);
  const verdict vc = compare(a, c, false, true);
  ++g_one_pass_cases;
  if (!vc.ok)
    ++g_one_pass_differs;
  if (one_pass_agrees) {
    report(vc.ok, fmt("planar offset (one pass) %s d=%g", name.c_str(), d), vc.detail);
  } else {
    std::printf("INFO planar offset (one pass) %s d=%g: %s, %s\n", name.c_str(), d,
                vc.detail.c_str(), vc.ok ? "agrees" : "differs (past an edge event, by design)");
    std::fflush(stdout);
  }
}

void ab_offsets(rng &r) {
  double ta = 0, tb = 0, tc = 0;
  int n = 0;
  for (int i = 0; i < 4; ++i) {
    const pl::ring s = star(r, 0, 0, 1e6, 16 + 8 * i);
    const pl::ring c = convex(r, 0, 0, 1e6, 12 + 4 * i);
    // 5% of the radius comes before any edge event of these stars; 20% is past some.
    for (double d : {5e4, 2e5}) {
      const bool shallow = d < 1e5;
      ab_offset(fmt("star #%d outward", i), s, d, shallow, ta, tb, tc);
      ab_offset(fmt("star #%d inward", i), s, -d, shallow, ta, tb, tc);
      ab_offset(fmt("convex #%d outward", i), c, d, true, ta, tb, tc);
      ab_offset(fmt("convex #%d inward", i), c, -d, true, ta, tb, tc);
      n += 4;
    }
  }
  // Dense contours, offset as ContourTiler's intersection removal does (dilate, erode): short
  // edges collapse almost at once, so every depth is past edge events.
  for (int i = 0; i < 3; ++i) {
    const pl::ring c = contour(r, 1e6, 300 + 300 * i);
    for (double d : {2e3, 2e4, 1e5}) {
      ab_offset(fmt("dense contour #%d outward", i), c, d, false, ta, tb, tc);
      ab_offset(fmt("dense contour #%d inward", i), c, -d, false, ta, tb, tc);
      n += 2;
    }
  }
  report_time(fmt("planar wavefront_offset (%d cases)", n), "cgal", ta, "planar", tb);
  report_time(fmt("planar offset, one pass (%d cases)", n), "cgal", ta, "planar", tc);
  std::printf(
      "INFO planar offset (one pass): differs from the straight skeleton in %d of %d cases; "
      "wavefront_offset is asserted in all\n",
      g_one_pass_differs, g_one_pass_cases);
  std::fflush(stdout);
}

// --- Minkowski sums -----------------------------------------------------------------------------

void ab_minkowski(rng &r) {
  double ta = 0, tb = 0;
  for (int i = 0; i < 4; ++i) {
    const pl::ring c1 = convex(r, 0, 0, 1e5, 8 + 4 * i), c2 = convex(r, 0, 0, 1e6, 10 + 2 * i);
    const pl::ring s = star(r, 0, 0, 1e6, 24 + 8 * i);
    for (int k = 0; k < 2; ++k) {
      const pl::ring &other = k == 0 ? c2 : s;
      Pwh a;
      pl::polygon_set b;
      ta += ms_of([&] {
        a = CGAL::minkowski_sum_2(to_cgal(pl::normalize(pl::polygon(c1)).outer),
                                  to_cgal(pl::normalize(pl::polygon(other)).outer));
      });
      tb += ms_of([&] { b = pl::minkowski_sum(c1, other); });
      const verdict v = compare({a}, b, k == 0, true);
      report(v.ok, fmt("planar minkowski_sum %s #%d", k == 0 ? "convex+convex" : "convex+star", i),
             v.detail);
    }
  }
  report_time("planar minkowski_sum (8 cases)", "cgal", ta, "planar", tb);
}

} // namespace

void ab_planar(rng &r) {
  ab_booleans(r);
  ab_offsets(r);
  ab_minkowski(r);
}

} // namespace cgal_ab
