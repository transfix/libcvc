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
//   Delaunay_triangulation_2                        delaunay (identical triangles)
//   Constrained_Delaunay_triangulation_2, the       triangulate (identical triangles)
//     faces inside the polygon
//   approx_convex_partition_2 (and Greene's)        convex_partition (valid by CGAL's own check)
//   natural_neighbor_coordinates_2,                 sibson_coordinates, sibson_interpolate
//     linear_interpolation
//   create_interior_straight_skeleton_2             medial_axis (the same on convex polygons;
//                                                     Hausdorff distance reported elsewhere)
//
// Results are compared by area, by the counts of outer rings and holes, and by the area of the
// symmetric difference between the two results (CGAL's exact vertices rounded to the grid): within
// a grid unit per unit of boundary, which is what rounding new vertices to the grid allows.

#include "cgal_ab.h"

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>
#include <CGAL/create_straight_skeleton_2.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/natural_neighbor_coordinates_2.h>
#include <CGAL/partition_2.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/geometry/planar.h>
#include <iterator>
#include <limits>
#include <list>
#include <map>
#include <string>
#include <utility>
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

// --- triangulations -----------------------------------------------------------------------------

using Vb2 = CGAL::Triangulation_vertex_base_with_info_2<int, Epick>;
using Fb2 = CGAL::Constrained_triangulation_face_base_2<Epick>;
using Tds2 = CGAL::Triangulation_data_structure_2<Vb2, Fb2>;
using CDT2 = CGAL::Constrained_Delaunay_triangulation_2<Epick, Tds2, CGAL::Exact_predicates_tag>;
using DtVb = CGAL::Triangulation_vertex_base_with_info_2<int, Epick>;
using DtTds = CGAL::Triangulation_data_structure_2<DtVb>;
using DT2 = CGAL::Delaunay_triangulation_2<Epick, DtTds>;
using tri_set = std::vector<std::array<int, 3>>;

tri_set sorted_tris(const std::vector<std::array<int, 3>> &ts) {
  tri_set out;
  for (std::array<int, 3> t : ts) {
    std::sort(t.begin(), t.end());
    out.push_back(t);
  }
  std::sort(out.begin(), out.end());
  return out;
}

// ContourTiler's use: a CDT of a polygon's rings, keeping the faces inside it.
tri_set cgal_polygon_cdt(const pl::polygon &p) {
  std::vector<pl::point> pts;
  std::vector<std::pair<int, int>> edges;
  auto add = [&](const pl::ring &r) {
    const int base = int(pts.size()), n = int(r.size());
    pts.insert(pts.end(), r.begin(), r.end());
    for (int i = 0; i < n; ++i)
      edges.emplace_back(base + i, base + (i + 1) % n);
  };
  add(p.outer);
  for (const pl::ring &h : p.holes)
    add(h);
  std::vector<std::pair<CDT2::Point, int>> in;
  for (int i = 0; i < int(pts.size()); ++i)
    in.emplace_back(CDT2::Point(double(pts[i].x), double(pts[i].y)), i);
  CDT2 cdt;
  cdt.insert(in.begin(), in.end());
  std::vector<CDT2::Vertex_handle> vh(pts.size());
  for (auto v = cdt.finite_vertices_begin(); v != cdt.finite_vertices_end(); ++v)
    vh[std::size_t(v->info())] = v;
  for (const auto &e : edges)
    cdt.insert_constraint(vh[std::size_t(e.first)], vh[std::size_t(e.second)]);
  const CGAL::Polygon_2<Epick> outer = to_epick(p.outer);
  std::vector<CGAL::Polygon_2<Epick>> holes;
  for (const pl::ring &h : p.holes)
    holes.push_back(to_epick(h));
  std::vector<std::array<int, 3>> out;
  for (auto f = cdt.finite_faces_begin(); f != cdt.finite_faces_end(); ++f) {
    const Epick::Point_2 c =
        CGAL::centroid(f->vertex(0)->point(), f->vertex(1)->point(), f->vertex(2)->point());
    bool inside = outer.bounded_side(c) == CGAL::ON_BOUNDED_SIDE;
    for (const auto &h : holes)
      inside = inside && h.bounded_side(c) != CGAL::ON_BOUNDED_SIDE;
    if (inside)
      out.push_back({f->vertex(0)->info(), f->vertex(1)->info(), f->vertex(2)->info()});
  }
  return sorted_tris(out);
}

void ab_triangulations(rng &r) {
  double ta = 0, tb = 0;
  for (int i = 0; i < 4; ++i) {
    std::vector<pl::point> p;
    for (int k = 0; k < 2000 * (i + 1) * (i + 1); ++k)
      p.emplace_back(pl::coord(std::llround(r.uniform(-1e6, 1e6))),
                     pl::coord(std::llround(r.uniform(-1e6, 1e6))));
    tri_set a, b;
    ta += ms_of([&] {
      std::vector<std::pair<DT2::Point, int>> in;
      for (int k = 0; k < int(p.size()); ++k)
        in.emplace_back(DT2::Point(double(p[k].x), double(p[k].y)), k);
      DT2 dt(in.begin(), in.end());
      std::vector<std::array<int, 3>> ts;
      for (auto f = dt.finite_faces_begin(); f != dt.finite_faces_end(); ++f)
        ts.push_back({f->vertex(0)->info(), f->vertex(1)->info(), f->vertex(2)->info()});
      a = sorted_tris(ts);
    });
    tb += ms_of([&] { b = sorted_tris(pl::delaunay(p).triangles); });
    report(a == b, fmt("planar delaunay uniform n=%zu", p.size()),
           fmt("cgal %zu triangles, planar %zu, %s", a.size(), b.size(),
               a == b ? "identical" : "different"));
  }
  report_time("planar delaunay (4 cases)", "cgal", ta, "planar", tb);

  ta = tb = 0;
  auto ab_poly = [&](const std::string &name, const pl::polygon &p) {
    tri_set a, b;
    ta += ms_of([&] { a = cgal_polygon_cdt(p); });
    tb += ms_of([&] { b = sorted_tris(pl::triangulate(p).triangles); });
    report(a == b, "planar triangulate " + name,
           fmt("cgal %zu triangles inside, planar %zu, %s", a.size(), b.size(),
               a == b ? "identical" : "different"));
  };
  for (int i = 0; i < 3; ++i)
    ab_poly(fmt("star #%d", i), pl::polygon(star(r, 0, 0, 1e6, 40 + 80 * i)));
  for (int i = 0; i < 3; ++i)
    ab_poly(fmt("dense contour #%d", i), pl::polygon(contour(r, 1e6, 500 + 500 * i)));
  {
    pl::polygon ph(contour(r, 1e6, 600));
    ph.holes.push_back(convex(r, 1e5, 0, 2e5, 40));
    pl::ring h2 = star(r, -3e5, 2e5, 1.5e5, 30);
    std::reverse(h2.begin(), h2.end());
    ph.holes.push_back(h2);
    ab_poly("contour with two holes", ph);
  }
  report_time("planar triangulate (7 cases)", "cgal", ta, "planar", tb);
}

// --- convex partitions
// ----------------------------------------------------------------------------

using PT = CGAL::Partition_traits_2<Epick>;

// A piece as its vertices from the lowest-then-leftmost, so equal pieces compare equal.
std::vector<pl::point> canonical(std::vector<pl::point> v) {
  const auto m = std::min_element(v.begin(), v.end(), [](const pl::point &a, const pl::point &b) {
    return a.y < b.y || (a.y == b.y && a.x < b.x);
  });
  std::rotate(v.begin(), m, v.end());
  return v;
}

std::vector<std::vector<pl::point>> canonical(const std::vector<pl::ring> &pieces) {
  std::vector<std::vector<pl::point>> out;
  for (const pl::ring &r : pieces)
    out.push_back(canonical(std::vector<pl::point>(r.begin(), r.end())));
  std::sort(out.begin(), out.end());
  return out;
}

std::vector<pl::ring> of_partition(const std::list<PT::Polygon_2> &ps) {
  std::vector<pl::ring> out;
  for (const auto &p : ps) {
    pl::ring r;
    for (auto v = p.vertices_begin(); v != p.vertices_end(); ++v)
      r.emplace_back(pl::coord(std::llround(v->x())), pl::coord(std::llround(v->y())));
    out.push_back(r);
  }
  return out;
}

int g_partition_same = 0, g_partition_cases = 0;

// ContourTiler's partition(): approx_convex_partition_2 (Hertel-Mehlhorn on CGAL's constrained
// triangulation), or Greene's if that is not valid. cvc::planar's must be valid (by CGAL's own
// convex_partition_is_valid_2) and within Hertel-Mehlhorn's bound; how often it equals CGAL's,
// piece for piece, is reported.
void ab_partition(const std::string &name, const pl::ring &ring_in, double &ta, double &tb) {
  const pl::ring rr = pl::normalize(pl::polygon(ring_in)).outer;
  std::vector<PT::Point_2> pts;
  for (const pl::point &v : rr)
    pts.emplace_back(double(v.x), double(v.y));
  std::list<PT::Polygon_2> hm, greene;
  std::vector<pl::ring> ours;
  ta += ms_of(
      [&] { CGAL::approx_convex_partition_2(pts.begin(), pts.end(), std::back_inserter(hm)); });
  CGAL::greene_approx_convex_partition_2(pts.begin(), pts.end(), std::back_inserter(greene));
  tb += ms_of([&] { ours = pl::convex_partition(pl::polygon(rr)); });
  std::list<PT::Polygon_2> ours_cgal;
  for (const pl::ring &q : ours) {
    PT::Polygon_2 cp;
    for (const pl::point &v : q)
      cp.push_back(PT::Point_2(double(v.x), double(v.y)));
    ours_cgal.push_back(cp);
  }
  const bool valid =
      CGAL::convex_partition_is_valid_2(pts.begin(), pts.end(), ours_cgal.begin(), ours_cgal.end());
  int reflex = 0;
  for (std::size_t i = 0; i < rr.size(); ++i)
    reflex +=
        pl::orientation(rr[(i + rr.size() - 1) % rr.size()], rr[i], rr[(i + 1) % rr.size()]) < 0;
  const bool within = int(ours.size()) <= 2 * reflex + 1;
  const bool same = canonical(ours) == canonical(of_partition(hm));
  ++g_partition_cases;
  g_partition_same += same;
  report(valid && within, "planar convex_partition " + name,
         fmt("planar %zu pieces (valid: %s; bound 2r+1 = %d), cgal approx (Hertel-Mehlhorn) %zu, "
             "greene %zu; %s CGAL's Hertel-Mehlhorn",
             ours.size(), valid ? "yes" : "no", 2 * reflex + 1, hm.size(), greene.size(),
             same ? "identical to" : "differs from"));
}

void ab_partitions(rng &r) {
  double ta = 0, tb = 0;
  for (int i = 0; i < 4; ++i)
    ab_partition(fmt("star #%d", i), star(r, 0, 0, 1e6, 20 + 30 * i), ta, tb);
  for (int i = 0; i < 4; ++i)
    ab_partition(fmt("dense contour #%d", i), contour(r, 1e6, 200 + 200 * i), ta, tb);
  report_time("planar convex_partition (8 cases)", "cgal", ta, "planar", tb);
  std::printf("INFO planar convex_partition: identical to CGAL's approx_convex_partition_2 in %d "
              "of %d cases\n",
              g_partition_same, g_partition_cases);
  std::fflush(stdout);
}

// --- Sibson coordinates
// ---------------------------------------------------------------------------

void ab_sibson(rng &r) {
  for (int round = 0; round < 3; ++round) {
    const int n = 200 * (round + 1) * (round + 1);
    std::vector<pl::point> p;
    for (int k = 0; k < n; ++k)
      p.emplace_back(pl::coord(std::llround(r.uniform(-1e6, 1e6))),
                     pl::coord(std::llround(r.uniform(-1e6, 1e6))));
    std::vector<double> f;
    for (const pl::point &v : p)
      f.push_back(std::sin(double(v.x) * 3e-6) + std::cos(double(v.y) * 2e-6));
    std::map<std::pair<double, double>, int> index;
    for (int k = 0; k < n; ++k)
      index.emplace(std::make_pair(double(p[k].x), double(p[k].y)), k);
    std::vector<Epick::Point_2> cp;
    for (const pl::point &v : p)
      cp.emplace_back(double(v.x), double(v.y));
    CGAL::Delaunay_triangulation_2<Epick> dt(cp.begin(), cp.end());
    const pl::triangulation ours = pl::delaunay(p);
    std::vector<pl::pointd> qs;
    for (int k = 0; k < 500; ++k)
      qs.emplace_back(r.uniform(-9e5, 9e5), r.uniform(-9e5, 9e5));
    double worst_w = 0, worst_f = 0, ta = 0, tb = 0;
    int compared = 0, mismatched = 0;
    for (const pl::pointd &q : qs) {
      std::vector<std::pair<Epick::Point_2, Epick::FT>> coords;
      bool ok = false;
      Epick::FT norm = 0;
      ta += ms_of([&] {
        const auto res = CGAL::natural_neighbor_coordinates_2(dt, Epick::Point_2(q.x, q.y),
                                                              std::back_inserter(coords));
        norm = res.second;
        ok = res.third;
      });
      std::vector<pl::natural_neighbor> w;
      tb += ms_of([&] { w = pl::sibson_coordinates(ours, q); });
      if (!ok || w.empty()) {
        mismatched += ok != !w.empty();
        continue;
      }
      std::map<int, double> want;
      double fa = 0;
      for (const auto &c : coords) {
        const int k = index.at({c.first.x(), c.first.y()});
        want[k] = c.second / norm;
        fa += want[k] * f[std::size_t(k)];
      }
      double fb = 0;
      for (const pl::natural_neighbor &nn : w) {
        worst_w = std::max(worst_w, std::fabs(nn.weight - want[nn.vertex]));
        want.erase(nn.vertex);
        fb += nn.weight * f[std::size_t(nn.vertex)];
      }
      for (const auto &rest : want)
        worst_w = std::max(worst_w, rest.second); // a neighbour cvc::planar left out
      worst_f = std::max(worst_f, std::fabs(fa - fb));
      ++compared;
    }
    const bool pass = worst_w <= 1e-9 && worst_f <= 1e-9 && mismatched == 0;
    report(pass, fmt("planar sibson_coordinates n=%d, 500 queries", n),
           fmt("%d compared; largest weight difference %.3g, interpolated value difference %.3g; "
               "inside/outside disagreements %d",
               compared, worst_w, worst_f, mismatched));
    report_time(fmt("planar sibson_coordinates n=%d", n), "cgal", ta, "planar", tb);
  }
}

// --- the medial axis against the straight skeleton ----------------------------------------------

using seg2 = std::pair<pl::pointd, pl::pointd>;

double seg_distance(const pl::pointd &q, const seg2 &s) {
  const double dx = s.second.x - s.first.x, dy = s.second.y - s.first.y, l2 = dx * dx + dy * dy;
  double t = l2 > 0 ? ((q.x - s.first.x) * dx + (q.y - s.first.y) * dy) / l2 : 0;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(q.x - (s.first.x + t * dx), q.y - (s.first.y + t * dy));
}

// The largest distance from a point of `a` (each segment sampled every `step`) to `b`.
double directed_hausdorff(const std::vector<seg2> &a, const std::vector<seg2> &b, double step) {
  double worst = 0;
  for (const seg2 &s : a) {
    const double len = std::hypot(s.second.x - s.first.x, s.second.y - s.first.y);
    const int n = std::max(1, int(std::ceil(len / step)));
    for (int k = 0; k <= n; ++k) {
      const double t = double(k) / n;
      const pl::pointd q(s.first.x + t * (s.second.x - s.first.x),
                         s.first.y + t * (s.second.y - s.first.y));
      double best = std::numeric_limits<double>::infinity();
      for (const seg2 &o : b)
        best = std::min(best, seg_distance(q, o));
      worst = std::max(worst, best);
    }
  }
  return worst;
}

// CGAL's interior straight skeleton's bisectors (what ContourTiler's skeleton.cpp built), each
// once.
std::vector<seg2> cgal_skeleton(const pl::ring &r, int &branch_nodes) {
  const pl::ring rr = pl::normalize(pl::polygon(r)).outer;
  const CGAL::Polygon_2<Epick> poly = to_epick(rr);
  const auto ss = CGAL::create_interior_straight_skeleton_2(poly.vertices_begin(),
                                                            poly.vertices_end(), Epick());
  std::vector<seg2> out;
  branch_nodes = 0;
  for (auto v = ss->vertices_begin(); v != ss->vertices_end(); ++v)
    branch_nodes += v->is_skeleton();
  for (auto h = ss->halfedges_begin(); h != ss->halfedges_end(); ++h) {
    if (!h->is_bisector() || &*h > &*h->opposite())
      continue;
    const auto &p = h->vertex()->point(), &q = h->opposite()->vertex()->point();
    out.push_back({pl::pointd(p.x(), p.y()), pl::pointd(q.x(), q.y())});
  }
  return out;
}

std::vector<seg2> axis_segments(const pl::medial_axis_graph &g) {
  std::vector<seg2> out;
  for (const auto &e : g.edges)
    out.push_back({g.vertices[std::size_t(e[0])], g.vertices[std::size_t(e[1])]});
  return out;
}

// On a convex polygon the medial axis is the straight skeleton (asserted: a Hausdorff distance of
// a grid unit or less). Elsewhere they differ by design: the skeleton's bisector runs into each
// reflex vertex, where the medial axis curves round it. There the Hausdorff distance is reported
// (INFO), relative to the polygon's size, with both sides' times.
void ab_medial_axis(const std::string &name, const pl::ring &r, bool convex_case, double &ta,
                    double &tb) {
  std::vector<seg2> a;
  int nodes = 0;
  pl::medial_axis_graph g;
  ta += ms_of([&] { a = cgal_skeleton(r, nodes); });
  tb += ms_of([&] { g = pl::medial_axis(pl::polygon(r)); });
  const std::vector<seg2> b = axis_segments(g);
  double size = 0;
  for (const pl::point &v : r)
    size = std::max(size, std::hypot(double(v.x), double(v.y)));
  const double step = size / 2000;
  const double ab = directed_hausdorff(a, b, step), ba = directed_hausdorff(b, a, step);
  const double h = std::max(ab, ba);
  int branches = 0;
  {
    std::vector<int> degree(g.vertices.size(), 0);
    for (const auto &e : g.edges)
      ++degree[std::size_t(e[0])], ++degree[std::size_t(e[1])];
    for (int d : degree)
      branches += d >= 3;
  }
  const std::string detail =
      fmt("Hausdorff distance %.3g (%.2g%% of the size: skeleton to axis %.2g%%, axis to skeleton "
          "%.2g%%); skeleton %zu bisectors, %d nodes; medial axis %zu edges, %d branch points",
          h, 100 * h / size, 100 * ab / size, 100 * ba / size, a.size(), nodes, b.size(), branches);
  if (convex_case) {
    report(h <= 1.0, "planar medial_axis " + name, detail);
  } else {
    std::printf("INFO planar medial_axis %s: %s (the medial axis curves round reflex vertices, the "
                "skeleton runs into them)\n",
                name.c_str(), detail.c_str());
    std::fflush(stdout);
  }
}

void ab_medial_axes(rng &r) {
  double ta = 0, tb = 0;
  for (int i = 0; i < 4; ++i)
    ab_medial_axis(fmt("convex #%d", i), convex(r, 0, 0, 1e6, 8 + 8 * i), true, ta, tb);
  for (int i = 0; i < 3; ++i)
    ab_medial_axis(fmt("star #%d", i), star(r, 0, 0, 1e6, 16 + 16 * i), false, ta, tb);
  for (int i = 0; i < 2; ++i)
    ab_medial_axis(fmt("dense contour #%d", i), contour(r, 1e6, 200 + 200 * i), false, ta, tb);
  report_time("planar medial_axis (9 cases)", "cgal skeleton", ta, "planar", tb);
}

} // namespace

void ab_planar(rng &r) {
  ab_booleans(r);
  ab_offsets(r);
  ab_minkowski(r);
  ab_triangulations(r);
  ab_partitions(r);
  ab_sibson(r);
  ab_medial_axes(r);
}

} // namespace cgal_ab
