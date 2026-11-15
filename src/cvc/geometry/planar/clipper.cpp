/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::planar's Booleans, offsets and Minkowski sums on Clipper2 2.0 (Boost Software Licence 1.0,
// cvcpkg's clipper2), replacing CGAL's Polygon_set_2, straight-skeleton offsets and
// minkowski_sum_2 (inc/cvc/geometry/planar.h). Clipper2 is exact on int64 coordinates; new
// vertices (intersections, offset corners) round to the grid.

#include "planar_clipper.h"
#include "planar_internal.h"

#include <clipper2/clipper.h>
#include <cmath>
#include <cstddef>

namespace cvc {
namespace planar {

namespace detail {

namespace {

Clipper2Lib::Path64 to_path(const ring &r) {
  Clipper2Lib::Path64 p;
  p.reserve(r.size());
  for (const point &v : r)
    p.emplace_back(v.x, v.y);
  return p;
}

ring to_ring(const Clipper2Lib::Path64 &p) {
  ring r;
  r.reserve(p.size());
  for (const auto &v : p)
    r.emplace_back(v.x, v.y);
  return r;
}

// Every outer ring under `node` with its holes, the islands inside those holes as polygons of
// their own.
void collect(const Clipper2Lib::PolyPath64 &node, polygon_set &out) {
  for (std::size_t i = 0; i < node.Count(); ++i) {
    const Clipper2Lib::PolyPath64 &outer = *node.Child(i);
    polygon p(to_ring(outer.Polygon()));
    for (std::size_t j = 0; j < outer.Count(); ++j) {
      const Clipper2Lib::PolyPath64 &hole = *outer.Child(j);
      p.holes.push_back(to_ring(hole.Polygon()));
      collect(hole, out);
    }
    polygon n = normalize(p);
    if (!n.outer.empty())
      out.push_back(std::move(n));
  }
}

Clipper2Lib::FillRule rule_of(fill_rule r) {
  switch (r) {
  case fill_rule::even_odd:
    return Clipper2Lib::FillRule::EvenOdd;
  case fill_rule::positive:
    return Clipper2Lib::FillRule::Positive;
  case fill_rule::negative:
    return Clipper2Lib::FillRule::Negative;
  default:
    return Clipper2Lib::FillRule::NonZero;
  }
}

Clipper2Lib::JoinType join_of(join_type j) {
  switch (j) {
  case join_type::round:
    return Clipper2Lib::JoinType::Round;
  case join_type::square:
    return Clipper2Lib::JoinType::Square;
  case join_type::bevel:
    return Clipper2Lib::JoinType::Bevel;
  default:
    return Clipper2Lib::JoinType::Miter;
  }
}

polygon_set boolean(const char *fn, Clipper2Lib::ClipType op, const polygon_set &a,
                    const polygon_set &b) {
  check_range(fn, a);
  check_range(fn, b);
  Clipper2Lib::PolyTree64 tree;
  Clipper2Lib::BooleanOp(op, Clipper2Lib::FillRule::NonZero, to_paths(a), to_paths(b), tree);
  return from_tree(tree);
}

} // namespace

Clipper2Lib::Paths64 to_paths(const polygon_set &s) {
  Clipper2Lib::Paths64 out;
  for (const polygon &p : s) {
    const polygon n = normalize(p);
    if (n.outer.empty())
      continue;
    out.push_back(to_path(n.outer));
    for (const ring &h : n.holes)
      out.push_back(to_path(h));
  }
  return out;
}

polygon_set from_tree(const Clipper2Lib::PolyTree64 &tree) {
  polygon_set out;
  collect(tree, out);
  return out;
}

polygon_set mitred_offset(const Clipper2Lib::Paths64 &paths, double delta, double miter_limit) {
  Clipper2Lib::ClipperOffset off(miter_limit);
  off.AddPaths(paths, Clipper2Lib::JoinType::Miter, Clipper2Lib::EndType::Polygon);
  Clipper2Lib::PolyTree64 tree;
  off.Execute(delta, tree);
  return from_tree(tree);
}

} // namespace detail

polygon_set unite(const polygon_set &a, const polygon_set &b) {
  return detail::boolean("planar::unite", Clipper2Lib::ClipType::Union, a, b);
}

polygon_set intersect(const polygon_set &a, const polygon_set &b) {
  return detail::boolean("planar::intersect", Clipper2Lib::ClipType::Intersection, a, b);
}

polygon_set subtract(const polygon_set &a, const polygon_set &b) {
  return detail::boolean("planar::subtract", Clipper2Lib::ClipType::Difference, a, b);
}

polygon_set exclusive_or(const polygon_set &a, const polygon_set &b) {
  return detail::boolean("planar::exclusive_or", Clipper2Lib::ClipType::Xor, a, b);
}

polygon_set resolve(const std::vector<ring> &rings, fill_rule rule) {
  Clipper2Lib::Paths64 paths;
  for (const ring &r : rings) {
    detail::check_range("planar::resolve", r);
    if (r.size() >= 3)
      paths.push_back(detail::to_path(r));
  }
  Clipper2Lib::PolyTree64 tree;
  Clipper2Lib::BooleanOp(Clipper2Lib::ClipType::Union, detail::rule_of(rule), paths, {}, tree);
  return detail::from_tree(tree);
}

polygon_set offset(const polygon_set &s, double delta, const offset_params &params) {
  detail::check_range("planar::offset", s);
  if (!std::isfinite(delta))
    throw planar_error("planar::offset: the distance is not finite");
  if (!(params.miter_limit > 0) || !(params.arc_tolerance >= 0))
    throw planar_error("planar::offset: miter_limit must be positive and arc_tolerance not "
                       "negative");
  Clipper2Lib::ClipperOffset off(params.miter_limit, params.arc_tolerance);
  off.AddPaths(detail::to_paths(s), detail::join_of(params.join), Clipper2Lib::EndType::Polygon);
  Clipper2Lib::PolyTree64 tree;
  off.Execute(delta, tree);
  polygon_set out = detail::from_tree(tree);
  detail::check_result_range("planar::offset", out);
  return out;
}

polygon_set minkowski_sum(const ring &a, const ring &b) {
  detail::check_range("planar::minkowski_sum", a);
  detail::check_range("planar::minkowski_sum", b);
  const ring ca = clean(a), cb = clean(b);
  if (ca.empty() || cb.empty())
    return {};
  // MinkowskiSum sweeps a along b's boundary. The sweep misses the inside of b, so b moved by a
  // vertex of a, and a moved by a vertex of b, are added; the union is the whole sum.
  const Clipper2Lib::Path64 pa = detail::to_path(ca), pb = detail::to_path(cb);
  Clipper2Lib::Paths64 parts = Clipper2Lib::MinkowskiSum(pa, pb, true);
  Clipper2Lib::Path64 bt = pb, at = pa;
  for (auto &v : bt)
    v = Clipper2Lib::Point64(v.x + ca[0].x, v.y + ca[0].y);
  for (auto &v : at)
    v = Clipper2Lib::Point64(v.x + cb[0].x, v.y + cb[0].y);
  parts.push_back(bt);
  parts.push_back(at);
  Clipper2Lib::PolyTree64 tree;
  Clipper2Lib::BooleanOp(Clipper2Lib::ClipType::Union, Clipper2Lib::FillRule::NonZero, parts, {},
                         tree);
  polygon_set out = detail::from_tree(tree);
  detail::check_result_range("planar::minkowski_sum", out);
  return out;
}

} // namespace planar
} // namespace cvc
