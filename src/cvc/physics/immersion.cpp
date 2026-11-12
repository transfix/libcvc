/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The columns of a body under water (immersion.h). Plain IEEE arithmetic and sqrt only, built
// without FMA contraction (the physics exact-math list in src/cvc/CMakeLists.txt), so a column is
// the same bits on every platform.

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/physics/immersion.h>
#include <cvc/volume/volume.h>
#include <stdexcept>
#include <string>
#include <variant>

namespace cvc {
namespace physics {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

bool finite(const vec3 &v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

vec3 cross(const vec3 &a, const vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// The rotation matrix of a unit quaternion, row-major: the field step's (field_step.cpp).
struct mat3 {
  double m[3][3];
  vec3 operator*(const vec3 &v) const {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
  }
  vec3 inverse(const vec3 &v) const {
    return {m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z,
            m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z,
            m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z};
  }
};

mat3 rotation(const quat &q) {
  const double w = q.w, x = q.x, y = q.y, z = q.z;
  return {{{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
           {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
           {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}}};
}

std::uint64_t fnv(const void *data, std::size_t n, std::uint64_t h) {
  const auto *p = static_cast<const unsigned char *>(data);
  for (std::size_t i = 0; i < n; ++i) {
    h ^= p[i];
    h *= 1099511628211ull;
  }
  return h;
}
std::uint64_t fnv(double v, std::uint64_t h) {
  if (v == 0.0)
    v = 0.0; // -0 and +0 alike
  return fnv(&v, sizeof v, h);
}

// The faces of the convex hull of `pts`: every plane through three of the points with all the
// points on one side of it, once each, outward (n . p <= offset for every point). Brute force,
// O(n^4) at worst but quick in practice (a triple fails at the first point on its far side): a
// hull is compiled once per body. The order is the triples', so the planes are the same on every
// platform.
std::vector<std::array<double, 4>> hull_planes(const std::vector<vec3> &pts) {
  if (pts.size() < 4)
    throw std::invalid_argument("immersion_body: a convex hull needs at least 4 points");
  vec3 lo = pts.front(), hi = lo;
  for (const vec3 &p : pts) {
    if (!finite(p))
      throw std::invalid_argument("immersion_body: a hull point is not finite");
    lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
    hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
  }
  const double ext = std::max(hi.x - lo.x, std::max(hi.y - lo.y, hi.z - lo.z));
  if (!(ext > 0.0))
    throw std::invalid_argument("immersion_body: the hull has no volume");
  const double eps = 1e-9 * ext;
  std::vector<std::array<double, 4>> planes;
  const std::size_t n = pts.size();
  for (std::size_t i = 0; i < n; ++i)
    for (std::size_t j = i + 1; j < n; ++j)
      for (std::size_t k = j + 1; k < n; ++k) {
        vec3 nn = cross(pts[j] - pts[i], pts[k] - pts[i]);
        const double len = std::sqrt(dot(nn, nn));
        if (!(len > 1e-12 * ext * ext))
          continue; // collinear
        nn = nn * (1.0 / len);
        double off = dot(nn, pts[i]);
        bool above = false, below = false;
        for (std::size_t m = 0; m < n && !(above && below); ++m) {
          const double d = dot(nn, pts[m]) - off;
          above = above || d > eps;
          below = below || d < -eps;
        }
        if (above && below)
          continue;
        if (above) { // every point is on the +n side: flip, so n points out
          nn = nn * -1.0;
          off = -off;
        }
        bool seen = false;
        for (const auto &q : planes)
          if (std::fabs(q[0] - nn.x) < 1e-9 && std::fabs(q[1] - nn.y) < 1e-9 &&
              std::fabs(q[2] - nn.z) < 1e-9 && std::fabs(q[3] - off) < eps) {
            seen = true;
            break;
          }
        if (!seen)
          planes.push_back({nn.x, nn.y, nn.z, off});
      }
  if (planes.size() < 4)
    throw std::invalid_argument("immersion_body: the hull has no volume");
  return planes;
}

// The outline moved to a pose: what each column reads.
struct posed_outline {
  immersion_body::kind kind;
  vec3 centre;                               // box, sphere
  double half[3] = {};                       // box
  double rz[3] = {};                         // box: R^T e_z, the vertical in the body's frame
  double radius = 0.0;                       // sphere
  mat3 r;                                    // box
  std::vector<std::array<double, 4>> planes; // convex: world n and offset
  std::vector<vec3> verts;                   // mesh: world vertices
  const std::vector<std::array<std::uint32_t, 3>> *tris = nullptr;
};

void pose_outline(immersion_body::kind kind, const vec3 &half, double radius,
                  const std::vector<std::array<double, 4>> &planes, const std::vector<vec3> &points,
                  const std::vector<std::array<std::uint32_t, 3>> &tris, const vec3 &position,
                  const quat &rot, posed_outline &out) {
  if (!finite(position) || !std::isfinite(rot.w) || !std::isfinite(rot.x) ||
      !std::isfinite(rot.y) || !std::isfinite(rot.z))
    throw std::invalid_argument("immersion: the pose is not finite");
  out.kind = kind;
  out.centre = position;
  out.r = rotation(rot);
  switch (kind) {
  case immersion_body::kind::box:
    out.half[0] = half.x, out.half[1] = half.y, out.half[2] = half.z;
    out.rz[0] = out.r.m[2][0], out.rz[1] = out.r.m[2][1], out.rz[2] = out.r.m[2][2];
    break;
  case immersion_body::kind::sphere:
    out.radius = radius;
    break;
  case immersion_body::kind::convex:
    out.planes.resize(planes.size());
    for (std::size_t i = 0; i < planes.size(); ++i) {
      const vec3 n = out.r * vec3{planes[i][0], planes[i][1], planes[i][2]};
      out.planes[i] = {n.x, n.y, n.z, planes[i][3] + dot(n, position)};
    }
    break;
  case immersion_body::kind::mesh:
    out.verts.resize(points.size());
    for (std::size_t i = 0; i < points.size(); ++i)
      out.verts[i] = position + out.r * points[i];
    out.tris = &tris;
    break;
  }
}

// The edge function of p against the edge u -> v, exactly antisymmetric in u and v: it is always
// computed from the lexicographically smaller end, so the two triangles sharing an edge see
// exactly opposite values and a point on it is counted once.
double edge_fn(double ux, double uy, double vx, double vy, double px, double py) {
  if (ux < vx || (ux == vx && uy < vy))
    return (vx - ux) * (py - uy) - (vy - uy) * (px - ux);
  return -((ux - vx) * (py - vy) - (uy - vy) * (px - vx));
}

// A counter-clockwise triangle's edge u -> v owns the points on it when it is a left edge (going
// down) or a top edge (horizontal, going left): the usual top-left rule, so a point on an edge
// shared by two triangles of one layer is inside exactly one of them.
bool owns(double ux, double uy, double vx, double vy) { return vy < uy || (vy == uy && vx < ux); }

// The length of the vertical line at (x, y) inside the posed outline, between floor and top.
double posed_column(const posed_outline &p, double x, double y, double floor, double top) {
  if (!(top > floor))
    return 0.0;
  double z0 = -kInf, z1 = kInf;
  switch (p.kind) {
  case immersion_body::kind::box: {
    const vec3 a = p.r.inverse({x - p.centre.x, y - p.centre.y, -p.centre.z});
    const double av[3] = {a.x, a.y, a.z};
    for (int k = 0; k < 3; ++k) {
      const double d = p.rz[k], h = p.half[k];
      if (d == 0.0) {
        if (std::fabs(av[k]) > h)
          return 0.0;
        continue;
      }
      double lo = (-h - av[k]) / d, hi = (h - av[k]) / d;
      if (lo > hi)
        std::swap(lo, hi);
      z0 = std::max(z0, lo);
      z1 = std::min(z1, hi);
    }
    break;
  }
  case immersion_body::kind::sphere: {
    const double rx = x - p.centre.x, ry = y - p.centre.y;
    const double q = p.radius * p.radius - (rx * rx + ry * ry);
    if (!(q > 0.0))
      return 0.0;
    const double s = std::sqrt(q);
    z0 = p.centre.z - s;
    z1 = p.centre.z + s;
    break;
  }
  case immersion_body::kind::convex:
    for (const auto &pl : p.planes) {
      // n . (x, y, z) <= offset
      const double rhs = pl[3] - pl[0] * x - pl[1] * y;
      if (pl[2] > 1e-12)
        z1 = std::min(z1, rhs / pl[2]);
      else if (pl[2] < -1e-12)
        z0 = std::max(z0, rhs / pl[2]);
      else if (rhs < 0.0)
        return 0.0;
    }
    break;
  case immersion_body::kind::mesh: {
    // Each face over (x, y) is a crossing: entering where it looks down, leaving where it looks
    // up. The inside between floor and top is the sum of the leaving heights less the entering
    // ones, each clamped to [floor, top] and taken from top (which cancels between a pair, and
    // keeps a crossing whose partner was missed by round-off from adding a height above the
    // water). Where faces fold over at the outline's edge, both are counted or neither: they
    // traverse their shared edge the same way, so the edge rule below decides both alike.
    double len = 0.0;
    for (const auto &t : *p.tris) {
      const vec3 *a = &p.verts[t[0]], *b = &p.verts[t[1]], *c = &p.verts[t[2]];
      if (x < std::min(a->x, std::min(b->x, c->x)) || x > std::max(a->x, std::max(b->x, c->x)) ||
          y < std::min(a->y, std::min(b->y, c->y)) || y > std::max(a->y, std::max(b->y, c->y)))
        continue;
      const double area = edge_fn(a->x, a->y, b->x, b->y, c->x, c->y);
      if (area == 0.0)
        continue;                               // vertical: no crossing
      const double s = area > 0.0 ? 1.0 : -1.0; // looks up (leaves) or down (enters)
      if (area < 0.0)
        std::swap(b, c); // counter-clockwise in x, y from here
      const double w0 = edge_fn(b->x, b->y, c->x, c->y, x, y);
      const double w1 = edge_fn(c->x, c->y, a->x, a->y, x, y);
      const double w2 = edge_fn(a->x, a->y, b->x, b->y, x, y);
      if (w0 < 0.0 || w1 < 0.0 || w2 < 0.0)
        continue;
      if ((w0 == 0.0 && !owns(b->x, b->y, c->x, c->y)) ||
          (w1 == 0.0 && !owns(c->x, c->y, a->x, a->y)) ||
          (w2 == 0.0 && !owns(a->x, a->y, b->x, b->y)))
        continue;
      const double z = (w0 * a->z + w1 * b->z + w2 * c->z) / (w0 + w1 + w2);
      len += s * (std::min(top, std::max(floor, z)) - top);
    }
    return len > 0.0 ? len : 0.0;
  }
  }
  z0 = std::max(z0, floor);
  z1 = std::min(z1, top);
  return z1 > z0 ? z1 - z0 : 0.0;
}

void posed_bounds(const posed_outline &p, vec3 &lo, vec3 &hi) {
  switch (p.kind) {
  case immersion_body::kind::box: {
    vec3 e;
    for (int row = 0; row < 3; ++row) {
      const double v = std::fabs(p.r.m[row][0]) * p.half[0] + std::fabs(p.r.m[row][1]) * p.half[1] +
                       std::fabs(p.r.m[row][2]) * p.half[2];
      (row == 0 ? e.x : row == 1 ? e.y : e.z) = v;
    }
    lo = p.centre - e;
    hi = p.centre + e;
    return;
  }
  case immersion_body::kind::sphere:
    lo = p.centre - vec3{p.radius, p.radius, p.radius};
    hi = p.centre + vec3{p.radius, p.radius, p.radius};
    return;
  case immersion_body::kind::convex:
  case immersion_body::kind::mesh:
    break;
  }
  const std::vector<vec3> &v = p.verts;
  lo = hi = v.front();
  for (const vec3 &q : v) {
    lo = {std::min(lo.x, q.x), std::min(lo.y, q.y), std::min(lo.z, q.z)};
    hi = {std::max(hi.x, q.x), std::max(hi.y, q.y), std::max(hi.z, q.z)};
  }
}

} // namespace

immersion_body::immersion_body(const shape &s) {
  if (const box *b = std::get_if<box>(&s)) {
    if (!finite(b->half_extents) || !(b->half_extents.x > 0.0) || !(b->half_extents.y > 0.0) ||
        !(b->half_extents.z > 0.0))
      throw std::invalid_argument("immersion_body: a box's half extents must be finite and > 0");
    _kind = kind::box;
    _half = b->half_extents;
  } else if (const sphere *sp = std::get_if<sphere>(&s)) {
    if (!std::isfinite(sp->radius) || !(sp->radius > 0.0))
      throw std::invalid_argument("immersion_body: a sphere's radius must be finite and > 0");
    _kind = kind::sphere;
    _radius = sp->radius;
  } else if (const convex_hull *h = std::get_if<convex_hull>(&s)) {
    _kind = kind::convex;
    _planes = hull_planes(h->points);
    _points = h->points;
  } else {
    throw std::invalid_argument("immersion_body: a mesh or terrain is static and never immersed");
  }
}

immersion_body::immersion_body(const hull_mesh &h) {
  if (h.vertices.empty() || h.triangles.empty())
    throw std::invalid_argument("immersion_body: the hull has no triangles");
  for (const vec3 &v : h.vertices)
    if (!finite(v))
      throw std::invalid_argument("immersion_body: a hull vertex is not finite");
  for (const auto &t : h.triangles)
    for (std::uint32_t i : t)
      if (i >= h.vertices.size())
        throw std::invalid_argument("immersion_body: a hull index is out of range");
  _kind = kind::mesh;
  _points = h.vertices;
  _triangles = h.triangles;
}

immersion_body immersion_body::of(const body_desc &d) {
  return d.hull ? immersion_body(*d.hull) : immersion_body(d.shape);
}

double immersion_body::column(const vec3 &position, const quat &rotation, double x, double y,
                              double top) const {
  posed_outline p;
  pose_outline(_kind, _half, _radius, _planes, _points, _triangles, position, rotation, p);
  if (_kind == kind::convex) { // the bounds need the points
    p.verts.resize(_points.size());
    for (std::size_t i = 0; i < _points.size(); ++i)
      p.verts[i] = position + p.r * _points[i];
  }
  return posed_column(p, x, y, -kInf, top);
}

void immersion_body::bounds(const vec3 &position, const quat &rotation, vec3 &lo, vec3 &hi) const {
  posed_outline p;
  pose_outline(_kind, _half, _radius, _planes, _points, _triangles, position, rotation, p);
  if (_kind == kind::convex) {
    p.verts.resize(_points.size());
    for (std::size_t i = 0; i < _points.size(); ++i)
      p.verts[i] = position + p.r * _points[i];
  }
  posed_bounds(p, lo, hi);
}

std::uint64_t immersion_body::digest() const {
  std::uint64_t h = 1469598103934665603ull;
  const std::uint8_t k = static_cast<std::uint8_t>(_kind);
  h = fnv(&k, 1, h);
  h = fnv(_half.x, fnv(_half.y, fnv(_half.z, fnv(_radius, h))));
  for (const auto &pl : _planes)
    for (double v : pl)
      h = fnv(v, h);
  for (const vec3 &p : _points)
    h = fnv(p.x, fnv(p.y, fnv(p.z, h)));
  for (const auto &t : _triangles)
    h = fnv(t.data(), sizeof(std::uint32_t) * 3, h);
  return h;
}

immersion_water immersion_water::of(const field &f, double t) {
  if (!f.volume)
    throw std::invalid_argument("immersion_water: the field has no volume");
  const cvc::bounding_box &b = f.volume->boundingBox();
  immersion_water w;
  w.surface = f.surface.get();
  w.time = t;
  w.lo = {b.minx, b.miny, b.minz};
  w.hi = {b.maxx, b.maxy, b.maxz};
  return w;
}

double add_immersion(const immersion_body &body, const vec3 &position, const quat &rotation,
                     const immersion_water &water, const immersion_grid &g,
                     std::vector<double> &zeta) {
  if (g.nx <= 0 || g.ny <= 0 || !(g.dx > 0.0) || !std::isfinite(g.dx) ||
      !std::isfinite(g.origin_x) || !std::isfinite(g.origin_y))
    throw std::invalid_argument("add_immersion: the grid must have cells, a finite dx above 0 and "
                                "a finite origin");
  if (g.samples < 1 || g.samples > 64)
    throw std::invalid_argument("add_immersion: samples must be in 1..64");
  if (zeta.size() != g.count())
    throw std::invalid_argument("add_immersion: zeta must have nx * ny values");
  posed_outline p;
  pose_outline(body._kind, body._half, body._radius, body._planes, body._points, body._triangles,
               position, rotation, p);
  if (body._kind == immersion_body::kind::convex) {
    p.verts.resize(body._points.size());
    for (std::size_t i = 0; i < body._points.size(); ++i)
      p.verts[i] = position + p.r * body._points[i];
  }
  vec3 lo, hi;
  posed_bounds(p, lo, hi);
  // Clip to the water's box: nothing outside it is under water.
  lo = {std::max(lo.x, water.lo.x), std::max(lo.y, water.lo.y), std::max(lo.z, water.lo.z)};
  hi = {std::min(hi.x, water.hi.x), std::min(hi.y, water.hi.y), std::min(hi.z, water.hi.z)};
  if (!(hi.x >= lo.x) || !(hi.y >= lo.y) || !(hi.z > lo.z))
    return 0.0;
  // The cells whose square (dx wide about the cell's point) meets the body's box.
  const double half = 0.5 * g.dx;
  const double fi0 = std::ceil((lo.x - half - g.origin_x) / g.dx),
               fi1 = std::floor((hi.x + half - g.origin_x) / g.dx),
               fj0 = std::ceil((lo.y - half - g.origin_y) / g.dx),
               fj1 = std::floor((hi.y + half - g.origin_y) / g.dx);
  if (fi1 < 0.0 || fj1 < 0.0 || fi0 > double(g.nx - 1) || fj0 > double(g.ny - 1))
    return 0.0;
  const int i0 = std::max(0, int(fi0)), i1 = std::min(g.nx - 1, int(fi1));
  const int j0 = std::max(0, int(fj0)), j1 = std::min(g.ny - 1, int(fj1));
  const int s = g.samples;
  const double inv = 1.0 / double(s * s), zfloor = water.lo.z;
  double added = 0.0;
  for (int j = j0; j <= j1; ++j) {
    const double py = g.origin_y + double(j) * g.dx;
    for (int i = i0; i <= i1; ++i) {
      const double px = g.origin_x + double(i) * g.dx;
      double top = water.hi.z;
      if (water.surface)
        top = std::min(top, water.surface->height(px, py, water.time));
      if (!(top > lo.z))
        continue; // the body is wholly above the water here
      double sum = 0.0;
      for (int b = 0; b < s; ++b) {
        const double y = py + ((double(b) + 0.5) / double(s) - 0.5) * g.dx;
        if (y < water.lo.y || y > water.hi.y)
          continue;
        for (int a = 0; a < s; ++a) {
          const double x = px + ((double(a) + 0.5) / double(s) - 0.5) * g.dx;
          if (x < water.lo.x || x > water.hi.x)
            continue;
          sum += posed_column(p, x, y, zfloor, top);
        }
      }
      if (sum > 0.0) {
        const double z = sum * inv;
        zeta[std::size_t(j) * std::size_t(g.nx) + std::size_t(i)] += z;
        added += z;
      }
    }
  }
  return added * g.dx * g.dx;
}

} // namespace physics
} // namespace cvc
