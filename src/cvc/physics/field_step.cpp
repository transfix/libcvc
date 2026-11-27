/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "field_step.h"

#include "det_math.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/core/exception.h>
#include <cvc/volume/volume.h>
#include <stdexcept>
#include <string>

namespace cvc {
namespace physics {
namespace detail {

namespace {

constexpr double kPi = 3.14159265358979323846;
// Sample cells along a body's longest side.
constexpr int kCellsAcross = 8;
// A flow with a wind and no volume (field.h): air of this density over a box this wide.
constexpr double kAirDensity = 1.225, kEverywhere = 1.0e7;
// The heights across a body a wind's profile is read at (flow_velocity.h), closer together near
// the bottom, where a profile over the ground or the water bends most; a cell's is linear between
// them.
constexpr int kProfiles = 9;

vec3 cross(const vec3 &a, const vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double norm(const vec3 &v) { return std::sqrt(dot(v, v)); }

// A rotation matrix, row-major.
struct mat3 {
  double m[3][3];
  vec3 operator*(const vec3 &v) const {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
  }
  // R^T v: a world vector in the body's frame.
  vec3 inverse(const vec3 &v) const {
    return {m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z,
            m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z,
            m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z};
  }
  // |R| v: the half extents of the world box around a rotated box of half extents v.
  vec3 abs_times(const vec3 &v) const {
    return {std::fabs(m[0][0]) * v.x + std::fabs(m[0][1]) * v.y + std::fabs(m[0][2]) * v.z,
            std::fabs(m[1][0]) * v.x + std::fabs(m[1][1]) * v.y + std::fabs(m[1][2]) * v.z,
            std::fabs(m[2][0]) * v.x + std::fabs(m[2][1]) * v.y + std::fabs(m[2][2]) * v.z};
  }
};

mat3 rotation(const quat &q) {
  const double w = q.w, x = q.x, y = q.y, z = q.z;
  return {{{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
           {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
           {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}}};
}

double component(const vec3 &v, int k) { return k == 0 ? v.x : k == 1 ? v.y : v.z; }

void need(bool ok, const std::string &what) {
  if (!ok)
    throw cvc::physics_error("field: " + what);
}

field_grid grid_of(const cvc::volume &v, const char *what, bool nonnegative) {
  field_grid g;
  g.n[0] = v.XDim();
  g.n[1] = v.YDim();
  g.n[2] = v.ZDim();
  need(g.n[0] > 0 && g.n[1] > 0 && g.n[2] > 0, std::string(what) + " has no voxels");
  const cvc::bounding_box &b = v.boundingBox();
  g.lo[0] = b.minx, g.lo[1] = b.miny, g.lo[2] = b.minz;
  g.hi[0] = b.maxx, g.hi[1] = b.maxy, g.hi[2] = b.maxz;
  for (int k = 0; k < 3; ++k) {
    need(std::isfinite(g.lo[k]) && std::isfinite(g.hi[k]) && g.hi[k] > g.lo[k],
         std::string(what) + "'s bounding box is flat or not finite");
    g.span[k] = g.n[k] > 1 ? (g.hi[k] - g.lo[k]) / static_cast<double>(g.n[k] - 1) : 0.0;
  }
  const std::uint64_t count = g.n[0] * g.n[1] * g.n[2];
  g.values.resize(static_cast<std::size_t>(count));
  for (std::uint64_t i = 0; i < count; ++i) {
    const double x = v(i);
    need(std::isfinite(x), std::string(what) + " has a value that is not finite");
    need(!nonnegative || x >= 0.0, std::string(what) + " has a negative value");
    g.values[static_cast<std::size_t>(i)] = x;
  }
  return g;
}

// `old`, when given, is the field this one replaces (field_set::set): the same wake is kept.
compiled_field compile(const field &f, const compiled_field *old = nullptr) {
  need(f.kind <= field_kind::flow, "bad kind");
  need(f.volume != nullptr || (f.kind == field_kind::flow && f.wind),
       "no volume (only a flow with a wind may have none)");
  need(std::isfinite(f.drag_coefficient) && f.drag_coefficient >= 0.0,
       "drag_coefficient must be finite and >= 0");
  need(std::isfinite(f.damping) && f.damping >= 0.0, "damping must be finite and >= 0");
  need(std::isfinite(f.traction) && f.traction >= 0.0, "traction must be finite and >= 0");
  need(!f.surface || f.kind == field_kind::fluid, "only a fluid has a free surface");
  need(!f.wind || f.kind == field_kind::flow, "only a flow has a wind");
  need(!f.inertial || f.kind == field_kind::fluid, "only a fluid is inertial");
  need(f.wake.cells >= 0, "wake.cells must be 0 (no wake) or a power of two in 8 .. 256");
  need(!f.wake.on() || (f.kind == field_kind::fluid && f.surface),
       "only a fluid with a free surface has a wake");
  const int components =
      (f.velocity[0] ? 1 : 0) + (f.velocity[1] ? 1 : 0) + (f.velocity[2] ? 1 : 0);
  need(components == 0 || components == 3, "give all three velocity components or none");
  compiled_field c;
  c.desc = f;
  if (f.volume) {
    c.scalar = grid_of(
        *f.volume,
        f.kind == field_kind::medium ? "the bearing modulus volume" : "the density volume", true);
  } else { // a wind without a volume: air at sea level's density, everywhere (field.h)
    for (int k = 0; k < 3; ++k) {
      c.scalar.n[k] = 1;
      c.scalar.lo[k] = -kEverywhere;
      c.scalar.hi[k] = kEverywhere;
    }
    c.scalar.values.assign(1, kAirDensity);
  }
  c.moving = components == 3;
  if (c.moving)
    for (int k = 0; k < 3; ++k)
      c.velocity[k] = grid_of(*f.velocity[k], "a velocity volume", false);
  if (f.ruts && f.kind == field_kind::medium) {
    need(std::isfinite(f.rut_cell) && f.rut_cell > 0.0, "rut_cell must be finite and > 0");
    const double cols = std::ceil((c.scalar.hi[0] - c.scalar.lo[0]) / f.rut_cell);
    const double rows = std::ceil((c.scalar.hi[1] - c.scalar.lo[1]) / f.rut_cell);
    need(cols * rows <= double(1u << 24),
         "rut_cell is too fine for the field's extent (more than 16M rut cells)");
    c.ruts.x0 = c.scalar.lo[0];
    c.ruts.y0 = c.scalar.lo[1];
    c.ruts.cell = f.rut_cell;
    c.ruts.cols = std::max(1, static_cast<int>(cols));
    c.ruts.rows = std::max(1, static_cast<int>(rows));
    c.ruts.depth.assign(static_cast<std::size_t>(c.ruts.cols) * c.ruts.rows, 0.0f);
  }
  // The wake: the old field's when it is the same wake (its waves under a new sea); else a new one,
  // its parameters checked here (the step gives it the world's gravity).
  if (f.wake.on() && old && old->wake && old->desc.wake == f.wake)
    c.wake = old->wake;
  else if (f.wake.on())
    c.wake = std::make_shared<fluid_wake>(f.wake, 9.80665);
  return c;
}

// A body's view of a fluid's wake: the others' waves (fluid_wake::sample less its own state).
struct wake_view {
  const fluid_wake *wake = nullptr;
  int own = -1;
  // Their height at (x, y), and their motion `depth` metres below the top there.
  double height(double x, double y) const { return wake->height(own, x, y); }
  void motion(double x, double y, double depth, wake_sample &out) const {
    wake->motion(own, x, y, depth, out);
  }
};

// What one cell reads of a field: whether any of it is in the stuff (immersed, the stuff's value
// there > 0), and then the immersed part's centre, its depth below the stuff's top, its share of
// the cell, the value and the stuff's velocity there. A pure function of its arguments (a
// surface's reads are, and its memo is the cell's own), so cells can be read in any order and on
// any thread.
struct cell_read {
  bool in = false;
  double p[3] = {0.0, 0.0, 0.0};
  double depth = 0.0;
  double f = 0.0;
  double value = 0.0;
  vec3 w;
};

// A flow's wind over one body (flow_velocity.h): its reference at the body's centre of mass, since
// the gusts vary over tens of metres, and its profile at kProfiles heights from its base (or the
// body's bottom) to the body's top, spaced as the squares. Made once per body and field, so the
// wind costs O(bodies) a step; at() is the reference times the profile at a cell's height.
struct body_wind {
  vec3 reference;
  double floor = 0.0; // the air starts here: the wind's base
  double z0 = 0.0, span = 0.0;
  double z[kProfiles] = {}, prof[kProfiles] = {};
  vec3 at(double pz) const {
    const double u = std::min(std::max((pz - z0) / span, 0.0), 1.0);
    const int j = std::min(static_cast<int>((kProfiles - 1) * std::sqrt(u)), kProfiles - 2);
    const double gap = z[j + 1] - z[j];
    const double fr = gap > 0.0 ? std::min(std::max((pz - z[j]) / gap, 0.0), 1.0) : 0.0;
    return reference * (prof[j] + (prof[j + 1] - prof[j]) * fr);
  }
};

cell_read read_cell(const vec3 &local, const body_state &s, const mat3 &r, const double ext[3],
                    const compiled_field &cf, const fluid_surface *surface, double t,
                    const body_wind *wind = nullptr, const wake_view *wake = nullptr) {
  cell_read out;
  const vec3 c = s.position + r * local;
  // The immersed part of the cell: its world box clipped to the field's, and below a free
  // surface's height over the clipped centre (x and y come first), the others' waves on it with a
  // wake. The velocity below is read at that centre too, so the two reads share the cell's memo.
  surface_memo memo;
  double p[3], f = 1.0, ztop = 0.0;
  bool in = true;
  for (int k = 0; k < 3 && in; ++k) {
    double top = cf.scalar.hi[k];
    if (k == 2 && surface) {
      double sh = surface->height_with(p[0], p[1], t, memo);
      if (wake)
        sh += wake->height(p[0], p[1]);
      top = std::min(top, sh);
    }
    if (k == 2)
      ztop = top;
    double bottom = cf.scalar.lo[k];
    if (k == 2 && wind) // the air starts at the wind's base
      bottom = std::max(bottom, wind->floor);
    const double lo = std::max(component(c, k) - ext[k], bottom);
    const double hi = std::min(component(c, k) + ext[k], top);
    in = hi > lo;
    p[k] = 0.5 * (lo + hi);
    f *= (hi - lo) / (2.0 * ext[k]);
  }
  if (!in)
    return out;
  const double value = cf.scalar.at(p);
  if (!(value > 0.0))
    return out;
  out.in = true;
  out.p[0] = p[0];
  out.p[1] = p[1];
  out.p[2] = p[2];
  out.depth = ztop - p[2];
  out.f = f;
  out.value = value;
  const vec3 at{p[0], p[1], p[2]};
  if (surface) {
    out.w = surface->velocity_with(at, t, memo);
    if (wake) { // the others' waves' motion at the immersed part's depth below the top
      wake_sample ws;
      wake->motion(p[0], p[1], out.depth, ws);
      out.w = out.w + vec3{ws.u, ws.v, ws.w};
    }
  } else if (wind) // the body's wind, times its profile at the cell's height
    out.w = wind->at(p[2]);
  else if (cf.moving)
    out.w = {cf.velocity[0].contains(p) ? cf.velocity[0].at(p) : 0.0,
             cf.velocity[1].contains(p) ? cf.velocity[1].at(p) : 0.0,
             cf.velocity[2].contains(p) ? cf.velocity[2].at(p) : 0.0};
  return out;
}

// The world box of a body's cells, from its state.
void body_box(const field_body &fb, const body_state &s, double lo[3], double hi[3]) {
  const mat3 r = rotation(s.rotation);
  const vec3 mid = s.position + r * ((fb.lo + fb.hi) * 0.5);
  const vec3 half = r.abs_times((fb.hi - fb.lo) * 0.5 + fb.cell * 0.5);
  for (int k = 0; k < 3; ++k) {
    lo[k] = component(mid, k) - component(half, k);
    hi[k] = component(mid, k) + component(half, k);
  }
}

bool overlaps(const double lo[3], const double hi[3], const field_grid &g) {
  for (int k = 0; k < 3; ++k)
    if (hi[k] <= g.lo[k] || lo[k] >= g.hi[k])
      return false;
  return true;
}

// A grid of cells over the box [lo, hi], about kCellsAcross along its longest side.
void lay_cells(const vec3 &lo, const vec3 &hi, std::vector<vec3> &cells, vec3 &cell) {
  const double ext[3] = {hi.x - lo.x, hi.y - lo.y, hi.z - lo.z};
  const double longest = std::max(ext[0], std::max(ext[1], ext[2]));
  int n[3];
  double size[3];
  for (int k = 0; k < 3; ++k) {
    n[k] = std::max(1, static_cast<int>(std::lround(kCellsAcross * ext[k] / longest)));
    size[k] = ext[k] / n[k];
  }
  cell = {size[0], size[1], size[2]};
  cells.clear();
  cells.reserve(static_cast<std::size_t>(n[0]) * n[1] * n[2]);
  for (int k = 0; k < n[2]; ++k)
    for (int j = 0; j < n[1]; ++j)
      for (int i = 0; i < n[0]; ++i)
        cells.push_back(
            {lo.x + (i + 0.5) * size[0], lo.y + (j + 0.5) * size[1], lo.z + (k + 0.5) * size[2]});
}

void bounds_of(field_body &fb) {
  fb.lo = fb.hi = fb.cells.front();
  for (const vec3 &c : fb.cells) {
    fb.lo = {std::min(fb.lo.x, c.x), std::min(fb.lo.y, c.y), std::min(fb.lo.z, c.z)};
    fb.hi = {std::max(fb.hi.x, c.x), std::max(fb.hi.y, c.y), std::max(fb.hi.z, c.z)};
  }
}

// The body's area across the body-local unit direction d.
double area_across(const field_body &fb, const vec3 &d) {
  switch (fb.shape) {
  case field_body::outline::sphere:
    return kPi * fb.radius * fb.radius;
  case field_body::outline::box:
    return 4.0 * (std::fabs(d.x) * fb.half.y * fb.half.z + std::fabs(d.y) * fb.half.x * fb.half.z +
                  std::fabs(d.z) * fb.half.x * fb.half.y);
  case field_body::outline::other:
    break;
  }
  // The sphere of the same volume.
  const double r = det::cbrt(3.0 * fb.volume / (4.0 * kPi));
  return kPi * r * r;
}

std::uint64_t fnv1a(const void *data, std::size_t n, std::uint64_t h) {
  const unsigned char *p = static_cast<const unsigned char *>(data);
  for (std::size_t i = 0; i < n; ++i) {
    h ^= p[i];
    h *= 1099511628211ull;
  }
  return h;
}

// Half the least moment of inertia per kilogram of a solid box with sides e about its centre,
// (a^2 + b^2) / 24 over its two shortest sides.
double half_least_box_moment(const vec3 &e) {
  double s[3] = {e.x, e.y, e.z};
  std::sort(s, s + 3);
  return (s[0] * s[0] + s[1] * s[1]) / 24.0;
}

// Checks a hull and builds what the fluids need of it: the triangles' normals, the length for
// the Reynolds number, the slamming length, the digest and the inertia estimate per kilogram.
// It keeps a private copy of the mesh: a caller may still hold a non-const pointer to the one it
// gave, and the cached normals, the wetted areas and the digest must describe what is simulated.
void compile_hull(const hull_mesh &given, field_hull &out) {
  const auto bad = [](const std::string &what) {
    throw cvc::physics_error("add_body: hull: " + what);
  };
  auto desc = std::make_shared<const hull_mesh>(given);
  const hull_mesh &h = *desc;
  if (h.vertices.size() < 4 || h.triangles.size() < 4)
    bad("needs at least 4 vertices and 4 triangles");
  if (h.vertices.size() > 0xffffffffu)
    bad("too many vertices");
  for (const vec3 &v : h.vertices)
    if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
      bad("a vertex is not finite");
  if (!std::isfinite(h.length) || !(h.length >= 0.0))
    bad("length must be finite and >= 0");
  if (!std::isfinite(h.kinematic_viscosity) || !(h.kinematic_viscosity > 0.0))
    bad("kinematic_viscosity must be finite and > 0");
  if (!std::isfinite(h.suction_coefficient) || !(h.suction_coefficient >= 0.0) ||
      !std::isfinite(h.slam_coefficient) || !(h.slam_coefficient >= 0.0))
    bad("suction_coefficient and slam_coefficient must be finite and >= 0");
  if (!std::isfinite(h.lift_coefficient) || !(h.lift_coefficient >= 0.0))
    bad("lift_coefficient must be finite and >= 0");
  double volume = 0.0;
  out.normal.clear();
  out.normal.reserve(h.triangles.size());
  for (const auto &t : h.triangles) {
    if (t[0] >= h.vertices.size() || t[1] >= h.vertices.size() || t[2] >= h.vertices.size())
      bad("a triangle's index is out of range");
    const vec3 &a = h.vertices[t[0]], &b = h.vertices[t[1]], &c = h.vertices[t[2]];
    const vec3 n = cross(b - a, c - a);
    const double len = norm(n);
    out.normal.push_back(len > 0.0 ? n * (1.0 / len) : vec3{}); // a degenerate one is skipped
    volume += dot(a, cross(b, c)) / 6.0;
  }
  if (!(volume > 0.0))
    bad("the triangles must enclose a volume, counter-clockwise seen from outside");
  vec3 lo = h.vertices.front(), hi = lo;
  for (const vec3 &p : h.vertices) {
    lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
    hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
  }
  out.length = h.length > 0.0 ? h.length : hi.x - lo.x;
  if (!(out.length > 0.0))
    bad("the hull has no extent along x; give its length");
  out.slam_length = det::cbrt(volume);
  // Half a solid box's least moment about its centre, m (a^2 + b^2) / 12 over the two shortest
  // sides of the hull's bounding box. body_added lowers it to the collision shape's estimate
  // when that is less (the body's true inertia comes from its shape, not from its hull).
  out.inertia = half_least_box_moment(hi - lo);
  std::uint64_t d = 1469598103934665603ull;
  d = fnv1a(h.vertices.data(), h.vertices.size() * sizeof(vec3), d);
  d = fnv1a(h.triangles.data(), h.triangles.size() * sizeof(h.triangles[0]), d);
  const double knobs[4] = {h.length, h.kinematic_viscosity, h.suction_coefficient,
                           h.slam_coefficient};
  out.digest = fnv1a(knobs, sizeof(knobs), d);
  // The lift only when it is on, so a hull without it keeps the digest (and a world the state
  // hash) it had before the term.
  if (h.lift_coefficient > 0.0)
    out.digest = fnv1a(&h.lift_coefficient, sizeof(h.lift_coefficient), out.digest);
  out.wetted.assign(h.triangles.size(), 0.0);
  out.desc = std::move(desc);
}

// What one body's fields add up to before the velocity terms are integrated.
struct push_sums {
  vec3 push_f, push_t, drag_f, drag_t;
  double rate = 0.0;       // kg/s: the drag's linear rate, sum of c_i
  double rate_heave = 0.0; // kg/s: the part of `rate` acting on vertical motion only (damping)
  double rate_rot = 0.0;   // kg m^2/s: the hull drag's rotational rate, sum of c_i |arm|^2
  double immersed = 0.0;
  bool wet = false; // a hull triangle is under water
};

// A hull's push from one fluid field (hull.h): each triangle split at the waterline, its
// submerged part's pressure at its centre of pressure, and its velocity terms. `world_v` and
// `depth` are scratch, one per vertex; `wet` collects each triangle's submerged area.
void push_hull(const field_body &fb, const body_state &s, const mat3 &r, const vec3 &com,
               const compiled_field &cf, double gn, double dt, double t, std::vector<vec3> &world_v,
               std::vector<double> &depth, std::vector<double> &wet, push_sums &sum,
               const wake_view *wake) {
  const hull_mesh &h = *fb.hull.desc;
  const field_grid &box = cf.scalar;
  const fluid_surface *surface = cf.desc.surface.get();
  const std::size_t nv = h.vertices.size();
  world_v.resize(nv);
  depth.resize(nv);
  wake_sample ws;
  // The water's top over each vertex, once: the surface's height (and the others' waves on it,
  // with a wake) or the box's top.
  for (std::size_t i = 0; i < nv; ++i) {
    const vec3 p = s.position + r * h.vertices[i];
    world_v[i] = p;
    if (p.x >= box.lo[0] && p.x <= box.hi[0] && p.y >= box.lo[1] && p.y <= box.hi[1]) {
      double top = box.hi[2];
      if (surface) {
        double sh = surface->height(p.x, p.y, t);
        if (wake)
          sh += wake->height(p.x, p.y);
        top = std::min(top, sh);
      }
      depth[i] = top - p.z;
    } else {
      depth[i] = -1.0; // outside the field's box: dry
    }
  }
  const double viscosity = h.kinematic_viscosity;
  const double ln10 = 2.302585092994045684;
  // The damping's sums, scaled once the immersed volume is known (below).
  double volume = 0.0, weight = 0.0, damp_rot = 0.0;
  vec3 damp_f, damp_t;
  for (std::size_t i = 0; i < h.triangles.size(); ++i) {
    const auto &tri = h.triangles[i];
    const double d0 = depth[tri[0]], d1 = depth[tri[1]], d2 = depth[tri[2]];
    if (!(d0 > 0.0 || d1 > 0.0 || d2 > 0.0))
      continue;
    const vec3 nl = fb.hull.normal[i];
    if (nl.x == 0.0 && nl.y == 0.0 && nl.z == 0.0)
      continue;
    const vec3 n = r * nl;
    // The submerged polygon: the triangle clipped to depth > 0, 3 or 4 corners, the new ones on
    // the waterline (depth 0).
    const vec3 P[3] = {world_v[tri[0]], world_v[tri[1]], world_v[tri[2]]};
    const double D[3] = {d0, d1, d2};
    vec3 Q[4];
    double E[4];
    int m = 0;
    for (int j = 0; j < 3; ++j) {
      const int k = j == 2 ? 0 : j + 1;
      if (D[j] > 0.0) {
        Q[m] = P[j];
        E[m] = D[j];
        ++m;
      }
      if ((D[j] > 0.0) != (D[k] > 0.0)) {
        const double f = D[j] / (D[j] - D[k]);
        Q[m] = P[j] + (P[k] - P[j]) * f;
        E[m] = 0.0;
        ++m;
      }
    }
    // Over the fan of the polygon: the area, the centroid, the pressure's integral (depth over
    // the area, exact for depth linear on each piece) and its first moment about the centre of
    // mass, int depth (x - com) dA = A/12 (sum d_i sum x_i + sum d_i x_i) a piece.
    double area = 0.0, press = 0.0;
    vec3 centroid, moment;
    for (int j = 1; j + 1 < m; ++j) {
      const vec3 a = Q[0] - com, b = Q[j] - com, c = Q[j + 1] - com;
      const double ak = 0.5 * norm(cross(b - a, c - a));
      if (!(ak > 0.0))
        continue;
      const double ds = E[0] + E[j] + E[j + 1];
      area += ak;
      press += ak * ds / 3.0;
      centroid = centroid + (a + b + c) * (ak / 3.0);
      moment = moment + ((a + b + c) * ds + a * E[0] + b * E[j] + c * E[j + 1]) * (ak / 12.0);
    }
    if (!(area > 0.0))
      continue;
    const vec3 arm = centroid * (1.0 / area);
    const vec3 at = com + arm;
    double q[3] = {at.x, at.y, at.z};
    for (int k = 0; k < 3; ++k)
      q[k] = std::min(std::max(q[k], box.lo[k]), box.hi[k]);
    const double rho = box.at(q);
    if (!(rho > 0.0))
      continue;
    wet[i] += area;
    sum.wet = true;

    // Hydrostatic pressure rho g depth, pushing along -n, at the centre of pressure.
    const double pg = rho * gn;
    const vec3 fp = n * (-pg * press);
    sum.push_f = sum.push_f + fp;
    sum.push_t = sum.push_t + cross(moment, n) * (-pg);
    volume -= press * n.z; // the divergence theorem: V = -int depth n_z dA

    // The velocity terms, against the water's velocity at the submerged part's centroid (the
    // others' waves' at its depth, the mean of the depths over the part: press / area).
    vec3 w;
    if (surface) {
      w = surface->velocity(at, t);
      if (wake) {
        wake->motion(at.x, at.y, press / area, ws);
        w = w + vec3{ws.u, ws.v, ws.w};
      }
    } else if (cf.moving)
      w = {cf.velocity[0].contains(q) ? cf.velocity[0].at(q) : 0.0,
           cf.velocity[1].contains(q) ? cf.velocity[1].at(q) : 0.0,
           cf.velocity[2].contains(q) ? cf.velocity[2].at(q) : 0.0};
    const vec3 u = w - (s.linear_velocity + cross(s.angular_velocity, arm));
    const double un = dot(u, n);
    const vec3 ut = u - n * un;
    vec3 f;
    double c_sum = 0.0;
    // Viscous resistance, ITTC-57.
    const double speed = norm(u);
    if (speed > 0.0) {
      const double re = std::max(speed * fb.hull.length / viscosity, 1e5);
      const double lg = det::log(re) / ln10 - 2.0;
      const double c = 0.5 * rho * (0.075 / (lg * lg)) * area * norm(ut);
      f = f + ut * c;
      c_sum += c;
    }
    // Pressure drag on a face the water meets (suction on one it leaves).
    const double cp = un < 0.0 ? cf.desc.drag_coefficient : h.suction_coefficient;
    if (cp > 0.0 && un != 0.0) {
      const double c = 0.5 * rho * cp * area * std::fabs(un);
      f = f + n * (un * c);
      c_sum += c;
    }
    // Lift on a face the water meets: a lifting plate's potential normal force, 1/2 rho K |u|^2
    // sin a cos a, as Polhamus's suction analogy gives it (hull.h).
    if (h.lift_coefficient > 0.0 && un < 0.0) {
      const double c = 0.5 * rho * h.lift_coefficient * area * norm(ut);
      f = f + n * (un * c);
      c_sum += c;
    }
    // Slamming: a face entering the water sets the water it wets moving.
    const double grew = area - fb.hull.wetted[i];
    if (un < 0.0 && grew > 0.0 && h.slam_coefficient > 0.0) {
      const double c = h.slam_coefficient * rho * fb.hull.slam_length * grew / dt;
      f = f + n * (un * c);
      c_sum += c;
    }
    if (c_sum > 0.0) {
      sum.drag_f = sum.drag_f + f;
      sum.drag_t = sum.drag_t + cross(arm, f);
      sum.rate += c_sum;
      sum.rate_rot += c_sum * dot(arm, arm);
    }
    // Damping, on the vertical part of u: shared among the downward faces by the water column
    // over each, press |n_z|. Upward faces under water take none, so it sums to the cells'
    // c V_sub (below) however deep the hull is.
    if (cf.desc.damping > 0.0 && n.z < 0.0) {
      const double wt = -press * n.z;
      weight += wt;
      damp_f.z += wt * u.z;
      damp_t = damp_t + cross(arm, vec3{0.0, 0.0, wt * u.z});
      damp_rot += wt * dot(arm, arm);
    }
  }
  sum.immersed += volume;
  // The damping's rate, c V_sub, the cells' over the same volume.
  if (cf.desc.damping > 0.0 && weight > 0.0 && volume > 0.0) {
    const double c = cf.desc.damping * volume;
    const double scale = c / weight;
    sum.drag_f = sum.drag_f + damp_f * scale;
    sum.drag_t = sum.drag_t + damp_t * scale;
    sum.rate += c;
    sum.rate_heave += c;
    sum.rate_rot += damp_rot * scale;
  }
}

} // namespace

bool field_grid::contains(const double p[3]) const {
  for (int k = 0; k < 3; ++k)
    if (!(p[k] >= lo[k] && p[k] <= hi[k]))
      return false;
  return true;
}

double field_grid::at(const double p[3]) const {
  std::uint64_t i[3];
  double t[3];
  for (int k = 0; k < 3; ++k) {
    if (n[k] == 1) {
      i[k] = 0;
      t[k] = 0.0;
      continue;
    }
    const double s =
        std::min(std::max((p[k] - lo[k]) / span[k], 0.0), static_cast<double>(n[k] - 1));
    i[k] = std::min(static_cast<std::uint64_t>(s), n[k] - 2);
    t[k] = s - static_cast<double>(i[k]);
  }
  const std::uint64_t step[3] = {n[0] > 1 ? 1u : 0u, n[1] > 1 ? n[0] : 0u,
                                 n[2] > 1 ? n[0] * n[1] : 0u};
  const std::uint64_t base = i[0] + n[0] * (i[1] + n[1] * i[2]);
  double out = 0.0;
  for (int c = 0; c < 8; ++c) {
    const int a = c & 1, b = (c >> 1) & 1, e = (c >> 2) & 1;
    const double w = (a ? t[0] : 1.0 - t[0]) * (b ? t[1] : 1.0 - t[1]) * (e ? t[2] : 1.0 - t[2]);
    if (w == 0.0)
      continue;
    out += w * values[static_cast<std::size_t>(base + a * step[0] + b * step[1] + e * step[2])];
  }
  return out;
}

field_id field_set::add(world_backend &be, const field &f) {
  compiled_field c = compile(f);
  need(_next < 0xffffffffu, "too many fields");
  field_id id;
  id.value = _next++;
  wake_in(be, c.scalar);
  _fields.emplace(id.value, std::move(c));
  return id;
}

void field_set::set(world_backend &be, field_id id, const field &f) {
  auto it = _fields.find(id.value);
  need(it != _fields.end(), "unknown or removed field id " + std::to_string(id.value));
  compiled_field c = compile(f, &it->second); // the same wake keeps its waves (compile)
  const rut_map &old = it->second.ruts;
  if (!c.ruts.depth.empty() && c.ruts.x0 == old.x0 && c.ruts.y0 == old.y0 &&
      c.ruts.cell == old.cell && c.ruts.cols == old.cols && c.ruts.rows == old.rows) {
    c.ruts.depth = std::move(it->second.ruts.depth);
    c.rut_cells = it->second.rut_cells;
    c.rut_deepest = it->second.rut_deepest;
  }
  wake_in(be, it->second.scalar);
  wake_in(be, c.scalar);
  it->second = std::move(c);
}

field field_set::get(field_id id) const {
  auto it = _fields.find(id.value);
  need(it != _fields.end(), "unknown or removed field id " + std::to_string(id.value));
  return it->second.desc;
}

void field_set::snapshot(std::vector<field_snapshot> &out) const {
  out.reserve(out.size() + _fields.size());
  for (const auto &fe : _fields) {
    field_snapshot s;
    s.id.value = fe.first;
    s.desc = fe.second.desc;
    s.rut_cells = fe.second.rut_cells;
    s.rut_deepest = fe.second.rut_deepest;
    const field_grid &g = fe.second.scalar;
    s.lo = {g.lo[0], g.lo[1], g.lo[2]};
    s.hi = {g.hi[0], g.hi[1], g.hi[2]};
    out.push_back(std::move(s));
  }
}

void field_set::remove(field_id id) {
  need(_fields.erase(id.value) == 1, "unknown or removed field id " + std::to_string(id.value));
}

void field_set::body_added(const world_backend &be, body_id id, const body_desc &d) {
  if (d.motion != motion_type::dynamic)
    return;
  field_body fb;
  if (const sphere *s = std::get_if<sphere>(&d.shape)) {
    const double r = s->radius;
    std::vector<vec3> grid;
    lay_cells({-r, -r, -r}, {r, r, r}, grid, fb.cell);
    for (const vec3 &c : grid)
      if (dot(c, c) <= r * r)
        fb.cells.push_back(c);
    fb.volume = 4.0 / 3.0 * kPi * r * r * r;
    fb.shape = field_body::outline::sphere;
    fb.radius = r;
  } else if (const box *b = std::get_if<box>(&d.shape)) {
    const vec3 h = b->half_extents;
    lay_cells(h * -1.0, h, fb.cells, fb.cell);
    fb.volume = 8.0 * h.x * h.y * h.z;
    fb.shape = field_body::outline::box;
    fb.half = h;
  } else if (const convex_hull *hull = std::get_if<convex_hull>(&d.shape)) {
    vec3 lo = hull->points.front(), hi = lo;
    for (const vec3 &p : hull->points) {
      lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
      hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    }
    std::vector<vec3> grid;
    lay_cells(lo, hi, grid, fb.cell);
    std::vector<std::uint8_t> inside;
    if (!be.shape_interior(id, grid, inside, fb.volume, fb.center_of_mass))
      return;
    for (std::size_t i = 0; i < grid.size(); ++i)
      if (inside[i])
        fb.cells.push_back(grid[i]);
  } else {
    return; // meshes and terrain are static
  }
  if (fb.cells.empty() || !(fb.volume > 0.0))
    return;
  // A wake cuts its hull, else its shape (outline_of): a convex hull keeps its points for that, a
  // box or a sphere needs only `half` or `radius`.
  if (!d.hull && fb.shape == field_body::outline::other)
    fb.cut_shape = std::make_shared<const physics::shape>(d.shape);
  fb.cell_volume = fb.volume / static_cast<double>(fb.cells.size());
  fb.mass = d.mass > 0.0 ? d.mass : d.density * fb.volume;
  fb.buoyancy = fb.water_share = d.displacement;
  bounds_of(fb);
  if (d.hull) {
    compile_hull(*d.hull, fb.hull);
    // The torque's implicit scale needs an inertia no more than the body's least moment, which
    // comes from the collision shape: take half the shape's own where that is lower. A sphere's
    // is exact, 0.4 m r^2, and a box's too; for a convex hull the bounding box stands in, which
    // holds while the shape roughly fills its bounding box.
    double shape_inertia = 0.0;
    switch (fb.shape) {
    case field_body::outline::sphere:
      shape_inertia = 0.2 * fb.radius * fb.radius;
      break;
    case field_body::outline::box:
      shape_inertia = half_least_box_moment(fb.half * 2.0);
      break;
    case field_body::outline::other: {
      const convex_hull &ch = std::get<convex_hull>(d.shape);
      vec3 lo = ch.points.front(), hi = lo;
      for (const vec3 &p : ch.points) {
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
      }
      shape_inertia = half_least_box_moment(hi - lo);
      break;
    }
    }
    fb.hull.inertia = std::min(fb.hull.inertia, shape_inertia);
    // The body's box reaches the hull's vertices too, so a hull wider than the shape is pushed.
    for (const vec3 &p : fb.hull.desc->vertices) {
      fb.lo = {std::min(fb.lo.x, p.x), std::min(fb.lo.y, p.y), std::min(fb.lo.z, p.z)};
      fb.hi = {std::max(fb.hi.x, p.x), std::max(fb.hi.y, p.y), std::max(fb.hi.z, p.z)};
    }
  }
  _bodies[id.value] = std::move(fb);
}

void field_set::wake_in(world_backend &be, const field_grid &box) {
  for (const auto &entry : _bodies) {
    body_id id;
    id.value = entry.first;
    const body_state s = be.state(id);
    if (s.active)
      continue;
    double lo[3], hi[3];
    body_box(entry.second, s, lo, hi);
    if (overlaps(lo, hi, box))
      be.wake(id);
  }
}

void field_set::mark_vehicle(body_id id, const vec3 &center_of_mass, double buoyancy) {
  auto it = _bodies.find(id.value);
  if (it == _bodies.end())
    return;
  it->second.vehicle = true;
  it->second.center_of_mass = center_of_mass;
  it->second.buoyancy = buoyancy;
}

bool field_set::air_at(const vec3 &p, double t, double &rho, vec3 &wind) const {
  const double q[3] = {p.x, p.y, p.z};
  for (const auto &fe : _fields) {
    const compiled_field &cf = fe.second;
    if (cf.desc.kind != field_kind::flow || !cf.scalar.contains(q))
      continue;
    rho = cf.scalar.at(q);
    wind = vec3{};
    if (cf.desc.wind)
      wind = cf.desc.wind->velocity(p, t);
    else if (cf.moving)
      wind = {cf.velocity[0].contains(q) ? cf.velocity[0].at(q) : 0.0,
              cf.velocity[1].contains(q) ? cf.velocity[1].at(q) : 0.0,
              cf.velocity[2].contains(q) ? cf.velocity[2].at(q) : 0.0};
    return true;
  }
  return false;
}

bool field_set::water_at(const vec3 &p, double t, water_sample &out, const wake_read &wake) const {
  out = water_sample{};
  for (const auto &fe : _fields) {
    const compiled_field &cf = fe.second;
    const field_grid &g = cf.scalar;
    if (cf.desc.kind != field_kind::fluid || !(p.x >= g.lo[0] && p.x <= g.hi[0]) ||
        !(p.y >= g.lo[1] && p.y <= g.hi[1]) || !(p.z >= g.lo[2]))
      continue;
    const fluid_surface *surface = cf.desc.surface.get();
    // With a wake, as a body reads it in the step: the others' waves on the surface (all of them
    // for no body), their motion at the point's depth below that.
    const fluid_wake *fw = wake.on && surface ? cf.wake.get() : nullptr;
    const int own = fw && wake.body.valid() ? fw->own(wake.body.value) : -1;
    double top = g.hi[2];
    if (surface) {
      double sh = surface->height(p.x, p.y, t);
      if (fw)
        sh += fw->height(own, p.x, p.y);
      top = std::min(top, sh);
    }
    out.depth = top - p.z;
    const double q[3] = {p.x, p.y, std::min(p.z, g.hi[2])};
    out.density = g.at(q);
    out.wet = out.depth > 0.0 && out.density > 0.0;
    if (surface) {
      out.velocity = surface->velocity(p, t);
      if (fw) {
        wake_sample ws;
        fw->ready_depths(); // its callers read one at a time (a boat's pre-step, the world's lock)
        fw->motion(own, p.x, p.y, out.depth, ws);
        out.velocity = out.velocity + vec3{ws.u, ws.v, ws.w};
      }
    } else if (cf.moving)
      out.velocity = {cf.velocity[0].contains(q) ? cf.velocity[0].at(q) : 0.0,
                      cf.velocity[1].contains(q) ? cf.velocity[1].at(q) : 0.0,
                      cf.velocity[2].contains(q) ? cf.velocity[2].at(q) : 0.0};
    return true;
  }
  return false;
}

void field_set::press(std::uint32_t field, double x, double y, double dx, double dy, double length,
                      double width, double depth, bool behind) {
  auto it = _fields.find(field);
  if (it == _fields.end() || it->second.ruts.depth.empty() || !(depth > 0.0))
    return;
  compiled_field &cf = it->second;
  rut_map &m = cf.ruts;
  const double half_l = 0.5 * length, half_w = 0.5 * width;
  // The rectangle's box in cells, then each cell whose centre lies inside it.
  const double reach = std::sqrt(half_l * half_l + half_w * half_w) + m.cell;
  const int c0 = std::max(0, static_cast<int>(std::floor((x - reach - m.x0) / m.cell)));
  const int c1 = std::min(m.cols - 1, static_cast<int>(std::floor((x + reach - m.x0) / m.cell)));
  const int r0 = std::max(0, static_cast<int>(std::floor((y - reach - m.y0) / m.cell)));
  const int r1 = std::min(m.rows - 1, static_cast<int>(std::floor((y + reach - m.y0) / m.cell)));
  const float d = static_cast<float>(depth);
  for (int r = r0; r <= r1; ++r)
    for (int c = c0; c <= c1; ++c) {
      const double px = m.x0 + (c + 0.5) * m.cell - x, py = m.y0 + (r + 0.5) * m.cell - y;
      const double along = px * dx + py * dy, across = -px * dy + py * dx;
      if (std::fabs(across) > half_w || along > (behind ? 0.0 : half_l) || along < -half_l)
        continue;
      float &cell = m.depth[static_cast<std::size_t>(r) * m.cols + c];
      if (!(d > cell))
        continue;
      if (!(cell > 0.0f))
        ++cf.rut_cells;
      cell = d;
      cf.rut_deepest = std::max(cf.rut_deepest, static_cast<double>(d));
    }
}

double field_set::rut_at(std::uint32_t field, double x, double y) const {
  auto it = _fields.find(field);
  return it == _fields.end() ? 0.0 : it->second.ruts.at(x, y);
}

rut_map field_set::ruts(field_id id) const {
  auto it = _fields.find(id.value);
  need(it != _fields.end(), "unknown or removed field id " + std::to_string(id.value));
  return it->second.ruts;
}

bool field_set::has_surfaces() const {
  for (const auto &fe : _fields)
    if (fe.second.desc.surface || fe.second.desc.wind)
      return true;
  return false;
}

namespace {
// A field's time-dependent hook as (id, digest): a fluid's surface, or a flow's wind (a field has
// at most one). A wind's id carries the top bit, so a surface and a wind never read as each other.
constexpr std::uint32_t kWindTag = 0x80000000u;
bool hook_of(const compiled_field &cf, std::uint32_t field, std::uint32_t &id,
             std::uint64_t &digest) {
  if (cf.desc.surface) {
    id = field;
    digest = cf.desc.surface->digest();
    return true;
  }
  if (cf.desc.wind) {
    id = field | kWindTag;
    digest = cf.desc.wind->digest();
    return true;
  }
  return false;
}
} // namespace

void field_set::save_surfaces(std::string &out) const {
  for (const auto &fe : _fields) {
    std::uint32_t id = 0;
    std::uint64_t digest = 0;
    if (!hook_of(fe.second, fe.first, id, digest))
      continue;
    out.append(reinterpret_cast<const char *>(&id), sizeof(id));
    out.append(reinterpret_cast<const char *>(&digest), sizeof(digest));
  }
}

void field_set::check_surfaces(const std::string &in, std::size_t &pos) const {
  for (const auto &fe : _fields) {
    std::uint32_t want_id = 0;
    std::uint64_t want = 0;
    if (!hook_of(fe.second, fe.first, want_id, want))
      continue;
    std::uint32_t id = 0;
    std::uint64_t digest = 0;
    if (pos + sizeof(id) + sizeof(digest) > in.size())
      throw cvc::physics_error("restore: the free-surface state is truncated");
    std::memcpy(&id, in.data() + pos, sizeof(id));
    std::memcpy(&digest, in.data() + pos + sizeof(id), sizeof(digest));
    if (id != want_id || digest != want)
      throw cvc::physics_error("restore: this world's free surfaces or winds differ from the "
                               "state's");
    pos += sizeof(id) + sizeof(digest);
  }
}

bool field_set::has_hulls() const {
  for (const auto &be : _bodies)
    if (be.second.hull.desc)
      return true;
  return false;
}

void field_set::save_hulls(std::string &out) const {
  for (const auto &be : _bodies) {
    const field_hull &h = be.second.hull;
    if (!h.desc)
      continue;
    const std::uint32_t id = be.first;
    const std::uint64_t n = h.wetted.size();
    out.append(reinterpret_cast<const char *>(&id), sizeof(id));
    out.append(reinterpret_cast<const char *>(&h.digest), sizeof(h.digest));
    out.append(reinterpret_cast<const char *>(&n), sizeof(n));
    out.append(reinterpret_cast<const char *>(h.wetted.data()), h.wetted.size() * sizeof(double));
  }
}

void field_set::restore_hulls(const std::string &in, std::size_t &pos) {
  // Checked whole before any is written, so a refused state leaves the world as it was.
  std::size_t at = pos;
  for (const auto &be : _bodies) {
    const field_hull &h = be.second.hull;
    if (!h.desc)
      continue;
    std::uint32_t id = 0;
    std::uint64_t digest = 0, n = 0;
    if (at + sizeof(id) + sizeof(digest) + sizeof(n) > in.size())
      throw cvc::physics_error("restore: the hull state is truncated");
    std::memcpy(&id, in.data() + at, sizeof(id));
    std::memcpy(&digest, in.data() + at + sizeof(id), sizeof(digest));
    std::memcpy(&n, in.data() + at + sizeof(id) + sizeof(digest), sizeof(n));
    at += sizeof(id) + sizeof(digest) + sizeof(n);
    if (id != be.first || digest != h.digest || n != h.wetted.size() ||
        at + n * sizeof(double) > in.size())
      throw cvc::physics_error("restore: this world's hulls differ from the state's");
    at += static_cast<std::size_t>(n) * sizeof(double);
  }
  for (auto &be : _bodies) {
    field_hull &h = be.second.hull;
    if (!h.desc)
      continue;
    pos += sizeof(std::uint32_t) + 2 * sizeof(std::uint64_t);
    std::memcpy(h.wetted.data(), in.data() + pos, h.wetted.size() * sizeof(double));
    pos += h.wetted.size() * sizeof(double);
  }
}

bool field_set::has_wakes() const {
  for (const auto &fe : _fields)
    if (fe.second.wake)
      return true;
  return false;
}

namespace {
// A wake's parameters as its section of save() records them, each one, so a world with any other
// wake refuses the state.
void put_wake_params(std::string &out, const field_wake &p) {
  const std::int32_t ints[3] = {p.cells, p.samples, p.sources};
  const double reals[7] = {p.cell,    p.depth,        p.period,     p.damping_k2,
                           p.damping, p.sponge_cells, p.sponge_rate};
  out.append(reinterpret_cast<const char *>(ints), sizeof(ints));
  out.append(reinterpret_cast<const char *>(reals), sizeof(reals));
}
bool get_wake_params(const std::string &in, std::size_t &pos, field_wake &p) {
  std::int32_t ints[3];
  double reals[7];
  if (pos > in.size() || sizeof(ints) + sizeof(reals) > in.size() - pos)
    return false;
  std::memcpy(ints, in.data() + pos, sizeof(ints));
  std::memcpy(reals, in.data() + pos + sizeof(ints), sizeof(reals));
  pos += sizeof(ints) + sizeof(reals);
  p.cells = ints[0];
  p.samples = ints[1];
  p.sources = ints[2];
  p.cell = reals[0];
  p.depth = reals[1];
  p.period = reals[2];
  p.damping_k2 = reals[3];
  p.damping = reals[4];
  p.sponge_cells = reals[5];
  p.sponge_rate = reals[6];
  return true;
}
} // namespace

void field_set::save_wakes(std::string &out) const {
  for (const auto &fe : _fields) {
    if (!fe.second.wake)
      continue;
    const std::uint32_t id = fe.first;
    out.append(reinterpret_cast<const char *>(&id), sizeof(id));
    put_wake_params(out, fe.second.desc.wake);
    // Version 2: its shore, flagged and named by its digest.
    const std::uint8_t shore = fe.second.desc.wake.shore ? 1 : 0;
    const std::uint64_t digest = fe.second.desc.wake.shore_digest();
    out.append(reinterpret_cast<const char *>(&shore), sizeof(shore));
    out.append(reinterpret_cast<const char *>(&digest), sizeof(digest));
    fe.second.wake->save(out);
  }
}

void field_set::restore_wakes(const std::string &in, std::size_t &pos, int version) {
  // Every wake read whole first, then all taken: a state that fails part-way changes no wake.
  std::vector<std::pair<fluid_wake *, std::shared_ptr<fluid_wake::saved>>> read;
  for (const auto &fe : _fields) {
    if (!fe.second.wake)
      continue;
    std::uint32_t id = 0;
    field_wake p;
    if (pos > in.size() || sizeof(id) > in.size() - pos)
      throw cvc::physics_error("restore: the wakes' state is truncated");
    std::memcpy(&id, in.data() + pos, sizeof(id));
    pos += sizeof(id);
    if (!get_wake_params(in, pos, p))
      throw cvc::physics_error("restore: the wakes' state is truncated");
    // A version 1 state is from before shores: a wake without one.
    std::uint8_t shore = 0;
    std::uint64_t digest = 0;
    if (version >= 2) {
      if (pos > in.size() || sizeof(shore) + sizeof(digest) > in.size() - pos)
        throw cvc::physics_error("restore: the wakes' state is truncated");
      std::memcpy(&shore, in.data() + pos, sizeof(shore));
      std::memcpy(&digest, in.data() + pos + sizeof(shore), sizeof(digest));
      pos += sizeof(shore) + sizeof(digest);
    }
    const field_wake &mine = fe.second.desc.wake;
    p.shore = mine.shore; // compared by its flag and digest below
    if (id != fe.first || p != mine || shore != (mine.shore ? 1 : 0) ||
        digest != mine.shore_digest())
      throw cvc::physics_error("restore: this world's wakes differ from the state's");
    read.emplace_back(fe.second.wake.get(), fe.second.wake->parse(in, pos, version));
  }
  for (auto &r : read)
    r.first->commit(*r.second);
}

wake_info field_set::wake(field_id id) const {
  const auto it = _fields.find(id.value);
  need(it != _fields.end(), "unknown or removed field id " + std::to_string(id.value));
  return it->second.wake ? it->second.wake->info() : wake_info{};
}

double field_set::wake_height(field_id id, double x, double y) const {
  const auto it = _fields.find(id.value);
  need(it != _fields.end(), "unknown or removed field id " + std::to_string(id.value));
  if (!it->second.wake)
    return 0.0;
  wake_sample ws;
  it->second.wake->sample(-1, x, y, ws);
  return ws.h;
}

const immersion_body *field_set::outline_of(field_body &fb) {
  if (!fb.cut_tried) {
    fb.cut_tried = true;
    try {
      if (fb.hull.desc)
        fb.cut = std::make_shared<const immersion_body>(*fb.hull.desc);
      else if (fb.shape == field_body::outline::box)
        fb.cut = std::make_shared<const immersion_body>(physics::shape{box{fb.half}});
      else if (fb.shape == field_body::outline::sphere)
        fb.cut = std::make_shared<const immersion_body>(physics::shape{sphere{fb.radius}});
      else if (fb.cut_shape)
        fb.cut = std::make_shared<const immersion_body>(*fb.cut_shape);
    } catch (const std::invalid_argument &) {
      fb.cut.reset(); // an outline the columns cannot cut: it makes no waves
    }
    fb.cut_shape.reset();
  }
  return fb.cut.get();
}

void field_set::advance_wakes(
    world_backend &be, const world_config &cfg, double t, double gravity,
    const std::function<const fluid_surface *(const compiled_field &)> &held) {
  // Every body the fields record, awake or asleep, where it is now: in id order. Read only at the
  // steps the waves step at.
  std::vector<wake_body> bodies;
  std::vector<std::uint32_t> wake_up;
  for (auto &fe : _fields) {
    compiled_field &cf = fe.second;
    double dt = 0.0;
    if (!cf.wake || !cf.wake->begin_step(cfg.fixed_dt, dt))
      continue;
    cf.wake->set_gravity(gravity);
    const field_grid &box = cf.scalar;
    if (bodies.empty() && !_bodies.empty()) {
      bodies.reserve(_bodies.size());
      for (auto &entry : _bodies) {
        body_id id;
        id.value = entry.first;
        const body_state s = be.state(id);
        wake_body b;
        b.id = entry.first;
        b.position = s.position;
        b.rotation = s.rotation;
        b.hull = entry.second.hull.desc != nullptr;
        b.active = s.active;
        body_box(entry.second, s, b.lo, b.hi);
        bodies.push_back(b);
      }
    }
    std::size_t k = 0;
    for (auto &entry : _bodies) {
      wake_body &b = bodies[k++];
      b.over = overlaps(b.lo, b.hi, box);
      b.in_water = false;
      b.outline = b.over ? outline_of(entry.second) : nullptr;
    }
    cf.wake->advance(
        bodies, {box.lo[0], box.lo[1], box.lo[2]}, {box.hi[0], box.hi[1], box.hi[2]},
        [&]() { return held(cf); }, t, dt, wake_up);
  }
  for (const std::uint32_t v : wake_up) {
    body_id id;
    id.value = v;
    be.wake(id);
  }
}

bool field_set::has_ruts() const {
  for (const auto &fe : _fields)
    if (!fe.second.ruts.depth.empty())
      return true;
  return false;
}

void field_set::save_ruts(std::string &out) const {
  for (const auto &fe : _fields) {
    const std::vector<float> &d = fe.second.ruts.depth;
    if (d.empty())
      continue;
    const std::uint32_t id = fe.first;
    const std::uint64_t n = d.size();
    out.append(reinterpret_cast<const char *>(&id), sizeof(id));
    out.append(reinterpret_cast<const char *>(&n), sizeof(n));
    out.append(reinterpret_cast<const char *>(d.data()), d.size() * sizeof(float));
  }
}

void field_set::restore_ruts(const std::string &in, std::size_t &pos) {
  for (auto &fe : _fields) {
    std::vector<float> &d = fe.second.ruts.depth;
    if (d.empty())
      continue;
    std::uint32_t id = 0;
    std::uint64_t n = 0;
    if (pos + sizeof(id) + sizeof(n) > in.size())
      throw cvc::physics_error("restore: the rut state is truncated");
    std::memcpy(&id, in.data() + pos, sizeof(id));
    std::memcpy(&n, in.data() + pos + sizeof(id), sizeof(n));
    pos += sizeof(id) + sizeof(n);
    if (id != fe.first || n != d.size() || pos + n * sizeof(float) > in.size())
      throw cvc::physics_error("restore: this world's rutted media differ from the state's");
    std::memcpy(d.data(), in.data() + pos, d.size() * sizeof(float));
    pos += d.size() * sizeof(float);
    fe.second.rut_cells = 0;
    fe.second.rut_deepest = 0.0;
    for (const float z : d)
      if (z > 0.0f) {
        ++fe.second.rut_cells;
        fe.second.rut_deepest = std::max(fe.second.rut_deepest, static_cast<double>(z));
      }
  }
}

double field_set::medium_reach() const {
  double reach = 0.0;
  for (const auto &fe : _fields)
    if (fe.second.desc.kind == field_kind::medium)
      reach = std::max(reach, fe.second.scalar.hi[2] - fe.second.scalar.lo[2]);
  return reach;
}

bool field_set::medium_under(const vec3 &p, wheel_medium &out) const {
  bool found = false;
  for (const auto &fe : _fields) {
    const compiled_field &cf = fe.second;
    if (cf.desc.kind != field_kind::medium)
      continue;
    const field_grid &g = cf.scalar;
    if (p.x < g.lo[0] || p.x > g.hi[0] || p.y < g.lo[1] || p.y > g.hi[1] || p.z >= g.hi[2])
      continue;
    // The medium's top over (x, y): down from the box's top to the contact, then bisected to
    // where the modulus falls to zero. A uniform medium's top is its box's.
    const double bottom = std::max(p.z, g.lo[2]);
    double dz = g.span[2] > 0.0 ? g.span[2] : g.hi[2] - g.lo[2];
    dz = std::max(1e-3, std::min(dz, (g.hi[2] - bottom) / 8.0));
    double q[3] = {p.x, p.y, g.hi[2]};
    double surface = std::nan("");
    for (double z = g.hi[2]; z >= bottom; z -= dz) {
      q[2] = z;
      if (g.at(q) > 0.0) {
        surface = z;
        break;
      }
    }
    if (std::isnan(surface))
      continue;
    if (surface < g.hi[2]) {
      double below = surface, above = std::min(surface + dz, g.hi[2]);
      for (int i = 0; i < 24; ++i) {
        q[2] = 0.5 * (below + above);
        (g.at(q) > 0.0 ? below : above) = q[2];
      }
      surface = below;
    }
    const double depth = surface - p.z;
    if (!(depth > 0.0) || (found && depth <= out.depth))
      continue;
    q[2] = std::max(g.lo[2], p.z + 0.5 * depth);
    out.depth = depth;
    out.modulus = g.at(q);
    out.traction = cf.desc.traction;
    out.rut = cf.ruts.at(p.x, p.y);
    out.field = fe.first;
    found = true;
  }
  return found;
}

// ── field_workers ──

field_workers::field_workers(int threads) : _wanted(threads) {
#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
  threads = 0; // no threads in this build: run() runs every index here
#endif
  for (int i = 0; i < threads; ++i)
    _threads.emplace_back([this] { loop(); });
}

field_workers::~field_workers() {
  {
    std::lock_guard<std::mutex> lock(_m);
    _stop = true;
  }
  _go.notify_all();
  for (std::thread &th : _threads)
    th.join();
}

void field_workers::work() {
  for (std::size_t i; (i = _next.fetch_add(1)) < _n;) {
    try {
      (*_fn)(i);
    } catch (...) {
      std::lock_guard<std::mutex> lock(_m);
      if (!_error)
        _error = std::current_exception();
    }
  }
}

void field_workers::loop() {
  std::uint64_t seen = 0;
  for (;;) {
    {
      std::unique_lock<std::mutex> lock(_m);
      _go.wait(lock, [&] { return _stop || _generation != seen; });
      if (_stop)
        return;
      seen = _generation;
    }
    work();
    std::lock_guard<std::mutex> lock(_m);
    if (--_busy == 0)
      _done.notify_one();
  }
}

void field_workers::run(std::size_t n, const std::function<void(std::size_t)> &fn) {
  if (_threads.empty() || n < 2) {
    for (std::size_t i = 0; i < n; ++i)
      fn(i);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(_m);
    _fn = &fn;
    _n = n;
    _next.store(0);
    _error = nullptr;
    _busy = static_cast<int>(_threads.size());
    ++_generation;
  }
  _go.notify_all();
  work(); // this thread takes indices too
  std::unique_lock<std::mutex> lock(_m);
  _done.wait(lock, [&] { return _busy == 0; });
  _fn = nullptr;
  if (_error) {
    std::exception_ptr e = _error;
    _error = nullptr;
    std::rethrow_exception(e);
  }
}

field_load field_set::load(body_id id) const {
  const auto it = _bodies.find(id.value);
  return it == _bodies.end() ? field_load{} : it->second.load;
}

void field_set::apply(world_backend &be, const world_config &cfg, double t) {
  for (auto &entry : _bodies)
    entry.second.load = field_load{};
  if (_fields.empty())
    return;
  const vec3 g = cfg.gravity;
  const double gn = norm(g);
  const vec3 up = gn > 0.0 ? g * (-1.0 / gn) : vec3{0.0, 0.0, 1.0};
  std::vector<vec3> world_v; // a hull's scratch: its vertices in the world, their depths, and
  std::vector<double> depth; // its triangles' submerged areas this step
  std::vector<double> wet;

  // The awake bodies and where they are now, read once, in id order.
  struct placed {
    body_id id;
    field_body *fb = nullptr;
    body_state s;
    mat3 r;
    double ext[3]; // a cell's world box: the box around the rotated cell
    double blo[3], bhi[3];
  };
  std::vector<placed> awake;
  awake.reserve(_bodies.size());
  for (auto &entry : _bodies) {
    placed pb;
    pb.id.value = entry.first;
    pb.fb = &entry.second;
    pb.s = be.state(pb.id);
    if (!pb.s.active)
      continue;
    pb.r = rotation(pb.s.rotation);
    const vec3 e = pb.r.abs_times(pb.fb->cell * 0.5);
    pb.ext[0] = e.x;
    pb.ext[1] = e.y;
    pb.ext[2] = e.z;
    body_box(*pb.fb, pb.s, pb.blo, pb.bhi);
    awake.push_back(pb);
  }

  // Each field's surface held at t (fluid_surface::at: its per-time work done as tasks of the field
  // workers), made once a step as first needed, by a wake or by the cells' reads below.
  std::vector<std::pair<const compiled_field *, std::shared_ptr<const fluid_surface>>> held;
  fluid_surface::parallel_run par; // the workers, once some read needs a surface
  const auto held_of = [&](const compiled_field &cf) -> const fluid_surface * {
    const fluid_surface *own = cf.desc.surface.get();
    for (const auto &h : held)
      if (h.first == &cf)
        return h.second ? h.second.get() : own;
    if (!par) {
      if (!_workers || _workers->threads() != cfg.field_threads)
        _workers = std::make_unique<field_workers>(cfg.field_threads);
      par = [this](std::size_t n, const std::function<void(std::size_t)> &fn) {
        _workers->run(n, fn);
      };
    }
    held.emplace_back(&cf, own->at(t, par)); // its per-time work, as tasks for the workers
    return held.back().second ? held.back().second.get() : own;
  };

  // The wakes (field::wake): their waves move on and the bodies' sources go in, before the reads.
  for (const auto &fe : _fields)
    if (fe.second.wake) {
      advance_wakes(be, cfg, t, gn, held_of);
      break;
    }
  // And their motion below the top, for the reads below (made here, before the field workers
  // read it): only for a wake an awake body is in, so the waves' steps while every body sleeps
  // make none.
  for (const auto &fe : _fields)
    if (fe.second.wake)
      for (const placed &pb : awake)
        if (overlaps(pb.blo, pb.bhi, fe.second.scalar)) {
          fe.second.wake->ready_depths();
          break;
        }
  // What body `id` reads of field `cf`'s wake: the others' waves (none without a wake).
  const auto view_of = [](const compiled_field &cf, body_id id) {
    wake_view v;
    if (cf.wake) {
      v.wake = cf.wake.get();
      v.own = cf.wake->own(id.value);
    }
    return v;
  };

  // The cells' reads of free surfaces, first: each cell's (read_cell) into its own slot, on the
  // field workers when world_config::field_threads asks for them. The sums below take them in the
  // cells' order, so the same values add up in the same order with or without workers.
  struct surface_cells {
    std::size_t body; // in awake
    const compiled_field *cf;
    std::size_t first;       // its first cell's slot in reads
    const fluid_surface *at; // the surface at t, held (fluid_surface::at), or the field's own
    wake_view wake;          // the others' waves, with a wake
  };
  std::vector<surface_cells> runs;
  std::vector<cell_read> reads;
  for (std::size_t bi = 0; bi < awake.size(); ++bi) {
    const placed &pb = awake[bi];
    if (pb.fb->hull.desc != nullptr)
      continue; // a hull's triangles take the fluids' push (push_hull)
    for (const auto &fe : _fields) {
      const compiled_field &cf = fe.second; // a surface is a fluid's (compile)
      if (!cf.desc.surface || !overlaps(pb.blo, pb.bhi, cf.scalar))
        continue;
      runs.push_back({bi, &cf, reads.size(), held_of(cf), view_of(cf, pb.id)});
      reads.resize(reads.size() + pb.fb->cells.size());
    }
  }
  if (!reads.empty()) {
    constexpr std::size_t kChunk = 32;
    _workers->run((reads.size() + kChunk - 1) / kChunk, [&](std::size_t k) {
      const std::size_t i0 = k * kChunk, i1 = std::min(reads.size(), i0 + kChunk);
      std::size_t ri = std::size_t(std::upper_bound(runs.begin(), runs.end(), i0,
                                                    [](std::size_t i, const surface_cells &sc) {
                                                      return i < sc.first;
                                                    }) -
                                   runs.begin()) -
                       1;
      for (std::size_t i = i0; i < i1; ++i) {
        while (ri + 1 < runs.size() && i >= runs[ri + 1].first)
          ++ri;
        const surface_cells &sc = runs[ri];
        const placed &pb = awake[sc.body];
        reads[i] = read_cell(pb.fb->cells[i - sc.first], pb.s, pb.r, pb.ext, *sc.cf, sc.at, t,
                             nullptr, sc.wake.wake ? &sc.wake : nullptr);
      }
    });
  }

  std::size_t run_at = 0;
  for (std::size_t bi = 0; bi < awake.size(); ++bi) {
    const placed &pb = awake[bi];
    const body_id id = pb.id;
    field_body &fb = *pb.fb;
    const body_state &s = pb.s;
    const mat3 &r = pb.r;
    const vec3 com = s.position + r * fb.center_of_mass;
    const double *ext = pb.ext;

    // Buoyancy and support (push) apart from drag and damping, which are scaled
    // for stability below.
    vec3 push_f, push_t, drag_f, drag_t;
    double rate = 0.0, immersed = 0.0;
    double rate_heave = 0.0; // the part of rate an inertial fluid's damping gives the vertical
    bool felt = false;       // a field pushed on one of its cells
    // A hull's triangles take the fluids' push in place of the cells (hull.h).
    const bool hull = fb.hull.desc != nullptr;
    push_sums hs;
    if (hull)
      wet.assign(fb.hull.wetted.size(), 0.0);
    for (const auto &fe : _fields) {
      const compiled_field &cf = fe.second;
      if (fb.vehicle && cf.desc.kind == field_kind::flow)
        continue; // a vehicle's aero is its drag (vehicle.h)
      if (!overlaps(pb.blo, pb.bhi, cf.scalar))
        continue;
      const wake_view view = view_of(cf, id);
      const wake_view *wv = view.wake ? &view : nullptr;
      if (hull && cf.desc.kind == field_kind::fluid) {
        push_hull(fb, s, r, com, cf, gn, cfg.fixed_dt, t, world_v, depth, wet, hs, wv);
        continue;
      }
      // A flow's wind, once per body (body_wind, flow_velocity.h).
      const flow_velocity *wind = cf.desc.wind.get();
      body_wind bw;
      if (wind) {
        bw.floor = wind->base();
        const double zlo = std::max(pb.blo[2], bw.floor);
        if (!(pb.bhi[2] > zlo))
          continue; // all of the body is below the air (under the water or the ground)
        bw.reference = wind->reference(com, t);
        bw.z0 = zlo;
        bw.span = pb.bhi[2] - zlo;
        for (int j = 0; j < kProfiles; ++j) {
          const double sj = static_cast<double>(j) / (kProfiles - 1);
          bw.z[j] = zlo + bw.span * sj * sj;
          bw.prof[j] = wind->profile(bw.z[j]);
        }
      }
      const fluid_surface *surface = cf.desc.surface.get();
      // An inertial fluid (field.h): its damping on the vertical only, and the water's
      // acceleration over the immersed part, from its displaced mass and centroid.
      const bool inertial = cf.desc.inertial && cf.desc.kind == field_kind::fluid;
      double displaced = 0.0;       // kg of water the body displaces in this field
      vec3 displaced_at;            // their moment, kg m
      double displaced_depth = 0.0; // and of their depths below the top, kg m
      // The cells' reads: done above for a free surface (a fluid's, never a wind's), else here.
      const cell_read *pre = nullptr;
      if (run_at < runs.size() && runs[run_at].body == bi && runs[run_at].cf == &cf)
        pre = reads.data() + runs[run_at++].first;
      for (std::size_t ci = 0; ci < fb.cells.size(); ++ci) {
        const cell_read cr = pre ? pre[ci]
                                 : read_cell(fb.cells[ci], s, r, ext, cf, cf.desc.surface.get(), t,
                                             wind ? &bw : nullptr, wv);
        if (!cr.in)
          continue;
        const double fv = cr.f * fb.cell_volume;
        const vec3 at{cr.p[0], cr.p[1], cr.p[2]};
        const vec3 arm = at - com;
        felt = true;
        if (!wind) // a body is not immersed in the air
          immersed += fv;

        if (cf.desc.kind == field_kind::fluid) {
          // A body displaces only its share (body_desc::displacement; a flooded chassis,
          // vehicle.h's wading).
          const vec3 b = g * (-cr.value * fv * fb.buoyancy);
          push_f = push_f + b;
          push_t = push_t + cross(arm, b);
          if (inertial) {
            displaced += cr.value * fv * fb.buoyancy;
            displaced_at = displaced_at + at * (cr.value * fv * fb.buoyancy);
            displaced_depth += cr.depth * (cr.value * fv * fb.buoyancy);
          }
        } else if (cf.desc.kind == field_kind::medium) {
          const vec3 b = up * (cr.value * fv);
          push_f = push_f + b;
          push_t = push_t + cross(arm, b);
        }

        const vec3 u = cr.w - (s.linear_velocity + cross(s.angular_velocity, arm));
        double c_i = cf.desc.damping * fv * fb.water_share; // kg/s: the force is c_i u
        if (inertial) {                    // the damping on the vertical only
          if (c_i > 0.0) {
            const vec3 d{0.0, 0.0, u.z * c_i};
            drag_f = drag_f + d;
            drag_t = drag_t + cross(arm, d);
            rate += c_i;
            rate_heave += c_i;
          }
          c_i = 0.0;
        }
        if (cf.desc.kind != field_kind::medium) {
          const double speed = norm(u);
          // A fluid's drag on the body's share of its cells (field_body::water_share).
          const double share = cf.desc.kind == field_kind::fluid ? fb.water_share : 1.0;
          if (speed > 0.0)
            c_i += 0.5 * cr.value * cf.desc.drag_coefficient *
                   area_across(fb, r.inverse(u * (1.0 / speed))) * (fv / fb.volume) * speed * share;
        }
        if (c_i > 0.0) {
          const vec3 d = u * c_i;
          drag_f = drag_f + d;
          drag_t = drag_t + cross(arm, d);
          rate += c_i;
        }
      }
      // The water's acceleration pushes what it holds (Froude-Krylov): rho V du/dt sideways at
      // the immersed part's centroid, du/dt the water's velocity a step either side.
      if (inertial && displaced > 0.0 && surface) {
        const vec3 at = displaced_at * (1.0 / displaced);
        const double h = cfg.fixed_dt;
        const vec3 dudt =
            (surface->velocity(at, t + h) - surface->velocity(at, t - h)) * (1.0 / (2.0 * h));
        vec3 fk{displaced * dudt.x, displaced * dudt.y, 0.0};
        if (wv) { // and the others' waves': du/dt = grad(dphi/dt) = -g grad h, at the displaced
                  // water's centroid and mean depth (as the surface's is read at its centroid)
          wake_sample ws;
          wv->motion(at.x, at.y, displaced_depth / displaced, ws);
          fk.x += displaced * (-gn * ws.gx);
          fk.y += displaced * (-gn * ws.gy);
        }
        push_f = push_f + fk;
        push_t = push_t + cross(at - com, fk);
      }
    }
    if (hull) {
      fb.hull.wetted.swap(wet);
      if (!hs.wet && !felt)
        continue;
      push_f = push_f + hs.push_f;
      push_t = push_t + hs.push_t;
      drag_f = drag_f + hs.drag_f;
      drag_t = drag_t + hs.drag_t;
      rate += hs.rate;
      immersed += hs.immersed;
    } else if (!felt) {
      continue;
    }
    // Drag integrated implicitly in its own rate: a step never carries the body
    // past the stuff's velocity, however light the body or dense the fluid.
    const double x = rate * cfg.fixed_dt / fb.mass;
    const double k = x > 1e-9 ? -det::expm1(-x) / x : 1.0;
    // A hull's drag torque also in its rotational rate, over a low estimate of the inertia.
    double kt = k;
    if (hull && hs.rate_rot > 0.0 && fb.hull.inertia > 0.0) {
      const double xr = hs.rate_rot * cfg.fixed_dt / (fb.mass * fb.hull.inertia);
      kt = std::min(k, xr > 1e-9 ? -det::expm1(-xr) / xr : 1.0);
    }
    field_load load;
    load.force = push_f + drag_f * k;
    const double heave = (hull ? hs.rate_heave : 0.0) + rate_heave;
    if (heave > 0.0) {
      // A hull's damping, and an inertial fluid's, acts on vertical motion only, so the
      // horizontal drag is scaled in the rate without it: the damping does not slow a towed hull
      // or a drifting crate. Each axis still stops short of the water's velocity, since only z
      // carries the damping's force.
      const double xh = (rate - heave) * cfg.fixed_dt / fb.mass;
      const double kh = xh > 1e-9 ? -det::expm1(-xh) / xh : 1.0;
      load.force.x = push_f.x + drag_f.x * kh;
      load.force.y = push_f.y + drag_f.y * kh;
    }
    load.torque = push_t + drag_t * kt;
    load.immersed_volume = immersed;
    be.add_field_load(id, load.force, load.torque);
    fb.load = load;
  }
}

} // namespace detail

std::shared_ptr<const hull_mesh> box_hull(const vec3 &half_extents) {
  auto h = std::make_shared<hull_mesh>();
  const vec3 e = half_extents;
  // Vertex i + 2 j + 4 k sits at (+-x, +-y, +-z), + where its bit is set.
  for (int k = 0; k < 2; ++k)
    for (int j = 0; j < 2; ++j)
      for (int i = 0; i < 2; ++i)
        h->vertices.push_back({i ? e.x : -e.x, j ? e.y : -e.y, k ? e.z : -e.z});
  h->triangles = {{0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6}, {0, 1, 5}, {0, 5, 4},
                  {2, 6, 7}, {2, 7, 3}, {0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}};
  return h;
}

} // namespace physics
} // namespace cvc
