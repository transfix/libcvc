/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "field_step.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/core/exception.h>
#include <cvc/volume/volume.h>
#include <string>

namespace cvc {
namespace physics {
namespace detail {

namespace {

constexpr double kPi = 3.14159265358979323846;
// Sample cells along a body's longest side.
constexpr int kCellsAcross = 8;

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

compiled_field compile(const field &f) {
  need(f.kind <= field_kind::flow, "bad kind");
  need(f.volume != nullptr, "no volume");
  need(std::isfinite(f.drag_coefficient) && f.drag_coefficient >= 0.0,
       "drag_coefficient must be finite and >= 0");
  need(std::isfinite(f.damping) && f.damping >= 0.0, "damping must be finite and >= 0");
  need(std::isfinite(f.traction) && f.traction >= 0.0, "traction must be finite and >= 0");
  const int components =
      (f.velocity[0] ? 1 : 0) + (f.velocity[1] ? 1 : 0) + (f.velocity[2] ? 1 : 0);
  need(components == 0 || components == 3, "give all three velocity components or none");
  compiled_field c;
  c.desc = f;
  c.scalar = grid_of(
      *f.volume, f.kind == field_kind::medium ? "the bearing modulus volume" : "the density volume",
      true);
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
  return c;
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
  const double r = std::cbrt(3.0 * fb.volume / (4.0 * kPi));
  return kPi * r * r;
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
  compiled_field c = compile(f);
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
  fb.cell_volume = fb.volume / static_cast<double>(fb.cells.size());
  fb.mass = d.mass > 0.0 ? d.mass : d.density * fb.volume;
  bounds_of(fb);
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

void field_set::mark_vehicle(body_id id, const vec3 &center_of_mass) {
  auto it = _bodies.find(id.value);
  if (it == _bodies.end())
    return;
  it->second.vehicle = true;
  it->second.center_of_mass = center_of_mass;
}

bool field_set::air_at(const vec3 &p, double &rho, vec3 &wind) const {
  const double q[3] = {p.x, p.y, p.z};
  for (const auto &fe : _fields) {
    const compiled_field &cf = fe.second;
    if (cf.desc.kind != field_kind::flow || !cf.scalar.contains(q))
      continue;
    rho = cf.scalar.at(q);
    wind = vec3{};
    if (cf.moving)
      wind = {cf.velocity[0].contains(q) ? cf.velocity[0].at(q) : 0.0,
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

field_load field_set::load(body_id id) const {
  const auto it = _bodies.find(id.value);
  return it == _bodies.end() ? field_load{} : it->second.load;
}

void field_set::apply(world_backend &be, const world_config &cfg) {
  for (auto &entry : _bodies)
    entry.second.load = field_load{};
  if (_fields.empty())
    return;
  const vec3 g = cfg.gravity;
  const double gn = norm(g);
  const vec3 up = gn > 0.0 ? g * (-1.0 / gn) : vec3{0.0, 0.0, 1.0};

  for (auto &entry : _bodies) {
    body_id id;
    id.value = entry.first;
    field_body &fb = entry.second;
    const body_state s = be.state(id);
    if (!s.active)
      continue;
    const mat3 r = rotation(s.rotation);
    const vec3 com = s.position + r * fb.center_of_mass;
    // A cell's world box: the box around the rotated cell.
    const vec3 e = r.abs_times(fb.cell * 0.5);
    const double ext[3] = {e.x, e.y, e.z};
    double blo[3], bhi[3];
    body_box(fb, s, blo, bhi);

    // Buoyancy and support (push) apart from drag and damping, which are scaled
    // for stability below.
    vec3 push_f, push_t, drag_f, drag_t;
    double rate = 0.0, immersed = 0.0;
    for (const auto &fe : _fields) {
      const compiled_field &cf = fe.second;
      if (fb.vehicle && cf.desc.kind == field_kind::flow)
        continue; // a vehicle's aero is its drag (vehicle.h)
      if (!overlaps(blo, bhi, cf.scalar))
        continue;
      for (const vec3 &local : fb.cells) {
        const vec3 c = s.position + r * local;
        // The immersed part of the cell: its world box clipped to the field's.
        double p[3], f = 1.0;
        bool in = true;
        for (int k = 0; k < 3 && in; ++k) {
          const double lo = std::max(component(c, k) - ext[k], cf.scalar.lo[k]);
          const double hi = std::min(component(c, k) + ext[k], cf.scalar.hi[k]);
          in = hi > lo;
          p[k] = 0.5 * (lo + hi);
          f *= (hi - lo) / (2.0 * ext[k]);
        }
        if (!in)
          continue;
        const double value = cf.scalar.at(p);
        if (!(value > 0.0))
          continue;
        const double fv = f * fb.cell_volume;
        const vec3 at{p[0], p[1], p[2]};
        const vec3 arm = at - com;
        immersed += fv;

        if (cf.desc.kind == field_kind::fluid) {
          const vec3 b = g * (-value * fv);
          push_f = push_f + b;
          push_t = push_t + cross(arm, b);
        } else if (cf.desc.kind == field_kind::medium) {
          const vec3 b = up * (value * fv);
          push_f = push_f + b;
          push_t = push_t + cross(arm, b);
        }

        vec3 w;
        if (cf.moving)
          w = {cf.velocity[0].contains(p) ? cf.velocity[0].at(p) : 0.0,
               cf.velocity[1].contains(p) ? cf.velocity[1].at(p) : 0.0,
               cf.velocity[2].contains(p) ? cf.velocity[2].at(p) : 0.0};
        const vec3 u = w - (s.linear_velocity + cross(s.angular_velocity, arm));
        double c_i = cf.desc.damping * fv; // kg/s: the force is c_i u
        if (cf.desc.kind != field_kind::medium) {
          const double speed = norm(u);
          if (speed > 0.0)
            c_i += 0.5 * value * cf.desc.drag_coefficient *
                   area_across(fb, r.inverse(u * (1.0 / speed))) * (fv / fb.volume) * speed;
        }
        if (c_i > 0.0) {
          const vec3 d = u * c_i;
          drag_f = drag_f + d;
          drag_t = drag_t + cross(arm, d);
          rate += c_i;
        }
      }
    }
    if (!(immersed > 0.0))
      continue;
    // Drag integrated implicitly in its own rate: a step never carries the body
    // past the stuff's velocity, however light the body or dense the fluid.
    const double x = rate * cfg.fixed_dt / fb.mass;
    const double k = x > 1e-9 ? -std::expm1(-x) / x : 1.0;
    field_load load;
    load.force = push_f + drag_f * k;
    load.torque = push_t + drag_t * k;
    load.immersed_volume = immersed;
    be.add_field_load(id, load.force, load.torque);
    fb.load = load;
  }
}

} // namespace detail
} // namespace physics
} // namespace cvc
