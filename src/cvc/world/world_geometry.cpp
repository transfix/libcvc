/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/volume/volume.h>
#include <cvc/world/world_geometry.h>
#include <set>
#include <stdexcept>

namespace cvc {
namespace world {

std::shared_ptr<const cvc::height_field> geometry_item::height_field() const {
  return std::dynamic_pointer_cast<const cvc::height_field>(geometry);
}

world_geometry::world_geometry(const surface_registry &surfaces) : _surfaces(&surfaces) {}

void world_geometry::add(geometry_item item) {
  if (item.name.empty())
    throw std::invalid_argument("world_geometry::add: an item needs a name");
  if (find(item.name) || find_volume(item.name))
    throw std::invalid_argument("world_geometry::add: an item called '" + item.name +
                                "' already exists");
  if (!item.geometry)
    throw std::invalid_argument("world_geometry::add: '" + item.name + "' has no geometry");
  const std::size_t n = _surfaces->size();
  if (item.surface >= n)
    throw std::invalid_argument("world_geometry::add: '" + item.name +
                                "': surface id outside the surface table");
  auto check = [&](const std::vector<cvc::face_material_t> &ids, const char *what) {
    for (const cvc::face_material_t id : ids)
      if (id >= n)
        throw std::invalid_argument("world_geometry::add: '" + item.name + "': a " + what +
                                    " is outside the surface table");
  };
  if (const auto hf = item.height_field())
    check(hf->cell_materials(), "cell material");
  else if (item.geometry->has_face_materials())
    check(item.geometry->const_face_materials(), "face material");
  const auto &q = item.rotation;
  const double len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!(std::fabs(len - 1.0) < 1e-6))
    throw std::invalid_argument("world_geometry::add: '" + item.name +
                                "': the rotation is not a unit quaternion");
  for (const double p : item.position)
    if (!std::isfinite(p))
      throw std::invalid_argument("world_geometry::add: '" + item.name +
                                  "': the position is not finite");
  _items.push_back(std::move(item));
}

const geometry_item *world_geometry::find(const std::string &name) const {
  for (const geometry_item &i : _items)
    if (i.name == name)
      return &i;
  return nullptr;
}

void world_geometry::add_volume(volume_item item) {
  if (item.name.empty())
    throw std::invalid_argument("world_geometry::add_volume: a volume needs a name");
  if (find(item.name) || find_volume(item.name))
    throw std::invalid_argument("world_geometry::add_volume: an item called '" + item.name +
                                "' already exists");
  if (!item.field.volume)
    throw std::invalid_argument("world_geometry::add_volume: '" + item.name + "' has no volume");
  _volumes.push_back(std::move(item));
}

const volume_item *world_geometry::find_volume(const std::string &name) const {
  for (const volume_item &v : _volumes)
    if (v.name == name)
      return &v;
  return nullptr;
}

std::shared_ptr<const cvc::height_field> world_geometry::terrain() const {
  for (const geometry_item &i : _items)
    if (auto hf = i.height_field())
      return hf;
  return nullptr;
}

world_bounds world_geometry::bounds() const {
  world_bounds b;
  auto grow = [&](double x, double y, double z) {
    const double v[3] = {x, y, z};
    for (int k = 0; k < 3; ++k) {
      b.min[k] = std::min(b.min[k], v[k]);
      b.max[k] = std::max(b.max[k], v[k]);
    }
  };
  for (const geometry_item &i : _items) {
    const auto &pts = i.geometry->const_points();
    if (pts.empty())
      continue;
    const bool placed = !i.height_field();
    const auto &q = i.rotation;
    for (const auto &p : pts) {
      double x = p[0], y = p[1], z = p[2];
      if (placed) {
        // v' = q v q*, then the translation.
        const double w = q[0], qx = q[1], qy = q[2], qz = q[3];
        const double tx = 2 * (qy * z - qz * y), ty = 2 * (qz * x - qx * z),
                     tz = 2 * (qx * y - qy * x);
        const double rx = x + w * tx + (qy * tz - qz * ty);
        const double ry = y + w * ty + (qz * tx - qx * tz);
        const double rz = z + w * tz + (qx * ty - qy * tx);
        x = rx + i.position[0];
        y = ry + i.position[1];
        z = rz + i.position[2];
      }
      if (b.empty()) {
        b.min = b.max = {{x, y, z}};
      } else {
        grow(x, y, z);
      }
    }
  }
  for (const volume_item &v : _volumes) {
    const cvc::bounding_box &box = v.field.volume->boundingBox();
    if (b.empty()) {
      b.min = {{box.minx, box.miny, box.minz}};
      b.max = {{box.maxx, box.maxy, box.maxz}};
    } else {
      grow(box.minx, box.miny, box.minz);
      grow(box.maxx, box.maxy, box.maxz);
    }
  }
  return b;
}

std::vector<std::uint16_t> world_geometry::surfaces_used() const {
  std::set<std::uint16_t> used;
  for (const geometry_item &i : _items) {
    if (const auto hf = i.height_field()) {
      if (hf->cell_materials().empty())
        used.insert(i.surface);
      else
        used.insert(hf->cell_materials().begin(), hf->cell_materials().end());
    } else if (i.geometry->has_face_materials()) {
      const auto &fm = i.geometry->const_face_materials();
      used.insert(fm.begin(), fm.end());
    } else {
      used.insert(i.surface);
    }
  }
  return std::vector<std::uint16_t>(used.begin(), used.end());
}

namespace {

// p rotated by the unit quaternion q (w, x, y, z), then moved by t.
std::array<double, 3> placed(const std::array<double, 3> &p, const std::array<double, 4> &q,
                             const std::array<double, 3> &t) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  // v' = v + 2 w (u x v) + 2 u x (u x v), u the quaternion's vector part
  const double cx = y * p[2] - z * p[1], cy = z * p[0] - x * p[2], cz = x * p[1] - y * p[0];
  const double ccx = y * cz - z * cy, ccy = z * cx - x * cz, ccz = x * cy - y * cx;
  return {p[0] + 2.0 * (w * cx + ccx) + t[0], p[1] + 2.0 * (w * cy + ccy) + t[1],
          p[2] + 2.0 * (w * cz + ccz) + t[2]};
}

bool inside(const world_rect &r, double x, double y) {
  return x >= r.min_x && x <= r.max_x && y >= r.min_y && y <= r.max_y;
}

// The samples [lo, hi] of an axis (origin o, spacing d, n samples) that cover [a, b]: whole cells,
// at least one; lo > hi when [a, b] misses the axis.
void cover(double o, double d, int n, double a, double b, int &lo, int &hi) {
  lo = std::max(0, static_cast<int>(std::floor((a - o) / d)));
  hi = std::min(n - 1, static_cast<int>(std::ceil((b - o) / d)));
  if (hi == lo && hi + 1 < n)
    ++hi;
  else if (hi == lo && lo > 0)
    --lo;
}

std::shared_ptr<const cvc::height_field> crop_field(const cvc::height_field &f,
                                                    const world_rect &r) {
  int c0, c1, r0, r1;
  cover(f.x0(), f.dx(), f.cols(), r.min_x, r.max_x, c0, c1);
  cover(f.y0(), f.dy(), f.rows(), r.min_y, r.max_y, r0, r1);
  if (c1 <= c0 || r1 <= r0)
    return nullptr;
  const int rows = r1 - r0 + 1, cols = c1 - c0 + 1;
  std::vector<float> h(static_cast<std::size_t>(rows) * cols);
  for (int i = 0; i < rows; ++i)
    for (int j = 0; j < cols; ++j)
      h[static_cast<std::size_t>(i) * cols + j] = f.height_at(r0 + i, c0 + j);
  std::vector<cvc::height_field::face_material_t> m;
  if (!f.cell_materials().empty()) {
    m.resize(static_cast<std::size_t>(rows - 1) * (cols - 1));
    for (int i = 0; i + 1 < rows; ++i)
      for (int j = 0; j + 1 < cols; ++j)
        m[static_cast<std::size_t>(i) * (cols - 1) + j] = f.cell_material(r0 + i, c0 + j);
  }
  return std::make_shared<const cvc::height_field>(rows, cols, f.x0() + c0 * f.dx(),
                                                   f.y0() + r0 * f.dy(), f.dx(), f.dy(),
                                                   std::move(h), std::move(m));
}

// A mesh item's faces (and lines) inside r, its unused vertices dropped; null when none are.
std::shared_ptr<const cvc::geometry> crop_mesh(const geometry_item &it, const world_rect &r) {
  const cvc::geometry &g = *it.geometry;
  const auto &pts = g.const_points();
  auto at = [&](std::size_t v) {
    return placed({pts[v][0], pts[v][1], pts[v][2]}, it.rotation, it.position);
  };
  std::vector<std::size_t> tris, lines;
  for (std::size_t t = 0; t < g.const_tris().size(); ++t) {
    const auto &f = g.const_tris()[t];
    const auto a = at(f[0]), b = at(f[1]), c = at(f[2]);
    if (inside(r, (a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0))
      tris.push_back(t);
  }
  for (std::size_t l = 0; l < g.const_lines().size(); ++l) {
    const auto a = at(g.const_lines()[l][0]), b = at(g.const_lines()[l][1]);
    if (inside(r, (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0))
      lines.push_back(l);
  }
  if (tris.empty() && lines.empty())
    return nullptr;
  if (tris.size() == g.const_tris().size() && lines.size() == g.const_lines().size())
    return it.geometry; // all of it: shared, not copied
  const std::size_t nv = pts.size();
  std::vector<std::int64_t> remap(nv, -1);
  std::vector<std::size_t> order;
  auto use = [&](std::size_t v) {
    if (remap[v] < 0) {
      remap[v] = static_cast<std::int64_t>(order.size());
      order.push_back(v);
    }
  };
  for (const std::size_t t : tris)
    for (int k = 0; k < 3; ++k)
      use(g.const_tris()[t][k]);
  for (const std::size_t l : lines)
    for (int k = 0; k < 2; ++k)
      use(g.const_lines()[l][k]);
  auto out = std::make_shared<cvc::geometry>();
  const bool normals = g.const_normals().size() == nv, colors = g.const_colors().size() == nv,
             uvs = g.const_uvs().size() == nv;
  for (const std::size_t v : order) {
    out->points().push_back(pts[v]);
    if (normals)
      out->normals().push_back(g.const_normals()[v]);
    if (colors)
      out->colors().push_back(g.const_colors()[v]);
    if (uvs)
      out->uvs().push_back(g.const_uvs()[v]);
  }
  const auto &fm = g.const_face_materials();
  const bool per_face = fm.size() == g.const_tris().size() && !fm.empty();
  for (const std::size_t t : tris) {
    cvc::geometry::tri_t nt;
    for (int k = 0; k < 3; ++k)
      nt[k] = static_cast<cvc::geometry::index_t>(remap[g.const_tris()[t][k]]);
    out->tris().push_back(nt);
    if (per_face)
      out->face_materials().push_back(fm[t]);
  }
  for (const std::size_t l : lines) {
    cvc::geometry::line_t nl;
    for (int k = 0; k < 2; ++k)
      nl[k] = static_cast<cvc::geometry::index_t>(remap[g.const_lines()[l][k]]);
    out->lines().push_back(nl);
  }
  return out;
}

} // namespace

world_geometry crop_world_geometry(const world_geometry &w, const world_rect &r) {
  if (!r.valid())
    throw std::invalid_argument("crop_world_geometry: the region's max must be past its min");
  world_geometry out(w.surfaces());
  out.set_edge_walls(w.edge_walls());
  bool terrain = false, any_field = false;
  for (const geometry_item &it : w.items()) {
    geometry_item c = it;
    if (const std::shared_ptr<const cvc::height_field> f = it.height_field()) {
      any_field = true;
      c.geometry = crop_field(*f, r);
      terrain = terrain || c.geometry != nullptr;
    } else {
      c.geometry = crop_mesh(it, r);
    }
    if (c.geometry)
      out.add(std::move(c));
  }
  if (any_field && !terrain)
    throw std::invalid_argument("crop_world_geometry: the region misses the terrain");
  for (const volume_item &v : w.volumes())
    out.add_volume(v);
  return out;
}

} // namespace world
} // namespace cvc
