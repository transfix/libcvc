/*
  Copyright 2024 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License version 2.1 as published by the
  Free Software Foundation.
*/

#include <cmath>
#include <cvc/model/model.h>
#include <utility>

namespace cvc {

model::transform_t model::identity_transform() {
  transform_t t;
  t.fill(0.0);
  t[0] = t[5] = t[10] = t[15] = 1.0;
  return t;
}

bool model::is_identity(const transform_t &t) { return t == identity_transform(); }

model::transform_t model::compose(const transform_t &a, const transform_t &b) {
  transform_t r;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k)
        s += a[i * 4 + k] * b[k * 4 + j];
      r[i * 4 + j] = s;
    }
  return r;
}

// ------------------
// model::instances
// ------------------
// Purpose:
//   A depth-first walk of the scene graph from nodes[0], on an explicit stack (a
//   file's tree can be deep), composing each node's transform onto its parent's.
//   on_path marks the nodes between the root and the one being visited, so a
//   child that is its own ancestor is skipped rather than walked forever.
std::vector<model::instance> model::instances() const {
  std::vector<instance> out;
  if (nodes.empty()) {
    out.reserve(meshes.size());
    for (std::size_t i = 0; i < meshes.size(); ++i) {
      instance in;
      in.mesh = static_cast<int>(i);
      out.push_back(in);
    }
    return out;
  }
  struct frame {
    int node;
    transform_t transform;
    std::size_t next_child;
  };
  std::vector<frame> stack;
  std::vector<char> on_path(nodes.size(), 0);
  const int n_meshes = static_cast<int>(meshes.size());
  const int n_nodes = static_cast<int>(nodes.size());
  const auto enter = [&](int n, const transform_t &parent) {
    const transform_t t = compose(parent, nodes[n].transform);
    for (int m : nodes[n].meshes)
      if (m >= 0 && m < n_meshes) {
        instance in;
        in.mesh = m;
        in.node = n;
        in.transform = t;
        out.push_back(in);
      }
    on_path[n] = 1;
    stack.push_back(frame{n, t, 0});
  };
  enter(0, identity_transform());
  while (!stack.empty()) {
    frame &f = stack.back();
    const std::vector<int> &children = nodes[f.node].children;
    if (f.next_child < children.size()) {
      const int c = children[f.next_child++];
      if (c >= 0 && c < n_nodes && !on_path[c]) {
        const transform_t parent = f.transform; // enter() may move the stack
        enter(c, parent);
      }
    } else {
      on_path[f.node] = 0;
      stack.pop_back();
    }
  }
  return out;
}

// ---------------
// model::placed
// ---------------
// Purpose:
//   Points take the full affine transform. Normals take the inverse transpose of
//   its upper 3x3, which is the cofactor matrix over the determinant: the
//   cofactor matrix times the determinant's sign keeps the direction, and the
//   result is made unit. Tangents (a direction in the surface) take the 3x3
//   itself. A mirror (negative determinant) would turn every face inside out, so
//   triangles and quads are rewound and the tangents' handedness flipped. Lines
//   and volume elements keep their indices; their points move.
geometry model::placed(const geometry &g, const transform_t &t) {
  if (is_identity(t))
    return g;
  geometry out = g; // COW copy; each accessor below detaches what it changes
  const double a00 = t[0], a01 = t[1], a02 = t[2];
  const double a10 = t[4], a11 = t[5], a12 = t[6];
  const double a20 = t[8], a21 = t[9], a22 = t[10];
  {
    geometry::points_t &pts = out.points();
    for (geometry::point_t &p : pts) {
      const double x = p[0], y = p[1], z = p[2];
      p[0] = a00 * x + a01 * y + a02 * z + t[3];
      p[1] = a10 * x + a11 * y + a12 * z + t[7];
      p[2] = a20 * x + a21 * y + a22 * z + t[11];
    }
  }
  const double c00 = a11 * a22 - a12 * a21, c01 = a12 * a20 - a10 * a22,
               c02 = a10 * a21 - a11 * a20;
  const double c10 = a02 * a21 - a01 * a22, c11 = a00 * a22 - a02 * a20,
               c12 = a01 * a20 - a00 * a21;
  const double c20 = a01 * a12 - a02 * a11, c21 = a02 * a10 - a00 * a12,
               c22 = a00 * a11 - a01 * a10;
  const double det = a00 * c00 + a01 * c01 + a02 * c02;
  const bool mirrors = det < 0.0;
  const auto unit = [](double &x, double &y, double &z) {
    const double len = std::sqrt(x * x + y * y + z * z);
    if (len > 0.0) {
      x /= len;
      y /= len;
      z /= len;
    }
    return len > 0.0;
  };
  if (!g.const_normals().empty()) {
    const double s = mirrors ? -1.0 : 1.0;
    for (geometry::normal_t &n : out.normals()) {
      const double x = n[0], y = n[1], z = n[2];
      double nx = s * (c00 * x + c01 * y + c02 * z);
      double ny = s * (c10 * x + c11 * y + c12 * z);
      double nz = s * (c20 * x + c21 * y + c22 * z);
      if (unit(nx, ny, nz)) { // a degenerate transform keeps the normal it had
        n[0] = nx;
        n[1] = ny;
        n[2] = nz;
      }
    }
  }
  if (!g.const_tangents().empty())
    for (geometry::tangent_t &tg : out.tangents()) {
      const double x = tg[0], y = tg[1], z = tg[2];
      double tx = a00 * x + a01 * y + a02 * z;
      double ty = a10 * x + a11 * y + a12 * z;
      double tz = a20 * x + a21 * y + a22 * z;
      if (unit(tx, ty, tz)) {
        tg[0] = tx;
        tg[1] = ty;
        tg[2] = tz;
      }
      if (mirrors)
        tg[3] = -tg[3];
    }
  if (mirrors) {
    if (!g.const_tris().empty())
      for (geometry::tri_t &tri : out.tris())
        std::swap(tri[1], tri[2]);
    if (!g.const_quads().empty())
      for (geometry::quad_t &q : out.quads())
        std::swap(q[1], q[3]);
  }
  return out;
}

// ---------------
// model::merged
// ---------------
// Purpose:
//   Concatenate every instance's mesh, placed, into a single geometry.
//   cvc::geometry::merge already appends the per-vertex attribute arrays and
//   offsets the appended line/tri/quad indices by the running vertex count, so a
//   fold over merge() produces the correct flattened index buffer.
geometry model::merged() const {
  // geometry::merge appends the per-vertex attribute arrays (normals/colors/uvs/
  // tangents) by raw insert, WITHOUT padding them to the vertex count — it only
  // offsets the appended index buffers. So merging meshes with heterogeneous
  // attribute presence (e.g. one textured mesh carrying UVs+tangents and one
  // untextured mesh carrying neither, which is routine for a multi-material
  // OBJ/glTF) would leave a merged array shorter than points() and misaligned
  // relative to the correctly-offset triangle indices — a consumer indexing
  // uvs[vertexIndex] then reads out of bounds or the wrong vertex. Normalize
  // first: if ANY mesh carries an attribute, pad the meshes that lack it to full
  // length with a neutral default, so every per-vertex array in the result stays
  // exactly num_points() long and index-aligned.
  const std::vector<instance> drawn = instances();
  bool any_normals = false, any_colors = false, any_uvs = false, any_tangents = false;
  for (const instance &in : drawn) {
    const geometry &g = meshes[in.mesh].geom;
    any_normals = any_normals || !g.const_normals().empty();
    any_colors = any_colors || !g.const_colors().empty();
    any_uvs = any_uvs || !g.const_uvs().empty();
    any_tangents = any_tangents || !g.const_tangents().empty();
  }

  geometry::normal_t def_n;
  def_n[0] = 0;
  def_n[1] = 0;
  def_n[2] = 1;
  geometry::color_t def_c;
  def_c[0] = 1;
  def_c[1] = 1;
  def_c[2] = 1;
  geometry::uv_t def_uv;
  def_uv[0] = 0;
  def_uv[1] = 0;
  geometry::tangent_t def_t;
  def_t[0] = 1;
  def_t[1] = 0;
  def_t[2] = 0;
  def_t[3] = 1;

  geometry out;
  for (const instance &in : drawn) {
    // A COW copy (placed() shares it untransformed); the padding below detaches only what it
    // touches.
    geometry g = placed(meshes[in.mesh].geom, in.transform);
    const std::size_t nv = static_cast<std::size_t>(g.num_points());
    if (any_normals && g.const_normals().empty())
      g.normals().assign(nv, def_n);
    if (any_colors && g.const_colors().empty())
      g.colors().assign(nv, def_c);
    if (any_uvs && g.const_uvs().empty())
      g.uvs().assign(nv, def_uv);
    if (any_tangents && g.const_tangents().empty())
      g.tangents().assign(nv, def_t);
    out.merge(g);
  }
  return out;
}

// ---------------
// model::extents
// ---------------
// Purpose:
//   Union of every instance's bounding box, placed. generic_bounding_box::operator+
//   treats a null (zero-volume) box as the identity, so an empty model yields a
//   null box.
bounding_box model::extents() const {
  bounding_box bbox;
  for (const instance &in : instances())
    bbox = bbox + placed(meshes[in.mesh].geom, in.transform).extents();
  return bbox;
}

// ---------------
// model::extents_metres
// ---------------
// Purpose:
//   extents() scaled into canonical SI metres via metres_per_source_unit. A null
//   box (empty model) has no corners to scale and is returned as-is, so it stays
//   the additive identity for callers that union several models' footprints.
bounding_box model::extents_metres() const {
  bounding_box b = extents();
  if (b.isNull())
    return b;
  const double s = metres_per_source_unit;
  return bounding_box(b.minx * s, b.miny * s, b.minz * s, b.maxx * s, b.maxy * s, b.maxz * s);
}

} // namespace cvc
