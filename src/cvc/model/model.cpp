/*
  Copyright 2024 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License version 2.1 as published by the
  Free Software Foundation.
*/

#include <algorithm>
#include <cctype>
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
namespace {
// The instances of `m`, each node's transform taken from `locals` (one per node), the walk
// model::instances describes.
std::vector<model::instance> walk_instances(const model &m,
                                            const std::vector<model::transform_t> &locals) {
  typedef model::transform_t transform_t;
  typedef model::instance instance;
  const std::vector<model::node> &nodes = m.nodes;
  const std::vector<model::mesh> &meshes = m.meshes;
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
    const transform_t t = model::compose(parent, locals[n]);
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
  enter(0, model::identity_transform());
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
} // namespace

std::vector<model::instance> model::instances() const {
  std::vector<transform_t> locals;
  locals.reserve(nodes.size());
  for (const node &n : nodes)
    locals.push_back(n.transform);
  return walk_instances(*this, locals);
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

// ---------------------------------------------------------------------------
// Animation: clips, skins and morph targets
// ---------------------------------------------------------------------------

namespace {
typedef boost::array<double, 3> vec3;
typedef boost::array<double, 4> quat; // w, x, y, z

// The keyframe pair either side of `t` in `times` (ascending): i and the blend toward i + 1, or
// i alone (u 0) at or past the ends.
std::pair<std::size_t, double> key_at(const std::vector<double> &times, double t, bool step) {
  const std::size_t n = times.size();
  if (n < 2 || !(t > times.front()))
    return {0, 0.0};
  if (t >= times.back())
    return {n - 1, 0.0};
  const std::size_t i =
      static_cast<std::size_t>(std::upper_bound(times.begin(), times.end(), t) - times.begin()) - 1;
  const double h = times[i + 1] - times[i];
  return {i, step || !(h > 0.0) ? 0.0 : (t - times[i]) / h};
}

vec3 sample(const model::clip::vec3_keys &k, double t) {
  const std::size_t n = std::min(k.t.size(), k.v.size());
  if (n == 0)
    return vec3{{0.0, 0.0, 0.0}};
  const auto [i, u] = key_at(k.t, t, k.step);
  if (u == 0.0 || i + 1 >= n)
    return k.v[std::min(i, n - 1)];
  vec3 r;
  for (int c = 0; c < 3; ++c)
    r[c] = k.v[i][c] + (k.v[i + 1][c] - k.v[i][c]) * u;
  return r;
}

quat normalized(quat q) {
  const double len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (len > 0.0)
    for (double &c : q)
      c /= len;
  else
    q = quat{{1.0, 0.0, 0.0, 0.0}};
  return q;
}

// Spherical, along the shorter arc.
quat slerp(const quat &a, quat b, double u) {
  double d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (d < 0.0) {
    d = -d;
    for (double &c : b)
      c = -c;
  }
  quat r;
  if (d > 0.9995) { // nearly the same: a straight line, made unit
    for (int c = 0; c < 4; ++c)
      r[c] = a[c] + (b[c] - a[c]) * u;
    return normalized(r);
  }
  const double th = std::acos(std::min(1.0, d)), s = std::sin(th);
  const double wa = std::sin((1.0 - u) * th) / s, wb = std::sin(u * th) / s;
  for (int c = 0; c < 4; ++c)
    r[c] = wa * a[c] + wb * b[c];
  return normalized(r);
}

quat sample(const model::clip::quat_keys &k, double t) {
  const std::size_t n = std::min(k.t.size(), k.q.size());
  if (n == 0)
    return quat{{1.0, 0.0, 0.0, 0.0}};
  const auto [i, u] = key_at(k.t, t, k.step);
  if (u == 0.0 || i + 1 >= n)
    return normalized(k.q[std::min(i, n - 1)]);
  return slerp(normalized(k.q[i]), normalized(k.q[i + 1]), u);
}

std::vector<double> sample(const model::clip::weights_channel &k, double t) {
  const std::size_t n = std::min(k.t.size(), k.weights.size());
  if (n == 0)
    return {};
  const auto [i, u] = key_at(k.t, t, k.step);
  if (u == 0.0 || i + 1 >= n)
    return k.weights[std::min(i, n - 1)];
  const std::vector<double> &a = k.weights[i], &b = k.weights[i + 1];
  std::vector<double> r(std::max(a.size(), b.size()), 0.0);
  for (std::size_t c = 0; c < r.size(); ++c) {
    const double x = c < a.size() ? a[c] : 0.0, y = c < b.size() ? b[c] : 0.0;
    r[c] = x + (y - x) * u;
  }
  return r;
}

// A transform as translation, rotation and scale (its columns' lengths; a mirror's sign on x).
void decompose(const model::transform_t &m, vec3 &t, quat &q, vec3 &s) {
  t = vec3{{m[3], m[7], m[11]}};
  double c[3][3];
  for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 3; ++i)
      c[j][i] = m[i * 4 + j];
    s[j] = std::sqrt(c[j][0] * c[j][0] + c[j][1] * c[j][1] + c[j][2] * c[j][2]);
  }
  const double det = c[0][0] * (c[1][1] * c[2][2] - c[1][2] * c[2][1]) -
                     c[1][0] * (c[0][1] * c[2][2] - c[0][2] * c[2][1]) +
                     c[2][0] * (c[0][1] * c[1][2] - c[0][2] * c[1][1]);
  if (det < 0.0)
    s[0] = -s[0];
  double r[3][3]; // row i, column j
  for (int j = 0; j < 3; ++j)
    for (int i = 0; i < 3; ++i)
      r[i][j] = s[j] != 0.0 ? c[j][i] / s[j] : (i == j ? 1.0 : 0.0);
  const double tr = r[0][0] + r[1][1] + r[2][2];
  if (tr > 0.0) {
    const double k = 0.5 / std::sqrt(tr + 1.0);
    q = quat{{0.25 / k, (r[2][1] - r[1][2]) * k, (r[0][2] - r[2][0]) * k, (r[1][0] - r[0][1]) * k}};
  } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
    const double k = 2.0 * std::sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]);
    q = quat{{(r[2][1] - r[1][2]) / k, 0.25 * k, (r[0][1] + r[1][0]) / k, (r[0][2] + r[2][0]) / k}};
  } else if (r[1][1] > r[2][2]) {
    const double k = 2.0 * std::sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]);
    q = quat{{(r[0][2] - r[2][0]) / k, (r[0][1] + r[1][0]) / k, 0.25 * k, (r[1][2] + r[2][1]) / k}};
  } else {
    const double k = 2.0 * std::sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]);
    q = quat{{(r[1][0] - r[0][1]) / k, (r[0][2] + r[2][0]) / k, (r[1][2] + r[2][1]) / k, 0.25 * k}};
  }
  q = normalized(q);
}

// T R S as one transform.
model::transform_t compose_trs(const vec3 &t, const quat &q0, const vec3 &s) {
  const quat q = normalized(q0);
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double r[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
                          {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
                          {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}};
  model::transform_t m = model::identity_transform();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j)
      m[i * 4 + j] = r[i][j] * s[j];
    m[i * 4 + 3] = t[i];
  }
  return m;
}

// Each node's transform into the model's space, by the first path from nodes[0] that reaches it
// (a node no path reaches keeps its own).
std::vector<model::transform_t> node_globals(const model &m,
                                             const std::vector<model::transform_t> &locals) {
  const int n_nodes = static_cast<int>(m.nodes.size());
  std::vector<model::transform_t> out(locals);
  if (n_nodes == 0)
    return out;
  std::vector<char> seen(m.nodes.size(), 0);
  std::vector<int> stack{0};
  seen[0] = 1;
  out[0] = locals[0];
  while (!stack.empty()) {
    const int n = stack.back();
    stack.pop_back();
    for (int c : m.nodes[n].children)
      if (c >= 0 && c < n_nodes && !seen[c]) {
        seen[c] = 1;
        out[c] = model::compose(out[n], locals[c]);
        stack.push_back(c);
      }
  }
  return out;
}

// p moved by the affine m, and a direction by its linear part.
void apply_point(const model::transform_t &m, const double in[3], double out[3]) {
  for (int i = 0; i < 3; ++i)
    out[i] = m[i * 4] * in[0] + m[i * 4 + 1] * in[1] + m[i * 4 + 2] * in[2] + m[i * 4 + 3];
}
void apply_direction(const model::transform_t &m, const double in[3], double out[3]) {
  for (int i = 0; i < 3; ++i)
    out[i] = m[i * 4] * in[0] + m[i * 4 + 1] * in[1] + m[i * 4 + 2] * in[2];
}

// A normal under m: by its cofactor matrix (the inverse transpose, up to scale), the sign kept
// through a mirror, made unit (a degenerate m keeps the normal).
void apply_normal(const model::transform_t &t, double n[3]) {
  const double a00 = t[0], a01 = t[1], a02 = t[2], a10 = t[4], a11 = t[5], a12 = t[6], a20 = t[8],
               a21 = t[9], a22 = t[10];
  const double c00 = a11 * a22 - a12 * a21, c01 = a12 * a20 - a10 * a22,
               c02 = a10 * a21 - a11 * a20;
  const double c10 = a02 * a21 - a01 * a22, c11 = a00 * a22 - a02 * a20,
               c12 = a01 * a20 - a00 * a21;
  const double c20 = a01 * a12 - a02 * a11, c21 = a02 * a10 - a00 * a12,
               c22 = a00 * a11 - a01 * a10;
  const double s = a00 * c00 + a01 * c01 + a02 * c02 < 0.0 ? -1.0 : 1.0;
  const double x = s * (c00 * n[0] + c01 * n[1] + c02 * n[2]);
  const double y = s * (c10 * n[0] + c11 * n[1] + c12 * n[2]);
  const double z = s * (c20 * n[0] + c21 * n[1] + c22 * n[2]);
  const double len = std::sqrt(x * x + y * y + z * z);
  if (len > 0.0) {
    n[0] = x / len;
    n[1] = y / len;
    n[2] = z / len;
  }
}
} // namespace

int model::find_clip(const std::string &name) const {
  for (std::size_t i = 0; i < clips.size(); ++i)
    if (clips[i].name == name)
      return static_cast<int>(i);
  if (!name.empty() &&
      std::all_of(name.begin(), name.end(), [](unsigned char c) { return std::isdigit(c) != 0; })) {
    const unsigned long i = std::stoul(name);
    if (i < clips.size())
      return static_cast<int>(i);
  }
  return -1;
}

bool model::animated() const {
  if (!clips.empty())
    return true;
  for (const mesh &m : meshes)
    if (!m.bones.empty() || !m.morphs.empty())
      return true;
  return false;
}

std::vector<model::transform_t> model::local_transforms(int c, double t) const {
  std::vector<transform_t> out;
  out.reserve(nodes.size());
  for (const node &n : nodes)
    out.push_back(n.transform);
  if (c < 0 || c >= static_cast<int>(clips.size()))
    return out;
  for (const clip::node_channel &ch : clips[c].nodes) {
    if (ch.node < 0 || ch.node >= static_cast<int>(nodes.size()))
      continue;
    vec3 tr, sc;
    quat rot;
    decompose(nodes[ch.node].transform, tr, rot, sc);
    if (!ch.position.t.empty())
      tr = sample(ch.position, t);
    if (!ch.rotation.t.empty())
      rot = sample(ch.rotation, t);
    if (!ch.scale.t.empty())
      sc = sample(ch.scale, t);
    out[ch.node] = compose_trs(tr, rot, sc);
  }
  return out;
}

void model::posed(int c, double t, geometry::points_t &points, geometry::normals_t &normals) const {
  const std::vector<transform_t> locals = local_transforms(c, t);
  const std::vector<instance> drawn = walk_instances(*this, locals);
  const std::vector<transform_t> globals = node_globals(*this, locals);
  // Each mesh's morph weights: its rest weights, or what the clip sets for the node drawing it.
  std::vector<std::vector<double>> weights(meshes.size());
  for (std::size_t i = 0; i < meshes.size(); ++i)
    weights[i] = meshes[i].morph_weights;
  if (c >= 0 && c < static_cast<int>(clips.size()))
    for (const clip::weights_channel &wc : clips[c].weights)
      if (wc.node >= 0 && wc.node < static_cast<int>(nodes.size())) {
        const std::vector<double> w = sample(wc, t);
        for (int mi : nodes[wc.node].meshes)
          if (mi >= 0 && mi < static_cast<int>(meshes.size()))
            weights[mi] = w;
      }
  bool any_normals = false;
  for (const instance &in : drawn)
    any_normals = any_normals || !meshes[in.mesh].geom.const_normals().empty();
  points.clear();
  normals.clear();
  std::vector<transform_t> skin_mats;
  for (const instance &in : drawn) {
    const mesh &me = meshes[in.mesh];
    const geometry::points_t &P = me.geom.const_points();
    const geometry::normals_t &N = me.geom.const_normals();
    const std::size_t nv = P.size();
    const bool has_n = N.size() == nv && nv > 0;
    geometry::points_t p(P);
    geometry::normals_t n = has_n ? N : geometry::normals_t();
    for (std::size_t k = 0; k < me.morphs.size(); ++k) { // morph first, in the mesh's own space
      const double w = k < weights[in.mesh].size() ? weights[in.mesh][k] : 0.0;
      const morph_target &mt = me.morphs[k];
      if (w == 0.0)
        continue;
      for (std::size_t v = 0; v < nv && v < mt.position_deltas.size(); ++v)
        for (int a = 0; a < 3; ++a)
          p[v][a] += w * mt.position_deltas[v][a];
      if (has_n)
        for (std::size_t v = 0; v < nv && v < mt.normal_deltas.size(); ++v)
          for (int a = 0; a < 3; ++a)
            n[v][a] += w * mt.normal_deltas[v][a];
    }
    if (!me.bones.empty() && me.bones.first.size() == nv + 1) { // placed by its bones
      skin_mats.assign(me.bones.bones.size(), identity_transform());
      for (std::size_t b = 0; b < me.bones.bones.size(); ++b) {
        const bone &bn = me.bones.bones[b];
        if (bn.node >= 0 && bn.node < static_cast<int>(globals.size()))
          skin_mats[b] = compose(globals[bn.node], bn.offset);
      }
      for (std::size_t v = 0; v < nv; ++v) {
        const boost::uint32_t f0 = me.bones.first[v], f1 = me.bones.first[v + 1];
        if (f0 >= f1)
          continue; // no influences: where it is
        const double src[3] = {p[v][0], p[v][1], p[v][2]};
        double acc[3] = {0, 0, 0}, nacc[3] = {0, 0, 0};
        const double nsrc[3] = {has_n ? n[v][0] : 0.0, has_n ? n[v][1] : 0.0,
                                has_n ? n[v][2] : 0.0};
        for (boost::uint32_t k = f0; k < f1 && k < me.bones.influences.size(); ++k) {
          const auto [b, w] = me.bones.influences[k];
          if (b < 0 || b >= static_cast<int>(skin_mats.size()))
            continue;
          double q[3];
          apply_point(skin_mats[b], src, q);
          for (int a = 0; a < 3; ++a)
            acc[a] += w * q[a];
          if (has_n) {
            apply_direction(skin_mats[b], nsrc, q);
            for (int a = 0; a < 3; ++a)
              nacc[a] += w * q[a];
          }
        }
        p[v] = {{acc[0], acc[1], acc[2]}};
        if (has_n) {
          const double len = std::sqrt(nacc[0] * nacc[0] + nacc[1] * nacc[1] + nacc[2] * nacc[2]);
          if (len > 0.0)
            n[v] = {{nacc[0] / len, nacc[1] / len, nacc[2] / len}};
        }
      }
    } else if (!is_identity(in.transform)) { // placed by its node
      for (std::size_t v = 0; v < nv; ++v) {
        const double src[3] = {p[v][0], p[v][1], p[v][2]};
        double q[3];
        apply_point(in.transform, src, q);
        p[v] = {{q[0], q[1], q[2]}};
        if (has_n) {
          double nn[3] = {n[v][0], n[v][1], n[v][2]};
          apply_normal(in.transform, nn);
          n[v] = {{nn[0], nn[1], nn[2]}};
        }
      }
    }
    points.insert(points.end(), p.begin(), p.end());
    if (any_normals) {
      if (has_n)
        normals.insert(normals.end(), n.begin(), n.end());
      else
        normals.insert(normals.end(), nv, geometry::normal_t{{0.0, 0.0, 1.0}});
    }
  }
}

geometry model::posed(int c, double t) const {
  geometry g = merged();
  geometry::points_t pts;
  geometry::normals_t nrm;
  posed(c, t, pts, nrm);
  if (pts.size() == g.const_points().size())
    g.points() = pts;
  if (!nrm.empty() && nrm.size() == g.const_points().size())
    g.normals() = nrm;
  return g;
}

} // namespace cvc
