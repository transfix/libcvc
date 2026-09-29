/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// simplify.cpp -- native Quadric Error Metrics mesh decimation.
//
// Half-edge collapse variant: each collapse snaps the higher-error endpoint onto
// the lower-error one, so surviving vertices never move and keep their exact
// position + uv + color. That is robust (no optimal-placement 3x3 solve or its
// singular fallbacks), attribute-safe across uv seams, and a good fit for the
// feature-aligned architectural meshes this first targets. A per-vertex 4x4
// quadric accumulates the squared distance to the planes of every incident face
// (Garland & Heckbert '97); boundary edges add a perpendicular constraint plane
// so open borders stay put. The collapse cost is that quadric evaluated at the
// surviving endpoint; a lazy binary heap keyed on a per-vertex version pops the
// cheapest valid collapse. Fold-overs are rejected by a per-face normal-flip
// test so the coarse rung never self-intersects.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/core/thread_pool.h>
#include <cvc/geometry/simplify.h>
#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

namespace cvc {
namespace {

typedef std::uint32_t u32;
typedef std::uint64_t u64;

// Symmetric 4x4 error quadric stored as its 10 upper-triangular entries:
//   [ a2 ab ac ad ]
//   [ ab b2 bc bd ]
//   [ ac bc c2 cd ]
//   [ ad bd cd d2 ]
// for the homogeneous plane p = (a,b,c,d) with a^2+b^2+c^2 = 1 and d = -n.v0.
struct Quadric {
  double m[10];
  Quadric() { std::fill(m, m + 10, 0.0); }

  static Quadric from_plane(double a, double b, double c, double d, double w) {
    Quadric q;
    q.m[0] = w * a * a;
    q.m[1] = w * a * b;
    q.m[2] = w * a * c;
    q.m[3] = w * a * d;
    q.m[4] = w * b * b;
    q.m[5] = w * b * c;
    q.m[6] = w * b * d;
    q.m[7] = w * c * c;
    q.m[8] = w * c * d;
    q.m[9] = w * d * d;
    return q;
  }
  void operator+=(const Quadric &o) {
    for (int i = 0; i < 10; ++i)
      m[i] += o.m[i];
  }
  // v^T Q v for v = (x,y,z,1); the squared, plane-summed distance at that point.
  double error(double x, double y, double z) const {
    return m[0] * x * x + 2 * m[1] * x * y + 2 * m[2] * x * z + 2 * m[3] * x + m[4] * y * y +
           2 * m[5] * y * z + 2 * m[6] * y + m[7] * z * z + 2 * m[8] * z + m[9];
  }
};

struct Vec3 {
  double x, y, z;
};
inline Vec3 sub(const Vec3 &a, const Vec3 &b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double dot(const Vec3 &a, const Vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double len(const Vec3 &a) { return std::sqrt(dot(a, a)); }

// Unit face normal (0 for a degenerate tri).
inline Vec3 face_normal(const Vec3 &a, const Vec3 &b, const Vec3 &c) {
  Vec3 n = cross(sub(b, a), sub(c, a));
  double l = len(n);
  if (l < 1e-30)
    return {0, 0, 0};
  return {n.x / l, n.y / l, n.z / l};
}

// One pending collapse on the heap. Popped lazily: an entry is stale when either
// endpoint's version has advanced past the value stamped when it was pushed.
struct HeapItem {
  double cost;
  u32 keep, drop; // collapse `drop` onto `keep`
  u32 keep_ver, drop_ver;
  bool operator<(const HeapItem &o) const { return cost > o.cost; } // min-heap via std::greater
};

inline u64 edge_key(u32 a, u32 b) {
  if (a > b)
    std::swap(a, b);
  return (u64(a) << 32) | u64(b);
}

// Run fn(i) for i in [0,n) on the pool when one is attached, else serially on the
// caller. fn must be data-parallel (write only to its own index i).
inline void pfor(thread_pool *pool, u32 n, const std::function<void(int)> &fn) {
  if (pool && n > 1)
    pool->parallel_for(int(n), fn);
  else
    for (u32 i = 0; i < n; ++i)
      fn(int(i));
}

} // namespace

geometry simplify(const geometry &mesh, const simplify_params &params, simplify_result *out,
                  thread_pool *pool) {
  const geometry::points_t &in_pts = mesh.const_points();
  const geometry::tris_t &in_tris = mesh.const_tris();
  const geometry::quads_t &in_quads = mesh.const_quads();

  // --- gather triangles (triangulating quads) + positions ---
  std::vector<Vec3> P(in_pts.size());
  for (std::size_t i = 0; i < in_pts.size(); ++i)
    P[i] = {in_pts[i][0], in_pts[i][1], in_pts[i][2]};

  std::vector<std::array<u32, 3>> T;
  T.reserve(in_tris.size() + 2 * in_quads.size());
  for (const auto &t : in_tris)
    T.push_back({u32(t[0]), u32(t[1]), u32(t[2])});
  for (const auto &q : in_quads) { // fan-triangulate each quad
    T.push_back({u32(q[0]), u32(q[1]), u32(q[2])});
    T.push_back({u32(q[0]), u32(q[2]), u32(q[3])});
  }

  const u64 n_in = T.size();
  u64 target = params.target_tris;
  if (target == 0) {
    double r = params.target_ratio;
    if (r <= 0.0)
      r = 0.0;
    if (r > 1.0)
      r = 1.0;
    target = u64(std::llround(double(n_in) * r));
  }
  if (out) {
    out->in_tris = n_in;
    out->out_tris = n_in;
    out->collapses = 0;
    out->world_error = 0.0;
    out->hit_error_limit = false;
  }
  // Nothing to do: no faces, or already at/under target.
  if (n_in == 0 || target >= n_in)
    return mesh;

  const u32 nv = u32(P.size());
  std::vector<bool> vremoved(nv, false);
  std::vector<u32> vver(nv, 0);
  std::vector<bool> tremoved(T.size(), false);

  // vertex -> incident triangle indices
  std::vector<std::vector<u32>> vtri(nv);
  for (u32 ti = 0; ti < T.size(); ++ti)
    for (int k = 0; k < 3; ++k)
      vtri[T[ti][k]].push_back(ti);

  // --- per-vertex quadrics from incident face planes ---
  // The natural form is a scatter (each face adds to its 3 vertices), which races
  // under a pool. Split it into two data-parallel gathers via the vtri adjacency:
  // (1) each face computes its own area-weighted plane quadric; (2) each vertex
  // sums the quadrics of its incident faces. Result is bit-identical to serial.
  std::vector<Quadric> FQ(T.size());
  pfor(pool, u32(T.size()), [&](int ti) {
    const Vec3 &a = P[T[ti][0]], &b = P[T[ti][1]], &c = P[T[ti][2]];
    Vec3 n = cross(sub(b, a), sub(c, a));
    double l = len(n);
    if (l < 1e-30)
      return;
    n = {n.x / l, n.y / l, n.z / l};
    double d = -dot(n, a);
    FQ[ti] = Quadric::from_plane(n.x, n.y, n.z, d, 0.5 * l); // area-weighted
  });
  std::vector<Quadric> Q(nv);
  pfor(pool, nv, [&](int v) {
    Quadric acc;
    for (u32 ti : vtri[v])
      acc += FQ[ti];
    Q[v] = acc;
  });

  // --- boundary constraint quadrics: for each edge on exactly one triangle, add
  // a plane through the edge, perpendicular to the face, so the border holds. ---
  if (params.preserve_boundary) {
    std::unordered_map<u64, int> edge_count;
    edge_count.reserve(T.size() * 3);
    for (u32 ti = 0; ti < T.size(); ++ti)
      for (int k = 0; k < 3; ++k)
        edge_count[edge_key(T[ti][k], T[ti][(k + 1) % 3])]++;
    for (u32 ti = 0; ti < T.size(); ++ti) {
      const Vec3 &a = P[T[ti][0]], &b = P[T[ti][1]], &c = P[T[ti][2]];
      Vec3 fn = face_normal(a, b, c);
      for (int k = 0; k < 3; ++k) {
        u32 v0 = T[ti][k], v1 = T[ti][(k + 1) % 3];
        if (edge_count[edge_key(v0, v1)] != 1)
          continue; // interior edge
        // plane containing edge (v0,v1) and perpendicular to the face
        Vec3 e = sub(P[v1], P[v0]);
        Vec3 pn = cross(e, fn);
        double pl = len(pn);
        if (pl < 1e-30)
          continue;
        pn = {pn.x / pl, pn.y / pl, pn.z / pl};
        double d = -dot(pn, P[v0]);
        double w = params.boundary_weight;
        Quadric bq = Quadric::from_plane(pn.x, pn.y, pn.z, d, w);
        Q[v0] += bq;
        Q[v1] += bq;
      }
    }
  }

  const double flip_cos = std::cos(params.max_normal_flip_deg * 3.14159265358979323846 / 180.0);

  // Evaluate the half-edge collapse of edge (u,v): returns the cheaper direction.
  auto eval = [&](u32 u, u32 v, double &cost, u32 &keep, u32 &drop) {
    Quadric q = Q[u];
    q += Q[v];
    double eu = q.error(P[u].x, P[u].y, P[u].z);
    double ev = q.error(P[v].x, P[v].y, P[v].z);
    if (eu <= ev) {
      keep = u;
      drop = v;
      cost = eu;
    } else {
      keep = v;
      drop = u;
      cost = ev;
    }
  };

  std::priority_queue<HeapItem> heap;
  auto push_edge = [&](u32 u, u32 v) {
    if (u == v || vremoved[u] || vremoved[v])
      return;
    double cost;
    u32 keep, drop;
    eval(u, v, cost, keep, drop);
    heap.push(HeapItem{cost, keep, drop, vver[keep], vver[drop]});
  };

  // seed the heap with every unique edge
  {
    std::unordered_map<u64, char> seen;
    seen.reserve(T.size() * 3);
    for (u32 ti = 0; ti < T.size(); ++ti)
      for (int k = 0; k < 3; ++k) {
        u32 a = T[ti][k], b = T[ti][(k + 1) % 3];
        u64 key = edge_key(a, b);
        if (seen.emplace(key, 1).second)
          push_edge(a, b);
      }
  }

  // Would collapsing `drop` onto `keep` flip any surviving incident face?
  auto causes_flip = [&](u32 keep, u32 drop) -> bool {
    for (u32 ti : vtri[drop]) {
      if (tremoved[ti])
        continue;
      const std::array<u32, 3> &f = T[ti];
      if (f[0] == keep || f[1] == keep || f[2] == keep)
        continue; // this face degenerates away, not a flip
      Vec3 p[3];
      for (int k = 0; k < 3; ++k)
        p[k] = (f[k] == drop) ? P[keep] : P[f[k]];
      Vec3 before = face_normal(P[f[0]], P[f[1]], P[f[2]]);
      Vec3 after = face_normal(p[0], p[1], p[2]);
      if (len(before) == 0.0 || len(after) == 0.0)
        return true; // collapse makes a sliver
      if (dot(before, after) < flip_cos)
        return true;
    }
    return false;
  };

  u64 ntris = n_in;
  u64 ncollapse = 0;
  double worst = 0.0;

  while (ntris > target && !heap.empty()) {
    HeapItem it = heap.top();
    heap.pop();
    // stale? (either endpoint changed or was removed since this was pushed)
    if (vremoved[it.keep] || vremoved[it.drop] || vver[it.keep] != it.keep_ver ||
        vver[it.drop] != it.drop_ver)
      continue;
    if (params.max_error >= 0.0 && std::sqrt(std::max(0.0, it.cost)) > params.max_error) {
      if (out)
        out->hit_error_limit = true;
      break;
    }
    if (causes_flip(it.keep, it.drop))
      continue; // leave this edge out; a re-pushed copy may succeed later

    const u32 keep = it.keep, drop = it.drop;

    // rewrite drop's faces onto keep; drop degenerate ones
    for (u32 ti : vtri[drop]) {
      if (tremoved[ti])
        continue;
      std::array<u32, 3> &f = T[ti];
      if (f[0] == keep || f[1] == keep || f[2] == keep) {
        tremoved[ti] = true; // shared face collapses to a line
        --ntris;
      } else {
        for (int k = 0; k < 3; ++k)
          if (f[k] == drop)
            f[k] = keep;
        vtri[keep].push_back(ti);
      }
    }

    Q[keep] += Q[drop];
    vremoved[drop] = true;
    ++vver[keep];
    worst = std::max(worst, it.cost);
    ++ncollapse;

    // compact keep's incident list (drop removed faces) and re-cost its edges
    std::vector<u32> &kt = vtri[keep];
    kt.erase(std::remove_if(kt.begin(), kt.end(), [&](u32 ti) { return tremoved[ti]; }), kt.end());
    for (u32 ti : kt)
      for (int k = 0; k < 3; ++k) {
        u32 w = T[ti][k];
        if (w != keep && !vremoved[w])
          push_edge(keep, w);
      }
  }

  if (out) {
    out->out_tris = ntris;
    out->collapses = ncollapse;
    out->world_error = std::sqrt(std::max(0.0, worst));
  }

  // --- rebuild a compact geometry from the survivors ---
  geometry result(mesh.ctx());
  std::vector<u32> remap(nv, u32(-1));
  const geometry::uvs_t &in_uv = mesh.const_uvs();
  const geometry::colors_t &in_col = mesh.const_colors();
  const bool has_uv = in_uv.size() == in_pts.size();
  const bool has_col = in_col.size() == in_pts.size();

  geometry::points_t &op = result.points();
  geometry::uvs_t *ouv = has_uv ? &result.uvs() : nullptr;
  geometry::colors_t *ocol = has_col ? &result.colors() : nullptr;
  for (u32 v = 0; v < nv; ++v) {
    if (vremoved[v])
      continue;
    remap[v] = u32(op.size());
    op.push_back(in_pts[v]);
    if (ouv)
      ouv->push_back(in_uv[v]);
    if (ocol)
      ocol->push_back(in_col[v]);
  }
  geometry::tris_t &ot = result.tris();
  for (u32 ti = 0; ti < T.size(); ++ti) {
    if (tremoved[ti])
      continue;
    const std::array<u32, 3> &f = T[ti];
    // guard against any residual degeneracy
    if (f[0] == f[1] || f[1] == f[2] || f[0] == f[2])
      continue;
    ot.push_back({remap[f[0]], remap[f[1]], remap[f[2]]});
  }
  result.set_geometry_type(geometry::SURFACE_TRI);
  result.compute_normals();
  return result;
}

} // namespace cvc
