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
// position + uv + color + normal. That is robust (no optimal-placement 3x3 solve
// or its singular fallbacks), attribute-safe across uv seams, and a good fit for
// the feature-aligned architectural meshes this first targets. A per-vertex 4x4
// quadric accumulates the area-weighted squared distance to the planes of every
// incident face (Garland & Heckbert '97); boundary edges add a perpendicular
// constraint plane weighted by |edge|^2 so open borders stay put and every term
// shares the same units (length^2 weight x length^2 distance), which keeps the
// collapse order invariant under a uniform scale. The collapse cost is that
// quadric evaluated at the surviving endpoint (in a frame centred on the mesh,
// with round-off snapped to 0 and the rest compared at 24-bit precision, so
// geometric ties stay ties when the mesh is scaled or moved); a lazy binary heap
// keyed on a per-vertex version pops the cheapest valid collapse, with ties
// broken by vertex index so the pop sequence never depends on the standard
// library's heap.
// Fold-overs are rejected by a per-face normal-flip test so the coarse rung
// never self-intersects, and vertices that coincide with a vertex of another
// connected component are never dropped, so touching unwelded parts stay closed.
//
// The quadric cost is area x distance^2 -- not a length -- so the reported error
// is measured instead of derived: a sampled symmetric Hausdorff distance between
// the input and each result, answered by exact point-to-triangle distances over
// a uniform grid.
//
// One engine serves both entry points: it runs a single collapse pass toward the
// smallest requested target and snapshots the mesh as each target is reached.
// simplify() is that engine with one target, which is what makes a progressive
// snapshot bit-identical to an independent simplify() to the same target.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cvc/core/thread_pool.h>
#include <cvc/geometry/simplify.h>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>
#include <vector>

namespace cvc {
namespace {

typedef std::uint32_t u32;
typedef std::uint64_t u64;
typedef std::int64_t i64;
typedef std::array<u32, 3> Tri;

const double kInf = std::numeric_limits<double>::infinity();

// Symmetric 4x4 error quadric stored as its 10 upper-triangular entries:
//   [ a2 ab ac ad ]
//   [ ab b2 bc bd ]
//   [ ac bc c2 cd ]
//   [ ad bd cd d2 ]
// for the homogeneous plane p = (a,b,c,d) with a^2+b^2+c^2 = 1 and d = -n.v0,
// plus the total weight `w` of the planes summed into it (so cost / w is the
// weighted mean squared plane distance -- the max_error proxy).
struct Quadric {
  double m[10];
  double w;
  Quadric() : w(0.0) { std::fill(m, m + 10, 0.0); }

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
    q.w = w;
    return q;
  }
  void operator+=(const Quadric &o) {
    for (int i = 0; i < 10; ++i)
      m[i] += o.m[i];
    w += o.w;
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
inline double axis(const Vec3 &a, int k) { return k == 0 ? a.x : (k == 1 ? a.y : a.z); }
// a + t * d
inline Vec3 along(const Vec3 &a, double t, const Vec3 &d) {
  return {a.x + t * d.x, a.y + t * d.y, a.z + t * d.z};
}
inline double dist2(const Vec3 &a, const Vec3 &b) {
  const Vec3 d = sub(a, b);
  return dot(d, d);
}

// Unit face normal (0 for a degenerate tri).
inline Vec3 face_normal(const Vec3 &a, const Vec3 &b, const Vec3 &c) {
  Vec3 n = cross(sub(b, a), sub(c, a));
  double l = len(n);
  if (l < 1e-30)
    return {0, 0, 0};
  return {n.x / l, n.y / l, n.z / l};
}

// Squared distance from p to the segment [a,b].
inline double seg_dist2(const Vec3 &p, const Vec3 &a, const Vec3 &b) {
  const Vec3 ab = sub(b, a);
  const double l2 = dot(ab, ab);
  double t = l2 > 0.0 ? dot(sub(p, a), ab) / l2 : 0.0;
  t = std::min(1.0, std::max(0.0, t));
  return dist2(p, along(a, t, ab));
}

// Squared distance from p to the triangle abc: classify p against the
// triangle's vertex/edge/face Voronoi regions (Ericson, Real-Time Collision
// Detection, 5.1.5). A zero-area triangle falls back to its three edges.
double tri_dist2(const Vec3 &p, const Vec3 &a, const Vec3 &b, const Vec3 &c) {
  const Vec3 ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
  const Vec3 n = cross(ab, ac);
  if (!(dot(n, n) > 0.0))
    return std::min(seg_dist2(p, a, b), std::min(seg_dist2(p, a, c), seg_dist2(p, b, c)));
  const double d1 = dot(ab, ap), d2 = dot(ac, ap);
  if (d1 <= 0.0 && d2 <= 0.0)
    return dot(ap, ap); // vertex a
  const Vec3 bp = sub(p, b);
  const double d3 = dot(ab, bp), d4 = dot(ac, bp);
  if (d3 >= 0.0 && d4 <= d3)
    return dot(bp, bp); // vertex b
  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) { // edge ab
    const double den = d1 - d3;
    return dist2(p, along(a, den > 0.0 ? d1 / den : 0.0, ab));
  }
  const Vec3 cp = sub(p, c);
  const double d5 = dot(ab, cp), d6 = dot(ac, cp);
  if (d6 >= 0.0 && d5 <= d6)
    return dot(cp, cp); // vertex c
  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) { // edge ac
    const double den = d2 - d6;
    return dist2(p, along(a, den > 0.0 ? d2 / den : 0.0, ac));
  }
  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) { // edge bc
    const double den = (d4 - d3) + (d5 - d6);
    return dist2(p, along(b, den > 0.0 ? (d4 - d3) / den : 0.0, sub(c, b)));
  }
  const double den = va + vb + vc; // face interior
  if (!(den > 0.0))
    return std::min(seg_dist2(p, a, b), std::min(seg_dist2(p, a, c), seg_dist2(p, b, c)));
  const double v = vb / den, w = vc / den;
  return dist2(p, along(along(a, v, ab), w, ac));
}

// One pending collapse on the heap. Popped lazily: an entry is stale when either
// endpoint's version has advanced past the value stamped when it was pushed.
struct HeapItem {
  double cost;
  u32 keep, drop; // collapse `drop` onto `keep`
  u32 keep_ver, drop_ver;
  // std::priority_queue pops the LARGEST item, so "less" means "pops later":
  // higher cost first, then higher indices. The order is total (equal items are
  // identical), so the pop sequence is fixed by the costs alone and does not
  // depend on how a given standard library arranges its heap.
  bool operator<(const HeapItem &o) const {
    if (cost != o.cost)
      return cost > o.cost;
    if (keep != o.keep)
      return keep > o.keep;
    if (drop != o.drop)
      return drop > o.drop;
    if (keep_ver != o.keep_ver)
      return keep_ver > o.keep_ver;
    return drop_ver > o.drop_ver;
  }
};

// Round a positive collapse cost to 24 significant bits. Geometrically equal
// costs (the mirror-image corners of a symmetric part) differ in their last bits
// by round-off that depends on the mesh's scale and placement; rounding them
// together lets the index tie-break decide instead, so the collapse order -- and
// hence the rung -- does not change when the mesh is scaled or moved.
// Done on the IEEE-754 bits (round half up on the 29 dropped mantissa bits; a
// carry into the exponent is the correct round-up), which is exact and cheap.
inline double quantize(double c) {
  if (!(c > 0.0))
    return 0.0;
  u64 bits;
  std::memcpy(&bits, &c, sizeof(bits));
  const u64 drop = 52 - 23; // 52 stored mantissa bits; keep 23 (+ the implicit one)
  bits = (bits + (u64(1) << (drop - 1))) & ~((u64(1) << drop) - 1);
  std::memcpy(&c, &bits, sizeof(c));
  return c;
}

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

// pfor over [0,n) in fixed-size blocks: fn(begin, end) per block. The blocking
// is independent of the pool, so per-block results reduce identically either way.
inline void pfor_blocks(thread_pool *pool, std::size_t n, std::size_t block,
                        const std::function<void(std::size_t, std::size_t)> &fn) {
  const std::size_t nb = (n + block - 1) / block;
  pfor(pool, u32(nb), [&](int b) {
    const std::size_t lo = std::size_t(b) * block;
    fn(lo, std::min(n, lo + block));
  });
}

// Every undirected edge of T as a sorted key list (duplicates kept, so a run
// length is the number of incident triangles).
std::vector<u64> sorted_edge_keys(const std::vector<Tri> &T) {
  std::vector<u64> keys;
  keys.reserve(T.size() * 3);
  for (const Tri &t : T)
    for (int k = 0; k < 3; ++k)
      keys.push_back(edge_key(t[k], t[(k + 1) % 3]));
  std::sort(keys.begin(), keys.end());
  return keys;
}

// Uniform grid over a triangle set, for nearest-surface distance queries. The
// cell size is ~sqrt(mean triangle area) (a surface fills few cells of its
// bounding box), capped so the dense cell array stays within ~8 cells per
// triangle. Each triangle is binned into every cell of its bounding box that its
// plane passes through (a conservative plane/box overlap test), stored CSR-style
// in triangle order, so construction and every query are deterministic.
class tri_grid {
public:
  tri_grid(const std::vector<Vec3> &P, const std::vector<Tri> &T) : _P(P), _T(T) {
    if (T.empty())
      return;
    double lo[3] = {kInf, kInf, kInf}, hi[3] = {-kInf, -kInf, -kInf};
    double area = 0.0;
    for (const Tri &t : T) {
      for (int k = 0; k < 3; ++k)
        for (int a = 0; a < 3; ++a) {
          lo[a] = std::min(lo[a], axis(P[t[k]], a));
          hi[a] = std::max(hi[a], axis(P[t[k]], a));
        }
      area += 0.5 * len(cross(sub(P[t[1]], P[t[0]]), sub(P[t[2]], P[t[0]])));
    }
    const double nt = double(T.size());
    const double ext[3] = {hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]};
    const double diag = std::sqrt(ext[0] * ext[0] + ext[1] * ext[1] + ext[2] * ext[2]);
    _diag = diag;
    double h = area > 0.0 ? std::sqrt(area / nt) : diag / std::cbrt(nt);
    if (!(h > 0.0))
      h = 1.0; // every vertex coincides; one cell holds everything
    const double cap = 8.0 * nt + 1024.0;
    double cells = 1.0;
    for (int a = 0; a < 3; ++a)
      cells *= std::max(1.0, ext[a] / h);
    if (cells > cap)
      h *= std::cbrt(cells / cap);
    _h = h;
    _inv_h = 1.0 / h;
    for (int a = 0; a < 3; ++a) {
      _org[a] = lo[a];
      _n[a] = std::max<i64>(1, i64(std::ceil(ext[a] / h)));
      _top[a] = lo[a] + double(_n[a]) * h;
    }
    const std::size_t ncell = std::size_t(_n[0] * _n[1] * _n[2]);

    // Two passes over the same cell enumeration: count, then fill.
    _start.assign(ncell + 1, 0);
    for (u32 ti = 0; ti < u32(T.size()); ++ti)
      for_each_cell(ti, [&](std::size_t c) { ++_start[c + 1]; });
    for (std::size_t c = 0; c < ncell; ++c)
      _start[c + 1] += _start[c];
    _items.resize(_start[ncell]);
    std::vector<u32> fill(_start.begin(), _start.end() - 1);
    for (u32 ti = 0; ti < u32(T.size()); ++ti)
      for_each_cell(ti, [&](std::size_t c) { _items[fill[c]++] = ti; });
  }

  bool empty() const { return _T.empty(); }
  double diagonal() const { return _diag; } // of the triangles' bounding box

  // Squared distance from p to the nearest triangle. Searches shells of cells
  // outward from p's cell until no unvisited cell can hold anything closer.
  // Returns early -- with some value <= stop2 -- as soon as the distance is known
  // not to exceed stop2 (the caller only wants the largest distance).
  double nearest2(const Vec3 &p, double stop2) const {
    double best = kInf;
    // Clamp p into the grid box: q. For any point x in the box,
    // |p-x|^2 >= |p-q|^2 + |q-x|^2, so bounds taken from q stay valid for p.
    double q[3];
    i64 c[3];
    for (int a = 0; a < 3; ++a) {
      q[a] = std::min(_top[a], std::max(_org[a], axis(p, a)));
      c[a] = index(q[a], a);
    }
    const double dpq2 =
        (q[0] - p.x) * (q[0] - p.x) + (q[1] - p.y) * (q[1] - p.y) + (q[2] - p.z) * (q[2] - p.z);
    if (scan(cell(c[0], c[1], c[2]), p, stop2, best)) // shell 0: p's own cell
      return best;
    for (i64 r = 0;; ++r) {
      // Distance from q to the nearest cell outside the (2r+1)^3 block; sides
      // clipped by the grid border have nothing beyond them.
      double lb = kInf;
      for (int a = 0; a < 3; ++a) {
        if (c[a] - r > 0)
          lb = std::min(lb, q[a] - (_org[a] + double(c[a] - r) * _h));
        if (c[a] + r + 1 < _n[a])
          lb = std::min(lb, (_org[a] + double(c[a] + r + 1) * _h) - q[a]);
      }
      if (lb == kInf)
        return best; // the block covers the whole grid
      lb = std::max(0.0, lb);
      if (best <= dpq2 + lb * lb)
        return best;
      if (visit_shell(p, c, r + 1, stop2, best))
        return best;
    }
  }

private:
  template <class F> void for_each_cell(u32 ti, F f) const {
    const Vec3 &a = _P[_T[ti][0]], &b = _P[_T[ti][1]], &c = _P[_T[ti][2]];
    i64 i0[3], i1[3];
    for (int k = 0; k < 3; ++k) {
      const double mn = std::min(axis(a, k), std::min(axis(b, k), axis(c, k)));
      const double mx = std::max(axis(a, k), std::max(axis(b, k), axis(c, k)));
      // padded by a hair so rounding can only add a cell
      i0[k] = index(mn - 1e-9 * _h, k);
      i1[k] = index(mx + 1e-9 * _h, k);
    }
    if (i0[0] == i1[0] && i0[1] == i1[1] && i0[2] == i1[2]) {
      f(cell(i0[0], i0[1], i0[2])); // the common case: a triangle inside one cell
      return;
    }
    // plane/box: the box's support along n is (h/2)(|nx|+|ny|+|nz|); pad it so
    // rounding can only add cells, never drop one.
    const Vec3 n = cross(sub(b, a), sub(c, a));
    const double reach =
        0.5 * _h * (std::fabs(n.x) + std::fabs(n.y) + std::fabs(n.z)) * (1.0 + 1e-6) + 1e-300;
    for (i64 iz = i0[2]; iz <= i1[2]; ++iz)
      for (i64 iy = i0[1]; iy <= i1[1]; ++iy)
        for (i64 ix = i0[0]; ix <= i1[0]; ++ix) {
          const Vec3 ctr = {_org[0] + (double(ix) + 0.5) * _h, _org[1] + (double(iy) + 0.5) * _h,
                            _org[2] + (double(iz) + 0.5) * _h};
          if (std::fabs(dot(n, sub(ctr, a))) <= reach)
            f(cell(ix, iy, iz));
        }
  }

  // Cell index of coordinate x along axis k, clamped to the grid.
  i64 index(double x, int k) const {
    const double f = (x - _org[k]) * _inv_h;
    return f <= 0.0 ? 0 : std::min(_n[k] - 1, i64(f)); // f > 0: truncation is floor
  }
  std::size_t cell(i64 ix, i64 iy, i64 iz) const {
    return std::size_t((iz * _n[1] + iy) * _n[0] + ix);
  }

  // Test the triangles of one cell, lowering `best`. True once best <= stop2.
  bool scan(std::size_t cl, const Vec3 &p, double stop2, double &best) const {
    for (u32 k = _start[cl]; k < _start[cl + 1]; ++k) {
      const Tri &t = _T[_items[k]];
      const double d = tri_dist2(p, _P[t[0]], _P[t[1]], _P[t[2]]);
      if (d < best) {
        best = d;
        if (best <= stop2)
          return true;
      }
    }
    return false;
  }

  // Visit the cells at Chebyshev distance exactly r from c, lowering `best`.
  // Returns true once best <= stop2.
  bool visit_shell(const Vec3 &p, const i64 c[3], i64 r, double stop2, double &best) const {
    auto visit = [&](i64 ix, i64 iy, i64 iz) { return scan(cell(ix, iy, iz), p, stop2, best); };
    for (i64 dz = -r; dz <= r; ++dz) {
      const i64 iz = c[2] + dz;
      if (iz < 0 || iz >= _n[2])
        continue;
      for (i64 dy = -r; dy <= r; ++dy) {
        const i64 iy = c[1] + dy;
        if (iy < 0 || iy >= _n[1])
          continue;
        if (dz == -r || dz == r || dy == -r || dy == r) { // a full row of the shell
          const i64 x0 = std::max<i64>(0, c[0] - r), x1 = std::min(_n[0] - 1, c[0] + r);
          for (i64 ix = x0; ix <= x1; ++ix)
            if (visit(ix, iy, iz))
              return true;
        } else { // interior row: only its two end cells are on the shell
          if (c[0] - r >= 0 && visit(c[0] - r, iy, iz))
            return true;
          if (c[0] + r < _n[0] && visit(c[0] + r, iy, iz))
            return true;
        }
      }
    }
    return best <= stop2;
  }

  const std::vector<Vec3> &_P;
  const std::vector<Tri> &_T;
  double _org[3] = {0.0, 0.0, 0.0}; // grid box: _org .. _top
  double _top[3] = {0.0, 0.0, 0.0};
  double _h = 1.0, _inv_h = 1.0; // cell size
  double _diag = 0.0;
  i64 _n[3] = {1, 1, 1};
  std::vector<u32> _start; // cell c's triangles are _items[_start[c] .. _start[c+1])
  std::vector<u32> _items;
};

// Hausdorff sample points on a triangle surface: vertices, edge midpoints and
// face centroids, held as indices rather than points (~14 bytes per triangle
// for a full set).
struct sample_set {
  const std::vector<Vec3> *P;
  const std::vector<Tri> *T;
  std::vector<u32> verts; // vertex samples
  std::vector<u64> edges; // edge_key()s: midpoint samples
  std::vector<u32> tris;  // indices into *T: centroid samples

  std::size_t size() const { return verts.size() + edges.size() + tris.size(); }
  Vec3 at(std::size_t i) const {
    if (i < verts.size())
      return (*P)[verts[i]];
    i -= verts.size();
    if (i < edges.size()) {
      const Vec3 &a = (*P)[u32(edges[i] >> 32)], &b = (*P)[u32(edges[i] & 0xffffffffu)];
      return {0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.z + b.z)};
    }
    const Tri &t = (*T)[tris[i - edges.size()]];
    const Vec3 &a = (*P)[t[0]], &b = (*P)[t[1]], &c = (*P)[t[2]];
    return {(a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0, (a.z + b.z + c.z) / 3.0};
  }
};

// Every vertex a triangle of T uses, ascending.
std::vector<u32> used_vertices(std::size_t nv, const std::vector<Tri> &T) {
  std::vector<char> used(nv, 0);
  for (const Tri &t : T)
    used[t[0]] = used[t[1]] = used[t[2]] = 1;
  std::vector<u32> v;
  for (u32 i = 0; i < u32(nv); ++i)
    if (used[i])
      v.push_back(i);
  return v;
}

std::vector<u64> unique_edges(const std::vector<Tri> &T) {
  std::vector<u64> e = sorted_edge_keys(T);
  e.erase(std::unique(e.begin(), e.end()), e.end());
  return e;
}

// The documented full sample set of a surface: every used vertex, every unique
// edge midpoint and every face centroid.
sample_set all_samples(const std::vector<Vec3> &P, const std::vector<Tri> &T) {
  sample_set s{&P, &T, used_vertices(P.size(), T), unique_edges(T), std::vector<u32>(T.size())};
  std::iota(s.tris.begin(), s.tris.end(), 0u);
  return s;
}

// max over the samples of the squared distance to the grid's surface, exact
// whenever it exceeds floor2. Blocks fan over the pool; each block keeps its own
// running max, which lets a query stop as soon as it cannot raise it (or is
// already known to be under floor2), and the block maxima reduce in order: the
// sample achieving the max is never cut short, so pooled == serial bit for bit.
double max_dist2(const sample_set &s, const tri_grid &g, double floor2, thread_pool *pool) {
  const std::size_t block = 2048, stride = 61;
  const std::size_t n = s.size();
  // A sparse first pass seeds every block with a lower bound on the answer.
  double seed = 0.0;
  for (std::size_t i = 0; i < n; i += stride)
    seed = std::max(seed, g.nearest2(s.at(i), std::max(seed, floor2)));
  std::vector<double> bmax((n + block - 1) / block, 0.0);
  pfor_blocks(pool, n, block, [&](std::size_t lo, std::size_t hi) {
    double m = seed;
    for (std::size_t i = lo; i < hi; ++i)
      m = std::max(m, g.nearest2(s.at(i), std::max(m, floor2)));
    bmax[lo / block] = m;
  });
  double m = seed;
  for (double b : bmax)
    m = std::max(m, b);
  return m;
}

// Distances under 1e-9 of the larger surface's extent are round-off (a sample
// on a coplanar triangle lands ~1e-16 off it): they count as zero, so a result
// that lost no geometry reports exactly 0, and every sample that lands on the
// other surface stops at the first triangle that proves it.
double hausdorff(const sample_set &sa, const tri_grid &ga, const sample_set &sb, const tri_grid &gb,
                 thread_pool *pool) {
  if (ga.empty() && gb.empty())
    return 0.0;
  if (ga.empty() || gb.empty())
    return kInf;
  const double floor = 1e-9 * std::max(ga.diagonal(), gb.diagonal());
  const double d2 =
      std::max(max_dist2(sa, gb, floor * floor, pool), max_dist2(sb, ga, floor * floor, pool));
  return d2 <= floor * floor ? 0.0 : std::sqrt(d2);
}

// Positions + triangles (quads fan-triangulated) of a geometry.
void gather(const geometry &g, std::vector<Vec3> &P, std::vector<Tri> &T) {
  const geometry::points_t &pts = g.const_points();
  P.resize(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i)
    P[i] = {pts[i][0], pts[i][1], pts[i][2]};
  const geometry::tris_t &tris = g.const_tris();
  const geometry::quads_t &quads = g.const_quads();
  T.clear();
  T.reserve(tris.size() + 2 * quads.size());
  for (const auto &t : tris)
    T.push_back({u32(t[0]), u32(t[1]), u32(t[2])});
  for (const auto &q : quads) { // fan-triangulate each quad
    T.push_back({u32(q[0]), u32(q[1]), u32(q[2])});
    T.push_back({u32(q[0]), u32(q[2]), u32(q[3])});
  }
}

// Mark every vertex whose position lies within eps of a vertex of a DIFFERENT
// connected component (triangles joined through shared vertex indices); those
// are the seams between touching unwelded parts. Returns the number marked.
u64 find_seam_locks(const std::vector<Vec3> &P, const std::vector<Tri> &T, double seam_epsilon,
                    thread_pool *pool, std::vector<char> &locked) {
  const u32 nv = u32(P.size());
  // components: union-find over triangle edges, rooted at the smallest index
  std::vector<u32> parent(nv);
  std::iota(parent.begin(), parent.end(), 0u);
  auto find = [&](u32 x) {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  };
  auto unite = [&](u32 a, u32 b) {
    a = find(a);
    b = find(b);
    if (a != b)
      parent[std::max(a, b)] = std::min(a, b);
  };
  std::vector<char> used(nv, 0);
  for (const Tri &t : T) {
    used[t[0]] = used[t[1]] = used[t[2]] = 1;
    unite(t[0], t[1]);
    unite(t[0], t[2]);
  }
  std::vector<u32> comp(nv);
  std::vector<u32> verts;
  double lo[3] = {kInf, kInf, kInf}, hi[3] = {-kInf, -kInf, -kInf};
  bool several = false;
  for (u32 v = 0; v < nv; ++v) {
    comp[v] = find(v);
    if (!used[v])
      continue;
    if (!verts.empty() && comp[v] != comp[verts.front()])
      several = true;
    verts.push_back(v);
    for (int a = 0; a < 3; ++a) {
      lo[a] = std::min(lo[a], axis(P[v], a));
      hi[a] = std::max(hi[a], axis(P[v], a));
    }
  }
  if (!several)
    return 0; // one component: nothing to seam

  const double diag =
      std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) + (hi[1] - lo[1]) * (hi[1] - lo[1]) +
                (hi[2] - lo[2]) * (hi[2] - lo[2]));
  const double eps = seam_epsilon >= 0.0 ? seam_epsilon : 1e-6 * diag;
  // Hash cells of 64*eps: an eps-ball then touches a second cell along an axis
  // only when it lies within eps of that cell's face (1 time in 32), so nearly
  // every search stays inside the vertex's own cell.
  // (Never finer than 1e-12 of the extent, so cell indices stay far from overflow.)
  const double cellw =
      std::max(eps > 0.0 ? 64.0 * eps : (diag > 0.0 ? 1e-6 * diag : 1.0), 1e-12 * diag);
  auto cell_of = [&](double x, int a) { return i64(std::floor((x - lo[a]) / cellw)); };

  struct Key {
    i64 c[3];
    u32 v;
  };
  auto cell_less = [](const Key &a, const Key &b) {
    if (a.c[0] != b.c[0])
      return a.c[0] < b.c[0];
    if (a.c[1] != b.c[1])
      return a.c[1] < b.c[1];
    return a.c[2] < b.c[2];
  };
  std::vector<Key> keys(verts.size());
  for (std::size_t i = 0; i < verts.size(); ++i) {
    const Vec3 &p = P[verts[i]];
    keys[i] = {{cell_of(p.x, 0), cell_of(p.y, 1), cell_of(p.z, 2)}, verts[i]};
  }
  std::sort(keys.begin(), keys.end(), [&](const Key &a, const Key &b) {
    return cell_less(a, b) || (!cell_less(b, a) && a.v < b.v);
  });

  // Each vertex writes only its own flag.
  const double eps2 = eps * eps;
  auto partner = [&](u32 v, const Key &k) {
    return comp[k.v] != comp[v] && dist2(P[k.v], P[v]) <= eps2;
  };
  pfor_blocks(pool, keys.size(), 1024, [&](std::size_t b, std::size_t e) {
    for (std::size_t i = b; i < e; ++i) {
      const u32 v = keys[i].v;
      const Vec3 &p = P[v];
      bool hit = false;
      // own cell: the run of equal cells around i
      for (std::size_t j = i; j-- > 0 && !cell_less(keys[j], keys[i]) && !hit;)
        hit = partner(v, keys[j]);
      for (std::size_t j = i + 1; j < keys.size() && !cell_less(keys[i], keys[j]) && !hit; ++j)
        hit = partner(v, keys[j]);
      // neighbour cells the eps-ball reaches into
      i64 c0[3], c1[3];
      for (int a = 0; a < 3; ++a) {
        c0[a] = cell_of(axis(p, a) - eps, a);
        c1[a] = cell_of(axis(p, a) + eps, a);
      }
      for (i64 x = c0[0]; x <= c1[0] && !hit; ++x)
        for (i64 y = c0[1]; y <= c1[1] && !hit; ++y)
          for (i64 z = c0[2]; z <= c1[2] && !hit; ++z) {
            const Key probe = {{x, y, z}, 0};
            if (!cell_less(probe, keys[i]) && !cell_less(keys[i], probe))
              continue; // own cell, done above
            auto it = std::lower_bound(keys.begin(), keys.end(), probe, cell_less);
            for (; it != keys.end() && !cell_less(probe, *it) && !hit; ++it)
              hit = partner(v, *it);
          }
      if (hit)
        locked[v] = 1;
    }
  });
  u64 n = 0;
  for (u32 v : verts)
    n += locked[v] ? 1 : 0;
  return n;
}

} // namespace

std::vector<geometry> simplify_progressive(const geometry &mesh,
                                           const std::vector<std::uint64_t> &targets,
                                           const simplify_params &params,
                                           std::vector<simplify_result> *out, thread_pool *pool) {
  const std::size_t K = targets.size();
  std::vector<geometry> snaps(K, mesh); // a target at/above the input keeps the input
  if (out)
    out->assign(K, simplify_result());
  if (K == 0)
    return snaps;

  // --- gather triangles (triangulating quads) + positions ---
  std::vector<Vec3> P;
  std::vector<Tri> T;
  gather(mesh, P, T);
  const u64 n_in = T.size();
  if (out)
    for (simplify_result &r : *out)
      r.in_tris = r.out_tris = n_in;

  // Visit the targets from the largest count down; snapshot k fires the first
  // time the live triangle count is at or below targets[k].
  std::vector<std::size_t> order(K);
  std::iota(order.begin(), order.end(), std::size_t(0));
  std::stable_sort(order.begin(), order.end(),
                   [&](std::size_t a, std::size_t b) { return targets[a] > targets[b]; });
  std::size_t next = 0;
  // Nothing to do for these: no faces, or already at/under the target.
  while (next < K && (n_in == 0 || targets[order[next]] >= n_in))
    ++next;
  if (next == K)
    return snaps;

  const std::vector<Tri> T0 = out ? T : std::vector<Tri>(); // source surface, for world_error
  const u32 nv = u32(P.size());

  // The quadrics work in a frame centred on the mesh's bounding box, so their
  // round-off does not grow with the distance from the world origin. Costs whose
  // RMS plane distance is below 1e-6 of the bounding-box diagonal are that
  // round-off, not geometry: they are snapped to exactly 0 so coplanar collapses
  // tie exactly (and resolve by vertex index) whatever the mesh's scale or
  // placement.
  std::vector<Vec3> L(P);
  double cost_floor = 0.0; // squared RMS distance below which a cost is 0
  {
    double lo[3] = {kInf, kInf, kInf}, hi[3] = {-kInf, -kInf, -kInf};
    for (const Tri &t : T)
      for (int k = 0; k < 3; ++k)
        for (int a = 0; a < 3; ++a) {
          lo[a] = std::min(lo[a], axis(P[t[k]], a));
          hi[a] = std::max(hi[a], axis(P[t[k]], a));
        }
    const Vec3 c = {0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2])};
    for (Vec3 &v : L)
      v = sub(v, c);
    const Vec3 ext = {hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]};
    cost_floor = 1e-12 * dot(ext, ext);
  }
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
    const Vec3 &a = L[T[ti][0]], &b = L[T[ti][1]], &c = L[T[ti][2]];
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

  // Every undirected edge once (sorted), and the boundary ones: on exactly one tri.
  std::vector<u64> edges = sorted_edge_keys(T);
  std::vector<u64> boundary;
  for (std::size_t i = 0; i < edges.size();) {
    std::size_t j = i + 1;
    while (j < edges.size() && edges[j] == edges[i])
      ++j;
    if (j - i == 1)
      boundary.push_back(edges[i]);
    i = j;
  }
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());

  // --- boundary constraint quadrics: for each boundary edge, add a plane through
  // the edge, perpendicular to the face, so the border holds. Weighted by
  // boundary_weight * |e|^2 -- the same length^2 units as the area-weighted face
  // planes -- so the relative strength (and the collapse order) is independent
  // of the mesh's scale. ---
  if (params.preserve_boundary && !boundary.empty()) {
    for (u32 ti = 0; ti < T.size(); ++ti) {
      const Vec3 &a = L[T[ti][0]], &b = L[T[ti][1]], &c = L[T[ti][2]];
      Vec3 fn = face_normal(a, b, c);
      for (int k = 0; k < 3; ++k) {
        u32 v0 = T[ti][k], v1 = T[ti][(k + 1) % 3];
        if (!std::binary_search(boundary.begin(), boundary.end(), edge_key(v0, v1)))
          continue; // interior edge
        // plane containing edge (v0,v1) and perpendicular to the face
        Vec3 e = sub(L[v1], L[v0]);
        Vec3 pn = cross(e, fn);
        double pl = len(pn);
        if (pl < 1e-30)
          continue;
        pn = {pn.x / pl, pn.y / pl, pn.z / pl};
        double d = -dot(pn, L[v0]);
        double w = params.boundary_weight * dot(e, e);
        Quadric bq = Quadric::from_plane(pn.x, pn.y, pn.z, d, w);
        Q[v0] += bq;
        Q[v1] += bq;
      }
    }
  }

  // --- seam locks: never drop a vertex shared with another component ---
  std::vector<char> locked(nv, 0);
  const u64 nlocked =
      params.lock_component_seams ? find_seam_locks(P, T, params.seam_epsilon, pool, locked) : 0;

  const double flip_cos = std::cos(params.max_normal_flip_deg * 3.14159265358979323846 / 180.0);

  // Evaluate the half-edge collapse of edge (u,v): the cheaper allowed direction.
  // A seam-locked vertex may absorb its neighbour but is never the one dropped;
  // an edge between two locked vertices is not collapsible at all.
  auto eval = [&](u32 u, u32 v, double &cost, u32 &keep, u32 &drop) -> bool {
    if (locked[u] && locked[v])
      return false;
    Quadric q = Q[u];
    q += Q[v];
    double eu = q.error(L[u].x, L[u].y, L[u].z);
    double ev = q.error(L[v].x, L[v].y, L[v].z);
    eu = eu <= cost_floor * q.w ? 0.0 : quantize(eu);
    ev = ev <= cost_floor * q.w ? 0.0 : quantize(ev);
    if (locked[u] || (!locked[v] && eu <= ev)) {
      keep = u;
      drop = v;
      cost = eu;
    } else {
      keep = v;
      drop = u;
      cost = ev;
    }
    return true;
  };

  std::priority_queue<HeapItem> heap;
  auto push_edge = [&](u32 u, u32 v) {
    if (u == v || vremoved[u] || vremoved[v])
      return;
    double cost;
    u32 keep, drop;
    if (eval(u, v, cost, keep, drop))
      heap.push(HeapItem{cost, keep, drop, vver[keep], vver[drop]});
  };

  // seed the heap with every unique edge
  for (u64 key : edges)
    push_edge(u32(key >> 32), u32(key & 0xffffffffu));

  // Would collapsing `drop` onto `keep` flip any surviving incident face?
  auto causes_flip = [&](u32 keep, u32 drop) -> bool {
    for (u32 ti : vtri[drop]) {
      if (tremoved[ti])
        continue;
      const Tri &f = T[ti];
      if (f[0] == keep || f[1] == keep || f[2] == keep)
        continue; // this face degenerates away, not a flip
      Vec3 p[3];
      for (int k = 0; k < 3; ++k)
        p[k] = (f[k] == drop) ? L[keep] : L[f[k]];
      Vec3 before = face_normal(L[f[0]], L[f[1]], L[f[2]]);
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
  bool hit_limit = false;

  // --- snapshots: rebuild a compact geometry from the survivors ---
  const geometry::points_t &in_pts = mesh.const_points();
  const geometry::uvs_t &in_uv = mesh.const_uvs();
  const geometry::colors_t &in_col = mesh.const_colors();
  const geometry::normals_t &in_nrm = mesh.const_normals();
  const bool has_uv = in_uv.size() == in_pts.size();
  const bool has_col = in_col.size() == in_pts.size();
  const bool has_nrm = !params.recompute_normals && in_nrm.size() == in_pts.size();

  auto build = [&]() {
    geometry result(mesh.ctx());
    std::vector<u32> remap(nv, u32(-1));
    geometry::points_t &op = result.points();
    geometry::uvs_t *ouv = has_uv ? &result.uvs() : nullptr;
    geometry::colors_t *ocol = has_col ? &result.colors() : nullptr;
    geometry::normals_t *onrm = has_nrm ? &result.normals() : nullptr;
    for (u32 v = 0; v < nv; ++v) {
      if (vremoved[v])
        continue;
      remap[v] = u32(op.size());
      op.push_back(in_pts[v]);
      if (ouv)
        ouv->push_back(in_uv[v]);
      if (ocol)
        ocol->push_back(in_col[v]);
      if (onrm)
        onrm->push_back(in_nrm[v]); // half-edge collapse: the vertex never moved
    }
    geometry::tris_t &ot = result.tris();
    for (u32 ti = 0; ti < T.size(); ++ti) {
      if (tremoved[ti])
        continue;
      const Tri &f = T[ti];
      // guard against any residual degeneracy
      if (f[0] == f[1] || f[1] == f[2] || f[0] == f[2])
        continue;
      ot.push_back({remap[f[0]], remap[f[1]], remap[f[2]]});
    }
    result.set_geometry_type(geometry::SURFACE_TRI);
    if (!has_nrm)
      result.compute_normals();
    return result;
  };

  // world_error: symmetric sampled Hausdorff between the source and the live
  // surface. The source side (grid, used vertices, unique edges) is built once
  // and reused by every snapshot; construction is deterministic, so a
  // progressive snapshot measures exactly what an independent run would. Samples
  // the two surfaces share -- every live vertex, an edge present in both, an
  // untouched triangle's centroid -- lie ON the other surface (distance 0, under
  // the round-off floor), so they are not queried: the value is bit-identical to
  // sampled_hausdorff(mesh, snapshot), for less work on fine rungs.
  std::unique_ptr<tri_grid> src_grid;
  std::vector<u32> src_verts;
  std::vector<u64> src_edges;
  auto measure = [&]() {
    if (!src_grid) {
      src_grid.reset(new tri_grid(P, T0));
      src_verts = used_vertices(nv, T0);
      src_edges = unique_edges(T0);
    }
    std::vector<Tri> live;
    live.reserve(ntris);
    std::vector<char> untouched(T.size(), 0); // a source triangle that is still live, as is
    std::vector<u32> changed;                 // live triangles (indices into `live`) that are new
    for (u32 ti = 0; ti < T.size(); ++ti) {
      if (tremoved[ti])
        continue;
      if (T[ti] == T0[ti])
        untouched[ti] = 1;
      else
        changed.push_back(u32(live.size()));
      live.push_back(T[ti]);
    }
    const tri_grid grid(P, live);
    const std::vector<u64> live_edges = unique_edges(live);
    std::vector<char> on_live(nv, 0);
    for (const Tri &t : live)
      on_live[t[0]] = on_live[t[1]] = on_live[t[2]] = 1;

    sample_set src{&P, &T0, {}, {}, {}}; // source samples to test against the live surface
    for (u32 v : src_verts)
      if (!on_live[v])
        src.verts.push_back(v);
    std::set_difference(src_edges.begin(), src_edges.end(), live_edges.begin(), live_edges.end(),
                        std::back_inserter(src.edges));
    for (u32 ti = 0; ti < T0.size(); ++ti)
      if (!untouched[ti])
        src.tris.push_back(ti);
    // live samples to test against the source; every live vertex is a source vertex
    sample_set lv{&P, &live, {}, {}, changed};
    std::set_difference(live_edges.begin(), live_edges.end(), src_edges.begin(), src_edges.end(),
                        std::back_inserter(lv.edges));
    return hausdorff(src, *src_grid, lv, grid, pool);
  };

  std::size_t last = K; // most recent snapshot, reused while no collapse happened since
  u64 last_collapses = 0;
  auto emit = [&](std::size_t k) {
    if (last != K && last_collapses == ncollapse) {
      snaps[k] = snaps[last];
      if (out) {
        (*out)[k] = (*out)[last];
        (*out)[k].hit_error_limit = hit_limit;
      }
      return;
    }
    snaps[k] = build();
    if (out) {
      simplify_result &r = (*out)[k];
      r.out_tris = ntris;
      r.collapses = ncollapse;
      r.world_error = ncollapse > 0 ? measure() : 0.0;
      r.hit_error_limit = hit_limit;
      r.locked_vertices = nlocked;
    }
    last = k;
    last_collapses = ncollapse;
  };

  while (true) {
    while (next < K && ntris <= targets[order[next]])
      emit(order[next++]);
    if (next == K || heap.empty())
      break;
    HeapItem it = heap.top();
    heap.pop();
    // stale? (either endpoint changed or was removed since this was pushed)
    if (vremoved[it.keep] || vremoved[it.drop] || vver[it.keep] != it.keep_ver ||
        vver[it.drop] != it.drop_ver)
      continue;
    if (params.max_error >= 0.0) {
      // metre-valued proxy: RMS distance of `keep` to the planes summed in
      const double w = Q[it.keep].w + Q[it.drop].w;
      const double proxy = w > 0.0 ? std::sqrt(it.cost / w) : 0.0;
      if (proxy > params.max_error) {
        hit_limit = true;
        break;
      }
    }
    if (causes_flip(it.keep, it.drop))
      continue; // leave this edge out; a re-pushed copy may succeed later

    const u32 keep = it.keep, drop = it.drop;

    // rewrite drop's faces onto keep; drop degenerate ones
    for (u32 ti : vtri[drop]) {
      if (tremoved[ti])
        continue;
      Tri &f = T[ti];
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
  // Targets the loop could not reach (heap exhausted, or the error budget hit)
  // all get the final state.
  while (next < K)
    emit(order[next++]);
  return snaps;
}

geometry simplify(const geometry &mesh, const simplify_params &params, simplify_result *out,
                  thread_pool *pool) {
  const u64 n_in = mesh.const_tris().size() + 2 * mesh.const_quads().size();
  u64 target = params.target_tris;
  if (target == 0) {
    double r = params.target_ratio;
    if (r <= 0.0)
      r = 0.0;
    if (r > 1.0)
      r = 1.0;
    target = u64(std::llround(double(n_in) * r));
  }
  std::vector<simplify_result> res;
  std::vector<geometry> snaps = simplify_progressive(mesh, std::vector<std::uint64_t>(1, target),
                                                     params, out ? &res : nullptr, pool);
  if (out)
    *out = res[0];
  return snaps[0];
}

double sampled_hausdorff(const geometry &a, const geometry &b, thread_pool *pool) {
  std::vector<Vec3> PA, PB;
  std::vector<Tri> TA, TB;
  gather(a, PA, TA);
  gather(b, PB, TB);
  const tri_grid ga(PA, TA), gb(PB, TB);
  return hausdorff(all_samples(PA, TA), ga, all_samples(PB, TB), gb, pool);
}

} // namespace cvc
