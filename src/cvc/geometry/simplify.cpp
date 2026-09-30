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
// the lower-error one, so no vertex is ever placed anywhere but at an input
// position. That is robust (no optimal-placement 3x3 solve or its singular
// fallbacks), attribute-safe across uv seams, and a good fit for the
// feature-aligned architectural meshes this first targets.
//
// Collapses run on the WELDED topology: input vertices that coincide across
// connected components (unwelded parts that touch, like a roof on its walls) or
// sit at bit-identical positions (splits for hard-edge normals, uv seams) form
// one collapse vertex -- a "class" -- so the parts decimate together and no seam
// opens. The input vertices themselves ("wedges") stay distinct and keep their
// own uv, color and normal. When a class is dropped onto a neighbour, each of its
// wedges follows the wedge it shares a vanishing triangle with; a wedge with no
// such partner (a face split away from the collapsed edge) moves onto the kept
// position.
//
// A per-class 4x4 quadric accumulates the area-weighted squared distance to the
// planes of every incident face (Garland & Heckbert '97); boundary edges add a
// perpendicular constraint plane weighted by |edge|^2, so open borders stay put
// and every term shares the same units (length^2 weight x length^2 distance),
// which keeps the collapse order invariant under a uniform scale. Each quadric
// lives in a frame centred on its own vertex, where every incident plane passes
// through the origin exactly; absorbing a neighbour translates that neighbour's
// quadric by the edge vector (an exact difference of nearby coordinates). The
// round-off of a cost is thus relative to local lengths -- never to the mesh's
// extent or to its distance from the world origin -- and is tracked: a cost
// inside its round-off band is snapped to exactly 0 and the rest are compared at
// 24-bit precision, so geometric ties stay ties when the mesh is scaled or moved,
// and geometry elsewhere cannot perturb a local collapse. A lazy binary heap
// keyed on a per-vertex version pops the cheapest valid collapse, with ties
// broken by vertex index so the pop sequence never depends on the standard
// library's heap. Fold-overs are rejected by a per-face normal-flip test so the
// coarse rung never self-intersects.
//
// The quadric cost is area x distance^2 -- not a length -- so the reported error
// is measured instead of derived: a sampled symmetric Hausdorff distance between
// the input and each result, answered by exact point-to-triangle distances over
// a bounding-volume hierarchy.
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
#include <utility>
#include <vector>

namespace cvc {
namespace {

typedef std::uint32_t u32;
typedef std::uint64_t u64;
typedef std::int64_t i64;
typedef std::array<u32, 3> Tri;

const double kInf = std::numeric_limits<double>::infinity();
const u32 kNone = u32(-1);

// A collapse cost within this fraction of the magnitude of the terms it was
// summed from (Quadric::mag, Quadric::m) is round-off, not geometry. 2^-40 is
// ~4096 ulps: well above what an evaluation, a merge or a long plane sum can
// accumulate, and far below any real cost (it is an RMS plane distance of ~1e-6
// of the lengths involved).
const double kCostRoundoff = 1.0 / 1099511627776.0;

struct Vec3 {
  double x, y, z;
};
inline Vec3 add(const Vec3 &a, const Vec3 &b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
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

// Symmetric 4x4 error quadric in a frame centred on its vertex: the squared,
// plane-summed distance at offset o from the vertex is
//   o^T A o + 2 b.o + c,   A = sum w n n^T  (stored xx xy xz yy yz zz).
// A plane through the vertex adds to A only; the planes of an absorbed
// neighbour bring their offsets in b and c. `w` is the total weight of the
// planes summed in (so cost / w is the weighted mean squared plane distance --
// the max_error proxy), and `m` accumulates the magnitude of every term folded
// into b and c: the scale of their round-off.
struct Quadric {
  double a[6];
  Vec3 b;
  double c, w, m;
  Quadric() : b{0.0, 0.0, 0.0}, c(0.0), w(0.0), m(0.0) { std::fill(a, a + 6, 0.0); }

  // Add the plane through the vertex with unit normal n, weighted wt.
  void add_plane(const Vec3 &n, double wt) {
    a[0] += wt * n.x * n.x;
    a[1] += wt * n.x * n.y;
    a[2] += wt * n.x * n.z;
    a[3] += wt * n.y * n.y;
    a[4] += wt * n.y * n.z;
    a[5] += wt * n.z * n.z;
    w += wt;
  }
  // Add planes through the vertex held in q (q.b, q.c are 0).
  void add_planes(const Quadric &q) {
    for (int i = 0; i < 6; ++i)
      a[i] += q.a[i];
    w += q.w;
  }
  Vec3 A(const Vec3 &o) const {
    return {a[0] * o.x + a[1] * o.y + a[2] * o.z, a[1] * o.x + a[3] * o.y + a[4] * o.z,
            a[2] * o.x + a[4] * o.y + a[5] * o.z};
  }
  // The squared, plane-summed distance at offset o from the vertex.
  double at(const Vec3 &o) const { return dot(o, A(o)) + 2.0 * dot(b, o) + c; }
  // A bound on the magnitude of at(o)'s terms: sum w (|o| + |d|)^2 over the
  // planes (d = a plane's offset), which Cauchy-Schwarz bounds by
  // (sqrt(w)|o| + sqrt(c))^2.
  double mag(const Vec3 &o) const {
    const double s = std::sqrt(w) * len(o) + std::sqrt(std::max(0.0, c));
    return s * s;
  }
  // Absorb q, the quadric of a vertex at offset -o from this one (o = this
  // vertex's position minus q's): q's planes, re-expressed in this frame.
  void merge(const Quadric &q, const Vec3 &o) {
    m += q.m + q.mag(o);
    c += q.at(o);
    b = add(b, add(q.b, q.A(o)));
    add_planes(q);
  }
};

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

// Bounding-volume hierarchy over a triangle set, for nearest-surface distance
// queries: median split along the longest axis of the triangle centroids' box,
// a few triangles per leaf (their corners copied into leaf order). Unlike a
// uniform grid it adapts to wildly mixed triangle sizes (a ground quad
// kilometres wide under centimetre detail), so a query stays logarithmic
// wherever it lands. Ties in the split order break by triangle index, so the
// tree -- and every query -- is a pure function of the input.
class tri_bvh {
public:
  tri_bvh(const std::vector<Vec3> &P, const std::vector<Tri> &T) {
    const u32 n = u32(T.size());
    if (n == 0)
      return;
    std::vector<Vec3> ctr(n);
    for (u32 i = 0; i < n; ++i) {
      const Vec3 &a = P[T[i][0]], &b = P[T[i][1]], &c = P[T[i][2]];
      ctr[i] = {(a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0, (a.z + b.z + c.z) / 3.0};
    }
    std::vector<u32> items(n);
    std::iota(items.begin(), items.end(), 0u);
    _nodes.reserve(2 * (n / kLeaf) + 2);
    _nodes.push_back(Node());
    struct Job {
      u32 node, b, e;
    };
    std::vector<Job> jobs(1, Job{0, 0, n});
    while (!jobs.empty()) {
      const Job j = jobs.back();
      jobs.pop_back();
      Node nd;
      double clo[3] = {kInf, kInf, kInf}, chi[3] = {-kInf, -kInf, -kInf};
      for (u32 i = j.b; i < j.e; ++i) {
        const Tri &t = T[items[i]];
        for (int k = 0; k < 3; ++k)
          for (int a = 0; a < 3; ++a) {
            nd.lo[a] = std::min(nd.lo[a], axis(P[t[k]], a));
            nd.hi[a] = std::max(nd.hi[a], axis(P[t[k]], a));
          }
        for (int a = 0; a < 3; ++a) {
          clo[a] = std::min(clo[a], axis(ctr[items[i]], a));
          chi[a] = std::max(chi[a], axis(ctr[items[i]], a));
        }
      }
      if (j.e - j.b <= kLeaf) {
        nd.first = j.b;
        nd.count = j.e - j.b;
        _nodes[j.node] = nd;
        continue;
      }
      int ax = 0;
      for (int a = 1; a < 3; ++a)
        if (chi[a] - clo[a] > chi[ax] - clo[ax])
          ax = a;
      // A NaN coordinate sorts last, so the order stays strict and weak.
      auto key = [&](u32 i) {
        const double v = axis(ctr[i], ax);
        return v == v ? v : kInf;
      };
      const u32 mid = j.b + (j.e - j.b) / 2;
      std::nth_element(items.begin() + j.b, items.begin() + mid, items.begin() + j.e,
                       [&](u32 x, u32 y) {
                         const double kx = key(x), ky = key(y);
                         return kx < ky || (kx == ky && x < y);
                       });
      nd.first = u32(_nodes.size());
      nd.count = 0;
      _nodes[j.node] = nd;
      _nodes.push_back(Node());
      _nodes.push_back(Node());
      jobs.push_back(Job{nd.first, j.b, mid});
      jobs.push_back(Job{nd.first + 1, mid, j.e});
    }
    _tri.resize(n);
    for (u32 i = 0; i < n; ++i)
      _tri[i] = {P[T[items[i]][0]], P[T[items[i]][1]], P[T[items[i]][2]]};
    const Node &r = _nodes[0];
    const double ext[3] = {r.hi[0] - r.lo[0], r.hi[1] - r.lo[1], r.hi[2] - r.lo[2]};
    _diag = std::sqrt(ext[0] * ext[0] + ext[1] * ext[1] + ext[2] * ext[2]);
  }

  bool empty() const { return _tri.empty(); }
  double diagonal() const { return _diag; } // of the triangles' bounding box

  // Squared distance from p to the nearest triangle, visiting the nearer child
  // first and skipping any box no closer than the best so far. Returns early --
  // with some value <= stop2, never below the true distance -- as soon as the
  // distance is known not to exceed stop2 (the caller only wants the largest).
  // `hint` names a leaf to try first (kNone for none) and receives the leaf of
  // the nearest triangle found: consecutive samples are usually neighbours, so
  // a query that only has to prove "within stop2" often ends there, without a
  // descent. The hint never changes an exact distance returned above stop2.
  double nearest2(const Vec3 &p, double stop2, u32 &hint) const {
    double best = kInf;
    if (_nodes.empty())
      return best;
    if (hint != kNone) {
      const Node &nd = _nodes[hint];
      for (u32 k = nd.first; k < nd.first + nd.count; ++k) {
        best = std::min(best, tri_dist2(p, _tri[k][0], _tri[k][1], _tri[k][2]));
        if (best <= stop2)
          return best;
      }
    }
    struct Entry {
      double d2;
      u32 node;
    };
    // Each level pushes at most one entry more than it pops, and the median
    // split halves every node, so the depth (< 32 levels) bounds the stack.
    Entry stack[96];
    int sp = 0;
    stack[sp++] = Entry{box_dist2(_nodes[0], p), 0};
    while (sp > 0) {
      const Entry e = stack[--sp];
      if (e.d2 >= best)
        continue;
      const Node &nd = _nodes[e.node];
      if (nd.count > 0) {
        for (u32 k = nd.first; k < nd.first + nd.count; ++k) {
          const double d = tri_dist2(p, _tri[k][0], _tri[k][1], _tri[k][2]);
          if (d < best) {
            best = d;
            hint = e.node;
            if (best <= stop2)
              return best;
          }
        }
        continue;
      }
      const double dl = box_dist2(_nodes[nd.first], p), dr = box_dist2(_nodes[nd.first + 1], p);
      const bool left_first = !(dr < dl);
      const Entry nearer = left_first ? Entry{dl, nd.first} : Entry{dr, nd.first + 1};
      const Entry farther = left_first ? Entry{dr, nd.first + 1} : Entry{dl, nd.first};
      if (farther.d2 < best)
        stack[sp++] = farther;
      if (nearer.d2 < best)
        stack[sp++] = nearer; // popped first
    }
    return best;
  }

private:
  static constexpr u32 kLeaf = 4; // triangles per leaf
  struct Node {
    double lo[3] = {kInf, kInf, kInf}; // bounding box of the node's triangles
    double hi[3] = {-kInf, -kInf, -kInf};
    u32 first = 0; // leaf: _tri[first .. first+count); interior: children first, first+1
    u32 count = 0;
  };

  static double box_dist2(const Node &nd, const Vec3 &p) {
    double d2 = 0.0;
    for (int a = 0; a < 3; ++a) {
      const double x = axis(p, a);
      const double d = x < nd.lo[a] ? nd.lo[a] - x : (x > nd.hi[a] ? x - nd.hi[a] : 0.0);
      d2 += d * d;
    }
    return d2;
  }

  std::vector<Node> _nodes;
  std::vector<std::array<Vec3, 3>> _tri; // triangle corners in leaf order
  double _diag = 0.0;
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

// Every undirected edge of T once, sorted.
std::vector<u64> unique_edges(const std::vector<Tri> &T) {
  std::vector<u64> e;
  e.reserve(T.size() * 3);
  for (const Tri &t : T)
    for (int k = 0; k < 3; ++k)
      e.push_back(edge_key(t[k], t[(k + 1) % 3]));
  std::sort(e.begin(), e.end());
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

// max over the samples of the squared distance to the tree's surface, exact
// whenever it exceeds floor2. Blocks fan over the pool; each block keeps its own
// running max, which lets a query stop as soon as it cannot raise it (or is
// already known to be under floor2), and the block maxima reduce in order: the
// sample achieving the max is never cut short, so pooled == serial bit for bit.
double max_dist2(const sample_set &s, const tri_bvh &g, double floor2, thread_pool *pool) {
  const std::size_t block = 2048, stride = 61;
  const std::size_t n = s.size();
  // A sparse first pass seeds every block with a lower bound on the answer.
  double seed = 0.0;
  u32 hint = kNone;
  for (std::size_t i = 0; i < n; i += stride)
    seed = std::max(seed, g.nearest2(s.at(i), std::max(seed, floor2), hint));
  std::vector<double> bmax((n + block - 1) / block, 0.0);
  pfor_blocks(pool, n, block, [&](std::size_t lo, std::size_t hi) {
    double m = seed;
    u32 last = kNone; // each block threads its own hint through its samples
    for (std::size_t i = lo; i < hi; ++i)
      m = std::max(m, g.nearest2(s.at(i), std::max(m, floor2), last));
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
double hausdorff(const sample_set &sa, const tri_bvh &ga, const sample_set &sb, const tri_bvh &gb,
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

// Union-find root (the smallest index of the set, given unite() below).
inline u32 find_root(std::vector<u32> &parent, u32 x) {
  while (parent[x] != x) {
    parent[x] = parent[parent[x]];
    x = parent[x];
  }
  return x;
}
inline void unite(std::vector<u32> &parent, u32 a, u32 b) {
  a = find_root(parent, a);
  b = find_root(parent, b);
  if (a != b)
    parent[std::max(a, b)] = std::min(a, b);
}

// Collapse classes: the input vertices a collapse moves as one.
struct weld_classes {
  std::vector<u32> of;  // input vertex -> class (kNone: used by no triangle)
  std::vector<u32> rep; // class -> representative (its smallest input vertex)
  u64 seam = 0;         // used input vertices that share their class with another
};

// Without welding every used vertex is its own class. With it, a used vertex
// joins every used vertex of a DIFFERENT connected component (triangles joined
// through shared vertex indices) within eps of it -- the seam between touching
// unwelded parts -- and every vertex at the bit-identical position -- a split
// for hard-edge normals or a uv seam -- transitively. A class sits at its
// representative's position; every other member is within eps of it.
weld_classes weld(const std::vector<Vec3> &P, const std::vector<Tri> &T, bool on,
                  double seam_epsilon, thread_pool *pool) {
  const u32 nv = u32(P.size());
  std::vector<char> used(nv, 0);
  for (const Tri &t : T)
    used[t[0]] = used[t[1]] = used[t[2]] = 1;
  std::vector<u32> cls(nv);
  std::iota(cls.begin(), cls.end(), 0u);

  std::vector<u32> verts, finite; // used vertices; those a coincidence search can place
  for (u32 v = 0; v < nv; ++v)
    if (used[v]) {
      verts.push_back(v);
      if (std::isfinite(P[v].x) && std::isfinite(P[v].y) && std::isfinite(P[v].z))
        finite.push_back(v);
    }
  if (on && finite.size() > 1) {
    // components: union-find over triangle edges
    std::vector<u32> comp(nv);
    std::iota(comp.begin(), comp.end(), 0u);
    for (const Tri &t : T) {
      unite(comp, t[0], t[1]);
      unite(comp, t[0], t[2]);
    }
    double lo[3] = {kInf, kInf, kInf}, hi[3] = {-kInf, -kInf, -kInf};
    for (u32 v : finite) {
      comp[v] = find_root(comp, v);
      for (int a = 0; a < 3; ++a) {
        lo[a] = std::min(lo[a], axis(P[v], a));
        hi[a] = std::max(hi[a], axis(P[v], a));
      }
    }
    const double diag =
        std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) + (hi[1] - lo[1]) * (hi[1] - lo[1]) +
                  (hi[2] - lo[2]) * (hi[2] - lo[2]));
    // Default tolerance: 1e-6 of the extent, but never more than 1e-3 of the
    // median edge, so a large scene cannot weld the vertices of its fine parts.
    double eps = seam_epsilon;
    if (eps < 0.0) {
      std::vector<double> e2;
      e2.reserve(T.size() * 3);
      for (const Tri &t : T)
        for (int k = 0; k < 3; ++k) {
          const double l2 = dist2(P[t[k]], P[t[(k + 1) % 3]]);
          if (std::isfinite(l2))
            e2.push_back(l2);
        }
      const std::size_t mid = e2.size() / 2;
      std::nth_element(e2.begin(), e2.begin() + mid, e2.end());
      eps = std::min(1e-6 * diag, e2.empty() ? 0.0 : 1e-3 * std::sqrt(e2[mid]));
    }
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
    std::vector<Key> keys(finite.size());
    for (std::size_t i = 0; i < finite.size(); ++i) {
      const Vec3 &p = P[finite[i]];
      keys[i] = {{cell_of(p.x, 0), cell_of(p.y, 1), cell_of(p.z, 2)}, finite[i]};
    }
    std::sort(keys.begin(), keys.end(), [&](const Key &a, const Key &b) {
      return cell_less(a, b) || (!cell_less(b, a) && a.v < b.v);
    });

    // Each block lists the pairs it finds, each once (from its larger vertex);
    // the union below does not depend on their order. Bit-identical positions
    // are an equivalence, so a vertex pairs only with the smallest of its exact
    // duplicates: a pile of them costs one pair each, not one per couple.
    const double eps2 = eps * eps;
    const std::size_t block = 1024;
    std::vector<std::vector<std::pair<u32, u32>>> found((keys.size() + block - 1) / block);
    pfor_blocks(pool, keys.size(), block, [&](std::size_t b, std::size_t e) {
      std::vector<std::pair<u32, u32>> &out = found[b / block];
      for (std::size_t i = b; i < e; ++i) {
        const u32 v = keys[i].v;
        const Vec3 &p = P[v];
        u32 same = v; // smallest exact duplicate of v
        auto test = [&](const Key &k) {
          if (k.v >= v)
            return;
          const double d2 = dist2(P[k.v], p);
          if (d2 == 0.0)
            same = std::min(same, k.v);
          else if (d2 <= eps2 && comp[k.v] != comp[v])
            out.push_back(std::make_pair(k.v, v));
        };
        // own cell: the run of equal cells around i
        for (std::size_t j = i; j-- > 0 && !cell_less(keys[j], keys[i]);)
          test(keys[j]);
        for (std::size_t j = i + 1; j < keys.size() && !cell_less(keys[i], keys[j]); ++j)
          test(keys[j]);
        // neighbour cells the eps-ball reaches into
        i64 c0[3], c1[3];
        for (int a = 0; a < 3; ++a) {
          c0[a] = cell_of(axis(p, a) - eps, a);
          c1[a] = cell_of(axis(p, a) + eps, a);
        }
        for (i64 x = c0[0]; x <= c1[0]; ++x)
          for (i64 y = c0[1]; y <= c1[1]; ++y)
            for (i64 z = c0[2]; z <= c1[2]; ++z) {
              const Key probe = {{x, y, z}, 0};
              if (!cell_less(probe, keys[i]) && !cell_less(keys[i], probe))
                continue; // own cell, done above
              auto it = std::lower_bound(keys.begin(), keys.end(), probe, cell_less);
              for (; it != keys.end() && !cell_less(probe, *it); ++it)
                test(*it);
            }
        if (same != v)
          out.push_back(std::make_pair(same, v));
      }
    });
    for (const auto &blk : found)
      for (const auto &pr : blk)
        unite(cls, pr.first, pr.second);
  }

  weld_classes wc;
  wc.of.assign(nv, kNone);
  std::vector<u32> members;
  for (u32 v : verts) {
    const u32 r = find_root(cls, v); // the class's smallest vertex: seen first
    if (r == v) {
      wc.of[v] = u32(wc.rep.size());
      wc.rep.push_back(v);
      members.push_back(0);
    } else {
      wc.of[v] = wc.of[r];
    }
    ++members[wc.of[v]];
  }
  for (u32 v : verts)
    wc.seam += members[wc.of[v]] > 1 ? 1 : 0;
  return wc;
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

  // --- gather triangles (triangulating quads) over the input vertices ---
  std::vector<Vec3> Pin;
  std::vector<Tri> W; // each triangle's input vertices ("wedges"), rewritten by collapses
  gather(mesh, Pin, W);
  const u64 n_in = W.size();
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

  // --- collapse classes: the vertices of the welded topology the loop runs on ---
  const weld_classes wc = weld(Pin, W, params.weld_seams, params.seam_epsilon, pool);
  const u32 nv = u32(wc.rep.size());
  std::vector<Vec3> P(nv);
  for (u32 v = 0; v < nv; ++v)
    P[v] = Pin[wc.rep[v]];
  std::vector<Tri> T(W.size());
  for (std::size_t ti = 0; ti < W.size(); ++ti)
    T[ti] = {wc.of[W[ti][0]], wc.of[W[ti][1]], wc.of[W[ti][2]]};
  const std::vector<Tri> T0 = out ? T : std::vector<Tri>(); // source surface, for world_error
  std::vector<u32> wcls(wc.of);              // each wedge's class; a moved wedge changes class
  std::vector<char> wremoved(Pin.size(), 0); // wedges collapsed onto a partner

  std::vector<bool> vremoved(nv, false);
  std::vector<u32> vver(nv, 0);
  std::vector<bool> tremoved(T.size(), false);
  u64 ntris = n_in;
  // A triangle with two corners in one class is (to within seam_epsilon) a
  // segment: it takes no part in the collapse and is left out of the result.
  u64 ndropped = 0;
  for (u32 ti = 0; ti < T.size(); ++ti) {
    const Tri &f = T[ti];
    if (f[0] == f[1] || f[1] == f[2] || f[0] == f[2]) {
      tremoved[ti] = true;
      --ntris;
      ++ndropped;
    }
  }

  // vertex -> incident triangle indices
  std::vector<std::vector<u32>> vtri(nv);
  for (u32 ti = 0; ti < T.size(); ++ti)
    if (!tremoved[ti])
      for (int k = 0; k < 3; ++k)
        vtri[T[ti][k]].push_back(ti);

  // --- per-vertex quadrics from incident face planes ---
  // The natural form is a scatter (each face adds to its 3 vertices), which races
  // under a pool. Split it into two data-parallel gathers via the vtri adjacency:
  // (1) each face computes its own area-weighted plane; (2) each vertex sums the
  // planes of its incident faces. In the vertex's own frame each of those planes
  // passes through the origin, so only A and w accumulate. Result is bit-identical
  // to serial.
  std::vector<Quadric> FQ(T.size());
  pfor(pool, u32(T.size()), [&](int ti) {
    if (tremoved[ti])
      return;
    const Vec3 &a = P[T[ti][0]], &b = P[T[ti][1]], &c = P[T[ti][2]];
    Vec3 n = cross(sub(b, a), sub(c, a));
    double l = len(n);
    if (l < 1e-30)
      return;
    FQ[ti].add_plane({n.x / l, n.y / l, n.z / l}, 0.5 * l); // area-weighted
  });
  std::vector<Quadric> Q(nv);
  pfor(pool, nv, [&](int v) {
    Quadric acc;
    for (u32 ti : vtri[v])
      acc.add_planes(FQ[ti]);
    Q[v] = acc;
  });

  // Every undirected edge once (sorted), and the boundary ones: an edge whose
  // triangles all share one opposite corner -- a single triangle, or coincident
  // copies of it (the two faces of a double-sided sheet).
  std::vector<u64> edges, boundary;
  {
    std::vector<std::pair<u64, u32>> eo; // (edge, opposite corner)
    eo.reserve(3 * ntris);
    for (u32 ti = 0; ti < T.size(); ++ti)
      if (!tremoved[ti])
        for (int k = 0; k < 3; ++k)
          eo.push_back(std::make_pair(edge_key(T[ti][k], T[ti][(k + 1) % 3]), T[ti][(k + 2) % 3]));
    std::sort(eo.begin(), eo.end());
    for (std::size_t i = 0; i < eo.size();) {
      std::size_t j = i + 1;
      while (j < eo.size() && eo[j].first == eo[i].first)
        ++j;
      edges.push_back(eo[i].first);
      if (eo[j - 1].second == eo[i].second) // sorted: first == last opposite => one opposite
        boundary.push_back(eo[i].first);
      i = j;
    }
  }

  // --- boundary constraint quadrics: for each boundary edge, add a plane through
  // the edge, perpendicular to the face, so the border holds. Weighted by
  // boundary_weight * |e|^2 -- the same length^2 units as the area-weighted face
  // planes -- so the relative strength (and the collapse order) is independent
  // of the mesh's scale. ---
  if (params.preserve_boundary && !boundary.empty()) {
    for (u32 ti = 0; ti < T.size(); ++ti) {
      if (tremoved[ti])
        continue;
      const Vec3 &a = P[T[ti][0]], &b = P[T[ti][1]], &c = P[T[ti][2]];
      Vec3 fn = face_normal(a, b, c);
      for (int k = 0; k < 3; ++k) {
        u32 v0 = T[ti][k], v1 = T[ti][(k + 1) % 3];
        if (!std::binary_search(boundary.begin(), boundary.end(), edge_key(v0, v1)))
          continue; // interior edge
        // plane containing edge (v0,v1) and perpendicular to the face
        Vec3 e = sub(P[v1], P[v0]);
        Vec3 pn = cross(e, fn);
        double pl = len(pn);
        if (pl < 1e-30)
          continue;
        pn = {pn.x / pl, pn.y / pl, pn.z / pl};
        const double w = params.boundary_weight * dot(e, e);
        Q[v0].add_plane(pn, w);
        Q[v1].add_plane(pn, w);
      }
    }
  }

  const double flip_cos = std::cos(params.max_normal_flip_deg * 3.14159265358979323846 / 180.0);

  // Evaluate the half-edge collapse of edge (u,v): the cheaper direction. Keeping
  // u costs u's own quadric at u (its c) plus v's quadric at u's offset from v.
  // A cost inside its round-off band -- the magnitude of the terms it sums, plus
  // the round-off already carried in both quadrics -- is exactly 0.
  auto eval = [&](u32 u, u32 v, double &cost, u32 &keep, u32 &drop) {
    const Vec3 o = sub(P[u], P[v]); // u seen from v; v seen from u is exactly -o
    const Vec3 no = {-o.x, -o.y, -o.z};
    const double carried = Q[u].m + Q[v].m;
    double eu = Q[u].c + Q[v].at(o);
    double ev = Q[v].c + Q[u].at(no);
    const double bu = kCostRoundoff * (carried + std::max(0.0, Q[u].c) + Q[v].mag(o));
    const double bv = kCostRoundoff * (carried + std::max(0.0, Q[v].c) + Q[u].mag(no));
    eu = eu <= bu ? 0.0 : quantize(eu);
    ev = ev <= bv ? 0.0 : quantize(ev);
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

  u64 ncollapse = 0;
  bool hit_limit = false;

  // --- snapshots: rebuild a compact geometry from the surviving wedges ---
  const geometry::points_t &in_pts = mesh.const_points();
  const geometry::uvs_t &in_uv = mesh.const_uvs();
  const geometry::colors_t &in_col = mesh.const_colors();
  const geometry::normals_t &in_nrm = mesh.const_normals();
  const bool has_uv = in_uv.size() == in_pts.size();
  const bool has_col = in_col.size() == in_pts.size();
  const bool has_nrm = !params.recompute_normals && in_nrm.size() == in_pts.size();

  auto build = [&]() {
    geometry result(mesh.ctx());
    const u32 nw = u32(in_pts.size());
    std::vector<u32> remap(nw, kNone);
    geometry::points_t &op = result.points();
    geometry::uvs_t *ouv = has_uv ? &result.uvs() : nullptr;
    geometry::colors_t *ocol = has_col ? &result.colors() : nullptr;
    geometry::normals_t *onrm = has_nrm ? &result.normals() : nullptr;
    for (u32 w = 0; w < nw; ++w) {
      if (wremoved[w])
        continue;
      remap[w] = u32(op.size());
      // a wedge that was moved sits exactly on its new class's input position
      op.push_back(wcls[w] == wc.of[w] ? in_pts[w] : in_pts[wc.rep[wcls[w]]]);
      if (ouv)
        ouv->push_back(in_uv[w]);
      if (ocol)
        ocol->push_back(in_col[w]);
      if (onrm)
        onrm->push_back(in_nrm[w]); // every wedge keeps its own attributes
    }
    geometry::tris_t &ot = result.tris();
    for (u32 ti = 0; ti < W.size(); ++ti) {
      if (tremoved[ti])
        continue;
      const Tri &f = W[ti];
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
  // surface, both over the class positions (exactly the input and result
  // surfaces when seams coincide exactly; within seam_epsilon otherwise). The
  // source side (tree, used vertices, unique edges) is built once and reused by
  // every snapshot; construction is deterministic, so a progressive snapshot
  // measures exactly what an independent run would. Samples the two surfaces
  // share -- every live vertex, an edge present in both, an untouched triangle's
  // centroid -- lie ON the other surface (distance 0, under the round-off
  // floor), so they are not queried: the value is bit-identical to
  // sampled_hausdorff(mesh, snapshot), for less work on fine rungs.
  std::unique_ptr<tri_bvh> src_tree;
  std::vector<u32> src_verts;
  std::vector<u64> src_edges;
  auto measure = [&]() {
    if (!src_tree) {
      src_tree.reset(new tri_bvh(P, T0));
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
    const tri_bvh tree(P, live);
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
    return hausdorff(src, *src_tree, lv, tree, pool);
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
      r.world_error = ncollapse > 0 || ndropped > 0 ? measure() : 0.0;
      r.hit_error_limit = hit_limit;
      r.seam_vertices = wc.seam;
    }
    last = k;
    last_collapses = ncollapse;
  };

  // drop's wedges paired with the keep wedge they share a vanishing triangle with
  std::vector<std::pair<u32, u32>> pairs;
  auto partner = [&](u32 w) {
    for (const auto &pr : pairs)
      if (pr.first == w)
        return pr.second;
    return kNone;
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

    // The triangles on the collapsed edge vanish. Each pairs the drop wedge at
    // its drop corner with the keep wedge at its keep corner (first pairing wins).
    pairs.clear();
    u64 vanish = 0;
    for (u32 ti : vtri[drop]) {
      if (tremoved[ti])
        continue;
      const Tri &f = T[ti];
      int kd = 0, kk = -1;
      for (int k = 0; k < 3; ++k) {
        if (f[k] == drop)
          kd = k;
        else if (f[k] == keep)
          kk = k;
      }
      if (kk < 0)
        continue;
      ++vanish;
      if (partner(W[ti][kd]) == kNone)
        pairs.push_back(std::make_pair(W[ti][kd], W[ti][kk]));
    }
    if (vanish == ntris)
      continue; // never remove the last triangle: an empty rung has no finite error

    // rewrite drop's faces onto keep; drop degenerate ones
    for (u32 ti : vtri[drop]) {
      if (tremoved[ti])
        continue;
      Tri &f = T[ti];
      if (f[0] == keep || f[1] == keep || f[2] == keep) {
        tremoved[ti] = true; // shared face collapses to a line
        --ntris;
        continue;
      }
      for (int k = 0; k < 3; ++k)
        if (f[k] == drop) {
          f[k] = keep;
          const u32 dw = W[ti][k], kw = partner(dw);
          if (kw != kNone)
            W[ti][k] = kw; // follow its partner
          else
            wcls[dw] = keep; // no partner on this face's side: move onto keep
        }
      vtri[keep].push_back(ti);
    }
    for (const auto &pr : pairs)
      wremoved[pr.first] = 1;

    Q[keep].merge(Q[drop], sub(P[keep], P[drop]));
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
  const tri_bvh ga(PA, TA), gb(PB, TB);
  return hausdorff(all_samples(PA, TA), ga, all_samples(PB, TB), gb, pool);
}

} // namespace cvc
