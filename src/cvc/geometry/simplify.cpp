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
// Collapses run on the WELDED topology (simplify_params::weld_seams). First,
// input vertices that are bit-identical in position and in every carried
// attribute (uv, color, normal) become one "wedge": a soup or a split mesh whose
// attributes are continuous is then exactly the welded mesh. Wedges that still
// share a position -- they differ in an attribute (a uv seam, a hard edge) or
// belong to parts that touch within the coincidence tolerance (a roof on its
// walls) -- form one collapse vertex, a "class". The loop runs on classes, and
// wedges never
// move: collapsing class D onto class K re-points each triangle's D corner from
// its wedge to that wedge's partner, the wedge of K it shares a vanishing
// triangle with. A collapse is allowed only when every wedge of D that a
// surviving triangle still uses has exactly one partner, so a seam collapses
// only along itself, each side onto its own side: it stays closed, and no uv,
// color or normal is ever attached to a position other than its own. A
// triangle side whose wedges meet no other triangle -- an open border, or one
// side of a seam -- adds a perpendicular constraint plane, so borders and seams
// keep their line. The result holds only the wedges its triangles use.
//
// A per-class 4x4 quadric accumulates the area-weighted squared distance to the
// planes of every incident face (Garland & Heckbert '97); constraint planes are
// weighted by |edge|^2, so every term shares the same units (length^2 weight x
// length^2 distance), which keeps the collapse order invariant under a uniform
// scale. Each quadric lives in a frame centred on its own vertex, where every
// incident plane passes through the origin exactly; absorbing a neighbour
// translates that neighbour's quadric by the edge vector (an exact difference of
// nearby coordinates). The round-off of a cost is thus relative to local lengths
// -- never to the mesh's extent or to its distance from the world origin -- and
// is tracked: a cost inside its round-off band is snapped to exactly 0 and the
// rest are compared at 24-bit precision, so geometric ties stay ties when the
// mesh is scaled or moved, and geometry elsewhere cannot perturb a local
// collapse. A lazy binary heap keyed on a per-vertex version pops the cheapest
// valid collapse, with ties broken by vertex index so the pop sequence never
// depends on the standard library's heap. Fold-overs are rejected by a per-face
// normal-flip test so the coarse rung never self-intersects, and a collapse that
// breaks the link condition (it would fold two faces onto one another, or put a
// third triangle on an edge) is rejected too.
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
#include <unordered_set>
#include <utility>
#include <vector>

namespace cvc {
namespace {

typedef std::uint32_t u32;
typedef std::uint64_t u64;
typedef std::array<u32, 3> Tri;

const double kInf = std::numeric_limits<double>::infinity();
const u32 kNone = u32(-1);
const u32 kMixed = u32(-2); // a position whose wedges belong to more than one component

// A collapse cost within this fraction of the magnitude of the terms it was
// summed from (Quadric::mag, Quadric::m) is round-off, not geometry. 2^-40 is
// ~4096 ulps: well above what an evaluation, a merge or a long plane sum can
// accumulate, and far below any real cost (it is an RMS plane distance of ~1e-6
// of the lengths involved).
const double kCostRoundoff = 1.0 / 1099511627776.0;

// A distance within this fraction of the largest coordinate magnitude is
// round-off: a sample point (an edge midpoint or a centroid) is rounded to within
// ~2 ulps of its coordinates, and a point-to-triangle distance adds a few ulps
// of the local lengths. 2^-48 is 16 ulps.
const double kCoordRoundoff = 1.0 / 281474976710656.0;

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
inline bool is_finite(const Vec3 &a) {
  return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}
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
// A NaN (an overflowed quadric) costs +infinity, so it pops last, never first.
inline double quantize(double c) {
  if (!(c > 0.0))
    return c == c ? 0.0 : kInf;
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

// --- median-split trees ------------------------------------------------------
//
// Both spatial trees below (the Hausdorff BVH over triangles, and the weld's
// k-d tree over positions) split their items at the median along the longest
// axis of the items' box, down to at most `leaf` items per leaf, and store their
// nodes in preorder: a node's left child follows it, and its right child starts
// after the left subtree, whose size is a function of its item count alone. Ties
// in the split order break by index, so a tree is a pure function of its items
// whether or not a pool builds its lower subtrees.

// A point (a triangle's centroid, or a position) and its index.
struct split_item {
  double c[3];
  u32 id;
};

// Leaves (and nodes) of the subtree over m items.
u32 subtree_leaves(u32 m, u32 leaf) {
  return m <= leaf ? 1 : subtree_leaves(m / 2, leaf) + subtree_leaves(m - m / 2, leaf);
}
inline u32 subtree_nodes(u32 m, u32 leaf) { return 2 * subtree_leaves(m, leaf) - 1; }

// Reorder items[b, e) about its median along the longest axis of their box;
// returns the split point.
u32 median_split(std::vector<split_item> &items, u32 b, u32 e) {
  double lo[3] = {kInf, kInf, kInf}, hi[3] = {-kInf, -kInf, -kInf};
  for (u32 i = b; i < e; ++i)
    for (int a = 0; a < 3; ++a) {
      lo[a] = std::min(lo[a], items[i].c[a]);
      hi[a] = std::max(hi[a], items[i].c[a]);
    }
  int ax = 0;
  for (int a = 1; a < 3; ++a)
    if (hi[a] - lo[a] > hi[ax] - lo[ax])
      ax = a;
  const u32 mid = b + (e - b) / 2;
  std::nth_element(items.begin() + b, items.begin() + mid, items.begin() + e,
                   [ax](const split_item &x, const split_item &y) {
                     return x.c[ax] < y.c[ax] || (x.c[ax] == y.c[ax] && x.id < y.id);
                   });
  return mid;
}

// Build the median-split tree over `items` (non-empty): make_leaf(node, parent,
// b, e) for a leaf over items[b, e); inner(node, parent, b, e, right) for an
// interior node, before its subtrees; join(node) for an interior node, after
// both of them. The top levels split serially, down to subtrees small enough to
// fan out over the pool; each of those writes only its own nodes and items.
template <class Leaf, class Inner, class Join>
void build_median_tree(std::vector<split_item> &items, u32 leaf, thread_pool *pool,
                       const Leaf &make_leaf, const Inner &inner, const Join &join) {
  struct Subtree {
    std::vector<split_item> &items;
    u32 leaf;
    const Leaf &make_leaf;
    const Inner &inner;
    const Join &join;
    void operator()(u32 node, u32 parent, u32 b, u32 e) const {
      if (e - b <= leaf) {
        make_leaf(node, parent, b, e);
        return;
      }
      const u32 mid = median_split(items, b, e);
      const u32 right = node + 1 + subtree_nodes(mid - b, leaf);
      inner(node, parent, b, e, right);
      (*this)(node + 1, node, b, mid);
      (*this)(right, node, mid, e);
      join(node);
    }
  };
  const Subtree subtree{items, leaf, make_leaf, inner, join};
  const u32 n = u32(items.size());
  struct Job {
    u32 node, parent, b, e;
  };
  std::vector<Job> stack(1, Job{0, kNone, 0, n}), frontier;
  std::vector<u32> top;
  const u32 grain = std::max(u32(4096), n / 64);
  while (!stack.empty()) {
    const Job j = stack.back();
    stack.pop_back();
    if (j.e - j.b <= grain) {
      frontier.push_back(j);
      continue;
    }
    const u32 mid = median_split(items, j.b, j.e);
    const u32 right = j.node + 1 + subtree_nodes(mid - j.b, leaf);
    inner(j.node, j.parent, j.b, j.e, right);
    top.push_back(j.node);
    stack.push_back(Job{right, j.node, mid, j.e});
    stack.push_back(Job{j.node + 1, j.node, j.b, mid});
  }
  pfor(pool, u32(frontier.size()), [&](int i) {
    const Job &j = frontier[i];
    subtree(j.node, j.parent, j.b, j.e);
  });
  for (auto it = top.rbegin(); it != top.rend(); ++it) // children first
    join(*it);
}

// Squared distance from p to the box [lo, hi] (0 inside).
inline double point_box_dist2(const double lo[3], const double hi[3], const Vec3 &p) {
  double d2 = 0.0;
  for (int a = 0; a < 3; ++a) {
    const double x = axis(p, a);
    const double d = x < lo[a] ? lo[a] - x : (x > hi[a] ? x - hi[a] : 0.0);
    d2 += d * d;
  }
  return d2;
}

// Bounding-volume hierarchy over a triangle set, for nearest-surface distance
// queries: a median-split tree over the triangle centroids, a few triangles per
// leaf (their corners copied into leaf order). Unlike a uniform grid it adapts
// to wildly mixed triangle sizes (a ground quad kilometres wide under
// centimetre detail), so a query stays logarithmic wherever it lands. The tree
// -- and so every query -- is a pure function of the input, whether or not
// `pool` builds the lower subtrees in parallel.
class tri_bvh {
public:
  tri_bvh(const std::vector<Vec3> &P, const std::vector<Tri> &T, thread_pool *pool) {
    const u32 n = u32(T.size());
    _ntri = n;
    if (n == 0)
      return;
    for (const Tri &t : T)
      if (!is_finite(P[t[0]]) || !is_finite(P[t[1]]) || !is_finite(P[t[2]])) {
        _finite = false; // not measurable: see hausdorff()
        return;
      }
    // Each triangle's centroid travels with its index, so the splits read
    // memory in order.
    std::vector<split_item> items(n);
    for (u32 i = 0; i < n; ++i) {
      const Vec3 &a = P[T[i][0]], &b = P[T[i][1]], &c = P[T[i][2]];
      items[i] = split_item{
          {(a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0, (a.z + b.z + c.z) / 3.0}, i};
    }
    _nodes.resize(subtree_nodes(n, kLeaf));
    _parent.resize(_nodes.size());
    _tri.resize(n);
    build_median_tree(
        items, kLeaf, pool,
        [&](u32 node, u32 parent, u32 b, u32 e) { // a leaf: its triangles' corners, and its box
          _parent[node] = parent;
          Node &nd = _nodes[node];
          nd.first = b;
          nd.count = e - b;
          for (u32 i = b; i < e; ++i) {
            const Tri &t = T[items[i].id];
            _tri[i] = {P[t[0]], P[t[1]], P[t[2]]};
            for (int k = 0; k < 3; ++k)
              for (int a = 0; a < 3; ++a) {
                nd.lo[a] = std::min(nd.lo[a], axis(_tri[i][k], a));
                nd.hi[a] = std::max(nd.hi[a], axis(_tri[i][k], a));
              }
          }
        },
        [&](u32 node, u32 parent, u32, u32, u32 right) {
          _parent[node] = parent;
          _nodes[node].first = right;
          _nodes[node].count = 0;
        },
        [&](u32 node) { join(node); });
    const Node &r = _nodes[0];
    const double ext[3] = {r.hi[0] - r.lo[0], r.hi[1] - r.lo[1], r.hi[2] - r.lo[2]};
    _diag = std::sqrt(ext[0] * ext[0] + ext[1] * ext[1] + ext[2] * ext[2]);
    for (int a = 0; a < 3; ++a)
      _mag = std::max(_mag, std::max(std::fabs(r.lo[a]), std::fabs(r.hi[a])));
    // An extent whose square overflows cannot hold a squared distance either.
    _finite = std::isfinite(_diag);
  }

  bool empty() const { return _ntri == 0; }
  // Every corner finite, and the extent small enough for squared distances.
  bool finite() const { return _finite; }
  double diagonal() const { return _diag; }    // of the triangles' bounding box
  double magnitude() const { return _mag; }    // largest |coordinate| of any corner
  void box(double lo[3], double hi[3]) const { // the triangles' bounding box (0s when empty)
    for (int a = 0; a < 3; ++a) {
      lo[a] = _nodes.empty() ? 0.0 : _nodes[0].lo[a];
      hi[a] = _nodes.empty() ? 0.0 : _nodes[0].hi[a];
    }
  }

  // Squared distance from p to the nearest triangle, visiting the nearer child
  // first and skipping any box no closer than the best so far. Returns early --
  // with some value <= stop2, never below the true distance -- as soon as the
  // distance is known not to exceed stop2 (the caller only wants the largest).
  // `hint` names a leaf to try first (kNone for none) and receives the leaf of
  // the nearest triangle found: consecutive samples are usually neighbours, so
  // a query that only has to prove "within stop2" often ends in that leaf, or
  // in the small subtree a few levels above it, without a descent from the
  // root. The hint never changes an exact distance returned above stop2: it
  // only seeds the search with real triangle distances.
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
      u32 near = hint;
      for (int k = 0; k < kClimb && _parent[near] != kNone; ++k)
        near = _parent[near];
      if (search(near, p, stop2, best, hint))
        return best;
    }
    search(0, p, stop2, best, hint);
    return best;
  }

private:
  static constexpr u32 kLeaf = 4;  // triangles per leaf
  static constexpr int kClimb = 5; // levels above the hint leaf searched before the root
  struct Node {
    double lo[3] = {kInf, kInf, kInf}; // bounding box of the node's triangles
    double hi[3] = {-kInf, -kInf, -kInf};
    u32 first = 0; // leaf: _tri[first .. first+count); interior: right child (left: next node)
    u32 count = 0;
  };

  // An interior node's box: the union of its children's.
  void join(u32 node) {
    Node &nd = _nodes[node];
    const Node &l = _nodes[node + 1], &r = _nodes[nd.first];
    for (int a = 0; a < 3; ++a) {
      nd.lo[a] = std::min(l.lo[a], r.lo[a]);
      nd.hi[a] = std::max(l.hi[a], r.hi[a]);
    }
  }

  static double box_dist2(const Node &nd, const Vec3 &p) {
    return point_box_dist2(nd.lo, nd.hi, p);
  }

  // Best-first search of the subtree at `root`, lowering best (and moving the
  // hint to the leaf that holds it); true as soon as best <= stop2.
  bool search(u32 root, const Vec3 &p, double stop2, double &best, u32 &hint) const {
    struct Entry {
      double d2;
      u32 node;
    };
    // Each level pushes at most one entry more than it pops, and the median
    // split halves every node, so the depth (< 32 levels) bounds the stack.
    Entry stack[96];
    int sp = 0;
    stack[sp++] = Entry{box_dist2(_nodes[root], p), root};
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
              return true;
          }
        }
        continue;
      }
      const u32 left = e.node + 1, right = nd.first;
      const double dl = box_dist2(_nodes[left], p), dr = box_dist2(_nodes[right], p);
      const bool left_first = !(dr < dl);
      const Entry nearer = left_first ? Entry{dl, left} : Entry{dr, right};
      const Entry farther = left_first ? Entry{dr, right} : Entry{dl, left};
      if (farther.d2 < best)
        stack[sp++] = farther;
      if (nearer.d2 < best)
        stack[sp++] = nearer; // popped first
    }
    return false;
  }

  std::vector<Node> _nodes;
  std::vector<u32> _parent;              // kNone for the root
  std::vector<std::array<Vec3, 3>> _tri; // triangle corners in leaf order
  u32 _ntri = 0;
  bool _finite = true;
  double _diag = 0.0, _mag = 0.0;
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

// The 10 bits of x spread to every third bit (bit k -> bit 3k).
inline u32 spread10(u32 x) {
  x &= 0x3ffu;
  x = (x | (x << 16)) & 0x030000ffu;
  x = (x | (x << 8)) & 0x0300f00fu;
  x = (x | (x << 4)) & 0x030c30c3u;
  x = (x | (x << 2)) & 0x09249249u;
  return x;
}

// The samples' indices in Morton (Z-curve) order of their positions, on a
// 1024^3 lattice over the tree's box -- one scale on every axis, so a thin
// surface keeps its lateral order -- with ties in index order: consecutive
// samples are then neighbours whatever order the mesh lists its vertices and
// triangles in, and each query's hint lands. An LSD radix sort on the 30 code
// bits, stable, so the order is a pure function of the samples.
std::vector<u64> morton_order(const sample_set &s, const tri_bvh &g, thread_pool *pool) {
  const std::size_t n = s.size();
  std::vector<u64> key(n); // code << 32 | index
  double lo[3], hi[3], ext = 0.0;
  g.box(lo, hi);
  for (int a = 0; a < 3; ++a)
    ext = std::max(ext, hi[a] - lo[a]);
  const double scale = ext > 0.0 ? 1023.0 / ext : 0.0;
  pfor_blocks(pool, n, 1 << 14, [&](std::size_t b, std::size_t e) {
    for (std::size_t i = b; i < e; ++i) {
      const Vec3 p = s.at(i);
      u32 code = 0;
      for (int a = 0; a < 3; ++a) {
        const double t = std::min(1023.0, std::max(0.0, (axis(p, a) - lo[a]) * scale));
        code |= spread10(u32(t)) << a;
      }
      key[i] = (u64(code) << 32) | u64(i);
    }
  });
  std::vector<u64> tmp(n);
  std::vector<std::size_t> start(1025);
  for (int shift = 32; shift < 62; shift += 10) {
    std::fill(start.begin(), start.end(), 0);
    for (u64 k : key)
      ++start[((k >> shift) & 1023u) + 1];
    for (int d = 0; d < 1024; ++d)
      start[d + 1] += start[d];
    for (u64 k : key)
      tmp[start[(k >> shift) & 1023u]++] = k;
    key.swap(tmp);
  }
  return key;
}

// max over the samples of the squared distance to the tree's surface, exact
// whenever it exceeds floor2. The samples are visited in Morton order, in
// fixed-size blocks that fan over the pool; each block keeps its own running
// max, which lets a query stop as soon as it cannot raise it (or is already
// known to be under floor2), and the block maxima reduce in order: the sample
// achieving the max is never cut short, so pooled == serial bit for bit.
double max_dist2(const sample_set &s, const tri_bvh &g, double floor2, thread_pool *pool) {
  const std::size_t block = 2048, stride = 61;
  const std::size_t n = s.size();
  const std::vector<u64> order = morton_order(s, g, pool);
  auto sample = [&](std::size_t j) { return s.at(std::size_t(order[j] & 0xffffffffu)); };
  // A sparse first pass seeds every block with a lower bound on the answer.
  double seed = 0.0;
  u32 hint = kNone;
  for (std::size_t j = 0; j < n; j += stride)
    seed = std::max(seed, g.nearest2(sample(j), std::max(seed, floor2), hint));
  std::vector<double> bmax((n + block - 1) / block, 0.0);
  pfor_blocks(pool, n, block, [&](std::size_t lo, std::size_t hi) {
    double m = seed;
    u32 last = kNone; // each block threads its own hint through its samples
    for (std::size_t j = lo; j < hi; ++j)
      m = std::max(m, g.nearest2(sample(j), std::max(m, floor2), last));
    bmax[lo / block] = m;
  });
  double m = seed;
  for (double b : bmax)
    m = std::max(m, b);
  return m;
}

// Distances under the round-off floor -- 1e-9 of the larger surface's extent,
// or kCoordRoundoff of its largest coordinate magnitude, whichever is larger (a
// sample on a coplanar triangle lands a few ulps off it) -- count as zero, so a
// result that lost no geometry reports exactly 0 wherever it sits, and every
// sample that lands on the other surface stops at the first triangle that
// proves it. A surface the arithmetic cannot measure (a non-finite corner, or
// an extent whose square overflows) is +infinity away.
double hausdorff(const sample_set &sa, const tri_bvh &ga, const sample_set &sb, const tri_bvh &gb,
                 thread_pool *pool) {
  if (ga.empty() && gb.empty())
    return 0.0;
  if (ga.empty() || gb.empty() || !ga.finite() || !gb.finite())
    return kInf;
  const double floor = std::max(1e-9 * std::max(ga.diagonal(), gb.diagonal()),
                                kCoordRoundoff * std::max(ga.magnitude(), gb.magnitude()));
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

// Can hausdorff() measure a surface over these vertices? Every used coordinate
// finite, and an extent whose square does not overflow (the same test tri_bvh
// applies).
bool measurable(const std::vector<Vec3> &P, const std::vector<Tri> &T) {
  double lo[3] = {kInf, kInf, kInf}, hi[3] = {-kInf, -kInf, -kInf};
  for (const Tri &t : T)
    for (int k = 0; k < 3; ++k) {
      const Vec3 &p = P[t[k]];
      if (!is_finite(p))
        return false;
      for (int a = 0; a < 3; ++a) {
        lo[a] = std::min(lo[a], axis(p, a));
        hi[a] = std::max(hi[a], axis(p, a));
      }
    }
  double s = 0.0;
  for (int a = 0; a < 3; ++a)
    s += T.empty() ? 0.0 : (hi[a] - lo[a]) * (hi[a] - lo[a]);
  return std::isfinite(std::sqrt(s));
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

// The per-vertex attributes a result carries (null: not carried), compared bit
// for bit.
struct carried {
  const geometry::uvs_t *uv;
  const geometry::colors_t *col;
  const geometry::normals_t *nrm;
  int compare(u32 a, u32 b) const {
    int c = 0;
    if (uv && (c = std::memcmp(&(*uv)[a], &(*uv)[b], sizeof((*uv)[a]))) != 0)
      return c;
    if (col && (c = std::memcmp(&(*col)[a], &(*col)[b], sizeof((*col)[a]))) != 0)
      return c;
    if (nrm && (c = std::memcmp(&(*nrm)[a], &(*nrm)[b], sizeof((*nrm)[a]))) != 0)
      return c;
    return 0;
  }
};

// The welded topology.
struct weld_classes {
  // input vertex -> its wedge: the smallest input vertex bit-identical to it in
  // position and every carried attribute (kNone: used by no triangle)
  std::vector<u32> wedge;
  // input vertex -> its position: one id per bit-identical position (-0 == +0)
  // under welding, the vertex itself without (kNone: used by no triangle)
  std::vector<u32> pos;
  std::vector<u32> of;     // input vertex -> class (kNone: used by no triangle)
  std::vector<u32> rep;    // class -> representative (its smallest input vertex)
  std::vector<char> multi; // class -> holds more than one wedge (a seam runs through it)
  u64 seam = 0;            // used input vertices that share their class with another
  u64 pairs_held = 0;      // the most pairs a block of the near search held at once
};

// Keep only a spanning forest of `pr`: the pairs that join two sets the pairs
// before them have not joined yet. Their union is unchanged.
void spanning_pairs(std::vector<std::pair<u32, u32>> &pr) {
  std::vector<u32> ids;
  ids.reserve(2 * pr.size());
  for (const auto &p : pr) {
    ids.push_back(p.first);
    ids.push_back(p.second);
  }
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  auto at = [&](u32 x) { return u32(std::lower_bound(ids.begin(), ids.end(), x) - ids.begin()); };
  std::vector<u32> up(ids.size());
  std::iota(up.begin(), up.end(), 0u);
  std::size_t k = 0;
  for (const auto &p : pr) {
    const u32 a = find_root(up, at(p.first)), b = find_root(up, at(p.second));
    if (a != b) {
      up[std::max(a, b)] = std::min(a, b);
      pr[k++] = p;
    }
  }
  pr.resize(k);
}

// The near-coincidence search. Two distinct positions g, h pair when they lie
// within min(tol[g], tol[h]) of each other (a tolerance of 0 takes no part) and
// belong to different components, or either one to several (kMixed); every pair
// is united in `cls` (over input vertices; gmin names each position's vertex).
//
// A median-split k-d tree over the positions answers each position's query of
// its own radius. A node records the largest tolerance under it and, when all
// its positions belong to one component, that component, so a query skips any
// subtree out of every tolerance's reach, and any subtree of its own component:
// a finely tessellated part costs its own vertices nothing, and the work per
// position is bounded by the positions of other parts within its reach. A node
// whose box fits within the smallest tolerance under it and that holds more
// than one component (or a kMixed position) is a clique -- every two of its
// positions pair, directly or through a third of another component -- so its
// positions are chained together once, and a query that reaches it looks for
// one pair into it, not one per position; and the members of a clique, which
// sit side by side in the tree's order, pair with each clique or position only
// once between them. So a pile of near-coincident parts costs O(n log n), not
// O(n^2) -- unless parts of a much finer scale are mixed into it, whose small
// tolerance keeps its nodes from being cliques; then its pairs are found one
// by one. A pair is sought from its later position in the tree's order only.
// The queries run in fixed blocks over the pool; each block keeps only a
// spanning forest of the pairs it finds -- fewer pairs than the positions they
// touch -- so memory does not grow with the number of pairs, and the forests
// are united in block order. The union is a function of the positions alone.
// Returns the most pairs any block held at once.
std::size_t weld_near(const std::vector<Vec3> &P, const std::vector<u32> &gmin,
                      const std::vector<u32> &gcomp, const std::vector<double> &tol,
                      std::vector<u32> &cls, thread_pool *pool) {
  std::vector<split_item> items;
  for (u32 g = 0; g < u32(gmin.size()); ++g)
    if (tol[g] > 0.0) {
      const Vec3 &p = P[gmin[g]];
      items.push_back(split_item{{p.x, p.y, p.z}, g});
    }
  const u32 n = u32(items.size());
  if (n < 2)
    return 0;
  const u32 kLeaf = 8;
  const u32 kVarious = u32(-3); // a node holding more than one component
  struct Rec {                  // a position, in tree order
    Vec3 p;
    double tol;
    u32 g, comp;
  };
  struct Node {
    double lo[3] = {kInf, kInf, kInf}; // the box of its positions
    double hi[3] = {-kInf, -kInf, -kInf};
    double maxtol = 0.0, mintol = kInf; // over its positions
    u32 b = 0, e = 0;                   // its positions: rec[b, e)
    u32 right = 0;                      // interior: the right child (the left one follows); leaf: 0
    u32 comp = kNone;                   // the one component of all its positions, or kVarious
    bool clique = false;
  };
  std::vector<Rec> rec(n);
  std::vector<Node> nodes(subtree_nodes(n, kLeaf));
  // Its box within its smallest tolerance -- by a margin far above round-off,
  // so every pair in it passes the exact test below too -- and more than one
  // component in it.
  auto set_clique = [&](Node &nd) {
    double d2 = 0.0;
    for (int a = 0; a < 3; ++a)
      d2 += (nd.hi[a] - nd.lo[a]) * (nd.hi[a] - nd.lo[a]);
    nd.clique = nd.e - nd.b > 1 && (nd.comp == kVarious || nd.comp == kMixed) &&
                d2 <= (1.0 - 1e-9) * nd.mintol * nd.mintol;
  };
  build_median_tree(
      items, kLeaf, pool,
      [&](u32 node, u32, u32 b, u32 e) {
        Node &nd = nodes[node];
        nd.b = b;
        nd.e = e;
        nd.comp = gcomp[items[b].id];
        for (u32 i = b; i < e; ++i) {
          const u32 g = items[i].id;
          rec[i] = Rec{P[gmin[g]], tol[g], g, gcomp[g]};
          for (int a = 0; a < 3; ++a) {
            nd.lo[a] = std::min(nd.lo[a], axis(rec[i].p, a));
            nd.hi[a] = std::max(nd.hi[a], axis(rec[i].p, a));
          }
          nd.maxtol = std::max(nd.maxtol, tol[g]);
          nd.mintol = std::min(nd.mintol, tol[g]);
          if (gcomp[g] != nd.comp)
            nd.comp = kVarious;
        }
        set_clique(nd);
      },
      [&](u32 node, u32, u32 b, u32 e, u32 right) {
        nodes[node].b = b;
        nodes[node].e = e;
        nodes[node].right = right;
      },
      [&](u32 node) {
        Node &nd = nodes[node];
        const Node &l = nodes[node + 1], &r = nodes[nd.right];
        for (int a = 0; a < 3; ++a) {
          nd.lo[a] = std::min(l.lo[a], r.lo[a]);
          nd.hi[a] = std::max(l.hi[a], r.hi[a]);
        }
        nd.maxtol = std::max(l.maxtol, r.maxtol);
        nd.mintol = std::min(l.mintol, r.mintol);
        nd.comp = l.comp == r.comp ? l.comp : kVarious;
        set_clique(nd);
      });

  // Chain the positions of every top clique (one with no clique above it), and
  // note each position's.
  std::vector<u32> top(n, kNone), walk(1, 0u);
  while (!walk.empty()) {
    const u32 node = walk.back();
    walk.pop_back();
    const Node &nd = nodes[node];
    if (nd.clique) {
      top[nd.b] = node;
      for (u32 i = nd.b + 1; i < nd.e; ++i) {
        top[i] = node;
        unite(cls, gmin[rec[i - 1].g], gmin[rec[i].g]);
      }
    } else if (nd.right != 0) {
      walk.push_back(nd.right);
      walk.push_back(node + 1);
    }
  }

  const std::size_t block = 1024;
  std::vector<std::vector<std::pair<u32, u32>>> found((n + block - 1) / block);
  std::vector<std::size_t> held(found.size(), 0); // per block: the most pairs held at once
  pfor_blocks(pool, n, block, [&](std::size_t b, std::size_t e) {
    std::vector<std::pair<u32, u32>> &out = found[b / block];
    std::size_t limit = 4 * block;
    // The members of a clique are one set and sit side by side in the tree's
    // order, so what one of them has paired with -- a clique (by node) or a
    // position (by slot, offset past the nodes) -- the next need not pair with.
    u32 clique = kNone;
    std::unordered_set<u64> linked;
    for (u32 s = u32(b); s < u32(e); ++s) {
      const Rec &me = rec[s];
      const bool mixed = me.comp == kMixed;
      if (top[s] != clique) {
        clique = top[s];
        linked.clear();
      }
      // false if my clique has already paired with `key`; else note it
      auto fresh = [&](u64 key) { return clique == kNone || linked.insert(key).second; };
      auto pairs_with = [&](u32 j) {
        const Rec &o = rec[j];
        if (o.comp == me.comp && !mixed)
          return false; // one component: never welded to itself
        const double r = std::min(me.tol, o.tol);
        return dist2(me.p, o.p) <= r * r;
      };
      // The median split halves every node, so the depth (< 32 levels) bounds
      // the stack: each level pushes at most one entry more than it pops.
      u32 stack[64];
      int sp = 0;
      stack[sp++] = 0;
      while (sp > 0) {
        const u32 node = stack[--sp];
        const Node &nd = nodes[node];
        if (nd.b >= s)
          continue; // its pairs with me are sought from their other end
        const double r = std::min(me.tol, nd.maxtol);
        if (!(point_box_dist2(nd.lo, nd.hi, me.p) <= r * r))
          continue; // out of reach
        if (nd.comp == me.comp && !mixed)
          continue; // my own component only
        if (nd.clique) {
          if (s < nd.e || (clique != kNone && linked.count(node)))
            continue; // my own clique, whose chain holds me, or one mine has joined
          for (u32 j = nd.b; j < nd.e; ++j)
            if (pairs_with(j)) {
              if (fresh(node))
                out.push_back(std::make_pair(rec[j].g, me.g)); // one pair joins all of it
              break;
            }
          continue;
        }
        if (nd.right == 0) {
          for (u32 j = nd.b; j < std::min(nd.e, s); ++j)
            if (pairs_with(j) && fresh(u64(nodes.size()) + j))
              out.push_back(std::make_pair(rec[j].g, me.g));
          continue;
        }
        stack[sp++] = nd.right;
        stack[sp++] = node + 1;
      }
      held[b / block] = std::max(held[b / block], out.size());
      if (out.size() > limit) {
        spanning_pairs(out);
        limit = std::max(limit, 2 * out.size());
      }
    }
    spanning_pairs(out);
  });
  for (const auto &blk : found)
    for (const auto &pr : blk)
      unite(cls, gmin[pr.first], gmin[pr.second]);
  return *std::max_element(held.begin(), held.end());
}

// Without welding every used vertex is its own wedge, position and class. With
// it, used vertices bit-identical in position and in every carried attribute
// are one wedge; wedges at one position (-0 == +0) are one class, and so are,
// transitively, positions of DIFFERENT connected components (triangles joined
// through shared wedges) within the coincidence tolerance of each other -- the
// seam between touching unwelded parts. A class sits at its representative's
// position; every other member is within tolerance of a member.
//
// The tolerance is seam_epsilon when given. By default it is 1e-6 of the
// bounding-box diagonal, but at each position no more than 1e-3 of its median
// incident edge (over the two sides of every triangle corner there, zero
// lengths left out; the upper median of an even count), and two positions pair
// only within both their tolerances: a fine part in a large scene keeps its
// own scale, so separate parts a few of their own edge lengths apart never
// weld into one another, and a part's own triangles weld away through its
// neighbours only where they are slivers, thinner than 1e-3 of the edges
// around them. (The median, not the shortest edge: a sliver at a join vertex
// does not shrink its tolerance and leave the join open.) Nor is it below 4
// float32 epsilons of the position's largest coordinate magnitude (still
// within the 1e-6 cap): coordinates stored as floats far from the origin are
// rounded by about that much, and the two sides of a join rounded apart must
// still weld -- so there, parts closer than that weld too.
//
// Exact coincidence is found by sorting the position bits, O(n log n) however
// many vertices pile onto one point; the near-coincidence search (weld_near)
// runs over the distinct positions only.
weld_classes weld(const std::vector<Vec3> &P, const std::vector<Tri> &T, const carried &attr,
                  bool on, double seam_epsilon, thread_pool *pool) {
  const u32 nv = u32(P.size());
  std::vector<char> used(nv, 0);
  for (const Tri &t : T)
    used[t[0]] = used[t[1]] = used[t[2]] = 1;
  std::vector<u32> verts; // used vertices, ascending
  for (u32 v = 0; v < nv; ++v)
    if (used[v])
      verts.push_back(v);

  weld_classes wc;
  wc.wedge.assign(nv, kNone);
  wc.pos.assign(nv, kNone);
  for (u32 v : verts)
    wc.wedge[v] = wc.pos[v] = v;
  std::vector<u32> cls(nv); // union-find over input vertices
  std::iota(cls.begin(), cls.end(), 0u);

  if (on && verts.size() > 1) {
    // --- exact coincidence: sort by (position bits, attribute bits, index) ---
    typedef std::array<u64, 3> Bits;
    std::vector<Bits> pb(nv);
    for (u32 v : verts)
      for (int a = 0; a < 3; ++a) {
        const double x = axis(P[v], a) + 0.0; // -0 -> +0
        std::memcpy(&pb[v][a], &x, sizeof(x));
      }
    std::vector<u32> order(verts);
    std::sort(order.begin(), order.end(), [&](u32 a, u32 b) {
      if (pb[a] != pb[b])
        return pb[a] < pb[b];
      const int c = attr.compare(a, b);
      return c != 0 ? c < 0 : a < b;
    });
    std::vector<u32> gmin; // distinct position -> its smallest input vertex
    for (std::size_t i = 0; i < order.size();) {
      const u32 g = u32(gmin.size());
      u32 lo = order[i];
      std::size_t j = i;
      while (j < order.size() && pb[order[j]] == pb[order[i]]) {
        std::size_t k = j; // a run of equal attributes; its first vertex is the smallest
        for (; k < order.size() && pb[order[k]] == pb[order[j]] &&
               attr.compare(order[k], order[j]) == 0;
             ++k) {
          wc.wedge[order[k]] = order[j];
          wc.pos[order[k]] = g;
        }
        lo = std::min(lo, order[j]);
        j = k;
      }
      gmin.push_back(lo);
      i = j;
    }
    for (u32 v : verts)
      unite(cls, v, gmin[wc.pos[v]]);

    // --- near coincidence across components, over the distinct positions ---
    std::vector<u32> comp(nv);
    std::iota(comp.begin(), comp.end(), 0u);
    for (const Tri &t : T) {
      unite(comp, wc.wedge[t[0]], wc.wedge[t[1]]);
      unite(comp, wc.wedge[t[0]], wc.wedge[t[2]]);
    }
    const u32 ng = u32(gmin.size());
    std::vector<u32> gcomp(ng, kNone); // a position's component, or kMixed
    for (u32 v : verts) {
      const u32 c = find_root(comp, wc.wedge[v]), g = wc.pos[v];
      gcomp[g] = gcomp[g] == kNone || gcomp[g] == c ? c : kMixed;
    }
    double lo[3] = {kInf, kInf, kInf}, hi[3] = {-kInf, -kInf, -kInf};
    for (u32 v : verts)
      for (int a = 0; a < 3; ++a) {
        lo[a] = std::min(lo[a], axis(P[v], a));
        hi[a] = std::max(hi[a], axis(P[v], a));
      }
    const double diag =
        std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) + (hi[1] - lo[1]) * (hi[1] - lo[1]) +
                  (hi[2] - lo[2]) * (hi[2] - lo[2]));
    const double eps = seam_epsilon >= 0.0 ? seam_epsilon : 1e-6 * diag;
    if (eps > 0.0 && ng > 1) {
      std::vector<double> tol(ng, eps);
      if (seam_epsilon < 0.0) {
        // The default: capped at 1e-3 of each position's median non-zero
        // incident side (every triangle corner there brings its two), but not
        // below 4 float32 epsilons of its largest coordinate magnitude; 0 --
        // no near welding -- for a position with no such side.
        auto each_side = [&](auto &&visit) { // (position, squared length)
          for (const Tri &t : T) {
            const double l2[3] = {dist2(P[t[0]], P[t[1]]), dist2(P[t[1]], P[t[2]]),
                                  dist2(P[t[2]], P[t[0]])};
            for (int k = 0; k < 3; ++k) // corner k: sides k and k+2
              for (int s : {k, (k + 2) % 3})
                if (l2[s] > 0.0)
                  visit(wc.pos[t[k]], l2[s]);
          }
        };
        std::vector<std::size_t> at(ng + 1, 0); // position g's sides: side[at[g], at[g+1])
        each_side([&](u32 g, double) { ++at[g + 1]; });
        std::partial_sum(at.begin(), at.end(), at.begin());
        std::vector<double> side(at[ng]);
        std::vector<std::size_t> fill(at.begin(), at.end() - 1);
        each_side([&](u32 g, double l2) { side[fill[g]++] = l2; });
        for (u32 g = 0; g < ng; ++g) {
          if (at[g] == at[g + 1]) {
            tol[g] = 0.0;
            continue;
          }
          const auto b = side.begin() + std::ptrdiff_t(at[g]);
          const auto e = side.begin() + std::ptrdiff_t(at[g + 1]);
          const auto mid = b + (e - b) / 2; // the median (the upper one of two)
          std::nth_element(b, mid, e);
          const Vec3 &p = P[gmin[g]];
          const double mag = std::max({std::fabs(p.x), std::fabs(p.y), std::fabs(p.z)});
          tol[g] = std::min(eps, std::max(1e-3 * std::sqrt(*mid),
                                          4.0 * std::numeric_limits<float>::epsilon() * mag));
        }
      }
      wc.pairs_held = weld_near(P, gmin, gcomp, tol, cls, pool);
    }
  }

  wc.of.assign(nv, kNone);
  std::vector<u32> members, wedges;
  for (u32 v : verts) {
    const u32 r = find_root(cls, v); // the class's smallest vertex: seen first
    if (r == v) {
      wc.of[v] = u32(wc.rep.size());
      wc.rep.push_back(v);
      members.push_back(0);
      wedges.push_back(0);
    } else {
      wc.of[v] = wc.of[r];
    }
    ++members[wc.of[v]];
    wedges[wc.of[v]] += wc.wedge[v] == v ? 1 : 0;
  }
  wc.multi.resize(wc.rep.size());
  for (std::size_t c = 0; c < wc.rep.size(); ++c)
    wc.multi[c] = wedges[c] > 1 ? 1 : 0;
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
  std::vector<Tri> W; // each triangle's wedges, re-pointed by collapses
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

  // An input the error measure cannot handle -- a non-finite coordinate, or an
  // extent whose square overflows -- is returned unchanged, at an unbounded
  // error (sampled_hausdorff reports +infinity for it too).
  if (!measurable(Pin, W)) {
    if (out)
      for (; next < K; ++next)
        (*out)[order[next]].world_error = kInf;
    return snaps;
  }

  const geometry::points_t &in_pts = mesh.const_points();
  const geometry::uvs_t &in_uv = mesh.const_uvs();
  const geometry::colors_t &in_col = mesh.const_colors();
  const geometry::normals_t &in_nrm = mesh.const_normals();
  const bool has_uv = in_uv.size() == in_pts.size();
  const bool has_col = in_col.size() == in_pts.size();
  const bool has_nrm = !params.recompute_normals && in_nrm.size() == in_pts.size();
  const carried attr = {has_uv ? &in_uv : nullptr, has_col ? &in_col : nullptr,
                        has_nrm ? &in_nrm : nullptr};

  // --- collapse classes: the vertices of the welded topology the loop runs on ---
  const weld_classes wc = weld(Pin, W, attr, params.weld_seams, params.seam_epsilon, pool);
  const u32 nv = u32(wc.rep.size());
  std::vector<Vec3> P(nv);
  for (u32 v = 0; v < nv; ++v)
    P[v] = Pin[wc.rep[v]];
  std::vector<Tri> T(W.size()); // each triangle's classes
  for (std::size_t ti = 0; ti < W.size(); ++ti)
    for (int k = 0; k < 3; ++k) {
      W[ti][k] = wc.wedge[W[ti][k]];
      T[ti][k] = wc.of[W[ti][k]];
    }

  std::vector<char> vremoved(nv, 0);
  std::vector<u32> vver(nv, 0);
  std::vector<char> tremoved(T.size(), 0);
  u64 ntris = n_in;
  // A triangle with two corners in one class takes no part in the collapse and
  // is left out of the result. When two of its corners share a position it is a
  // zero-area needle, and no part of the surface world_error measures against
  // either; one that only welds shut within the coincidence tolerance is
  // measured, so its loss counts.
  std::vector<char> needle(T.size(), 0);
  u64 welded_away = 0; // measured triangles left out before the loop
  for (u32 ti = 0; ti < T.size(); ++ti) {
    const Tri &f = T[ti], &w = W[ti];
    if (f[0] == f[1] || f[1] == f[2] || f[0] == f[2]) {
      tremoved[ti] = 1;
      --ntris;
      const u32 a = wc.pos[w[0]], b = wc.pos[w[1]], c = wc.pos[w[2]];
      needle[ti] = a == b || b == c || a == c ? 1 : 0;
      welded_away += needle[ti] ? 0 : 1;
    }
  }
  if (ntris == 0) // every triangle welds away: keep the input rather than return nothing
    return snaps;

  // The source surface world_error measures against: the input without its
  // needles.
  std::vector<u32> src_ids;
  std::vector<Tri> S0;
  if (out)
    for (u32 ti = 0; ti < T.size(); ++ti)
      if (!needle[ti]) {
        src_ids.push_back(ti);
        S0.push_back(W[ti]);
      }
  std::vector<char>().swap(needle);

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
  std::vector<Quadric>().swap(FQ); // setup only: free it before the loop and the measurements

  // Every undirected class edge once (sorted), and the constrained triangle
  // sides: those whose WEDGE edge has triangles on one side only (all of them
  // share one opposite corner -- a single triangle, or coincident copies of it,
  // like the two faces of a double-sided sheet). That is an open border of the
  // welded surface, or one side of a seam: the class edge goes on through
  // other wedges.
  std::vector<u64> edges;
  std::vector<char> pinned(3 * T.size(), 0); // side k of triangle ti: corners k, k+1
  {
    struct Side {
      u64 edge, wedges; // class edge, wedge edge
      u32 opp, at;      // opposite class, 3 * ti + k
    };
    std::vector<Side> sd;
    sd.reserve(3 * ntris);
    for (u32 ti = 0; ti < T.size(); ++ti)
      if (!tremoved[ti])
        for (u32 k = 0; k < 3; ++k)
          sd.push_back(Side{edge_key(T[ti][k], T[ti][(k + 1) % 3]),
                            edge_key(W[ti][k], W[ti][(k + 1) % 3]), T[ti][(k + 2) % 3],
                            3 * ti + k});
    std::sort(sd.begin(), sd.end(), [](const Side &a, const Side &b) {
      if (a.edge != b.edge)
        return a.edge < b.edge;
      if (a.wedges != b.wedges)
        return a.wedges < b.wedges;
      return a.opp != b.opp ? a.opp < b.opp : a.at < b.at;
    });
    for (std::size_t i = 0; i < sd.size();) {
      std::size_t j = i + 1;
      while (j < sd.size() && sd[j].wedges == sd[i].wedges)
        ++j;
      if (edges.empty() || edges.back() != sd[i].edge)
        edges.push_back(sd[i].edge);
      if (sd[j - 1].opp == sd[i].opp) // sorted: first == last opposite => one opposite
        for (std::size_t k = i; k < j; ++k)
          pinned[sd[k].at] = 1;
      i = j;
    }
  }

  // --- constraint quadrics: for each constrained side, add a plane through the
  // edge, perpendicular to the face, so borders and seams hold their line.
  // Weighted by boundary_weight * |e|^2 -- the same length^2 units as the
  // area-weighted face planes -- so the relative strength (and the collapse
  // order) is independent of the mesh's scale. ---
  if (params.preserve_boundary) {
    for (u32 ti = 0; ti < T.size(); ++ti) {
      if (tremoved[ti])
        continue;
      const Vec3 &a = P[T[ti][0]], &b = P[T[ti][1]], &c = P[T[ti][2]];
      Vec3 fn = face_normal(a, b, c);
      for (int k = 0; k < 3; ++k) {
        if (!pinned[3 * ti + k])
          continue; // interior side
        u32 v0 = T[ti][k], v1 = T[ti][(k + 1) % 3];
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

  std::vector<char>().swap(pinned);
  const double flip_cos = std::cos(params.max_normal_flip_deg * 3.14159265358979323846 / 180.0);

  // The wedge map of collapsing class `drop` onto `keep`: each wedge of drop goes
  // to its partner, the wedge of keep it shares a vanishing triangle with. false
  // when a wedge of drop that a surviving triangle still uses has no partner
  // (drop lies on a seam the edge does not run along) or two (a seam ends at
  // drop): re-pointing it anywhere would attach its attributes to a position
  // other than its own, or tear the triangles that share it apart.
  // The map is sorted by drop wedge, so a class with many wedges (a pile of
  // near-coincident parts) costs O(k log k), not O(k^2).
  const u32 kTwo = kMixed; // a wedge with two different partners
  auto partner_of = [](const std::vector<std::pair<u32, u32>> &pr, u32 w) {
    const auto p = std::lower_bound(pr.begin(), pr.end(), std::make_pair(w, u32(0)));
    return p != pr.end() && p->first == w ? p->second : kNone;
  };
  auto partners = [&](u32 keep, u32 drop, std::vector<std::pair<u32, u32>> &pr) {
    pr.clear();
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
      if (kk >= 0)
        pr.push_back(std::make_pair(W[ti][kd], W[ti][kk]));
    }
    if (pr.empty())
      return false; // no triangle on the edge
    if (!wc.multi[drop] && !wc.multi[keep]) {
      pr.resize(1); // one wedge each: a unique partner
      return true;
    }
    std::sort(pr.begin(), pr.end());
    pr.erase(std::unique(pr.begin(), pr.end()), pr.end());
    std::size_t n = 0;
    for (std::size_t i = 0; i < pr.size();) {
      std::size_t j = i + 1;
      while (j < pr.size() && pr[j].first == pr[i].first)
        ++j;
      pr[n++] = std::make_pair(pr[i].first, j - i > 1 ? kTwo : pr[i].second);
      i = j;
    }
    pr.resize(n);
    for (u32 ti : vtri[drop]) {
      if (tremoved[ti])
        continue;
      const Tri &f = T[ti];
      if (f[0] == keep || f[1] == keep || f[2] == keep)
        continue;
      const u32 kw = partner_of(pr, W[ti][f[0] == drop ? 0 : (f[1] == drop ? 1 : 2)]);
      if (kw == kNone || kw == kTwo)
        return false;
    }
    return true;
  };

  // Evaluate the half-edge collapse of edge (u,v): the cheaper direction the
  // seams allow (false: neither). Keeping u costs u's own quadric at u (its c)
  // plus v's quadric at u's offset from v. A cost inside its round-off band --
  // the magnitude of the terms it sums, plus the round-off already carried in
  // both quadrics -- is exactly 0.
  std::vector<std::pair<u32, u32>> scratch;
  auto eval = [&](u32 u, u32 v, double &cost, u32 &keep, u32 &drop) {
    const Vec3 o = sub(P[u], P[v]); // u seen from v; v seen from u is exactly -o
    const Vec3 no = {-o.x, -o.y, -o.z};
    const double carried_m = Q[u].m + Q[v].m;
    double eu = Q[u].c + Q[v].at(o);
    double ev = Q[v].c + Q[u].at(no);
    const double bu = kCostRoundoff * (carried_m + std::max(0.0, Q[u].c) + Q[v].mag(o));
    const double bv = kCostRoundoff * (carried_m + std::max(0.0, Q[v].c) + Q[u].mag(no));
    eu = eu <= bu ? 0.0 : quantize(eu);
    ev = ev <= bv ? 0.0 : quantize(ev);
    const bool seam = wc.multi[u] || wc.multi[v];
    const bool u_first = eu <= ev;
    const u32 a = u_first ? u : v, b = u_first ? v : u;
    if (!seam || partners(a, b, scratch)) {
      keep = a;
      drop = b;
      cost = u_first ? eu : ev;
      return true;
    }
    if (partners(b, a, scratch)) {
      keep = b;
      drop = a;
      cost = u_first ? ev : eu;
      return true;
    }
    return false;
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
  std::vector<u64>().swap(edges);

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

  // The link condition, over classes: the classes adjacent to both keep and
  // drop must be exactly the third corners of the triangles on the edge. Any
  // other common neighbour w would merge the edges keep-w and drop-w into one
  // carrying the triangles of both: more than two, or two faces folded onto
  // the same three positions. Checked in O(degree) with per-class stamps: a
  // neighbour of drop off the edge gets `stamp`, a third corner `stamp + 1`.
  std::vector<u32> link(nv, 0);
  u32 stamp = 0;
  auto links_ok = [&](u32 keep, u32 drop) -> bool {
    if (stamp > u32(-4)) { // restart the stamps before they wrap
      std::fill(link.begin(), link.end(), 0u);
      stamp = 0;
    }
    stamp += 2;
    for (u32 ti : vtri[drop]) {
      if (tremoved[ti])
        continue;
      const Tri &f = T[ti];
      const bool on_edge = f[0] == keep || f[1] == keep || f[2] == keep;
      for (int k = 0; k < 3; ++k)
        if (f[k] != drop && f[k] != keep && link[f[k]] != stamp + 1)
          link[f[k]] = on_edge ? stamp + 1 : stamp;
    }
    for (u32 ti : vtri[keep]) {
      if (tremoved[ti])
        continue;
      const Tri &f = T[ti];
      if (f[0] == drop || f[1] == drop || f[2] == drop)
        continue; // on the edge
      for (int k = 0; k < 3; ++k)
        if (f[k] != keep && link[f[k]] == stamp)
          return false; // a common neighbour that is no third corner
    }
    return true;
  };

  u64 ncollapse = 0;
  bool hit_limit = false;

  // --- snapshots: a compact geometry of the wedges the live triangles use ---
  auto build = [&]() {
    geometry result(mesh.ctx());
    std::vector<u32> remap(in_pts.size(), kNone);
    for (u32 ti = 0; ti < W.size(); ++ti)
      if (!tremoved[ti])
        for (int k = 0; k < 3; ++k)
          remap[W[ti][k]] = 0; // referenced
    geometry::points_t &op = result.points();
    geometry::uvs_t *ouv = has_uv ? &result.uvs() : nullptr;
    geometry::colors_t *ocol = has_col ? &result.colors() : nullptr;
    geometry::normals_t *onrm = has_nrm ? &result.normals() : nullptr;
    for (u32 w = 0; w < u32(remap.size()); ++w) {
      if (remap[w] == kNone)
        continue;
      remap[w] = u32(op.size());
      op.push_back(in_pts[w]); // wedges never move: every attribute stays with its position
      if (ouv)
        ouv->push_back(in_uv[w]);
      if (ocol)
        ocol->push_back(in_col[w]);
      if (onrm)
        onrm->push_back(in_nrm[w]);
    }
    geometry::tris_t &ot = result.tris();
    for (u32 ti = 0; ti < W.size(); ++ti)
      if (!tremoved[ti])
        ot.push_back({remap[W[ti][0]], remap[W[ti][1]], remap[W[ti][2]]});
    result.set_geometry_type(geometry::SURFACE_TRI);
    if (!has_nrm)
      result.compute_normals();
    return result;
  };

  // world_error: symmetric sampled Hausdorff between the source (the input
  // without its needles, S0) and the live surface, both over the wedges' own
  // positions -- exactly the input and result surfaces. The source side (tree,
  // used vertices, unique edges) is built once and reused by every snapshot;
  // construction is deterministic, so a progressive snapshot measures exactly
  // what an independent run would. Samples the two surfaces share -- every live
  // vertex, an edge present in both, an untouched triangle's centroid -- lie ON
  // the other surface (under the round-off floor), so they are not queried: the
  // value is bit-identical to sampled_hausdorff(input, snapshot) when the input
  // has no needles, for less work on fine rungs.
  std::unique_ptr<tri_bvh> src_tree;
  std::vector<u32> src_verts;
  std::vector<u64> src_edges;
  auto measure = [&]() {
    if (!src_tree) {
      src_tree.reset(new tri_bvh(Pin, S0, pool));
      src_verts = used_vertices(Pin.size(), S0);
      src_edges = unique_edges(S0);
    }
    std::vector<Tri> live;
    live.reserve(ntris);
    std::vector<char> untouched(S0.size(), 0); // a source triangle that is still live, as is
    std::vector<u32> changed;                  // live triangles (indices into `live`) that are new
    for (std::size_t j = 0; j < S0.size(); ++j) {
      const u32 ti = src_ids[j];
      if (tremoved[ti])
        continue;
      if (W[ti] == S0[j])
        untouched[j] = 1;
      else
        changed.push_back(u32(live.size()));
      live.push_back(W[ti]);
    }
    const tri_bvh tree(Pin, live, pool);
    const std::vector<u64> live_edges = unique_edges(live);
    std::vector<char> on_live(Pin.size(), 0);
    for (const Tri &t : live)
      on_live[t[0]] = on_live[t[1]] = on_live[t[2]] = 1;

    sample_set src{&Pin, &S0, {}, {}, {}}; // source samples to test against the live surface
    for (u32 v : src_verts)
      if (!on_live[v])
        src.verts.push_back(v);
    std::set_difference(src_edges.begin(), src_edges.end(), live_edges.begin(), live_edges.end(),
                        std::back_inserter(src.edges));
    for (u32 j = 0; j < u32(S0.size()); ++j)
      if (!untouched[j])
        src.tris.push_back(j);
    // live samples to test against the source; every live vertex is a source vertex
    sample_set lv{&Pin, &live, {}, {}, changed};
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
      r.world_error = ncollapse > 0 || welded_away > 0 ? measure() : 0.0;
      r.hit_error_limit = hit_limit;
      r.seam_vertices = wc.seam;
      r.weld_pairs_held = wc.pairs_held;
    }
    last = k;
    last_collapses = ncollapse;
  };

  std::vector<std::pair<u32, u32>> pairs; // drop's wedges -> their partners
  std::vector<u32> ring;                  // keep's neighbours, to re-cost
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
    const u32 keep = it.keep, drop = it.drop;
    if (causes_flip(keep, drop))
      continue; // leave this edge out; a re-pushed copy may succeed later
    if (!partners(keep, drop, pairs))
      continue; // no clean wedge map (it was clean when pushed; kept as a guard)
    if (!links_ok(keep, drop))
      continue; // it would fold the surface onto itself

    // The triangles on the collapsed edge vanish.
    u64 vanish = 0;
    for (u32 ti : vtri[drop])
      if (!tremoved[ti] && (T[ti][0] == keep || T[ti][1] == keep || T[ti][2] == keep))
        ++vanish;
    if (vanish == ntris)
      continue; // never remove the last triangle: an empty rung has no finite error

    // rewrite drop's faces onto keep, each wedge onto its partner; drop the
    // vanishing ones
    for (u32 ti : vtri[drop]) {
      if (tremoved[ti])
        continue;
      Tri &f = T[ti];
      if (f[0] == keep || f[1] == keep || f[2] == keep) {
        tremoved[ti] = 1; // shared face collapses to a line
        --ntris;
        continue;
      }
      for (int k = 0; k < 3; ++k)
        if (f[k] == drop) {
          f[k] = keep;
          W[ti][k] = partner_of(pairs, W[ti][k]);
        }
      vtri[keep].push_back(ti);
    }

    Q[keep].merge(Q[drop], sub(P[keep], P[drop]));
    vremoved[drop] = 1;
    ++vver[keep];
    ++ncollapse;

    // compact keep's incident list (drop removed faces) and re-cost its edges
    std::vector<u32> &kt = vtri[keep];
    kt.erase(std::remove_if(kt.begin(), kt.end(), [&](u32 ti) { return tremoved[ti] != 0; }),
             kt.end());
    ring.clear();
    for (u32 ti : kt)
      for (int k = 0; k < 3; ++k)
        if (T[ti][k] != keep)
          ring.push_back(T[ti][k]);
    std::sort(ring.begin(), ring.end());
    ring.erase(std::unique(ring.begin(), ring.end()), ring.end());
    for (u32 w : ring)
      push_edge(keep, w);
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
  const tri_bvh ga(PA, TA, pool), gb(PB, TB, pool);
  return hausdorff(all_samples(PA, TA), ga, all_samples(PB, TB), gb, pool);
}

} // namespace cvc
