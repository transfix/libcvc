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

// cvc::mesh_contour_tree_builder and compute_contour_tree on a mesh
// (inc/cvc/volume/contour_tree.h).
//
// Carr, Snoeyink & Axen's algorithm, as the grid builder (contour_tree.cpp) runs it, on the vertex
// graph of a simplicial mesh: the used vertices sorted by (value, index); a join tree and a split
// tree from two union-find sweeps; their merge, one leaf (up-degree in the join tree + down-degree
// in the split tree == 1) at a time; then the reduction past regular vertices. The trees are kept
// augmented (every vertex a node), with each vertex's single down (join) or up (split) neighbour
// stored and its other side as a count plus the XOR of the neighbours, which names the one left
// once the count is 1: removing a leaf is O(1). The graph is the elements' edges in CSR form (an
// edge two elements share is listed twice, which the sweeps tolerate). Every phase is a loop that
// stops at its budget and resumes; the sort is a bottom-up merge sort so that it can too.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/geometry/geometry.h>
#include <cvc/volume/contour_mesh.h>
#include <cvc/volume/contour_tree.h>
#include <exception>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace {

using u32 = std::uint32_t;
constexpr u32 kNone = 0xffffffffu;
constexpr uint64 kUnlimited = std::numeric_limits<uint64>::max();
// compute_contour_tree's slice when it has a progress callback to call.
constexpr std::size_t kOneShotBudget = std::size_t(1) << 16;

contour_tree_error mesh_tree_error(const std::string &what) {
  return contour_tree_error("compute_contour_tree (mesh): " + what);
}

contour_tree::node_kind kind_of(u32 up, u32 down) {
  using k = contour_tree::node_kind;
  if (down == 0)
    return k::minimum;
  if (up == 0)
    return k::maximum;
  if (up >= 2 && down >= 2)
    return k::join_split;
  return up >= 2 ? k::join : k::split;
}

// The corner pairs of a tetrahedron's six edges and of a triangle's three.
constexpr int kTetEdges[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
constexpr int kTriEdges[3][2] = {{0, 1}, {0, 2}, {1, 2}};

} // namespace

struct mesh_contour_tree_builder::impl {
  enum class stage {
    start,
    degrees, // count each vertex's edges (and check the elements)
    offsets, // the CSR offsets, the NaN check, the value range
    edges,   // list the edges
    sort,    // order the used vertices by (value, index)
    join,    // the join tree, sweeping down
    split,   // the split tree, sweeping up
    leaves,  // the first leaves
    merge,   // the merge, one leaf at a time
    reduce,  // the contour tree's degrees and up-neighbour lists
    nodes,   // the critical vertices, in order
    arcs,    // the arcs between them, past the regular vertices
    done,
    failed
  };

  std::shared_ptr<const contour_mesh> mesh;
  contour_tree_options opt;
  stage st = stage::start;
  std::exception_ptr error;
  double failed_progress = 0.0;
  uint64 left = 0, work = 0;
  double estimate = 1.0; // the expected units, for progress()

  bool solid = false;
  u32 npts = 0;
  std::size_t nelem = 0;
  const double *f = nullptr;
  std::size_t cursor = 0; // the current loop's position
  bool begun = false;     // the current stage's setup ran

  // The vertex graph: v's neighbours are adj[off[v] .. off[v + 1]).
  std::vector<u32> off, adj, fill;
  // The used vertices in (value, index) order, and each one's place in it.
  std::vector<u32> order, tmp, rank;
  u32 m = 0;
  double vmin = 0.0, vmax = 0.0;
  // The merge sort: runs of `width`; the pair starting at `run_at`, merged up to ia / ib into io.
  u32 width = 1, run_at = 0, ia = 0, ib = 0, io = 0;
  bool pair_open = false;
  // Union-find, and each set's lowest (join) or highest (split) vertex.
  std::vector<u32> parent, ext;
  std::vector<std::uint8_t> uf_rank;
  // The join tree: each vertex's down neighbour, and its up neighbours' count and XOR. The split
  // tree: its up neighbour, and its down neighbours' count and XOR.
  std::vector<u32> jdown, jup_n, jup_x, sup, sdn_n, sdn_x;
  u32 ncomp = 0; // connected pieces: the vertices the join sweep left with no down neighbour
  // The merge.
  std::vector<u32> queue;
  std::size_t qhead = 0;
  std::vector<std::uint8_t> removed;
  std::vector<std::pair<u32, u32>> carcs; // the augmented contour tree, (above, below)
  u32 top = 0;                            // where a forced leaf's search resumes (an order index)
  // The reduction.
  std::vector<u32> cup_n, cdn_n, coff, cadj, node_of;
  u32 walk_nb = 0, walk_at = kNone; // the arc walk: the neighbour index and the vertex reached
  contour_tree out;
  bool taken = false;

  impl(std::shared_ptr<const contour_mesh> mesh_, const contour_tree_options &o)
      : mesh(std::move(mesh_)), opt(o) {}

  void spend(uint64 units) {
    work += units;
    left = units >= left ? 0 : left - units;
  }

  const geometry &geom() const { return *mesh->mesh; }

  bool less(u32 a, u32 b) const { return f[a] < f[b] || (f[a] == f[b] && a < b); }

  void begin() {
    if (!mesh || !mesh->mesh)
      throw mesh_tree_error("no mesh");
    const std::size_t n = geom().const_points().size();
    if (n >= std::size_t(kNone))
      throw mesh_tree_error("2^32 or more points");
    if (mesh->values.size() != n)
      throw mesh_tree_error("the mesh has " + std::to_string(n) + " points but " +
                            std::to_string(mesh->values.size()) + " values");
    npts = u32(n);
    f = mesh->values.data();
    solid = !geom().const_tets().empty();
    nelem = solid ? geom().const_tets().size() : geom().const_tris().size();
    if (nelem == 0)
      throw mesh_tree_error("the mesh has neither tetrahedra nor triangles");
    const double edges = static_cast<double>(nelem) * (solid ? 12.0 : 6.0);
    const double v = static_cast<double>(npts);
    estimate = 2.0 * static_cast<double>(nelem) + v + v * std::max(1.0, std::log2(v)) +
               2.0 * (v + edges) + 5.0 * v;
    off.assign(std::size_t(npts) + 1, 0);
    st = stage::degrees;
    cursor = 0;
  }

  template <std::size_t N> void check(const boost::array<geometry::index_t, N> &e) const {
    for (std::size_t c = 0; c < N; ++c)
      if (e[c] >= npts)
        throw mesh_tree_error("an element names point " + std::to_string(e[c]) + " of " +
                              std::to_string(npts));
  }

  // An edge with a NaN end is left out, and so is a point left with no edge: the field is not
  // defined there (a curvature at a degenerate vertex, say).
  bool defined(u32 a, u32 b) const { return !std::isnan(f[a]) && !std::isnan(f[b]); }

  void count(u32 a, u32 b) {
    if (defined(a, b)) {
      ++off[a];
      ++off[b];
    }
  }

  void degrees() {
    while (left > 0 && cursor < nelem) {
      if (solid) {
        const geometry::tet_t &t = geom().const_tets()[cursor];
        check(t);
        for (const auto &e : kTetEdges)
          count(u32(t[e[0]]), u32(t[e[1]]));
      } else {
        const geometry::tri_t &t = geom().const_tris()[cursor];
        check(t);
        for (const auto &e : kTriEdges)
          count(u32(t[e[0]]), u32(t[e[1]]));
      }
      ++cursor;
      spend(1);
    }
    if (cursor == nelem) {
      st = stage::offsets;
      cursor = 0;
    }
  }

  // Degrees to offsets in place (off[v] becomes the start of v's list), and the used vertices'
  // range.
  u32 running = 0;
  void offsets() {
    while (left > 0 && cursor < npts) {
      const u32 v = u32(cursor++);
      const u32 d = off[v];
      off[v] = running;
      running += d;
      if (d > 0) {
        if (m == 0) {
          vmin = vmax = f[v];
        } else {
          vmin = std::min(vmin, f[v]);
          vmax = std::max(vmax, f[v]);
        }
        ++m;
      }
      spend(1);
    }
    if (cursor < npts)
      return;
    off[npts] = running;
    if (m == 0)
      throw mesh_tree_error("no edge has two values that are not NaN");
    out.dims = {{uint64(npts), 1, 1}};
    out.value_min = vmin;
    out.value_max = vmax;
    if (vmin == vmax) { // constant: one node, no arcs
      for (u32 v = 0; v < npts; ++v)
        if (off[v + 1] > off[v]) {
          contour_tree::node nd;
          nd.voxel = v;
          nd.value = f[v];
          nd.kind = contour_tree::node_kind::minimum;
          out.nodes.push_back(nd);
          break;
        }
      finish();
      return;
    }
    adj.resize(running);
    fill.assign(off.begin(), off.end() - 1);
    st = stage::edges;
    cursor = 0;
  }

  void link(u32 a, u32 b) {
    if (!defined(a, b))
      return;
    adj[fill[a]++] = b;
    adj[fill[b]++] = a;
  }

  void edges() {
    while (left > 0 && cursor < nelem) {
      if (solid) {
        const geometry::tet_t &t = geom().const_tets()[cursor];
        for (const auto &e : kTetEdges)
          link(u32(t[e[0]]), u32(t[e[1]]));
      } else {
        const geometry::tri_t &t = geom().const_tris()[cursor];
        for (const auto &e : kTriEdges)
          link(u32(t[e[0]]), u32(t[e[1]]));
      }
      ++cursor;
      spend(1);
    }
    if (cursor < nelem)
      return;
    fill = std::vector<u32>();
    order.reserve(m);
    for (u32 v = 0; v < npts; ++v) // O(points), once
      if (off[v + 1] > off[v])
        order.push_back(v);
    tmp.resize(m);
    width = 1;
    run_at = 0;
    pair_open = false;
    st = stage::sort;
  }

  void sort() {
    while (left > 0) {
      if (width >= m) {
        tmp = std::vector<u32>();
        rank.assign(npts, kNone);
        for (u32 i = 0; i < m; ++i) // O(vertices), once
          rank[order[i]] = i;
        st = stage::join;
        begun = false;
        return;
      }
      if (run_at >= m) { // a pass is done
        std::swap(order, tmp);
        width = width > m / 2 ? m : width * 2;
        run_at = 0;
        continue;
      }
      const u32 mid = u32(std::min<uint64>(m, uint64(run_at) + width));
      const u32 end = u32(std::min<uint64>(m, uint64(run_at) + 2 * uint64(width)));
      if (!pair_open) {
        ia = run_at;
        ib = mid;
        io = run_at;
        pair_open = true;
      }
      while (left > 0 && io < end) {
        if (ia < mid && (ib >= end || !less(order[ib], order[ia])))
          tmp[io++] = order[ia++];
        else
          tmp[io++] = order[ib++];
        spend(1);
      }
      if (io == end) {
        run_at = end;
        pair_open = false;
      }
    }
  }

  u32 find(u32 x) {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  }

  u32 unite(u32 a, u32 b) {
    if (uf_rank[a] < uf_rank[b])
      std::swap(a, b);
    parent[b] = a;
    if (uf_rank[a] == uf_rank[b])
      ++uf_rank[a];
    return a;
  }

  void reset_union_find() {
    parent.resize(npts);
    ext.assign(npts, kNone);
    uf_rank.assign(npts, 0);
    for (u32 v = 0; v < npts; ++v) // O(points), once per sweep
      parent[v] = v;
  }

  // The join tree: from the top down, each vertex joins the components above it it touches; each
  // one's lowest vertex so far gets it as its down neighbour.
  void join() {
    if (!begun) {
      begun = true;
      reset_union_find();
      jdown.assign(npts, kNone);
      jup_n.assign(npts, 0);
      jup_x.assign(npts, 0);
      cursor = m;
    }
    while (left > 0 && cursor > 0) {
      const u32 v = order[--cursor];
      const u32 rv = rank[v];
      u32 root = v;
      for (u32 k = off[v]; k < off[v + 1]; ++k) {
        const u32 u = adj[k];
        if (rank[u] < rv)
          continue;
        const u32 ru = find(u);
        if (ru == root)
          continue;
        const u32 low = ext[ru];
        jdown[low] = v;
        ++jup_n[v];
        jup_x[v] ^= low;
        root = unite(ru, root);
      }
      ext[root] = v;
      spend(1 + (off[v + 1] - off[v]));
    }
    if (cursor == 0) {
      ncomp = 0;
      for (u32 i = 0; i < m; ++i)
        ncomp += jdown[order[i]] == kNone ? 1 : 0;
      st = stage::split;
      begun = false;
    }
  }

  // The split tree, the same from the bottom up.
  void split() {
    if (!begun) {
      begun = true;
      reset_union_find();
      sup.assign(npts, kNone);
      sdn_n.assign(npts, 0);
      sdn_x.assign(npts, 0);
      cursor = 0;
    }
    while (left > 0 && cursor < m) {
      const u32 v = order[cursor++];
      const u32 rv = rank[v];
      u32 root = v;
      for (u32 k = off[v]; k < off[v + 1]; ++k) {
        const u32 u = adj[k];
        if (rank[u] > rv)
          continue;
        const u32 ru = find(u);
        if (ru == root)
          continue;
        const u32 high = ext[ru];
        sup[high] = v;
        ++sdn_n[v];
        sdn_x[v] ^= high;
        root = unite(ru, root);
      }
      ext[root] = v;
      spend(1 + (off[v + 1] - off[v]));
    }
    if (cursor == m) {
      parent = std::vector<u32>();
      ext = std::vector<u32>();
      uf_rank = std::vector<std::uint8_t>();
      adj = std::vector<u32>();
      off = std::vector<u32>();
      removed.assign(npts, 0);
      queue.reserve(m);
      carcs.reserve(m);
      st = stage::leaves;
      cursor = 0;
    }
  }

  bool leaf(u32 v) const { return jup_n[v] + sdn_n[v] == 1; }

  void leaves() {
    while (left > 0 && cursor < m) {
      const u32 v = order[cursor++];
      if (leaf(v))
        queue.push_back(v);
      spend(1);
    }
    if (cursor == m) {
      top = m;
      st = stage::merge;
    }
  }

  // x leaves the trees with no up neighbour in the join tree: its arc goes down to its join
  // neighbour j, and in the split tree its down neighbours move up to its up neighbour. A leaf has
  // one down neighbour there; a forced one (below) may have more, which `kids` then lists.
  void remove_upper(u32 x, const std::vector<u32> *kids) {
    const u32 j = jdown[x];
    if (j == kNone)
      throw mesh_tree_error("internal error: a leaf with no join neighbour");
    carcs.emplace_back(x, j);
    --jup_n[j];
    jup_x[j] ^= x;
    const u32 p = sup[x];
    if (kids && p == kNone && kids->size() > 1) {
      // x tops its split tree too. Moved up to nothing, its kids would each be a root, and the
      // split tree a forest whose lower leaves have no split neighbour. Hang the others from the
      // highest kid (`kids` is in sweep order) instead: one tree, as cutting the loop at x allows.
      const u32 r = kids->back();
      u32 xs = 0;
      sup[r] = kNone;
      for (std::size_t i = 0; i + 1 < kids->size(); ++i) {
        sup[(*kids)[i]] = r;
        xs ^= (*kids)[i];
      }
      sdn_n[r] += u32(kids->size()) - 1;
      sdn_x[r] ^= xs;
    } else if (kids) {
      u32 xs = 0;
      for (const u32 d : *kids) {
        sup[d] = p;
        xs ^= d;
      }
      if (p != kNone) {
        sdn_n[p] += u32(kids->size()) - 1;
        sdn_x[p] ^= x ^ xs;
      }
    } else {
      const u32 d = sdn_x[x];
      sup[d] = p;
      if (p != kNone)
        sdn_x[p] ^= x ^ d;
    }
    removed[x] = 1;
    if (!removed[j] && leaf(j))
      queue.push_back(j);
  }

  // The mirror image: no down neighbour in the split tree.
  void remove_lower(u32 x) {
    const u32 s = sup[x];
    if (s == kNone)
      throw mesh_tree_error("internal error: a leaf with no split neighbour");
    carcs.emplace_back(s, x);
    --sdn_n[s];
    sdn_x[s] ^= x;
    const u32 u = jup_x[x], q = jdown[x];
    jdown[u] = q;
    if (q != kNone)
      jup_x[q] ^= x ^ u;
    removed[x] = 1;
    if (!removed[s] && leaf(s))
      queue.push_back(s);
  }

  void merge() {
    while (left > 0) {
      if (qhead == queue.size()) {
        // Every piece reduced to one vertex: done. Otherwise the field's level sets form a loop
        // here (the domain is not simply connected) and no vertex is a leaf: take the highest
        // remaining one that is not alone in its piece as an upper leaf, whatever its split-tree
        // down neighbours.
        if (m - u32(carcs.size()) <= ncomp) {
          queue = std::vector<u32>();
          st = stage::reduce;
          cursor = 0;
          begun = false;
          return;
        }
        while (top > 0 &&
               (removed[order[top - 1]] || jup_n[order[top - 1]] + sdn_n[order[top - 1]] == 0))
          --top;
        if (top == 0)
          throw mesh_tree_error("internal error: the merge ran out of vertices");
        const u32 x = order[top - 1];
        std::vector<u32> kids;
        for (u32 i = 0; i < m; ++i) {
          const u32 d = order[i];
          if (!removed[d] && sup[d] == x)
            kids.push_back(d);
        }
        spend(m);
        if (jup_n[x] != 0 || kids.size() != sdn_n[x])
          throw mesh_tree_error("internal error: a forced leaf is not the top of its piece");
        remove_upper(x, &kids);
        continue;
      }
      const u32 x = queue[qhead++];
      spend(1);
      if (removed[x] || !leaf(x))
        continue; // already gone, or the last vertex of its piece
      if (jup_n[x] == 0)
        remove_upper(x, nullptr);
      else
        remove_lower(x);
    }
  }

  // The contour tree's degrees, and each vertex's up neighbours (CSR by the lower end).
  void reduce() {
    if (!begun) {
      begun = true;
      jdown = jup_n = jup_x = sup = sdn_n = sdn_x = std::vector<u32>();
      removed = std::vector<std::uint8_t>();
      cup_n.assign(npts, 0);
      cdn_n.assign(npts, 0);
      for (const auto &a : carcs) { // O(arcs), once
        ++cdn_n[a.first];
        ++cup_n[a.second];
      }
      coff.assign(std::size_t(npts) + 1, 0);
      for (u32 v = 0; v < npts; ++v)
        coff[v + 1] = coff[v] + cup_n[v];
      cadj.resize(carcs.size());
      cursor = 0;
    }
    while (left > 0 && cursor < carcs.size()) {
      const auto &a = carcs[cursor++];
      cadj[coff[a.second + 1] - cup_n[a.second]] = a.first;
      --cup_n[a.second];
      spend(1);
    }
    if (cursor < carcs.size())
      return;
    for (u32 v = 0; v < npts; ++v) // restore the up-degrees the fill counted down, once
      cup_n[v] = coff[v + 1] - coff[v];
    carcs = std::vector<std::pair<u32, u32>>();
    node_of.assign(npts, kNone);
    out.nodes.clear();
    st = stage::nodes;
    cursor = 0;
  }

  bool critical(u32 v) const { return !(cup_n[v] == 1 && cdn_n[v] == 1); }

  void nodes() {
    while (left > 0 && cursor < m) {
      const u32 v = order[cursor++];
      if (critical(v)) {
        node_of[v] = u32(out.nodes.size());
        contour_tree::node nd;
        nd.voxel = v;
        nd.value = f[v];
        nd.kind = kind_of(cup_n[v], cdn_n[v]);
        out.nodes.push_back(nd);
      }
      spend(1);
    }
    if (cursor == m) {
      st = stage::arcs;
      cursor = 0;
      walk_nb = 0;
      walk_at = kNone;
    }
  }

  // From each critical vertex up each of its contour-tree arcs, past the regular vertices (one up
  // neighbour each), to the next critical vertex: one arc of the tree.
  void arcs() {
    while (left > 0 && cursor < m) {
      const u32 c = order[cursor];
      if (!critical(c) || walk_nb == cup_n[c]) {
        ++cursor;
        walk_nb = 0;
        walk_at = kNone;
        spend(1);
        continue;
      }
      if (walk_at == kNone)
        walk_at = cadj[coff[c] + walk_nb];
      while (left > 0 && !critical(walk_at)) {
        walk_at = cadj[coff[walk_at]];
        spend(1);
      }
      if (!critical(walk_at))
        return; // the budget ran out mid-walk
      contour_tree::arc a;
      a.hi = node_of[walk_at];
      a.lo = node_of[c];
      out.arcs.push_back(a);
      ++walk_nb;
      walk_at = kNone;
      spend(1);
    }
    if (cursor == m)
      finish();
  }

  void finish() {
    order = rank = cup_n = cdn_n = coff = cadj = node_of = std::vector<u32>();
    f = nullptr;
    mesh.reset();
    st = stage::done;
  }

  void run(uint64 budget) {
    left = budget;
    if (st == stage::start)
      begin();
    while (left > 0) {
      switch (st) {
      case stage::degrees:
        degrees();
        break;
      case stage::offsets:
        offsets();
        break;
      case stage::edges:
        edges();
        break;
      case stage::sort:
        sort();
        break;
      case stage::join:
        join();
        break;
      case stage::split:
        split();
        break;
      case stage::leaves:
        leaves();
        break;
      case stage::merge:
        merge();
        break;
      case stage::reduce:
        reduce();
        break;
      case stage::nodes:
        nodes();
        break;
      case stage::arcs:
        arcs();
        break;
      default:
        return;
      }
    }
  }

  double progress() const {
    if (st == stage::start)
      return 0.0;
    if (st == stage::done)
      return 1.0;
    if (st == stage::failed)
      return failed_progress;
    return std::min(0.99, static_cast<double>(work) / estimate);
  }

  void fail(std::exception_ptr e) {
    failed_progress = progress();
    error = std::move(e);
    st = stage::failed;
    mesh.reset();
    f = nullptr;
    off = adj = fill = order = tmp = rank = parent = ext = jdown = jup_n = jup_x = sup = sdn_n =
        sdn_x = queue = cup_n = cdn_n = coff = cadj = node_of = std::vector<u32>();
    uf_rank = removed = std::vector<std::uint8_t>();
    carcs = std::vector<std::pair<u32, u32>>();
    out = contour_tree();
  }
};

mesh_contour_tree_builder::mesh_contour_tree_builder(std::shared_ptr<const contour_mesh> mesh,
                                                     const contour_tree_options &opt)
    : _impl(std::make_unique<impl>(std::move(mesh), opt)) {}

mesh_contour_tree_builder::~mesh_contour_tree_builder() = default;
mesh_contour_tree_builder::mesh_contour_tree_builder(mesh_contour_tree_builder &&) noexcept =
    default;
mesh_contour_tree_builder &
mesh_contour_tree_builder::operator=(mesh_contour_tree_builder &&) noexcept = default;

bool mesh_contour_tree_builder::step(std::size_t budget) {
  if (!_impl)
    return true;
  impl &s = *_impl;
  if (s.st == impl::stage::failed)
    std::rethrow_exception(s.error);
  if (s.st == impl::stage::done)
    return true;
  try {
    s.run(budget == 0 ? kUnlimited : static_cast<uint64>(budget));
    if (s.opt.progress && !s.opt.progress(s.progress()))
      throw mesh_tree_error("cancelled");
  } catch (...) {
    s.fail(std::current_exception());
    throw;
  }
  return s.st == impl::stage::done;
}

bool mesh_contour_tree_builder::done() const {
  return !_impl || _impl->st == impl::stage::done || _impl->st == impl::stage::failed;
}

double mesh_contour_tree_builder::progress() const { return _impl ? _impl->progress() : 1.0; }

const char *mesh_contour_tree_builder::phase() const {
  if (!_impl)
    return "done";
  switch (_impl->st) {
  case impl::stage::start:
  case impl::stage::degrees:
  case impl::stage::offsets:
  case impl::stage::edges:
    return "preparing";
  case impl::stage::sort:
    return "sorting";
  case impl::stage::join:
    return "join tree";
  case impl::stage::split:
    return "split tree";
  case impl::stage::leaves:
  case impl::stage::merge:
  case impl::stage::reduce:
  case impl::stage::nodes:
  case impl::stage::arcs:
    return "merging";
  case impl::stage::failed:
    return "failed";
  case impl::stage::done:
    break;
  }
  return "done";
}

contour_tree mesh_contour_tree_builder::take() {
  if (!_impl)
    throw contour_tree_error("mesh_contour_tree_builder: a moved-from builder has no tree");
  impl &s = *_impl;
  if (s.st == impl::stage::failed)
    std::rethrow_exception(s.error);
  if (s.st != impl::stage::done || s.taken)
    throw contour_tree_error(s.taken ? "mesh_contour_tree_builder: the tree was taken"
                                     : "mesh_contour_tree_builder: the tree is not ready");
  s.taken = true;
  return std::move(s.out);
}

contour_tree compute_contour_tree(const contour_mesh &mesh, const contour_tree_options &opt) {
  // Borrowed for the call: an aliasing pointer that owns nothing.
  mesh_contour_tree_builder builder(
      std::shared_ptr<const contour_mesh>(std::shared_ptr<const contour_mesh>(), &mesh), opt);
  const std::size_t budget = opt.progress ? kOneShotBudget : 0;
  while (!builder.step(budget)) {
  }
  return builder.take();
}

} // namespace cvc
