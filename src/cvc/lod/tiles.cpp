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

// tiles.cpp -- ground-plane tiling of multi-part models + pooled per-tile pyramids.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cvc/core/thread_pool.h>
#include <cvc/lod/tiles.h>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace lod {
namespace {

typedef geometry::index_t index_t;
typedef geometry::tri_t tri_t;

// Lattice step for the non-positional attributes in content_hash (unit normals,
// colours in [0,1], uvs, tangents).
const double k_attr_quantum = 1e-6;

// llround(v / q), saturated so a huge coordinate or a NaN attribute stays
// deterministic instead of overflowing (NaN maps to the minimum). Positions are
// always finite here: surface_tris drops triangles with a non-finite corner.
std::int64_t quantize(double v, double q) {
  const double s = v / q;
  const double lim = 9.2e18; // just inside +-2^63
  if (!(s > -lim))
    return std::numeric_limits<std::int64_t>::min();
  if (!(s < lim))
    return std::numeric_limits<std::int64_t>::max();
  return static_cast<std::int64_t>(std::llround(s));
}

// floor((c - origin) / cell_m) as an integer cell coordinate, saturated for the
// same reason (a finite centroid far outside the int64 range, or one whose
// computation overflowed to +-inf).
std::int64_t cell_coord(double c, double origin, double cell_m) {
  const double f = std::floor((c - origin) / cell_m);
  const double lim = 4.0e18;
  if (!(f > -lim))
    return -static_cast<std::int64_t>(lim);
  if (!(f < lim))
    return static_cast<std::int64_t>(lim);
  return static_cast<std::int64_t>(f);
}

void validate(double cell_m, const partition_params &p) {
  if (!(std::isfinite(cell_m) && cell_m > 0.0))
    throw std::invalid_argument("cvc::lod: tile cell size must be finite and > 0");
  if (p.up_axis < 0 || p.up_axis > 2)
    throw std::invalid_argument("cvc::lod: partition_params::up_axis must be 0, 1 or 2");
  if (!(std::isfinite(p.hash_quantum_m) && p.hash_quantum_m > 0.0))
    throw std::invalid_argument(
        "cvc::lod: partition_params::hash_quantum_m must be finite and > 0");
  for (int a = 0; a < 3; ++a)
    if (a != p.up_axis && !std::isfinite(p.origin[a]))
      throw std::invalid_argument("cvc::lod: partition_params::origin must be finite");
}

// The two ground axes for an up axis, in increasing order.
void ground_axes(int up, int &a0, int &a1) {
  a0 = up == 0 ? 1 : 0;
  a1 = up == 2 ? 1 : 2;
}

// The triangle surface the tiler works on: tris, then quads fan-split
// (0,1,2),(0,2,3) as cvc::simplify does, dropping any triangle with an
// out-of-range vertex index or a non-finite (NaN/inf) corner position. A
// malformed part loses a triangle, not the tile; a non-finite corner has no cell
// to land in and would otherwise escape the tile's bounds while still being carried.
std::vector<tri_t> surface_tris(const geometry &g) {
  const geometry::points_t &P = g.const_points();
  const index_t n = P.size();
  const auto good = [&P, n](index_t v) {
    return v < n && std::isfinite(P[v][0]) && std::isfinite(P[v][1]) && std::isfinite(P[v][2]);
  };
  const auto ok = [&good](index_t a, index_t b, index_t c) {
    return good(a) && good(b) && good(c);
  };
  std::vector<tri_t> out;
  out.reserve(g.num_tris() + 2 * g.num_quads());
  for (const tri_t &t : g.const_tris())
    if (ok(t[0], t[1], t[2]))
      out.push_back(t);
  for (const geometry::quad_t &q : g.const_quads()) {
    if (ok(q[0], q[1], q[2]))
      out.push_back({{q[0], q[1], q[2]}});
    if (ok(q[0], q[2], q[3]))
      out.push_back({{q[0], q[2], q[3]}});
  }
  return out;
}

// Which per-vertex attributes a geometry carries aligned with its points.
struct attr_mask {
  bool normals = false, colors = false, uvs = false, tangents = false;

  attr_mask &operator|=(const attr_mask &o) {
    normals = normals || o.normals;
    colors = colors || o.colors;
    uvs = uvs || o.uvs;
    tangents = tangents || o.tangents;
    return *this;
  }
};

attr_mask attrs_of(const geometry &g) {
  attr_mask m;
  const std::size_t np = g.const_points().size();
  if (np == 0)
    return m;
  m.normals = g.const_normals().size() == np;
  m.colors = g.const_colors().size() == np;
  m.uvs = g.const_uvs().size() == np;
  m.tangents = g.const_tangents().size() == np;
  return m;
}

// One partition input: a whole named part, or one connected component of a
// merged geometry. `tris` is non-empty and every index is valid for *g.
struct chunk {
  const geometry *g = nullptr;
  std::string name;
  std::vector<tri_t> tris;
  double lo[3], hi[3]; // AABB of the vertices `tris` reference
};

void chunk_bounds(chunk &c) {
  const geometry::points_t &P = c.g->const_points();
  for (int a = 0; a < 3; ++a) {
    c.lo[a] = std::numeric_limits<double>::max();
    c.hi[a] = -std::numeric_limits<double>::max();
  }
  for (const tri_t &t : c.tris)
    for (int k = 0; k < 3; ++k)
      for (int a = 0; a < 3; ++a) {
        c.lo[a] = std::min(c.lo[a], P[t[k]][a]);
        c.hi[a] = std::max(c.hi[a], P[t[k]][a]);
      }
}

// A set of chunks placed as one unit, in merge order.
struct group {
  std::vector<std::size_t> chunks;
  double lo[3], hi[3];
};

group make_group(const std::vector<chunk> &chunks, std::vector<std::size_t> members) {
  group g;
  g.chunks = std::move(members);
  for (int a = 0; a < 3; ++a) {
    g.lo[a] = std::numeric_limits<double>::max();
    g.hi[a] = -std::numeric_limits<double>::max();
  }
  for (std::size_t ci : g.chunks)
    for (int a = 0; a < 3; ++a) {
      g.lo[a] = std::min(g.lo[a], chunks[ci].lo[a]);
      g.hi[a] = std::max(g.hi[a], chunks[ci].hi[a]);
    }
  return g;
}

// A tile's arrays while it is being merged; moved into the tile geometry at the end.
struct tile_arrays {
  geometry::points_t points;
  geometry::normals_t normals;
  geometry::colors_t colors;
  geometry::uvs_t uvs;
  geometry::tangents_t tangents;
  geometry::tris_t tris;
};

// Append chunk `c` to `a`, whose attribute arrays follow mask `m`: the vertices
// c's triangles reference, in ascending source-index order (attributes the source
// lacks padded with model::merged()'s neutral defaults), then c's triangles
// remapped onto them.
void append_chunk(const chunk &c, const attr_mask &m, tile_arrays &a) {
  const geometry &g = *c.g;
  const attr_mask have = attrs_of(g);
  const geometry::normal_t def_n = {{0.0, 0.0, 1.0}};
  const geometry::color_t def_c = {{1.0, 1.0, 1.0}};
  const geometry::uv_t def_uv = {{0.0, 0.0}};
  const geometry::tangent_t def_t = {{1.0, 0.0, 0.0, 1.0}};

  std::vector<index_t> ids;
  ids.reserve(3 * c.tris.size());
  for (const tri_t &t : c.tris)
    ids.insert(ids.end(), t.begin(), t.end());
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());

  const index_t base = a.points.size();
  for (index_t v : ids) {
    a.points.push_back(g.const_points()[v]);
    if (m.normals)
      a.normals.push_back(have.normals ? g.const_normals()[v] : def_n);
    if (m.colors)
      a.colors.push_back(have.colors ? g.const_colors()[v] : def_c);
    if (m.uvs)
      a.uvs.push_back(have.uvs ? g.const_uvs()[v] : def_uv);
    if (m.tangents)
      a.tangents.push_back(have.tangents ? g.const_tangents()[v] : def_t);
  }
  for (const tri_t &t : c.tris) {
    tri_t r;
    for (int k = 0; k < 3; ++k)
      r[k] = base + index_t(std::lower_bound(ids.begin(), ids.end(), t[k]) - ids.begin());
    a.tris.push_back(r);
  }
}

// Merge the chunks of groups `gids` (in order) into `out`, then fill its bounds
// and content hash. Touches nothing but `out`, so tiles build in parallel.
void build_tile(tile &out, const std::vector<chunk> &chunks, const std::vector<group> &groups,
                const std::vector<std::size_t> &gids, const partition_params &p) {
  attr_mask m;
  for (std::size_t gi : gids)
    for (std::size_t ci : groups[gi].chunks)
      m |= attrs_of(*chunks[ci].g);

  tile_arrays a;
  for (std::size_t gi : gids)
    for (std::size_t ci : groups[gi].chunks) {
      append_chunk(chunks[ci], m, a);
      out.parts.push_back(chunks[ci].name);
    }

  const chunk &first = chunks[groups[gids.front()].chunks.front()];
  geometry g(first.g->ctx());
  g.points().swap(a.points);
  if (m.normals)
    g.normals().swap(a.normals);
  if (m.colors)
    g.colors().swap(a.colors);
  if (m.uvs)
    g.uvs().swap(a.uvs);
  if (m.tangents)
    g.tangents().swap(a.tangents);
  g.tris().swap(a.tris);
  g.set_geometry_type(geometry::SURFACE_TRI);

  double lo[3], hi[3];
  for (int k = 0; k < 3; ++k) {
    lo[k] = std::numeric_limits<double>::max();
    hi[k] = -std::numeric_limits<double>::max();
  }
  for (const geometry::point_t &q : g.const_points())
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], q[k]);
      hi[k] = std::max(hi[k], q[k]);
    }
  out.bounds = bounding_box(lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]);
  out.content_hash = content_hash(g, p.hash_quantum_m);
  out.geom = g;
}

// Place each group (in rank order) into its cell, then merge the non-empty cells
// in row-major order -- fanned over `pool` when there is more than one.
std::vector<tile> assemble(const std::vector<chunk> &chunks, const std::vector<group> &groups,
                           double cell_m, const partition_params &p, thread_pool *pool) {
  int a0 = 0, a1 = 1;
  ground_axes(p.up_axis, a0, a1);
  std::map<cell_index, std::vector<std::size_t>> cells;
  for (std::size_t gi = 0; gi < groups.size(); ++gi) {
    const group &gr = groups[gi];
    cell_index c;
    c.i = cell_coord(0.5 * (gr.lo[a0] + gr.hi[a0]), p.origin[a0], cell_m);
    c.j = cell_coord(0.5 * (gr.lo[a1] + gr.hi[a1]), p.origin[a1], cell_m);
    cells[c].push_back(gi);
  }

  std::vector<tile> tiles(cells.size());
  std::vector<const std::vector<std::size_t> *> members;
  members.reserve(cells.size());
  for (const auto &kv : cells) {
    tiles[members.size()].cell = kv.first;
    members.push_back(&kv.second);
  }
  const int n = int(tiles.size());
  const auto build = [&](int k) { build_tile(tiles[k], chunks, groups, *members[k], p); };
  if (pool && n > 1)
    pool->parallel_for(n, build);
  else
    for (int k = 0; k < n; ++k)
      build(k);
  return tiles;
}

// Minimal FNV-1a 64, fed whole 64-bit words as little-endian bytes so the value
// is the same on every platform.
struct fnv1a64 {
  std::uint64_t h = 14695981039346656037ull;

  void word(std::uint64_t w) {
    for (int b = 0; b < 8; ++b) {
      h ^= (w >> (8 * b)) & 0xffu;
      h *= 1099511628211ull;
    }
  }
};

} // namespace

group_key_fn suffix_group_key(const std::vector<std::string> &suffixes) {
  return [suffixes](const std::string &name) -> std::string {
    for (const std::string &s : suffixes)
      if (!s.empty() && name.size() > s.size() &&
          name.compare(name.size() - s.size(), s.size(), s) == 0)
        return name.substr(0, name.size() - s.size());
    return name;
  };
}

std::vector<tile> partition_parts(const std::vector<named_part> &parts, double cell_m,
                                  const group_key_fn &group_key, const partition_params &params,
                                  thread_pool *pool) {
  validate(cell_m, params);
  for (const named_part &np : parts)
    if (!np.geom)
      throw std::invalid_argument("cvc::lod::partition_parts: part '" + np.name +
                                  "' has no geometry");

  std::vector<chunk> chunks;
  std::map<std::string, std::vector<std::size_t>> by_key; // std::map => groups in key order
  for (const named_part &np : parts) {
    chunk c;
    c.g = np.geom;
    c.name = np.name;
    c.tris = surface_tris(*np.geom);
    if (c.tris.empty())
      continue; // nothing to render or simplify
    chunk_bounds(c);
    by_key[group_key ? group_key(np.name) : np.name].push_back(chunks.size());
    chunks.push_back(std::move(c));
  }

  std::vector<group> groups;
  groups.reserve(by_key.size());
  for (auto &kv : by_key) {
    // Parts in name order (input order only among equal names), so the merge
    // does not depend on the order the caller listed the parts in.
    std::stable_sort(kv.second.begin(), kv.second.end(),
                     [&](std::size_t x, std::size_t y) { return chunks[x].name < chunks[y].name; });
    groups.push_back(make_group(chunks, std::move(kv.second)));
  }
  return assemble(chunks, groups, cell_m, params, pool);
}

std::vector<tile> partition_model(const model &m, double cell_m, const group_key_fn &group_key,
                                  const partition_params &params, thread_pool *pool) {
  std::vector<named_part> parts;
  parts.reserve(m.meshes.size());
  for (const model::mesh &mm : m.meshes)
    parts.push_back(named_part(mm.name, mm.geom));
  return partition_parts(parts, cell_m, group_key, params, pool);
}

std::vector<tile> partition_components(const geometry &merged, double cell_m,
                                       const partition_params &params, thread_pool *pool) {
  validate(cell_m, params);
  const std::vector<tri_t> tris = surface_tris(merged);
  const index_t nv = merged.num_points();

  // Union-find over vertices; the smaller index always becomes the root, so the
  // forest (and hence the numbering below) is a pure function of the input.
  std::vector<index_t> parent(nv);
  std::iota(parent.begin(), parent.end(), index_t(0));
  const auto root_of = [&parent](index_t v) {
    while (parent[v] != v) {
      parent[v] = parent[parent[v]];
      v = parent[v];
    }
    return v;
  };
  const auto unite = [&](index_t x, index_t y) {
    x = root_of(x);
    y = root_of(y);
    if (x == y)
      return;
    if (y < x)
      std::swap(x, y);
    parent[y] = x;
  };
  for (const tri_t &t : tris) {
    unite(t[0], t[1]);
    unite(t[1], t[2]);
  }

  if (params.connect_coincident) {
    // Sort the referenced vertices by lattice position and join equal runs.
    std::vector<char> used(nv, 0);
    for (const tri_t &t : tris)
      for (int k = 0; k < 3; ++k)
        used[t[k]] = 1;
    typedef std::pair<std::array<std::int64_t, 3>, index_t> keyed;
    std::vector<keyed> keys;
    const geometry::points_t &P = merged.const_points();
    for (index_t v = 0; v < nv; ++v)
      if (used[v])
        keys.push_back(keyed(
            {{quantize(P[v][0], params.hash_quantum_m), quantize(P[v][1], params.hash_quantum_m),
              quantize(P[v][2], params.hash_quantum_m)}},
            v));
    std::sort(keys.begin(), keys.end());
    for (std::size_t k = 1; k < keys.size(); ++k)
      if (keys[k].first == keys[k - 1].first)
        unite(keys[k - 1].second, keys[k].second);
  }

  // Number components by their first triangle; each is one chunk and one group.
  const std::size_t none = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> comp_of_root(nv, none);
  std::vector<chunk> chunks;
  for (const tri_t &t : tris) {
    const index_t r = root_of(t[0]);
    if (comp_of_root[r] == none) {
      comp_of_root[r] = chunks.size();
      chunks.emplace_back();
      chunks.back().g = &merged;
      chunks.back().name = "component_" + std::to_string(chunks.size() - 1);
    }
    chunks[comp_of_root[r]].tris.push_back(t);
  }
  std::vector<group> groups;
  groups.reserve(chunks.size());
  for (std::size_t ci = 0; ci < chunks.size(); ++ci) {
    chunk_bounds(chunks[ci]);
    groups.push_back(make_group(chunks, std::vector<std::size_t>(1, ci)));
  }
  return assemble(chunks, groups, cell_m, params, pool);
}

std::uint64_t content_hash(const geometry &g, double quantum_m) {
  if (!(std::isfinite(quantum_m) && quantum_m > 0.0))
    throw std::invalid_argument("cvc::lod::content_hash: quantum_m must be finite and > 0");

  const std::vector<tri_t> tris = surface_tris(g);
  const attr_mask m = attrs_of(g);
  const std::size_t W =
      3 + (m.normals ? 3 : 0) + (m.colors ? 3 : 0) + (m.uvs ? 2 : 0) + (m.tangents ? 4 : 0);

  // One quantized key row of W values per vertex.
  const std::size_t nv = g.const_points().size();
  std::vector<std::int64_t> key(nv * W);
  for (std::size_t v = 0; v < nv; ++v) {
    std::int64_t *r = &key[v * W];
    for (int a = 0; a < 3; ++a)
      *r++ = quantize(g.const_points()[v][a], quantum_m);
    if (m.normals)
      for (int a = 0; a < 3; ++a)
        *r++ = quantize(g.const_normals()[v][a], k_attr_quantum);
    if (m.colors)
      for (int a = 0; a < 3; ++a)
        *r++ = quantize(g.const_colors()[v][a], k_attr_quantum);
    if (m.uvs)
      for (int a = 0; a < 2; ++a)
        *r++ = quantize(g.const_uvs()[v][a], k_attr_quantum);
    if (m.tangents)
      for (int a = 0; a < 4; ++a)
        *r++ = quantize(g.const_tangents()[v][a], k_attr_quantum);
  }

  // Lexicographic comparison of two triangles' corner-key sequences.
  const auto cmp = [&](const tri_t &x, const tri_t &y) {
    for (int k = 0; k < 3; ++k) {
      const std::int64_t *rx = &key[x[k] * W];
      const std::int64_t *ry = &key[y[k] * W];
      for (std::size_t w = 0; w < W; ++w)
        if (rx[w] != ry[w])
          return rx[w] < ry[w] ? -1 : 1;
    }
    return 0;
  };

  // Rotate each triangle to its smallest rotation (winding preserved), then sort.
  std::vector<tri_t> canon;
  canon.reserve(tris.size());
  for (const tri_t &t : tris) {
    tri_t best = t;
    for (int s = 1; s < 3; ++s) {
      const tri_t rot = {{t[s], t[(s + 1) % 3], t[(s + 2) % 3]}};
      if (cmp(rot, best) < 0)
        best = rot;
    }
    canon.push_back(best);
  }
  std::sort(canon.begin(), canon.end(),
            [&](const tri_t &x, const tri_t &y) { return cmp(x, y) < 0; });

  fnv1a64 h;
  h.word((m.normals ? 1u : 0u) | (m.colors ? 2u : 0u) | (m.uvs ? 4u : 0u) | (m.tangents ? 8u : 0u));
  h.word(std::uint64_t(canon.size()));
  for (const tri_t &t : canon)
    for (int k = 0; k < 3; ++k)
      for (std::size_t w = 0; w < W; ++w)
        h.word(static_cast<std::uint64_t>(key[t[k] * W + w]));
  return h.h;
}

std::vector<mesh_pyramid> build_tiled_pyramids(const std::vector<tile> &tiles,
                                               const pyramid_params &params, thread_pool *pool,
                                               const tile_done_fn &on_tile) {
  const int n = int(tiles.size());
  std::vector<mesh_pyramid> out(tiles.size());
  if (n == 0)
    return out;

  std::mutex cb_mtx;
  const bool fan = pool && n > 1;
  // Use the pool at one level only: across tiles when there are several, else
  // inside the single tile's pyramid (build_mesh_pyramid is identical either way).
  thread_pool *inner = fan ? nullptr : pool;
  const auto build = [&](int t) {
    out[t] = build_mesh_pyramid(tiles[t].geom, params, inner);
    if (on_tile) {
      std::lock_guard<std::mutex> lock(cb_mtx);
      on_tile(std::size_t(t), tiles[t], out[t]);
    }
  };

  if (!fan) {
    for (int t = 0; t < n; ++t)
      build(t);
    return out;
  }
  // Largest tile first (longest-processing-time order): the pool's shared cursor
  // then self-balances the small tail instead of waiting on a big tile that
  // happened to be claimed last. Only the schedule changes; each slot's content
  // does not.
  std::vector<int> order(n);
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int x, int y) {
    return tiles[x].geom.num_tris() > tiles[y].geom.num_tris();
  });
  pool->parallel_for(n, [&](int k) { build(order[k]); });
  return out;
}

} // namespace lod
} // namespace cvc
