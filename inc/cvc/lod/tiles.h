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

// tiles.h -- cut a multi-part model into ground-plane tiles and build one LOD
// pyramid per tile, fanned over a thread pool.
//
// Why tiles: select_rung places a rung by the camera's distance to the asset's
// bounds, so ONE merged mesh spanning a whole scene never leaves rung 0 -- the
// camera is always inside (or next to) its box. Partitioning the scene into
// cells and building an independent pyramid per cell lets far cells coarsen
// while near cells stay at full detail.
//
// The partition rule. Parts are first gathered into GROUPS by a caller-supplied
// key (default: the part name, so every part is its own group); a caller maps
// e.g. "<x>_walls" and "<x>_roof" to "<x>" so the parts of one object travel
// together. Each group is then placed WHOLE into the cell containing the centroid
// of its bounding box, projected onto the ground plane (the two axes other than
// partition_params::up_axis; Z-up => the XY plane). An object is therefore never
// split across tiles: one larger than its cell overhangs it, and the tile's
// bounds grow to contain it -- the same conservative rule cvc::vis::tile_triangles
// applies per triangle. (Assigning per triangle instead would cut objects along
// every cell border.) Cells are anchored at partition_params::origin, not at the
// scene's extent, so a cell's index is a pure function of world position: the
// same object lands in the same cell whatever else is in the scene.
//
// Determinism. Tiles come out in row-major cell order (cell_index::operator<),
// empty cells produce no tile, and a tile's merged geometry lists its groups in
// key order and each group's parts in name order -- so for uniquely-named parts
// the output is independent of the input part order and of the pool. Every tile
// also carries a canonical content_hash (see below) that is independent of part
// order, vertex order and triangle order, for cache validation (a baked pyramid
// is reused only when the hash of the tile it was built from still matches).
//
// Like the rest of cvc::lod this is hermetic (no VTK/GL/I/O) and a render proxy
// only: nav/material/RF paths keep reading the untiled source.

#ifndef __CVC_LOD_TILES_H__
#define __CVC_LOD_TILES_H__

#include <cstddef>
#include <cstdint>
#include <cvc/geometry/geometry.h>
#include <cvc/lod/pyramid.h>
#include <cvc/model/model.h>
#include <cvc/volume/bounding_box.h>
#include <functional>
#include <string>
#include <vector>

namespace cvc {
class thread_pool;

namespace lod {

// Maps a part name to its group key. Parts with equal keys form one group and
// always land in the same tile. An empty (default-constructed) function means
// identity: every part is its own group.
typedef std::function<std::string(const std::string &)> group_key_fn;

// A group_key_fn that strips the FIRST matching suffix from a part name
// ("b7_walls" -> "b7" for suffixes {"_walls", "_roof"}); a name matching none of
// them (or equal to a suffix) is its own key. Suffixes are tried in order.
group_key_fn suffix_group_key(const std::vector<std::string> &suffixes);

// One input part: a name plus a NON-owning pointer to its geometry, which must
// outlive the partition call (the tiles copy what they need).
struct named_part {
  std::string name;
  const geometry *geom = nullptr;

  named_part() {}
  named_part(const std::string &n, const geometry &g) : name(n), geom(&g) {}
  named_part(const std::string &, geometry &&) = delete; // would dangle
};

// Integer cell coordinates on the ground plane: i along the lower-numbered
// ground axis, j along the higher one (Z-up: i = x, j = y), each
// floor((centroid - origin) / cell_m). Ordered row-major (j, then i).
struct cell_index {
  std::int64_t i = 0;
  std::int64_t j = 0;

  bool operator==(const cell_index &o) const { return i == o.i && j == o.j; }
  bool operator!=(const cell_index &o) const { return !(*this == o); }
  bool operator<(const cell_index &o) const { return j != o.j ? j < o.j : i < o.i; }
};

struct partition_params {
  // World up axis (0 = X, 1 = Y, 2 = Z); cells span the other two. glTF scenes
  // are Y-up (1); cvc scenes and terrain are Z-up (2, the default).
  int up_axis = 2;
  // Grid anchor: cell (0,0) spans [origin, origin + cell_m) on both ground axes.
  // The up-axis component is ignored.
  geometry::point_t origin = {{0.0, 0.0, 0.0}};
  // Position lattice of the content hash (and of partition_components' coincident
  // vertex test), in world units. 1 mm for a metre-scale scene.
  double hash_quantum_m = 0.001;
  // partition_components only: also connect vertices whose positions fall on
  // the same hash_quantum_m lattice point, so unwelded parts of one object (a
  // roof whose rim duplicates the walls' top vertices) stay one component.
  bool connect_coincident = true;
};

// One non-empty cell of a partition.
//
// `geom` is a SURFACE_TRI merge of the tile's groups: quads are fan-split
// (0,1,2),(0,2,3) as cvc::simplify does, triangles with an out-of-range index are
// dropped, only vertices referenced by a kept triangle are carried, and indices
// are remapped. Per-vertex normals, colours, uvs and tangents are carried when a
// part has them aligned with its points; if only some parts of a tile carry an
// attribute, the rest are padded with a neutral default (normal +Z, colour white,
// uv 0, tangent +X) exactly as model::merged() does, so every per-vertex array
// is either empty or num_points() long. Lines/tets/hexs and other attributes are
// not carried: a tile is a render proxy.
struct tile {
  cell_index cell;
  geometry geom;
  std::vector<std::string> parts; // names of the parts merged into geom, in merge order
  bounding_box bounds;            // world AABB of geom's points
  std::uint64_t content_hash = 0; // content_hash(geom, params.hash_quantum_m)
};

// Partition named parts into tiles of `cell_m` world units (metres for a
// metre-scale scene). Parts with no usable triangles (none, or only ones with an
// out-of-range index) contribute nothing and are not listed in any tile. Throws
// std::invalid_argument if cell_m is not finite and > 0, up_axis is not 0..2,
// hash_quantum_m is not finite and > 0, or a part has a null geometry pointer.
//
// `pool`, when given, fans the per-tile merge + hash over its workers; the
// result is identical either way.
std::vector<tile> partition_parts(const std::vector<named_part> &parts, double cell_m,
                                  const group_key_fn &group_key = group_key_fn(),
                                  const partition_params &params = partition_params(),
                                  thread_pool *pool = nullptr);

// partition_parts over a model's meshes, each named by model::mesh::name.
// Materials are not carried (a tile mixes meshes of any material).
std::vector<tile> partition_model(const model &m, double cell_m,
                                  const group_key_fn &group_key = group_key_fn(),
                                  const partition_params &params = partition_params(),
                                  thread_pool *pool = nullptr);

// Partition ONE already-merged geometry by connected component, under the same
// cell/centroid rule: each component is a group. Two triangles are connected when
// they share a vertex index (or, with params.connect_coincident, a vertex
// position on the hash lattice). Components are numbered by their first triangle
// in input order and named "component_<k>"; a tile merges its components in
// that order. Vertices no kept triangle references are dropped. Throws like
// partition_parts.
std::vector<tile> partition_components(const geometry &merged, double cell_m,
                                       const partition_params &params = partition_params(),
                                       thread_pool *pool = nullptr);

// A canonical 64-bit content hash of `g`'s triangle surface (the same triangle
// set a tile carries: tris, then fan-split quads, out-of-range ones dropped).
//
// Each triangle corner is keyed by its position quantized to `quantum_m`
// (llround(p / quantum_m)) followed by whichever of normal, colour, uv and
// tangent are present (aligned with the points), each quantized to 1e-6. A
// triangle is rotated -- never reflected, so winding still counts -- to start
// at its lexicographically smallest rotation, and the triangles are sorted; the
// hash is FNV-1a 64 over a header (attribute mask, triangle count) and that
// sorted key stream, fed as little-endian bytes. It therefore does NOT depend
// on vertex order, triangle order, the rotation a triangle starts at,
// unreferenced vertices, or the platform, but it DOES change when any
// position moves to another lattice point, a triangle flips, or a carried
// attribute changes. (A value sitting on a rounding boundary may still flip on
// sub-quantum noise; that is inherent to any lattice.) Throws
// std::invalid_argument if quantum_m is not finite and > 0.
std::uint64_t content_hash(const geometry &g, double quantum_m = 0.001);

// Called by build_tiled_pyramids as each tile's pyramid completes.
typedef std::function<void(std::size_t tile_index, const tile &t, const mesh_pyramid &pyr)>
    tile_done_fn;

// Build one mesh pyramid per tile (build_mesh_pyramid on tile.geom); the result
// has one entry per tile, in tile order.
//
// With a pool and more than one tile, the tiles fan over the pool (largest
// first, so a big tile does not start last) and each tile's rungs build serially
// on the thread that claimed it -- the pool is used at one level only. With a
// single tile the pool goes to build_mesh_pyramid instead. A null pool runs
// everything on the caller in tile order. Every pyramid is bit-identical to the
// serial result either way.
//
// `on_tile`, when set, fires exactly once per tile as it finishes, on the thread
// that built it (a pool worker or the caller). Calls are SERIALIZED by an
// internal mutex, so the callback never runs concurrently with itself and needs
// no locking of its own against other on_tile calls -- but it does run
// concurrently with the remaining builds, and in completion order (tile order
// only on the serial path). `pyr` is the final result slot, valid until this
// function returns; copy what outlives it. Keep the callback cheap (e.g. push
// the index onto a queue a render loop drains): while it runs, any other
// finishing builder waits on the mutex. An exception thrown by the callback or
// a build propagates to the caller after the fan-out joins, as
// thread_pool::parallel_for documents.
std::vector<mesh_pyramid> build_tiled_pyramids(const std::vector<tile> &tiles,
                                               const pyramid_params &params = pyramid_params(),
                                               thread_pool *pool = nullptr,
                                               const tile_done_fn &on_tile = tile_done_fn());

} // namespace lod
} // namespace cvc

#endif // __CVC_LOD_TILES_H__
