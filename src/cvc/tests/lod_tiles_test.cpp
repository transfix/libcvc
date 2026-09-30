// lod_tiles_test -- spatial tiling of multi-part models + pooled per-tile pyramids
// (cvc::lod::partition_parts / partition_model / partition_components /
// content_hash / build_tiled_pyramids).
//
// Everything runs on a synthetic city: a grid of buildings, each a subdivided
// wall ring plus a separate (unwelded) roof, with some buildings straddling cell
// borders, a block of empty cells, and one building at negative coordinates.
//
// Covers: every group lands whole in exactly one tile, triangle and vertex counts
// are conserved and empty cells produce no tile; the content hash is stable
// across input permutation, repeat runs, vertex/triangle order and triangle
// rotation but changes with geometry, winding and attributes; pooled == serial
// for both the partition and the pyramids (bit-identical); the progress callback
// fires once per tile and never concurrently; the connected-component overload
// matches the named-part path; and the edge cases (no parts, triangle-less
// parts, bad cell sizes, a huge cell, Y-up, quads, bad indices).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/thread_pool.h>
#include <cvc/geometry/geometry.h>
#include <cvc/lod/pyramid.h>
#include <cvc/lod/tiles.h>
#include <cvc/model/model.h>
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

using cvc::geometry;
namespace lod = cvc::lod;

namespace {

const double kCell = 100.0;

double lerp_at(double a, double b, int i, int n) { return a + (b - a) * double(i) / double(n); }

struct building {
  std::string name; // group key; parts are name + "_walls" / "_roof"
  double x0, y0, x1, y1, h;
  geometry walls, roof;
};

struct city_opts {
  int sub = 4;              // subdivisions per wall side / roof edge
  bool mixed_attrs = true;  // roofs carry uvs, some walls carry colours
  bool with_normals = true; // every part carries normals
};

// One building over footprint [x0,x1]x[y0,y1], height h. The walls are a welded
// ring (one connected piece, 8*sub^2 triangles); the roof is a separate sub x sub
// grid (2*sub^2 triangles) whose rim positions are bitwise equal to the walls' top
// ring, so the two touch without sharing indices (unwelded, as exported meshes
// usually are).
building make_building(cvc::app &ctx, const std::string &name, double x0, double y0, double x1,
                       double y1, double h, const city_opts &o, bool colour_walls) {
  building b;
  b.name = name;
  b.x0 = x0;
  b.y0 = y0;
  b.x1 = lerp_at(x0, x1, o.sub, o.sub); // the exact value the grids produce
  b.y1 = lerp_at(y0, y1, o.sub, o.sub);
  b.h = h;
  const int S = o.sub, P = 4 * S;
  const auto gx = [&](int i) { return lerp_at(x0, x1, i, S); };
  const auto gy = [&](int j) { return lerp_at(y0, y1, j, S); };
  const double cx = 0.5 * (x0 + x1), cy = 0.5 * (y0 + y1);

  // Perimeter point p in [0, 4S) as roof-grid indices, counter-clockwise.
  const auto perim = [&](int p, int &i, int &j) {
    const int side = p / S, t = p % S;
    if (side == 0) {
      i = t;
      j = 0;
    } else if (side == 1) {
      i = S;
      j = t;
    } else if (side == 2) {
      i = S - t;
      j = S;
    } else {
      i = 0;
      j = S - t;
    }
  };

  b.walls = geometry(ctx);
  for (int l = 0; l <= S; ++l)
    for (int p = 0; p < P; ++p) {
      int i = 0, j = 0;
      perim(p, i, j);
      b.walls.points().push_back({{gx(i), gy(j), lerp_at(0.0, h, l, S)}});
      if (o.with_normals) {
        const double nx = gx(i) - cx, ny = gy(j) - cy, len = std::sqrt(nx * nx + ny * ny);
        b.walls.normals().push_back({{nx / len, ny / len, 0.0}});
      }
      if (colour_walls)
        b.walls.colors().push_back({{0.8, 0.3, 0.1}});
    }
  for (int l = 0; l < S; ++l)
    for (int p = 0; p < P; ++p) {
      const std::uint64_t a = std::uint64_t(l * P + p), bb = std::uint64_t(l * P + (p + 1) % P);
      const std::uint64_t c = bb + P, d = a + P;
      b.walls.tris().push_back({{a, bb, c}});
      b.walls.tris().push_back({{a, c, d}});
    }

  b.roof = geometry(ctx);
  const double top = lerp_at(0.0, h, S, S);
  for (int j = 0; j <= S; ++j)
    for (int i = 0; i <= S; ++i) {
      b.roof.points().push_back({{gx(i), gy(j), top}});
      if (o.with_normals)
        b.roof.normals().push_back({{0.0, 0.0, 1.0}});
      if (o.mixed_attrs)
        b.roof.uvs().push_back({{double(i) / S, double(j) / S}});
    }
  const auto id = [S](int i, int j) { return std::uint64_t(j * (S + 1) + i); };
  for (int j = 0; j < S; ++j)
    for (int i = 0; i < S; ++i) {
      b.roof.tris().push_back({{id(i, j), id(i + 1, j), id(i + 1, j + 1)}});
      b.roof.tris().push_back({{id(i, j), id(i + 1, j + 1), id(i, j + 1)}});
    }
  return b;
}

// A 6x6 grid of buildings on a 45 m pitch (centres 10, 55, 100, 145, 190, 235),
// 24 m wide, so the ones centred on 100 and 190 straddle a 100 m cell border;
// the 3x3 block centred on cell (1,1) is left out so that cell is empty; plus one
// building at negative x (cell -1).
std::vector<building> make_city(cvc::app &ctx, const city_opts &o = city_opts()) {
  std::vector<building> city;
  for (int j = 0; j < 6; ++j)
    for (int i = 0; i < 6; ++i) {
      if (i >= 2 && i <= 4 && j >= 2 && j <= 4)
        continue;
      const double cx = 10.0 + 45.0 * i, cy = 10.0 + 45.0 * j;
      city.push_back(make_building(ctx, "b" + std::to_string(i) + "_" + std::to_string(j),
                                   cx - 12.0, cy - 12.0, cx + 12.0, cy + 12.0, 8.0 + i + j, o,
                                   o.mixed_attrs && (i + j) % 3 == 0));
    }
  city.push_back(make_building(ctx, "neg", -42.0, 4.0, -18.0, 28.0, 12.0, o, false));
  return city;
}

// The city as named parts, walls before roof per building.
std::vector<lod::named_part> parts_of(const std::vector<building> &city) {
  std::vector<lod::named_part> parts;
  for (const building &b : city) {
    parts.push_back(lod::named_part(b.name + "_walls", b.walls));
    parts.push_back(lod::named_part(b.name + "_roof", b.roof));
  }
  return parts;
}

// A named_part only points at its geometry, so it must not bind a temporary.
static_assert(!std::is_constructible<lod::named_part, const std::string &, geometry &&>::value,
              "named_part must reject a temporary geometry");

lod::group_key_fn building_key() { return lod::suffix_group_key({"_walls", "_roof"}); }

lod::cell_index expected_cell(const building &b) {
  lod::cell_index c;
  c.i = std::int64_t(std::floor(0.5 * (b.x0 + b.x1) / kCell));
  c.j = std::int64_t(std::floor(0.5 * (b.y0 + b.y1) / kCell));
  return c;
}

template <class A> bool bytes_equal(const std::vector<A> &x, const std::vector<A> &y) {
  return x.size() == y.size() &&
         (x.empty() || std::memcmp(x.data(), y.data(), x.size() * sizeof(A)) == 0);
}

// Bitwise equality of the arrays a tile or rung carries.
void expect_geometry_identical(const geometry &x, const geometry &y, const std::string &what) {
  EXPECT_TRUE(bytes_equal(x.const_points(), y.const_points())) << what << ": points";
  EXPECT_TRUE(bytes_equal(x.const_tris(), y.const_tris())) << what << ": tris";
  EXPECT_TRUE(bytes_equal(x.const_normals(), y.const_normals())) << what << ": normals";
  EXPECT_TRUE(bytes_equal(x.const_colors(), y.const_colors())) << what << ": colors";
  EXPECT_TRUE(bytes_equal(x.const_uvs(), y.const_uvs())) << what << ": uvs";
  EXPECT_TRUE(bytes_equal(x.const_tangents(), y.const_tangents())) << what << ": tangents";
}

void expect_tiles_identical(const std::vector<lod::tile> &x, const std::vector<lod::tile> &y) {
  ASSERT_EQ(x.size(), y.size());
  for (std::size_t k = 0; k < x.size(); ++k) {
    EXPECT_EQ(x[k].cell, y[k].cell) << "tile " << k;
    EXPECT_EQ(x[k].parts, y[k].parts) << "tile " << k;
    EXPECT_EQ(x[k].content_hash, y[k].content_hash) << "tile " << k;
    expect_geometry_identical(x[k].geom, y[k].geom, "tile " + std::to_string(k));
  }
}

geometry one_triangle(cvc::app &ctx, double x0, double x1) {
  geometry g(ctx);
  g.points().push_back({{x0, 0.0, 0.0}});
  g.points().push_back({{x1, 0.0, 0.0}});
  g.points().push_back({{x0, 5.0, 0.0}});
  g.tris().push_back({{0, 1, 2}});
  return g;
}

} // namespace

TEST(LodTiles, SuffixGroupKey) {
  const lod::group_key_fn key = lod::suffix_group_key({"", "_walls", "_roof"});
  EXPECT_EQ(key("b1_walls"), "b1");
  EXPECT_EQ(key("b1_roof"), "b1");
  EXPECT_EQ(key("b1"), "b1");
  EXPECT_EQ(key("_walls"), "_walls"); // a bare suffix is its own key
  EXPECT_EQ(key("roof_walls_roof"), "roof_walls");
}

TEST(LodTiles, EveryGroupLandsWholeInExactlyOneTile) {
  cvc::app ctx;
  const std::vector<building> city = make_city(ctx);
  const std::vector<lod::named_part> parts = parts_of(city);
  const std::vector<lod::tile> tiles = lod::partition_parts(parts, kCell, building_key());

  // Expected occupancy, computed independently from the building footprints.
  std::map<lod::cell_index, std::set<std::string>> want;
  for (const building &b : city)
    want[expected_cell(b)].insert(b.name);
  ASSERT_EQ(tiles.size(), want.size());
  EXPECT_EQ(want.count(lod::cell_index()), 1u);
  lod::cell_index empty_cell;
  empty_cell.i = 1;
  empty_cell.j = 1;
  EXPECT_EQ(want.count(empty_cell), 0u); // the left-out block

  std::map<std::string, int> seen; // part name -> tiles containing it
  std::uint64_t tris = 0, verts = 0;
  for (std::size_t k = 0; k < tiles.size(); ++k) {
    const lod::tile &t = tiles[k];
    if (k > 0)
      EXPECT_TRUE(tiles[k - 1].cell < t.cell) << "tiles must be in row-major cell order";
    EXPECT_NE(t.cell, empty_cell);
    ASSERT_FALSE(t.parts.empty());
    ASSERT_GT(t.geom.num_tris(), 0u);
    EXPECT_EQ(t.geom.get_geometry_type(), geometry::SURFACE_TRI);

    // Both parts of every building in this tile are here, and only the expected buildings.
    std::set<std::string> groups;
    for (const std::string &p : t.parts) {
      ++seen[p];
      groups.insert(p.substr(0, p.rfind('_')));
    }
    ASSERT_EQ(want.count(t.cell), 1u);
    EXPECT_EQ(groups, want[t.cell]);
    EXPECT_EQ(t.parts.size(), 2 * groups.size());

    // Attribute arrays are empty or vertex-aligned; bounds contain every point.
    EXPECT_EQ(t.geom.const_normals().size(), t.geom.num_points());
    EXPECT_EQ(t.geom.const_uvs().size(), t.geom.num_points());
    EXPECT_TRUE(t.geom.const_colors().empty() ||
                t.geom.const_colors().size() == t.geom.num_points());
    for (const geometry::point_t &p : t.geom.const_points()) {
      EXPECT_GE(p[0], t.bounds.minx);
      EXPECT_LE(p[0], t.bounds.maxx);
      EXPECT_GE(p[1], t.bounds.miny);
      EXPECT_LE(p[1], t.bounds.maxy);
    }
    for (const geometry::tri_t &f : t.geom.const_tris())
      for (int c = 0; c < 3; ++c)
        EXPECT_LT(f[c], t.geom.num_points());
    EXPECT_EQ(t.content_hash, lod::content_hash(t.geom));
    tris += t.geom.num_tris();
    verts += t.geom.num_points();
  }

  // Every part in exactly one tile; nothing lost or duplicated.
  EXPECT_EQ(seen.size(), parts.size());
  for (const auto &kv : seen)
    EXPECT_EQ(kv.second, 1) << kv.first;
  std::uint64_t src_tris = 0, src_verts = 0;
  for (const lod::named_part &p : parts) {
    src_tris += p.geom->num_tris();
    src_verts += p.geom->num_points();
  }
  EXPECT_EQ(tris, src_tris);
  EXPECT_EQ(verts, src_verts);
}

TEST(LodTiles, GroupIsNeverSplitAcrossACellBorder) {
  cvc::app ctx;
  city_opts o;
  // Walls over x in [180,226] (centroid 203 => cell 2), but the roof only covers
  // [180,196] (centroid 188 => cell 1 if it were placed on its own).
  building b = make_building(ctx, "tower", 180.0, 10.0, 226.0, 30.0, 20.0, o, false);
  building pent = make_building(ctx, "pent", 180.0, 10.0, 196.0, 30.0, 24.0, o, false);
  std::vector<lod::named_part> parts;
  parts.push_back(lod::named_part("tower_walls", b.walls));
  parts.push_back(lod::named_part("tower_roof", pent.roof));

  std::vector<lod::tile> grouped = lod::partition_parts(parts, kCell, building_key());
  ASSERT_EQ(grouped.size(), 1u);
  EXPECT_EQ(grouped[0].cell.i, 2);
  EXPECT_EQ(grouped[0].parts, (std::vector<std::string>{"tower_roof", "tower_walls"}));
  EXPECT_EQ(grouped[0].bounds.minx, 180.0);
  EXPECT_EQ(grouped[0].bounds.maxx, 226.0);

  // Without the key each part is its own group, and the two do land apart.
  std::vector<lod::tile> ungrouped = lod::partition_parts(parts, kCell);
  ASSERT_EQ(ungrouped.size(), 2u);
  EXPECT_EQ(ungrouped[0].cell.i, 1);
  EXPECT_EQ(ungrouped[0].parts, std::vector<std::string>{"tower_roof"});
  EXPECT_EQ(ungrouped[1].cell.i, 2);
}

TEST(LodTiles, AttributesArePaddedWithNeutralDefaults) {
  cvc::app ctx;
  city_opts o;
  building b = make_building(ctx, "b", 10.0, 10.0, 30.0, 30.0, 10.0, o, true);
  std::vector<lod::named_part> parts;
  parts.push_back(lod::named_part("b_walls", b.walls)); // normals + colours, no uvs
  parts.push_back(lod::named_part("b_roof", b.roof));   // normals + uvs, no colours
  const std::vector<lod::tile> tiles = lod::partition_parts(parts, kCell, building_key());
  ASSERT_EQ(tiles.size(), 1u);
  const geometry &g = tiles[0].geom;
  // Merge order is by name: the roof first, then the walls.
  const std::size_t nroof = b.roof.num_points();
  ASSERT_EQ(g.num_points(), nroof + b.walls.num_points());
  ASSERT_EQ(g.const_colors().size(), g.num_points());
  ASSERT_EQ(g.const_uvs().size(), g.num_points());
  for (std::size_t v = 0; v < nroof; ++v) {
    EXPECT_EQ(g.const_colors()[v], (geometry::color_t{{1.0, 1.0, 1.0}}));
    EXPECT_EQ(g.const_uvs()[v], b.roof.const_uvs()[v]);
  }
  for (std::size_t v = nroof; v < g.num_points(); ++v) {
    EXPECT_EQ(g.const_uvs()[v], (geometry::uv_t{{0.0, 0.0}}));
    EXPECT_EQ(g.const_colors()[v], b.walls.const_colors()[v - nroof]);
    EXPECT_EQ(g.const_normals()[v], b.walls.const_normals()[v - nroof]);
  }
  EXPECT_TRUE(g.const_tangents().empty());

  // A tangent basis on the roof alone pads the walls with the +X default, and
  // is part of the hashed content.
  geometry roof = b.roof;
  roof.tangents().assign(roof.num_points(), geometry::tangent_t{{0.0, 1.0, 0.0, -1.0}});
  parts[1] = lod::named_part("b_roof", roof);
  const std::vector<lod::tile> tt = lod::partition_parts(parts, kCell, building_key());
  ASSERT_EQ(tt.size(), 1u);
  const geometry &gt = tt[0].geom;
  ASSERT_EQ(gt.const_tangents().size(), gt.num_points());
  EXPECT_EQ(gt.const_tangents()[0], roof.const_tangents()[0]);
  EXPECT_EQ(gt.const_tangents()[nroof], (geometry::tangent_t{{1.0, 0.0, 0.0, 1.0}}));
  EXPECT_NE(tt[0].content_hash, tiles[0].content_hash);
}

TEST(LodTiles, HashIsDeterministicAcrossPermutationAndRuns) {
  cvc::app ctx;
  const std::vector<building> city = make_city(ctx);
  std::vector<lod::named_part> parts = parts_of(city);
  const std::vector<lod::tile> a = lod::partition_parts(parts, kCell, building_key());
  const std::vector<lod::tile> again = lod::partition_parts(parts, kCell, building_key());
  expect_tiles_identical(a, again);

  // Any input order yields the same tiles -- not just the same hashes: parts are
  // uniquely named, so the merge order itself is canonical.
  std::mt19937 rng(12345);
  for (int trial = 0; trial < 3; ++trial) {
    std::shuffle(parts.begin(), parts.end(), rng);
    expect_tiles_identical(a, lod::partition_parts(parts, kCell, building_key()));
  }

  std::set<std::uint64_t> distinct;
  for (const lod::tile &t : a)
    distinct.insert(t.content_hash);
  EXPECT_EQ(distinct.size(), a.size()) << "different tiles should not collide";
}

TEST(LodTiles, HashIgnoresOrderButTracksContent) {
  cvc::app ctx;
  city_opts o;
  const building b = make_building(ctx, "b", 12.0, 7.0, 30.0, 29.0, 9.0, o, true);
  const geometry &src = b.walls;
  const std::uint64_t h0 = lod::content_hash(src);
  EXPECT_EQ(h0, lod::content_hash(src)); // repeatable

  // Triangle order, triangle rotation, vertex order and unreferenced vertices
  // do not matter.
  {
    geometry g = src;
    std::reverse(g.tris().begin(), g.tris().end());
    for (geometry::tri_t &t : g.tris())
      t = geometry::tri_t{{t[1], t[2], t[0]}};
    EXPECT_EQ(lod::content_hash(g), h0);
  }
  {
    // Reverse the vertex order (and every attribute with it), remapping indices.
    geometry g(ctx);
    const std::size_t n = src.num_points();
    for (std::size_t v = n; v-- > 0;) {
      g.points().push_back(src.const_points()[v]);
      g.normals().push_back(src.const_normals()[v]);
      g.colors().push_back(src.const_colors()[v]);
    }
    for (const geometry::tri_t &t : src.const_tris())
      g.tris().push_back({{n - 1 - t[0], n - 1 - t[1], n - 1 - t[2]}});
    EXPECT_EQ(lod::content_hash(g), h0);
    g.points().push_back({{1e6, 1e6, 1e6}});
    g.normals().push_back({{0.0, 0.0, 1.0}});
    g.colors().push_back({{0.0, 0.0, 0.0}});
    EXPECT_EQ(lod::content_hash(g), h0);
  }
  // Sub-quantum noise on a millimetre-aligned vertex does not change the hash...
  {
    geometry g = src;
    g.points()[0][0] += 1e-5;
    EXPECT_EQ(lod::content_hash(g), h0);
  }
  // ...but a real move, a winding flip, or an attribute change does.
  {
    geometry g = src;
    g.points()[0][0] += 0.005;
    EXPECT_NE(lod::content_hash(g), h0);
  }
  {
    geometry g = src;
    std::swap(g.tris()[3][1], g.tris()[3][2]);
    EXPECT_NE(lod::content_hash(g), h0);
  }
  {
    geometry g = src;
    g.colors()[5][1] = 0.31;
    EXPECT_NE(lod::content_hash(g), h0);
  }
  {
    geometry g = src;
    g.colors().clear(); // the attribute set is part of the content
    EXPECT_NE(lod::content_hash(g), h0);
  }
  {
    geometry g = src;
    g.tris().pop_back();
    EXPECT_NE(lod::content_hash(g), h0);
  }
  // A coarser lattice absorbs the 5 mm move.
  {
    geometry g = src;
    g.points()[0][0] += 0.005;
    EXPECT_EQ(lod::content_hash(g, 0.1), lod::content_hash(src, 0.1));
  }

  // In a partition, editing one building changes only its own tile's hash.
  const std::vector<building> city = make_city(ctx);
  std::vector<building> edited = city;
  edited[3].roof.points()[0][2] += 0.25;
  const std::vector<lod::tile> a = lod::partition_parts(parts_of(city), kCell, building_key());
  const std::vector<lod::tile> e = lod::partition_parts(parts_of(edited), kCell, building_key());
  ASSERT_EQ(a.size(), e.size());
  int changed = 0;
  for (std::size_t k = 0; k < a.size(); ++k) {
    const bool has =
        std::find(a[k].parts.begin(), a[k].parts.end(), city[3].name + "_roof") != a[k].parts.end();
    EXPECT_EQ(a[k].content_hash != e[k].content_hash, has) << "tile " << k;
    changed += has ? 1 : 0;
  }
  EXPECT_EQ(changed, 1);
}

TEST(LodTiles, PooledPartitionMatchesSerial) {
  cvc::app ctx;
  const std::vector<building> city = make_city(ctx);
  const std::vector<lod::named_part> parts = parts_of(city);
  cvc::thread_pool pool(3);
  const std::vector<lod::tile> s = lod::partition_parts(parts, kCell, building_key());
  const std::vector<lod::tile> p =
      lod::partition_parts(parts, kCell, building_key(), lod::partition_params(), &pool);
  expect_tiles_identical(s, p);

  geometry merged(ctx);
  for (const lod::named_part &np : parts)
    merged.merge(*np.geom);
  expect_tiles_identical(lod::partition_components(merged, kCell),
                         lod::partition_components(merged, kCell, lod::partition_params(), &pool));
}

TEST(LodTiles, PooledPyramidsAreBitIdenticalToSerial) {
  cvc::app ctx;
  const std::vector<building> city = make_city(ctx);
  const std::vector<lod::tile> tiles = lod::partition_parts(parts_of(city), kCell, building_key());
  ASSERT_GE(tiles.size(), 4u);

  lod::pyramid_params pp;
  pp.max_rungs = 3;
  pp.mesh_min_tris = 16;
  const std::vector<lod::mesh_pyramid> s = lod::build_tiled_pyramids(tiles, pp);
  cvc::thread_pool pool(3);
  const std::vector<lod::mesh_pyramid> p = lod::build_tiled_pyramids(tiles, pp, &pool);
  const std::vector<lod::mesh_pyramid> c = lod::build_tiled_pyramids(tiles, pp, &ctx.computePool());

  ASSERT_EQ(s.size(), tiles.size());
  ASSERT_EQ(p.size(), tiles.size());
  ASSERT_EQ(c.size(), tiles.size());
  bool coarsened = false;
  for (std::size_t t = 0; t < tiles.size(); ++t) {
    // Each entry is exactly the single-tile pyramid of its tile.
    const lod::mesh_pyramid ref = lod::build_mesh_pyramid(tiles[t].geom, pp);
    for (const lod::mesh_pyramid *r : {&s[t], &p[t], &c[t]}) {
      ASSERT_EQ(r->rungs.size(), ref.rungs.size()) << "tile " << t;
      EXPECT_EQ(r->world_error_m, ref.world_error_m) << "tile " << t;
      for (std::size_t k = 0; k < ref.rungs.size(); ++k)
        expect_geometry_identical(r->rungs[k], ref.rungs[k],
                                  "tile " + std::to_string(t) + " rung " + std::to_string(k));
    }
    EXPECT_EQ(s[t].rungs[0].num_tris(), tiles[t].geom.num_tris()); // rung 0 is the tile
    coarsened = coarsened || s[t].rungs.size() > 1;
  }
  EXPECT_TRUE(coarsened) << "the test city should be big enough to build coarse rungs";

  // A single tile hands the pool to build_mesh_pyramid instead; still identical.
  const std::vector<lod::tile> one(1, tiles[0]);
  const std::vector<lod::mesh_pyramid> one_p = lod::build_tiled_pyramids(one, pp, &pool);
  ASSERT_EQ(one_p.size(), 1u);
  ASSERT_EQ(one_p[0].rungs.size(), s[0].rungs.size());
  for (std::size_t k = 0; k < s[0].rungs.size(); ++k)
    expect_geometry_identical(one_p[0].rungs[k], s[0].rungs[k], "single tile rung");
}

TEST(LodTiles, CallbackFiresOncePerTileAndIsSerialized) {
  cvc::app ctx;
  const std::vector<lod::tile> tiles =
      lod::partition_parts(parts_of(make_city(ctx)), kCell, building_key());
  lod::pyramid_params pp;
  pp.max_rungs = 2;
  pp.mesh_min_tris = 16;

  // Serial: fires in tile order, on the caller.
  {
    std::vector<std::size_t> order;
    const std::thread::id caller = std::this_thread::get_id();
    bool on_caller = true;
    const std::vector<lod::mesh_pyramid> out = lod::build_tiled_pyramids(
        tiles, pp, nullptr, [&](std::size_t i, const lod::tile &t, const lod::mesh_pyramid &pyr) {
          order.push_back(i);
          on_caller = on_caller && std::this_thread::get_id() == caller;
          EXPECT_EQ(&t, &tiles[i]);
          EXPECT_EQ(pyr.rungs[0].num_tris(), t.geom.num_tris());
        });
    std::vector<std::size_t> want(tiles.size());
    for (std::size_t i = 0; i < want.size(); ++i)
      want[i] = i;
    EXPECT_EQ(order, want);
    EXPECT_TRUE(on_caller);
    EXPECT_EQ(out.size(), tiles.size());
  }

  // Pooled: once per tile, never two at a time, and each sees a finished pyramid.
  {
    cvc::thread_pool pool(4);
    std::vector<int> calls(tiles.size(), 0);
    std::vector<std::size_t> rungs_seen(tiles.size(), 0);
    std::atomic<int> inside(0), max_inside(0);
    const std::vector<lod::mesh_pyramid> out = lod::build_tiled_pyramids(
        tiles, pp, &pool, [&](std::size_t i, const lod::tile &, const lod::mesh_pyramid &pyr) {
          const int now = ++inside;
          int prev = max_inside.load();
          while (now > prev && !max_inside.compare_exchange_weak(prev, now)) {
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          ++calls[i];
          rungs_seen[i] = pyr.rungs.size();
          --inside;
        });
    EXPECT_EQ(max_inside.load(), 1);
    for (std::size_t i = 0; i < tiles.size(); ++i) {
      EXPECT_EQ(calls[i], 1) << "tile " << i;
      EXPECT_EQ(rungs_seen[i], out[i].rungs.size()) << "tile " << i;
    }
  }

  // A throwing callback propagates to the caller.
  cvc::thread_pool pool(2);
  EXPECT_THROW(
      lod::build_tiled_pyramids(tiles, pp, &pool,
                                [](std::size_t i, const lod::tile &, const lod::mesh_pyramid &) {
                                  if (i == 1)
                                    throw std::runtime_error("stop");
                                }),
      std::runtime_error);
}

TEST(LodTiles, ConnectedComponentsMatchNamedParts) {
  cvc::app ctx;
  city_opts o;
  o.mixed_attrs = false; // model::merged pads attributes model-wide; keep them uniform
  const std::vector<building> city = make_city(ctx, o);
  cvc::model m;
  for (const building &b : city) {
    m.meshes.push_back(cvc::model::mesh());
    m.meshes.back().geom = b.walls;
    m.meshes.back().name = b.name + "_walls";
    m.meshes.push_back(cvc::model::mesh());
    m.meshes.back().geom = b.roof;
    m.meshes.back().name = b.name + "_roof";
  }
  const std::vector<lod::tile> named = lod::partition_model(m, kCell, building_key());
  expect_tiles_identical(named, lod::partition_parts(parts_of(city), kCell, building_key()));
  const geometry merged = m.merged();

  for (int coincident = 0; coincident < 2; ++coincident) {
    lod::partition_params pp;
    pp.connect_coincident = coincident != 0;
    const std::vector<lod::tile> cc = lod::partition_components(merged, kCell, pp);
    ASSERT_EQ(cc.size(), named.size()) << "coincident=" << coincident;
    std::size_t comps = 0;
    for (std::size_t k = 0; k < cc.size(); ++k) {
      EXPECT_EQ(cc[k].cell, named[k].cell) << "tile " << k;
      EXPECT_EQ(cc[k].content_hash, named[k].content_hash) << "tile " << k;
      EXPECT_EQ(cc[k].geom.num_tris(), named[k].geom.num_tris()) << "tile " << k;
      EXPECT_EQ(cc[k].geom.num_points(), named[k].geom.num_points()) << "tile " << k;
      EXPECT_EQ(cc[k].bounds.minx, named[k].bounds.minx);
      EXPECT_EQ(cc[k].bounds.maxx, named[k].bounds.maxx);
      EXPECT_EQ(cc[k].bounds.miny, named[k].bounds.miny);
      EXPECT_EQ(cc[k].bounds.maxy, named[k].bounds.maxy);
      EXPECT_EQ(cc[k].bounds.maxz, named[k].bounds.maxz);
      for (const std::string &n : cc[k].parts)
        EXPECT_EQ(n.rfind("component_", 0), 0u) << n;
      comps += cc[k].parts.size();
    }
    // Welding coincident rims joins each building's walls and roof into one component.
    EXPECT_EQ(comps, coincident ? city.size() : 2 * city.size());
  }
}

TEST(LodTiles, CoincidentVerticesJoinComponents) {
  cvc::app ctx;
  // Two triangles sharing the position (90,0,0) but not an index: joined, their
  // union's centroid (125) is in cell 1; apart, the first (45) falls in cell 0.
  geometry g = one_triangle(ctx, 0.0, 90.0);
  g.merge(one_triangle(ctx, 90.0, 250.0)); // vertices 1 and 3 are both (90,0,0)
  lod::partition_params pp;
  const std::vector<lod::tile> joined = lod::partition_components(g, kCell, pp);
  ASSERT_EQ(joined.size(), 1u);
  EXPECT_EQ(joined[0].cell.i, 1);
  EXPECT_EQ(joined[0].parts, std::vector<std::string>{"component_0"});
  pp.connect_coincident = false;
  const std::vector<lod::tile> apart = lod::partition_components(g, kCell, pp);
  ASSERT_EQ(apart.size(), 2u);
  EXPECT_EQ(apart[0].cell.i, 0);
  EXPECT_EQ(apart[0].parts, std::vector<std::string>{"component_0"});
  EXPECT_EQ(apart[1].cell.i, 1);
  EXPECT_EQ(apart[1].parts, std::vector<std::string>{"component_1"});
}

TEST(LodTiles, EdgeCases) {
  cvc::app ctx;
  const std::vector<building> city = make_city(ctx);
  const std::vector<lod::named_part> parts = parts_of(city);

  // No parts / an empty merged geometry: no tiles, and no pyramids to build.
  EXPECT_TRUE(lod::partition_parts(std::vector<lod::named_part>(), kCell).empty());
  EXPECT_TRUE(lod::partition_components(geometry(ctx), kCell).empty());
  EXPECT_TRUE(lod::partition_model(cvc::model(), kCell).empty());
  int calls = 0;
  EXPECT_TRUE(lod::build_tiled_pyramids(
                  std::vector<lod::tile>(), lod::pyramid_params(), nullptr,
                  [&](std::size_t, const lod::tile &, const lod::mesh_pyramid &) { ++calls; })
                  .empty());
  EXPECT_EQ(calls, 0);

  // Parts without usable triangles contribute nothing and are not listed.
  geometry points_only(ctx);
  points_only.points().push_back({{5.0, 5.0, 0.0}});
  geometry bad_index = one_triangle(ctx, 0.0, 10.0);
  bad_index.tris()[0][2] = 7; // out of range
  const geometry tri = one_triangle(ctx, 0.0, 10.0);
  std::vector<lod::named_part> odd;
  odd.push_back(lod::named_part("points_only", points_only));
  odd.push_back(lod::named_part("bad_index", bad_index));
  EXPECT_TRUE(lod::partition_parts(odd, kCell).empty());
  EXPECT_TRUE(lod::partition_components(points_only, kCell).empty());
  odd.push_back(lod::named_part("tri", tri));
  const std::vector<lod::tile> just_tri = lod::partition_parts(odd, kCell);
  ASSERT_EQ(just_tri.size(), 1u);
  EXPECT_EQ(just_tri[0].parts, std::vector<std::string>{"tri"});
  EXPECT_EQ(just_tri[0].geom.num_tris(), 1u);

  // Bad cell sizes (and other bad parameters) are rejected.
  const double bad_cells[] = {0.0, -5.0, std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity()};
  for (double c : bad_cells) {
    EXPECT_THROW(lod::partition_parts(parts, c), std::invalid_argument) << c;
    EXPECT_THROW(lod::partition_components(tri, c), std::invalid_argument) << c;
  }
  lod::partition_params bad_up;
  bad_up.up_axis = 3;
  EXPECT_THROW(lod::partition_parts(parts, kCell, lod::group_key_fn(), bad_up),
               std::invalid_argument);
  lod::partition_params bad_q;
  bad_q.hash_quantum_m = 0.0;
  EXPECT_THROW(lod::partition_components(tri, kCell, bad_q), std::invalid_argument);
  EXPECT_THROW(lod::content_hash(tri, -1.0), std::invalid_argument);
  std::vector<lod::named_part> null_part(1);
  null_part[0].name = "nothing";
  EXPECT_THROW(lod::partition_parts(null_part, kCell), std::invalid_argument);

  // A huge cell holds the whole city in one tile -- after anchoring the grid at the
  // scene's corner, since one building sits at negative x.
  lod::partition_params anchored;
  anchored.origin = {{-1000.0, -1000.0, 0.0}};
  const std::vector<lod::tile> whole = lod::partition_parts(parts, 1e9, building_key(), anchored);
  ASSERT_EQ(whole.size(), 1u);
  EXPECT_EQ(whole[0].parts.size(), parts.size());
  std::uint64_t src_tris = 0;
  for (const lod::named_part &p : parts)
    src_tris += p.geom->num_tris();
  EXPECT_EQ(whole[0].geom.num_tris(), src_tris);
  // Re-partitioning that single tile by component reproduces it.
  const std::vector<lod::tile> whole_cc = lod::partition_components(whole[0].geom, 1e9, anchored);
  ASSERT_EQ(whole_cc.size(), 1u);
  EXPECT_EQ(whole_cc[0].content_hash, whole[0].content_hash);
  // Unanchored, the same huge cell splits at the world origin: cells -1 and 0.
  const std::vector<lod::tile> split = lod::partition_parts(parts, 1e9, building_key());
  ASSERT_EQ(split.size(), 2u);
  EXPECT_EQ(split[0].cell.i, -1);
  EXPECT_EQ(split[0].parts, (std::vector<std::string>{"neg_roof", "neg_walls"}));
  EXPECT_EQ(split[1].cell.i, 0);
}

TEST(LodTiles, UpAxisSelectsTheGroundPlane) {
  cvc::app ctx;
  const std::vector<building> city = make_city(ctx);
  // The same city stood up along +Y: (x, y, z) -> (x, z, y).
  std::vector<building> yup = city;
  for (building &b : yup)
    for (geometry *g : {&b.walls, &b.roof})
      for (geometry::point_t &p : g->points())
        std::swap(p[1], p[2]);
  lod::partition_params pp;
  pp.up_axis = 1;
  const std::vector<lod::tile> z = lod::partition_parts(parts_of(city), kCell, building_key());
  const std::vector<lod::tile> y = lod::partition_parts(parts_of(yup), kCell, building_key(), pp);
  ASSERT_EQ(z.size(), y.size());
  for (std::size_t k = 0; k < z.size(); ++k) {
    EXPECT_EQ(z[k].cell, y[k].cell) << "tile " << k;
    EXPECT_EQ(z[k].parts, y[k].parts) << "tile " << k;
  }
  // X-up: cells span (y, z); every building's base is at x in [..], z in [0, h].
  pp.up_axis = 0;
  const std::vector<lod::tile> x = lod::partition_parts(parts_of(city), kCell, building_key(), pp);
  ASSERT_FALSE(x.empty());
  for (const lod::tile &t : x)
    EXPECT_EQ(t.cell.j, 0); // heights < 100 m => one row of cells in z
}

TEST(LodTiles, QuadsAreFanSplitAndBadIndicesDropped) {
  cvc::app ctx;
  geometry q(ctx);
  for (int j = 0; j < 2; ++j)
    for (int i = 0; i < 3; ++i)
      q.points().push_back({{10.0 * i, 10.0 * j, 0.0}});
  q.quads().push_back({{0, 1, 4, 3}});
  q.quads().push_back({{1, 2, 5, 4}});
  q.quads().push_back({{1, 2, 5, 9}}); // second half references vertex 9: dropped
  q.tris().push_back({{0, 1, 8}});     // out of range: dropped
  q.set_geometry_type(geometry::SURFACE_QUAD);

  geometry t(ctx);
  t.points() = q.points();
  t.tris().push_back({{0, 1, 4}});
  t.tris().push_back({{0, 4, 3}});
  t.tris().push_back({{1, 2, 5}});
  t.tris().push_back({{1, 5, 4}});
  t.tris().push_back({{1, 2, 5}}); // the surviving half of the third quad
  EXPECT_EQ(lod::content_hash(q), lod::content_hash(t));

  std::vector<lod::named_part> parts(1, lod::named_part("q", q));
  const std::vector<lod::tile> tiles = lod::partition_parts(parts, kCell);
  ASSERT_EQ(tiles.size(), 1u);
  EXPECT_EQ(tiles[0].geom.num_tris(), 5u);
  EXPECT_EQ(tiles[0].geom.num_points(), 6u);
  EXPECT_EQ(tiles[0].geom.num_quads(), 0u);
  EXPECT_EQ(tiles[0].content_hash, lod::content_hash(t));
}

TEST(LodTiles, ExtremeCoordinatesSaturateDeterministically) {
  cvc::app ctx;
  // Coordinates far beyond the int64 cell / lattice range saturate rather than
  // overflow, so they still partition and hash the same way every time.
  const geometry far = one_triangle(ctx, 1e300, 2e300);
  const geometry nfar = one_triangle(ctx, -2e300, -1e300);
  std::vector<lod::named_part> parts;
  parts.push_back(lod::named_part("far", far));
  parts.push_back(lod::named_part("nfar", nfar));
  const std::vector<lod::tile> tiles = lod::partition_parts(parts, kCell);
  ASSERT_EQ(tiles.size(), 2u);
  EXPECT_EQ(tiles[0].parts, std::vector<std::string>{"nfar"});
  EXPECT_LT(tiles[0].cell.i, -std::int64_t(1e18));
  EXPECT_GT(tiles[1].cell.i, std::int64_t(1e18));
  EXPECT_EQ(tiles[0].content_hash, lod::content_hash(nfar));
  expect_tiles_identical(tiles, lod::partition_parts(parts, kCell));

  geometry nan = one_triangle(ctx, 0.0, 10.0);
  nan.points()[1][0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(lod::content_hash(nan), lod::content_hash(nan));
  EXPECT_NE(lod::content_hash(nan), lod::content_hash(one_triangle(ctx, 0.0, 10.0)));
}
