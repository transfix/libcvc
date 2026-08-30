/*
  Copyright 2026 The University of Texas at Austin

  Unit tests for cvc/vis/mesh_tiler.h -- splitting a merged mesh into cullable
  spatial tiles.

  What must hold:
    * the tiles PARTITION the triangle set -- every valid triangle in exactly
      one tile, none dropped, none duplicated;
    * each tile's AABB contains every vertex of every triangle in it (the
      conservative bound the culler relies on);
    * a triangle lands in the tile its CENTROID falls in;
    * malformed triangles are skipped, not fatal; a tiny cell size coarsens
      rather than allocating; and the whole thing degrades cleanly to empty.
  A closing integration test tiles a city and culls the tiles.
*/

#include <array> // std::array (MSVC does not pull it in transitively)
#include <cmath>
#include <cstdint>
#include <cvc/vis/grid_index.h>
#include <cvc/vis/mesh_tiler.h>
#include <cvc/vis/reference.h>
#include <gtest/gtest.h>
#include <random>
#include <vector>

using namespace cvc::vis;

namespace {

// A mesh of axis-aligned boxes (12 triangles each) at given ground positions,
// standing along +Z. Returns flat points (xyz) and tris (index triples).
struct mesh {
  std::vector<double> points;
  std::vector<std::uint32_t> tris;
  std::size_t npoints() const { return points.size() / 3; }
  std::size_t ntris() const { return tris.size() / 3; }
};

mesh boxes(const std::vector<std::array<float, 2>> &centres, float half, float height) {
  mesh m;
  for (auto &c : centres) {
    const std::uint32_t base = static_cast<std::uint32_t>(m.npoints());
    const float x0 = c[0] - half, x1 = c[0] + half, y0 = c[1] - half, y1 = c[1] + half;
    const float corners[8][3] = {{x0, y0, 0},      {x1, y0, 0},      {x1, y1, 0},
                                 {x0, y1, 0},      {x0, y0, height}, {x1, y0, height},
                                 {x1, y1, height}, {x0, y1, height}};
    for (auto &p : corners) {
      m.points.push_back(p[0]);
      m.points.push_back(p[1]);
      m.points.push_back(p[2]);
    }
    const int faces[12][3] = {{0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {0, 4, 5}, {0, 5, 1},
                              {1, 5, 6}, {1, 6, 2}, {2, 6, 7}, {2, 7, 3}, {3, 7, 4}, {3, 4, 0}};
    for (auto &f : faces) {
      m.tris.push_back(base + f[0]);
      m.tris.push_back(base + f[1]);
      m.tris.push_back(base + f[2]);
    }
  }
  return m;
}

mesh grid_city(int n_per_side, float spacing) {
  std::vector<std::array<float, 2>> c;
  for (int i = 0; i < n_per_side; ++i)
    for (int j = 0; j < n_per_side; ++j)
      c.push_back({i * spacing, j * spacing});
  return boxes(c, 4.0f, 20.0f);
}

} // namespace

TEST(VisTiler, PartitionsTrianglesExactly) {
  const mesh m = grid_city(8, 100.0f); // 64 boxes, 768 triangles
  const mesh_tiling t =
      tile_triangles(m.points.data(), m.npoints(), m.tris.data(), m.ntris(), 100.0f);

  ASSERT_GT(t.tile_count(), 0u);

  // Every triangle id appears exactly once across all tiles.
  std::vector<int> seen(m.ntris(), 0);
  std::uint32_t total = 0;
  for (std::size_t k = 0; k < t.tile_count(); ++k) {
    for (std::uint32_t j = t.start[k]; j < t.start[k + 1]; ++j) {
      const std::uint32_t id = t.tri_ids[j];
      ASSERT_LT(id, m.ntris());
      ++seen[id];
      ++total;
    }
  }
  EXPECT_EQ(total, m.ntris());
  for (std::size_t i = 0; i < seen.size(); ++i)
    EXPECT_EQ(seen[i], 1) << "triangle " << i << " landed in " << seen[i] << " tiles";
}

TEST(VisTiler, TileBoundsContainTheirTriangles) {
  const mesh m = grid_city(6, 80.0f);
  const mesh_tiling t =
      tile_triangles(m.points.data(), m.npoints(), m.tris.data(), m.ntris(), 80.0f);

  for (std::size_t k = 0; k < t.tile_count(); ++k) {
    const aabb &b = t.bounds[k];
    for (std::uint32_t j = t.start[k]; j < t.start[k + 1]; ++j) {
      const std::uint32_t *tri = m.tris.data() + 3 * t.tri_ids[j];
      for (int c = 0; c < 3; ++c) {
        const double *p = m.points.data() + 3 * tri[c];
        for (int a = 0; a < 3; ++a) {
          EXPECT_GE(p[a], b.mn[a] - 1e-4);
          EXPECT_LE(p[a], b.mx[a] + 1e-4);
        }
      }
    }
  }
}

TEST(VisTiler, PlacesByCentroid) {
  // Two far-apart boxes at a 200 m cell size land in different tiles; two close
  // boxes in the same cell share a tile.
  const mesh far = boxes({{0, 0}, {500, 500}}, 4.0f, 10.0f);
  const mesh_tiling tf =
      tile_triangles(far.points.data(), far.npoints(), far.tris.data(), far.ntris(), 200.0f);
  EXPECT_EQ(tf.tile_count(), 2u);

  const mesh near = boxes({{0, 0}, {10, 10}}, 4.0f, 10.0f);
  const mesh_tiling tn =
      tile_triangles(near.points.data(), near.npoints(), near.tris.data(), near.ntris(), 200.0f);
  EXPECT_EQ(tn.tile_count(), 1u);
  EXPECT_EQ(tn.tris_in(0), 24u); // both boxes' 12+12 triangles
}

TEST(VisTiler, SkipsMalformedTriangles) {
  mesh m = boxes({{0, 0}}, 4.0f, 10.0f);
  const std::size_t good = m.ntris();
  // Append a triangle referencing a nonexistent vertex.
  m.tris.push_back(0);
  m.tris.push_back(1);
  m.tris.push_back(999999);
  const mesh_tiling t =
      tile_triangles(m.points.data(), m.npoints(), m.tris.data(), m.ntris(), 100.0f);
  std::uint32_t total = 0;
  for (std::size_t k = 0; k < t.tile_count(); ++k)
    total += t.tris_in(k);
  EXPECT_EQ(total, good) << "the malformed triangle should be skipped, the rest kept";
}

TEST(VisTiler, CoarsensAndDegrades) {
  const mesh m = grid_city(4, 100.0f);
  // 1 mm cells over a 300 m span would be billions of cells: coarsen instead.
  const mesh_tiling t =
      tile_triangles(m.points.data(), m.npoints(), m.tris.data(), m.ntris(), 0.001f);
  EXPECT_GT(t.cell_size, 0.001f);
  std::uint32_t total = 0;
  for (std::size_t k = 0; k < t.tile_count(); ++k)
    total += t.tris_in(k);
  EXPECT_EQ(total, m.ntris());

  // Degenerate inputs.
  EXPECT_EQ(tile_triangles(nullptr, 0, nullptr, 0, 10.0f).tile_count(), 0u);
  EXPECT_EQ(tile_triangles(m.points.data(), m.npoints(), m.tris.data(), 0, 10.0f).tile_count(), 0u);
}

TEST(VisTiler, TiledSceneCullsCorrectly) {
  // The end-to-end shape: tile a city, hand the tile AABBs to the culler, and
  // check the visible tiles match a direct frustum test over the same boxes.
  const mesh m = grid_city(10, 128.0f); // 100 boxes over ~1.2 km
  const mesh_tiling t =
      tile_triangles(m.points.data(), m.npoints(), m.tris.data(), m.ntris(), 128.0f);
  ASSERT_GT(t.tile_count(), 1u);

  scene_view s;
  s.count = static_cast<std::uint32_t>(t.tile_count());
  s.bounds = t.bounds.data();

  // A frustum box over one corner of the city.
  frustum f;
  f.p[0] = {{1, 0, 0}, -100.0f};
  f.p[1] = {{-1, 0, 0}, 700.0f};
  f.p[2] = {{0, 1, 0}, -100.0f};
  f.p[3] = {{0, -1, 0}, 700.0f};
  f.p[4] = {{0, 0, 1}, 1e6f};
  f.p[5] = {{0, 0, -1}, 1e6f};
  view_params v;
  v.f = f;
  v.proj = cvc::lod::preset_view(cvc::lod::quality_preset::balanced);
  v.min_screen_px = 0.0;

  grid_index grid;
  grid.build(s, 256.0f, 0, 1); // grid over X,Y (buildings stand along Z)
  visible_set g, o;
  grid.cull(s, v, g);
  reference_cull(s, v, o);

  ASSERT_EQ(g.size(), o.size());
  for (std::size_t i = 0; i < g.size(); ++i)
    EXPECT_EQ(g.ids[i], o.ids[i]);
  EXPECT_GT(g.size(), 0u);
  EXPECT_LT(g.size(), t.tile_count()) << "some tiles must be culled";
}
