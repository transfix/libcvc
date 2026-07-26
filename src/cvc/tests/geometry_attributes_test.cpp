// Tests for the Phase-2 additive geometry attributes: per-vertex UVs + tangents
// on cvc::geometry. Verifies the accessors, copy-on-write, the copy paths
// (copy-ctor / operator= / deep copy), and merge() all carry the new containers
// exactly like normals/colors do.

#include <cvc/geometry/geometry.h>
#include <gtest/gtest.h>

using cvc::geometry;

namespace {

// A 3-vertex triangle carrying uvs + tangents (and colors, for cross-checking).
geometry make_uv_tri() {
  geometry g;
  g.points() = {{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}};
  g.colors() = {{{1, 0, 0}}, {{0, 1, 0}}, {{0, 0, 1}}};
  g.uvs() = {{{0.0, 0.0}}, {{1.0, 0.0}}, {{0.0, 1.0}}};
  g.tangents() = {{{1, 0, 0, 1}}, {{1, 0, 0, 1}}, {{0, 1, 0, -1}}};
  g.tris() = {{{0, 1, 2}}};
  return g;
}

} // namespace

TEST(GeometryAttributes, DefaultHasEmptyUvTangentContainers) {
  geometry g;
  // init_ptrs() allocates all containers, so these are empty (not a crash).
  EXPECT_EQ(g.uvs().size(), 0u);
  EXPECT_EQ(g.tangents().size(), 0u);
  EXPECT_EQ(g.const_uvs().size(), 0u);
  EXPECT_EQ(g.const_tangents().size(), 0u);
}

TEST(GeometryAttributes, PopulateAndRead) {
  geometry g = make_uv_tri();
  ASSERT_EQ(g.uvs().size(), 3u);
  ASSERT_EQ(g.tangents().size(), 3u);
  EXPECT_DOUBLE_EQ(g.uvs()[1][0], 1.0);
  EXPECT_DOUBLE_EQ(g.uvs()[2][1], 1.0);
  EXPECT_DOUBLE_EQ(g.tangents()[2][3], -1.0); // handedness
  // const accessors return the same data
  EXPECT_DOUBLE_EQ(g.const_uvs()[1][0], 1.0);
  EXPECT_DOUBLE_EQ(g.const_tangents()[0][0], 1.0);
}

TEST(GeometryAttributes, CopyOnWriteUvs) {
  geometry a = make_uv_tri();
  geometry b = a; // shallow: shares the uv container
  // mutating b's uvs must not touch a
  b.uvs()[0][0] = 0.5;
  EXPECT_DOUBLE_EQ(a.uvs()[0][0], 0.0);
  EXPECT_DOUBLE_EQ(b.uvs()[0][0], 0.5);
}

TEST(GeometryAttributes, CopyOnWriteTangents) {
  geometry a = make_uv_tri();
  geometry b = a;
  b.tangents()[0][3] = 7.0;
  EXPECT_DOUBLE_EQ(a.tangents()[0][3], 1.0);
  EXPECT_DOUBLE_EQ(b.tangents()[0][3], 7.0);
}

TEST(GeometryAttributes, CopyConstructorPreservesUvsTangents) {
  geometry a = make_uv_tri();
  geometry b(a);
  ASSERT_EQ(b.uvs().size(), 3u);
  ASSERT_EQ(b.tangents().size(), 3u);
  EXPECT_DOUBLE_EQ(b.uvs()[1][0], 1.0);
  EXPECT_DOUBLE_EQ(b.tangents()[2][3], -1.0);
}

TEST(GeometryAttributes, AssignmentPreservesUvsTangents) {
  geometry a = make_uv_tri();
  geometry b;
  b = a;
  ASSERT_EQ(b.uvs().size(), 3u);
  ASSERT_EQ(b.tangents().size(), 3u);
  EXPECT_DOUBLE_EQ(b.uvs()[2][1], 1.0);
}

TEST(GeometryAttributes, DeepCopyPreservesAndDecouplesUvs) {
  geometry a = make_uv_tri();
  geometry b;
  b.copy(a, /*deepCopy=*/true);
  ASSERT_EQ(b.uvs().size(), 3u);
  ASSERT_EQ(b.tangents().size(), 3u);
  EXPECT_DOUBLE_EQ(b.uvs()[1][0], 1.0);
  // a genuinely separate buffer: mutating b never affects a
  b.uvs()[1][0] = 9.0;
  EXPECT_DOUBLE_EQ(a.uvs()[1][0], 1.0);
}

TEST(GeometryAttributes, MergeAppendsUvsAndTangents) {
  geometry a = make_uv_tri();
  geometry b = make_uv_tri();
  a.merge(b);
  EXPECT_EQ(a.num_points(), 6u);
  ASSERT_EQ(a.uvs().size(), 6u);
  ASSERT_EQ(a.tangents().size(), 6u);
  // the appended block matches b
  EXPECT_DOUBLE_EQ(a.uvs()[3 + 1][0], 1.0);
  EXPECT_DOUBLE_EQ(a.tangents()[3 + 2][3], -1.0);
}

TEST(GeometryAttributes, ClearResetsUvsTangents) {
  geometry a = make_uv_tri();
  a.clear();
  EXPECT_EQ(a.uvs().size(), 0u);
  EXPECT_EQ(a.tangents().size(), 0u);
  EXPECT_EQ(a.num_points(), 0u);
}

TEST(GeometryAttributes, UvsPtrSharesWithoutDetach) {
  geometry a = make_uv_tri();
  auto owner = a.uvs_ptr(); // non-detaching shared owner (for zero-copy views)
  ASSERT_TRUE(static_cast<bool>(owner));
  EXPECT_EQ(owner->size(), 3u);
  EXPECT_DOUBLE_EQ((*owner)[1][0], 1.0);
}
