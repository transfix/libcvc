/*
  Copyright 2026 The University of Texas at Austin

  Unit tests for cvc/vis/reference.h -- the reference culler (the oracle) and the
  conservativeness checker every faster culler is validated against.

  The oracle defines what "visible" means, so these tests pin that definition:
    * it keeps exactly the proxies that pass layer + frustum + distance, in
      ascending id order;
    * conservativeness_violations counts ONLY the correctness failure (a visible
      proxy wrongly dropped), and treats benign over-draw as fine;
    * layer masking, frustum culling, and the unified distance/small-feature cut
      each remove what they should and nothing else;
    * it scales to a 10^5-proxy synthetic scene.
*/

#include <cstdint>
#include <cvc/vis/reference.h>
#include <gtest/gtest.h>
#include <random>
#include <vector>

using namespace cvc::vis;

namespace {

frustum box_frustum(float minx, float maxx, float miny, float maxy, float minz, float maxz) {
  frustum f;
  f.p[0] = {{1, 0, 0}, -minx};
  f.p[1] = {{-1, 0, 0}, maxx};
  f.p[2] = {{0, 1, 0}, -miny};
  f.p[3] = {{0, -1, 0}, maxy};
  f.p[4] = {{0, 0, 1}, -minz};
  f.p[5] = {{0, 0, -1}, maxz};
  return f;
}

// A view whose frustum is a big box (so frustum culling is off unless a test
// places a proxy outside it) and whose eye is at the origin looking down -z.
view_params wide_view() {
  view_params v;
  v.f = box_frustum(-1e6f, 1e6f, -1e6f, 1e6f, -1e6f, 1e6f);
  v.proj = cvc::lod::preset_view(cvc::lod::quality_preset::balanced);
  v.proj.eye[0] = v.proj.eye[1] = v.proj.eye[2] = 0.0;
  v.min_screen_px = 0.0; // distance cull off by default; tests enable it
  return v;
}

aabb box_at(float cx, float cy, float cz, float h) {
  return {{cx - h, cy - h, cz - h}, {cx + h, cy + h, cz + h}};
}

} // namespace

TEST(VisOracle, KeepsOnlyVisibleInAscendingOrder) {
  // Four proxies; the frustum box excludes the one at x = 500.
  std::vector<aabb> bounds = {
      box_at(0, 0, -10, 1),   // in
      box_at(500, 0, -10, 1), // out (+x)
      box_at(-5, 0, -20, 1),  // in
      box_at(0, 5, -30, 1),   // in
  };
  scene_view s;
  s.count = 4;
  s.bounds = bounds.data();

  view_params v = wide_view();
  v.f = box_frustum(-100, 100, -100, 100, -100, 100);

  visible_set vs;
  reference_cull(s, v, vs);

  ASSERT_EQ(vs.size(), 3u);
  EXPECT_EQ(vs.ids[0], 0u);
  EXPECT_EQ(vs.ids[1], 2u);
  EXPECT_EQ(vs.ids[2], 3u); // ascending, id 1 dropped
  for (proxy_id id : vs.ids)
    EXPECT_TRUE(reference_visible(s, v, id, nullptr, nullptr));
  // Payload is populated.
  EXPECT_GT(vs.dist_m[0], 0.0f);
  EXPECT_GT(vs.screen_px[0], 0.0f);
}

TEST(VisOracle, ConservativenessCounter) {
  std::vector<aabb> bounds;
  for (int i = 0; i < 50; ++i)
    bounds.push_back(box_at(static_cast<float>(i - 25), 0, -10, 0.5f));
  scene_view s;
  s.count = static_cast<std::uint32_t>(bounds.size());
  s.bounds = bounds.data();
  const view_params v = wide_view();

  visible_set truth;
  reference_cull(s, v, truth);
  ASSERT_FALSE(truth.empty());

  // The exact oracle set has zero violations against itself.
  EXPECT_EQ(conservativeness_violations(truth.ids, s, v), 0u);

  // Drop one visible proxy -> exactly one violation.
  std::vector<proxy_id> missing_one(truth.ids.begin() + 1, truth.ids.end());
  EXPECT_EQ(conservativeness_violations(missing_one, s, v), 1u);

  // A superset (extra ids the oracle would cull) is benign over-draw -> zero.
  std::vector<proxy_id> superset = truth.ids;
  superset.push_back(999999u); // an out-of-range / would-be-culled id
  EXPECT_EQ(conservativeness_violations(superset, s, v), 0u);

  // The empty set drops everything visible.
  EXPECT_EQ(conservativeness_violations({}, s, v), truth.size());
}

TEST(VisOracle, LayerMask) {
  std::vector<aabb> bounds = {box_at(0, 0, -10, 1), box_at(1, 0, -10, 1), box_at(2, 0, -10, 1)};
  std::vector<std::uint32_t> layer = {0b001, 0b010, 0b100};
  scene_view s;
  s.count = 3;
  s.bounds = bounds.data();
  s.layer = layer.data();

  view_params v = wide_view();
  v.layer_mask = 0b010; // only the middle proxy's layer

  visible_set vs;
  reference_cull(s, v, vs);
  ASSERT_EQ(vs.size(), 1u);
  EXPECT_EQ(vs.ids[0], 1u);

  // No layer column => everyone is in layer 0 (bit 0). A mask without bit 0
  // culls all; a mask with bit 0 keeps all.
  s.layer = nullptr;
  v.layer_mask = 0b010;
  reference_cull(s, v, vs);
  EXPECT_EQ(vs.size(), 0u);
  v.layer_mask = 0b011;
  reference_cull(s, v, vs);
  EXPECT_EQ(vs.size(), 3u);
}

TEST(VisOracle, DistanceAndSmallFeatureAreOneTest) {
  // A pebble and a tree at the same distance; the pebble falls below 1.5 px of
  // width, the tree does not. Both are inside the frustum.
  std::vector<aabb> bounds = {box_at(0, 0, -200, 1), box_at(3, 0, -200, 1)};
  std::vector<sphere> spheres = {sphere{{0, 0, -200}, 0.05f}, sphere{{3, 0, -200}, 5.0f}};
  scene_view s;
  s.count = 2;
  s.bounds = bounds.data();
  s.spheres = spheres.data();

  view_params v = wide_view();

  v.min_screen_px = 0.0; // off: both survive
  visible_set vs;
  reference_cull(s, v, vs);
  EXPECT_EQ(vs.size(), 2u);

  v.min_screen_px = 1.5; // on: the pebble goes, the tree stays
  reference_cull(s, v, vs);
  ASSERT_EQ(vs.size(), 1u);
  EXPECT_EQ(vs.ids[0], 1u);
}

TEST(VisOracle, FrustumCullsOutsideProxies) {
  std::vector<aabb> bounds = {
      box_at(0, 0, -10, 1),    // in
      box_at(0, 0, -10000, 1), // out (far)
      box_at(5000, 0, -10, 1), // out (+x)
  };
  scene_view s;
  s.count = 3;
  s.bounds = bounds.data();

  view_params v = wide_view();
  v.f = box_frustum(-100, 100, -100, 100, -1000, 100);

  visible_set vs;
  reference_cull(s, v, vs);
  ASSERT_EQ(vs.size(), 1u);
  EXPECT_EQ(vs.ids[0], 0u);
}

TEST(VisOracle, ScalesToHundredThousandProxies) {
  constexpr std::uint32_t N = 100000;
  std::vector<aabb> bounds;
  bounds.reserve(N);
  std::mt19937 rng(424242);
  std::uniform_real_distribution<float> P(-2000.0f, 2000.0f);
  std::uniform_real_distribution<float> H(0.2f, 8.0f);
  for (std::uint32_t i = 0; i < N; ++i) {
    const float h = H(rng);
    bounds.push_back(box_at(P(rng), P(rng), -std::abs(P(rng)) - 1.0f, h));
  }
  scene_view s;
  s.count = N;
  s.bounds = bounds.data();

  view_params v = wide_view();
  v.f = box_frustum(-500, 500, -500, 500, -4000, 0);
  v.min_screen_px = 1.5;

  visible_set vs;
  reference_cull(s, v, vs);

  // A frustum covering a quarter of the XY spread keeps a meaningful, bounded
  // slice -- not nothing, not everything.
  EXPECT_GT(vs.size(), 0u);
  EXPECT_LT(vs.size(), static_cast<std::size_t>(N));
  // Everything returned is genuinely visible, and the set has no holes.
  EXPECT_EQ(conservativeness_violations(vs.ids, s, v), 0u);
  // Ascending, unique.
  for (std::size_t k = 1; k < vs.ids.size(); ++k)
    ASSERT_LT(vs.ids[k - 1], vs.ids[k]);
}
