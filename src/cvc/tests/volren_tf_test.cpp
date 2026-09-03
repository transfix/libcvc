// Unit tests for cvc::volren transfer functions (transfer_function.h/.cpp):
// piecewise-linear transfer_function, the baked flat LUT the ray-marcher
// samples, and the gradient-magnitude opacity ramp.

#include <cvc/volren/transfer_function.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <random>
#include <vector>

using namespace cvc::volren;

namespace {

void expect_rgba_near(const rgba_f &c, float r, float g, float b, float a, float tol) {
  EXPECT_NEAR(c.r, r, tol);
  EXPECT_NEAR(c.g, g, tol);
  EXPECT_NEAR(c.b, b, tol);
  EXPECT_NEAR(c.a, a, tol);
}

void expect_rgba_eq(const rgba_f &lhs, const rgba_f &rhs, float tol) {
  EXPECT_NEAR(lhs.r, rhs.r, tol);
  EXPECT_NEAR(lhs.g, rhs.g, tol);
  EXPECT_NEAR(lhs.b, rhs.b, tol);
  EXPECT_NEAR(lhs.a, rhs.a, tol);
}

transfer_point make_point(double value, float r, float g, float b, float a) {
  transfer_point p;
  p.value = value;
  p.r = r;
  p.g = g;
  p.b = b;
  p.a = a;
  return p;
}

} // namespace

// ============================================================================
// transfer_function
// ============================================================================

TEST(TransferFunctionTest, EmptyFunctionSamplesTransparentBlack) {
  transfer_function tf;
  EXPECT_TRUE(tf.empty());
  for (double v : {-100.0, -1.0, 0.0, 0.5, 1.0, 255.0}) {
    expect_rgba_near(tf.sample(v), 0.f, 0.f, 0.f, 0.f, 0.f);
  }
}

TEST(TransferFunctionTest, EmptyFunctionDomainDefaultsToUnitInterval) {
  transfer_function tf;
  EXPECT_DOUBLE_EQ(tf.domain_min(), 0.0);
  EXPECT_DOUBLE_EQ(tf.domain_max(), 1.0);
}

TEST(TransferFunctionTest, AddKeepsPointsSortedRegardlessOfInsertionOrder) {
  std::mt19937 rng(42);
  std::vector<transfer_point> pts = {
      make_point(-3.0, 0.1f, 0.2f, 0.3f, 0.4f), make_point(0.0, 0.5f, 0.6f, 0.7f, 0.8f),
      make_point(1.5, 0.9f, 0.1f, 0.2f, 0.3f),  make_point(7.25, 0.4f, 0.5f, 0.6f, 0.7f),
      make_point(10.0, 0.8f, 0.9f, 0.1f, 0.2f), make_point(42.0, 0.3f, 0.4f, 0.5f, 0.6f)};
  std::vector<transfer_point> shuffled = pts;
  std::shuffle(shuffled.begin(), shuffled.end(), rng);

  transfer_function tf;
  for (const auto &p : shuffled)
    tf.add(p);

  ASSERT_EQ(tf.points().size(), pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    EXPECT_DOUBLE_EQ(tf.points()[i].value, pts[i].value) << "index " << i;
    // Values are distinct so the color must travel with its value.
    EXPECT_NEAR(tf.points()[i].r, pts[i].r, 1e-6f) << "index " << i;
    EXPECT_NEAR(tf.points()[i].a, pts[i].a, 1e-6f) << "index " << i;
  }
  for (std::size_t i = 1; i < tf.points().size(); ++i)
    EXPECT_LE(tf.points()[i - 1].value, tf.points()[i].value);
}

TEST(TransferFunctionTest, VectorConstructorSortsPointsByValue) {
  transfer_function tf({make_point(5.0, 0.5f, 0.f, 0.f, 0.5f), make_point(-1.0, 0.f, 0.5f, 0.f, 0.1f),
                        make_point(2.0, 0.f, 0.f, 0.5f, 0.9f)});
  ASSERT_EQ(tf.points().size(), 3u);
  EXPECT_DOUBLE_EQ(tf.points()[0].value, -1.0);
  EXPECT_DOUBLE_EQ(tf.points()[1].value, 2.0);
  EXPECT_DOUBLE_EQ(tf.points()[2].value, 5.0);
  EXPECT_NEAR(tf.points()[0].g, 0.5f, 1e-6f);
  EXPECT_NEAR(tf.points()[1].b, 0.5f, 1e-6f);
  EXPECT_NEAR(tf.points()[2].r, 0.5f, 1e-6f);
}

TEST(TransferFunctionTest, VectorConstructorSortIsStableForEqualValues) {
  // Two points share value 1.0; stable_sort must keep A before B.
  transfer_function tf({make_point(1.0, 0.1f, 0.f, 0.f, 0.1f), // A
                        make_point(0.0, 0.f, 0.f, 0.f, 0.f),
                        make_point(1.0, 0.2f, 0.f, 0.f, 0.2f)}); // B
  ASSERT_EQ(tf.points().size(), 3u);
  EXPECT_DOUBLE_EQ(tf.points()[0].value, 0.0);
  EXPECT_DOUBLE_EQ(tf.points()[1].value, 1.0);
  EXPECT_DOUBLE_EQ(tf.points()[2].value, 1.0);
  EXPECT_NEAR(tf.points()[1].r, 0.1f, 1e-6f); // A first
  EXPECT_NEAR(tf.points()[2].r, 0.2f, 1e-6f); // B second
}

TEST(TransferFunctionTest, AddPlacesDuplicateValueAfterExistingPoints) {
  transfer_function tf;
  tf.add(make_point(1.0, 0.1f, 0.f, 0.f, 0.1f)); // A
  tf.add(make_point(1.0, 0.2f, 0.f, 0.f, 0.2f)); // B
  tf.add(make_point(1.0, 0.3f, 0.f, 0.f, 0.3f)); // C
  ASSERT_EQ(tf.points().size(), 3u);
  EXPECT_NEAR(tf.points()[0].r, 0.1f, 1e-6f);
  EXPECT_NEAR(tf.points()[1].r, 0.2f, 1e-6f);
  EXPECT_NEAR(tf.points()[2].r, 0.3f, 1e-6f);
}

TEST(TransferFunctionTest, SampleClampsBelowFirstPoint) {
  transfer_function tf({make_point(10.0, 0.25f, 0.5f, 0.75f, 1.f), make_point(20.0, 1.f, 0.f, 0.f, 0.f)});
  expect_rgba_near(tf.sample(-1000.0), 0.25f, 0.5f, 0.75f, 1.f, 1e-6f);
  expect_rgba_near(tf.sample(9.999), 0.25f, 0.5f, 0.75f, 1.f, 1e-6f);
  expect_rgba_near(tf.sample(10.0), 0.25f, 0.5f, 0.75f, 1.f, 1e-6f);
}

TEST(TransferFunctionTest, SampleClampsAboveLastPoint) {
  transfer_function tf({make_point(10.0, 0.25f, 0.5f, 0.75f, 1.f), make_point(20.0, 1.f, 0.f, 0.5f, 0.5f)});
  expect_rgba_near(tf.sample(20.0), 1.f, 0.f, 0.5f, 0.5f, 1e-6f);
  expect_rgba_near(tf.sample(20.001), 1.f, 0.f, 0.5f, 0.5f, 1e-6f);
  expect_rgba_near(tf.sample(1e9), 1.f, 0.f, 0.5f, 0.5f, 1e-6f);
}

TEST(TransferFunctionTest, SampleInterpolatesLinearlyAtMidpointAllChannels) {
  transfer_function tf({make_point(0.0, 0.f, 1.f, 0.2f, 0.f), make_point(2.0, 1.f, 0.f, 0.8f, 0.5f)});
  // Midpoint: exact channel-wise average.
  expect_rgba_near(tf.sample(1.0), 0.5f, 0.5f, 0.5f, 0.25f, 1e-6f);
  // Quarter point: 25% blend toward the second point.
  expect_rgba_near(tf.sample(0.5), 0.25f, 0.75f, 0.35f, 0.125f, 1e-6f);
}

TEST(TransferFunctionTest, SampleAtInteriorControlPointReturnsThatPointColor) {
  transfer_function tf({make_point(0.0, 0.f, 0.f, 0.f, 0.f), make_point(1.0, 0.3f, 0.6f, 0.9f, 0.4f),
                        make_point(2.0, 1.f, 1.f, 1.f, 1.f)});
  expect_rgba_near(tf.sample(1.0), 0.3f, 0.6f, 0.9f, 0.4f, 1e-6f);
}

TEST(TransferFunctionTest, DuplicateValuePointsSampleUsesOneSide) {
  // Two coincident points at v=1: A (alpha 0.2) then B (alpha 0.8).
  transfer_function tf({make_point(0.0, 0.f, 0.f, 0.f, 0.f),  // start
                        make_point(1.0, 0.2f, 0.f, 0.f, 0.2f), // A
                        make_point(1.0, 0.8f, 0.f, 0.f, 0.8f), // B
                        make_point(2.0, 1.f, 0.f, 0.f, 1.f)}); // end
  // Exactly at the duplicated value: lower_bound lands on the FIRST duplicate
  // and interpolation degenerates to t=1, so sampling returns A's color.
  expect_rgba_near(tf.sample(1.0), 0.2f, 0.f, 0.f, 0.2f, 1e-6f);
  // Below the step: interpolates start -> A.
  expect_rgba_near(tf.sample(0.5), 0.1f, 0.f, 0.f, 0.1f, 1e-6f);
  // Above the step: interpolates B -> end (the other side of the step).
  expect_rgba_near(tf.sample(1.5), 0.9f, 0.f, 0.f, 0.9f, 1e-6f);
}

TEST(TransferFunctionTest, DomainMinMaxReflectControlPointExtent) {
  transfer_function tf;
  tf.add(make_point(3.5, 0.f, 0.f, 0.f, 1.f));
  EXPECT_DOUBLE_EQ(tf.domain_min(), 3.5);
  EXPECT_DOUBLE_EQ(tf.domain_max(), 3.5);
  tf.add(make_point(-2.25, 0.f, 0.f, 0.f, 1.f));
  tf.add(make_point(9.75, 0.f, 0.f, 0.f, 1.f));
  EXPECT_DOUBLE_EQ(tf.domain_min(), -2.25);
  EXPECT_DOUBLE_EQ(tf.domain_max(), 9.75);
}

TEST(TransferFunctionTest, SinglePointFunctionSamplesConstantColor) {
  transfer_function tf({make_point(5.0, 0.4f, 0.3f, 0.2f, 0.1f)});
  expect_rgba_near(tf.sample(-1.0), 0.4f, 0.3f, 0.2f, 0.1f, 1e-6f);
  expect_rgba_near(tf.sample(5.0), 0.4f, 0.3f, 0.2f, 0.1f, 1e-6f);
  expect_rgba_near(tf.sample(99.0), 0.4f, 0.3f, 0.2f, 0.1f, 1e-6f);
}

// ============================================================================
// baked_transfer_function
// ============================================================================

namespace {

transfer_function three_point_tf() {
  return transfer_function({make_point(0.0, 0.f, 1.f, 0.2f, 0.f),
                            make_point(0.5, 1.f, 0.f, 0.8f, 0.25f),
                            make_point(1.0, 0.5f, 0.5f, 0.f, 1.f)});
}

} // namespace

TEST(BakedTransferFunctionTest, DefaultConstructedIsEmptyAndTransparent) {
  baked_transfer_function baked;
  EXPECT_TRUE(baked.empty());
  EXPECT_EQ(baked.size(), 0u);
  EXPECT_DOUBLE_EQ(baked.domain_min(), 0.0);
  EXPECT_DOUBLE_EQ(baked.domain_max(), 1.0);
  expect_rgba_near(baked.sample(0.5), 0.f, 0.f, 0.f, 0.f, 0.f);
}

TEST(BakedTransferFunctionTest, SampleAtKnotValuesMatchesSourceFunction) {
  const transfer_function tf = three_point_tf();
  const double lo = 0.0, hi = 1.0;
  const std::size_t n = 65;
  baked_transfer_function baked(tf, lo, hi, n);
  ASSERT_FALSE(baked.empty());
  ASSERT_EQ(baked.size(), n);
  const float tol = float(1.0 / double(n)); // one LUT quantum
  for (std::size_t i = 0; i < n; ++i) {
    const double v = lo + (hi - lo) * double(i) / double(n - 1);
    expect_rgba_eq(baked.sample(v), tf.sample(v), tol);
  }
}

TEST(BakedTransferFunctionTest, SampleSnapsToNearestLutEntry) {
  // Alpha ramp 0->1 over [0,1] baked into 5 entries: knots at 0, .25, .5, .75, 1
  // with alpha equal to the knot value.
  transfer_function tf({make_point(0.0, 0.f, 0.f, 0.f, 0.f), make_point(1.0, 0.f, 0.f, 0.f, 1.f)});
  baked_transfer_function baked(tf, 0.0, 1.0, 5);
  ASSERT_EQ(baked.size(), 5u);
  // Values between entries snap to the nearest knot (legacy int-index lookup).
  EXPECT_NEAR(baked.sample(0.26).a, 0.25f, 1e-6f);
  EXPECT_NEAR(baked.sample(0.36).a, 0.25f, 1e-6f);
  EXPECT_NEAR(baked.sample(0.40).a, 0.50f, 1e-6f);
  EXPECT_NEAR(baked.sample(0.10).a, 0.00f, 1e-6f);
  EXPECT_NEAR(baked.sample(0.90).a, 1.00f, 1e-6f);
  // A whole snap-neighborhood returns one identical entry.
  expect_rgba_eq(baked.sample(0.24), baked.sample(0.26), 1e-7f);
}

TEST(BakedTransferFunctionTest, SampleClampsOutsideBakedDomain) {
  const transfer_function tf = three_point_tf();
  baked_transfer_function baked(tf, 0.0, 1.0, 33);
  // Below lo: first LUT entry == tf.sample(lo).
  expect_rgba_eq(baked.sample(-5.0), tf.sample(0.0), 1e-6f);
  expect_rgba_eq(baked.sample(-5.0), baked.sample(0.0), 1e-7f);
  // Above hi: last LUT entry == tf.sample(hi).
  expect_rgba_eq(baked.sample(6.0), tf.sample(1.0), 1e-6f);
  expect_rgba_eq(baked.sample(6.0), baked.sample(1.0), 1e-7f);
}

TEST(BakedTransferFunctionTest, EmptySourceFunctionBakesEmpty) {
  transfer_function tf;
  baked_transfer_function baked(tf, 0.0, 1.0, 64);
  EXPECT_TRUE(baked.empty());
  EXPECT_EQ(baked.size(), 0u);
  expect_rgba_near(baked.sample(0.5), 0.f, 0.f, 0.f, 0.f, 0.f);
}

TEST(BakedTransferFunctionTest, InvertedOrZeroWidthDomainBakesEmpty) {
  const transfer_function tf = three_point_tf();
  baked_transfer_function zero_width(tf, 0.5, 0.5, 64);
  EXPECT_TRUE(zero_width.empty());
  expect_rgba_near(zero_width.sample(0.5), 0.f, 0.f, 0.f, 0.f, 0.f);
  EXPECT_DOUBLE_EQ(zero_width.domain_min(), 0.5);
  EXPECT_DOUBLE_EQ(zero_width.domain_max(), 0.5);

  baked_transfer_function inverted(tf, 1.0, 0.0, 64);
  EXPECT_TRUE(inverted.empty());
  expect_rgba_near(inverted.sample(0.5), 0.f, 0.f, 0.f, 0.f, 0.f);
}

TEST(BakedTransferFunctionTest, SizeBelowTwoBakesEmpty) {
  const transfer_function tf = three_point_tf();
  baked_transfer_function size_zero(tf, 0.0, 1.0, 0);
  EXPECT_TRUE(size_zero.empty());
  expect_rgba_near(size_zero.sample(0.5), 0.f, 0.f, 0.f, 0.f, 0.f);

  baked_transfer_function size_one(tf, 0.0, 1.0, 1);
  EXPECT_TRUE(size_one.empty());
  expect_rgba_near(size_one.sample(0.5), 0.f, 0.f, 0.f, 0.f, 0.f);

  baked_transfer_function size_two(tf, 0.0, 1.0, 2);
  EXPECT_FALSE(size_two.empty());
  EXPECT_EQ(size_two.size(), 2u);
}

TEST(BakedTransferFunctionTest, DefaultSizeIsDefaultsLutSize) {
  const transfer_function tf = three_point_tf();
  baked_transfer_function baked(tf, 0.0, 1.0);
  EXPECT_EQ(baked.size(), defaults::lut_size);
  EXPECT_DOUBLE_EQ(baked.domain_min(), 0.0);
  EXPECT_DOUBLE_EQ(baked.domain_max(), 1.0);
}

TEST(BakedTransferFunctionTest, AlphaRampRoundTripsAtMidpoint) {
  // Ramp alpha 0 -> 1 over [0, 255], baked at the default resolution: sampling
  // the middle of the domain must land very close to 0.5.
  transfer_function tf({make_point(0.0, 0.f, 0.f, 0.f, 0.f), make_point(255.0, 1.f, 1.f, 1.f, 1.f)});
  baked_transfer_function baked(tf, 0.0, 255.0);
  ASSERT_FALSE(baked.empty());
  EXPECT_NEAR(baked.sample(128.0).a, 0.5f, 0.01f);
  EXPECT_NEAR(baked.sample(128.0).r, 0.5f, 0.01f);
  EXPECT_NEAR(baked.sample(64.0).a, 0.25f, 0.01f);
  EXPECT_NEAR(baked.sample(191.0).a, 0.75f, 0.01f);
}

// ============================================================================
// gradient_opacity_ramp
// ============================================================================

TEST(GradientOpacityRampTest, DisabledRampAlwaysReturnsUnitFactor) {
  gradient_opacity_ramp ramp; // disabled by default
  EXPECT_FALSE(ramp.enabled);
  for (double gm : {-1.0, 0.0, 0.5, 10.0, 1e6}) {
    EXPECT_NEAR(ramp.factor(gm), 1.0f, 1e-7f);
  }
}

TEST(GradientOpacityRampTest, FactorIsZeroOutsideRampBounds) {
  gradient_opacity_ramp ramp;
  ramp.enabled = true;
  ramp.ramp0 = 1.0;
  ramp.ramp1 = 2.0;
  ramp.ramp2 = 4.0;
  ramp.plateau = 0.9;
  EXPECT_NEAR(ramp.factor(0.5), 0.f, 1e-7f);  // below ramp0
  EXPECT_NEAR(ramp.factor(4.5), 0.f, 1e-7f);  // above ramp2
  EXPECT_NEAR(ramp.factor(-1.0), 0.f, 1e-7f); // far below
}

TEST(GradientOpacityRampTest, FactorRisesLinearlyBetweenRampZeroAndRampOne) {
  gradient_opacity_ramp ramp;
  ramp.enabled = true;
  ramp.ramp0 = 1.0;
  ramp.ramp1 = 2.0;
  ramp.ramp2 = 4.0;
  ramp.plateau = 0.8;
  EXPECT_NEAR(ramp.factor(1.0), 0.f, 1e-6f);    // start of the rise
  EXPECT_NEAR(ramp.factor(1.25), 0.2f, 1e-6f);  // quarter
  EXPECT_NEAR(ramp.factor(1.5), 0.4f, 1e-6f);   // midpoint -> plateau/2
  EXPECT_NEAR(ramp.factor(1.75), 0.6f, 1e-6f);  // three quarters
}

TEST(GradientOpacityRampTest, FactorHoldsPlateauBetweenRampOneAndRampTwo) {
  gradient_opacity_ramp ramp;
  ramp.enabled = true;
  ramp.ramp0 = 1.0;
  ramp.ramp1 = 2.0;
  ramp.ramp2 = 4.0;
  ramp.plateau = 0.75;
  EXPECT_NEAR(ramp.factor(2.0), 0.75f, 1e-6f); // at ramp1
  EXPECT_NEAR(ramp.factor(3.0), 0.75f, 1e-6f); // inside the hold
  EXPECT_NEAR(ramp.factor(4.0), 0.75f, 1e-6f); // at ramp2 (inclusive)
}

TEST(GradientOpacityRampTest, ZeroSpanRiseReturnsPlateauAtThreshold) {
  gradient_opacity_ramp ramp;
  ramp.enabled = true;
  ramp.ramp0 = 2.0;
  ramp.ramp1 = 2.0; // degenerate rise
  ramp.ramp2 = 4.0;
  ramp.plateau = 0.9;
  EXPECT_NEAR(ramp.factor(2.0), 0.9f, 1e-6f);
  EXPECT_NEAR(ramp.factor(3.0), 0.9f, 1e-6f);
  EXPECT_NEAR(ramp.factor(1.9), 0.f, 1e-7f);
}
