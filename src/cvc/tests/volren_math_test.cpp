// Unit tests for the cvc::volren math layer:
//   - vec3d arithmetic, dot/cross handedness, normalized() epsilon contract
//   - mat4 identity/from_row_major, point-vs-vector transforms,
//     affine_inverse round-trips and error contracts, transform_normal
//   - depth_to_window_z (camera.h) ortho/perspective mapping, clamping, throws
//   - gradient_opacity_ramp piecewise factor (transfer_function.h)

#include <cvc/volren/camera.h>
#include <cvc/volren/transfer_function.h>
#include <cvc/volren/types.h>

#include <gtest/gtest.h>

#include <cmath>
#include <random>

using namespace cvc::volren;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Row-major matrix product (points are column vectors, so composed transforms
// apply right-to-left).  Local helper; mat4 itself does not expose operator*.
mat4 mat_mul(const mat4 &A, const mat4 &B) {
  mat4 out;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k)
        s += A.m[r * 4 + k] * B.m[k * 4 + c];
      out.m[r * 4 + c] = s;
    }
  return out;
}

mat4 make_translation(double tx, double ty, double tz) {
  mat4 out;
  out.m[3] = tx;
  out.m[7] = ty;
  out.m[11] = tz;
  return out;
}

mat4 make_scale(double sx, double sy, double sz) {
  mat4 out;
  out.m[0] = sx;
  out.m[5] = sy;
  out.m[10] = sz;
  return out;
}

mat4 make_rotation_z(double radians) {
  mat4 out;
  const double c = std::cos(radians), s = std::sin(radians);
  out.m[0] = c;
  out.m[1] = -s;
  out.m[4] = s;
  out.m[5] = c;
  return out;
}

mat4 make_rotation_x(double radians) {
  mat4 out;
  const double c = std::cos(radians), s = std::sin(radians);
  out.m[5] = c;
  out.m[6] = -s;
  out.m[9] = s;
  out.m[10] = c;
  return out;
}

void expect_vec_near(const vec3d &actual, const vec3d &expected, double tol) {
  EXPECT_NEAR(actual.x, expected.x, tol);
  EXPECT_NEAR(actual.y, expected.y, tol);
  EXPECT_NEAR(actual.z, expected.z, tol);
}

} // namespace

// ============================================================================
// vec3d
// ============================================================================

TEST(Vec3dMath, DefaultAndArrayConstructionRoundTrip) {
  const vec3d zero;
  EXPECT_DOUBLE_EQ(zero.x, 0.0);
  EXPECT_DOUBLE_EQ(zero.y, 0.0);
  EXPECT_DOUBLE_EQ(zero.z, 0.0);

  const std::array<double, 3> a{1.5, -2.25, 3.75};
  const vec3d v(a);
  EXPECT_DOUBLE_EQ(v.x, 1.5);
  EXPECT_DOUBLE_EQ(v.y, -2.25);
  EXPECT_DOUBLE_EQ(v.z, 3.75);

  const std::array<double, 3> back = v.to_array();
  EXPECT_DOUBLE_EQ(back[0], 1.5);
  EXPECT_DOUBLE_EQ(back[1], -2.25);
  EXPECT_DOUBLE_EQ(back[2], 3.75);
}

TEST(Vec3dMath, ArithmeticOperatorsAreComponentwise) {
  const vec3d a{1.0, 2.0, 3.0};
  const vec3d b{-4.0, 0.5, 2.0};

  expect_vec_near(a + b, {-3.0, 2.5, 5.0}, 0.0);
  expect_vec_near(a - b, {5.0, 1.5, 1.0}, 0.0);
  expect_vec_near(a * 2.5, {2.5, 5.0, 7.5}, 0.0);
  expect_vec_near(-a, {-1.0, -2.0, -3.0}, 0.0);

  vec3d c = a;
  c += b;
  expect_vec_near(c, {-3.0, 2.5, 5.0}, 0.0);
}

TEST(Vec3dMath, DotProductMatchesHandComputation) {
  const vec3d a{1.0, 2.0, 3.0};
  const vec3d b{4.0, -5.0, 6.0};
  EXPECT_DOUBLE_EQ(dot(a, b), 1.0 * 4.0 + 2.0 * -5.0 + 3.0 * 6.0); // 12
  EXPECT_DOUBLE_EQ(dot(a, a), 14.0);

  // Orthogonal axes.
  EXPECT_DOUBLE_EQ(dot(vec3d{1, 0, 0}, vec3d{0, 1, 0}), 0.0);
  EXPECT_DOUBLE_EQ(dot(vec3d{1, 0, 0}, vec3d{0, 0, 1}), 0.0);
}

TEST(Vec3dMath, CrossProductIsRightHanded) {
  const vec3d x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};

  // Right-handed cyclic identities: x cross y = z, y cross z = x, z cross x = y.
  expect_vec_near(cross(x, y), z, 0.0);
  expect_vec_near(cross(y, z), x, 0.0);
  expect_vec_near(cross(z, x), y, 0.0);

  // Anticommutative.
  expect_vec_near(cross(y, x), -z, 0.0);

  // Parallel vectors give zero.
  expect_vec_near(cross(x, x), {0, 0, 0}, 0.0);

  // Result is perpendicular to both inputs.
  const vec3d a{1.0, 2.0, 3.0}, b{-2.0, 1.5, 0.25};
  const vec3d c = cross(a, b);
  EXPECT_NEAR(dot(c, a), 0.0, 1e-12);
  EXPECT_NEAR(dot(c, b), 0.0, 1e-12);
}

TEST(Vec3dMath, NormalizedGivesUnitLengthPreservingDirection) {
  const vec3d v{3.0, -4.0, 12.0}; // length 13
  const vec3d n = normalized(v);
  EXPECT_NEAR(length(n), 1.0, 1e-12);
  expect_vec_near(n, {3.0 / 13.0, -4.0 / 13.0, 12.0 / 13.0}, 1e-12);

  // Already-unit vector is unchanged.
  expect_vec_near(normalized(vec3d{0, 1, 0}), {0, 1, 0}, 1e-15);
}

TEST(Vec3dMath, NormalizedReturnsZeroAtOrBelowEpsilon) {
  // The legacy vrNormalize contract: length <= epsilon normalizes to zero.
  expect_vec_near(normalized(vec3d{0, 0, 0}), {0, 0, 0}, 0.0);

  // Below default epsilon (1e-12).
  expect_vec_near(normalized(vec3d{1e-13, 0, 0}), {0, 0, 0}, 0.0);

  // Just above default epsilon normalizes normally.
  expect_vec_near(normalized(vec3d{1e-11, 0, 0}), {1, 0, 0}, 1e-12);

  // Custom epsilon: a length-0.5 vector is "flat" against epsilon 1.0...
  expect_vec_near(normalized(vec3d{0.5, 0, 0}, 1.0), {0, 0, 0}, 0.0);
  // ...but normalizes against a smaller one.
  expect_vec_near(normalized(vec3d{0.5, 0, 0}, 0.25), {1, 0, 0}, 1e-15);
}

// ============================================================================
// mat4
// ============================================================================

TEST(Mat4Transform, DefaultConstructionIsIdentity) {
  const mat4 def;
  EXPECT_TRUE(def.is_identity());
  EXPECT_TRUE(mat4::identity().is_identity());
  EXPECT_TRUE(def.is_affine());

  // Identity is a no-op for points and vectors.
  const vec3d p{1.5, -2.0, 0.25};
  expect_vec_near(def.transform_point(p), p, 0.0);
  expect_vec_near(def.transform_vector(p), p, 0.0);

  // A perturbed entry breaks is_identity.
  mat4 not_id;
  not_id.m[3] = 1e-9;
  EXPECT_FALSE(not_id.is_identity());
}

TEST(Mat4Transform, FromRowMajorCopiesAllSixteenEntries) {
  double values[16];
  for (int i = 0; i < 16; ++i)
    values[i] = double(i) + 0.5;
  const mat4 M = mat4::from_row_major(values);
  for (int i = 0; i < 16; ++i)
    EXPECT_DOUBLE_EQ(M.m[i], double(i) + 0.5);
  EXPECT_FALSE(M.is_identity());

  // Row-major layout means m[3], m[7], m[11] are the translation column.
  const double t[16] = {1, 0, 0, 7, 0, 1, 0, 8, 0, 0, 1, 9, 0, 0, 0, 1};
  const mat4 T = mat4::from_row_major(t);
  expect_vec_near(T.transform_point({0, 0, 0}), {7, 8, 9}, 0.0);
}

TEST(Mat4Transform, TransformVectorIgnoresTranslation) {
  const mat4 T = make_translation(10.0, -20.0, 30.0);
  const vec3d p{1.0, 2.0, 3.0};

  expect_vec_near(T.transform_point(p), {11.0, -18.0, 33.0}, 0.0);
  expect_vec_near(T.transform_vector(p), p, 0.0); // directions unaffected

  // Under a general affine M, transform_vector is the linear part:
  // M*p - M*0 == transform_vector(p).
  const mat4 M =
      mat_mul(make_translation(1.0, 2.0, 3.0),
              mat_mul(make_rotation_z(0.3), make_scale(2.0, 0.5, 1.5)));
  const vec3d lhs = M.transform_point(p) - M.transform_point({0, 0, 0});
  expect_vec_near(lhs, M.transform_vector(p), 1e-12);
}

TEST(Mat4Transform, AffineInverseRoundTripsTranslationRotationScale) {
  // Compose translation * rotZ * rotX * nonuniform scale.
  const mat4 M = mat_mul(
      make_translation(0.75, -1.5, 2.0),
      mat_mul(make_rotation_z(kPi / 5.0),
              mat_mul(make_rotation_x(-kPi / 7.0), make_scale(2.0, 0.5, 3.0))));
  const mat4 inv = M.affine_inverse();

  // M * M^-1 is the identity (entrywise).
  const mat4 prod = mat_mul(M, inv);
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      EXPECT_NEAR(prod.m[r * 4 + c], r == c ? 1.0 : 0.0, 1e-12)
          << "entry (" << r << "," << c << ")";

  // Round-trip random points through M then M^-1.
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist(-5.0, 5.0);
  for (int i = 0; i < 16; ++i) {
    const vec3d p{dist(rng), dist(rng), dist(rng)};
    expect_vec_near(inv.transform_point(M.transform_point(p)), p, 1e-10);
  }

  // The inverse of a pure translation is the negative translation.
  const mat4 Tinv = make_translation(3.0, -4.0, 5.0).affine_inverse();
  expect_vec_near(Tinv.transform_point({0, 0, 0}), {-3.0, 4.0, -5.0}, 1e-15);
}

TEST(Mat4Transform, AffineInverseThrowsOnNonAffineBottomRow) {
  // Perturbed bottom-row entries (projective-style matrices).
  {
    mat4 M;
    M.m[12] = 0.1;
    EXPECT_THROW(M.affine_inverse(), cvc::volren_error);
  }
  {
    mat4 M;
    M.m[14] = -1.0; // classic perspective bottom row
    EXPECT_THROW(M.affine_inverse(), cvc::volren_error);
  }
  {
    mat4 M;
    M.m[15] = 2.0; // homogeneous w-scale
    EXPECT_THROW(M.affine_inverse(), cvc::volren_error);
  }
}

TEST(Mat4Transform, AffineInverseThrowsOnSingularLinearPart) {
  // Zero scale on one axis collapses the linear part (det exactly 0).
  EXPECT_THROW(make_scale(1.0, 1.0, 0.0).affine_inverse(), cvc::volren_error);
  EXPECT_THROW(make_scale(0.0, 0.0, 0.0).affine_inverse(), cvc::volren_error);

  // Rank-deficient: two identical rows.
  mat4 M;
  M.m[0] = 1.0;
  M.m[1] = 2.0;
  M.m[2] = 3.0;
  M.m[4] = 1.0;
  M.m[5] = 2.0;
  M.m[6] = 3.0;
  M.m[8] = 0.0;
  M.m[9] = 0.0;
  M.m[10] = 1.0;
  EXPECT_THROW(M.affine_inverse(), cvc::volren_error);
}

TEST(Mat4Transform, TransformNormalEqualsRotationForPureRotation) {
  // For a pure rotation, transpose(inverse(R)) == R, so normals transform
  // exactly like vectors.
  const mat4 R = mat_mul(make_rotation_z(0.6), make_rotation_x(-1.1));
  const vec3d samples[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1.0, -2.0, 0.5}};
  for (const vec3d &v : samples)
    expect_vec_near(R.transform_normal(v), R.transform_vector(v), 1e-12);

  // Translation does not change the normal transform (linear part only).
  const mat4 TR = mat_mul(make_translation(5.0, 6.0, 7.0), R);
  expect_vec_near(TR.transform_normal({1.0, -2.0, 0.5}),
                  R.transform_vector({1.0, -2.0, 0.5}), 1e-12);
}

TEST(Mat4Transform, TransformNormalKeepsScaledPlanePerpendicular) {
  // A plane's tangent transforms with M, its normal with transpose(inv(M));
  // perpendicularity must survive a nonuniform scale.
  const vec3d t{1.0, 2.0, 3.0};
  const vec3d n{3.0, 0.0, -1.0}; // dot(t, n) == 0
  ASSERT_DOUBLE_EQ(dot(t, n), 0.0);

  const mat4 M = mat_mul(
      make_translation(1.0, 2.0, 3.0),
      mat_mul(make_rotation_z(kPi / 3.0), make_scale(2.0, 0.5, 4.0)));

  const vec3d t_w = M.transform_vector(t);
  const vec3d n_w = M.transform_normal(n);
  EXPECT_NEAR(dot(t_w, n_w), 0.0, 1e-12);

  // Sanity: naively transforming the normal as a vector does NOT stay
  // perpendicular under nonuniform scale -- that is why transform_normal exists.
  const mat4 S = make_scale(2.0, 1.0, 1.0);
  const vec3d t2{1.0, 1.0, 0.0}, n2{1.0, -1.0, 0.0};
  EXPECT_GT(std::fabs(dot(S.transform_vector(t2), S.transform_vector(n2))), 1.0);
  EXPECT_NEAR(dot(S.transform_vector(t2), S.transform_normal(n2)), 0.0, 1e-12);
}

// ============================================================================
// depth_to_window_z
// ============================================================================

TEST(DepthToWindowZ, ZeroAtNearOneAtFarForBothProjections) {
  const double n = 0.5, f = 20.0;
  EXPECT_NEAR(depth_to_window_z(n, n, f, camera::projection_type::orthographic), 0.0, 1e-15);
  EXPECT_NEAR(depth_to_window_z(f, n, f, camera::projection_type::orthographic), 1.0, 1e-15);
  EXPECT_NEAR(depth_to_window_z(n, n, f, camera::projection_type::perspective), 0.0, 1e-15);
  EXPECT_NEAR(depth_to_window_z(f, n, f, camera::projection_type::perspective), 1.0, 1e-15);
}

TEST(DepthToWindowZ, OrthographicIsExactlyLinear) {
  const double n = 1.0, f = 11.0;
  for (int i = 0; i <= 10; ++i) {
    const double d = n + (f - n) * double(i) / 10.0;
    EXPECT_DOUBLE_EQ(depth_to_window_z(d, n, f, camera::projection_type::orthographic),
                     (d - n) / (f - n))
        << "depth " << d;
  }
  // Midpoint depth maps exactly to 0.5.
  EXPECT_DOUBLE_EQ(depth_to_window_z(6.0, n, f, camera::projection_type::orthographic), 0.5);
}

TEST(DepthToWindowZ, PerspectiveMatchesClosedForm) {
  const double n = 0.25, f = 50.0;
  const double depths[] = {0.25, 0.5, 1.0, 2.0, 5.0, 12.5, 30.0, 50.0};
  for (const double d : depths) {
    const double expected = (f / (f - n)) * (1.0 - n / d);
    EXPECT_NEAR(depth_to_window_z(d, n, f, camera::projection_type::perspective), expected, 1e-14)
        << "depth " << d;
  }
  // Perspective depth is nonlinear: the midpoint of the clip range lands well
  // above window z 0.5 (near-plane precision bias).
  const double mid = 0.5 * (n + f);
  EXPECT_GT(depth_to_window_z(mid, n, f, camera::projection_type::perspective), 0.9);
}

TEST(DepthToWindowZ, StrictlyMonotonicBetweenNearAndFar) {
  const double n = 0.5, f = 40.0;
  for (const auto proj :
       {camera::projection_type::orthographic, camera::projection_type::perspective}) {
    double prev = depth_to_window_z(n, n, f, proj);
    for (int i = 1; i <= 64; ++i) {
      const double d = n + (f - n) * double(i) / 64.0;
      const double z = depth_to_window_z(d, n, f, proj);
      EXPECT_GT(z, prev) << "depth " << d;
      EXPECT_GE(z, 0.0);
      // Perspective at d == far can land a ULP above 1.0 (the closed form
      // f/(f-n)*(1-n/d) rounds); allow rounding-scale slack only.
      EXPECT_LE(z, 1.0 + 1e-12);
      prev = z;
    }
  }
}

TEST(DepthToWindowZ, ClampsDepthsOutsideClipRange) {
  const double n = 1.0, f = 10.0;
  for (const auto proj :
       {camera::projection_type::orthographic, camera::projection_type::perspective}) {
    EXPECT_DOUBLE_EQ(depth_to_window_z(0.001, n, f, proj), 0.0); // in front of near
    EXPECT_DOUBLE_EQ(depth_to_window_z(-5.0, n, f, proj), 0.0);  // behind the eye
    EXPECT_DOUBLE_EQ(depth_to_window_z(1000.0, n, f, proj), 1.0); // beyond far
  }
}

TEST(DepthToWindowZ, ThrowsOnInvalidClipRange) {
  const auto persp = camera::projection_type::perspective;
  EXPECT_THROW(depth_to_window_z(1.0, 0.0, 10.0, persp), cvc::volren_error);  // near == 0
  EXPECT_THROW(depth_to_window_z(1.0, -1.0, 10.0, persp), cvc::volren_error); // near < 0
  EXPECT_THROW(depth_to_window_z(1.0, 2.0, 2.0, persp), cvc::volren_error);   // far == near
  EXPECT_THROW(depth_to_window_z(1.0, 5.0, 2.0, persp), cvc::volren_error);   // far < near
  EXPECT_THROW(depth_to_window_z(1.0, 0.0, 10.0, camera::projection_type::orthographic),
               cvc::volren_error);
}

// ============================================================================
// gradient_opacity_ramp
// ============================================================================

TEST(GradientOpacityRamp, DisabledReturnsOneEverywhere) {
  gradient_opacity_ramp ramp; // enabled defaults to false
  ramp.ramp0 = 1.0;
  ramp.ramp1 = 3.0;
  ramp.ramp2 = 5.0;
  for (const double g : {-10.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 100.0})
    EXPECT_FLOAT_EQ(ramp.factor(g), 1.0f) << "magnitude " << g;
}

TEST(GradientOpacityRamp, PiecewiseFactorAcrossTheRamp) {
  gradient_opacity_ramp ramp;
  ramp.enabled = true;
  ramp.ramp0 = 1.0;
  ramp.ramp1 = 3.0;
  ramp.ramp2 = 5.0;
  const float plateau = static_cast<float>(ramp.plateau); // defaults::gradient_plateau = 0.9
  EXPECT_FLOAT_EQ(plateau, 0.9f);

  // Below ramp0: zero.
  EXPECT_FLOAT_EQ(ramp.factor(0.0), 0.0f);
  EXPECT_FLOAT_EQ(ramp.factor(0.999), 0.0f);
  // At ramp0 the linear rise starts from zero.
  EXPECT_NEAR(ramp.factor(1.0), 0.0f, 1e-7f);
  // Midway up the rise: plateau / 2.
  EXPECT_NEAR(ramp.factor(2.0), plateau * 0.5f, 1e-6f);
  // Quarter of the way up.
  EXPECT_NEAR(ramp.factor(1.5), plateau * 0.25f, 1e-6f);
  // At ramp1 and between ramp1 and ramp2: full plateau.
  EXPECT_FLOAT_EQ(ramp.factor(3.0), plateau);
  EXPECT_FLOAT_EQ(ramp.factor(4.0), plateau);
  EXPECT_FLOAT_EQ(ramp.factor(5.0), plateau); // ramp2 is inclusive
  // Above ramp2: zero again.
  EXPECT_FLOAT_EQ(ramp.factor(5.001), 0.0f);
  EXPECT_FLOAT_EQ(ramp.factor(1e6), 0.0f);
}

TEST(GradientOpacityRamp, CustomPlateauScalesTheRise) {
  gradient_opacity_ramp ramp;
  ramp.enabled = true;
  ramp.ramp0 = 0.0;
  ramp.ramp1 = 10.0;
  ramp.ramp2 = 20.0;
  ramp.plateau = 0.5;
  EXPECT_NEAR(ramp.factor(5.0), 0.25f, 1e-6f);
  EXPECT_FLOAT_EQ(ramp.factor(10.0), 0.5f);
  EXPECT_FLOAT_EQ(ramp.factor(15.0), 0.5f);
}

TEST(GradientOpacityRamp, DegenerateRiseGivesImmediatePlateau) {
  gradient_opacity_ramp ramp;
  ramp.enabled = true;
  ramp.ramp0 = 2.0;
  ramp.ramp1 = 2.0; // zero-width rise
  ramp.ramp2 = 5.0;

  const float plateau = static_cast<float>(ramp.plateau);
  EXPECT_FLOAT_EQ(ramp.factor(1.999), 0.0f);   // still zero below ramp0
  EXPECT_FLOAT_EQ(ramp.factor(2.0), plateau);  // steps straight to the plateau
  EXPECT_FLOAT_EQ(ramp.factor(3.5), plateau);
  EXPECT_FLOAT_EQ(ramp.factor(5.0), plateau);
  EXPECT_FLOAT_EQ(ramp.factor(5.5), 0.0f);
}
