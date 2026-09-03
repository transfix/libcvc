/*
  Unit tests for cvc::volren::camera -- view basis construction, per-pixel
  ray generation (perspective + orthographic), pose interop, depth-to-window-z
  conversion, and degenerate-pose error handling.
*/

#include <cmath>
#include <cvc/volren/camera.h>
#include <gtest/gtest.h>

using cvc::volren::camera;
using cvc::volren::cross;
using cvc::volren::dot;
using cvc::volren::length;
using cvc::volren::normalized;
using cvc::volren::ray;
using cvc::volren::vec3d;
using cvc::volren::view_basis;

namespace {

constexpr double kPi = 3.14159265358979323846;

// A well-posed reference camera: on the -Y axis, Z-up, looking at the origin.
camera minus_y_camera() {
  camera c;
  c.eye = {0.0, -4.0, 0.0};
  c.focal = {0.0, 0.0, 0.0};
  c.up = {0.0, 0.0, 1.0};
  return c;
}

// A deliberately oblique pose (nothing axis-aligned, up not orthogonal to
// the view direction) to exercise the general orthonormalization path.
camera oblique_camera() {
  camera c;
  c.eye = {1.3, -2.7, 2.1};
  c.focal = {-0.4, 0.6, -0.9};
  c.up = {0.2, 0.3, 1.0}; // not unit, not perpendicular to view
  return c;
}

void expect_vec_near(const vec3d &a, const vec3d &b, double tol) {
  EXPECT_NEAR(a.x, b.x, tol);
  EXPECT_NEAR(a.y, b.y, tol);
  EXPECT_NEAR(a.z, b.z, tol);
}

} // namespace

// ============================================================================
// basis(): right-handed orthonormal frame
// ============================================================================

TEST(CameraBasis, ZUpMinusYPoseGivesAxisAlignedFrame) {
  const view_basis b = minus_y_camera().basis();
  // Looking along +Y with Z-up: right = +X, true_up = +Z, back = -Y.
  expect_vec_near(b.right, vec3d(1.0, 0.0, 0.0), 1e-15);
  expect_vec_near(b.true_up, vec3d(0.0, 0.0, 1.0), 1e-15);
  expect_vec_near(b.back, vec3d(0.0, -1.0, 0.0), 1e-15);
}

TEST(CameraBasis, ObliquePoseFrameIsOrthonormal) {
  const view_basis b = oblique_camera().basis();
  EXPECT_NEAR(length(b.right), 1.0, 1e-12);
  EXPECT_NEAR(length(b.true_up), 1.0, 1e-12);
  EXPECT_NEAR(length(b.back), 1.0, 1e-12);
  EXPECT_NEAR(dot(b.right, b.true_up), 0.0, 1e-12);
  EXPECT_NEAR(dot(b.right, b.back), 0.0, 1e-12);
  EXPECT_NEAR(dot(b.true_up, b.back), 0.0, 1e-12);
}

TEST(CameraBasis, FrameIsRightHanded) {
  const view_basis b = oblique_camera().basis();
  // right x true_up == back for a right-handed orthonormal frame.
  expect_vec_near(cross(b.right, b.true_up), b.back, 1e-12);
  // And cyclically: true_up x back == right, back x right == true_up.
  expect_vec_near(cross(b.true_up, b.back), b.right, 1e-12);
  expect_vec_near(cross(b.back, b.right), b.true_up, 1e-12);
}

TEST(CameraBasis, BackPointsFromFocalTowardEye) {
  const camera c = oblique_camera();
  const view_basis b = c.basis();
  const vec3d expected = normalized(vec3d(c.eye) - vec3d(c.focal));
  expect_vec_near(b.back, expected, 1e-15);
}

TEST(CameraBasis, NonUnitUpIsNormalizedAway) {
  camera c = minus_y_camera();
  c.up = {0.0, 0.0, 7.5}; // same direction, different length
  const view_basis b = c.basis();
  expect_vec_near(b.true_up, vec3d(0.0, 0.0, 1.0), 1e-15);
  expect_vec_near(b.right, vec3d(1.0, 0.0, 0.0), 1e-15);
}

TEST(CameraBasis, UpNotPerpendicularToViewStillYieldsOrthogonalTrueUp) {
  camera c = minus_y_camera();
  c.up = {0.0, 0.5, 1.0}; // leans toward the focal point
  const view_basis b = c.basis();
  // true_up is the projection of up into the plane perpendicular to back.
  EXPECT_NEAR(dot(b.true_up, b.back), 0.0, 1e-12);
  EXPECT_GT(dot(b.true_up, normalized(vec3d(c.up))), 0.0);
}

TEST(CameraBasis, DefaultFieldValuesMatchDocumentedDefaults) {
  const camera c;
  EXPECT_DOUBLE_EQ(c.eye[0], 0.0);
  EXPECT_DOUBLE_EQ(c.eye[1], -4.0);
  EXPECT_DOUBLE_EQ(c.eye[2], 0.0);
  EXPECT_DOUBLE_EQ(c.focal[0], 0.0);
  EXPECT_DOUBLE_EQ(c.focal[1], 0.0);
  EXPECT_DOUBLE_EQ(c.focal[2], 0.0);
  EXPECT_DOUBLE_EQ(c.up[0], 0.0);
  EXPECT_DOUBLE_EQ(c.up[1], 0.0);
  EXPECT_DOUBLE_EQ(c.up[2], 1.0); // Z-up
  EXPECT_EQ(c.projection, camera::projection_type::perspective);
  EXPECT_DOUBLE_EQ(c.vfov_degrees, 30.0);
  EXPECT_DOUBLE_EQ(c.parallel_scale, 1.0);
  EXPECT_EQ(c.width, 512);
  EXPECT_EQ(c.height, 512);
}

// Documents CURRENT behavior: the default pose puts the eye on the +Z axis
// while up is also +Z, so up is parallel to the view direction and basis()
// throws.  A default-constructed camera therefore cannot generate rays until
// the pose is changed.  (Flagged as a suspected API wart; see camera.h:36-38.)
TEST(CameraBasis, DefaultPoseIsValidAndZUp) {
  // The default eye sits on -Y looking north with +Z up, so a
  // default-constructed camera has a well-defined frame out of the box.
  const camera c;
  const view_basis b = c.basis();
  EXPECT_NEAR(b.right.x, 1.0, 1e-12);
  EXPECT_NEAR(b.true_up.z, 1.0, 1e-12);
  EXPECT_NEAR(b.back.y, -1.0, 1e-12);
}

// ============================================================================
// generate_ray: perspective
// ============================================================================

TEST(CameraRay, SinglePixelRasterCenterRayMatchesViewDirectionExactly) {
  camera c = minus_y_camera();
  c.width = 1;
  c.height = 1;
  const ray r = c.generate_ray(0, 0);
  // Axis-aligned pose: u = v = 0, so direction is exactly normalize(focal-eye).
  EXPECT_DOUBLE_EQ(r.direction.x, 0.0);
  EXPECT_DOUBLE_EQ(r.direction.y, 1.0);
  EXPECT_DOUBLE_EQ(r.direction.z, 0.0);
  EXPECT_DOUBLE_EQ(r.origin.x, c.eye[0]);
  EXPECT_DOUBLE_EQ(r.origin.y, c.eye[1]);
  EXPECT_DOUBLE_EQ(r.origin.z, c.eye[2]);
}

TEST(CameraRay, SinglePixelRasterObliquePoseRayIsViewDirection) {
  camera c = oblique_camera();
  c.width = 1;
  c.height = 1;
  const ray r = c.generate_ray(0, 0);
  const vec3d expected = normalized(vec3d(c.focal) - vec3d(c.eye));
  expect_vec_near(r.direction, expected, 1e-12);
  expect_vec_near(r.origin, vec3d(c.eye), 0.0);
}

TEST(CameraRay, PerspectiveOriginIsAlwaysEye) {
  camera c = oblique_camera();
  c.width = 8;
  c.height = 8;
  for (int py = 0; py < c.height; py += 7) {
    for (int px = 0; px < c.width; px += 7) {
      const ray r = c.generate_ray(px, py);
      expect_vec_near(r.origin, vec3d(c.eye), 0.0);
      EXPECT_NEAR(length(r.direction), 1.0, 1e-12);
    }
  }
}

TEST(CameraRay, TopLeftOriginTopRowRaysPointUpBottomRowDown) {
  camera c = minus_y_camera();
  c.width = 2;
  c.height = 2;
  const view_basis b = c.basis();
  // py = 0 is the TOP row -> positive true_up component.
  EXPECT_GT(dot(c.generate_ray(0, 0).direction, b.true_up), 0.0);
  EXPECT_GT(dot(c.generate_ray(1, 0).direction, b.true_up), 0.0);
  // py = 1 is the BOTTOM row -> negative true_up component.
  EXPECT_LT(dot(c.generate_ray(0, 1).direction, b.true_up), 0.0);
  EXPECT_LT(dot(c.generate_ray(1, 1).direction, b.true_up), 0.0);
}

TEST(CameraRay, LeftColumnRaysPointLeftRightColumnRight) {
  camera c = minus_y_camera();
  c.width = 2;
  c.height = 2;
  const view_basis b = c.basis();
  // px = 0 is the LEFT column -> negative right component.
  EXPECT_LT(dot(c.generate_ray(0, 0).direction, b.right), 0.0);
  EXPECT_LT(dot(c.generate_ray(0, 1).direction, b.right), 0.0);
  // px = 1 -> positive right component.
  EXPECT_GT(dot(c.generate_ray(1, 0).direction, b.right), 0.0);
  EXPECT_GT(dot(c.generate_ray(1, 1).direction, b.right), 0.0);
}

TEST(CameraRay, VerticalFovSetsUpOverForwardRatio) {
  camera c = minus_y_camera();
  c.width = 2; // even, tall and thin
  c.height = 64;
  c.vfov_degrees = 40.0;
  const view_basis b = c.basis();
  const vec3d forward = -b.back;
  const double tan_half = std::tan(0.5 * c.vfov_degrees * kPi / 180.0);

  // Top-row pixel center: v = 1 - 1/h.  The unnormalized ray is
  // forward + right*(u*tan*aspect) + true_up*(v*tan), so the ratio of the
  // true_up component to the forward component is exactly v*tan(vfov/2)
  // (normalization cancels in the ratio).
  const double v = 1.0 - 1.0 / double(c.height);
  const ray r = c.generate_ray(c.width / 2, 0);
  const double up_over_forward = dot(r.direction, b.true_up) / dot(r.direction, forward);
  EXPECT_NEAR(up_over_forward, v * tan_half, 1e-9);
}

TEST(CameraRay, HorizontalSpreadScalesWithAspectRatio) {
  camera c = minus_y_camera();
  c.width = 96; // wide raster -> aspect = 2
  c.height = 48;
  c.vfov_degrees = 30.0;
  const view_basis b = c.basis();
  const vec3d forward = -b.back;
  const double tan_half = std::tan(0.5 * c.vfov_degrees * kPi / 180.0);
  const double aspect = double(c.width) / double(c.height);
  EXPECT_NEAR(c.aspect(), aspect, 0.0);

  // Left-column pixel center: u = 1/w - 1.
  const double u = 1.0 / double(c.width) - 1.0;
  const ray r = c.generate_ray(0, c.height / 2);
  const double right_over_forward = dot(r.direction, b.right) / dot(r.direction, forward);
  EXPECT_NEAR(right_over_forward, u * tan_half * aspect, 1e-9);
}

TEST(CameraRay, MirroredPixelsGiveMirroredDirections) {
  camera c = oblique_camera();
  c.width = 6;
  c.height = 6;
  const view_basis b = c.basis();
  // Pixels mirrored about the raster center have opposite lateral components
  // of equal magnitude.
  const ray a = c.generate_ray(1, 2);
  const ray m = c.generate_ray(4, 3); // (w-1-1, h-1-2)
  EXPECT_NEAR(dot(a.direction, b.right), -dot(m.direction, b.right), 1e-12);
  EXPECT_NEAR(dot(a.direction, b.true_up), -dot(m.direction, b.true_up), 1e-12);
  EXPECT_NEAR(dot(a.direction, b.back), dot(m.direction, b.back), 1e-12);
}

// ============================================================================
// generate_ray: orthographic
// ============================================================================

TEST(CameraOrtho, AllRayDirectionsEqualForward) {
  camera c = oblique_camera();
  c.projection = camera::projection_type::orthographic;
  c.width = 5;
  c.height = 4;
  const view_basis b = c.basis();
  const vec3d forward = -b.back;
  for (int py = 0; py < c.height; ++py) {
    for (int px = 0; px < c.width; ++px) {
      const ray r = c.generate_ray(px, py);
      EXPECT_DOUBLE_EQ(r.direction.x, forward.x);
      EXPECT_DOUBLE_EQ(r.direction.y, forward.y);
      EXPECT_DOUBLE_EQ(r.direction.z, forward.z);
    }
  }
}

TEST(CameraOrtho, OriginsLieInThePlaneOfTheEye) {
  camera c = oblique_camera();
  c.projection = camera::projection_type::orthographic;
  c.width = 4;
  c.height = 4;
  c.parallel_scale = 2.5;
  const view_basis b = c.basis();
  for (int py = 0; py < c.height; ++py) {
    for (int px = 0; px < c.width; ++px) {
      const vec3d offset = c.generate_ray(px, py).origin - vec3d(c.eye);
      EXPECT_NEAR(dot(offset, b.back), 0.0, 1e-12);
    }
  }
}

TEST(CameraOrtho, AdjacentPixelOriginSpacingMatchesParallelScale) {
  camera c = minus_y_camera();
  c.projection = camera::projection_type::orthographic;
  c.width = 4;
  c.height = 4;
  c.parallel_scale = 2.0;
  const view_basis b = c.basis();

  // Horizontal neighbors: delta = right * (2/w * parallel_scale * aspect).
  const vec3d dx = c.generate_ray(2, 1).origin - c.generate_ray(1, 1).origin;
  const double expected_dx = 2.0 / double(c.width) * c.parallel_scale * c.aspect();
  expect_vec_near(dx, b.right * expected_dx, 1e-12);

  // Vertical neighbors going DOWN the raster move along -true_up.
  const vec3d dy = c.generate_ray(1, 2).origin - c.generate_ray(1, 1).origin;
  const double expected_dy = 2.0 / double(c.height) * c.parallel_scale;
  expect_vec_near(dy, b.true_up * (-expected_dy), 1e-12);
}

TEST(CameraOrtho, ParallelScaleScalesTheFootprint) {
  camera base = minus_y_camera();
  base.projection = camera::projection_type::orthographic;
  base.width = 8;
  base.height = 8;
  base.parallel_scale = 1.0;

  camera scaled = base;
  scaled.parallel_scale = 3.0;

  const vec3d eye(base.eye);
  const vec3d off1 = base.generate_ray(0, 0).origin - eye;
  const vec3d off3 = scaled.generate_ray(0, 0).origin - eye;
  expect_vec_near(off3, off1 * 3.0, 1e-12);
}

TEST(CameraOrtho, TopRowOriginOffsetIsPixelCenterFractionOfHalfHeight) {
  camera c = minus_y_camera();
  c.projection = camera::projection_type::orthographic;
  c.width = 2;
  c.height = 10;
  c.parallel_scale = 4.0;
  const view_basis b = c.basis();
  // Top-row pixel center sits at v = 1 - 1/h of the half-height above the eye.
  const double v = 1.0 - 1.0 / double(c.height);
  const vec3d offset = c.generate_ray(0, 0).origin - vec3d(c.eye);
  EXPECT_NEAR(dot(offset, b.true_up), v * c.parallel_scale, 1e-12);
}

TEST(CameraOrtho, SinglePixelRasterOriginIsEye) {
  camera c = minus_y_camera();
  c.projection = camera::projection_type::orthographic;
  c.width = 1;
  c.height = 1;
  const ray r = c.generate_ray(0, 0);
  expect_vec_near(r.origin, vec3d(c.eye), 0.0);
  expect_vec_near(r.direction, vec3d(0.0, 1.0, 0.0), 1e-15);
}

// ============================================================================
// from_pose
// ============================================================================

TEST(CameraFromPose, RoundTripsEyeFocalUpFovAndRaster) {
  const double eye[3] = {1.5, -2.25, 3.125};
  const double focal[3] = {-0.5, 0.75, 0.25};
  const double up[3] = {0.1, 0.2, 0.9};
  const camera c = camera::from_pose(eye, focal, up, 42.5, 96, 64);

  EXPECT_DOUBLE_EQ(c.eye[0], 1.5);
  EXPECT_DOUBLE_EQ(c.eye[1], -2.25);
  EXPECT_DOUBLE_EQ(c.eye[2], 3.125);
  EXPECT_DOUBLE_EQ(c.focal[0], -0.5);
  EXPECT_DOUBLE_EQ(c.focal[1], 0.75);
  EXPECT_DOUBLE_EQ(c.focal[2], 0.25);
  EXPECT_DOUBLE_EQ(c.up[0], 0.1);
  EXPECT_DOUBLE_EQ(c.up[1], 0.2);
  EXPECT_DOUBLE_EQ(c.up[2], 0.9);
  EXPECT_DOUBLE_EQ(c.vfov_degrees, 42.5);
  EXPECT_EQ(c.width, 96);
  EXPECT_EQ(c.height, 64);
}

TEST(CameraFromPose, LeavesProjectionAndParallelScaleAtDefaults) {
  const double eye[3] = {0.0, -4.0, 0.0};
  const double focal[3] = {0.0, 0.0, 0.0};
  const double up[3] = {0.0, 0.0, 1.0};
  const camera c = camera::from_pose(eye, focal, up, 30.0, 32, 32);
  EXPECT_EQ(c.projection, camera::projection_type::perspective);
  EXPECT_DOUBLE_EQ(c.parallel_scale, 1.0);
}

TEST(CameraFromPose, ResultGeneratesSameRaysAsEquivalentStruct) {
  const double eye[3] = {1.3, -2.7, 2.1};
  const double focal[3] = {-0.4, 0.6, -0.9};
  const double up[3] = {0.2, 0.3, 1.0};
  const camera a = camera::from_pose(eye, focal, up, 35.0, 16, 16);

  camera b = oblique_camera(); // same pose, set by hand
  b.vfov_degrees = 35.0;
  b.width = 16;
  b.height = 16;

  const ray ra = a.generate_ray(3, 11);
  const ray rb = b.generate_ray(3, 11);
  expect_vec_near(ra.origin, rb.origin, 0.0);
  expect_vec_near(ra.direction, rb.direction, 0.0);
}

// ============================================================================
// Error handling
// ============================================================================

TEST(CameraErrors, ZeroOrNegativeRasterThrows) {
  camera c = minus_y_camera();
  c.width = 0;
  EXPECT_THROW(c.basis(), cvc::volren_error);
  c.width = 32;
  c.height = 0;
  EXPECT_THROW(c.basis(), cvc::volren_error);
  c.height = -3;
  EXPECT_THROW(c.basis(), cvc::volren_error);
}

TEST(CameraErrors, PerspectiveFovOutOfRangeThrows) {
  camera c = minus_y_camera();
  c.vfov_degrees = 0.0;
  EXPECT_THROW(c.basis(), cvc::volren_error);
  c.vfov_degrees = -10.0;
  EXPECT_THROW(c.basis(), cvc::volren_error);
  c.vfov_degrees = 180.0;
  EXPECT_THROW(c.basis(), cvc::volren_error);
  c.vfov_degrees = 250.0;
  EXPECT_THROW(c.basis(), cvc::volren_error);
  c.vfov_degrees = 179.9; // still valid
  EXPECT_NO_THROW(c.basis());
}

TEST(CameraErrors, OrthographicNonPositiveParallelScaleThrows) {
  camera c = minus_y_camera();
  c.projection = camera::projection_type::orthographic;
  c.parallel_scale = 0.0;
  EXPECT_THROW(c.basis(), cvc::volren_error);
  c.parallel_scale = -1.0;
  EXPECT_THROW(c.basis(), cvc::volren_error);
}

TEST(CameraErrors, OrthographicIgnoresFovPerspectiveIgnoresParallelScale) {
  // Documents current behavior: each projection validates only its own knob.
  camera ortho = minus_y_camera();
  ortho.projection = camera::projection_type::orthographic;
  ortho.vfov_degrees = -5.0; // invalid for perspective, unused for ortho
  EXPECT_NO_THROW(ortho.basis());

  camera persp = minus_y_camera();
  persp.parallel_scale = -5.0; // invalid for ortho, unused for perspective
  EXPECT_NO_THROW(persp.basis());
}

TEST(CameraErrors, EyeCoincidingWithFocalThrows) {
  camera c = minus_y_camera();
  c.eye = {0.25, 0.5, -1.0};
  c.focal = {0.25, 0.5, -1.0};
  EXPECT_THROW(c.basis(), cvc::volren_error);
}

TEST(CameraErrors, ZeroUpVectorThrows) {
  camera c = minus_y_camera();
  c.up = {0.0, 0.0, 0.0};
  EXPECT_THROW(c.basis(), cvc::volren_error);
}

TEST(CameraErrors, UpParallelToViewDirectionThrows) {
  camera c = minus_y_camera(); // view direction is +Y
  c.up = {0.0, 1.0, 0.0};
  EXPECT_THROW(c.basis(), cvc::volren_error);
  c.up = {0.0, -3.0, 0.0}; // anti-parallel, any length
  EXPECT_THROW(c.basis(), cvc::volren_error);
}

TEST(CameraErrors, GenerateRayPropagatesDegeneratePoseError) {
  camera c = minus_y_camera();
  c.width = 0;
  EXPECT_THROW(c.generate_ray(0, 0), cvc::volren_error);
}

// ============================================================================
// depth_to_window_z
// ============================================================================

TEST(DepthToWindowZ, HitsZeroAtNearAndOneAtFarForBothProjections) {
  using cvc::volren::depth_to_window_z;
  for (auto proj : {camera::projection_type::perspective, camera::projection_type::orthographic}) {
    EXPECT_NEAR(depth_to_window_z(0.1, 0.1, 100.0, proj), 0.0, 1e-12);
    EXPECT_NEAR(depth_to_window_z(100.0, 0.1, 100.0, proj), 1.0, 1e-12);
  }
}

TEST(DepthToWindowZ, OrthographicIsLinearInDepth) {
  using cvc::volren::depth_to_window_z;
  const double z = depth_to_window_z(5.5, 1.0, 10.0, camera::projection_type::orthographic);
  EXPECT_NEAR(z, 0.5, 1e-12);
}

TEST(DepthToWindowZ, PerspectiveMatchesGlDepthFormula) {
  using cvc::volren::depth_to_window_z;
  const double n = 1.0, f = 10.0, d = 4.0;
  // z_ndc = (f+n)/(f-n) - 2fn/(d(f-n)); window z = z_ndc/2 + 1/2.
  const double z_ndc = (f + n) / (f - n) - 2.0 * f * n / (d * (f - n));
  const double expected = 0.5 * z_ndc + 0.5;
  EXPECT_NEAR(depth_to_window_z(d, n, f, camera::projection_type::perspective), expected, 1e-12);
}

TEST(DepthToWindowZ, DepthsOutsideClipRangeClamp) {
  using cvc::volren::depth_to_window_z;
  const auto persp = camera::projection_type::perspective;
  EXPECT_NEAR(depth_to_window_z(0.01, 1.0, 10.0, persp), 0.0, 1e-12);
  EXPECT_NEAR(depth_to_window_z(1e6, 1.0, 10.0, persp), 1.0, 1e-12);
}

TEST(DepthToWindowZ, InvalidClipRangeThrows) {
  using cvc::volren::depth_to_window_z;
  const auto persp = camera::projection_type::perspective;
  EXPECT_THROW(depth_to_window_z(1.0, 0.0, 10.0, persp), cvc::volren_error);  // near == 0
  EXPECT_THROW(depth_to_window_z(1.0, -1.0, 10.0, persp), cvc::volren_error); // near < 0
  EXPECT_THROW(depth_to_window_z(1.0, 5.0, 5.0, persp), cvc::volren_error);   // far == near
  EXPECT_THROW(depth_to_window_z(1.0, 5.0, 2.0, persp), cvc::volren_error);   // far < near
}
