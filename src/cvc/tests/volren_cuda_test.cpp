// volren_cuda_test — CPU/GPU parity for the cvc::volren CUDA backend
// (raycaster_cuda.h / raycast.cu).
//
// raycast.cu is a SEMANTIC mirror of raycaster.cpp's render_ray, not a
// bit-exact one: the translation unit is compiled with --use_fast_math in
// Release (src/cvc/CMakeLists.txt deliberately puts volume rendering in the
// "no bit/float-equivalence contract" class), so parity is asserted at the
// image level -- a per-channel LSB budget over the raster, plus a fractional
// budget and a one-cell absolute bound on the depth map (see below) -- never
// as float equality.
//
// Every test renders the SAME raycaster configuration twice, switching only
// set_backend(), and checks backend_used() afterwards: a silent cvc::cuda_error
// fallback to the CPU would otherwise make every metric trivially zero.
//
// Two-level skip: this whole file collapses to one SKIP without
// CVC_ENABLE_CUDA, and each test skips at runtime on a GPU-less machine.
//
// ---------------------------------------------------------------------------
// Why the depth map gets a FRACTIONAL budget and not a hard relative tolerance
// ---------------------------------------------------------------------------
// The depth map latches at the first sample that pushes accumulated alpha past
// depth_alpha_threshold, and contributions are quantized to whole CELLS (the
// volren per-cell sampling model).  That makes the latched t a step function of
// accumulated alpha: an epsilon change in alpha near the threshold moves the
// latch by a full cell -- there is no intermediate value it can take.  So the
// renderer is ill-conditioned in depth by construction, and no
// non-bit-identical implementation can satisfy a max-relative bound.
//
// Measured on a GTX 1650, the shaded-TF scene below, CPU against ITSELF with
// only depth_alpha_threshold nudged (an alpha perturbation far smaller than
// the ~2 LSB = 8e-3 that fast math costs):
//
//   threshold 0.5 vs 0.5+1e-6 :  0.00% of pixels differ, worst 0.0e+00
//   threshold 0.5 vs 0.5+1e-5 :  0.19%                 , worst 6.1e-03
//   threshold 0.5 vs 0.5+1e-4 :  1.12%                 , worst 6.2e-03, 16 mask flips
//   threshold 0.5 vs 0.5+1e-3 : 11.09%                 , worst 6.4e-03, 48 mask flips
//
// CPU-vs-CUDA on the same scene lands at 2.52% and worst 6.4e-03 with 16 mask
// flips -- the same signature, and the worst value is pinned at the same
// quantum in every case because it IS one cell.  What a parity test can
// therefore assert is (1) that the disagreement stays rare and (2) that every
// disagreement is bounded by one cell: a real defect (a t-versus-eye-depth
// confusion, a scale error, the wrong surface) is off by far more than a cell
// and trips the absolute bound immediately.
//
// The colour outliers have the same character: the residual differences sit in
// the specular band, not at silhouettes, and come from --use_fast_math routing
// powf() to __powf() for the Blinn-Phong exponent.  Nudging the isovalue by
// 1e-9/1e-7/1e-5 on the CPU moves nothing, so they are a shading-precision
// effect, not a geometry disagreement.  They stay well inside the stated
// 3-LSB / 99% budget (worst measured: 0.24% of the raster).

#include <gtest/gtest.h>

#ifndef CVC_ENABLE_CUDA
TEST(VolrenCuda, SkippedNoCuda) { GTEST_SKIP() << "built without CVC_ENABLE_CUDA"; }
#else

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/volren/raycaster.h>
#include <cvc/volren/raycaster_cuda.h>
#include <cvc/volume/volume.h>

using cvc::volren::backend;
using cvc::volren::camera;
using cvc::volren::cut_plane;
using cvc::volren::frame;
using cvc::volren::isosurface;
using cvc::volren::light;
using cvc::volren::mat4;
using cvc::volren::raycaster;
using cvc::volren::transfer_function;
using cvc::volren::transfer_point;
using cvc::volren::volume_settings;

namespace {

// Raster and grid are sized for the parity budget, not for looks: 192^2 gives
// ~37k samples so a "99% of pixels" budget is 368 pixels wide, while both
// backends still render in tens of milliseconds.  Do not raise kGrid casually
// -- finer cells mean smaller per-cell alpha increments, so more rays sit near
// the depth-latch threshold: at 256^2/48^3 the shaded-TF depth violation rate
// measured 4.78%, against 2.52% here, for no extra coverage.
constexpr int kRaster = 192;
constexpr int kGrid = 40;
constexpr int kSteps = 256;

// Every volume here is the unit box [-0.5, 0.5]^3 on a kGrid^3 node-centered
// grid, and every model transform is rigid, so one cell measures this much in
// world units along its diagonal -- the quantum of the per-cell depth latch.
const double kCellDiagonal = std::sqrt(3.0) / double(kGrid - 1);

// Tolerances (see the file header).  Fast math costs ~1 LSB across most of the
// frame; the budget is 3 so a rounding disagreement at a silhouette does not
// turn into a flake.
constexpr int kChannelBudget = 3;
constexpr double kChannelViolationFraction = 0.01; // >= 99% of pixels within budget
// Gross-error tripwire on the single worst channel, NOT a parity budget: the
// fractional rule above is the contract.  Worst measured across every test
// here is 14 (the __powf specular band); this only fires on a broken pixel.
constexpr int kChannelHardCap = 40;
constexpr double kMaskMismatchFraction = 0.005; // alpha and depth coverage masks

// Depth: relative agreement is asserted as a fraction, and every outlier is
// bounded absolutely by one cell of the sampled grid -- see the file header
// for the CPU-versus-itself conditioning measurements behind this split.
constexpr double kDepthRelativeTolerance = 1e-3;
constexpr double kDepthViolationFraction = 0.05; // of the pixels finite on BOTH sides

// ---------------------------------------------------------------------------
// Local scene helpers (deliberate copies of volren_render_test.cpp's -- the
// two suites must be able to drift independently).
// ---------------------------------------------------------------------------

// val = 1 - r about the box center: a smooth radial field whose isosurfaces
// are spheres and whose gradient points INWARD everywhere.
cvc::volume makeSphereVolume(cvc::app &ctx, unsigned n,
                             const cvc::bounding_box &box = cvc::bounding_box(-0.5, -0.5, -0.5, 0.5,
                                                                              0.5, 0.5)) {
  cvc::volume vol(ctx, cvc::dimension(n, n, n), cvc::Float, box);
  const double cx = 0.5 * (box.minx + box.maxx);
  const double cy = 0.5 * (box.miny + box.maxy);
  const double cz = 0.5 * (box.minz + box.maxz);
  for (unsigned k = 0; k < n; ++k)
    for (unsigned j = 0; j < n; ++j)
      for (unsigned i = 0; i < n; ++i) {
        const double x = box.minx + double(i) * vol.XSpan() - cx;
        const double y = box.miny + double(j) * vol.YSpan() - cy;
        const double z = box.minz + double(k) * vol.ZSpan() - cz;
        vol(i, j, k, 1.0 - std::sqrt(x * x + y * y + z * z));
      }
  return vol;
}

// val = local z, strictly increasing in k -- an axis-aligned ramp, so a model
// rotation actually changes the image (a radial field would not).
cvc::volume makeLinearVolume(cvc::app &ctx, unsigned n) {
  cvc::volume vol(ctx, cvc::dimension(n, n, n), cvc::Float,
                  cvc::bounding_box(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5));
  for (unsigned k = 0; k < n; ++k)
    for (unsigned j = 0; j < n; ++j)
      for (unsigned i = 0; i < n; ++i)
        vol(i, j, k, -0.5 + double(k) * vol.ZSpan());
  return vol;
}

camera perspectiveCam() {
  camera c;
  c.eye = {0.0, 0.0, 4.0};
  c.focal = {0.0, 0.0, 0.0};
  c.up = {0.0, 1.0, 0.0};
  c.projection = camera::projection_type::perspective;
  c.vfov_degrees = 30.0;
  c.width = kRaster;
  c.height = kRaster;
  return c;
}

camera orthoCam() {
  camera c = perspectiveCam();
  c.projection = camera::projection_type::orthographic;
  c.parallel_scale = 1.0;
  return c;
}

// Rotation about +x by `degrees`, then translation -- row-major, column-vector
// points (the mat4 convention), so m[3]/m[7]/m[11] carry the translation.
mat4 rotateXThenTranslate(double degrees, double tx, double ty, double tz) {
  const double a = degrees * M_PI / 180.0;
  const double c = std::cos(a), s = std::sin(a);
  mat4 m;
  m.m[0] = 1.0;
  m.m[1] = 0.0;
  m.m[2] = 0.0;
  m.m[3] = tx;
  m.m[4] = 0.0;
  m.m[5] = c;
  m.m[6] = -s;
  m.m[7] = ty;
  m.m[8] = 0.0;
  m.m[9] = s;
  m.m[10] = c;
  m.m[11] = tz;
  return m;
}

mat4 translate(double tx, double ty, double tz) {
  mat4 m;
  m.m[3] = tx;
  m.m[7] = ty;
  m.m[11] = tz;
  return m;
}

transfer_function rampTF(double lo, double hi, float r, float g, float b, float a_hi) {
  transfer_function tf;
  tf.add(transfer_point{lo, r, g, b, 0.f});
  tf.add(transfer_point{hi, r, g, b, a_hi});
  return tf;
}

// ---------------------------------------------------------------------------
// Parity metrics
// ---------------------------------------------------------------------------

struct parity_metrics {
  int pixels = 0;
  int worst_channel_diff = 0; // max |cpu - gpu| over every RGBA byte
  int channel_violations = 0; // pixels where some channel exceeds kChannelBudget
  int alpha_mask_mismatch = 0;
  int depth_mask_mismatch = 0; // finite-vs-inf disagreement
  int both_finite = 0;
  double worst_depth_rel = 0.0; // over pixels finite on BOTH sides
  double worst_depth_abs = 0.0; // ... in world units, for the one-cell bound
  int depth_violations = 0;     // both-finite pixels over kDepthRelativeTolerance
  int worst_x = -1, worst_y = -1;
};

const unsigned char *pixel(const frame &f, int x, int y) {
  const cvc::image &img = f.color; // const ref: no copy-on-write detach
  return img.data() + (std::size_t(y) * img.width() + x) * 4;
}

float depthAt(const frame &f, int x, int y) {
  const cvc::image &img = f.depth;
  return reinterpret_cast<const float *>(img.data())[std::size_t(y) * img.width() + x];
}

parity_metrics compare(const frame &cpu, const frame &gpu) {
  parity_metrics m;
  const int w = cpu.color.width(), h = cpu.color.height();
  m.pixels = w * h;
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const unsigned char *a = pixel(cpu, x, y);
      const unsigned char *b = pixel(gpu, x, y);
      int worst = 0;
      for (int c = 0; c < 4; ++c)
        worst = std::max(worst, std::abs(int(a[c]) - int(b[c])));
      if (worst > m.worst_channel_diff) {
        m.worst_channel_diff = worst;
        m.worst_x = x;
        m.worst_y = y;
      }
      if (worst > kChannelBudget)
        ++m.channel_violations;
      if ((a[3] > 0) != (b[3] > 0))
        ++m.alpha_mask_mismatch;

      const double dc = double(depthAt(cpu, x, y));
      const double dg = double(depthAt(gpu, x, y));
      const bool fc = std::isfinite(dc), fg = std::isfinite(dg);
      if (fc != fg) {
        ++m.depth_mask_mismatch;
      } else if (fc) {
        ++m.both_finite;
        const double abs_diff = std::abs(dc - dg);
        const double rel = abs_diff / std::max(std::abs(dc), 1e-9);
        m.worst_depth_abs = std::max(m.worst_depth_abs, abs_diff);
        m.worst_depth_rel = std::max(m.worst_depth_rel, rel);
        if (rel > kDepthRelativeTolerance)
          ++m.depth_violations;
      }
    }
  return m;
}

// Repo convention: print the worst metric, then assert on it.  `cell_diagonal`
// is the world-space size of one grid cell -- the quantum the depth latch moves
// in, and therefore the hard bound every depth outlier must respect.
void expectParity(const char *label, const parity_metrics &m, double cell_diagonal) {
  const double pct = 100.0 / double(m.pixels);
  const double dpct = 100.0 / double(std::max(m.both_finite, 1));
  std::printf("[volren-cuda %s] worst_channel_diff=%d @(%d,%d) violations(>%d)=%d/%d (%.3f%%) "
              "alpha_mask_mismatch=%d (%.3f%%) | depth both_finite=%d worst_rel=%.3e "
              "worst_abs=%.3e (%.2f cells) violations(>%.0e)=%d (%.3f%%) mask_mismatch=%d "
              "(%.3f%%)\n",
              label, m.worst_channel_diff, m.worst_x, m.worst_y, kChannelBudget,
              m.channel_violations, m.pixels, m.channel_violations * pct, m.alpha_mask_mismatch,
              m.alpha_mask_mismatch * pct, m.both_finite, m.worst_depth_rel, m.worst_depth_abs,
              m.worst_depth_abs / cell_diagonal, kDepthRelativeTolerance, m.depth_violations,
              m.depth_violations * dpct, m.depth_mask_mismatch, m.depth_mask_mismatch * pct);
  std::fflush(stdout);

  EXPECT_LE(m.channel_violations, int(kChannelViolationFraction * double(m.pixels)))
      << label << ": more than 1% of pixels differ by more than " << kChannelBudget << " LSB";
  EXPECT_LE(m.worst_channel_diff, kChannelHardCap)
      << label << ": a single pixel is grossly wrong, not merely rounded";
  EXPECT_LE(m.alpha_mask_mismatch, int(kMaskMismatchFraction * double(m.pixels)))
      << label << ": alpha coverage disagrees";

  EXPECT_LE(m.depth_violations, int(kDepthViolationFraction * double(m.both_finite)))
      << label << ": too many depth latches slipped";
  // The bound that actually catches a wrong depth: a latch that slipped is off
  // by one cell, anything else is a defect.
  EXPECT_LE(m.worst_depth_abs, cell_diagonal)
      << label << ": a depth disagreement exceeds one cell (" << cell_diagonal
      << " world units) -- that is not a latch slip";
  EXPECT_LE(m.depth_mask_mismatch, int(kMaskMismatchFraction * double(m.pixels)))
      << label << ": depth coverage disagrees";
}

struct frame_pair {
  frame cpu, gpu;
};

// Render the same configuration on both backends.  backend::cuda throws for a
// scene outside the v1 device scope and falls back only on cvc::cuda_error, so
// the backend_used() check is what proves the GPU actually ran.
frame_pair renderBoth(raycaster &rc) {
  frame_pair out;
  rc.set_backend(backend::cpu);
  out.cpu = rc.render();
  EXPECT_EQ(int(rc.backend_used()), int(backend::cpu));
  rc.set_backend(backend::cuda);
  out.gpu = rc.render();
  EXPECT_EQ(int(rc.backend_used()), int(backend::cuda))
      << "the CUDA render silently fell back to the CPU -- parity would be vacuous";
  return out;
}

// A frame is only worth comparing if something was actually drawn.
int countAlphaPositive(const frame &f) {
  const cvc::image &img = f.color;
  const unsigned char *d = img.data();
  const int n = img.width() * img.height();
  int count = 0;
  for (int i = 0; i < n; ++i)
    if (d[i * 4 + 3] > 0)
      ++count;
  return count;
}

class VolrenCudaTest : public ::testing::Test {
protected:
  cvc::app ctx;

  void SetUp() override {
    if (!cvc::volren::raycast_cuda_available())
      GTEST_SKIP() << "no CUDA device";
  }
};

// ---------------------------------------------------------------------------
// (a) Isosurface parity -- MC cell intersection, spline-gradient normals,
//     Blinn-Phong, and the iso depth latch.
// ---------------------------------------------------------------------------

TEST_F(VolrenCudaTest, IsosurfaceSphereParity) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = kSteps;
  rc.settings().background = {0.05f, 0.06f, 0.1f};
  rc.settings().two_sided_lighting = true;
  light l;
  l.direction = {0.3, 0.4, 1.0};
  l.color = {1.f, 0.95f, 0.85f};
  rc.settings().lights.push_back(l);

  // Iso-only: value 0.6 on the 1-r field is the r = 0.4 sphere.
  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  isosurface iso;
  iso.value = 0.6;
  iso.opacity = 1.0f;
  iso.color = {0.9f, 0.5f, 0.2f};
  iso.shininess = 24.0f;
  vs.isosurfaces.push_back(iso);
  rc.add_volume(makeSphereVolume(ctx, kGrid), vs);

  const frame_pair f = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(f.cpu), 1000) << "the CPU reference drew nothing";
  expectParity("iso-sphere", compare(f.cpu, f.gpu), kCellDiagonal);
}

// ---------------------------------------------------------------------------
// (b) Shaded transfer function + gradient-magnitude opacity ramp, two
//     accumulating lights, non-zero ambient, perspective projection.
// ---------------------------------------------------------------------------

TEST_F(VolrenCudaTest, ShadedTransferFunctionGradientRampParity) {
  raycaster rc(ctx);
  rc.view() = perspectiveCam();
  rc.settings().steps = kSteps;
  rc.settings().background = {0.1f, 0.1f, 0.12f};
  rc.settings().ambient = 0.15f;
  rc.settings().two_sided_lighting = true;
  light key;
  key.direction = {0.0, 0.0, 1.0};
  key.color = {1.f, 0.9f, 0.8f};
  light fill;
  fill.direction = {-0.6, 0.5, 0.3};
  fill.color = {0.2f, 0.3f, 0.5f};
  rc.settings().lights.push_back(key);
  rc.settings().lights.push_back(fill);

  volume_settings vs; // shaded = true by default
  vs.tf = rampTF(0.0, 1.0, 0.85f, 0.55f, 0.25f, 0.7f);
  vs.tf_auto_domain = false;
  // |gradient| of the 1-r field is ~1 per world unit; a [0, 0.5, 4] ramp puts
  // the interesting range on the rising leg and the plateau.
  vs.gradient_ramp.enabled = true;
  vs.gradient_ramp.ramp0 = 0.0;
  vs.gradient_ramp.ramp1 = 0.5;
  vs.gradient_ramp.ramp2 = 4.0;
  rc.add_volume(makeSphereVolume(ctx, kGrid), vs);

  const frame_pair f = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(f.cpu), 1000) << "the CPU reference drew nothing";
  expectParity("shaded-tf-ramp", compare(f.cpu, f.gpu), kCellDiagonal);
}

// ---------------------------------------------------------------------------
// (c) Unshaded transfer function + density window + a cut plane.
// ---------------------------------------------------------------------------

TEST_F(VolrenCudaTest, UnshadedWindowCutPlaneParity) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = kSteps;
  rc.settings().background = {0.f, 0.f, 0.f};
  // Keep x >= 0: samples with dot(p - point, normal) < 0 are culled.
  cut_plane cp;
  cp.point = {0.0, 0.0, 0.0};
  cp.normal = {1.0, 0.2, 0.0};
  rc.settings().cut_planes.push_back(cp);

  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = true;
  vs.tf = rampTF(0.0, 1.0, 0.2f, 0.7f, 0.9f, 0.85f);
  vs.tf_auto_domain = false;
  // The 1-r field spans ~[0.13, 1]; [0.45, 0.95] carves out a shell.
  vs.window_enabled = true;
  vs.window_min = 0.45;
  vs.window_max = 0.95;
  rc.add_volume(makeSphereVolume(ctx, kGrid), vs);

  const frame_pair f = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(f.cpu), 500) << "the window/cut plane culled everything";
  expectParity("unshaded-window-cut", compare(f.cpu, f.gpu), kCellDiagonal);
}

// ---------------------------------------------------------------------------
// (d) Transformed volume -- object-space sampling through the affine inverse
//     and inverse-transpose normals.  The ramp field (not the radial one) is
//     used so a rotation genuinely changes the image.
// ---------------------------------------------------------------------------

TEST_F(VolrenCudaTest, TransformedVolumeParity) {
  raycaster rc(ctx);
  rc.view() = perspectiveCam();
  rc.settings().steps = kSteps;
  rc.settings().background = {0.02f, 0.02f, 0.04f};
  rc.settings().two_sided_lighting = true;
  light l;
  l.direction = {0.2, 0.6, 0.8};
  rc.settings().lights.push_back(l);

  volume_settings vs; // shaded TF + one isosurface through the same transform
  vs.tf = rampTF(-0.5, 0.5, 0.3f, 0.8f, 0.6f, 0.5f);
  vs.tf_auto_domain = false;
  isosurface iso;
  iso.value = 0.1; // a local z = 0.1 plane, tilted by the model rotation
  iso.opacity = 0.8f;
  iso.color = {1.f, 0.4f, 0.4f};
  vs.isosurfaces.push_back(iso);
  vs.model_transform = rotateXThenTranslate(35.0, 0.2, -0.15, 0.1);
  rc.add_volume(makeLinearVolume(ctx, kGrid), vs);

  const frame_pair f = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(f.cpu), 1000) << "the CPU reference drew nothing";
  expectParity("transformed", compare(f.cpu, f.gpu), kCellDiagonal);
}

// ---------------------------------------------------------------------------
// (e) Depth-map parity.  The generic comparison already covers depth; this
//     pins the two specific contracts -- where the map latches, and that both
//     backends agree on the same value at a known geometric distance.
// ---------------------------------------------------------------------------

TEST_F(VolrenCudaTest, DepthMapParity) {
  raycaster rc(ctx);
  rc.view() = orthoCam(); // straight down -z from eye z = 4
  rc.settings().steps = kSteps;
  rc.settings().depth_alpha_threshold = 0.5f;

  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = true;
  vs.tf = rampTF(-0.5, 0.5, 1.f, 1.f, 1.f, 1.f);
  vs.tf_auto_domain = false;
  rc.add_volume(makeLinearVolume(ctx, kGrid), vs);

  const frame_pair f = renderBoth(rc);
  const parity_metrics m = compare(f.cpu, f.gpu);
  ASSERT_GT(m.both_finite, 1000) << "nothing latched a depth on either backend";
  expectParity("depth-map", m, kCellDiagonal);

  // Center ray: the front face at z = 0.5 is 3.5 from the eye, and both
  // backends must latch within the same step of it.
  const int c = kRaster / 2;
  const double dcpu = double(depthAt(f.cpu, c, c));
  const double dgpu = double(depthAt(f.gpu, c, c));
  std::printf("[volren-cuda depth-center] cpu=%.9g gpu=%.9g\n", dcpu, dgpu);
  std::fflush(stdout);
  ASSERT_TRUE(std::isfinite(dcpu));
  ASSERT_TRUE(std::isfinite(dgpu));
  EXPECT_NEAR(dcpu, 3.5, 3.0 * std::sqrt(3.0) / double(kSteps));
  EXPECT_NEAR(dgpu, dcpu, 1e-4);

  // Corner rays miss the scene box on both backends.
  EXPECT_TRUE(std::isinf(depthAt(f.cpu, 0, 0)));
  EXPECT_TRUE(std::isinf(depthAt(f.gpu, 0, 0)));
}

// ---------------------------------------------------------------------------
// (f) Both projections through the identical scene: perspective ray
//     divergence vs. the shared-direction orthographic bundle.
// ---------------------------------------------------------------------------

TEST_F(VolrenCudaTest, PerspectiveAndOrthographicCameraParity) {
  raycaster rc(ctx);
  rc.settings().steps = kSteps;
  rc.settings().background = {0.08f, 0.04f, 0.12f};
  rc.settings().ambient = 0.1f;
  light l;
  l.direction = {0.0, 0.5, 1.0};
  rc.settings().lights.push_back(l);

  volume_settings vs;
  vs.tf = rampTF(0.0, 1.0, 0.6f, 0.6f, 0.9f, 0.6f);
  vs.tf_auto_domain = false;
  isosurface iso;
  iso.value = 0.65;
  iso.opacity = 0.9f;
  vs.isosurfaces.push_back(iso);
  rc.add_volume(makeSphereVolume(ctx, kGrid), vs);

  rc.view() = perspectiveCam();
  const frame_pair p = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(p.cpu), 1000);
  expectParity("camera-perspective", compare(p.cpu, p.gpu), kCellDiagonal);

  rc.view() = orthoCam();
  const frame_pair o = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(o.cpu), 1000);
  expectParity("camera-orthographic", compare(o.cpu, o.gpu), kCellDiagonal);

  // Sanity: the two projections are genuinely different images, so the pair
  // above is not comparing the same rays twice.
  EXPECT_NE(countAlphaPositive(p.cpu), countAlphaPositive(o.cpu));
}

// ---------------------------------------------------------------------------
// Dispatch contract
// ---------------------------------------------------------------------------

TEST_F(VolrenCudaTest, BackendDefaultsToCpu) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 64;
  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = true;
  vs.tf = rampTF(0.0, 1.0, 1.f, 1.f, 1.f, 1.f);
  vs.tf_auto_domain = false;
  rc.add_volume(makeSphereVolume(ctx, 16), vs);

  EXPECT_EQ(int(rc.backend_selected()), int(backend::cpu));
  EXPECT_EQ(int(rc.backend_used()), int(backend::cpu)); // before any render
  rc.render();
  EXPECT_EQ(int(rc.backend_used()), int(backend::cpu));

  // A single-volume scene under backend::automatic does reach the GPU.
  rc.set_backend(backend::automatic);
  rc.render();
  EXPECT_EQ(int(rc.backend_used()), int(backend::cuda));
}

TEST_F(VolrenCudaTest, MoreVolumesThanTheCapFallBackUnderAutomatic) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 64;
  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = true;
  vs.tf = rampTF(0.0, 1.0, 1.f, 1.f, 1.f, 1.f);
  vs.tf_auto_domain = false;
  const cvc::volume small = makeSphereVolume(ctx, 16);
  for (int i = 0; i <= cvc::volren::cuda_limits::max_volumes; ++i)
    rc.add_volume(small, vs);

  // One past the cap: an explicit request is an error (never a silent CPU
  // march), automatic degrades quietly.
  rc.set_backend(backend::cuda);
  EXPECT_THROW(rc.render(), cvc::volren_error);

  rc.set_backend(backend::automatic);
  const frame f = rc.render();
  EXPECT_EQ(int(rc.backend_used()), int(backend::cpu));
  EXPECT_EQ(f.color.width(), kRaster);
}

// ---------------------------------------------------------------------------
// (g) Multi-volume parity.  Both scenes stack volumes ALONG the view direction
//     so every ray crosses more than one: that is what exercises the merged
//     per-ray hit stream, the per-volume cull windows, the shared
//     (volume, cell)-keyed spline cache, and the per-volume LUT/settings
//     indirection all at once.  A side-by-side layout would only prove the
//     kernel can index an array.
// ---------------------------------------------------------------------------

// Two translucent spheres one behind the other from the camera, with
// DIFFERENT per-volume settings (only the far one carries an isosurface, and
// the two gradient ramps differ) so a volume reading its neighbour's block
// shows up immediately.  Both use the gradient ramp for the same reason
// ShadedTransferFunctionGradientRampParity does: an unramped high-alpha ramp
// saturates a ray within a few cells, which puts most of the frame on the
// depth-latch/specular knife edge and makes the metric measure fast-math
// conditioning rather than multi-volume correctness (measured: the SAME scene
// with one volume and no ramp already violates the budget on 8.7% of pixels).
TEST_F(VolrenCudaTest, TwoVolumeParity) {
  raycaster rc(ctx);
  rc.view() = perspectiveCam(); // eye at +z, so ±z translations stack on a ray
  rc.settings().steps = kSteps;
  rc.settings().background = {0.05f, 0.05f, 0.08f};
  rc.settings().ambient = 0.12f;
  rc.settings().two_sided_lighting = true;
  light l;
  l.direction = {0.2, 0.5, 1.0};
  rc.settings().lights.push_back(l);

  volume_settings near;
  near.tf = rampTF(0.0, 1.0, 0.9f, 0.4f, 0.3f, 0.7f);
  near.tf_auto_domain = false;
  near.gradient_ramp.enabled = true;
  near.gradient_ramp.ramp0 = 0.0;
  near.gradient_ramp.ramp1 = 0.5;
  near.gradient_ramp.ramp2 = 4.0;
  // A translucent surface on the near volume: an isosurface hit latches the
  // depth map unconditionally, so most rays get a GEOMETRIC depth instead of
  // an alpha-threshold one.  Without it a stack of two translucent volumes
  // puts far more rays than a single one on the latch knife edge the file
  // header describes (measured: 6.5% depth violations against a 5% budget) --
  // conditioning, not disagreement, but not worth asserting through.
  isosurface near_iso;
  near_iso.value = 0.58;
  near_iso.opacity = 0.35f;
  near_iso.color = {0.4f, 1.f, 0.7f};
  near_iso.shininess = 12.f;
  near.isosurfaces.push_back(near_iso);
  near.model_transform = translate(0.0, 0.0, 0.8);
  rc.add_volume(makeSphereVolume(ctx, kGrid), near);

  volume_settings far;
  far.tf = rampTF(0.0, 1.0, 0.3f, 0.6f, 0.95f, 0.6f);
  far.tf_auto_domain = false;
  far.gradient_ramp.enabled = true;
  far.gradient_ramp.ramp0 = 0.1;
  far.gradient_ramp.ramp1 = 0.8;
  far.gradient_ramp.ramp2 = 3.0;
  isosurface iso;
  iso.value = 0.68;
  iso.opacity = 0.5f;
  iso.color = {1.f, 0.9f, 0.4f};
  far.isosurfaces.push_back(iso);
  far.model_transform = translate(0.0, 0.0, -0.8);
  rc.add_volume(makeSphereVolume(ctx, kGrid), far);

  const frame_pair f = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(f.cpu), 1000) << "the CPU reference drew nothing";
  expectParity("two-volume", compare(f.cpu, f.gpu), kCellDiagonal);

  // The far volume really is behind the near one on the central rays: drop it
  // and the image must change, or "two volumes" was only ever one.
  raycaster solo(ctx);
  solo.view() = rc.view();
  solo.settings() = rc.settings();
  solo.add_volume(makeSphereVolume(ctx, kGrid), near);
  const frame one = solo.render();
  int differing = 0;
  for (int y = 0; y < kRaster; ++y)
    for (int x = 0; x < kRaster; ++x)
      if (std::memcmp(pixel(one, x, y), pixel(f.cpu, x, y), 4) != 0)
        ++differing;
  EXPECT_GT(differing, 500) << "the second volume contributed nothing";
}

// Four volumes in two stacked columns: an isosurface pair and a shaded-TF
// pair, so hits from different volumes interleave in the same ray's stream and
// the two volumes' settings must not leak into each other.
TEST_F(VolrenCudaTest, FourVolumeParity) {
  raycaster rc(ctx);
  rc.view() = perspectiveCam();
  rc.settings().steps = kSteps;
  rc.settings().background = {0.03f, 0.04f, 0.06f};
  rc.settings().ambient = 0.1f;
  rc.settings().two_sided_lighting = true;
  light l;
  l.direction = {0.3, 0.4, 1.0};
  rc.settings().lights.push_back(l);

  for (int column = 0; column < 2; ++column) {
    const double tx = column == 0 ? -0.55 : 0.55;
    for (int depth = 0; depth < 2; ++depth) {
      const double tz = depth == 0 ? 0.7 : -0.7;
      volume_settings vs;
      if (column == 0) {
        // Translucent isosurface shells: the merged hit buffer sees hits from
        // both of this column's volumes on the same ray.
        vs.shaded = false;
        vs.unshaded = false;
        isosurface iso;
        iso.value = depth == 0 ? 0.62 : 0.7;
        iso.opacity = 0.55f;
        iso.color = {0.95f, 0.6f, 0.25f};
        iso.shininess = 18.f;
        vs.isosurfaces.push_back(iso);
      } else {
        vs.tf = rampTF(0.0, 1.0, 0.35f, 0.75f, 0.9f, 0.4f);
        vs.tf_auto_domain = false;
        vs.gradient_ramp.enabled = depth == 0; // different per volume on purpose
        vs.gradient_ramp.ramp0 = 0.0;
        vs.gradient_ramp.ramp1 = 0.5;
        vs.gradient_ramp.ramp2 = 4.0;
      }
      vs.model_transform = translate(tx, 0.0, tz);
      rc.add_volume(makeSphereVolume(ctx, kGrid), vs);
    }
  }
  ASSERT_EQ(rc.volume_count(), std::size_t(4));

  const frame_pair f = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(f.cpu), 1000) << "the CPU reference drew nothing";
  expectParity("four-volume", compare(f.cpu, f.gpu), kCellDiagonal);
}

// The device cull box has to contain grid_cell_index()'s acceptance region
// exactly like the host one does -- see volren_render_test's
// SilhouetteKeepsTheLowSideVoxelOfCellSlack for why the region is asymmetric.
// A GPU-only slip there costs one pixel column, which the fractional budgets
// above would swallow, so the boundary is asserted directly.
TEST_F(VolrenCudaTest, CullBoundaryMatchesTheHostSilhouette) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = kSteps;

  transfer_function flat;
  flat.add(transfer_point{-1000.0, 1.f, 1.f, 1.f, 1.f});
  flat.add(transfer_point{1000.0, 1.f, 1.f, 1.f, 1.f});
  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = true;
  vs.tf = flat;
  rc.add_volume(makeSphereVolume(ctx, kGrid), vs);
  // Off-screen, purely to widen scene_bounds so the low-side column reaches
  // the per-volume window instead of being rejected by the scene slab.
  volume_settings offscreen = vs;
  offscreen.model_transform = translate(-2.0, 0.0, 0.0);
  rc.add_volume(makeSphereVolume(ctx, kGrid), offscreen);

  const frame_pair f = renderBoth(rc);

  const double span = 1.0 / double(kGrid - 1);
  const int row = kRaster / 2;
  const auto u = [](int px) { return (double(px) + 0.5) / double(kRaster) * 2.0 - 1.0; };
  int first_lit_cpu = -1, first_lit_gpu = -1, last_lit_cpu = -1, last_lit_gpu = -1;
  for (int x = 0; x < kRaster; ++x) {
    if (pixel(f.cpu, x, row)[3] > 0) {
      if (first_lit_cpu < 0)
        first_lit_cpu = x;
      last_lit_cpu = x;
    }
    if (pixel(f.gpu, x, row)[3] > 0) {
      if (first_lit_gpu < 0)
        first_lit_gpu = x;
      last_lit_gpu = x;
    }
  }
  std::printf("[volren-cuda cull-boundary] cpu=[%d,%d] gpu=[%d,%d]\n", first_lit_cpu, last_lit_cpu,
              first_lit_gpu, last_lit_gpu);
  std::fflush(stdout);

  EXPECT_EQ(first_lit_gpu, first_lit_cpu) << "the device cull box clipped the low side differently";
  EXPECT_EQ(last_lit_gpu, last_lit_cpu) << "the device cull box clipped the high side differently";
  // And the shared boundary is the one cell_index() dictates: one voxel of
  // slack below min, none above max.
  ASSERT_GE(first_lit_cpu, 0);
  EXPECT_GT(u(first_lit_cpu), -0.5 - span);
  EXPECT_LT(u(first_lit_cpu - 1), -0.5 - span);
  EXPECT_LT(u(last_lit_cpu), 0.5);
  EXPECT_GT(u(last_lit_cpu + 1), 0.5);
}

// ---------------------------------------------------------------------------
// (h) Resident device volume cache.  Voxels stay on the device between renders,
//     so the thing worth proving is that a CHANGED volume is never served from
//     a stale copy -- through both the automatic path (copy-on-write hands the
//     renderer a different buffer) and the announced path (an in-place write
//     through voxels::data_ptr(), which no copy-on-write can see).
// ---------------------------------------------------------------------------

int framesDiffer(const frame &a, const frame &b) {
  const int w = a.color.width(), h = a.color.height();
  int n = 0;
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const unsigned char *p = pixel(a, x, y);
      const unsigned char *q = pixel(b, x, y);
      if (p[0] != q[0] || p[1] != q[1] || p[2] != q[2] || p[3] != q[3])
        ++n;
    }
  return n;
}

TEST_F(VolrenCudaTest, ResidentCacheSeesVoxelChanges) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = kSteps;
  rc.settings().background = {0.f, 0.f, 0.f};

  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = true;
  vs.tf = rampTF(0.0, 1.0, 1.f, 1.f, 1.f, 0.6f);
  vs.tf_auto_domain = false; // never re-derive the domain from min()/max()

  cvc::volume vol = makeSphereVolume(ctx, kGrid);
  rc.add_volume(vol, vs);
  rc.set_backend(backend::cuda);

  const frame first = rc.render();
  ASSERT_EQ(int(rc.backend_used()), int(backend::cuda));
  ASSERT_GT(countAlphaPositive(first), 1000);

  // A re-render with nothing changed takes the cache-hit path and must be
  // byte-identical -- a resident copy that drifted would show up here.
  const frame again = rc.render();
  EXPECT_EQ(framesDiffer(first, again), 0) << "a cache hit changed the image";

  // (1) Mutation through the supported API.  The cache co-owns the voxel block,
  // so preWrite() copy-on-writes into a NEW buffer: a different cache key, no
  // announcement needed.  (clear_volumes() first, so the only co-owner left is
  // the cache itself -- that is the case the pin exists for.)
  const unsigned char *before_ptr = vol.data_ptr();
  rc.clear_volumes();
  for (unsigned k = 0; k < kGrid; ++k)
    for (unsigned j = 0; j < kGrid; ++j)
      for (unsigned i = 0; i < kGrid / 2; ++i)
        vol(i, j, k, -1.0); // hollow out half the sphere
  EXPECT_NE(vol.data_ptr(), before_ptr)
      << "the cache did not co-own the block, so the write was in place";
  rc.add_volume(vol, vs);

  const frame hollowed = rc.render();
  ASSERT_EQ(int(rc.backend_used()), int(backend::cuda));
  EXPECT_GT(framesDiffer(first, hollowed), 500)
      << "the GPU re-rendered a stale resident copy of the volume";
  {
    // ... and it is the RIGHT new image, not merely a different one.
    rc.set_backend(backend::cpu);
    const frame reference = rc.render();
    rc.set_backend(backend::cuda);
    expectParity("cache-cow", compare(reference, hollowed), kCellDiagonal);
  }

  // (2) In-place mutation through the legacy escape hatch: data_ptr() skips
  // preWrite(), so the buffer the renderer holds changes underneath it and only
  // invalidate_device_volume() can tell the cache.
  float *raw = reinterpret_cast<float *>(vol.data_ptr());
  for (unsigned k = 0; k < kGrid; ++k)
    for (unsigned j = 0; j < kGrid / 2; ++j)
      for (unsigned i = 0; i < kGrid; ++i)
        raw[i + std::size_t(j) * kGrid + std::size_t(k) * kGrid * kGrid] = -1.f;
  rc.invalidate_device_volume(0);

  const frame carved = rc.render();
  ASSERT_EQ(int(rc.backend_used()), int(backend::cuda));
  EXPECT_GT(framesDiffer(hollowed, carved), 500)
      << "invalidate_device_volume() did not force a re-upload";
  rc.set_backend(backend::cpu);
  const frame carved_cpu = rc.render();
  rc.set_backend(backend::cuda);
  expectParity("cache-invalidated", compare(carved_cpu, carved), kCellDiagonal);
}

// The point of the cache: a re-render that only moved the camera, and the
// clear_volumes()/add_volume() rebuild cvcGL's VolRenNode does on EVERY frame,
// must both cost zero host-to-device voxel traffic.
TEST_F(VolrenCudaTest, ResidentCacheSkipsTheUploadOnCameraOnlyChanges) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 64;

  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = true;
  vs.tf = rampTF(0.0, 1.0, 1.f, 1.f, 1.f, 1.f);
  vs.tf_auto_domain = false;
  const cvc::volume vol = makeSphereVolume(ctx, kGrid);
  rc.add_volume(vol, vs);
  rc.set_backend(backend::cuda);

  const std::uint64_t before_first = cvc::volren::raycast_cuda_cache_upload_bytes();
  rc.render();
  ASSERT_EQ(int(rc.backend_used()), int(backend::cuda));
  const std::uint64_t after_first = cvc::volren::raycast_cuda_cache_upload_bytes();
  const std::size_t volume_bytes = std::size_t(kGrid) * kGrid * kGrid * sizeof(float);
  EXPECT_GE(after_first - before_first, volume_bytes) << "the first render did not upload";

  // (1) Camera only.
  camera moved = rc.view();
  moved.eye = {0.4, 0.3, 4.0};
  rc.view() = moved;
  rc.render();
  EXPECT_EQ(cvc::volren::raycast_cuda_cache_upload_bytes(), after_first)
      << "a camera-only change re-uploaded the volume";

  // (2) The VolRenNode rebuild: same voxel buffer, fresh registration.
  rc.clear_volumes();
  rc.add_volume(vol, vs);
  rc.render();
  EXPECT_EQ(cvc::volren::raycast_cuda_cache_upload_bytes(), after_first)
      << "re-registering an unchanged volume re-uploaded it";

  // (3) An announced in-place change must upload again.
  rc.invalidate_device_volume(0);
  rc.render();
  EXPECT_GE(cvc::volren::raycast_cuda_cache_upload_bytes() - after_first, volume_bytes)
      << "invalidate_device_volume() did not re-upload";
}

TEST_F(VolrenCudaTest, CacheBudgetEvictsAndStaysCorrect) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 64;

  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = true;
  vs.tf = rampTF(0.0, 1.0, 1.f, 1.f, 1.f, 1.f);
  vs.tf_auto_domain = false;
  rc.add_volume(makeSphereVolume(ctx, kGrid), vs);
  rc.set_backend(backend::cuda);

  const frame warm = rc.render();
  ASSERT_EQ(int(rc.backend_used()), int(backend::cuda));
  EXPECT_GT(cvc::volren::raycast_cuda_cache_bytes(), std::size_t(0));

  // A budget too small for the scene must not break it: the render exceeds the
  // budget rather than freeing a block it is about to read.
  const std::size_t saved = cvc::volren::raycast_cuda_cache_budget();
  cvc::volren::raycast_cuda_set_cache_budget(1);
  const frame squeezed = rc.render();
  EXPECT_EQ(int(rc.backend_used()), int(backend::cuda));
  EXPECT_EQ(framesDiffer(warm, squeezed), 0) << "eviction changed the image";
  cvc::volren::raycast_cuda_set_cache_budget(saved);

  cvc::volren::raycast_cuda_clear_cache();
  EXPECT_EQ(cvc::volren::raycast_cuda_cache_bytes(), std::size_t(0));
  const frame reloaded = rc.render();
  EXPECT_EQ(framesDiffer(warm, reloaded), 0) << "a re-upload changed the image";
  EXPECT_GT(cvc::volren::raycast_cuda_cache_bytes(), std::size_t(0));
}

// ---------------------------------------------------------------------------
// Supersampled anti-aliasing: the resolve is per pixel, in one thread, so the
// two backends have to agree on the sub-sample GRID as well as on the march.
// ---------------------------------------------------------------------------
//
// A high-contrast opaque isosurface is the worst case on purpose: every
// silhouette pixel's alpha is now a coverage COUNT, so one sub-sample landing
// on the other side of the surface on one backend is a visible 1/n^2 step in
// the alpha channel -- an off-by-one in the offsets, or a float-vs-double
// sub-pixel coordinate, would blow the channel budget instead of hiding inside
// the existing shading noise.  The depth resolve is min-over-sub-samples, so a
// grid disagreement also shows up as depth mask flips.
TEST_F(VolrenCudaTest, SupersampledIsosurfaceParity) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = kSteps;
  rc.settings().background = {0.05f, 0.06f, 0.1f};
  rc.settings().two_sided_lighting = true;
  rc.settings().supersample = 2;
  light l;
  l.direction = {0.3, 0.4, 1.0};
  l.color = {1.f, 0.95f, 0.85f};
  rc.settings().lights.push_back(l);

  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  isosurface iso;
  iso.value = 0.6;
  iso.opacity = 1.0f;
  iso.color = {0.9f, 0.5f, 0.2f};
  iso.shininess = 24.0f;
  vs.isosurfaces.push_back(iso);
  rc.add_volume(makeSphereVolume(ctx, kGrid), vs);

  const frame_pair f = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(f.cpu), 1000) << "the CPU reference drew nothing";

  // Both backends must actually have anti-aliased: alpha is binary on this
  // scene at one sample per pixel, so a partially covered pixel can only come
  // from the sub-sample grid.  Without this the parity check would pass
  // trivially if supersample were silently ignored on one side.
  int partial_cpu = 0, partial_gpu = 0;
  for (int y = 0; y < kRaster; ++y)
    for (int x = 0; x < kRaster; ++x) {
      const int a = int(pixel(f.cpu, x, y)[3]), b = int(pixel(f.gpu, x, y)[3]);
      if (a > 0 && a < 255)
        ++partial_cpu;
      if (b > 0 && b < 255)
        ++partial_gpu;
    }
  std::printf("[volren-cuda supersample-2] partially covered pixels cpu=%d gpu=%d\n", partial_cpu,
              partial_gpu);
  std::fflush(stdout);
  EXPECT_GT(partial_cpu, 100) << "the CPU frame has no anti-aliased edge to compare";
  EXPECT_GT(partial_gpu, 100) << "the CUDA path ignored render_settings::supersample";

  expectParity("iso-sphere supersample=2", compare(f.cpu, f.gpu), kCellDiagonal);
}

// The device path enforces the SAME closed range as the CPU one, so an
// out-of-scope value is a loud error on both rather than a silent fallback.
TEST_F(VolrenCudaTest, SupersampleOutOfRangeRejectedOnTheDevice) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 64;
  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = true;
  vs.tf = rampTF(0.0, 1.0, 1.f, 1.f, 1.f, 1.f);
  vs.tf_auto_domain = false;
  rc.add_volume(makeSphereVolume(ctx, kGrid), vs);
  rc.set_backend(backend::cuda);

  rc.settings().supersample = cvc::volren::limits::max_supersample + 1;
  EXPECT_THROW(rc.render(), cvc::volren_error);
  rc.settings().supersample = 0;
  EXPECT_THROW(rc.render(), cvc::volren_error);
}

// ---------------------------------------------------------------------------
// Volumetric shadows (shadow.h)
// ---------------------------------------------------------------------------
// The maps themselves are DATA -- built by a nested render() that picks its own
// backend -- so what these tests pin is that the two marchers CONSUME the same
// map the same way.  Both scenes therefore run the identical raycaster twice,
// switching only set_backend(), and assert backend_used() == cuda so a silent
// fallback cannot make parity vacuous.

// Two disjoint lobes in ONE volume, as a normalized distance field whose
// 1.0-isosurface is the union of a radius-0.22 sphere at the origin and a
// radius-0.10 sphere at (0, 0.38, 0.38).  Three deliberate properties:
//  - the gradient points OUTWARD on both (makeSphereVolume's 1 - r field
//    points inward, which lights the far side and leaves nothing to shadow);
//  - the small lobe sits exactly on the (0, 1, 1) light ray through the big
//    lobe, so it genuinely shadows it;
//  - the two lobes do NOT overlap in SCREEN space under orthoCam().  They
//    must not: two surfaces 0.2 world units apart in depth sharing a pixel
//    would put that pixel's depth latch on a knife edge, and a fast-math
//    epsilon there produces a depth disagreement of several cells that has
//    nothing to do with shadows (the same reason TwoVolumeParity stacks its
//    volumes along the view direction).
cvc::volume makeTwoLobeVolume(cvc::app &ctx, unsigned n) {
  cvc::volume vol(ctx, cvc::dimension(n, n, n), cvc::Float,
                  cvc::bounding_box(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5));
  for (unsigned k = 0; k < n; ++k)
    for (unsigned j = 0; j < n; ++j)
      for (unsigned i = 0; i < n; ++i) {
        const double x = -0.5 + double(i) * vol.XSpan();
        const double y = -0.5 + double(j) * vol.YSpan();
        const double z = -0.5 + double(k) * vol.ZSpan();
        const double a = std::sqrt(x * x + y * y + z * z) / 0.22;
        const double by = y - 0.38, bz = z - 0.38;
        const double b = std::sqrt(x * x + by * by + bz * bz) / 0.10;
        vol(i, j, k, std::min(a, b));
      }
  return vol;
}

// A ball whose value IS the radius, so the gradient points outward and the
// isosurface is lit on the side facing the light.
cvc::volume makeBallVolume(cvc::app &ctx, unsigned n) {
  cvc::volume vol(ctx, cvc::dimension(n, n, n), cvc::Float,
                  cvc::bounding_box(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5));
  for (unsigned k = 0; k < n; ++k)
    for (unsigned j = 0; j < n; ++j)
      for (unsigned i = 0; i < n; ++i) {
        const double x = -0.5 + double(i) * vol.XSpan();
        const double y = -0.5 + double(j) * vol.YSpan();
        const double z = -0.5 + double(k) * vol.ZSpan();
        vol(i, j, k, std::sqrt(x * x + y * y + z * z));
      }
  return vol;
}

int luminanceAt(const frame &f, int x, int y) {
  const unsigned char *p = pixel(f, x, y);
  return int(p[0]) + int(p[1]) + int(p[2]);
}

TEST_F(VolrenCudaTest, SelfShadowParity) {
  // ONE volume shadowing ITSELF: the small lobe sits on the light ray through
  // the big lobe's apex.  This exercises the device lookup at the isosurface
  // shading site without needing a second volume.
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = kSteps;
  rc.settings().ambient = 0.15f;
  light l;
  l.direction = {0.0, 1.0, 1.0}; // through the small lobe onto the big one
  l.color = {1.f, 0.95f, 0.85f};
  rc.settings().lights.push_back(l);

  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  isosurface iso;
  iso.value = 1.0;
  iso.opacity = 1.0f;
  iso.color = {0.9f, 0.5f, 0.2f};
  iso.shininess = 24.0f;
  vs.isosurfaces.push_back(iso);
  rc.add_volume(makeTwoLobeVolume(ctx, kGrid), vs);

  // Reference with shadows off, so "parity" cannot pass on two identically
  // unshadowed images.
  rc.set_backend(backend::cpu);
  const frame unshadowed = rc.render();

  rc.settings().shadows.enabled = true;
  rc.settings().shadows.resolution = 512;
  const frame_pair f = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(f.cpu), 1000) << "the CPU reference drew nothing";

  int darkened = 0;
  for (int y = 0; y < kRaster; ++y)
    for (int x = 0; x < kRaster; ++x) {
      if (pixel(unshadowed, x, y)[3] == 0)
        continue;
      const int l0 = luminanceAt(unshadowed, x, y);
      if (luminanceAt(f.gpu, x, y) < l0 - l0 / 10)
        ++darkened;
    }
  EXPECT_GT(darkened, 100) << "the GPU render shows no shadow at all -- parity would be vacuous";

  expectParity("shadow-self", compare(f.cpu, f.gpu), kCellDiagonal);
}

TEST_F(VolrenCudaTest, InterVolumeShadowParity) {
  // TWO volumes, one shadowing the other.  (The v1 device path was
  // single-volume, which is why the design note said this scene could not run
  // on the GPU; the multi-volume kernel landed since.  One map over the whole
  // registered set is what makes "A shadows B" and "A shadows itself" the
  // identical lookup.)
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = kSteps;
  rc.settings().ambient = 0.15f;
  light l;
  l.direction = {1.0, 0.0, 1.0}; // 45 degrees in the x-z plane
  rc.settings().lights.push_back(l);

  // Receiver: the r = 0.4 ball at the origin, dome facing the camera.
  volume_settings receiver;
  receiver.shaded = false;
  receiver.unshaded = false;
  isosurface iso;
  iso.value = 0.4;
  iso.opacity = 1.0f;
  iso.color = {0.85f, 0.85f, 0.9f};
  receiver.isosurfaces.push_back(iso);
  rc.add_volume(makeBallVolume(ctx, kGrid), receiver);

  // Occluder: the same ball moved up and along +x so that it sits ON the light
  // ray through the receiver's apex, and OFF the receiver in screen space.
  volume_settings occluder = receiver;
  occluder.isosurfaces[0].color = {0.9f, 0.4f, 0.4f};
  occluder.model_transform = translate(0.9, 0.0, 0.9);
  rc.add_volume(makeBallVolume(ctx, kGrid), occluder);

  rc.set_backend(backend::cpu);
  const frame unshadowed = rc.render();

  rc.settings().shadows.enabled = true;
  rc.settings().shadows.resolution = 512;
  const frame_pair f = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(f.cpu), 1000);

  int darkened = 0;
  for (int y = 0; y < kRaster; ++y)
    for (int x = 0; x < kRaster; ++x) {
      // Receiver only: the occluder is nearer the camera (its dome tops out at
      // z = 1.3, eye-space depth 2.7) and is itself unshadowed.
      if (pixel(unshadowed, x, y)[3] == 0 || depthAt(unshadowed, x, y) < 3.55f)
        continue;
      const int l0 = luminanceAt(unshadowed, x, y);
      if (luminanceAt(f.gpu, x, y) < l0 - l0 / 10)
        ++darkened;
    }
  EXPECT_GT(darkened, 100) << "the occluder cast nothing on the GPU";

  expectParity("shadow-inter-volume", compare(f.cpu, f.gpu), kCellDiagonal);
}

TEST_F(VolrenCudaTest, ShadowedShadedTransferFunctionParity) {
  // The other device shading site: a marched shaded-TF sample rather than an
  // isosurface hit.  Both sites call the same shadow_factors(), but they pass
  // different world points (a march sample versus an exact MC intersection),
  // so a mix-up between them only shows up here.
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = kSteps;
  rc.settings().ambient = 0.2f;
  light l;
  l.direction = {0.6, 0.0, 0.8};
  rc.settings().lights.push_back(l);

  volume_settings vs;
  vs.shaded = true;
  vs.tf_auto_domain = false;
  vs.tf = rampTF(0.35, 0.65, 0.8f, 0.85f, 0.95f, 0.5f);
  rc.add_volume(makeSphereVolume(ctx, kGrid), vs);

  rc.settings().shadows.enabled = true;
  rc.settings().shadows.resolution = 512;
  const frame_pair f = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(f.cpu), 1000);
  expectParity("shadow-shaded-tf", compare(f.cpu, f.gpu), kCellDiagonal);
}

TEST_F(VolrenCudaTest, ShadowsOffLeaveTheDevicePathUntouched) {
  // The no-op contract on the GPU side: with shadows disabled the kernel takes
  // the pre-shadow path, and the frame must be BYTE-identical to the one it
  // produced before -- asserted here against itself across a render with
  // strength 0, which drives the same branch from the other direction.
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = kSteps;
  light l;
  l.direction = {0.3, 0.4, 1.0};
  rc.settings().lights.push_back(l);

  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  isosurface iso;
  iso.value = 0.6;
  iso.opacity = 1.0f;
  vs.isosurfaces.push_back(iso);
  rc.add_volume(makeSphereVolume(ctx, kGrid), vs);

  rc.set_backend(backend::cuda);
  const frame off = rc.render();
  ASSERT_EQ(int(rc.backend_used()), int(backend::cuda));

  rc.settings().shadows.enabled = true;
  rc.settings().shadows.strength = 0.f;
  const frame zero = rc.render();
  ASSERT_EQ(int(rc.backend_used()), int(backend::cuda));

  const std::size_t cbytes = std::size_t(kRaster) * kRaster * 4;
  const std::size_t dbytes = std::size_t(kRaster) * kRaster * sizeof(float);
  EXPECT_EQ(std::memcmp(off.color.data(), zero.color.data(), cbytes), 0)
      << "strength 0 changed the device image";
  EXPECT_EQ(std::memcmp(off.depth.data(), zero.depth.data(), dbytes), 0);
}

} // namespace

#endif // CVC_ENABLE_CUDA
