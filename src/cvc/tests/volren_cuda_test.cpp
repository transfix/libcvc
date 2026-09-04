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
  // NOT named `small`: the Windows SDK's rpcndr.h has `#define small char`,
  // so `cvc::volume small` becomes `cvc::volume char` and MSVC rejects the
  // whole translation unit (this is how it first broke the Windows build).
  const cvc::volume tiny = makeSphereVolume(ctx, 16);
  for (int i = 0; i <= cvc::volren::cuda_limits::max_volumes; ++i)
    rc.add_volume(tiny, vs);

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

  volume_settings nearVol;
  nearVol.tf = rampTF(0.0, 1.0, 0.9f, 0.4f, 0.3f, 0.7f);
  nearVol.tf_auto_domain = false;
  nearVol.gradient_ramp.enabled = true;
  nearVol.gradient_ramp.ramp0 = 0.0;
  nearVol.gradient_ramp.ramp1 = 0.5;
  nearVol.gradient_ramp.ramp2 = 4.0;
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
  nearVol.isosurfaces.push_back(near_iso);
  nearVol.model_transform = translate(0.0, 0.0, 0.8);
  rc.add_volume(makeSphereVolume(ctx, kGrid), nearVol);

  volume_settings farVol;
  farVol.tf = rampTF(0.0, 1.0, 0.3f, 0.6f, 0.95f, 0.6f);
  farVol.tf_auto_domain = false;
  farVol.gradient_ramp.enabled = true;
  farVol.gradient_ramp.ramp0 = 0.1;
  farVol.gradient_ramp.ramp1 = 0.8;
  farVol.gradient_ramp.ramp2 = 3.0;
  isosurface iso;
  iso.value = 0.68;
  iso.opacity = 0.5f;
  iso.color = {1.f, 0.9f, 0.4f};
  farVol.isosurfaces.push_back(iso);
  farVol.model_transform = translate(0.0, 0.0, -0.8);
  rc.add_volume(makeSphereVolume(ctx, kGrid), farVol);

  const frame_pair f = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(f.cpu), 1000) << "the CPU reference drew nothing";
  expectParity("two-volume", compare(f.cpu, f.gpu), kCellDiagonal);

  // The far volume really is behind the near one on the central rays: drop it
  // and the image must change, or "two volumes" was only ever one.
  raycaster solo(ctx);
  solo.view() = rc.view();
  solo.settings() = rc.settings();
  solo.add_volume(makeSphereVolume(ctx, kGrid), nearVol);
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

TEST_F(VolrenCudaTest, DeepShadowParity) {
  // shadow_mode::deep changes BOTH sides of the device path: the light pass
  // runs the capture instantiation of the kernel (a second output per ray) and
  // the main pass runs the two-channel profile lookup.  The scene is chosen so
  // both are exercised and neither can be vacuous -- a TRANSLUCENT occluder, so
  // the terminal channel alone cannot answer, over an opaque receiver.
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = kSteps;
  rc.settings().ambient = 0.15f;
  light l;
  l.direction = {1.0, 0.0, 1.0}; // 45 degrees in the x-z plane
  l.color = {0.5f, 0.5f, 0.5f};  // dim enough that the receiver does not clamp
  rc.settings().lights.push_back(l);

  volume_settings receiver;
  receiver.shaded = false;
  receiver.unshaded = false;
  isosurface iso;
  iso.value = 0.4;
  iso.opacity = 1.0f;
  iso.color = {0.85f, 0.85f, 0.9f};
  receiver.isosurfaces.push_back(iso);
  rc.add_volume(makeBallVolume(ctx, kGrid), receiver);

  volume_settings occluder = receiver;
  occluder.isosurfaces[0].opacity = 0.5f; // TRANSLUCENT: the profile must carry it
  occluder.isosurfaces[0].color = {0.9f, 0.4f, 0.4f};
  occluder.model_transform = translate(0.9, 0.0, 0.9);
  rc.add_volume(makeBallVolume(ctx, kGrid), occluder);

  rc.set_backend(backend::cpu);
  const frame unshadowed = rc.render();

  rc.settings().shadows.enabled = true;
  rc.settings().shadows.resolution = 512;
  rc.settings().shadows.mode = cvc::volren::shadow_mode::deep;
  rc.settings().shadows.depth_slices = cvc::volren::defaults::shadow_depth_slices;

  // renderBoth() renders the same raycaster twice, and the shadow-map cache is
  // keyed on the light pass's inputs -- which do not include the backend -- so
  // the CUDA pass CONSUMES the map the CPU pass built.  That is the point here:
  // this test isolates the lookup.  Parity of the map PRODUCTION is a separate
  // question, pinned by DeepShadowProfileProducedOnEitherBackendAgrees below.
  const frame_pair f = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(f.cpu), 1000);

  // Not vacuous in either direction: the deep shadow must darken the receiver,
  // and it must NOT darken it all the way to the hard answer.
  rc.set_backend(backend::cpu);
  rc.settings().shadows.mode = cvc::volren::shadow_mode::hard;
  // Below the 0.5 default the translucent occluder is dropped from the HARD
  // light pass entirely, and there would be no fully-shadowed end to compare
  // against.  (Deep mode ignores this knob.)
  rc.settings().shadows.min_occluder_opacity = 0.2f;
  const frame hard = rc.render();

  int darkened = 0, partial = 0;
  for (int y = 0; y < kRaster; ++y)
    for (int x = 0; x < kRaster; ++x) {
      // Receiver only: the occluder is nearer the camera and is itself unshadowed.
      if (pixel(unshadowed, x, y)[3] == 0 || depthAt(unshadowed, x, y) < 3.55f)
        continue;
      const int l0 = luminanceAt(unshadowed, x, y);
      const int ld = luminanceAt(f.gpu, x, y);
      const int lh = luminanceAt(hard, x, y);
      if (!(lh < l0 - l0 / 10))
        continue; // not shadowed by the hard map either
      if (ld < l0 - l0 / 20)
        ++darkened;
      if (ld > lh + 2)
        ++partial; // strictly lighter than the fully-shadowed answer
    }
  EXPECT_GT(darkened, 100) << "the GPU deep render shows no shadow at all";
  EXPECT_GT(partial, 100)
      << "the GPU deep shadow is as dark as the hard one -- the profile was ignored";

  expectParity("shadow-deep", compare(f.cpu, f.gpu), kCellDiagonal);
}

TEST_F(VolrenCudaTest, DeepShadowProfileProducedOnEitherBackendAgrees) {
  // The light pass is an ordinary render(), so its profile can be produced by
  // either marcher.  Build the SAME map both ways and compare the payload
  // itself -- the strongest statement about the new kernel output, and the one
  // an image-level parity check cannot make (a wrong knot deep inside an opaque
  // occluder is invisible in the frame).
  const auto build = [&](backend b) {
    raycaster rc(ctx);
    rc.set_backend(b);
    rc.view() = orthoCam();
    rc.settings().steps = kSteps;
    rc.settings().ambient = 0.15f;
    light l;
    l.direction = {1.0, 0.0, 1.0};
    rc.settings().lights.push_back(l);

    volume_settings vs;
    vs.shaded = false;
    vs.unshaded = false;
    isosurface iso;
    iso.value = 0.4;
    iso.opacity = 0.5f; // translucent: the knots carry real values
    iso.color = {0.85f, 0.85f, 0.9f};
    vs.isosurfaces.push_back(iso);
    rc.add_volume(makeBallVolume(ctx, kGrid), vs);

    rc.settings().shadows.enabled = true;
    rc.settings().shadows.resolution = 256;
    rc.settings().shadows.mode = cvc::volren::shadow_mode::deep;
    rc.settings().shadows.depth_slices = 16;
    rc.render();
    EXPECT_EQ(int(rc.backend_used()), int(b));
    return rc.shadow_map_profile(0);
  };

  const cvc::image cpu = build(backend::cpu);
  const cvc::image gpu = build(backend::cuda);
  ASSERT_EQ(cpu.width(), gpu.width());
  ASSERT_EQ(cpu.height(), gpu.height());
  ASSERT_GT(cpu.width(), 0);

  const float *a = reinterpret_cast<const float *>(cpu.data());
  const float *b = reinterpret_cast<const float *>(gpu.data());
  const std::size_t n = std::size_t(cpu.width()) * std::size_t(cpu.height());
  const std::size_t plane = n / 17; // slices + 1 planes
  double worst_alpha = 0.0, worst_term = 0.0;
  int nonzero = 0, term_mismatch = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const bool is_terminal = i < plane;
    if (is_terminal) {
      // The terminal is +inf almost everywhere here (nothing saturates a 0.5
      // surface crossed twice); where it is finite the two must agree to well
      // inside a cell.
      const bool ia = std::isinf(a[i]), ib = std::isinf(b[i]);
      if (ia != ib)
        ++term_mismatch;
      else if (!ia)
        worst_term = std::max(worst_term, double(std::fabs(a[i] - b[i])));
      continue;
    }
    if (a[i] > 0.f || b[i] > 0.f)
      ++nonzero;
    worst_alpha = std::max(worst_alpha, double(std::fabs(a[i] - b[i])));
  }
  std::printf("[volren-cuda shadow-deep-profile] knots=%zu nonzero=%d worst_alpha_diff=%.3e "
              "worst_terminal_diff=%.3e terminal_mask_mismatch=%d\n",
              n - plane, nonzero, worst_alpha, worst_term, term_mismatch);
  std::fflush(stdout);

  ASSERT_GT(nonzero, 1000) << "the profile is empty -- the comparison would be vacuous";
  // Accumulated alpha is a product of float opacities on both sides; --use_fast_math
  // moves the LAST bits, not the value.
  EXPECT_LE(worst_alpha, 1e-4) << "the two backends built different transmittance profiles";
  EXPECT_LE(worst_term, kCellDiagonal) << "the two backends terminated at different depths";
  EXPECT_LE(term_mismatch, int(0.001 * double(plane)))
      << "the two backends disagree about WHETHER the light ray terminated";
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

// ---------------------------------------------------------------------------
// The lighting rig: soft shadows, ambient occlusion, hemisphere ambient
// ---------------------------------------------------------------------------
// Each of these turns ONE new term on and renders the identical raycaster on
// both backends.  The point is narrow and worth stating: every one of them adds
// a new arithmetic path to the device kernel (a tap loop, a trilinear cone, a
// normal-dependent tint), and a transcription slip in any of them is a
// systematic image difference, not a rounding one.

TEST_F(VolrenCudaTest, SoftShadowParity) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = kSteps;
  rc.settings().ambient = 0.15f;
  light l;
  l.direction = {0.0, 1.0, 1.0}; // the SelfShadowParity geometry
  l.color = {1.f, 0.95f, 0.85f};
  rc.settings().lights.push_back(l);

  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  isosurface iso;
  iso.value = 1.0;
  iso.opacity = 1.0f;
  iso.color = {0.9f, 0.85f, 0.8f};
  vs.isosurfaces.push_back(iso);
  rc.add_volume(makeTwoLobeVolume(ctx, kGrid), vs);

  rc.settings().shadows.enabled = true;
  rc.settings().shadows.resolution = 256;

  // Every combination of payload and filter shape, because the tap loop wraps
  // BOTH lookups and the two payloads reach it by different branches.
  for (const cvc::volren::shadow_mode mode :
       {cvc::volren::shadow_mode::hard, cvc::volren::shadow_mode::deep}) {
    rc.settings().shadows.mode = mode;
    rc.settings().shadows.depth_slices = 16;
    for (const int taps : {3, 5, 7}) {
      rc.settings().shadows.pcf_radius = 3.f;
      rc.settings().shadows.pcf_taps = taps;
      const frame_pair p = renderBoth(rc);
      ASSERT_GT(countAlphaPositive(p.cpu), 200);
      char label[64];
      std::snprintf(label, sizeof(label), "soft-shadow-%s-taps%d",
                    mode == cvc::volren::shadow_mode::deep ? "deep" : "hard", taps);
      expectParity(label, compare(p.cpu, p.gpu), kCellDiagonal);
    }
  }
}

TEST_F(VolrenCudaTest, AmbientOcclusionParity) {
  // The cone is `samples` trilinear fetches per hit in the volume's LOCAL
  // frame; the sweep over sample counts is what would catch an off-by-one in
  // the falloff or in the tap spacing rather than a wholesale mistake.
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = kSteps;
  rc.settings().ambient = 0.6f; // AO attenuates AMBIENT, so give it something
  light l;
  l.direction = {0.3, 0.4, 1.0};
  l.color = {0.6f, 0.6f, 0.6f};
  rc.settings().lights.push_back(l);

  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  // makeTwoLobeVolume's field is a distance NORMALIZED by each lobe's radius,
  // so `f - 1` is the true distance over-reported by 1/0.22.  That is not the
  // exact SDF the estimator is documented against, and it is deliberate here:
  // this test is about the two backends agreeing on the same arithmetic, and a
  // field whose distances are stretched simply under-occludes -- the "it
  // actually did something" check below is what keeps that from being vacuous.
  vs.distance_field = true;
  isosurface iso;
  iso.value = 1.0;
  iso.opacity = 1.0f;
  iso.color = {0.9f, 0.85f, 0.8f};
  vs.isosurfaces.push_back(iso);
  rc.add_volume(makeTwoLobeVolume(ctx, kGrid), vs);

  rc.settings().ao.strength = 1.0f;
  for (const int samples : {1, 5, 16}) {
    for (const double radius : {0.15, 0.5}) {
      rc.settings().ao.samples = samples;
      rc.settings().ao.radius = radius;
      const frame_pair p = renderBoth(rc);
      ASSERT_GT(countAlphaPositive(p.cpu), 200);
      char label[64];
      std::snprintf(label, sizeof(label), "ao-n%d-r%.2f", samples, radius);
      expectParity(label, compare(p.cpu, p.gpu), kCellDiagonal);
    }
  }

  // And the cone must actually have DONE something, or the parity above is a
  // statement about two identical no-ops.
  rc.settings().ao.samples = 5;
  rc.settings().ao.radius = 0.5;
  rc.set_backend(backend::cuda);
  const frame with_ao = rc.render();
  ASSERT_EQ(int(rc.backend_used()), int(backend::cuda));
  rc.settings().ao.strength = 0.f;
  const frame without = rc.render();
  int darkened = 0;
  for (int y = 0; y < kRaster; ++y)
    for (int x = 0; x < kRaster; ++x)
      if (pixel(without, x, y)[3] > 0 &&
          luminanceAt(with_ao, x, y) < luminanceAt(without, x, y) - 3)
        ++darkened;
  EXPECT_GT(darkened, 20) << "the device AO cone changed nothing, so its parity is vacuous";

  // Turning it off on the DEVICE is byte-exact, not merely close: the kernel
  // takes the flat-ambient branch, which is the pre-AO expression.
  rc.settings().ao.strength = 1.f;
  rc.settings().ao.radius = 0.0;
  const frame radius_zero = rc.render();
  const std::size_t cbytes = std::size_t(kRaster) * kRaster * 4;
  EXPECT_EQ(std::memcmp(without.color.data(), radius_zero.color.data(), cbytes), 0)
      << "radius 0 changed the device image";
  rc.settings().ao.radius = 0.5;
  rc.volume_config(0).distance_field = false;
  EXPECT_EQ(std::memcmp(without.color.data(), rc.render().color.data(), cbytes), 0)
      << "a volume that is not a distance field still ran the device cone";
}

TEST_F(VolrenCudaTest, AmbientRigParity) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = kSteps;
  rc.settings().ambient = 0.5f;
  rc.settings().shading_gain = 1.0f;
  rc.settings().specular = 0.25f;
  rc.settings().ambient_hemisphere.enabled = true;
  rc.settings().ambient_hemisphere.sky = {0.55f, 0.7f, 1.f};
  rc.settings().ambient_hemisphere.ground = {0.42f, 0.32f, 0.22f};
  rc.settings().ambient_hemisphere.up = {0.0, 1.0, 0.0}; // orthoCam's screen up
  // Two lights of different colours, so the accumulation and the per-channel
  // specular are both exercised.
  light key, fill;
  key.direction = {0.3, 0.4, 1.0};
  key.color = {0.8f, 0.76f, 0.7f};
  fill.direction = {-0.7, -0.2, 0.4};
  fill.color = {0.2f, 0.26f, 0.4f};
  rc.settings().lights = {key, fill};

  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  isosurface iso;
  iso.value = 0.6;
  iso.opacity = 1.0f;
  iso.color = {0.9f, 0.85f, 0.8f};
  vs.isosurfaces.push_back(iso);
  rc.add_volume(makeSphereVolume(ctx, kGrid), vs);

  frame_pair p = renderBoth(rc);
  ASSERT_GT(countAlphaPositive(p.cpu), 200);
  expectParity("ambient-rig-isosurface", compare(p.cpu, p.gpu), kCellDiagonal);

  // The same rig on a SHADED transfer-function volume, whose shading site is a
  // different call in both marchers.
  raycaster tf(ctx);
  tf.view() = orthoCam();
  tf.settings() = rc.settings();
  volume_settings tvs;
  tvs.shaded = true;
  tvs.unshaded = false;
  tvs.tf = rampTF(0.0, 1.0, 0.9f, 0.7f, 0.5f, 0.6f);
  tvs.tf_auto_domain = false;
  tf.add_volume(makeSphereVolume(ctx, kGrid), tvs);
  p = renderBoth(tf);
  ASSERT_GT(countAlphaPositive(p.cpu), 200);
  expectParity("ambient-rig-shaded-tf", compare(p.cpu, p.gpu), kCellDiagonal);
}

} // namespace

#endif // CVC_ENABLE_CUDA
