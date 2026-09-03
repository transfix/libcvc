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
  m.m[0] = 1.0;  m.m[1] = 0.0; m.m[2] = 0.0;  m.m[3] = tx;
  m.m[4] = 0.0;  m.m[5] = c;   m.m[6] = -s;   m.m[7] = ty;
  m.m[8] = 0.0;  m.m[9] = s;   m.m[10] = c;   m.m[11] = tz;
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

TEST_F(VolrenCudaTest, TwoVolumesRejectedByCudaAndFallBackUnderAutomatic) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 64;
  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = true;
  vs.tf = rampTF(0.0, 1.0, 1.f, 1.f, 1.f, 1.f);
  vs.tf_auto_domain = false;
  rc.add_volume(makeSphereVolume(ctx, 16), vs);
  rc.add_volume(makeSphereVolume(ctx, 16), vs);

  // The v1 device path renders exactly one volume: an explicit request is an
  // error (never a silent CPU march), automatic degrades quietly.
  rc.set_backend(backend::cuda);
  EXPECT_THROW(rc.render(), cvc::volren_error);

  rc.set_backend(backend::automatic);
  const frame f = rc.render();
  EXPECT_EQ(int(rc.backend_used()), int(backend::cpu));
  EXPECT_EQ(f.color.width(), kRaster);
}

} // namespace

#endif // CVC_ENABLE_CUDA
