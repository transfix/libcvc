// End-to-end tests for the cvc::volren raycaster (raycaster.h/.cpp).
//
// Geometry used throughout: volumes live in [-0.5, 0.5]^3 (node-centered,
// voxel i at min + i*span), the camera sits at (0,0,4) looking at the origin
// with up = +y, so screen right is world +x and screen up is world +y.  The
// orthographic camera uses parallel_scale 1.0 over a 64x64 raster, mapping
// pixel (px,py) center to world (u, v) with u = (px+0.5)/64*2-1 and
// v = 1-(py+0.5)/64*2 -- the 0.5-halfwidth volume box projects to the middle
// 32x32 pixels.

#include <algorithm>
#include <boost/thread/thread.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/volren/raycaster.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>
#include <limits>
#include <vector>

using cvc::volren::camera;
using cvc::volren::cut_plane;
using cvc::volren::frame;
using cvc::volren::isosurface;
using cvc::volren::light;
using cvc::volren::mat4;
using cvc::volren::raycaster;
using cvc::volren::render_settings;
using cvc::volren::shadow_view;
using cvc::volren::transfer_function;
using cvc::volren::transfer_point;
using cvc::volren::volume_settings;

namespace {

constexpr int kRaster = 64;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// val = world z coordinate (range [-0.5, 0.5], strictly increasing in k).
cvc::volume makeLinearVolume(cvc::app &ctx, unsigned n) {
  cvc::volume vol(ctx, cvc::dimension(n, n, n), cvc::Float,
                  cvc::bounding_box(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5));
  for (unsigned k = 0; k < n; ++k)
    for (unsigned j = 0; j < n; ++j)
      for (unsigned i = 0; i < n; ++i)
        vol(i, j, k, -0.5 + double(k) * vol.ZSpan());
  return vol;
}

// val = 1 - r about the box center (center high, decreasing outward).
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

// Flat RGBA over any bake domain (control points far outside all data).
transfer_function flatTF(float r, float g, float b, float a) {
  transfer_function tf;
  tf.add(transfer_point{-1000.0, r, g, b, a});
  tf.add(transfer_point{1000.0, r, g, b, a});
  return tf;
}

volume_settings unshadedSettings(const transfer_function &tf) {
  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = true;
  vs.tf = tf;
  return vs;
}

mat4 translation(double tx, double ty, double tz) {
  mat4 m;
  m.m[3] = tx;
  m.m[7] = ty;
  m.m[11] = tz;
  return m;
}

mat4 uniformScale(double s) {
  mat4 m;
  m.m[0] = m.m[5] = m.m[10] = s;
  return m;
}

const unsigned char *pixel(const frame &f, int x, int y) {
  const cvc::image &img = f.color; // const ref: no copy-on-write detach
  return img.data() + (std::size_t(y) * img.width() + x) * 4;
}

float depthAt(const frame &f, int x, int y) {
  const cvc::image &img = f.depth;
  return reinterpret_cast<const float *>(img.data())[std::size_t(y) * img.width() + x];
}

// Replicates the renderer's to_byte rounding.
int expectedByte(float c) {
  const float v = std::min(std::max(c, 0.f), 1.f);
  return int(v * 255.0f + 0.5f);
}

// Pixel-center world coordinates under orthoCam() (parallel_scale 1, aspect 1).
double orthoU(int px) { return (double(px) + 0.5) / double(kRaster) * 2.0 - 1.0; }
double orthoV(int py) { return 1.0 - (double(py) + 0.5) / double(kRaster) * 2.0; }

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

std::uint64_t sumRGB(const frame &f) {
  const cvc::image &img = f.color;
  const unsigned char *d = img.data();
  const int n = img.width() * img.height();
  std::uint64_t sum = 0;
  for (int i = 0; i < n; ++i)
    sum += std::uint64_t(d[i * 4]) + d[i * 4 + 1] + d[i * 4 + 2];
  return sum;
}

class VolrenRenderTest : public ::testing::Test {
protected:
  cvc::app ctx;
};

// ---------------------------------------------------------------------------
// 1. Validation
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, RejectsDegenerateVolumesAndSettings) {
  raycaster rc(ctx);
  rc.view() = orthoCam();

  // dim < 2 on any axis.
  cvc::volume thin(ctx, cvc::dimension(1, 4, 4), cvc::Float,
                   cvc::bounding_box(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5));
  EXPECT_THROW(rc.add_volume(thin), cvc::volren_error);

  // Render with no volumes registered.
  EXPECT_THROW(rc.render(), cvc::volren_error);
  EXPECT_THROW(rc.scene_bounds(), cvc::volren_error);

  // volume_config out of range (none registered yet).
  EXPECT_THROW(rc.volume_config(0), cvc::volren_error);

  // Register a valid volume, then break scene settings one at a time.
  cvc::volume vol = makeSphereVolume(ctx, 8);
  const std::size_t idx = rc.add_volume(vol, unshadedSettings(flatTF(1.f, 1.f, 1.f, 1.f)));
  EXPECT_EQ(idx, 0u);
  EXPECT_THROW(rc.volume_config(1), cvc::volren_error);

  rc.settings().steps = 0;
  EXPECT_THROW(rc.render(), cvc::volren_error);
  rc.settings().steps = 64;

  rc.settings().opacity_cutoff = 0.f;
  EXPECT_THROW(rc.render(), cvc::volren_error);
  rc.settings().opacity_cutoff = cvc::volren::defaults::opacity_cutoff;

  // supersample is a closed range, not just "positive": the cap is part of the
  // contract (see limits::max_supersample), so both ends reject.
  rc.settings().supersample = 0;
  EXPECT_THROW(rc.render(), cvc::volren_error);
  rc.settings().supersample = cvc::volren::limits::max_supersample + 1;
  EXPECT_THROW(rc.render(), cvc::volren_error);
  rc.settings().supersample = cvc::volren::limits::max_supersample;
  EXPECT_NO_THROW(rc.render());
}

// ---------------------------------------------------------------------------
// 2. Missed rays
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, MissedRaysAreBackground) {
  raycaster rc(ctx);
  rc.view() = perspectiveCam();
  rc.settings().steps = 128;
  rc.settings().background = {0.25f, 0.5f, 0.75f};
  rc.add_volume(makeSphereVolume(ctx, 32), unshadedSettings(flatTF(1.f, 1.f, 1.f, 1.f)));

  const frame f = rc.render();

  const int corners[4][2] = {
      {0, 0}, {kRaster - 1, 0}, {0, kRaster - 1}, {kRaster - 1, kRaster - 1}};
  for (const auto &c : corners) {
    const unsigned char *px = pixel(f, c[0], c[1]);
    EXPECT_EQ(int(px[0]), expectedByte(0.25f)) << "corner " << c[0] << "," << c[1];
    EXPECT_EQ(int(px[1]), expectedByte(0.5f)) << "corner " << c[0] << "," << c[1];
    EXPECT_EQ(int(px[2]), expectedByte(0.75f)) << "corner " << c[0] << "," << c[1];
    EXPECT_EQ(int(px[3]), 0) << "corner " << c[0] << "," << c[1];
    EXPECT_TRUE(std::isinf(depthAt(f, c[0], c[1]))) << "corner " << c[0] << "," << c[1];
  }
}

// ---------------------------------------------------------------------------
// 3. Orthographic silhouette
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, VolumeSilhouetteOrtho) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 256;

  // Alpha ramps 0 -> 1 over the sphere field's auto-baked [min, max] domain.
  transfer_function tf;
  tf.add(transfer_point{0.0, 1.f, 1.f, 1.f, 0.f});
  tf.add(transfer_point{1.0, 1.f, 1.f, 1.f, 1.f});
  rc.add_volume(makeSphereVolume(ctx, 32), unshadedSettings(tf));

  const frame f = rc.render();

  // The box projects to |u| <= 0.5 (pixels ~16..47); its center is hit.
  EXPECT_GT(int(pixel(f, 32, 32)[3]), 0);

  // Pixels well outside the projected box see nothing.
  const int outside[4][2] = {{4, 32}, {60, 32}, {32, 4}, {32, 60}};
  for (const auto &c : outside)
    EXPECT_EQ(int(pixel(f, c[0], c[1])[3]), 0) << "pixel " << c[0] << "," << c[1];
}

// ---------------------------------------------------------------------------
// 4. Unshaded color path
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, UnshadedColorIsTransferFunction) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 256; // background stays default black; no lights

  // Single flat color (0.8, 0.4, 0.2) with an alpha ramp over an explicit
  // [0, 1] domain (tf_auto_domain = false bakes over the point extents).
  transfer_function tf;
  tf.add(transfer_point{0.0, 0.8f, 0.4f, 0.2f, 0.f});
  tf.add(transfer_point{1.0, 0.8f, 0.4f, 0.2f, 0.9f});
  volume_settings vs = unshadedSettings(tf);
  vs.tf_auto_domain = false;
  rc.add_volume(makeSphereVolume(ctx, 32), vs);

  const frame f = rc.render();
  const unsigned char *px = pixel(f, 32, 32);

  // Accumulated color is (0.8, 0.4, 0.2) * acc_a over black, so the channel
  // ratios must match the TF's 1 : 0.5 : 0.25 within a few LSB of rounding.
  ASSERT_GT(int(px[3]), 0);
  EXPECT_GT(int(px[0]), 0);
  EXPECT_NEAR(double(px[1]), double(px[0]) * 0.5, 3.0);
  EXPECT_NEAR(double(px[2]), double(px[0]) * 0.25, 3.0);
}

// ---------------------------------------------------------------------------
// 5. Degenerate auto domain
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, ConstantVolumeAutoDomainIsTransparent) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 128;

  cvc::volume vol(ctx, cvc::dimension(16, 16, 16), cvc::Float,
                  cvc::bounding_box(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5));
  vol.fill(0.5);
  // tf_auto_domain (default true) bakes over [0.5, 0.5] -- documented to
  // produce an empty (fully transparent) LUT rather than a one-value table.
  rc.add_volume(vol, unshadedSettings(flatTF(1.f, 1.f, 1.f, 1.f)));

  const frame f = rc.render();

  EXPECT_EQ(countAlphaPositive(f), 0);
}

// ---------------------------------------------------------------------------
// 6. Depth map
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, DepthMapLatch) {
  raycaster rc(ctx);
  rc.view() = orthoCam(); // straight down -z from eye z = 4
  rc.settings().steps = 256;
  rc.add_volume(makeLinearVolume(ctx, 32), unshadedSettings(flatTF(1.f, 1.f, 1.f, 1.f)));

  const frame f = rc.render();

  // Front face at z = 0.5 is 3.5 from the eye; the first contributing sample
  // sits within a step or two of the entry point.
  const double diag = std::sqrt(3.0);
  const double tol = 3.0 * diag / 256.0;
  EXPECT_TRUE(std::isfinite(depthAt(f, 32, 32)));
  EXPECT_NEAR(double(depthAt(f, 32, 32)), 3.5, tol);

  // A corner ray misses the scene box entirely.
  EXPECT_TRUE(std::isinf(depthAt(f, 0, 0)));

  // Depth is finite wherever accumulated alpha reached depth_alpha_threshold
  // (alpha byte >= 128 implies acc_a >= 0.5, the default threshold).
  int violations = 0;
  for (int py = 0; py < kRaster; ++py)
    for (int px = 0; px < kRaster; ++px)
      if (pixel(f, px, py)[3] >= 128 && !std::isfinite(depthAt(f, px, py)))
        ++violations;
  EXPECT_EQ(violations, 0);
}

TEST_F(VolrenRenderTest, DepthRespectsThreshold) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 256;
  // With per-cell alpha 0.6 and the default 0.95 cutoff, accumulation stops
  // at acc_a ~= 0.9744 -- below a 0.99 depth threshold, so depth never
  // latches even though alpha saturates past the cutoff.
  rc.settings().depth_alpha_threshold = 0.99f;
  rc.add_volume(makeLinearVolume(ctx, 32), unshadedSettings(flatTF(1.f, 1.f, 1.f, 0.6f)));

  const frame f = rc.render();

  EXPECT_GE(int(pixel(f, 32, 32)[3]), int(0.95f * 255.f)); // saturated
  int finite = 0;
  for (int py = 0; py < kRaster; ++py)
    for (int px = 0; px < kRaster; ++px)
      if (std::isfinite(depthAt(f, px, py)))
        ++finite;
  EXPECT_EQ(finite, 0);
}

// ---------------------------------------------------------------------------
// 8. Thread-count invariance
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, RenderIsThreadCountInvariant) {
  raycaster rc(ctx);
  rc.view() = perspectiveCam();
  rc.settings().steps = 128;
  rc.settings().two_sided_lighting = true;
  light l;
  l.direction = {0.0, 0.0, 1.0};
  rc.settings().lights.push_back(l);

  // Shaded TF + an isosurface exercises the gradient and MC paths.
  transfer_function tf;
  tf.add(transfer_point{0.0, 0.9f, 0.6f, 0.3f, 0.f});
  tf.add(transfer_point{1.0, 0.9f, 0.6f, 0.3f, 0.6f});
  volume_settings vs;
  vs.tf = tf;
  isosurface iso;
  iso.value = 0.6;
  iso.opacity = 0.5f;
  vs.isosurfaces.push_back(iso);
  rc.add_volume(makeSphereVolume(ctx, 32), vs);

  rc.settings().threads = 1;
  const frame serial = rc.render();
  rc.settings().threads = 0;
  const frame pooled1 = rc.render();
  const frame pooled2 = rc.render();

  const std::size_t color_bytes = std::size_t(kRaster) * kRaster * 4;
  const std::size_t depth_bytes = std::size_t(kRaster) * kRaster * sizeof(float);
  const cvc::image &sc = serial.color, &p1c = pooled1.color, &p2c = pooled2.color;
  const cvc::image &sd = serial.depth, &p1d = pooled1.depth, &p2d = pooled2.depth;

  EXPECT_EQ(std::memcmp(sc.data(), p1c.data(), color_bytes), 0);
  EXPECT_EQ(std::memcmp(sd.data(), p1d.data(), depth_bytes), 0);
  EXPECT_EQ(std::memcmp(p1c.data(), p2c.data(), color_bytes), 0);
  EXPECT_EQ(std::memcmp(p1d.data(), p2d.data(), depth_bytes), 0);
}

// ---------------------------------------------------------------------------
// 9. Early termination
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, OpaqueVolumeSaturatesCenterAlpha) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 128;
  rc.add_volume(makeSphereVolume(ctx, 32), unshadedSettings(flatTF(1.f, 1.f, 1.f, 1.f)));

  const frame f = rc.render();
  const int a = int(pixel(f, 32, 32)[3]);
  EXPECT_GE(a, int(rc.settings().opacity_cutoff * 255.f));
  EXPECT_LE(a, 255);
}

// ---------------------------------------------------------------------------
// 10. Cut planes
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, CutPlaneCullsHalf) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 256;
  // Keep x >= 0 (samples with dot(p - point, normal) < 0 are culled).
  // right = up x back = (1,0,0), so the kept side is screen RIGHT.
  cut_plane cp;
  cp.point = {0.0, 0.0, 0.0};
  cp.normal = {1.0, 0.0, 0.0};
  rc.settings().cut_planes.push_back(cp);
  rc.add_volume(makeSphereVolume(ctx, 32), unshadedSettings(flatTF(1.f, 1.f, 1.f, 1.f)));

  const frame f = rc.render();

  // Columns inside the box projection (px 16..47), away from the x=0 seam.
  for (int px : {38, 42, 46})
    EXPECT_GT(int(pixel(f, px, 32)[3]), 0) << "kept column px=" << px;
  for (int px : {18, 22, 26})
    EXPECT_EQ(int(pixel(f, px, 32)[3]), 0) << "culled column px=" << px;
}

// ---------------------------------------------------------------------------
// 11. Two-sided lighting
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, TwoSidedLightingBrightensInwardGradients) {
  raycaster rc(ctx);
  rc.view() = perspectiveCam();
  rc.settings().steps = 128;
  light l;
  l.direction = {0.0, 0.0, 1.0}; // from +z, same side as the camera
  rc.settings().lights.push_back(l);

  // 1 - r decreases outward, so gradients point INWARD: front-facing samples
  // get N.L < 0 and shade black with one-sided lighting.
  volume_settings vs; // shaded = true by default
  vs.tf = flatTF(1.f, 1.f, 1.f, 0.5f);
  rc.add_volume(makeSphereVolume(ctx, 32), vs);

  rc.settings().two_sided_lighting = false;
  const std::uint64_t one_sided = sumRGB(rc.render());
  rc.settings().two_sided_lighting = true;
  const std::uint64_t two_sided = sumRGB(rc.render());

  EXPECT_GT(two_sided, one_sided);
}

// ---------------------------------------------------------------------------
// 12. Isosurface rendering
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, IsosurfaceCircle) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 256;
  light l;
  l.direction = {0.0, 0.0, 1.0};
  rc.settings().lights.push_back(l);

  // Iso-only: value 0.6 on the 1 - r field is the r = 0.4 sphere.
  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  isosurface iso;
  iso.value = 0.6;
  vs.isosurfaces.push_back(iso);
  rc.add_volume(makeSphereVolume(ctx, 32), vs);

  const frame f = rc.render();

  // Silhouette: hit within the projected circle minus a 2px band, miss
  // outside it plus 2px (2px = 0.0625 world units at parallel_scale 1).
  const double band = 2.0 * 2.0 / double(kRaster);
  int violations = 0;
  for (int py = 0; py < kRaster; ++py)
    for (int px = 0; px < kRaster; ++px) {
      const double wr = std::sqrt(orthoU(px) * orthoU(px) + orthoV(py) * orthoV(py));
      const bool hit = pixel(f, px, py)[3] > 0;
      if (wr <= 0.4 - band && !hit)
        ++violations;
      else if (wr >= 0.4 + band && hit)
        ++violations;
    }
  EXPECT_EQ(violations, 0);

  // The sphere bulges toward the eye: the center hit is nearer than a hit
  // close to the rim.
  const float d_center = depthAt(f, 32, 32);
  const float d_rim = depthAt(f, 41, 32); // world radius ~0.30 < 0.4
  ASSERT_TRUE(std::isfinite(d_center));
  ASSERT_TRUE(std::isfinite(d_rim));
  EXPECT_LT(d_center, d_rim);
}

// ---------------------------------------------------------------------------
// 13/14. Model transforms
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, TransformTranslationEquivalence) {
  transfer_function tf;
  tf.add(transfer_point{0.0, 1.f, 1.f, 1.f, 0.f});
  tf.add(transfer_point{1.0, 1.f, 1.f, 1.f, 1.f});

  // A: centered box moved by a model_transform translation of (0.3, 0, 0).
  raycaster rca(ctx);
  rca.view() = orthoCam();
  rca.settings().steps = 256;
  volume_settings vsa = unshadedSettings(tf);
  vsa.model_transform = translation(0.3, 0.0, 0.0);
  rca.add_volume(makeSphereVolume(ctx, 32), vsa);
  const frame fa = rca.render();

  // B: the same field built directly in the already-translated box.
  raycaster rcb(ctx);
  rcb.view() = orthoCam();
  rcb.settings().steps = 256;
  rcb.add_volume(makeSphereVolume(ctx, 32, cvc::bounding_box(-0.2, -0.5, -0.5, 0.8, 0.5, 0.5)),
                 unshadedSettings(tf));
  const frame fb = rcb.render();

  int channel_mismatches = 0;
  int mask_mismatches = 0;
  for (int py = 0; py < kRaster; ++py)
    for (int px = 0; px < kRaster; ++px) {
      const unsigned char *a = pixel(fa, px, py);
      const unsigned char *b = pixel(fb, px, py);
      int max_diff = 0;
      for (int c = 0; c < 4; ++c)
        max_diff = std::max(max_diff, std::abs(int(a[c]) - int(b[c])));
      if (max_diff > 2)
        ++channel_mismatches;
      if ((a[3] > 0) != (b[3] > 0))
        ++mask_mismatches;
    }
  const int budget = kRaster * kRaster / 100; // 1% of the raster
  EXPECT_LE(channel_mismatches, budget);
  EXPECT_LE(mask_mismatches, budget);
}

TEST_F(VolrenRenderTest, TransformScaleGrowsSilhouette) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 256;
  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  isosurface iso;
  iso.value = 0.6;
  vs.isosurfaces.push_back(iso);
  const std::size_t idx = rc.add_volume(makeSphereVolume(ctx, 32), vs);

  const int base_count = countAlphaPositive(rc.render());
  ASSERT_GT(base_count, 0);

  // Doubling the model scale doubles the projected radius, so the covered
  // pixel count roughly quadruples.
  rc.volume_config(idx).model_transform = uniformScale(2.0);
  const int scaled_count = countAlphaPositive(rc.render());

  EXPECT_GT(double(scaled_count), 3.0 * double(base_count));
  EXPECT_LT(double(scaled_count), 5.0 * double(base_count));
}

// ---------------------------------------------------------------------------
// 15. Multiple volumes
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, MultiVolumeDisjointPlacements) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 256;

  // Two copies of the box translated to x in [-1.4,-0.4] and [0.4,1.4]:
  // both partly visible with background in the gap (|x| < 0.4).
  volume_settings left = unshadedSettings(flatTF(1.f, 1.f, 1.f, 1.f));
  left.model_transform = translation(-0.9, 0.0, 0.0);
  volume_settings right = unshadedSettings(flatTF(1.f, 1.f, 1.f, 1.f));
  right.model_transform = translation(0.9, 0.0, 0.0);
  rc.add_volume(makeSphereVolume(ctx, 32), left);
  rc.add_volume(makeSphereVolume(ctx, 32), right);

  const frame f = rc.render();

  EXPECT_GT(int(pixel(f, 9, 32)[3]), 0);  // u ~ -0.70, inside the left box
  EXPECT_GT(int(pixel(f, 54, 32)[3]), 0); // u ~ +0.70, inside the right box
  for (int px : {30, 32, 34})             // |u| < 0.08, in the gap
    EXPECT_EQ(int(pixel(f, px, 32)[3]), 0) << "gap column px=" << px;
}

// The exact silhouette of a volume, to the pixel, on BOTH sides.  This is what
// pins the per-ray volume cull: the march visits a volume only inside the
// [t_enter, t_exit] window solved against a cull box, and that box has to
// contain grid_sampler::cell_index()'s acceptance region -- which is NOT the
// bounding box.  cell_index truncates toward zero, so a point up to one full
// voxel BELOW min still resolves into cell 0, while the idx <= dim-2 clamp cuts
// the high side off exactly at max.  A cull box set naively to the bounding box
// would clip that one low-side column and nothing else in this suite would
// notice.
//
// A second volume parked off-screen at x in [-2.5, -1.5] widens scene_bounds so
// the low-side column survives the SCENE slab test and the volume's own window
// is what decides.
TEST_F(VolrenRenderTest, SilhouetteKeepsTheLowSideVoxelOfCellSlack) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 256;

  constexpr unsigned kDim = 32;
  const double span = 1.0 / double(kDim - 1); // node-centered over [-0.5, 0.5]

  rc.add_volume(makeSphereVolume(ctx, kDim), unshadedSettings(flatTF(1.f, 1.f, 1.f, 1.f)));
  volume_settings offscreen = unshadedSettings(flatTF(1.f, 1.f, 1.f, 1.f));
  offscreen.model_transform = translation(-2.0, 0.0, 0.0);
  rc.add_volume(makeSphereVolume(ctx, kDim), offscreen);

  const frame f = rc.render();

  // Low side: the ray at u = -0.515625 is outside the box but within one voxel
  // of it, so cell 0 accepts it (with an extrapolating weight).
  ASSERT_GT(orthoU(15), -0.5 - span);
  ASSERT_LT(orthoU(15), -0.5);
  EXPECT_GT(int(pixel(f, 15, 32)[3]), 0)
      << "the cull box clipped the low-side voxel of cell_index() slack";
  // One column further out is beyond the slack on both accounts.
  ASSERT_LT(orthoU(14), -0.5 - span);
  EXPECT_EQ(int(pixel(f, 14, 32)[3]), 0);

  // High side: cut off AT max, with no matching slack.
  ASSERT_LT(orthoU(47), 0.5);
  EXPECT_GT(int(pixel(f, 47, 32)[3]), 0);
  ASSERT_GT(orthoU(48), 0.5);
  EXPECT_EQ(int(pixel(f, 48, 32)[3]), 0);

  // The off-screen volume really is off-screen: it only widened the scene box.
  EXPECT_EQ(int(pixel(f, 0, 32)[3]), 0);
}

// ---------------------------------------------------------------------------
// 16. Density window
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, WindowExcludesEverything) {
  raycaster rc(ctx);
  rc.view() = orthoCam();
  rc.settings().steps = 128;

  // The 1 - r field lies in ~[0.13, 1]; a [2, 3] window excludes all of it.
  volume_settings vs = unshadedSettings(flatTF(1.f, 1.f, 1.f, 1.f));
  vs.window_enabled = true;
  vs.window_min = 2.0;
  vs.window_max = 3.0;
  rc.add_volume(makeSphereVolume(ctx, 32), vs);

  const frame f = rc.render();

  EXPECT_EQ(countAlphaPositive(f), 0);
}

// ---------------------------------------------------------------------------
// 17. Scene bounds
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, SceneBoundsUnionTransformedBoxes) {
  raycaster rc(ctx);
  rc.add_volume(makeSphereVolume(ctx, 8)); // identity at [-0.5, 0.5]^3
  volume_settings vs;
  vs.model_transform = translation(0.3, 0.2, -0.1);
  rc.add_volume(makeSphereVolume(ctx, 8), vs); // [-0.2,0.8]x[-0.3,0.7]x[-0.6,0.4]

  const cvc::bounding_box b = rc.scene_bounds();

  EXPECT_NEAR(b.minx, -0.5, 1e-9);
  EXPECT_NEAR(b.maxx, 0.8, 1e-9);
  EXPECT_NEAR(b.miny, -0.5, 1e-9);
  EXPECT_NEAR(b.maxy, 0.7, 1e-9);
  EXPECT_NEAR(b.minz, -0.6, 1e-9);
  EXPECT_NEAR(b.maxz, 0.5, 1e-9);
}

// ---------------------------------------------------------------------------
// Regression tests from the port review
// ---------------------------------------------------------------------------

// A translucent isosurface must latch the depth map at its FIRST hit even
// when its opacity never pushes accumulated alpha past depth_alpha_threshold
// (the frame contract: first iso hit or first threshold crossing, whichever
// comes first).
TEST_F(VolrenRenderTest, TranslucentIsosurfaceLatchesDepthAtFirstHit) {
  raycaster rc(ctx);
  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  isosurface iso;
  iso.value = 0.6;    // r = 0.4 sphere on the 1-r field
  iso.opacity = 0.2f; // two crossings accumulate to 0.36 < default 0.5 threshold
  vs.isosurfaces.push_back(iso);
  rc.add_volume(makeSphereVolume(ctx, 32), vs);
  rc.view() = orthoCam();
  rc.settings().two_sided_lighting = true;

  const frame f = rc.render();

  // Center ray: front hit at eye distance 4 - 0.4 = 3.6.
  const unsigned char *px = pixel(f, 32, 32);
  EXPECT_GT(px[3], 0);
  ASSERT_TRUE(std::isfinite(depthAt(f, 32, 32)));
  EXPECT_NEAR(double(depthAt(f, 32, 32)), 3.6, 0.1);
}

// NaN voxels must not crash or corrupt neighboring pixels: NaN samples are
// transparent (LUT entry 0 of a ramp whose low end is alpha 0) and every
// output byte is defined.
TEST_F(VolrenRenderTest, NaNVoxelsRenderDefined) {
  cvc::volume vol = makeSphereVolume(ctx, 16);
  for (unsigned k = 6; k < 10; ++k)
    for (unsigned j = 6; j < 10; ++j)
      for (unsigned i = 6; i < 10; ++i)
        vol(i, j, k, std::numeric_limits<double>::quiet_NaN());
  vol.unsetMinMax();
  transfer_function tf;
  tf.add({0.0, 1.f, 1.f, 1.f, 0.f});
  tf.add({1.0, 1.f, 1.f, 1.f, 1.f});
  volume_settings vs = unshadedSettings(tf);
  vs.tf_auto_domain = false;
  raycaster rc(ctx);
  rc.add_volume(vol, vs);
  rc.view() = orthoCam();

  const frame f = rc.render(); // must not crash / hang
  const cvc::image &img = f.color;
  EXPECT_EQ(img.width(), kRaster);
  // Rays that miss the volume are still exact background.
  EXPECT_EQ(pixel(f, 0, 0)[3], 0);
}

// Non-finite camera poses and model transforms are rejected up front instead
// of producing an unbounded march.
TEST_F(VolrenRenderTest, NonFinitePoseOrTransformThrows) {
  raycaster rc(ctx);
  rc.add_volume(makeSphereVolume(ctx, 8));
  rc.view() = orthoCam();

  rc.view().eye[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(rc.render(), cvc::volren_error);
  rc.view() = orthoCam();

  rc.view().vfov_degrees = std::numeric_limits<double>::infinity();
  rc.view().projection = camera::projection_type::perspective;
  EXPECT_THROW(rc.render(), cvc::volren_error);
  rc.view() = orthoCam();

  rc.volume_config(0).model_transform.m[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(rc.render(), cvc::volren_error);
}

// Regression: the isosurface path enumerates EVERY cell the ray crosses
// (Amanatides-Woo DDA) instead of testing only sample-visited cells -- the
// legacy point-sampling skipped corner-clipped cells and peppered silhouettes
// with alpha-0 holes (worse at finer grids).  A closed sphere surface must
// have zero interior holes at a deliberately coarse step count.
TEST_F(VolrenRenderTest, IsosurfaceSilhouetteHasNoHoles) {
  cvc::volume vol(ctx, cvc::dimension(48, 48, 48), cvc::Float,
                  cvc::bounding_box(-1, -1, -1, 1, 1, 1));
  for (unsigned k = 0; k < 48; ++k)
    for (unsigned j = 0; j < 48; ++j)
      for (unsigned i = 0; i < 48; ++i) {
        const double x = -1 + i * vol.XSpan();
        const double y = -1 + j * vol.YSpan();
        const double z = -1 + k * vol.ZSpan();
        vol(i, j, k, std::sqrt(x * x + y * y + z * z) - 0.6);
      }
  volume_settings vs;
  vs.shaded = false;
  isosurface iso;
  iso.value = 0.0;
  iso.opacity = 1.0f;
  vs.isosurfaces.push_back(iso);
  raycaster rc(ctx);
  rc.add_volume(vol, vs);
  rc.view() = orthoCam();
  rc.settings().steps = 96; // coarse: the legacy sampling model rioted here
  rc.settings().two_sided_lighting = true;

  const frame f = rc.render();

  // Every pixel strictly inside the projected circle (radius 0.6 world =
  // 0.6/1.0 * 32px silhouette) must be opaque.
  const int r_px = int(0.6 * (kRaster / 2) * 0.85); // 15% safety band
  int holes = 0;
  for (int py = 0; py < kRaster; ++py)
    for (int px = 0; px < kRaster; ++px) {
      const int dx = px - kRaster / 2, dy = py - kRaster / 2;
      if (dx * dx + dy * dy < r_px * r_px && pixel(f, px, py)[3] < 250)
        ++holes;
    }
  EXPECT_EQ(holes, 0) << "isosurface silhouette has " << holes << " holes";
}

// Two raycasters rendering concurrently must not interfere: each defaults to
// its own thread pool (cvc::thread_pool allows only one in-flight
// parallel_for per pool).
TEST_F(VolrenRenderTest, ConcurrentRendersOnSeparateRaycastersComplete) {
  transfer_function tf;
  tf.add({-0.5, 0.2f, 0.4f, 0.8f, 0.1f});
  tf.add({0.5, 0.8f, 0.4f, 0.2f, 0.9f});
  volume_settings vs = unshadedSettings(tf);
  vs.tf_auto_domain = false;

  raycaster a(ctx), b(ctx);
  a.add_volume(makeLinearVolume(ctx, 16), vs);
  b.add_volume(makeLinearVolume(ctx, 16), vs);
  a.view() = orthoCam();
  b.view() = orthoCam();

  frame fa, fb;
  boost::thread ta([&] { fa = a.render(); });
  boost::thread tb([&] { fb = b.render(); });
  ta.join();
  tb.join();

  const cvc::image &ia = fa.color;
  const cvc::image &ib = fb.color;
  ASSERT_EQ(ia.size_bytes(), ib.size_bytes());
  EXPECT_EQ(std::memcmp(ia.data(), ib.data(), ia.size_bytes()), 0);
}

// ---------------------------------------------------------------------------
// 21. Supersampled anti-aliasing (render_settings::supersample)
// ---------------------------------------------------------------------------

// A scene whose alpha channel is exactly BINARY at one sample per pixel: a
// single fully opaque isosurface, no transfer function, so every ray either
// composites opacity 1.0 or composites nothing at all.  That makes "how many
// distinct alpha values exist" a direct measurement of edge resolution rather
// than a proxy for it.
void setupOpaqueSilhouette(cvc::app &ctx, raycaster &rc) {
  rc.view() = orthoCam();
  rc.settings().steps = 256;
  rc.settings().two_sided_lighting = true;
  light l;
  l.direction = {0.0, 0.0, 1.0};
  rc.settings().lights.push_back(l);

  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  isosurface iso;
  iso.value = 0.6; // the r = 0.4 sphere of the 1 - r field
  iso.opacity = 1.0f;
  vs.isosurfaces.push_back(iso);
  rc.add_volume(makeSphereVolume(ctx, 32), vs);
}

// Distinct alpha bytes in the frame, and how many pixels are PARTIALLY covered.
// With the scene above, one sample per pixel can only ever produce {0, 255}.
void alphaHistogram(const frame &f, int &distinct, int &partial) {
  bool seen[256] = {false};
  partial = 0;
  const cvc::image &img = f.color;
  const int n = img.width() * img.height();
  for (int i = 0; i < n; ++i) {
    const int a = int(img.data()[i * 4 + 3]);
    seen[a] = true;
    if (a > 0 && a < 255)
      ++partial;
  }
  distinct = 0;
  for (int a = 0; a < 256; ++a)
    if (seen[a])
      ++distinct;
}

// (b) The measurable definition of anti-aliasing: a high-contrast silhouette
// resolves into strictly MORE distinct alpha levels as the sub-sample grid gets
// finer.  n sub-samples per edge can express n^2 + 1 coverage levels, so the
// counts are bounded above as well -- an implementation that quietly jittered
// or double-counted would overshoot.
TEST_F(VolrenRenderTest, SupersampleAddsEdgeAlphaLevels) {
  raycaster rc(ctx);
  setupOpaqueSilhouette(ctx, rc);
  // A longer silhouette than the suite's 64^2 default: the number of levels a
  // grid can express is an upper bound the edge has to be long enough to reach,
  // and at 64^2 the circle is only ~50 pixels around, so 4x4's 17 levels would
  // be sampling-limited rather than filter-limited.
  rc.view().width = rc.view().height = 2 * kRaster;

  int distinct[5] = {0, 0, 0, 0, 0}, partial[5] = {0, 0, 0, 0, 0};
  for (int n : {1, 2, 3, 4}) {
    rc.settings().supersample = n;
    const frame f = rc.render();
    alphaHistogram(f, distinct[n], partial[n]);
    std::printf("[volren-aa] supersample=%d: %d distinct alpha values, %d partially covered "
                "pixels (max expressible %d)\n",
                n, distinct[n], partial[n], n * n + 1);
    std::fflush(stdout);
  }

  // One ray per pixel cannot express partial coverage AT ALL on this scene.
  EXPECT_EQ(distinct[1], 2) << "the reference scene is supposed to be alpha-binary at 1x";
  EXPECT_EQ(partial[1], 0);

  EXPECT_GT(distinct[2], distinct[1]) << "2x2 did not anti-alias the silhouette";
  EXPECT_GT(distinct[3], distinct[2]);
  EXPECT_GT(distinct[4], distinct[3]) << "4x4 did not refine 3x3's edge";
  EXPECT_LE(distinct[4], 4 * 4 + 1) << "more alpha levels than an n x n box filter can produce";
  EXPECT_GT(partial[4], partial[2]) << "a finer grid must widen the anti-aliased band";
}

// The sub-sample OFFSETS, pinned exactly rather than by eyeball.  A regular
// n x n grid at ((i+0.5)/n, (j+0.5)/n) inside pixel px of a W-wide raster lands
// on exactly the pixel centers of an (n*W)-wide raster:
//   ((px + (i+0.5)/n) / W)  ==  ((n*px + i + 0.5) / (n*W))
// so an n-supersampled render must equal the n x n box downsample of the finer
// single-sampled one.  This is the whole reason to prefer a regular grid over a
// rotated or low-discrepancy one here: the placement stays checkable.
//
// COLOR agrees to 1 LSB, not exactly, because the reference quantizes each
// sub-sample to u8 before this test can average them while the renderer
// averages in float.  DEPTH agrees EXACTLY: both sides take the minimum of the
// same four f32 values.
TEST_F(VolrenRenderTest, SupersampleGridLandsOnAFinerRastersPixelCenters) {
  constexpr int kN = 2;

  raycaster rc(ctx);
  setupOpaqueSilhouette(ctx, rc);
  rc.settings().supersample = kN;
  const frame super = rc.render();

  // Same scene, same camera, one sample per pixel, kN times the raster.  Only
  // width/height change, so the basis, fov, scene bounds and step size are
  // untouched.
  raycaster fine(ctx);
  setupOpaqueSilhouette(ctx, fine);
  fine.view().width = kRaster * kN;
  fine.view().height = kRaster * kN;
  const frame reference = fine.render();

  int worst_channel = 0, depth_mismatches = 0;
  for (int py = 0; py < kRaster; ++py)
    for (int px = 0; px < kRaster; ++px) {
      int sum[4] = {0, 0, 0, 0};
      float nearest = std::numeric_limits<float>::infinity();
      for (int sj = 0; sj < kN; ++sj)
        for (int si = 0; si < kN; ++si) {
          const unsigned char *p = pixel(reference, px * kN + si, py * kN + sj);
          for (int c = 0; c < 4; ++c)
            sum[c] += int(p[c]);
          const float d = depthAt(reference, px * kN + si, py * kN + sj);
          if (d < nearest)
            nearest = d;
        }
      const unsigned char *got = pixel(super, px, py);
      for (int c = 0; c < 4; ++c) {
        const int want = (sum[c] + (kN * kN) / 2) / (kN * kN); // rounded mean of the block
        worst_channel = std::max(worst_channel, std::abs(int(got[c]) - want));
      }
      if (depthAt(super, px, py) != nearest)
        ++depth_mismatches;
    }

  std::printf("[volren-aa] %dx%d grid vs a %dx raster: worst channel delta %d LSB, %d depth "
              "mismatches\n",
              kN, kN, kN, worst_channel, depth_mismatches);
  std::fflush(stdout);
  EXPECT_LE(worst_channel, 1) << "sub-samples are not on the finer raster's pixel centers";
  EXPECT_EQ(depth_mismatches, 0) << "the depth resolve is not the nearest sub-sample";
}

// (d) Determinism survives supersampling: the resolve happens entirely inside
// one pixel's thread, so tile scheduling still cannot reorder anything.
TEST_F(VolrenRenderTest, SupersampledRenderIsThreadCountInvariant) {
  raycaster rc(ctx);
  setupOpaqueSilhouette(ctx, rc);
  // A shaded transfer function on top of the isosurface so the gradient cache
  // (which IS shared across a tile's sub-samples) is exercised too.
  transfer_function tf;
  tf.add(transfer_point{0.0, 0.9f, 0.6f, 0.3f, 0.f});
  tf.add(transfer_point{1.0, 0.9f, 0.6f, 0.3f, 0.6f});
  rc.volume_config(0).shaded = true;
  rc.volume_config(0).tf = tf;
  rc.settings().supersample = 3;

  rc.settings().threads = 1;
  const frame serial = rc.render();
  rc.settings().threads = 0;
  const frame pooled1 = rc.render();
  const frame pooled2 = rc.render();

  const std::size_t color_bytes = std::size_t(kRaster) * kRaster * 4;
  const std::size_t depth_bytes = std::size_t(kRaster) * kRaster * sizeof(float);
  EXPECT_EQ(std::memcmp(serial.color.data(), pooled1.color.data(), color_bytes), 0);
  EXPECT_EQ(std::memcmp(serial.depth.data(), pooled1.depth.data(), depth_bytes), 0);
  EXPECT_EQ(std::memcmp(pooled1.color.data(), pooled2.color.data(), color_bytes), 0);
  EXPECT_EQ(std::memcmp(pooled1.depth.data(), pooled2.depth.data(), depth_bytes), 0);
}

// The resolve averages STRAIGHT (background-over-blended) RGBA, never
// premultiplied.  On a NON-BLACK background that distinction is visible: a
// half-covered edge pixel must land halfway between the surface color and the
// background, and a fully uncovered pixel must stay exactly the background.
// Premultiplying before the average would drag both toward black.
TEST_F(VolrenRenderTest, SupersampledEdgesBlendTowardTheBackgroundNotBlack) {
  raycaster rc(ctx);
  setupOpaqueSilhouette(ctx, rc);
  rc.settings().background = {1.f, 1.f, 1.f}; // white: premultiplying reads as black
  rc.settings().supersample = 4;

  const frame f = rc.render();

  // Every partially covered pixel is a convex combination of an opaque
  // (necessarily darker: the surface is lit, never brighter than white) sample
  // and the white background, so its RGB can never fall below what its own
  // coverage allows: rgb >= 255 - alpha, i.e. the pixel is at least as bright
  // as "alpha of pure black over white".
  int violations = 0, partial = 0;
  const cvc::image &img = f.color;
  for (int i = 0; i < img.width() * img.height(); ++i) {
    const int a = int(img.data()[i * 4 + 3]);
    if (a == 0) {
      // No coverage at all: pure background, exactly.
      for (int c = 0; c < 3; ++c)
        if (int(img.data()[i * 4 + c]) != 255)
          ++violations;
      continue;
    }
    if (a == 255)
      continue;
    ++partial;
    for (int c = 0; c < 3; ++c)
      if (int(img.data()[i * 4 + c]) < 255 - a - 1) // 1 LSB of resolve rounding
        ++violations;
  }
  ASSERT_GT(partial, 0) << "no partially covered pixels: the test scene has no edge";
  EXPECT_EQ(violations, 0) << "partially covered pixels are darker than their coverage allows -- "
                              "the resolve premultiplied the background away";
}

// ---------------------------------------------------------------------------
// 8. Volumetric shadows (shadow.h)
// ---------------------------------------------------------------------------
// The scene these tests are built on, and why it is the one that makes the
// feature falsifiable rather than merely exercised:
//
//   * RECEIVER: a flat plate.  makeLinearVolume's field is the local z
//     coordinate, so its 0-isosurface is the plane z = 0 and its normal is
//     exactly +z everywhere -- no curvature to smear the geometry check.
//   * OCCLUDER: a small ball, a SEPARATE volume, parked well outside the
//     camera's ortho window so it cannot draw over its own shadow.
//   * LIGHT at 45 degrees in the x-z plane, with the ball placed on that exact
//     ray, so the shadow lands at a world position computable by hand.
//
// A 45-degree light projects a sphere of radius R onto a horizontal plane as
// an ELLIPSE with semi-axes R across the light and R*sqrt(2) along it.  Every
// number below is that prediction, not a recorded output.

constexpr int kShadowRaster = 128;
constexpr double kShadowScale = 0.6; // ortho half-height; the plate spans +/-0.5
constexpr double kBallRadius = 0.2;
constexpr double kBallHeight = 1.6; // ball center at (h, 0, h) on the light ray

// val = distance from the box center, so the gradient points OUTWARD and an
// isosurface is lit on the side facing the light (makeSphereVolume's 1 - r
// points inward, which would light the far side).
cvc::volume makeBallVolume(cvc::app &ctx, unsigned n, double half) {
  cvc::volume vol(ctx, cvc::dimension(n, n, n), cvc::Float,
                  cvc::bounding_box(-half, -half, -half, half, half, half));
  for (unsigned k = 0; k < n; ++k)
    for (unsigned j = 0; j < n; ++j)
      for (unsigned i = 0; i < n; ++i) {
        const double x = -half + double(i) * vol.XSpan();
        const double y = -half + double(j) * vol.YSpan();
        const double z = -half + double(k) * vol.ZSpan();
        vol(i, j, k, std::sqrt(x * x + y * y + z * z));
      }
  return vol;
}

camera plateCam(int raster = kShadowRaster, double scale = kShadowScale) {
  camera c;
  c.eye = {0.0, 0.0, 4.0};
  c.focal = {0.0, 0.0, 0.0};
  c.up = {0.0, 1.0, 0.0};
  c.projection = camera::projection_type::orthographic;
  c.parallel_scale = scale;
  c.width = c.height = raster;
  return c;
}

volume_settings opaqueIso(double value, float opacity = 1.0f) {
  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  isosurface s;
  s.value = value;
  s.opacity = opacity;
  s.color = {0.9f, 0.9f, 0.9f};
  vs.isosurfaces.push_back(s);
  return vs;
}

// Unit vector toward a light at `elevation_deg` above the +x axis in the x-z
// plane (90 degrees is straight overhead).
std::array<double, 3> lightTowardXZ(double elevation_deg) {
  const double a = elevation_deg * M_PI / 180.0;
  return {std::cos(a), 0.0, std::sin(a)};
}

render_settings shadowSceneSettings(double elevation_deg = 45.0) {
  render_settings rs;
  light key;
  key.color = {1.f, 1.f, 1.f};
  key.direction = lightTowardXZ(elevation_deg);
  rs.lights = {key};
  rs.ambient = 0.15f; // a floor, so a shadowed pixel is dark but not black
  rs.steps = 256;
  return rs;
}

int luminance(const frame &f, int x, int y) {
  const unsigned char *p = pixel(f, x, y);
  return int(p[0]) + int(p[1]) + int(p[2]);
}

bool framesIdentical(const frame &a, const frame &b) {
  const cvc::image &ca = a.color, &cb = b.color;
  const cvc::image &da = a.depth, &db = b.depth;
  const std::size_t cn = std::size_t(ca.width()) * ca.height() * 4;
  const std::size_t dn = std::size_t(da.width()) * da.height() * sizeof(float);
  return ca.width() == cb.width() && ca.height() == cb.height() &&
         std::memcmp(ca.data(), cb.data(), cn) == 0 && std::memcmp(da.data(), db.data(), dn) == 0;
}

// Build the plate-plus-ball scene on `rc`.  `ball_opacity` < the shadow
// settings' min_occluder_opacity makes the ball a non-caster.
void buildPlateAndBall(raycaster &rc, const cvc::volume &plate, const cvc::volume &ball,
                       bool with_ball, float ball_opacity = 1.0f) {
  rc.add_volume(plate, opaqueIso(0.0));
  if (with_ball) {
    volume_settings bs = opaqueIso(kBallRadius, ball_opacity);
    bs.isosurfaces[0].color = {0.9f, 0.4f, 0.4f};
    bs.model_transform = translation(kBallHeight, 0.0, kBallHeight);
    rc.add_volume(ball, bs);
  }
  rc.view() = plateCam();
  rc.settings() = shadowSceneSettings();
}

// ---- 8.1 The no-op contract, pinned from three directions -----------------

TEST_F(VolrenRenderTest, ShadowsOffAndStrengthZeroAreIdentical) {
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  raycaster off(ctx), zero(ctx), dark_off(ctx), dark_on(ctx);
  buildPlateAndBall(off, plate, ball, true);
  buildPlateAndBall(zero, plate, ball, true);
  buildPlateAndBall(dark_off, plate, ball, true);
  buildPlateAndBall(dark_on, plate, ball, true);

  render_settings rs = zero.settings();
  rs.shadows.enabled = true;
  rs.shadows.strength = 0.f;
  zero.settings() = rs;

  // A scene with no lights at all: shadows have nothing to attenuate, so
  // enabling them must not move a single byte either.
  rs = dark_off.settings();
  rs.lights.clear();
  dark_off.settings() = rs;
  rs.shadows.enabled = true;
  dark_on.settings() = rs;

  const frame a = off.render();
  EXPECT_TRUE(framesIdentical(a, zero.render()))
      << "strength 0 must be a byte-identical no-op, not merely a small change";
  EXPECT_TRUE(framesIdentical(dark_off.render(), dark_on.render()))
      << "shadows with no lights must be a byte-identical no-op";
  EXPECT_EQ(dark_on.shadow_map_count(), 0u);
}

// ---- 8.2 The money test: a volume between the light and another volume -----

TEST_F(VolrenRenderTest, OccluderVolumeDarkensReceiverAtThePredictedPlace) {
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  raycaster off(ctx), on(ctx);
  buildPlateAndBall(off, plate, ball, true);
  buildPlateAndBall(on, plate, ball, true);
  render_settings rs = on.settings();
  rs.shadows.enabled = true;
  on.settings() = rs;

  const frame a = off.render();
  const frame b = on.render();
  ASSERT_EQ(on.shadow_map_count(), 1u);

  // World <-> pixel for plateCam(): world x = u * scale, world y = v * scale.
  const double px_per_world = 0.5 * double(kShadowRaster) / kShadowScale;
  const double center_px = 0.5 * double(kShadowRaster) - 0.5;
  // The ball sits at (h, 0, h) and the light comes from (h, 0, h) normalized,
  // so light travels along (-1, 0, -1)/sqrt(2): from the ball center it lands
  // on z = 0 at x = h - h = 0.  The ellipse is R across (y) and R*sqrt(2)
  // along the light (x).
  const double semi_y_px = kBallRadius * px_per_world;
  const double semi_x_px = kBallRadius * std::sqrt(2.0) * px_per_world;

  int darkened = 0, alpha_changes = 0;
  int minx = kShadowRaster, maxx = -1, miny = kShadowRaster, maxy = -1;
  // Mean RGB inside the predicted shadow ellipse, before and after.
  int core_px = 0;
  double core_before = 0.0, core_after = 0.0;
  for (int y = 0; y < kShadowRaster; ++y)
    for (int x = 0; x < kShadowRaster; ++x) {
      if (pixel(a, x, y)[3] != pixel(b, x, y)[3])
        ++alpha_changes;
      if (pixel(a, x, y)[3] == 0)
        continue;
      const int la = luminance(a, x, y), lb = luminance(b, x, y);
      // Half the predicted ellipse: comfortably inside the umbra, so this is a
      // prediction about brightness, not about the silhouette.
      const double dx = (double(x) - center_px) / (0.5 * semi_x_px);
      const double dy = (double(y) - center_px) / (0.5 * semi_y_px);
      if (dx * dx + dy * dy <= 1.0) {
        ++core_px;
        core_before += la;
        core_after += lb;
      }
      if (lb < la - la / 10) {
        ++darkened;
        minx = std::min(minx, x);
        maxx = std::max(maxx, x);
        miny = std::min(miny, y);
        maxy = std::max(maxy, y);
      }
    }

  // Shadows change COLOR, never opacity.  This is the sharp invariant: it
  // catches any accidental coupling into the compositing path.
  EXPECT_EQ(alpha_changes, 0) << "the alpha channel moved -- shadows leaked into compositing";

  ASSERT_GT(darkened, 0) << "nothing darkened: the occluder cast no shadow at all";
  ASSERT_GT(core_px, 200);
  // With ambient 0.15, a fully shadowed pixel keeps only the ambient term, so
  // the umbra must fall to well under half its lit brightness.
  EXPECT_LT(core_after / core_px, 0.5 * core_before / core_px)
      << "the occluded region did not get measurably darker (" << core_before / core_px << " -> "
      << core_after / core_px << ")";

  // Where it landed, against the hand-computed ellipse (+/- 3 px of MC
  // silhouette and the 10% darkening threshold).
  const double cx = 0.5 * (minx + maxx), cy = 0.5 * (miny + maxy);
  EXPECT_NEAR(cx, center_px, 3.0) << "the shadow is not centered where the light geometry puts it";
  EXPECT_NEAR(cy, center_px, 3.0);
  EXPECT_NEAR(0.5 * (maxx - minx), semi_x_px, 3.0)
      << "the shadow is not stretched along the light by sqrt(2)";
  EXPECT_NEAR(0.5 * (maxy - miny), semi_y_px, 3.0)
      << "the shadow is the wrong size across the light";

  // The UNOCCLUDED reference region -- the plate outside the ellipse, with a
  // 6 px margin -- must be BITWISE unchanged.  This is what says "I darkened
  // the shadow", not "I darkened everything".
  int reference_px = 0, reference_changed = 0;
  for (int y = 0; y < kShadowRaster; ++y)
    for (int x = 0; x < kShadowRaster; ++x) {
      if (pixel(a, x, y)[3] == 0)
        continue;
      const double dx = (double(x) - center_px) / (semi_x_px + 6.0);
      const double dy = (double(y) - center_px) / (semi_y_px + 6.0);
      if (dx * dx + dy * dy <= 1.0)
        continue;
      ++reference_px;
      if (std::memcmp(pixel(a, x, y), pixel(b, x, y), 4) != 0)
        ++reference_changed;
    }
  ASSERT_GT(reference_px, 1000) << "the reference region is too small to mean anything";
  EXPECT_EQ(reference_changed, 0)
      << reference_changed << " of " << reference_px
      << " unoccluded pixels changed -- the shadow is not local to the occluder";
}

// ---- 8.3 The shadow moves where the light says it should ------------------

TEST_F(VolrenRenderTest, ShadowCentroidTracksTheLightDirection) {
  // A larger plate so both shadows land well inside it, and the ball directly
  // overhead so a light tilted +/-45 degrees throws the shadow symmetrically
  // to either side.  A sign error in `right`, a v-flip, or a forward/back
  // confusion all move a centroid and fail here immediately.
  const unsigned n = 48;
  cvc::volume plate(ctx, cvc::dimension(n, n, n), cvc::Float,
                    cvc::bounding_box(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0));
  for (unsigned k = 0; k < n; ++k)
    for (unsigned j = 0; j < n; ++j)
      for (unsigned i = 0; i < n; ++i)
        plate(i, j, k, -1.0 + double(k) * plate.ZSpan());
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  constexpr double kHeight = 0.5;
  constexpr double kScale = 1.1;
  const double px_per_world = 0.5 * double(kShadowRaster) / kScale;
  const double center_px = 0.5 * double(kShadowRaster) - 0.5;

  const auto centroid = [&](double elevation_deg, double &cx, double &cy) {
    raycaster off(ctx), on(ctx);
    for (raycaster *rc : {&off, &on}) {
      rc->add_volume(plate, opaqueIso(0.0));
      volume_settings bs = opaqueIso(kBallRadius);
      bs.model_transform = translation(0.0, 0.0, kHeight);
      rc->add_volume(ball, bs);
      rc->view() = plateCam(kShadowRaster, kScale);
      rc->settings() = shadowSceneSettings(elevation_deg);
    }
    render_settings rs = on.settings();
    rs.shadows.enabled = true;
    on.settings() = rs;

    const frame a = off.render(), b = on.render();
    double sx = 0.0, sy = 0.0;
    int count = 0;
    for (int y = 0; y < kShadowRaster; ++y)
      for (int x = 0; x < kShadowRaster; ++x) {
        // Plate pixels only: the ball is between the camera and the plate, so
        // its own surface would otherwise pull the centroid toward it.  The
        // plate sits at z = 0 under an ortho eye at z = 4, so its eye-space
        // depth is exactly 4; the ball's is below 3.4.
        if (pixel(a, x, y)[3] == 0 || depthAt(a, x, y) < 3.9f)
          continue;
        const int la = luminance(a, x, y), lb = luminance(b, x, y);
        if (lb >= la - la / 10)
          continue;
        sx += x;
        sy += y;
        ++count;
      }
    ASSERT_GT(count, 100) << "no shadow found on the plate at elevation " << elevation_deg;
    cx = sx / count;
    cy = sy / count;
  };

  double cx45 = 0, cy45 = 0, cx135 = 0, cy135 = 0;
  centroid(45.0, cx45, cy45);    // light toward (+x, +z): shadow shifts to -x
  centroid(135.0, cx135, cy135); // light toward (-x, +z): shadow shifts to +x

  // A light at 45 degrees puts the shadow of a body at height h exactly h to
  // the far side: |dx| = h * cot(45) = h.
  EXPECT_NEAR(cx45, center_px - kHeight * px_per_world, 3.0);
  EXPECT_NEAR(cx135, center_px + kHeight * px_per_world, 3.0);
  // Neither light has a y component, so neither shadow may move in y.  This is
  // the assertion an axis swap in the light-view basis cannot survive.
  EXPECT_NEAR(cy45, center_px, 2.0);
  EXPECT_NEAR(cy135, center_px, 2.0);
}

// ---- 8.4 Self-shadowing: the bias, pinned ---------------------------------

TEST_F(VolrenRenderTest, SelfShadowingHasNoAcne) {
  // A smooth sphere, lit obliquely, shadowing only itself.  Every receiver
  // point's light ray hits THAT SAME SURFACE at essentially the same depth, so
  // the comparison is a coin flip unless the bias is right -- the classic
  // shimmering-speckle failure, worst at grazing incidence.  Run at two
  // incidences and for both an exact-MC isosurface and a marched shaded TF,
  // because the two latch their depth by completely different mechanisms: an
  // exact ray/MC intersection versus the first march SAMPLE that pushed
  // accumulated alpha past depth_alpha_threshold.  The second is quantized to
  // the CELL, which is what detail::shadow_bias's latch_quantum is sized on.
  //
  // The TF here is a SURFACE-LIKE shell about one cell thick.  A genuinely
  // thick translucent medium is a different problem and no bias fixes it: the
  // map records the light direction's 50%-transmittance surface, which really
  // is a different surface from the view direction's, and the gap grows with
  // thickness and obliquity.  That is what deep/opacity shadow maps are for
  // (docs/VOLREN_API.md, future work); `bias_scale` is the knob until then.
  const cvc::volume sphere = makeBallVolume(ctx, 64, 0.5);
  transfer_function tf;
  tf.add(transfer_point{0.0, 0.9f, 0.9f, 0.9f, 0.0f});
  tf.add(transfer_point{0.386, 0.9f, 0.9f, 0.9f, 0.0f});
  tf.add(transfer_point{0.400, 0.9f, 0.9f, 0.9f, 0.9f});
  tf.add(transfer_point{0.414, 0.9f, 0.9f, 0.9f, 0.0f});
  tf.add(transfer_point{1.0, 0.9f, 0.9f, 0.9f, 0.0f});

  for (int mode = 0; mode < 2; ++mode) {
    for (const double elevation : {90.0, 30.0}) {
      raycaster off(ctx), on(ctx);
      for (raycaster *rc : {&off, &on}) {
        volume_settings vs = opaqueIso(0.4);
        if (mode == 1) {
          vs = volume_settings();
          vs.shaded = true;
          vs.tf = tf;
          vs.tf_auto_domain = false;
        }
        rc->add_volume(sphere, vs);
        rc->view() = plateCam();
        rc->settings() = shadowSceneSettings(elevation);
      }
      render_settings rs = on.settings();
      rs.shadows.enabled = true;
      on.settings() = rs;

      const frame a = off.render(), b = on.render();
      int lit = 0, acne = 0;
      for (int y = 0; y < kShadowRaster; ++y)
        for (int x = 0; x < kShadowRaster; ++x) {
          if (pixel(a, x, y)[3] == 0)
            continue;
          const int la = luminance(a, x, y);
          // Only the well-lit cap can show acne: an ambient-only pixel has no
          // diffuse or specular term for a shadow to remove.
          if (la <= 3 * 60)
            continue;
          ++lit;
          if (luminance(b, x, y) < la - la / 10)
            ++acne;
        }
      ASSERT_GT(lit, 500) << "mode " << mode << " elevation " << elevation
                          << ": the scene has no well-lit region to test";
      EXPECT_LE(acne, lit / 100)
          << "mode " << mode << " elevation " << elevation << ": " << acne << " of " << lit
          << " lit pixels self-shadowed -- the bias is wrong, do not loosen this tolerance";
    }
  }
}

// ---- 8.5 The knobs --------------------------------------------------------

TEST_F(VolrenRenderTest, ShadowStrengthScalesMonotonically) {
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  const auto renderAt = [&](float strength) {
    raycaster rc(ctx);
    buildPlateAndBall(rc, plate, ball, true);
    render_settings rs = rc.settings();
    rs.shadows.enabled = true;
    rs.shadows.strength = strength;
    rc.settings() = rs;
    return rc.render();
  };

  const frame full = renderAt(1.0f);
  const frame half = renderAt(0.5f);
  const frame none = renderAt(0.0f);

  // Measure on the pixels the full-strength render actually shadowed.
  int n = 0;
  double s_full = 0, s_half = 0, s_none = 0;
  for (int y = 0; y < kShadowRaster; ++y)
    for (int x = 0; x < kShadowRaster; ++x) {
      if (pixel(none, x, y)[3] == 0)
        continue;
      const int l0 = luminance(none, x, y);
      if (luminance(full, x, y) >= l0 - l0 / 10)
        continue;
      ++n;
      s_none += l0;
      s_half += luminance(half, x, y);
      s_full += luminance(full, x, y);
    }
  ASSERT_GT(n, 100);
  EXPECT_LT(s_half, s_none) << "strength 0.5 must be darker than strength 0";
  EXPECT_LT(s_full, s_half) << "strength 1.0 must be darker than strength 0.5";
}

TEST_F(VolrenRenderTest, TranslucentIsosurfaceDoesNotCast) {
  // The volren_bunny --shell regression.  The light pass latches its depth on
  // the FIRST isosurface hit whatever that surface's opacity, so without the
  // min_occluder_opacity filter a decorative 0.16-opacity shell becomes the
  // occluder for everything it wraps.
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  const auto renderWith = [&](float ball_opacity, float min_occluder) {
    raycaster rc(ctx);
    buildPlateAndBall(rc, plate, ball, true, ball_opacity);
    render_settings rs = rc.settings();
    rs.shadows.enabled = true;
    rs.shadows.min_occluder_opacity = min_occluder;
    rc.settings() = rs;
    return rc.render();
  };

  raycaster base(ctx);
  buildPlateAndBall(base, plate, ball, true, 0.16f);
  const frame unshadowed = base.render();

  const auto darkCount = [&](const frame &f) {
    int n = 0;
    for (int y = 0; y < kShadowRaster; ++y)
      for (int x = 0; x < kShadowRaster; ++x) {
        if (pixel(unshadowed, x, y)[3] == 0)
          continue;
        const int l0 = luminance(unshadowed, x, y);
        if (luminance(f, x, y) < l0 - l0 / 10)
          ++n;
      }
    return n;
  };

  EXPECT_EQ(darkCount(renderWith(0.16f, cvc::volren::defaults::shadow_min_occluder_opacity)), 0)
      << "a 0.16-opacity surface cast a shadow at the 0.5 default";
  // And the knob is actually wired, not merely defaulted.
  EXPECT_GT(darkCount(renderWith(0.16f, 0.1f)), 100)
      << "lowering min_occluder_opacity below the surface's opacity did not make it cast";
}

TEST_F(VolrenRenderTest, CutPlanesRemoveShadowsToo) {
  // The light pass must inherit the scene's cut planes: a plane that removes
  // the occluder has to remove its shadow with it.
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  raycaster on(ctx), cut(ctx);
  buildPlateAndBall(on, plate, ball, true);
  buildPlateAndBall(cut, plate, ball, true);
  render_settings rs = on.settings();
  rs.shadows.enabled = true;
  on.settings() = rs;
  // Keep x <= 1.0: the plate (x in [-0.5, 0.5]) survives, the ball
  // (x in [1.35, 1.85]) is culled.
  rs.cut_planes.push_back(cut_plane{{1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}});
  cut.settings() = rs;

  raycaster reference(ctx);
  buildPlateAndBall(reference, plate, ball, false);
  const frame plate_only = reference.render();

  const frame shadowed = on.render();
  const frame cut_away = cut.render();

  int darkened_without_cut = 0, darkened_with_cut = 0;
  for (int y = 0; y < kShadowRaster; ++y)
    for (int x = 0; x < kShadowRaster; ++x) {
      if (pixel(plate_only, x, y)[3] == 0)
        continue;
      const int l0 = luminance(plate_only, x, y);
      if (luminance(shadowed, x, y) < l0 - l0 / 10)
        ++darkened_without_cut;
      if (luminance(cut_away, x, y) < l0 - l0 / 10)
        ++darkened_with_cut;
    }
  EXPECT_GT(darkened_without_cut, 100);
  EXPECT_EQ(darkened_with_cut, 0) << "the cut plane removed the occluder but not its shadow";
}

// ---- 8.6 Contracts and edges ---------------------------------------------

TEST_F(VolrenRenderTest, UnshadedSamplesAreNotShadowed) {
  // volume_settings::unshaded is defined as "TF readout with no lighting
  // model", so there is no light term to attenuate.  Darkening it would invent
  // a lighting model for the one mode whose contract is that it has none.
  // Pinned here so nobody "fixes" it later.
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  raycaster off(ctx), on(ctx);
  for (raycaster *rc : {&off, &on}) {
    rc->add_volume(plate, unshadedSettings(flatTF(0.8f, 0.8f, 0.8f, 0.3f)));
    volume_settings bs = opaqueIso(kBallRadius);
    bs.model_transform = translation(kBallHeight, 0.0, kBallHeight);
    rc->add_volume(ball, bs);
    rc->view() = plateCam();
    rc->settings() = shadowSceneSettings();
  }
  render_settings rs = on.settings();
  rs.shadows.enabled = true;
  on.settings() = rs;

  const frame a = off.render();
  const frame b = on.render();
  ASSERT_EQ(on.shadow_map_count(), 1u) << "the light pass did not run at all";
  EXPECT_TRUE(framesIdentical(a, b))
      << "an unshaded volume was darkened -- unshaded has no lighting model to attenuate";
}

TEST_F(VolrenRenderTest, DegenerateLightDirectionIsIgnored) {
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  for (const std::array<double, 3> dir :
       {std::array<double, 3>{0.0, 0.0, 0.0},
        std::array<double, 3>{std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0},
        std::array<double, 3>{std::numeric_limits<double>::infinity(), 0.0, 1.0}}) {
    raycaster off(ctx), on(ctx);
    for (raycaster *rc : {&off, &on}) {
      buildPlateAndBall(*rc, plate, ball, true);
      render_settings rs = rc->settings();
      rs.lights[0].direction = dir;
      rc->settings() = rs;
    }
    render_settings rs = on.settings();
    rs.shadows.enabled = true;
    on.settings() = rs;

    frame a, b;
    ASSERT_NO_THROW(a = off.render());
    ASSERT_NO_THROW(b = on.render()) << "a degenerate light direction must not throw";
    EXPECT_EQ(on.shadow_map_count(), 0u) << "a degenerate light must build no map";
    EXPECT_TRUE(framesIdentical(a, b));
  }
}

TEST_F(VolrenRenderTest, TooManyShadowLightsThrows) {
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  raycaster rc(ctx);
  rc.add_volume(plate, opaqueIso(0.0));
  rc.view() = plateCam();

  render_settings rs = shadowSceneSettings();
  rs.lights.clear();
  for (int i = 0; i < cvc::volren::limits::max_shadow_maps + 1; ++i) {
    light l;
    l.direction = lightTowardXZ(30.0 + 10.0 * i);
    rs.lights.push_back(l);
  }
  rs.shadows.enabled = true;
  rc.settings() = rs;
  // Refused loudly, not by silently dropping a light -- and refused for BOTH
  // backends, because the renderer cap and the device cap are ONE contract
  // (static_assert'd equal in raycaster_cuda.h), so the CPU path never accepts
  // a scene the kernel could not represent.
  EXPECT_THROW(rc.render(), cvc::volren_error);

  // An index naming a light that does not exist.
  rs.lights.resize(2);
  rs.shadows.lights = {0, 2};
  rc.settings() = rs;
  EXPECT_THROW(rc.render(), cvc::volren_error);

  // ... and the valid subset renders.
  rs.shadows.lights = {1};
  rc.settings() = rs;
  ASSERT_NO_THROW(rc.render());
  EXPECT_EQ(rc.shadow_map_count(), 1u);
  EXPECT_EQ(rc.shadow_map_view(0).light_index, 1);
  EXPECT_THROW(rc.shadow_map_view(1), cvc::volren_error);
  EXPECT_THROW(rc.shadow_map_depth(1), cvc::volren_error);
}

TEST_F(VolrenRenderTest, ShadowMapIsCachedAcrossCameraMovesAndRebuiltOnSceneChanges) {
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  raycaster rc(ctx);
  buildPlateAndBall(rc, plate, ball, true);
  render_settings rs = rc.settings();
  rs.shadows.enabled = true;
  rs.shadows.resolution = 128; // small: this test is about identity, not quality
  rc.settings() = rs;

  rc.render();
  ASSERT_EQ(rc.shadow_map_count(), 1u);
  const cvc::image first = rc.shadow_map_depth(0);
  const shadow_view first_view = rc.shadow_map_view(0);
  const std::size_t bytes = std::size_t(first.width()) * first.height() * sizeof(float);
  std::vector<unsigned char> first_bytes(first.data(), first.data() + bytes);

  // A camera move must NOT rebuild: that is the whole reason shadows are
  // affordable while the user orbits.
  camera moved = rc.view();
  moved.eye = {1.0, 1.0, 4.0};
  moved.focal = {0.2, 0.0, 0.0};
  rc.view() = moved;
  rc.render();
  ASSERT_EQ(rc.shadow_map_count(), 1u);
  const cvc::image after_move = rc.shadow_map_depth(0);
  EXPECT_EQ(std::memcmp(after_move.data(), first_bytes.data(), bytes), 0)
      << "moving the camera rebuilt the shadow map";
  EXPECT_EQ(first_view.eye, rc.shadow_map_view(0).eye);
  EXPECT_DOUBLE_EQ(first_view.parallel_scale, rc.shadow_map_view(0).parallel_scale);

  // Moving a volume MUST rebuild.
  rc.volume_config(1).model_transform = translation(kBallHeight, 0.3, kBallHeight);
  rc.render();
  const cvc::image after_move_volume = rc.shadow_map_depth(0);
  EXPECT_NE(std::memcmp(after_move_volume.data(), first_bytes.data(), bytes), 0)
      << "moving the occluder did not rebuild the shadow map";
  std::vector<unsigned char> moved_bytes(after_move_volume.data(),
                                         after_move_volume.data() + bytes);

  // An explicit invalidation rebuilds and lands on the same answer: the cache
  // key is a performance decision, never a correctness one.
  rc.invalidate_shadow_maps();
  rc.render();
  const cvc::image after_invalidate = rc.shadow_map_depth(0);
  EXPECT_EQ(std::memcmp(after_invalidate.data(), moved_bytes.data(), bytes), 0)
      << "an invalidated rebuild produced a different map than the cached one";

  // Turning shadows off drops the maps.
  rs = rc.settings();
  rs.shadows.enabled = false;
  rc.settings() = rs;
  rc.render();
  EXPECT_EQ(rc.shadow_map_count(), 0u);
}

} // namespace
