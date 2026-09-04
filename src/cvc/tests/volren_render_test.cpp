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
  // thickness and obliquity.  `bias_scale` is the knob in HARD mode;
  // shadow_mode::deep is the actual fix, and 8.7 measures it.  This test stays
  // on hard deliberately -- it is what pins the bias.
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
      // A 128-texel light map, not the 512 default.  This test builds four of
      // them, and at the default the light pass alone is 16x the main pass
      // (512^2 vs 128^2 rays) -- enough to blow CI's 300 s per-test budget in a
      // Debug build, which is exactly how it first failed.  Cutting resolution
      // makes an ACNE test stricter, not weaker: coarser texels mean a larger
      // depth step per texel and so more chance for a surface to shadow
      // itself, which is the artifact being ruled out.
      rs.shadows.resolution = 128;
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

// ---------------------------------------------------------------------------
// 8.7 DEEP shadow maps (shadow_mode::deep)
// ---------------------------------------------------------------------------
// The plate-and-ball geometry is reused unchanged; only the OCCLUDER's opacity
// and the shadow mode move, which is what makes these tests about the payload
// rather than about the geometry (that is 8.2-8.3's job).
//
// Every measurement below is a VISIBILITY, recovered exactly rather than eyed:
// a shaded pixel's luminance is affine in the light's visibility factor
// (L = ambient_term + vis * (diffuse + specular)), so for three renders of the
// SAME scene differing only in the shadow mode,
//
//     vis_deep = (L_deep - L_hard) / (L_lit - L_hard)
//
// because the hard render pins vis = 0 on those pixels and the unshadowed one
// pins vis = 1.  All three register the same volumes, so scene_bounds -- and
// therefore unit_step and the whole receiver-side march -- is identical and
// cancels.
//
// That relation holds only while nothing CLAMPS: blinn_phong saturates its
// output at 1, and 8.2's white key lights this plate to pure 255 -- which flattens
// the vis = 1 end and inflates every recovered visibility (measured: it put the
// half-strength check 0.067 out, and quietly biased the full-strength one).  So
// the deep tests dim the key and assert that no measured pixel reached 255.

// 8.2's scene with a key dim enough that the lit plate stays well inside the
// [0, 255] range blinn_phong clamps to.
render_settings deepSceneSettings(double elevation_deg = 45.0) {
  render_settings rs = shadowSceneSettings(elevation_deg);
  rs.lights[0].color = {0.5f, 0.5f, 0.5f};
  return rs;
}

// Highest channel anywhere the frame drew something: 255 means the shading
// clamped and the visibility recovery below is not valid.
int peakChannel(const frame &f) {
  int peak = 0;
  for (int y = 0; y < kShadowRaster; ++y)
    for (int x = 0; x < kShadowRaster; ++x)
      for (int c = 0; c < 3; ++c)
        peak = std::max(peak, int(pixel(f, x, y)[c]));
  return peak;
}

// A ball whose r = kBallRadius isosurface is TRANSLUCENT.  A closed surface, so
// a light ray through it crosses TWICE: the front-to-back accumulation is
// a + a(1-a), which is the number every expectation below is built from.
volume_settings translucentBall(float opacity) {
  volume_settings bs = opaqueIso(kBallRadius, opacity);
  bs.isosurfaces[0].color = {0.9f, 0.4f, 0.4f};
  bs.model_transform = translation(kBallHeight, 0.0, kBallHeight);
  return bs;
}

// Mean recovered visibility over the pixels the HARD render actually shadowed,
// and how many those were.
double meanShadowVisibility(const frame &lit, const frame &hard, const frame &probe, int &count) {
  double sum = 0.0;
  count = 0;
  for (int y = 0; y < kShadowRaster; ++y)
    for (int x = 0; x < kShadowRaster; ++x) {
      if (pixel(lit, x, y)[3] == 0)
        continue;
      const double l_lit = luminance(lit, x, y);
      const double l_hard = luminance(hard, x, y);
      if (!(l_hard < l_lit - l_lit / 10.0)) // not meaningfully shadowed
        continue;
      ++count;
      sum += (luminance(probe, x, y) - l_hard) / (l_lit - l_hard);
    }
  return count > 0 ? sum / count : 0.0;
}

TEST_F(VolrenRenderTest, DeepShadowsAreOptInAndOpaqueOccludersMatchHardExactly) {
  // The default is hard, so no existing scene moves.  Pinned here rather than
  // trusted, because it is the whole no-op guarantee.
  EXPECT_TRUE(cvc::volren::shadow_settings().mode == cvc::volren::shadow_mode::hard);

  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  raycaster hard_rc(ctx), deep_rc(ctx);
  for (raycaster *rc : {&hard_rc, &deep_rc}) {
    buildPlateAndBall(*rc, plate, ball, true); // an OPAQUE ball
    render_settings rs = rc->settings();
    rs.shadows.enabled = true;
    rc->settings() = rs;
  }
  render_settings deep_rs = deep_rc.settings();
  deep_rs.shadows.mode = cvc::volren::shadow_mode::deep;
  deep_rc.settings() = deep_rs;

  const frame hard = hard_rc.render();
  const frame deep = deep_rc.render();

  // BYTE-identical, not merely close.  An opaque occluder saturates the light
  // ray at an exactly known depth, the profile's terminal channel records that
  // depth, and the terminal comparison is written as the same expression the
  // hard lookup uses -- so the two lookups are the same function here.
  EXPECT_TRUE(framesIdentical(hard, deep))
      << "an opaque occluder rendered differently in deep mode";

  // The payload is exposed, with the documented plane-major shape.
  ASSERT_EQ(deep_rc.shadow_map_count(), 1u);
  const shadow_view &v = deep_rc.shadow_map_view(0);
  EXPECT_EQ(v.slices, cvc::volren::defaults::shadow_depth_slices);
  EXPECT_GT(v.slice_dz, 0.0);
  const cvc::image prof = deep_rc.shadow_map_profile(0);
  EXPECT_EQ(prof.width(), v.width);
  EXPECT_EQ(prof.height(), v.height * (v.slices + 1));

  // ... and a HARD map carries none of it.
  ASSERT_EQ(hard_rc.shadow_map_count(), 1u);
  EXPECT_EQ(hard_rc.shadow_map_view(0).slices, 0);
  EXPECT_EQ(hard_rc.shadow_map_profile(0).width(), 0);
  EXPECT_THROW(hard_rc.shadow_map_profile(1), cvc::volren_error);
}

TEST_F(VolrenRenderTest, TranslucentOccluderCastsAPartialShadow) {
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  // 0.6 is ABOVE the default min_occluder_opacity, so the ball casts in hard
  // mode too -- which is what gives the comparison a fully-shadowed end.
  const float opacity = 0.6f;
  const auto build = [&](raycaster &rc, int mode) {
    rc.add_volume(plate, opaqueIso(0.0));
    rc.add_volume(ball, translucentBall(opacity));
    rc.view() = plateCam();
    render_settings rs = deepSceneSettings();
    if (mode >= 0) {
      rs.shadows.enabled = true;
      rs.shadows.mode = mode == 1 ? cvc::volren::shadow_mode::deep : cvc::volren::shadow_mode::hard;
    }
    rc.settings() = rs;
  };

  raycaster lit_rc(ctx), hard_rc(ctx), deep_rc(ctx);
  build(lit_rc, -1);
  build(hard_rc, 0);
  build(deep_rc, 1);
  const frame lit = lit_rc.render(), hard = hard_rc.render(), deep = deep_rc.render();
  ASSERT_LT(peakChannel(lit), 255)
      << "the lit reference clamped -- the visibility recovery is not valid";

  int n = 0;
  const double vis = meanShadowVisibility(lit, hard, deep, n);
  ASSERT_GT(n, 500) << "the hard render cast no shadow to compare against";

  // STRICTLY between the two ends: that is the entire claim of a deep map.
  EXPECT_GT(vis, 0.02) << "the deep shadow is as dark as the hard one";
  EXPECT_LT(vis, 0.98) << "the deep shadow does not darken at all";

  // And it is not merely "somewhere in between": a closed translucent surface
  // is crossed twice, so the light ray accumulates a + a(1-a) and the receiver
  // must see exactly the remaining transmittance.
  const double alpha = double(opacity) + double(opacity) * (1.0 - double(opacity));
  EXPECT_NEAR(vis, 1.0 - alpha, 0.005)
      << "the partial shadow is not the two-crossing transmittance " << (1.0 - alpha);

  // The same measurement on the HARD render recovers 0 by construction, and on
  // the unshadowed one recovers 1 -- the ends the ratio is calibrated against.
  int m = 0;
  EXPECT_NEAR(meanShadowVisibility(lit, hard, hard, m), 0.0, 1e-9);
  EXPECT_NEAR(meanShadowVisibility(lit, hard, lit, m), 1.0, 1e-9);

  // strength still scales the whole thing, deep or hard.
  raycaster half_rc(ctx);
  build(half_rc, 1);
  render_settings hs = half_rc.settings();
  hs.shadows.strength = 0.5f;
  half_rc.settings() = hs;
  const double half = meanShadowVisibility(lit, hard, half_rc.render(), m);
  EXPECT_NEAR(half, 1.0 - 0.5 * alpha, 0.005) << "strength does not scale the deep attenuation";
}

TEST_F(VolrenRenderTest, DeepShadowsStackOccluderLayersThatHardShadowsCannot) {
  // The documented hard-mode limit -- "ONE occluder layer per light ray, so a
  // point behind two thin sheets is exactly as dark as behind one" -- is the
  // thing deep maps exist to remove.  Two identical translucent balls on the
  // SAME light ray: hard cannot tell them apart, deep must square the
  // transmittance.
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  // 0.4 per surface: a closed ball accumulates 0.4 + 0.4*0.6 = 0.64, and TWO of
  // them reach 0.8704 -- still under opacity_cutoff, so the light ray does not
  // terminate and the second layer is carried by the SLICES rather than by the
  // terminal channel.  (The saturating case is pinned separately below.)
  const float opacity = 0.4f;
  const double one = double(opacity) + double(opacity) * (1.0 - double(opacity));

  // Both centres lie on the 45-degree light ray through the origin, so both
  // shadow the SAME patch of plate.  `second_opacity` 0 keeps the far ball
  // registered -- scene_bounds, unit_step and the whole receiver-side march
  // stay identical across every render here -- while removing it from the
  // light's way.
  const auto build = [&](raycaster &rc, float first_opacity, float second_opacity, int mode) {
    rc.add_volume(plate, opaqueIso(0.0));
    // NOT `near`/`far`: minwindef.h defines both as empty macros, so those
    // names vanish under MSVC and the declarations become syntax errors.
    volume_settings nearBall = translucentBall(first_opacity);
    rc.add_volume(ball, nearBall);
    volume_settings farBall = translucentBall(second_opacity);
    farBall.model_transform = translation(kBallHeight * 2, 0.0, kBallHeight * 2);
    rc.add_volume(ball, farBall);
    rc.view() = plateCam();
    render_settings rs = deepSceneSettings();
    if (mode >= 0) {
      rs.shadows.enabled = true;
      rs.shadows.mode = mode == 1 ? cvc::volren::shadow_mode::deep : cvc::volren::shadow_mode::hard;
      // Below the 0.5 default these surfaces would be dropped from the HARD
      // light pass entirely and there would be no vis = 0 end to calibrate
      // against.  Deep mode ignores this knob (pinned by its own test).
      rs.shadows.min_occluder_opacity = 0.2f;
    }
    rc.settings() = rs;
  };

  const auto renderWith = [&](float a1, float a2, int mode) {
    raycaster rc(ctx);
    build(rc, a1, a2, mode);
    return rc.render();
  };

  const frame lit = renderWith(opacity, 0.f, -1);
  const frame hard1 = renderWith(opacity, 0.f, 0);
  const frame hard2 = renderWith(opacity, opacity, 0);
  const frame deep1 = renderWith(opacity, 0.f, 1);
  const frame deep2 = renderWith(opacity, opacity, 1);
  ASSERT_LT(peakChannel(lit), 255)
      << "the lit reference clamped -- the visibility recovery is not valid";

  int n = 0;
  const double v_hard1 = meanShadowVisibility(lit, hard1, hard1, n);
  ASSERT_GT(n, 500) << "the hard render cast no shadow to compare against";
  const double v_hard2 = meanShadowVisibility(lit, hard1, hard2, n);
  const double v_deep1 = meanShadowVisibility(lit, hard1, deep1, n);
  const double v_deep2 = meanShadowVisibility(lit, hard1, deep2, n);

  // Hard: the second layer changes nothing -- it is already fully dark.
  EXPECT_NEAR(v_hard2, v_hard1, 1e-9)
      << "hard shadows suddenly grew a second occluder layer; this test's premise is stale";

  // Deep: two layers multiply.  One closed surface leaves 1 - (a + a(1-a));
  // two of them leave the SQUARE of that.
  EXPECT_NEAR(v_deep1, 1.0 - one, 0.005);
  EXPECT_NEAR(v_deep2, (1.0 - one) * (1.0 - one), 0.005)
      << "a second translucent layer did not compose";
  EXPECT_LT(v_deep2, v_deep1 - 0.05) << "the second layer did not darken anything";

  // ... and the documented ceiling on that composition: once the light ray's
  // accumulation reaches opacity_cutoff the profile records a TERMINAL depth and
  // everything behind it is fully blocked, discarding the residual
  // 1 - opacity_cutoff.  Two 0.6 balls accumulate 0.9744, past the 0.95 cutoff,
  // so deep collapses onto the hard answer exactly.
  const frame sat_lit = renderWith(0.6f, 0.f, -1);
  const frame sat_hard = renderWith(0.6f, 0.6f, 0);
  const frame sat_deep = renderWith(0.6f, 0.6f, 1);
  int sn = 0;
  const double v_sat = meanShadowVisibility(sat_lit, sat_hard, sat_deep, sn);
  ASSERT_GT(sn, 500);
  EXPECT_NEAR(v_sat, 0.0, 1e-9)
      << "an accumulation past opacity_cutoff did not collapse to the hard answer";
}

TEST_F(VolrenRenderTest, DeepShadowProfileIsTheLightRaysAccumulation) {
  // Read the payload back and check it against the accumulation computed by
  // hand.  This is the strongest statement available about the map itself,
  // independent of how the main march consumes it.
  //
  // ONE volume: a 0.5-radius ball carrying a translucent r = 0.3 isosurface,
  // lit straight down +z.  The centre texel's light ray enters at z = +0.3 and
  // leaves at z = -0.3, so its profile has to be a two-step staircase.
  const cvc::volume ball = makeBallVolume(ctx, 64, 0.5);
  const float opacity = 0.6f;

  raycaster rc(ctx);
  volume_settings vs = opaqueIso(0.3, opacity);
  rc.add_volume(ball, vs);
  rc.view() = plateCam();
  render_settings rs = shadowSceneSettings(90.0); // straight overhead
  rs.shadows.enabled = true;
  rs.shadows.mode = cvc::volren::shadow_mode::deep;
  rs.shadows.resolution = 64; // this test is about the payload, not its detail
  rs.shadows.depth_slices = 16;
  rc.settings() = rs;
  rc.render();

  ASSERT_EQ(rc.shadow_map_count(), 1u);
  const shadow_view &v = rc.shadow_map_view(0);
  ASSERT_EQ(v.slices, 16);
  const cvc::image prof = rc.shadow_map_profile(0); // const: no COW detach
  const float *p = reinterpret_cast<const float *>(prof.data());
  const std::size_t plane = std::size_t(v.width) * std::size_t(v.height);

  const std::size_t centre = std::size_t(v.height / 2) * std::size_t(v.width) + v.width / 2;
  const auto knot = [&](int j) { return double(p[std::size_t(j) * plane + centre]); };

  // Nothing saturates a 0.6 + 0.6*(1-0.6) = 0.84 accumulation, so the ray never
  // terminates and slot 0 stays +inf.
  EXPECT_TRUE(std::isinf(p[centre])) << "the light ray terminated where nothing is opaque";

  const double one = double(opacity);
  const double two = one + one * (1.0 - one);
  // Monotone, starting at 0, ending at the two-crossing accumulation, and only
  // ever sitting on one of the three exact levels -- the profile stores the
  // staircase's SAMPLES, so no knot may hold an interpolated value.
  double previous = 0.0;
  bool saw_one = false;
  for (int j = 1; j <= v.slices; ++j) {
    const double a = knot(j);
    EXPECT_GE(a, previous - 1e-6) << "knot " << j << " went backwards";
    const bool exact =
        std::fabs(a) < 1e-6 || std::fabs(a - one) < 1e-6 || std::fabs(a - two) < 1e-6;
    EXPECT_TRUE(exact) << "knot " << j << " holds " << a << ", which is none of {0, " << one << ", "
                       << two << "}";
    saw_one = saw_one || std::fabs(a - one) < 1e-6;
    previous = a;
  }
  EXPECT_NEAR(knot(1), 0.0, 1e-6) << "the first knot is inside the ball";
  EXPECT_NEAR(knot(v.slices), two, 1e-6) << "the last knot missed the far crossing";
  EXPECT_TRUE(saw_one) << "no knot landed between the two crossings";

  // A texel whose ray misses the ball entirely accumulates nothing at all.
  const std::size_t corner = 0;
  EXPECT_TRUE(std::isinf(p[corner]));
  for (int j = 1; j <= v.slices; ++j)
    EXPECT_FLOAT_EQ(p[std::size_t(j) * plane + corner], 0.f) << "corner knot " << j;
}

TEST_F(VolrenRenderTest, DeepShadowsIgnoreMinOccluderOpacity) {
  // min_occluder_opacity exists only because a hard latch cannot represent a
  // low-opacity surface at all: it would treat a 0.16 decorative shell as a
  // full occluder, so the shell is DELETED from the light pass instead.  A deep
  // map represents it, so the knob is deliberately inert -- and a surface below
  // the threshold casts its true 0.16 rather than nothing.
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  const auto renderWith = [&](float min_occluder) {
    raycaster rc(ctx);
    buildPlateAndBall(rc, plate, ball, true, 0.16f);
    render_settings rs = rc.settings();
    rs.shadows.enabled = true;
    rs.shadows.mode = cvc::volren::shadow_mode::deep;
    rs.shadows.min_occluder_opacity = min_occluder;
    rc.settings() = rs;
    return rc.render();
  };

  raycaster base(ctx);
  buildPlateAndBall(base, plate, ball, true, 0.16f);
  const frame unshadowed = base.render();

  const frame at_default = renderWith(cvc::volren::defaults::shadow_min_occluder_opacity);
  const frame at_low = renderWith(0.1f);
  EXPECT_TRUE(framesIdentical(at_default, at_low))
      << "min_occluder_opacity changed a deep render -- it is documented as ignored there";

  // And the shell really does cast: measurably darker than unshadowed, but far
  // from the black a hard map would have produced.
  int lit = 0, darkened = 0;
  double sum_ratio = 0.0;
  for (int y = 0; y < kShadowRaster; ++y)
    for (int x = 0; x < kShadowRaster; ++x) {
      if (pixel(unshadowed, x, y)[3] == 0)
        continue;
      const double l0 = luminance(unshadowed, x, y);
      if (!(l0 > 0.0))
        continue;
      ++lit;
      const double l1 = luminance(at_default, x, y);
      if (l1 < l0 - 1.0) {
        ++darkened;
        sum_ratio += l1 / l0;
      }
    }
  ASSERT_GT(lit, 500);
  EXPECT_GT(darkened, 100) << "a 0.16-opacity surface cast nothing in deep mode";
  EXPECT_GT(sum_ratio / darkened, 0.5)
      << "a 0.16-opacity surface cast a near-total shadow -- that is the hard-mode bug";
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

// ---------------------------------------------------------------------------
// 9. Soft shadows (shadow_settings::pcf_radius / pcf_taps)
// ---------------------------------------------------------------------------
// Same plate-and-ball scene as section 8, because the thing being measured is
// the EDGE of the shadow it already casts.  The camera window shows only the
// plate (the ball is parked outside it), so every pixel with alpha > 0 is a
// flat, uniformly shaded receiver and any luminance strictly between the lit
// and umbra plateaus is penumbra -- not curvature, not silhouette.

// Lit / umbra plateaus and the count of pixels between them.
struct penumbra_census {
  int lit = 0, umbra = 0, partial = 0, levels = 0;
};

penumbra_census censusPenumbra(const frame &f) {
  penumbra_census c;
  c.lit = 0;
  c.umbra = 1 << 30;
  std::vector<int> values;
  for (int y = 0; y < kShadowRaster; ++y)
    for (int x = 0; x < kShadowRaster; ++x) {
      if (pixel(f, x, y)[3] == 0)
        continue;
      const int l = luminance(f, x, y);
      values.push_back(l);
      c.lit = std::max(c.lit, l);
      c.umbra = std::min(c.umbra, l);
    }
  for (const int l : values)
    if (l > c.umbra + 2 && l < c.lit - 2)
      ++c.partial;
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
  c.levels = int(values.size());
  return c;
}

TEST_F(VolrenRenderTest, SoftShadowsAreOptInAndTheTapCountIsInertWithoutARadius) {
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  raycaster plain(ctx), taps(ctx), zero_radius(ctx), one_tap(ctx);
  for (raycaster *rc : {&plain, &taps, &zero_radius, &one_tap}) {
    buildPlateAndBall(*rc, plate, ball, true);
    rc->settings().shadows.enabled = true;
  }
  // Every one of these must reproduce the unfiltered render BIT FOR BIT: the
  // filter is inert from either knob, and both callers branch to the
  // historical single-tap expression rather than averaging one value.
  taps.settings().shadows.pcf_taps = 7;
  zero_radius.settings().shadows.pcf_radius = 0.f;
  zero_radius.settings().shadows.pcf_taps = 5;
  one_tap.settings().shadows.pcf_radius = 8.f;
  one_tap.settings().shadows.pcf_taps = 1;

  const frame a = plain.render();
  EXPECT_TRUE(framesIdentical(a, taps.render()))
      << "pcf_taps moved a pixel at radius 0 -- the filter is not inert";
  EXPECT_TRUE(framesIdentical(a, zero_radius.render()));
  EXPECT_TRUE(framesIdentical(a, one_tap.render())) << "a single tap is not the unfiltered lookup";

  // A negative or NaN radius is inert too, rather than throwing or filtering.
  raycaster negative(ctx);
  buildPlateAndBall(negative, plate, ball, true);
  negative.settings().shadows.enabled = true;
  negative.settings().shadows.pcf_radius = -4.f;
  EXPECT_TRUE(framesIdentical(a, negative.render()));
  negative.settings().shadows.pcf_radius = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(framesIdentical(a, negative.render()));
}

TEST_F(VolrenRenderTest, SoftShadowRadiusSetsThePenumbraWidthAndTapsSetItsSmoothness) {
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  for (const cvc::volren::shadow_mode mode :
       {cvc::volren::shadow_mode::hard, cvc::volren::shadow_mode::deep}) {
    raycaster rc(ctx);
    buildPlateAndBall(rc, plate, ball, true);
    rc.settings().shadows.enabled = true;
    rc.settings().shadows.mode = mode;
    rc.settings().shadows.depth_slices = 16;
    const char *label = mode == cvc::volren::shadow_mode::deep ? "deep" : "hard";

    // Unfiltered: the comparison is binary, so the plate has exactly two
    // levels and nothing between them.
    rc.settings().shadows.pcf_radius = 0.f;
    const penumbra_census sharp = censusPenumbra(rc.render());
    ASSERT_GT(sharp.lit, sharp.umbra + 20) << label << ": the scene casts no measurable shadow";
    EXPECT_EQ(sharp.partial, 0) << label << ": an unfiltered shadow has a penumbra";
    EXPECT_EQ(sharp.levels, 2) << label << ": an unfiltered shadow is not two-valued";

    // RADIUS is the width knob: the band of partly-lit pixels grows with it,
    // and it is a BAND -- the umbra and the lit plateau are still there.
    int previous = 0;
    for (const float radius : {1.f, 2.f, 4.f}) {
      rc.settings().shadows.pcf_radius = radius;
      rc.settings().shadows.pcf_taps = 3;
      const penumbra_census c = censusPenumbra(rc.render());
      EXPECT_GT(c.partial, previous)
          << label << ": radius " << radius << " did not widen the penumbra";
      EXPECT_EQ(c.lit, sharp.lit) << label << ": the filter moved the LIT plateau";
      EXPECT_EQ(c.umbra, sharp.umbra) << label << ": the filter moved the UMBRA";
      previous = c.partial;
    }

    // TAPS is the smoothness knob, and it is INDEPENDENT of the width: at a
    // fixed radius more taps resolve the same band into more levels.
    rc.settings().shadows.pcf_radius = 4.f;
    int levels3 = 0, width3 = 0;
    for (const int taps : {3, 5, 7}) {
      rc.settings().shadows.pcf_taps = taps;
      const penumbra_census c = censusPenumbra(rc.render());
      if (taps == 3) {
        levels3 = c.levels;
        width3 = c.partial;
      } else {
        EXPECT_GT(c.levels, levels3) << label << ": " << taps << " taps added no levels";
        // The band's WIDTH is set by the radius alone, so it must not move
        // more than the level quantization can explain.
        EXPECT_NEAR(c.partial, width3, width3 / 4 + 4)
            << label << ": the tap count changed the penumbra WIDTH";
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 10. Ambient occlusion (ao_settings, volume_settings::distance_field)
// ---------------------------------------------------------------------------
// Two overlapping balls in ONE signed distance field: the seam where they meet
// is a crease with a known geometry, and the outer caps are open surface.  The
// scene is lit by AMBIENT ALONE (no lights at all), which turns the test into
// arithmetic: every pixel is gain * ambient * base * ao, so the occlusion
// factor can be read straight off the image instead of inferred.

constexpr double kSeamBallOffset = 0.32;
constexpr double kSeamBallRadius = 0.4;

cvc::volume makeTwoBallSdf(cvc::app &ctx, unsigned n) {
  cvc::volume vol(ctx, cvc::dimension(n, n, n), cvc::Float,
                  cvc::bounding_box(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0));
  for (unsigned k = 0; k < n; ++k)
    for (unsigned j = 0; j < n; ++j)
      for (unsigned i = 0; i < n; ++i) {
        const double x = -1.0 + double(i) * vol.XSpan();
        const double y = -1.0 + double(j) * vol.YSpan();
        const double z = -1.0 + double(k) * vol.ZSpan();
        const double a = std::sqrt((x + kSeamBallOffset) * (x + kSeamBallOffset) + y * y + z * z) -
                         kSeamBallRadius;
        const double b = std::sqrt((x - kSeamBallOffset) * (x - kSeamBallOffset) + y * y + z * z) -
                         kSeamBallRadius;
        vol(i, j, k, std::min(a, b));
      }
  return vol;
}

// Looking along +y at the seam, up = +z: screen x is world x, screen y is -z.
camera seamCam() {
  camera c;
  c.eye = {0.0, -3.0, 0.0};
  c.focal = {0.0, 0.0, 0.0};
  c.up = {0.0, 0.0, 1.0};
  c.projection = camera::projection_type::orthographic;
  c.parallel_scale = 0.8;
  c.width = c.height = kRaster;
  return c;
}

void buildSeamScene(raycaster &rc, const cvc::volume &vol, bool distance_field) {
  volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  vs.distance_field = distance_field;
  isosurface s;
  s.value = 0.0;
  s.opacity = 1.0f;
  s.color = {0.8f, 0.8f, 0.8f};
  vs.isosurfaces.push_back(s);
  rc.add_volume(vol, vs);
  rc.view() = seamCam();
  render_settings rs;
  rs.lights.clear(); // ambient ONLY: the image IS the occlusion factor
  rs.ambient = 1.0f;
  rs.shading_gain = 1.0f;
  rs.steps = 256;
  rc.settings() = rs;
}

TEST_F(VolrenRenderTest, AmbientOcclusionDarkensACreaseAndLeavesOpenSurfaceBitwiseAlone) {
  const cvc::volume vol = makeTwoBallSdf(ctx, 48);

  raycaster off(ctx), on(ctx);
  buildSeamScene(off, vol, true);
  buildSeamScene(on, vol, true);
  on.settings().ao.strength = 1.0f;
  on.settings().ao.radius = 0.3; // well short of the far ball (0.53 away)
  on.settings().ao.samples = 5;

  const frame a = off.render();
  const frame b = on.render();

  // Unlit, so every visible pixel is exactly gain * ambient * base.
  const int seam_x = kRaster / 2, cap_x = 10, row = kRaster / 2;
  ASSERT_GT(pixel(a, seam_x, row)[3], 0);
  ASSERT_GT(pixel(a, cap_x, row)[3], 0);
  const int flat = expectedByte(0.8f);
  EXPECT_EQ(int(pixel(a, seam_x, row)[1]), flat);
  EXPECT_EQ(int(pixel(a, cap_x, row)[1]), flat);

  // The crease loses a real fraction of its ambient...
  const int seam = int(pixel(b, seam_x, row)[1]);
  EXPECT_LT(seam, flat - 20) << "the crease did not darken";
  EXPECT_GT(seam, 0) << "the crease went black -- occlusion above 1";
  // ...and the open cap keeps its value to within ONE level.  Not bitwise, and
  // the reason is worth stating: the estimator reads the DISCRETIZED field, and
  // trilinear reconstruction of a curved distance field cuts chords across the
  // level sets, so f(p + n*h) comes back a hair SHORT of h on a convex surface.
  // At 48^3 over a 0.4-radius ball that is a few tenths of a percent of
  // occlusion -- one 0-255 level out of the crease's twenty-plus, i.e. the
  // localization is a property of the geometry and not of the tolerance.
  EXPECT_LE(flat - int(pixel(b, cap_x, row)[1]), 1) << "an exposed surface darkened materially";

  // Alpha never moves: occlusion is a shading term, not a compositing one.
  int alpha_changes = 0, darkened = 0, brightened = 0;
  for (int y = 0; y < kRaster; ++y)
    for (int x = 0; x < kRaster; ++x) {
      if (pixel(a, x, y)[3] != pixel(b, x, y)[3])
        ++alpha_changes;
      if (pixel(a, x, y)[3] == 0)
        continue;
      const int d = int(pixel(a, x, y)[1]) - int(pixel(b, x, y)[1]);
      if (d > 0)
        ++darkened;
      if (d < 0)
        ++brightened;
    }
  EXPECT_EQ(alpha_changes, 0) << "occlusion leaked into the alpha channel";
  EXPECT_EQ(brightened, 0) << "occlusion made a pixel BRIGHTER";
  EXPECT_GT(darkened, 20);

  // Where the REAL darkening is: every pixel that lost more than 4 levels sits
  // in the seam column, not scattered over the caps.
  int strong = 0, strong_max_dx = 0;
  for (int y = 0; y < kRaster; ++y)
    for (int x = 0; x < kRaster; ++x) {
      if (pixel(a, x, y)[3] == 0)
        continue;
      if (int(pixel(a, x, y)[1]) - int(pixel(b, x, y)[1]) > 4) {
        ++strong;
        strong_max_dx = std::max(strong_max_dx, std::abs(x - seam_x));
      }
    }
  EXPECT_GT(strong, 20) << "the crease darkening is a handful of pixels, not a band";
  EXPECT_LE(strong_max_dx, 12) << "occlusion reached " << strong_max_dx
                               << " px from the seam -- it is not localized";

  // Strength scales the whole term linearly, so half the strength is half the
  // darkening (to within the byte quantization).
  on.settings().ao.strength = 0.5f;
  const int half = int(pixel(on.render(), seam_x, row)[1]);
  EXPECT_NEAR(double(flat - half), 0.5 * double(flat - seam), 1.5);
}

TEST_F(VolrenRenderTest, AmbientOcclusionIsOffUnlessEveryPreconditionHolds) {
  const cvc::volume vol = makeTwoBallSdf(ctx, 48);

  raycaster reference(ctx);
  buildSeamScene(reference, vol, true);
  const frame a = reference.render();

  // Each of these must be BYTE-IDENTICAL to the no-AO render, because each
  // removes one of the four things the cone needs.
  struct variant {
    const char *why;
    bool distance_field;
    float strength;
    double radius;
    float ambient;
  };
  const variant variants[] = {
      {"the volume is not declared a distance field", false, 1.f, 0.3, 1.f},
      {"strength is 0", true, 0.f, 0.3, 1.f},
      {"the radius is 0", true, 1.f, 0.0, 1.f},
      {"the radius is negative", true, 1.f, -0.3, 1.f},
  };
  for (const variant &v : variants) {
    raycaster rc(ctx);
    buildSeamScene(rc, vol, v.distance_field);
    rc.settings().ao.strength = v.strength;
    rc.settings().ao.radius = v.radius;
    rc.settings().ao.samples = 5;
    rc.settings().ambient = v.ambient;
    EXPECT_TRUE(framesIdentical(a, rc.render())) << "AO ran when " << v.why;
  }

  // With ambient exactly 0 there is nothing for AO to attenuate, so it is
  // skipped outright rather than run and multiplied away -- and the proof is
  // that a LIT render is unchanged by it.
  raycaster lit_off(ctx), lit_on(ctx);
  for (raycaster *rc : {&lit_off, &lit_on}) {
    buildSeamScene(*rc, vol, true);
    light key;
    key.color = {1.f, 1.f, 1.f};
    key.direction = {0.0, -1.0, 0.0};
    rc->settings().lights = {key};
    rc->settings().ambient = 0.f;
  }
  lit_on.settings().ao.strength = 1.f;
  lit_on.settings().ao.radius = 0.3;
  EXPECT_TRUE(framesIdentical(lit_off.render(), lit_on.render()))
      << "AO changed an image whose ambient term is zero";

  // An out-of-range strength is refused loudly (the shadows.strength rule),
  // because it would drive the ambient term negative.
  raycaster bad(ctx);
  buildSeamScene(bad, vol, true);
  bad.settings().ao.strength = 1.5f;
  EXPECT_THROW(bad.render(), cvc::volren_error);
  bad.settings().ao.strength = -0.5f;
  EXPECT_THROW(bad.render(), cvc::volren_error);
  bad.settings().ao.strength = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(bad.render(), cvc::volren_error);

  // `samples` is a resource and is CLAMPED, not refused: 0 and a huge value
  // both render, and both land on the in-range answer.
  raycaster clamp_lo(ctx), clamp_hi(ctx), in_range_lo(ctx), in_range_hi(ctx);
  const int lo = cvc::volren::limits::min_ao_samples;
  const int hi = cvc::volren::limits::max_ao_samples;
  const int counts[4] = {0, lo, 10000, hi};
  raycaster *rcs[4] = {&clamp_lo, &in_range_lo, &clamp_hi, &in_range_hi};
  frame rendered[4];
  for (int i = 0; i < 4; ++i) {
    buildSeamScene(*rcs[i], vol, true);
    rcs[i]->settings().ao.strength = 1.f;
    rcs[i]->settings().ao.radius = 0.3;
    rcs[i]->settings().ao.samples = counts[i];
    rendered[i] = rcs[i]->render();
  }
  EXPECT_TRUE(framesIdentical(rendered[0], rendered[1]));
  EXPECT_TRUE(framesIdentical(rendered[2], rendered[3]));
}

// ---------------------------------------------------------------------------
// 11. Energy: shadows and occlusion attenuate DIFFERENT terms
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, ShadowsAndOcclusionDoNotDoubleDarken) {
  // The identity: shading is L = A*ao + D*vis, with A the ambient term and D
  // the direct term.  A*ao and D*vis are attenuated by DIFFERENT factors, so L
  // is affine in (ao, vis) and therefore
  //     L(ao, vis) + L(1, 1) == L(ao, 1) + L(1, vis)
  // EXACTLY.  If either factor ever multiplied the other's term -- the whole
  // failure mode this test exists for -- the product term breaks the identity
  // by exactly the amount of the double attenuation.
  // The two-ball field is the right scene for this: it has a CREASE for the
  // occlusion cone and, lit from the side, one ball is an occluder for the
  // other, so both attenuations land on the same visible pixels.
  const cvc::volume vol = makeTwoBallSdf(ctx, 48);

  const auto build = [&](raycaster &rc, bool shadows_on, bool ao_on) {
    buildSeamScene(rc, vol, true);
    render_settings rs = rc.settings();
    light key;
    key.color = {0.55f, 0.55f, 0.55f};
    key.direction = {0.82, -0.3, 0.49};
    rs.lights = {key};
    rs.ambient = 0.4f;
    rs.shading_gain = 0.9f;
    // No specular: the identity is about the LINEAR part of the model, and a
    // clamped highlight is the one thing that could break it for a reason that
    // is not double-darkening.
    rs.specular = 0.f;
    rs.shadows.enabled = shadows_on;
    rs.shadows.resolution = 256;
    rs.ao.strength = ao_on ? 1.0f : 0.f;
    rs.ao.radius = 0.3;
    rs.ao.samples = 5;
    rc.settings() = rs;
  };

  raycaster none(ctx), ao_only(ctx), shadow_only(ctx), both(ctx);
  build(none, false, false);
  build(ao_only, false, true);
  build(shadow_only, true, false);
  build(both, true, true);
  const frame f_none = none.render(), f_ao = ao_only.render();
  const frame f_shadow = shadow_only.render(), f_both = both.render();

  int checked = 0, worst = 0, ao_effect = 0, shadow_effect = 0;
  for (int y = 0; y < kRaster; ++y)
    for (int x = 0; x < kRaster; ++x) {
      if (pixel(f_none, x, y)[3] == 0)
        continue;
      ++checked;
      for (int c = 0; c < 3; ++c) {
        const int n = pixel(f_none, x, y)[c], a = pixel(f_ao, x, y)[c];
        const int s = pixel(f_shadow, x, y)[c], b = pixel(f_both, x, y)[c];
        worst = std::max(worst, std::abs((b + n) - (a + s)));
        ao_effect = std::max(ao_effect, n - a);
        shadow_effect = std::max(shadow_effect, n - s);
      }
    }
  ASSERT_GT(checked, 500);
  // Both effects have to be REAL for the identity to say anything.
  EXPECT_GT(ao_effect, 8) << "the AO cone did nothing on this scene";
  EXPECT_GT(shadow_effect, 20) << "the shadow did nothing on this scene";
  // Four independently rounded renders, so the identity can only hold to the
  // quantization: two roundings on each side, 1 LSB each.
  EXPECT_LE(worst, 2) << "shadows and occlusion attenuate the same term somewhere: the affine "
                         "identity is off by "
                      << worst << " levels, against an AO effect of " << ao_effect
                      << " and a shadow effect of " << shadow_effect;
}

// ---------------------------------------------------------------------------
// 12. The light rig: hemisphere ambient, output gain, specular reflectance
// ---------------------------------------------------------------------------

TEST_F(VolrenRenderTest, HemisphereAmbientIsNeutralUntilItsColoursDiffer) {
  const cvc::volume vol = makeTwoBallSdf(ctx, 48);

  raycaster off(ctx), neutral(ctx), tinted(ctx), flipped(ctx);
  for (raycaster *rc : {&off, &neutral, &tinted, &flipped})
    buildSeamScene(*rc, vol, true);
  neutral.settings().ambient_hemisphere.enabled = true; // both colours white
  tinted.settings().ambient_hemisphere.enabled = true;
  tinted.settings().ambient_hemisphere.sky = {0.f, 0.f, 1.f};
  tinted.settings().ambient_hemisphere.ground = {1.f, 0.f, 0.f};
  flipped.settings().ambient_hemisphere = tinted.settings().ambient_hemisphere;
  flipped.settings().ambient_hemisphere.up = {0.0, 0.0, -1.0};

  const frame a = off.render();
  EXPECT_TRUE(framesIdentical(a, neutral.render()))
      << "the hemisphere is not a no-op with sky == ground; a0 == a1 must make the mix exact";

  // Up-facing surface -> sky, down-facing -> ground.  The rows are FOUND by
  // scanning the column rather than computed, so the test states a fact about
  // normals and not about where the silhouette happens to land.
  const frame t = tinted.render();
  const int ball_x = 10;
  int first = -1, last = -1;
  for (int y = 0; y < kRaster; ++y)
    if (pixel(t, ball_x, y)[3] > 0) {
      if (first < 0)
        first = y;
      last = y;
    }
  ASSERT_GE(last - first, 12) << "the probe column misses the ball";
  const int top_y = first + 4, bottom_y = last - 4;
  EXPECT_GT(int(pixel(t, ball_x, top_y)[2]), int(pixel(t, ball_x, top_y)[0]))
      << "the up-facing surface is not sky-coloured";
  EXPECT_GT(int(pixel(t, ball_x, bottom_y)[0]), int(pixel(t, ball_x, bottom_y)[2]))
      << "the down-facing surface is not ground-coloured";

  // Inverting `up` swaps them, which is the sharpest statement that the mix is
  // driven by the normal and not by anything else.
  const frame fl = flipped.render();
  EXPECT_EQ(int(pixel(fl, ball_x, top_y)[0]), int(pixel(t, ball_x, bottom_y)[0]));
  EXPECT_EQ(int(pixel(fl, ball_x, bottom_y)[2]), int(pixel(t, ball_x, top_y)[2]));
}

TEST_F(VolrenRenderTest, ShadingGainAndSpecularAreNeutralAtTheirDefaults) {
  const cvc::volume plate = makeLinearVolume(ctx, 48);
  const cvc::volume ball = makeBallVolume(ctx, 48, 0.25);

  raycaster reference(ctx), explicit_defaults(ctx);
  buildPlateAndBall(reference, plate, ball, true);
  buildPlateAndBall(explicit_defaults, plate, ball, true);
  explicit_defaults.settings().shading_gain = cvc::volren::defaults::shading_gain;
  explicit_defaults.settings().specular = cvc::volren::defaults::specular;
  const frame a = reference.render();
  EXPECT_TRUE(framesIdentical(a, explicit_defaults.render()))
      << "writing the defaults back is not a no-op -- the knobs are not folded exactly";

  // The gain is a pure output scale, so below the clamp it is exactly linear:
  // halving it halves every channel.  Ambient only (no lights) keeps the
  // scene far from the clamp so the linearity is testable at all.
  const cvc::volume seam = makeTwoBallSdf(ctx, 48);
  raycaster full(ctx), half(ctx);
  buildSeamScene(full, seam, true);
  buildSeamScene(half, seam, true);
  full.settings().shading_gain = 0.8f;
  half.settings().shading_gain = 0.4f;
  const frame ff = full.render(), fh = half.render();
  int compared = 0;
  for (int y = 0; y < kRaster; ++y)
    for (int x = 0; x < kRaster; ++x) {
      if (pixel(ff, x, y)[3] == 0)
        continue;
      ++compared;
      EXPECT_NEAR(int(pixel(ff, x, y)[1]), 2 * int(pixel(fh, x, y)[1]), 2);
    }
  ASSERT_GT(compared, 100);

  // Specular scales the highlight and NOTHING else: with the light exactly
  // behind the camera the plate carries a strong lobe, and driving the
  // reflectance to 0 must darken the highlight without touching the ambient
  // floor of a pixel the light cannot reach.
  raycaster spec_on(ctx), spec_off(ctx);
  for (raycaster *rc : {&spec_on, &spec_off}) {
    buildPlateAndBall(*rc, plate, ball, false);
    render_settings rs = rc->settings();
    rs.lights[0].direction = {0.0, 0.0, 1.0}; // straight down the view axis
    rs.lights[0].color = {0.3f, 0.3f, 0.3f};  // dim, so nothing clamps
    rs.ambient = 0.2f;
    rc->settings() = rs;
  }
  spec_off.settings().specular = 0.f;
  const frame s_on = spec_on.render(), s_off = spec_off.render();
  const int cx = kShadowRaster / 2;
  ASSERT_GT(pixel(s_on, cx, cx)[3], 0);
  EXPECT_GT(int(pixel(s_on, cx, cx)[1]), int(pixel(s_off, cx, cx)[1]) + 10)
      << "specular 0 did not remove the highlight";
  // Nothing got BRIGHTER, and the diffuse+ambient floor survives.
  int brightened = 0, lowest = 255;
  for (int y = 0; y < kShadowRaster; ++y)
    for (int x = 0; x < kShadowRaster; ++x) {
      if (pixel(s_on, x, y)[3] == 0)
        continue;
      if (int(pixel(s_off, x, y)[1]) > int(pixel(s_on, x, y)[1]))
        ++brightened;
      lowest = std::min(lowest, int(pixel(s_off, x, y)[1]));
    }
  EXPECT_EQ(brightened, 0);
  EXPECT_GT(lowest, 0) << "removing the specular crushed the surface to black";
}

} // namespace
