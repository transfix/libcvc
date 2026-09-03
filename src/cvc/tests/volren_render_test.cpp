// End-to-end tests for the cvc::volren raycaster (raycaster.h/.cpp).
//
// Geometry used throughout: volumes live in [-0.5, 0.5]^3 (node-centered,
// voxel i at min + i*span), the camera sits at (0,0,4) looking at the origin
// with up = +y, so screen right is world +x and screen up is world +y.  The
// orthographic camera uses parallel_scale 1.0 over a 64x64 raster, mapping
// pixel (px,py) center to world (u, v) with u = (px+0.5)/64*2-1 and
// v = 1-(py+0.5)/64*2 -- the 0.5-halfwidth volume box projects to the middle
// 32x32 pixels.

#include <boost/thread/thread.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/volren/raycaster.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>
#include <limits>

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

} // namespace
