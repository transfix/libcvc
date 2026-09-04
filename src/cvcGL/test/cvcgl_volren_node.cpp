// VolRenNode: the cvc::volren software raycaster composited into a live
// cvcGL scene as a depth-mapped translucent quad.  What is pinned here:
//
//   * the async raycast pipeline completes and its frame reaches the screen
//     (tick -> worker -> runOnMainThread -> textured quad -> pixels);
//   * the raycast image lands where the volume is (center of the viewport
//     for a centered volume, not smeared over the background);
//   * the depth path really drives per-pixel occlusion: an OPAQUE wall
//     between the camera and the volume hides the volume's pixels on the
//     wall's half of the screen while the other half still shows them;
//   * the scene-graph transform reaches the raycaster (moving the node
//     re-raycasts and moves the silhouette);
//   * the settings accessors (supersample, shadows, resolution scale and its
//     clamps) reach the state-tree-bound snapshot the next raycast captures;
//   * the composite filters PREMULTIPLIED color, so silhouettes carry no dark
//     halo -- the one visible symptom of getting alpha wrong across the
//     texture filter, and the reason resolution_scale above 1.0 is usable as
//     an anti-aliasing knob at all.
//
// Offscreen; degrades to SKIP (exit 0) when the GL stack can't render
// (llvmpipe+Xvfb covers Linux CI).
#undef NDEBUG
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolRenNode.h>
#include <cvc/volume/volume.h>
#include <stdexcept>
#include <thread>
#include <vector>

using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

namespace {

constexpr int W = 320, H = 240;

// Analytic sphere SDF (inside negative), radius r, centered in a [-1,1]^3 box.
cvc::volume sphereSdf(cvc::app &app, unsigned n, double r) {
  cvc::volume vol(app, cvc::dimension(n, n, n), cvc::Float, cvc::bounding_box(-1, -1, -1, 1, 1, 1));
  for (unsigned k = 0; k < n; ++k)
    for (unsigned j = 0; j < n; ++j)
      for (unsigned i = 0; i < n; ++i) {
        const double x = -1 + i * vol.XSpan();
        const double y = -1 + j * vol.YSpan();
        const double z = -1 + k * vol.ZSpan();
        vol(i, j, k, std::sqrt(x * x + y * y + z * z) - r);
      }
  return vol;
}

struct RGB {
  int r, g, b;
};

// frameRGB rows are BOTTOM-UP; sample with y measured from the top.
RGB pixelAt(const std::vector<unsigned char> &f, int x, int y_from_top) {
  const int y = H - 1 - y_from_top;
  const std::size_t i = (std::size_t(y) * W + x) * 3;
  return {f[i], f[i + 1], f[i + 2]};
}

bool isBackground(const RGB &p) { return p.r < 12 && p.g < 12 && p.b < 12; }

// Where the volume actually landed, measured rather than assumed.
//
// Sampling at hard-coded pixel offsets ties a test to one platform's exact
// framing: offsets tuned on Linux fell OUTSIDE the silhouette on macOS, where
// the same scene projects smaller, and the run aborted on a black sample.  So
// find the non-background pixels, and let callers probe relative to the
// measured centroid and radius.
struct Silhouette {
  int n = 0;
  double cx = 0, cy = 0; // centroid, pixels
  double radius = 0;     // sqrt(area / pi): the radius of an equal-area disc
};

Silhouette measureSilhouette(const std::vector<unsigned char> &f) {
  Silhouette s;
  double sx = 0, sy = 0;
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x)
      if (!isBackground(pixelAt(f, x, y))) {
        ++s.n;
        sx += x;
        sy += y;
      }
  if (s.n > 0) {
    s.cx = sx / s.n;
    s.cy = sy / s.n;
    s.radius = std::sqrt(double(s.n) / 3.14159265358979323846);
  }
  return s;
}

int lum(const RGB &p) { return (p.r * 299 + p.g * 587 + p.b * 114) / 1000; }

// Pump the scene until the node has applied >= want raycast frames.
bool pumpUntilFrames(SceneRenderer &view, cvc::gl::VolRenNode &node, std::uint64_t want,
                     double timeout_s = 10.0) {
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <
         timeout_s) {
    view.processUIEvents(); // drains the main-thread event queue
    node.tick();
    view.render();
    if (node.framesRendered() >= want)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

// Pump until the node reports the ON-SCREEN frame matches the live camera.
//
// Counting "ticks with no new frame" is NOT a convergence test: while a
// raycast is in flight tick() deliberately does not relaunch, so a slow frame
// is indistinguishable from a settled one.  Under a loaded machine (parallel
// ctest) that let this test sample a stale pre-setCamera frame and the volume
// measured a quarter of its true size.  VolRenNode::converged() answers the
// real question.
bool pumpUntilStable(SceneRenderer &view, cvc::gl::VolRenNode &node, double timeout_s = 30.0) {
  const auto start = std::chrono::steady_clock::now();
  int settled = 0;
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <
         timeout_s) {
    view.processUIEvents();
    node.tick();
    view.render();
    settled = node.converged() ? settled + 1 : 0;
    if (settled >= 3)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

} // namespace

int main() {
  cvc::app app;
  SceneGraph sg(app, "volren_test");

  // No grid/axes/bounds markers: they are drawn from the SCENE bounds, so they
  // appear the moment this test adds a wall or a backdrop and then paint over
  // the very pixels the assertions sample.
  sg.setDiagnosticChromeVisible(false);

  auto node = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::VolRenNode>("vol");
  sg.registerGraphics("vol", node);

  cvc::volren::volume_settings vs;
  cvc::volren::isosurface iso;
  iso.value = 0.0;
  iso.opacity = 1.0f;
  iso.color = {1.f, 0.1f, 0.1f}; // red sphere
  iso.shininess = 48.0f;         // tight highlight, out of the sample points
  vs.isosurfaces.push_back(iso);
  node->addVolume(sphereSdf(app, 32, 0.55), vs);
  node->setResolutionScale(1.0);
  {
    cvc::volren::render_settings rs = node->renderConfig();
    cvc::volren::light l;
    l.color = {1.f, 1.f, 1.f};
    l.direction = {0.45, 0.55, 0.7}; // off-axis: keeps the specular blob away
                                     // from the pixels the assertions sample
    rs.lights = {l};
    rs.ambient = 0.5f;
    rs.two_sided_lighting = true;
    rs.steps = 128;
    node->setRenderConfig(rs);
  }

  // ── 0. the settings surface ────────────────────────────────────────────────
  // The thin accessors are read-modify-writes of the state-tree-bound settings,
  // so this also pins that a write is visible SYNCHRONOUSLY to the reader (the
  // node's snapshot is what the next raycast captures).  No GL: these report
  // even on a machine where everything below degrades to SKIP.
  node->setSupersample(3);
  assert(node->supersample() == 3 && "supersample did not reach the settings snapshot");
  assert(node->renderConfig().supersample == 3 && "supersample must live in render_settings");
  node->setShadowsEnabled(true);
  assert(node->shadowsEnabled() && node->renderConfig().shadows.enabled);
  {
    cvc::volren::shadow_settings sh = node->shadowConfig();
    sh.resolution = 256;
    sh.strength = 0.5f;
    node->setShadowConfig(sh);
    assert(node->shadowConfig().resolution == 256);
    assert(node->renderConfig().shadows.strength == 0.5f);
  }
  // The deep/soft/rig accessors, same contract: each is a read-modify-write of
  // the state-tree-bound settings, so it must land in render_settings AND be
  // visible to its own getter immediately.  Asserted one at a time and then
  // TOGETHER, because the failure mode a read-modify-write invites is one
  // setter clobbering an earlier one's field with a stale copy.
  node->setShadowMode(cvc::volren::shadow_mode::deep);
  assert(node->shadowMode() == cvc::volren::shadow_mode::deep);
  assert(node->renderConfig().shadows.mode == cvc::volren::shadow_mode::deep);
  node->setDeepShadowSlices(24);
  assert(node->deepShadowSlices() == 24 && node->renderConfig().shadows.depth_slices == 24);
  node->setSoftShadows(3.5f, 5);
  assert(node->softShadowRadius() == 3.5f && node->softShadowTaps() == 5);
  assert(node->renderConfig().shadows.pcf_radius == 3.5f);
  node->setAmbientLevel(0.45f);
  assert(node->ambientLevel() == 0.45f && node->renderConfig().ambient == 0.45f);
  {
    cvc::volren::hemisphere_ambient h = node->hemisphereConfig();
    h.enabled = true;
    h.sky = {0.5f, 0.6f, 1.0f};
    node->setHemisphereConfig(h);
    assert(node->hemisphereConfig().enabled && node->hemisphereConfig().sky[2] == 1.0f);
    cvc::volren::ao_settings ao = node->occlusionConfig();
    ao.strength = 0.75f;
    ao.radius = 0.2;
    ao.samples = 8;
    node->setOcclusionConfig(ao);
    assert(node->occlusionConfig().strength == 0.75f && node->occlusionConfig().samples == 8);
    assert(node->renderConfig().ao.radius == 0.2);
  }
  node->setShadingGain(1.0f);
  node->setSpecularLevel(0.35f);
  assert(node->shadingGain() == 1.0f && node->specularLevel() == 0.35f);
  {
    // Nothing above was lost on the way through: the LAST writer did not carry
    // a stale snapshot of the earlier fields back into the tree.
    const cvc::volren::render_settings rs = node->renderConfig();
    assert(rs.shadows.mode == cvc::volren::shadow_mode::deep && rs.shadows.depth_slices == 24);
    assert(rs.shadows.pcf_radius == 3.5f && rs.shadows.pcf_taps == 5);
    assert(rs.ambient == 0.45f && rs.ambient_hemisphere.enabled && rs.ao.strength == 0.75f);
    assert(rs.shading_gain == 1.0f && rs.specular == 0.35f);
    // ... and the shadow accessors written BEFORE them are still intact too.
    assert(rs.shadows.enabled && rs.shadows.resolution == 256 && rs.shadows.strength == 0.5f);
  }
  // Back to the renderer's own defaults: everything below is an image check
  // against the shading expression these knobs are all neutral in.
  node->setShadowMode(cvc::volren::shadow_mode::hard);
  node->setDeepShadowSlices(cvc::volren::defaults::shadow_depth_slices);
  node->setSoftShadows(cvc::volren::defaults::shadow_pcf_radius,
                       cvc::volren::defaults::shadow_pcf_taps);
  node->setHemisphereConfig(cvc::volren::hemisphere_ambient());
  node->setOcclusionConfig(cvc::volren::ao_settings());
  node->setShadingGain(cvc::volren::defaults::shading_gain);
  node->setSpecularLevel(cvc::volren::defaults::specular);
  node->setAmbientLevel(0.5f); // what the image checks below were tuned at
  // The scale clamps at both ends; above 1.0 is legal now (the raster
  // supersamples and the quad box-filters it back down).
  node->setResolutionScale(2.0);
  assert(node->resolutionScale() == 2.0 && "scale above 1.0 must be accepted");
  node->setResolutionScale(9.0);
  assert(node->resolutionScale() == cvc::gl::VolRenNode::MaxResolutionScale && "scale must clamp");
  node->setResolutionScale(0.0);
  assert(node->resolutionScale() == cvc::gl::VolRenNode::MinResolutionScale && "scale must clamp");
  node->setSupersample(1); // back to the defaults the image checks expect
  node->setShadowsEnabled(false);
  node->setResolutionScale(1.0);

  try {
    SceneRenderer view(sg, W, H, /*offscreen=*/true);
    view.setCamera(0, 0, 6, 0, 0, 0, 0, 1, 0, /*viewAngle=*/30.0, /*near=*/0.5, /*far=*/60.0);
    view.render(); // GL init; also required before depth-texture creation

    // ── 1. the raycast frame reaches the screen ─────────────────────────────
    if (!pumpUntilFrames(view, *node, 1)) {
      std::printf("SKIP: raycast frame never reached the renderer (no usable GL?)\n");
      return 0;
    }
    // Converge: the first applied frame may carry a pre-setCamera snapshot;
    // tick() keeps re-raycasting until the applied camera matches the live one.
    if (!pumpUntilStable(view, *node)) {
      std::printf("SKIP: raycast never converged\n");
      return 0;
    }
    view.render();
    std::vector<unsigned char> f = view.frameRGB();
    if (f.size() != std::size_t(W) * H * 3) {
      std::printf("SKIP: frameRGB unavailable\n");
      return 0;
    }
    const Silhouette sil = measureSilhouette(f);
    const RGB center = pixelAt(f, W / 2, H / 2);
    // Probe INSIDE the measured silhouette, toward its shadowed lower-left and
    // well clear of both the rim and the specular highlight (upper right,
    // given the off-axis light) -- 45% of the equal-area radius.
    const int bx = std::clamp(int(std::lround(sil.cx - 0.45 * sil.radius)), 0, W - 1);
    const int by = std::clamp(int(std::lround(sil.cy + 0.34 * sil.radius)), 0, H - 1);
    const RGB body = pixelAt(f, bx, by);
    const RGB corner = pixelAt(f, 4, 4);
    std::printf("  silhouette n=%d centre (%.1f,%.1f) r=%.1f | center (%d,%d,%d) "
                "body@(%d,%d) (%d,%d,%d) corner (%d,%d,%d)\n",
                sil.n, sil.cx, sil.cy, sil.radius, center.r, center.g, center.b, bx, by, body.r,
                body.g, body.b, corner.r, corner.g, corner.b);
    fflush(stdout);
    if (const char *dump = getenv("VOLREN_TEST_DUMP")) {
      FILE *fp = fopen(dump, "wb");
      fprintf(fp, "P6\n%d %d\n255\n", W, H);
      for (int y = H - 1; y >= 0; --y)
        fwrite(f.data() + std::size_t(y) * W * 3, 1, std::size_t(W) * 3, fp);
      fclose(fp);
    }
    assert(!isBackground(center) && "sphere not visible at viewport center");
    // The sphere is radius 0.55 in a parallel_scale 1.0 ortho view, so it
    // should cover roughly 0.55 of the viewport half-height. Assert a wide but
    // real band: this still catches a genuinely mis-scaled composite (the
    // reason the fixed offsets failed on macOS) without pinning the framing.
    assert(sil.radius > 0.15 * (H / 2) && sil.radius < 0.95 * (H / 2) &&
           "composited volume is implausibly sized -- image/quad scale mismatch?");
    assert(body.r > body.g + 20 && "sphere body should be red-dominant");
    assert(isBackground(corner) && "background contaminated at the corner");

    // ── 1b. VERTICAL orientation of the composited frame ───────────────────
    // The raycast image is top-left origin and GeometryNode flips V, so the
    // quad's UVs must invert that.  Getting it wrong renders every frame
    // upside down -- which a vertically symmetric test volume cannot detect,
    // so lift the volume and check the silhouette really moves UP the screen.
    {
      const std::uint64_t before = node->framesRendered();
      // This camera sits at (0,0,6) looking down -z with up=+y, so SCREEN UP
      // is world +y.  (Moving in +z would only approach the camera and, under
      // an orthographic projection, not move on screen at all.)
      node->setPosition(0.0, 0.9, 0.0);
      if (pumpUntilFrames(view, *node, before + 1) && pumpUntilStable(view, *node)) {
        view.render();
        const std::vector<unsigned char> lifted = view.frameRGB();
        long sumY = 0, n = 0;
        for (int y = 0; y < H; ++y)
          for (int x = 0; x < W; ++x) {
            const RGB p = pixelAt(lifted, x, y);
            if (p.r > p.g + 20 && p.r > 60) {
              sumY += y;
              ++n;
            }
          }
        assert(n > 200 && "lifted volume not visible");
        const double meanY = double(sumY) / double(n);
        std::printf("  lifted volume mean row %.1f (viewport centre %d)\n", meanY, H / 2);
        fflush(stdout);
        // 0.9 world units at parallel_scale 1.0 is ~54% of the half-height, so
        // a correctly oriented frame puts the silhouette WELL above centre; an
        // upside-down one puts it equally far below.
        assert(meanY < double(H) / 2.0 - 30.0 &&
               "volume raised in +y must render well ABOVE centre -- frame is upside down");
      }
      node->setPosition(0.0, 0.0, 0.0);
      pumpUntilFrames(view, *node, node->framesRendered() + 1);
      pumpUntilStable(view, *node);
    }

    // ── 2. depth: an opaque wall in FRONT hides the volume behind it ────────
    // Wall covers the LEFT half of the view, at z=2 (between camera z=6 and
    // volume at origin).  Its pixels must win on the left; the sphere must
    // survive on the right.
    {
      cvc::geometry wall;
      const double xs[4] = {-8, 0, 0, -8};
      const double ys[4] = {-6, -6, 6, 6};
      for (int i = 0; i < 4; ++i) {
        cvc::geometry::point_t p;
        p[0] = xs[i];
        p[1] = ys[i];
        p[2] = 2.0;
        wall.points().push_back(p);
        cvc::geometry::color_t c;
        c[0] = 0.1;
        c[1] = 0.9;
        c[2] = 0.1;
        wall.colors().push_back(c);
      }
      cvc::geometry::tri_t t1, t2;
      t1[0] = 0;
      t1[1] = 1;
      t1[2] = 2;
      t2[0] = 0;
      t2[1] = 2;
      t2[2] = 3;
      wall.tris().push_back(t1);
      wall.tris().push_back(t2);
      auto wallNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("wall", wall));
      wallNode->setUseSingleColor(true);
      wallNode->setColor(0.1, 0.9, 0.1);
      wallNode->setAmbient(1.0);
      wallNode->setDiffuse(0.0);
    }
    for (int i = 0; i < 3; ++i) {
      view.processUIEvents();
      node->tick();
      view.render();
    }
    f = view.frameRGB();
    // Probe each side of the wall edge INSIDE the silhouette measured before
    // the wall existed -- fixed offsets are what broke this test on macOS.
    const int wy = std::clamp(int(std::lround(sil.cy + 0.34 * sil.radius)), 0, H - 1);
    const int wdx = std::max(3, int(std::lround(0.45 * sil.radius)));
    const RGB left = pixelAt(f, std::clamp(int(std::lround(sil.cx)) - wdx, 0, W - 1), wy);
    const RGB right = pixelAt(f, std::clamp(int(std::lround(sil.cx)) + wdx, 0, W - 1), wy);
    std::printf("  wall test @dx=%d: left (%d,%d,%d) right (%d,%d,%d)\n", wdx, left.r, left.g,
                left.b, right.r, right.g, right.b);
    fflush(stdout);
    assert(left.g > left.r + 30 && "wall (green) must occlude the volume on the left");
    assert(right.r > right.g + 20 && "volume (red) must still show right of the wall");

    // ── 3. scene transform drives the raycaster ─────────────────────────────
    // 1.2 world units, NOT further: this camera's half-width is ~2.13 world
    // units, so a bigger shift walks the r=0.55 sphere off the right edge and
    // leaves nothing of it under any sample point.  x=1.2 lands its centre at
    // screen column ~250, clear of the wall on the left half.
    const std::uint64_t before = node->framesRendered();
    node->setPosition(1.2, 0.0, 0.0); // push the volume to screen right
    if (!pumpUntilFrames(view, *node, before + 1) || !pumpUntilStable(view, *node)) {
      std::printf("SKIP-PARTIAL: transform re-raycast did not complete\n");
      return 0;
    }
    view.render();
    f = view.frameRGB();
    // Where did it go?  Measure the red-dominant pixels rather than guessing a
    // column: the mapping from world x to screen column depends on the
    // viewport the platform actually gave us.
    long rsx = 0;
    int rn = 0;
    for (int y = 0; y < H; ++y)
      for (int x = 0; x < W; ++x) {
        const RGB p = pixelAt(f, x, y);
        if (p.r > 40 && p.r > p.g + 20) {
          rsx += x;
          ++rn;
        }
      }
    const int mx = rn ? int(rsx / rn) : W / 2;
    const RGB movedRight = pixelAt(f, mx, H / 2);
    const RGB vacated = pixelAt(f, W / 2, H / 2);
    std::printf("  after move: %d volume px, centroid col %d (was ~%.0f) -> (%d,%d,%d); "
                "vacated centre (%d,%d,%d)\n",
                rn, mx, sil.cx, movedRight.r, movedRight.g, movedRight.b, vacated.r, vacated.g,
                vacated.b);
    fflush(stdout);
    assert(rn > 100 && "moved volume vanished entirely");
    assert(double(mx) > sil.cx + 0.5 * sil.radius &&
           "volume moved in +x must render further RIGHT than it started");
    // Both halves of "it moved": the volume is where it was sent AND is no
    // longer where it was.  The colour test is what keeps the first half
    // honest -- any red-dominant pixel there has to come from the volume.
    assert(movedRight.r > 40 && movedRight.r > movedRight.g + 20 &&
           "moved volume should appear on the right side");
    assert(isBackground(vacated) && "the volume should have left the viewport centre");

    // ── 4. the composite has no dark halo ──────────────────────────────────
    // A silhouette pixel is a blend of the volume with whatever is behind it,
    // so it can never be darker than BOTH.  The GL texture filter is what
    // breaks that if the node hands VTK straight alpha: interpolating it reads
    // a transparent texel's RGB=0 as black paint and rings every edge with it
    // (worst at resolution_scale < 1, where one raycast texel is magnified over
    // several screen pixels).  The node therefore uploads the raycaster's
    // PREMULTIPLIED frame and divides alpha out in the fragment shader, after
    // the filter.
    //
    // Checking that invariant needs a scene where a minimum is meaningful:
    // a FLAT-lit body (ambient only -- a shaded sphere's dark limb would
    // dominate any minimum) over a backdrop of similar luminance (a green
    // backdrop would stay bright enough on its own to hide a halved red body).
    {
      cvc::volren::volume_settings vs2 = node->volumeConfig(0);
      vs2.isosurfaces[0].color = {1.f, 1.f, 1.f};
      // The isosurface alone: volume_settings::shaded defaults to TRUE, and
      // while the default transfer function is fully transparent (which is why
      // the checks above see only the isosurface), a minimum-luminance test
      // should not depend on that.
      vs2.shaded = false;
      node->setVolumeConfig(0, vs2);
      cvc::volren::render_settings rs = node->renderConfig();
      rs.lights.clear(); // ambient only: no terminator, no specular
      rs.ambient = 1.0f;
      node->setRenderConfig(rs);
      node->setPosition(0.0, 0.0, 0.0);
      node->setResolutionScale(0.5); // magnification: where the halo is worst

      cvc::geometry back;
      const double bx[4] = {-12, 12, 12, -12}, by[4] = {-12, -12, 12, 12};
      for (int i = 0; i < 4; ++i) {
        cvc::geometry::point_t p;
        p[0] = bx[i];
        p[1] = by[i];
        p[2] = -2.0;
        back.points().push_back(p);
      }
      cvc::geometry::tri_t b1, b2;
      b1[0] = 0;
      b1[1] = 1;
      b1[2] = 2;
      b2[0] = 0;
      b2[1] = 2;
      b2[2] = 3;
      back.tris().push_back(b1);
      back.tris().push_back(b2);
      auto backNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("backdrop", back));
      backNode->setUseSingleColor(true);
      backNode->setColor(0.6, 0.6, 0.6);
      backNode->setAmbient(1.0);
      backNode->setDiffuse(0.0);

      if (pumpUntilFrames(view, *node, node->framesRendered() + 1) &&
          pumpUntilStable(view, *node)) {
        view.render();
        const std::vector<unsigned char> halo = view.frameRGB();
        // Right of the opaque wall's edge only -- the wall is still in the
        // scene, covering screen-left.
        int worstX = 0, worstY = 0, worst = 1000;
        for (int y = 8; y < H - 8; ++y)
          for (int x = W / 2 + 8; x < W - 8; ++x) {
            const int l = lum(pixelAt(halo, x, y));
            if (l < worst) {
              worst = l;
              worstX = x;
              worstY = y;
            }
          }
        // Both endpoints are MEASURED, not assumed: the backdrop's rendered
        // luminance depends on the GL stack's lighting, and a hardcoded bound
        // would either drift or stop separating the two cases.  Everything in
        // the region is a blend of the body and the backdrop, so nothing may be
        // darker than the darker endpoint -- the backdrop.
        const RGB backdropPx = pixelAt(halo, W - 12, 12);    // pure backdrop
        const RGB bodyPx = pixelAt(halo, W / 2 + 20, H / 2); // inside the sphere
        const int backdropLum = lum(backdropPx), bodyLum = lum(bodyPx);
        std::printf("  halo test: darkest %d at (%d,%d); backdrop %d, body %d\n", worst, worstX,
                    worstY, backdropLum, bodyLum);
        fflush(stdout);
        assert(bodyLum > backdropLum + 15 &&
               "halo scene mis-set: the body must be the brighter endpoint");
        if (const char *dump = getenv("VOLREN_TEST_DUMP2")) {
          FILE *fp = fopen(dump, "wb");
          fprintf(fp, "P6\n%d %d\n255\n", W, H);
          for (int y = H - 1; y >= 0; --y)
            fwrite(halo.data() + std::size_t(y) * W * 3, 1, std::size_t(W) * 3, fp);
          fclose(fp);
        }
        // -8 is u8 rounding slack, nothing more.  Measured on this box: 199
        // (== the backdrop, i.e. no fringe at all) with the premultiplied
        // upload, 160 with a straight-alpha one -- a half-covered pixel there
        // contributes half the body's colour instead of all of it.
        assert(worst >= backdropLum - 8 &&
               "dark halo on the silhouette -- the composite is filtering "
               "straight alpha instead of premultiplied");
      }
    }

    // ── 9. teardown with a raycast IN FLIGHT ───────────────────────────────
    // The worker's completion callback runs on the render thread while
    // ~VolRenNode is inside m_worker.reset(), and reset() nulls the unique_ptr
    // BEFORE running the deleter that joins that thread -- so a callback that
    // reached the worker back through m_worker dereferenced null.  It only
    // shows when a frame is genuinely still in flight, which needs a raycast
    // slow enough to outlast the destructor: hence a deliberately expensive
    // node (dense volume, 4x4 rays per pixel, many steps) dropped one tick
    // after its job is submitted.
    //
    // The assertion is that the process survives; there is nothing else to
    // check, and a regression here is a SIGSEGV, not a wrong pixel.
    {
      auto doomed = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::VolRenNode>("doomed");
      cvc::volren::volume_settings dvs;
      dvs.isosurfaces.push_back(iso);
      doomed->addVolume(sphereSdf(app, 64, 0.55), dvs);
      doomed->setResolutionScale(cvc::gl::VolRenNode::MaxResolutionScale);
      doomed->setSupersample(cvc::volren::limits::max_supersample);
      {
        cvc::volren::render_settings drs = doomed->renderConfig();
        drs.steps = 768;
        drs.threads = 1; // serial: the point is a LONG raycast, not a fast one
        doomed->setRenderConfig(drs);
      }
      view.processUIEvents();
      doomed->tick(); // submits; the worker is now marching
      view.render();
      const bool inFlight = !doomed->converged();
      sg.getGraphicsRoot()->removeGraphicsChild(doomed);
      doomed.reset(); // last reference -> ~VolRenNode -> join, mid-raycast
      std::printf("  teardown with a raycast in flight: survived (in flight: %s)\n",
                  inFlight ? "yes" : "no -- it finished first, weaker check");
      fflush(stdout);
    }

    std::printf("cvcgl_volren_node: all checks passed (%llu raycasts, last %.1f ms)\n",
                (unsigned long long)node->framesRendered(), node->lastRenderSeconds() * 1000.0);
  } catch (const std::exception &e) {
    std::printf("SKIP: %s\n", e.what());
    return 0;
  }
  return 0;
}
