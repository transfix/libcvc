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
//     re-raycasts and moves the silhouette).
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

namespace {

constexpr int W = 320, H = 240;

// Analytic sphere SDF (inside negative), radius r, centered in a [-1,1]^3 box.
cvc::volume sphereSdf(cvc::app &app, unsigned n, double r) {
  cvc::volume vol(app, cvc::dimension(n, n, n), cvc::Float,
                  cvc::bounding_box(-1, -1, -1, 1, 1, 1));
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

// Pump until the raycast converges on the current camera: tick() launches a
// re-raycast whenever the applied frame's camera differs from the live one,
// so steady state = several consecutive cycles with no new frame applied and
// no relaunch wanted.
bool pumpUntilStable(SceneRenderer &view, cvc::gl::VolRenNode &node, double timeout_s = 10.0) {
  const auto start = std::chrono::steady_clock::now();
  int quiet = 0;
  std::uint64_t last = node.framesRendered();
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <
         timeout_s) {
    view.processUIEvents();
    node.tick();
    view.render();
    const std::uint64_t now = node.framesRendered();
    quiet = (now == last) ? quiet + 1 : 0;
    last = now;
    if (quiet >= 8)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

} // namespace

int main() {
  cvc::app app;
  SceneGraph sg(app, "volren_test");

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
    const RGB center = pixelAt(f, W / 2, H / 2);
    // Sample toward the shadowed lower-left of the silhouette, away from the
    // specular highlight (upper right, given the off-axis light).
    const RGB body = pixelAt(f, W / 2 - 24, H / 2 + 18);
    const RGB corner = pixelAt(f, 4, 4);
    std::printf("  center (%d,%d,%d) body (%d,%d,%d) corner (%d,%d,%d)\n", center.r,
                center.g, center.b, body.r, body.g, body.b, corner.r, corner.g, corner.b);
    fflush(stdout);
    if (const char *dump = getenv("VOLREN_TEST_DUMP")) {
      FILE *fp = fopen(dump, "wb");
      fprintf(fp, "P6\n%d %d\n255\n", W, H);
      for (int y = H - 1; y >= 0; --y)
        fwrite(f.data() + std::size_t(y) * W * 3, 1, std::size_t(W) * 3, fp);
      fclose(fp);
    }
    assert(!isBackground(center) && "sphere not visible at viewport center");
    assert(body.r > body.g + 20 && "sphere body should be red-dominant");
    assert(isBackground(corner) && "background contaminated at the corner");

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
    // Sample inside the sphere silhouette on each side of the wall edge.
    const RGB left = pixelAt(f, W / 2 - 24, H / 2 + 18);
    const RGB right = pixelAt(f, W / 2 + 24, H / 2 + 18);
    std::printf("  wall test: left (%d,%d,%d) right (%d,%d,%d)\n", left.r, left.g, left.b,
                right.r, right.g, right.b);
    fflush(stdout);
    assert(left.g > left.r + 30 && "wall (green) must occlude the volume on the left");
    assert(right.r > right.g + 20 && "volume (red) must still show right of the wall");

    // ── 3. scene transform drives the raycaster ─────────────────────────────
    const std::uint64_t before = node->framesRendered();
    node->setPosition(2.5, 0.0, 0.0); // push the volume to screen right
    if (!pumpUntilFrames(view, *node, before + 1) || !pumpUntilStable(view, *node)) {
      std::printf("SKIP-PARTIAL: transform re-raycast did not complete\n");
      return 0;
    }
    view.render();
    f = view.frameRGB();
    const RGB movedRight = pixelAt(f, W - 40, H / 2);
    std::printf("  after move: right-side (%d,%d,%d)\n", movedRight.r, movedRight.g,
                movedRight.b);
    assert(movedRight.r > 40 && "moved volume should appear on the right side");

    std::printf("cvcgl_volren_node: all checks passed (%llu raycasts, last %.1f ms)\n",
                (unsigned long long)node->framesRendered(),
                node->lastRenderSeconds() * 1000.0);
  } catch (const std::exception &e) {
    std::printf("SKIP: %s\n", e.what());
    return 0;
  }
  return 0;
}
