// SceneRenderer: the properties that make a render target reusable across
// frames, as opposed to a one-shot screenshot helper.
//
// What is actually being pinned here:
//   * a sequence of frames costs ONE GL context, not one per frame;
//   * successive frames differ when the scene changes (the classic failure is
//     a cached vtkWindowToImageFilter handing back frame 0 forever);
//   * an explicitly set camera SURVIVES the next render instead of being
//     reset by it, which is what a scripted fly-through depends on;
//   * geometry added after construction shows up without re-attaching;
//   * close() is idempotent and use-after-close is diagnosed, not a crash.
//
// Offscreen throughout, so this runs headless in CI.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void addQuad(SceneGraph &sg, const std::string &name, double x0, double y0, double x1, double y1,
             double z) {
  cvc::geometry g;
  const double xs[4] = {x0, x1, x1, x0};
  const double ys[4] = {y0, y0, y1, y1};
  for (int i = 0; i < 4; ++i) {
    cvc::geometry::point_t p;
    p[0] = xs[i];
    p[1] = ys[i];
    p[2] = z;
    g.points().push_back(p);
  }
  const int idx[2][3] = {{0, 1, 2}, {0, 2, 3}};
  for (auto &tri : idx) {
    cvc::geometry::tri_t t;
    t[0] = tri[0];
    t[1] = tri[1];
    t[2] = tri[2];
    g.tris().push_back(t);
  }
  sg.addGraphics(name, g);
}

// Fraction of bytes that differ between two frames.
double frameDelta(const std::vector<unsigned char> &a, const std::vector<unsigned char> &b) {
  assert(a.size() == b.size());
  if (a.empty())
    return 0.0;
  size_t diff = 0;
  for (size_t i = 0; i < a.size(); ++i)
    if (a[i] != b[i])
      ++diff;
  return static_cast<double>(diff) / static_cast<double>(a.size());
}

} // namespace

int main() {
  SceneGraph sg;
  addQuad(sg, "ground", -50, -50, 50, 50, 0.0);

  SceneRenderer r(sg, 320, 240, /*offscreen=*/true);
  assert(!r.isClosed());
  assert(r.frameWidth() == 320);
  assert(r.frameHeight() == 240);
  // Offscreen has no window, so it can never report a closed one.
  assert(!r.windowClosed());

  // ── many frames, one context ──────────────────────────────────────────────
  // If a context were built per frame this loop would be the slow path the
  // class exists to remove; correctness-wise what matters is that it simply
  // works repeatedly against a live renderer.
  for (int i = 0; i < 30; ++i)
    r.render();

  std::vector<unsigned char> f0 = r.frameRGB();
  assert(f0.size() == static_cast<size_t>(320) * 240 * 3);

  // ── the scene can change under a live renderer ────────────────────────────
  // A node added AFTER attachment must appear without re-attaching the scene.
  addQuad(sg, "marker", -10, -10, 10, 10, 5.0);
  std::vector<unsigned char> f1 = r.frameRGB();
  assert(f1.size() == f0.size());
  const double added = frameDelta(f0, f1);
  printf("  frame delta after adding geometry: %.4f\n", added);
  assert(added > 0.0 && "a node added after attach never reached the renderer");

  // ── frames are not cached ─────────────────────────────────────────────────
  // Move the camera and the picture must change. The bug this guards is a
  // vtkWindowToImageFilter that executes once and returns frame 0 forever.
  r.setCamera(0, 0, 400, 0, 0, 0, 0, 1, 0);
  std::vector<unsigned char> top = r.frameRGB();
  r.setCamera(300, 0, 120, 0, 0, 0, 0, 0, 1);
  std::vector<unsigned char> oblique = r.frameRGB();
  const double moved = frameDelta(top, oblique);
  printf("  frame delta after moving the camera: %.4f\n", moved);
  assert(moved > 0.0 && "frames are cached: the camera moved and the pixels did not");

  // ── an explicit camera survives the next render ───────────────────────────
  // The one-shot helpers call ResetCamera() on every frame, so a scripted
  // camera is overwritten before it is ever seen. Render twice and require the
  // picture to be identical: nothing may re-frame behind the caller's back.
  r.setCamera(300, 0, 120, 0, 0, 0, 0, 0, 1);
  std::vector<unsigned char> a = r.frameRGB();
  std::vector<unsigned char> b = r.frameRGB();
  const double drift = frameDelta(a, b);
  printf("  frame delta across two renders with a fixed camera: %.4f\n", drift);
  assert(drift == 0.0 && "something re-framed the camera between identical frames");

  // resetCamera is still available when auto-framing IS what you want.
  r.resetCamera();
  std::vector<unsigned char> framed = r.frameRGB();
  assert(frameDelta(framed, a) > 0.0 && "resetCamera did not change the view");

  // ── resize ────────────────────────────────────────────────────────────────
  r.resize(160, 120);
  std::vector<unsigned char> small = r.frameRGB();
  assert(r.frameWidth() == 160 && r.frameHeight() == 120);
  assert(small.size() == static_cast<size_t>(160) * 120 * 3);

  bool threw = false;
  try {
    r.resize(0, 100);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw && "a zero-width resize must be rejected, not silently accepted");

  // ── lifetime ──────────────────────────────────────────────────────────────
  r.close();
  assert(r.isClosed());
  r.close(); // idempotent

  threw = false;
  try {
    r.render();
  } catch (const std::runtime_error &) {
    threw = true;
  }
  assert(threw && "use-after-close must be diagnosed rather than crash");

  // The scene outlives its renderer, and a second one can attach to it.
  {
    SceneRenderer again(sg, 64, 64, /*offscreen=*/true);
    again.render();
    assert(again.frameWidth() == 64);
  } // destructor closes it

  // ...and a third, proving nothing global was consumed by the first two.
  {
    SceneRenderer third(sg, 48, 48, /*offscreen=*/true);
    third.render();
  }

  // Bad construction arguments are rejected.
  threw = false;
  try {
    SceneRenderer bad(sg, 0, 10);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw && "a zero-width renderer must be rejected");

  // ── the VTK handles are reachable ─────────────────────────────────────────
  // Callers need these to light the scene, overlay a HUD, or set a gradient
  // background. Returning them is only half the job -- pycvc_gl bridges them
  // into live vtkmodules objects so Python can use them too.
  {
    SceneRenderer h(sg, 64, 64, /*offscreen=*/true);
    assert(h.renderer() != nullptr);
    assert(h.renderWindow() != nullptr);
    h.render();
    h.close();
    bool threw = false;
    try {
      (void)h.renderer();
    } catch (const std::runtime_error &) {
      threw = true;
    }
    assert(threw && "renderer() after close must be diagnosed");
  }

  printf("cvcgl_renderer: OK\n");
  return 0;
}
