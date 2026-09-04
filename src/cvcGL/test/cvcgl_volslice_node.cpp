// VolSliceNode end-to-end render tests (offscreen; llvmpipe-friendly).
//
// What is asserted is the RENDERED IMAGE, with tolerance bands, following
// cvcgl_volren_node.cpp: silhouette measurements instead of hard-coded pixel
// offsets, and every threshold derived from a prediction stated in a comment.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolSliceNode.h>
#include <cvc/volume/volume.h>
#include <string>
#include <vector>
#include <vtkRenderer.h>

using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::VolSliceNode;

namespace {

constexpr int W = 200, H = 200;

struct RGB {
  int r, g, b;
};

RGB pixelAt(const std::vector<unsigned char> &f, int x, int y_from_top) {
  const int y = H - 1 - y_from_top; // frameRGB is bottom-up
  const unsigned char *p = &f[(std::size_t(y) * W + x) * 3];
  return {p[0], p[1], p[2]};
}

bool isBackground(const RGB &p) { return p.r < 12 && p.g < 12 && p.b < 12; }

struct Silhouette {
  int n = 0;
  double cx = 0, cy = 0, radius = 0;
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

// A radial density ball: 1 at the center falling linearly to 0 at r=0.8 (of
// the box half-extent), 0 outside.  Node-centered grid over [-1,1]^3.
cvc::volume makeBallVolume(cvc::app &ctx, unsigned n,
                           cvc::bounding_box box = cvc::bounding_box(-1, -1, -1, 1, 1, 1)) {
  cvc::volume vol(ctx, cvc::dimension(n, n, n), cvc::Float, box);
  for (unsigned k = 0; k < n; ++k)
    for (unsigned j = 0; j < n; ++j)
      for (unsigned i = 0; i < n; ++i) {
        const double x = -1.0 + 2.0 * i / (n - 1);
        const double y = -1.0 + 2.0 * j / (n - 1);
        const double z = -1.0 + 2.0 * k / (n - 1);
        const double r = std::sqrt(x * x + y * y + z * z);
        vol(i, j, k, r >= 0.8 ? 0.0 : 1.0 - r / 0.8);
      }
  vol.min(0.0);
  vol.max(1.0);
  return vol;
}

// Red TF: transparent at density 0, opacity `amax` at density 1.
cvc::volslice::render_settings redSettings(float amax) {
  cvc::volslice::render_settings s;
  s.tf.add({0.0, 1.f, 0.f, 0.f, 0.f});
  s.tf.add({1.0, 1.f, 0.f, 0.f, amax});
  return s;
}

// Let the synchronous pipeline settle: state handlers, slice recompute, draw.
void pump(SceneGraph &sg, SceneRenderer &view, VolSliceNode &node, int frames = 4) {
  for (int i = 0; i < frames; ++i) {
    sg.processEvents();
    node.tick();
    view.render();
  }
}

int lum(const RGB &p) { return (p.r * 299 + p.g * 587 + p.b * 114) / 1000; }

} // namespace

int main() {
  setvbuf(stdout, nullptr, _IONBF, 0); // assertions must not eat the trail
  cvc::app app;
  SceneGraph sg(app, "volslice_test");
  sg.setDiagnosticChromeVisible(false);

  auto node = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::VolSliceNode>("vol");
  sg.registerGraphics("vol", node);
  node->setVolume(makeBallVolume(app, 32));
  node->setConfig(redSettings(0.35f));

  SceneRenderer view(sg, W, H, /*offscreen=*/true);
  view.setCamera(0, 0, 6, 0, 0, 0, 0, 1, 0, /*viewAngle=*/30.0, /*near=*/0.5, /*far=*/60.0);

  // ---- The OIT switch: adding the node must have flipped the renderer to
  // sequential translucency (the header's scene-wide effect).  Without it the
  // slice stack averages instead of compositing and every assertion below
  // about opacity accumulation is meaningless.
  pump(sg, view, *node);
  assert(view.renderer() != nullptr);
  assert(!view.renderer()->GetUseOIT() && "VolSliceNode must disable OIT on its renderer");

  // ---- A red ball, composited from slices.
  {
    std::vector<unsigned char> f = view.frameRGB();
    const Silhouette sil = measureSilhouette(f);
    // Predicted silhouette: ball radius 0.8 of half-extent 1, camera at 6 with
    // vfov 30 deg -> half-height = 6*tan(15) = 1.607 world units over H/2 px;
    // visible radius where the TF alpha is still meaningful is a bit inside
    // 0.8, so expect ~40-55 px.
    std::printf("silhouette: n=%d c=(%.1f,%.1f) r=%.1f\n", sil.n, sil.cx, sil.cy, sil.radius);
    assert(sil.n > 1000 && "the volume must be visible");
    assert(std::fabs(sil.cx - W / 2) < 6 && std::fabs(sil.cy - H / 2) < 6);
    assert(sil.radius > 30 && sil.radius < 65);

    // Center: hundreds of slices at rising alpha -> saturated toward the TF
    // color (red), and PURELY red (the TF has no green/blue anywhere).
    const RGB center = pixelAt(f, W / 2, H / 2);
    std::printf("center: %d %d %d\n", center.r, center.g, center.b);
    assert(center.r > 150 && "slice stack must accumulate toward opaque red");
    assert(center.g < 30 && center.b < 30);
    assert(std::size_t(node->planesRendered()) > 200); // q=0.5 default -> ~270*sqrt(3)/... >200
  }

  // ---- Quality drives the plane count (the arand formula, end to end).
  {
    auto s = node->config();
    s.slices.quality = 0.0; // N = 20
    node->setConfig(s);
    pump(sg, view, *node);
    const std::size_t low = node->planesRendered();
    s.slices.quality = 0.5;
    node->setConfig(s);
    pump(sg, view, *node);
    const std::size_t mid = node->planesRendered();
    std::printf("planes: q=0 -> %zu, q=0.5 -> %zu\n", low, mid);
    assert(low >= 10 && low <= 40);
    assert(mid > 5 * low);
  }

  // ---- The node follows its scene-graph transform.
  {
    node->setPosition(1.2, 0, 0);
    pump(sg, view, *node);
    std::vector<unsigned char> f = view.frameRGB();
    const Silhouette sil = measureSilhouette(f);
    // 1.2 world units right = 1.2/1.607 * 100 px ~ 75 px.
    std::printf("moved silhouette: c=(%.1f,%.1f)\n", sil.cx, sil.cy);
    assert(sil.n > 500);
    assert(sil.cx > W / 2 + 40 && "silhouette must follow the node transform");
    node->setPosition(0, 0, 0);
    pump(sg, view, *node);
  }

  // ---- near_plane cuts the viewer side of the stack: the remaining slices
  // are only the far part, so the accumulated center opacity must DROP.
  // Runs on a LOW-alpha TF: at amax=0.35 even 40%% of the sweep saturates the
  // center to full red, and the cut is invisible (measured, the original
  // version of this test asserted exactly that mistake).
  {
    node->setConfig(redSettings(0.04f));
    pump(sg, view, *node);
    std::vector<unsigned char> before = view.frameRGB();
    auto s = node->config();
    s.slices.near_plane = 0.6;
    node->setConfig(s);
    pump(sg, view, *node);
    std::vector<unsigned char> after = view.frameRGB();
    const int lb = lum(pixelAt(before, W / 2, H / 2));
    const int la = lum(pixelAt(after, W / 2, H / 2));
    std::printf("near_plane: center lum %d -> %d\n", lb, la);
    assert(la < lb - 15 && "cutting 60%% of the sweep must dim the composite");
    s.slices.near_plane = 0.0;
    node->setConfig(s);
    pump(sg, view, *node);
  }

  // ---- Opacity correction: with it ON the composite is stable across
  // quality; OFF (the faithful legacy behavior), more slices = denser image.
  // Low TF alpha so neither setting saturates.
  {
    auto s = node->config();

    s.opacity_correction = false;
    s.slices.quality = 0.2;
    node->setConfig(s);
    pump(sg, view, *node);
    const int off_low = lum(pixelAt(view.frameRGB(), W / 2, H / 2));
    s.slices.quality = 0.9;
    node->setConfig(s);
    pump(sg, view, *node);
    const int off_high = lum(pixelAt(view.frameRGB(), W / 2, H / 2));

    s.opacity_correction = true;
    s.slices.quality = 0.2;
    node->setConfig(s);
    pump(sg, view, *node);
    const int on_low = lum(pixelAt(view.frameRGB(), W / 2, H / 2));
    s.slices.quality = 0.9;
    node->setConfig(s);
    pump(sg, view, *node);
    const int on_high = lum(pixelAt(view.frameRGB(), W / 2, H / 2));

    std::printf("opacity correction: off %d->%d, on %d->%d\n", off_low, off_high, on_low, on_high);
    // Legacy behavior: quality visibly changes density (the known artifact).
    assert(off_high > off_low + 15);
    // Corrected: the quality-dependence must shrink substantially.  It cannot
    // vanish: the composite accumulates through an 8-bit framebuffer, and at
    // high quality the corrected per-slice alpha is tiny (~0.007 at q=0.9
    // here), so ~1500 sequential blend roundings systematically lose
    // brightness (measured off 32->72 vs on 74->57 on llvmpipe).  The legacy
    // renderer had the same characteristic -- one reason its transfer
    // functions used chunky alphas.
    assert(std::abs(on_high - on_low) < (off_high - off_low) * 2 / 3);
  }

  // ---- REGRESSION: a volume whose bounding box is NOT centered on the
  // origin.  VTK's mapper AUTO shift-scale re-centers VBO coordinates for
  // boxes offset from the origin and compensates in MCDCMatrix -- which
  // silently breaks a shader that derives texcoords from vertexMC.  The node
  // disables shift-scale; this is the scene that caught it (the demo bunny,
  // z in [0,100], rendered ZERO pixels while the origin-centered ball above
  // passed).
  {
    node->setConfig(redSettings(0.35f));
    node->setVolume(makeBallVolume(app, 32, cvc::bounding_box(-1, 0.2, -1, 1, 2.2, 1)));
    pump(sg, view, *node);
    std::vector<unsigned char> f = view.frameRGB();
    const Silhouette sil = measureSilhouette(f);
    std::printf("off-origin silhouette: n=%d c=(%.1f,%.1f)\n", sil.n, sil.cx, sil.cy);
    assert(sil.n > 1000 && "an off-origin volume must still render (VBO shift-scale)");
    // The box center is at y=+1.2 and screen up is +y, so the silhouette
    // must sit ABOVE the image center.
    assert(sil.cy < H / 2 - 10);
    // And it is still RED: texcoords must address the volume, not a corner.
    const RGB p = pixelAt(f, int(sil.cx), int(sil.cy));
    assert(p.r > 100 && p.g < 40 && p.b < 40);
  }

  // ---- Two slice nodes in one scene: does the TRANSLUCENT PASS order the
  // PROPS back to front?  Each node's own slices are ordered by construction,
  // but node-vs-node ordering is VTK's job once UseOIT is off.  A green ball
  // in front of a red ball (disjoint boxes along the view axis): if props
  // render back to front, the center composites green-over-red (green wins);
  // if they render in ADD order with green added first, red blends on top.
  // The demo's multi-bunny grid depends on the answer.
  {
    node->setVisible(false); // retire the ball from the sections above

    auto back = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::VolSliceNode>("back");
    sg.registerGraphics("back", back);
    back->setVolume(makeBallVolume(app, 32, cvc::bounding_box(-1, -1, -2.2, 1, 1, -0.2)));
    back->setConfig(redSettings(0.6f));

    // Deliberately added SECOND: green is nearer, so add order (green after
    // red? no -- back was added first) disagrees with depth order only if
    // the sort is missing.  To make the failure mode unambiguous, the FRONT
    // node is added first in a second scene... keeping one scene, the front
    // node added AFTER the back one matches both hypotheses, so instead add
    // front FIRST is impossible here; assert the composite is green-dominant
    // and leave the add-order permutation to the swap below.
    auto front = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::VolSliceNode>("front");
    sg.registerGraphics("front", front);
    front->setVolume(makeBallVolume(app, 32, cvc::bounding_box(-1, -1, 0.2, 1, 1, 2.2)));
    {
      cvc::volslice::render_settings s;
      s.tf.add({0.0, 0.f, 1.f, 0.f, 0.f});
      s.tf.add({1.0, 0.f, 1.f, 0.f, 0.6f});
      front->setConfig(s);
    }

    for (int i = 0; i < 4; ++i) {
      sg.processEvents();
      back->tick();
      front->tick();
      view.render();
    }
    std::vector<unsigned char> f = view.frameRGB();
    const RGB center = pixelAt(f, W / 2, H / 2);
    std::printf("two-node center (green in front of red): %d %d %d\n", center.r, center.g,
                center.b);
    // Green is nearer: correct prop ordering composites green over red.
    assert(center.g > center.r && "front slice node must composite over the back one");

    // The adversarial permutation: make the NEARER volume the FIRST-added
    // node ('back' hops in front).  If VTK only renders translucent props in
    // add order, the second-added 'front' node would now wrongly paint over
    // the nearest volume.
    back->setVolume(makeBallVolume(app, 32, cvc::bounding_box(-1, -1, 2.4, 1, 1, 4.4)));
    for (int i = 0; i < 4; ++i) {
      sg.processEvents();
      back->tick();
      front->tick();
      view.render();
    }
    f = view.frameRGB();
    const RGB unsortedCenter = pixelAt(f, W / 2, H / 2);
    std::printf("two-node center (red hops in front, NO sort): %d %d %d\n", unsortedCenter.r,
                unsortedCenter.g, unsortedCenter.b);
    // MEASURED VTK behavior, pinned so a change in VTK is noticed: without
    // help, translucent props render in ADD order -- the later-added green
    // node still paints over the now-nearer red one.  This is exactly why
    // depthSortSliceProps() exists.
    assert(unsortedCenter.g > unsortedCenter.r && "VTK renders translucent props in add order");

    // With the per-frame sort, depth wins: red is nearest, red shows.
    for (int i = 0; i < 4; ++i) {
      sg.processEvents();
      back->tick();
      front->tick();
      VolSliceNode::depthSortSliceProps(view.renderer(), {back.get(), front.get()});
      view.render();
    }
    f = view.frameRGB();
    const RGB sorted = pixelAt(f, W / 2, H / 2);
    std::printf("two-node center (red in front, sorted): %d %d %d\n", sorted.r, sorted.g, sorted.b);
    assert(sorted.r > sorted.g && "depthSortSliceProps must restore depth ordering");
  }

  std::printf("cvcgl_volslice_node: all assertions passed\n");
  return 0;
}
