// Regression guard: enabling shadows must not hide VOLUMES.
//
// Shadows in cvcGL are a render-PASS sequence installed on the renderer via
// SetPass(): a shadow-map baker plus a shadow pass that draws the OPAQUE layer
// with shadows. On its own that sequence renders nothing else -- translucent
// geometry, volumes and 2-D overlays are all dropped. So the moment a scene with
// a VolumeNode (the sea, a cloud slab, any transfer-function volume) switched
// shadows on, every volume silently vanished: the field was right, the transfer
// function was right, the volume was in the renderer and marked visible, and it
// still drew nothing. SceneGraph::setShadowsEnabled now follows the shadow pass
// with the rest of VTK's standard layer order (translucent -> volumetric ->
// overlay), so the full scene draws with shadows on.
//
// Two halves, deliberately independent:
//   * STRUCTURAL -- the installed pass tree must contain a vtkVolumetricPass.
//     This is the exact regression and needs no GPU, so it guards even in a
//     headless CI where the offscreen render below is a no-op.
//   * END-TO-END -- a solid white volume, framed and rendered offscreen, must
//     paint bright pixels with shadows ON just as it does with shadows OFF.
//     Skipped automatically when the build can't actually rasterise.
//
// cvcpkg builds tests Release, where NDEBUG makes assert() expand to nothing and
// every check would pass vacuously. Undefine it before <cassert>.
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/volume/volume.h>
#include <vector>
#include <vtkCameraPass.h>
#include <vtkRenderPass.h>
#include <vtkRenderPassCollection.h>
#include <vtkRenderer.h>
#include <vtkSequencePass.h>
#include <vtkVolumetricPass.h>

namespace {

// Walk a render-pass tree (cameraPass -> sequencePass -> {...}) for a volumetric
// pass. The shadow sequence nests one camera pass around one sequence pass; this
// stays robust if that nesting is reshuffled.
bool passTreeHasVolumetric(vtkRenderPass *p) {
  if (!p)
    return false;
  if (vtkVolumetricPass::SafeDownCast(p))
    return true;
  if (auto *cam = vtkCameraPass::SafeDownCast(p))
    return passTreeHasVolumetric(cam->GetDelegatePass());
  if (auto *seq = vtkSequencePass::SafeDownCast(p)) {
    if (vtkRenderPassCollection *passes = seq->GetPasses()) {
      passes->InitTraversal();
      while (vtkRenderPass *child = passes->GetNextRenderPass())
        if (passTreeHasVolumetric(child))
          return true;
    }
  }
  return false;
}

// A dark opaque ground quad (with normals, so the shadow shader compiles) — the
// realistic case is a volume floating over lit geometry, and it gives the shadow
// baker something to render. Kept dark so it never trips the "bright" pixel test.
cvc::geometry ground(cvc::app &app) {
  cvc::geometry g(app);
  const double xy[4][2] = {{-3, -3}, {3, -3}, {3, 3}, {-3, 3}};
  for (auto &c : xy) {
    g.points().push_back({c[0], c[1], -1.4});
    g.colors().push_back({0.12, 0.12, 0.14});
  }
  g.tris().push_back({0, 1, 2});
  g.tris().push_back({0, 2, 3});
  return g;
}

// Count bright (near-white) pixels: the volume is pure white, the ground is dark,
// the background is black — so this isolates the volume's contribution.
long brightPixels(SceneRenderer &sr) {
  std::vector<unsigned char> px = sr.frameRGB();
  long n = 0;
  for (std::size_t i = 0; i + 2 < px.size(); i += 3)
    if (px[i] > 150 && px[i + 1] > 150 && px[i + 2] > 150)
      ++n;
  return n;
}

void test_shadows_do_not_hide_volumes() {
  cvc::app app;
  SceneGraph sg(app, "shadowvol");
  sg.addDirectionalLight(-52.0, 34.0);
  sg.addGraphics("ground", ground(app));

  // A solid, opaque, pure-white constant-density cube centred at the origin.
  const int N = 16;
  std::vector<float> field(static_cast<std::size_t>(N) * N * N, 1.0f);
  cvc::volume vol(app, reinterpret_cast<const unsigned char *>(field.data()),
                  cvc::dimension(N, N, N), cvc::Float, cvc::bounding_box(-1, -1, -1, 1, 1, 1));
  auto cube = sg.addGraphics("cube", vol);
  cube->setTransferFunction({0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {0.0, 1.0, 1.0, 1.0});
  cube->setShading(false); // paint the transfer-function colour flat, unlit

  SceneRenderer sr(sg, 128, 128, /*offscreen=*/true);
  sr.setBackground(0.0, 0.0, 0.0);
  sr.resetCamera();

  // Baseline with shadows OFF. If this build cannot rasterise (no GPU/context),
  // it is 0 and the end-to-end half below simply can't speak — the structural
  // half still catches the regression.
  const long litNoShadow = brightPixels(sr);

  const bool on = sg.setShadowsEnabled(true);
  if (!on) {
    std::printf("  ok: shadows declined on this build; volume-visibility check skipped\n");
    return;
  }

  // STRUCTURAL: the installed pass tree must run a volumetric pass.
  assert(passTreeHasVolumetric(sr.renderer()->GetPass()));
  std::printf("  ok: the shadow pass sequence carries a volumetric pass\n");

  // END-TO-END: the cube must still paint with shadows on. Before the fix this
  // collapsed to zero.
  const long litShadow = brightPixels(sr);
  if (litNoShadow > 0) {
    assert(litShadow > 0);
    assert(litShadow > litNoShadow / 2); // not just a few stray survivors
    std::printf("  ok: the volume still renders with shadows on (bright px: %ld off, %ld on)\n",
                litNoShadow, litShadow);
  } else {
    std::printf("  ok: structural check only (this build did not rasterise; %ld bright px)\n",
                litShadow);
  }
}

} // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  test_shadows_do_not_hide_volumes();
  std::printf("cvcgl_shadow_volume: OK\n");
  return 0;
}
