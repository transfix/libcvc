// Regression guard for the "solid opaque block" volume bug.
//
// Two independent causes, both fixed in VolumeNode:
//
//   1. cvc::volume CACHES min/max (minIsSet/maxIsSet). Editing voxels in place
//      through the zero-copy grid view does NOT invalidate that cache, so a
//      volume created empty and filled afterwards still reported [0, 0].
//      VolumeNode now takes the range from the vtkImageData it just uploaded.
//
//   2. A degenerate range put both transfer-function control points on the same
//      scalar. VTK keeps one, and a piecewise function with a single point
//      returns it for EVERY input -- opacity 1.0 everywhere. An empty volume
//      must render as nothing, not as an opaque slab hiding the scene.
//
// The symptom was invisible from the outside: the field was right, the transfer
// function looked right, and lowering the opacity 50x changed nothing.
//
// These tests assert(), and cvcpkg builds them Release -- where NDEBUG makes
// assert() expand to nothing and every check below would pass vacuously.
// Undefine it before <cassert> so the assertions actually run.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/volume/volume.h>
#include <vector>

using cvc::gl::SceneGraph;
using cvc::gl::VolumeNode;

namespace {

constexpr int N = 8;

cvc::volume make_volume(cvc::app &app, float fill) {
  std::vector<float> data(static_cast<std::size_t>(N) * N * N, fill);
  return cvc::volume(app, reinterpret_cast<const unsigned char *>(data.data()),
                     cvc::dimension(N, N, N), cvc::Float, cvc::bounding_box(0, 0, 0, 1, 1, 1));
}

// The opacity table is stored as flat scalar,alpha pairs.
double top_opacity(const std::vector<double> &opacity) {
  assert(opacity.size() >= 2);
  return opacity[opacity.size() - 1];
}

// An empty (all-zero) volume must not come out opaque.
void test_empty_volume_is_not_opaque() {
  cvc::app app;
  SceneGraph sg(app);
  auto node = sg.addGraphics("empty", make_volume(app, 0.0f));
  auto *vn = dynamic_cast<VolumeNode *>(node.get());
  assert(vn != nullptr);

  auto opacity = vn->getTransferFunctionOpacityTable();
  assert(!opacity.empty());
  // Before the fix this was 1.0 — applied to every scalar, hence a solid block.
  assert(std::fabs(top_opacity(opacity)) < 1e-12);
  std::printf("  ok: an all-zero volume defaults to transparent, not opaque\n");
}

// A constant non-zero volume is degenerate too, and must be handled the same.
void test_constant_volume_is_not_opaque() {
  cvc::app app;
  SceneGraph sg(app);
  auto node = sg.addGraphics("flat", make_volume(app, 4.0f));
  auto *vn = dynamic_cast<VolumeNode *>(node.get());
  assert(vn != nullptr);
  auto opacity = vn->getTransferFunctionOpacityTable();
  assert(!opacity.empty());
  assert(std::fabs(top_opacity(opacity)) < 1e-12);
  // The two control points must still be distinct, or VTK collapses them.
  assert(opacity.size() >= 4);
  assert(opacity[2] > opacity[0]);
  std::printf("  ok: a constant volume keeps distinct control points\n");
}

// A real range still produces the normal 0 -> 1 ramp.
void test_real_range_still_ramps() {
  cvc::app app;
  SceneGraph sg(app);
  std::vector<float> data(static_cast<std::size_t>(N) * N * N);
  for (std::size_t i = 0; i < data.size(); ++i)
    data[i] = static_cast<float>(i);
  cvc::volume v(app, reinterpret_cast<const unsigned char *>(data.data()), cvc::dimension(N, N, N),
                cvc::Float, cvc::bounding_box(0, 0, 0, 1, 1, 1));

  auto node = sg.addGraphics("ramp", v);
  auto *vn = dynamic_cast<VolumeNode *>(node.get());
  assert(vn != nullptr);
  auto opacity = vn->getTransferFunctionOpacityTable();
  assert(opacity.size() >= 4);
  assert(std::fabs(top_opacity(opacity) - 1.0) < 1e-12);
  std::printf("  ok: a volume with a real range still gets the 0->1 ramp\n");
}

// THE ORIGINAL BUG: add an empty volume, fill it in place afterwards, re-set it.
// vol.min()/max() are still the cached [0, 0]; the node must use the uploaded
// image data instead and pick up the real range.
void test_inplace_fill_then_resetvolume() {
  cvc::app app;
  SceneGraph sg(app);
  cvc::volume v = make_volume(app, 0.0f);
  auto node = sg.addGraphics("late", v);
  auto *vn = dynamic_cast<VolumeNode *>(node.get());
  assert(vn != nullptr);

  // Fill the voxels behind the volume's back, exactly as a zero-copy numpy
  // view does, and WITHOUT calling unsetMinMax().
  float *voxels = reinterpret_cast<float *>(v.data_ptr());
  assert(voxels != nullptr);
  for (std::size_t i = 0; i < static_cast<std::size_t>(N) * N * N; ++i)
    voxels[i] = static_cast<float>(i % 17);

  // The cache is stale — this is the trap, and the reason the node cannot
  // trust it.
  assert(std::fabs(v.max() - 0.0) < 1e-12);

  vn->setVolume(v);
  auto opacity = vn->getTransferFunctionOpacityTable();
  assert(opacity.size() >= 4);
  // With the stale range the ramp collapsed and everything went opaque; with
  // the image-data range it is a real 0 -> 1 ramp over 0..16.
  assert(std::fabs(top_opacity(opacity) - 1.0) < 1e-12);
  assert(opacity[2] > 1.0); // top scalar is the real max (16), not 0 or 1
  std::printf("  ok: in-place fill + setVolume picks up the real range\n");
}

// Volumetric scattering / self-shadowing controls forward to the mapper and
// round-trip through the state tree. SceneNode dispatches state handlers inline on
// the owner thread (setInstanceThreading(false)), so a setter's effect is visible
// synchronously here — the getter reflects the value the handler pushed to the
// vtkSmartVolumeMapper / vtkVolumeProperty. Defaults are 0 (a plain absorption
// volume) so existing volumes are unchanged until a caller opts in.
void test_scattering_forwards() {
  cvc::app app;
  SceneGraph sg(app);
  auto node = sg.addGraphics("scat", make_volume(app, 1.0f));
  auto *vn = dynamic_cast<VolumeNode *>(node.get());
  assert(vn != nullptr);

  assert(vn->getVolumetricScattering() == 0.0);
  assert(vn->getGlobalIlluminationReach() == 0.0);
  assert(vn->getScatteringAnisotropy() == 0.0);

  // Each setter writes state; SceneNode fires the handler inline, which pushes the
  // value to the mapper/property AND updates the cached member the getter returns —
  // so a getter that reflects the set value proves the whole reactive path ran.
  vn->setVolumetricScattering(1.5);
  vn->setGlobalIlluminationReach(0.6);
  vn->setScatteringAnisotropy(0.7);
  assert(std::fabs(vn->getVolumetricScattering() - 1.5) < 1e-9);
  assert(std::fabs(vn->getGlobalIlluminationReach() - 0.6) < 1e-9);
  assert(std::fabs(vn->getScatteringAnisotropy() - 0.7) < 1e-9);
  std::printf("  ok: volumetric scattering / GI reach / anisotropy forward + round-trip\n");
}

// updateScalars() is the per-frame animation fast path: overwrite the voxels in
// place (memcpy + Modified) WITHOUT re-importing. Its whole point is what it does
// NOT do — no realloc, no scalar-range rescan, and crucially no setDefaultTransferFunction
// — so a transfer function the caller set stays put across an update, where a full
// setVolume would reset it. That contract is what this pins; that scalars actually
// reach the GPU is exercised by the animated demo end to end.
void test_updatescalars_is_the_cheap_path() {
  cvc::app app;
  SceneGraph sg(app);
  const int M = 8;
  const std::size_t NV = static_cast<std::size_t>(M) * M * M;
  // A genuine 0..12 range so setVolume builds a normal ramp we can tell a custom
  // function apart from.
  std::vector<float> f(NV);
  for (std::size_t i = 0; i < NV; ++i)
    f[i] = static_cast<float>(i % 13);
  cvc::volume v(app, reinterpret_cast<const unsigned char *>(f.data()), cvc::dimension(M, M, M),
                cvc::Float, cvc::bounding_box(0, 0, 0, 1, 1, 1));
  auto node = sg.addGraphics("keep", v);
  auto *vn = dynamic_cast<VolumeNode *>(node.get());
  assert(vn != nullptr);

  // A deliberately non-default transfer function (red->blue, a distinctive opacity
  // ramp) so it can't be confused with the ramp setVolume would install.
  vn->setTransferFunction({0.0, 1.0, 0.0, 0.0, 12.0, 0.0, 0.2, 1.0}, {0.0, 0.05, 12.0, 0.85});
  const auto customOpacity = vn->getTransferFunctionOpacityTable();
  const auto customColor = vn->getTransferFunctionColorTable();
  assert(!customOpacity.empty());

  // The fast path: overwrite every voxel in place. The transfer function must be left
  // byte-for-byte as set — updateScalars runs no setDefaultTransferFunction.
  std::vector<float> g(NV, 5.0f);
  vn->updateScalars(g);
  assert(vn->getTransferFunctionOpacityTable() == customOpacity);
  assert(vn->getTransferFunctionColorTable() == customColor);
  std::printf("  ok: updateScalars leaves the transfer function untouched\n");

  // Contrast: a full setVolume DOES reset the transfer function to the default ramp,
  // so the custom one must NOT survive it. This is exactly the work updateScalars skips.
  vn->setVolume(v);
  assert(vn->getTransferFunctionOpacityTable() != customOpacity);
  std::printf("  ok: setVolume resets the transfer function (the cost updateScalars avoids)\n");

  // GUARD: a voxel-count mismatch is an honest no-op — rejected before any VTK work,
  // so no crash, no partial memcpy, and the transfer function is left intact.
  vn->setTransferFunction({0.0, 1.0, 0.0, 0.0, 12.0, 0.0, 0.2, 1.0}, {0.0, 0.05, 12.0, 0.85});
  const auto restored = vn->getTransferFunctionOpacityTable();
  std::vector<float> wrong(static_cast<std::size_t>(M) * M * (M - 1), 5.0f);
  vn->updateScalars(wrong);
  assert(vn->getTransferFunctionOpacityTable() == restored);
  std::printf("  ok: updateScalars ignores a voxel-count mismatch\n");
}

} // namespace

int main() {
  test_empty_volume_is_not_opaque();
  test_constant_volume_is_not_opaque();
  test_real_range_still_ramps();
  test_inplace_fill_then_resetvolume();
  test_scattering_forwards();
  test_updatescalars_is_the_cheap_path();
  std::printf("cvcgl_volume_range: OK\n");
  return 0;
}
