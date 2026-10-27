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
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/state/state.h>
#include <cvc/volume/volume.h>
#include <string>
#include <thread>
#include <vector>
#include <vtkAbstractVolumeMapper.h>
#include <vtkDataSet.h>
#include <vtkVolume.h>
#include <vtkVolumeProperty.h>

using cvc::gl::SceneGraph;
using cvc::gl::VolumeNode;

namespace {

constexpr int N = 8;

cvc::volume make_volume(cvc::app &app, float fill) {
  std::vector<float> data(static_cast<std::size_t>(N) * N * N, fill);
  return cvc::volume(app, reinterpret_cast<const unsigned char *>(data.data()),
                     cvc::dimension(N, N, N), cvc::Float, cvc::bounding_box(0, 0, 0, 1, 1, 1));
}

// Voxels running linearly from lo (the first) to hi (the last): the range is exactly [lo, hi].
cvc::volume linear_volume(cvc::app &app, float lo, float hi) {
  const std::size_t n = static_cast<std::size_t>(N) * N * N;
  std::vector<float> data(n);
  for (std::size_t i = 0; i < n; ++i)
    data[i] = lo + (hi - lo) * static_cast<float>(i) / static_cast<float>(n - 1);
  data[n - 1] = hi;
  return cvc::volume(app, reinterpret_cast<const unsigned char *>(data.data()),
                     cvc::dimension(N, N, N), cvc::Float, cvc::bounding_box(0, 0, 0, 1, 1, 1));
}

// The tests below report every failed check instead of stopping at the first, so one regression
// cannot hide another; main() fails if any did.
int g_failures = 0;
void check(bool ok, const std::string &what) {
  std::printf("  %s: %s\n", ok ? "ok" : "FAIL", what.c_str());
  if (!ok)
    ++g_failures;
}

bool near(const std::vector<double> &a, const std::vector<double> &b) {
  if (a.size() != b.size())
    return false;
  for (std::size_t i = 0; i < a.size(); ++i)
    if (std::fabs(a[i] - b[i]) > 1e-12)
      return false;
  return true;
}

double state_double(VolumeNode &vn, const char *key) {
  try {
    return vn.getState(key).value<double>();
  } catch (...) {
    return std::nan("");
  }
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
// — so a transfer function the caller set stays put across an update (a full setVolume
// keeps an authored one too, but re-imports). That contract is what this pins; that scalars
// actually reach the GPU is exercised by the animated demo end to end.
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

  // A full setVolume keeps an AUTHORED transfer function (a tf_binding or a document set it);
  // only a node still on the default ramp is re-ramped over the new data range.
  vn->setVolume(v);
  assert(vn->getTransferFunctionOpacityTable() == customOpacity);
  std::printf("  ok: setVolume keeps the authored transfer function\n");

  // GUARD: a voxel-count mismatch is an honest no-op — rejected before any VTK work,
  // so no crash, no partial memcpy, and the transfer function is left intact.
  vn->setTransferFunction({0.0, 1.0, 0.0, 0.0, 12.0, 0.0, 0.2, 1.0}, {0.0, 0.05, 12.0, 0.85});
  const auto restored = vn->getTransferFunctionOpacityTable();
  std::vector<float> wrong(static_cast<std::size_t>(M) * M * (M - 1), 5.0f);
  vn->updateScalars(wrong);
  assert(vn->getTransferFunctionOpacityTable() == restored);
  std::printf("  ok: updateScalars ignores a voxel-count mismatch\n");
}

// setVolume publishes the range as two writes (data_min, data_max), and each runs the node's
// range handler INLINE -- which re-reads both keys. After the first write it saw the new minimum
// beside the OLD maximum and loaded that into the node, and setVolume then wrote the stale maximum
// straight back: a node created at the default [0, 1] given a signed [-2, 3] volume published
// [-2, 1], and its default ramp went fully opaque above 1. Every earlier test volume starts at 0,
// so the minimum never moved and nothing noticed. Reloads must also never show a reader an
// inverted pair in between.
void test_signed_range_reaches_state() {
  std::printf("== a signed range reaches data_min/data_max ==\n");
  cvc::app app;
  SceneGraph sg(app);
  auto vn = sg.addGraphics("signed", linear_volume(app, -2.0f, 3.0f));
  check(vn != nullptr, "the volume node exists");
  if (!vn)
    return;

  bool inverted = false;
  const auto watch = [&]() {
    if (state_double(*vn, "data_min") > state_double(*vn, "data_max"))
      inverted = true;
  };
  auto cMin = vn->getState("data_min").valueChanged.connect(watch);
  auto cMax = vn->getState("data_max").valueChanged.connect(watch);

  const auto expect_range = [&](double lo, double hi, bool defaultRamp, const std::string &what) {
    check(state_double(*vn, "data_min") == lo && state_double(*vn, "data_max") == hi,
          what + ": data_min/data_max = [" + std::to_string(lo) + ", " + std::to_string(hi) +
              "] (got [" + std::to_string(state_double(*vn, "data_min")) + ", " +
              std::to_string(state_double(*vn, "data_max")) + "])");
    if (!defaultRamp)
      return;
    // The default ramp spans the node's own range: it shows m_dataMin/m_dataMax.
    const auto op = vn->getTransferFunctionOpacityTable();
    check(op.size() == 4 && op[0] == lo && op[2] == hi && op[3] == 1.0,
          what + ": the default ramp spans the new range, opaque only at its top");
  };

  expect_range(-2.0, 3.0, true, "[0, 1] -> signed [-2, 3]");
  vn->setVolume(linear_volume(app, 5.0f, 10.0f));
  expect_range(5.0, 10.0, true, "reload wholly above the old range [5, 10]");
  vn->setVolume(linear_volume(app, -9.0f, -4.0f));
  expect_range(-9.0, -4.0, true, "reload wholly below the old range [-9, -4]");

  // An authored transfer function survives a reload, and the range keys still follow the volume.
  const std::vector<double> color = {-9.0, 1.0, 0.0, 0.0, -4.0, 0.0, 0.0, 1.0};
  const std::vector<double> opacity = {-9.0, 0.1, -4.0, 0.6};
  vn->setTransferFunction(color, opacity);
  vn->setVolume(linear_volume(app, -2.0f, 3.0f));
  expect_range(-2.0, 3.0, false, "authored TF, reload to [-2, 3]");
  check(vn->getTransferFunctionOpacityTable() == opacity, "the authored TF survives the reload");

  check(!inverted, "no reader ever saw data_min > data_max");
  cMin.disconnect();
  cMax.disconnect();
}

// The node skips the echo of its OWN mirror write of transfer_function.color/.opacity (VTK already
// holds those tables at full precision). That guard was one flag on the node, consulted on the
// WRITER's thread: a write from another thread while the owner was mid-write was dropped before it
// could be marshalled, leaving state holding the peer's table and VTK drawing the node's own.
void test_foreign_tf_write_during_own_write() {
  std::printf("== a foreign-thread TF write during the node's own write ==\n");
  cvc::app app;
  SceneGraph sg(app);
  auto vn = sg.addGraphics("peer", linear_volume(app, 0.0f, 12.0f));
  check(vn != nullptr, "the volume node exists");
  if (!vn)
    return;

  // The own echo is still skipped: VTK keeps more than the mirror's 6 decimals.
  const std::vector<double> color = {0.0, 1.0, 0.0, 0.0, 12.0, 0.0, 0.0, 1.0};
  vn->setTransferFunction(color, {0.0, 0.1234567891, 12.0, 0.9});
  check(near(vn->getTransferFunctionOpacityTable(), {0.0, 0.1234567891, 12.0, 0.9}),
        "the node's own mirror write is not re-applied at 6 decimals");

  // A slot on the `transfer_function` node fires after the volume node's own handler (ancestors'
  // slots run first) but still inside the node's write of opacity: there another thread writes it.
  const std::string foreign = "0,0.3,12,0.4";
  std::atomic<bool> armed{true};
  auto conn = vn->getState("transfer_function").childChanged.connect([&](const std::string &child) {
    if (child != "opacity" || !armed.exchange(false))
      return;
    std::thread([&]() { vn->getState("transfer_function.opacity").value(foreign); }).join();
  });
  vn->setTransferFunction(color, {0.0, 0.0, 12.0, 0.5});
  conn.disconnect();
  check(!armed, "the foreign write landed inside the node's own write");

  sg.processEvents(); // the foreign thread's handler marshals its apply onto the owner thread
  check(vn->getState("transfer_function.opacity").value() == foreign,
        "state holds the foreign opacity (it was the last write)");
  check(near(vn->getTransferFunctionOpacityTable(), {0.0, 0.3, 12.0, 0.4}),
        "VTK draws the foreign opacity too, not the node's own");
  check(near(vn->getTransferFunctionColorTable(), color), "the colour table is the node's");
}

// A transfer-function change rebuilds VTK's 1D lookup tables, nothing more. Marking the IMAGE data
// modified too made the GPU ray-cast mapper re-upload the whole 3D scalar texture on every edit --
// every frame of an editor drag, for a 512^3 volume.
void test_tf_change_leaves_the_image_alone() {
  std::printf("== a transfer-function change does not touch the image data ==\n");
  cvc::app app;
  SceneGraph sg(app);
  auto vn = sg.addGraphics("tex", linear_volume(app, 0.0f, 12.0f));
  auto *vol = vn ? vtkVolume::SafeDownCast(vn->prop()) : nullptr;
  vtkDataSet *img = vol && vol->GetMapper() ? vol->GetMapper()->GetDataSetInput() : nullptr;
  check(img != nullptr && vol->GetProperty() != nullptr, "the mapper input and property exist");
  if (!img || !vol->GetProperty())
    return;
  const vtkMTimeType imgBefore = img->GetMTime();

  vtkMTimeType propBefore = vol->GetProperty()->GetMTime();
  vn->setTransferFunction({0.0, 1.0, 0.0, 0.0, 12.0, 0.0, 0.0, 1.0}, {0.0, 0.1, 12.0, 0.9});
  check(vol->GetProperty()->GetMTime() > propBefore, "setTransferFunction reaches the property");
  check(img->GetMTime() == imgBefore, "setTransferFunction leaves the image data unmodified");

  // An editor's live write through state (tf_binding lowering an edit, every drag frame).
  propBefore = vol->GetProperty()->GetMTime();
  vn->getState("transfer_function.opacity").value(std::string("0,0.2,12,0.7"));
  check(near(vn->getTransferFunctionOpacityTable(), {0.0, 0.2, 12.0, 0.7}),
        "a state write applies live");
  check(vol->GetProperty()->GetMTime() > propBefore, "a state write reaches the property");
  check(img->GetMTime() == imgBefore, "a state write leaves the image data unmodified");

  // A range change re-fits transfer functions, never the voxels.
  vn->getState("data_max").value(20.0);
  check(img->GetMTime() == imgBefore, "a data_max change leaves the image data unmodified");

  // The volume path still re-uploads: a reload and an updateScalars mark the image modified.
  vn->setVolume(linear_volume(app, 0.0f, 12.0f));
  const vtkMTimeType afterReload = img->GetMTime();
  check(afterReload > imgBefore, "setVolume marks the image data modified");
  vn->updateScalars(std::vector<float>(static_cast<std::size_t>(N) * N * N, 3.0f));
  check(img->GetMTime() > afterReload, "updateScalars marks the image data modified");
}

} // namespace

int main() {
  test_empty_volume_is_not_opaque();
  test_constant_volume_is_not_opaque();
  test_real_range_still_ramps();
  test_inplace_fill_then_resetvolume();
  test_scattering_forwards();
  test_updatescalars_is_the_cheap_path();
  test_signed_range_reaches_state();
  test_foreign_tf_write_during_own_write();
  test_tf_change_leaves_the_image_alone();
  if (g_failures != 0) {
    std::printf("cvcgl_volume_range: %d check(s) FAILED\n", g_failures);
    return 1;
  }
  std::printf("cvcgl_volume_range: OK\n");
  return 0;
}
