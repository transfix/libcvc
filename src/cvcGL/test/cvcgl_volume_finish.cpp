// VolumeNode turns VTK's FinishEveryRender off: a still render queues the GPU ray cast and
// returns, instead of waiting for the GPU (glFinish) for a timer only an auto-adjusting render
// uses. The property is cvcpkg's VTK patch (vtk 9.5.0+cvc.5, written for upstream); against a
// VTK without it (CVC_VTK_HAS_FINISH_EVERY_RENDER undefined) this checks only that the volume
// draws. Checked, natively:
//   * the VolumeNode's vtkSmartVolumeMapper has FinishEveryRender off;
//   * the pixels are the same with it off and on (it changes when the CPU waits, not what is
//     drawn), and the same again after turning it back off.
// Skipped when the build cannot rasterise offscreen.
//
// cvcpkg builds tests Release, where NDEBUG makes assert() expand to nothing. Undefine it.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/volume/volume.h>
#include <vector>
#include <vtkRenderer.h>
#include <vtkSmartVolumeMapper.h>
#include <vtkVolume.h>
#include <vtkVolumeCollection.h>

using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

namespace {

long lit(const std::vector<unsigned char> &px) {
  long n = 0;
  for (std::size_t i = 0; i + 2 < px.size(); i += 3)
    if (px[i] + px[i + 1] + px[i + 2] > 30)
      ++n;
  return n;
}

long differing(const std::vector<unsigned char> &a, const std::vector<unsigned char> &b) {
  assert(a.size() == b.size());
  long n = 0;
  for (std::size_t i = 0; i < a.size(); ++i)
    if (a[i] != b[i])
      ++n;
  return n;
}

} // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  cvc::app app;
  SceneGraph sg(app, "finish");
  // A soft ball whose density ramps from the rim to the centre, coloured blue -> orange.
  const int N = 32;
  std::vector<float> field(static_cast<std::size_t>(N) * N * N);
  for (int z = 0; z < N; ++z)
    for (int y = 0; y < N; ++y)
      for (int x = 0; x < N; ++x) {
        const double dx = (x + 0.5) / N - 0.5, dy = (y + 0.5) / N - 0.5, dz = (z + 0.5) / N - 0.5;
        field[(static_cast<std::size_t>(z) * N + y) * N + x] =
            static_cast<float>(std::max(0.0, 1.0 - 2.0 * std::sqrt(dx * dx + dy * dy + dz * dz)));
      }
  cvc::volume vol(app, reinterpret_cast<const unsigned char *>(field.data()),
                  cvc::dimension(N, N, N), cvc::Float, cvc::bounding_box(-1, -1, -1, 1, 1, 1));
  auto ball = sg.addGraphics("ball", vol);
  ball->setTransferFunction({0.0, 0.1, 0.2, 0.9, 1.0, 1.0, 0.6, 0.1},
                            {0.0, 0.0, 0.2, 0.05, 1.0, 0.9});
  ball->setShading(false);

  SceneRenderer sr(sg, 128, 128, /*offscreen=*/true);
  sr.setBackground(0.0, 0.0, 0.0);
  sr.resetCamera();
  const std::vector<unsigned char> first = sr.frameRGB();
  if (lit(first) == 0) {
    std::printf("  ok: this build did not rasterise; check skipped\n");
    std::printf("cvcgl_volume_finish: OK\n");
    return 0;
  }
  vtkVolumeCollection *vols = sr.renderer()->GetVolumes();
  vols->InitTraversal();
  vtkVolume *v = vols->GetNextVolume();
  assert(v);
  auto *mapper = vtkSmartVolumeMapper::SafeDownCast(v->GetMapper());
  assert(mapper);
  std::printf("  ok: the ball draws (%ld lit px) through a vtkSmartVolumeMapper\n", lit(first));

#ifdef CVC_VTK_HAS_FINISH_EVERY_RENDER
  assert(!mapper->GetFinishEveryRender());
  std::printf("  ok: VolumeNode's mapper has FinishEveryRender off\n");
  const std::vector<unsigned char> off = sr.frameRGB();
  mapper->FinishEveryRenderOn();
  const std::vector<unsigned char> on = sr.frameRGB();
  mapper->FinishEveryRenderOff();
  const std::vector<unsigned char> offAgain = sr.frameRGB();
  const long d1 = differing(off, on), d2 = differing(off, offAgain);
  std::printf("  bytes that differ: off vs on %ld, off vs off again %ld\n", d1, d2);
  assert(d1 == 0 && d2 == 0);
  std::printf("  ok: the pixels are the same with FinishEveryRender off and on\n");
#else
  std::printf("  ok: this VTK has no FinishEveryRender (cvcpkg's vtk 9.5.0+cvc.5 adds it); the "
              "volume renders with VTK's glFinish\n");
#endif
  std::printf("cvcgl_volume_finish: OK\n");
  return 0;
}
