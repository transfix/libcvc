// OverlappingVolumeMapper: VolumeNode's GPU ray caster without VTK's glFinish at the end of each
// still draw (inc/cvc/gl/OverlappingVolumeMapper.h).
//
// VTK's vtkOpenGLGPUVolumeRayCastMapper::GPURender ends every draw with glFinish() so its CPU
// timer measures the GPU's work, which only an auto-adjusting (interactive) draw uses. This
// counts glFinish through VTK's GL loader (glad's pointer, swapped for a counter once the context
// is up) and checks, natively:
//   * a VolumeNode draws through an OverlappingVolumeMapper;
//   * a still render of it calls no glFinish, and glad's pointer is the counter again after;
//   * an interactive render (the window's desired update rate at or above the mapper's
//     interactive rate) keeps VTK's glFinish, so auto-adjust still times its draws;
//   * a plain vtkSmartVolumeMapper does call glFinish on a still render (the wait this removes);
//   * the picture is the same: the same volume drawn by either mapper gives the same pixels.
// Skipped when the build cannot rasterise offscreen, and on WebAssembly (no loader pointer).
//
// cvcpkg builds tests Release, where NDEBUG makes assert() expand to nothing. Undefine it.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/gl/OverlappingVolumeMapper.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/volume/volume.h>
#include <vector>
#include <vtkColorTransferFunction.h>
#include <vtkFloatArray.h>
#include <vtkImageData.h>
#include <vtkPiecewiseFunction.h>
#include <vtkPointData.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSmartVolumeMapper.h>
#include <vtkVolume.h>
#include <vtkVolumeCollection.h>
#include <vtkVolumeProperty.h>
#include <vtk_glad.h>

using cvc::gl::OverlappingVolumeMapper;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

#if !defined(__EMSCRIPTEN__)
namespace {

const int N = 32;

// A soft ball whose density ramps from the rim to the centre.
float density(int x, int y, int z) {
  const double dx = (x + 0.5) / N - 0.5, dy = (y + 0.5) / N - 0.5, dz = (z + 0.5) / N - 0.5;
  return static_cast<float>(std::max(0.0, 1.0 - 2.0 * std::sqrt(dx * dx + dy * dy + dz * dz)));
}

// What glad's glFinish runs while the test counts: nothing, counted.
long finishes = 0;
void GLAD_API_PTR count_finish(void) { ++finishes; }

long lit(const std::vector<unsigned char> &px) {
  long n = 0;
  for (std::size_t i = 0; i + 2 < px.size(); i += 3)
    if (px[i] + px[i + 1] + px[i + 2] > 30)
      ++n;
  return n;
}

// The glFinish calls of `frames` renders of `sr`, with glad's pointer the counter for them.
long finishesOver(SceneRenderer &sr, int frames) {
  const PFNGLFINISHPROC was = glad_glFinish;
  glad_glFinish = &count_finish;
  finishes = 0;
  for (int i = 0; i < frames; ++i)
    sr.render();
  assert(glad_glFinish == &count_finish); // put back after every draw
  glad_glFinish = was;
  return finishes;
}

// 1. A VolumeNode: still renders wait for nothing; interactive ones keep VTK's timing.
bool volumeNode() {
  cvc::app app;
  SceneGraph sg(app, "overlap");
  std::vector<float> field(static_cast<std::size_t>(N) * N * N);
  for (int z = 0; z < N; ++z)
    for (int y = 0; y < N; ++y)
      for (int x = 0; x < N; ++x)
        field[(static_cast<std::size_t>(z) * N + y) * N + x] = density(x, y, z);
  cvc::volume vol(app, reinterpret_cast<const unsigned char *>(field.data()),
                  cvc::dimension(N, N, N), cvc::Float, cvc::bounding_box(-1, -1, -1, 1, 1, 1));
  auto ball = sg.addGraphics("ball", vol);
  ball->setTransferFunction({0.0, 0.1, 0.2, 0.9, 1.0, 1.0, 0.6, 0.1},
                            {0.0, 0.0, 0.2, 0.05, 1.0, 0.9});
  ball->setShading(false);

  SceneRenderer sr(sg, 128, 128, /*offscreen=*/true);
  sr.setBackground(0.0, 0.0, 0.0);
  sr.resetCamera();
  if (lit(sr.frameRGB()) == 0) // also loads GL, so glad's pointer is the driver's from here
    return false;

  vtkVolumeCollection *vols = sr.renderer()->GetVolumes();
  vols->InitTraversal();
  vtkVolume *v = vols->GetNextVolume();
  assert(v);
  auto *mapper = OverlappingVolumeMapper::SafeDownCast(v->GetMapper());
  assert(mapper);
  std::printf("  ok: the VolumeNode draws through an OverlappingVolumeMapper\n");

  vtkRenderWindow *rw = sr.renderWindow();
  const double stillRate = rw->GetDesiredUpdateRate();
  assert(!mapper->TimesItsDraws(sr.renderer()));
  const long still = finishesOver(sr, 4);
  std::printf("  glFinish over 4 still renders: %ld\n", still);
  assert(still == 0);
  std::printf("  ok: a still render waits for nothing, and glad's glFinish is put back\n");

  rw->SetDesiredUpdateRate(mapper->GetInteractiveUpdateRate() * 10.0);
  assert(mapper->TimesItsDraws(sr.renderer()));
  const long interactive = finishesOver(sr, 4);
  std::printf("  glFinish over 4 interactive renders: %ld\n", interactive);
  assert(interactive >= 4);
  rw->SetDesiredUpdateRate(stillRate);
  std::printf("  ok: an interactive render keeps VTK's glFinish (auto-adjust times its draws)\n");
  return true;
}

// 2. The same volume through either mapper: the same pixels; only the plain one waits.
void samePicture() {
  cvc::app app;
  SceneGraph sg(app, "overlap_raw");
  SceneRenderer sr(sg, 128, 128, /*offscreen=*/true);
  sr.setBackground(0.0, 0.0, 0.0);

  auto img = vtkSmartPointer<vtkImageData>::New();
  img->SetDimensions(N, N, N);
  img->SetSpacing(2.0 / (N - 1), 2.0 / (N - 1), 2.0 / (N - 1));
  img->SetOrigin(-1.0, -1.0, -1.0);
  auto scalars = vtkSmartPointer<vtkFloatArray>::New();
  scalars->SetNumberOfTuples(static_cast<vtkIdType>(N) * N * N);
  for (int z = 0; z < N; ++z)
    for (int y = 0; y < N; ++y)
      for (int x = 0; x < N; ++x)
        scalars->SetValue((static_cast<vtkIdType>(z) * N + y) * N + x, density(x, y, z));
  img->GetPointData()->SetScalars(scalars);

  auto color = vtkSmartPointer<vtkColorTransferFunction>::New();
  color->AddRGBPoint(0.0, 0.1, 0.2, 0.9);
  color->AddRGBPoint(1.0, 1.0, 0.6, 0.1);
  auto opacity = vtkSmartPointer<vtkPiecewiseFunction>::New();
  opacity->AddPoint(0.0, 0.0);
  opacity->AddPoint(0.2, 0.05);
  opacity->AddPoint(1.0, 0.9);
  auto prop = vtkSmartPointer<vtkVolumeProperty>::New();
  prop->SetColor(color);
  prop->SetScalarOpacity(opacity);
  prop->SetInterpolationTypeToLinear();
  prop->ShadeOff();

  auto overlap = vtkSmartPointer<OverlappingVolumeMapper>::New();
  auto plain = vtkSmartPointer<vtkSmartVolumeMapper>::New();
  for (vtkSmartVolumeMapper *m : {static_cast<vtkSmartVolumeMapper *>(overlap), plain.Get()}) {
    m->SetInputData(img);
    m->SetRequestedRenderModeToGPU();
    m->SetBlendModeToComposite();
  }
  auto volume = vtkSmartPointer<vtkVolume>::New();
  volume->SetProperty(prop);
  volume->SetMapper(plain);
  sr.renderer()->AddVolume(volume);
  sr.resetCamera();

  const std::vector<unsigned char> byPlain = sr.frameRGB();
  const long plainWaits = finishesOver(sr, 2);
  volume->SetMapper(overlap);
  const std::vector<unsigned char> byOverlap = sr.frameRGB();
  const long overlapWaits = finishesOver(sr, 2);
  const std::vector<unsigned char> byOverlapAgain = sr.frameRGB();
  std::printf("  glFinish over 2 still renders: %ld (vtkSmartVolumeMapper) vs %ld "
              "(OverlappingVolumeMapper)\n",
              plainWaits, overlapWaits);
  assert(plainWaits >= 2);
  assert(overlapWaits == 0);
  std::printf("  ok: VTK's mapper waits for the GPU at each still draw; this one does not\n");

  assert(lit(byPlain) > 0);
  assert(byPlain.size() == byOverlap.size());
  long differ = 0;
  for (std::size_t i = 0; i < byPlain.size(); ++i)
    if (byPlain[i] != byOverlap[i] || byOverlap[i] != byOverlapAgain[i])
      ++differ;
  std::printf("  lit px %ld; bytes that differ between the mappers: %ld\n", lit(byPlain), differ);
  assert(differ == 0);
  std::printf("  ok: both mappers draw the same pixels\n");
}

} // namespace
#endif

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
#if defined(__EMSCRIPTEN__)
  std::printf("  ok: WebAssembly keeps VTK's glFinish; check skipped\n");
#else
  if (!volumeNode()) {
    std::printf("  ok: this build did not rasterise; glFinish check skipped\n");
    std::printf("cvcgl_volume_overlap: OK\n");
    return 0;
  }
  samePicture();
#endif
  std::printf("cvcgl_volume_overlap: OK\n");
  return 0;
}
