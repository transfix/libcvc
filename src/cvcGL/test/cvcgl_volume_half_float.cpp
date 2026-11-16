// VolumeNode's half-float fallback for contexts that cannot filter 32-bit float textures.
//
// VTK's GPU ray caster keeps a volume's transfer functions as float textures (RGB32F colour,
// R32F opacity), filtered with the volume's interpolation. WebGL2 filters 32F linearly only with
// OES_texture_float_linear, which iOS Safari lacks: a LINEAR 32F texture there samples as zero, so
// the volume gets no colour and no opacity. VolumeNode then switches VTK's default float formats to
// 16F while the volume draws. Desktop GL never needs it, so this forces it on
// (VolumeNode::setHalfFloatTextures(1) on the node) and checks, natively:
//   * the volume renders the same with 16F tables as with 32F ones (no lost colour or opacity);
//   * with it on, the volume's tables really are 16F textures, and with it off none are;
//   * the defaults are put back after the draw, so the rest of the scene stays 32F.
// Skipped when the build cannot rasterise offscreen.
//
// cvcpkg builds tests Release, where NDEBUG makes assert() expand to nothing. Undefine it.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/volume/volume.h>
#include <vector>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLState.h>
#include <vtkRenderer.h>
#include <vtk_glad.h>

using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::VolumeNode;

namespace {

struct Frame {
  std::vector<unsigned char> rgb;
  int halfTextures = 0; // R16F / RG16F / RGB16F / RGBA16F textures alive after the draw
  bool restored = false;
};

// Count the context's live textures in 16-bit float formats (desktop GL can ask a texture its
// format; GLES cannot, so -1 there).
int halfFloatTextures(vtkOpenGLRenderWindow *win) {
#ifdef GL_ES_VERSION_3_0
  (void)win;
  return -1;
#else
  win->MakeCurrent();
  int n = 0;
  GLint was = 0;
  glGetIntegerv(GL_TEXTURE_BINDING_2D, &was);
  for (GLuint t = 1; t < 512; ++t) {
    if (!glIsTexture(t))
      continue;
    glBindTexture(GL_TEXTURE_2D, t);
    if (glGetError() != GL_NO_ERROR) // not a 2-D texture
      continue;
    GLint f = 0;
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_INTERNAL_FORMAT, &f);
    if (f == GL_R16F || f == GL_RG16F || f == GL_RGB16F || f == GL_RGBA16F)
      ++n;
  }
  glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(was));
  while (glGetError() != GL_NO_ERROR) {
  }
  return n;
#endif
}

Frame render(int halfMode) {
  cvc::app app;
  SceneGraph sg(app, "halffloat");
  // A soft ball whose density ramps from the rim to the centre, coloured by density through a
  // blue -> orange ramp: linear interpolation reads the tables between their entries all over.
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
  ball->setHalfFloatTextures(halfMode);

  SceneRenderer sr(sg, 128, 128, /*offscreen=*/true);
  sr.setBackground(0.0, 0.0, 0.0);
  sr.resetCamera();
  Frame f;
  f.rgb = sr.frameRGB();
  auto *win = vtkOpenGLRenderWindow::SafeDownCast(sr.renderer()->GetRenderWindow());
  if (win) {
    f.halfTextures = halfFloatTextures(win);
    f.restored = win->GetState()->GetDefaultTextureInternalFormat(VTK_FLOAT, 1, false, true,
                                                                  false) == GL_R32F &&
                 win->GetState()->GetDefaultTextureInternalFormat(VTK_FLOAT, 3, false, true,
                                                                  false) == GL_RGB32F;
  }
  return f;
}

long lit(const std::vector<unsigned char> &px) {
  long n = 0;
  for (std::size_t i = 0; i + 2 < px.size(); i += 3)
    if (px[i] + px[i + 1] + px[i + 2] > 30)
      ++n;
  return n;
}

} // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  const Frame full = render(0);
  const Frame half = render(1);
  const long litFull = lit(full.rgb), litHalf = lit(half.rgb);
  if (litFull == 0) {
    std::printf("  ok: this build did not rasterise; half-float check skipped\n");
    std::printf("cvcgl_volume_half_float: OK\n");
    return 0;
  }
  assert(full.rgb.size() == half.rgb.size());
  double sum = 0.0;
  int worst = 0;
  for (std::size_t i = 0; i < full.rgb.size(); ++i) {
    const int d = std::abs(int(full.rgb[i]) - int(half.rgb[i]));
    sum += d;
    worst = std::max(worst, d);
  }
  const double mean = sum / full.rgb.size();
  std::printf("  lit px: %ld (32F) vs %ld (16F); mean |diff| %.3f, worst %d\n", litFull, litHalf,
              mean, worst);
  assert(litHalf > litFull * 9 / 10);
  assert(mean < 1.0);
  std::printf("  ok: the volume draws the same with 16-bit float tables\n");
  std::printf("  16F textures alive: %d (32F run) vs %d (16F run)\n", full.halfTextures,
              half.halfTextures);
  if (half.halfTextures >= 0) {
    assert(full.halfTextures == 0);
    assert(half.halfTextures >= 2); // the colour table and the opacity table at least
    std::printf("  ok: forced on, the volume's tables are 16F textures; off, none are\n");
  }
  assert(full.restored && half.restored);
  std::printf("  ok: the context's default float formats are 32F again after the draw\n");
  std::printf("cvcgl_volume_half_float: OK\n");
  return 0;
}
