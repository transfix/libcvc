// VolumeNode's half-float fallback for contexts that cannot filter 32-bit float textures.
//
// VTK's GPU ray caster keeps a volume's transfer functions as float textures (RGB32F colour,
// R32F opacity), filtered with the volume's interpolation. WebGL2 filters 32F linearly only with
// OES_texture_float_linear, which iOS Safari lacks: a LINEAR 32F texture there samples as zero, so
// the volume gets no colour and no opacity. VolumeNode then switches VTK's default float formats to
// 16F while the volume draws. Desktop GL never needs it, so this forces it on
// (VolumeNode::setHalfFloatTextures(1) on the node) and checks, natively:
//   * the volume renders the same with 16F tables as with 32F ones (no lost colour or opacity);
//   * with it on, VTK asks GL for the volume's tables as 16F textures, and with it off for none.
//     That is read where VTK asks, at glTexImage2D, through VTK's glad pointer;
//   * GL keeps them as 16F textures. Apple's software renderer (the macOS CI runners) reports a
//     texture asked for as 16F in the matching 32F format, so there GL's formats cannot show the
//     fallback: that check alone is skipped, and the skip says why;
//   * the defaults are put back after the draw, so the rest of the scene stays 32F.
// Skipped when the build cannot rasterise offscreen. GLES cannot ask a texture its format and has
// no glad, so the format checks are desktop GL only.
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
#include <map>
#include <string>
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
  std::string renderer;  // GL_RENDERER
  int made = 0;          // 2-D textures VTK made in the draw (glTexImage2D, level 0)
  int asked16 = 0;       // live ones VTK asked GL to keep as R16F / RG16F / RGB16F / RGBA16F
  int halfTextures = 0;  // live 2-D textures GL says it keeps as 16F (-1: GLES cannot say)
  bool keptAs32 = false; // GL keeps every one asked for as 16F as the matching 32F format
  std::string kept;      // what GL keeps each one asked for as 16F as
  bool restored = false;
};

#ifndef GL_ES_VERSION_3_0
bool isHalf(GLint f) { return f == GL_R16F || f == GL_RG16F || f == GL_RGB16F || f == GL_RGBA16F; }

// The 32F format with a 16F format's channels.
GLint fullOf(GLint f) {
  switch (f) {
  case GL_R16F:
    return GL_R32F;
  case GL_RG16F:
    return GL_RG32F;
  case GL_RGB16F:
    return GL_RGB32F;
  case GL_RGBA16F:
    return GL_RGBA32F;
  default:
    return 0;
  }
}

std::string formatName(GLint f) {
  switch (f) {
  case GL_R16F:
    return "R16F";
  case GL_RG16F:
    return "RG16F";
  case GL_RGB16F:
    return "RGB16F";
  case GL_RGBA16F:
    return "RGBA16F";
  case GL_R32F:
    return "R32F";
  case GL_RG32F:
    return "RG32F";
  case GL_RGB32F:
    return "RGB32F";
  case GL_RGBA32F:
    return "RGBA32F";
  default: {
    char hex[16];
    std::snprintf(hex, sizeof(hex), "0x%04X", static_cast<unsigned>(f));
    return hex;
  }
  }
}

// The internal format VTK gave glTexImage2D for each 2-D texture, by name: what it asked for,
// whatever GL then keeps.
std::map<GLuint, GLint> g_asked;
int g_made = 0;
decltype(glad_glTexImage2D) g_texImage2D = nullptr;

void GLAD_API_PTR recordTexImage2D(GLenum target, GLint level, GLint internalFormat, GLsizei w,
                                   GLsizei h, GLint border, GLenum format, GLenum type,
                                   const void *data) {
  if (target == GL_TEXTURE_2D && level == 0) {
    GLint bound = 0;
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &bound);
    g_asked[static_cast<GLuint>(bound)] = internalFormat;
    ++g_made;
  }
  g_texImage2D(target, level, internalFormat, w, h, border, format, type, data);
}
#endif

// Starts recording what VTK asks for. Call once VTK has loaded GL for this window: VTK loads glad
// once per new context, which puts the driver's glTexImage2D back.
void recordTextures(SceneRenderer &sr) {
#ifdef GL_ES_VERSION_3_0
  (void)sr;
#else
  sr.renderWindow()->Initialize(); // the context, and glad loaded for it
  g_asked.clear();
  g_made = 0;
  if (glad_glTexImage2D && glad_glTexImage2D != recordTexImage2D) {
    g_texImage2D = glad_glTexImage2D;
    glad_glTexImage2D = recordTexImage2D;
  }
#endif
}

// Ask GL what it keeps the context's live 2-D textures as, next to what VTK asked for.
void readFormats(vtkOpenGLRenderWindow *win, Frame &f) {
#ifdef GL_ES_VERSION_3_0
  (void)win;
  f.halfTextures = -1;
#else
  win->MakeCurrent();
  while (glGetError() != GL_NO_ERROR) {
  }
  if (const GLubyte *r = glGetString(GL_RENDERER))
    f.renderer = reinterpret_cast<const char *>(r);
  f.made = g_made;
  GLint was = 0;
  glGetIntegerv(GL_TEXTURE_BINDING_2D, &was);
  bool allAs32 = true;
  for (GLuint t = 1; t < 4096; ++t) {
    if (!glIsTexture(t))
      continue;
    glBindTexture(GL_TEXTURE_2D, t);
    if (glGetError() != GL_NO_ERROR) // not a 2-D texture
      continue;
    GLint kept = 0;
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_INTERNAL_FORMAT, &kept);
    if (isHalf(kept))
      ++f.halfTextures;
    const auto asked = g_asked.find(t);
    if (asked == g_asked.end() || !isHalf(asked->second))
      continue;
    ++f.asked16;
    allAs32 = allAs32 && kept == fullOf(asked->second);
    f.kept += (f.kept.empty() ? "" : ", ") + formatName(asked->second) + " as " + formatName(kept);
  }
  f.keptAs32 = f.asked16 > 0 && allAs32;
  glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(was));
  while (glGetError() != GL_NO_ERROR) {
  }
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
  recordTextures(sr);
  Frame f;
  f.rgb = sr.frameRGB();
  auto *win = vtkOpenGLRenderWindow::SafeDownCast(sr.renderer()->GetRenderWindow());
  if (win) {
    readFormats(win, f);
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
  if (half.halfTextures >= 0) {
    std::printf("  GL: %s; VTK made %d 2-D textures (32F run) and %d (16F run)\n",
                half.renderer.c_str(), full.made, half.made);
    assert(full.made > 0 && half.made > 0); // the glTexImage2D hook saw VTK make them
    std::printf("  asked for as 16F: %d (32F run) vs %d (16F run)\n", full.asked16, half.asked16);
    assert(full.asked16 == 0);
    assert(half.asked16 >= 2); // the colour table and the opacity table at least
    std::printf("  ok: forced on, VTK asks for the volume's tables as 16F textures; off, none\n");
    std::printf("  16F textures alive: %d (32F run) vs %d (16F run); kept: %s\n", full.halfTextures,
                half.halfTextures, half.kept.c_str());
    assert(full.halfTextures == 0);
    if (half.halfTextures == 0 && half.keptAs32) {
      std::printf("  skipped: %s keeps 16F textures as 32F ones, so its formats cannot show "
                  "the 16F tables\n",
                  half.renderer.c_str());
    } else {
      assert(half.halfTextures >= 2);
      std::printf("  ok: and GL keeps them as 16F textures\n");
    }
  }
  assert(full.restored && half.restored);
  std::printf("  ok: the context's default float formats are 32F again after the draw\n");
  std::printf("cvcgl_volume_half_float: OK\n");
  return 0;
}
