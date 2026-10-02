// GeometryNode::texture_modified_rows / texture_modified_rect: patch part of a
// zero-copy texture on the GPU without marking anything modified.
//
// texture_modified() bumps the vtkTexture (and the actor): the next frame
// re-uploads the WHOLE texture, and since vtkActor::GetMTime() includes the
// texture's, vtkShadowMapBakerPass re-renders the scene into its depth maps. A
// fog layer repainting ~1% of a 1024^2 texture paid a 4 MiB upload and a bake.
//
// Pins, with a real GL context (skipped where nothing rasterises, unless
// CVC_REQUIRE_RENDER=1):
//   A. after texture_modified_rows the texture, its input and the actor keep
//      their MTimes, a frame is requested, and the next render does not bake;
//   B. the GPU texture holds the new pixels in exactly the rows named -- rows
//      edited in memory but NOT named still hold the old pixels, which a full
//      re-upload would have replaced;
//   C. the rectangle variant patches only the rectangle (GL_UNPACK_ROW_LENGTH);
//   D. ranges are clamped; a call from another thread is marshalled;
//   E. texture_modified() still re-uploads everything (and bakes);
//   F. a copy-path (non-zero-copy) texture falls back to texture_modified().
// Desktop GL only: the GPU read-back uses glGetTexImage, absent from GLES.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/image/image.h>
#include <string>
#include <thread>
#include <vector>
#include <vtkActor.h>
#include <vtkCameraPass.h>
#include <vtkImageData.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLTexture.h>
#include <vtkRenderPassCollection.h>
#include <vtkRenderer.h>
#include <vtkSequencePass.h>
#include <vtkShadowMapBakerPass.h>
#include <vtkTextureObject.h>
#include <vtk_glad.h>

using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

static int fails = 0;
static void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

class TexNode : public GeometryNode {
public:
  TexNode(cvc::app &a, const std::string &path, const std::string &name)
      : GeometryNode(a, path, name) {}
  vtkActor *actor() { return vtkActor::SafeDownCast(getProp()); }
};

static cvc::geometry uvQuad(double s) {
  cvc::geometry g;
  g.points() = {{{-s, -s, 0}}, {{s, -s, 0}}, {{s, s, 0}}, {{-s, s, 0}}};
  g.uvs() = {{{0.0, 0.0}}, {{1.0, 0.0}}, {{1.0, 1.0}}, {{0.0, 1.0}}};
  g.tris() = {{{0, 1, 2}}, {{0, 2, 3}}};
  return g;
}

static vtkShadowMapBakerPass *findBaker(vtkRenderPass *p) {
  if (!p)
    return nullptr;
  if (auto *b = vtkShadowMapBakerPass::SafeDownCast(p))
    return b;
  if (auto *c = vtkCameraPass::SafeDownCast(p))
    return findBaker(c->GetDelegatePass());
  if (auto *s = vtkSequencePass::SafeDownCast(p))
    if (vtkRenderPassCollection *pc = s->GetPasses()) {
      pc->InitTraversal();
      while (vtkRenderPass *child = pc->GetNextRenderPass())
        if (auto *b = findBaker(child))
          return b;
    }
  return nullptr;
}

static const int W = 64, H = 32;

static void fillRows(unsigned char *px, int r0, int r1, unsigned char v) {
  for (int r = r0; r < r1; ++r)
    for (int x = 0; x < W; ++x) {
      unsigned char *p = px + (static_cast<std::size_t>(r) * W + x) * 4;
      p[0] = v;
      p[1] = static_cast<unsigned char>(255 - v);
      p[2] = static_cast<unsigned char>(r);
      p[3] = 255;
    }
}

// Every texel of the live GL texture, read back.
static std::vector<unsigned char> gpuPixels(SceneRenderer &view, vtkActor *actor) {
  std::vector<unsigned char> out(static_cast<std::size_t>(W) * H * 4, 0);
  auto *otex = vtkOpenGLTexture::SafeDownCast(actor->GetTexture());
  vtkTextureObject *to = otex ? otex->GetTextureObject() : nullptr;
  if (!to || to->GetHandle() == 0)
    return {};
#ifndef GL_ES_VERSION_3_0
  if (auto *w = vtkOpenGLRenderWindow::SafeDownCast(view.renderWindow()))
    w->MakeCurrent();
  to->Activate();
  glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, out.data());
  to->Deactivate();
#else
  (void)view;
#endif
  return out;
}

static bool rowsEqual(const std::vector<unsigned char> &a, const unsigned char *b, int r0, int r1) {
  if (a.empty())
    return false;
  const std::size_t off = static_cast<std::size_t>(r0) * W * 4;
  const std::size_t len = static_cast<std::size_t>(r1 - r0) * W * 4;
  return std::memcmp(a.data() + off, b + off, len) == 0;
}

static bool canRasterise(cvc::app &app) {
  SceneGraph sg(app, "texrowscontrol");
  sg.setDiagnosticChromeVisible(false);
  auto g = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("control");
  g->setGeometry(uvQuad(1.0));
  SceneRenderer sr(sg, 32, 32, /*offscreen=*/true);
  sr.setBackground(0.0, 0.0, 0.0);
  sr.setCamera(0, 0, 4, 0, 0, 0, 0, 1, 0, 40.0, 0.5, 100.0);
  for (unsigned char c : sr.frameRGB())
    if (c > 20)
      return true;
  return false;
}

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
#ifdef GL_ES_VERSION_3_0
  std::printf("skipped: GLES build (no glGetTexImage)\n");
  return 0;
#endif
  if (!canRasterise(app)) {
    const char *require = std::getenv("CVC_REQUIRE_RENDER");
    if (require && *require && std::string(require) != "0") {
      std::printf("CVC_REQUIRE_RENDER is set, but this build did not rasterise\n");
      return 1;
    }
    std::printf("skipped: this build did not rasterise\n");
    return 0;
  }

  SceneGraph sg(app, "texrows");
  sg.setDiagnosticChromeVisible(false);
  auto node = sg.getGraphicsRoot()->addGraphicsChild<TexNode>("ground");
  node->setGeometry(uvQuad(2.0));
  node->setUseSingleColor(true);
  auto blocker = sg.addGraphics("blocker", uvQuad(0.5)); // something to cast a shadow
  blocker->setPosition(0.0, 0.0, 0.6);

  cvc::image img(W, H, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  unsigned char *px = img.storage().get();
  fillRows(px, 0, H, 10);
  node->setTexture(img); // zero copy: the texture aliases px

  SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");
  view.setCamera(0, -1, 6, 0, 0, 0, 0, 1, 0, 50.0, 0.1, 100.0);
  sg.addDirectionalLight(-30.0, 60.0);
  const bool shadows = sg.setShadowsEnabled(true);
  sg.setShadowUpdateInterval(1); // the baker decides every frame
  for (int i = 0; i < 4; ++i) {  // settle: first load, first bakes
    sg.processEvents();
    view.render();
  }
  vtkActor *actor = node->actor();
  vtkTexture *tex = actor->GetTexture();
  vtkShadowMapBakerPass *baker = shadows ? findBaker(view.renderer()->GetPass()) : nullptr;
  chk(rowsEqual(gpuPixels(view, actor), px, 0, H), "the first draw uploaded the whole texture");

  std::printf("A. nothing is marked modified\n");
  const vtkMTimeType texT = tex->GetMTime(), actT = actor->GetMTime();
  const vtkMTimeType inT = tex->GetInput()->GetMTime();
  (void)sg.checkAndResetRenderNeeded();
  fillRows(px, 8, 12, 200);  // edited AND named
  fillRows(px, 20, 24, 120); // edited, NOT named
  node->texture_modified_rows(8, 12);
  chk(tex->GetMTime() == texT && actor->GetMTime() == actT && tex->GetInput()->GetMTime() == inT,
      "texture, its input and the actor keep their MTimes");
  chk(sg.checkAndResetRenderNeeded(), "a frame was requested");
  view.render();
  if (baker)
    chk(!baker->GetNeedUpdate(), "the next frame did not re-bake the shadow map");
  else
    std::printf("  (shadows unavailable: bake check skipped)\n");

  std::printf("B. exactly the named rows went up\n");
  std::vector<unsigned char> gpu = gpuPixels(view, actor);
  chk(rowsEqual(gpu, px, 8, 12), "rows [8,12) hold the new pixels");
  std::vector<unsigned char> old(static_cast<std::size_t>(W) * H * 4);
  std::memcpy(old.data(), px, old.size());
  fillRows(old.data(), 20, 24, 10); // what rows 20..23 held before the edit
  chk(rowsEqual(gpu, old.data(), 20, 24), "rows [20,24), edited but not named, are untouched");
  chk(rowsEqual(gpu, px, 0, 8) && rowsEqual(gpu, px, 12, 20) && rowsEqual(gpu, px, 24, H),
      "every other row unchanged");

  std::printf("C. the rectangle variant\n");
  fillRows(px, 25, 29, 77);                   // whole rows edited in memory...
  node->texture_modified_rect(4, 25, 10, 29); // ...only a 6x4 rectangle named
  view.render();
  gpu = gpuPixels(view, actor);
  bool rectOk = !gpu.empty(), outsideOld = !gpu.empty();
  for (int r = 25; r < 29 && !gpu.empty(); ++r)
    for (int x = 0; x < W; ++x) {
      const std::size_t o = (static_cast<std::size_t>(r) * W + x) * 4;
      const bool inside = x >= 4 && x < 10;
      if (inside && std::memcmp(gpu.data() + o, px + o, 4) != 0)
        rectOk = false;
      if (!inside && gpu[o] != 10) // still the original fill value
        outsideOld = false;
    }
  chk(rectOk, "the rectangle holds the new pixels");
  chk(outsideOld, "the rest of those rows is untouched");

  std::printf("D. clamping, and a call from another thread\n");
  fillRows(px, 0, H, 33);
  std::thread t([&] { node->texture_modified_rows(-5, 1000); });
  t.join();
  chk(!rowsEqual(gpuPixels(view, actor), px, 0, H), "an off-thread call waits for the owner");
  sg.processEvents();
  chk(rowsEqual(gpuPixels(view, actor), px, 0, H), "clamped to every row once pumped");
  node->texture_modified_rows(5, 5);
  node->texture_modified_rect(10, 10, 3, 20);
  chk(tex->GetMTime() == texT && actor->GetMTime() == actT, "empty ranges change nothing");

  std::printf("E. texture_modified() still re-uploads everything\n");
  fillRows(px, 0, H, 150);
  node->texture_modified();
  chk(tex->GetMTime() > texT && actor->GetMTime() > actT, "texture_modified bumps the MTimes");
  view.render();
  chk(rowsEqual(gpuPixels(view, actor), px, 0, H), "and the whole texture went up");
  if (baker)
    chk(baker->GetNeedUpdate(), "and the shadow map was re-baked");

  std::printf("F. a copy-path texture falls back\n");
  node->setTexture(img, /*zeroCopy=*/false);
  view.render();
  tex = actor->GetTexture();
  const vtkMTimeType copyT = tex->GetMTime();
  node->texture_modified_rows(0, 4);
  chk(tex->GetMTime() > copyT, "texture_modified_rows on a copy is texture_modified");

  std::printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
