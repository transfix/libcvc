/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib> // getenv: VTK_FORCE_MSAA, as the window reads it
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/Settings.h>
#include <cvc/gl/Viewport.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkObjectFactory.h>
#include <vtkOpenGLFramebufferObject.h>
#include <vtkOpenGLQuadHelper.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLShaderCache.h>
#include <vtkOpenGLState.h>
#include <vtkRenderer.h>
#include <vtkRendererDelegate.h>
#include <vtkShaderProgram.h>
#include <vtkSmartPointer.h>
#include <vtkTextureObject.h>
#include <vtk_glad.h> // GL_DRAW_FRAMEBUFFER_BINDING, GL_COLOR_BUFFER_BIT

namespace {
// A textured viewport's multisampled draw, resolved into the texture a UI shows as VTK 9.5 resolves
// the window's frame when it shows it (vtkOpenGLRenderWindow::ResolveFlipRenderFramebuffer): each
// pixel's samples averaged in gamma 2.2, so its anti-aliased edges are the ones the window would
// have drawn.
constexpr char kResolveShader[] = R"***(//VTK::System::Dec
  in vec2 texCoord;
  uniform sampler2DMS tex;
  uniform int samplecount;
  //VTK::Output::Dec

  void main()
  {
    float gamma = 2.2;
    ivec2 itexcoords = ivec2(floor(textureSize(tex) * texCoord));
    vec3 accumulate = vec3(0.0,0.0,0.0);
    float alpha = 0.0;
    for (int i = 0; i < samplecount; i++)
    {
      vec4 sampleValue = texelFetch(tex, itexcoords, i);
      accumulate += pow(sampleValue.rgb, vec3(gamma));
      alpha += sampleValue.a;
    }
    accumulate /= float(samplecount);
    gl_FragData[0] = vec4(pow(accumulate, vec3(1.0/gamma)), alpha/float(samplecount));
  }
  )***";

// Whether the window's pixels reach their reader through that shader, as VTK 9.5 decides it. The
// window shows its frame through it (Frame(): a multisampled colour texture, unless $VTK_FORCE_MSAA
// says otherwise or the driver is one whose multisampled textures VTK does not trust), but a read
// of its pixels (GetPixelData: a capture, an offscreen window's frame) resolves them with a blit.
// So an offscreen window's views blit too, and its captures are what they were when the views drew
// into the window.
bool resolvesWithShader(vtkOpenGLRenderWindow *rw, int samples, bool colorTexture) {
  if (samples <= 1 || !colorTexture || rw->GetOffScreenRendering())
    return false;
  if (const char *env = std::getenv("VTK_FORCE_MSAA"))
    return std::strlen(env) ? std::atoi(env) == 1 : true;
  vtkOpenGLState *st = rw->GetState();
  static const char *const kBug[][3] = {
      {"Intel", "", "Mesa Intel"}, {"X.Org", "", "AMD"}, {"Mesa", "", "NV"}};
  for (const auto &b : kBug)
    if (st->GetVendor().find(b[0]) == 0 && st->GetVersion().find(b[1]) == 0 &&
        st->GetRenderer().find(b[2]) == 0)
      return false;
  return true;
}

// Shows a throttled viewport's last pixels in place of its scene (Viewport::setRefreshHz), and
// draws a textured one into its texture (Viewport::setTextureSize). While it is Used,
// vtkRenderer::Render() calls it instead of rendering, before even its Draw check.
class LastPixels : public vtkRendererDelegate {
public:
  static LastPixels *New();
  vtkTypeMacro(LastPixels, vtkRendererDelegate);
  std::function<void(vtkRenderer *)> show;
  void Render(vtkRenderer *r) override {
    if (show)
      show(r);
  }

protected:
  LastPixels() = default;
  ~LastPixels() override = default;
};
vtkStandardNewMacro(LastPixels);
} // namespace

namespace cvc {
namespace gl {

struct Viewport::Impl {
  std::string name;
  bool mirror = false;
  SceneGraph *scene = nullptr; // the scene this viewport draws (owned elsewhere)
  vtkSmartPointer<vtkRenderer> renderer;
  std::unique_ptr<CameraController> camera;
  // Kept so a BARE (unmanaged) viewport can lazily build its controller if
  // something asks for camera() after construction.
  cvc::app *app = nullptr;
  std::string cameraStatePath;
  bool managed = true;
  double region[4] = {0.0, 0.0, 1.0, 1.0}; // normalized, VTK y-up
  int layer = 0;
  bool visible = true;
  bool inputEnabled = true;            // routed input reaches this viewport
  vtkRenderer *mirrorSource = nullptr; // set only for a mirror viewport
  bool mirrorLive = true;              // re-sync from source every frame vs frozen once
  PropFilter mirrorFilter;             // which source props a re-sync copies (all when empty)
  // region/layer/visible mirrored to cvc::state so a PiP arrangement round-trips.
  std::unique_ptr<ViewportLayout> layout;

  // setRefreshHz: the redraw rate, and the pixels of the last draw.
  double refreshHz = 0.0;
  bool dirty = true;    // draw at the next render, whatever the rate
  bool drawing = false; // this render draws the scene and keeps its pixels
  long draws = 0;
  std::chrono::steady_clock::time_point lastDraw{};
  bool kept = false;              // `cache` holds the last draw
  int keptRect[4] = {0, 0, 0, 0}; // its x, y, w, h: window pixels from the lower left
  int keptWindow[2] = {0, 0};     // the window size it was kept at (the cache's size)
  vtkSmartPointer<vtkOpenGLFramebufferObject> cache;
  vtkSmartPointer<LastPixels> lastPixels;    // the renderer's delegate, once a rate is set
  vtkSmartPointer<vtkCallbackCommand> onEnd; // the renderer's EndEvent: drawn()
  unsigned long onEndTag = 0;

  // setTextureSize: drawn into a texture of its own. `shown` is what a UI shows (one sample a
  // pixel, its GL name stable across sizes and sample counts); `target` is what the scene is drawn
  // into, at the window's samples, and `resolved` writes `shown`.
  int texAsk[2] = {0, 0};  // the size asked for (0: drawn into the window)
  int texSize[2] = {0, 0}; // the size it draws at: the asked one within the window
  bool texDrawn = false;   // `shown` holds a draw at texSize
  int texSamples = -1;     // the samples `target` was made with
  vtkSmartPointer<vtkTextureObject> shown;
  vtkSmartPointer<vtkOpenGLFramebufferObject> target, resolved;
  std::unique_ptr<vtkOpenGLQuadHelper> resolveQuad;
  std::function<void()> beforeDraw; // the manager's: a mirror's re-sync

  bool textured() const { return texAsk[0] > 0; }

  // The delegate a rate or a texture needs, made once.
  void ensureDelegate() {
    if (lastPixels)
      return;
    lastPixels = vtkSmartPointer<LastPixels>::New();
    lastPixels->UsedOff();
    lastPixels->show = [this](vtkRenderer *r) {
      if (textured())
        drawTextured(r);
      else
        show(r);
    };
    renderer->SetDelegate(lastPixels);
  }

  void applyVisible(bool on) {
    if (on && !visible)
      dirty = true; // what it kept is stale
    visible = on;
    renderer->SetDraw(on ? 1 : 0); // skipped in the composite when off
    if (lastPixels)                // the delegate runs before VTK's Draw check
      lastPixels->SetUsed(on && textured() ? 1 : 0);
  }

  // texSize from the asked size and the window's, and the renderer's viewport on it: the
  // texture's rect at the window's lower left, so its display coordinates are texture pixels.
  void fitTexture() {
    vtkRenderWindow *win = renderer->GetRenderWindow();
    const int *ws = win ? win->GetSize() : nullptr;
    if (!textured() || !ws || ws[0] <= 0 || ws[1] <= 0)
      return;
    texSize[0] = std::clamp(texAsk[0], 1, ws[0]);
    texSize[1] = std::clamp(texAsk[1], 1, ws[1]);
    const double vx = static_cast<double>(texSize[0]) / ws[0];
    const double vy = static_cast<double>(texSize[1]) / ws[1];
    const double *now = renderer->GetViewport();
    if (now[0] != 0.0 || now[1] != 0.0 || now[2] != vx || now[3] != vy)
      renderer->SetViewport(0.0, 0.0, vx, vy);
  }

  // `shown` at texSize: made, or resized (its picture is then gone). Needs the window's context.
  bool ensureShown(vtkOpenGLRenderWindow *rw) {
    if (!rw || !textured())
      return false;
    fitTexture();
    if (shown && shown->GetHandle() && static_cast<int>(shown->GetWidth()) == texSize[0] &&
        static_cast<int>(shown->GetHeight()) == texSize[1])
      return true;
    if (!rw->GetInitialized())
      return false;
    rw->MakeCurrent();
    if (!rw->IsCurrent())
      return false;
    if (!shown) {
      shown = vtkSmartPointer<vtkTextureObject>::New();
      shown->SetContext(rw);
      shown->SetWrapS(vtkTextureObject::ClampToEdge);
      shown->SetWrapT(vtkTextureObject::ClampToEdge);
      shown->SetMinificationFilter(vtkTextureObject::Nearest);
      shown->SetMagnificationFilter(vtkTextureObject::Nearest);
    }
    if (!shown->GetHandle())
      shown->Allocate2D(texSize[0], texSize[1], 4, VTK_UNSIGNED_CHAR);
    else
      shown->Resize(texSize[0], texSize[1]);
    texDrawn = false;
    return shown->GetHandle() != 0;
  }

  // The framebuffers a draw needs: `target` as the window's own render framebuffer is made
  // (vtkOpenGLRenderWindow::CreateFramebuffers: its samples, depth bits and stencil), at texSize,
  // and `resolved` over `shown`. Inside the window's render, where its framebuffers exist.
  bool ensureTarget(vtkOpenGLRenderWindow *rw) {
    if (!ensureShown(rw))
      return false;
    vtkOpenGLState *st = rw->GetState();
    vtkOpenGLFramebufferObject *frame = rw->GetRenderFramebuffer();
    const int samples = frame ? frame->GetMultiSamples() : 0;
    if (target && samples != texSamples) {
      target->ReleaseGraphicsResources(rw);
      target = nullptr;
    }
    if (!target) {
      target = vtkSmartPointer<vtkOpenGLFramebufferObject>::New();
      target->SetContext(rw);
#ifdef GL_TEXTURE_2D_MULTISAMPLE
      const bool textures = true;
#else
      const bool textures = samples == 0;
#endif
      st->PushFramebufferBindings();
      const bool ok = target->PopulateFramebuffer(
          texSize[0], texSize[1], textures, 1, VTK_UNSIGNED_CHAR, /*wantDepthAttachment=*/true,
          rw->GetRenderBufferTargetDepthSize(), samples, rw->GetStencilCapable() != 0);
      st->PopFramebufferBindings();
      if (!ok) {
        target->ReleaseGraphicsResources(rw);
        target = nullptr;
        return false;
      }
      texSamples = samples;
    } else if (target->GetLastSize()[0] != texSize[0] || target->GetLastSize()[1] != texSize[1]) {
      target->Resize(texSize[0], texSize[1]);
    }
    if (!resolved) {
      resolved = vtkSmartPointer<vtkOpenGLFramebufferObject>::New();
      resolved->SetContext(rw);
      st->PushFramebufferBindings();
      resolved->Bind();
      resolved->AddColorAttachment(0, shown);
      resolved->ActivateDrawBuffer(0);
      st->PopFramebufferBindings();
    } else if (resolved->GetLastSize()[0] != texSize[0] ||
               resolved->GetLastSize()[1] != texSize[1]) {
      resolved->Resize(texSize[0], texSize[1]);
    }
    return true;
  }

  // `target` into `shown`, as the window resolves its frame, and `shown` made opaque: a view's
  // alpha is the scene's, not coverage, and a UI blends what it shows.
  void resolve(vtkOpenGLRenderWindow *rw) {
    vtkOpenGLState *st = rw->GetState();
    const int w = texSize[0], h = texSize[1];
    vtkOpenGLState::ScopedglEnableDisable scissor(st, GL_SCISSOR_TEST);
    vtkOpenGLState::ScopedglEnableDisable depthTest(st, GL_DEPTH_TEST);
    vtkOpenGLState::ScopedglEnableDisable blend(st, GL_BLEND);
    vtkOpenGLState::ScopedglViewport viewport(st);
    st->vtkglDisable(GL_SCISSOR_TEST);
    st->vtkglDisable(GL_DEPTH_TEST);
    st->vtkglDisable(GL_BLEND);
    st->vtkglViewport(0, 0, w, h);
    resolved->Bind(GL_DRAW_FRAMEBUFFER);
    resolved->ActivateDrawBuffer(0);
    vtkTextureObject *ms = target->GetColorAttachmentAsTextureObject(0);
    bool done = false;
    if (resolvesWithShader(rw, texSamples, ms != nullptr)) {
      // Made once; where it does not compile (a GL without multisampled samplers) it blits, as
      // the window does, without trying again each draw.
      if (!resolveQuad)
        resolveQuad.reset(new vtkOpenGLQuadHelper(rw, nullptr, kResolveShader, ""));
      else if (resolveQuad->Program && resolveQuad->Program->GetCompiled())
        rw->GetShaderCache()->ReadyShaderProgram(resolveQuad->Program);
      if (resolveQuad->Program && resolveQuad->Program->GetCompiled()) {
        ms->Activate();
        resolveQuad->Program->SetUniformi("samplecount", texSamples);
        resolveQuad->Program->SetUniformi("tex", ms->GetTextureUnit());
        resolveQuad->Render();
        ms->Deactivate();
        done = true;
      }
    }
    if (!done) {
      target->Bind(GL_READ_FRAMEBUFFER);
      target->ActivateReadBuffer(0);
      st->vtkglBlitFramebuffer(0, 0, w, h, 0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    }
    st->vtkglColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_TRUE);
    st->vtkglClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    st->vtkglClear(GL_COLOR_BUFFER_BIT);
    st->vtkglColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  }

  // A textured viewport's turn in the window's render: draw its scene into its texture when it is
  // due, else leave the last picture there.
  void drawTextured(vtkRenderer *r) {
    auto *rw = vtkOpenGLRenderWindow::SafeDownCast(r->GetRenderWindow());
    if (!rw || !visible)
      return;
    const auto now = std::chrono::steady_clock::now();
    fitTexture();
    const bool sized = shown && static_cast<int>(shown->GetWidth()) == texSize[0] &&
                       static_cast<int>(shown->GetHeight()) == texSize[1];
    const bool due = dirty || !texDrawn || !sized || refreshHz <= 0.0 ||
                     std::chrono::duration<double>(now - lastDraw).count() >= 1.0 / refreshHz;
    if (!due)
      return;
    if (beforeDraw)
      beforeDraw();
    if (!ensureTarget(rw))
      return;
    dirty = false;
    lastDraw = now;
    vtkOpenGLState *st = rw->GetState();
    st->PushFramebufferBindings();
    target->Bind();
    target->ActivateDrawBuffer(0);
    target->ActivateReadBuffer(0);
    lastPixels->UsedOff(); // the renderer's own Render(), this once
    r->Render();
    lastPixels->UsedOn();
    resolve(rw);
    st->PopFramebufferBindings();
    texDrawn = true;
  }

  void releaseTexture() {
    auto *rw = vtkOpenGLRenderWindow::SafeDownCast(renderer->GetRenderWindow());
    if (rw && (shown || target || resolved || resolveQuad))
      rw->MakeCurrent();
    if (resolveQuad)
      resolveQuad->ReleaseGraphicsResources(rw);
    resolveQuad.reset();
    if (resolved)
      resolved->ReleaseGraphicsResources(rw);
    if (target)
      target->ReleaseGraphicsResources(rw);
    if (shown)
      shown->ReleaseGraphicsResources(rw);
    resolved = target = nullptr;
    shown = nullptr;
    texDrawn = false;
    texSamples = -1;
  }

  // The renderer finished drawing its scene: count it, and keep its pixels if it is throttled.
  // This runs before any layer above it draws, so what is kept is this viewport's alone.
  void drawn(vtkRenderer *r) {
    ++draws;
    if (!drawing)
      return;
    drawing = false;
    auto *rw = vtkOpenGLRenderWindow::SafeDownCast(r->GetRenderWindow());
    int w = 0, h = 0, x = 0, y = 0;
    r->GetTiledSizeAndOrigin(&w, &h, &x, &y);
    const int *ws = rw ? rw->GetSize() : nullptr;
    kept = false;
    if (!ws || w <= 0 || h <= 0 || ws[0] <= 0 || ws[1] <= 0)
      return;
    vtkOpenGLState *st = rw->GetState();
    int drawFb = 0; // what the renderer drew into: the window's render framebuffer
    st->vtkglGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &drawFb);
    st->PushFramebufferBindings();
    if (!cache) {
      cache = vtkSmartPointer<vtkOpenGLFramebufferObject>::New();
      cache->SetContext(rw);
    }
    if (keptWindow[0] == 0) {
      cache->PopulateFramebuffer(ws[0], ws[1], /*useTextures=*/true, 1, VTK_UNSIGNED_CHAR,
                                 /*wantDepthAttachment=*/false, 0, /*multisamples=*/0);
      if (vtkTextureObject *t = cache->GetColorAttachmentAsTextureObject(0)) {
        t->SetMinificationFilter(vtkTextureObject::Nearest);
        t->SetMagnificationFilter(vtkTextureObject::Nearest);
      }
    } else if (keptWindow[0] != ws[0] || keptWindow[1] != ws[1]) {
      cache->Resize(ws[0], ws[1]);
    }
    keptWindow[0] = ws[0];
    keptWindow[1] = ws[1];
    // The same rectangle on both sides: a multisampled source resolves only to an identical one.
    st->vtkglBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<unsigned int>(drawFb));
    st->vtkglReadBuffer(drawFb ? GL_COLOR_ATTACHMENT0 : GL_BACK);
    cache->Bind(GL_DRAW_FRAMEBUFFER);
    cache->ActivateDrawBuffer(0);
    {
      vtkOpenGLState::ScopedglEnableDisable scissor(st, GL_SCISSOR_TEST); // a blit honours it
      st->vtkglDisable(GL_SCISSOR_TEST);
      const int ext[4] = {x, x + w, y, y + h};
      vtkOpenGLFramebufferObject::Blit(ext, ext, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    }
    st->PopFramebufferBindings();
    keptRect[0] = x;
    keptRect[1] = y;
    keptRect[2] = w;
    keptRect[3] = h;
    kept = true;
  }

  // In place of the scene: the kept pixels, drawn as a quad (a blit cannot write a multisampled
  // framebuffer in GLES / WebGL 2), and its depth cleared, as the renderer clears it when it draws.
  void show(vtkRenderer *r) {
    auto *rw = vtkOpenGLRenderWindow::SafeDownCast(r->GetRenderWindow());
    vtkTextureObject *tex = kept && cache ? cache->GetColorAttachmentAsTextureObject(0) : nullptr;
    if (!rw || !tex)
      return;
    vtkOpenGLState *st = rw->GetState();
    const int x = keptRect[0], y = keptRect[1], w = keptRect[2], h = keptRect[3];
    vtkOpenGLState::ScopedglEnableDisable depthTest(st, GL_DEPTH_TEST);
    vtkOpenGLState::ScopedglEnableDisable blend(st, GL_BLEND);
    vtkOpenGLState::ScopedglEnableDisable scissor(st, GL_SCISSOR_TEST);
    vtkOpenGLState::ScopedglScissor scissorBox(st);
    vtkOpenGLState::ScopedglDepthMask depthMask(st);
    vtkOpenGLState::ScopedglViewport viewport(st);
    st->vtkglEnable(GL_SCISSOR_TEST);
    st->vtkglScissor(x, y, w, h);
    st->vtkglDepthMask(GL_TRUE);
    st->vtkglClearDepth(1.0);
    st->vtkglClear(GL_DEPTH_BUFFER_BIT);
    st->vtkglDisable(GL_SCISSOR_TEST);
    st->vtkglDisable(GL_DEPTH_TEST);
    st->vtkglDisable(GL_BLEND);
    st->vtkglViewport(x, y, w, h);
    // Texel for pixel: the quad fills the rect and its texture coordinates are the rect's edges,
    // so each pixel's centre samples the texel it was copied from.
    const float W = static_cast<float>(keptWindow[0]), H = static_cast<float>(keptWindow[1]);
    float tcoords[] = {x / W,       y / H,       (x + w) / W, y / H,
                       (x + w) / W, (y + h) / H, x / W,       (y + h) / H};
    float verts[] = {-1.f, -1.f, 0.f, 1.f, -1.f, 0.f, 1.f, 1.f, 0.f, -1.f, 1.f, 0.f};
    tex->CopyToFrameBuffer(tcoords, verts, nullptr, nullptr);
  }

  // object -> state: publish the current placement (set() does not call back into
  // the apply callback, so this does not loop).
  void writeLayout() {
    if (!layout)
      return;
    ViewportLayout::Values v;
    for (int i = 0; i < 4; ++i)
      v.region[i] = region[i];
    v.layer = layer;
    v.visible = visible;
    layout->set(v);
  }
};

Viewport::Viewport(cvc::app &app, SceneGraph &scene, const std::string &cameraStatePath,
                   const std::string &name, bool mirror, bool managed)
    : m_impl(new Impl) {
  m_impl->name = name;
  m_impl->mirror = mirror;
  m_impl->scene = &scene;
  m_impl->app = &app;
  m_impl->cameraStatePath = cameraStatePath;
  m_impl->managed = managed;
  m_impl->renderer = vtkSmartPointer<vtkRenderer>::New();
  m_impl->renderer->SetViewport(m_impl->region[0], m_impl->region[1], m_impl->region[2],
                                m_impl->region[3]);
  m_impl->renderer->SetLayer(m_impl->layer);
  // Counts each draw of its scene and keeps the pixels of a throttled one (setRefreshHz).
  m_impl->onEnd = vtkSmartPointer<vtkCallbackCommand>::New();
  m_impl->onEnd->SetClientData(m_impl.get());
  m_impl->onEnd->SetCallback([](vtkObject *caller, unsigned long, void *self, void *) {
    static_cast<Impl *>(self)->drawn(static_cast<vtkRenderer *>(caller));
  });
  m_impl->onEndTag = m_impl->renderer->AddObserver(vtkCommand::EndEvent, m_impl->onEnd);

  // A BARE (unmanaged) viewport stops here: no eager CameraController and no
  // ViewportLayout, so it writes NO ".viewers.<name>.camera/.layout" state at
  // construction — matching the classic single-view SceneRenderer. camera()
  // builds the controller lazily if something ever asks (HostStyle does not).
  if (!managed)
    return;

  // This viewport's own camera controller, rooted at its own state path. Built
  // with the low-level (app, path) ctor and wired to THIS renderer/camera/scene —
  // it never attaches to the window interactor (the manager's router owns that).
  m_impl->camera.reset(new CameraController(app, cameraStatePath));
  m_impl->camera->setCamera(m_impl->renderer->GetActiveCamera());
  m_impl->camera->setRenderer(m_impl->renderer);
  m_impl->camera->setScene(&scene);

  // Placement (region/layer/visible) mirrored to cvc::state. The apply callback
  // is the STATE -> object direction: a write from a script / config / restored
  // layout drives the renderer directly, and never writes state back (no loop).
  // The manager re-syncs its window layer count each render(), so a layer change
  // arriving this way is composited correctly.
  Impl *impl = m_impl.get();
  const std::string layoutPath = ViewportLayout::viewerStatePath(scene.getStatePrefix(), name);
  impl->layout.reset(new ViewportLayout(app, layoutPath, [impl](ViewportLayout::Values v) {
    for (int i = 0; i < 4; ++i)
      impl->region[i] = v.region[i];
    impl->layer = v.layer;
    if (!impl->textured()) // a textured one's renderer draws at its texture's rect
      impl->renderer->SetViewport(v.region[0], v.region[1], v.region[2], v.region[3]);
    impl->renderer->SetLayer(v.layer);
    impl->applyVisible(v.visible); // sets visible, and marks a throttled one stale when shown
  }));
}

Viewport::~Viewport() {
  // The renderer can outlive this viewport (the window holds it too): leave it nothing that points
  // back here.
  m_impl->renderer->RemoveObserver(m_impl->onEndTag);
  if (m_impl->lastPixels) {
    m_impl->renderer->SetDelegate(nullptr);
    m_impl->lastPixels->show = nullptr;
  }
}

const std::string &Viewport::name() const { return m_impl->name; }

void Viewport::setRegion(double x0, double y0, double x1, double y1) {
  m_impl->region[0] = x0;
  m_impl->region[1] = y0;
  m_impl->region[2] = x1;
  m_impl->region[3] = y1;
  if (!m_impl->textured()) // a textured one's renderer draws at its texture's rect
    m_impl->renderer->SetViewport(x0, y0, x1, y1);
  m_impl->writeLayout();
}

void Viewport::region(double out[4]) const {
  for (int i = 0; i < 4; ++i)
    out[i] = m_impl->region[i];
}

void Viewport::setLayer(int layer) {
  m_impl->layer = layer;
  m_impl->renderer->SetLayer(layer);
  m_impl->writeLayout();
}
int Viewport::layer() const { return m_impl->layer; }

void Viewport::setVisible(bool on) {
  m_impl->applyVisible(on);
  m_impl->writeLayout();
}
bool Viewport::visible() const { return m_impl->visible; }

void Viewport::setBackground(double r, double g, double b, bool opaque) {
  m_impl->renderer->SetBackground(r, g, b);
  // Opaque panel clears its own rect (default); a translucent overlay must
  // preserve the colour buffer under it or it wipes the base view.
  m_impl->renderer->SetPreserveColorBuffer(opaque ? 0 : 1);
}

CameraController &Viewport::camera() {
  // A bare (unmanaged) viewport deferred its controller; build it on first use
  // so a stray caller still gets one wired to this renderer/scene. In the
  // normal managed path it already exists and this is a straight return.
  if (!m_impl->camera) {
    m_impl->camera.reset(new CameraController(*m_impl->app, m_impl->cameraStatePath));
    m_impl->camera->setCamera(m_impl->renderer->GetActiveCamera());
    m_impl->camera->setRenderer(m_impl->renderer);
    m_impl->camera->setScene(m_impl->scene);
  }
  return *m_impl->camera;
}
SceneGraph &Viewport::scene() const { return *m_impl->scene; }
vtkRenderer *Viewport::renderer() const { return m_impl->renderer; }
bool Viewport::isMirror() const { return m_impl->mirror; }
vtkRenderer *Viewport::mirrorSource() const { return m_impl->mirrorSource; }
void Viewport::setMirrorSource(vtkRenderer *src) { m_impl->mirrorSource = src; }
bool Viewport::mirrorLive() const { return m_impl->mirrorLive; }
void Viewport::setMirrorLive(bool on) { m_impl->mirrorLive = on; }
void Viewport::setMirrorFilter(PropFilter keep) { m_impl->mirrorFilter = std::move(keep); }
const Viewport::PropFilter &Viewport::mirrorFilter() const { return m_impl->mirrorFilter; }
void Viewport::setInputEnabled(bool on) { m_impl->inputEnabled = on; }
bool Viewport::inputEnabled() const { return m_impl->inputEnabled; }

void Viewport::setRefreshHz(double hz) {
  Impl &m = *m_impl;
  m.refreshHz = std::isfinite(hz) && hz > 0.0 ? hz : 0.0;
  if (m.refreshHz > 0.0)
    m.ensureDelegate();
  if (m.lastPixels && m.refreshHz == 0.0 && !m.textured())
    m.lastPixels->UsedOff();
  m.dirty = true;
}
double Viewport::refreshHz() const { return m_impl->refreshHz; }
void Viewport::invalidate() { m_impl->dirty = true; }
long Viewport::draws() const { return m_impl->draws; }

void Viewport::setTextureSize(int w, int h) {
  Impl &m = *m_impl;
  if (w <= 0 || h <= 0)
    w = h = 0;
  if (w == m.texAsk[0] && h == m.texAsk[1])
    return;
  const bool was = m.textured();
  m.texAsk[0] = w;
  m.texAsk[1] = h;
  m.dirty = true;
  if (m.textured()) {
    m.ensureDelegate();
    m.lastPixels->SetUsed(m.visible ? 1 : 0); // it draws the scene into the texture
    m.kept = false;                           // the window's pixels it kept are no longer its
    m.fitTexture();
  } else if (was) { // back into the window, where region() says
    m.renderer->SetViewport(m.region[0], m.region[1], m.region[2], m.region[3]);
    m.lastPixels->UsedOff();
    m.releaseTexture();
  }
}

bool Viewport::textured() const { return m_impl->textured(); }

void Viewport::textureSize(int out[2]) const {
  const Impl &m = *m_impl;
  out[0] = out[1] = 0;
  if (!m.textured())
    return;
  vtkRenderWindow *win = m.renderer->GetRenderWindow();
  const int *ws = win ? win->GetSize() : nullptr;
  out[0] = ws && ws[0] > 0 ? std::min(m.texAsk[0], ws[0]) : m.texAsk[0];
  out[1] = ws && ws[1] > 0 ? std::min(m.texAsk[1], ws[1]) : m.texAsk[1];
}

unsigned int Viewport::texture() {
  Impl &m = *m_impl;
  auto *rw = vtkOpenGLRenderWindow::SafeDownCast(m.renderer->GetRenderWindow());
  return m.ensureShown(rw) ? m.shown->GetHandle() : 0u;
}

void Viewport::windowToRenderer(double x, double y, double out[2]) const {
  const Impl &m = *m_impl;
  out[0] = x;
  out[1] = y;
  vtkRenderWindow *win = m.renderer->GetRenderWindow();
  const int *ws = win ? win->GetSize() : nullptr;
  if (!m.textured() || !ws || ws[0] <= 0 || ws[1] <= 0)
    return;
  const double w = (m.region[2] - m.region[0]) * ws[0], h = (m.region[3] - m.region[1]) * ws[1];
  if (!(w > 0.0) || !(h > 0.0))
    return;
  int ts[2];
  textureSize(ts);
  out[0] = (x - m.region[0] * ws[0]) * ts[0] / w;
  out[1] = (y - m.region[1] * ws[1]) * ts[1] / h;
}

void Viewport::rendererToWindow(double x, double y, double out[2]) const {
  const Impl &m = *m_impl;
  out[0] = x;
  out[1] = y;
  vtkRenderWindow *win = m.renderer->GetRenderWindow();
  const int *ws = win ? win->GetSize() : nullptr;
  int ts[2];
  textureSize(ts);
  if (!m.textured() || !ws || ws[0] <= 0 || ws[1] <= 0 || ts[0] <= 0 || ts[1] <= 0)
    return;
  const double w = (m.region[2] - m.region[0]) * ws[0], h = (m.region[3] - m.region[1]) * ws[1];
  out[0] = m.region[0] * ws[0] + x * w / ts[0];
  out[1] = m.region[1] * ws[1] + y * h / ts[1];
}

void Viewport::setBeforeDraw(std::function<void()> fn) { m_impl->beforeDraw = std::move(fn); }

void Viewport::releaseGraphics() {
  Impl &m = *m_impl;
  m.releaseTexture();
  if (m.cache) {
    m.cache->ReleaseGraphicsResources(m.renderer->GetRenderWindow());
    m.cache = nullptr;
    m.kept = false;
    m.keptWindow[0] = m.keptWindow[1] = 0;
  }
}

bool Viewport::beginRender() {
  Impl &m = *m_impl;
  m.drawing = false;
  if (!m.visible)
    return false;
  if (m.textured()) // it decides as the render reaches it (drawTextured)
    return true;
  vtkRenderWindow *win = m.renderer->GetRenderWindow();
  // Throttled: a rate, a window, and an opaque rect (a translucent one draws over a picture that
  // changes under it).
  if (!m.lastPixels || m.refreshHz <= 0.0 || !win || m.renderer->GetPreserveColorBuffer()) {
    if (m.lastPixels)
      m.lastPixels->UsedOff();
    return true;
  }
  int w = 0, h = 0, x = 0, y = 0;
  m.renderer->GetTiledSizeAndOrigin(&w, &h, &x, &y);
  const int *ws = win->GetSize();
  const bool same = m.kept && m.keptRect[0] == x && m.keptRect[1] == y && m.keptRect[2] == w &&
                    m.keptRect[3] == h && m.keptWindow[0] == ws[0] && m.keptWindow[1] == ws[1];
  const auto now = std::chrono::steady_clock::now();
  const bool due = m.dirty || !same ||
                   std::chrono::duration<double>(now - m.lastDraw).count() >= 1.0 / m.refreshHz;
  m.lastPixels->SetUsed(!due);
  if (due) {
    m.drawing = true;
    m.dirty = false;
    m.lastDraw = now;
  }
  return due;
}

} // namespace gl
} // namespace cvc
