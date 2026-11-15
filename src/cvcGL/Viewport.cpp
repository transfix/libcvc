/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <chrono>
#include <cmath>
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
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLState.h>
#include <vtkRenderer.h>
#include <vtkRendererDelegate.h>
#include <vtkSmartPointer.h>
#include <vtkTextureObject.h>
#include <vtk_glad.h> // GL_DRAW_FRAMEBUFFER_BINDING, GL_COLOR_BUFFER_BIT

namespace {
// Shows a throttled viewport's last pixels in place of its scene (Viewport::setRefreshHz). While
// it is Used, vtkRenderer::Render() calls it instead of rendering, before even its Draw check.
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

  void applyVisible(bool on) {
    if (on && !visible)
      dirty = true; // what it kept is stale
    visible = on;
    renderer->SetDraw(on ? 1 : 0); // skipped in the composite when off
    if (!on && lastPixels)
      lastPixels->UsedOff(); // the delegate runs before VTK's Draw check
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
  if (m.refreshHz > 0.0 && !m.lastPixels) {
    m.lastPixels = vtkSmartPointer<LastPixels>::New();
    Impl *impl = m_impl.get();
    m.lastPixels->show = [impl](vtkRenderer *r) { impl->show(r); };
    m.renderer->SetDelegate(m.lastPixels);
  }
  if (m.lastPixels && m.refreshHz == 0.0)
    m.lastPixels->UsedOff();
  m.dirty = true;
}
double Viewport::refreshHz() const { return m_impl->refreshHz; }
void Viewport::invalidate() { m_impl->dirty = true; }
long Viewport::draws() const { return m_impl->draws; }

bool Viewport::beginRender() {
  Impl &m = *m_impl;
  m.drawing = false;
  if (!m.visible)
    return false;
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
