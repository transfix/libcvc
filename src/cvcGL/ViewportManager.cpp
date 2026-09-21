/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/Viewport.h>
#include <cvc/gl/ViewportManager.h>
#include <map>
#include <set>
#include <stdexcept>
#include <vtkCollection.h> // vtkCollectionSimpleIterator (reentrant prop traversal)
#include <vtkNew.h>
#include <vtkOutputWindow.h> // route VTK's ERR/WARN to stderr, not a Win32 message box
#include <vtkPNGWriter.h>
#include <vtkProp.h>
#include <vtkPropCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkWindowToImageFilter.h>

namespace cvc {
namespace gl {

namespace {
// Copy the source renderer's current props into the mirror's renderer, so the
// mirror shows the live scene (any node added / re-meshed since last frame) from
// its OWN camera. The props are shared by reference — one geometry, drawn twice.
void syncMirrorProps(Viewport &mirror) {
  vtkRenderer *dst = mirror.renderer();
  vtkRenderer *src = mirror.mirrorSource();
  if (!dst || !src)
    return;
  dst->RemoveAllViewProps();
  vtkPropCollection *props = src->GetViewProps();
  vtkCollectionSimpleIterator it;
  props->InitTraversal(it);
  while (vtkProp *p = props->GetNextProp(it))
    dst->AddViewProp(p);
}
} // namespace

struct ViewportManager::Impl {
  cvc::app *app = nullptr;
  std::string name = "main";
  bool offscreen = true;
  bool closed = false;
  vtkSmartPointer<vtkRenderWindow> window;
  vtkSmartPointer<vtkRenderWindowInteractor> interactor; // onscreen only

  std::vector<std::unique_ptr<Viewport>> viewports; // owns them; [0] is primary
  std::map<std::string, Viewport *> byName;
  // A SceneGraph attaches to exactly ONE renderer (setRenderer replaces the
  // previous binding and blanks that view). We track which scenes a SCENE
  // viewport already drives so a second addSceneViewport over the same scene is
  // a loud error, not a silently blanked first viewport. Mirror viewports do
  // NOT attach the scene (they reuse another viewport's props), so they are not
  // tracked here.
  std::set<SceneGraph *> attachedScenes;

  void requireOpen() const {
    if (closed || !window)
      throw std::runtime_error("ViewportManager: window is closed");
  }

  int highestLayer() const {
    int m = 0;
    for (const auto &v : viewports)
      m = std::max(m, v->layer());
    return m;
  }

  // Keep the window's layer count strictly greater than the highest layer index
  // in use, or VTK silently drops the top layer.
  void syncLayerCount() { window->SetNumberOfLayers(highestLayer() + 1); }
};

ViewportManager::ViewportManager(SceneGraph &mainScene, int width, int height, bool offscreen,
                                 const std::string &name)
    : m_impl(new Impl) {
  if (width < 1 || height < 1)
    throw std::invalid_argument("ViewportManager: width and height must be >= 1");

  // Same one-time VTK diagnostic routing as SceneRenderer: send ERR/WARN to
  // stderr instead of a Win32 message box (the shadow-map pass logs cosmetic
  // driver warnings), and never prompt.
  static bool s_vtkOutputConfigured = false;
  if (!s_vtkOutputConfigured) {
    if (auto *ow = vtkOutputWindow::GetInstance()) {
      ow->SetDisplayModeToAlwaysStdErr();
      ow->SetPromptUser(0);
    }
    s_vtkOutputConfigured = true;
  }

  m_impl->app = &mainScene.appContext();
  m_impl->name = name;
  m_impl->offscreen = offscreen;
  m_impl->window = vtkSmartPointer<vtkRenderWindow>::New();
  m_impl->window->SetOffScreenRendering(offscreen ? 1 : 0);
  m_impl->window->SetSize(width, height);
  m_impl->window->SetNumberOfLayers(1);

  if (!offscreen) {
    m_impl->window->SetWindowName("cvcGL");
    // Initialize(), never Start(): Start() would take the event loop away from
    // the caller (see SceneRenderer). The interactor only delivers
    // resize/close/mouse events, drained on demand by processUIEvents().
    m_impl->interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    m_impl->interactor->SetRenderWindow(m_impl->window);
    m_impl->interactor->Initialize();
  }

  // Auto-create the full-screen PRIMARY viewport over the main scene at layer 0,
  // so the common single-view case behaves exactly like a SceneRenderer.
  const std::string camPath = CameraController::viewerStatePath(mainScene.getStatePrefix(), name);
  std::unique_ptr<Viewport> primary(
      new Viewport(*m_impl->app, mainScene, camPath, name, /*mirror=*/false));
  primary->setRegion(0.0, 0.0, 1.0, 1.0);
  primary->setLayer(0);
  m_impl->window->AddRenderer(primary->renderer());

  // Attach the scene to the primary renderer (walks the scene, hands every
  // node's actor to the renderer), then frame it.
  mainScene.setRenderer(primary->renderer());
  mainScene.processEvents();
  primary->renderer()->ResetCamera();

  m_impl->attachedScenes.insert(&mainScene);
  m_impl->byName[name] = primary.get();
  m_impl->viewports.push_back(std::move(primary));
}

ViewportManager::~ViewportManager() {
  if (!m_impl || m_impl->closed)
    return;
  m_impl->closed = true;
  try {
    // Detach every scene BEFORE the window dies: the scenes hold actors that
    // belong to these renderers, and tearing the window down under them is how
    // offscreen backends crash at exit (see SceneRenderer::close).
    for (auto &v : m_impl->viewports) {
      if (!v->isMirror())
        v->scene().setRenderer(nullptr);
    }
    if (m_impl->interactor) {
      m_impl->interactor->TerminateApp();
      m_impl->interactor->SetRenderWindow(nullptr);
      m_impl->interactor = nullptr;
    }
    if (m_impl->window)
      m_impl->window->Finalize();
  } catch (...) {
    // Throwing out of a destructor during teardown is strictly worse than a
    // leaked GL context.
  }
  // Drop the viewports (and their renderers/cameras) before the window.
  m_impl->viewports.clear();
  m_impl->byName.clear();
  m_impl->attachedScenes.clear();
  m_impl->window = nullptr;
}

Viewport &ViewportManager::primary() {
  m_impl->requireOpen();
  return *m_impl->viewports.front();
}

Viewport &ViewportManager::viewport(const std::string &name) {
  m_impl->requireOpen();
  auto it = m_impl->byName.find(name);
  if (it == m_impl->byName.end())
    throw std::out_of_range("ViewportManager: no viewport named '" + name + "'");
  return *it->second;
}

bool ViewportManager::hasViewport(const std::string &name) const {
  return m_impl->byName.find(name) != m_impl->byName.end();
}

std::vector<std::string> ViewportManager::viewportNames() const {
  std::vector<std::string> names;
  names.reserve(m_impl->viewports.size());
  for (const auto &v : m_impl->viewports)
    names.push_back(v->name());
  return names;
}

Viewport &ViewportManager::addSceneViewport(const std::string &name, SceneGraph &scene,
                                            const double region[4], int layer) {
  m_impl->requireOpen();
  if (m_impl->byName.find(name) != m_impl->byName.end())
    throw std::invalid_argument("ViewportManager: a viewport named '" + name + "' already exists");
  if (m_impl->attachedScenes.find(&scene) != m_impl->attachedScenes.end())
    throw std::invalid_argument("ViewportManager: that SceneGraph is already drawn by another "
                                "viewport (a scene attaches to one renderer; use a mirror "
                                "viewport for an alternate view of it)");
  if (layer < 0)
    throw std::invalid_argument("ViewportManager: layer must be >= 0");

  const std::string camPath = CameraController::viewerStatePath(scene.getStatePrefix(), name);
  std::unique_ptr<Viewport> vp(new Viewport(*m_impl->app, scene, camPath, name, /*mirror=*/false));
  vp->setRegion(region[0], region[1], region[2], region[3]);
  vp->setLayer(layer);
  m_impl->window->AddRenderer(vp->renderer());

  scene.setRenderer(vp->renderer());
  scene.processEvents();
  vp->renderer()->ResetCamera();

  Viewport &ref = *vp;
  m_impl->attachedScenes.insert(&scene);
  m_impl->byName[name] = vp.get();
  m_impl->viewports.push_back(std::move(vp));
  // AFTER the push, so highestLayer() sees this viewport: the window's layer
  // count must exceed the highest layer index or VTK rejects the top layer
  // ("Invalid layer for renderer: not rendered") and never draws it.
  m_impl->syncLayerCount();
  return ref;
}

Viewport &ViewportManager::addMirrorViewport(const std::string &name,
                                             const std::string &sourceViewport,
                                             const double region[4], int layer) {
  m_impl->requireOpen();
  if (m_impl->byName.find(name) != m_impl->byName.end())
    throw std::invalid_argument("ViewportManager: a viewport named '" + name + "' already exists");
  auto srcIt = m_impl->byName.find(sourceViewport);
  if (srcIt == m_impl->byName.end())
    throw std::invalid_argument("ViewportManager: no source viewport named '" + sourceViewport +
                                "' to mirror");
  if (layer < 0)
    throw std::invalid_argument("ViewportManager: layer must be >= 0");

  Viewport *src = srcIt->second;
  SceneGraph &scene = src->scene(); // the mirror echoes the source's scene
  const std::string camPath = CameraController::viewerStatePath(scene.getStatePrefix(), name);
  std::unique_ptr<Viewport> vp(new Viewport(*m_impl->app, scene, camPath, name, /*mirror=*/true));
  vp->setRegion(region[0], region[1], region[2], region[3]);
  vp->setLayer(layer);
  vp->setMirrorSource(src->renderer());
  m_impl->window->AddRenderer(vp->renderer());

  // Prime the mirror with the source's current props so ResetCamera has bounds
  // to frame. It does NOT attach the scene (the source owns that), so it is not
  // tracked in attachedScenes and never detaches the scene in the destructor.
  syncMirrorProps(*vp);
  vp->renderer()->ResetCamera();

  Viewport &ref = *vp;
  m_impl->byName[name] = vp.get();
  m_impl->viewports.push_back(std::move(vp));
  m_impl->syncLayerCount();
  return ref;
}

void ViewportManager::render() {
  m_impl->requireOpen();
  // Drain each viewport's scene events (a re-meshed node appears without
  // re-attaching), then composite every layer in one pass. A scene shared by a
  // mirror viewport is drained once via its owning scene viewport; mirrors carry
  // the same SceneGraph* so processEvents() is idempotent-cheap when repeated.
  for (auto &v : m_impl->viewports)
    v->scene().processEvents();
  // Mirrors have no scene attachment of their own: refresh their props from the
  // source renderer so a node added / re-meshed this frame shows up in the
  // mirror too, then draw it with the mirror's own camera.
  for (auto &v : m_impl->viewports)
    if (v->isMirror())
      syncMirrorProps(*v);
  m_impl->window->Render();
}

void ViewportManager::resize(int width, int height) {
  m_impl->requireOpen();
  if (width < 1 || height < 1)
    throw std::invalid_argument("ViewportManager::resize: width and height must be >= 1");
  m_impl->window->SetSize(width, height);
}

void ViewportManager::resetCamera() {
  m_impl->requireOpen();
  Viewport &p = *m_impl->viewports.front();
  p.scene().processEvents();
  p.renderer()->ResetCamera();
}

void ViewportManager::processUIEvents() {
  m_impl->requireOpen();
  if (m_impl->interactor)
    m_impl->interactor->ProcessEvents();
}

bool ViewportManager::windowClosed() const {
  if (!m_impl || m_impl->closed || !m_impl->window)
    return true;
  if (!m_impl->interactor)
    return false; // offscreen has no window to close
  return m_impl->interactor->GetDone() != 0;
}

void ViewportManager::writePNG(const std::string &path) {
  render();
  vtkNew<vtkWindowToImageFilter> w2i;
  w2i->SetInput(m_impl->window);
  w2i->SetInputBufferTypeToRGB();
  w2i->ReadFrontBufferOff();
  w2i->Modified(); // else every PNG in a sequence is a copy of frame 0
  w2i->Update();
  vtkNew<vtkPNGWriter> writer;
  writer->SetFileName(path.c_str());
  writer->SetInputConnection(w2i->GetOutputPort());
  writer->Write();
}

std::vector<unsigned char> ViewportManager::frameRGB() {
  render();
  const int w = m_impl->window->GetSize()[0];
  const int h = m_impl->window->GetSize()[1];
  vtkSmartPointer<vtkUnsignedCharArray> buf = vtkSmartPointer<vtkUnsignedCharArray>::New();
  m_impl->window->GetPixelData(0, 0, w - 1, h - 1, /*front=*/0, buf);
  const unsigned char *p = buf->GetPointer(0);
  const size_t n = static_cast<size_t>(buf->GetNumberOfTuples()) * buf->GetNumberOfComponents();
  return std::vector<unsigned char>(p, p + n);
}

int ViewportManager::frameWidth() const {
  m_impl->requireOpen();
  return m_impl->window->GetSize()[0];
}

int ViewportManager::frameHeight() const {
  m_impl->requireOpen();
  return m_impl->window->GetSize()[1];
}

vtkRenderWindow *ViewportManager::renderWindow() const {
  m_impl->requireOpen();
  return m_impl->window;
}

} // namespace gl
} // namespace cvc
