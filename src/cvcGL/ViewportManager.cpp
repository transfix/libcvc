/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/core/state.h> // active_viewport focus, stored in cvc::state
#include <cvc/gl/CameraController.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/Viewport.h>
#include <cvc/gl/ViewportManager.h>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vtkActor2D.h>    // detect 2-D overlay props to skip in a mirror
#include <vtkCollection.h> // vtkCollectionSimpleIterator (reentrant prop traversal)
#include <vtkInteractorStyle.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h> // vtkStandardNewMacro
#include <vtkOutputWindow.h>  // route VTK's ERR/WARN to stderr, not a Win32 message box
#include <vtkPNGWriter.h>
#include <vtkProp.h>
#include <vtkPropCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkWindowToImageFilter.h>

// File-local interactor style: the ONE interactor-coupled piece of the input
// router. Onscreen it is installed on the manager's single interactor; each On*
// override just reads the event off this->Interactor and forwards to the
// manager's public, interactor-free route*() methods (which do the picking and
// dispatch, and are unit-tested offscreen without any interactor). Declared at
// global scope (like CameraController.cpp's CvcCameraInteractorStyle) so the VTK
// object macros work. It must remain a STYLE (priority 0.0), never an observer:
// ImGuiOverlay/FpsHud observe the interactor at priority 1.0 above it and abort
// camera input via a flag, and it must NOT set DefaultRenderer (that would pin
// routing to one renderer and defeat per-viewport picking).
class ViewportInputRouterStyle : public vtkInteractorStyle {
public:
  static ViewportInputRouterStyle *New();
  vtkTypeMacro(ViewportInputRouterStyle, vtkInteractorStyle);

  void setManager(cvc::gl::ViewportManager *m) { m_mgr = m; }

  void OnLeftButtonDown() override { button(cvc::gl::ViewportManager::MouseButton::Left, true); }
  void OnLeftButtonUp() override { button(cvc::gl::ViewportManager::MouseButton::Left, false); }
  void OnMiddleButtonDown() override {
    button(cvc::gl::ViewportManager::MouseButton::Middle, true);
  }
  void OnMiddleButtonUp() override { button(cvc::gl::ViewportManager::MouseButton::Middle, false); }
  void OnRightButtonDown() override { button(cvc::gl::ViewportManager::MouseButton::Right, true); }
  void OnRightButtonUp() override { button(cvc::gl::ViewportManager::MouseButton::Right, false); }

  void OnMouseMove() override {
    if (!m_mgr || !this->Interactor)
      return;
    int x, y;
    this->Interactor->GetEventPosition(x, y);
    m_mgr->routeMouseMove(x, y);
  }
  void OnMouseWheelForward() override { wheel(1.0); }
  void OnMouseWheelBackward() override { wheel(-1.0); }

  void OnKeyDown() override {
    if (!m_mgr || !this->Interactor)
      return;
    const char *ks = this->Interactor->GetKeySym();
    m_mgr->routeKey(ks ? ks : "", true);
  }
  void OnKeyUp() override {
    if (!m_mgr || !this->Interactor)
      return;
    const char *ks = this->Interactor->GetKeySym();
    m_mgr->routeKey(ks ? ks : "", false);
  }
  void OnChar() override {} // our navigation owns the keyboard

private:
  void button(cvc::gl::ViewportManager::MouseButton b, bool down) {
    if (!m_mgr || !this->Interactor)
      return;
    int x, y;
    this->Interactor->GetEventPosition(x, y);
    m_mgr->routeMouseButton(b, down, x, y);
  }
  void wheel(double steps) {
    if (!m_mgr || !this->Interactor)
      return;
    int x, y;
    this->Interactor->GetEventPosition(x, y);
    m_mgr->routeMouseWheel(x, y, steps);
  }
  cvc::gl::ViewportManager *m_mgr = nullptr;
};
vtkStandardNewMacro(ViewportInputRouterStyle);

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
  // 3-D props only: skip vtkActor2D (FpsHud, ScreenTextHud, scalar bars). A 2-D
  // overlay is positioned in the WINDOW, not the scene, so copying it would draw
  // the source's HUD a second time on top of the inset.
  dst->RemoveAllViewProps();
  vtkPropCollection *props = src->GetViewProps();
  vtkCollectionSimpleIterator it;
  props->InitTraversal(it);
  while (vtkProp *p = props->GetNextProp(it)) {
    if (vtkActor2D::SafeDownCast(p))
      continue;
    dst->AddViewProp(p);
  }
  // Copy the source's lights so a mirror of a StageLighting-rigged scene is lit
  // the same instead of rendering flat. World-space lights are correct from the
  // mirror's own camera; with none, the mirror falls back to VTK's automatic
  // headlight following its own camera.
  dst->RemoveAllLights();
  vtkLightCollection *lights = src->GetLights();
  vtkCollectionSimpleIterator lit;
  lights->InitTraversal(lit);
  while (vtkLight *l = lights->GetNextLight(lit))
    dst->AddLight(l);
}
} // namespace

struct ViewportManager::Impl {
  cvc::app *app = nullptr;
  std::string name = "main";
  bool offscreen = true;
  bool closed = false;
  vtkSmartPointer<vtkRenderWindow> window;
  vtkSmartPointer<vtkRenderWindowInteractor> interactor;   // onscreen only
  vtkSmartPointer<::ViewportInputRouterStyle> routerStyle; // onscreen only
  std::string activeStatePath;                             // "<main scene prefix>.active_viewport"

  // Input-router state (see routeMouse*/routeKey). dragTarget is latched on
  // button-down and owns the whole drag; buttonsDown is the held-button mask;
  // lastX/lastY + haveLast give mouseLook its per-move delta; hoverTarget lets a
  // non-drag (Fly free-look) move reset its delta baseline when the pointer
  // crosses into a different viewport.
  Viewport *dragTarget = nullptr;
  Viewport *hoverTarget = nullptr;
  int buttonsDown = 0;
  int lastX = 0, lastY = 0;
  bool haveLast = false;
  // CameraController collapses orbit-drag and pan onto ONE internal flag
  // (beginPan sets it too; endDrag and endPan both clear it), so the router owns
  // the single gesture the latched viewport is in and re-derives it from the
  // held-button mask on every transition — otherwise releasing one button of an
  // overlapping Left+Middle chord clears the flag and freezes the still-held one.
  enum class Gesture { None, Drag, Pan };
  Gesture gesture = Gesture::None;

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
                                 const std::string &name, InputMode inputMode)
    : m_impl(new Impl) {
  const bool managed = (inputMode == InputMode::Router);
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
  // Keyboard focus lives at the main scene's prefix (viewport names are unique
  // across the manager). Matches the existing ".viewers.<name>" scheme.
  m_impl->activeStatePath = mainScene.getStatePrefix() + ".active_viewport";
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
    // The multi-viewport input router: one style on the one interactor, feeding
    // per-viewport cameras via route*(). Never SetDefaultRenderer on it (that
    // would pin routing to one renderer) and never attach() the per-viewport
    // controllers (they stay detached; this is their only input path).
    //
    // In HostStyle we leave the style slot FREE instead, so a consumer's
    // CameraController(view).attach() installs its classic single-view style
    // there (the interactor itself still exists for processUIEvents / window
    // close / HUD-overlay observers) — the SceneRenderer facade path.
    if (managed) {
      m_impl->routerStyle = vtkSmartPointer<::ViewportInputRouterStyle>::New();
      m_impl->routerStyle->setManager(this);
      m_impl->interactor->SetInteractorStyle(m_impl->routerStyle);
    }
  }

  // Auto-create the full-screen PRIMARY viewport over the main scene at layer 0,
  // so the common single-view case behaves exactly like a SceneRenderer.
  const std::string camPath = CameraController::viewerStatePath(mainScene.getStatePrefix(), name);
  std::unique_ptr<Viewport> primary(
      new Viewport(*m_impl->app, mainScene, camPath, name, /*mirror=*/false, managed));
  primary->setRegion(0.0, 0.0, 1.0, 1.0);
  primary->setLayer(0);
  m_impl->window->AddRenderer(primary->renderer());
  // Give the (detached) controller the window so pan scaling reads a valid size
  // and Fly pointer capture can hide/recenter the cursor onscreen. This is NOT
  // attach() — it installs no interactor style; the router owns all input.
  // Skipped in HostStyle: the primary is bare (no managed controller), so
  // building one here would defeat the empty-state guarantee; the host's own
  // CameraController(view) takes the window when it attaches.
  if (managed)
    primary->camera().setRenderWindow(m_impl->window);

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
    // Cut the router's back-pointer and detach it before the interactor dies, so
    // no late event dereferences a half-destroyed manager.
    if (m_impl->routerStyle)
      m_impl->routerStyle->setManager(nullptr);
    if (m_impl->interactor) {
      m_impl->interactor->SetInteractorStyle(nullptr);
      m_impl->interactor->TerminateApp();
      m_impl->interactor->SetRenderWindow(nullptr);
      m_impl->interactor = nullptr;
    }
    m_impl->routerStyle = nullptr;
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
  vp->camera().setRenderWindow(m_impl->window); // pan scaling + Fly capture (not attach)

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
                                             const double region[4], int layer, bool liveSync) {
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
  vp->setMirrorLive(liveSync); // false => primed once here, never re-synced
  m_impl->window->AddRenderer(vp->renderer());
  vp->camera().setRenderWindow(m_impl->window); // pan scaling + Fly capture (not attach)

  // Prime the mirror with the source's current props so ResetCamera has bounds
  // to frame. It does NOT attach the scene (the source owns that), so it is not
  // tracked in attachedScenes and never detaches the scene in the destructor.
  // A frozen mirror (liveSync == false) keeps exactly this snapshot afterwards.
  syncMirrorProps(*vp);
  vp->renderer()->ResetCamera();

  Viewport &ref = *vp;
  m_impl->byName[name] = vp.get();
  m_impl->viewports.push_back(std::move(vp));
  m_impl->syncLayerCount();
  return ref;
}

void ViewportManager::removeViewport(const std::string &name) {
  m_impl->requireOpen();
  if (name == m_impl->name)
    throw std::invalid_argument("ViewportManager: the primary viewport cannot be removed");
  auto it = m_impl->byName.find(name);
  if (it == m_impl->byName.end())
    throw std::out_of_range("ViewportManager: no viewport named '" + name + "'");
  Viewport *vp = it->second;

  // A mirror that still echoes this viewport's renderer would dangle; make the
  // caller remove the mirror first.
  for (const auto &other : m_impl->viewports)
    if (other->isMirror() && other->mirrorSource() == vp->renderer())
      throw std::invalid_argument("ViewportManager: viewport '" + name + "' is mirrored by '" +
                                  other->name() + "'; remove the mirror first");

  // A scene viewport detaches its scene so it can be drawn again; a mirror never
  // attached one.
  if (!vp->isMirror()) {
    vp->scene().setRenderer(nullptr);
    m_impl->attachedScenes.erase(&vp->scene());
  }
  m_impl->window->RemoveRenderer(vp->renderer());

  // Drop any router state that points at it, so a live drag / hover does not
  // dereference the removed viewport.
  if (m_impl->dragTarget == vp) {
    m_impl->dragTarget = nullptr;
    m_impl->buttonsDown = 0;
    m_impl->gesture = Impl::Gesture::None;
    m_impl->haveLast = false;
  }
  if (m_impl->hoverTarget == vp)
    m_impl->hoverTarget = nullptr;

  // If it held keyboard focus, hand it back to the primary (before it leaves
  // byName, so the handoff can release its held keys).
  if (cvc::state::instance(*m_impl->app)(m_impl->activeStatePath).value() == name)
    setActiveViewport(m_impl->name);

  m_impl->byName.erase(it);
  m_impl->viewports.erase(
      std::remove_if(m_impl->viewports.begin(), m_impl->viewports.end(),
                     [vp](const std::unique_ptr<Viewport> &u) { return u.get() == vp; }),
      m_impl->viewports.end());
  m_impl->syncLayerCount();
}

void ViewportManager::render() {
  m_impl->requireOpen();
  // A viewport's layer can change out from under us (a state write / restored
  // layout drives Viewport::setLayer via its ViewportLayout), so re-sync the
  // window's layer count before compositing or VTK drops the new top layer.
  m_impl->syncLayerCount();
  // Drain each viewport's scene events (a re-meshed node appears without
  // re-attaching), then composite every layer in one pass. A scene shared by a
  // mirror viewport is drained once via its owning scene viewport; mirrors carry
  // the same SceneGraph* so processEvents() is idempotent-cheap when repeated.
  for (auto &v : m_impl->viewports)
    v->scene().processEvents();
  // Mirrors have no scene attachment of their own: refresh their props from the
  // source renderer so a node added / re-meshed this frame shows up in the
  // mirror too, then draw it with the mirror's own camera. A FROZEN mirror
  // (mirrorLive() == false) keeps its add-time snapshot and is skipped here.
  for (auto &v : m_impl->viewports)
    if (v->isMirror() && v->mirrorLive())
      syncMirrorProps(*v);
  m_impl->window->Render();
}

void ViewportManager::updateCameras(double dtSeconds) {
  m_impl->requireOpen();
  for (auto &v : m_impl->viewports)
    v->camera().update(dtSeconds);
}

// ---- input routing ---------------------------------------------------------

Viewport *ViewportManager::viewportAt(int x, int y) const {
  m_impl->requireOpen();
  Viewport *best = nullptr;
  int bestLayer = 0;
  for (const auto &v : m_impl->viewports) {
    if (!v->visible() || !v->inputEnabled())
      continue;
    if (!v->renderer()->IsInViewport(x, y))
      continue;
    // Descending layer, later-added wins on a tie: iterate in add order and take
    // any hit whose layer >= the best so far (so the last max-layer hit wins).
    if (!best || v->layer() >= bestLayer) {
      best = v.get();
      bestLayer = v->layer();
    }
  }
  return best;
}

void ViewportManager::routeMouseButton(MouseButton button, bool down, int x, int y) {
  m_impl->requireOpen();
  // Right is reserved: a true no-op. Handling it (even just to latch/focus) would
  // let a right-click silently steal keyboard focus and release the previous
  // viewport's held keys — no CameraController gesture maps to it anyway.
  if (button == MouseButton::Right)
    return;
  const int bit = (button == MouseButton::Left) ? 1 : 2;
  const bool wasIdle = (m_impl->buttonsDown == 0);
  if (down)
    m_impl->buttonsDown |= bit;
  else
    m_impl->buttonsDown &= ~bit;

  if (down && wasIdle) {
    // First button of a gesture: hit-test, latch it for the whole gesture, and
    // make it active (focus-follows-click). A gutter miss latches nothing (do
    // NOT fall back to primary the way FindPokedRenderer would).
    Viewport *vp = viewportAt(x, y);
    if (!vp) {
      m_impl->buttonsDown &= ~bit; // undo: no gesture actually started
      return;
    }
    m_impl->dragTarget = vp;
    m_impl->lastX = x;
    m_impl->lastY = y;
    m_impl->haveLast = true;
    setActiveViewport(vp->name());
  }
  if (!m_impl->dragTarget)
    return; // a release with no latch (e.g. after a gutter press)

  // Re-derive the ONE gesture the latched controller should be in from the held
  // buttons (Middle=pan wins over Left=orbit), and transition to it: end the old
  // gesture, begin the new. This is what keeps an overlapping chord consistent —
  // releasing Left while Middle is still held re-issues nothing (still Pan), and
  // releasing Middle while Left is held ends the pan and re-begins the orbit,
  // rather than a stray endDrag/endPan freezing the survivor.
  const Impl::Gesture desired = (m_impl->buttonsDown & 2)   ? Impl::Gesture::Pan
                                : (m_impl->buttonsDown & 1) ? Impl::Gesture::Drag
                                                            : Impl::Gesture::None;
  if (desired != m_impl->gesture) {
    if (m_impl->gesture == Impl::Gesture::Drag)
      m_impl->dragTarget->camera().endDrag();
    else if (m_impl->gesture == Impl::Gesture::Pan)
      m_impl->dragTarget->camera().endPan();
    if (desired == Impl::Gesture::Drag)
      m_impl->dragTarget->camera().beginDrag();
    else if (desired == Impl::Gesture::Pan)
      m_impl->dragTarget->camera().beginPan();
    m_impl->gesture = desired;
  }

  if (m_impl->buttonsDown == 0) {
    // Whole gesture released: drop the latch and the delta baseline.
    m_impl->dragTarget = nullptr;
    m_impl->haveLast = false;
  }
}

void ViewportManager::routeMouseMove(int x, int y) {
  m_impl->requireOpen();
  if (m_impl->dragTarget) {
    // Latched drag: deltas go to the origin viewport even past its edge.
    const int dx = m_impl->haveLast ? (x - m_impl->lastX) : 0;
    const int dy = m_impl->haveLast ? (y - m_impl->lastY) : 0;
    m_impl->lastX = x;
    m_impl->lastY = y;
    m_impl->haveLast = true;
    m_impl->dragTarget->camera().mouseLook(dx, dy);
    return;
  }
  // Free (undragged) move: Fly free-look on the hovered viewport. When the
  // hovered viewport changes, reset the delta baseline and skip a frame so no
  // giant jump is fed across the boundary.
  Viewport *vp = viewportAt(x, y);
  if (vp != m_impl->hoverTarget) {
    m_impl->hoverTarget = vp;
    m_impl->lastX = x;
    m_impl->lastY = y;
    m_impl->haveLast = (vp != nullptr);
    return;
  }
  if (!vp)
    return;
  const int dx = m_impl->haveLast ? (x - m_impl->lastX) : 0;
  const int dy = m_impl->haveLast ? (y - m_impl->lastY) : 0;
  m_impl->lastX = x;
  m_impl->lastY = y;
  m_impl->haveLast = true;
  vp->camera().mouseLook(dx, dy);
}

void ViewportManager::routeMouseWheel(int x, int y, double steps) {
  m_impl->requireOpen();
  // The wheel goes to the viewport under the cursor, not the latched/active one.
  Viewport *vp = viewportAt(x, y);
  if (!vp)
    return;
  vp->camera().mouseWheel(steps);
}

void ViewportManager::routeKey(const std::string &keySym, bool down) {
  m_impl->requireOpen();
  Viewport *vp = activeViewport();
  if (!vp)
    return;
  if (down) {
    // Escape releases pointer capture (the single-view style intercepts it too),
    // and is NOT forwarded as a held key — a held "Escape" would drift the camera.
    if (keySym == "Escape") {
      vp->camera().setPointerCapture(false);
      return;
    }
    vp->camera().keyDown(keySym);
  } else {
    vp->camera().keyUp(keySym);
  }
}

Viewport *ViewportManager::activeViewport() const {
  m_impl->requireOpen();
  const std::string name = cvc::state::instance(*m_impl->app)(m_impl->activeStatePath).value();
  if (!name.empty()) {
    auto it = m_impl->byName.find(name);
    if (it != m_impl->byName.end())
      return it->second;
  }
  // Unset or names a viewport that no longer exists: fall back to the primary so
  // this is never null after construction.
  return m_impl->viewports.empty() ? nullptr : m_impl->viewports.front().get();
}

void ViewportManager::setActiveViewport(const std::string &name) {
  m_impl->requireOpen();
  auto it = m_impl->byName.find(name);
  if (it == m_impl->byName.end())
    return; // ignore unknown
  Viewport *prev = activeViewport();
  if (prev && prev != it->second)
    prev->camera().releaseHeldKeys(); // no key stuck-held across the handoff
  cvc::state::instance (*m_impl->app)(m_impl->activeStatePath).value(name);
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
