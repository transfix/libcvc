/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/gl/FpsHud.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>

#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkCoordinate.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>

#include <atomic>
#include <chrono>
#include <cstdio>

namespace cvc {
namespace gl {

namespace {
// Same normalization CameraController applies to interactor key syms.
std::string normKey(std::string k) {
  if (k.size() == 1 && k[0] >= 'A' && k[0] <= 'Z')
    k[0] = static_cast<char>(k[0] - 'A' + 'a');
  return k;
}
} // namespace

struct FpsHud::Impl {
  // config (mirrored to state)
  bool enabled = true;
  std::string toggleKey = "f";
  double updateHz = 2.0;
  double posX = 0.02, posY = 0.95; // normalized viewport
  int fontSize = 18;

  // runtime (not state)
  double fpsSmoothed = 0.0;
  bool haveLast = false;
  std::chrono::steady_clock::time_point lastFrame;
  double sinceRefresh = 0.0;
  std::atomic<bool> selfWrite{false};

  vtkRenderer *renderer = nullptr;
  vtkRenderWindow *window = nullptr;
  vtkRenderWindowInteractor *interactor = nullptr;
  vtkSmartPointer<vtkTextActor> actor;
  vtkSmartPointer<vtkCallbackCommand> endCb, keyCb;
  unsigned long endTag = 0, keyTag = 0;
};

std::string FpsHud::viewerStatePath(const std::string &scenePrefix, const std::string &viewerName) {
  return scenePrefix + ".viewers." + viewerName + ".hud";
}

FpsHud::FpsHud(cvc::app &ctx, const std::string &statePath)
    : cvc::state_object<FpsHud>(ctx, statePath), m_impl(std::make_unique<Impl>()) {
  // Synchronous reactions on the calling thread, like CameraController: the
  // overlay is driven from the render thread anyway.
  this->setInstanceThreading(false);
  seedState();
}

FpsHud::FpsHud(SceneRenderer &viewer)
    : FpsHud(viewer.scene().appContext(),
             viewerStatePath(viewer.scene().getStatePrefix(), viewer.name())) {
  setRenderer(viewer.renderer());
  attach(viewer.renderWindow(),
         viewer.renderWindow() ? viewer.renderWindow()->GetInteractor() : nullptr);
}

FpsHud::~FpsHud() { detach(); }

// ---- wiring ----
void FpsHud::setRenderer(vtkRenderer *renderer) { m_impl->renderer = renderer; }

void FpsHud::attach(vtkRenderWindow *window, vtkRenderWindowInteractor *interactor) {
  Impl &s = *m_impl;
  detach();
  s.window = window;
  s.interactor = interactor;

  if (s.renderer && !s.actor) {
    s.actor = vtkSmartPointer<vtkTextActor>::New();
    s.actor->SetInput("-- fps");
    s.actor->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
    s.actor->GetPositionCoordinate()->SetValue(s.posX, s.posY);
    vtkTextProperty *tp = s.actor->GetTextProperty();
    tp->SetFontSize(s.fontSize);
    tp->SetColor(1.0, 1.0, 1.0);
    tp->ShadowOn(); // readable over any scene
    s.renderer->AddActor2D(s.actor);
  }

  if (s.window) {
    s.endCb = vtkSmartPointer<vtkCallbackCommand>::New();
    s.endCb->SetClientData(this);
    s.endCb->SetCallback(&FpsHud::onRenderEnd);
    // Every vtkRenderWindow::Render() fires EndEvent — one mark per frame.
    s.endTag = s.window->AddObserver(vtkCommand::EndEvent, s.endCb);
  }

  if (s.interactor) {
    s.keyCb = vtkSmartPointer<vtkCallbackCommand>::New();
    s.keyCb->SetClientData(this);
    s.keyCb->SetCallback(&FpsHud::onKeyPress);
    // Priority above the camera's vtkInteractorStyle (0.0) so the HUD sees the
    // key first and coexists with the style regardless of construction order.
    // No abort: 'f' falls through to the camera's held-set where it is inert.
    s.keyTag = s.interactor->AddObserver(vtkCommand::KeyPressEvent, s.keyCb, 1.0f);
  }

  apply();
}

void FpsHud::detach() {
  Impl &s = *m_impl;
  if (s.window && s.endTag) {
    s.window->RemoveObserver(s.endTag);
    s.endTag = 0;
  }
  if (s.interactor && s.keyTag) {
    s.interactor->RemoveObserver(s.keyTag);
    s.keyTag = 0;
  }
  if (s.renderer && s.actor)
    s.renderer->RemoveActor2D(s.actor);
  s.actor = nullptr;
  s.endCb = nullptr;
  s.keyCb = nullptr;
  s.window = nullptr;
  s.interactor = nullptr;
}

// ---- config ----
bool FpsHud::enabled() const { return m_impl->enabled; }

void FpsHud::setEnabled(bool on) {
  m_impl->enabled = on;
  apply();
  syncConfigToState();
}

std::string FpsHud::toggleKey() const { return m_impl->toggleKey; }

void FpsHud::setToggleKey(const std::string &keySym) {
  m_impl->toggleKey = normKey(keySym);
  syncConfigToState();
}

void FpsHud::setUpdateHz(double hz) {
  m_impl->updateHz = hz > 0.0 ? hz : 2.0;
  syncConfigToState();
}

double FpsHud::fps() const { return m_impl->fpsSmoothed; }

// ---- state seeding / sync ----
void FpsHud::seedState() {
  cvc::state_init_scope<FpsHud> guard(*this); // suppress change signals
  Impl &s = *m_impl;
  getState("enabled").value(s.enabled ? 1 : 0);
  getState("keys.toggle").value(s.toggleKey);
  getState("update_hz").value(s.updateHz);
  getState("position.x").value(s.posX);
  getState("position.y").value(s.posY);
  getState("font_size").value(s.fontSize);
  getState("fps").value(0.0);
}

void FpsHud::readAllFromState() {
  Impl &s = *m_impl;
  auto d = [&](const char *k, double def) {
    try {
      return getState(k).value<double>();
    } catch (...) {
      return def;
    }
  };
  auto i = [&](const char *k, int def) {
    try {
      return getState(k).value<int>();
    } catch (...) {
      return def;
    }
  };
  auto str = [&](const char *k, const std::string &def) {
    std::string v = getState(k).value();
    return v.empty() ? def : v;
  };
  s.enabled = i("enabled", s.enabled ? 1 : 0) != 0;
  s.toggleKey = normKey(str("keys.toggle", s.toggleKey));
  double hz = d("update_hz", s.updateHz);
  s.updateHz = hz > 0.0 ? hz : s.updateHz;
  s.posX = d("position.x", s.posX);
  s.posY = d("position.y", s.posY);
  s.fontSize = i("font_size", s.fontSize);
}

void FpsHud::syncConfigToState() {
  Impl &s = *m_impl;
  s.selfWrite = true;
  getState("enabled").value(s.enabled ? 1 : 0);
  getState("keys.toggle").value(s.toggleKey);
  getState("update_hz").value(s.updateHz);
  getState("position.x").value(s.posX);
  getState("position.y").value(s.posY);
  getState("font_size").value(s.fontSize);
  s.selfWrite = false;
}

void FpsHud::handleStateChanged(const std::string &childState) {
  (void)childState;
  if (m_impl->selfWrite.load())
    return; // our own write echoing back
  readAllFromState();
  apply();
}

void FpsHud::apply() {
  Impl &s = *m_impl;
  if (!s.actor)
    return;
  s.actor->SetVisibility(s.enabled ? 1 : 0);
  s.actor->GetPositionCoordinate()->SetValue(s.posX, s.posY);
  s.actor->GetTextProperty()->SetFontSize(s.fontSize);
}

// ---- measurement ----
void FpsHud::frameRendered() {
  Impl &s = *m_impl;
  const auto now = std::chrono::steady_clock::now();
  if (!s.haveLast) {
    s.haveLast = true;
    s.lastFrame = now;
    return;
  }
  const double dt = std::chrono::duration<double>(now - s.lastFrame).count();
  s.lastFrame = now;
  if (dt <= 0.0)
    return;
  const double inst = 1.0 / dt;
  // EMA over ~1 s of frames so the readout is steady but tracks real changes.
  const double alpha = dt / (dt + 1.0);
  s.fpsSmoothed = s.fpsSmoothed <= 0.0 ? inst : s.fpsSmoothed + alpha * (inst - s.fpsSmoothed);

  // Refresh the text (FreeType re-rasterization) and mirror to state only at
  // update_hz — per-frame state writes are a known perf sink.
  s.sinceRefresh += dt;
  if (s.sinceRefresh < 1.0 / s.updateHz)
    return;
  s.sinceRefresh = 0.0;
  if (s.actor && s.enabled) {
    char text[64];
    std::snprintf(text, sizeof text, "%.1f fps", s.fpsSmoothed);
    s.actor->SetInput(text); // marks modified; draws next frame
  }
  s.selfWrite = true;
  getState("fps").value(s.fpsSmoothed);
  s.selfWrite = false;
}

// ---- VTK callbacks ----
void FpsHud::onRenderEnd(vtkObject *, unsigned long, void *clientData, void *) {
  static_cast<FpsHud *>(clientData)->frameRendered();
}

void FpsHud::onKeyPress(vtkObject *caller, unsigned long, void *clientData, void *) {
  auto *self = static_cast<FpsHud *>(clientData);
  auto *iren = vtkRenderWindowInteractor::SafeDownCast(caller);
  if (!iren || !iren->GetKeySym())
    return;
  if (normKey(iren->GetKeySym()) == self->m_impl->toggleKey)
    self->setEnabled(!self->enabled());
}

} // namespace gl
} // namespace cvc
