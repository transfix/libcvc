/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// SceneRenderer is now a thin FACADE over ViewportManager: it owns one manager
// configured for a single, full-screen primary viewport in HostStyle input mode
// (no internal input router, and a BARE primary that writes no
// ".viewers.<name>.camera/.layout" state) so a lone SceneRenderer behaves and
// looks — down to its cvc::state footprint — exactly like the classic
// single-view renderer it always was, while sharing ONE rendering code path with
// the multi-viewport / picture-in-picture case. Every call forwards to the
// manager (render/capture/resize/...) or to its primary viewport's renderer
// (the four VTK-direct behaviours: setCamera / setBackground / pickWorld /
// renderer). The manager owns the vtkRenderWindow + interactor and the
// deterministic teardown; close() is just letting the manager go.

#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/Viewport.h>
#include <cvc/gl/ViewportManager.h>
#include <stdexcept>
#include <vtkCamera.h>
#include <vtkCellPicker.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

namespace cvc {
namespace gl {

struct SceneRenderer::impl {
  std::unique_ptr<ViewportManager> vm;
  SceneGraph *scene = nullptr; // cached so scene()/name() are valid after close()
  std::string name = "main";
  bool closed = false;

  void requireOpen() const {
    if (closed || !vm)
      throw std::runtime_error("SceneRenderer: renderer is closed");
  }
  // The single primary viewport's renderer — the target for the VTK-direct
  // behaviours SceneRenderer exposes but ViewportManager does not wrap.
  vtkRenderer *primaryRenderer() const { return vm->primary().renderer(); }
};

SceneRenderer::SceneRenderer(SceneGraph &scene, int width, int height, bool offscreen,
                             const std::string &name)
    : m_impl(new impl) {
  if (width < 1 || height < 1)
    throw std::invalid_argument("SceneRenderer: width and height must be >= 1");
  m_impl->scene = &scene;
  m_impl->name = name;
  // ONE full-screen primary over the scene, HostStyle so the interactor's style
  // slot stays free (a CameraController(*this).attach() installs the classic
  // single-view style there) and no viewer camera/layout state is written. The
  // manager ctor does the VTK-diagnostics routing, window/interactor setup, and
  // the one-time scene attach + ResetCamera that this class used to do inline.
  m_impl->vm.reset(new ViewportManager(scene, width, height, offscreen, name,
                                       ViewportManager::InputMode::HostStyle));
}

SceneRenderer::~SceneRenderer() {
  try {
    close();
  } catch (...) {
    // Throwing out of a destructor during teardown is strictly worse than a
    // leaked GL context.
  }
}

void SceneRenderer::close() {
  if (!m_impl || m_impl->closed)
    return;
  m_impl->closed = true;
  // The ViewportManager destructor performs the same ordered teardown this class
  // used to do by hand: detach the scene, tear down the interactor while its
  // window is live, then Finalize() the window. scene/name stay cached so the
  // accessors remain valid after close.
  m_impl->vm.reset();
}

bool SceneRenderer::isClosed() const { return !m_impl || m_impl->closed; }

void SceneRenderer::render() {
  m_impl->requireOpen();
  m_impl->vm->render();
}

void SceneRenderer::writePNG(const std::string &path) {
  m_impl->requireOpen();
  m_impl->vm->writePNG(path);
}

std::vector<unsigned char> SceneRenderer::frameRGB() {
  m_impl->requireOpen();
  return m_impl->vm->frameRGB();
}

int SceneRenderer::frameWidth() const {
  m_impl->requireOpen();
  return m_impl->vm->frameWidth();
}

int SceneRenderer::frameHeight() const {
  m_impl->requireOpen();
  return m_impl->vm->frameHeight();
}

void SceneRenderer::resetCamera() {
  m_impl->requireOpen();
  m_impl->vm->resetCamera();
}

void SceneRenderer::setCamera(double eyeX, double eyeY, double eyeZ, double focalX, double focalY,
                              double focalZ, double upX, double upY, double upZ, double viewAngle,
                              double clipNear, double clipFar) {
  m_impl->requireOpen();
  vtkCamera *cam = m_impl->primaryRenderer()->GetActiveCamera();
  cam->SetPosition(eyeX, eyeY, eyeZ);
  cam->SetFocalPoint(focalX, focalY, focalZ);
  cam->SetViewUp(upX, upY, upZ);
  cam->SetViewAngle(viewAngle);
  cam->SetClippingRange(clipNear, clipFar);
}

void SceneRenderer::setBackground(double r, double g, double b) {
  m_impl->requireOpen();
  m_impl->primaryRenderer()->SetBackground(r, g, b);
}

void SceneRenderer::resize(int width, int height) {
  m_impl->requireOpen();
  if (width < 1 || height < 1)
    throw std::invalid_argument("SceneRenderer::resize: width and height must be >= 1");
  m_impl->vm->resize(width, height);
}

void SceneRenderer::processUIEvents() {
  m_impl->requireOpen();
  m_impl->vm->processUIEvents();
}

bool SceneRenderer::windowClosed() const {
  if (isClosed())
    return true;
  return m_impl->vm->windowClosed();
}

bool SceneRenderer::pickWorld(double displayX, double displayY, double outWorld[3]) const {
  m_impl->requireOpen();
  vtkRenderer *ren = m_impl->primaryRenderer();
  if (!ren)
    return false;
  // vtkCellPicker does a real geometry hit test (unlike vtkWorldPointPicker,
  // which always returns a focal-plane point), so a miss over empty space is
  // reported as a miss rather than a bogus coordinate.
  auto picker = vtkSmartPointer<vtkCellPicker>::New();
  picker->SetTolerance(0.0005);
  if (!picker->Pick(displayX, displayY, 0.0, ren))
    return false;
  double p[3];
  picker->GetPickPosition(p);
  outWorld[0] = p[0];
  outWorld[1] = p[1];
  outWorld[2] = p[2];
  return true;
}

vtkRenderer *SceneRenderer::renderer() const {
  m_impl->requireOpen();
  return m_impl->primaryRenderer();
}

vtkRenderWindow *SceneRenderer::renderWindow() const {
  m_impl->requireOpen();
  return m_impl->vm->renderWindow();
}

SceneGraph &SceneRenderer::scene() const { return *m_impl->scene; }
const std::string &SceneRenderer::name() const { return m_impl->name; }

ViewportManager &SceneRenderer::viewportManager() const {
  m_impl->requireOpen();
  return *m_impl->vm;
}

} // namespace gl
} // namespace cvc
