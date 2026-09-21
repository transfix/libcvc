/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/gl/CameraController.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/Viewport.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

namespace cvc {
namespace gl {

struct Viewport::Impl {
  std::string name;
  bool mirror = false;
  SceneGraph *scene = nullptr; // the scene this viewport draws (owned elsewhere)
  vtkSmartPointer<vtkRenderer> renderer;
  std::unique_ptr<CameraController> camera;
  double region[4] = {0.0, 0.0, 1.0, 1.0}; // normalized, VTK y-up
  int layer = 0;
  bool visible = true;
  vtkRenderer *mirrorSource = nullptr; // set only for a mirror viewport
};

Viewport::Viewport(cvc::app &app, SceneGraph &scene, const std::string &cameraStatePath,
                   const std::string &name, bool mirror)
    : m_impl(new Impl) {
  m_impl->name = name;
  m_impl->mirror = mirror;
  m_impl->scene = &scene;
  m_impl->renderer = vtkSmartPointer<vtkRenderer>::New();
  m_impl->renderer->SetViewport(m_impl->region[0], m_impl->region[1], m_impl->region[2],
                                m_impl->region[3]);
  m_impl->renderer->SetLayer(m_impl->layer);

  // This viewport's own camera controller, rooted at its own state path. Built
  // with the low-level (app, path) ctor and wired to THIS renderer/camera/scene —
  // it never attaches to the window interactor (the manager's router owns that).
  m_impl->camera.reset(new CameraController(app, cameraStatePath));
  m_impl->camera->setCamera(m_impl->renderer->GetActiveCamera());
  m_impl->camera->setRenderer(m_impl->renderer);
  m_impl->camera->setScene(&scene);
}

Viewport::~Viewport() = default;

const std::string &Viewport::name() const { return m_impl->name; }

void Viewport::setRegion(double x0, double y0, double x1, double y1) {
  m_impl->region[0] = x0;
  m_impl->region[1] = y0;
  m_impl->region[2] = x1;
  m_impl->region[3] = y1;
  m_impl->renderer->SetViewport(x0, y0, x1, y1);
}

void Viewport::region(double out[4]) const {
  for (int i = 0; i < 4; ++i)
    out[i] = m_impl->region[i];
}

void Viewport::setLayer(int layer) {
  m_impl->layer = layer;
  m_impl->renderer->SetLayer(layer);
}
int Viewport::layer() const { return m_impl->layer; }

void Viewport::setVisible(bool on) {
  m_impl->visible = on;
  m_impl->renderer->SetDraw(on ? 1 : 0); // skipped in the composite when off
}
bool Viewport::visible() const { return m_impl->visible; }

void Viewport::setBackground(double r, double g, double b, bool opaque) {
  m_impl->renderer->SetBackground(r, g, b);
  // Opaque panel clears its own rect (default); a translucent overlay must
  // preserve the colour buffer under it or it wipes the base view.
  m_impl->renderer->SetPreserveColorBuffer(opaque ? 0 : 1);
}

CameraController &Viewport::camera() { return *m_impl->camera; }
SceneGraph &Viewport::scene() const { return *m_impl->scene; }
vtkRenderer *Viewport::renderer() const { return m_impl->renderer; }
bool Viewport::isMirror() const { return m_impl->mirror; }
vtkRenderer *Viewport::mirrorSource() const { return m_impl->mirrorSource; }
void Viewport::setMirrorSource(vtkRenderer *src) { m_impl->mirrorSource = src; }

} // namespace gl
} // namespace cvc
