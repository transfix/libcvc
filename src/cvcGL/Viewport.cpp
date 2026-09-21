/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/Settings.h>
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
  bool inputEnabled = true;            // routed input reaches this viewport
  vtkRenderer *mirrorSource = nullptr; // set only for a mirror viewport
  // region/layer/visible mirrored to cvc::state so a PiP arrangement round-trips.
  std::unique_ptr<ViewportLayout> layout;

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
    impl->visible = v.visible;
    impl->renderer->SetViewport(v.region[0], v.region[1], v.region[2], v.region[3]);
    impl->renderer->SetLayer(v.layer);
    impl->renderer->SetDraw(v.visible ? 1 : 0);
  }));
}

Viewport::~Viewport() = default;

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
  m_impl->visible = on;
  m_impl->renderer->SetDraw(on ? 1 : 0); // skipped in the composite when off
  m_impl->writeLayout();
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
void Viewport::setInputEnabled(bool on) { m_impl->inputEnabled = on; }
bool Viewport::inputEnabled() const { return m_impl->inputEnabled; }

} // namespace gl
} // namespace cvc
