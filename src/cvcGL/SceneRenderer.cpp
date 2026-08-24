/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <stdexcept>
#include <vtkCamera.h>
#include <vtkNew.h>
#include <vtkOutputWindow.h> // route VTK's ERR/WARN to stderr, not a Win32 message box
#include <vtkPNGWriter.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkWindowToImageFilter.h>

struct SceneRenderer::impl {
  SceneGraph *scene = nullptr;
  std::string name = "main";
  vtkSmartPointer<vtkRenderer> renderer;
  vtkSmartPointer<vtkRenderWindow> window;
  vtkSmartPointer<vtkRenderWindowInteractor> interactor; // onscreen only
  bool offscreen = true;
  bool closed = false;

  void requireOpen() const {
    if (closed)
      throw std::runtime_error("SceneRenderer: renderer is closed");
  }
};

SceneRenderer::SceneRenderer(SceneGraph &scene, int width, int height, bool offscreen,
                             const std::string &name)
    : m_impl(new impl) {
  if (width < 1 || height < 1)
    throw std::invalid_argument("SceneRenderer: width and height must be >= 1");

  // Route VTK diagnostics through stderr instead of a Win32 message box, and
  // silence WARN-level output. VTK's shadow-map pass logs "Could not create
  // shader object" / "Hardware does not support the number of textures defined"
  // ERR/WARN lines on some driver configs; those are cosmetic (VTK falls back
  // and keeps rendering) and just spam the console. Done once per process.
  static bool s_vtkOutputConfigured = false;
  if (!s_vtkOutputConfigured) {
    if (auto *ow = vtkOutputWindow::GetInstance()) {
      ow->SetDisplayModeToAlwaysStdErr();
      // Keep ERR (people probably want to know) but drop WARN.
      ow->SetPromptUser(0);
    }
    s_vtkOutputConfigured = true;
  }

  m_impl->scene = &scene;
  m_impl->name = name;
  m_impl->offscreen = offscreen;
  m_impl->renderer = vtkSmartPointer<vtkRenderer>::New();
  m_impl->window = vtkSmartPointer<vtkRenderWindow>::New();
  m_impl->window->SetOffScreenRendering(offscreen ? 1 : 0);
  m_impl->window->AddRenderer(m_impl->renderer);
  m_impl->window->SetSize(width, height);

  if (!offscreen) {
    m_impl->window->SetWindowName("cvcGL");
    // Initialize(), never Start(): Start() runs VTK's own event loop and blocks
    // until the window closes, which would take the loop away from the caller.
    // A simulation has to keep the loop to step its own clock, so the
    // interactor exists only to deliver resize/close/mouse events, drained on
    // demand by processUIEvents().
    m_impl->interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    m_impl->interactor->SetRenderWindow(m_impl->window);
    m_impl->interactor->Initialize();
  }

  // Attach ONCE. This is the step the one-shot helpers repeat every frame: it
  // walks the scene and hands every node's actor to the renderer.
  scene.setRenderer(m_impl->renderer);
  scene.processEvents();
  m_impl->renderer->ResetCamera();
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
  // Detach BEFORE the window dies: the scene holds actors that belong to this
  // renderer, and tearing the window down under them is how offscreen backends
  // crash at exit.
  if (m_impl->scene)
    m_impl->scene->setRenderer(nullptr);
  // Tear down the interactor FIRST, while its render window is still live.
  // Order matters on Windows: vtkWin32RenderWindowInteractor's destructor,
  // fired after Finalize(), tries to touch its render window; that surfaces
  // as a blank native window briefly popping up at exit (and, on some driver
  // configs, "Could not create shader object" spam as VTK re-inits GL). Call
  // TerminateApp() so any pending message loop actually quits, then null the
  // interactor before finalizing the window.
  if (m_impl->interactor) {
    m_impl->interactor->TerminateApp();
    m_impl->interactor->SetRenderWindow(nullptr);
    m_impl->interactor = nullptr;
  }
  if (m_impl->window)
    m_impl->window->Finalize();
  m_impl->window = nullptr;
  m_impl->renderer = nullptr;
  m_impl->scene = nullptr;
}

bool SceneRenderer::isClosed() const { return !m_impl || m_impl->closed; }

void SceneRenderer::render() {
  m_impl->requireOpen();
  // Geometry added or replaced since the last frame arrives as queued scene
  // events; draining them here is what makes a re-meshed node appear without
  // re-attaching the whole scene.
  m_impl->scene->processEvents();
  m_impl->window->Render();
}

void SceneRenderer::writePNG(const std::string &path) {
  render();
  vtkNew<vtkWindowToImageFilter> w2i;
  w2i->SetInput(m_impl->window);
  w2i->SetInputBufferTypeToRGB();
  w2i->ReadFrontBufferOff();
  // Without Modified() the filter caches its first execution and every
  // subsequent PNG in a sequence is a copy of frame 0.
  w2i->Modified();
  w2i->Update();
  vtkNew<vtkPNGWriter> writer;
  writer->SetFileName(path.c_str());
  writer->SetInputConnection(w2i->GetOutputPort());
  writer->Write();
}

std::vector<unsigned char> SceneRenderer::frameRGB() {
  render();
  const int w = m_impl->window->GetSize()[0];
  const int h = m_impl->window->GetSize()[1];
  // Straight out of the framebuffer: no PNG encode on this side and no decode
  // on the other, which is the entire point when piping frames to an encoder.
  vtkSmartPointer<vtkUnsignedCharArray> buf = vtkSmartPointer<vtkUnsignedCharArray>::New();
  m_impl->window->GetPixelData(0, 0, w - 1, h - 1, /*front=*/0, buf);
  const unsigned char *p = buf->GetPointer(0);
  const size_t n = static_cast<size_t>(buf->GetNumberOfTuples()) * buf->GetNumberOfComponents();
  return std::vector<unsigned char>(p, p + n);
}

int SceneRenderer::frameWidth() const {
  m_impl->requireOpen();
  return m_impl->window->GetSize()[0];
}

int SceneRenderer::frameHeight() const {
  m_impl->requireOpen();
  return m_impl->window->GetSize()[1];
}

void SceneRenderer::resetCamera() {
  m_impl->requireOpen();
  m_impl->scene->processEvents(); // frame what is in the scene NOW
  m_impl->renderer->ResetCamera();
}

void SceneRenderer::setCamera(double eyeX, double eyeY, double eyeZ, double focalX, double focalY,
                              double focalZ, double upX, double upY, double upZ, double viewAngle,
                              double clipNear, double clipFar) {
  m_impl->requireOpen();
  vtkCamera *cam = m_impl->renderer->GetActiveCamera();
  cam->SetPosition(eyeX, eyeY, eyeZ);
  cam->SetFocalPoint(focalX, focalY, focalZ);
  cam->SetViewUp(upX, upY, upZ);
  cam->SetViewAngle(viewAngle);
  cam->SetClippingRange(clipNear, clipFar);
}

void SceneRenderer::setBackground(double r, double g, double b) {
  m_impl->requireOpen();
  m_impl->renderer->SetBackground(r, g, b);
}

void SceneRenderer::resize(int width, int height) {
  m_impl->requireOpen();
  if (width < 1 || height < 1)
    throw std::invalid_argument("SceneRenderer::resize: width and height must be >= 1");
  m_impl->window->SetSize(width, height);
}

void SceneRenderer::processUIEvents() {
  m_impl->requireOpen();
  if (m_impl->interactor)
    m_impl->interactor->ProcessEvents();
}

bool SceneRenderer::windowClosed() const {
  if (isClosed())
    return true;
  if (!m_impl->interactor)
    return false; // offscreen has no window to close
  return m_impl->interactor->GetDone() != 0;
}

vtkRenderer *SceneRenderer::renderer() const {
  m_impl->requireOpen();
  return m_impl->renderer;
}

vtkRenderWindow *SceneRenderer::renderWindow() const {
  m_impl->requireOpen();
  return m_impl->window;
}

SceneGraph &SceneRenderer::scene() const { return *m_impl->scene; }
const std::string &SceneRenderer::name() const { return m_impl->name; }
