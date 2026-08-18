/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/gl/CameraController.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <string>

#include <vtkCamera.h>
#include <vtkInteractorStyle.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkSmartPointer.h>

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

struct Vec3 {
  double x, y, z;
};
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double length(Vec3 a) { return std::sqrt(dot(a, a)); }

// FLY basis, Z-UP. yaw about +Z (0 looks toward +X), pitch tilts toward +Z.
inline Vec3 flyForward(double yawDeg, double pitchDeg) {
  const double y = yawDeg * kDeg2Rad, p = pitchDeg * kDeg2Rad;
  return {std::cos(p) * std::cos(y), std::cos(p) * std::sin(y), std::sin(p)};
}
// Horizontal strafe-right = forwardHoriz x worldUp.
inline Vec3 flyRight(double yawDeg) {
  const double y = yawDeg * kDeg2Rad;
  return {std::sin(y), -std::cos(y), 0.0};
}
// ORBIT eye offset from center, Z-UP.
inline Vec3 orbitOffset(double azDeg, double elDeg, double dist) {
  const double a = azDeg * kDeg2Rad, e = elDeg * kDeg2Rad;
  return {dist * std::cos(e) * std::cos(a), dist * std::cos(e) * std::sin(a), dist * std::sin(e)};
}

} // namespace

// A file-local interactor style that forwards VTK window events to a
// CameraController. It swallows VTK's default single-key bindings (w=wireframe,
// e/q=quit, r=reset, s=surface, ...) so WASD means movement.
class CvcCameraInteractorStyle : public vtkInteractorStyle {
public:
  static CvcCameraInteractorStyle *New();
  vtkTypeMacro(CvcCameraInteractorStyle, vtkInteractorStyle);

  void setController(cvc::gl::CameraController *c) { m_controller = c; }

  void OnKeyDown() override {
    if (!m_controller || !this->Interactor)
      return;
    const std::string sym = this->Interactor->GetKeySym() ? this->Interactor->GetKeySym() : "";
    if (sym == "Tab") {
      m_controller->toggleMode();
      return;
    }
    m_controller->keyDown(sym);
  }
  void OnKeyUp() override {
    if (!m_controller || !this->Interactor)
      return;
    const std::string sym = this->Interactor->GetKeySym() ? this->Interactor->GetKeySym() : "";
    m_controller->keyUp(sym);
  }
  // Swallow the base char bindings entirely; our navigation owns the keyboard.
  void OnChar() override {}

  void OnMouseMove() override {
    if (!m_controller || !this->Interactor)
      return;
    int x, y, lx, ly;
    this->Interactor->GetEventPosition(x, y);
    this->Interactor->GetLastEventPosition(lx, ly);
    m_controller->mouseLook(x - lx, y - ly);
  }
  void OnLeftButtonDown() override {
    if (m_controller)
      m_controller->beginDrag();
  }
  void OnLeftButtonUp() override {
    if (m_controller)
      m_controller->endDrag();
  }
  void OnMouseWheelForward() override {
    if (m_controller)
      m_controller->mouseWheel(1.0);
  }
  void OnMouseWheelBackward() override {
    if (m_controller)
      m_controller->mouseWheel(-1.0);
  }

private:
  cvc::gl::CameraController *m_controller = nullptr;
};
vtkStandardNewMacro(CvcCameraInteractorStyle);

namespace cvc {
namespace gl {

struct CameraController::Impl {
  Mode mode = Mode::Orbit;

  // Orbit state
  Vec3 orbitCenter{0, 0, 0};
  double orbitDistance = 10.0;
  double orbitAzimuth = -60.0;
  double orbitElevation = 30.0;

  // Fly state
  Vec3 flyPos{0, 0, 0};
  double flyYaw = 0.0;
  double flyPitch = 0.0;

  // Held keys (X keysyms, lowercased ASCII where applicable)
  std::set<std::string> held;
  bool dragging = false;

  // Tunables
  double moveSpeed = 5.0;
  double sprintMultiplier = 4.0;
  double sensitivity = 0.25; // degrees per pixel
  bool invertPitch = false;

  vtkCamera *camera = nullptr;
  vtkRenderer *renderer = nullptr;
  vtkRenderWindowInteractor *interactor = nullptr;
  vtkSmartPointer<CvcCameraInteractorStyle> style;

  bool held_has(const std::string &k) const { return held.find(k) != held.end(); }
};

CameraController::CameraController() : m_impl(std::make_unique<Impl>()) {}
CameraController::~CameraController() { detach(); }

void CameraController::setCamera(vtkCamera *camera) { m_impl->camera = camera; }
void CameraController::setRenderer(vtkRenderer *renderer) { m_impl->renderer = renderer; }

void CameraController::attach(vtkRenderWindowInteractor *interactor) {
  m_impl->interactor = interactor;
  if (!interactor)
    return;
  if (!m_impl->renderer && interactor->GetRenderWindow()) {
    if (auto *rens = interactor->GetRenderWindow()->GetRenderers())
      m_impl->renderer = rens->GetFirstRenderer();
  }
  if (!m_impl->camera && m_impl->renderer)
    m_impl->camera = m_impl->renderer->GetActiveCamera();
  m_impl->style = vtkSmartPointer<CvcCameraInteractorStyle>::New();
  m_impl->style->setController(this);
  if (m_impl->renderer)
    m_impl->style->SetDefaultRenderer(m_impl->renderer);
  interactor->SetInteractorStyle(m_impl->style);
}

void CameraController::detach() {
  if (m_impl->interactor && m_impl->style &&
      m_impl->interactor->GetInteractorStyle() == m_impl->style.Get())
    m_impl->interactor->SetInteractorStyle(nullptr);
  m_impl->style = nullptr;
  m_impl->interactor = nullptr;
}

CameraController::Mode CameraController::mode() const { return m_impl->mode; }

void CameraController::setMode(Mode m) {
  if (m == m_impl->mode)
    return;
  toggleMode();
}

void CameraController::toggleMode() {
  // Seed the target mode from the LIVE camera pose so the switch is seamless.
  Impl &s = *m_impl;
  if (s.camera) {
    double e[3], f[3];
    s.camera->GetPosition(e);
    s.camera->GetFocalPoint(f);
    Vec3 eye{e[0], e[1], e[2]}, focal{f[0], f[1], f[2]};
    if (s.mode == Mode::Orbit) {
      // -> Fly: stand at the eye, look toward the focal point.
      s.flyPos = eye;
      Vec3 d = focal - eye;
      double len = length(d);
      if (len > 1e-9) {
        d = d * (1.0 / len);
        s.flyYaw = std::atan2(d.y, d.x) / kDeg2Rad;
        s.flyPitch = std::asin(std::max(-1.0, std::min(1.0, d.z))) / kDeg2Rad;
      }
    } else {
      // -> Orbit: orbit the point we were looking at.
      s.orbitCenter = focal;
      Vec3 off = eye - focal;
      s.orbitDistance = std::max(1e-6, length(off));
      s.orbitAzimuth = std::atan2(off.y, off.x) / kDeg2Rad;
      s.orbitElevation = std::asin(std::max(-1.0, std::min(1.0, off.z / s.orbitDistance))) / kDeg2Rad;
    }
  }
  s.mode = (s.mode == Mode::Orbit) ? Mode::Fly : Mode::Orbit;
  s.held.clear();
  applyToCamera();
}

void CameraController::frameBounds(double minX, double minY, double minZ, double maxX, double maxY,
                                   double maxZ) {
  Impl &s = *m_impl;
  Vec3 lo{minX, minY, minZ}, hi{maxX, maxY, maxZ};
  s.orbitCenter = (lo + hi) * 0.5;
  double diag = length(hi - lo);
  if (diag < 1e-9)
    diag = 1.0;
  s.orbitDistance = diag * 1.1;
  s.orbitAzimuth = -60.0;
  s.orbitElevation = 30.0;
  s.moveSpeed = diag / 8.0; // cross the scene in ~8s; sprint (x4) in ~2s
  // Seed fly to the same vantage looking at the center.
  Vec3 off = orbitOffset(s.orbitAzimuth, s.orbitElevation, s.orbitDistance);
  s.flyPos = s.orbitCenter + off;
  Vec3 d = s.orbitCenter - s.flyPos;
  double len = length(d);
  if (len > 1e-9) {
    d = d * (1.0 / len);
    s.flyYaw = std::atan2(d.y, d.x) / kDeg2Rad;
    s.flyPitch = std::asin(std::max(-1.0, std::min(1.0, d.z))) / kDeg2Rad;
  }
  applyToCamera();
}

void CameraController::update(double dtSeconds) {
  Impl &s = *m_impl;
  if (s.mode == Mode::Fly) {
    double fwd = (s.held_has("w") ? 1.0 : 0.0) - (s.held_has("s") ? 1.0 : 0.0);
    double strafe = (s.held_has("d") ? 1.0 : 0.0) - (s.held_has("a") ? 1.0 : 0.0);
    double rise = (s.held_has("space") ? 1.0 : 0.0) -
                  (s.held_has("Control_L") || s.held_has("Control_R") ? 1.0 : 0.0);
    if (fwd != 0.0 || strafe != 0.0 || rise != 0.0) {
      double speed = s.moveSpeed * dtSeconds;
      if (s.held_has("Shift_L") || s.held_has("Shift_R"))
        speed *= s.sprintMultiplier;
      Vec3 f = flyForward(s.flyYaw, s.flyPitch);
      Vec3 r = flyRight(s.flyYaw);
      s.flyPos = s.flyPos + f * (fwd * speed) + r * (strafe * speed) + Vec3{0, 0, 1} * (rise * speed);
    }
  }
  applyToCamera();
}

void CameraController::keyDown(const std::string &keySym) {
  // Normalize single letters to lowercase so Shift+W still means "w".
  std::string k = keySym;
  if (k.size() == 1 && k[0] >= 'A' && k[0] <= 'Z')
    k[0] = static_cast<char>(k[0] - 'A' + 'a');
  m_impl->held.insert(k);
}
void CameraController::keyUp(const std::string &keySym) {
  std::string k = keySym;
  if (k.size() == 1 && k[0] >= 'A' && k[0] <= 'Z')
    k[0] = static_cast<char>(k[0] - 'A' + 'a');
  m_impl->held.erase(k);
}

void CameraController::mouseLook(int dxPixels, int dyPixels) {
  Impl &s = *m_impl;
  const double dpitch = (s.invertPitch ? -1.0 : 1.0) * dyPixels * s.sensitivity;
  if (s.mode == Mode::Fly) {
    s.flyYaw -= dxPixels * s.sensitivity;
    s.flyPitch = std::max(-89.0, std::min(89.0, s.flyPitch + dpitch));
    applyToCamera();
  } else if (s.dragging) {
    s.orbitAzimuth -= dxPixels * s.sensitivity;
    s.orbitElevation = std::max(-89.0, std::min(89.0, s.orbitElevation + dpitch));
    applyToCamera();
  }
}

void CameraController::mouseWheel(double steps) {
  Impl &s = *m_impl;
  if (s.mode == Mode::Fly) {
    s.moveSpeed = std::max(1e-4, s.moveSpeed * std::pow(1.25, steps));
  } else {
    s.orbitDistance = std::max(1e-4, s.orbitDistance * std::pow(0.9, steps));
    applyToCamera();
  }
}

void CameraController::beginDrag() { m_impl->dragging = true; }
void CameraController::endDrag() { m_impl->dragging = false; }

void CameraController::setMoveSpeed(double u) { m_impl->moveSpeed = u; }
void CameraController::setSprintMultiplier(double f) { m_impl->sprintMultiplier = f; }
void CameraController::setMouseSensitivity(double d) { m_impl->sensitivity = d; }
void CameraController::setInvertPitch(bool invert) { m_impl->invertPitch = invert; }

void CameraController::applyToCamera() {
  Impl &s = *m_impl;
  if (!s.camera)
    return;
  Vec3 eye, focal;
  const Vec3 up{0, 0, 1};
  if (s.mode == Mode::Fly) {
    eye = s.flyPos;
    focal = s.flyPos + flyForward(s.flyYaw, s.flyPitch);
  } else {
    eye = s.orbitCenter + orbitOffset(s.orbitAzimuth, s.orbitElevation, s.orbitDistance);
    focal = s.orbitCenter;
  }
  s.camera->SetPosition(eye.x, eye.y, eye.z);
  s.camera->SetFocalPoint(focal.x, focal.y, focal.z);
  s.camera->SetViewUp(up.x, up.y, up.z);
  if (s.renderer)
    s.renderer->ResetCameraClippingRange();
}

void CameraController::getPose(double eye[3], double focal[3], double up[3]) const {
  Impl &s = *m_impl;
  Vec3 e, f;
  if (s.mode == Mode::Fly) {
    e = s.flyPos;
    f = s.flyPos + flyForward(s.flyYaw, s.flyPitch);
  } else {
    e = s.orbitCenter + orbitOffset(s.orbitAzimuth, s.orbitElevation, s.orbitDistance);
    f = s.orbitCenter;
  }
  eye[0] = e.x; eye[1] = e.y; eye[2] = e.z;
  focal[0] = f.x; focal[1] = f.y; focal[2] = f.z;
  up[0] = 0; up[1] = 0; up[2] = 1;
}

} // namespace gl
} // namespace cvc
