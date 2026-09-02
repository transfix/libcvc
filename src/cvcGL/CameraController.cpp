/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
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

// X11 pointer warp for continuous (Quake) mouse-look. Guarded so non-X11 builds
// degrade to cursor-delta look. VTK already links X11 on Linux.
#if defined(__linux__)
#define CVCGL_HAVE_X11 1
#include <X11/Xlib.h>
#endif

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

struct Vec3 {
  double x = 0, y = 0, z = 0;
};
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Vec3 normalize(Vec3 a) {
  double l = length(a);
  return l > 1e-12 ? a * (1.0 / l) : Vec3{0, 0, 1};
}
// Frame-rate-independent exponential move toward `nw` with time constant `tau`.
inline Vec3 ema(Vec3 prev, Vec3 nw, double dt, double tau) {
  double a = tau > 0.0 ? (1.0 - std::exp(-dt / tau)) : 1.0;
  return prev + (nw - prev) * a;
}

// The horizontal basis (north N = yaw-0 forward, east E = yaw-90) derived from an
// arbitrary world up. For up=+Z this is N=+X, E=+Y (so the Z-up math is unchanged);
// it generalises predictably to any up (e.g. +Y).
struct Basis {
  Vec3 up, N, E;
};
inline Basis basisFromUp(Vec3 up) {
  Vec3 u = normalize(up);
  // Pick the world axis most perpendicular to u as the forward reference.
  Vec3 cand;
  double ax = std::fabs(u.x), ay = std::fabs(u.y), az = std::fabs(u.z);
  if (ax <= ay && ax <= az)
    cand = {1, 0, 0};
  else if (ay <= az)
    cand = {0, 1, 0};
  else
    cand = {0, 0, 1};
  Vec3 N = normalize(cand - u * dot(cand, u));
  Vec3 E = normalize(cross(u, N));
  return {u, N, E};
}
inline Vec3 forwardVec(const Basis &b, double yawDeg, double pitchDeg) {
  double y = yawDeg * kDeg2Rad, p = pitchDeg * kDeg2Rad;
  return b.N * (std::cos(p) * std::cos(y)) + b.E * (std::cos(p) * std::sin(y)) + b.up * std::sin(p);
}
inline Vec3 rightVec(const Basis &b, double yawDeg) {
  double y = yawDeg * kDeg2Rad;
  return b.N * std::sin(y) - b.E * std::cos(y);
}
inline Vec3 orbitOffset(const Basis &b, double azDeg, double elDeg, double dist) {
  double a = azDeg * kDeg2Rad, e = elDeg * kDeg2Rad;
  return (b.N * (std::cos(e) * std::cos(a)) + b.E * (std::cos(e) * std::sin(a)) +
          b.up * std::sin(e)) *
         dist;
}
// yaw/pitch of a direction in a basis.
inline void dirToYawPitch(const Basis &b, Vec3 d, double &yawDeg, double &pitchDeg) {
  d = normalize(d);
  pitchDeg = std::asin(std::max(-1.0, std::min(1.0, dot(d, b.up)))) / kDeg2Rad;
  Vec3 h = d - b.up * dot(d, b.up);
  if (length(h) > 1e-9)
    yawDeg = std::atan2(dot(h, b.E), dot(h, b.N)) / kDeg2Rad;
}

} // namespace

// File-local interactor style: forwards VTK window events to a CameraController,
// swallows VTK's default single-key bindings, and (fly + captured) recenters the
// pointer for continuous mouse-look.
class CvcCameraInteractorStyle : public vtkInteractorStyle {
public:
  static CvcCameraInteractorStyle *New();
  vtkTypeMacro(CvcCameraInteractorStyle, vtkInteractorStyle);

  void setController(cvc::gl::CameraController *c) { m_controller = c; }

  void OnKeyDown() override {
    if (!m_controller || !this->Interactor)
      return;
    const char *ks = this->Interactor->GetKeySym();
    const std::string sym = ks ? ks : "";
    if (sym == "Escape") {
      m_controller->setPointerCapture(false);
      return;
    }
    m_controller->keyDown(sym);
  }
  void OnKeyUp() override {
    if (!m_controller || !this->Interactor)
      return;
    const char *ks = this->Interactor->GetKeySym();
    m_controller->keyUp(ks ? ks : "");
  }
  void OnChar() override {} // our navigation owns the keyboard

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
  // Middle button = pan, the standard "move the scene" drag.
  void OnMiddleButtonDown() override {
    if (m_controller)
      m_controller->beginPan();
  }
  void OnMiddleButtonUp() override {
    if (m_controller)
      m_controller->endPan();
  }

  void OnMouseWheelForward() override {
    if (m_controller)
      m_controller->mouseWheel(1.0);
  }
  void OnMouseWheelBackward() override {
    if (m_controller)
      m_controller->mouseWheel(-1.0);
  }

  // ---- multi-touch gestures (desktop touchscreens) -------------------------
  // vtkInteractorStyle declares these as EMPTY virtuals and only
  // vtkInteractorStyleMultiTouchCamera overrides them, so without these a
  // recognized pinch is silently dropped. VTK's recognizer reports an ABSOLUTE
  // scale relative to the gesture start, so convert to relative steps.
  // (In the browser VTK's recognizer never fires — its wasm interactor mis-feeds
  // multi-touch — which is what cvc::gl::TouchGestures exists to work around.)
  void OnStartPinch() override {
    if (this->Interactor)
      m_lastScale = this->Interactor->GetScale();
  }
  void OnPinch() override {
    if (!m_controller || !this->Interactor)
      return;
    const double s = this->Interactor->GetScale();
    if (s > 0.0 && m_lastScale > 0.0 && std::abs(s - m_lastScale) > 1e-6) {
      m_controller->mouseWheel(4.0 * std::log2(s / m_lastScale));
      m_lastScale = s;
    }
  }
  void OnEndPinch() override { m_lastScale = 1.0; }
  void OnPan() override {
    if (!m_controller || !this->Interactor)
      return;
    double *t = this->Interactor->GetTranslation();
    double *lt = this->Interactor->GetLastTranslation();
    if (!t || !lt)
      return;
    m_controller->beginDrag();
    m_controller->mouseLook(static_cast<int>(t[0] - lt[0]), static_cast<int>(t[1] - lt[1]));
    m_controller->endDrag();
  }

private:
  cvc::gl::CameraController *m_controller = nullptr;
  double m_lastScale = 1.0; // pinch: VTK reports absolute scale, we need relative
};
vtkStandardNewMacro(CvcCameraInteractorStyle);

namespace cvc {
namespace gl {

struct CameraController::Impl {
  // config / pose (mirrored to state)
  Mode mode = Mode::Orbit;
  Vec3 up{0, 0, 1};
  Vec3 orbitCenter{0, 0, 0};
  double orbitDistance = 10.0;
  double orbitAzimuth = -60.0;
  double orbitElevation = 30.0;
  Vec3 flyPos{0, 0, 0};
  double flyYaw = 0.0, flyPitch = 0.0;
  double moveSpeed = 5.0, sprintMultiplier = 4.0, sensitivity = 0.25;
  bool invertPitch = false, pointerCapture = false;
  double poseMirrorHz = 15.0;
  // key bindings (VTK key syms)
  std::string kForward = "w", kBackward = "s", kLeft = "a", kRight = "d";
  std::string kUp = "space", kDown = "Control_L", kToggle = "Tab", kSprint = "Shift_L";

  // cinematic track config (mirrored to state "track.*")
  std::string trackTarget;
  double trackBack = 55.0, trackHeight = 40.0, trackLookAhead = 0.0, trackLookUp = 3.0;
  double trackPosTau = 0.15, trackVelTau = 0.40, trackCamTau = 0.55, trackMinSpeed = 0.05;

  // runtime (not state)
  std::set<std::string> held;
  bool dragging = false;
  bool panning = false; // middle-button drag, or primary drag on touch
  bool primaryDragPans = false;
  double poseMirrorAccum = 0.0;
  // Map-mode fit: the rect frameMap() was asked to show, the aspect it was last
  // fitted at, and whether the user has since zoomed (which ends auto-fitting).
  double mapFitHalfW = 0.0, mapFitHalfH = 0.0, mapFitAspect = 0.0;
  bool mapFitUserZoomed = false;
  std::atomic<bool> selfWrite{false};
  // track smoothing state (harvested from ChaseCamera)
  Vec3 trackP, trackPrev, trackV, trackHead, trackEye, trackFocal;
  bool havP = false, havPrev = false, haveV = false, haveHead = false, haveEye = false;

  vtkCamera *camera = nullptr;
  vtkRenderer *renderer = nullptr;
  vtkRenderWindow *window = nullptr;
  vtkRenderWindowInteractor *interactor = nullptr;
  SceneGraph *scene = nullptr;
  vtkSmartPointer<CvcCameraInteractorStyle> style;

  Basis basis() const { return basisFromUp(up); }
  bool held_has(const std::string &k) const { return !k.empty() && held.count(k) > 0; }
};

std::string CameraController::viewerStatePath(const std::string &scenePrefix,
                                              const std::string &viewerName) {
  return scenePrefix + ".viewers." + viewerName + ".camera";
}

CameraController::CameraController(cvc::app &ctx, const std::string &statePath)
    : cvc::state_object<CameraController>(ctx, statePath), m_impl(std::make_unique<Impl>()) {
  // Synchronous reactions on the calling thread; no thread-per-change floods,
  // no teardown races. Drive the camera from the render thread.
  this->setInstanceThreading(false);
  seedState();
}

CameraController::CameraController(SceneRenderer &viewer)
    : CameraController(viewer.scene().appContext(),
                       viewerStatePath(viewer.scene().getStatePrefix(), viewer.name())) {
  setScene(&viewer.scene());
  setRenderer(viewer.renderer());
  if (viewer.renderer())
    setCamera(viewer.renderer()->GetActiveCamera());
  setRenderWindow(viewer.renderWindow());
  if (viewer.renderWindow())
    attach(viewer.renderWindow()->GetInteractor());
}

CameraController::~CameraController() { detach(); }

// ---- wiring ----
void CameraController::setCamera(vtkCamera *camera) { m_impl->camera = camera; }
void CameraController::setRenderer(vtkRenderer *renderer) { m_impl->renderer = renderer; }
void CameraController::setRenderWindow(vtkRenderWindow *window) { m_impl->window = window; }

void CameraController::attach(vtkRenderWindowInteractor *interactor) {
  m_impl->interactor = interactor;
  if (!interactor)
    return;
  if (!m_impl->renderer && interactor->GetRenderWindow())
    if (auto *rens = interactor->GetRenderWindow()->GetRenderers())
      m_impl->renderer = rens->GetFirstRenderer();
  if (!m_impl->window)
    m_impl->window = interactor->GetRenderWindow();
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

// ---- state seeding / sync ----
void CameraController::seedState() {
  cvc::state_init_scope<CameraController> guard(*this); // suppress change signals
  Impl &s = *m_impl;
  getState("mode").value(static_cast<int>(s.mode));
  getState("up.x").value(s.up.x);
  getState("up.y").value(s.up.y);
  getState("up.z").value(s.up.z);
  getState("orbit.center.x").value(s.orbitCenter.x);
  getState("orbit.center.y").value(s.orbitCenter.y);
  getState("orbit.center.z").value(s.orbitCenter.z);
  getState("orbit.distance").value(s.orbitDistance);
  getState("orbit.azimuth").value(s.orbitAzimuth);
  getState("orbit.elevation").value(s.orbitElevation);
  getState("fly.position.x").value(s.flyPos.x);
  getState("fly.position.y").value(s.flyPos.y);
  getState("fly.position.z").value(s.flyPos.z);
  getState("fly.yaw").value(s.flyYaw);
  getState("fly.pitch").value(s.flyPitch);
  getState("settings.move_speed").value(s.moveSpeed);
  getState("settings.sprint_multiplier").value(s.sprintMultiplier);
  getState("settings.mouse_sensitivity").value(s.sensitivity);
  getState("settings.invert_pitch").value(s.invertPitch ? 1 : 0);
  getState("settings.pointer_capture").value(s.pointerCapture ? 1 : 0);
  getState("settings.primary_drag_pans").value(s.primaryDragPans ? 1 : 0);
  getState("settings.pose_mirror_hz").value(s.poseMirrorHz);
  getState("keys.forward").value(s.kForward);
  getState("keys.backward").value(s.kBackward);
  getState("keys.strafe_left").value(s.kLeft);
  getState("keys.strafe_right").value(s.kRight);
  getState("keys.up").value(s.kUp);
  getState("keys.down").value(s.kDown);
  getState("keys.toggle_mode").value(s.kToggle);
  getState("keys.sprint").value(s.kSprint);
  getState("track.target").value(s.trackTarget);
  getState("track.back").value(s.trackBack);
  getState("track.height").value(s.trackHeight);
  getState("track.look_ahead").value(s.trackLookAhead);
  getState("track.look_up").value(s.trackLookUp);
  getState("track.pos_tau").value(s.trackPosTau);
  getState("track.vel_tau").value(s.trackVelTau);
  getState("track.cam_tau").value(s.trackCamTau);
  getState("track.min_speed").value(s.trackMinSpeed);
}

void CameraController::readAllFromState() {
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
  int mv = i("mode", static_cast<int>(s.mode));
  s.mode = mv == 2 ? Mode::Track : (mv == 1 ? Mode::Fly : Mode::Orbit);
  s.up = {d("up.x", s.up.x), d("up.y", s.up.y), d("up.z", s.up.z)};
  s.orbitCenter = {d("orbit.center.x", s.orbitCenter.x), d("orbit.center.y", s.orbitCenter.y),
                   d("orbit.center.z", s.orbitCenter.z)};
  s.orbitDistance = d("orbit.distance", s.orbitDistance);
  s.orbitAzimuth = d("orbit.azimuth", s.orbitAzimuth);
  s.orbitElevation = d("orbit.elevation", s.orbitElevation);
  s.flyPos = {d("fly.position.x", s.flyPos.x), d("fly.position.y", s.flyPos.y),
              d("fly.position.z", s.flyPos.z)};
  s.flyYaw = d("fly.yaw", s.flyYaw);
  s.flyPitch = d("fly.pitch", s.flyPitch);
  s.moveSpeed = d("settings.move_speed", s.moveSpeed);
  s.sprintMultiplier = d("settings.sprint_multiplier", s.sprintMultiplier);
  s.sensitivity = d("settings.mouse_sensitivity", s.sensitivity);
  s.invertPitch = i("settings.invert_pitch", s.invertPitch ? 1 : 0) != 0;
  s.pointerCapture = i("settings.pointer_capture", s.pointerCapture ? 1 : 0) != 0;
  s.primaryDragPans = i("settings.primary_drag_pans", s.primaryDragPans ? 1 : 0) != 0;
  s.poseMirrorHz = d("settings.pose_mirror_hz", s.poseMirrorHz);
  s.kForward = str("keys.forward", s.kForward);
  s.kBackward = str("keys.backward", s.kBackward);
  s.kLeft = str("keys.strafe_left", s.kLeft);
  s.kRight = str("keys.strafe_right", s.kRight);
  s.kUp = str("keys.up", s.kUp);
  s.kDown = str("keys.down", s.kDown);
  s.kToggle = str("keys.toggle_mode", s.kToggle);
  s.kSprint = str("keys.sprint", s.kSprint);
  s.trackTarget = str("track.target", s.trackTarget);
  s.trackBack = d("track.back", s.trackBack);
  s.trackHeight = d("track.height", s.trackHeight);
  s.trackLookAhead = d("track.look_ahead", s.trackLookAhead);
  s.trackLookUp = d("track.look_up", s.trackLookUp);
  s.trackPosTau = d("track.pos_tau", s.trackPosTau);
  s.trackVelTau = d("track.vel_tau", s.trackVelTau);
  s.trackCamTau = d("track.cam_tau", s.trackCamTau);
  s.trackMinSpeed = d("track.min_speed", s.trackMinSpeed);
}

void CameraController::syncConfigToState() {
  Impl &s = *m_impl;
  s.selfWrite = true;
  getState("mode").value(static_cast<int>(s.mode));
  getState("up.x").value(s.up.x);
  getState("up.y").value(s.up.y);
  getState("up.z").value(s.up.z);
  getState("orbit.center.x").value(s.orbitCenter.x);
  getState("orbit.center.y").value(s.orbitCenter.y);
  getState("orbit.center.z").value(s.orbitCenter.z);
  getState("orbit.distance").value(s.orbitDistance);
  getState("settings.move_speed").value(s.moveSpeed);
  getState("settings.sprint_multiplier").value(s.sprintMultiplier);
  getState("settings.mouse_sensitivity").value(s.sensitivity);
  getState("settings.invert_pitch").value(s.invertPitch ? 1 : 0);
  getState("settings.pointer_capture").value(s.pointerCapture ? 1 : 0);
  getState("settings.primary_drag_pans").value(s.primaryDragPans ? 1 : 0);
  getState("settings.pose_mirror_hz").value(s.poseMirrorHz);
  getState("keys.forward").value(s.kForward);
  getState("keys.backward").value(s.kBackward);
  getState("keys.strafe_left").value(s.kLeft);
  getState("keys.strafe_right").value(s.kRight);
  getState("keys.up").value(s.kUp);
  getState("keys.down").value(s.kDown);
  getState("keys.toggle_mode").value(s.kToggle);
  getState("keys.sprint").value(s.kSprint);
  getState("track.target").value(s.trackTarget);
  getState("track.back").value(s.trackBack);
  getState("track.height").value(s.trackHeight);
  getState("track.look_ahead").value(s.trackLookAhead);
  getState("track.look_up").value(s.trackLookUp);
  getState("track.pos_tau").value(s.trackPosTau);
  getState("track.vel_tau").value(s.trackVelTau);
  getState("track.cam_tau").value(s.trackCamTau);
  getState("track.min_speed").value(s.trackMinSpeed);
  s.selfWrite = false;
}

void CameraController::syncPoseToState() {
  Impl &s = *m_impl;
  s.selfWrite = true;
  getState("orbit.azimuth").value(s.orbitAzimuth);
  getState("orbit.elevation").value(s.orbitElevation);
  getState("orbit.distance").value(s.orbitDistance);
  getState("fly.position.x").value(s.flyPos.x);
  getState("fly.position.y").value(s.flyPos.y);
  getState("fly.position.z").value(s.flyPos.z);
  getState("fly.yaw").value(s.flyYaw);
  getState("fly.pitch").value(s.flyPitch);
  double e[3], f[3], u[3];
  getPose(e, f, u);
  getState("pose.eye.x").value(e[0]);
  getState("pose.eye.y").value(e[1]);
  getState("pose.eye.z").value(e[2]);
  getState("pose.focal.x").value(f[0]);
  getState("pose.focal.y").value(f[1]);
  getState("pose.focal.z").value(f[2]);
  s.selfWrite = false;
}

void CameraController::handleStateChanged(const std::string &childState) {
  (void)childState;
  if (m_impl->selfWrite.load())
    return; // our own write echoing back
  readAllFromState();
  applyToCamera();
}

// ---- mode / up ----
CameraController::Mode CameraController::mode() const { return m_impl->mode; }

// Reset the cinematic smoothing so Track eases in from the current view.
void CameraController::resetTrack() {
  Impl &s = *m_impl;
  s.havP = s.havPrev = s.haveV = s.haveHead = false;
  if (s.camera) {
    double e[3], f[3];
    s.camera->GetPosition(e);
    s.camera->GetFocalPoint(f);
    s.trackEye = {e[0], e[1], e[2]};
    s.trackFocal = {f[0], f[1], f[2]};
    s.haveEye = true;
  } else {
    s.haveEye = false;
  }
}

// World position of the tracked node (its transform origin). false if unresolved.
bool CameraController::trackedWorldPos(double out[3]) {
  Impl &s = *m_impl;
  if (!s.scene || s.trackTarget.empty())
    return false;
  auto node = s.scene->getGraphics(s.trackTarget);
  if (!node)
    return false;
  auto wt = node->getWorldTransform();
  if (!wt)
    return false;
  out[0] = wt->GetElement(0, 3);
  out[1] = wt->GetElement(1, 3);
  out[2] = wt->GetElement(2, 3);
  return true;
}

void CameraController::setMode(Mode m) {
  Impl &s = *m_impl;
  if (m == s.mode)
    return;
  Basis b = s.basis();
  // Seed the interactive modes seamlessly from the live camera pose.
  if (s.camera && (m == Mode::Orbit || m == Mode::Fly)) {
    double e[3], f[3];
    s.camera->GetPosition(e);
    s.camera->GetFocalPoint(f);
    Vec3 eye{e[0], e[1], e[2]}, focal{f[0], f[1], f[2]};
    if (m == Mode::Fly) {
      s.flyPos = eye;
      dirToYawPitch(b, focal - eye, s.flyYaw, s.flyPitch);
    } else {
      s.orbitCenter = focal;
      Vec3 off = eye - focal;
      s.orbitDistance = std::max(1e-6, length(off));
      dirToYawPitch(b, off, s.orbitAzimuth, s.orbitElevation);
    }
  }
  if (m == Mode::Track)
    resetTrack();
  s.mode = m;
  s.held.clear();
  setPointerCapture(m == Mode::Fly);
  syncConfigToState();
  syncPoseToState();
  applyToCamera();
}

// Tab toggles the two interactive modes; Track is entered explicitly (it needs a
// target). From Track, Tab returns to Orbit.
void CameraController::toggleMode() {
  // Map is a deliberate 2-D lock: Tab must not tumble the view out of it.
  if (mode() == Mode::Map)
    return;
  setMode(mode() == Mode::Orbit ? Mode::Fly : Mode::Orbit);
}

void CameraController::setScene(SceneGraph *scene) { m_impl->scene = scene; }

void CameraController::setTrackTarget(const std::string &nodeName) {
  m_impl->trackTarget = nodeName;
  if (m_impl->mode == Mode::Track)
    resetTrack();
  syncConfigToState();
}
std::string CameraController::trackTarget() const { return m_impl->trackTarget; }

void CameraController::setUpAxis(double x, double y, double z) {
  m_impl->up = normalize({x, y, z});
  syncConfigToState();
  applyToCamera();
}
void CameraController::getUpAxis(double &x, double &y, double &z) const {
  x = m_impl->up.x;
  y = m_impl->up.y;
  z = m_impl->up.z;
}

// The parallel scale that fits a (2*halfWidth x 2*halfHeight) rect in the
// current viewport. VTK's parallel scale is the half-HEIGHT of the view, so
// fitting the width means dividing by the aspect ratio; taking the max of the
// two fits the whole rect (letterboxed) instead of cropping its sides.
double CameraController::mapFitScale(double halfHeight, double halfWidth) const {
  if (halfWidth <= 0.0)
    return halfHeight;
  const double aspect = viewportAspect();
  if (aspect <= 0.0)
    return halfHeight; // unknown yet: refitMapIfResized() corrects on frame 1
  return std::max(halfHeight, halfWidth / aspect);
}

void CameraController::frameMap(double cx, double cy, double halfHeight, double halfWidth) {
  Impl &s = *m_impl;
  s.mode = Mode::Map;
  s.held.clear();
  setPointerCapture(false);
  s.mapFitHalfH = halfHeight;
  s.mapFitHalfW = halfWidth;
  s.mapFitUserZoomed = false;
  if (s.camera) {
    // Straight down the +z axis, +y up — a north-up map.
    s.camera->SetPosition(cx, cy, 1000.0);
    s.camera->SetFocalPoint(cx, cy, 0.0);
    s.camera->SetViewUp(0.0, 1.0, 0.0);
    s.camera->ParallelProjectionOn();
    s.camera->SetParallelScale(std::max(1e-3, mapFitScale(halfHeight, halfWidth)));
    if (s.renderer)
      s.renderer->ResetCameraClippingRange();
  }
  s.mapFitAspect = viewportAspect();
  syncConfigToState();
  syncPoseToState();
}

double CameraController::viewportAspect() const {
  const Impl &s = *m_impl;
  if (!s.renderer)
    return 0.0;
  // Prefer the window: a renderer only reports a real size once it has rendered,
  // so before the first frame it answers with a default that would fit wrongly.
  const int *sz = nullptr;
  if (vtkRenderWindow *w = s.renderer->GetRenderWindow())
    sz = w->GetSize();
  if (!sz || sz[0] <= 0 || sz[1] <= 0)
    sz = s.renderer->GetSize();
  if (!sz || sz[0] <= 0 || sz[1] <= 0)
    return 0.0;
  return static_cast<double>(sz[0]) / static_cast<double>(sz[1]);
}

// Re-fit after the viewport changes shape — a phone rotating, entering
// fullscreen, or any window resize. Without this the map crops the moment the
// aspect narrows. Once the user zooms, the framing is theirs and we stop.
void CameraController::refitMapIfResized() {
  Impl &s = *m_impl;
  if (s.mode != Mode::Map || s.mapFitHalfW <= 0.0 || s.mapFitUserZoomed || !s.camera)
    return;
  const double aspect = viewportAspect();
  if (aspect <= 0.0 || std::abs(aspect - s.mapFitAspect) < 1e-4)
    return;
  s.mapFitAspect = aspect;
  s.camera->SetParallelScale(std::max(1e-3, mapFitScale(s.mapFitHalfH, s.mapFitHalfW)));
  if (s.renderer)
    s.renderer->ResetCameraClippingRange();
}

void CameraController::setOrbitCenter(double x, double y, double z) {
  m_impl->orbitCenter = {x, y, z};
  syncConfigToState();
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
  s.moveSpeed = diag / 8.0;
  Basis b = s.basis();
  s.flyPos = s.orbitCenter + orbitOffset(b, s.orbitAzimuth, s.orbitElevation, s.orbitDistance);
  dirToYawPitch(b, s.orbitCenter - s.flyPos, s.flyYaw, s.flyPitch);
  syncConfigToState();
  syncPoseToState();
  applyToCamera();
}

// ---- per-frame ----
void CameraController::update(double dtSeconds) {
  Impl &s = *m_impl;
  refitMapIfResized(); // cheap: only does work when the viewport changed shape
  if (s.mode == Mode::Fly) {
    double fwd = (s.held_has(s.kForward) ? 1 : 0) - (s.held_has(s.kBackward) ? 1 : 0);
    double strafe = (s.held_has(s.kRight) ? 1 : 0) - (s.held_has(s.kLeft) ? 1 : 0);
    double rise = (s.held_has(s.kUp) ? 1 : 0) - (s.held_has(s.kDown) ? 1 : 0);
    if (fwd != 0 || strafe != 0 || rise != 0) {
      double speed = s.moveSpeed * dtSeconds;
      if (s.held_has(s.kSprint))
        speed *= s.sprintMultiplier;
      Basis b = s.basis();
      s.flyPos = s.flyPos + forwardVec(b, s.flyYaw, s.flyPitch) * (fwd * speed) +
                 rightVec(b, s.flyYaw) * (strafe * speed) + b.up * (rise * speed);
    }
  } else if (s.mode == Mode::Track) {
    // Cinematic follow of the tracked actor (ChaseCamera math, up-axis aware).
    double tp3[3];
    if (trackedWorldPos(tp3)) {
      Vec3 tp{tp3[0], tp3[1], tp3[2]};
      Basis b = s.basis();
      double dtc = std::max(dtSeconds, 1e-4);
      s.trackP = s.havP ? ema(s.trackP, tp, dtc, s.trackPosTau) : tp;
      s.havP = true;
      if (s.havPrev) {
        Vec3 rawv = (s.trackP - s.trackPrev) * (1.0 / dtc);
        s.trackV = s.haveV ? ema(s.trackV, rawv, dtc, s.trackVelTau) : rawv;
        s.haveV = true;
      }
      s.trackPrev = s.trackP;
      s.havPrev = true;
      if (s.haveV) {
        Vec3 vh = s.trackV - b.up * dot(s.trackV, b.up); // horizontal velocity
        double sp = length(vh);
        if (sp >= s.trackMinSpeed) {
          s.trackHead = vh * (1.0 / sp);
          s.haveHead = true;
        }
      }
      Vec3 h = s.haveHead ? s.trackHead : b.N; // hold heading (stop-safe)
      Vec3 teye = s.trackP - h * s.trackBack + b.up * s.trackHeight;
      Vec3 tlook = s.trackP + h * s.trackLookAhead + b.up * s.trackLookUp;
      if (s.haveEye) {
        s.trackEye = ema(s.trackEye, teye, dtc, s.trackCamTau);
        s.trackFocal = ema(s.trackFocal, tlook, dtc, s.trackCamTau);
      } else {
        s.trackEye = teye;
        s.trackFocal = tlook;
        s.haveEye = true;
      }
    }
  }
  applyToCamera();
  // Mirror the live pose to state on a throttle (per-frame writes are a known
  // state-tree perf sink; poseMirrorHz<=0 disables).
  if (s.poseMirrorHz > 0.0) {
    s.poseMirrorAccum += dtSeconds;
    if (s.poseMirrorAccum >= 1.0 / s.poseMirrorHz) {
      s.poseMirrorAccum = 0.0;
      syncPoseToState();
    }
  }
}

// ---- events ----
static std::string normKey(std::string k) {
  if (k.size() == 1 && k[0] >= 'A' && k[0] <= 'Z')
    k[0] = static_cast<char>(k[0] - 'A' + 'a');
  return k;
}
void CameraController::keyDown(const std::string &keySym) {
  std::string k = normKey(keySym);
  if (k == m_impl->kToggle) {
    toggleMode();
    return;
  }
  m_impl->held.insert(k);
}
void CameraController::keyUp(const std::string &keySym) { m_impl->held.erase(normKey(keySym)); }

void CameraController::mouseLook(int dxPixels, int dyPixels) {
  Impl &s = *m_impl;
  const double dpitch = (s.invertPitch ? -1.0 : 1.0) * dyPixels * s.sensitivity;
  if (s.mode == Mode::Fly) {
    s.flyYaw -= dxPixels * s.sensitivity;
    s.flyPitch = std::max(-89.0, std::min(89.0, s.flyPitch + dpitch));
    applyToCamera();
    // Recenter the OS pointer so continuous look never stops at the edge.
    if (s.pointerCapture)
      recenterPointer();
  } else if (s.mode == Mode::Map) {
    // 2-D map: drag PANS the view; rotation is deliberately unreachable.
    if (s.dragging && s.camera) {
      // Convert pixel motion to world units through the parallel scale so the
      // grabbed point stays under the cursor at any zoom.
      int *sz = s.window ? s.window->GetSize() : nullptr;
      const double vh = (sz && sz[1] > 0) ? sz[1] : 1.0;
      const double perPx = 2.0 * s.camera->GetParallelScale() / vh;
      double pos[3], foc[3];
      s.camera->GetPosition(pos);
      s.camera->GetFocalPoint(foc);
      // Screen right/up in world space (Map looks down -z with +y up).
      const double dx = -dxPixels * perPx, dy = dyPixels * perPx;
      pos[0] += dx;
      foc[0] += dx;
      pos[1] += dy;
      foc[1] += dy;
      s.camera->SetPosition(pos);
      s.camera->SetFocalPoint(foc);
      if (s.renderer)
        s.renderer->ResetCameraClippingRange();
      syncPoseToState();
    }
  } else if (s.dragging && (s.panning || s.primaryDragPans)) {
    // PAN: slide the orbit centre across the screen plane. Scaled by orbit
    // distance so the world tracks the cursor at any zoom, and by the vertical
    // field of view so it is right for any lens.
    if (s.camera) {
      const Basis b = basisFromUp(s.up);
      const Vec3 fwd = normalize(orbitOffset(b, s.orbitAzimuth, s.orbitElevation, 1.0) * -1.0);
      const Vec3 right = normalize(cross(fwd, b.up));
      const Vec3 up = normalize(cross(right, fwd));
      int *sz = s.window ? s.window->GetSize() : nullptr;
      const double vh = (sz && sz[1] > 0) ? sz[1] : 1.0;
      const double perPx =
          2.0 * s.orbitDistance * std::tan(0.5 * s.camera->GetViewAngle() * kDeg2Rad) / vh;
      s.orbitCenter = s.orbitCenter + right * (-dxPixels * perPx) + up * (-dyPixels * perPx);
      applyToCamera();
      syncConfigToState();
    }
  } else if (s.dragging) {
    s.orbitAzimuth -= dxPixels * s.sensitivity;
    s.orbitElevation = std::max(-89.0, std::min(89.0, s.orbitElevation + dpitch));
    applyToCamera();
  }
}

void CameraController::mouseWheel(double steps) {
  Impl &s = *m_impl;
  if (s.mode == Mode::Fly)
    s.moveSpeed = std::max(1e-4, s.moveSpeed * std::pow(1.25, steps));
  else if (s.mode == Mode::Map) {
    if (s.camera) {              // zoom = parallel scale, the only 2-D zoom that means anything
      s.mapFitUserZoomed = true; // their framing now — stop auto-fitting on resize
      s.camera->SetParallelScale(
          std::max(1e-3, s.camera->GetParallelScale() * std::pow(0.9, steps)));
      if (s.renderer)
        s.renderer->ResetCameraClippingRange();
    }
  } else {
    s.orbitDistance = std::max(1e-4, s.orbitDistance * std::pow(0.9, steps));
    applyToCamera();
  }
  syncConfigToState();
}

void CameraController::dolly(double steps) {
  Impl &s = *m_impl;
  if (s.mode != Mode::Fly) {
    mouseWheel(steps); // orbit/map already zoom correctly through the wheel
    return;
  }
  // Step along the view direction, scaled by move speed so the gesture feels
  // the same in a room-sized scene and an island-sized one.
  const Basis b = basisFromUp(s.up);
  const Vec3 fwd = forwardVec(b, s.flyYaw, s.flyPitch);
  s.flyPos = s.flyPos + fwd * (steps * s.moveSpeed * 0.35);
  applyToCamera();
  syncPoseToState();
}

void CameraController::beginPan() {
  m_impl->panning = true;
  m_impl->dragging = true;
}
void CameraController::endPan() {
  m_impl->panning = false;
  m_impl->dragging = false;
}
void CameraController::setPrimaryDragPans(bool on) {
  m_impl->primaryDragPans = on;
  // selfWrite, like every other config writer here. Without it this write
  // echoes back through handleStateChanged -> readAllFromState ->
  // applyToCamera, which re-reads a pose that is only mirrored at poseMirrorHz
  // and so can undo motion applied in the same frame — which is exactly what
  // made TouchGestures' old flag-flipping two-finger drag a no-op.
  m_impl->selfWrite = true;
  getState("settings.primary_drag_pans").value(on ? 1 : 0);
  m_impl->selfWrite = false;
}
bool CameraController::primaryDragPans() const { return m_impl->primaryDragPans; }

void CameraController::beginDrag() { m_impl->dragging = true; }
void CameraController::endDrag() { m_impl->dragging = false; }

// ---- tunables ----
// _ctx is the protected app reference state_object already holds; a reference
// member is unaffected by this method's constness, so no cast is needed.
cvc::app &CameraController::appContext() const { return _ctx; }

void CameraController::setMoveSpeed(double u) {
  m_impl->moveSpeed = u;
  syncConfigToState();
}
void CameraController::setSprintMultiplier(double f) {
  m_impl->sprintMultiplier = f;
  syncConfigToState();
}
void CameraController::setMouseSensitivity(double d) {
  m_impl->sensitivity = d;
  syncConfigToState();
}
void CameraController::setInvertPitch(bool invert) {
  m_impl->invertPitch = invert;
  syncConfigToState();
}
void CameraController::setPoseMirrorHz(double hz) {
  m_impl->poseMirrorHz = hz;
  syncConfigToState();
}

void CameraController::releaseHeldKeys() { m_impl->held.clear(); }

void CameraController::setPointerCapture(bool on) {
  Impl &s = *m_impl;
  s.pointerCapture = on;
  if (s.window) {
    if (on)
      s.window->HideCursor();
    else
      s.window->ShowCursor();
  }
  syncConfigToState();
}
bool CameraController::pointerCapture() const { return m_impl->pointerCapture; }

void CameraController::recenterPointer() {
#ifdef CVCGL_HAVE_X11
  Impl &s = *m_impl;
  if (!s.window)
    return;
  auto *dpy = static_cast<Display *>(s.window->GetGenericDisplayId());
  auto win = reinterpret_cast<Window>(s.window->GetGenericWindowId());
  if (!dpy || !win)
    return;
  int *size = s.window->GetSize();
  int cx = size[0] / 2, cy = size[1] / 2;
  XWarpPointer(dpy, None, win, 0, 0, 0, 0, cx, cy);
  XFlush(dpy);
  // Tell VTK the pointer is now at centre so the next move delta is measured from
  // there rather than reporting a huge jump.
  if (s.interactor)
    s.interactor->SetLastEventPosition(cx, size[1] - cy);
#endif
}

void CameraController::setKeyBinding(const std::string &action, const std::string &keySym) {
  Impl &s = *m_impl;
  if (action == "forward")
    s.kForward = keySym;
  else if (action == "backward")
    s.kBackward = keySym;
  else if (action == "strafe_left")
    s.kLeft = keySym;
  else if (action == "strafe_right")
    s.kRight = keySym;
  else if (action == "up")
    s.kUp = keySym;
  else if (action == "down")
    s.kDown = keySym;
  else if (action == "toggle_mode")
    s.kToggle = keySym;
  else if (action == "sprint")
    s.kSprint = keySym;
  syncConfigToState();
}
std::string CameraController::keyBinding(const std::string &action) const {
  Impl &s = *m_impl;
  if (action == "forward")
    return s.kForward;
  if (action == "backward")
    return s.kBackward;
  if (action == "strafe_left")
    return s.kLeft;
  if (action == "strafe_right")
    return s.kRight;
  if (action == "up")
    return s.kUp;
  if (action == "down")
    return s.kDown;
  if (action == "toggle_mode")
    return s.kToggle;
  if (action == "sprint")
    return s.kSprint;
  return "";
}

// ---- apply / query ----
void CameraController::applyToCamera() {
  Impl &s = *m_impl;
  if (!s.camera)
    return;
  double e[3], f[3], u[3];
  getPose(e, f, u);
  s.camera->SetPosition(e);
  s.camera->SetFocalPoint(f);
  s.camera->SetViewUp(u);
  if (s.renderer)
    s.renderer->ResetCameraClippingRange();
}

void CameraController::getPose(double eye[3], double focal[3], double up[3]) const {
  Impl &s = *m_impl;
  Basis b = s.basis();
  Vec3 e, f;
  if (s.mode == Mode::Fly) {
    e = s.flyPos;
    f = s.flyPos + forwardVec(b, s.flyYaw, s.flyPitch);
  } else if (s.mode == Mode::Track) {
    e = s.trackEye;
    f = s.trackFocal;
  } else {
    e = s.orbitCenter + orbitOffset(b, s.orbitAzimuth, s.orbitElevation, s.orbitDistance);
    f = s.orbitCenter;
  }
  eye[0] = e.x;
  eye[1] = e.y;
  eye[2] = e.z;
  focal[0] = f.x;
  focal[1] = f.y;
  focal[2] = f.z;
  up[0] = b.up.x;
  up[1] = b.up.y;
  up[2] = b.up.z;
}

} // namespace gl
} // namespace cvc
