/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_CAMERA_CONTROLLER_H__
#define __CVC_GL_CAMERA_CONTROLLER_H__

#include <memory>
#include <string>

class vtkCamera;                 // VTK (global namespace)
class vtkRenderer;               // VTK (global namespace)
class vtkRenderWindowInteractor; // VTK (global namespace)

namespace cvc {
namespace gl {

// -----------------
// CameraController
// -----------------
// Built-in camera navigation for cvcGL: an ORBIT mode (turntable — azimuth /
// elevation about a center, wheel dollies, drag to look) and a Quake-style FLY
// mode (WASD strafe/forward, mouse-look, Space/Ctrl for world up/down, Shift to
// sprint, wheel changes speed), with a runtime toggle between them.
//
// cvcGL is otherwise headless — SceneGraph::setRenderer only touches props, never
// the vtkCamera — so before this the view was 100% the host's problem. This makes
// "orbit + fly" a first-class capability so a lab demo (or, eventually, a real-
// time sim / game loop) gets usable navigation for free.
//
// WORLD IS Z-UP. Unlike volrover3's Qt CameraController (whose fly/orbit math is
// hardcoded Y-up), every axis here treats +Z as world up, which is what cvc's
// scenes (terrain, forests, volumes) are built in.
//
// Two ways to drive it, and they compose:
//   * attach(interactor) installs an internal vtkInteractorStyle so an ONSCREEN
//     SceneRenderer window steers the camera directly — the zero-wiring path.
//   * feed events yourself (keyDown/keyUp/mouseLook/mouseWheel/beginDrag) from
//     any host loop (Qt, SDL, a headless script) — the toolkit-agnostic path.
// Call update(dtSeconds) ONCE PER FRAME: held-key fly motion is integrated there
// (frame-rate independent), not per-event, so movement is smooth under a manual
// render loop.
//
// Not a SceneNode and not on the state tree: it owns plain camera pose and writes
// a vtkCamera. Construct/drive it on the thread that pumps the SceneGraph.
//
class CameraController {
public:
  enum class Mode { Orbit, Fly };

  CameraController();
  ~CameraController();
  CameraController(const CameraController &) = delete;
  CameraController &operator=(const CameraController &) = delete;

  // The camera this drives (e.g. SceneRenderer::renderer()->GetActiveCamera()).
  // The renderer is used to reset the clipping range as the view moves and to
  // frame bounds; pass it too (SceneRenderer::renderer()).
  void setCamera(vtkCamera *camera);
  void setRenderer(vtkRenderer *renderer);

  // Install an internal interactor style on `interactor` so an onscreen window
  // drives the camera. Captures the renderer + camera from the interactor's
  // first renderer if they were not set explicitly. detach() restores nothing —
  // it just stops receiving events.
  void attach(vtkRenderWindowInteractor *interactor);
  void detach();

  // Mode. toggleMode() carries the current pose across (fly seeds from the orbit
  // eye/look, orbit seeds its center from where fly is looking) so the switch is
  // seamless. The interactor style binds Tab to toggleMode().
  void setMode(Mode m);
  Mode mode() const;
  void toggleMode();

  // Frame the whole scene: orbit center = box center, distance from its diagonal,
  // a pleasant 3/4 default view, and an auto move-speed scaled to the scene. Also
  // seeds the fly pose. Applies immediately.
  void frameBounds(double minX, double minY, double minZ, double maxX, double maxY, double maxZ);

  // Integrate held-key fly motion and push the pose to the camera. Call once per
  // rendered frame with the real elapsed seconds.
  void update(double dtSeconds);

  // ---- manual event feed (no-ops that are also called by the attached style) ----
  void keyDown(const std::string &keySym); // "w","a","s","d","space","Control_L","Shift_L","Tab"
  void keyUp(const std::string &keySym);
  void mouseLook(int dxPixels, int dyPixels); // relative look; fly=always, orbit=while dragging
  void mouseWheel(double steps);              // fly: speed; orbit: dolly
  void beginDrag();                           // orbit: start look drag (e.g. left button down)
  void endDrag();

  // ---- tunables ----
  void setMoveSpeed(double unitsPerSecond); // base fly speed (frameBounds auto-sets it)
  void setSprintMultiplier(double factor);  // Shift multiplier (default 4x)
  void setMouseSensitivity(double degPerPixel); // look speed (default 0.25)
  void setInvertPitch(bool invert);

  // Push the current pose to the vtkCamera and reset the clipping range. update()
  // and frameBounds() call this; exposed for callers that change tunables/pose
  // and want to apply without a frame step.
  void applyToCamera();

  void getPose(double eye[3], double focal[3], double up[3]) const;

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_CAMERA_CONTROLLER_H__
