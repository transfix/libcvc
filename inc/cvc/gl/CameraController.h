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

#include <cvc/core/state_object.h>

class vtkCamera;                 // VTK (global namespace)
class vtkRenderer;               // VTK (global namespace)
class vtkRenderWindow;           // VTK (global namespace)
class vtkRenderWindowInteractor; // VTK (global namespace)
class SceneRenderer;             // cvcGL (global namespace)
class SceneGraph;                // cvcGL (global namespace)

namespace cvc {
class app;
namespace gl {

// -----------------
// CameraController
// -----------------
// Built-in camera navigation for cvcGL: an ORBIT mode (turntable — azimuth /
// elevation about a center, wheel dollies, drag to look) and a Quake-style FLY
// mode (WASD along the look vector, mouse-look with the pointer captured, Space/
// Ctrl for world up/down, Shift sprint, wheel = speed), with a runtime toggle.
//
// EVERYTHING IS cvc::state. CameraController is a state_object: mode, the up
// axis, orbit center/distance/azimuth/elevation, fly position/yaw/pitch, the
// movement/mouse/sprint settings, pointer-capture, AND the key bindings all live
// in the state tree and are two-way bound — set them from anywhere (a UI, a
// script, a saved file) and the camera follows; drive the camera and the state
// reflects it. handleStateChanged runs synchronously (setInstanceThreading
// false), and config is applied on the render thread via update()/applyToCamera.
//
// CANONICAL, VIEWER-ASSOCIATED LOCATION. Constructed from a SceneRenderer, the
// state roots at "<scene prefix>.viewers.<viewer name>.camera" — so it is obvious
// which viewer a camera belongs to, and multiple viewers of one scene get
// distinct, discoverable camera state. (A low-level ctor takes an explicit app +
// path for headless / custom use.)
//
// UP AXIS is configurable (state "up.{x,y,z}", default +Z — cvc scenes are Z-up).
// The fly/orbit basis is derived from it, so a Y-up scene works by setting up=+Y.
//
// Two drive paths that compose: constructing from a SceneRenderer auto-installs an
// internal vtkInteractorStyle so the onscreen window steers the camera; or feed
// events yourself (keyDown/keyUp/mouseLook/...) from any host loop. Call
// update(dtSeconds) once per rendered frame — held-key motion is integrated there
// (frame-rate independent) and the live pose is mirrored to state on a throttle.
//
class CameraController : public cvc::state_object<CameraController> {
public:
  // Orbit (turntable), Fly (Quake), Track (cinematic follow of a scene actor).
  enum class Mode { Orbit = 0, Fly = 1, Track = 2 };

  // Canonical: state at "<viewer.scene prefix>.viewers.<viewer.name>.camera",
  // rooted in the viewer's scene app, and auto-wired to the viewer's camera,
  // renderer and (onscreen) interactor.
  explicit CameraController(SceneRenderer &viewer);

  // Low-level: explicit app context + full state path (headless / custom).
  CameraController(cvc::app &ctx, const std::string &statePath);

  ~CameraController();
  CameraController(const CameraController &) = delete;
  CameraController &operator=(const CameraController &) = delete;

  // The canonical state path for a viewer's camera.
  static std::string viewerStatePath(const std::string &scenePrefix,
                                     const std::string &viewerName);

  // Wiring (the SceneRenderer ctor does all of this for you).
  void setCamera(vtkCamera *camera);
  void setRenderer(vtkRenderer *renderer);
  void setRenderWindow(vtkRenderWindow *window); // for onscreen pointer capture
  void attach(vtkRenderWindowInteractor *interactor);
  void detach();

  // Mode. toggleMode() carries the pose across so the switch is seamless. Written
  // to / read from state "mode".
  void setMode(Mode m);
  Mode mode() const;
  void toggleMode();

  // World up axis (state "up"). Default +Z.
  void setUpAxis(double x, double y, double z);
  void getUpAxis(double &x, double &y, double &z) const;

  // Frame the whole scene: orbit center = box center, distance from its diagonal,
  // a pleasant 3/4 default view, auto move-speed. Seeds the fly pose too.
  void frameBounds(double minX, double minY, double minZ, double maxX, double maxY, double maxZ);

  // Orbit center (state "orbit.center").
  void setOrbitCenter(double x, double y, double z);

  // Cinematic TRACK mode (the third mode) — follow a named actor in the scene
  // with two-stage smoothing (position -> heading) and a critically-damped pose,
  // so the view eases filmically even when the actor's motion is noisy. Harvested
  // from grl-snam's ChaseCamera. The actor's world position comes from its scene
  // node each frame; set which node via state "track.target" or setTrackTarget().
  // Trailing distance/height/look-ahead/easing time-constants are state "track.*"
  // (back, height, look_ahead, look_up, pos_tau, vel_tau, cam_tau, min_speed).
  // The scene to resolve the actor in is taken from the viewer (or setScene()).
  void setScene(SceneGraph *scene);
  void setTrackTarget(const std::string &nodeName);
  std::string trackTarget() const;

  // Integrate held-key fly motion, push the pose to the camera, and mirror the
  // live pose to state on a throttle. Call once per rendered frame.
  void update(double dtSeconds);

  // ---- manual event feed (also called by the attached interactor style) ----
  void keyDown(const std::string &keySym);
  void keyUp(const std::string &keySym);
  void mouseLook(int dxPixels, int dyPixels);
  void mouseWheel(double steps);
  void beginDrag();
  void endDrag();

  // ---- tunables (mirrored to state "settings.*") ----
  void setMoveSpeed(double unitsPerSecond);
  void setSprintMultiplier(double factor);
  void setMouseSensitivity(double degPerPixel);
  void setInvertPitch(bool invert);
  void setPoseMirrorHz(double hz); // rate the live pose is written to state (0=off)

  // Quake pointer capture: hide the cursor and recenter it each frame so mouse-
  // look is continuous (no window-edge stop). Auto-enabled in fly mode; Escape
  // releases it. State "settings.pointer_capture". Recentering is X11 today; on
  // other platforms it degrades to cursor-delta look.
  void setPointerCapture(bool on);
  bool pointerCapture() const;

  // Rebind a movement key (also settable via state "keys.<action>"). Actions:
  // "forward","backward","strafe_left","strafe_right","up","down","toggle_mode",
  // "sprint". Values are VTK key syms ("w","space","Control_L","Tab","Shift_L").
  void setKeyBinding(const std::string &action, const std::string &keySym);
  std::string keyBinding(const std::string &action) const;

  // Push the current pose to the vtkCamera and reset the clipping range.
  void applyToCamera();
  void getPose(double eye[3], double focal[3], double up[3]) const;

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void seedState();          // write defaults with the change signal suppressed
  void readAllFromState();   // pull every setting/pose from state into members
  void syncConfigToState();  // write config back (guarded against echo)
  void syncPoseToState();    // write the live pose back (guarded against echo)
  void recenterPointer();    // X11 pointer warp for continuous captured look
  void resetTrack();         // reset cinematic smoothing (ease in from current view)
  bool trackedWorldPos(double out[3]); // world pos of the tracked actor, or false

  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_CAMERA_CONTROLLER_H__
