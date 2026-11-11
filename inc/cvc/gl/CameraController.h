/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_CAMERA_CONTROLLER_H__
#define __CVC_GL_CAMERA_CONTROLLER_H__

#include <cvc/state/state_object.h>
#include <memory>
#include <string>
#include <vector>

class vtkCamera;                 // VTK (global namespace)
class vtkRenderer;               // VTK (global namespace)
class vtkRenderWindow;           // VTK (global namespace)
class vtkRenderWindowInteractor; // VTK (global namespace)

namespace cvc {
class app;
namespace gl {
class SceneRenderer;
class SceneGraph;

// -----------------
// CameraController
// -----------------
// Built-in camera navigation for cvcGL: an ORBIT mode (turntable — azimuth /
// elevation about a center, wheel dollies, drag to look) and a Quake-style FLY
// mode (WASD along the look vector, mouse-look with the pointer captured, Space/
// Ctrl for world up/down, Shift sprint, wheel = speed), with a runtime toggle.
//
// EVERYTHING IS cvc::state. CameraController is a state_object: mode, the up
// axis, orbit center/distance/azimuth/elevation, fly position/yaw/pitch, map
// center/scale, the movement/mouse/sprint settings, pointer-capture, AND the key
// bindings all live in the state tree and are two-way bound — set them from
// anywhere (a UI, a script, a saved file) and the camera follows; drive the
// camera and the state reflects it. handleStateChanged runs synchronously
// (setInstanceThreading false), and config is applied on the render thread via
// update()/applyToCamera.
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
  // Map = a true 2-D map view: parallel projection straight down -z with north
  // (+y) up (Z-up scenes; Map does not follow setUpAxis), drag PANS -- the
  // grabbed point stays under the pointer -- and the wheel ZOOMS, and rotation is
  // impossible. For ortho "matplotlib" views where tumbling the camera only ever
  // disorients.
  enum class Mode { Orbit = 0, Fly = 1, Track = 2, Map = 3 };

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
  static std::string viewerStatePath(const std::string &scenePrefix, const std::string &viewerName);

  // Wiring (the SceneRenderer ctor does all of this for you).
  void setCamera(vtkCamera *camera);
  void setRenderer(vtkRenderer *renderer);
  void setRenderWindow(vtkRenderWindow *window); // for onscreen pointer capture
  void attach(vtkRenderWindowInteractor *interactor);
  void detach();

  // Mode. toggleMode() carries the pose across so the switch is seamless. Written
  // to / read from state "mode".
  //
  // Map keeps its OWN pose (state "map.*"): entering it shows the map as it was
  // last framed, panned or zoomed, and leaving it returns Orbit and Fly to the
  // poses they had -- a view straight down has no heading to carry across --
  // with perspective projection restored. So Orbit -> Map -> Orbit comes back
  // to the same orbit view, and Map -> Orbit -> Map to the same map.
  void setMode(Mode m);
  Mode mode() const;

  // ---- panning (translating the view) --------------------------------------
  // A drag with the MIDDLE button always pans: it slides the orbit centre in the
  // screen plane, which is the "move the scene" gesture people expect from every
  // other 3-D tool.
  void beginPan();
  void endPan();

  // Make the PRIMARY drag pan instead of orbit. Two-way bound at state
  // "settings.primary_drag_pans" — note the "settings." prefix; the bare
  // "primary_drag_pans" this once documented never existed, and the key was
  // written but never read back, so it was inert from the state tree.
  //
  // Default OFF, and TouchGestures deliberately does NOT set it: forcing it on
  // there made every drag pan, mouse included, because this is the camera's
  // flag rather than a touch-only one, and orbit became unreachable. The
  // gesture split is one finger / left-drag ORBITS, two fingers / middle-drag
  // TRANSLATES — TouchGestures gets the second by bracketing the drag with
  // beginPan()/endPan(), not by toggling this.
  void setPrimaryDragPans(bool on);
  bool primaryDragPans() const;

  // Make the PRIMARY drag STEER instead of orbit, in Orbit without mouse look: a press on the
  // scene and the drag that follows turn nothing here, and steerPoint() says where the pointer is
  // while the button is held, so a host can turn what it walks toward the point under it. The
  // RIGHT button (two fingers, through TouchGestures) turns the view instead; it does in any case.
  // A release near its press is still a click (takeClick). State "settings.primary_drag_steers";
  // off by default.
  void setPrimaryDragSteers(bool on);
  bool primaryDragSteers() const;
  // The primary button is down in a press that steers, from the press on: the interactor begins
  // no drag (no turn of the view) for it.
  bool steering() const;
  // While a steering drag is held, once it has gone further than a click's slop (4 px) from its
  // press: true, and where the pointer is (VTK display coordinates, origin bottom-left, as
  // takeClick's).
  bool steerPoint(int &x, int &y) const;

  // Move the eye along its own view direction by `steps` (positive = forward).
  // This is what "zoom" has to mean in FLY mode, where there is no orbit centre
  // to pull toward and mouseWheel() adjusts movement SPEED instead. Step size
  // scales with the current move speed, so it feels the same at any scale.
  void dolly(double steps);
  void toggleMode();

  // World up axis (state "up"). Default +Z.
  void setUpAxis(double x, double y, double z);
  void getUpAxis(double &x, double &y, double &z) const;

  // Frame the whole scene: orbit center = box center, distance from its diagonal,
  // a pleasant 3/4 default view, auto move-speed. Seeds the fly pose too, and
  // the map pose (the box's x/y footprint, fitted like frameMap with a width).
  void frameBounds(double minX, double minY, double minZ, double maxX, double maxY, double maxZ);

  // Back to the view a scene opens with: frameBounds() of everything the scene draws now (so a
  // world regenerated larger is framed whole), else of the bounds frameBounds() last framed. Fly
  // and Track return to Orbit (a captured pointer is let go); Map stays Map, refitted. The home
  // key (state "keys.home", default "H") and Home do it, and so does a write of anything to state
  // "home" (a button: (state-set "viewers.main.camera.home" "1")), which is cleared again.
  void resetView();

  // In Orbit and Map the movement keys (and the arrow keys) slide the view across the ground --
  // forward / back / left / right, the same share of the view a second at any zoom -- and up /
  // down (Space / Left Ctrl) zoom out / in; sprint is faster. On by default; state
  // "settings.orbit_keys". A host whose Orbit keys do something else (arigl's room walks its avatar
  // with them) turns it off. Fly moves with them whatever this says.
  void setOrbitKeys(bool on);
  bool orbitKeys() const;

  // Map mode framing: look straight down at (cx, cy) with a half-height of
  // `halfHeight` world units (the parallel scale). Sets Mode::Map. The result is
  // the map pose (state "map.center.{x,y}", "map.scale"), which update() holds
  // and pans / zooms move -- not a one-off camera write the next update() undoes.
  //
  // Pass `halfWidth` > 0 to fit the whole RECT rather than just its height.
  // VTK's parallel scale is a half-HEIGHT, so a plain height fit crops the sides
  // on any viewport narrower than the map — which is every phone in portrait.
  // With a half-width the controller keeps the rect framed across resizes too
  // (rotation, entering fullscreen), re-fitting when the viewport aspect
  // changes, until the user zooms and takes over.
  void frameMap(double cx, double cy, double halfHeight, double halfWidth = 0.0);

  // Orbit center (state "orbit.center").
  void setOrbitCenter(double x, double y, double z);
  // The orbit's distance from its centre (state "orbit.distance"; the wheel changes it).
  double orbitDistance() const;
  // How far the orbit's elevation may go, in degrees (state "orbit.min_elevation" and
  // "orbit.max_elevation"; -89 and 89 by default). A third-person camera over a floor keeps it
  // above the floor this way: looking up past the lower limit would put the eye under it.
  void setOrbitElevationRange(double minDeg, double maxDeg);
  // A floor the orbit's eye stays above, `height` along the up axis plus `clearance` (state
  // "orbit.floor.on", "orbit.floor.height", "orbit.floor.clearance"; off by default). An eye
  // that would go under it -- a third-person view looking up -- slides in along its line of sight
  // toward the centre instead, so the view can look straight up from just above the floor. The
  // orbit's distance is kept: the eye goes back out as the view comes down.
  void setOrbitFloor(double height, double clearance = 0.25);
  void clearOrbitFloor();

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

  // Phase A — feed the tracked target from a SIMULATION STREAM instead of a
  // scene node: the first feedTrackTarget() switches Track to push mode (the
  // scene node is ignored; the last fed value persists so a dropped frame does
  // not teleport). setTrackTarget(non-empty) or clearTrackFeed() reverts to
  // node-pull. resetTracking() clears the smoothing state (a fresh cut). This is
  // the seam that lets one Track definition serve grl-snam's pushed ChaseCamera
  // stream AND a node-follow, identically in C++ and Python.
  void feedTrackTarget(double x, double y, double z);
  void clearTrackFeed();
  bool trackFed() const;
  void resetTracking();
  // Asymmetric follow easing: a separate (usually slower) camera time-constant
  // when the eye is falling behind an accelerating target (the trail "widens").
  // 0 (default) = symmetric = bit-identical to before. State "track.widen_tau".
  void setWidenTau(double tau);
  double widenTau() const;

  // Integrate held-key fly motion, push the pose to the camera, and mirror the
  // live pose to state on a throttle. Call once per rendered frame.
  void update(double dtSeconds);

  // ---- manual event feed (also called by the attached interactor style) ----
  void keyDown(const std::string &keySym);
  void keyUp(const std::string &keySym);
  void mouseLook(int dxPixels, int dyPixels);
  void mouseWheel(double steps);
  void beginDrag();
  // The right button's drag: it turns the view (Orbit), whatever the primary drag does.
  void beginSecondaryDrag();
  void endSecondaryDrag();
  // Primary-button press / release positions (VTK display coordinates: origin bottom-left), from
  // the attached interactor or fed by a host. A release within 4 px of its press is a CLICK -- the
  // drag it also began barely moved the camera -- and takeClick() returns the latest click's
  // position once (then clears it), so a host can act on clicks in the scene (e.g. "face there")
  // while drags keep orbiting. A click the UI overlay swallowed never reaches here.
  void notePointerDown(int x, int y);
  void notePointerMove(int x, int y); // where a steering drag's pointer is now
  void notePointerUp(int x, int y);
  bool takeClick(int &x, int &y);
  void endDrag();
  // A gesture took over the press (a second finger landed, TouchGestures): it is no click when it
  // comes up, and its drag, turning or steering, ends here.
  void cancelPointer();

  // The app context this controller was built with. Together with the inherited
  // stateName() it lets a UI bind widgets to "settings.*" without the caller
  // hand-splicing the path — three demos spliced it as a literal, which would
  // have gone quietly wrong if viewerStatePath()'s scheme ever changed.
  cvc::app &appContext() const;

  // ---- tunables (mirrored to state "settings.*") ----
  void setMoveSpeed(double unitsPerSecond);
  void setSprintMultiplier(double factor);
  void setMouseSensitivity(double degPerPixel);
  void setInvertPitch(bool invert);
  void setInvertYaw(bool invert);  // horizontal look + drag, any mode ("settings.invert_yaw")
  void setPoseMirrorHz(double hz); // rate the live pose is written to state (0=off)
  // The wheel's zoom glides to where its notches put it: each frame closes 1 - e^(-dt / tau) of
  // the remaining ratio in update(). 0 zooms at once on each notch. Default 0.15 s; state
  // "settings.zoom_tau". zooming() says a glide is under way.
  void setZoomSmoothing(double tauSeconds);
  double zoomSmoothing() const;
  bool zooming() const;
  // Vertical field of view in degrees for this controller's camera (each viewport
  // carries its own lens). 0 (default) leaves the VTK default untouched; a value
  // is applied in perspective modes (not Map). State "settings.field_of_view".
  void setFieldOfView(double degrees);
  double fieldOfView() const;

  // Pointer capture: the cursor is hidden and the pointer's motion keeps coming at the window's
  // edge, so mouse-look is continuous. Auto-enabled in fly mode; Escape releases it. State
  // "settings.pointer_capture". cvcGL's SDL window uses SDL's relative mouse mode (released while
  // the window has no focus); in the browser it is the page's pointer lock, which the browser
  // ends on Escape or when the tab loses focus (pointerCapture() then reads false); VTK's own
  // windows hide the cursor and warp it back to the centre after each move.
  void setPointerCapture(bool on);
  bool pointerCapture() const;

  // Third-person MOUSE LOOK, in Orbit: with the pointer captured, its motion turns the orbit as a
  // drag does, with no button held -- the game camera, where a host walks its avatar the way the
  // camera faces. A click on the scene captures the pointer and is not reported (takeClick);
  // Escape releases it, and the pointer is the UI's again until the next click on the scene,
  // drags orbiting as before. Turning it off releases the pointer. State "settings.mouse_look";
  // off by default.
  void setMouseLookEnabled(bool on);
  bool mouseLookEnabled() const;
  // Mouse look is on and the pointer is captured: the view follows the mouse.
  bool mouseLooking() const;
  // Where the UI's windows are: rectangles {x0, y0, x1, y1, ...} in the window's pixels, origin
  // top-left (an overlay sets them every frame). In the browser a press on the canvas takes the
  // pointer lock in the press itself -- the browser may refuse a request made later from the
  // frame -- unless it lands in one of them. The page tests the press's own position, so a
  // press that comes before the frame has seen the pointer arrive over a window is still the
  // UI's.
  void setUiRegions(std::vector<float> rects);

  // Drop every currently-held movement key. Call when input focus leaves the
  // viewport (e.g. a UI overlay takes the keyboard): a key pressed before the
  // handoff and released after it would otherwise stay "held" forever and the
  // camera would drift on its own.
  void releaseHeldKeys();

  // Whether `keySym` (a VTK key sym: "w", "Up", ...; a single letter matches either
  // case) is held right now -- the same held set Fly navigation reads, so a host
  // driving something else from the keys (an avatar in Orbit mode) sees exactly what
  // the camera does, including the drop when a UI overlay takes the keyboard.
  bool keyHeld(const std::string &keySym) const;

  // Rebind a movement key (also settable via state "keys.<action>"). Actions:
  // "forward","backward","strafe_left","strafe_right","up","down","toggle_mode",
  // "sprint", "release_pointer" (Escape: lets a captured pointer go), and "home" (H:
  // resetView()). The arrow keys move as well as the bindings. Values are key names,
  // SDL's ("W","Space","Left Ctrl","Tab","Left Shift", the defaults) or VTK's keysyms
  // ("w","space","Control_L","Shift_L"): cvc::ariadne::canonical_key matches either spelling.
  void setKeyBinding(const std::string &action, const std::string &keySym);
  std::string keyBinding(const std::string &action) const;

  // Push the current pose to the vtkCamera (in Map, its parallel projection and
  // scale too) and reset the clipping range.
  // update() calls it only when something it depends on moved: the pose, the
  // camera (changed by someone else), the renderer's prop list, or -- with a
  // scene -- SceneGraph::contentVersion(). Without a scene it calls it every
  // frame. Call it yourself after changing what is visible in a way none of
  // those see (vertices pushed far out in place, a host's own prop shown with
  // SetVisibility, an unregistered node moved), or bump the scene's
  // markContentChanged().
  void applyToCamera();
  // The pose applyToCamera() applies; `up` is the camera's view-up: the world up
  // axis, or north (+y) in Map.
  void getPose(double eye[3], double focal[3], double up[3]) const;

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void seedState();                    // write defaults with the change signal suppressed
  void readAllFromState();             // pull every setting/pose from state into members
  void syncConfigToState();            // write config back (guarded against echo)
  void syncPoseToState();              // write the live pose back (guarded against echo)
  void recenterPointer();              // warp the pointer back to the centre (VTK's windows)
  bool pointerHeld() const; // the pointer is captured now (the browser has granted its lock)
  void applyPointerCapture();          // tell the window / the browser what pointerCapture() is
  void pollPointerLock();              // the browser's pointer-lock motion and release
  void resetTrack();                   // reset cinematic smoothing (ease in from current view)
  bool trackedWorldPos(double out[3]); // world pos of the tracked actor, or false
  double viewportAspect() const;       // renderer width/height, or 0 if unknown
  double mapFitScale(double halfHeight, double halfWidth) const; // parallel scale fitting a rect
  void fitMap(double cx, double cy, double halfHeight, double halfWidth); // set the map pose
  void refitMapIfResized();     // re-fit the map rect when the viewport changes shape
  bool applyNeeded() const;     // update(): has anything applyToCamera() derives from moved?
  bool poseMirrorStale() const; // does the pose differ from the one last mirrored to state?
  void poseMirrorValues(double out[17]) const; // what syncPoseToState() writes, in order

  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_CAMERA_CONTROLLER_H__
