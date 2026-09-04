/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_FPS_HUD_H__
#define __CVC_GL_FPS_HUD_H__

#include <cvc/core/state_object.h>
#include <memory>
#include <string>

class vtkObject;                 // VTK (global namespace)
class vtkRenderer;               // VTK (global namespace)
class vtkRenderWindow;           // VTK (global namespace)
class vtkRenderWindowInteractor; // VTK (global namespace)

namespace cvc {
class app;
namespace gl {
class SceneRenderer;

// -------
// FpsHud
// -------
// A small on-screen frames-per-second readout, drawn by VTK itself (a
// vtkTextActor 2-D overlay) so it appears wherever the scene renders: the
// native window, an offscreen capture, or the WebAssembly canvas.
//
// EVERYTHING IS cvc::state, like CameraController: visibility, the toggle key,
// the refresh rate, placement and font size live under
// "<scene prefix>.viewers.<viewer name>.hud" and are two-way bound — write
// "...hud.enabled" from a script and the overlay follows; press the toggle key
// and the state reflects it. The measured rate is mirrored to "...hud.fps" at
// the (throttled) refresh cadence for dashboards.
//
// Frames are measured with a vtkCommand::EndEvent observer on the render
// window — every vtkRenderWindow::Render() marks a frame — so no host-loop
// changes are needed and the same code measures native and wasm builds.
//
// The toggle key ('f' by default, state "keys.toggle") is watched with a
// vtkCallbackCommand observer on the interactor at priority 1.0. That
// deliberately does NOT install a vtkInteractorStyle: the interactor holds
// only one style slot and CameraController owns it; an observer coexists with
// any style regardless of construction order.
class FpsHud : public cvc::state_object<FpsHud> {
public:
  // Canonical: state at "<viewer.scene prefix>.viewers.<viewer.name>.hud",
  // rooted in the viewer's scene app, overlay added to the viewer's renderer,
  // frame/key observers attached to its window/interactor (offscreen viewers
  // have no interactor — the HUD still measures and draws, just untogglable).
  explicit FpsHud(SceneRenderer &viewer);

  // Low-level: explicit app context + full state path (headless / custom).
  FpsHud(cvc::app &ctx, const std::string &statePath);

  ~FpsHud();
  FpsHud(const FpsHud &) = delete;
  FpsHud &operator=(const FpsHud &) = delete;

  // The canonical state path for a viewer's HUD.
  static std::string viewerStatePath(const std::string &scenePrefix, const std::string &viewerName);

  // Wiring (the SceneRenderer ctor does all of this for you).
  void setRenderer(vtkRenderer *renderer);
  void attach(vtkRenderWindow *window, vtkRenderWindowInteractor *interactor);
  void detach();

  // Visibility (state "enabled"). The toggle key flips this.
  void setEnabled(bool on);
  bool enabled() const;

  // Toggle key as a VTK key sym (state "keys.toggle", default "f") — same
  // rebindable convention as CameraController's "keys.*".
  void setToggleKey(const std::string &keySym);
  std::string toggleKey() const;

  // Readout refresh rate in Hz (state "update_hz", default 2). Also throttles
  // the state mirror — per-frame state writes are a known perf sink.
  void setUpdateHz(double hz);

  // Latest smoothed measurement (also mirrored to state "fps").
  double fps() const;

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void seedState();         // write defaults with the change signal suppressed
  void readAllFromState();  // pull every setting from state into members
  void syncConfigToState(); // write config back (guarded against echo)
  void apply();             // push members to the text actor
  void frameRendered();     // EndEvent: accumulate timing, refresh the readout

  static void onRenderEnd(vtkObject *caller, unsigned long eventId, void *clientData,
                          void *callData);
  static void onKeyPress(vtkObject *caller, unsigned long eventId, void *clientData,
                         void *callData);

  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_FPS_HUD_H__
