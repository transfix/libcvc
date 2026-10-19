#pragma once

// TerminalScene — cvcGL's extension to the terminal backends: show an Ariadne document's 3D scene
// in a terminal (roadmap §16.6, the "pixel mirror"). The base terminal support stays GL-free in
// libcvc (cvc::ariadne::FtxuiBackend, TvisionBackend); this is the cvcGL piece that plugs a real
// renderer into it, the way ImGuiBackend + the GL scene adapter plug into the neutral core.
//
// It is a cvc::ariadne::SceneAdapter. Hand it to AppRuntime, and a document's `scene:` is realized
// into its own SceneGraph and drawn by an OFFSCREEN SceneRenderer (no window, no display: VTK picks
// X, EGL or OSMesa at run time). Every tick renders a frame at the terminal's half-block pixel grid
// (two pixels per cell, supersampled), publishes it to the backend as an image, and makes it the
// backend's background (FtxuiBackend::set_background), so the widgets, menus and windows draw over
// the scene exactly as ImGui draws over the GL canvas.
//
// Input: pass handle_input as FtxuiBackend::RunOptions::background_input. A drag on the background
// orbits the camera, the wheel zooms, and anything else falls through to the document's on_key /
// on_pointer.

#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/scene_adapter.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace gl {
class SceneGraph;
class SceneRenderer;
class CameraController;
namespace ariadne {

class TerminalScene : public cvc::ariadne::SceneAdapter {
public:
  // The name the rendered frame is published under (FtxuiBackend::set_image).
  static constexpr const char *kImage = "cvcgl.terminal_scene";

  // Render for `backend` (it must outlive this). `supersample` renders that many pixels per
  // half-block pixel each way and lets the backend's box filter smooth them.
  TerminalScene(cvc::app &app, cvc::ariadne::FtxuiBackend &backend, int supersample = 2,
                const std::string &state_prefix = "terminal_scene");
  ~TerminalScene() override;
  TerminalScene(const TerminalScene &) = delete;
  TerminalScene &operator=(const TerminalScene &) = delete;

  // SceneAdapter: realize the parsed scene (and frame the camera to it), mirror bound visibility,
  // and per frame service the scene, render it and publish the frame.
  void realize(const cvc::ariadne::Scene &scene, const std::string &prefix,
               std::vector<std::string> *warns) override;
  void sync_visibility(cvc::app &app) override;
  void tick() override;

  // Camera input from the terminal (see the header): true when it took the event.
  bool handle_input(const cvc::ariadne::TermEvent &e);

  // Write the last rendered frame as a PNG (a capture of what the terminal shows, at full res).
  void write_png(const std::string &path);

  SceneGraph &scene();
  SceneRenderer &view();
  CameraController &camera();

private:
  struct Impl;
  std::unique_ptr<Impl> m_;
};

} // namespace ariadne
} // namespace gl
} // namespace cvc
