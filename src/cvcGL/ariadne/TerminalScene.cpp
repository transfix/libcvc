// TerminalScene — see inc/cvc/gl/ariadne/TerminalScene.h. An offscreen SceneRenderer whose frames
// become the terminal backend's background image.

#include <algorithm>
#include <chrono>
#include <cvc/ariadne/bind.h> // sync_scene_visibility
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/TerminalScene.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/image/image.h>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

struct TerminalScene::Impl {
  ari::FtxuiBackend &backend;
  int supersample;
  SceneGraph scene;
  SceneRenderer view;
  CameraController camera;
  RealizedScene realized;
  bool have_scene = false;
  std::chrono::steady_clock::time_point last_tick{};
  // A drag in progress: which button, and where the pointer was (cells).
  int drag_button = 0;
  int last_x = 0, last_y = 0;

  Impl(cvc::app &app, ari::FtxuiBackend &be, int ss, const std::string &prefix)
      : backend(be), supersample(std::max(1, ss)), scene(app, prefix),
        view(scene, 160, 96, /*offscreen=*/true, "terminal"), camera(view) {
    view.setBackground(0.09, 0.10, 0.12);
  }
};

TerminalScene::TerminalScene(cvc::app &app, ari::FtxuiBackend &backend, int supersample,
                             const std::string &state_prefix)
    : m_(new Impl(app, backend, supersample, state_prefix)) {}

TerminalScene::~TerminalScene() {
  if (m_->have_scene)
    m_->backend.set_background(""); // the image it names is about to stop updating
}

void TerminalScene::realize(const ari::Scene &scene, const std::string &prefix,
                            std::vector<std::string> *warns) {
  m_->realized = realize_scene(m_->scene, scene, prefix, warns);
  m_->have_scene = true;
  const cvc::bounding_box bb = m_->scene.computeGraphicsBounds();
  if (!bb.isNull())
    m_->camera.frameBounds(bb.minx, bb.miny, bb.minz, bb.maxx, bb.maxy, bb.maxz);
  m_->backend.set_background(kImage);
}

void TerminalScene::sync_visibility(cvc::app &app) {
  if (m_->have_scene)
    ari::sync_scene_visibility(app, m_->realized.visibility);
}

void TerminalScene::tick() {
  if (!m_->have_scene)
    return;
  const auto now = std::chrono::steady_clock::now();
  const double dt = m_->last_tick == std::chrono::steady_clock::time_point{}
                        ? 0.0
                        : std::chrono::duration<double>(now - m_->last_tick).count();
  m_->last_tick = now;
  tick_scene(m_->realized, m_->view.renderer(), dt);
  m_->camera.update(dt);
  // The terminal's half-block grid: a cell is one pixel wide and two tall (square pixels on a 1:2
  // cell), times the supersample factor the backend's box filter folds back down.
  const int ss = m_->supersample;
  const int w = std::max(2, m_->backend.screen_width() * ss);
  const int h = std::max(2, m_->backend.screen_height() * 2 * ss);
  if (m_->view.frameWidth() != w || m_->view.frameHeight() != h)
    m_->view.resize(w, h);
  std::vector<unsigned char> rgb = m_->view.frameRGB(); // renders; rows bottom-up
  const int fw = m_->view.frameWidth(), fh = m_->view.frameHeight();
  if (rgb.size() < static_cast<std::size_t>(fw) * fh * 3)
    return;
  const cvc::image frame(fw, fh, cvc::image::pixel_format::RGB, cvc::image::data_type::u8,
                         rgb.data());
  m_->backend.set_image(kImage, frame.flipped_vertical());
}

bool TerminalScene::handle_input(const ari::TermEvent &e) {
  if (!m_->have_scene || e.kind != ari::TermEvent::Kind::Mouse)
    return false;
  constexpr int kCellPxW = 8, kCellPxH = 16; // the backend's nominal cell, for mouseLook's pixels
  switch (e.mouse) {
  case ari::TermEvent::Mouse::Press:
    if (e.button != 1 && e.button != 2)
      return false;
    m_->drag_button = e.button;
    m_->last_x = e.x;
    m_->last_y = e.y;
    if (e.button == 2)
      m_->camera.beginPan(); // middle-drag pans, as in every other cvcGL viewer
    m_->camera.beginDrag();
    return true;
  case ari::TermEvent::Mouse::Move:
    if (!m_->drag_button)
      return false;
    m_->camera.mouseLook((e.x - m_->last_x) * kCellPxW, (e.y - m_->last_y) * kCellPxH);
    m_->last_x = e.x;
    m_->last_y = e.y;
    return true;
  case ari::TermEvent::Mouse::Release:
    if (!m_->drag_button)
      return false;
    m_->camera.endDrag();
    if (m_->drag_button == 2)
      m_->camera.endPan();
    m_->drag_button = 0;
    return true;
  case ari::TermEvent::Mouse::WheelUp:
  case ari::TermEvent::Mouse::WheelDown:
    m_->camera.mouseWheel(e.mouse == ari::TermEvent::Mouse::WheelUp ? 1.0 : -1.0);
    return true;
  }
  return false;
}

void TerminalScene::write_png(const std::string &path) { m_->view.writePNG(path); }

SceneGraph &TerminalScene::scene() { return m_->scene; }
SceneRenderer &TerminalScene::view() { return m_->view; }
CameraController &TerminalScene::camera() { return m_->camera; }

} // namespace ariadne
} // namespace gl
} // namespace cvc
