// bunny_shadow — a Stanford-bunny shader & shadow-mapping test bench in cvcGL.
//
// The smallest scene that exercises the renderer honestly: ONE known-good mesh
// (the Stanford bunny, ~69k tris) on a ground plane, lit by a StageLighting rig
// with shadows on. It exists to answer "why do the shadows look wrong?" — a
// single opaque caster on a single receiver, where a shadow that lands in the
// wrong place, self-shadows (acne), or floats off the contact (peter-panning) is
// obvious, and the StageLighting panel's gizmos + per-light solo let you attribute
// it. The rig frames a TIGHT cone on the bunny, which is the whole trick to a
// crisp shadow map (see cvc/gl/StageLighting.h): a directional light bakes its
// map over the entire scene bbox and wastes texels, an aimed spot spends them on
// the subject.
//
// Run (navigable):            bunny_shadow
//   Tab orbits/flies; drag to look; the Scene menu toggles shadows + the lighting
//   panel (turn on gizmos to SEE each light's shadow cone).
// Verify (offscreen):         bunny_shadow --offscreen --frames 1 --png bunny.png

#define _USE_MATH_DEFINES
#include <algorithm>
#include <array>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GridNode.h>
#include <cvc/gl/ImGuiBinding.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/gl/TouchGestures.h>
#include <functional>
#include <iostream>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif
#ifdef __EMSCRIPTEN__
#include <cvc/gl/state_publisher.h>
#include <emscripten.h>
#endif

using cvc::gl::CameraController;
using cvc::gl::StageLighting;
using cvc::gl::TouchGestures;

namespace {

// Load the bunny, orient it Z-up (the baked mesh is the canonical Y-up Stanford
// bunny — rotate +90 deg about X: (x,y,z) -> (x,-z,y), a proper rotation so face
// winding and normals stay valid), centre it in XY, sit its base on z=0, and
// scale its tallest extent to `targetSize`. One clean opaque mesh to shadow.
cvc::geometry stand_bunny(double targetSize) {
  const cvc::geometry raw = cvc::read_geometry("stanford.bunny"); // bunny_io: filename-independent
  const auto &P = raw.points();
  const auto &N = raw.normals();
  auto rot = [](double x, double y, double z, double o[3]) {
    o[0] = x;
    o[1] = -z;
    o[2] = y;
  };
  double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
  for (const auto &p : P) {
    double w[3];
    rot(p[0], p[1], p[2], w);
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], w[k]);
      hi[k] = std::max(hi[k], w[k]);
    }
  }
  const double ext = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});
  const double s = ext > 0 ? targetSize / ext : 1.0;
  const double cx = 0.5 * (lo[0] + hi[0]), cy = 0.5 * (lo[1] + hi[1]);
  cvc::geometry g;
  for (std::size_t i = 0; i < P.size(); ++i) {
    double w[3];
    rot(P[i][0], P[i][1], P[i][2], w);
    g.points().push_back({(w[0] - cx) * s, (w[1] - cy) * s, (w[2] - lo[2]) * s});
    if (i < N.size()) {
      double n[3];
      rot(N[i][0], N[i][1], N[i][2], n); // rotation preserves unit length
      g.normals().push_back({n[0], n[1], n[2]});
    }
  }
  for (const auto &t : raw.tris())
    g.tris().push_back(t);
  return g;
}

// A ground quad with UP normals (so it RECEIVES shadows and is lit), spanning
// [-h,h]^2 at z=0.
cvc::geometry ground(double h, const double rgb[3]) {
  cvc::geometry g;
  const double v[4][3] = {{-h, -h, 0}, {h, -h, 0}, {h, h, 0}, {-h, h, 0}};
  for (const auto &p : v) {
    g.points().push_back({p[0], p[1], p[2]});
    g.normals().push_back({0, 0, 1});
    g.colors().push_back({rgb[0], rgb[1], rgb[2]});
  }
  g.tris().push_back({0, 1, 2});
  g.tris().push_back({0, 2, 3});
  return g;
}

} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  int width = 1280, height = 720;
  long frames = 0;
  double fps = 30.0;
  bool offscreen = false, no_shadows = false, no_ui = false, no_grid = false;
  std::string png;

  po::options_description desc("bunny_shadow — a Stanford-bunny shadow/shader test in cvcGL");
  desc.add_options()("help,h", "show this help")("width",
                                                 po::value<int>(&width)->default_value(1280))(
      "height", po::value<int>(&height)->default_value(720))("offscreen",
                                                             po::bool_switch(&offscreen))(
      "no-shadows", po::bool_switch(&no_shadows))("no-ui", po::bool_switch(&no_ui))(
      "no-grid", po::bool_switch(&no_grid))("frames", po::value<long>(&frames)->default_value(0))(
      "fps", po::value<double>(&fps)->default_value(30.0))("png", po::value<std::string>(&png));
  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "bunny_shadow: %s\n", e.what());
    return 2;
  }
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 0;
  }
  const bool capturing = offscreen || !png.empty();

  cvc::app app; // ctor registers bunny_io, so read_geometry(".bunny") works
  app.properties("system.log_verbosity", "2");
  SceneGraph sg(app, "bunny");

  const double S = 100.0; // bunny target height in world units

  // The subject: the bunny.
  auto bunnyNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("bunny", stand_bunny(S)));
  if (bunnyNode) {
    bunnyNode->setUseSingleColor(true);
    bunnyNode->setColor(0.85, 0.82, 0.88);
    bunnyNode->setAmbient(0.22);
    bunnyNode->setDiffuse(0.9);
    bunnyNode->setSpecular(0.25);
    bunnyNode->setSpecularPower(24.0);
  }

  // The receiver: a ground plane the shadow lands on.
  const double ground_rgb[3] = {0.32, 0.34, 0.38};
  auto groundNode = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("ground", ground(2.2 * S, ground_rgb)));
  if (groundNode) {
    groundNode->setUseSingleColor(true);
    groundNode->setColor(ground_rgb[0], ground_rgb[1], ground_rgb[2]);
    groundNode->setAmbient(0.35);
    groundNode->setDiffuse(0.85);
  }

  // A rig framed TIGHT on the bunny: the aimed spot's shadow-map frustum lands on
  // the subject, which is what makes the shadow crisp (StageLighting.h). Radius =
  // the bunny's own extent, NOT the whole scene.
  StageLighting rig(sg);
  rig.setStage(0.0, 0.0, 0.5 * S, 0.62 * S);
  rig.applyPreset(StageLighting::Preset::ThreePoint);
  rig.setKey(1.9, -38.0, 52.0, 34.0); // intensity, azimuth, elevation, cone(deg)
  rig.setFill(0.85);
  rig.setBack(0.6);
  rig.setWarmth(0.3);
  rig.setEnvironment(0.7); // lift the ground/edges the cones miss
  rig.setAmbient(0.4);     // shadowed sides readable, not pitch black
  rig.apply();

  SceneRenderer view(sg, width, height, offscreen, "main");
  const bool shadows = !no_shadows && sg.setShadowsEnabled(true);
  if (shadows) {
    sg.setShadowResolution(2048);
    sg.setShadowUpdateInterval(1); // static scene: bake every frame, no lag
  }

  // --no-grid strips the diagnostic chrome: the bounds grid, the origin axis
  // marker, and the per-node bounding boxes with their corner coordinate
  // readouts. All of it earns its keep while you are asking "why does this
  // shadow look wrong?" and none of it belongs in a captured still — the axis
  // marker in particular lands squarely on the bunny's chest from the default
  // orbit. Presentation only; the scene and lighting are untouched.
  if (no_grid) {
    sg.setGridVisible(false);
    sg.setAxisVisible(false);
    std::function<void(const std::shared_ptr<GraphicsNode> &)> hide_bbox =
        [&](const std::shared_ptr<GraphicsNode> &n) {
          if (!n)
            return;
          n->setShowBBox(false);
          n->setShowExtentLabels(false);
          for (const auto &c : n->getGraphicsChildren())
            hide_bbox(std::dynamic_pointer_cast<GraphicsNode>(c));
        };
    hide_bbox(sg.getGraphicsRoot());
  }

  CameraController cam(view);
  cam.frameBounds(-2.2 * S, -2.2 * S, 0.0, 2.2 * S, 2.2 * S, 1.3 * S);
  cam.setMode(CameraController::Mode::Orbit);
  TouchGestures touch(view, cam); // declared unconditionally: the loop uses it every frame

#ifdef CVC_ENABLE_IMGUI
  bool uiShadows = shadows, uiLighting = false;
  cvc::gl::ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  ui.setVisible(!no_ui && !capturing);
  ui.setDrawCallback([&] {
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("Scene")) {
        if (ImGui::MenuItem("Shadows", nullptr, &uiShadows))
          sg.setShadowsEnabled(uiShadows);
        ImGui::MenuItem("Stage lighting", nullptr, &uiLighting);
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }
    cvc::gl::ui::StageLightingPanel(rig, &uiLighting);
  });
#else
  (void)no_ui;
#endif

  const auto t0 = std::chrono::steady_clock::now();
  double last = 0.0;
  long frame = 0;
  while (!view.windowClosed()) {
    double t, dt;
    if (capturing) {
      t = frame / fps;
      dt = 1.0 / fps;
    } else {
      t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      dt = t - last;
      last = t;
    }
    view.processUIEvents();
    touch.update();
    cam.update(dt);

    if (capturing) {
      char path[1024];
      if (!png.empty() && frames <= 1)
        std::snprintf(path, sizeof path, "%s", png.c_str());
      else
        std::snprintf(path, sizeof path, "bunny_%05ld.png", frame);
      view.writePNG(path);
    } else {
      view.render();
    }
#ifdef __EMSCRIPTEN__
#ifndef __EMSCRIPTEN_PTHREADS__
    sg.publisher().flush();
#endif
    emscripten_sleep(0);
#endif
    ++frame;
#ifndef __EMSCRIPTEN__
    if (frames > 0 && frame >= frames)
      break;
#endif
  }
  cam.detach();
  std::printf("bunny_shadow: done (%ld frames)\n", frame);
  return 0;
}
