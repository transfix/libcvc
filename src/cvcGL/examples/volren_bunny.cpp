// volren_bunny — the cvc::volren showcase: a signed-distance-field volume of
// the Stanford bunny raycast on the CPU and dropped into a live cvcGL scene
// through VolRenNode, next to the classic mesh bunny under the exact
// bunny_shadow StageLighting rig.
//
// What it demonstrates:
//  - cvc::sdf(bunny mesh) -> Float volume at startup (~0.4 s at 64^3);
//  - VolRenNode: async CPU raycast (isosurface at distance 0 + a faint
//    translucent shell), composited into the scene as a translucent quad
//    whose gl_FragDepth comes from the raycast depth map — the volume
//    occludes and is occluded per pixel (watch it pass behind the mesh
//    bunny as it orbits);
//  - the scene-graph transform feeding the raycaster: the volume bunny spins
//    via the node's standard rotation key, re-raycasting as it moves;
//  - FpsHud + per-raycast timing on stdout for the performance check.
//
// Controls: orbit camera (drag), 'f' toggles the FPS HUD.  CLI mirrors
// bunny_shadow: --width/--height/--offscreen/--frames/--png/--no-shadows/
// --no-ui plus --dim (SDF resolution), --scale (raycast resolution scale),
// --no-spin, --no-mesh.

#include <cvc/core/app.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/FpsHud.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/gl/VolRenNode.h>
#include <cvc/gl/state_publisher.h> // SceneGraph.h only forward-declares it
#include <cvc/utility/algorithm.h>
#include <cvc/volume/volume.h>

#ifdef CVC_ENABLE_IMGUI
#include <cvc/gl/ImGuiBinding.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <imgui.h>
#endif

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <boost/program_options.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

using cvc::gl::CameraController;
using cvc::gl::StageLighting;
using cvc::gl::VolRenNode;

namespace {

// Rotate the raw Stanford bunny +90 deg about X (Y-up -> Z-up), center in XY,
// base on z=0, tallest extent scaled to targetSize (bunny_shadow's helper).
cvc::geometry stand_bunny(double targetSize) {
  const cvc::geometry raw = cvc::read_geometry("stanford.bunny");
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
      rot(N[i][0], N[i][1], N[i][2], n);
      g.normals().push_back({n[0], n[1], n[2]});
    }
  }
  for (const auto &t : raw.tris())
    g.tris().push_back(t);
  return g;
}

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

// Signed-distance volume of a standing bunny: cubic bbox with 10% padding
// (the clean no-resample branch of SDF v2), dim^3 Float voxels, inside
// negative / surface zero / outside positive, distances in world units.
cvc::volume bunny_sdf(cvc::app &app, const cvc::geometry &standing, unsigned dim) {
  const cvc::bounding_box e = standing.extents();
  const double ext = std::max({e.maxx - e.minx, e.maxy - e.miny, e.maxz - e.minz});
  const double half = ext * 1.1 * 0.5;
  const double cx = 0.5 * (e.minx + e.maxx);
  const double cy = 0.5 * (e.miny + e.maxy);
  const double cz = 0.5 * (e.minz + e.maxz);
  const cvc::bounding_box bbox(cx - half, cy - half, cz - half, cx + half, cy + half, cz + half);
#ifdef CVC_ENABLE_SDF
  return cvc::sdf(app, standing, cvc::dimension(dim, dim, dim), bbox, cvc::SDF_V2);
#else
  // Fallback for builds without the SDF module (e.g. a wasm feature cut):
  // an analytic sphere SDF sized like the bunny, so the demo still runs.
  std::printf("[volren_bunny] CVC_ENABLE_SDF is off — using an analytic sphere SDF\n");
  cvc::volume vol(app, cvc::dimension(dim, dim, dim), cvc::Float, bbox);
  const double r = 0.35 * ext;
  for (unsigned k = 0; k < dim; ++k)
    for (unsigned j = 0; j < dim; ++j)
      for (unsigned i = 0; i < dim; ++i) {
        const double x = bbox.minx + i * vol.XSpan() - cx;
        const double y = bbox.miny + j * vol.YSpan() - cy;
        const double z = bbox.minz + k * vol.ZSpan() - cz;
        vol(i, j, k, std::sqrt(x * x + y * y + z * z) - r);
      }
  return vol;
#endif
}

// Raycast settings for the SDF bunny: solid surface at distance 0 shaded by
// the spline gradient, plus a faint cool shell 4 world units out so the
// translucent compositing path is visible over the scene.
cvc::volren::volume_settings bunny_volume_settings(double shell_offset) {
  cvc::volren::volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  cvc::volren::isosurface body;
  body.value = 0.0;
  body.opacity = 1.0f;
  body.color = {0.92f, 0.86f, 0.98f};
  body.shininess = 24.0f;
  vs.isosurfaces.push_back(body);
  cvc::volren::isosurface shell;
  shell.value = shell_offset;
  shell.opacity = 0.16f;
  shell.color = {0.35f, 0.75f, 0.95f};
  shell.shininess = 8.0f;
  vs.isosurfaces.push_back(shell);
  return vs;
}

} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  int width = 1280, height = 720, frames = 0, fps = 30;
  unsigned dim = 64;
  double scale = 0.5;
  std::string png;
  bool offscreen = false, no_shadows = false, no_ui = false, no_spin = false, no_mesh = false,
       no_grid = false;

  po::options_description desc("volren_bunny options");
  desc.add_options()("help,h", "show help")("width", po::value<int>(&width))(
      "height", po::value<int>(&height))("offscreen", po::bool_switch(&offscreen))(
      "frames", po::value<int>(&frames))("fps", po::value<int>(&fps))(
      "png", po::value<std::string>(&png))("no-shadows", po::bool_switch(&no_shadows))(
      "no-ui", po::bool_switch(&no_ui))("no-grid", po::bool_switch(&no_grid))(
      "no-spin", po::bool_switch(&no_spin))(
      "no-mesh", po::bool_switch(&no_mesh))("dim", po::value<unsigned>(&dim),
                                            "SDF volume resolution (default 64)")(
      "scale", po::value<double>(&scale), "raycast resolution scale (default 0.5)");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if (vm.count("help")) {
    std::printf("%s\n", "volren_bunny: SDF-bunny CPU raycast in a live cvcGL scene");
    return 0;
  }
  const bool capturing = offscreen || !png.empty();

  cvc::app app;
  app.properties("system.log_verbosity", "2");
  SceneGraph sg(app, "volren_bunny");

  const double S = 100.0; // bunny height in world units
  const cvc::geometry standing = stand_bunny(S);

  // The mesh bunny (classic renderer) to the left; the SDF volume bunny to
  // the right, spun through the scene graph so occlusion + the transform path
  // both show.
  if (!no_mesh) {
    auto mesh = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("bunny_mesh", standing));
    mesh->setPosition(-0.62 * S, 0.0, 0.0);
    mesh->setUseSingleColor(true);
    mesh->setColor(0.85, 0.82, 0.88);
    mesh->setAmbient(0.22);
    mesh->setDiffuse(0.9);
    mesh->setSpecular(0.25);
    mesh->setSpecularPower(24.0);
  }

  const double ground_rgb[3] = {0.32, 0.34, 0.38};
  auto groundNode =
      std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("ground", ground(2.2 * S, ground_rgb)));
  groundNode->setUseSingleColor(true);
  groundNode->setColor(ground_rgb[0], ground_rgb[1], ground_rgb[2]);
  groundNode->setAmbient(0.35);
  groundNode->setDiffuse(0.85);

  // --- The volume bunny -----------------------------------------------------
  std::printf("[volren_bunny] computing %ux%ux%u SDF...\n", dim, dim, dim);
  const auto sdf_t0 = std::chrono::steady_clock::now();
  cvc::volume sdf_vol = bunny_sdf(app, standing, dim);
  std::printf("[volren_bunny] SDF ready in %.2f s (range %.2f .. %.2f)\n",
              std::chrono::duration<double>(std::chrono::steady_clock::now() - sdf_t0).count(),
              sdf_vol.min(), sdf_vol.max());

  auto volNode = sg.getGraphicsRoot()->addGraphicsChild<VolRenNode>("bunny_volume");
  sg.registerGraphics("bunny_volume", volNode);
  volNode->setResolutionScale(scale);
  volNode->addVolume(sdf_vol, bunny_volume_settings(0.04 * S));
  volNode->setPosition(no_mesh ? 0.0 : 0.62 * S, 0.0, 0.0);
  {
    // Match the StageLighting key (azimuth -38 deg, elevation 52 deg) so the
    // raycast shading agrees with the scene look.
    cvc::volren::render_settings rs = volNode->renderConfig();
    const double az = -38.0 * M_PI / 180.0, el = 52.0 * M_PI / 180.0;
    cvc::volren::light key;
    key.color = {1.f, 1.f, 1.f};
    key.direction = {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
    rs.lights = {key};
    rs.ambient = 0.25f;
    rs.steps = 384;
    volNode->setRenderConfig(rs);
  }

  // --- Lighting: the bunny_shadow rig, verbatim -----------------------------
  StageLighting rig(sg);
  rig.setStage(0.0, 0.0, 0.5 * S, 0.62 * S);
  rig.applyPreset(StageLighting::Preset::ThreePoint);
  rig.setKey(1.9, -38.0, 52.0, 34.0);
  rig.setFill(0.85);
  rig.setBack(0.6);
  rig.setWarmth(0.3);
  rig.setEnvironment(0.7);
  rig.setAmbient(0.4);
  rig.apply();

  bool spin = !no_spin; // live-toggleable from the UI panel

  SceneRenderer view(sg, width, height, capturing || offscreen, "main");
  const bool shadows = !no_shadows && sg.setShadowsEnabled(true);
  if (shadows) {
    sg.setShadowResolution(2048);
    sg.setShadowUpdateInterval(1);
  }

  if (no_grid)
    sg.setDiagnosticChromeVisible(false);

  CameraController cam(view);
  cam.frameBounds(-2.2 * S, -2.2 * S, 0.0, 2.2 * S, 2.2 * S, 1.3 * S);
  cam.setMode(CameraController::Mode::Orbit);
  cvc::gl::TouchGestures touch(view, cam);

  cvc::gl::FpsHud hud(view);
  if (capturing)
    hud.setEnabled(false);

#ifdef CVC_ENABLE_IMGUI
  cvc::gl::ImGuiOverlay ui(view);
  ImGui::SetCurrentContext(ui.imguiContext());
  ui.attachCamera(cam);
  ui.setVisible(!no_ui && !capturing);
  bool uiScene = false, uiLighting = false, uiVolren = true;
  ui.setDrawCallback([&] {
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("Scene")) {
        cvc::gl::ui::SceneMenuItems(sg, &uiScene, &uiLighting);
        ImGui::Separator();
        ImGui::MenuItem("Volume raycast", nullptr, &uiVolren);
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }
    cvc::gl::ui::ScenePanel(sg, &uiScene);
    cvc::gl::ui::StageLightingPanel(rig, &uiLighting);

    // The raycast panel: resolution scale is the headline performance knob
    // (the quad upscales, so lowering it trades sharpness for latency).
    if (uiVolren) {
      if (ImGui::Begin("Volume raycast", &uiVolren)) {
        const double ms = volNode->lastRenderSeconds() * 1000.0;
        const int rw = std::max(2, int(std::lround(width * volNode->resolutionScale())));
        const int rh = std::max(2, int(std::lround(height * volNode->resolutionScale())));
        ImGui::Text("%d x %d rays  |  %.1f ms  |  %.2f Mray/s", rw, rh, ms,
                    ms > 0.0 ? (double(rw) * rh / (ms / 1000.0)) / 1e6 : 0.0);
        ImGui::Text("SDF %ux%ux%u  |  %llu raycasts", dim, dim, dim,
                    (unsigned long long)volNode->framesRendered());
        ImGui::Separator();
        float rs = float(volNode->resolutionScale());
        if (ImGui::SliderFloat("resolution", &rs, 0.05f, 1.0f, "%.2f"))
          volNode->setResolutionScale(rs);
        bool cont = volNode->continuous();
        if (ImGui::Checkbox("re-raycast every frame", &cont))
          volNode->setContinuous(cont);
        ImGui::Checkbox("spin", &spin);
        cvc::volren::render_settings rset = volNode->renderConfig();
        int steps = rset.steps;
        if (ImGui::SliderInt("steps", &steps, 32, 768)) {
          rset.steps = steps;
          volNode->setRenderConfig(rset);
        }
      }
      ImGui::End();
    }
  });
#endif

  int frame = 0;
  std::uint64_t last_report_frames = 0;
  auto last_report = std::chrono::steady_clock::now();
  auto last_t = last_report;

  while (!view.windowClosed()) {
    double dt;
    if (capturing) {
      dt = 1.0 / double(fps);
    } else {
      const auto now = std::chrono::steady_clock::now();
      dt = std::chrono::duration<double>(now - last_t).count();
      last_t = now;
    }

    view.processUIEvents();
    touch.update();
    cam.update(dt);

    // Spin the VOLUME bunny through the scene graph: the node picks the new
    // composed matrix up in tick() and re-raycasts.
    if (spin) {
      static double angle = 0.0;
      angle += 12.0 * dt; // deg/s
      volNode->setRotation(0.0, 0.0, angle);
    }

    volNode->tick();

    if (!png.empty() && frame == std::max(0, frames - 1)) {
      view.writePNG(png);
      break;
    }
    view.render();

    // Perf report roughly once a second.
    const auto now = std::chrono::steady_clock::now();
    const double since = std::chrono::duration<double>(now - last_report).count();
    if (since >= 1.0) {
      const std::uint64_t rendered = volNode->framesRendered();
      const double ms = volNode->lastRenderSeconds() * 1000.0;
      const int rw = std::max(2, int(std::lround(width * scale)));
      const int rh = std::max(2, int(std::lround(height * scale)));
      std::printf("[volren_bunny] scene %.1f fps | raycast %.1f ms/frame (%.2f Mray/s, "
                  "%dx%d) | %llu raycasts (+%llu)\n",
                  hud.fps(), ms, ms > 0.0 ? (double(rw) * rh / (ms / 1000.0)) / 1e6 : 0.0, rw,
                  rh, (unsigned long long)rendered,
                  (unsigned long long)(rendered - last_report_frames));
      last_report_frames = rendered;
      last_report = now;
    }

#ifdef __EMSCRIPTEN__
#ifndef __EMSCRIPTEN_PTHREADS__
    sg.publisher().flush();
#endif
    emscripten_sleep(0);
#else
    if (frames > 0 && ++frame >= frames)
      break;
#endif
#ifdef __EMSCRIPTEN__
    ++frame;
#endif
  }

  hud.detach();
  cam.detach();
  return 0;
}
