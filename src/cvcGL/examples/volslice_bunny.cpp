// volslice_bunny — the Stanford bunny as a density volume, rendered by
// cvc::volslice: VolumeRover2's view-aligned slice compositor, live in a
// cvcGL scene.
//
// The volume is the same SDF volren_bunny raycasts, but consumed the way the
// legacy renderer consumed data: normalized through a value WINDOW into a
// byte texture and colored by a 256-entry transfer function.  The window is
// a narrow band around the zero level set, so the TF paints a glowing
// translucent shell-and-interior instead of an isosurface — the look slice
// rendering is good at.
//
// The panel exposes exactly the legacy tunables: quality (the VolumeRover2
// slider; slice count N = 2*(10 + max_planes*q^3)), near-plane peel,
// linear/nearest filtering, the value window — plus the port's one opt-in
// deviation, spacing-corrected opacity (see cvc/volslice/types.h).
//
// Up to NINE bunnies (volren_bunny's grid), each its OWN VolSliceNode -- the
// one-volume-per-node model composing in the scene graph.  Multi-node scenes
// need VolSliceNode::depthSortSliceProps() each frame: with UseOIT off, VTK
// renders translucent props in ADD order, so without the sort a bunny added
// later paints over a nearer one (measured; regression-tested).
//
// KNOWN LIMITS, by design of the pass it renders in: the slice bunny casts
// no shadow (the shadow baker consumes opaque geometry only), and adding the
// node flips its renderer to sequential translucency (UseOITOff — see
// VolSliceNode.h; slice compositing is order-dependent).
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/FpsHud.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/gl/VolSliceNode.h>
#include <cvc/gl/state_publisher.h> // SceneGraph.h only forward-declares it
#include <cvc/utility/algorithm.h>
#include <cvc/volume/volume.h>

#ifdef CVC_ENABLE_IMGUI
#include <cvc/gl/ImGuiOverlay.h>
#include <imgui.h>
#endif

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <algorithm>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

using cvc::gl::CameraController;
using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::StageLighting;
using cvc::gl::VolSliceNode;

namespace {

// volren_bunny's helpers, unchanged: Y-up -> Z-up, feet on z=0, height S.
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
  std::printf("[volslice_bunny] CVC_ENABLE_SDF is off — using an analytic sphere SDF\n");
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

// Where the Nth bunny stands: slot 0 at the origin, the rest ringing outward
// on a 3x3 grid whose pitch clears the bunny's footprint (volren_bunny's
// layout, so the two demos frame identically).
constexpr int kMaxBunnies = 9;
constexpr int kGrid[kMaxBunnies][2] = {{0, 0}, {1, 0},  {0, 1},   {-1, 0}, {0, -1},
                                       {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

// The TF over the RAW SDF domain for a window of half-width `w` (world
// units): amber interior fading through a warm shell to transparent just
// outside the surface.  Authored at the default quality; the panel's
// opacity-correction checkbox keeps this density when quality moves.
cvc::volslice::render_settings bunny_settings(double w) {
  cvc::volslice::render_settings s;
  s.window_min = -w;
  s.window_max = w;
  s.tf_auto_domain = false;
  s.tf.add({-w, 1.00f, 0.55f, 0.18f, 0.92f});        // deep interior: amber
  s.tf.add({-0.25 * w, 0.98f, 0.72f, 0.35f, 0.55f}); // inner shell
  s.tf.add({0.04 * w, 0.95f, 0.85f, 0.55f, 0.10f});  // glow right at the surface
  s.tf.add({0.18 * w, 0.f, 0.f, 0.f, 0.f});          // outside: air
  return s;
}

} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  int width = 1280, height = 720, frames = 0, bunnies = 1;
  unsigned dim = 64;
  double quality = cvc::volslice::defaults::quality;
  std::string png;
  bool offscreen = false, no_ui = false, no_grid = false, no_shadows = false;

  po::options_description desc("volslice_bunny options");
  desc.add_options()("help,h", "show help")("width", po::value<int>(&width))(
      "height", po::value<int>(&height))("offscreen", po::bool_switch(&offscreen))(
      "frames", po::value<int>(&frames))("png", po::value<std::string>(&png))(
      "dim", po::value<unsigned>(&dim), "SDF volume resolution (default 64)")(
      "quality", po::value<double>(&quality), "slice density 0..1 (default 0.5)")(
      "no-ui", po::bool_switch(&no_ui))("no-grid", po::bool_switch(&no_grid))(
      "no-shadows", po::bool_switch(&no_shadows))("bunnies", po::value<int>(&bunnies),
                                                  "initial bunny count 1..9 (default 1)");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if (vm.count("help")) {
    std::printf("volslice_bunny: the Stanford bunny as a density volume, rendered by\n"
                "cvc::volslice — VolumeRover2's view-aligned slice compositor.\n\n");
    std::cout << desc << std::endl;
    return 0;
  }
  const bool capturing = offscreen || !png.empty();

  cvc::app app;
  app.properties("system.log_verbosity", "2");
  SceneGraph sg(app, "volslice_bunny");

  const double S = 100.0; // bunny height in world units
  const cvc::geometry standing = stand_bunny(S);

  const double ground_rgb[3] = {0.32, 0.34, 0.38};
  // Sized for the FULL 3x3 grid (bunnies can be added at runtime), not just
  // the one on stage at startup (volren_bunny's formula).
  const double ground_half = std::max(2.2 * S, 1.35 * S * std::sqrt(2.0) + 1.2 * S);
  auto groundNode = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("ground", ground(ground_half, ground_rgb)));
  groundNode->setUseSingleColor(true);
  groundNode->setColor(ground_rgb[0], ground_rgb[1], ground_rgb[2]);
  groundNode->setAmbient(0.35);
  groundNode->setDiffuse(0.85);

  std::printf("[volslice_bunny] computing %ux%ux%u SDF...\n", dim, dim, dim);
  const auto t0 = std::chrono::steady_clock::now();
  cvc::volume sdf_vol = bunny_sdf(app, standing, dim);
  std::printf("[volslice_bunny] SDF ready in %.2f s (range %.2f .. %.2f)\n",
              std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(),
              sdf_vol.min(), sdf_vol.max());

  // One VolSliceNode per grid slot, all sharing the SAME volume (shallow
  // copies; each node owns its 3D texture -- 9 x 256KB at 64^3, negligible).
  // Pre-created and visibility-gated rather than added/removed live, so the
  // panel buttons are cheap and teardown-order questions never arise.
  const double pitch = 1.35 * S;
  double windowHalf = 0.06 * S;
  int bunnyCount = std::min(std::max(bunnies, 1), kMaxBunnies);
  std::vector<std::shared_ptr<VolSliceNode>> volNodes;
  for (int i = 0; i < kMaxBunnies; ++i) {
    const std::string name = "bunny_volume_" + std::to_string(i);
    auto n = sg.getGraphicsRoot()->addGraphicsChild<VolSliceNode>(name);
    sg.registerGraphics(name, n);
    n->setVolume(sdf_vol);
    n->setPosition(kGrid[i][0] * pitch, kGrid[i][1] * pitch, 0.0);
    n->setVisible(i < bunnyCount);
    volNodes.push_back(n);
  }
  auto applyConfigToAll = [&](const cvc::volslice::render_settings &s) {
    for (auto &n : volNodes)
      n->setConfig(s);
  };
  {
    cvc::volslice::render_settings s = bunny_settings(windowHalf);
    s.slices.quality = std::min(std::max(quality, 0.0), 1.0);
    applyConfigToAll(s);
  }
  VolSliceNode *volNode = volNodes[0].get(); // slot 0: the panel's readout node

  // The bunny_shadow rig for the ground and chrome; the slices are unlit by
  // design (the legacy unshaded path — shading is phase-2 work).
  StageLighting rig(sg);
  rig.setStage(0.0, 0.0, 0.5 * S, 0.62 * S);
  rig.applyPreset(StageLighting::Preset::ThreePoint);
  rig.setKey(1.9, -38.0, 52.0, 34.0);
  rig.setFill(0.85);
  rig.setBack(0.6);
  rig.setAmbient(0.4);
  rig.apply();

  SceneRenderer view(sg, width, height, capturing, "main");
  const bool shadows = !no_shadows && sg.setShadowsEnabled(true);
  if (shadows) {
    sg.setShadowResolution(2048);
    sg.setShadowUpdateInterval(1);
  }
  if (no_grid)
    sg.setDiagnosticChromeVisible(false);

  CameraController cam(view);
  auto frameAll = [&]() {
    double ring = 0.0;
    for (int i = 0; i < bunnyCount; ++i)
      ring = std::max(
          ring, std::sqrt(double(kGrid[i][0] * kGrid[i][0] + kGrid[i][1] * kGrid[i][1])) * pitch);
    const double r = ring + 2.2 * S;
    cam.frameBounds(-r, -r, 0.0, r, r, 1.3 * S);
  };
  frameAll();
  cam.setMode(CameraController::Mode::Orbit);
  cvc::gl::TouchGestures touch(view, cam);

  cvc::gl::FpsHud hud(view);
  if (capturing)
    hud.setEnabled(false);

  // Panel intents, applied in the loop (the ImGui callback runs mid-render).
  double wantQuality = quality, wantNear = 0.0, wantWindow = windowHalf;
  bool wantCorrection = false, wantNearest = false, wantFrameAll = false;
  int wantCount = bunnyCount;

#ifdef CVC_ENABLE_IMGUI
  cvc::gl::ImGuiOverlay ui(view);
  ImGui::SetCurrentContext(ui.imguiContext());
  ui.attachCamera(cam);
  ui.setDrawCallback([&]() {
    if (no_ui && !capturing)
      return;
    ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(330, 0), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("volslice — VolumeRover2 slice renderer")) {
      ImGui::Text("bunnies: %d / %d", bunnyCount, kMaxBunnies);
      ImGui::SameLine();
      if (ImGui::Button("Add") && wantCount < kMaxBunnies)
        wantCount = bunnyCount + 1;
      ImGui::SameLine();
      if (ImGui::Button("Remove") && wantCount > 1)
        wantCount = bunnyCount - 1;
      ImGui::SameLine();
      if (ImGui::Button("Frame all"))
        wantFrameAll = true;
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Each bunny is its OWN VolSliceNode -- the scene graph composes\n"
                          "them, depth-sorted per frame (VTK renders translucent props in\n"
                          "add order; see VolSliceNode::depthSortSliceProps).");
      ImGui::Separator();
      float q = float(wantQuality);
      if (ImGui::SliderFloat("quality", &q, 0.f, 1.f, "%.2f"))
        wantQuality = q;
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The VolumeRover2 slider: slice count N = 2*(10 + 1000*q^3).\n"
                          "q=0 -> 20 planes, q=0.5 -> 270, q=1 -> 2020.");
      float np = float(wantNear);
      if (ImGui::SliderFloat("near plane", &np, 0.f, 0.95f, "%.2f"))
        wantNear = np;
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Peels the viewer-side fraction of the sweep — the legacy\n"
                          "setNearPlane(), VolumeRover2's cut-in slider.");
      float wh = float(wantWindow / S);
      if (ImGui::SliderFloat("window half-width", &wh, 0.01f, 0.20f, "%.3f of height"))
        wantWindow = double(wh) * S;
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The value window normalized into the byte texture (the legacy\n"
                          "UChar coercion). Narrow = a crisp shell, wide = a soft cloud.");
      bool oc = wantCorrection;
      if (ImGui::Checkbox("opacity correction", &oc))
        wantCorrection = oc;
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The port's one opt-in deviation: scale each slice's alpha for\n"
                          "the actual plane spacing, so quality changes sharpness instead\n"
                          "of density. OFF is the faithful legacy behavior.");
      bool nn = wantNearest;
      if (ImGui::Checkbox("nearest-neighbor filter", &nn))
        wantNearest = nn;
      ImGui::Separator();
      std::size_t totalPlanes = 0;
      for (int i = 0; i < bunnyCount; ++i)
        totalPlanes += volNodes[i]->planesRendered();
      ImGui::Text("planes: %zu (%d node%s)", totalPlanes, bunnyCount, bunnyCount == 1 ? "" : "s");
      ImGui::Text("scene: %.1f fps", hud.fps());
    }
    ImGui::End();
  });
#endif

  int frame = 0, ticks = 0;
  constexpr int kCaptureTickLimit = 600;
  auto last_report = std::chrono::steady_clock::now();
  auto last_t = last_report;

  while (!view.windowClosed()) {
    ++ticks;
    double dt;
    if (capturing) {
      dt = 1.0 / 30.0;
    } else {
      const auto now = std::chrono::steady_clock::now();
      dt = std::chrono::duration<double>(now - last_t).count();
      last_t = now;
    }

    view.processUIEvents();
    touch.update();
    cam.update(dt);

    // Apply the panel's intents.
    if (wantCount != bunnyCount) {
      bunnyCount = std::min(std::max(wantCount, 1), kMaxBunnies);
      wantCount = bunnyCount;
      for (int i = 0; i < kMaxBunnies; ++i)
        volNodes[i]->setVisible(i < bunnyCount);
    }
    if (wantFrameAll) {
      wantFrameAll = false;
      frameAll();
    }
    {
      cvc::volslice::render_settings s = volNode->config();
      const bool windowChanged = std::fabs(wantWindow - windowHalf) > 1e-9;
      if (windowChanged) {
        windowHalf = wantWindow;
        const double q = s.slices.quality;
        s = bunny_settings(windowHalf); // re-author the TF for the new band
        s.slices.quality = q;
        s.slices.near_plane = wantNear;
      }
      if (s.slices.quality != wantQuality || s.slices.near_plane != wantNear ||
          s.opacity_correction != wantCorrection ||
          (s.filter == cvc::volslice::interpolation::nearest) != wantNearest || windowChanged) {
        s.slices.quality = wantQuality;
        s.slices.near_plane = wantNear;
        s.opacity_correction = wantCorrection;
        s.filter = wantNearest ? cvc::volslice::interpolation::nearest
                               : cvc::volslice::interpolation::linear;
        applyConfigToAll(s);
      }
    }

    sg.processEvents();
    std::vector<VolSliceNode *> active;
    for (int i = 0; i < bunnyCount; ++i) {
      volNodes[i]->tick();
      active.push_back(volNodes[i].get());
    }
    // Node-vs-node compositing: reorder the actors back to front for the
    // current camera (no-op when the order is already right).
    VolSliceNode::depthSortSliceProps(view.renderer(), active);

    if (!png.empty() && ticks >= 5) {
      view.render();
      view.writePNG(png);
      std::size_t totalPlanes = 0;
      for (int i = 0; i < bunnyCount; ++i)
        totalPlanes += volNodes[i]->planesRendered();
      std::printf("[volslice_bunny] wrote %s (%d bunnies, %zu planes)\n", png.c_str(), bunnyCount,
                  totalPlanes);
      break;
    }
    if (capturing && ticks > kCaptureTickLimit)
      break;
    view.render();

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_report).count() >= 1.0) {
      std::size_t totalPlanes = 0;
      for (int i = 0; i < bunnyCount; ++i)
        totalPlanes += volNodes[i]->planesRendered();
      std::printf("[volslice_bunny] %d bunny(s) | %zu planes | scene %.1f fps | q=%.2f%s\n",
                  bunnyCount, totalPlanes, hud.fps(), wantQuality,
                  wantCorrection ? " | corrected" : "");
      last_report = now;
    }

#ifdef __EMSCRIPTEN__
#ifndef __EMSCRIPTEN_PTHREADS__
    sg.publisher().flush();
#endif
    emscripten_sleep(0);
    ++frame;
#else
    if (frames > 0 && ++frame >= frames)
      break;
#endif
  }

  hud.detach();
  cam.detach();
  return 0;
}
