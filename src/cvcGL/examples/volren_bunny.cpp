// volren_bunny — the cvc::volren showcase: bunny_shadow's scene with the mesh
// bunny REPLACED by a signed-distance-field volume of the same bunny, raycast
// by cvc::volren and composited through VolRenNode, under the identical
// StageLighting rig.
//
// What it demonstrates:
//  - cvc::sdf(bunny mesh) -> Float volume at startup (~0.4 s at 64^3), then
//    rendered as an isosurface at distance 0 -- no mesh anywhere in the scene;
//  - VolRenNode compositing: the raycast frame's depth map drives
//    gl_FragDepth, so the volume occludes and is occluded per pixel by the
//    ground plane and by other bunnies;
//  - the scene-graph transform path: the Add/Remove buttons place up to 9
//    bunnies on a 3x3 grid purely by per-volume model_transform, one raycaster
//    handling them all;
//  - CPU vs CUDA backends and the resolution knob, with a live Mray/s readout;
//  - the lighting rig on top of that: soft (percentage-closer) shadows,
//    SDF ambient occlusion in the bunny's own creases, a sky/ground ambient
//    instead of a flat constant, and the specular/gain knobs that keep the
//    result off the clamp.  All off by default -- the shipped image is
//    byte-identical to the one before any of them existed.
//
// The first bunny stands exactly where bunny_shadow's mesh bunny stands:
// centred on the origin, base on the ground, not moving.
//
// Controls: orbit camera (drag), 'f' toggles the FPS HUD, the Volume raycast
// panel holds the rest.  Nothing in the scene moves on its own -- there is no
// auto-rotation, and the bunnies stand still until Add/Remove is pressed.
//
// The panel is a fixed readout (backend, milliseconds, Mray/s, the raster and
// the ray budget it resolves to) over four collapsing groups, one per COST
// MODEL rather than one per feature:
//   Scene     -- bunny count and the translucent shell; every control here
//                invalidates the shadow maps and pays a rebuild frame.
//   Sampling  -- resolution scale, anti-aliasing, steps, continuous mode: the
//                knobs priced in rays per frame.
//   Shadows   -- the light-view pass, whose cost is camera-INDEPENDENT: the
//                representation (hard vs deep), the softness filter, and a
//                folded "map & bias" subgroup for the tuning knobs.
//   Lighting  -- the shading model: two presets, then ambient, the sky/ground
//                hemisphere, occlusion, output gain and specular.  Nearly free
//                per frame except the occlusion cone, which has its own dial.
//
// CLI mirrors bunny_shadow:
// --width/--height/--offscreen/--frames/--png/--no-shadows/--no-ui/--no-grid
// plus --dim (SDF resolution), --scale (raycast resolution scale, 0.05..2.0),
// --supersample N (anti-aliasing, n x n rays per pixel, 1..4),
// --volume-shadows (volumetric self/inter-volume shadows),
// --bunnies N (initial count, 1..9), --shell (translucent outer shell),
// --cpu (force the CPU backend), --soft-shadows R (percentage-closer filter
// radius in shadow-map texels), --deep-shadows (a transmittance profile per
// light-map texel instead of one depth) with --depth-slices N, --ao S
// (ambient-occlusion strength), --rig (the full lighting rig: sky/ground
// ambient, a cool fill, unity output gain and a damped specular), and
// --continuous (the panel's "re-raycast every frame" checkbox, which is what
// makes the Mray/s readout a steady-state number from the command line).
//
// --no-shadows and --volume-shadows are different mechanisms and both are
// real: the first turns off VTK's shadow pass (the ground's cast shadows,
// which the raycast volume cannot participate in yet), the second turns on
// cvc::volren's own light-view pass inside the raycast.

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

#include <algorithm>
#include <boost/program_options.hpp>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
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

// Raycast settings for the SDF bunny: the surface is the zero level set,
// shaded from the spline gradient with bunny_shadow's material colour.
//
// The optional shell is a second isosurface at a POSITIVE distance, i.e. an
// offset surface floating `shell_offset` world units outside the bunny in
// every direction -- including below its feet, which reads as the bunny
// hovering in a haze.  Off by default for that reason; it exists to show the
// translucent compositing path.
cvc::volren::volume_settings bunny_volume_settings(double shell_offset) {
  cvc::volren::volume_settings vs;
  vs.shaded = false;
  vs.unshaded = false;
  // cvc::sdf produces a true signed distance field in world units, positive
  // outside -- exactly what ambient occlusion's cone trace needs, and the flag
  // is a statement about the DATA, so it is set unconditionally.  It changes
  // nothing on its own: AO is off until render_settings::ao asks for it.
  vs.distance_field = true;
  cvc::volren::isosurface body;
  body.value = 0.0;
  body.opacity = 1.0f;
  body.color = {0.85f, 0.82f, 0.88f}; // bunny_shadow's mesh colour
  body.shininess = 24.0f;
  vs.isosurfaces.push_back(body);
  if (shell_offset > 0.0) {
    cvc::volren::isosurface shell;
    shell.value = shell_offset;
    shell.opacity = 0.16f;
    shell.color = {0.35f, 0.75f, 0.95f};
    shell.shininess = 8.0f;
    vs.isosurfaces.push_back(shell);
  }
  return vs;
}

// The stage key light, shared by the VTK rig and the raycaster (bunny_shadow's
// values).  Azimuth is fixed; elevation is a live control -- see the comment
// on apply_key_light.
constexpr double kKeyAzimuth = -38.0;
constexpr double kKeyElevation = 52.0;

// Where the Nth bunny stands.  Slot 0 is the origin -- exactly where
// bunny_shadow's mesh bunny stands -- and the rest ring outward on a 3x3 grid
// whose pitch exceeds the bunny's footprint, so they never overlap.
constexpr int kMaxBunnies = 9;
constexpr int kGrid[kMaxBunnies][2] = {{0, 0}, {1, 0},  {0, 1},   {-1, 0}, {0, -1},
                                       {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

cvc::volren::mat4 translation(double tx, double ty, double tz) {
  cvc::volren::mat4 m;
  m.m[3] = tx;
  m.m[7] = ty;
  m.m[11] = tz;
  return m;
}

#ifdef CVC_ENABLE_IMGUI
// Widget widths for putting TWO labelled knobs on one row, each sized so its
// own right-hand label still fits inside its half-column.
//
// Used only where the two knobs are genuinely one decision -- a filter's width
// and its tap count, an exposure gain and the specular it is compensating for
// -- and not merely to save space: a pair on one line says "read these
// together", and spending that on unrelated knobs would be a lie about the
// panel's own structure.  It also keeps the whole panel inside a 720-tall
// viewport without a scrollbar, which is what the demo is captured at.
ImVec2 two_up_widths(const char *left, const char *right) {
  const ImGuiStyle &st = ImGui::GetStyle();
  const float col = (ImGui::GetContentRegionAvail().x - st.ItemSpacing.x) * 0.5f;
  return ImVec2(col - ImGui::CalcTextSize(left).x - st.ItemInnerSpacing.x,
                col - ImGui::CalcTextSize(right).x - st.ItemInnerSpacing.x);
}
#endif

} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  int width = 1280, height = 720, frames = 0, fps = 30;
  unsigned dim = 64;
  double scale = 0.5, light_elevation = kKeyElevation;
  int bunnies = 1, supersample = 1;
  std::string png;
  bool offscreen = false, no_shadows = false, no_ui = false, no_grid = false, shell = false,
       cpu_only = false, volume_shadows = false, force_ui = false, full_rig = false,
       deep_shadows = false, continuous = false;
  double soft_shadows = 0.0, ao_strength = 0.0;
  int depth_slices = cvc::volren::defaults::shadow_depth_slices;

  po::options_description desc("volren_bunny options");
  desc.add_options()("help,h", "show help")("width", po::value<int>(&width))(
      "height", po::value<int>(&height))("offscreen", po::bool_switch(&offscreen))(
      "frames", po::value<int>(&frames))("fps", po::value<int>(&fps))(
      "png", po::value<std::string>(&png))("no-shadows", po::bool_switch(&no_shadows))(
      "no-ui", po::bool_switch(&no_ui))("ui", po::bool_switch(&force_ui),
                                        "keep the panel visible while capturing (screenshots)")(
      "no-grid", po::bool_switch(&no_grid))("shell", po::bool_switch(&shell),
                                            "add the translucent offset shell")(
      "cpu", po::bool_switch(&cpu_only), "force the CPU backend")(
      "bunnies", po::value<int>(&bunnies), "initial bunny count 1..9 (default 1)")(
      "dim", po::value<unsigned>(&dim), "SDF volume resolution (default 64)")(
      "scale", po::value<double>(&scale), "raycast resolution scale 0.05..2.0 (default 0.5)")(
      "supersample", po::value<int>(&supersample), "anti-aliasing: n x n rays per pixel, 1..4")(
      "volume-shadows", po::bool_switch(&volume_shadows),
      "volumetric shadows in the raycast")("light-elevation", po::value<double>(&light_elevation),
                                           "key light elevation in degrees, 5..85 (default 52)")(
      "soft-shadows", po::value<double>(&soft_shadows),
      "percentage-closer filter radius in shadow-map texels (0 = unfiltered, the default)")(
      "deep-shadows", po::bool_switch(&deep_shadows),
      "store a transmittance profile per light-map texel instead of one depth, so a "
      "translucent occluder casts a partial shadow (implies --volume-shadows to be visible)")(
      "depth-slices", po::value<int>(&depth_slices),
      "knots in a deep map's profile, 1..64 (default 16)")(
      "continuous", po::bool_switch(&continuous),
      "re-raycast every frame instead of only on change (the panel's own checkbox); the "
      "steady-state number the Mray/s readout shows in a live session, where a still camera "
      "otherwise reports one cold raycast")("ao", po::value<double>(&ao_strength),
                                            "ambient-occlusion strength 0..1 (0 = off)")(
      "rig", po::bool_switch(&full_rig),
      "the full lighting rig: sky/ground ambient, cool fill, unity gain, damped specular");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if (vm.count("help")) {
    // Print the option table, not just a banner: every flag below carries a
    // help string and none of it was reachable before.
    std::printf("volren_bunny: an SDF of the Stanford bunny, raycast by cvc::volren\n"
                "and composited into a live cvcGL scene.\n\n");
    std::cout << desc << std::endl;
    return 0;
  }
  const bool capturing = offscreen || !png.empty();

  cvc::app app;
  app.properties("system.log_verbosity", "2");
  SceneGraph sg(app, "volren_bunny");

  const double S = 100.0; // bunny height in world units
  const cvc::geometry standing = stand_bunny(S);

  // The ground is sized for the FULL 3x3 grid (bunnies can be added at
  // runtime), not just the bunny on stage at startup, so nobody ever ends up
  // standing off the edge.
  const double ground_rgb[3] = {0.32, 0.34, 0.38};
  const double ground_half = std::max(2.2 * S, 1.35 * S * std::sqrt(2.0) + 1.2 * S);
  auto groundNode = std::dynamic_pointer_cast<GeometryNode>(
      sg.addGraphics("ground", ground(ground_half, ground_rgb)));
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
  // Off by default, exactly like the panel checkbox it mirrors: with a still
  // camera the node raycasts twice and stops, so the readout reports a raycast
  // taken on a GPU that has barely woken up.  Measured on this box at
  // 1280x720: one bunny reads 26.1 ms still and 19.3 ms warm, nine read 67-82
  // still and 61.5 warm.  The still figure is not wrong, it is just not the
  // one a user orbiting the scene sees -- and it is misleading enough to
  // invert an ordering: still-camera timings made nine bunnies WITH shadows
  // look faster than without, which the warm numbers correctly reverse.
  volNode->setContinuous(continuous);
  // The node itself stays at the origin and never moves; each bunny is placed
  // by its volume's own model_transform (see place_bunnies).
  if (!cpu_only)
    volNode->setBackend(cvc::volren::backend::automatic);
  // The AMBIENT half of the lighting rig, as a function of one bool, so the
  // panel's preset buttons and --rig are the same code rather than two
  // descriptions of it that can drift.  The direct half (key colour, fill) is
  // apply_key_light below, which reads the same `full_rig`.
  //
  // Deliberately NOT touched here: ao.strength.  Occlusion is an independent
  // knob with its own --ao flag and its own slider, and folding it into the
  // preset would make the button mean two things.
  auto apply_lighting_preset = [&](bool rig) {
    cvc::volren::render_settings rs = volNode->renderConfig();
    if (rig) {
      // A real rig instead of one white lamp plus a flat constant:
      //  - the ambient becomes a SKY/GROUND hemisphere, so an up-facing fold
      //    reads as open air and a down-facing one as bounce off the plate;
      //  - it carries more of the exposure (0.45), which is what gives the
      //    occlusion term something to bite into;
      //  - the gain goes to 1.0, because 0.9 is a legacy damping that also
      //    dims the ambient, and the key is dropped to compensate;
      //  - the specular is damped to 0.35: the legacy model adds the highlight
      //    at the light's FULL colour with no material term, which clamps 15.8%
      //    of this object's pixels at the shipped settings.
      rs.ambient = 0.45f;
      rs.ambient_hemisphere.enabled = true;
      rs.ambient_hemisphere.sky = {0.55f, 0.70f, 1.00f};
      rs.ambient_hemisphere.ground = {0.42f, 0.32f, 0.22f};
      rs.ambient_hemisphere.up = {0.0, 0.0, 1.0};
      rs.shading_gain = 1.0f;
      rs.specular = 0.35f;
      // Only the KEY casts.  An empty list means every light casts, which for a
      // two-light rig is two full light-view passes -- and a fill light exists
      // precisely to open up what the key left dark, so having it drop its own
      // shadows there is both twice the rebuild cost and the wrong picture.
      rs.shadows.lights = {0};
    } else {
      // The legacy expression, exactly: one flat constant, the 0.9 damping and
      // no specular material term.  The hemisphere's colours go back to white
      // as well as off, so "legacy" is a state and not just a disabled flag.
      rs.ambient = 0.25f;
      rs.ambient_hemisphere = cvc::volren::hemisphere_ambient();
      rs.shading_gain = cvc::volren::defaults::shading_gain;
      rs.specular = cvc::volren::defaults::specular;
      rs.shadows.lights.clear();
    }
    volNode->setRenderConfig(rs);
  };

  {
    cvc::volren::render_settings rs = volNode->renderConfig();
    rs.steps = 384;
    rs.supersample = std::min(std::max(supersample, 1), cvc::volren::limits::max_supersample);
    rs.shadows.enabled = volume_shadows;
    // What a light-view texel stores.  hard is the default and the cheap one;
    // deep carries a transmittance profile, which is what lets the --shell
    // occluder dim the body it wraps instead of being thresholded away.
    rs.shadows.mode =
        deep_shadows ? cvc::volren::shadow_mode::deep : cvc::volren::shadow_mode::hard;
    rs.shadows.depth_slices =
        std::min(std::max(depth_slices, cvc::volren::limits::min_shadow_depth_slices),
                 cvc::volren::limits::max_shadow_depth_slices);
    // Soft shadows: a percentage-closer filter over the light-view map.  The
    // radius is in map TEXELS and sets the penumbra WIDTH; the tap count sets
    // how many levels that band resolves into.  Both are off by default (radius
    // 0 takes the single-tap comparison the renderer has always taken).
    rs.shadows.pcf_radius = float(std::max(0.0, soft_shadows));
    rs.shadows.pcf_taps = 5;
    // Ambient occlusion.  It attenuates the AMBIENT term only, so its visible
    // magnitude is bounded by `ambient` -- at the 0.25 this scene ships with, a
    // fully occluded crease can lose at most a quarter of its shading, which
    // reads as depth rather than as dirt.  The radius is in world units: the
    // bunny is 100 tall, so 8 is a couple of fur folds.
    rs.ao.strength = float(std::min(std::max(ao_strength, 0.0), 1.0));
    rs.ao.radius = 8.0;
    rs.ao.samples = 5;
    volNode->setRenderConfig(rs);
  }
  apply_lighting_preset(full_rig);

  // --- Lighting: the bunny_shadow rig, verbatim -----------------------------
  StageLighting rig(sg);
  rig.setStage(0.0, 0.0, 0.5 * S, 0.62 * S);
  rig.applyPreset(StageLighting::Preset::ThreePoint);
  rig.setKey(1.9, kKeyAzimuth, kKeyElevation, 34.0);
  rig.setFill(0.85);
  rig.setBack(0.6);
  rig.setWarmth(0.3);
  rig.setEnvironment(0.7);
  rig.setAmbient(0.4);

  // The raycaster's key light and the scene rig's key are ONE light: the
  // raycast bunny and the VTK ground have to agree about where the sun is, or
  // the composite reads as two scenes.  Every elevation change therefore
  // drives both.
  //
  // Elevation is exposed because it is the knob that decides whether a
  // volumetric shadow is VISIBLE at all here: a bunny of height h casts
  // h/tan(elevation) across the ground, and the neighbouring bunny stands
  // 1.35 h away, so above ~36 deg every shadow lands short of its neighbour
  // and only the self-shadowed creases change.  At the rig's default 52 deg
  // that reach is 0.78 h.
  double keyElevation = std::min(std::max(light_elevation, 5.0), 85.0);
  //
  // Split in two on purpose.  The RAYCASTER'S lights change with the preset
  // (colour, and whether a fill exists) as well as with the elevation; the
  // SCENE rig's key only ever moves with the elevation.  Re-poking the scene
  // rig when only the preset changed is not free: measured, a
  // StageLighting::setKey with byte-identical arguments re-lights the ground
  // plate by up to 116 levels over 533 595 pixels (the raycast bunny is
  // untouched -- it is a cvcGL rig quirk, not a volren one), so a preset button
  // that called it would visibly repaint the plate for no reason.
  auto apply_raycast_lights = [&](double el_deg) {
    const double az = kKeyAzimuth * M_PI / 180.0, el = el_deg * M_PI / 180.0;
    cvc::volren::render_settings rs = volNode->renderConfig();
    cvc::volren::light key;
    key.color =
        full_rig ? std::array<float, 3>{0.75f, 0.72f, 0.66f} : std::array<float, 3>{1.f, 1.f, 1.f};
    key.direction = {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
    rs.lights = {key};
    if (full_rig) {
      // A cool fill from the opposite side and low, the other half of a
      // two-light rig.  Multiple lights ACCUMULATE here (the legacy overwrite
      // bug is fixed), and each carries its own colour into BOTH its diffuse
      // and its specular term, so this is a tint and not just more light.
      const double faz = (kKeyAzimuth + 150.0) * M_PI / 180.0, fel = 16.0 * M_PI / 180.0;
      cvc::volren::light fill;
      fill.color = {0.20f, 0.25f, 0.34f};
      fill.direction = {std::cos(fel) * std::cos(faz), std::cos(fel) * std::sin(faz),
                        std::sin(fel)};
      rs.lights.push_back(fill);
    }
    volNode->setRenderConfig(rs);
  };
  // Both halves, for the one knob that moves both.
  auto apply_key_light = [&](double el_deg) {
    apply_raycast_lights(el_deg);
    rig.setKey(1.9, kKeyAzimuth, el_deg, 34.0);
    rig.apply();
  };
  apply_key_light(keyElevation);

  // Placing the bunnies: slot 0 is the origin, the rest ring outward on a
  // grid whose pitch clears the bunny's ~S-wide footprint.  The stage (the
  // aimed key/fill cones) widens with the ring so every bunny stays lit --
  // for a single bunny this is bunny_shadow's rig unchanged.
  const double pitch = 1.35 * S;
  int bunnyCount = std::min(std::max(bunnies, 1), kMaxBunnies);
  bool shellOn = shell;
  auto place_bunnies = [&](int count, bool with_shell) {
    volNode->clearVolumes();
    double ring = 0.0;
    for (int i = 0; i < count; ++i) {
      cvc::volren::volume_settings vs = bunny_volume_settings(with_shell ? 0.04 * S : 0.0);
      const double tx = kGrid[i][0] * pitch, ty = kGrid[i][1] * pitch;
      vs.model_transform = translation(tx, ty, 0.0);
      volNode->addVolume(sdf_vol, vs);
      ring = std::max(ring, std::sqrt(tx * tx + ty * ty));
    }
    rig.setStage(0.0, 0.0, 0.5 * S, ring + 0.62 * S);
    rig.apply();
  };
  place_bunnies(bunnyCount, shellOn);

  // The ImGui draw callback runs mid-render, so it only records intent here;
  // the main loop applies it before the next tick().
  int wantCount = bunnyCount;
  bool wantFrameAll = false, wantShellChange = false;
  double wantElevation = keyElevation;
  // -1 = nothing asked for; 0 = the legacy expression; 1 = the studio rig.  An
  // int rather than a bool because "no request" and "asked for legacy" are
  // different, and the preset has to be re-appliable after the user has moved
  // the individual sliders.
  int wantRig = -1;

  SceneRenderer view(sg, width, height, capturing || offscreen, "main");
  const bool shadows = !no_shadows && sg.setShadowsEnabled(true);
  if (shadows) {
    sg.setShadowResolution(2048);
    sg.setShadowUpdateInterval(1);
  }

  if (no_grid)
    sg.setDiagnosticChromeVisible(false);

  CameraController cam(view);
  {
    // Frame whatever the demo starts with: bunny_shadow's exact framing for a
    // single bunny, widened to hold the ring when --bunnies asks for more.
    double ring = 0.0;
    for (int i = 0; i < bunnyCount; ++i)
      ring = std::max(
          ring, std::sqrt(double(kGrid[i][0] * kGrid[i][0] + kGrid[i][1] * kGrid[i][1])) * pitch);
    const double r = ring + 2.2 * S;
    cam.frameBounds(-r, -r, 0.0, r, r, 1.3 * S);
  }
  cam.setMode(CameraController::Mode::Orbit);
  cvc::gl::TouchGestures touch(view, cam);

  cvc::gl::FpsHud hud(view);
  if (capturing)
    hud.setEnabled(false);

#ifdef CVC_ENABLE_IMGUI
  cvc::gl::ImGuiOverlay ui(view);
  ImGui::SetCurrentContext(ui.imguiContext());
  ui.attachCamera(cam);
  // A capture normally hides the overlay so the frame is clean; --ui keeps it
  // so the panel itself can be screenshotted.
  ui.setVisible(!no_ui && (!capturing || force_ui));
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

    // The raycast panel.  Four GROUPS under one always-visible readout, rather
    // than one flat column of sliders: the knobs fall into genuinely different
    // cost models (a scene edit rebuilds the shadow maps, a sampling knob costs
    // rays per frame, a lighting knob is nearly free), and a reader who cannot
    // see that division has to be told it in prose instead.  Everything that
    // reports what the renderer is ACTUALLY doing -- backend, ray budget,
    // Mray/s -- stays outside the groups so it is legible whatever is folded
    // away.
    if (uiVolren) {
      // A new window auto-fits to its FIRST frame's content -- when every
      // readout still says 0 and is at its shortest.  Ask for room for the
      // longest line the panel can ever show (~62 characters) or the ray budget
      // gets silently clipped once the numbers fill in; in font sizes, so the
      // reservation follows the UI scale.  Height 0 keeps the auto-fit.
      //
      // Parked under the menu bar rather than at ImGui's default (60, 60): four
      // open groups need most of a 720-tall viewport, and 60 px of wasted head
      // room is the difference between fitting and scrolling.  The height
      // CONSTRAINT is the safety net for the case the layout cannot fix -- a
      // short viewport, or a large UI scale -- where the window would otherwise
      // auto-fit to a height that runs off the bottom of the screen.
      const float top = ImGui::GetFrameHeight() + 6.f;
      ImGui::SetNextWindowPos(ImVec2(8.f, top), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(ImGui::GetFontSize() * 36.f, 0.f), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSizeConstraints(
          ImVec2(0.f, 0.f), ImVec2(FLT_MAX, ImGui::GetIO().DisplaySize.y - top - 8.f));
      if (ImGui::Begin("Volume raycast", &uiVolren)) {
        // ---- readout: what the renderer is doing, always visible -----------
        const cvc::volren::render_settings rset = volNode->renderConfig();
        const int ss = rset.supersample;
        const double ms = volNode->lastRenderSeconds() * 1000.0;
        // The raster the node actually marched, not width * scale re-derived:
        // it is the one that rounded, floored at 2 and (in a resized window)
        // used a viewport these CLI values no longer describe.
        const int rw = volNode->raycastWidth(), rh = volNode->raycastHeight();
        // Rays, not pixels: supersampling multiplies the ray count by its
        // square, so a Mray/s figure that ignored it would read as a slowdown.
        const double rays = double(rw) * rh * ss * ss;
        const bool onGpu = volNode->backendUsed() == cvc::volren::backend::cuda;
        ImGui::Text("%s  |  %.2f ms  |  %.2f Mray/s  |  %llu raycasts", onGpu ? "CUDA" : "CPU", ms,
                    ms > 0.0 ? (rays / (ms / 1000.0)) / 1e6 : 0.0,
                    (unsigned long long)volNode->framesRendered());
        // The whole cost model on one line: what the two quality knobs did to
        // the ray budget, and the output that budget resolves to.  Kept under
        // ~60 characters -- the panel does not auto-widen for text, so a longer
        // line is silently clipped.  The output size comes from ImGui's own
        // display size (the live viewport), not the CLI --width/--height, which
        // a window resize makes stale.
        const ImGuiIO &io = ImGui::GetIO();
        ImGui::Text("raster %dx%d x%d ray/px = %.2f Mray -> %dx%d out", rw, rh, ss * ss, rays / 1e6,
                    int(io.DisplaySize.x), int(io.DisplaySize.y));
        ImGui::TextDisabled("SDF %ux%ux%u  |  isosurface at distance 0", dim, dim, dim);

        // ---- Scene: what is being rendered ---------------------------------
        // First, because every control here invalidates the shadow maps and so
        // costs a rebuild frame, which none of the others do.
        if (ImGui::CollapsingHeader("Scene", ImGuiTreeNodeFlags_DefaultOpen)) {
          ImGui::Text("Bunnies: %d / %d", bunnyCount, kMaxBunnies);
          ImGui::SameLine();
          ImGui::BeginDisabled(bunnyCount >= kMaxBunnies);
          if (ImGui::Button("Add"))
            wantCount = bunnyCount + 1;
          ImGui::EndDisabled();
          ImGui::SameLine();
          ImGui::BeginDisabled(bunnyCount <= 1);
          if (ImGui::Button("Remove"))
            wantCount = bunnyCount - 1;
          ImGui::EndDisabled();
          ImGui::SameLine();
          if (ImGui::Button("Frame all"))
            wantFrameAll = true;
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Re-frames the camera on the ring.  The bunnies never\n"
                              "move and nothing auto-rotates: slot 0 stands on the\n"
                              "origin with its base on the ground, and the rest ring\n"
                              "outward on a 3x3 grid whose pitch clears the footprint.");
          if (ImGui::Checkbox("translucent shell", &shellOn))
            wantShellChange = true;
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("A second isosurface at a POSITIVE distance: an offset\n"
                              "surface floating outside the bunny at opacity 0.16.\n"
                              "It is the scene that separates the two shadow modes --\n"
                              "see 'representation' under Shadows.");
        }

        // ---- Sampling: what a frame costs in rays --------------------------
        if (ImGui::CollapsingHeader("Sampling", ImGuiTreeNodeFlags_DefaultOpen)) {
          // Knob 1 of 2: how many PIXELS get raycast.  Above 1.0 the raster is
          // larger than the viewport and the quad's bilinear filter box-filters
          // it back down, so the slider spans undersample -> 1:1 -> supersample.
          float rscale = float(volNode->resolutionScale());
          if (ImGui::SliderFloat("resolution", &rscale, float(VolRenNode::MinResolutionScale),
                                 float(VolRenNode::MaxResolutionScale), "%.2f x viewport"))
            volNode->setResolutionScale(rscale);
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Raster = viewport x this, rescaled onto the quad.  Costs\n"
                              "scale^2 rays.  Below 1.0 it buys latency and blurs the WHOLE\n"
                              "image; at 2.0 each screen pixel is an exact 2x2 box average\n"
                              "of the raster, which anti-aliases without softening the\n"
                              "interior.  2.0 is the cap because the bilinear filter reads\n"
                              "4 texels: past 2x the extra rays would be thrown away.");

          // Knob 2 of 2: how many RAYS each of those pixels averages.  Same
          // price in rays, opposite trade -- this one leaves the output size
          // alone and averages every ray it casts, at any raster.
          int aa = ss - 1;
          const char *kAaLabels[] = {"off (1 ray/px)", "2x2 (4 rays/px)", "3x3 (9 rays/px)",
                                     "4x4 (16 rays/px)"};
          static_assert(cvc::volren::limits::max_supersample == 4,
                        "anti-aliasing combo labels must cover every supersample level");
          if (ImGui::Combo("anti-aliasing", &aa, kAaLabels, IM_ARRAYSIZE(kAaLabels)))
            volNode->setSupersample(aa + 1);
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("n x n rays per pixel on a regular sub-pixel grid, box-filtered:\n"
                              "n^2 x the cost, output size unchanged.  Anti-aliases isosurface\n"
                              "silhouettes while the interior stays as sharp as the raster\n"
                              "allows, where 'resolution' below 1.0 blurs both.");

          int steps = rset.steps;
          if (ImGui::SliderInt("steps", &steps, 32, 768)) {
            cvc::volren::render_settings next = rset;
            next.steps = steps;
            volNode->setRenderConfig(next);
          }
          bool cont = volNode->continuous();
          if (ImGui::Checkbox("re-raycast every frame", &cont))
            volNode->setContinuous(cont);
        }

        // ---- Shadows: the light-view pass ----------------------------------
        // Its cost model is unlike Sampling's: the light-view pass is
        // CAMERA-INDEPENDENT, so orbiting is free and only a Scene change (or a
        // knob in here) pays for a rebuild.
        if (ImGui::CollapsingHeader("Shadows", ImGuiTreeNodeFlags_DefaultOpen)) {
          bool shadows = volNode->shadowsEnabled();
          if (ImGui::Checkbox("volumetric shadows", &shadows))
            volNode->setShadowsEnabled(shadows);
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("One extra light-view raycast per casting light, CACHED across\n"
                              "camera motion: orbiting costs nothing, a scene change costs\n"
                              "one rebuild frame.  Inter-bunny shadows come free with it --\n"
                              "one map over every registered volume makes 'A shadows B' and\n"
                              "'A shadows itself' the same lookup.  The volume does NOT yet\n"
                              "shadow the ground: that quad is scene geometry, lit by VTK.");

          // Shared by the raycast and the VTK rig, and the knob that decides
          // whether a shadow reaches anything but its own creases here.  Live
          // even with the raycast pass off, because it still moves the scene's
          // own key light.
          float elev = float(keyElevation);
          if (ImGui::SliderFloat("key elevation", &elev, 5.f, 85.f, "%.0f deg"))
            wantElevation = elev;
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Drives BOTH the raycast key light and the scene rig.\n"
                              "A bunny casts height/tan(elevation): below ~36 deg that\n"
                              "reaches past the 1.35-height grid pitch and the bunnies\n"
                              "start shadowing EACH OTHER, above it only their own creases.");

          // Everything below needs the pass: DISABLED rather than hidden, so
          // the panel does not change height when the checkbox is toggled and
          // the reader can see what turning it on would buy.
          ImGui::BeginDisabled(!shadows);
          const cvc::volren::shadow_settings sh = volNode->shadowConfig();

          // WHAT a texel stores.  This is the one shadow control that changes
          // the answer rather than its quality, so it leads.
          int mode = sh.mode == cvc::volren::shadow_mode::deep ? 1 : 0;
          const char *kModeLabels[] = {"hard (one depth)", "deep (transmittance)"};
          if (ImGui::Combo("representation", &mode, kModeLabels, IM_ARRAYSIZE(kModeLabels)))
            volNode->setShadowMode(mode == 1 ? cvc::volren::shadow_mode::deep
                                             : cvc::volren::shadow_mode::hard);
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("hard: one depth per texel, so every occluder is opaque and\n"
                              "every shadow is binary -- and a translucent shell has to be\n"
                              "excluded by hand ('min caster') or it eclipses the\n"
                              "body it wraps.  deep: an accumulated-alpha profile, so the\n"
                              "shell dims what it wraps by its own 0.16 and two layers\n"
                              "multiply.  An OPAQUE occluder is byte-identical either way.\n"
                              "Cost: (slices + 1) floats per texel, host AND device.");

          // Deep-only and hard-only knobs, both shown, both disabled in the
          // mode that ignores them -- which is more honest than hiding them,
          // since 'min caster opacity' being INERT in deep mode is exactly the
          // point of deep mode.
          // The two mode-specific knobs, side by side and each disabled in the
          // mode that ignores it -- which is the clearest statement the panel
          // can make about what changing the representation actually does.
          const bool deep = mode == 1;
          const ImVec2 modeW = two_up_widths("depth slices", "min caster");
          ImGui::BeginDisabled(!deep);
          int slices = sh.depth_slices;
          ImGui::SetNextItemWidth(modeW.x);
          if (ImGui::SliderInt("depth slices", &slices,
                               cvc::volren::limits::min_shadow_depth_slices,
                               cvc::volren::limits::max_shadow_depth_slices))
            volNode->setDeepShadowSlices(slices);
          if (ImGui::IsItemHovered() && deep)
            ImGui::SetTooltip("Knots in the transmittance profile, uniform in light-space\n"
                              "depth, plus one EXACT terminal depth that keeps an opaque\n"
                              "step sharp.  A separated occluder needs 2; a receiver INSIDE\n"
                              "the medium wants 16 (the default: 1.6 LSB mean vs a 512-slice\n"
                              "reference, where 8 gives 5.1).  Memory is linear in it:\n"
                              "resolution^2 x (slices + 1) x 4 bytes per casting light.");
          ImGui::EndDisabled();

          ImGui::SameLine();
          ImGui::BeginDisabled(deep);
          float minOcc = sh.min_occluder_opacity;
          ImGui::SetNextItemWidth(modeW.y);
          if (ImGui::SliderFloat("min caster", &minOcc, 0.f, 1.f, "%.2f")) {
            cvc::volren::shadow_settings next = sh;
            next.min_occluder_opacity = minOcc;
            volNode->setShadowConfig(next);
          }
          if (ImGui::IsItemHovered() && !deep)
            ImGui::SetTooltip("hard mode only: an isosurface casts only when its opacity\n"
                              "reaches this.  It exists because the hard latch fires on the\n"
                              "FIRST hit whatever its opacity, so without it the 0.16 shell\n"
                              "would drop the whole bunny into shadow.  deep mode IGNORES\n"
                              "it -- there the shell is represented, not thresholded away.");
          ImGui::EndDisabled();

          // SOFTNESS, orthogonal to the representation: either payload can be
          // filtered, and the two knobs below are independent of each other --
          // the radius alone sets the penumbra WIDTH, the tap count alone sets
          // how many levels it resolves into.
          float pcfR = sh.pcf_radius;
          int pcfT = sh.pcf_taps;
          const ImVec2 softW = two_up_widths("soft radius", "soft taps");
          ImGui::SetNextItemWidth(softW.x);
          bool softChanged = ImGui::SliderFloat("soft radius", &pcfR, 0.f, 16.f, "%.1f tx");
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Percentage-closer filter half-width, in light-map TEXELS.\n"
                              "0 is the single-tap comparison -- byte-identical to no\n"
                              "filter at all.  The penumbra is this wide whatever the\n"
                              "tap count, so raising the shadow map SHARPENS it.\n"
                              "Measured here: 4 screen px of penumbra per texel of radius.");
          ImGui::SameLine();
          ImGui::SetNextItemWidth(softW.y);
          softChanged |= ImGui::SliderInt("soft taps", &pcfT, cvc::volren::limits::min_pcf_taps,
                                          cvc::volren::limits::max_pcf_taps);
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Taps per EDGE, so the lookup costs this SQUARED texel\n"
                              "reads per shaded sample (x2 in deep mode).  It buys\n"
                              "levels inside the band, never width: 3 taps over a wide\n"
                              "radius is a staircase, 7 is a gradient.  The grid is fixed\n"
                              "and unjittered -- determinism is a contract here.");
          if (softChanged)
            volNode->setSoftShadows(pcfR, pcfT);

          // Map size and the two bias knobs: quality tuning, folded away
          // because a reader almost never touches them and they are the least
          // interesting thing in the group.
          if (ImGui::TreeNode("map & bias")) {
            cvc::volren::shadow_settings next = sh;
            bool changed = false;
            changed |= ImGui::SliderInt("shadow map", &next.resolution,
                                        cvc::volren::limits::min_shadow_resolution, 2048);
            if (ImGui::IsItemHovered())
              ImGui::SetTooltip("Light-view raster edge: quadratic in rebuild cost AND in\n"
                                "a deep map's memory, inversely linear in the slope bias\n"
                                "and in the world width of a soft radius.");
            changed |= ImGui::SliderFloat("shadow strength", &next.strength, 0.f, 1.f, "%.2f");
            changed |= ImGui::SliderFloat("shadow bias", &next.bias_scale, 0.f, 4.f, "%.2f");
            if (ImGui::IsItemHovered())
              ImGui::SetTooltip("In units of the light-view depth latch's own quantum.\n"
                                "1.0 is the measured bound; drop it toward 0 to watch\n"
                                "acne appear, raise it to peter-pan the crease shadows.");
            changed |= ImGui::SliderFloat("slope bias", &next.slope_scale, 0.f, 4.f, "%.2f");
            if (ImGui::IsItemHovered())
              ImGui::SetTooltip("In light-map texels x tan(normal, light).  The soft filter\n"
                                "widens it by (1 + 2 x soft radius): PCF grows the lateral\n"
                                "footprint the bias exists to cover, and without that the\n"
                                "outer taps self-shadow into a ring of acne.");
            if (changed)
              volNode->setShadowConfig(next);
            ImGui::TreePop();
          }
          ImGui::EndDisabled();
        }

        // ---- Lighting: the shading model itself ----------------------------
        // Nearly free per frame (the AO cone is the one exception, and it is
        // priced on its own slider), and all of it neutral at the defaults.
        if (ImGui::CollapsingHeader("Lighting", ImGuiTreeNodeFlags_DefaultOpen)) {
          // Two presets, because the individual knobs below only make sense
          // once a reader has seen the two ENDS: the legacy expression this
          // renderer shipped with, and a rig that uses every term added since.
          if (ImGui::Button("Preset: studio rig"))
            wantRig = 1;
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Sky/ground ambient at 0.45, a cool fill opposite the key,\n"
                              "unity output gain, specular damped to 0.35, occlusion on,\n"
                              "and only the key casting.  Identical to --rig.");
          ImGui::SameLine();
          if (ImGui::Button("Preset: legacy"))
            wantRig = 0;
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("One white key, flat ambient 0.25, gain 0.9, specular 1.0:\n"
                              "the expression the renderer shipped with, in which 15.8%%\n"
                              "of this bunny's pixels clamp.");

          // The flat constant, and the ceiling on what the two knobs after it
          // can do: the hemisphere only TINTS this term and occlusion only
          // ATTENUATES it, so at 0 both are invisible by arithmetic.
          float amb = volNode->ambientLevel();
          if (ImGui::SliderFloat("ambient", &amb, 0.f, 1.f, "%.2f"))
            volNode->setAmbientLevel(amb);
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The flat term added to every sample whichever way it faces.\n"
                              "It is also the BUDGET the two knobs below spend: the\n"
                              "hemisphere tints it, occlusion attenuates it, so at 0 they\n"
                              "both do nothing at all.");

          cvc::volren::hemisphere_ambient hemi = volNode->hemisphereConfig();
          bool hemiChanged = ImGui::Checkbox("sky/ground ambient", &hemi.enabled);
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Tints the ambient term by the sample's own normal:\n"
                              "sky overhead, bounce underfoot.  One dot product and\n"
                              "three lerps -- measured at -0.9%% on nine bunnies, i.e.\n"
                              "free.  Neutral while both colours are white.");
          if (hemi.enabled) {
            // On the checkbox's own row: the two colours ARE the knob it turns
            // on, and giving them a row of their own would make the panel grow
            // past the viewport exactly when a reader turns on the second
            // optional group.
            ImGui::SameLine();
            hemiChanged |= ImGui::ColorEdit3("sky", hemi.sky.data(), ImGuiColorEditFlags_NoInputs);
            ImGui::SameLine();
            hemiChanged |=
                ImGui::ColorEdit3("ground", hemi.ground.data(), ImGuiColorEditFlags_NoInputs);
          }
          if (hemiChanged)
            volNode->setHemisphereConfig(hemi);

          cvc::volren::ao_settings ao = volNode->occlusionConfig();
          bool aoChanged = ImGui::SliderFloat("occlusion", &ao.strength, 0.f, 1.f, "%.2f");
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Ambient occlusion, cone-traced through the SDF: a few\n"
                              "samples along the normal, each certifying an empty\n"
                              "SPHERE of the radius the field reports.  It attenuates\n"
                              "AMBIENT only -- direct light already has an exact\n"
                              "visibility term -- so raise `ambient` to see more of it.\n"
                              "Measures ONE volume: bunnies do not occlude each other.");
          if (ao.strength > 0.f) {
            // The cone's WHAT and its HOW WELL, on one row: radius decides what
            // the term is about (curvature, creases, contact) and samples is the
            // price of resolving it, and neither reads correctly alone.
            const ImVec2 aoW = two_up_widths("occlusion radius", "samples");
            float ao_radius = float(ao.radius);
            ImGui::SetNextItemWidth(aoW.x);
            if (ImGui::SliderFloat("occlusion radius", &ao_radius, 0.f, 30.f, "%.1f world")) {
              ao.radius = double(ao_radius);
              aoChanged = true;
            }
            if (ImGui::IsItemHovered())
              ImGui::SetTooltip("How far the cone reaches, in world units on a\n"
                                "100-unit bunny.  Under a cell it measures\n"
                                "interpolation error, not geometry.");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(aoW.y);
            aoChanged |=
                ImGui::SliderInt("samples", &ao.samples, cvc::volren::limits::min_ao_samples,
                                 cvc::volren::limits::max_ao_samples);
            if (ImGui::IsItemHovered())
              ImGui::SetTooltip("The quality/cost dial, and the cost is exact: this\n"
                                "many trilinear fetches per shaded isosurface hit.\n"
                                "Measured on CUDA, nine bunnies: +10%% at 1, +35%% at\n"
                                "5, +87%% at 16.  One tap is a hard threshold at one\n"
                                "distance; more integrate the cone and soften it.");
          }
          if (aoChanged)
            volNode->setOcclusionConfig(ao);

          // One row, because they are one decision: the gain is what the legacy
          // 0.9 damped, and the specular is what that damping was compensating
          // for.  The rig moves them together (1.0 / 0.35) for that reason.
          const ImVec2 expW = two_up_widths("output gain", "specular");
          float gain = volNode->shadingGain();
          ImGui::SetNextItemWidth(expW.x);
          if (ImGui::SliderFloat("output gain", &gain, 0.f, 1.5f, "%.2f"))
            volNode->setShadingGain(gain);
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The legacy 0.9 damping, now a knob.  It scales the\n"
                              "AMBIENT term too, so at 0.9 the renderer cannot\n"
                              "reproduce its own material colour.");
          ImGui::SameLine();
          float spec = volNode->specularLevel();
          ImGui::SetNextItemWidth(expW.y);
          if (ImGui::SliderFloat("specular", &spec, 0.f, 1.f, "%.2f"))
            volNode->setSpecularLevel(spec);
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The legacy model adds the highlight at the light's FULL\n"
                              "colour with no material term: at 1.0, 15.8%% of this\n"
                              "object's pixels clamp and lose their shading.");
        }
      }
      ImGui::End();
    }
  });
#endif

  int frame = 0;
  // Ticks since the loop started, counted separately from `frame` because
  // `frame` only advances when --frames asked for a fixed count.  It bounds the
  // wait for convergence below so a capture can never hang.
  int ticks = 0;
  constexpr int kCaptureTickLimit = 600;
  std::uint64_t last_report_frames = 0;
  auto last_report = std::chrono::steady_clock::now();
  auto last_t = last_report;

  while (!view.windowClosed()) {
    ++ticks;
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

    // Apply anything the UI asked for since the last frame.  The bunnies are
    // static: nothing here moves unless the user changes the layout.
    if (wantCount != bunnyCount || wantShellChange) {
      bunnyCount = std::min(std::max(wantCount, 1), kMaxBunnies);
      wantCount = bunnyCount;
      wantShellChange = false;
      place_bunnies(bunnyCount, shellOn);
    }
    if (wantElevation != keyElevation) {
      keyElevation = wantElevation;
      apply_key_light(keyElevation);
    }
    if (wantRig >= 0) {
      // Both halves of the RAYCASTER's rig, in this order: apply_raycast_lights
      // reads `full_rig` to decide the key's colour and whether the cool fill
      // exists, so flipping the flag without re-running it would leave a
      // two-light rig lit by one white lamp.  The scene rig is deliberately not
      // touched -- see apply_raycast_lights.
      full_rig = wantRig == 1;
      wantRig = -1;
      apply_lighting_preset(full_rig);
      apply_raycast_lights(keyElevation);
    }
    if (wantFrameAll) {
      wantFrameAll = false;
      double ring = 0.0;
      for (int i = 0; i < bunnyCount; ++i)
        ring = std::max(
            ring, std::sqrt(double(kGrid[i][0] * kGrid[i][0] + kGrid[i][1] * kGrid[i][1])) * pitch);
      const double r = ring + 1.2 * S;
      cam.frameBounds(-r, -r, 0.0, r, r, 1.3 * S);
    }

    volNode->tick();

    // Capture only a CONVERGED frame.  VolRenNode raycasts on a worker thread,
    // so the picture on screen at an arbitrary tick came out of whatever
    // settings were live when THAT raycast started -- which for the first ticks
    // is the pre-configuration state.  Without this gate, `--png` at the
    // default frame count writes a frame that is not of the scene it was asked
    // for: measured, `--rig`, `--ao 1.0` and `--soft-shadows 6` each wrote a
    // PNG byte-identical to the default one, which makes the demo's own
    // screenshots evidence for the wrong thing.  converged() is documented as
    // the poll for exactly this (VolRenNode.h); the tick bound keeps a scene
    // that never settles from hanging the capture instead of hanging the
    // renderer.
    if (!png.empty() && frame >= std::max(0, frames - 1) &&
        (volNode->converged() || ticks >= kCaptureTickLimit)) {
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
      // The live knobs, not the CLI seeds: both are editable in the panel, and
      // the ray count is what the Mray/s figure has to be divided by.
      const int ssn = volNode->supersample();
      const int rw = volNode->raycastWidth(), rh = volNode->raycastHeight();
      const double rays = double(rw) * rh * ssn * ssn;
      std::printf("[volren_bunny] %d bunny(s) | %s | scene %.1f fps | raycast %.1f ms "
                  "(%.2f Mray/s, %dx%d x%d ray/px%s) | %llu raycasts (+%llu)\n",
                  bunnyCount, volNode->backendUsed() == cvc::volren::backend::cuda ? "CUDA" : "CPU",
                  hud.fps(), ms, ms > 0.0 ? (rays / (ms / 1000.0)) / 1e6 : 0.0, rw, rh, ssn * ssn,
                  volNode->shadowsEnabled() ? ", shadows" : "", (unsigned long long)rendered,
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
    // `--frames N` with `--png` is a FLOOR, not a deadline: the capture above
    // still waits for a converged frame, so ending the loop on the count would
    // exit without writing anything whenever the raycast is a tick behind.
    if (frames > 0 && ++frame >= frames && png.empty())
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
