/*
  Unit tests for cvc::volren::state_settings -- the app-state-tree binding
  for the software raycast volume renderer.

  state_settings handlers are SYNCHRONOUS (setInstanceThreading(false)), so a
  state write applies before the write call returns; seedState() runs in the
  constructor under a re-entry guard and does NOT invoke the apply callback.
  Every test uses a fresh cvc::app and a distinct state path so tests stay
  independent.
*/

#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/volren/raycaster.h>
#include <cvc/volren/state_settings.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <vector>

using cvc::volren::state_settings;

namespace {

// Parse a flat comma-separated list of doubles (the encoding state_settings
// uses for every aggregate key).  Empty string => empty vector.
std::vector<double> parseCsv(const std::string &s) {
  std::vector<double> out;
  std::istringstream in(s);
  std::string tok;
  while (std::getline(in, tok, ','))
    out.push_back(std::stod(tok));
  return out;
}

// Read a key under the given volren state path from the tree, the same way
// external writers address it: cvc::state::instance(ctx)("<path>.<key>").
cvc::state &treeKey(cvc::app &ctx, const std::string &path, const std::string &key) {
  return cvc::state::instance(ctx)(path + "." + key);
}

// A 16-entry row-major translation matrix (tx, ty, tz in column 3).
std::vector<double> translationRowMajor(double tx, double ty, double tz) {
  return {1, 0, 0, tx, 0, 1, 0, ty, 0, 0, 1, tz, 0, 0, 0, 1};
}

// Regression (port review): the gradient ramp's plateau survives the state
// round-trip -- both the 4-value encoding and a legacy 3-value string.
TEST(VolrenStateSettings, GradientRampPlateauRoundTrips) {
  cvc::app ctx;
  const std::string path = "t8.volren";
  cvc::volren::state_settings ss(ctx, path);

  cvc::volren::state_settings::snapshot snap = ss.get();
  cvc::volren::volume_settings vs;
  vs.gradient_ramp.enabled = true;
  vs.gradient_ramp.ramp0 = 1.0;
  vs.gradient_ramp.ramp1 = 2.0;
  vs.gradient_ramp.ramp2 = 3.0;
  vs.gradient_ramp.plateau = 0.5;
  snap.volumes.push_back(vs);
  ss.set(snap);

  const cvc::volren::state_settings::snapshot back = ss.get();
  ASSERT_EQ(back.volumes.size(), 1u);
  EXPECT_NEAR(back.volumes[0].gradient_ramp.plateau, 0.5, 1e-12);

  // External 4-value write parses the plateau; a legacy 3-value write keeps
  // the default.
  cvc::state::instance(ctx)(path + ".volumes.0.gradient_ramp").value(std::string("1,2,3,0.25"));
  EXPECT_NEAR(ss.get().volumes[0].gradient_ramp.plateau, 0.25, 1e-12);
  cvc::state::instance(ctx)(path + ".volumes.0.gradient_ramp").value(std::string("1,2,3"));
  EXPECT_NEAR(ss.get().volumes[0].gradient_ramp.plateau, cvc::volren::defaults::gradient_plateau,
              1e-12);
}

} // namespace

// ============================================================================
// Seeding: constructing state_settings writes the full default snapshot.
// ============================================================================

TEST(VolrenStateSettings, SeedWritesDefaults) {
  cvc::app ctx;
  const std::string path = "t1.volren";
  state_settings ss(ctx, path);

  EXPECT_EQ(state_settings::sceneStatePath("scene"), "scene.volren");

  EXPECT_EQ(treeKey(ctx, path, "camera.eye").value(), "0,-4,0");
  EXPECT_EQ(treeKey(ctx, path, "camera.focal").value(), "0,0,0");
  EXPECT_EQ(treeKey(ctx, path, "camera.up").value(), "0,0,1");
  EXPECT_EQ(treeKey(ctx, path, "camera.projection").value<int>(), 0);
  EXPECT_NEAR(treeKey(ctx, path, "camera.vfov_degrees").value<double>(),
              cvc::volren::defaults::vfov_degrees, 1e-12);
  EXPECT_NEAR(treeKey(ctx, path, "camera.parallel_scale").value<double>(), 1.0, 1e-12);
  EXPECT_EQ(treeKey(ctx, path, "image.width").value<int>(), 512);
  EXPECT_EQ(treeKey(ctx, path, "image.height").value<int>(), 512);

  EXPECT_EQ(treeKey(ctx, path, "steps").value<int>(), cvc::volren::defaults::steps);
  EXPECT_NEAR(treeKey(ctx, path, "opacity_cutoff").value<double>(),
              double(cvc::volren::defaults::opacity_cutoff), 1e-6);
  EXPECT_NEAR(treeKey(ctx, path, "depth_alpha_threshold").value<double>(),
              double(cvc::volren::defaults::depth_alpha_threshold), 1e-6);
  EXPECT_EQ(treeKey(ctx, path, "two_sided_lighting").value<int>(), 0);
  EXPECT_EQ(treeKey(ctx, path, "background").value(), "0,0,0");
  EXPECT_NEAR(treeKey(ctx, path, "ambient").value<double>(), 0.0, 1e-12);
  EXPECT_EQ(treeKey(ctx, path, "threads").value<int>(), 0);
  EXPECT_EQ(treeKey(ctx, path, "supersample").value<int>(), cvc::volren::defaults::supersample);

  EXPECT_EQ(treeKey(ctx, path, "shadows.enabled").value<int>(), 0);
  EXPECT_EQ(treeKey(ctx, path, "shadows.lights").value(), "");
  EXPECT_EQ(treeKey(ctx, path, "shadows.resolution").value<int>(),
            cvc::volren::defaults::shadow_resolution);
  EXPECT_NEAR(treeKey(ctx, path, "shadows.strength").value<double>(), 1.0, 1e-12);
  EXPECT_NEAR(treeKey(ctx, path, "shadows.bias_scale").value<double>(),
              double(cvc::volren::defaults::shadow_bias_scale), 1e-12);
  EXPECT_NEAR(treeKey(ctx, path, "shadows.slope_scale").value<double>(),
              double(cvc::volren::defaults::shadow_slope_scale), 1e-12);
  EXPECT_NEAR(treeKey(ctx, path, "shadows.min_occluder_opacity").value<double>(),
              double(cvc::volren::defaults::shadow_min_occluder_opacity), 1e-12);

  EXPECT_EQ(treeKey(ctx, path, "lights").value(), "");
  EXPECT_EQ(treeKey(ctx, path, "cut_planes").value(), "");
  EXPECT_EQ(treeKey(ctx, path, "volumes.count").value<int>(), 0);
}

// ============================================================================
// set(): object -> state encodes every aggregate as a flat CSV of doubles.
// ============================================================================

TEST(VolrenStateSettings, SetWritesState) {
  cvc::app ctx;
  const std::string path = "t2.volren";
  state_settings ss(ctx, path);

  state_settings::snapshot snap;
  snap.camera.projection = cvc::volren::camera::projection_type::orthographic;
  snap.camera.parallel_scale = 2.5;
  snap.camera.eye = {1.0, -2.0, 3.0};
  snap.camera.width = 96;
  snap.camera.height = 64;

  cvc::volren::light l;
  l.color = {0.5f, 0.25f, 1.0f};
  l.direction = {1.0, 2.0, 3.0};
  snap.settings.lights.push_back(l);

  cvc::volren::cut_plane cp;
  cp.point = {0.5, 0.0, -0.5};
  cp.normal = {0.0, 1.0, 0.0};
  snap.settings.cut_planes.push_back(cp);

  cvc::volren::volume_settings vs;
  vs.tf.add({2.0, 1.f, 0.f, 0.f, 0.1f});
  vs.tf.add({8.0, 0.f, 0.f, 1.f, 0.9f});
  vs.window_enabled = true;
  vs.window_min = 2.25;
  vs.window_max = 8.5;
  vs.gradient_ramp.enabled = true;
  vs.gradient_ramp.ramp0 = 0.0;
  vs.gradient_ramp.ramp1 = 5.0;
  vs.gradient_ramp.ramp2 = 10.0;
  cvc::volren::isosurface iso;
  iso.value = 5.0;
  iso.opacity = 0.5f;
  iso.color = {0.25f, 0.5f, 0.75f};
  iso.shininess = 12.0f;
  vs.isosurfaces.push_back(iso);
  const std::vector<double> xlate = translationRowMajor(1.5, -2.0, 3.25);
  vs.model_transform = cvc::volren::mat4::from_row_major(xlate.data());
  snap.volumes.push_back(vs);

  ss.set(snap);

  EXPECT_EQ(treeKey(ctx, path, "camera.projection").value<int>(), 1);
  EXPECT_NEAR(treeKey(ctx, path, "camera.parallel_scale").value<double>(), 2.5, 1e-12);
  EXPECT_EQ(treeKey(ctx, path, "camera.eye").value(), "1,-2,3");
  EXPECT_EQ(treeKey(ctx, path, "image.width").value<int>(), 96);
  EXPECT_EQ(treeKey(ctx, path, "image.height").value<int>(), 64);

  // lights: one light = 6 flat numbers (r,g,b,dx,dy,dz)
  const std::vector<double> lights = parseCsv(treeKey(ctx, path, "lights").value());
  ASSERT_EQ(lights.size(), 6u);
  EXPECT_NEAR(lights[0], 0.5, 1e-6);
  EXPECT_NEAR(lights[1], 0.25, 1e-6);
  EXPECT_NEAR(lights[2], 1.0, 1e-6);
  EXPECT_NEAR(lights[3], 1.0, 1e-12);
  EXPECT_NEAR(lights[4], 2.0, 1e-12);
  EXPECT_NEAR(lights[5], 3.0, 1e-12);

  // cut_planes: one plane = 6 flat numbers (px,py,pz,nx,ny,nz)
  const std::vector<double> planes = parseCsv(treeKey(ctx, path, "cut_planes").value());
  ASSERT_EQ(planes.size(), 6u);
  EXPECT_NEAR(planes[0], 0.5, 1e-12);
  EXPECT_NEAR(planes[2], -0.5, 1e-12);
  EXPECT_NEAR(planes[4], 1.0, 1e-12);

  EXPECT_EQ(treeKey(ctx, path, "volumes.count").value<int>(), 1);

  // 2-point TF: color ramp has 4 values per point, opacity ramp 2 per point.
  const std::vector<double> color =
      parseCsv(treeKey(ctx, path, "volumes.0.transfer_function.color").value());
  ASSERT_EQ(color.size(), 8u);
  EXPECT_NEAR(color[0], 2.0, 1e-12); // value
  EXPECT_NEAR(color[1], 1.0, 1e-6);  // r
  EXPECT_NEAR(color[4], 8.0, 1e-12); // value
  EXPECT_NEAR(color[7], 1.0, 1e-6);  // b
  const std::vector<double> opacity =
      parseCsv(treeKey(ctx, path, "volumes.0.transfer_function.opacity").value());
  ASSERT_EQ(opacity.size(), 4u);
  EXPECT_NEAR(opacity[0], 2.0, 1e-12);
  EXPECT_NEAR(opacity[1], 0.1, 1e-6);
  EXPECT_NEAR(opacity[2], 8.0, 1e-12);
  EXPECT_NEAR(opacity[3], 0.9, 1e-6);

  // matrix: 16 row-major doubles, translation in column 3.
  const std::vector<double> matrix = parseCsv(treeKey(ctx, path, "volumes.0.matrix").value());
  ASSERT_EQ(matrix.size(), 16u);
  for (std::size_t i = 0; i < 16; ++i)
    EXPECT_NEAR(matrix[i], xlate[i], 1e-12) << "matrix[" << i << "]";

  // window: "min,max" -- compare parsed values rather than the exact string.
  const std::vector<double> window = parseCsv(treeKey(ctx, path, "volumes.0.window").value());
  ASSERT_EQ(window.size(), 2u);
  EXPECT_NEAR(window[0], 2.25, 1e-12);
  EXPECT_NEAR(window[1], 8.5, 1e-12);

  // The ramp encodes as "r0,r1,r2,plateau" (plateau defaults when reading a
  // legacy 3-value string).
  const std::vector<double> ramp = parseCsv(treeKey(ctx, path, "volumes.0.gradient_ramp").value());
  ASSERT_EQ(ramp.size(), 4u);
  EXPECT_NEAR(ramp[0], 0.0, 1e-12);
  EXPECT_NEAR(ramp[1], 5.0, 1e-12);
  EXPECT_NEAR(ramp[2], 10.0, 1e-12);
  EXPECT_NEAR(ramp[3], cvc::volren::defaults::gradient_plateau, 1e-12);

  const std::vector<double> isos = parseCsv(treeKey(ctx, path, "volumes.0.isosurfaces").value());
  ASSERT_EQ(isos.size(), 6u);
  EXPECT_NEAR(isos[0], 5.0, 1e-12); // value
  EXPECT_NEAR(isos[1], 0.5, 1e-6);  // opacity
  EXPECT_NEAR(isos[2], 0.25, 1e-6); // r
  EXPECT_NEAR(isos[5], 12.0, 1e-6); // shininess
}

// ============================================================================
// State -> object: external writes apply synchronously through the callback.
// ============================================================================

TEST(VolrenStateSettings, StateDrivesApply) {
  cvc::app ctx;
  const std::string path = "t3.volren";

  int applied = 0;
  state_settings::snapshot last;
  state_settings ss(ctx, path, [&](const state_settings::snapshot &s) {
    ++applied;
    last = s;
  });

  // seedState() in the constructor must NOT invoke apply (re-entry guard).
  EXPECT_EQ(applied, 0);

  treeKey(ctx, path, "steps").value(99);
  EXPECT_EQ(applied, 1); // synchronous: fired before the write returned
  EXPECT_EQ(ss.get().settings.steps, 99);
  EXPECT_EQ(last.settings.steps, 99);

  treeKey(ctx, path, "camera.eye").value(std::string("1,2,3"));
  EXPECT_EQ(applied, 2);
  EXPECT_NEAR(ss.get().camera.eye[0], 1.0, 1e-12);
  EXPECT_NEAR(ss.get().camera.eye[1], 2.0, 1e-12);
  EXPECT_NEAR(ss.get().camera.eye[2], 3.0, 1e-12);
  EXPECT_NEAR(last.camera.eye[2], 3.0, 1e-12);

  treeKey(ctx, path, "two_sided_lighting").value(1);
  EXPECT_EQ(applied, 3);
  EXPECT_TRUE(ss.get().settings.two_sided_lighting);
  EXPECT_TRUE(last.settings.two_sided_lighting);

  // The earlier changes persist through later re-reads.
  EXPECT_EQ(last.settings.steps, 99);
}

// ============================================================================
// State -> object round trip of the full per-volume key set.
// ============================================================================

TEST(VolrenStateSettings, VolumeRoundTrip) {
  cvc::app ctx;
  const std::string path = "t4.volren";
  state_settings ss(ctx, path);

  // Write every volumes.0.* key first, then flip volumes.count last so the
  // final synchronous re-read parses the complete volume.
  treeKey(ctx, path, "volumes.0.shaded").value(1);
  treeKey(ctx, path, "volumes.0.unshaded").value(1);
  treeKey(ctx, path, "volumes.0.tf_auto_domain").value(0);
  treeKey(ctx, path, "volumes.0.matrix").value(std::string("1,0,0,4,0,1,0,5,0,0,1,6,0,0,0,1"));
  treeKey(ctx, path, "volumes.0.transfer_function.color").value(std::string("0,1,0,0,10,0,0,1"));
  treeKey(ctx, path, "volumes.0.transfer_function.opacity").value(std::string("0,0,10,1"));
  treeKey(ctx, path, "volumes.0.window").value(std::string("2,8"));
  treeKey(ctx, path, "volumes.0.gradient_ramp").value(std::string("0,5,10"));
  treeKey(ctx, path, "volumes.0.isosurfaces").value(std::string("5,0.5,1,1,1,10"));
  treeKey(ctx, path, "volumes.count").value(1);

  const state_settings::snapshot snap = ss.get();
  ASSERT_EQ(snap.volumes.size(), 1u);
  const cvc::volren::volume_settings &vs = snap.volumes[0];

  EXPECT_TRUE(vs.shaded);
  EXPECT_TRUE(vs.unshaded);
  EXPECT_FALSE(vs.tf_auto_domain);

  // The merged TF has control points at the union of the color and opacity
  // ramp scalars {0, 10}: color from the color ramp, alpha from the opacity
  // ramp.
  const std::vector<cvc::volren::transfer_point> &pts = vs.tf.points();
  ASSERT_EQ(pts.size(), 2u);
  EXPECT_NEAR(pts[0].value, 0.0, 1e-12);
  EXPECT_NEAR(pts[0].r, 1.0f, 1e-6f);
  EXPECT_NEAR(pts[0].g, 0.0f, 1e-6f);
  EXPECT_NEAR(pts[0].b, 0.0f, 1e-6f);
  EXPECT_NEAR(pts[0].a, 0.0f, 1e-6f);
  EXPECT_NEAR(pts[1].value, 10.0, 1e-12);
  EXPECT_NEAR(pts[1].r, 0.0f, 1e-6f);
  EXPECT_NEAR(pts[1].g, 0.0f, 1e-6f);
  EXPECT_NEAR(pts[1].b, 1.0f, 1e-6f);
  EXPECT_NEAR(pts[1].a, 1.0f, 1e-6f);

  EXPECT_TRUE(vs.window_enabled);
  EXPECT_NEAR(vs.window_min, 2.0, 1e-12);
  EXPECT_NEAR(vs.window_max, 8.0, 1e-12);

  EXPECT_TRUE(vs.gradient_ramp.enabled);
  EXPECT_NEAR(vs.gradient_ramp.ramp0, 0.0, 1e-12);
  EXPECT_NEAR(vs.gradient_ramp.ramp1, 5.0, 1e-12);
  EXPECT_NEAR(vs.gradient_ramp.ramp2, 10.0, 1e-12);

  ASSERT_EQ(vs.isosurfaces.size(), 1u);
  EXPECT_NEAR(vs.isosurfaces[0].value, 5.0, 1e-12);
  EXPECT_NEAR(vs.isosurfaces[0].opacity, 0.5f, 1e-6f);
  EXPECT_NEAR(vs.isosurfaces[0].color[0], 1.0f, 1e-6f);
  EXPECT_NEAR(vs.isosurfaces[0].color[1], 1.0f, 1e-6f);
  EXPECT_NEAR(vs.isosurfaces[0].color[2], 1.0f, 1e-6f);
  EXPECT_NEAR(vs.isosurfaces[0].shininess, 10.0f, 1e-6f);

  // Row-major translation matrix parsed back into model_transform.
  EXPECT_NEAR(vs.model_transform.m[3], 4.0, 1e-12);
  EXPECT_NEAR(vs.model_transform.m[7], 5.0, 1e-12);
  EXPECT_NEAR(vs.model_transform.m[11], 6.0, 1e-12);
  EXPECT_NEAR(vs.model_transform.m[0], 1.0, 1e-12);
  EXPECT_NEAR(vs.model_transform.m[15], 1.0, 1e-12);
}

// ============================================================================
// Malformed writes never corrupt the last good snapshot or invoke apply.
// ============================================================================

TEST(VolrenStateSettings, MalformedStateIgnored) {
  cvc::app ctx;
  const std::string path = "t5.volren";

  int applied = 0;
  state_settings ss(ctx, path, [&](const state_settings::snapshot &) { ++applied; });

  const int goodSteps = ss.get().settings.steps;
  EXPECT_EQ(goodSteps, cvc::volren::defaults::steps);

  // Non-numeric steps: readAllFromState throws internally, apply skipped.
  treeKey(ctx, path, "steps").value(std::string("notanumber"));
  EXPECT_EQ(applied, 0);
  EXPECT_EQ(ss.get().settings.steps, goodSteps);

  // lights with 3 numbers (not a multiple of 6): rejected.
  treeKey(ctx, path, "lights").value(std::string("1,2,3"));
  EXPECT_EQ(applied, 0);
  EXPECT_TRUE(ss.get().settings.lights.empty());

  // A good write to one key cannot apply while any other key is broken --
  // the whole snapshot re-read fails as a unit.
  treeKey(ctx, path, "steps").value(77);
  EXPECT_EQ(applied, 0);
  EXPECT_EQ(ss.get().settings.steps, goodSteps);

  // Repair the last broken key: the full snapshot now parses and applies.
  treeKey(ctx, path, "lights").value(std::string(""));
  EXPECT_EQ(applied, 1);
  EXPECT_EQ(ss.get().settings.steps, 77);
  EXPECT_TRUE(ss.get().settings.lights.empty());
}

// ============================================================================
// apply_to(): the snapshot lands on a raycaster's camera/settings/volumes.
// ============================================================================

TEST(VolrenStateSettings, ApplyTo) {
  cvc::app ctx;

  cvc::volume vol(ctx, cvc::dimension(8, 8, 8), cvc::Float,
                  cvc::bounding_box(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5));
  for (int k = 0; k < 8; ++k)
    for (int j = 0; j < 8; ++j)
      for (int i = 0; i < 8; ++i)
        vol(i, j, k, double(i + j + k));

  cvc::volren::raycaster rc(ctx);
  ASSERT_EQ(rc.add_volume(vol), 0u);

  state_settings ss(ctx, "t6.volren");

  state_settings::snapshot snap;
  snap.camera.eye = {1.0, 2.0, 5.0};
  snap.camera.focal = {0.0, 1.0, 0.0};
  snap.camera.projection = cvc::volren::camera::projection_type::orthographic;
  snap.camera.parallel_scale = 3.5;
  snap.camera.width = 96;
  snap.camera.height = 64;

  snap.settings.steps = 128;
  snap.settings.two_sided_lighting = true;
  snap.settings.ambient = 0.25f;
  snap.settings.background = {0.1f, 0.2f, 0.3f};
  cvc::volren::light l;
  l.direction = {0.0, -1.0, 0.5};
  snap.settings.lights.push_back(l);

  cvc::volren::volume_settings vs;
  vs.shaded = false;
  vs.unshaded = true;
  vs.window_enabled = true;
  vs.window_min = 3.0;
  vs.window_max = 15.0;
  vs.tf.add({0.0, 0.f, 0.f, 0.f, 0.f});
  vs.tf.add({21.0, 1.f, 1.f, 1.f, 1.f});
  const std::vector<double> xlate = translationRowMajor(0.5, -1.0, 2.0);
  vs.model_transform = cvc::volren::mat4::from_row_major(xlate.data());
  snap.volumes.push_back(vs);

  ss.set(snap);
  ss.apply_to(rc);

  EXPECT_NEAR(rc.view().eye[0], 1.0, 1e-12);
  EXPECT_NEAR(rc.view().eye[1], 2.0, 1e-12);
  EXPECT_NEAR(rc.view().eye[2], 5.0, 1e-12);
  EXPECT_NEAR(rc.view().focal[1], 1.0, 1e-12);
  EXPECT_EQ(rc.view().projection, cvc::volren::camera::projection_type::orthographic);
  EXPECT_NEAR(rc.view().parallel_scale, 3.5, 1e-12);
  EXPECT_EQ(rc.view().width, 96);
  EXPECT_EQ(rc.view().height, 64);

  EXPECT_EQ(rc.settings().steps, 128);
  EXPECT_TRUE(rc.settings().two_sided_lighting);
  EXPECT_NEAR(rc.settings().ambient, 0.25f, 1e-6f);
  EXPECT_NEAR(rc.settings().background[2], 0.3f, 1e-6f);
  ASSERT_EQ(rc.settings().lights.size(), 1u);
  EXPECT_NEAR(rc.settings().lights[0].direction[1], -1.0, 1e-12);

  const cvc::volren::volume_settings &applied = rc.volume_config(0);
  EXPECT_FALSE(applied.shaded);
  EXPECT_TRUE(applied.unshaded);
  EXPECT_TRUE(applied.window_enabled);
  EXPECT_NEAR(applied.window_min, 3.0, 1e-12);
  EXPECT_NEAR(applied.window_max, 15.0, 1e-12);
  ASSERT_EQ(applied.tf.points().size(), 2u);
  EXPECT_NEAR(applied.tf.points()[1].value, 21.0, 1e-12);
  for (std::size_t i = 0; i < 16; ++i)
    EXPECT_NEAR(applied.model_transform.m[i], xlate[i], 1e-12) << "matrix[" << i << "]";
}

// ============================================================================
// supersample: both directions, and the deliberate absence of clamping.
// ============================================================================

TEST(VolrenStateSettings, SupersampleRoundTrips) {
  cvc::app ctx;
  const std::string path = "t9.volren";

  int applied = 0;
  state_settings::snapshot last;
  state_settings ss(ctx, path, [&](const state_settings::snapshot &s) {
    ++applied;
    last = s;
  });

  // Object -> state.
  state_settings::snapshot snap = ss.get();
  snap.settings.supersample = 4;
  ss.set(snap);
  EXPECT_EQ(treeKey(ctx, path, "supersample").value<int>(), 4);
  EXPECT_EQ(ss.get().settings.supersample, 4);
  EXPECT_EQ(applied, 0) << "set() must not invoke apply (object -> state only)";

  // State -> object, synchronously through the callback.
  treeKey(ctx, path, "supersample").value(3);
  EXPECT_EQ(applied, 1);
  EXPECT_EQ(ss.get().settings.supersample, 3);
  EXPECT_EQ(last.settings.supersample, 3);

  // Out-of-range values are carried, NOT clamped: render() owns the range check
  // (same as `steps`), so a bad write fails loudly at the renderer instead of
  // silently becoming a value the writer never asked for.
  treeKey(ctx, path, "supersample").value(99);
  EXPECT_EQ(ss.get().settings.supersample, 99);

  // ... and it really does reach a raycaster and get rejected there.
  cvc::volume vol(ctx, cvc::dimension(8, 8, 8), cvc::Float,
                  cvc::bounding_box(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5));
  for (int k = 0; k < 8; ++k)
    for (int j = 0; j < 8; ++j)
      for (int i = 0; i < 8; ++i)
        vol(i, j, k, double(i + j + k));
  cvc::volren::raycaster rc(ctx);
  rc.add_volume(vol);
  ss.apply_to(rc);
  EXPECT_EQ(rc.settings().supersample, 99);
  EXPECT_THROW(rc.render(), cvc::volren_error);

  treeKey(ctx, path, "supersample").value(2);
  ss.apply_to(rc);
  EXPECT_EQ(rc.settings().supersample, 2);
  EXPECT_NO_THROW(rc.render());
}

// ============================================================================
// TF encoding: the split color/opacity ramps match cvcGL VolumeNode's keys.
// ============================================================================

TEST(VolrenStateSettings, TransferFunctionEncodingMatchesVolumeNode) {
  cvc::app ctx;
  const std::string path = "t7.volren";
  state_settings ss(ctx, path);

  state_settings::snapshot snap;
  cvc::volren::volume_settings vs;
  vs.tf.add({0.0, 0.f, 0.f, 0.f, 0.f});
  vs.tf.add({10.0, 1.f, 1.f, 1.f, 1.f});
  snap.volumes.push_back(vs);
  ss.set(snap);

  // color: "value,r,g,b" per point => 0,0,0,0,10,1,1,1
  const std::vector<double> color =
      parseCsv(treeKey(ctx, path, "volumes.0.transfer_function.color").value());
  const std::vector<double> expectedColor = {0, 0, 0, 0, 10, 1, 1, 1};
  ASSERT_EQ(color.size(), expectedColor.size());
  for (std::size_t i = 0; i < color.size(); ++i)
    EXPECT_NEAR(color[i], expectedColor[i], 1e-12) << "color[" << i << "]";

  // opacity: "value,a" per point => 0,0,10,1
  const std::vector<double> opacity =
      parseCsv(treeKey(ctx, path, "volumes.0.transfer_function.opacity").value());
  const std::vector<double> expectedOpacity = {0, 0, 10, 1};
  ASSERT_EQ(opacity.size(), expectedOpacity.size());
  for (std::size_t i = 0; i < opacity.size(); ++i)
    EXPECT_NEAR(opacity[i], expectedOpacity[i], 1e-12) << "opacity[" << i << "]";
}

// ============================================================================
// Volumetric shadows (shadow.h): the seven keys, both directions.
// ============================================================================

TEST(VolrenStateSettings, ShadowSettingsRoundTrip) {
  cvc::app ctx;
  const std::string path = "t20.volren";
  state_settings ss(ctx, path);

  // Object -> state.
  state_settings::snapshot snap = ss.get();
  snap.settings.lights.resize(3);
  snap.settings.shadows.enabled = true;
  snap.settings.shadows.lights = {0, 2};
  snap.settings.shadows.resolution = 1024;
  snap.settings.shadows.strength = 0.625f;
  snap.settings.shadows.bias_scale = 2.5f;
  snap.settings.shadows.slope_scale = 0.25f;
  snap.settings.shadows.min_occluder_opacity = 0.75f;
  ss.set(snap);

  EXPECT_EQ(treeKey(ctx, path, "shadows.enabled").value<int>(), 1);
  EXPECT_EQ(treeKey(ctx, path, "shadows.lights").value(), "0,2");
  EXPECT_EQ(treeKey(ctx, path, "shadows.resolution").value<int>(), 1024);
  EXPECT_NEAR(treeKey(ctx, path, "shadows.strength").value<double>(), 0.625, 1e-12);
  EXPECT_NEAR(treeKey(ctx, path, "shadows.bias_scale").value<double>(), 2.5, 1e-12);
  EXPECT_NEAR(treeKey(ctx, path, "shadows.slope_scale").value<double>(), 0.25, 1e-12);
  EXPECT_NEAR(treeKey(ctx, path, "shadows.min_occluder_opacity").value<double>(), 0.75, 1e-12);

  // State -> object, through an external write on each key.
  cvc::state::instance(ctx)(path + ".shadows.strength").value(0.5);
  EXPECT_NEAR(double(ss.get().settings.shadows.strength), 0.5, 1e-6);
  cvc::state::instance(ctx)(path + ".shadows.bias_scale").value(3.0);
  EXPECT_NEAR(double(ss.get().settings.shadows.bias_scale), 3.0, 1e-6);
  cvc::state::instance(ctx)(path + ".shadows.slope_scale").value(0.125);
  EXPECT_NEAR(double(ss.get().settings.shadows.slope_scale), 0.125, 1e-6);
  cvc::state::instance(ctx)(path + ".shadows.min_occluder_opacity").value(0.2);
  EXPECT_NEAR(double(ss.get().settings.shadows.min_occluder_opacity), 0.2, 1e-6);
  cvc::state::instance(ctx)(path + ".shadows.enabled").value(0);
  EXPECT_FALSE(ss.get().settings.shadows.enabled);

  // "" means "every light casts" and must round-trip as an EMPTY list, not as
  // a one-element list holding a parsed zero.
  cvc::state::instance(ctx)(path + ".shadows.lights").value(std::string(""));
  EXPECT_TRUE(ss.get().settings.shadows.lights.empty());
  cvc::state::instance(ctx)(path + ".shadows.lights").value(std::string("1,0,2"));
  EXPECT_EQ(ss.get().settings.shadows.lights, (std::vector<int>{1, 0, 2}));

  // Malformed state leaves the object alone (the readAllFromState contract):
  // a light index is an index, so junk, a negative, and a fraction are all
  // rejected rather than rounded into something that looks like it worked.
  for (const char *bad : {"0,x", "-1", "0.5", "1,,2"}) {
    cvc::state::instance(ctx)(path + ".shadows.lights").value(std::string(bad));
    EXPECT_EQ(ss.get().settings.shadows.lights, (std::vector<int>{1, 0, 2}))
        << "malformed shadows.lights \"" << bad << "\" was accepted";
  }

  // Malformed state is STICKY until it is repaired: readAllFromState is
  // all-or-nothing, so while shadows.lights is junk no later write to any key
  // lands either.  Repair it before moving on.
  cvc::state::instance(ctx)(path + ".shadows.lights").value(std::string("1,0,2"));

  // resolution CLAMPS on read (the `threads` convention) rather than being
  // rejected: a light-view raster is an implementation resource with a
  // defensible range, not a contract the caller can violate meaningfully.
  cvc::state::instance(ctx)(path + ".shadows.resolution").value(1);
  EXPECT_EQ(ss.get().settings.shadows.resolution, cvc::volren::limits::min_shadow_resolution);
  cvc::state::instance(ctx)(path + ".shadows.resolution").value(1 << 24);
  EXPECT_EQ(ss.get().settings.shadows.resolution, cvc::volren::limits::max_raster_dim);
}

TEST(VolrenStateSettings, ShadowSettingsReachTheRaycasterThroughApplyTo) {
  cvc::app ctx;
  const std::string path = "t21.volren";
  state_settings ss(ctx, path);

  cvc::state::instance(ctx)(path + ".lights").value(std::string("1,1,1,0,0,1"));
  cvc::state::instance(ctx)(path + ".shadows.enabled").value(1);
  cvc::state::instance(ctx)(path + ".shadows.resolution").value(128);
  cvc::state::instance(ctx)(path + ".shadows.strength").value(0.5);

  cvc::volume vol(ctx, cvc::dimension(8, 8, 8), cvc::Float,
                  cvc::bounding_box(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5));
  for (unsigned k = 0; k < 8; ++k)
    for (unsigned j = 0; j < 8; ++j)
      for (unsigned i = 0; i < 8; ++i)
        vol(i, j, k, double(k));

  cvc::volren::raycaster rc(ctx);
  rc.add_volume(vol);
  ss.apply_to(rc);
  EXPECT_TRUE(rc.settings().shadows.enabled);
  EXPECT_EQ(rc.settings().shadows.resolution, 128);
  EXPECT_NEAR(double(rc.settings().shadows.strength), 0.5, 1e-6);
}
