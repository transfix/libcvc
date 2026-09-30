// Every field of a multi-field StageLighting setter must stick.
//
// The rig mirrors its settings into cvc::state, and its own change callback
// re-reads EVERY field from state (so a UI edit to one key flows back in). The
// multi-field setters used to write their fields one at a time with that
// callback live: the first write re-read the rest from state -- still holding the
// OLD values -- and the remaining writes then stored those old values right back.
// setStage(1, 2, 3, 10) left the stage at (1, 0, 0, <old radius>), setWash kept
// only the intensity, and applyPreset() kept only the first changed field of the
// preset. Nothing here needs a GL context: the rig is headless on a SceneGraph.
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state_object.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/StageLighting.h>
#include <string>

using cvc::gl::SceneGraph;
using cvc::gl::StageLighting;

static int fails = 0;
static void check(bool ok, const std::string &w) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", w.c_str());
  if (!ok)
    ++fails;
}

// A passive probe rooted at the rig's path (see cvcgl_state_binding for why the
// path is derived independently and why the probe is unthreaded).
class Peer : public cvc::state_object<Peer> {
public:
  Peer(cvc::app &c, const std::string &p) : cvc::state_object<Peer>(c, p) {
    this->setInstanceThreading(false);
  }
  double num(const std::string &k) {
    const std::string v = getState(k).value();
    return v.empty() ? std::nan("") : std::stod(v);
  }
  template <class T> void wr(const std::string &k, T v) { getState(k).value(v); }
};

static bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

static void expect_state(Peer &L, const char *key, double want) {
  const double got = L.num(key);
  check(near(got, want),
        std::string(key) + " = " + std::to_string(got) + " (want " + std::to_string(want) + ")");
}

static void expect_stage(StageLighting &rig, double x, double y, double z, double r,
                         const std::string &what) {
  double cx, cy, cz, cr;
  rig.stage(cx, cy, cz, cr);
  check(near(cx, x) && near(cy, y) && near(cz, z) && near(cr, r),
        what + ": stage() = (" + std::to_string(cx) + ", " + std::to_string(cy) + ", " +
            std::to_string(cz) + ", " + std::to_string(cr) + ")");
}

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  SceneGraph sg(app, "stg");
  StageLighting rig(sg);
  Peer L(app, StageLighting::sceneStatePath(sg.getStatePrefix()));

  std::printf("== setStage ==\n");
  // Every coordinate differs from the default (0, 0, 0, 10), so a field that
  // falls back to its old value cannot pass by coincidence.
  rig.setStage(1.0, 2.0, 3.0, 7.0);
  expect_stage(rig, 1.0, 2.0, 3.0, 7.0, "object");
  expect_state(L, "stage_x", 1.0);
  expect_state(L, "stage_y", 2.0);
  expect_state(L, "stage_z", 3.0);
  expect_state(L, "stage_radius", 7.0);

  std::printf("== frameBounds (routes through setStage) ==\n");
  rig.frameBounds(-4.0, -2.0, 0.0, 4.0, 2.0, 10.0);
  // centre (0, 0), z = minZ + 0.15 * height, radius = half the footprint diagonal
  expect_stage(rig, 0.0, 0.0, 1.5, 0.5 * std::sqrt(8.0 * 8.0 + 4.0 * 4.0), "frameBounds");

  std::printf("== setKey ==\n");
  rig.setKey(1.2, 45.0, 30.0, 35.0);
  expect_state(L, "key_intensity", 1.2);
  expect_state(L, "key_azimuth", 45.0);
  expect_state(L, "key_elevation", 30.0);
  expect_state(L, "key_cone", 35.0);

  std::printf("== setWash ==\n");
  rig.setWash(0.7, 5, 2.5);
  expect_state(L, "wash_intensity", 0.7);
  expect_state(L, "wash_count", 5.0);
  expect_state(L, "wash_height", 2.5);

  std::printf("== applyPreset(Dramatic) ==\n");
  rig.applyPreset(StageLighting::Preset::Dramatic);
  expect_state(L, "key_intensity", 1.35);
  expect_state(L, "key_elevation", 30.0);
  expect_state(L, "key_cone", 22.0);
  expect_state(L, "fill_intensity", 0.08);
  expect_state(L, "back_intensity", 0.85);
  expect_state(L, "wash_intensity", 0.05);
  expect_state(L, "wash_count", 2.0);
  expect_state(L, "ambient", 0.08);
  expect_state(L, "warm_key", 0.55);

  std::printf("== state -> object still flows in ==\n");
  // The setters silence the rig's OWN callback while mirroring; an outside edit
  // (a bound UI slider) must still reach the object.
  L.wr("stage_radius", 12.5);
  double cx, cy, cz, cr;
  rig.stage(cx, cy, cz, cr);
  check(near(cr, 12.5), "external stage_radius edit reaches the rig: " + std::to_string(cr));

  std::printf(fails ? "FAILED (%d)\n" : "OK\n", fails);
  return fails ? 1 : 0;
}
