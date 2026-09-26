// cvcgl_nav_stats_publish — publish_nav_stats writes a base episode into <prefix>.nav_stats.* on
// the value lane, and the values read back correctly after a flush. Pins the path layout + the
// per-convoy formation aggregate the ImGui panel / DSL will read (SPEC 1, CVC-DBG/cvcdbg
// stats-statetree spec).
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/gl/nav_stats_publish.h>
#include <cvc/gl/state_publisher.h>
#include <cvc/volume/volume.h>
#include <string>

static int failures = 0;
static void ck(bool ok, const char *w) {
  if (!ok) {
    std::printf("FAIL: %s\n", w);
    ++failures;
  }
}

int main() {
  cvc::app app;
  cvc::gl::state_publisher pub(app);

  cvc::nav::episode_nav_stats e;
  e.scene_id = "austin";
  e.seed = 7;
  e.n_vehicles = 2;
  e.dt_s = 0.1;
  e.arrived = 1;
  e.success = false;
  e.makespan_s = 12.5;
  e.min_sep_m = 5.0;
  e.coverage.explored_frac = 0.625;
  e.coverage.phantom_frac = 0.25;
  cvc::nav::veh_nav_stats v0; // anchor (arrived)
  v0.veh_index = 0;
  v0.convoy_id = 0;
  v0.formation_parent = -1;
  v0.arrived = true;
  cvc::nav::veh_nav_stats v1; // follower, in-slot
  v1.veh_index = 1;
  v1.convoy_id = 0;
  v1.formation_parent = 0;
  v1.formation_arrived = true;
  v1.slot_error_mean_m = 1.5;
  v1.closest_approach_m = 3.0;
  v1.stall_steps = 4;
  v1.sense_flips = 8;
  e.per_vehicle = {v0, v1};

  cvc::gl::publish_nav_stats(pub, "scene", e, /*tick*/ 42);
  pub.flush();

  auto &st = cvc::state::instance(app);
  ck(st("scene.nav_stats.meta.scene_id").value<std::string>() == "austin", "meta scene_id");
  ck(st("scene.nav_stats.meta.tick").value<int>() == 42, "meta tick");
  ck(st("scene.nav_stats.meta.n_vehicles").value<int>() == 2, "meta n_vehicles");
  ck(std::fabs(st("scene.nav_stats.episode.makespan_s").value<double>() - 12.5) < 1e-6,
     "episode makespan");
  ck(st("scene.nav_stats.episode.success").value<int>() == 0, "episode success=0");
  ck(std::fabs(st("scene.nav_stats.coverage.explored_frac").value<double>() - 0.625) < 1e-6,
     "coverage explored");
  ck(std::fabs(st("scene.nav_stats.coverage.phantom_frac").value<double>() - 0.25) < 1e-6,
     "coverage phantom");
  // per-vehicle
  ck(st("scene.nav_stats.veh.1.formation_parent").value<int>() == 0, "veh1 parent");
  ck(std::fabs(st("scene.nav_stats.veh.1.slot_error_mean_m").value<double>() - 1.5) < 1e-6,
     "veh1 slot err");
  ck(st("scene.nav_stats.veh.1.formation_arrived").value<int>() == 1, "veh1 in-slot");
  ck(st("scene.nav_stats.veh.1.sense_flips").value<int>() == 8, "veh1 sense_flips");
  ck(std::fabs(st("scene.nav_stats.veh.1.closest_approach_m").value<double>() - 3.0) < 1e-6,
     "veh1 closest");
  ck(st("scene.nav_stats.veh.0.formation_parent").value<int>() == -1, "veh0 anchor parent -1");
  // per-convoy formation: convoy 0 has 1 follower, in-slot, anchor arrived -> form_arrival 1.
  ck(st("scene.nav_stats.formation.0.followers").value<int>() == 1, "convoy0 followers");
  ck(std::fabs(st("scene.nav_stats.formation.0.slot_error_mean_m").value<double>() - 1.5) < 1e-6,
     "convoy0 slot err");
  ck(st("scene.nav_stats.formation.0.form_arrival").value<int>() == 1, "convoy0 form_arrival");

  // complete flag round-trips.
  cvc::gl::nav_stats_publish_params p;
  p.complete = true;
  cvc::gl::publish_nav_stats(pub, "scene", e, 43, p);
  pub.flush();
  ck(st("scene.nav_stats.meta.complete").value<int>() == 1, "meta complete=1");
  ck(st("scene.nav_stats.meta.tick").value<int>() == 43,
     "meta tick advanced (coalesced last value)");

  // per_vehicle_every stride: over 3 vehicles, stride 2 publishes veh 0 and 2, skips 1 (fresh
  // prefix).
  cvc::nav::episode_nav_stats e3;
  e3.n_vehicles = 3;
  cvc::nav::veh_nav_stats s0, s1, s2;
  s0.veh_index = 0;
  s1.veh_index = 1;
  s2.veh_index = 2;
  e3.per_vehicle = {s0, s1, s2};
  cvc::gl::nav_stats_publish_params sp;
  sp.per_vehicle_every = 2;
  cvc::gl::publish_nav_stats(pub, "strd", e3, 1, sp);
  pub.flush();
  ck(st("strd.nav_stats.veh.0.arrived").initialized(), "stride: veh0 published");
  ck(!st("strd.nav_stats.veh.1.arrived").initialized(), "stride: veh1 skipped");
  ck(st("strd.nav_stats.veh.2.arrived").initialized(), "stride: veh2 published");

  // anchor-not-arrived convoy: a follower in-slot but the anchor missed the goal -> form_arrival 0.
  cvc::nav::episode_nav_stats e4;
  e4.n_vehicles = 2;
  cvc::nav::veh_nav_stats an, fo;
  an.veh_index = 0;
  an.convoy_id = 1;
  an.formation_parent = -1;
  an.arrived = false; // anchor did NOT reach the objective
  fo.veh_index = 1;
  fo.convoy_id = 1;
  fo.formation_parent = 0;
  fo.formation_arrived = true; // follower is in-slot
  e4.per_vehicle = {an, fo};
  cvc::gl::publish_nav_stats(pub, "anch", e4, 1);
  pub.flush();
  ck(st("anch.nav_stats.formation.1.form_arrival").value<int>() == 0,
     "anchor-not-arrived -> form_arrival 0");
  ck(st("anch.nav_stats.formation.1.followers").value<int>() == 1, "anch convoy1 one follower");

  // ── raster bridge: publish_nav_rasters wraps planes into the data() lane + version-gates ──
  const int rows = 2, cols = 2, planes = 2;
  std::uint8_t truthR[4] = {0, 1, 0, 0};
  std::uint8_t beliefR[8] = {10, 20, 30, 40, /*plane1*/ 50, 60, 70, 80};
  std::uint8_t everseenR[8] = {1, 1, 1, 1, 1, 1, 0, 0};
  std::uint8_t lastvisR[8] = {1, 0, 0, 0, 0, 0, 0, 1};
  int versions[2] = {0, 5};
  cvc::gl::nav_raster_dims rd;
  rd.rows = rows;
  rd.cols = cols;
  rd.planes = planes;
  rd.min_x = -10;
  rd.min_y = -10;
  rd.max_x = 10;
  rd.max_y = 10;
  cvc::gl::nav_raster_pub_state rst;
  cvc::gl::publish_nav_rasters(app, pub, "rast", rd, truthR, beliefR, everseenR, lastvisR, versions,
                               rst);
  pub.flush();
  ck(st("rast.nav_stats.rasters.dims.rows").value<int>() == 2, "raster dims rows");
  ck(st("rast.nav_stats.rasters.dims.planes").value<int>() == 2, "raster dims planes");
  ck(st("rast.nav_stats.rasters.truth.version").value<int>() == 0, "truth version 0");
  ck(st("rast.nav_stats.rasters.plane.1.version").value<int>() == 5, "plane1 version 5");
  ck(st("rast.nav_stats.rasters.plane.0.belief.data").isData<cvc::volume>(),
     "plane0 belief is a cvc::volume handle");
  {
    const cvc::volume vol = st("rast.nav_stats.rasters.plane.0.belief.data").data<cvc::volume>();
    ck(vol.XDim() == 2 && vol.YDim() == 2 && vol.ZDim() == 1, "plane0 belief z=1 dims");
    // (i=col, j=row): row-major raster [10,20 / 30,40] -> vol(0,0,0)=10, vol(1,0,0)=20,
    // vol(0,1,0)=30.
    ck((int)vol(0, 0, 0) == 10 && (int)vol(1, 0, 0) == 20 && (int)vol(0, 1, 0) == 30,
       "plane0 belief voxels round-trip");
  }
  ck(st("rast.nav_stats.rasters.truth.data").isData<cvc::volume>(),
     "truth is a cvc::volume handle");

  // version gate: plane 0 unchanged (v 0), plane 1 bumped (v 6) -> only plane 1 re-published.
  int versions2[2] = {0, 6};
  cvc::gl::publish_nav_rasters(app, pub, "rast", rd, truthR, beliefR, everseenR, lastvisR,
                               versions2, rst);
  pub.flush();
  ck(st("rast.nav_stats.rasters.plane.1.version").value<int>() == 6, "plane1 version bumped to 6");
  ck(st("rast.nav_stats.rasters.plane.0.version").value<int>() == 0,
     "plane0 version still 0 (gated/skipped)");

  std::printf(failures ? "cvcgl_nav_stats_publish: %d FAILURES\n" : "cvcgl_nav_stats_publish: OK\n",
              failures);
  return failures ? 1 : 0;
}
