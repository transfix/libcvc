// nav_abi_smoke — a headless (no-GL) consistency guard the cvcgl-examples recipe
// runs after building the demos.
//
// The nav demos compile against the IN-TREE cvc/nav headers but link the SHARED
// libcvc from the deps prefix (the published SDK bundle). `sim_world` embeds
// `config` (which holds `veh_params`) and `material_config` BY VALUE, so if those
// structs grew in the headers while the linked libcvc is an older build, then
// `sim_world::from_occupancy` — which returns a `sim_world` by value — writes a
// wrong-sized object into the caller's storage. The result is garbage
// `size()`/`planes()` and a smashed return address: the demo segfaults at startup
// with no useful error, and (worse) a stale published bundle would ship that way.
//
// This program exercises exactly that construction path with no renderer and
// asserts the read-back is sane. Run post-build by the recipe, it turns a
// silent, hard-to-debug ABI skew into a loud build failure that names the fix:
// rebuild/republish libcvc so the examples link a matching version. It is a
// build-time check only and is not installed.

#include <cstdio>
#include <cstdlib>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h> // city_scene, training_scene
#include <cvc/nav/sim_world.h>

using namespace cvc::nav;

namespace {

// Build a sim_world the way nav_city_swarm does at startup and check the
// renderer-facing read-back. Returns 0 on success, 1 on an ABI mismatch.
int check(int n, sim_world::belief_mode mode, int clusters) {
  training_scene ts = city_scene(96);

  sim_world::config cfg;
  cfg.rows = ts.rows;
  cfg.cols = ts.cols;
  cfg.min_x = ts.min_x;
  cfg.min_y = ts.min_y;
  cfg.max_x = ts.max_x;
  cfg.max_y = ts.max_y;
  cfg.cx = ts.cx;
  cfg.cy = ts.cy;
  cfg.scale = ts.scale;
  cfg.veh.rr = ts.rr;
  cfg.veh.d_hat = ts.d_hat;
  cfg.veh.dt = ts.dt;
  cfg.veh.vmax = ts.vmax;

  sim_world w = sim_world::from_occupancy(cfg, ts.occ.data(), coef_mlp::default_biased(), n,
                                          /*seed=*/7, mode, clusters);

  const int got_n = w.size();
  const int got_p = w.planes();
  const int p_max = (clusters > 1) ? clusters : 1; // shared => 1; clustered => <= K
  if (got_n != n || got_p < 1 || got_p > p_max) {
    std::fprintf(stderr,
                 "nav_abi_smoke: ABI MISMATCH — sim_world::from_occupancy(N=%d) returned size()=%d "
                 "planes()=%d (expected size=%d, planes in [1,%d]).\n"
                 "  The linked libcvc's sim_world layout disagrees with these headers — sim_world "
                 "embeds config+material_config BY VALUE, so a from_occupancy() returned by value "
                 "corrupts the caller when the two disagree.\n"
                 "  FIX: rebuild/republish libcvc so the examples link a matching version (see "
                 "docs — build libcvc from source into the prefix before the examples).\n",
                 n, got_n, got_p, n, p_max);
    // Exit immediately: `w` holds garbage from the wrong-sized by-value return,
    // so letting it destruct would crash with a noisy exit code and bury the
    // message above. A clean non-zero exit is what the recipe checks for.
    std::fflush(stderr);
    std::_Exit(1);
  }
  return 0;
}

} // namespace

int main() {
  check(64, sim_world::belief_mode::shared, 1);
  check(512, sim_world::belief_mode::clustered, 8);
  std::printf("nav_abi_smoke: OK — sim_world ABI matches the linked libcvc.\n");
  return 0;
}
