/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_swarm_demo — the whole GRL-SNAM navigation swarm from PURE C++: no Python,
// no libtorch. A rasterized scene (here a synthetic walled grid; in a real app
// the occupancy of your terrain / lsystem_forest) drives thousands of vehicles
// reacting to it. This is the template a cvcGL scene copies to add navigating
// agents — feed sim_world::snapshot() poses into per-agent GeometryNodes each
// frame (or run it on a sim_thread and read() lock-free off the render thread).
//
// Build (standalone, against an installed libcvc):
//   g++ -std=c++17 nav_swarm_demo.cpp -o nav_swarm_demo \
//       -I$PREFIX/include -L$PREFIX/lib -lcvc && ./nav_swarm_demo
// or inside the tree: enable -DCVC_BUILD_NAV_EXAMPLE=ON.

#include <cstdint>
#include <cstdio>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/sim_world.h>
#include <vector>

int main() {
  using namespace cvc::nav;

  // 1. A scene as an occupancy grid (0 = free, nonzero = obstacle). Real apps
  //    rasterize their terrain / trees here; this is a bordered room with a bar.
  const int R = 192, C = 192;
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  auto at = [&](int r, int c) -> std::uint8_t & { return occ[r * C + c]; };
  for (int r = 0; r < R; ++r)
    for (int c = 0; c < C; ++c)
      if (r == 0 || c == 0 || r == R - 1 || c == C - 1)
        at(r, c) = 1; // walls
  for (int r = R / 3; r < 2 * R / 3; ++r)
    at(r, C / 2) = 1; // a bar to route around

  // 2. Configure the world (bounds in metres, the SDFField scale/center).
  sim_world::config cfg;
  cfg.rows = R;
  cfg.cols = C;
  cfg.min_x = -400;
  cfg.min_y = -400;
  cfg.max_x = 400;
  cfg.max_y = 400;
  cfg.cx = 0;
  cfg.cy = 0;
  cfg.scale = 0.02; // world -> normalized
  cfg.veh.rr = 3.0f;
  cfg.veh.d_hat = 7.0f;
  cfg.veh.dt = 0.06f;
  cfg.veh.vmax = 0.9f;
  cfg.veh.nsub = 1;
  cfg.reach_tol = 0.8f;
  cfg.freeze_sense = true; // known map (no fog); flip off for discover-as-you-go

  // 3. Build the swarm — a default bias-initialized policy needs NO weights file
  //    (load a trained coef_mlp::load("coef_mlp.cvcnav") for learned driving).
  const int N = 2000;
  sim_world world = sim_world::from_occupancy(cfg, occ.data(), coef_mlp::default_biased(), N, 42);

  // 4. Run. Each tick: snapshot() gives world poses to hand a renderer.
  std::vector<float> pos(2 * N), heading(N), speed(N);
  std::vector<int> mode(N);
  std::vector<std::uint8_t> reached(N);
  for (int t = 0; t < 400; ++t) {
    world.step(); // threaded internally
    if (t % 100 == 99) {
      world.snapshot(pos.data(), heading.data(), speed.data(), mode.data(), reached.data());
      int done = 0;
      for (int i = 0; i < N; ++i)
        done += reached[i];
      std::printf("tick %3d: %4d/%d agents reached  (agent0 @ %.1f, %.1f  heading %.2f)\n", t + 1,
                  done, N, pos[0], pos[1], heading[0]);
    }
  }
  std::puts("done — pure C++, zero torch/Python.");
  return 0;
}
