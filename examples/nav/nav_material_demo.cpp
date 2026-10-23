/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_material_demo — material-aware navigation from PURE C++: the same
// sim_world swarm as nav_swarm_demo, but the terrain carries SEMANTICS — a
// soft material-risk field (mud) and a hard-hazard lake (water: lethal, not
// geometry) — and the drive feels them (docs/NAV_MATERIAL.md).
//
// The demo runs the SAME lane of vehicles twice, material off then on, prints
// the mud exposure / hazard-entry / arrival stats side by side, and writes
// nav_material_demo_{off,on}.ppm — terrain shading with every trajectory
// drawn over it — so the detours are visible with zero dependencies.
//
// With Jolt in the build it runs the lanes twice more on Humvees (cvc/physics/plant_host.h), at
// their real size and limits, beside nav's kinematic bicycle; the first argument runs one plant
// alone (kinematic, humvee, roboracer). The material layer was tuned on the bicycle, and on the
// Humvee it does not yet keep the lanes out of the lake (it prints how far it misses).
//
// Build (standalone, against an installed libcvc):
//   g++ -std=c++17 nav_material_demo.cpp -o nav_material_demo \
//       -I$PREFIX/include -L$PREFIX/lib -lcvc && ./nav_material_demo
// or inside the tree (the repo's examples/ tree):
//   cmake -DCVC_BUILD_CVCGL=ON -DCVC_BUILD_EXAMPLES=ON … && cmake --build build
//         --target nav_material_demo

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/material.h>
#include <cvc/nav/sim_world.h>
#include <cvc/physics/plant_host.h>
#include <string>
#include <vector>

using namespace cvc::nav;

namespace {

constexpr int GRID = 128;
constexpr double WORLD = 100.0; // bounds ±WORLD metres
constexpr double SCALE = 0.05;  // world -> normalized
constexpr int N_AGENTS = 12;
constexpr int MAX_TICKS = 900;

int cell_of(float w) {
  const double c = (static_cast<double>(w) + WORLD) / (2.0 * WORLD) * (GRID - 1);
  return std::min(std::max(static_cast<int>(std::rint(c)), 0), GRID - 1);
}

struct run_stats {
  int reached = 0;
  int hazard_entries = 0;
  double mud_exposure = 0.0;
  std::vector<std::vector<float>> traces; // per agent, xy pairs (world)
};

run_stats run(const std::vector<std::uint8_t> &occ, const std::vector<float> &risk,
              const std::vector<std::uint8_t> &hard, bool with_material, const std::string &plant) {
  sim_world::config cfg;
  cfg.rows = GRID;
  cfg.cols = GRID;
  cfg.min_x = -WORLD;
  cfg.min_y = -WORLD;
  cfg.max_x = WORLD;
  cfg.max_y = WORLD;
  cfg.cx = 0;
  cfg.cy = 0;
  cfg.scale = SCALE;
  cfg.veh.rr = 0.15f;
  cfg.veh.d_hat = 0.35f;
  cfg.veh.dt = 0.06f;
  cfg.veh.vmax = 0.9f;
  cfg.veh.nsub = 1;
  cfg.reach_tol = 0.8f;
  cfg.freeze_sense = true; // known static map — the deployment path

  // A lane of vehicles crossing left -> right, spread over y.
  std::vector<float> o(2 * N_AGENTS), goal(2 * N_AGENTS), color(3 * N_AGENTS, 1.0f);
  for (int i = 0; i < N_AGENTS; ++i) {
    const float y = -55.0f + 10.0f * static_cast<float>(i);
    o[2 * i] = static_cast<float>(-80.0 * SCALE);
    o[2 * i + 1] = static_cast<float>(y * SCALE);
    goal[2 * i] = static_cast<float>(80.0 * SCALE);
    goal[2 * i + 1] = static_cast<float>(y * SCALE);
  }

  sim_world world(cfg, occ.data(), occ.data(), coef_mlp::default_biased(), o.data(), goal.data(),
                  color.data(), N_AGENTS);
  if (with_material) {
    material_config mc; // GRL-SNAM sim-frame defaults (lam 0.5/1.0, k 1.25, d_hat 12 m)
    mc.gate.horizon_cells = 12;
    world.set_material(risk.data(), hard.data(), mc);
  }

  cvc::physics::plant_host host; // what the vehicles drive on
  host.use(world, plant);

  run_stats st;
  st.traces.assign(N_AGENTS, {});
  std::vector<float> pos(2 * N_AGENTS), heading(N_AGENTS), speed(N_AGENTS);
  std::vector<int> mode(N_AGENTS);
  std::vector<std::uint8_t> reached(N_AGENTS);
  for (int t = 0; t < MAX_TICKS; ++t) {
    world.step();
    world.snapshot(pos.data(), heading.data(), speed.data(), mode.data(), reached.data());
    int done = 0;
    for (int i = 0; i < N_AGENTS; ++i) {
      st.traces[i].push_back(pos[2 * i]);
      st.traces[i].push_back(pos[2 * i + 1]);
      const int r = cell_of(pos[2 * i + 1]);
      const int c = cell_of(pos[2 * i]);
      if (!reached[i]) {
        st.mud_exposure += risk[r * GRID + c];
        st.hazard_entries += hard[r * GRID + c];
      }
      done += reached[i];
    }
    if (done == N_AGENTS)
      break;
  }
  for (int i = 0; i < N_AGENTS; ++i)
    st.reached += reached[i];
  return st;
}

void write_ppm(const std::string &path, const std::vector<std::uint8_t> &occ,
               const std::vector<float> &risk, const std::vector<std::uint8_t> &hard,
               const run_stats &st) {
  const int S = 5; // upscale
  const int W = GRID * S, H = GRID * S;
  std::vector<std::uint8_t> img(static_cast<std::size_t>(3) * W * H);
  for (int r = 0; r < H; ++r)
    for (int c = 0; c < W; ++c) {
      const int gr = r / S, gc = c / S;
      const std::size_t gi = static_cast<std::size_t>(gr) * GRID + gc;
      std::uint8_t R = 235, G = 235, B = 225; // open ground
      const float rk = risk[gi];
      if (rk > 0.02f) { // mud: darker brown with risk
        R = static_cast<std::uint8_t>(200 - 90 * rk);
        G = static_cast<std::uint8_t>(180 - 110 * rk);
        B = static_cast<std::uint8_t>(140 - 100 * rk);
      }
      if (hard[gi]) { // water
        R = 70;
        G = 110;
        B = 200;
      }
      if (occ[gi]) { // walls
        R = G = B = 30;
      }
      std::uint8_t *px = &img[(static_cast<std::size_t>(r) * W + c) * 3];
      px[0] = R;
      px[1] = G;
      px[2] = B;
    }
  // trajectories: one hue per agent, drawn as 2x2 dots (upscaled world -> px)
  for (int i = 0; i < static_cast<int>(st.traces.size()); ++i) {
    const float hue = static_cast<float>(i) / static_cast<float>(st.traces.size());
    const std::uint8_t R = static_cast<std::uint8_t>(60 + 180 * hue);
    const std::uint8_t G = static_cast<std::uint8_t>(200 - 160 * hue);
    const std::uint8_t B = 40;
    const std::vector<float> &tr = st.traces[i];
    for (std::size_t k = 0; k + 1 < tr.size(); k += 2) {
      const int c = static_cast<int>((tr[k] + WORLD) / (2.0 * WORLD) * (W - 1));
      const int r = static_cast<int>((tr[k + 1] + WORLD) / (2.0 * WORLD) * (H - 1));
      for (int dr = 0; dr < 2; ++dr)
        for (int dc = 0; dc < 2; ++dc) {
          const int rr = std::min(std::max(r + dr, 0), H - 1);
          const int cc = std::min(std::max(c + dc, 0), W - 1);
          std::uint8_t *px = &img[(static_cast<std::size_t>(rr) * W + cc) * 3];
          px[0] = R;
          px[1] = G;
          px[2] = B;
        }
    }
  }
  FILE *f = std::fopen(path.c_str(), "wb");
  if (!f) {
    std::printf("could not write %s\n", path.c_str());
    return;
  }
  std::fprintf(f, "P6\n%d %d\n255\n", W, H);
  std::fwrite(img.data(), 1, img.size(), f);
  std::fclose(f);
  std::printf("wrote %s\n", path.c_str());
}

} // namespace

int main(int argc, char **argv) {
  const std::string plant = argc > 1                                ? argv[1]
                            : cvc::physics::plant_host::available() ? "humvee"
                                                                    : "kinematic";
  // 1. Geometry: just the border walls — everything else is SEMANTICS.
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(GRID) * GRID, 0);
  for (int r = 0; r < GRID; ++r)
    for (int c = 0; c < GRID; ++c)
      if (r == 0 || c == 0 || r == GRID - 1 || c == GRID - 1)
        occ[r * GRID + c] = 1;

  // 2. Material: mud blobs sitting on the northern lanes, and a hazard lake
  //    grazing the southern ones. All raw risk; material_build smooths it.
  std::vector<float> risk(static_cast<std::size_t>(GRID) * GRID, 0.0f);
  std::vector<std::uint8_t> hard(static_cast<std::size_t>(GRID) * GRID, 0);
  auto blob = [&](double wy, double wx, double rad_m, float value, bool is_hard) {
    for (int r = 0; r < GRID; ++r)
      for (int c = 0; c < GRID; ++c) {
        const double y = -WORLD + r / static_cast<double>(GRID - 1) * 2.0 * WORLD;
        const double x = -WORLD + c / static_cast<double>(GRID - 1) * 2.0 * WORLD;
        if ((y - wy) * (y - wy) + (x - wx) * (x - wx) <= rad_m * rad_m) {
          risk[r * GRID + c] = std::max(risk[r * GRID + c], value);
          if (is_hard)
            hard[r * GRID + c] = 1;
        }
      }
  };
  // Blob centres sit a few metres OFF the lanes: the reactive field deflects
  // around lateral gradients; a blob EXACTLY centred on a lane is the
  // documented potential-field limitation (zero lateral component dead-ahead —
  // route-level avoidance is the planner's job; see docs/NAV_MATERIAL.md).
  blob(39.0, -10.0, 16.0, 0.95f, false); // mud over the y=25..45 lanes
  blob(9.0, 30.0, 13.0, 0.9f, false);    // mud over the y=-5..15 lanes
  blob(-30.0, 5.0, 15.0, 1.0f, true);    // water lake over the y=-45..-25 lanes

  // Each plant's lane, material off then on.
  std::vector<std::string> plants = {plant};
  if (argc <= 1 && plant != "kinematic")
    plants.insert(plants.begin(), "kinematic"); // the bicycle the material layer was tuned on
  std::vector<run_stats> off(plants.size()), on(plants.size());
  try {
    for (std::size_t k = 0; k < plants.size(); ++k) {
      off[k] = run(occ, risk, hard, false, plants[k]);
      on[k] = run(occ, risk, hard, true, plants[k]);
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "nav_material_demo: %s\n", e.what());
    return 2;
  }

  std::printf("%-22s", "");
  for (const std::string &p : plants)
    std::printf("  %-20s", p.c_str());
  std::printf("\n%-22s", "material");
  for (std::size_t k = 0; k < plants.size(); ++k)
    std::printf("  %8s  %8s  ", "OFF", "ON");
  std::printf("\n%-22s", "agents reached");
  for (std::size_t k = 0; k < plants.size(); ++k)
    std::printf("  %5d/%-2d  %5d/%-2d  ", off[k].reached, N_AGENTS, on[k].reached, N_AGENTS);
  std::printf("\n%-22s", "mud exposure (sum r~)");
  for (std::size_t k = 0; k < plants.size(); ++k)
    std::printf("  %8.1f  %8.1f  ", off[k].mud_exposure, on[k].mud_exposure);
  std::printf("\n%-22s", "hazard-cell entries");
  for (std::size_t k = 0; k < plants.size(); ++k)
    std::printf("  %8d  %8d  ", off[k].hazard_entries, on[k].hazard_entries);
  std::printf("\n");

  for (std::size_t k = 0; k < plants.size(); ++k) {
    const std::string tag = plants[k] == "kinematic" ? "" : "_" + plants[k];
    write_ppm("nav_material_demo" + tag + "_off.ppm", occ, risk, hard, off[k]);
    write_ppm("nav_material_demo" + tag + "_on.ppm", occ, risk, hard, on[k]);
  }
  // The material field was tuned on nav's instant-response bicycle. On a physics vehicle, which
  // turns at its own rate and corners within its grip, it does not yet keep the lanes out of the
  // lake: that waits on training through the vehicle (docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md,
  // P8).
  for (std::size_t k = 0; k < plants.size(); ++k)
    if (plants[k] != "kinematic" && on[k].hazard_entries > 0)
      std::printf("note: on %s the material-aware drive still enters the hazard (%d entries); "
                  "it was tuned on the kinematic bicycle and awaits training through the "
                  "vehicle\n",
                  plants[k].c_str(), on[k].hazard_entries);
  std::puts("done — pure C++, zero torch/Python. Compare the .ppm renders.");
  // The material layer's contract, on the first plant run: no hazard entries with it on.
  return on[0].hazard_entries == 0 ? 0 : 1;
}
