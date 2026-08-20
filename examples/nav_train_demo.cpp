/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_train_demo — SELF-SUPERVISED CoefMLP training from PURE C++: no Python, no
// libtorch. Trains the navigation policy on a scene's SDF via a differentiable
// rollout (no dataset, no labels) and writes the versioned .cvcnav the pure-C++
// swarm (sim_world / sim_world_cuda) then drives with. This is how you retrain /
// fine-tune the policy on the box you deploy on — e.g. on the actual occupancy of
// your terrain / lsystem_forest scene — without a torch install anywhere.
//
// Build: -DCVC_BUILD_NAV_EXAMPLE=ON. Run:
//   nav_train_demo [out.cvcnav] [steps]
// Trains on the Python "city" scene by default; swap city_scene() for
// occupancy_scene(your_grid, ...) to train on any rasterized scene. Uses the
// CUDA trainer when this build has CUDA and a device, else the CPU trainer.

#include <cstdio>
#include <cstdlib>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <string>

int main(int argc, char **argv) {
  using namespace cvc::nav;
  const std::string out = argc > 1 ? argv[1] : "coef_mlp.cvcnav";
  const int steps = argc > 2 ? std::atoi(argv[2]) : 300;

  // 1. The scene to train on. city_scene() ports the Python STORIES["city"]; for
  //    a real deployment, rasterize your terrain to an occupancy grid and use
  //    occupancy_scene(occ, rows, cols, min_x, min_y, max_x, max_y, scale).
  const training_scene scene = city_scene(96);
  std::printf("scene: %dx%d, %zu free cells\n", scene.rows, scene.cols, scene.free_cells.size());

  // 2. Train (self-supervised: the differentiable rollout is the whole signal).
  train_config cfg;
  cfg.steps = steps;
  cfg.n = 128;
  cfg.horizon = 28;
  cfg.window = 7;
  // cfg.lr defaults to the refinement rate (2e-4) — this REFINES the hand-tuned
  // (1,3,4) basin the net is centered on rather than learning from scratch.

  coef_mlp policy = [&] {
#ifdef CVC_ENABLE_CUDA
    if (train_cuda_available()) {
      std::puts("training on the GPU (CUDA)...");
      return train_coef_mlp_cuda(scene, cfg, /*verbose=*/true);
    }
#endif
    std::puts("training on the CPU...");
    coef_trainer tr(cfg, /*init_seed=*/1);
    tr.train(scene, /*verbose=*/true);
    return tr.to_coef_mlp();
  }();

  // 3. Persist to the versioned .cvcnav a pure-C++ host loads.
  policy.save(out, "trained by nav_train_demo (cvc::nav::coef_trainer)");
  std::printf("wrote %s  — load it with coef_mlp::load(\"%s\") and drive sim_world.\n", out.c_str(),
              out.c_str());
  return 0;
}
