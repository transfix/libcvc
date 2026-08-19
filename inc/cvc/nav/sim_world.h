/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// sim_world.h — the pure-C++ reactive swarm runtime (port P6).
//
// The owning aggregate that runs the whole GRL-SNAM swarm with NO libtorch and
// NO Python: it holds the shared belief, the SDF field, the coefficient policy
// and the struct-of-arrays agent columns, and its step() ties together the
// already-ported pieces — the belief sense (grid_nav sense_batch), the
// occupancy/field rebuild (belief_occupancy + build_sdf), the carrot FSM
// (drive.carrot_step) and the fused drive (drive.drive_step). This is what a
// renderer / game engine embeds to draw thousands of vehicles reacting to a live
// map. Shared belief (M=1) — the thousands-of-agents deployment path; the
// per-agent fog-of-war twin stays in Python. Float-equivalent to the torch
// Swarm; gated behaviorally, not bit-for-bit (docs/CVCNAV_CPP_PORT_ROADMAP.md P6).

#ifndef __CVC_NAV_SIM_WORLD_H__
#define __CVC_NAV_SIM_WORLD_H__

#include <cstdint>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/drive.h>
#include <vector>

namespace cvc {
namespace nav {

class sim_world {
public:
  struct config {
    int rows = 0, cols = 0;
    double min_x = 0, min_y = 0, max_x = 0, max_y = 0, cx = 0, cy = 0, scale = 1.0;
    double range_m = 60.0; // sensor
    int n_rays = 240;
    double fov_rad = 6.283185307179586;
    veh_params veh;         // vehicle + integration params (rr/d_hat/dt/vmax/... /nsub)
    float reach_tol = 0.8f; // normalized "arrived" radius (also drives park)
    int sense_every = 4;
    bool freeze_sense = false; // static known map: skip the sense/rebuild path
    double l_occ = 2.2, l_free = -1.4, l_clamp = 8.0;
    bool optimistic = true; // unknown-space policy
    double p_thresh = 0.5, band = 0.15, ttl_s = 4.0;
  };

  // truth / prior_occ are row-major rows*cols uint8; the initial field is built
  // from prior_occ. o/goal are [n*2] normalized (centered) start/goal; color is
  // [n*3] (renderer passthrough). `model` is moved in.
  sim_world(const config &cfg, const std::uint8_t *truth, const std::uint8_t *prior_occ,
            coef_mlp model, const float *o, const float *goal, const float *color, int n);

  // Advance one fixed-dt tick: sense (gated) -> rebuild -> carrot FSM -> drive ->
  // reached/park. Threaded across agents / the sense fold.
  void step(int num_threads = 0);

  int size() const { return n_; }
  long tick() const { return gstep_; }
  int field_version() const { return field_ver_; }

  // Renderer snapshot into caller buffers (any may be null): pose in WORLD
  // metres, heading (rad), speed (world m/s), FSM mode (0 seek / 1 wall),
  // reached flag. `field_data()` is the shared [1,3,H,W] SDF texture (by ref).
  void snapshot(float *pos_world, float *heading, float *speed, int *mode,
                std::uint8_t *reached) const;
  const float *field_data() const { return field_.data(); }

  // Live-scene edits (for the threading layer): retarget agent i (normalized
  // goal) keeping its escape state; stamp a decaying obstacle box (cell rect).
  void retarget(int i, float gx_n, float gy_n);
  void add_obstacle(int r0, int r1, int c0, int c1);

private:
  config cfg_;
  int n_ = 0, rows_ = 0, cols_ = 0;
  long gstep_ = 0;
  int field_ver_ = 0;
  int last_version_ = 0;

  coef_mlp model_;
  std::vector<std::uint8_t> truth_;

  // shared belief (M=1) + dynamic layer
  std::vector<float> logodds_; // rows*cols
  std::vector<std::uint8_t> lastvis_, everseen_;
  std::int32_t version_ = 0;
  std::vector<double> dyn_stamp_; // rows*cols, -inf where unmarked
  std::vector<std::uint8_t> occ_; // current planning raster

  // shared SDF field [1,3,H,W]
  std::vector<float> field_;

  // SoA agent columns
  std::vector<float> o_, goal_, th_, sp_, color_, wall_entry_, pos_hist_;
  std::vector<int> stall_, mode_, hist_count_;
  std::vector<float> turn_, dhit_, best_, init_;
  std::vector<std::uint8_t> we_valid_, tracking_, parked_, reached_, active_;

  field_stack field_view() const;
  void rebuild_field();
};

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_SIM_WORLD_H__
