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
// map. Belief is M planes via a per-agent map_id — shared (M=1, the
// thousands-of-agents path), clustered (K groups), or private (M=N, the
// fog-of-war twin) — mirroring the Python Swarm's belief_mode. Float-equivalent
// to the torch Swarm; gated behaviorally, not bit-for-bit
// (docs/CVCNAV_CPP_PORT_ROADMAP.md P6).

#ifndef __CVC_NAV_SIM_WORLD_H__
#define __CVC_NAV_SIM_WORLD_H__

#include <cstdint>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/drive.h>
#include <cvc/nav/material.h>
#include <vector>

namespace cvc {
namespace nav {

class sim_world {
public:
  // How belief is grouped across agents (the C++ counterpart of the Python
  // Swarm's belief_mode). shared = one plane all agents sense into / sample from
  // (M == 1, the thousands-of-agents deployment path); clustered = K groups, one
  // belief plane each; private = one plane per agent (M == N, the fog-of-war
  // twin). Agents in different planes are isolated — one's sensing never touches
  // another's map.
  enum class belief_mode { shared, clustered, private_belief };

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
    // Inter-agent separation (optional swarm collision avoidance). Each tick,
    // nudge every agent's carrot away from peers within `sep_radius` (normalized
    // units, like o/goal) with strength `sep_gain`. Both default 0 = OFF, so
    // single-agent, fog and finale runs are byte-unchanged. This is the swarm
    // counterpart to the squad's peer stamping: mode-agnostic (per-agent,
    // independent of the belief planes, which are shared in the scale path), via
    // the neighbors_within_radius spatial hash (grid_nav.h). It steers the
    // reactive drive AROUND neighbours instead of through them.
    float sep_radius = 0.0f;
    float sep_gain = 0.0f;
  };

  // truth / prior_occ are row-major rows*cols uint8; the initial field is built
  // from prior_occ. o/goal are [n*2] normalized (centered) start/goal; color is
  // [n*3] (renderer passthrough). `model` is moved in. `map_id` (optional, [n])
  // selects each agent's belief plane; `n_planes` is the plane count M. Pass
  // map_id == nullptr for shared belief (M forced to 1). Every plane is seeded
  // from prior_occ and diverges as its agents sense.
  sim_world(const config &cfg, const std::uint8_t *truth, const std::uint8_t *prior_occ,
            coef_mlp model, const float *o, const float *goal, const float *color, int n,
            const int *map_id = nullptr, int n_planes = 1);

  // Convenience factory: build a sim_world from one occupancy grid (used as both
  // the truth and the initial known map), auto-scattering `n_agents` starts +
  // goals on free cells (occ == 0) with random colors — the few-line path for a
  // pure-C++ host (e.g. a cvcGL scene rasterized to occupancy) to drop navigating
  // agents in. `model` is moved (use coef_mlp::default_biased() for zero-setup).
  // `mode` picks the belief grouping: shared (M=1), private (M=n_agents), or
  // clustered into `clusters` groups (k-means-lite on start positions).
  static sim_world from_occupancy(const config &cfg, const std::uint8_t *occ, coef_mlp model,
                                  int n_agents, unsigned seed = 0,
                                  belief_mode mode = belief_mode::shared, int clusters = 1);

  // Scatter `n` starts + goals on free cells (occ == 0) with random colors,
  // seed-deterministic — the shared core of from_occupancy (reused by the CUDA
  // twin so both scatter identically). Writes o[n*2], goal[n*2] (normalized,
  // centered) and color[n*3].
  static void scatter_free(const config &cfg, const std::uint8_t *occ, int n, unsigned seed,
                           float *o, float *goal, float *color);

  // Advance one fixed-dt tick: sense (gated) -> rebuild -> carrot FSM -> drive ->
  // reached/park. Threaded across agents / the sense fold.
  void step(int num_threads = 0);

  int size() const { return n_; }
  int planes() const { return M_; } // belief-plane count (M): 1 shared, N private
  int rows() const { return rows_; }
  int cols() const { return cols_; }
  // Per-agent belief-plane id [n] (each in [0, M)) — lets a renderer colour agents
  // by their group (shared: all 0; clustered: group id; private: 0..n-1).
  const int *agent_planes() const { return map_id_.data(); }
  long tick() const { return gstep_; }
  int field_version() const { return field_ver_; }

  // Renderer snapshot into caller buffers (any may be null): pose in WORLD
  // metres, heading (rad), speed (world m/s), FSM mode (0 seek / 1 wall),
  // reached flag. `field_data()` is the [M,3,H,W] SDF texture block (by ref;
  // plane m at m*3*H*W).
  void snapshot(float *pos_world, float *heading, float *speed, int *mode,
                std::uint8_t *reached) const;
  // Per-agent goal positions in WORLD metres ([n*2], snapshot's conversion) —
  // lets a renderer draw where each agent is trying to go.
  void goals_world(float *out) const;
  // Per-agent live carrot (the FSM's current steering target) in WORLD metres
  // ([n*2]); before the first step() it falls back to the goals. Watching the
  // carrot deflect around obstacles IS the reactive drive, made visible.
  void carrots_world(float *out) const;
  const float *field_data() const { return field_.data(); }

  // Epistemic read surface ([rows*cols] rasters) — lets a renderer draw honest
  // belief-vs-truth: truth() is the world as it IS; belief_occ(m) is plane m's
  // current planning raster (belief & ~truth = a phantom the agents believe);
  // ever_seen(m) / last_visible(m) are plane m's fog tiers (remembered vs in
  // view right now). Plane pointers index m*rows*cols into the [M,H,W] blocks.
  const std::uint8_t *truth() const { return truth_.data(); }
  const std::uint8_t *belief_occ(int m) const {
    return occ_.data() + static_cast<std::size_t>(m) * rows_ * cols_;
  }
  const std::uint8_t *ever_seen(int m) const {
    return everseen_.data() + static_cast<std::size_t>(m) * rows_ * cols_;
  }
  const std::uint8_t *last_visible(int m) const {
    return lastvis_.data() + static_cast<std::size_t>(m) * rows_ * cols_;
  }

  // Live-scene edits (for the threading layer): retarget agent i (normalized
  // goal) keeping its escape state; stamp a decaying obstacle blob (a centered
  // radius over the cell rect, matching Swarm.add_obstacle) into the dynamic
  // layer. NOTE: the dynamic layer is only composited into the planning surface
  // on a SENSE tick, so add_obstacle takes effect only when freeze_sense == false
  // (the fog path). Under freeze_sense == true (the static-map deployment path)
  // it is inert — rebuild the field from a fresh occupancy to change a static map.
  void retarget(int i, float gx_n, float gy_n);
  void add_obstacle(int r0, int r1, int c0, int c1);

  // Live-tune the optional inter-agent separation (config.sep_radius/sep_gain);
  // gain 0 disables it. Lets a host toggle/scale swarm collision avoidance at
  // runtime without rebuilding the world.
  void set_separation(float radius, float gain) {
    cfg_.sep_radius = radius;
    cfg_.sep_gain = gain;
  }

  // ── material-aware navigation (cvc/nav/material.h) ────────────────────────
  // COPIES the rasters, derives the material planes (material_build), and
  // computes the witness gate's feasibility surface as material.hard | TRUTH
  // occupancy (the oracle setting — matching the Python Swarm, which has no
  // planner; the fog-of-war FogScenario gates against belief instead). Every
  // subsequent step() evaluates the frame-wise gate per agent against its own
  // goal and drives with the material forces. Default off = byte-unchanged
  // runs (the sep_radius/sep_gain pattern).
  // `planes` == 1 (default): one shared material plane — every agent samples it
  // regardless of its belief plane (the current behavior, back-compat). `planes`
  // > 1: `risk_raw`/`hard` are [planes,H,W] stacks and one material + gate plane
  // is built per group; each agent indexes its material plane by the SAME map_id
  // as its belief plane, so every map_id must be < `planes` (throws otherwise).
  void set_material(const float *risk_raw, const std::uint8_t *hard, const material_config &mc,
                    int planes = 1);
  void clear_material() { mat_on_ = false; }
  bool has_material() const { return mat_on_; }
  // Last tick's per-agent gate decisions (renderer/telemetry hook); valid only
  // while material is set.
  const std::uint8_t *material_gate_active() const { return mat_gate_active_.data(); }

private:
  config cfg_;
  int n_ = 0, rows_ = 0, cols_ = 0, M_ = 1;
  long gstep_ = 0;
  int field_ver_ = 0;

  coef_mlp model_;
  std::vector<std::uint8_t> truth_;
  std::vector<int> map_id_; // [n] agent -> belief plane in [0, M)

  // M belief planes (contiguous, plane m at offset m*rows*cols) + per-plane
  // dynamic layer, occupancy raster, and version / last-rebuilt-version.
  std::vector<float> logodds_; // [M*rows*cols]
  std::vector<std::uint8_t> lastvis_, everseen_;
  std::vector<std::int32_t> version_; // [M]
  std::vector<double> dyn_stamp_;     // [M*rows*cols], -inf where unmarked
  std::vector<std::uint8_t> occ_;     // [M*rows*cols] current planning rasters
  std::vector<int> last_version_;     // [M]

  // M SDF fields [M,3,H,W] (plane m at m*3*rows*cols).
  std::vector<float> field_;

  // SoA agent columns
  std::vector<float> o_, goal_, th_, sp_, color_, wall_entry_, pos_hist_;
  std::vector<float> carrot_; // [n*2] last tick's FSM steering target (for renderers)
  std::vector<int> stall_, mode_, hist_count_;
  std::vector<float> turn_, dhit_, best_, init_;
  std::vector<std::uint8_t> we_valid_, tracking_, parked_, reached_, active_;

  // material state (set_material; inert while mat_on_ == false)
  bool mat_on_ = false;
  int mat_planes_ = 1;                             // material/gate plane count (P2a)
  material_config mat_cfg_;
  std::vector<float> mat_stack_;                   // [mat_planes_,6,H,W] derived planes
  std::vector<float> mat_risk_;                    // [mat_planes_,H,W] contiguous risk (ch 0) for the gate
  std::vector<std::uint8_t> mat_gate_hard_;        // [mat_planes_,H,W] hard | truth
  std::vector<float> mat_clear_m_;                 // [mat_planes_,H,W] metres EDT of mat_gate_hard_
  std::vector<float> mat_lam_soft_, mat_lam_hard_; // [n] per-tick columns
  std::vector<std::uint8_t> mat_gate_active_;      // [n]
  double mat_hard_margin_m_ = 0.0;

  field_stack field_view() const;
  material_stack material_view() const;
  void rebuild_all_fields(); // build every plane's SDF from its occ
  void rebuild_plane(int m); // build plane m's SDF from occ_ plane m
};

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_SIM_WORLD_H__
