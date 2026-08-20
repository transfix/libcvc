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

// sim_world_cuda.h — the DEVICE-RESIDENT reactive swarm runtime (the fused GPU
// twin of sim_world, port P6-CUDA).
//
// sim_world (CPU) uploads/copies the field + weights + agent columns on every
// drive_step_cuda call; this keeps ALL of it resident on the GPU across ticks —
// the shared SDF field, the .cvcnav policy weights, and every SoA agent column
// (pose AND the full carrot-FSM state). step() launches three kernels (carrot
// FSM -> fused sample/coef_feats/coef_mlp/bicycle drive -> reached/park) with
// zero host round-trips; only snapshot() copies device->host, and only the
// pose-sized columns a renderer needs. This is the GPU deployment path a
// renderer / game engine uses when N outgrows the CPU (drop thousands of
// navigating agents into a cvcGL / lsystem_forest scene).
//
// Scope: STATIC known maps — no on-device sense/rebuild. Belief is M planes via a
// per-agent map_id (the GPU analog of sim_world's grouped belief under
// freeze_sense): shared (M == 1, one map), or grouped/private (M planes, one
// known map each — different groups can carry genuinely different intel). Each
// plane's field is built once on the host (one EDT per plane) and uploaded as
// [M,3,H,W]; after that the world never leaves the GPU and agent i always samples
// plane map_id[i]. For fog-of-war (live belief that diverges as agents sense) use
// sim_world (CPU). Float-equivalent to sim_world::step on the same config with
// freeze_sense = true — the kernels share drive.cu's device math and the same
// .cvcnav weights, and the carrot FSM is a per-agent transcription of
// carrot_step (docs/CVCNAV_CPP_PORT_ROADMAP.md §CUDA).
//
// Requires CVC_ENABLE_CUDA at build time and a CUDA device at run time; guard
// callers with available() (a build without CUDA does not define these symbols).

#ifndef __CVC_NAV_SIM_WORLD_CUDA_H__
#define __CVC_NAV_SIM_WORLD_CUDA_H__

#include <cstdint>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/sim_world.h> // sim_world::config

namespace cvc {
namespace nav {

class sim_world_cuda {
public:
  // Same inputs as sim_world's ctor. Each belief plane's field is built on the
  // HOST (one EDT per plane) and uploaded; the maps are then static (no
  // sense/rebuild). o/goal are [n*2] normalized (centered) start/goal, color is
  // [n*3]. `model` is moved in. `map_id` (optional, [n]) selects each agent's
  // plane and `n_planes` is the plane count M; when map_id != nullptr `occ` is
  // [n_planes * rows*cols] (one known map per plane), otherwise it is the shared
  // single map [rows*cols] (M forced to 1). Throws std::runtime_error if CUDA is
  // unavailable.
  sim_world_cuda(const sim_world::config &cfg, const std::uint8_t *occ, coef_mlp model,
                 const float *o, const float *goal, const float *color, int n,
                 const int *map_id = nullptr, int n_planes = 1);

  // Convenience factory mirroring sim_world::from_occupancy — scatter n_agents on
  // free cells (occ == 0) with random colors, then upload the shared single map.
  // Deterministic in seed. (Grouped/private belief needs per-plane maps — use the
  // ctor with map_id + a [n_planes*rows*cols] occ block.)
  static sim_world_cuda from_occupancy(const sim_world::config &cfg, const std::uint8_t *occ,
                                       coef_mlp model, int n_agents, unsigned seed = 0);

  ~sim_world_cuda();
  sim_world_cuda(sim_world_cuda &&) noexcept;
  sim_world_cuda &operator=(sim_world_cuda &&) noexcept;
  sim_world_cuda(const sim_world_cuda &) = delete;
  sim_world_cuda &operator=(const sim_world_cuda &) = delete;

  // One device tick: carrot FSM -> fused drive -> reached/park, all on the GPU
  // (no host round-trip). Kernels serialize on the default stream.
  void step();

  int size() const { return n_; }
  int planes() const; // belief-plane count M (1 shared, >1 grouped/private)
  long tick() const { return gstep_; }

  // Pose-only device->host snapshot (this is the only D2H). pos in WORLD metres,
  // heading rad, speed world m/s, FSM mode (0 seek / 1 wall), reached flag; any
  // pointer may be null. Blocks until the pending step() kernels finish.
  void snapshot(float *pos_world, float *heading, float *speed, int *mode,
                std::uint8_t *reached) const;

  // Live retarget of agent i to a normalized goal, keeping its escape state
  // (one 1-thread kernel; no host round-trip).
  void retarget(int i, float gx_n, float gy_n);

  // True iff built with CUDA AND a device is present. Safe to call always.
  static bool available();

private:
  struct impl;
  impl *p_ = nullptr;
  int n_ = 0;
  long gstep_ = 0;
};

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_SIM_WORLD_CUDA_H__
