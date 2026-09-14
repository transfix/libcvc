/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// raster.h — the single seam every terrain consumer plugs into.
//
// world_model = heightfield + surface registry + placed props. raster() samples
// it onto a grid_spec and emits every plane at once, from one grid, so a
// material-vs-occupancy-vs-height misalignment is unrepresentable. Invariants
// asserted by raster() and by tests (roadmap §7.3):
//
//   klass.size()==risk_raw.size()==hard.size()==occupancy.size()==height.size()
//   risk_raw[i] == registry[klass[i]].rho     (exact)
//   hard[i]     == registry[klass[i]].hard     (exact)
//   hard ⊆ occupancy                           (every hard cell is occupied)
//
// The three-layer priority stack (§7.4): layer 0 derived predicates over
// (height, slope); layer 1 grammar-paint (prop footprints); occupied footprints
// win over soft ones. Every cell is independent, so the result is identical at
// any thread count. Row 0 == min_y.

#ifndef CVC_WORLD_RASTER_H
#define CVC_WORLD_RASTER_H

#include <cstdint>
#include <cvc/world/grid.h>
#include <cvc/world/heightfield.h>
#include <cvc/world/scatter.h>
#include <cvc/world/surface.h>
#include <vector>

namespace cvc {
namespace world {

struct world_params {
  double min_x = -120.0, min_y = -120.0, max_x = 120.0, max_y = 120.0; // metres (±120 DBG scale)
  std::uint64_t seed = 0;
  heightfield_params hf;
  scatter_params sc = scatter_params::defaults();
  std::string ontology = "merged_default";
};

class world_model {
public:
  static world_model generate(const world_params &wp);

  const heightfield &hf() const noexcept { return hf_; }
  const surface_registry &registry() const noexcept { return *reg_; }
  const std::vector<placed_prop> &props() const noexcept { return props_; }
  const world_params &params() const noexcept { return wp_; }

private:
  explicit world_model(const world_params &wp) : wp_(wp), hf_(wp.hf) {}
  world_params wp_;
  heightfield hf_;
  const surface_registry *reg_ = nullptr;
  std::vector<placed_prop> props_;
};

struct raster_out {
  int rows = 0, cols = 0;
  std::vector<std::uint16_t> klass;      // semantic class id (the authored truth)
  std::vector<float> risk_raw;           // [0,1] == registry[klass].rho
  std::vector<std::uint8_t> hard;        // 0/1 == registry[klass].hard
  std::vector<std::uint8_t> occupancy;   // 0/1 (hard ⊆ occupancy)
  std::vector<float> height;             // metres
  std::vector<std::uint8_t> layer_owner; // 0 derived, 1 grammar-paint

  std::size_t occupied_count() const {
    std::size_t n = 0;
    for (std::uint8_t o : occupancy)
      n += o ? 1 : 0;
    return n;
  }
};

// THE single entry point.
void raster(const world_model &wm, const grid_spec &g, raster_out &out);

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_RASTER_H
