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

// bundle.h — write a cvcworld/2 training bundle (roadmap §7.5).
//
// We write risk_raw + hard (the RAW contract inputs) and nothing derived: the
// blurred `risk`, phi_m and gradients belong to the consumer
// (cvc::nav::material_build / grl_snam.material.MaterialGrid), so a change to
// their blur sigma / EDT / gradient normalisation invalidates no bundle. The
// manifest carries grid facts + the §7.1a frame block (sigma as a LENGTH, since
// the consumer measures it in CELLS) + raw statistics.

#ifndef CVC_WORLD_BUNDLE_H
#define CVC_WORLD_BUNDLE_H

#include <cvc/world/grid.h>
#include <cvc/world/raster.h>
#include <string>

namespace cvc {
namespace world {

struct bundle_options {
  std::string scene_kind = "outdoor"; // outdoor | indoor | mixed  -> selects sigma_m
  double scale = 0.05;                // world->normalized (SdfNavigator frame)
  double agent_radius_m = 0.35;
  bool previews = true; // class/risk/hard PNG (PPM fallback)
  std::string tool_version = "1.0.0";
  std::string libcvc_commit = ""; // filled by the CLI if known
};

// Write the bundle directory (created if absent). Throws std::runtime_error on
// an I/O failure. Returns the manifest JSON text (also written to manifest.json).
std::string write_bundle(const std::string &dir, const world_model &wm, const grid_spec &g,
                         const raster_out &out, const bundle_options &opt = bundle_options{});

// Just the manifest JSON (no files written) — for tests and previews.
std::string bundle_manifest(const world_model &wm, const grid_spec &g, const raster_out &out,
                            const bundle_options &opt);

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_BUNDLE_H
