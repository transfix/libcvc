/*
  Copyright 2026 The University of Texas at Austin

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

// fetch.h — fetch and lee: how far the wind has blown over open water to each point
// (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md E2).
//
//   cvc::ocean::fetch_field f = cvc::ocean::build_fetch(grid, wind_dir, 120000.0);
//   double F = f.sample(x, y);                                  // metres of open water upwind
//   double s = cvc::ocean::fetch_scale(F, cvc::ocean::band_full_fetch(9.0, k_min, 120000.0));
//
// A wind sea's height grows as the square root of its fetch (JONSWAP; Hasselmann 1973), so the
// water just downwind of land (the lee) is calm, and a small lake stays small. For each wet sample
// the fetch is the distance upwind across water to the first dry sample, capped at the body's
// open fetch; where the march leaves the grid over water the water goes on past it, so the fetch
// there is the open fetch.
//
// The march is one sweep: the samples are visited column by column (or row by row, whichever
// axis the wind runs more along) from the upwind edge, and each takes the fetch of the point one
// column upwind, read linearly between the two samples there (a dry sample reads 0), plus the
// step. That is O(rows x cols) whatever the wind, and the lee's edge comes out soft over a few
// samples, widening with distance, as the wind fills in behind an island.
//
// The fetch map is coarse (a few metres to tens of metres a sample; build_fetch takes a stride)
// and depends on the wind's direction only, so it is built again only when the direction has
// changed (sea_lag.h debounces it), never per frame.
//
// The waves are scaled, not re-spectralized: each cascade's amplitude is multiplied by
// fetch_scale(F, full) = clamp(sqrt(F / full), 0, 1), where `full` is the fetch at which the
// cascade's band is fully grown. For the long waves that is the spectrum's own fetch (the roadmap's
// sqrt(F_local / F_spectrum)). A shorter band is fully grown sooner: once the fetch has carried
// JONSWAP's peak (omega_p = 22 (g^2 / U F)^(1/3)) below the band's lowest frequency, the band sits
// in the spectrum's equilibrium tail and grows no more (band_full_fetch). So in the lee the ripples
// come back within a few hundred metres to a few kilometres while the long waves wait for the open
// sea. A shorter fetch also shortens the peak period; beyond this per-band clamp that is ignored.
//
// Standard library only and deterministic: the same bits on every platform (no transcendentals but
// det_math's; built without FMA contraction).

#ifndef CVC_OCEAN_FETCH_H
#define CVC_OCEAN_FETCH_H

#include <cstddef>
#include <cstdint>
#include <cvc/ocean/shore.h>
#include <vector>

namespace cvc {
namespace ocean {

struct fetch_field {
  int rows = 0, cols = 0;
  double min_x = 0.0, min_y = 0.0, cell = 1.0; // row 0 = min y, x fastest
  double dir_rad = 0.0;                        // the wind it was built for (the way it blows)
  double open_m = 0.0;                         // the open fetch past the grid
  std::vector<float> fetch_m;                  // per sample: metres of water upwind; 0 when dry

  bool empty() const noexcept { return fetch_m.empty(); }
  std::size_t count() const noexcept { return std::size_t(rows) * std::size_t(cols); }
  // Bilinear at (x, y), clamped to the grid.
  double sample(double x, double y) const;
  std::uint64_t digest() const;
};

// The fetch over shore grid g (its depth: > 0 is water) for a wind blowing toward dir_rad (from +x
// toward +y), every `stride` samples of g. `open_fetch_m` is the fetch of water past the grid's
// edge; `cap` (null, or one value per sample of g) the most fetch each sample's body allows (its
// open extent). Throws std::invalid_argument on an inconsistent grid or a stride below 1.
fetch_field build_fetch(const shore_grid &g, double dir_rad, double open_fetch_m,
                        const std::vector<float> *cap = nullptr, int stride = 1);

// The spectrum's own fetch, as wind_sea_jonswap clamps it: at least 400 m, at most the fully
// developed sea's 2.2e4 U^2 / g.
double spectrum_fetch(double wind_speed_mps, double fetch_m);

// The fetch (m) at which a cascade whose band starts at wavenumber k_min (rad/m, deep water) is
// fully grown for wind speed U: where JONSWAP's peak frequency falls to the band's lowest, but at
// most the spectrum's fetch (spectrum_fetch). k_min 0: the spectrum's fetch.
double band_full_fetch(double wind_speed_mps, double k_min, double spectrum_fetch_m);

// clamp(sqrt(F / full), 0, 1); 1 when full is not positive.
double fetch_scale(double fetch_m, double full_m);

} // namespace ocean
} // namespace cvc

#endif // CVC_OCEAN_FETCH_H
