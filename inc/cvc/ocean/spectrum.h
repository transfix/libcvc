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

  ---------------------------------------------------------------------------
  The spectrum is a port of ABYSSAL's (github.com/Token-Gremlin/natural-disasters,
  src/ocean/OceanFFT.js at 849ff7f4), MIT licensed, Copyright (c) 2026 Davi
  (Token-Gremlin). MIT notice retained; see THIRD_PARTY_NOTICES.md.
*/

// spectrum.h — the sea state as a directional wave spectrum: a wind sea and a swell
// (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md, B1 and E1, section 10.1).
//
//   cvc::ocean::sea_state s;          // wind sea (speed, fetch, direction) + swell (Hs, Tp, dir)
//   double S = cvc::ocean::spectrum(kx, ky, s);  // m^4: the variance density at wavenumber k
//   double hs = cvc::ocean::significant_wave_height(s);
//
// Each part is a JONSWAP frequency spectrum with the TMA shallow-water correction, spread over
// direction by a blend of cos^2 and Donelan-Banner's cos^2s, and carried to wavenumber through
// the finite-depth dispersion relation omega^2 = g k tanh(k d). The wind sea's JONSWAP comes from
// the wind speed and fetch (the fetch clamped to the fully developed sea); the swell's from its
// significant height and peak period (DNV-RP-C205), narrowed in direction and cut off below its
// long waves. One spectrum for every consumer: the GPU ocean's cascades (h0.h) and, later, the
// physics band all sample this function, so they agree by construction (the roadmap's R3).
//
// ABYSSAL's cos^2 term is not rotated by the wind; here it is (the roadmap's D10). And ABYSSAL's
// cos^2 spans the whole turn, where 2 / pi normalises it over half of one: its upwind lobe
// doubled that part's energy and sent it against the wind. Here it covers the downwind half, so
// the spread integrates to 1 and the spectrum holds the sea's m0 (OCEAN-AND-WEATHER-ROADMAP.md,
// "The sea's height for its wind"). Standard library only; a pure function of its arguments.

#ifndef CVC_OCEAN_SPECTRUM_H
#define CVC_OCEAN_SPECTRUM_H

namespace cvc {
namespace ocean {

constexpr double gravity = 9.80665; // m/s^2

// The local wind sea: chop and whitecaps raised by the wind over the fetch.
struct wind_sea_params {
  double speed_mps = 9.0;    // U10
  double fetch_m = 120000.0; // clamped to the fully developed sea, and to at least 400 m
  double dir_rad = 0.0;      // the direction the waves travel, from +x toward +y
  double spread = 0.72;      // 0: cos^2 about the wind only; 1: Donelan-Banner only
  double gamma = 3.3;        // JONSWAP peak enhancement
  double short_fade_m = 0.0; // the short waves fade as exp(-(fade k)^2); 0 keeps them all
};

// The swell: long, narrow-banded waves from distant weather (what breaks on a beach).
struct swell_params {
  double hs_m = 0.0;      // significant height; 0 turns the swell off
  double tp_s = 11.0;     // peak period (at least 3 s)
  double dir_rad = 0.0;   // the direction it travels
  double gamma = 6.0;     // narrow-banded
  double cutoff_m = 14.0; // fades the short-wave tail as exp(-(cutoff k)^2): a pure swell
};

struct sea_state {
  wind_sea_params wind;
  swell_params swell;
  double depth_m = 1000.0; // the water depth for dispersion and TMA
  double amplitude = 1.0;  // scales the whole spectrum (variance)
};

// The JONSWAP parameters of one part.
struct jonswap_params {
  double alpha = 0.0;      // the Phillips constant
  double peak_omega = 1.0; // rad/s
  double gamma = 3.3;
};
jonswap_params wind_sea_jonswap(const wind_sea_params &w);
jonswap_params swell_jonswap(const swell_params &s);

// Angular frequency of a wave of wavenumber k (rad/m) in water depth d: sqrt(g k tanh(k d)).
double dispersion(double k, double depth_m);
// Its derivative d omega / d k (the group velocity).
double dispersion_dk(double k, double depth_m);

// JONSWAP S(omega) (m^2 s) with the TMA correction for depth d.
double jonswap(double omega, const jonswap_params &p, double depth_m);

// The directional spread at angle theta from the waves' mean direction, for frequency omega:
// (1 - spread) x (2/pi) cos^2(theta) over |theta| < pi/2 (0 upwind) + spread x Donelan-Banner
// cos^2s(theta / 2), whose exponent narrows with `swell_narrowing` (0 for a wind sea, 1 for a
// swell). Each part integrates to 1 over the turn.
double directional(double theta, double omega, double peak_omega, double spread,
                   double swell_narrowing);

// The directional wavenumber spectrum at k = (kx, ky): the wind sea's plus the swell's, in m^4.
double spectrum(double kx, double ky, const sea_state &s);
// The wind sea's alone, and the swell's alone.
double wind_sea_spectrum(double kx, double ky, const sea_state &s);
double swell_spectrum(double kx, double ky, const sea_state &s);

// 4 sqrt(m0), m0 the variance of the 1-D frequency spectra (deep water, before the spread and the
// short-wave fades): the wave height a HUD reports. Both parts, the wind sea's, the swell's.
double significant_wave_height(const sea_state &s);
double wind_sea_hs(const sea_state &s);
double swell_hs(const sea_state &s);

} // namespace ocean
} // namespace cvc

#endif // CVC_OCEAN_SPECTRUM_H
