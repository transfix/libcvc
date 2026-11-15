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
  A port of ABYSSAL's spectrum (github.com/Token-Gremlin/natural-disasters,
  src/ocean/OceanFFT.js at 849ff7f4: SPECTRUM_FRAG and OceanFFT._jonswapParams,
  _m0, _buildSpectra), MIT licensed, Copyright (c) 2026 Davi (Token-Gremlin).
  MIT notice retained; see THIRD_PARTY_NOTICES.md.
*/

// spectrum.cpp — the directional wave spectrum (see spectrum.h).

#include <algorithm>
#include <cmath>
#include <cvc/algo/det_math.h>
#include <cvc/ocean/spectrum.h>

namespace cvc {
namespace ocean {

namespace det = ::cvc::algo::det; // the same bits on every platform (det_math.h)

namespace {

constexpr double kPi = 3.14159265358979323846;

// Donelan-Banner's normalisation of cos^2s(theta / 2) over a full turn, a fit in s.
double normFactor(double s) {
  const double s2 = s * s, s3 = s2 * s, s4 = s3 * s;
  if (s < 5.0)
    return -0.000564 * s4 + 0.00776 * s3 - 0.044 * s2 + 0.192 * s + 0.163;
  return -4.80e-8 * s4 + 1.07e-5 * s3 - 9.53e-4 * s2 + 5.90e-2 * s + 3.93e-1;
}

double cos2s(double theta, double s) {
  return normFactor(s) * det::pow(std::max(std::fabs(det::cos(0.5 * theta)), 1e-5), 2.0 * s);
}

// Donelan-Banner's spreading exponent: broadest at the peak, narrowing either side of it.
double spreadPower(double omega, double peak) {
  const double r = std::max(omega / peak, 1e-5);
  return omega > peak ? 9.77 * det::pow(r, -2.5) : 6.97 * det::pow(r, 5.0);
}

// Kitaigorodskii's depth factor for the TMA spectrum (Thompson and Vincent's piecewise fit).
double tmaCorrection(double omega, double depth) {
  const double oh = omega * std::sqrt(depth / gravity);
  if (oh <= 1.0)
    return 0.5 * oh * oh;
  if (oh < 2.0)
    return 1.0 - 0.5 * (2.0 - oh) * (2.0 - oh);
  return 1.0;
}

// The deep-water JONSWAP variance of omega in [0.08, 14] rad/s, integrated on a log grid.
double m0(const jonswap_params &p) {
  const int N = 128;
  const double wMin = 0.08, wMax = 14.0, lr = det::log(wMax / wMin);
  double m = 0.0;
  for (int i = 0; i < N; ++i) {
    const double w = wMin * det::exp(lr * (i + 0.5) / N);
    const double dw = w * lr / N;
    const double sigma = w <= p.peak_omega ? 0.07 : 0.09;
    const double r = det::exp(-((w - p.peak_omega) * (w - p.peak_omega)) /
                              (2.0 * sigma * sigma * p.peak_omega * p.peak_omega));
    const double S = p.alpha * gravity * gravity / det::pow(w, 5.0) *
                     det::exp(-1.25 * det::pow(p.peak_omega / w, 4.0)) * det::pow(p.gamma, r);
    m += S * dw;
  }
  return m;
}

// One part of the spectrum at |k| = kLen, travelling at angle kAngle: amp x JONSWAP x spread x
// short-wave fade x |d omega / d k| / k (the Jacobian from S(omega, theta) to S(kx, ky)).
double part(double kLen, double kAngle, double amp, double dir, double spread, double narrowing,
            const jonswap_params &jp, double fade, double depth) {
  if (amp <= 1e-6 || kLen <= 1e-6)
    return 0.0;
  const double omega = dispersion(kLen, depth);
  return amp * jonswap(omega, jp, depth) *
         directional(kAngle - dir, omega, jp.peak_omega, spread, narrowing) *
         det::exp(-fade * fade * kLen * kLen) * std::fabs(dispersion_dk(kLen, depth)) / kLen;
}

} // namespace

jonswap_params wind_sea_jonswap(const wind_sea_params &w) {
  const double u = std::max(w.speed_mps, 0.5);
  const double fullyDeveloped = 2.2e4 * u * u / gravity;
  const double F = std::max(std::min(w.fetch_m, fullyDeveloped), 400.0);
  jonswap_params p;
  p.alpha = 0.076 * det::pow(u * u / (F * gravity), 0.22);
  p.peak_omega = std::max(22.0 * det::cbrt(gravity * gravity / (u * F)), 0.30);
  p.gamma = w.gamma;
  return p;
}

jonswap_params swell_jonswap(const swell_params &s) {
  const double Tp = std::max(s.tp_s, 3.0), gamma = std::max(s.gamma, 1.0);
  const double Hs = std::max(s.hs_m, 0.0);
  jonswap_params p;
  p.alpha = 5.061 * (Hs * Hs / (Tp * Tp * Tp * Tp)) * (1.0 - 0.287 * det::log(gamma));
  p.peak_omega = 2.0 * kPi / Tp;
  p.gamma = gamma;
  return p;
}

double dispersion(double k, double depth_m) {
  return std::sqrt(gravity * k * det::tanh(std::min(k * depth_m, 20.0)));
}

double dispersion_dk(double k, double depth_m) {
  const double kd = k * depth_m;
  const double th = det::tanh(std::min(kd, 20.0));
  const double ch = kd < 20.0 ? det::cosh(kd) : HUGE_VAL; // sech^2 is 0 past 20 to a double
  return gravity * (depth_m * k / (ch * ch) + th) /
         std::max(2.0 * std::sqrt(gravity * k * th), 1e-9);
}

double jonswap(double omega, const jonswap_params &p, double depth_m) {
  const double peak = p.peak_omega;
  const double sigma = omega <= peak ? 0.07 : 0.09;
  const double r = det::exp(-(omega - peak) * (omega - peak) / (2.0 * sigma * sigma * peak * peak));
  const double invO = 1.0 / std::max(omega, 1e-5);
  const double invO5 = invO * invO * invO * invO * invO;
  return tmaCorrection(omega, depth_m) * p.alpha * gravity * gravity * invO5 *
         det::exp(-1.25 * det::pow(peak * invO, 4.0)) * det::pow(std::fabs(p.gamma), r);
}

double directional(double theta, double omega, double peak_omega, double spread,
                   double swell_narrowing) {
  const double s = spreadPower(omega, peak_omega) +
                   16.0 * det::tanh(std::min(omega / std::max(peak_omega, 1e-4), 20.0)) *
                       swell_narrowing * swell_narrowing;
  // cos^2 over the downwind half only, where 2 / pi normalises it (ABYSSAL's spans the turn).
  const double c = det::cos(theta);
  return (1.0 - spread) * (c > 0.0 ? 2.0 / kPi * c * c : 0.0) + spread * cos2s(theta, s);
}

double wind_sea_spectrum(double kx, double ky, const sea_state &s) {
  const double kLen = det::hypot(kx, ky);
  const wind_sea_params &w = s.wind;
  return part(kLen, det::atan2(ky, kx), s.amplitude, w.dir_rad, w.spread, 0.0, wind_sea_jonswap(w),
              w.short_fade_m, s.depth_m);
}

double swell_spectrum(double kx, double ky, const sea_state &s) {
  const swell_params &sw = s.swell;
  if (sw.hs_m <= 1e-3)
    return 0.0;
  const double kLen = det::hypot(kx, ky);
  return part(kLen, det::atan2(ky, kx), s.amplitude, sw.dir_rad, 1.0, 1.0, swell_jonswap(sw),
              sw.cutoff_m, s.depth_m);
}

double spectrum(double kx, double ky, const sea_state &s) {
  return wind_sea_spectrum(kx, ky, s) + swell_spectrum(kx, ky, s);
}

double wind_sea_hs(const sea_state &s) {
  return 4.0 * std::sqrt(std::max(m0(wind_sea_jonswap(s.wind)) * s.amplitude, 1e-9));
}

double swell_hs(const sea_state &s) {
  const double m = s.swell.hs_m > 1e-3 ? m0(swell_jonswap(s.swell)) * s.amplitude : 0.0;
  return 4.0 * std::sqrt(std::max(m, 1e-9));
}

double significant_wave_height(const sea_state &s) {
  const double mw = m0(wind_sea_jonswap(s.wind)) * s.amplitude;
  const double ms = s.swell.hs_m > 1e-3 ? m0(swell_jonswap(s.swell)) * s.amplitude : 0.0;
  return 4.0 * std::sqrt(std::max(mw + ms, 1e-9));
}

} // namespace ocean
} // namespace cvc
