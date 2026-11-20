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

// sea_lag.cpp — the sea lags the wind (see sea_lag.h).

#include <algorithm>
#include <cmath>
#include <cvc/algo/det_math.h>
#include <cvc/ocean/sea_lag.h>

namespace cvc {
namespace ocean {

namespace det = ::cvc::algo::det;

namespace {

constexpr double kPi = 3.14159265358979323846;

// b - a on the circle, in (-pi, pi].
double arc(double a, double b) {
  double d = std::fmod(b - a, 2.0 * kPi);
  if (d > kPi)
    d -= 2.0 * kPi;
  else if (d <= -kPi)
    d += 2.0 * kPi;
  return d;
}

} // namespace

double crossfade_weight(double t, double t0, double t1) noexcept {
  if (!(t1 > t0))
    return 1.0;
  if (t <= t0)
    return 0.0;
  if (t >= t1)
    return 1.0;
  const double u = (t - t0) / (t1 - t0);
  return u * u * (3.0 - 2.0 * u);
}

double sea_lag::weight(double t) const noexcept { return crossfade_weight(t, _t0, _t1); }

bool sea_lag::advance(double t, const sea_wind &wind) {
  if (!_started) {
    _started = true;
    _last = t;
    _lagged = _from = _to = wind;
    _t0 = _t1 = t;
    _keyframes = 1;
    return true;
  }
  const double dt = t - _last;
  if (!(_p.lag_s > 0.0)) { // no lag: the sea's wind is the wind, whether the clock runs or not
    _lagged.speed_mps = wind.speed_mps;
    _lagged.dir_rad = wind.dir_rad;
    _last = std::max(_last, t);
  } else if (dt > 0.0) {
    const double a = det::exp(-dt / _p.lag_s);
    _lagged.speed_mps = wind.speed_mps + (_lagged.speed_mps - wind.speed_mps) * a;
    _lagged.dir_rad += arc(_lagged.dir_rad, wind.dir_rad) * (1.0 - a);
    _lagged.dir_rad = std::fmod(_lagged.dir_rad, 2.0 * kPi);
    _last = t;
  }
  _lagged.fetch_m = wind.fetch_m;
  if (t < _t1)
    return false; // one keyframe per crossfade
  // How far apart two winds are, in steps: past 1 in any of the three, they differ.
  auto apart = [this](const sea_wind &a, const sea_wind &b) {
    const double sp =
        std::max(_p.speed_step * std::max(a.speed_mps, b.speed_mps), _p.speed_step_mps);
    const double ds = sp > 0.0 ? std::fabs(a.speed_mps - b.speed_mps) / sp : 0.0;
    const double dd =
        _p.dir_step_rad > 0.0 ? std::fabs(arc(a.dir_rad, b.dir_rad)) / _p.dir_step_rad : 0.0;
    const double fs = _p.fetch_step * std::max(a.fetch_m, b.fetch_m);
    const double df = fs > 0.0 ? std::fabs(a.fetch_m - b.fetch_m) / fs : 0.0;
    return std::max({ds, dd, df});
  };
  auto same = [](const sea_wind &a, const sea_wind &b) {
    return a.speed_mps == b.speed_mps && a.dir_rad == b.dir_rad && a.fetch_m == b.fetch_m;
  };
  sea_wind next;
  if (apart(_lagged, wind) < 1.0) {
    // Within a step of the wind: the last keyframe takes the wind itself.
    if (same(_to, wind))
      return false;
    next = wind;
  } else if (apart(_lagged, _to) >= 1.0) {
    next = _lagged;
  } else {
    return false;
  }
  _from = _to;
  _to = next;
  _t0 = t;
  _t1 = t + std::max(_p.crossfade_s, 0.0);
  ++_keyframes;
  return true;
}

} // namespace ocean
} // namespace cvc
