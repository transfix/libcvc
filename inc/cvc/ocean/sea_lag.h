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

// sea_lag.h — the sea lags the wind (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md E4).
//
//   cvc::ocean::sea_lag lag;                       // the sea starts grown for the first wind
//   if (lag.advance(t, {speed, dir, fetch})) { ... build h0 for lag.to(), keep lag.from()'s ... }
//   double w = lag.weight(t);                       // 0 -> 1: the old sea's share to the new's
//
// A real sea takes hours to grow to a new wind (duration-limited growth) and turns slowly when the
// wind shifts. Here the sea's wind follows the wind with a time constant, `lag_s` (a compressed
// scale, a knob): its speed exponentially, its direction along the shorter arc. Its fetch follows
// at once.
//
// The cascades' h0 is not rebuilt every frame as the lagged wind moves (build_h0 takes tens of
// milliseconds). The sea steps along it by keyframes instead: when no crossfade is running and the
// lagged wind has moved past a step from the last keyframe's (`speed_step` of its speed or
// `speed_step_mps`, whichever is more; `dir_step_rad`; `fetch_step` of the fetch), a new keyframe
// is made, and the old sea crossfades into the new one over `crossfade_s` seconds (a smoothstep
// in time). Once the lagged wind is within a step of the wind itself, the last keyframe takes the
// wind exactly, so the sea lands where the wind is. At most one keyframe per crossfade: a slider
// dragged back and forth makes one every `crossfade_s` seconds, not one per frame. That is the
// debounce for everything a keyframe rebuilds (h0, and the fetch map when the direction moved).
//
// h0(k, wind) = g(k) sqrt(S(k)) dk / 2 with the same noise g at every wind (h0.h), so the old and
// the new h0 are in phase texel by texel, and a linear crossfade of h0 is a continuous sea whose
// every mode's amplitude moves monotonically: no pop, no dip in energy, the waves' phases
// untouched. The physics sea (sea_surface's crossfade) and the drawn sea mix the same two h0 with
// the same weight at the same clock time, so the waves drawn and felt stay in phase through it.
//
// Deterministic: a pure function of the (time, wind) pairs it is advanced with (det_math's exp).
// A clock that runs backwards does not move the lagged wind.

#ifndef CVC_OCEAN_SEA_LAG_H
#define CVC_OCEAN_SEA_LAG_H

#include <cstdint>

namespace cvc {
namespace ocean {

struct sea_wind {
  double speed_mps = 0.0; // U10
  double dir_rad = 0.0;   // the way it blows, from +x toward +y
  double fetch_m = 0.0;   // the open fetch
};

struct sea_lag_params {
  double lag_s = 20.0;      // the sea's wind follows the wind with this time constant (0: at once)
  double crossfade_s = 4.0; // the old sea crossfades into a new keyframe's over this (0: at once)
  double speed_step = 0.05; // a new keyframe when the speed is this share off the last's ...
  double speed_step_mps = 0.25;  // ... or this many m/s, whichever is more
  double dir_step_rad = 0.05236; // 3 degrees
  double fetch_step = 0.02;      // a share of the fetch
};

class sea_lag {
public:
  explicit sea_lag(const sea_lag_params &p = {}) : _p(p) {}

  const sea_lag_params &params() const noexcept { return _p; }
  void set_params(const sea_lag_params &p) noexcept { _p = p; }

  // Advance to clock time t, the wind now `wind`. True when a new keyframe was made (the first call
  // always makes one, the sea grown for that wind).
  bool advance(double t, const sea_wind &wind);

  bool started() const noexcept { return _started; }
  const sea_wind &lagged() const noexcept { return _lagged; } // the sea's wind now
  const sea_wind &from() const noexcept { return _from; }     // the old keyframe's
  const sea_wind &to() const noexcept { return _to; }         // the new keyframe's
  double t0() const noexcept { return _t0; }                  // the crossfade's start
  double t1() const noexcept { return _t1; }                  // and end
  std::uint64_t keyframes() const noexcept { return _keyframes; }

  // The new keyframe's share at time t: 0 up to t0, a smoothstep to 1 at t1, 1 after.
  double weight(double t) const noexcept;
  // Whether the crossfade is under way at t (t0 <= t < t1).
  bool fading(double t) const noexcept { return t >= _t0 && t < _t1; }

private:
  sea_lag_params _p;
  bool _started = false;
  double _last = 0.0;
  sea_wind _lagged, _from, _to;
  double _t0 = 0.0, _t1 = 0.0;
  std::uint64_t _keyframes = 0;
};

// The crossfade's weight at t for a fade over [t0, t1]: a smoothstep, 1 when t1 <= t0.
double crossfade_weight(double t, double t0, double t1) noexcept;

} // namespace ocean
} // namespace cvc

#endif // CVC_OCEAN_SEA_LAG_H
