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

// whitecaps.h — how much of the open sea the drawn wave foam covers, and the foam threshold that
// makes it cover Monahan's share for the wind (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md, O5 and
// section 10.2).
//
//   cvc::ocean::foam_field f({h0_long, h0_mid, h0_short});      // build_h0's fields
//   double bias = f.bias_for(cvc::ocean::monahan_coverage(9.0)); // lsystem_coast's uFoamBias
//
// The drawn sea's wave foam is a threshold on the Jacobian of its choppy map. Each cascade's
// J = (1 + l Dxx)(1 + l Dyy) - (l Dxy)^2 (OceanFFT's assemble pass, l its chop); the vertex shader
// keeps the least of the cascades' (a faded cascade folds less; on the open sea none is faded),
// and the fragment shader whitens the water by foam = smoothstep(0, 1, (bias - J) / ramp).
//
// A fixed bias gives a fixed share of foam on one sea only: on a calmer sea J seldom dips that
// low, on a rougher one it often does. Whitecaps cover a share of the sea that grows with the
// wind, W = 3.84e-6 U^3.41 (Monahan and O'Muircheartaigh 1980, U the wind at 10 m; capped at
// 0.16, as ABYSSAL caps it). So the bias follows the sea: foam_field evolves the cascades' h0 on
// the CPU as OceanFFT does, samples the least J over a lattice of the open sea at a few times,
// and finds the bias whose mean foam, the share of the surface the foam whitens, is W.
//
// The share is the mean of the foam value, since the shader mixes the water toward white by it:
// a fully white patch counts whole and a half-white one half. It is measured over the open sea's
// least J at lattice points; the drawn surface interpolates J between its vertices, so a coarse
// mesh shows a little less foam than this says.

#ifndef CVC_OCEAN_WHITECAPS_H
#define CVC_OCEAN_WHITECAPS_H

#include <cstddef>
#include <cstdint>
#include <cvc/ocean/h0.h>
#include <utility>
#include <vector>

namespace cvc {
namespace ocean {

// The share of the sea under whitecaps at a 10 m wind of u10 m/s: min(3.84e-6 u10^3.41, 0.16)
// (Monahan and O'Muircheartaigh 1980); 0 for u10 <= 0.
double monahan_coverage(double u10_mps);

struct foam_params {
  double chop = 1.15; // the Jacobian's l: OceanFFT::chop (lsystem_coast's coast.water.chop)
  double ramp = 0.1;  // the shader's foam ramp, in J: foam reaches 1 at J = bias - ramp
  // The shader's fray: each fragment's J shifted by fray x (n - 0.5), n = 0.65 v1 + 0.35 v2 with
  // v1, v2 value noise (a smoothstep-weighted bilinear mix of four uniform hashes) at two scales,
  // so a patch's edge is ragged rather than the straight line J makes across a triangle. Modelled
  // by 16 quantiles of n's distribution (from a fixed draw); 0: no fray.
  double fray = 0.0;
  double depth_m = 1000.0;  // the depth the cascades disperse in (OceanFFT::depth_m)
  double gravity = 9.80665; // m/s^2 (OceanFFT's)
  double spacing_m = 0.37;  // the open-sea lattice's spacing (the side is the largest tile)
  // The drawn mesh's vertex spacing: J taken at its vertices and linear over its triangles, as
  // the fragments see it, mesh_samples x mesh_samples points a cell. 0: J at the lattice points.
  double mesh_m = 0.0;
  int mesh_samples = 4;
  std::vector<double> times_s = {0.0, 3.7}; // the times the sea is sampled at
};

// The open sea's least Jacobian over a lattice, at a few times, and the foam it makes.
class foam_field {
public:
  // `cascades` are build_h0's fields (any n, any tiles). Throws std::invalid_argument when there
  // are none, a field is empty, spacing_m <= 0, ramp <= 0 or times_s is empty.
  explicit foam_field(const std::vector<h0_field> &cascades, const foam_params &p = {});

  // The mean foam over the drawn surface at this bias (each sample weighed by |J|, its share of
  // the surface; the fray averaged over): the share of the open sea whitened.
  double coverage(double bias) const;
  // The bias whose coverage is `share` (clamped to [0, 1]); coverage rises with the bias, so
  // this is a bisection. At 0 it is below the least sampled J (no foam).
  double bias_for(double share) const;
  // The share of the drawn surface at least `level` white (0 < level <= 1) at this bias.
  double white(double bias, double level = 0.5) const;

  std::size_t samples() const noexcept { return _j.size(); }
  double least() const noexcept { return _j.empty() ? 1.0 : double(_j.front()); }
  double mean() const noexcept { return _mean; }
  // The share of the samples (unweighted) with J < j (the fold's share when j is 0).
  double below(double j) const;

private:
  double coverage_at(double bias) const; // without the fray
  std::vector<float> _j;                 // the samples' least J, ascending
  double _ramp = 0.1;
  double _fray = 0.0;
  std::vector<double> _fray_q; // n - 0.5 at 16 quantiles of the fray noise
  double _mean = 1.0;
  std::vector<double> _area; // the sums of |J| (the drawn surface) of the first i samples
};

// ── Persistent foam (OCEAN-AND-WEATHER-ROADMAP.md section 10.2, O5) ──────────────────────
//
// A breaking wave leaves foam behind that thins over seconds. The drawn sea keeps it in a
// turbulence texture beside its long cascades, in each cascade's own texels (OceanFoam, in cvcGL):
// each frame a texel takes its cascade's fold, smoothstep((bias - J) / ramp) with J that
// cascade's Jacobian, and its foam builds at build_per_s a second while it folds and decays as
// exp(-t / decay_s) after (foam_step). Bubbles build at bubble_gain of that rate and decay more
// slowly; the crest channel is the injection itself and the spray channel latches it, decaying in
// a sixth of a second (for spray particles, section 10.5; nothing draws them yet).
//
// Only the first `cascades` cascades (the long waves: lsystem_coast's and the water node's 251 m
// and 83 m tiles, waves over about 7 m) leave foam that lasts. The short cascade's folds are its
// metre-long waves flickering in and out in a fraction of a second: integrated, they would whiten
// the sea evenly, which is not what breaking looks like, and their 29 m tile would print its
// pattern on the foam. They keep the instantaneous whitecaps alone. (ABYSSAL weights its short
// cascade's foam by 0.22 and fades it four times faster, for the same reason.)
//
// The water's fragment shader draws the stronger cascade's foam carved into windrows (foam_look):
// fresh foam is white with lace in it, and as it ages only the streaks the windrow noise picks
// survive, until it is gone (below about 0.2). The instantaneous whitecaps (foam_field's:
// smoothstep((bias - J + fray (n - 0.5)) / ramp) on the least of all the cascades' J) draw on top,
// at the fragment's resolution.
//
// Together they cover Monahan's share for the wind: foam_history evolves the cascades' h0 on the
// CPU over a few seconds, runs foam_step on every texel of the long cascades, and measures what
// the shader would draw on a lattice of the open sea. The aging foam alone takes `share` of it
// (its own bias), and the whitecaps' bias makes up the rest.

struct foam_persistence {
  double decay_s = 2.5;        // the foam's e-folding time (s); 0: no persistent foam
  double build_per_s = 4.0;    // foam gained a second while a texel folds fully (inject 1)
  double bubble_decay_s = 6.0; // the bubbles' e-folding time (s)
  double bubble_gain = 0.55;   // the bubbles' share of the foam's injection
  double spray_decay_s = 1.0 / 6.0;
  double max_dt_s = 0.25; // a longer step (a stall, a jump of the clock) integrates this much
  int cascades = 2;       // the first this many cascades leave persistent foam
  double share = 0.6;     // the share of Monahan's cover the aging foam alone draws
};

// One texel's turbulence: (foam, bubbles, crest, spray), each in [0, 1], OceanFoam's RGBA.
struct foam_state {
  double foam = 0.0, bubbles = 0.0, crest = 0.0, spray = 0.0;
};

// The fold's injection at Jacobian j: smoothstep((bias - j) / ramp), ramp > 0.
double foam_inject(double j, double bias, double ramp);

// One step of dt_s seconds (clamped to [0, max_dt_s]) with injection `inject` in [0, 1], as
// OceanFoam's pass runs it (in floats there):
//   foam    <- clamp(foam exp(-dt / decay_s) + inject build_per_s dt, 0, 1)
//   bubbles <- clamp(bubbles exp(-dt / bubble_decay_s) + inject build_per_s dt bubble_gain, 0, 1)
//   crest   <- inject
//   spray   <- max(spray exp(-dt / spray_decay_s), smoothstep(0.45, 0.95, inject))
// A decay time of 0 or less keeps nothing from the step before.
void foam_step(foam_state &s, double inject, double dt_s, const foam_persistence &p);

// The drawn whiteness of persistent foam `foam` at a point whose windrow noise is w (in [0, 1]):
// smoothstep((foam (0.1 + 1.6 w) - 0.35) / 0.3). The shader's windrow noise is
// 0.45 v1 + 0.3 v2 + 0.25 v3, three scales of value noise: streaks downwind about 12 m by 2.5 m
// and 3 m by 0.8 m, and a lace 0.5 m across that makes the edges ragged (fading to its mean, 0.5,
// as a pixel grows past it).
double foam_look(double foam, double windrow);

struct foam_history_params {
  // The longest cascade's Jacobian is evaluated every step_s seconds, a shorter one's every
  // step_s sqrt(tile / longest tile) (its waves are faster); the integration steps dt_s, the
  // Jacobian a Catmull-Rom spline in time through the evaluations.
  double step_s = 0.4;
  double dt_s = 1.0 / 24.0;
  // The history before the first sample (0: twice decay_s; older foam has thinned past drawing),
  // then the samples, seconds after it.
  double window_s = 0.0;
  std::vector<double> samples_s = {0.0, 0.5, 1.0};
  double t0_s = 0.0; // the history's first time
};

// The two thresholds the drawn foam takes: the instantaneous whitecaps' on the least J of all the
// cascades (the shader's uFoamBias) and the persistent foam's injection on each long cascade's J.
struct foam_biases {
  double whitecaps = 0.8;
  double persistent = 0.9;
};

// The drawn foam's share of the open sea with persistent foam: the instantaneous whitecaps and the
// persistent foam drawn with foam_look, the larger of the two at each point, weighted by |J| as
// foam_field weighs. The lattice is foam_field's (its spacing_m, or the mesh's vertices and
// mesh_samples points a cell, J linear over each triangle), the persistent foam read at each point
// from the nearest texel of each long cascade, as the shader reads it. The windrow noise is
// averaged over 64 quantiles of its distribution (foam_look is monotone in the foam, so the
// stronger cascade's foam decides), and the fray takes foam_field's 16 quantiles in turn from
// point to point. Deterministic.
//
// The Jacobians are the work: units() of them, one cascade at one time each (two n x n
// transforms): the long cascades' over the history, and every cascade's at the samples. step()
// does them a few at a time, so a caller can spread them over frames or hand them to a worker;
// the shares and biases_for() need them all (ready()).
class foam_history {
public:
  // Throws std::invalid_argument as foam_field does, or for decay_s <= 0, build_per_s < 0,
  // cascades < 1, share outside [0, 1), step_s or dt_s <= 0, or samples_s empty or negative.
  foam_history(const std::vector<h0_field> &cascades, const foam_params &p,
               const foam_persistence &q, const foam_history_params &h = {});

  std::size_t units() const noexcept { return _units.size(); }
  std::size_t units_done() const noexcept { return _done; }
  bool ready() const noexcept { return _done == _units.size(); }
  // Does up to n units (0: all); true once all are done.
  bool step(std::size_t n = 1);

  // Need ready(); throw std::logic_error before.
  double coverage(const foam_biases &b) const; // what the shader draws
  double persistent(double bias) const;        // the aging foam alone, at its injection's bias
  double active(double bias) const;            // the whitecaps alone, at theirs
  // The persistent bias whose aging foam alone covers share x `share` of the sea, then the
  // whitecaps' bias that brings the drawn foam to `share` (each a bisection).
  foam_biases biases_for(double share) const;

private:
  // The aging foam's look at every lattice point and sample, at injection bias `bias`.
  std::vector<float> looks(double bias) const;
  double drawn(double whitecaps, const std::vector<float> *looks) const; // looks null: none
  void lattice(); // once every Jacobian is in: the lattice's points, their texels and J

  std::vector<h0_field> _h;
  foam_params _p;
  foam_persistence _q;
  foam_history_params _hp;
  std::size_t _np = 0; // the long cascades
  double _window = 0.0, _end = 0.0, _side = 0.0;
  std::vector<double> _step;                        // each long cascade's time between evaluations
  std::vector<std::vector<std::vector<float>>> _j;  // [long cascade][evaluation] n x n Jacobians
  std::vector<std::vector<float>> _jmin;            // [long cascade] each texel's least J in it
  std::vector<std::vector<std::vector<float>>> _js; // [sample][cascade] the Jacobians at samples
  struct unit {
    std::size_t cascade, index;
    bool sample; // a sample's field (index the sample), else a history's (index its evaluation)
  };
  std::vector<unit> _units;
  std::size_t _done = 0;
  std::vector<double> _fray_q; // as foam_field's
  std::vector<double> _look;   // foam_look averaged over the windrow noise, at foam i / 256
  double _dt = 0.0;            // the integration's step
  std::vector<long> _at;       // the integration step each sample is read after
  std::vector<std::vector<std::uint32_t>> _texel; // [long cascade] each lattice point's texel
  std::vector<float> _pj;   // [sample][point] the least J the shader sees there
  std::vector<float> _area; // [sample][point] its weight, |J|
  double _area_sum = 0.0;
};

} // namespace ocean
} // namespace cvc

#endif // CVC_OCEAN_WHITECAPS_H
