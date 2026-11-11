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

// sea_surface.h — the sea a floating body feels (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md,
// B2-B4): what lsystem_coast's water vertex shader draws, on the CPU, without the waves too short
// to move a body.
//
//   std::vector<cvc::ocean::sea_cascade> cs;   // each cascade's h0, as OceanFFT is given it
//   cs.push_back({cvc::ocean::build_h0(256, band, sea), fade_m, decay_k});
//   cvc::ocean::sea_surface s(std::move(cs), params, grid, layer);
//   s.height(x, y, t);          // the surface over (x, y) at time t
//   s.velocity(x, y, z, t);     // the water's velocity there
//
// The sea is the shader's sum:
//   - The cascades (B2). Each holds the waves of one wavenumber band on a periodic tile. The CPU
//     keeps its lowest band.modes x band.modes wavenumbers, evolves them to time t as OceanFFT
//     does, h(k, t) = h0(k) e^(i w t) + conj h0(-k) e^(-i w t), and transforms them onto a
//     band.grid x band.grid lattice of the tile: the height, the sideways moves i k/|k| h and
//     their time derivatives, packed two to a complex transform. The fields are read between the
//     lattice points by Catmull-Rom splines, wrapped. OceanFFT keeps all 256 x 256; the waves the
//     CPU drops are a few centimetres long, and the CPU-vs-GPU residual (B6) measures them.
//   - Near the shore (A2) each cascade fades with the still-water depth (smoothstep over fade_m),
//     and the sideways move is capped at shore_limit of the distance to the shore.
//   - The shore layer (A3, shore.h): the swell, refracted, shoaled and breaking.
//   - On the beach, the swash (shore_layer::swash): the water runs up to the front.
//   - wave_amp scales the cascades' heights and chop their sideways moves, as the shader's
//     uWaveAmp and uChop x OceanFFT::chop do; the shore layer is drawn at its true size.
//
// Height at a point (B3). The shader moves each point of the sea sideways by D, so the surface
// over (x, y) is the point that started at x0 with x0 + D(x0) = (x, y); its height is the level
// plus the cascades' and the shore layer's heights at x0. The roadmap's fixed point x0 <- (x, y) -
// D(x0) converges only by |dD/dx0| a step, slowly on steep, choppy water, so x0 is found by
// Broyden's method instead: the same first step and the same one evaluation of D a step, with a
// secant estimate of the Jacobian after that. On a steep test sea (chop 1.5, the whole spectrum
// down to 6 m waves) the worst miss after 4 steps is 3 cm where the fixed point's is 58 cm, and
// after 8 steps 1e-7 m.
//
// Velocity (B4). The cascades' particle velocity at x0, the time derivative of their moves,
// decaying below the still level as e^(decay_k z) (the cascade's peak wavenumber, deep water),
// plus the shore layer's orbital velocity in the local depth (shore_layer::velocity, with the
// breakers' bores). On the beach, the swash runs up and drains along the slope, at the front's
// rise over the slope, at most sqrt(2 g R).
//
// Determinism: everything is computed with cvc::algo::det and the FFT in fft.h, without FMA
// contraction, so a surface gives the same bits on every platform; physics folds digest() into
// its state_hash. The fields for one time are computed once and kept, so the many reads of one
// physics step share them; that cache does not change any value. The class is immutable and safe
// to read from several threads.
//
// Standard library only, like the rest of cvc::ocean.

#ifndef CVC_OCEAN_SEA_SURFACE_H
#define CVC_OCEAN_SEA_SURFACE_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cvc/ocean/fft.h>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/shore.h>
#include <cvc/ocean/spectrum.h>
#include <memory>
#include <mutex>
#include <vector>

namespace cvc {
namespace ocean {

// The physics band of each cascade (B2).
struct band_params {
  int modes = 32; // the lowest modes x modes wavenumbers kept (a power of two, at most h0's n)
  int grid = 64;  // the lattice they are transformed onto (a power of two, at least modes)
};

// One cascade, as lsystem_coast draws it.
struct sea_cascade {
  h0_field h0;          // build_h0's texels: what OceanFFT uploads
  double fade_m = 0.0;  // the still-water depth over which it fades out at the shore; 0: never
  double decay_k = 0.0; // its velocity decays as e^(decay_k z) below the still level; 0: never
};

struct sea_params {
  double level = 0.0;       // the still water (m, world z)
  double depth_m = 1000.0;  // the depth the cascades disperse in (OceanFFT::depth_m)
  double gravity = 9.80665; // m/s^2 (OceanFFT's)
  double wave_amp = 1.0;    // scales the cascades' heights (lsystem_coast's uWaveAmp)
  double chop = 1.0;        // scales their sideways moves (uChop x OceanFFT::chop)
  band_params band;         // B2
  int point_iterations = 4; // B3's steps (evaluations of D), as Arc Blanc's 4
  double shore_limit = 0.8; // a sideways move stays under this share of the way to the shore
  bool bores = true;        // breaking crests carry the water shoreward (shore_layer::velocity)
  bool swash = true;        // the swash runs up the beach
};

class sea_surface {
public:
  // `grid` gives the still-water depth (for the fades, the shore layer's velocity and the beach)
  // and the shore distance (for the limit); `layer` the swell at the coast. Either may be null:
  // without a grid the sea is deep and has no shore. Throws std::invalid_argument for parameters
  // out of range, a cascade whose h0 is smaller than band.modes or not square, or a grid whose
  // planes do not match its size.
  sea_surface(std::vector<sea_cascade> cascades, const sea_params &p,
              std::shared_ptr<const shore_grid> grid = nullptr,
              std::shared_ptr<const shore_layer> layer = nullptr);

  const sea_params &params() const noexcept { return _p; }
  std::size_t cascade_count() const noexcept { return _cascades.size(); }

  // The surface's height (m, world z) over (x, y) at time t (s). On dry ground it is the swash
  // front, or the still level where the swash does not reach (below the ground, so dry).
  double height(double x, double y, double t) const;
  // The water's velocity (m/s) at (x, y, z) at time t.
  std::array<double, 3> velocity(double x, double y, double z, double t) const;
  // The point x0 of the undisplaced sea that the shader moves over (x, y) at time t (B3).
  std::array<double, 2> source(double x, double y, double t) const;
  // The sideways move D the shader gives the undisplaced point (x, y) at time t: the cascades'
  // (faded, times chop) plus the shore layer's, capped by shore_limit.
  std::array<double, 2> displacement(double x, double y, double t) const;
  // The still-water depth at (x, y): level - ground; +infinity without a grid.
  double depth(double x, double y) const;

  // A 64-bit digest of everything the values depend on (state_hash folds it in).
  std::uint64_t digest() const noexcept { return _digest; }

  // Cascade c's band at time t on its lattice, row-major (row r at y = r tile / grid, column c at
  // x = c tile / grid): the height, the sideways moves and their time derivatives, before
  // wave_amp, chop, the fades and the shore. What the CPU-vs-GPU residual compares with
  // OceanFFT's texels (B6).
  struct band_frame {
    int grid = 0;
    double tile_m = 0.0;
    std::vector<double> eta, dx, dy, u, v, w;
  };
  band_frame band(std::size_t c, double t) const;

private:
  struct mode {
    int gx = 0, gy = 0; // the mode's place on the lattice (its wavenumber index, wrapped)
    double kx = 0.0, ky = 0.0, knx = 0.0, kny = 0.0, omega = 0.0;
    double h0r = 0.0, h0i = 0.0, hcr = 0.0, hci = 0.0; // h0(k) and conj h0(-k)
  };
  struct cascade_band {
    double tile_m = 0.0, fade_m = 0.0, decay_k = 0.0;
    std::vector<mode> modes;
  };
  struct frame; // the cascades' fields at one time

  std::shared_ptr<const frame> frame_at(double t) const;
  std::shared_ptr<frame> compute(double t) const;
  // The sideways move D at x0 (the shader's dh, limited), from frame f.
  std::array<double, 2> move(const frame &f, double x, double y, double t) const;
  std::array<double, 2> source(const frame &f, double x, double y, double t) const;
  double grid_sample(const std::vector<float> &plane, double x, double y) const;

  sea_params _p;
  fft_plan _fft; // band.grid points
  std::vector<cascade_band> _cascades;
  std::shared_ptr<const shore_grid> _grid;
  std::shared_ptr<const shore_layer> _layer;
  std::uint64_t _digest = 0;

  mutable std::mutex _cache_mutex;
  mutable std::shared_ptr<const frame> _cache;
};

} // namespace ocean
} // namespace cvc

#endif // CVC_OCEAN_SEA_SURFACE_H
