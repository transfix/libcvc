/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::mol::blur: a molecule's Gaussian-blob density on a grid, TexMol's blur map (Blurmaps'
// GOABlur, docs/roadmap/TEXMOL-F2DOCK-ARISRV-ROADMAP.md §2.2, T1).
//
//   cvc::mol::molecule m = cvc::mol::read_molecule(app, "1grl.pdb");
//   cvc::mol::blur_params p;
//   p.resolution = 1.5;                       // Å between grid points
//   cvc::volume density = cvc::mol::blur(app, m, p);
//   // its isosurface at 1 lies near the atoms' van der Waals spheres; a blobbiness nearer 0
//   // smooths it toward a low-resolution map, where helices read as rods
//
// Each atom of radius R at c adds exp(B (|x - c|^2 / R^2 - 1)) to every grid point x within its
// reach, R sqrt(1 + ln(cutoff) / B), where it has fallen to `cutoff`. B is the blobbiness, negative;
// at |x - c| = R an atom adds exactly 1. The grid's box is the atoms' centres padded by `padding`
// times the largest radius, and its spacing comes from `resolution` (the grid then has
// extent / resolution + 3 points along each axis) or from `dims`.
//
// It computes what GOABlur computed, in the same arithmetic: float positions, box and spacing,
// double kernels, float sums, atoms in order and each atom's points z, y, x. So with the defaults
// (a centre moved to its nearest grid point first, as GOABlur did) the volume is TexMol's, voxel
// for voxel (tools/texmol-goldens). snap_centres off keeps each centre where it is.
//
// blur_stepper does it a slice at a time (VOLROVER plan §1.9): phases "checking" (the parameters
// and the box, a unit per atom) and "splatting" (a unit per grid point an atom reaches). Each unit
// is one exp() or less. Everything here holds no state of its own: no statics, no globals.

#ifndef CVC_MOL_BLUR_H
#define CVC_MOL_BLUR_H

#include <cstddef>
#include <cvc/core/param_table.h>
#include <cvc/core/result_job.h>
#include <cvc/mol/molecule.h>
#include <cvc/volume/volume.h>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace cvc {
class app;

namespace mol {

struct blur_params {
  // B in exp(B (r^2 / R^2 - 1)); negative. TexMol's default, -2.3442, is Grant and Pickup's.
  double blobbiness = -2.3442;
  double resolution = 1.0; // Å between grid points, unless `dims` gives the grid
  std::vector<int> dims;   // the grid's points along x, y, z, instead of `resolution`
  double padding = 4.0;    // the box: the atoms' centres, padded by this many largest radii
  double cutoff = 1e-3;    // an atom reaches the grid points where it has fallen to this
  bool snap_centres = true; // move each atom's centre to its nearest grid point (GOABlur's way)
};

} // namespace mol

#ifndef SWIG
template <> struct param_table<mol::blur_params> : param_fields<mol::blur_params> {
  using P = mol::blur_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("blobbiness", &P::blobbiness, range(none, 0.0),
            doc("B in exp(B (r^2/R^2 - 1)), negative; nearer 0 blurs more")),
      field("resolution", &P::resolution, unit("Å"), range(0.0, none),
            doc("the spacing of the grid's points, unless dims gives the grid")),
      field("dims", &P::dims, range(2, none),
            doc("the grid's points along x, y and z, instead of resolution")),
      field("padding", &P::padding, range(0.0, none),
            doc("the box: the atoms' centres padded by this many of the largest radius")),
      field("cutoff", &P::cutoff, range(0.0, 1.0),
            doc("an atom reaches the grid points where its blob has fallen to this")),
      field("snap_centres", &P::snap_centres,
            doc("move each atom's centre to its nearest grid point first, as TexMol did")),
  };
  static void check(const P &p, std::vector<std::string> &problems) {
    if (!(p.blobbiness < 0))
      problems.push_back("blobbiness: must be negative");
    if (p.dims.empty() && !(p.resolution > 0))
      problems.push_back("resolution: must be positive");
    if (!p.dims.empty() && p.dims.size() != 3)
      problems.push_back("dims: " + std::to_string(p.dims.size()) + " values, not three (x, y, z)");
    if (!(p.cutoff > 0 && p.cutoff < 1))
      problems.push_back("cutoff: must lie between 0 and 1");
  }
};
#endif // SWIG

namespace mol {

// A molecule's density, a slice at a time. A failure (parameters that aren't valid, a molecule with
// no atoms, a grid too large to allocate) is a std::invalid_argument or std::bad_alloc thrown from
// step() and rethrown by later steps and take(). cancel() is the one thread-safe member: the step
// running (or the next) ends the stepper with no result. Not thread-safe otherwise, but it may move
// between threads between steps. Constructing one copies the atoms' positions and radii.
class blur_stepper {
public:
  blur_stepper(app &ctx, const molecule &m, blur_params params = {});
  ~blur_stepper();
  blur_stepper(blur_stepper &&) noexcept;
  blur_stepper &operator=(blur_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const; // "checking", "splatting", "done", "failed" or "cancelled"
  double progress() const;   // in [0, 1], monotone; 1 once done
  std::string status() const;
  void cancel();
  volume take(); // once done: the density (float), its box the grid's; moved out, call it once

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// The density at once: blur_stepper run to the end.
volume blur(app &ctx, const molecule &m, const blur_params &params = {});

std::unique_ptr<result_job<volume>> blur_job(app &ctx, const molecule &m,
                                             const blur_params &params = {});

} // namespace mol
} // namespace cvc

#endif // CVC_MOL_BLUR_H
