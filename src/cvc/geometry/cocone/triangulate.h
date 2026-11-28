/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The steps every cocone port's stepper shares (VOLROVER-TEXMOL-APPS-ROADMAP.md §2.4, V1): how a
// stepper reports its stages' progress, and the 3D Delaunay triangulation of its points into the
// core's Triangulation. The skeleton, PocketTunnel, SecondaryStructures and Tight Cocone each ran
// a copy of this triangulation; now they run this one.

#ifndef CVC_GEOMETRY_COCONE_TRIANGULATE_H
#define CVC_GEOMETRY_COCONE_TRIANGULATE_H

#include "cocone.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cvc/geometry/tet_complex.h>
#include <string>
#include <utility>
#include <vector>

namespace cvc::cocone {

// A cocone stepper's progress: its phase's name, its progress in [0, 1] (monotone), and the span
// of [0, 1] the current stage covers with the units it expects there. A port's stepper context
// derives from it; its step() counts stage_units and calls note_progress every so many.
struct stage_progress {
  const char *phase_name = "checking";
  double prog = 0.0;
  double stage_lo = 0.0, stage_hi = 0.0;
  std::size_t stage_units = 0, stage_estimate = 1;

  void stage(const char *name, double lo, double hi, std::size_t estimate) {
    phase_name = name;
    stage_lo = lo;
    stage_hi = hi;
    stage_units = 0;
    stage_estimate = std::max<std::size_t>(estimate, 1);
  }
  void note_progress(double within) {
    const double p = stage_lo + (stage_hi - stage_lo) * std::min(1.0, std::max(0.0, within));
    prog = std::max(prog, std::min(p, 1.0));
  }
};

// tet_complex_builder units per stepper unit, in triangulate().
constexpr std::size_t BUILD_UNITS = 16;

// The Delaunay triangulation of pts (cvc::tet_complex, exact), made into triang: BUILD_UNITS of
// the builder's units per unit, its phases ("checking", "ordering", "inserting", "compacting",
// "circumcentres") reported as the stepper's, its progress spread over [lo, hi]; then "building",
// triang's vertices and cells, over [hi, hi + 0.02]. Adds the points the builder merged as
// duplicates to `duplicates`. A tet_complex_error (too few points, all in one plane, a coordinate
// that is not finite) is thrown again as an Error, "<who>: " and the builder's message.
template <class Error>
step_task triangulate(std::vector<std::array<double, 3>> pts, std::string who,
                      Triangulation &triang, stage_progress &sp, double lo, double hi,
                      std::size_t &duplicates) {
  sp.stage(sp.phase_name, lo, lo, 1);
  tet_complex t;
  {
    tet_complex_builder builder(std::move(pts), tet_complex_options{});
    for (;;) {
      bool ready = false;
      try {
        ready = builder.step(BUILD_UNITS);
      } catch (const tet_complex_error &e) {
        std::string what = e.what();
        const std::string prefix = "tet_complex: ";
        if (what.rfind(prefix, 0) == 0)
          what = what.substr(prefix.size());
        throw Error(who + ": " + what);
      }
      sp.phase_name = builder.phase();
      sp.prog = std::max(sp.prog, lo + (hi - lo) * builder.progress());
      if (ready)
        break;
      SK_UNIT;
    }
    t = builder.take(); // the builder's buffers go here, before the cells are made
  }
  duplicates += std::size_t(t.num_points() - t.num_vertices());
  triang.reset(std::move(t));
  sp.stage("building", hi, hi + 0.02, triang.fill_total());
  while (!triang.fill_one())
    SK_UNIT;
}

} // namespace cvc::cocone

#endif // CVC_GEOMETRY_COCONE_TRIANGULATE_H
