/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// tight_cocone's parameters as a parameter table (cvc/core/param_table.h,
// VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.5): their state keys and properties, the `cvc cocone`
// flags, the DSL dict cocone.tight takes and pycvc's keywords. The fields, names and defaults are
// skeleton_params' Tight Cocone fields (cvc/geometry/skeleton_params.h).

#ifndef CVC_GEOMETRY_TIGHT_COCONE_PARAMS_H
#define CVC_GEOMETRY_TIGHT_COCONE_PARAMS_H

#include <cvc/core/param_table.h>
#include <cvc/geometry/tight_cocone.h>
#include <string>
#include <tuple>
#include <vector>

namespace cvc {

template <> struct param_table<tight_cocone_params> : param_fields<tight_cocone_params> {
  using P = tight_cocone_params;
  static constexpr double pi = 3.14159265358979323846;
  static constexpr auto fields = std::tuple{
      field("robust", &P::robust, doc("filter a noisy sample with Robust Cocone first")),
      field("big_ball_ratio", &P::big_ball_ratio, range(0.0, 1e300),
            doc("Robust Cocone: a Delaunay ball is big when its squared radius exceeds this "
                "times a corner's mean squared nearest-neighbour distance")),
      field("theta_if", &P::theta_if, unit("rad"), range(0.0, pi / 2),
            doc("Robust Cocone: the angle below which a big ball meets the infinite one deeply")),
      field("theta_ff", &P::theta_ff, unit("rad"), range(0.0, pi),
            doc("Robust Cocone: the angle below which two big balls meet deeply")),
      field("cocone_phi", &P::cocone_phi, unit("rad"), range(0.0, pi / 2),
            doc("Tight Cocone: half the cocone's opening angle around the surface")),
      field("sharp_phi", &P::sharp_phi, unit("rad"), range(0.0, pi),
            doc("Tight Cocone: facets meeting at more than this dihedral angle make a sharp edge "
                "(a constant in the legacy library)")),
      field("flat_phi", &P::flat_phi, unit("rad"), range(0.0, pi / 2),
            doc("Tight Cocone: the normal deviation allowed between cocone neighbours of a flat "
                "point")),
      field("flatness_ratio", &P::flatness_ratio, range(0.0, 1e300),
            doc("Tight Cocone: the squared thinness factor of a flat point")),
  };
};

} // namespace cvc

#endif // CVC_GEOMETRY_TIGHT_COCONE_PARAMS_H
