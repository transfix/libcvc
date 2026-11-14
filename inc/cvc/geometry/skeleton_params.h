/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// skeletonize's parameters as a parameter table (cvc/core/param_table.h,
// VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.5): their state keys and properties, the `cvc skeleton`
// flags, the DSL dict skeleton.compute takes and pycvc's keywords.

#ifndef CVC_GEOMETRY_SKELETON_PARAMS_H
#define CVC_GEOMETRY_SKELETON_PARAMS_H

#include <cvc/core/param_table.h>
#include <cvc/geometry/skeleton.h>
#include <string>
#include <tuple>
#include <vector>

namespace cvc {

template <> struct param_table<skeleton_params> : param_fields<skeleton_params> {
  using P = skeleton_params;
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
                "(hard-coded in VolumeRover2)")),
      field("flat_phi", &P::flat_phi, unit("rad"), range(0.0, pi / 2),
            doc("Tight Cocone: the normal deviation allowed between cocone neighbours of a flat "
                "point (uninitialised in VolumeRover2's dialog)")),
      field("flatness_ratio", &P::flatness_ratio, range(0.0, 1e300),
            doc("Tight Cocone: the squared thinness factor of a flat point")),
      field("medial_theta", &P::medial_theta, unit("rad"), range(0.0, pi / 2),
            doc("medial axis: a Delaunay edge within this angle of a tangent plane is not chosen "
                "by the angle condition")),
      field("medial_ratio", &P::medial_ratio, range(0.0, 1e300),
            doc("medial axis: a Delaunay edge is chosen when its squared length exceeds this "
                "times both ends' umbrella diameters")),
      field("discard_small", &P::discard_small,
            doc("sheets: keep every cluster of at least threshold times the largest area, "
                "instead of the largest sheet_count")),
      field("threshold", &P::threshold, range(0.0, 1.0),
            doc("sheets: the area fraction discard_small keeps")),
      field("sheet_count", &P::sheet_count, range(0, 1e9),
            doc("sheets: how many of the largest planar clusters are sheets (the others are "
                "stars)")),
      field("helix_window", &P::helix_window,
            doc("also output each strand's Voronoi vertices of cell radius in (2, 3), the legacy "
                "protein-helix filter")),
  };
};

} // namespace cvc

#endif // CVC_GEOMETRY_SKELETON_PARAMS_H
