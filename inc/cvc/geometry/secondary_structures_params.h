/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// find_secondary_structures' parameters as a parameter table (cvc/core/param_table.h,
// VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.5): their state keys and properties, the CLI flags, the DSL
// dict and pycvc's keywords.

#ifndef CVC_GEOMETRY_SECONDARY_STRUCTURES_PARAMS_H
#define CVC_GEOMETRY_SECONDARY_STRUCTURES_PARAMS_H

#include <cvc/core/param_table.h>
#include <cvc/geometry/secondary_structures.h>
#include <string>
#include <tuple>
#include <vector>

namespace cvc {

#ifndef SWIG
template <> struct param_table<secondary_params> : param_fields<secondary_params> {
  using P = secondary_params;
  static constexpr double pi = 3.14159265358979323846;
  static constexpr auto fields = std::tuple{
      field("cocone_phi", &P::cocone_phi, unit("rad"), range(0.0, pi / 2),
            doc("Tight Cocone: half the cocone's opening angle around the surface")),
      field("sharp_phi", &P::sharp_phi, unit("rad"), range(0.0, pi),
            doc("Tight Cocone: facets meeting at more than this dihedral angle make a sharp "
                "edge")),
      field("flat_phi", &P::flat_phi, unit("rad"), range(0.0, pi / 2),
            doc("Tight Cocone: the normal deviation allowed between cocone neighbours of a flat "
                "point")),
      field("flatness_ratio", &P::flatness_ratio, range(0.0, 1e300),
            doc("Tight Cocone: the squared thinness factor of a flat point")),
      field("medial_theta", &P::medial_theta, unit("rad"), range(0.0, pi / 2),
            doc("medial axis: a Delaunay edge within this angle of a tangent plane is not chosen "
                "by the angle condition")),
      field("medial_ratio", &P::medial_ratio, range(0.0, 1e300),
            doc("medial axis: a Delaunay edge is chosen when its squared length exceeds this "
                "times both ends' umbrella diameters")),
      field("helix_min_width", &P::helix_min_width, range(0.0, 1e300),
            doc("helices: the narrowest U2 edge kept (the radius of the tube around it)")),
      field("helix_max_width", &P::helix_max_width, range(0.0, 1e300),
            doc("helices: the widest U2 edge kept")),
      field("helix_join_distance", &P::helix_join_distance, range(0.0, 1e300),
            doc("helices: candidate points nearer than this are in one cluster")),
      field("helix_min_length", &P::helix_min_length, range(0.0, 1e300),
            doc("helices: a fitted axis shorter than this is not a helix")),
      field("helix_count", &P::helix_count, range(0, 1e9),
            doc("helices: how many of the longest axes are the helices")),
      field("helix_radius", &P::helix_radius, range(0.0, 1e300),
            doc("helices: the radius of the cylinders drawn")),
      field("sheet_min_width", &P::sheet_min_width, range(0.0, 1e300),
            doc("sheets: the thinnest polygon drawn (the length of its dual Delaunay edge)")),
      field("sheet_max_width", &P::sheet_max_width, range(0.0, 1e300),
            doc("sheets: the thickest polygon drawn")),
      field("sheet_min_area", &P::sheet_min_area, range(0.0, 1e300),
            doc("sheets: a component of polygons smaller than this is never a sheet")),
      field("sheet_count", &P::sheet_count, range(0, 1e9),
            doc("sheets: how many of the largest components are the sheets")),
  };
  static void check(const P &p, std::vector<std::string> &problems) {
    if (p.helix_min_width > p.helix_max_width)
      problems.push_back("helix_min_width: more than helix_max_width");
    if (p.sheet_min_width > p.sheet_max_width)
      problems.push_back("sheet_min_width: more than sheet_max_width");
  }
};
#endif // SWIG

} // namespace cvc

#endif // CVC_GEOMETRY_SECONDARY_STRUCTURES_PARAMS_H
