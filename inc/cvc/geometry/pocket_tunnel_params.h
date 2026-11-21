/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// pocket_tunnel's parameters as a parameter table (cvc/core/param_table.h,
// VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.5): their state keys and properties, and later the CLI
// flags, the DSL dict and pycvc's keywords.

#ifndef CVC_GEOMETRY_POCKET_TUNNEL_PARAMS_H
#define CVC_GEOMETRY_POCKET_TUNNEL_PARAMS_H

#include <cvc/core/param_table.h>
#include <cvc/geometry/pocket_tunnel.h>
#include <string>
#include <tuple>
#include <vector>

namespace cvc {

template <> struct param_table<pocket_tunnel_params> : param_fields<pocket_tunnel_params> {
  using P = pocket_tunnel_params;
  static constexpr double pi = 3.14159265358979323846;
  static constexpr auto fields = std::tuple{
      field("pockets", &P::pockets, range(0, 1e9),
            doc("how many of the largest pockets (one mouth) the output surface shows")),
      field("tunnels", &P::tunnels, range(0, 1e9),
            doc("how many of the largest tunnels (two or more mouths) and voids (none) the output "
                "surface shows")),
      field("cocone_phi", &P::cocone_phi, unit("rad"), range(0.0, pi / 2),
            doc("Tight Cocone: half the cocone's opening angle around the surface")),
      field("sharp_phi", &P::sharp_phi, unit("rad"), range(0.0, pi),
            doc("Tight Cocone: facets meeting at more than this dihedral angle make a sharp edge")),
      field("flat_phi", &P::flat_phi, unit("rad"), range(0.0, pi / 2),
            doc("Tight Cocone: the normal deviation allowed between cocone neighbours of a flat "
                "point")),
      field("flatness_ratio", &P::flatness_ratio, range(0.0, 1e300),
            doc("Tight Cocone: the squared thinness factor of a flat point")),
  };
};

} // namespace cvc

#endif // CVC_GEOMETRY_POCKET_TUNNEL_PARAMS_H
