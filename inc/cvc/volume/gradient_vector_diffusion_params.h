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

// The parameter table of cvc::gradient_vector_diffusion_params (cvc/core/param_table.h): its
// state keys, validate() and the rest, from one declaration
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.5).

#ifndef CVC_VOLUME_GRADIENT_VECTOR_DIFFUSION_PARAMS_H
#define CVC_VOLUME_GRADIENT_VECTOR_DIFFUSION_PARAMS_H

#include <cvc/core/param_table.h>
#include <cvc/volume/gradient_vector_diffusion.h>
#include <tuple>

namespace cvc {

template <>
struct param_table<gradient_vector_diffusion_params>
    : param_fields<gradient_vector_diffusion_params> {
  using P = gradient_vector_diffusion_params;
  static constexpr auto fields = std::tuple{
      field("iterations", &P::iterations, range(0, 100000),
            doc("iterations of the diffusion (SegMed: 10)")),
      field("k", &P::k, range(0.0, 1000.0),
            doc("Eq. 3's k: a neighbour weighs exp(k (cos a - 1)), a the angle between the two "
                "vectors")),
      field("weights", &P::weights,
            doc("the field whose angles weigh the neighbours: current (the field being diffused, "
                "the paper) or initial (the field before the first iteration, SegMed)")),
      field("normalize", &P::normalize, doc("divide the result by its longest vector's length")),
  };
};

} // namespace cvc

#endif // CVC_VOLUME_GRADIENT_VECTOR_DIFFUSION_PARAMS_H
