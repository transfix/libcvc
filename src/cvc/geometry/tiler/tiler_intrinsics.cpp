/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// tiler.tile (cvc/geometry/tiler_intrinsics.h): tile_contours_job behind the generic job forms of
// cvc/core/job_intrinsics.h.

#include <cvc/ariadne/ariadne.h>
#include <cvc/geometry/contours.h>
#include <cvc/geometry/tiler.h>
#include <cvc/geometry/tiler_intrinsics.h>
#include <cvc/geometry/tiler_params.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/types.h>
#include <string>
#include <utility>
#include <vector>

namespace cvc {

namespace se = state_exec;

job_op tiler_tile_op(sliced_threading threading) {
  job_op op = make_job_op<tile_params, tile_result>(
      "tiler.tile", 1,
      [](const job_call &c, const tile_params &p) {
        return tile_contours_job(*c.input_as<contour_stack>(0), p);
      },
      [](tile_result &&r) {
        std::vector<std::pair<std::string, se::value_t>> surfaces;
        for (auto &[name, g] : r.components) {
          auto obj = std::make_shared<se::data_object>();
          obj->payload = std::move(g);
          obj->type_name = "cvc::geometry";
          surfaces.emplace_back(name, se::value_t(std::move(obj)));
        }
        std::vector<se::value_t> warnings;
        for (std::string &w : r.warnings)
          warnings.push_back(se::value_t(std::move(w)));
        return se::make_dict({
            {"surfaces", se::make_dict(std::move(surfaces))},
            {"warnings", se::make_list(std::move(warnings))},
        });
      },
      [](const job_call &c, const tile_params &) {
        const std::string *path = std::get_if<std::string>(&c.input(0).v);
        return path ? *path : std::string("a contour stack");
      });
  op.threading = threading;
  return op;
}

void register_tiler_intrinsics(se::environment_ptr env, se::intrinsics_context &ictx,
                               cvc::app &app) {
  register_job_op(env, ictx, app, tiler_tile_op());
  register_job_intrinsics(env, app);
}

void add_tiler_intrinsics(cvc::app &app) {
  ariadne::register_action_intrinsics(
      app, "tiler", [&app](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
        register_tiler_intrinsics(env, ictx, app);
      });
}

} // namespace cvc
