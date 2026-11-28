/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cocone.tight (cvc/geometry/tight_cocone_intrinsics.h): tight_cocone_job behind the generic job
// forms of cvc/core/job_intrinsics.h.

#include <cvc/ariadne/ariadne.h>
#include <cvc/geometry/tight_cocone.h>
#include <cvc/geometry/tight_cocone_intrinsics.h>
#include <cvc/geometry/tight_cocone_params.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/types.h>
#include <string>
#include <utility>
#include <vector>

namespace cvc {

namespace se = state_exec;

job_op tight_cocone_op(sliced_threading threading) {
  job_op op = make_job_op<tight_cocone_params, tight_cocone_result>(
      "cocone.tight", 1,
      [](const job_call &c, const tight_cocone_params &p) {
        return tight_cocone_job(*c.input_as<geometry>(0), p);
      },
      [](tight_cocone_result &&r) {
        std::vector<se::value_t> warnings;
        for (std::string &w : r.warnings)
          warnings.push_back(se::value_t(std::move(w)));
        const auto triangles = std::int64_t(r.surface.num_tris());
        const auto surface_points = std::int64_t(r.surface.num_points());
        // As a dataset holds a geometry (data.cpp's mesh decoder), so data-put can put it in one
        // and a scene node draw it from there.
        auto surface = std::make_shared<se::data_object>();
        surface->payload =
            std::shared_ptr<const geometry>(std::make_shared<const geometry>(std::move(r.surface)));
        surface->type_name = "cvc::geometry";
        return se::make_dict({
            {"surface", se::value_t(std::move(surface))},
            {"triangles", se::value_t(triangles)},
            {"surface_points", se::value_t(surface_points)},
            {"points", se::value_t(std::int64_t(r.points))},
            {"duplicates", se::value_t(std::int64_t(r.duplicates))},
            {"robust_dropped", se::value_t(std::int64_t(r.robust_dropped))},
            {"cells", se::value_t(std::int64_t(r.cells))},
            {"inside_cells", se::value_t(std::int64_t(r.inside_cells))},
            {"warnings", se::make_list(std::move(warnings))},
        });
      },
      [](const job_call &c, const tight_cocone_params &) {
        const std::string *path = std::get_if<std::string>(&c.input(0).v);
        return path ? *path : std::string("a geometry");
      });
  op.threading = threading;
  return op;
}

void register_tight_cocone_intrinsics(se::environment_ptr env, se::intrinsics_context &ictx,
                                      cvc::app &app) {
  register_job_op(env, ictx, app, tight_cocone_op());
  register_job_intrinsics(env, app);
}

void install_tight_cocone_intrinsics(cvc::app &app) {
  cvc::app *const a = &app;
  ariadne::register_action_intrinsics(
      app, "cvc.tight-cocone",
      [a](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
        register_tight_cocone_intrinsics(std::move(env), ictx, *a);
      });
}

} // namespace cvc
