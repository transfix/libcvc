/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// skeleton.compute (cvc/geometry/skeleton_intrinsics.h): skeletonize_job behind the generic job
// forms of cvc/core/job_intrinsics.h.

#include <cvc/ariadne/ariadne.h>
#include <cvc/geometry/skeleton.h>
#include <cvc/geometry/skeleton_intrinsics.h>
#include <cvc/geometry/skeleton_params.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/types.h>
#include <string>
#include <utility>
#include <vector>

namespace cvc {

namespace se = state_exec;

namespace {

se::value_t geometry_object(geometry g, const char *what) {
  auto obj = std::make_shared<se::data_object>();
  obj->payload = std::move(g);
  (void)what;
  obj->type_name = "cvc::geometry";
  return se::value_t(obj);
}

} // namespace

job_op skeleton_compute_op(sliced_threading threading) {
  job_op op = make_job_op<skeleton_params, skeleton>(
      "skeleton.compute", 1,
      [](const job_call &c, const skeleton_params &p) {
        return skeletonize_job(*c.input_as<geometry>(0), p);
      },
      [](skeleton &&s) {
        std::vector<se::value_t> warnings;
        for (std::string &w : s.warnings)
          warnings.push_back(se::value_t(std::move(w)));
        return se::make_dict({
            {"strands", geometry_object(std::move(s.strands), "strands")},
            {"sheets", geometry_object(std::move(s.sheets), "sheets")},
            {"helix", geometry_object(std::move(s.helix), "helix")},
            {"polylines", se::value_t(std::int64_t(s.polylines))},
            {"clusters", se::value_t(std::int64_t(s.clusters))},
            {"sheet_clusters", se::value_t(std::int64_t(s.sheet_clusters))},
            {"warnings", se::make_list(std::move(warnings))},
        });
      },
      [](const job_call &c, const skeleton_params &) {
        const std::string *path = std::get_if<std::string>(&c.input(0).v);
        return path ? *path : std::string("a geometry");
      });
  op.threading = threading;
  return op;
}

void register_skeleton_intrinsics(se::environment_ptr env, se::intrinsics_context &ictx,
                                  cvc::app &app) {
  register_job_op(env, ictx, app, skeleton_compute_op());
  register_job_intrinsics(env, app);
}

void add_skeleton_intrinsics(cvc::app &app) {
  ariadne::register_action_intrinsics(
      app, "skeleton", [&app](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
        register_skeleton_intrinsics(env, ictx, app);
      });
}

} // namespace cvc
