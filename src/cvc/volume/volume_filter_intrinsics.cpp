/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// volume.filter in the DSL. See inc/cvc/volume/volume_filter_intrinsics.h.

#include <cvc/ariadne/ariadne.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_filter.h>
#include <cvc/volume/volume_filter_intrinsics.h>
#include <memory>
#include <string>
#include <utility>

namespace cvc {

namespace se = state_exec;

job_op volume_filter_op(sliced_threading threading) {
  job_op op = make_job_op<volume_filter_params, std::shared_ptr<const volume>>(
      "volume.filter", 1,
      [](const job_call &c, const volume_filter_params &p) {
        return volume_filter_job(c.input_as<volume>(0), p);
      },
      [](std::shared_ptr<const volume> &&v) {
        auto obj = std::make_shared<se::data_object>();
        obj->payload = std::move(v);
        obj->type_name = "cvc::volume";
        return se::value_t(std::move(obj));
      },
      [](const job_call &c, const volume_filter_params &) {
        const std::string *path = std::get_if<std::string>(&c.input(0).v);
        return path ? *path : std::string("a volume");
      });
  op.threading = threading;
  return op;
}

void register_volume_filter_intrinsics(se::environment_ptr env, se::intrinsics_context &ictx,
                                       cvc::app &app) {
  register_job_op(env, ictx, app, volume_filter_op());
  register_job_intrinsics(env, app);
}

void install_volume_filter_intrinsics(cvc::app &app) {
  cvc::app *const a = &app;
  ariadne::register_action_intrinsics(
      app, "cvc.volume-filter",
      [a](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
        register_volume_filter_intrinsics(std::move(env), ictx, *a);
      });
}

} // namespace cvc
