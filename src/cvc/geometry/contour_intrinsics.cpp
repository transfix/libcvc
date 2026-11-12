/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// contours.* in the DSL. See cvc/geometry/contour_intrinsics.h.

#include <cvc/ariadne/ariadne.h>
#include <cvc/core/app.h>
#include <cvc/core/job_intrinsics.h>
#include <cvc/core/param_dsl.h>
#include <cvc/geometry/contour_intrinsics.h>
#include <cvc/geometry/contours.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/types.h>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {

namespace {

namespace se = cvc::state_exec;

const std::string &path_arg(std::span<const se::value_t> args, std::size_t i, const char *fn) {
  const std::string *s = std::get_if<std::string>(&args[i].v);
  if (!s)
    throw se::dsl_error("TypeError",
                        std::string(fn) + ": argument " + std::to_string(i + 1) +
                            " is a file's path (a string), not " + param_detail::dsl_kind(args[i]),
                        "native");
  return *s;
}

// Argument i as a stack: a data object, or the data channel of the node a state path names.
std::shared_ptr<const contour_stack> stack_arg(cvc::app &app, se::intrinsics_context &ictx,
                                               std::span<const se::value_t> args, std::size_t i,
                                               const char *fn) {
  const job_call call(app, ictx, fn, std::vector<se::value_t>(args.begin(), args.end()),
                      se::value_t());
  return call.input_as<contour_stack>(i);
}

void expect_args(std::span<const se::value_t> args, std::size_t lo, std::size_t hi, const char *fn,
                 const char *usage) {
  if (args.size() < lo || args.size() > hi)
    throw se::dsl_error("TypeError", std::string(fn) + ": takes " + usage, "native");
}

se::value_t color_value(const std::vector<double> &c) {
  std::vector<se::value_t> out;
  for (double v : c)
    out.emplace_back(v);
  return se::make_list(std::move(out));
}

se::value_t info(const contour_stack &s) {
  std::vector<se::value_t> comps;
  for (const contour_component &c : s.components) {
    std::int64_t polys = 0;
    for (const auto &sl : s.slices) {
      auto it = sl.second.components.find(c.name);
      if (it != sl.second.components.end())
        polys += static_cast<std::int64_t>(it->second.size());
    }
    comps.push_back(se::make_dict({{"name", se::value_t(c.name)},
                                   {"color", color_value(c.color)},
                                   {"visible", se::value_t(c.visible)},
                                   {"polygons", se::value_t(polys)}}));
  }
  const bool empty = s.slices.empty();
  return se::make_dict(
      {{"first", empty ? se::value_t() : se::value_t(std::int64_t(s.slices.begin()->first))},
       {"last", empty ? se::value_t() : se::value_t(std::int64_t(s.slices.rbegin()->first))},
       {"slices", se::value_t(static_cast<std::int64_t>(s.slices.size()))},
       {"polygons", se::value_t(static_cast<std::int64_t>(s.polygon_count()))},
       {"vertices", se::value_t(static_cast<std::int64_t>(s.vertex_count()))},
       {"z_spacing", se::value_t(s.z_spacing)},
       {"z_origin", se::value_t(s.z_origin)},
       {"components", se::make_list(std::move(comps))}});
}

se::value_t string_list(const std::vector<std::string> &v) {
  std::vector<se::value_t> out;
  for (const std::string &s : v)
    out.emplace_back(s);
  return se::make_list(std::move(out));
}

} // namespace

se::value_t contour_stack_value(contour_stack s) {
  auto obj = std::make_shared<se::data_object>();
  obj->payload =
      std::shared_ptr<const contour_stack>(std::make_shared<contour_stack>(std::move(s)));
  obj->type_name = "cvc::contour_stack";
  return se::value_t(std::move(obj));
}

void register_contour_params(cvc::app &app) {
  register_params<contour_read_params>(app, "contours.read");
  register_params<contour_component>(app, "contours.component");
}

void register_contour_intrinsics(se::environment_ptr env, se::intrinsics_context &ictx,
                                 cvc::app &app) {
  register_contour_params(app);
  register_param_intrinsics(env, app);
  cvc::app *const a = &app;
  se::intrinsics_context *const c = &ictx;
  se::builtins::register_fn(
      env, "contours.read", [a](std::span<const se::value_t> args) -> se::value_t {
        expect_args(args, 1, 2, "contours.read", "a path and an optional dict of parameters");
        const std::string &path = path_arg(args, 0, "contours.read");
        const param_result<contour_read_params> p =
            params_from_value<contour_read_params>(args.size() > 1 ? args[1] : se::value_t());
        if (!p.ok())
          throw std::invalid_argument("contours.read: " + p.message());
        return contour_stack_value(read_contours(*a, path, p.value));
      });
  se::builtins::register_fn(
      env, "contours.write", [a, c](std::span<const se::value_t> args) -> se::value_t {
        expect_args(args, 2, 2, "contours.write", "a stack and a path");
        const std::string &path = path_arg(args, 1, "contours.write");
        write_contours(*a, *stack_arg(*a, *c, args, 0, "contours.write"), path);
        return se::value_t(path);
      });
  se::builtins::register_fn(env, "contours.info",
                            [a, c](std::span<const se::value_t> args) -> se::value_t {
                              expect_args(args, 1, 1, "contours.info", "a stack");
                              return info(*stack_arg(*a, *c, args, 0, "contours.info"));
                            });
  se::builtins::register_fn(
      env, "contours.formats", [a](std::span<const se::value_t> args) -> se::value_t {
        expect_args(args, 0, 0, "contours.formats", "no arguments");
        return se::make_dict({{"read", string_list(contour_read_extensions(*a))},
                              {"write", string_list(contour_write_extensions(*a))}});
      });
}

void install_contour_intrinsics(cvc::app &app) {
  register_contour_params(app);
  cvc::app *const a = &app;
  cvc::ariadne::register_action_intrinsics(
      app, "cvc.contours", [a](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
        register_contour_intrinsics(std::move(env), ictx, *a);
      });
}

} // namespace cvc
