/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// point-classes.* in the DSL. See cvc/volume/point_class_intrinsics.h.

#include <cvc/ariadne/ariadne.h>
#include <cvc/core/app.h>
#include <cvc/core/job_intrinsics.h>
#include <cvc/core/param_dsl.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/volume/point_class_intrinsics.h>
#include <cvc/volume/point_classes.h>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {

namespace {

namespace se = cvc::state_exec;

void expect_args(std::span<const se::value_t> args, std::size_t lo, std::size_t hi, const char *fn,
                 const char *usage) {
  if (args.size() < lo || args.size() > hi)
    throw se::dsl_error("TypeError", std::string(fn) + ": takes " + usage, "native");
}

const std::string &string_arg(std::span<const se::value_t> args, std::size_t i, const char *fn,
                              const char *what) {
  const std::string *s = std::get_if<std::string>(&args[i].v);
  if (!s)
    throw se::dsl_error("TypeError",
                        std::string(fn) + ": argument " + std::to_string(i + 1) + " is " + what +
                            " (a string), not " + param_detail::dsl_kind(args[i]),
                        "native");
  return *s;
}

int int_arg(std::span<const se::value_t> args, std::size_t i, const char *fn, const char *what) {
  const std::int64_t *v = std::get_if<std::int64_t>(&args[i].v);
  if (!v)
    throw se::dsl_error("TypeError",
                        std::string(fn) + ": argument " + std::to_string(i + 1) + " is " + what +
                            " (an integer), not " + param_detail::dsl_kind(args[i]),
                        "native");
  if (*v < std::numeric_limits<int>::min() || *v > std::numeric_limits<int>::max())
    throw std::invalid_argument(std::string(fn) + ": argument " + std::to_string(i + 1) + " (" +
                                std::to_string(*v) + ") is out of range");
  return static_cast<int>(*v);
}

// Argument i as point classes: nil (when `nil_ok`: none), a data object, or the data channel of
// the node a state path names.
point_classes classes_arg(cvc::app &app, se::intrinsics_context &ictx,
                          std::span<const se::value_t> args, std::size_t i, const char *fn,
                          bool nil_ok) {
  if (nil_ok && args[i].is_nil())
    return point_classes();
  const job_call call(app, ictx, fn, std::vector<se::value_t>(args.begin(), args.end()),
                      se::value_t());
  return *call.input_as<point_classes>(i);
}

se::value_t number_list(const std::vector<double> &v) {
  std::vector<se::value_t> out;
  for (double d : v)
    out.emplace_back(d);
  return se::make_list(std::move(out));
}

se::value_t string_list(const std::vector<std::string> &v) {
  std::vector<se::value_t> out;
  for (const std::string &s : v)
    out.emplace_back(s);
  return se::make_list(std::move(out));
}

// (fn pc class i j k [variable timestep]): the class and the voxel, checked.
struct edit_args {
  point_classes pc;
  std::string cls;
  int i = 0, j = 0, k = 0, variable = 0, timestep = 0;
};

edit_args read_edit(cvc::app &app, se::intrinsics_context &ictx, std::span<const se::value_t> args,
                    const char *fn) {
  if (args.size() != 5 && args.size() != 7)
    throw se::dsl_error("TypeError",
                        std::string(fn) +
                            ": takes point classes (or nil), a class, i, j, k, and optionally a "
                            "variable and a timestep",
                        "native");
  edit_args e;
  e.pc = classes_arg(app, ictx, args, 0, fn, true);
  e.cls = string_arg(args, 1, fn, "a class's name");
  e.i = int_arg(args, 2, fn, "i");
  e.j = int_arg(args, 3, fn, "j");
  e.k = int_arg(args, 4, fn, "k");
  if (args.size() == 7) {
    e.variable = int_arg(args, 5, fn, "a variable");
    e.timestep = int_arg(args, 6, fn, "a timestep");
  }
  return e;
}

se::value_t info(const point_classes &pc) {
  std::vector<se::value_t> classes;
  for (const point_class &c : pc.classes)
    classes.push_back(se::make_dict({{"name", se::value_t(c.name)},
                                     {"color", number_list(c.color)},
                                     {"radius", se::value_t(c.radius)},
                                     {"variable", se::value_t(std::int64_t(c.variable))},
                                     {"timestep", se::value_t(std::int64_t(c.timestep))},
                                     {"points", se::value_t(std::int64_t(c.size()))}}));
  return se::make_dict({{"points", se::value_t(static_cast<std::int64_t>(pc.point_count()))},
                        {"classes", se::make_list(std::move(classes))}});
}

} // namespace

se::value_t point_classes_value(point_classes pc) {
  auto obj = std::make_shared<se::data_object>();
  obj->payload =
      std::shared_ptr<const point_classes>(std::make_shared<point_classes>(std::move(pc)));
  obj->type_name = "cvc::point_classes";
  return se::value_t(std::move(obj));
}

void register_point_class_params(cvc::app &app) {
  register_params<point_class_read_params>(app, "point-classes.read");
  register_params<point_class>(app, "point-classes.class");
}

void register_point_class_intrinsics(se::environment_ptr env, se::intrinsics_context &ictx,
                                     cvc::app &app) {
  register_point_class_params(app);
  register_param_intrinsics(env, app);
  cvc::app *const a = &app;
  se::intrinsics_context *const c = &ictx;
  se::builtins::register_fn(
      env, "point-classes.read", [a](std::span<const se::value_t> args) -> se::value_t {
        expect_args(args, 1, 2, "point-classes.read", "a path and an optional dict of parameters");
        const std::string &path = string_arg(args, 0, "point-classes.read", "a file's path");
        const param_result<point_class_read_params> p =
            params_from_value<point_class_read_params>(args.size() > 1 ? args[1] : se::value_t());
        if (!p.ok())
          throw std::invalid_argument("point-classes.read: " + p.message());
        return point_classes_value(read_point_classes(*a, path, p.value));
      });
  se::builtins::register_fn(
      env, "point-classes.write", [a, c](std::span<const se::value_t> args) -> se::value_t {
        expect_args(args, 2, 2, "point-classes.write", "point classes and a path");
        const std::string &path = string_arg(args, 1, "point-classes.write", "a file's path");
        write_point_classes(*a, classes_arg(*a, *c, args, 0, "point-classes.write", false), path);
        return se::value_t(path);
      });
  se::builtins::register_fn(
      env, "point-classes.add", [a, c](std::span<const se::value_t> args) -> se::value_t {
        edit_args e = read_edit(*a, *c, args, "point-classes.add");
        try {
          e.pc.add(e.cls, e.i, e.j, e.k, e.variable, e.timestep);
        } catch (const std::invalid_argument &x) {
          throw std::invalid_argument(std::string("point-classes.add: ") + x.what());
        }
        return point_classes_value(std::move(e.pc));
      });
  se::builtins::register_fn(env, "point-classes.remove",
                            [a, c](std::span<const se::value_t> args) -> se::value_t {
                              edit_args e = read_edit(*a, *c, args, "point-classes.remove");
                              e.pc.remove(e.cls, e.i, e.j, e.k, e.variable, e.timestep);
                              return point_classes_value(std::move(e.pc));
                            });
  se::builtins::register_fn(
      env, "point-classes.points", [a, c](std::span<const se::value_t> args) -> se::value_t {
        if (args.size() != 2 && args.size() != 4)
          throw se::dsl_error("TypeError",
                              "point-classes.points: takes point classes, a class, and "
                              "optionally a variable and a timestep",
                              "native");
        const point_classes pc = classes_arg(*a, *c, args, 0, "point-classes.points", false);
        const std::string &cls = string_arg(args, 1, "point-classes.points", "a class's name");
        const int var =
            args.size() == 4 ? int_arg(args, 2, "point-classes.points", "a variable") : 0;
        const int ts =
            args.size() == 4 ? int_arg(args, 3, "point-classes.points", "a timestep") : 0;
        const int k = pc.class_index(cls, var, ts);
        if (k < 0)
          throw std::invalid_argument("point-classes.points: no class '" + cls + "' of variable " +
                                      std::to_string(var) + ", timestep " + std::to_string(ts));
        const point_class &pcl = pc.classes[static_cast<std::size_t>(k)];
        std::vector<se::value_t> out;
        out.reserve(pcl.size());
        for (std::size_t n = 0; n < pcl.size(); ++n)
          out.push_back(se::make_list({se::value_t(std::int64_t(pcl.points[3 * n])),
                                       se::value_t(std::int64_t(pcl.points[3 * n + 1])),
                                       se::value_t(std::int64_t(pcl.points[3 * n + 2]))}));
        return se::make_list(std::move(out));
      });
  se::builtins::register_fn(
      env, "point-classes.info", [a, c](std::span<const se::value_t> args) -> se::value_t {
        expect_args(args, 1, 1, "point-classes.info", "point classes");
        return info(classes_arg(*a, *c, args, 0, "point-classes.info", false));
      });
  se::builtins::register_fn(
      env, "point-classes.formats", [a](std::span<const se::value_t> args) -> se::value_t {
        expect_args(args, 0, 0, "point-classes.formats", "no arguments");
        return se::make_dict({{"read", string_list(point_class_read_extensions(*a))},
                              {"write", string_list(point_class_write_extensions(*a))}});
      });
}

void install_point_class_intrinsics(cvc::app &app) {
  register_point_class_params(app);
  cvc::app *const a = &app;
  cvc::ariadne::register_action_intrinsics(
      app, "cvc.point-classes",
      [a](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
        register_point_class_intrinsics(std::move(env), ictx, *a);
      });
}

} // namespace cvc
