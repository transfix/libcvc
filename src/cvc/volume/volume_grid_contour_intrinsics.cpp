/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// grid-contours.* in the DSL. See cvc/volume/volume_grid_contour_intrinsics.h.

#include <cvc/ariadne/ariadne.h>
#include <cvc/core/app.h>
#include <cvc/core/job_intrinsics.h>
#include <cvc/core/param_dsl.h>
#include <cvc/geometry/contour_intrinsics.h>
#include <cvc/geometry/contours.h>
#include <cvc/geometry/tiler_params.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_grid_contour_intrinsics.h>
#include <cvc/volume/volume_grid_contours.h>
#include <cvc/volume/volume_slice.h>
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

std::int64_t int_arg(std::span<const se::value_t> args, std::size_t i, const char *fn,
                     const char *what) {
  const std::int64_t *v = std::get_if<std::int64_t>(&args[i].v);
  if (!v)
    throw se::dsl_error("TypeError",
                        std::string(fn) + ": argument " + std::to_string(i + 1) + " is " + what +
                            " (an integer), not " + param_detail::dsl_kind(args[i]),
                        "native");
  return *v;
}

double number_of(const se::value_t &v, const char *fn, const char *what) {
  if (const std::int64_t *i = std::get_if<std::int64_t>(&v.v))
    return static_cast<double>(*i);
  if (const double *d = std::get_if<double>(&v.v))
    return *d;
  throw se::dsl_error("TypeError",
                      std::string(fn) + ": " + what + " holds " + param_detail::dsl_kind(v) +
                          ", not a number",
                      "native");
}

slice_axis axis_arg(std::span<const se::value_t> args, std::size_t i, const char *fn) {
  if (i >= args.size() || args[i].is_nil())
    return slice_axis::z;
  const std::string &s = string_arg(args, i, fn, "an axis (\"x\", \"y\" or \"z\")");
  slice_axis a = slice_axis::z;
  if (!parse_slice_axis(s, a))
    throw std::invalid_argument(std::string(fn) + ": axis '" + s + "' is not x, y or z");
  return a;
}

job_call call_of(cvc::app &app, se::intrinsics_context &ictx, std::span<const se::value_t> args,
                 const char *fn) {
  return job_call(app, ictx, fn, std::vector<se::value_t>(args.begin(), args.end()), se::value_t());
}

// Argument i as curves: nil (when `nil_ok`: none, on a unit frame), a data object, or the data
// channel of the node a state path names.
grid_contours curves_arg(cvc::app &app, se::intrinsics_context &ictx,
                         std::span<const se::value_t> args, std::size_t i, const char *fn,
                         bool nil_ok) {
  if (nil_ok && args[i].is_nil())
    return grid_contours();
  return *call_of(app, ictx, args, fn).input_as<grid_contours>(i);
}

se::value_t numbers(const std::vector<double> &v) {
  std::vector<se::value_t> out;
  out.reserve(v.size());
  for (double d : v)
    out.emplace_back(d);
  return se::make_list(std::move(out));
}

template <class A> se::value_t numbers3(const A &a) {
  return se::make_list({se::value_t(static_cast<double>(a[0])),
                        se::value_t(static_cast<double>(a[1])),
                        se::value_t(static_cast<double>(a[2]))});
}

se::value_t info(const grid_contours &c) {
  std::vector<se::value_t> objects;
  for (const grid_contour_object &o : c.objects) {
    std::size_t closed = 0, points = 0;
    for (const grid_curve &cv : o.curves) {
      closed += cv.closed ? 1 : 0;
      points += cv.size();
    }
    std::string interp;
    param_detail::enum_name(o.interpolation, interp);
    objects.push_back(
        se::make_dict({{"name", se::value_t(o.name)},
                       {"color", numbers(o.color)},
                       {"interpolation", se::value_t(interp)},
                       {"samples", se::value_t(std::int64_t(o.samples))},
                       {"visible", se::value_t(o.visible)},
                       {"curves", se::value_t(static_cast<std::int64_t>(o.curves.size()))},
                       {"closed", se::value_t(static_cast<std::int64_t>(closed))},
                       {"points", se::value_t(static_cast<std::int64_t>(points))}}));
  }
  return se::make_dict(
      {{"frame",
        se::make_dict({{"dims", se::make_list({se::value_t(std::int64_t(c.frame.dims[0])),
                                               se::value_t(std::int64_t(c.frame.dims[1])),
                                               se::value_t(std::int64_t(c.frame.dims[2]))})},
                       {"origin", numbers3(c.frame.origin)},
                       {"spacing", numbers3(c.frame.spacing)}})},
       {"curves", se::value_t(static_cast<std::int64_t>(c.curve_count()))},
       {"points", se::value_t(static_cast<std::int64_t>(c.point_count()))},
       {"objects", se::make_list(std::move(objects))}});
}

// A frame argument: curves (their frame) or a volume (its grid).
grid_frame frame_arg(cvc::app &app, se::intrinsics_context &ictx, std::span<const se::value_t> args,
                     std::size_t i, const char *fn) {
  const job_call call = call_of(app, ictx, args, fn);
  const boost::any a = call.input_any(i);
  if (const auto *c = boost::any_cast<std::shared_ptr<const grid_contours>>(&a); c && *c)
    return (*c)->frame;
  if (const auto *c = boost::any_cast<std::shared_ptr<grid_contours>>(&a); c && *c)
    return (*c)->frame;
  if (const auto *c = boost::any_cast<grid_contours>(&a))
    return c->frame;
  if (const auto *v = boost::any_cast<std::shared_ptr<const volume>>(&a); v && *v)
    return grid_frame_of(grid_of(**v));
  if (const auto *v = boost::any_cast<std::shared_ptr<volume>>(&a); v && *v)
    return grid_frame_of(grid_of(**v));
  if (const auto *v = boost::any_cast<volume>(&a))
    return grid_frame_of(grid_of(*v));
  throw std::invalid_argument(call.where(i) + " holds " + a.type().name() +
                              ", not volume grid contours or a volume (a frame)");
}

// A points argument: r u r u ... or (r u) pairs.
std::vector<double> points_arg(std::span<const se::value_t> args, std::size_t i, const char *fn) {
  const se::list_ptr *l = std::get_if<se::list_ptr>(&args[i].v);
  if (!l || !*l)
    throw se::dsl_error("TypeError",
                        std::string(fn) + ": argument " + std::to_string(i + 1) +
                            " is the points (a list), not " + param_detail::dsl_kind(args[i]),
                        "native");
  std::vector<double> out;
  for (const se::value_t &v : **l) {
    if (const se::list_ptr *pair = std::get_if<se::list_ptr>(&v.v)) {
      if (!*pair || (*pair)->size() != 2)
        throw std::invalid_argument(std::string(fn) + ": a point pair holds other than 2 values");
      out.push_back(number_of((**pair)[0], fn, "a point"));
      out.push_back(number_of((**pair)[1], fn, "a point"));
    } else {
      out.push_back(number_of(v, fn, "a point"));
    }
  }
  if (out.size() % 2)
    throw std::invalid_argument(std::string(fn) + ": an odd number of coordinates (" +
                                std::to_string(out.size()) + ")");
  return out;
}

} // namespace

se::value_t grid_contours_value(grid_contours c) {
  auto obj = std::make_shared<se::data_object>();
  obj->payload =
      std::shared_ptr<const grid_contours>(std::make_shared<grid_contours>(std::move(c)));
  obj->type_name = "cvc::grid_contours";
  return se::value_t(std::move(obj));
}

job_op grid_contours_tile_op(sliced_threading threading) {
  job_op op = make_job_op<tile_params, grid_tile_result>(
      "grid-contours.tile", 1,
      [](const job_call &c, const tile_params &p) {
        return tile_grid_contours_job(*c.input_as<grid_contours>(0), p);
      },
      [](grid_tile_result &&r) {
        std::vector<std::pair<std::string, se::value_t>> surfaces;
        for (auto &[name, g] : r.surfaces) {
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
        return path ? *path : std::string("volume grid contours");
      });
  op.threading = threading;
  return op;
}

void register_grid_contour_params(cvc::app &app) {
  register_params<grid_contour_object>(app, "grid-contours.object");
  register_params<volume_grid_contour_params>(app, "grid-contours.display");
}

void register_grid_contour_intrinsics(se::environment_ptr env, se::intrinsics_context &ictx,
                                      cvc::app &app, sliced_threading threading) {
  register_grid_contour_params(app);
  register_job_op(env, ictx, app, grid_contours_tile_op(threading));
  register_job_intrinsics(env, app);
  cvc::app *const a = &app;
  se::intrinsics_context *const c = &ictx;
  se::builtins::register_fn(env, "grid-contours.info",
                            [a, c](std::span<const se::value_t> args) -> se::value_t {
                              expect_args(args, 1, 1, "grid-contours.info", "volume grid contours");
                              return info(curves_arg(*a, *c, args, 0, "grid-contours.info", false));
                            });
  se::builtins::register_fn(
      env, "grid-contours.stack", [a, c](std::span<const se::value_t> args) -> se::value_t {
        expect_args(args, 1, 2, "grid-contours.stack",
                    "volume grid contours and optionally an axis");
        const grid_contours g = curves_arg(*a, *c, args, 0, "grid-contours.stack", false);
        return contour_stack_value(
            grid_contours_stack(g, axis_arg(args, 1, "grid-contours.stack")));
      });
  se::builtins::register_fn(
      env, "grid-contours.from-stack", [a, c](std::span<const se::value_t> args) -> se::value_t {
        expect_args(args, 2, 3, "grid-contours.from-stack",
                    "a contour stack, a frame (curves or a volume) and optionally an axis");
        const job_call call = call_of(*a, *c, args, "grid-contours.from-stack");
        const std::shared_ptr<const contour_stack> s = call.input_as<contour_stack>(0);
        const grid_frame f = frame_arg(*a, *c, args, 1, "grid-contours.from-stack");
        return grid_contours_value(
            grid_contours_from_stack(*s, f, axis_arg(args, 2, "grid-contours.from-stack")));
      });
  se::builtins::register_fn(
      env, "grid-contours.add-curve", [a, c](std::span<const se::value_t> args) -> se::value_t {
        const char *fn = "grid-contours.add-curve";
        expect_args(args, 5, 6, fn,
                    "curves (or nil), an object's name, an axis, a slice index, the points, and "
                    "optionally closed");
        grid_contours g = curves_arg(*a, *c, args, 0, fn, true);
        const std::string &name = string_arg(args, 1, fn, "an object's name");
        if (name.empty())
          throw std::invalid_argument(std::string(fn) + ": an object needs a name");
        grid_curve cv;
        cv.axis = axis_arg(args, 2, fn);
        cv.index = int_arg(args, 3, fn, "a slice index");
        cv.ru = points_arg(args, 4, fn);
        cv.closed = args.size() < 6 || args[5].is_truthy();
        long o = g.object_index(name);
        if (o < 0) {
          grid_contour_object obj;
          obj.name = name;
          obj.color = point_class_palette(g.objects.size());
          g.objects.push_back(std::move(obj));
          o = static_cast<long>(g.objects.size() - 1);
        }
        g.objects[static_cast<std::size_t>(o)].curves.push_back(std::move(cv));
        const std::vector<std::string> problems = grid_contours_problems(g);
        if (!problems.empty())
          throw std::invalid_argument(std::string(fn) + ": " + problems.front());
        return grid_contours_value(std::move(g));
      });
}

void install_grid_contour_intrinsics(cvc::app &app) {
  register_grid_contour_params(app);
  cvc::app *const a = &app;
  cvc::ariadne::register_action_intrinsics(
      app, "cvc.grid-contours",
      [a](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
        register_grid_contour_intrinsics(std::move(env), ictx, *a);
      });
}

} // namespace cvc
