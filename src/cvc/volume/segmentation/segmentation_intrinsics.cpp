/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// segment.seeded and segment.critical-points (cvc/volume/segmentation_intrinsics.h):
// segment_seeded_job and critical_point_seeds_job behind the generic job forms of
// cvc/core/job_intrinsics.h.

#include <cstdint>
#include <cvc/ariadne/ariadne.h>
#include <cvc/geometry/geometry.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/volume/class_surfaces.h>
#include <cvc/volume/point_class_intrinsics.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/segmentation.h>
#include <cvc/volume/segmentation_intrinsics.h>
#include <cvc/volume/segmentation_params.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace cvc {

namespace se = state_exec;

namespace {

se::value_t volume_object(volume v) {
  auto obj = std::make_shared<se::data_object>();
  obj->payload = std::shared_ptr<const volume>(std::make_shared<volume>(std::move(v)));
  obj->type_name = "cvc::volume";
  return se::value_t(std::move(obj));
}

se::value_t to_value(seeded_segmentation &&r) {
  std::vector<se::value_t> classes, names, counts, warnings;
  for (volume &c : r.classes)
    classes.push_back(volume_object(std::move(c)));
  for (std::string &n : r.class_names)
    names.push_back(se::value_t(std::move(n)));
  for (std::size_t c : r.class_voxels)
    counts.push_back(se::value_t(std::int64_t(c)));
  for (std::string &w : r.warnings)
    warnings.push_back(se::value_t(std::move(w)));
  point_class border;
  border.name = "border";
  border.points.reserve(3 * r.border.size());
  for (const auto &b : r.border)
    border.points.insert(border.points.end(), {b[0], b[1], b[2]});
  point_classes pc;
  pc.classes.push_back(std::move(border));
  return se::make_dict({
      {"labels", volume_object(std::move(r.labels))},
      {"classes", se::make_list(std::move(classes))},
      {"border", point_classes_value(std::move(pc))},
      {"class_names", se::make_list(std::move(names))},
      {"class_voxels", se::make_list(std::move(counts))},
      {"automatic_seeds", se::value_t(std::int64_t(r.automatic_seeds))},
      {"classified_seeds", se::value_t(std::int64_t(r.classified_seeds))},
      {"t_low_normalized", se::value_t(r.t_low_normalized)},
      {"t_high_normalized", se::value_t(r.t_high_normalized)},
      {"warnings", se::make_list(std::move(warnings))},
  });
}

} // namespace

job_op segment_seeded_op(sliced_threading threading) {
  job_op op = make_job_op<seeded_segmentation_params, seeded_segmentation>(
      "segment.seeded", 2,
      [](const job_call &c, const seeded_segmentation_params &p) {
        return segment_seeded_job(*c.input_as<volume>(0), *c.input_as<point_classes>(1), p);
      },
      [](seeded_segmentation &&r) { return to_value(std::move(r)); },
      [](const job_call &c, const seeded_segmentation_params &) {
        const std::string *path = std::get_if<std::string>(&c.input(0).v);
        return path ? *path : std::string("a volume");
      });
  op.threading = threading;
  return op;
}

job_op critical_point_seeds_op(sliced_threading threading) {
  job_op op = make_job_op<seeded_segmentation_params, point_classes>(
      "segment.critical-points", 1,
      [](const job_call &c, const seeded_segmentation_params &p) {
        return critical_point_seeds_job(*c.input_as<volume>(0), p);
      },
      [](point_classes &&r) { return se::value_t(point_classes_value(std::move(r))); },
      [](const job_call &c, const seeded_segmentation_params &) {
        const std::string *path = std::get_if<std::string>(&c.input(0).v);
        return path ? *path : std::string("a volume");
      });
  op.threading = threading;
  return op;
}

job_op class_surfaces_op(sliced_threading threading) {
  job_op op = make_job_op<class_surface_params, class_surface_result>(
      "segment.surfaces", 1,
      [](const job_call &c, const class_surface_params &p) {
        return class_surfaces_job(*c.input_as<volume>(0), p);
      },
      [](class_surface_result &&r) {
        std::vector<se::value_t> labels, surfaces, voxels, warnings;
        for (int l : r.labels)
          labels.push_back(se::value_t(std::int64_t(l)));
        for (geometry &g : r.surfaces) {
          auto obj = std::make_shared<se::data_object>();
          obj->payload = std::move(g);
          obj->type_name = "cvc::geometry";
          surfaces.push_back(se::value_t(std::move(obj)));
        }
        for (std::size_t v : r.voxels)
          voxels.push_back(se::value_t(std::int64_t(v)));
        for (std::string &w : r.warnings)
          warnings.push_back(se::value_t(std::move(w)));
        return se::make_dict({
            {"labels", se::make_list(std::move(labels))},
            {"surfaces", se::make_list(std::move(surfaces))},
            {"voxels", se::make_list(std::move(voxels))},
            {"warnings", se::make_list(std::move(warnings))},
        });
      },
      [](const job_call &c, const class_surface_params &) {
        const std::string *path = std::get_if<std::string>(&c.input(0).v);
        return path ? *path : std::string("labels");
      });
  op.threading = threading;
  return op;
}

void register_segmentation_intrinsics(se::environment_ptr env, se::intrinsics_context &ictx,
                                      cvc::app &app) {
  register_job_op(env, ictx, app, segment_seeded_op());
  register_job_op(env, ictx, app, critical_point_seeds_op());
  register_job_op(env, ictx, app, class_surfaces_op());
  register_job_intrinsics(env, app);
}

void install_segmentation_intrinsics(cvc::app &app) {
  cvc::app *const a = &app;
  ariadne::register_action_intrinsics(
      app, "cvc.segment", [a](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
        register_segmentation_intrinsics(std::move(env), ictx, *a);
      });
}

} // namespace cvc
