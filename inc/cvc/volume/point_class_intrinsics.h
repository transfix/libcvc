/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::point_classes in the DSL (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.6, P0d). Small
// and synchronous, so plain natives:
//
//   (point-classes.add pc class i j k [variable timestep])
//   (point-classes.remove pc class i j k [variable timestep])
//                                        a new data object: `pc` with the voxel added to (or
//                                        removed from) the class; `pc` itself is never changed,
//                                        and nil starts from no classes. Store the result with
//                                        state-data-set
//   (point-classes.points pc class [variable timestep])
//                                        the class's points, a list of (i j k) lists
//   (point-classes.info pc)              a dict: points (the total) and classes, a list of dicts
//                                        of name, color, radius, variable, timestep and points
//   (params.defaults name), (params.check name dict)
//                                        with the tables "point-classes.read"
//                                        (read_point_classes' parameters, point_class_read_params)
//                                        and "point-classes.class" (point_class's metadata)
//
// No intrinsic reads or writes files (Joe, 2026-11-12): the DSL deals with point classes on the
// state tree. They reach the tree, and leave it for a file, through the rest of libcvc:
// read_point_classes / write_point_classes, pycvc, a node's request.
//
// A `pc` argument is a data object (from state-data-get, an edit, or point_classes_value below) or
// a state path naming a node whose data channel holds one, resolved under the calling program's
// root. A bad argument raises TypeError or ValueError.
//
// Hosts: register_point_class_intrinsics binds them in one environment; install_point_class_
// intrinsics adds them to the app's own action-intrinsic providers (key "cvc.point-classes"), so
// that app's Ariadne documents get them and no other app's do.

#ifndef CVC_VOLUME_POINT_CLASS_INTRINSICS_H
#define CVC_VOLUME_POINT_CLASS_INTRINSICS_H

#include <memory>

namespace cvc {
class app;
struct point_classes;
namespace state_exec {
struct environment;
struct intrinsics_context;
struct value_tag;
} // namespace state_exec

// The data object point classes travel in: payload std::shared_ptr<const point_classes>, type
// name "cvc::point_classes".
state_exec::value_tag point_classes_value(point_classes pc);

// Register "point-classes.read" and "point-classes.class" in the app's parameter tables.
void register_point_class_params(cvc::app &app);

// Bind point-classes.* and params.* in `env`. `ictx` resolves state paths; it and the app must
// outlive the environment's evaluations.
void register_point_class_intrinsics(std::shared_ptr<state_exec::environment> env,
                                     state_exec::intrinsics_context &ictx, cvc::app &app);

// Add them to the app's action-intrinsic providers under the key "cvc.point-classes" (a second
// call replaces the first), and register the tables.
void install_point_class_intrinsics(cvc::app &app);

} // namespace cvc

#endif // CVC_VOLUME_POINT_CLASS_INTRINSICS_H
