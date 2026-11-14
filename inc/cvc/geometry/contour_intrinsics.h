/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::contour_stack in the DSL (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.6, P0d). Small
// and synchronous, so plain natives, not jobs:
//
//   (contours.info stack)           a dict: first, last (the z indices; nil when empty), slices,
//                                   polygons, vertices, z_spacing, z_origin, and components, a
//                                   list of dicts of name, color, visible and polygons
//   (params.defaults name), (params.check name dict)
//                                   with the tables "contours.read" (read_contours' parameters,
//                                   contour_read_params) and "contours.component"
//                                   (contour_component)
//
// No intrinsic reads or writes files (Joe, 2026-11-12): the DSL deals with stacks on the state
// tree. A stack reaches the tree, and leaves it for a file, through the rest of libcvc:
// read_contours / write_contours, pycvc, a node's request, the CLI.
//
// A stack argument is a data object (from state-data-get, or contour_stack_value below) or a state
// path naming a node whose data channel holds one, resolved under the calling program's root. A bad
// argument raises TypeError or ValueError.
//
// Hosts. register_contour_intrinsics binds them in one environment (cvc exec, a pycvc Exec, a
// test). install_contour_intrinsics adds them to the app's own action-intrinsic providers
// (cvc/ariadne/ariadne.h, keyed "cvc.contours"), so every program lane of that app's Ariadne
// documents gets them, and no other app's do.

#ifndef CVC_GEOMETRY_CONTOUR_INTRINSICS_H
#define CVC_GEOMETRY_CONTOUR_INTRINSICS_H

#include <memory>

namespace cvc {
class app;
struct contour_stack;
namespace state_exec {
struct environment;
struct intrinsics_context;
struct value_tag;
} // namespace state_exec

// The data object a stack travels in: payload std::shared_ptr<const contour_stack>, type name
// "cvc::contour_stack".
state_exec::value_tag contour_stack_value(contour_stack s);

// Register "contours.read" and "contours.component" in the app's parameter tables
// (cvc/core/param_dsl.h).
void register_contour_params(cvc::app &app);

// Bind contours.* and params.* in `env`. `ictx` resolves state paths (its pinned root); it and the
// app must outlive the environment's evaluations.
void register_contour_intrinsics(std::shared_ptr<state_exec::environment> env,
                                 state_exec::intrinsics_context &ictx, cvc::app &app);

// Add them to the app's action-intrinsic providers, under the key "cvc.contours" (a second call
// replaces the first), and register the tables.
void install_contour_intrinsics(cvc::app &app);

} // namespace cvc

#endif // CVC_GEOMETRY_CONTOUR_INTRINSICS_H
