/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// What world_binding's keys are, for other applications, in cvc/state/state_key_meta.h's form:
// each key's comment and its properties access, unit and recommended min and max. A build key
// changed after the world is built sets restart_required.

#ifndef __CVC_PHYSICS_BINDING_META_H__
#define __CVC_PHYSICS_BINDING_META_H__

#include <cvc/state/state_key_meta.h>
#include <string>

namespace cvc {

class app;

namespace physics {
namespace detail {

using key_meta = cvc::state_key_meta;

// Describe each key of a group under `base` (an absolute state path). Idempotent.
void describe_world(cvc::app &ctx, const std::string &base);    // the prefix's own keys
void describe_material(cvc::app &ctx, const std::string &base); // materials.<name>
void describe_body(cvc::app &ctx, const std::string &base);     // bodies.<name>
// vehicles.<name>, with its wheels and the tuning of its axles
void describe_vehicle(cvc::app &ctx, const std::string &base, int wheels, int axles);
void describe_field(cvc::app &ctx, const std::string &base); // fields.<name>

// One key: its comment, access, unit and recommended range.
void describe(cvc::app &ctx, const std::string &path, const key_meta &m);

} // namespace detail
} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_BINDING_META_H__
