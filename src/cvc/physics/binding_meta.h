/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// What world_binding's keys are, for other applications: each key's comment and its properties
// (cvc::state::property). Every key gets a comment and an "access" property:
//   build      read when the world is built; a later change sets restart_required
//   knob       read by every pull(), applied when it changes
//   command    an input the world acts on when it changes (a drive command, pedals)
//   telemetry  written by push()
// Numeric keys get a "unit" (SI; "1" for a ratio, "" for a count or a flag), and knobs and
// commands a recommended "min" and "max": what a UI's slider or a validator should offer. They
// are recommendations, not limits the world enforces (it validates on its own).

#ifndef __CVC_PHYSICS_BINDING_META_H__
#define __CVC_PHYSICS_BINDING_META_H__

#include <string>

namespace cvc {

class app;

namespace physics {
namespace detail {

struct key_meta {
  const char *key;    // relative to the group's base
  const char *access; // build | knob | command | telemetry
  const char *unit;   // nullptr: not a number
  double min, max;    // NaN: no recommendation
  const char *comment;
};

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
