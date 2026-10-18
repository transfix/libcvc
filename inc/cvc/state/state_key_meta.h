/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// What a bound key is, for other applications: its comment and its properties
// (cvc::state::property). A binding (physics' world_binding and nav_plant, nav's sim_world)
// describes each key it reads or writes with a comment and an "access" property:
//   build      read when the object is built; a later change has no effect until a rebuild
//   knob       read by every pull(), applied when it changes
//   command    an input the object acts on when it changes (a drive command, pedals)
//   telemetry  written by publish() or push()
// Numeric keys get a "unit" (SI; "1" for a ratio, "" for a count or a flag), and knobs and
// commands a recommended "min" and "max": what a UI's slider or a validator should offer. They
// are recommendations, not limits the object enforces (it validates on its own).

#ifndef __CVC_STATE_KEY_META_H__
#define __CVC_STATE_KEY_META_H__

#include <cstddef>
#include <string>

namespace cvc {

class app;

struct state_key_meta {
  const char *key;    // relative to the group's base
  const char *access; // build | knob | command | telemetry
  const char *unit;   // nullptr: not a number
  double min, max;    // NaN: no recommendation
  const char *comment;
};

// One key, at `path` (an absolute state path): its comment, access, unit and recommended range.
// A read-only or unreachable node keeps its metadata.
void describe_state_key(cvc::app &ctx, const std::string &path, const state_key_meta &m);

// Each key of a table under `base` ("<base>.<key>"; just the key when base is empty). With
// `once`, a node that already has a comment is left alone, so a binding can call this every
// pull without rewriting its properties.
void describe_state_keys(cvc::app &ctx, const std::string &base, const state_key_meta *table,
                         std::size_t n, bool once = false);
template <std::size_t K>
void describe_state_keys(cvc::app &ctx, const std::string &base, const state_key_meta (&table)[K],
                         bool once = false) {
  describe_state_keys(ctx, base, table, K, once);
}

} // namespace cvc

#endif // __CVC_STATE_KEY_META_H__
