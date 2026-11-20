/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The Runtime's half of Kind::Settings (private to the Ariadne core): walk the subtrees a settings
// editor names, choose each key's editor from its metadata (its node's comment and properties:
// cvc/state/state_key_meta.h's access, unit, min and max, cvc/core/param_table.h's type, choices
// and default, and an optional scale), hand the backend a SettingsSpec, and write its edits and
// resets back. The spec is rebuilt only when something under a root was written (a value, a
// comment, a property) or the search changed, so an editor at rest reads no state.

#ifndef CVC_ARIADNE_SETTINGS_EMIT_H
#define CVC_ARIADNE_SETTINGS_EMIT_H

#include <cstdint>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/widget.h>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>

namespace cvc {
class app;
namespace ariadne {

// The subtrees the Runtime's settings editors draw, each with a count of the writes at, under or
// above it. note() runs on whatever thread wrote (the tree root's childChanged); the frame thread
// reads the counts. A root is watched from the first time its count is asked for.
struct SettingsWatch {
  std::mutex mu;
  std::unordered_map<std::string, std::uint64_t> roots; // absolute path -> writes seen
  void note(const std::string &path);
  std::uint64_t count(const std::string &root);
};

// One editor instance's cache (keyed by its id and roots).
struct SettingsCache {
  std::string built_from; // the roots' write counts and the search it was built for
  SettingsSpec spec;      // what the backend is handed
  // Each key's value when this editor first saw it: its default when its metadata gives none.
  std::unordered_map<std::string, std::string> first;
  std::uint64_t used = 0; // the frame that last drew it (sweep_settings_caches)
};

struct SettingsContext {
  cvc::app &app;
  Backend &backend;
  std::function<std::string(const std::string &)> resolve; // bind -> state path (this scope)
  std::function<void(const std::string &)> enqueue;        // fire the editor's `on:`
  std::function<void(const std::string &)> warn_once;      // a de-duplicated diagnostic
  SettingsWatch &watch;
  std::unordered_map<std::string, SettingsCache> &caches;
  std::uint64_t frame = 0;
};

// Emit one Kind::Settings widget.
void emit_settings(SettingsContext &ctx, const Widget &w);

// Forget the caches of editors no frame drew for `max_age` frames (a closed window, a repeat's
// rows gone).
void sweep_settings_caches(std::unordered_map<std::string, SettingsCache> &caches,
                           std::uint64_t frame, std::uint64_t max_age = 300);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_SETTINGS_EMIT_H
