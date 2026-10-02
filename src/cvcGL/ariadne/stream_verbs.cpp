/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/ariadne/ariadne.h>            // register_action_intrinsics
#include <cvc/core/state_exec/builtins.h>   // register_fn, environment
#include <cvc/core/state_exec/intrinsics.h> // intrinsics_context, document_scope
#include <cvc/core/state_exec/types.h>      // value_t, make_dict, dict_ptr
#include <cvc/gl/ariadne/stream_verbs.h>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace se = cvc::state_exec;

namespace cvc {
namespace gl {
namespace ariadne {

namespace {

// -------- small value_t helpers (local: the stream_intrinsics.cpp copies are in another TU's
// anonymous namespace and are not reachable here) --------
se::value_t err(const std::string &msg) {
  return se::make_dict({{"ok", se::value_t(false)}, {"error", se::value_t(msg)}});
}
std::string as_str(const se::value_t &v) {
  if (const std::string *s = std::get_if<std::string>(&v.v))
    return *s;
  return std::string();
}
// A stream handle is EITHER the bare token string OR the (stream-open) dict (read its "token").
std::string handle_token(const se::value_t &v) {
  if (const std::string *s = std::get_if<std::string>(&v.v))
    return *s;
  if (const se::dict_ptr *dp = std::get_if<se::dict_ptr>(&v.v))
    if (*dp)
      for (const auto &kv : **dp)
        if (kv.first == "token")
          return as_str(kv.second);
  return std::string();
}

} // namespace

void register_gl_stream_intrinsics() {
  // Register unconditionally (append), like cvc::ariadne::register_stream_intrinsics — NOT guarded
  // by a once_flag. The provider is STATELESS (captures only each lane's ictx.document), so a
  // duplicate from a second AriRuntime is harmless: register_fn overwrites the name in each env and
  // the behavior is identical. Appending (rather than once) is also what keeps this correct across
  // a clear_action_intrinsics() + rebuild cycle (a once_flag would refuse to re-add after a clear).
  // Fully qualified: we are in cvc::gl::ariadne, but the program-lane seam lives in cvc::ariadne.
  cvc::ariadne::register_action_intrinsics([](std::shared_ptr<se::environment> env,
                                              se::intrinsics_context &ictx) {
    // Capture only THIS lane's per-document scope (ictx.document) — never a host object. The
    // provider runs once per action/resident env build, each with that document's scope, so the
    // verb bound into each env routes to that document's own sink. null on the init: lane.
    se::document_scope *doc = ictx.document;
    se::builtins::register_fn(
        env, "gl-bind-stream", [doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("gl-bind-stream: no document scope (call it from an action or resident "
                       "lane, not the load-time init: lane)");
          // Resolve THIS document's GL sink, published by the cvcGL host into its document_scope.
          // get-or-create yields an empty handle (null sink) for a non-cvcGL document -> clean
          // error.
          GlStreamSinkHandle *h = doc->slot<GlStreamSinkHandle>(kGlStreamSinkSlot);
          if (!h || !h->sink)
            return err("gl-bind-stream: no GL scene sink for this document (not a cvcGL host, or "
                       "called before the scene was set up)");
          if (args.size() < 2)
            return err("gl-bind-stream: expects (gl-bind-stream NODE-ID TOKEN)");
          const std::string node = as_str(args[0]);
          const std::string token = handle_token(args[1]);
          if (node.empty())
            return err("gl-bind-stream: NODE-ID (a scene node id string) is required");
          if (token.empty())
            return err("gl-bind-stream: TOKEN (a stream handle dict or token string) is required");
          // All GL work (node lookup, subscribe, binding construction + ownership) happens inside
          // the sink, on this render thread. bind_stream returns "" on success or a diagnostic.
          const std::string e = h->sink->bind_stream(node, token);
          if (!e.empty())
            return err(e);
          return se::make_dict({{"ok", se::value_t(true)},
                                {"node", se::value_t(node)},
                                {"token", se::value_t(token)}});
        });
  });
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
