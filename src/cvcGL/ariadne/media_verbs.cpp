/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cstdint>
#include <cvc/ariadne/ariadne.h> // register_action_intrinsics
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/synthetic_source.h>
#include <cvc/ariadne/stream_intrinsics.h> // cvc::ariadne::document_stream (the doc stream table)
#include <cvc/gl/ariadne/media_verbs.h>
#include <cvc/state/state_exec/builtins.h>   // register_fn, environment
#include <cvc/state/state_exec/intrinsics.h> // intrinsics_context, document_scope
#include <cvc/state/state_exec/types.h>      // value_t, make_dict, dict_ptr
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#ifdef CVC_ENABLE_SDL
#include <cvc/gl/capture/camera_source.h> // open_camera (SDL3; the device path)
#endif

namespace se = cvc::state_exec;
namespace sx = cvc::ariadne::stream;

namespace cvc {
namespace gl {
namespace ariadne {

namespace {

// -------- small value_t helpers (local; the stream_intrinsics.cpp copies are in another TU's
// anonymous namespace and are not reachable here, exactly as stream_verbs.cpp notes) --------
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
// Look up key `k` in a value_t that is a dict; null if `v` is not a dict or has no such key.
const se::value_t *dict_get(const se::value_t &v, const std::string &k) {
  if (const se::dict_ptr *dp = std::get_if<se::dict_ptr>(&v.v))
    if (*dp)
      for (const auto &kv : **dp)
        if (kv.first == k)
          return &kv.second;
  return nullptr;
}
long as_int(const se::value_t *v, long def) {
  if (v) {
    if (const std::int64_t *i = std::get_if<std::int64_t>(&v->v))
      return static_cast<long>(*i);
    if (const double *d = std::get_if<double>(&v->v))
      return static_cast<long>(*d);
  }
  return def;
}
double as_num(const se::value_t *v, double def) {
  if (v) {
    if (const double *d = std::get_if<double>(&v->v))
      return *d;
    if (const std::int64_t *i = std::get_if<std::int64_t>(&v->v))
      return static_cast<double>(*i);
  }
  return def;
}
bool as_bool(const se::value_t *v, bool def) {
  if (v)
    if (const bool *b = std::get_if<bool>(&v->v))
      return *b;
  return def;
}

} // namespace

void register_media_intrinsics() {
  // Append a STATELESS provider (see the header / stream_verbs.cpp for why append, not once): it
  // captures only each lane's ictx.document and routes to that document's own stream table.
  cvc::ariadne::register_action_intrinsics([](std::shared_ptr<se::environment> env,
                                              se::intrinsics_context &ictx) {
    se::document_scope *doc = ictx.document; // per-document owner; null on the init: lane
    se::builtins::register_fn(
        env, "stream-capture-camera", [doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("stream-capture-camera: no document scope (call from an action or resident "
                       "lane, not the load-time init: lane)");
          if (args.empty())
            return err("stream-capture-camera: expects (stream-capture-camera TOKEN [OPTS])");
          const std::string token = handle_token(args[0]);
          if (token.empty())
            return err("stream-capture-camera: TOKEN (a stream handle dict or token string) is "
                       "required");
          sx::stream *s = cvc::ariadne::document_stream(doc, token);
          if (!s)
            return err("stream-capture-camera: no open stream named '" + token +
                       "' in this document — call (stream-open) first");

          // OPTS (all optional): device index (<0 or absent -> synthetic), synthetic frame size
          // w/h, cadence hz, and an explicit synthetic:#t override (used by headless tests).
          long device = -1, w = 640, h = 480;
          double hz = 30.0;
          bool synthetic = false;
          if (args.size() >= 2) {
            device = as_int(dict_get(args[1], "device"), -1);
            w = as_int(dict_get(args[1], "w"), 640);
            h = as_int(dict_get(args[1], "h"), 480);
            hz = as_num(dict_get(args[1], "hz"), 30.0);
            synthetic = as_bool(dict_get(args[1], "synthetic"), false);
          }
          if (w <= 0 || h <= 0)
            return err("stream-capture-camera: 'w' and 'h' must be positive");
          double used_hz = hz > 0.0 ? hz : 30.0;

          std::unique_ptr<sx::frame_source> source;
          bool is_synth = true;
          if (device >= 0 && !synthetic) {
#ifdef CVC_ENABLE_SDL
            // Device path: open the camera and adopt its frame_source. open_camera() and the
            // resulting stream's teardown touch the SDL camera subsystem on this (owner/action)
            // thread — the same thread stream::close() runs on, so the contract holds.
            cvc::gl::capture::camera_open cam =
                cvc::gl::capture::open_camera(static_cast<int>(device));
            if (!cam.source)
              return err("stream-capture-camera: could not open camera device " +
                         std::to_string(device) + " (no such device, or permission not granted)");
            source = std::move(cam.source);
            if (hz <= 0.0 && cam.fps > 0.0)
              used_hz = cam.fps;
            is_synth = false;
#else
            return err("stream-capture-camera: device capture needs CVC_ENABLE_SDL in this build — "
                       "pass synthetic:#t for a hardware-free source");
#endif
          } else {
            // Synthetic path (hardware-free, the headless/CI source): a moving rgba8 test pattern
            // sized w x h. It MUST match the stream's open w/h, or start_producer rejects it below.
            source = std::make_unique<sx::synthetic_source>(static_cast<int>(w),
                                                            static_cast<int>(h), used_hz);
          }

          // Attach + start the producer thread on this (owner) thread. start_producer replaces any
          // existing producer (old stopped+joined first) and is a no-op if the source is unsized or
          // its frame_bytes exceed the stream's slab (a w/h mismatch with the open) — a bad source
          // then simply produces nothing rather than corrupting the pool.
          s->start_producer(std::move(source), used_hz);
          return se::make_dict({{"ok", se::value_t(true)},
                                {"token", se::value_t(token)},
                                {"synthetic", se::value_t(is_synth)},
                                {"hz", se::value_t(used_hz)}});
        });
  });
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
