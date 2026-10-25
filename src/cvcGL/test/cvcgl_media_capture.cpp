/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_media_capture — the (stream-capture-camera) media DSL verb (conference roadmap §4a),
// driven HARDWARE-FREE through the Ariadne Runtime with a synthetic source. It proves the Phase-1
// path end to end: a .ari program on the resident lane opens a video stream with the core
// (stream-open) verb and attaches a producer with (stream-capture-camera synthetic:#t), and frames
// then flow through the stream's channel to a subscriber — all with no camera and no GL, so it runs
// on CI. The SDL device path is compiled in but not exercised here.

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>       // Runtime
#include <cvc/ariadne/ftxui_backend.h> // a headless Backend (drain() never walks it)
#include <cvc/ariadne/stream/stream.h> // registry_key
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/ariadne/stream/stream_registry.h>
#include <cvc/ariadne/stream_intrinsics.h> // register_stream_intrinsics
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/media_verbs.h> // register_media_intrinsics (this PR)
#include <cvc/state/state.h>
#include <memory>
#include <string>
#include <thread>

namespace ari = cvc::ariadne;
namespace sx = cvc::ariadne::stream;

int main() {
  cvc::app app;
  ari::register_stream_intrinsics(app);          // (stream-open/info/close) — core
  cvc::gl::ariadne::register_media_intrinsics(); // (stream-capture-camera) — the verb under test

  ari::Runtime rt(app, "");  // empty root scope -> canonical token is "streams.<id>"
  ari::FtxuiBackend backend; // drain() does not touch the backend, but a real host always sets one
  rt.set_backend(&backend);

  // A one-shot (state-guarded) resident: open a tiny video stream and attach a SYNTHETIC producer.
  // The stream verbs require the action/resident lane (they need ictx.document); a resident runs
  // the body on drain(). (stream-open) is nested inside (get-attr … "token"), so no local-binding
  // form is needed — its handle dict flows straight into (stream-capture-camera).
  rt.set_tick_program(
      "(if (= (state-get \"cap.done\") \"1\") #f"
      "  (begin"
      "    (stream-capture-camera"
      "      (get-attr (stream-open (dict \"id\" \"cam\" \"w\" 64 \"h\" 48)) \"token\")"
      "      (dict \"synthetic\" #t \"w\" 64 \"h\" 48 \"hz\" 240))"
      "    (state-set \"cap.done\" \"1\")))");
  rt.drain(); // runs the body once: stream opened + a synthetic producer thread started

  for (const std::string &w : rt.take_reactive_warnings())
    std::printf("warn: %s\n", w.c_str());
  assert(rt.take_reactive_warnings().empty() && "the capture program evaluated with no warnings");
  assert(cvc::state::instance(app)("cap.done").value() == "1" && "the capture program ran");

  // The stream is open + registered under the canonical token for the empty-root scope.
  const std::string token = sx::stream::registry_key("", "cam");
  assert(token == "streams.cam");
  sx::stream_channel *ch = sx::stream_registry::for_app(app).lookup(token);
  assert(ch && "(stream-open) opened and registered the channel");

  // Subscribe and wait briefly for the synthetic producer thread to publish a frame. The producer
  // runs on its own thread (start_producer), independent of drain(), so we poll rather than
  // re-drain.
  std::shared_ptr<sx::subscription> sub = ch->subscribe(sx::deliver_mode::latest);
  assert(sub && "subscribe succeeded (the pool had headroom)");
  bool got = false;
  for (int i = 0; i < 400 && !got; ++i) {
    if (sub->latest())
      got = true;
    else
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  assert(got && "the synthetic producer attached by the verb published at least one frame");
  const long long published = static_cast<long long>(ch->total_published());
  assert(published > 0 && "the channel saw published frames");

  // Teardown is RAII: dropping the subscription, then the Runtime, clears the document scope, which
  // destroys the owned stream — joining the producer thread before the channel/pool go. A crash
  // here would fail the test, so this also exercises clean producer teardown.
  sub.reset();

  // The JPEG path's quality option (the conference sends q60): the same synthetic frames encode
  // smaller at a low quality than at a high one, and a quality outside 1-100 is refused.
  rt.set_tick_program(
      "(if (= (state-get \"q.done\") \"1\") #f"
      "  (begin"
      "    (stream-capture-camera"
      "      (get-attr (stream-open (dict \"id\" \"lo\" \"codec\" \"mjpeg\" \"w\" 160 \"h\" 120))"
      "                \"token\")"
      "      (dict \"synthetic\" #t \"w\" 160 \"h\" 120 \"hz\" 240 \"jpeg\" #t \"quality\" 20))"
      "    (stream-capture-camera"
      "      (get-attr (stream-open (dict \"id\" \"hi\" \"codec\" \"mjpeg\" \"w\" 160 \"h\" 120))"
      "                \"token\")"
      "      (dict \"synthetic\" #t \"w\" 160 \"h\" 120 \"hz\" 240 \"jpeg\" #t \"quality\" 95))"
      "    (state-set \"q.bad\""
      "      (if (get-attr (stream-capture-camera \"streams.lo\""
      "                      (dict \"synthetic\" #t \"w\" 160 \"h\" 120 \"jpeg\" #t \"quality\" 0))"
      "                    \"ok\") \"accepted\" \"refused\"))"
      "    (state-set \"q.done\" \"1\")))");
  rt.drain();
  assert(cvc::state::instance(app)("q.done").value() == "1" && "the quality program ran");
  assert(cvc::state::instance(app)("q.bad").value() == "refused" && "quality 0 is refused");
  const auto frame_size = [&](const char *id) -> std::size_t {
    sx::stream_channel *c =
        sx::stream_registry::for_app(app).lookup(sx::stream::registry_key("", id));
    assert(c && "the quality stream is open");
    std::shared_ptr<sx::subscription> s = c->subscribe(sx::deliver_mode::latest);
    for (int i = 0; i < 400; ++i) {
      if (sx::frame_ptr f = s->latest())
        return f->size;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return 0;
  };
  const std::size_t lo = frame_size("lo"), hi = frame_size("hi");
  std::printf("jpeg 160x120: q20 %zu bytes, q95 %zu bytes\n", lo, hi);
  assert(lo > 0 && hi > 0 && "both encoders published");
  assert(lo < hi && "a lower quality encodes smaller frames");

  std::printf("cvcgl_media_capture: OK (synthetic capture via DSL verb -> %lld frames)\n",
              published);
  return 0;
}
