/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_media_audio — the AUDIO half of the Phase-1 media verbs (conference roadmap §4a): the core
// (stream-open) extended for audio streams (codec "f32") and the cvcGL (stream-capture-audio) verb
// with its composable hardware-free graph (tone / gain / mix). Driven through the Ariadne Runtime:
// a .ari opens an f32 audio stream and attaches a MIX of two tones, and the resulting PCM chunks
// flow to a ring subscriber — all with no SDL device and no GL, so it runs on CI.

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>       // Runtime
#include <cvc/ariadne/ftxui_backend.h> // a headless Backend
#include <cvc/ariadne/stream/frame.h>  // frame_kind (assert the chunks are audio_pcm)
#include <cvc/ariadne/stream/stream.h> // registry_key
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/ariadne/stream/stream_registry.h>
#include <cvc/ariadne/stream_intrinsics.h> // register_stream_intrinsics
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/media_verbs.h> // register_media_intrinsics
#include <cvc/state/state.h>
#include <memory>
#include <string>
#include <thread>

namespace ari = cvc::ariadne;
namespace sx = cvc::ariadne::stream;

int main() {
  cvc::app app;
  ari::register_stream_intrinsics(app); // (stream-open/info/close) — core, now audio-aware
  cvc::gl::ariadne::register_media_intrinsics(); // (stream-capture-audio) — the verb under test

  ari::Runtime rt(app, "");
  ari::FtxuiBackend backend;
  rt.set_backend(&backend);

  // One drain pumps the scheduler for a bounded slice (0.1 s, its second half on this thread's CPU
  // clock), and a tick program that needs more goes on in the next frame's drain. In a Debug,
  // coverage-instrumented build on a busy runner the programs below can need more than one slice
  // (package-linux debug lanes failed "the deep-graph program ran" on all 3 attempts of a run), so
  // drain frames until the program has set its key, for 10 s at most.
  const auto drain_until_set = [&](const char *key) {
    cvc::state &root = cvc::state::instance(app);
    const auto t0 = std::chrono::steady_clock::now();
    int frames = 0;
    do {
      rt.drain();
      ++frames;
    } while (root(key).value() != "1" &&
             std::chrono::steady_clock::now() - t0 < std::chrono::seconds(10));
    if (frames > 1)
      std::printf("%s after %d drains\n", key, frames);
  };

  // Open an AUDIO stream (codec "f32", 48 kHz mono, 256-frame chunks) and attach a hardware-free
  // graph: a MIX of a 440 Hz and a 660 Hz tone. The capture SPEC's rate/ch/frames match the open.
  // 256-frame chunks at 48 kHz -> ~187 chunks/s, so frames arrive within a few ms.
  rt.set_tick_program("(if (= (state-get \"cap.done\") \"1\") #f"
                      "  (begin"
                      "    (stream-capture-audio"
                      "      (get-attr (stream-open (dict \"id\" \"mic\" \"codec\" \"f32\""
                      "                                   \"rate\" 48000 \"ch\" 1 "
                      "\"frames_per_chunk\" 256)) \"token\")"
                      "      (dict \"rate\" 48000 \"ch\" 1 \"frames\" 256"
                      "            \"mix\" (list (dict \"tone\" (dict \"hz\" 440))"
                      "                          (dict \"tone\" (dict \"hz\" 660)))))"
                      "    (state-set \"cap.done\" \"1\")))");
  drain_until_set("cap.done"); // runs once: audio stream opened + a mix producer thread started

  for (const std::string &w : rt.take_reactive_warnings())
    std::printf("warn: %s\n", w.c_str());
  assert(rt.take_reactive_warnings().empty() &&
         "the audio-capture program evaluated with no warnings");
  assert(cvc::state::instance(app)("cap.done").value() == "1" && "the audio-capture program ran");

  const std::string token = sx::stream::registry_key("", "mic");
  assert(token == "streams.mic");
  sx::stream_channel *ch = sx::stream_registry::for_app(app).lookup(token);
  assert(ch && "(stream-open) opened and registered the audio channel");

  // Audio consumers use ring mode (drop-oldest, drained in order) — drain a chunk with try_pop. Use
  // the default ring depth: the stream opened at the default subscriber_depth (3), and subscribe()
  // refuses a depth the fixed pool can't cover (so an 8-deep ring on a depth-3 stream returns
  // null).
  std::shared_ptr<sx::subscription> sub = ch->subscribe(sx::deliver_mode::ring);
  assert(sub && "subscribe (ring) succeeded");
  sx::frame_ptr f;
  bool got = false;
  for (int i = 0; i < 600 && !got; ++i) {
    if (sub->try_pop(f) && f)
      got = true;
    else
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  assert(got && "the mix(tone,tone) graph published at least one PCM chunk");
  // One interleaved-f32 chunk is frames*channels*sizeof(float) = 256 * 1 * 4 = 1024 bytes.
  assert(f->size == 256u * 1u * 4u && "an f32 audio chunk is frames*ch*4 bytes");
  assert(f->format.kind == sx::frame_kind::audio_pcm && "the frame is tagged audio_pcm");
  const long long published = static_cast<long long>(ch->total_published());

  sub.reset(); // drop before teardown; the Runtime then closes the stream (joins the producer)

  // A graph nesting mix/gain more than 64 levels deep is refused, not built: a built graph pulls
  // each chunk through one native call per level on the producer thread. 64 levels still play.
  rt.set_tick_program(
      "(if (= (state-get \"deep.done\") \"1\") #f"
      "  (begin"
      "    (set g (dict \"tone\" (dict \"hz\" 440)))"
      "    (set i 0)"
      "    (while (< i 63)"
      "      (begin (set g (dict \"gain\" (dict \"gain\" 1 \"in\" g))) (set i (+ i 1))))"
      "    (set ok (stream-capture-audio"
      "      (get-attr (stream-open (dict \"id\" \"deep64\" \"codec\" \"f32\""
      "                                   \"rate\" 48000 \"ch\" 1 \"frames_per_chunk\" 256))"
      "                \"token\")"
      "      (dict \"rate\" 48000 \"ch\" 1 \"frames\" 256 \"gain\" (dict \"gain\" 1 \"in\" g))))"
      "    (set g (dict \"gain\" (dict \"gain\" 1 \"in\" g)))"
      "    (set no (stream-capture-audio"
      "      (get-attr (stream-open (dict \"id\" \"deep65\" \"codec\" \"f32\""
      "                                   \"rate\" 48000 \"ch\" 1 \"frames_per_chunk\" 256))"
      "                \"token\")"
      "      (dict \"rate\" 48000 \"ch\" 1 \"frames\" 256 \"gain\" (dict \"gain\" 1 \"in\" g))))"
      "    (state-set \"deep.ok\" (get-attr ok \"ok\"))"
      "    (state-set \"deep.error\" (get-attr no \"error\"))"
      "    (state-set \"deep.done\" \"1\")))");
  drain_until_set("deep.done");
  for (const std::string &w : rt.take_reactive_warnings())
    std::printf("warn: %s\n", w.c_str());
  cvc::state &root = cvc::state::instance(app);
  assert(root("deep.done").value() == "1" && "the deep-graph program ran");
  assert(root("deep.ok").value() == "true" && "a 64-level graph is built");
  assert(root("deep.error").value().find("more than 64 levels deep") != std::string::npos &&
         "a 65-level graph is refused with the depth cap's error");

  std::printf("cvcgl_media_audio: OK (mix(tone,tone) via DSL -> %lld chunks of %zu-byte f32)\n",
              published, f->size);
  return 0;
}
