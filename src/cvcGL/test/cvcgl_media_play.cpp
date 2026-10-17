/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_media_play — the (stream-play-audio) verb (conference roadmap §4a), driven through the
// Ariadne Runtime. A .ari opens an f32 audio stream, attaches a tone producer, then plays it: the
// verb wires an audio_stream_player (#634) that subscribes the stream and pumps chunks to a speaker
// sink. With no SDL device (CI) the player is a graceful no-op for OUTPUT, but it still DRAINS the
// stream on its pump thread — observable here as a live subscriber on the channel — so the verb's
// wiring is testable headlessly (no speakers, no GL). The play() mechanics with a real sink are
// covered by cvcgl_audio_player (#634).

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>       // Runtime
#include <cvc/ariadne/ftxui_backend.h> // a headless Backend
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
  ari::register_stream_intrinsics(app);
  cvc::gl::ariadne::register_media_intrinsics();

  ari::Runtime rt(app, "");
  ari::FtxuiBackend backend;
  rt.set_backend(&backend);

  // Open an f32 audio stream with subscriber_depth 8 (the player subscribes a depth-8 ring, so the
  // pool must cover it), attach a 440 Hz tone producer, then (stream-play-audio) it. The canonical
  // token for id "mic" at the empty root is "streams.mic", so we name it directly (no local bind).
  rt.set_tick_program(
      "(if (= (state-get \"play.done\") \"1\") #f"
      "  (begin"
      "    (stream-open (dict \"id\" \"mic\" \"codec\" \"f32\" \"rate\" 48000 \"ch\" 1"
      "                       \"frames_per_chunk\" 256 \"subscriber_depth\" 8))"
      "    (stream-capture-audio \"streams.mic\""
      "                          (dict \"rate\" 48000 \"ch\" 1 \"frames\" 256 \"tone\" (dict "
      "\"hz\" 440)))"
      "    (stream-play-audio \"streams.mic\")"
      "    (state-set \"play.done\" \"1\")))");
  rt.drain();

  for (const std::string &w : rt.take_reactive_warnings())
    std::printf("warn: %s\n", w.c_str());
  assert(rt.take_reactive_warnings().empty() && "open+capture+play evaluated with no warnings");
  assert(cvc::state::instance(app)("play.done").value() == "1" && "the program ran");

  const std::string token = sx::stream::registry_key("", "mic");
  assert(token == "streams.mic");
  sx::stream_channel *ch = sx::stream_registry::for_app(app).lookup(token);
  assert(ch && "(stream-open) opened the audio channel");

  // The player subscribes synchronously inside (stream-play-audio), so its subscription shows on
  // the channel even with no sink (it drains silently). A short poll covers any scheduling slack.
  int subs = 0;
  for (int i = 0; i < 200; ++i) {
    subs = static_cast<int>(ch->subscriber_count());
    if (subs >= 1)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  assert(subs >= 1 && "stream-play-audio attached a draining player (a live subscriber)");

  // And the tone producer keeps publishing (the player's drain keeps the pool from backing up).
  long long published = 0;
  for (int i = 0; i < 200 && published == 0; ++i) {
    published = static_cast<long long>(ch->total_published());
    if (published == 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  assert(published > 0 && "the tone producer published while the player drained");

  // Teardown is RAII + LIFO: the Runtime clears the document scope, destroying the player slot
  // (created after the stream slot) FIRST — the pump stops+joins before the stream/channel closes.
  std::printf(
      "cvcgl_media_play: OK (play-audio attached a draining player; %d subscriber, %lld chunks)\n",
      subs, published);
  return 0;
}
