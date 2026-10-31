/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/media/playback/audio_stream_player.h>

namespace cvc {
namespace media {
namespace playback {

namespace cs = cvc::ariadne::stream;

audio_stream_player::audio_stream_player(cs::stream &src, const audio_output_spec &spec) {
  // Open on the constructing (SDL-main) thread. A null sink (no SDL / no device)
  // leaves the player a graceful no-op.
  audio_output_open opened = open_audio_output(spec);
  _owned = std::move(opened.sink);
  _sink = _owned.get();
  start(src);
}

audio_stream_player::audio_stream_player(cs::stream &src, audio_output &sink) {
  _sink = &sink;
  start(src);
}

void audio_stream_player::start(cs::stream &src) {
  // Ring (drop-oldest): a stalled sink must never back the stream up — audio is
  // realtime, a late chunk is worthless, so shed under backpressure at the source.
  _sub = src.channel().subscribe(cs::deliver_mode::ring, 8);
  if (!_sub)
    return; // pool could not cover another subscriber; nothing to pump

  cs::subscription *sub = _sub.get();
  audio_output *sink = _sink;
  std::atomic<std::uint64_t> *played = &_frames_played;
  _pump = std::thread([sub, sink, played]() {
    cs::frame_ptr f;
    while (sub->pop(f)) { // blocks; returns false when the subscription closes
      if (!f)
        continue;
      // play() is thread-safe and non-blocking (SDL queues asynchronously).
      if (sink && sink->play(f->data, f->size))
        played->fetch_add(1, std::memory_order_relaxed);
    }
  });
}

audio_stream_player::~audio_stream_player() { stop(); }

void audio_stream_player::stop() {
  if (_stopped)
    return;
  _stopped = true;
  if (_sub)
    _sub->close(); // unblock the pump's pop()
  if (_pump.joinable())
    _pump.join(); // joined before _owned (if any) is destroyed on this thread
}

} // namespace playback
} // namespace media
} // namespace cvc
