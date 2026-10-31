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

#ifndef __CVC_MEDIA_PLAYBACK_AUDIO_STREAM_PLAYER_H__
#define __CVC_MEDIA_PLAYBACK_AUDIO_STREAM_PLAYER_H__

// Phase-3 AUDIO playback, auto-pumped: drive an audio_output sink from an f32-PCM
// stream. audio_output is deliberately raw-bytes and app-pumped (pull each chunk
// off a stream subscription per tick and hand it to play()); this wraps that loop
// in a dedicated pump thread so "play this stream" is one object, not a per-tick
// chore in the host. It is the C++ core behind the stream-play-audio DSL verb.
//
// Two forms:
//   * OWNS the sink — opens an audio_output per `spec` on the constructing thread
//     (the SDL-main thread; see audio_output's thread contract). With no SDL / no
//     device the sink is null and the player is a graceful no-op: it still drains
//     the subscription so the stream never backs up, but nothing is played.
//     playing() reports which.
//   * BORROWS a sink — pumps into a caller-provided audio_output and neither owns
//     nor destroys it (tests inject a fake; a host may share one device across
//     players).
//
// Threading: the pump thread only calls audio_output::play(), documented safe
// from any thread. An OWNED sink is opened and destroyed on the constructing
// thread, and stop()/the destructor close the subscription (unblocking the pump)
// and join it BEFORE that sink is destroyed, so the sink outlives every play().

#include <atomic>
#include <cstdint>
#include <cvc/media/playback/audio_output.h>
#include <memory>
#include <thread>

namespace cvc {

namespace ariadne {
namespace stream {
class stream;
class subscription;
} // namespace stream
} // namespace ariadne

namespace media {
namespace playback {

class audio_stream_player {
public:
  // Owns an audio_output opened per `spec` (on this, the SDL-main, thread).
  audio_stream_player(cvc::ariadne::stream::stream &src, const audio_output_spec &spec = {});
  // Borrows `sink` (not owned); for tests and shared devices.
  audio_stream_player(cvc::ariadne::stream::stream &src, audio_output &sink);
  ~audio_stream_player();

  audio_stream_player(const audio_stream_player &) = delete;
  audio_stream_player &operator=(const audio_stream_player &) = delete;

  // Close the subscription (unblocking the pump) and join it. Idempotent; also
  // run by the destructor (before an owned sink is destroyed).
  void stop();

  // False when there is no sink (no SDL / no device) — the player drains but
  // plays nothing.
  bool playing() const noexcept { return _sink != nullptr; }

  // Chunks successfully handed to the sink's play() (0 while playing()==false).
  std::uint64_t frames_played() const noexcept { return _frames_played.load(); }

private:
  void start(cvc::ariadne::stream::stream &src);

  std::unique_ptr<audio_output> _owned;                     // set only by the owning ctor
  audio_output *_sink = nullptr;                            // borrowed, or == _owned.get()
  std::shared_ptr<cvc::ariadne::stream::subscription> _sub; // the drained ring subscription
  std::thread _pump;
  std::atomic<std::uint64_t> _frames_played{0};
  bool _stopped = false;
};

} // namespace playback
} // namespace media
} // namespace cvc

#endif // __CVC_MEDIA_PLAYBACK_AUDIO_STREAM_PLAYER_H__
