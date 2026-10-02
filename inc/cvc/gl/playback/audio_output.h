/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_PLAYBACK_AUDIO_OUTPUT_H__
#define __CVC_GL_PLAYBACK_AUDIO_OUTPUT_H__

// Phase-3 AUDIO playback: an SDL3 playback device (speakers, headphones, or any OS/virtual output
// endpoint) as a SINK that consumes interleaved 32-bit float PCM (codec "f32") — the output mirror
// of cvc::gl::capture::audio_source. Where a capture source is a frame_source a stream PULLS from,
// an audio_output is a sink the application PUSHES decoded/recorded frames into: pull the latest
// frames off a stream subscription on the app tick and hand each chunk's bytes to play(). This is
// deliberately NOT tied to the stream types (it takes raw bytes), so it plays a live mic stream, a
// recorded buffer replayed through a buffer_source, a tone, or any f32 PCM the app has in hand.
//
// Playback lives in cvcGL because SDL3 is cvcGL's peripheral seam; the cvc monolith does not link
// SDL. SDL types and the device live in the .cpp gated on CVC_ENABLE_SDL, and with no SDL in the
// build open_audio_output() returns an empty result and list_audio_playback_devices() an empty list
// (graceful absence) — callers branch on `result.sink == nullptr`.
//
// Holy-grail record/playback (docs: the mic demo): record = append each mic-stream frame's bytes to
// a buffer; playback = feed that buffer back (e.g. via a buffer_source stream) and pump its frames
// to an audio_output, which routes them to SDL exactly as the capture path routed the mic in.
//
//   auto out = cvc::gl::playback::open_audio_output({48000, 2}); // 48 kHz stereo
//   if (out.sink) {
//     // per app tick, for each f32 chunk pulled from the stream subscription:
//     out.sink->play(chunk.data(), chunk.size());
//     // ... and drop any unplayed tail when stopping:
//     // out.sink->flush();
//   }

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace gl {
namespace playback {

// What to open the playback device as. SDL converts our f32 chunks to the hardware's native format,
// so the app always hands over interleaved f32 at `sample_rate` on `channels`. Defaults: 48 kHz
// stereo.
struct audio_output_spec {
  int sample_rate = 48000;
  int channels = 2;
  int device_index = -1; // -1 = the system default playback device; else an index into
                         // list_audio_playback_devices()
};

// An opened playback device. play() queues PCM for the device to render on its own audio thread;
// the sink never blocks on the device.
class audio_output {
public:
  virtual ~audio_output() = default;

  // Queue one interleaved f32 PCM chunk (channels @ sample_rate) for playback. `bytes` must be a
  // whole number of audio frames (a multiple of channels * sizeof(float)); a non-aligned size is
  // rejected (returns false) rather than desyncing the channels. Returns false on a push failure or
  // when the sink is not open. Non-blocking: SDL plays the queued data asynchronously.
  virtual bool play(const void *data, std::size_t bytes) = 0;

  // Bytes still queued in the device and not yet played. 0 means everything handed to play() has
  // played out — use it to pace (avoid queueing unboundedly ahead) or to detect "done".
  virtual std::size_t queued_bytes() const = 0;

  // Drop any queued-but-unplayed audio immediately (e.g. stop playback when the user toggles the
  // mic off). Safe to call repeatedly. NOTE: this empties the stream's queue, but a short tail (up
  // to one device-buffer period, tens of ms) that SDL has already pulled into the hardware mixing
  // buffer may still play out — SDL exposes no stronger synchronous stop for a device-bound stream.
  virtual void flush() = 0;
};

// The outcome of open_audio_output(): the sink plus the format it plays (equal to the requested
// spec — SDL converts to the hardware's native format under it).
struct audio_output_open {
  std::unique_ptr<audio_output> sink; // null on failure / no SDL / no device
  int sample_rate = 0;
  int channels = 0;
  std::string name; // the opened device's human-readable name
};

// Open a playback device per `spec` and return a sink. Returns an empty result ({nullptr, …}) if
// SDL is unavailable, the index is out of range, or the device could not be opened.
//
// THREAD CONTRACT: open_audio_output(), list_audio_playback_devices(), and destruction of the sink
// touch the SDL audio subsystem (SDL_InitSubSystem / SDL_QuitSubSystem are NOT thread-safe), so
// call them on the SDL MAIN thread. play(), queued_bytes(), and flush() use SDL stream calls
// documented safe from any thread, so the app may pump them from its render/consumer loop.
audio_output_open open_audio_output(const audio_output_spec &spec = {});

// Human-readable names of the available playback devices (index aligns with
// audio_output_spec::device_index). Empty when SDL is unavailable or none is present. Call on the
// SDL main thread (see open_audio_output).
std::vector<std::string> list_audio_playback_devices();

} // namespace playback
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_PLAYBACK_AUDIO_OUTPUT_H__
