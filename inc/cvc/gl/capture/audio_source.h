/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_CAPTURE_AUDIO_SOURCE_H__
#define __CVC_GL_CAPTURE_AUDIO_SOURCE_H__

// Phase-3 AUDIO capture: an SDL3 recording device (a microphone, line-in, or any OS capture
// endpoint — including a virtual/loopback device the OS exposes) as a cvc::ariadne::stream::
// frame_source producing interleaved 32-bit float PCM (codec "f32"). This is the DEVICE member of
// the audio-source family (docs/STREAMING.md); the PURE/virtual members (tone, gain, mix) are
// cvc-core and need no SDL. Because the device source IS just a frame_source emitting f32 chunks,
// it composes with those exactly like any other — e.g. open_audio_capture() -> gain_source ->
// mix_source -> stream — and a stream and its sinks never learn the audio came from a microphone.
//
// Capture lives in cvcGL because SDL3 is cvcGL's peripheral seam (keyboard / mouse / gamepad /
// camera / audio); the cvc monolith does not link SDL. The source is the audio analogue of
// camera_source: SDL types and the capture loop live in the .cpp gated on CVC_ENABLE_SDL, and with
// no SDL in the build open_audio_capture() returns an empty result and list_audio_capture_devices()
// an empty list (graceful absence) — callers branch on `result.source == nullptr`.
//
// Drive it with stream::start_producer(), sizing the stream to frame_bytes():
//
//   cvc::gl::capture::audio_capture_spec req; // 48 kHz stereo, 1024-frame chunks (defaults)
//   auto mic = cvc::gl::capture::open_audio_capture(req);
//   if (mic.source) {
//     cvc::ariadne::stream::stream_params p;
//     p.id = "mic";
//     p.format.kind = cvc::ariadne::stream::frame_kind::audio_pcm;
//     p.format.codec = "f32";
//     p.format.sample_rate = mic.sample_rate;
//     p.format.channels = mic.channels;
//     p.format.bytes = mic.source->frame_bytes();
//     auto s = cvc::ariadne::stream::stream::open(app, p);
//     // Pull chunks at the capture cadence: sample_rate / frames_per_chunk pulls per second.
//     s->start_producer(std::move(mic.source),
//                        static_cast<double>(mic.sample_rate) / mic.frames_per_chunk);
//   }

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

// Full definition (NOT a forward declaration): audio_capture_open holds a unique_ptr<frame_source>,
// so ~audio_capture_open needs the complete type. frame_source.h pulls only <cstddef>/<cstdint> (no
// SDL), so the "SDL stays out of this header" invariant is preserved.
#include <cvc/ariadne/stream/frame_source.h>

namespace cvc {
namespace gl {
namespace capture {

// What to ask the device for. SDL converts the hardware's native format to this on the way out, so
// the source always yields exactly `channels` interleaved f32 samples at `sample_rate`, in chunks
// of `frames_per_chunk` frames. Defaults: 48 kHz stereo, 1024-frame chunks (~21 ms at 48 kHz).
struct audio_capture_spec {
  int sample_rate = 48000;
  int channels = 2;
  int frames_per_chunk = 1024;
  int device_index = -1; // -1 = the system default recording device; else an index into
                         // list_audio_capture_devices()
};

// The outcome of open_audio_capture(): the frame_source plus the format it actually produces (equal
// to the requested spec — SDL converts to it), so the caller sizes its stream to match.
struct audio_capture_open {
  std::unique_ptr<cvc::ariadne::stream::frame_source>
      source;               // null on failure / no SDL / no device
  int sample_rate = 0;      // frames per second
  int channels = 0;         // interleaved channels per frame
  int frames_per_chunk = 0; // frames per emitted chunk (frame_bytes == frames*channels*4)
  std::string name;         // the opened device's human-readable name
};

// Open a recording device per `spec` and return a frame_source that yields interleaved f32 PCM.
// Returns an empty result ({nullptr, …}) if SDL is unavailable, the index is out of range, or the
// device could not be opened. Unlike a camera, an audio recording device needs no asynchronous
// permission handshake on the desktop backends, so the source is ready to pull on return (early
// fill()s simply skip — bytes 0 — until the device has buffered a full chunk).
//
// THREAD CONTRACT: open_audio_capture(), list_audio_capture_devices(), and destruction of the
// stream/source returned here touch the SDL audio subsystem (SDL_InitSubSystem / SDL_QuitSubSystem
// are NOT thread-safe), so call them — and destroy the owning stream — on the SDL MAIN thread (the
// same thread that drives SDL and runs stream::close() / ~stream). Only the per-chunk pull inside
// fill() runs on the producer thread, using SDL_GetAudioStreamData / SDL_GetAudioStreamAvailable,
// which SDL documents as safe to call from any thread.
audio_capture_open open_audio_capture(const audio_capture_spec &spec = {});

// Human-readable names of the available recording devices (index aligns with
// audio_capture_spec::device_index). Empty when SDL is unavailable or none is present. Call on the
// SDL main thread (see open_audio_capture).
std::vector<std::string> list_audio_capture_devices();

} // namespace capture
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_CAPTURE_AUDIO_SOURCE_H__
