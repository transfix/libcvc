/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/ariadne/stream/frame_source.h>
#include <cvc/gl/capture/audio_source.h>

#if CVC_ENABLE_SDL
#include <SDL3/SDL_audio.h>
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_stdinc.h> // SDL_free
#include <cstdint>
#endif

namespace cvc {
namespace gl {
namespace capture {

#if CVC_ENABLE_SDL
namespace {
namespace st = cvc::ariadne::stream;

std::size_t f32_chunk_bytes(int frames, int channels) {
  if (frames <= 0 || channels <= 0)
    return 0;
  return static_cast<std::size_t>(frames) * static_cast<std::size_t>(channels) * sizeof(float);
}

// An SDL3 recording device as a frame_source producing interleaved f32 PCM. Owns the
// SDL_AudioStream (which, created via SDL_OpenAudioDeviceStream, also owns the device — destroying
// the stream closes the device) and one reference to the refcounted SDL audio subsystem; both are
// released in the dtor. stream::start_producer's join-before-free guarantee means the producer
// thread has already stopped before the dtor runs, so no fill() races the SDL_DestroyAudioStream.
class sdl_audio_capture_source : public st::frame_source {
public:
  sdl_audio_capture_source(SDL_AudioStream *stream, int sample_rate, int channels, int frames)
      : stream_(stream), sr_(sample_rate), ch_(channels), frames_(frames) {
    // Latency ceiling for drop-at-source (see fill()): keep at most ~80 ms of audio queued in SDL's
    // capture FIFO, but never below two chunks, so ordinary per-tick jitter never triggers a drop —
    // only a real consumer stall does.
    const std::size_t chunk = f32_chunk_bytes(frames_, ch_);
    const std::size_t eighty_ms =
        static_cast<std::size_t>(sr_) * static_cast<std::size_t>(ch_) * sizeof(float) / 12u;
    max_queued_bytes_ = (chunk * 2u > eighty_ms) ? chunk * 2u : eighty_ms;
  }
  ~sdl_audio_capture_source() override {
    if (stream_)
      SDL_DestroyAudioStream(stream_); // also closes the bound recording device
    SDL_QuitSubSystem(SDL_INIT_AUDIO); // balances the InitSubSystem open_audio_capture handed us
  }

  std::size_t frame_bytes() const override { return f32_chunk_bytes(frames_, ch_); }

  st::produced_frame fill(std::uint8_t *buf, std::size_t cap) override {
    st::produced_frame out; // bytes = 0 by default: "no chunk ready this tick" (producer runs on)
    const std::size_t need = frame_bytes();
    if (!stream_ || need == 0 || buf == nullptr || cap < need)
      return out;

    // A full chunk must be captured before we emit one; a sub-chunk remainder is left for the next
    // tick (consuming less than a whole chunk would desync frames). A warming-up or stalled device
    // simply skips here (bytes 0) — the live-but-silent tick the audio mix absorbs.
    const int avail = SDL_GetAudioStreamAvailable(stream_);
    if (avail < 0 || static_cast<std::size_t>(avail) < need)
      return out;

    // DROP AT SOURCE (frame_source.h BACKPRESSURE): unlike a camera, whose SDL API hands back only
    // the newest frame, an SDL recording stream is a FIFO that accumulates EVERY captured sample. A
    // stalled consumer would grow it without bound and add permanent monitoring latency. When the
    // backlog exceeds the latency ceiling, discard the oldest samples down to the single
    // most-recent chunk (the audio analogue of the camera dropping stale frames) and advance the
    // clock over the dropped audio so the pts keeps tracking real capture time rather than drifting
    // behind it.
    if (static_cast<std::size_t>(avail) > max_queued_bytes_) {
      std::size_t drop = static_cast<std::size_t>(avail) - need; // keep exactly one chunk
      std::size_t dropped = 0;
      while (drop > 0) {
        const int n =
            SDL_GetAudioStreamData(stream_, buf, static_cast<int>(drop < need ? drop : need));
        if (n <= 0)
          break; // stream drained faster than expected -> stop shedding
        drop -= static_cast<std::size_t>(n);
        dropped += static_cast<std::size_t>(n);
      }
      const std::size_t frame_sz = static_cast<std::size_t>(ch_) * sizeof(float); // bytes per frame
      if (frame_sz > 0)
        frames_consumed_ += dropped / frame_sz; // the clock skips past the discarded audio
    }

    const int got = SDL_GetAudioStreamData(stream_, buf, static_cast<int>(need));
    if (got != static_cast<int>(need))
      return out; // short/failed read (unreachable after the avail gate) -> clean skip

    out.bytes = need;
    // pts on the device sample timeline: seconds = frames consumed so far / sample_rate. Counting
    // the dropped frames above keeps it ~wall-clock (so audio stays in sync after a drop) while
    // remaining strictly monotonic (frames_consumed_ only ever increases). The sr_ > 0 branch is
    // dead-but-safe (open validated sr_ >= 1). Consistent with the pure audio sources.
    out.pts_seconds =
        (sr_ > 0) ? static_cast<double>(frames_consumed_) / static_cast<double>(sr_) : 0.0;
    frames_consumed_ += static_cast<std::uint64_t>(frames_); // this chunk's frames
    return out;
  }

private:
  SDL_AudioStream *stream_;
  int sr_;
  int ch_;
  int frames_;
  std::size_t max_queued_bytes_ = 0;  // drop-at-source latency ceiling, in bytes
  std::uint64_t frames_consumed_ = 0; // device frames consumed (emitted + dropped), for the pts
};
} // namespace
#endif // CVC_ENABLE_SDL

audio_capture_open open_audio_capture(const audio_capture_spec &spec) {
  audio_capture_open result;
#if CVC_ENABLE_SDL
  if (spec.sample_rate <= 0 || spec.channels <= 0 || spec.frames_per_chunk <= 0)
    return result; // a misconfigured request produces no source rather than a dead stream
  if (!SDL_InitSubSystem(SDL_INIT_AUDIO))
    return result; // refcounted; the source's dtor (or the cleanup below) quits it
  bool handed_off = false;

  // Resolve the target device id: the system default, or an entry of the enumerated list.
  SDL_AudioDeviceID devid = SDL_AUDIO_DEVICE_DEFAULT_RECORDING;
  std::string name;
  bool ok = true;
  if (spec.device_index >= 0) {
    int count = 0;
    SDL_AudioDeviceID *ids = SDL_GetAudioRecordingDevices(&count);
    if (ids && spec.device_index < count) {
      devid = ids[spec.device_index];
      const char *nm = SDL_GetAudioDeviceName(devid);
      name = nm ? nm : std::string();
    } else {
      ok = false; // index out of range -> no source
    }
    if (ids)
      SDL_free(ids);
  } else {
    const char *nm = SDL_GetAudioDeviceName(devid);
    name = nm ? nm : "default recording device";
  }

  if (ok) {
    // Ask SDL to convert the hardware's native format to interleaved f32 at our rate/channels, so
    // the source produces exactly the f32 "codec" the audio family speaks.
    SDL_AudioSpec want;
    want.format = SDL_AUDIO_F32;
    want.channels = spec.channels;
    want.freq = spec.sample_rate;
    // NULL callback -> pull model: the device captures into the stream and we read converted data
    // with SDL_GetAudioStreamData inside fill().
    SDL_AudioStream *stream = SDL_OpenAudioDeviceStream(devid, &want, nullptr, nullptr);
    // Streams open PAUSED; a device that cannot be resumed would be a silent source that forever
    // skips, so fail the open loudly (null source) rather than hand back a dead stream.
    if (stream && !SDL_ResumeAudioStreamDevice(stream)) {
      SDL_DestroyAudioStream(stream);
      stream = nullptr;
    }
    if (stream) {
      result.sample_rate = spec.sample_rate;
      result.channels = spec.channels;
      result.frames_per_chunk = spec.frames_per_chunk;
      result.name = std::move(name);
      result.source = std::make_unique<sdl_audio_capture_source>(
          stream, spec.sample_rate, spec.channels, spec.frames_per_chunk);
      handed_off = true; // the source now owns `stream` (and its device) and the subsystem ref
    }
  }

  if (!handed_off)
    SDL_QuitSubSystem(SDL_INIT_AUDIO); // nothing took the ref we added above
#else
  (void)spec;
#endif
  return result;
}

std::vector<std::string> list_audio_capture_devices() {
  std::vector<std::string> names;
#if CVC_ENABLE_SDL
  if (!SDL_InitSubSystem(SDL_INIT_AUDIO))
    return names;
  int count = 0;
  SDL_AudioDeviceID *ids = SDL_GetAudioRecordingDevices(&count);
  if (ids) {
    for (int i = 0; i < count; ++i) {
      const char *nm = SDL_GetAudioDeviceName(ids[i]);
      names.emplace_back(nm ? nm : "");
    }
    SDL_free(ids);
  }
  SDL_QuitSubSystem(SDL_INIT_AUDIO);
#endif
  return names;
}

} // namespace capture
} // namespace gl
} // namespace cvc
