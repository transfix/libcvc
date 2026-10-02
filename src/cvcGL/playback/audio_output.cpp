/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/gl/playback/audio_output.h>

#if CVC_ENABLE_SDL
#include <SDL3/SDL_audio.h>
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_stdinc.h> // SDL_free
#include <climits>           // INT_MAX
#include <cstdint>
#endif

namespace cvc {
namespace gl {
namespace playback {

#if CVC_ENABLE_SDL
namespace {

// An SDL3 playback device as an audio_output sink. Owns the SDL_AudioStream (which, created via
// SDL_OpenAudioDeviceStream, also owns the device — destroying the stream closes the device) and
// one reference to the refcounted SDL audio subsystem; both released in the dtor. Unlike a capture
// source there is no producer thread here — play()/queued_bytes()/flush() run on whatever app
// thread pumps the stream, using SDL calls documented thread-safe.
class sdl_audio_output : public audio_output {
public:
  sdl_audio_output(SDL_AudioStream *stream, int channels) : stream_(stream), ch_(channels) {}
  ~sdl_audio_output() override {
    if (stream_)
      SDL_DestroyAudioStream(stream_); // also closes the bound playback device
    SDL_QuitSubSystem(SDL_INIT_AUDIO); // balances the InitSubSystem open_audio_output handed us
  }

  bool play(const void *data, std::size_t bytes) override {
    if (!stream_ || data == nullptr || bytes == 0)
      return false;
    const std::size_t frame_sz = static_cast<std::size_t>(ch_) * sizeof(float);
    if (frame_sz == 0 || bytes % frame_sz != 0)
      return false; // not a whole number of audio frames -> reject rather than desync channels
    // SDL_PutAudioStreamData takes an int length; a single buffer larger than INT_MAX (e.g. a
    // one-shot replay of a long recording — ~93 min of f32 stereo @ 48 kHz) would truncate and
    // silently drop most of it. Push in frame-aligned slices so the whole buffer queues regardless
    // of size (ordinary per-chunk pumping is one slice).
    const std::size_t max_slice = (static_cast<std::size_t>(INT_MAX) / frame_sz) * frame_sz;
    const std::uint8_t *p = static_cast<const std::uint8_t *>(data);
    std::size_t left = bytes;
    while (left > 0) {
      const std::size_t n = left < max_slice ? left : max_slice;
      if (!SDL_PutAudioStreamData(stream_, p, static_cast<int>(n)))
        return false; // a failed slice leaves the earlier slices queued; report the failure
      p += n;
      left -= n;
    }
    return true;
  }

  std::size_t queued_bytes() const override {
    if (!stream_)
      return 0;
    const int q = SDL_GetAudioStreamQueued(stream_);
    return q > 0 ? static_cast<std::size_t>(q) : 0u; // negative = error -> report nothing queued
  }

  void flush() override {
    if (stream_)
      SDL_ClearAudioStream(stream_); // drop queued-but-unplayed audio immediately
  }

private:
  SDL_AudioStream *stream_;
  int ch_;
};
} // namespace
#endif // CVC_ENABLE_SDL

audio_output_open open_audio_output(const audio_output_spec &spec) {
  audio_output_open result;
#if CVC_ENABLE_SDL
  if (spec.sample_rate <= 0 || spec.channels <= 0)
    return result; // a misconfigured request produces no sink rather than a dead device
  if (!SDL_InitSubSystem(SDL_INIT_AUDIO))
    return result; // refcounted; the sink's dtor (or the cleanup below) quits it
  bool handed_off = false;

  // Resolve the target device id: the system default, or an entry of the enumerated list.
  SDL_AudioDeviceID devid = SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK;
  std::string name;
  bool ok = true;
  if (spec.device_index >= 0) {
    int count = 0;
    SDL_AudioDeviceID *ids = SDL_GetAudioPlaybackDevices(&count);
    if (ids && spec.device_index < count) {
      devid = ids[spec.device_index];
      const char *nm = SDL_GetAudioDeviceName(devid);
      name = nm ? nm : std::string();
    } else {
      ok = false; // index out of range -> no sink
    }
    if (ids)
      SDL_free(ids);
  } else {
    const char *nm = SDL_GetAudioDeviceName(devid);
    name = nm ? nm : "default playback device";
  }

  if (ok) {
    // Ask SDL to convert our interleaved f32 chunks to the hardware's native format at our
    // rate/channels, so the app always hands over the same f32 "codec" the capture side produces.
    SDL_AudioSpec want;
    want.format = SDL_AUDIO_F32;
    want.channels = spec.channels;
    want.freq = spec.sample_rate;
    // NULL callback -> push model: the app feeds data with SDL_PutAudioStreamData (play()).
    SDL_AudioStream *stream = SDL_OpenAudioDeviceStream(devid, &want, nullptr, nullptr);
    // Streams open PAUSED; a device that cannot be resumed would silently never play, so fail the
    // open loudly (null sink) rather than hand back a dead output.
    if (stream && !SDL_ResumeAudioStreamDevice(stream)) {
      SDL_DestroyAudioStream(stream);
      stream = nullptr;
    }
    if (stream) {
      result.sample_rate = spec.sample_rate;
      result.channels = spec.channels;
      result.name = std::move(name);
      result.sink = std::make_unique<sdl_audio_output>(stream, spec.channels);
      handed_off = true; // the sink now owns `stream` (and its device) and the subsystem ref
    }
  }

  if (!handed_off)
    SDL_QuitSubSystem(SDL_INIT_AUDIO); // nothing took the ref we added above
#else
  (void)spec;
#endif
  return result;
}

std::vector<std::string> list_audio_playback_devices() {
  std::vector<std::string> names;
#if CVC_ENABLE_SDL
  if (!SDL_InitSubSystem(SDL_INIT_AUDIO))
    return names;
  int count = 0;
  SDL_AudioDeviceID *ids = SDL_GetAudioPlaybackDevices(&count);
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

} // namespace playback
} // namespace gl
} // namespace cvc
