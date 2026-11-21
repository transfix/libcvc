/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cstdio>
#include <cvc/media/playback/audio_output.h>
#include <cvc/media/web_audio.h> // a browser: the speaker on an AudioWorklet, SDL the fallback

#if CVC_MEDIA_SDL
#include <SDL3/SDL_audio.h>
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_stdinc.h> // SDL_free
#include <climits>           // INT_MAX
#include <cstdint>
#include <utility>
#include <vector>
#endif

namespace cvc {
namespace media {
namespace playback {

#if CVC_MEDIA_SDL
namespace {

// An SDL3 playback device as an audio_output sink. Owns the SDL_AudioStream (which, created via
// SDL_OpenAudioDeviceStream, also owns the device — destroying the stream closes the device) and
// one reference to the refcounted SDL audio subsystem; both released in the dtor. Unlike a capture
// source there is no producer thread here — play()/queued_bytes()/flush() run on whatever app
// thread pumps the stream, using SDL calls documented thread-safe. A pull-mode sink also has a
// fill function, which SDL's get-callback (on_pull) calls whenever the device needs more audio.
class sdl_audio_output : public audio_output {
public:
  sdl_audio_output(int channels, audio_pull_fn fill) : ch_(channels), fill_(std::move(fill)) {}
  // The stream, opened (paused) with this sink as its callback's userdata.
  void attach(SDL_AudioStream *stream) { stream_ = stream; }
  // The sink holds the SDL audio subsystem reference open_output took, from a successful open.
  void own_subsystem() { owns_subsystem_ = true; }
  ~sdl_audio_output() override {
    if (stream_)
      SDL_DestroyAudioStream(stream_); // closes the device; no on_pull runs after this returns
    if (owns_subsystem_)
      SDL_QuitSubSystem(SDL_INIT_AUDIO); // balances the InitSubSystem open_output handed us
  }

  // SDL_AudioStreamCallback: the device wants ADDITIONAL more bytes than the stream holds. Ask the
  // fill function for that many whole frames and queue them.
  static void SDLCALL on_pull(void *self, SDL_AudioStream *stream, int additional, int /*total*/) {
    auto *o = static_cast<sdl_audio_output *>(self);
    if (!o || !o->fill_ || additional <= 0)
      return;
    const std::size_t frame_sz = static_cast<std::size_t>(o->ch_) * sizeof(float);
    if (frame_sz == 0)
      return;
    const std::size_t frames = (static_cast<std::size_t>(additional) + frame_sz - 1) / frame_sz;
    o->pull_buf_.resize(frames * static_cast<std::size_t>(o->ch_));
    o->fill_(o->pull_buf_.data(), frames);
    SDL_PutAudioStreamData(stream, o->pull_buf_.data(), static_cast<int>(frames * frame_sz));
  }

  double device_period_s() const override {
    if (!stream_)
      return 0.0;
    SDL_AudioSpec spec;
    int frames = 0;
    if (!SDL_GetAudioDeviceFormat(SDL_GetAudioStreamDevice(stream_), &spec, &frames) ||
        spec.freq <= 0 || frames <= 0)
      return 0.0;
    return static_cast<double>(frames) / static_cast<double>(spec.freq);
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
  SDL_AudioStream *stream_ = nullptr;
  int ch_;
  bool owns_subsystem_ = false;
  audio_pull_fn fill_;
  std::vector<float> pull_buf_; // on_pull's scratch (only SDL's audio callback touches it)
};
} // namespace
#endif // CVC_MEDIA_SDL

namespace {
audio_output_open open_output(const audio_output_spec &spec, audio_pull_fn fill) {
  audio_output_open result;
#if CVC_MEDIA_SDL
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
    // No fill -> push model: the app feeds data with SDL_PutAudioStreamData (play()). A fill ->
    // pull model: SDL's get-callback asks the sink, which asks FILL. The stream opens PAUSED, so
    // no callback comes before attach().
    const bool pull = static_cast<bool>(fill);
    auto sink = std::make_unique<sdl_audio_output>(spec.channels, std::move(fill));
    SDL_AudioStream *stream = SDL_OpenAudioDeviceStream(
        devid, &want, pull ? &sdl_audio_output::on_pull : nullptr, pull ? sink.get() : nullptr);
    if (stream)
      sink->attach(stream); // the sink owns the stream (and its device) from here
    // A device that cannot be resumed would silently never play, so fail the open loudly (null
    // sink) rather than hand back a dead output.
    if (stream && !SDL_ResumeAudioStreamDevice(stream))
      stream = nullptr;
    if (stream) {
      result.sample_rate = spec.sample_rate;
      result.channels = spec.channels;
      result.name = std::move(name);
      sink->own_subsystem();
      result.sink = std::move(sink);
      handed_off = true; // the sink now owns `stream` (and its device) and the subsystem ref
    } else {
      sink.reset(); // destroys an opened-but-dead stream, if any
    }
  }

  if (!handed_off)
    SDL_QuitSubSystem(SDL_INIT_AUDIO); // nothing took the ref we added above
#else
  (void)spec;
  (void)fill;
#endif
  return result;
}
} // namespace

audio_output_open open_audio_output(const audio_output_spec &spec) {
  return open_output(spec, nullptr);
}

audio_output_open open_audio_output_pull(const audio_output_spec &spec, audio_pull_fn fill) {
  if (!fill)
    return {};
  // In a browser the default device plays from an AudioWorklet, which keeps playing through a busy
  // main thread or a hidden tab (cvc/media/web_audio.h); SDL's Web Audio backend, whose callback
  // runs on the main thread, is the fallback (no AudioWorklet, another rate, CVC_WEB_AUDIO=sdl).
  if (spec.device_index < 0 && web_audio::prepare()) {
    if (auto sink = web_audio::open_output_pull(spec.sample_rate, spec.channels, fill)) {
      audio_output_open result;
      result.sink = std::move(sink);
      result.sample_rate = spec.sample_rate;
      result.channels = spec.channels;
      result.name = "AudioWorklet";
      return result;
    }
    std::fprintf(stderr, "web audio: the speaker on SDL (worklets %s)\n",
                 web_audio::status().c_str());
  }
  return open_output(spec, std::move(fill));
}

bool have_audio_output() {
#if CVC_MEDIA_SDL
  return true;
#else
  return false;
#endif
}

std::vector<std::string> list_audio_playback_devices() {
  std::vector<std::string> names;
#if CVC_MEDIA_SDL
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
} // namespace media
} // namespace cvc
