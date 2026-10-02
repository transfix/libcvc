/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_STREAM_AUDIO_SOURCES_H__
#define __CVC_ARIADNE_STREAM_AUDIO_SOURCES_H__

#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/stream/frame_source.h>
#include <memory>
#include <vector>

// The PURE / VIRTUAL members of the audio-source family (docs/STREAMING.md). All are frame_source
// producing interleaved 32-bit float PCM (codec "f32"), so they compose into an audio graph and
// feed an audio stream exactly like any other frame_source — no SDL, no device, fully testable. The
// DEVICE member (a microphone / any SDL capture device) is a separate cvcGL source; the family is
// deliberately NOT microphone-specific (Joe's directive): a source "could be another audio source,
// not necessarily a microphone", including virtual ones that mix or filter others.
//
// Convention: one frame is a CHUNK of `frames` interleaved samples across `channels`, so
// frame_bytes == frames * channels * sizeof(float). A composite (gain/mix) and its inputs MUST
// share the same channels + frames-per-chunk: a composite checks each input's frame_bytes() at
// construction and treats a mismatch LOUDLY — gain stops (reports end-of-stream), mix drops that
// input — rather than silently discarding frames forever. Mixing across differing rates/layouts is
// a later resampling concern.
//
// Range: tone amplitude is clamped to [0,1], but gain may amplify past unity and a summed mix can
// exceed it — mix_source clamps its output to [-1,1], but gain_source does NOT, so a consumer fed
// directly by gain (e.g. an audio-output sink) must clamp on conversion.

namespace cvc {
namespace ariadne {
namespace stream {

// frames*channels*4 for interleaved f32 — the byte size every source here produces. Use it to size
// a stream: p.format.kind = frame_kind::audio_pcm; p.format.codec = "f32"; p.format.sample_rate =
// sr; p.format.channels = ch; p.format.bytes = audio_f32_bytes(frames, ch);
std::size_t audio_f32_bytes(int frames_per_chunk, int channels) noexcept;

// A continuous sine generator — the hardware-free reference audio source (the audio analogue of
// synthetic_source). Produces `frames` f32 samples/chunk on `channels` (same tone on each), phase
// carried across chunks so the wave is seamless.
class tone_source : public frame_source {
public:
  tone_source(int sample_rate, int channels, int frames_per_chunk, double frequency_hz,
              double amplitude = 0.25);
  std::size_t frame_bytes() const override;
  produced_frame fill(std::uint8_t *buf, std::size_t cap) override;

private:
  int sr_, ch_, frames_;
  double freq_, amp_;
  double phase_ = 0.0;         // radians, carried across chunks
  std::uint64_t produced_ = 0; // chunks, for the pts
};

// A FILTER member: wraps one audio source and scales every sample by `gain` (a simple volume/gain
// stage, the simplest of the filter family). Owns its input and drives it one chunk per fill.
class gain_source : public frame_source {
public:
  gain_source(std::unique_ptr<frame_source> input, int channels, int frames_per_chunk, float gain);
  std::size_t frame_bytes() const override;
  produced_frame fill(std::uint8_t *buf, std::size_t cap) override;

private:
  std::unique_ptr<frame_source> in_;
  int ch_, frames_;
  float gain_;
  bool match_ = false; // input's frame_bytes() matches ours (checked at construction)
  std::vector<std::uint8_t> scratch_;
};

// The MIX member (an audio "multiplexer"): sums N audio sources sample-wise into one, clamping to
// [-1, 1]. Owns the inputs and drives each one chunk per fill; a spent input (stop) contributes
// silence thereafter, and the mix stops when ALL inputs are spent. The mix defines its OWN output
// time base (sample_rate): every emitted frame carries a monotonic pts from a chunk counter, so a
// tick where every input skips still advances pts (a device input that stalls can't break
// ordering).
class mix_source : public frame_source {
public:
  mix_source(std::vector<std::unique_ptr<frame_source>> inputs, int sample_rate, int channels,
             int frames_per_chunk);
  std::size_t frame_bytes() const override;
  produced_frame fill(std::uint8_t *buf, std::size_t cap) override;

private:
  std::vector<std::unique_ptr<frame_source>> ins_;
  std::vector<bool> spent_;
  int sr_, ch_, frames_;
  std::vector<std::uint8_t> scratch_;
  std::uint64_t produced_ = 0; // emitted chunks, for the monotonic output pts
};

} // namespace stream
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_AUDIO_SOURCES_H__
