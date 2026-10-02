/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/ariadne/stream/audio_sources.h>

namespace cvc {
namespace ariadne {
namespace stream {

namespace {
constexpr double kTwoPi = 6.28318530717958647692;
// Interpret a slab as interleaved f32 and report how many float samples one frame holds.
std::size_t samples(std::size_t frame_bytes) noexcept { return frame_bytes / sizeof(float); }
} // namespace

std::size_t audio_f32_bytes(int frames_per_chunk, int channels) noexcept {
  if (frames_per_chunk <= 0 || channels <= 0)
    return 0;
  return static_cast<std::size_t>(frames_per_chunk) * static_cast<std::size_t>(channels) *
         sizeof(float);
}

// ---------------------------------------------------------------------------- tone_source
tone_source::tone_source(int sample_rate, int channels, int frames_per_chunk, double frequency_hz,
                         double amplitude)
    : sr_(sample_rate), ch_(channels), frames_(frames_per_chunk), freq_(frequency_hz),
      amp_(std::clamp(amplitude, 0.0, 1.0)) {} // a source amplitude never exceeds unity

std::size_t tone_source::frame_bytes() const { return audio_f32_bytes(frames_, ch_); }

produced_frame tone_source::fill(std::uint8_t *buf, std::size_t cap) {
  produced_frame out;
  const std::size_t need = frame_bytes();
  if (need == 0 || buf == nullptr || cap < need)
    return out;
  if (sr_ <= 0) {
    out.stop = true; // a tone with no sample rate is misconfigured — stop, don't skip forever
    return out;
  }
  float *f = reinterpret_cast<float *>(buf);
  const double step = kTwoPi * freq_ / static_cast<double>(sr_);
  for (int n = 0; n < frames_; ++n) {
    const float s = static_cast<float>(amp_ * std::sin(phase_));
    phase_ += step;
    // Bring the accumulator back into [0, 2pi) for ANY step (incl. freq >= sr or negative), so it
    // stays bounded and sin() keeps its precision over long runs.
    phase_ -= kTwoPi * std::floor(phase_ / kTwoPi);
    for (int c = 0; c < ch_; ++c)
      *f++ = s; // same tone on every channel
  }
  out.bytes = need;
  out.pts_seconds = static_cast<double>(produced_) * frames_ / static_cast<double>(sr_);
  ++produced_;
  return out;
}

// ---------------------------------------------------------------------------- gain_source
gain_source::gain_source(std::unique_ptr<frame_source> input, int channels, int frames_per_chunk,
                         float gain)
    : in_(std::move(input)), ch_(channels), frames_(frames_per_chunk), gain_(gain) {
  // Validate the wrapped source produces OUR frame size; a mismatch is a misconfigured graph that
  // fill() reports loudly (stop) instead of discarding every frame forever.
  match_ = in_ && in_->frame_bytes() == audio_f32_bytes(frames_, ch_);
}

std::size_t gain_source::frame_bytes() const { return audio_f32_bytes(frames_, ch_); }

produced_frame gain_source::fill(std::uint8_t *buf, std::size_t cap) {
  produced_frame out;
  const std::size_t need = frame_bytes();
  if (need == 0 || buf == nullptr || cap < need || !in_)
    return out;
  if (!match_) {
    out.stop = true; // input's frame size disagrees with ours — stop rather than skip forever
    return out;
  }
  if (scratch_.size() < need)
    scratch_.resize(need);
  const produced_frame in = in_->fill(scratch_.data(), scratch_.size());
  if (in.bytes != need) {
    out.stop = in.stop; // a transient skip (bytes 0) or a clean stop — pass it through
    return out;
  }
  const std::size_t n = samples(need);
  const float *src = reinterpret_cast<const float *>(scratch_.data());
  float *dst = reinterpret_cast<float *>(buf);
  for (std::size_t i = 0; i < n; ++i)
    dst[i] = src[i] * gain_; // may exceed [-1,1]; a downstream sink clamps (see header)
  out.bytes = need;
  out.pts_seconds = in.pts_seconds;
  out.stop = in.stop;
  return out;
}

// ---------------------------------------------------------------------------- mix_source
mix_source::mix_source(std::vector<std::unique_ptr<frame_source>> inputs, int sample_rate,
                       int channels, int frames_per_chunk)
    : ins_(std::move(inputs)), spent_(ins_.size(), false), sr_(sample_rate), ch_(channels),
      frames_(frames_per_chunk) {
  // Drop (pre-spend) any input whose frame size disagrees with ours; it is never pulled. If that
  // leaves nothing live, the first fill() stops.
  const std::size_t need = audio_f32_bytes(frames_, ch_);
  for (std::size_t k = 0; k < ins_.size(); ++k)
    if (!ins_[k] || ins_[k]->frame_bytes() != need)
      spent_[k] = true;
}

std::size_t mix_source::frame_bytes() const { return audio_f32_bytes(frames_, ch_); }

produced_frame mix_source::fill(std::uint8_t *buf, std::size_t cap) {
  produced_frame out;
  const std::size_t need = frame_bytes();
  if (need == 0 || buf == nullptr || cap < need)
    return out;
  const std::size_t n = samples(need);
  float *acc = reinterpret_cast<float *>(buf);
  std::fill(acc, acc + n, 0.0f); // start from silence; absent/skipping inputs contribute nothing
  if (scratch_.size() < need)
    scratch_.resize(need);

  bool any_live = false;
  for (std::size_t k = 0; k < ins_.size(); ++k) {
    if (spent_[k] || !ins_[k])
      continue;
    any_live = true;
    const produced_frame in = ins_[k]->fill(scratch_.data(), scratch_.size());
    if (in.stop)
      spent_[k] = true;
    if (in.bytes == need) {
      const float *s = reinterpret_cast<const float *>(scratch_.data());
      for (std::size_t i = 0; i < n; ++i)
        acc[i] += s[i];
    }
  }
  if (!any_live) {
    out.stop = true; // every input is spent (or dropped as mismatched) -> the mix is spent
    return out;
  }
  for (std::size_t i = 0; i < n; ++i)
    acc[i] = std::clamp(acc[i], -1.0f, 1.0f); // summed signals can exceed unity
  out.bytes = need; // emit a frame per tick while any input is live (silence fills a quiet tick)
  // Monotonic OWN time base: advance pts every emitted chunk, even on an all-skip (silent) tick, so
  // a stalling input can never make the output pts go backwards or stick at 0.
  out.pts_seconds =
      (sr_ > 0) ? static_cast<double>(produced_) * frames_ / static_cast<double>(sr_) : 0.0;
  ++produced_;
  return out;
}

} // namespace stream
} // namespace ariadne
} // namespace cvc
