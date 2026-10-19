/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_STREAM_MUTED_SOURCE_H__
#define __CVC_ARIADNE_STREAM_MUTED_SOURCE_H__

// muted_source — a frame_source decorator that blanks its inner source's frames while a mute flag
// is set, so a producer can be muted WITHOUT tearing the stream down (the descriptor, the peers'
// subscriptions and the frame cadence all stay live; a muted producer simply emits blank frames).
//
// The inner source is ALWAYS pulled (so a device keeps draining and its clock stays continuous),
// then the slab is zeroed when muted. Zero bytes are the right "blank" for the conference's two
// stream kinds: f32 audio zeros are silence, and rgba8 video zeros encode to a BLACK JPEG (JPEG
// carries no alpha, so the zero alpha is irrelevant) when wrapped by jpeg_encode_source. A raw
// (un-encoded) rgba8 stream would instead read the zeros as transparent black.
//
// The flag is an external std::atomic<bool> (typically the owning stream's — see stream::mute_flag)
// read with acquire on the producer thread and written with release by the toggling verb; it must
// outlive this source. The stream joins its producer before destroying the source, and the flag
// lives on the stream (declared before the source), so the reference is valid for every fill().

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cvc/ariadne/stream/frame_source.h>
#include <memory>
#include <utility>

namespace cvc {
namespace ariadne {
namespace stream {

class muted_source : public frame_source {
public:
  muted_source(std::unique_ptr<frame_source> inner, const std::atomic<bool> &muted)
      : inner_(std::move(inner)), muted_(muted) {}

  std::size_t frame_bytes() const override { return inner_ ? inner_->frame_bytes() : 0; }

  produced_frame fill(std::uint8_t *buf, std::size_t cap) override {
    if (!inner_)
      return {};
    // Always drain the inner source first (keep a device / its clock live across a mute), then
    // blank the slab in place when muted so peers keep receiving well-formed frames (silence /
    // black).
    produced_frame pf = inner_->fill(buf, cap);
    if (pf.bytes > 0 && muted_.load(std::memory_order_acquire))
      std::memset(buf, 0, pf.bytes);
    return pf;
  }

private:
  std::unique_ptr<frame_source> inner_;
  const std::atomic<bool> &muted_;
};

} // namespace stream
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_MUTED_SOURCE_H__
