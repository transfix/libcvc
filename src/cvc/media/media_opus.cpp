/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/media/media_opus.h>

#ifdef CVC_HAVE_OPUS
#include <opus/opus.h>
#endif

namespace cvc {

namespace {
// 20 ms per packet, the speech default; the largest packet Opus writes is far below 4000 bytes.
constexpr int kFrameMs = 20;
constexpr int kMaxPacket = 4000;
// The longest frame a packet can hold (120 ms at 48 kHz), the decoder's buffer.
constexpr int kMaxDecodeFrames = 5760;
} // namespace

bool opus_available() noexcept {
#ifdef CVC_HAVE_OPUS
  return true;
#else
  return false;
#endif
}

struct opus_encoder::impl {
#ifdef CVC_HAVE_OPUS
  OpusEncoder *enc = nullptr;
  ~impl() {
    if (enc)
      opus_encoder_destroy(enc);
  }
#endif
  std::vector<float> pending; // interleaved samples not yet a whole frame
  int frame = 0;              // samples per channel in a packet
};

opus_encoder::opus_encoder(int rate, int channels, int bitrate)
    : _impl(std::make_unique<impl>()), _rate(rate), _channels(channels) {
  _impl->frame = rate * kFrameMs / 1000;
#ifdef CVC_HAVE_OPUS
  int err = 0;
  _impl->enc = opus_encoder_create(rate, channels, OPUS_APPLICATION_VOIP, &err);
  if (err != OPUS_OK) {
    _impl->enc = nullptr;
    return;
  }
  set_bitrate(bitrate);
#else
  (void)bitrate;
#endif
}

opus_encoder::~opus_encoder() = default;

bool opus_encoder::valid() const noexcept {
#ifdef CVC_HAVE_OPUS
  return _impl->enc != nullptr;
#else
  return false;
#endif
}

void opus_encoder::set_bitrate(int bitrate) {
#ifdef CVC_HAVE_OPUS
  if (_impl->enc)
    opus_encoder_ctl(_impl->enc, OPUS_SET_BITRATE(bitrate));
#else
  (void)bitrate;
#endif
}

std::vector<opus_packet> opus_encoder::encode(const float *pcm, int frames) {
  std::vector<opus_packet> out;
#ifdef CVC_HAVE_OPUS
  if (!_impl->enc || !pcm || frames <= 0)
    return out;
  _impl->pending.insert(_impl->pending.end(), pcm,
                        pcm + static_cast<std::size_t>(frames) * _channels);
  const std::size_t per = static_cast<std::size_t>(_impl->frame) * _channels;
  std::size_t at = 0;
  while (_impl->pending.size() - at >= per) {
    opus_packet p;
    p.bytes.resize(kMaxPacket);
    const opus_int32 n = opus_encode_float(_impl->enc, _impl->pending.data() + at, _impl->frame,
                                           p.bytes.data(), kMaxPacket);
    at += per;
    if (n <= 0)
      continue; // an encode error loses that frame, not the stream
    p.bytes.resize(static_cast<std::size_t>(n));
    p.frames = _impl->frame;
    out.push_back(std::move(p));
  }
  _impl->pending.erase(_impl->pending.begin(),
                       _impl->pending.begin() + static_cast<std::ptrdiff_t>(at));
#else
  (void)pcm;
  (void)frames;
#endif
  return out;
}

void opus_encoder::reset() {
  _impl->pending.clear();
#ifdef CVC_HAVE_OPUS
  if (_impl->enc)
    opus_encoder_ctl(_impl->enc, OPUS_RESET_STATE);
#endif
}

struct opus_decoder::impl {
#ifdef CVC_HAVE_OPUS
  OpusDecoder *dec = nullptr;
  ~impl() {
    if (dec)
      opus_decoder_destroy(dec);
  }
#endif
};

opus_decoder::opus_decoder(int rate, int channels)
    : _impl(std::make_unique<impl>()), _rate(rate), _channels(channels) {
#ifdef CVC_HAVE_OPUS
  int err = 0;
  _impl->dec = opus_decoder_create(rate, channels, &err);
  if (err != OPUS_OK)
    _impl->dec = nullptr;
#endif
}

opus_decoder::~opus_decoder() = default;

bool opus_decoder::valid() const noexcept {
#ifdef CVC_HAVE_OPUS
  return _impl->dec != nullptr;
#else
  return false;
#endif
}

std::vector<float> opus_decoder::decode(const std::uint8_t *packet, std::size_t size) {
#ifdef CVC_HAVE_OPUS
  if (!_impl->dec || !packet || size == 0)
    return {};
  std::vector<float> pcm(static_cast<std::size_t>(kMaxDecodeFrames) * _channels);
  const int n = opus_decode_float(_impl->dec, packet, static_cast<opus_int32>(size), pcm.data(),
                                  kMaxDecodeFrames, /*decode_fec=*/0);
  if (n <= 0)
    return {};
  pcm.resize(static_cast<std::size_t>(n) * _channels);
  return pcm;
#else
  (void)packet;
  (void)size;
  return {};
#endif
}

std::vector<float> opus_decoder::conceal(int frames) {
#ifdef CVC_HAVE_OPUS
  if (!_impl->dec || frames <= 0 || frames > kMaxDecodeFrames)
    return {};
  std::vector<float> pcm(static_cast<std::size_t>(frames) * _channels);
  const int n = opus_decode_float(_impl->dec, nullptr, 0, pcm.data(), frames, /*decode_fec=*/0);
  if (n <= 0)
    return {};
  pcm.resize(static_cast<std::size_t>(n) * _channels);
  return pcm;
#else
  (void)frames;
  return {};
#endif
}

} // namespace cvc
