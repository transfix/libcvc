/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_MEDIA_OPUS_H__
#define __CVC_MEDIA_OPUS_H__

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace cvc {

// ----------------
// Opus audio for the conference (ARIADNE_CONFERENCE_ROADMAP §5j)
// ----------------
// A mic sent as raw 32-bit float PCM at 48 kHz is 1.5 Mbit/s; Opus carries speech well at 32
// kbit/s. A sender encodes its mic chunks (codec "opus", one packet a frame), a viewer decodes
// them, and a relay decodes them to f32 for a viewer that cannot (an older client). Opus needs
// libopus (CVC_HAVE_OPUS); without it opus_available() is false and the mic stays f32.
//
// Opus frames are 2.5 to 60 ms; these use 20 ms (960 samples at 48 kHz), so a sender's encoder
// takes chunks of any size (the mic sends 1024 samples) and emits a packet per 20 ms it has.

// Whether this build has libopus.
bool opus_available() noexcept;

// One packet out of an encoder: the bytes, and how many samples per channel it carries.
struct opus_packet {
  std::vector<std::uint8_t> bytes;
  int frames = 0;
};

class opus_encoder {
public:
  // RATE (8000, 12000, 16000, 24000 or 48000), CHANNELS (1 or 2), BITRATE bits/s. Use valid()
  // to see whether it opened (it does not without libopus).
  opus_encoder(int rate, int channels, int bitrate);
  ~opus_encoder();
  opus_encoder(const opus_encoder &) = delete;
  opus_encoder &operator=(const opus_encoder &) = delete;

  bool valid() const noexcept;
  int rate() const noexcept { return _rate; }
  int channels() const noexcept { return _channels; }
  void set_bitrate(int bitrate);

  // Take FRAMES samples per channel of interleaved float PCM and return the packets they complete
  // (20 ms each); a remainder waits for the next call.
  std::vector<opus_packet> encode(const float *pcm, int frames);
  // Forget the remainder and the encoder's history: the next encode starts a new talk spurt (a
  // sender that stopped sending through a silence).
  void reset();

private:
  struct impl;
  std::unique_ptr<impl> _impl;
  int _rate, _channels;
};

class opus_decoder {
public:
  opus_decoder(int rate, int channels);
  ~opus_decoder();
  opus_decoder(const opus_decoder &) = delete;
  opus_decoder &operator=(const opus_decoder &) = delete;

  bool valid() const noexcept;
  int rate() const noexcept { return _rate; }
  int channels() const noexcept { return _channels; }

  // PACKET's interleaved float PCM; empty when it does not decode.
  std::vector<float> decode(const std::uint8_t *packet, std::size_t size);
  // FRAMES samples per channel standing in for a lost packet (Opus's concealment), to keep a
  // receiver's audio in step; empty without a decoder.
  std::vector<float> conceal(int frames);

private:
  struct impl;
  std::unique_ptr<impl> _impl;
  int _rate, _channels;
};

} // namespace cvc

#endif // __CVC_MEDIA_OPUS_H__
