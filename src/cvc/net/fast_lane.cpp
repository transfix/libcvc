/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cstring>
#include <cvc/net/fast_lane.h>

namespace cvc {

namespace {

// Little-endian writers and a bounds-checked reader over one datagram.
void put_u8(std::vector<std::uint8_t> &out, std::uint8_t v) { out.push_back(v); }

template <class T> void put_le(std::vector<std::uint8_t> &out, T v) {
  std::uint8_t b[sizeof(T)];
  std::memcpy(b, &v, sizeof(T));
  // Every target libcvc builds for (x86-64, arm64, wasm32) is little-endian already.
  out.insert(out.end(), b, b + sizeof(T));
}

bool put_str(std::vector<std::uint8_t> &out, const std::string &s) {
  if (s.size() > 255)
    return false;
  out.push_back(static_cast<std::uint8_t>(s.size()));
  out.insert(out.end(), s.begin(), s.end());
  return true;
}

struct reader {
  const std::uint8_t *p;
  std::size_t n;
  bool ok = true;

  bool take(void *dst, std::size_t k) {
    if (!ok || k > n)
      return ok = false;
    std::memcpy(dst, p, k);
    p += k;
    n -= k;
    return true;
  }
  template <class T> T le() {
    T v{};
    take(&v, sizeof(T));
    return v;
  }
  std::string str() {
    const auto len = le<std::uint8_t>();
    if (!ok || len > n) {
      ok = false;
      return {};
    }
    std::string s(reinterpret_cast<const char *>(p), len);
    p += len;
    n -= len;
    return s;
  }
};

} // namespace

std::uint8_t lane_kind_of(const std::uint8_t *data, std::size_t n) noexcept {
  return n == 0 || data == nullptr ? 0 : data[0];
}

std::vector<std::uint8_t> encode_lane_media(const media_frame &f) {
  std::vector<std::uint8_t> out;
  out.reserve(64 + f.payload.size());
  put_u8(out, static_cast<std::uint8_t>(lane_kind::media));
  if (!put_str(out, f.origin_node_id) || !put_str(out, f.stream_id) || !put_str(out, f.codec))
    return {};
  put_le(out, f.width);
  put_le(out, f.height);
  put_le(out, f.seq);
  put_le(out, f.pts_seconds);
  put_le(out, f.sample_rate);
  put_le(out, f.channels);
  if (out.size() + f.payload.size() > kLaneMaxDatagram)
    return {};
  out.insert(out.end(), f.payload.begin(), f.payload.end());
  return out;
}

bool decode_lane_media(const std::uint8_t *data, std::size_t n, media_frame &out) {
  if (lane_kind_of(data, n) != static_cast<std::uint8_t>(lane_kind::media))
    return false;
  reader r{data + 1, n - 1};
  out.origin_node_id = r.str();
  out.stream_id = r.str();
  out.codec = r.str();
  out.width = r.le<std::uint32_t>();
  out.height = r.le<std::uint32_t>();
  out.seq = r.le<std::int64_t>();
  out.pts_seconds = r.le<double>();
  out.sample_rate = r.le<std::uint32_t>();
  out.channels = r.le<std::uint32_t>();
  if (!r.ok)
    return false;
  out.payload.assign(r.p, r.p + r.n);
  return true;
}

std::vector<std::uint8_t> encode_lane_probe(lane_kind kind, std::uint64_t seq,
                                            std::int64_t sent_ns) {
  std::vector<std::uint8_t> out;
  out.reserve(17);
  put_u8(out, static_cast<std::uint8_t>(kind));
  put_le(out, seq);
  put_le(out, sent_ns);
  return out;
}

bool decode_lane_probe(const std::uint8_t *data, std::size_t n, lane_kind &kind, std::uint64_t &seq,
                       std::int64_t &sent_ns) {
  const std::uint8_t k = lane_kind_of(data, n);
  if (k != static_cast<std::uint8_t>(lane_kind::ping) &&
      k != static_cast<std::uint8_t>(lane_kind::pong))
    return false;
  reader r{data + 1, n - 1};
  seq = r.le<std::uint64_t>();
  sent_ns = r.le<std::int64_t>();
  if (!r.ok || r.n != 0)
    return false;
  kind = static_cast<lane_kind>(k);
  return true;
}

} // namespace cvc
