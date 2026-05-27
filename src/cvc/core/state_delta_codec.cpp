/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <cvc/core/state_delta_codec.h>
#include <stdexcept>

namespace cvc {

// Build the IEEE 802.3 CRC-32 look-up table at static init time.
static std::uint32_t make_crc_entry(std::uint32_t idx) {
  std::uint32_t crc = idx;
  for (int j = 0; j < 8; ++j)
    crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1u) + 1u));
  return crc;
}

struct crc_table_t {
  std::uint32_t t[256];
  crc_table_t() {
    for (int i = 0; i < 256; ++i)
      t[i] = make_crc_entry(static_cast<std::uint32_t>(i));
  }
};
static const crc_table_t crc_table_inst;

std::uint32_t state_delta_codec::crc32(const unsigned char *data, std::size_t len) {
  std::uint32_t crc = 0xFFFFFFFFu;
  for (std::size_t i = 0; i < len; ++i)
    crc = crc_table_inst.t[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  return crc ^ 0xFFFFFFFFu;
}

// Path-unaware encode: falls back to raw (no baseline).
std::vector<unsigned char> state_delta_codec::encode(const std::vector<unsigned char> &in) const {
  std::vector<unsigned char> out;
  out.reserve(1 + in.size());
  out.push_back(MAGIC_RAW);
  out.insert(out.end(), in.begin(), in.end());
  return out;
}

bool state_delta_codec::decode(const std::vector<unsigned char> &in,
                               std::vector<unsigned char> &out) const {
  if (in.empty()) {
    out.clear();
    return false;
  }
  if (in[0] == MAGIC_RAW) {
    out.assign(in.begin() + 1, in.end());
    return true;
  }
  // Cannot decode a delta without a path (no baseline lookup).
  out.clear();
  return false;
}

static void write_u64_le(std::vector<unsigned char> &buf, std::uint64_t v) {
  for (int i = 0; i < 8; ++i) {
    buf.push_back(static_cast<unsigned char>(v & 0xFF));
    v >>= 8;
  }
}

static std::uint64_t read_u64_le(const unsigned char *p) {
  std::uint64_t v = 0;
  for (int i = 7; i >= 0; --i)
    v = (v << 8) | p[i];
  return v;
}

static void write_u32_le(std::vector<unsigned char> &buf, std::uint32_t v) {
  buf.push_back(static_cast<unsigned char>(v & 0xFF));
  buf.push_back(static_cast<unsigned char>((v >> 8) & 0xFF));
  buf.push_back(static_cast<unsigned char>((v >> 16) & 0xFF));
  buf.push_back(static_cast<unsigned char>((v >> 24) & 0xFF));
}

static std::uint32_t read_u32_le(const unsigned char *p) {
  return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
         (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

std::vector<unsigned char> state_delta_codec::encode(const std::string &path,
                                                     const std::vector<unsigned char> &in) {
  std::vector<unsigned char> baseline;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    auto it = _baselines.find(path);
    if (it != _baselines.end())
      baseline = it->second;
  }

  if (baseline.empty()) {
    // No baseline: emit raw fallback and store as new baseline.
    std::lock_guard<std::mutex> lk(_mutex);
    _baselines[path] = in;

    std::vector<unsigned char> out;
    out.reserve(1 + in.size());
    out.push_back(MAGIC_RAW);
    out.insert(out.end(), in.begin(), in.end());
    return out;
  }

  // XOR delta against baseline.
  std::uint32_t base_crc = crc32(baseline.data(), baseline.size());
  std::size_t patch_len = std::max(in.size(), baseline.size());

  std::vector<unsigned char> out;
  // Header: magic(1) + orig_len(8) + base_crc(4) + patch(patch_len)
  out.reserve(13 + patch_len);
  out.push_back(MAGIC_DELTA);
  write_u64_le(out, static_cast<std::uint64_t>(in.size()));
  write_u32_le(out, base_crc);

  for (std::size_t i = 0; i < patch_len; ++i) {
    unsigned char a = (i < in.size()) ? in[i] : 0;
    unsigned char b = (i < baseline.size()) ? baseline[i] : 0;
    out.push_back(a ^ b);
  }

  // Update baseline.
  {
    std::lock_guard<std::mutex> lk(_mutex);
    _baselines[path] = in;
  }

  return out;
}

bool state_delta_codec::decode(const std::string &path, const std::vector<unsigned char> &in,
                               std::vector<unsigned char> &out) {
  if (in.empty()) {
    out.clear();
    return false;
  }

  if (in[0] == MAGIC_RAW) {
    out.assign(in.begin() + 1, in.end());
    // Update baseline so subsequent deltas can decode.
    std::lock_guard<std::mutex> lk(_mutex);
    _baselines[path] = out;
    return true;
  }

  if (in[0] != MAGIC_DELTA) {
    out.clear();
    return false;
  }

  // Header: magic(1) + orig_len(8) + base_crc(4) = 13 bytes minimum.
  if (in.size() < 13) {
    out.clear();
    return false;
  }

  std::uint64_t orig_len = read_u64_le(&in[1]);
  std::uint32_t expected_crc = read_u32_le(&in[9]);

  std::vector<unsigned char> baseline;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    auto it = _baselines.find(path);
    if (it != _baselines.end())
      baseline = it->second;
  }

  // Verify baseline CRC matches.
  std::uint32_t actual_crc = baseline.empty() ? 0u : crc32(baseline.data(), baseline.size());
  if (actual_crc != expected_crc) {
    out.clear();
    return false;
  }

  std::size_t patch_len = in.size() - 13;
  out.resize(static_cast<std::size_t>(orig_len));

  for (std::size_t i = 0; i < patch_len && i < out.size(); ++i) {
    unsigned char b = (i < baseline.size()) ? baseline[i] : 0;
    out[i] = in[13 + i] ^ b;
  }
  // If orig_len > patch_len, trailing bytes are zero XOR baseline.
  for (std::size_t i = patch_len; i < out.size(); ++i) {
    unsigned char b = (i < baseline.size()) ? baseline[i] : 0;
    out[i] = b;
  }

  // Update baseline.
  {
    std::lock_guard<std::mutex> lk(_mutex);
    _baselines[path] = out;
  }

  return true;
}

void state_delta_codec::set_baseline(const std::string &path,
                                     const std::vector<unsigned char> &baseline) {
  std::lock_guard<std::mutex> lk(_mutex);
  _baselines[path] = baseline;
}

bool state_delta_codec::clear_baseline(const std::string &path) {
  std::lock_guard<std::mutex> lk(_mutex);
  return _baselines.erase(path) > 0;
}

void state_delta_codec::clear_all_baselines() {
  std::lock_guard<std::mutex> lk(_mutex);
  _baselines.clear();
}

std::size_t state_delta_codec::baseline_count() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _baselines.size();
}

} // namespace cvc
