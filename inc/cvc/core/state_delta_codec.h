/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_DELTA_CODEC_H__
#define __CVC_STATE_DELTA_CODEC_H__

#include <cstdint>
#include <cvc/namespace.h>
#include <cvc/state_compression_registry.h>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {

// ----------------
// cvc::state_delta_codec
// ----------------
// Purpose:
//   Compression codec that produces XOR-based delta patches from a
//   known baseline. When a caller provides the same path's previous
//   blob bytes as the baseline, the delta contains only the changed
//   bytes (XOR with previous), which compresses extremely well with
//   a follow-up RLE or zstd pass.
//
//   The wire format:
//     [0] magic byte 0xD1 (delta v1)
//     [1..8] uint64_le: original length
//     [9..12] uint32_le: baseline digest CRC32 (identifies which
//             baseline was used; decoders with a mismatched baseline
//             reject the patch)
//     [13..N] XOR patch bytes (zero = unchanged)
//
//   When no baseline is stored for a path, encode() emits a raw
//   copy fallback prefixed by magic byte 0xD0, so the codec is
//   always safe to use even on first-write paths:
//     [0] magic byte 0xD0 (no-delta fallback)
//     [1..N] raw bytes
//
// Thread safety:
//   All methods are thread-safe. The baseline table is guarded by
//   a mutex. Concurrent encode()/decode() on different paths is
//   lock-free after the initial lookup.
//
class state_delta_codec : public state_compression_codec {
public:
  static constexpr std::uint8_t MAGIC_DELTA = 0xD1;
  static constexpr std::uint8_t MAGIC_RAW = 0xD0;

  std::string id() const override { return "delta"; }

  // Encode `in` against the stored baseline for `path`. After
  // encoding, `in` becomes the new baseline for `path`.
  std::vector<unsigned char> encode(const std::vector<unsigned char> &in) const override;
  bool decode(const std::vector<unsigned char> &in, std::vector<unsigned char> &out) const override;

  // Path-aware encode/decode. `path` selects the baseline.
  std::vector<unsigned char> encode(const std::string &path, const std::vector<unsigned char> &in);
  bool decode(const std::string &path, const std::vector<unsigned char> &in,
              std::vector<unsigned char> &out);

  // Manually set the baseline for `path`.
  void set_baseline(const std::string &path, const std::vector<unsigned char> &baseline);

  // Remove the baseline for `path`. Returns true if a baseline
  // existed.
  bool clear_baseline(const std::string &path);

  // Remove all baselines.
  void clear_all_baselines();

  // Number of paths with stored baselines.
  std::size_t baseline_count() const;

  // CRC32 of a byte buffer (used for baseline identification).
  static std::uint32_t crc32(const unsigned char *data, std::size_t len);

private:
  mutable std::mutex _mutex;
  std::unordered_map<std::string, std::vector<unsigned char>> _baselines;
};

} // namespace cvc

#endif // __CVC_STATE_DELTA_CODEC_H__
