/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_COMPRESSION_REGISTRY_H__
#define __CVC_STATE_COMPRESSION_REGISTRY_H__

#include <cvc/namespace.h>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {

// ----------------
// cvc::state_compression_codec
// ----------------
// Purpose:
//   Byte-stream transform applied to chunks before they hit the
//   blob store and to bytes pulled off the wire before they are
//   handed to a value codec. Distinct from state_codec_registry,
//   which encodes typed values to bytes; compression sits one
//   layer below.
//
// Threading:
//   Implementations must be thread-safe; encode/decode may be
//   called concurrently from multiple threads.
//
// Wire id:
//   `id()` is the string that travels in state_blob_ref::codec and
//   state_chunk_manifest::codec. Identifiers are opaque. Built-in
//   ids: "raw" (identity), "rle" (byte run-length).
//
class state_compression_codec {
public:
  virtual ~state_compression_codec() = default;
  virtual std::string id() const = 0;
  virtual std::vector<unsigned char> encode(const std::vector<unsigned char> &in) const = 0;
  // Returns true on success; on failure `out` is cleared.
  virtual bool decode(const std::vector<unsigned char> &in,
                      std::vector<unsigned char> &out) const = 0;
};

// ----------------
// cvc::state_compression_registry
// ----------------
// Purpose:
//   Lookup table for compression codecs by wire id. Created with
//   the built-ins ("raw", "rle") already registered. Application
//   code may install additional codecs (e.g. a zstd-backed one)
//   at startup.
//
// Threading: thread-safe.
//
class state_compression_registry {
public:
  state_compression_registry();

  // Register `codec`, replacing any previous binding for its id.
  void register_codec(std::shared_ptr<state_compression_codec> codec);

  // Look up by id. Returns nullptr if absent.
  std::shared_ptr<state_compression_codec> get(const std::string &id) const;

  bool has(const std::string &id) const;

  // Snapshot of registered ids in unspecified order.
  std::vector<std::string> ids() const;

  std::size_t size() const;

  // Convenience: encode/decode by id. encode returns the input
  // unchanged for unknown ids when `strict` is false; throws
  // std::runtime_error when `strict` is true. decode mirrors this.
  std::vector<unsigned char> encode(const std::string &id, const std::vector<unsigned char> &in,
                                    bool strict = true) const;
  bool decode(const std::string &id, const std::vector<unsigned char> &in,
              std::vector<unsigned char> &out, bool strict = true) const;

  // Process-wide default registry (lazily initialized with the
  // built-ins). Tests and library callers that want a shared
  // registry without threading one through every API should call
  // this.
  static state_compression_registry &shared();

private:
  mutable std::mutex _mutex;
  std::unordered_map<std::string, std::shared_ptr<state_compression_codec>> _codecs;
};

// Built-in codecs (exposed for direct use).
class state_raw_compression_codec : public state_compression_codec {
public:
  std::string id() const override { return "raw"; }
  std::vector<unsigned char> encode(const std::vector<unsigned char> &in) const override {
    return in;
  }
  bool decode(const std::vector<unsigned char> &in,
              std::vector<unsigned char> &out) const override {
    out = in;
    return true;
  }
};

// Simple byte-level run-length encoding. Format:
//   [count:uint8][byte:uint8] repeated, count in [1,255].
// Not a strong compressor, but it is dependency-free, exercises
// the codec plumbing end-to-end, and demonstrably shrinks runs.
class state_rle_compression_codec : public state_compression_codec {
public:
  std::string id() const override { return "rle"; }
  std::vector<unsigned char> encode(const std::vector<unsigned char> &in) const override;
  bool decode(const std::vector<unsigned char> &in, std::vector<unsigned char> &out) const override;
};

// zstd compression codec (production-grade). Requires libzstd.
class state_zstd_compression_codec : public state_compression_codec {
public:
  // compression_level: 1 (fastest) to 22 (best ratio); default 3.
  explicit state_zstd_compression_codec(int compression_level = 3);
  std::string id() const override { return "zstd"; }
  std::vector<unsigned char> encode(const std::vector<unsigned char> &in) const override;
  bool decode(const std::vector<unsigned char> &in, std::vector<unsigned char> &out) const override;

private:
  int _level;
};

} // namespace cvc

#endif // __CVC_STATE_COMPRESSION_REGISTRY_H__
