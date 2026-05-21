/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_CHUNKED_BLOB_H__
#define __CVC_STATE_CHUNKED_BLOB_H__

#include <cvc/namespace.h>
#include <cvc/state_blob_store.h>
#include <cvc/state_compression_registry.h>

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace CVC_NAMESPACE {

// ----------------
// cvc::state_chunk_manifest
// ----------------
// Purpose:
//   Describes a payload split into N fixed-size chunks. The
//   manifest itself is serialized to bytes and stored in the same
//   blob store; its SHA-256 digest is the addressable handle for
//   the chunked payload.
//
// Wire format (little-endian):
//   magic[4]    = 'C','V','C','M'
//   version     = uint32 (currently 1)
//   chunk_size  = uint32 (nominal chunk size)
//   total_size  = uint64 (sum of chunk_sizes; logical bytes)
//   codec_len   = uint32; codec_bytes[codec_len]
//   content_len = uint32; content_digest[content_len] (hex of full payload)
//   count       = uint32
//   for each chunk:
//     digest_len = uint32; digest_bytes[digest_len]
//     bytes      = uint32   (size of this chunk's stored bytes)
//
struct state_chunk_manifest {
  std::uint32_t version = 1;
  std::uint32_t chunk_size = 0;
  std::uint64_t total_size = 0;
  std::string codec;
  std::string content_digest;          // sha256 hex of full payload
  std::vector<std::string> chunks;     // per-chunk sha256 hex
  std::vector<std::uint32_t> chunk_bytes;

  std::vector<unsigned char> serialize() const;
  static bool parse(const std::vector<unsigned char> &bytes,
                    state_chunk_manifest &out);
};

// ----------------
// cvc::state_chunked_blob_writer
// ----------------
// Purpose:
//   Splits a payload into fixed-size chunks, writes each chunk into
//   the blob store (content-addressed dedup), and returns a manifest
//   blob_ref that names the whole payload. If a compression codec
//   is supplied via put(), each chunk is run through the codec
//   before being stored, and the codec id is recorded on the
//   manifest so the reader can decode on the way out.
//
// Threading:
//   Methods are safe to call from multiple threads against a
//   thread-safe blob_store. Counters are atomic.
//
class state_chunked_blob_writer {
public:
  // Default chunk size: 1 MiB. `compression` is the registry
  // consulted when put() is called with a non-empty codec id; if
  // null, only the empty / "raw" codec is honored.
  explicit state_chunked_blob_writer(
      state_blob_store &store, std::uint32_t chunk_size = 1u << 20,
      const state_compression_registry *compression = nullptr);

  state_chunked_blob_writer(const state_chunked_blob_writer &) = delete;
  state_chunked_blob_writer &
  operator=(const state_chunked_blob_writer &) = delete;

  // Write `bytes` as N chunks into the underlying store, then write
  // the serialized manifest as a blob. Each chunk is compressed
  // with the codec named by `codec` (empty / "raw" = identity).
  // The manifest records `codec` so the reader can decode.
  state_blob_ref put(const std::vector<unsigned char> &bytes,
                     const std::string &codec = std::string());

  std::uint32_t chunk_size() const { return _chunk_size; }
  state_blob_store &store() { return _store; }

  // Counters (cumulative across all put() calls).
  std::uint64_t total_chunks_written() const {
    return _chunks_written.load(std::memory_order_relaxed);
  }
  std::uint64_t total_chunks_dedup() const {
    return _chunks_dedup.load(std::memory_order_relaxed);
  }
  std::uint64_t total_bytes_written() const {
    return _bytes_written.load(std::memory_order_relaxed);
  }
  std::uint64_t total_manifests_written() const {
    return _manifests_written.load(std::memory_order_relaxed);
  }

private:
  state_blob_store &_store;
  std::uint32_t _chunk_size;
  const state_compression_registry *_compression;
  std::atomic<std::uint64_t> _chunks_written{0};
  std::atomic<std::uint64_t> _chunks_dedup{0};
  std::atomic<std::uint64_t> _bytes_written{0};
  std::atomic<std::uint64_t> _manifests_written{0};
};

// ----------------
// cvc::state_chunked_blob_reader
// ----------------
// Purpose:
//   Loads manifests, reassembles payloads, and reports which chunks
//   are missing from the local store (resume API). Resume is the
//   linchpin for transferring large blobs across an unreliable
//   transport: the receiver pulls only chunks it lacks.
//
class state_chunked_blob_reader {
public:
  explicit state_chunked_blob_reader(
      state_blob_store &store,
      const state_compression_registry *compression = nullptr);

  // Load+parse a manifest by digest. Returns false on missing or
  // malformed manifest.
  bool load_manifest(const std::string &manifest_digest,
                     state_chunk_manifest &out) const;

  // Return digests of chunks not present locally, in manifest
  // order.
  std::vector<std::string>
  missing_chunks(const state_chunk_manifest &manifest) const;
  std::vector<std::string>
  missing_chunks(const std::string &manifest_digest) const;

  // Reassemble the full payload referenced by `manifest`. Returns
  // false if any chunk is missing (use missing_chunks to plan a
  // resume). Verifies the reassembled content_digest if non-empty.
  bool get(const state_chunk_manifest &manifest,
           std::vector<unsigned char> &out) const;
  bool get(const std::string &manifest_digest,
           std::vector<unsigned char> &out) const;

  state_blob_store &store() { return _store; }

private:
  state_blob_store &_store;
  const state_compression_registry *_compression;
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_CHUNKED_BLOB_H__
