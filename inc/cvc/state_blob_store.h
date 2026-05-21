/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_BLOB_STORE_H__
#define __CVC_STATE_BLOB_STORE_H__

#include <cstdint>
#include <cvc/namespace.h>
#include <cvc/state_change_journal.h>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace CVC_NAMESPACE {

// ----------------
// cvc::state_blob_store
// ----------------
// Purpose:
//   Content-addressable storage for large payloads referenced by
//   state_blob_ref. Blob identity is the SHA-256 hex digest of the
//   contents. This is an abstract base class; the default
//   implementation is `memory_state_blob_store` which keeps bytes
//   in process memory. Later phases may add filesystem, object
//   storage, and zero-copy variants.
//
// Threading:
//   Implementations must be thread-safe. put/get/has/erase may be
//   called concurrently from multiple threads.
//
// Codec field:
//   The optional `codec` string on state_blob_ref is opaque to the
//   blob store. It is metadata for the codec registry and the
//   consuming peer; the store treats blobs as raw bytes.
//
class state_blob_store {
public:
  virtual ~state_blob_store() = default;

  // Store `bytes` and return a state_blob_ref describing it. If a
  // blob with the same content already exists the store is a no-op
  // and the existing ref is returned (with refcount semantics
  // implementation-defined; the in-memory default uses single-copy
  // dedup, no refcount).
  virtual state_blob_ref put(const std::vector<unsigned char> &bytes,
                             const std::string &codec = std::string()) = 0;

  // Retrieve the bytes for `digest`. Returns true on success and
  // writes the bytes to `out`. Returns false if not present.
  virtual bool get(const std::string &digest, std::vector<unsigned char> &out) const = 0;

  // True if the digest is known.
  virtual bool has(const std::string &digest) const = 0;

  // Remove a blob. Returns true if it was removed.
  virtual bool erase(const std::string &digest) = 0;

  // Number of distinct blobs.
  virtual std::size_t size() const = 0;

  // Sum of all blob byte sizes.
  virtual std::uint64_t bytes_stored() const = 0;

  // Snapshot of all known digests. Order is unspecified. The
  // default implementation is provided so existing concrete stores
  // need not override it; callers that want efficient enumeration
  // (e.g. for garbage collection) should provide a specialization.
  virtual std::vector<std::string> digests() const = 0;
};

// Compute the SHA-256 hex digest (lowercase) of `bytes`.
std::string sha256_hex(const std::vector<unsigned char> &bytes);
std::string sha256_hex(const unsigned char *data, std::size_t len);

// ----------------
// cvc::memory_state_blob_store
// ----------------
// Purpose:
//   In-process content-addressed blob store. All bytes live in RAM.
//   Suitable for tests, small payloads, and as a first-tier cache.
class memory_state_blob_store : public state_blob_store {
public:
  memory_state_blob_store();
  ~memory_state_blob_store() override;

  state_blob_ref put(const std::vector<unsigned char> &bytes,
                     const std::string &codec = std::string()) override;
  bool get(const std::string &digest, std::vector<unsigned char> &out) const override;
  bool has(const std::string &digest) const override;
  bool erase(const std::string &digest) override;
  std::size_t size() const override;
  std::uint64_t bytes_stored() const override;
  std::vector<std::string> digests() const override;

private:
  mutable std::mutex _mutex;
  std::unordered_map<std::string, std::vector<unsigned char>> _blobs;
  std::uint64_t _bytes_stored;
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_BLOB_STORE_H__
