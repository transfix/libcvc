/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_DATA_HYDRATOR_H__
#define __CVC_STATE_DATA_HYDRATOR_H__

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cvc/namespace.h>
#include <cvc/state_blob_store.h>
#include <cvc/state_chunked_blob.h>
#include <cvc/state_codec_registry.h>
#include <cvc/state_compression_registry.h>
#include <cvc/state_transport.h>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {

class state;

// ----------------
// cvc::state_data_hydrator
// ----------------
// Purpose:
//   Provides lazy hydration of blob-referenced data objects in the
//   distributed state tree. When a remote mutation arrives with a
//   blob_ref payload, the hydrator:
//     1. Checks if all chunks are present locally.
//     2. If not, requests missing chunks from the transport.
//     3. Reassembles the blob and decodes via the codec registry.
//     4. Installs the decoded data on the state node.
//     5. Notifies waiters.
//
//   Consumers can call wait_for_data<T>(path, timeout) to block
//   until the data is hydrated, or poll with is_hydrated(path).
//
// Threading:
//   All public methods are thread-safe.
//
class state_data_hydrator {
public:
  enum class hydration_status {
    unknown, // no pending hydration for this path
    pending, // chunks still missing, fetch in progress
    ready,   // all chunks present, data decoded and installed
    failed   // fetch or decode failed
  };

  // Callback signature for hydration completion.
  using on_hydrated_func = std::function<void(const std::string &path, hydration_status status)>;

  state_data_hydrator(state_blob_store &store, state_codec_registry &codecs,
                      const state_compression_registry *compression = nullptr);

  // Set the transport used to fetch missing chunks from remote
  // peers. Without a transport, the hydrator can only work with
  // chunks already in the local store.
  void set_transport(state_transport *t) noexcept { _transport = t; }

  // Request hydration of a blob-referenced payload at `path`.
  // This is called automatically when a shard ingests a remote
  // SET_DATA mutation with a blob payload. It can also be called
  // manually.
  //
  // `manifest_digest` is the blob_ref.digest from the mutation.
  // `type_name` is the mutation's type_name for codec lookup.
  // `target` is the state node where the decoded data will be
  // installed (via target->data(decoded_value)).
  //
  // Returns the current hydration status after the call.
  hydration_status request(const std::string &path, const std::string &manifest_digest,
                           const std::string &type_name, state *target = nullptr);

  // Query the hydration status of a path.
  hydration_status status(const std::string &path) const;

  // Returns true if the blob at `path` has been fully hydrated.
  bool is_hydrated(const std::string &path) const;

  // Block until the blob at `path` is hydrated or the timeout
  // expires. Returns the final status. A zero timeout waits
  // indefinitely.
  hydration_status wait(const std::string &path,
                        std::chrono::milliseconds timeout = std::chrono::milliseconds(0)) const;

  // Register a callback that fires when any path completes
  // hydration (status == ready or failed).
  void on_hydrated(on_hydrated_func callback);

  // Remove a pending entry for `path`. Returns true if it existed.
  bool cancel(const std::string &path);

  // Retry all pending hydrations (e.g. after a transport
  // reconnection). Returns the number of entries retried.
  std::size_t retry_pending();

  // Diagnostics.
  std::size_t pending_count() const;
  std::uint64_t total_hydrated() const noexcept { return _total_hydrated.load(); }
  std::uint64_t total_failed() const noexcept { return _total_failed.load(); }
  std::uint64_t total_chunks_fetched() const noexcept { return _total_chunks_fetched.load(); }

private:
  struct entry {
    std::string manifest_digest;
    std::string type_name;
    state *target = nullptr;
    hydration_status status = hydration_status::pending;
  };

  void try_hydrate(const std::string &path, entry &e);
  void notify(const std::string &path, hydration_status status);

  state_blob_store &_store;
  state_codec_registry &_codecs;
  const state_compression_registry *_compression;
  state_transport *_transport = nullptr;

  mutable std::mutex _mutex;
  mutable std::condition_variable _cv;
  std::unordered_map<std::string, entry> _entries;
  std::vector<on_hydrated_func> _callbacks;

  std::atomic<std::uint64_t> _total_hydrated{0};
  std::atomic<std::uint64_t> _total_failed{0};
  std::atomic<std::uint64_t> _total_chunks_fetched{0};
};

} // namespace cvc

#endif // __CVC_STATE_DATA_HYDRATOR_H__
