/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state.h>
#include <cvc/state_data_hydrator.h>

namespace CVC_NAMESPACE {

state_data_hydrator::state_data_hydrator(state_blob_store &store, state_codec_registry &codecs,
                                         const state_compression_registry *compression)
    : _store(store), _codecs(codecs), _compression(compression) {}

state_data_hydrator::hydration_status
state_data_hydrator::request(const std::string &path, const std::string &manifest_digest,
                             const std::string &type_name, state *target) {
  std::lock_guard<std::mutex> lk(_mutex);

  auto it = _entries.find(path);
  if (it != _entries.end() && it->second.status == hydration_status::ready)
    return hydration_status::ready;

  entry &e = _entries[path];
  e.manifest_digest = manifest_digest;
  e.type_name = type_name;
  e.target = target;
  e.status = hydration_status::pending;

  try_hydrate(path, e);
  return e.status;
}

state_data_hydrator::hydration_status state_data_hydrator::status(const std::string &path) const {
  std::lock_guard<std::mutex> lk(_mutex);
  auto it = _entries.find(path);
  if (it == _entries.end())
    return hydration_status::unknown;
  return it->second.status;
}

bool state_data_hydrator::is_hydrated(const std::string &path) const {
  return status(path) == hydration_status::ready;
}

state_data_hydrator::hydration_status
state_data_hydrator::wait(const std::string &path, std::chrono::milliseconds timeout) const {
  std::unique_lock<std::mutex> lk(_mutex);
  auto pred = [&]() {
    auto it = _entries.find(path);
    return it == _entries.end() || it->second.status == hydration_status::ready ||
           it->second.status == hydration_status::failed;
  };

  if (timeout.count() == 0) {
    _cv.wait(lk, pred);
  } else {
    if (!_cv.wait_for(lk, timeout, pred))
      return hydration_status::pending;
  }

  auto it = _entries.find(path);
  if (it == _entries.end())
    return hydration_status::unknown;
  return it->second.status;
}

void state_data_hydrator::on_hydrated(on_hydrated_func callback) {
  std::lock_guard<std::mutex> lk(_mutex);
  _callbacks.push_back(std::move(callback));
}

bool state_data_hydrator::cancel(const std::string &path) {
  std::lock_guard<std::mutex> lk(_mutex);
  return _entries.erase(path) > 0;
}

std::size_t state_data_hydrator::retry_pending() {
  std::lock_guard<std::mutex> lk(_mutex);
  std::size_t retried = 0;
  for (auto &kv : _entries) {
    if (kv.second.status == hydration_status::pending) {
      try_hydrate(kv.first, kv.second);
      ++retried;
    }
  }
  return retried;
}

std::size_t state_data_hydrator::pending_count() const {
  std::lock_guard<std::mutex> lk(_mutex);
  std::size_t n = 0;
  for (const auto &kv : _entries)
    if (kv.second.status == hydration_status::pending)
      ++n;
  return n;
}

void state_data_hydrator::try_hydrate(const std::string &path, entry &e) {
  // Must hold _mutex when called.

  state_chunked_blob_reader reader(_store, _compression);

  // 1. Load manifest.
  state_chunk_manifest manifest;
  if (!reader.load_manifest(e.manifest_digest, manifest)) {
    // Manifest not in local store — try fetching it.
    if (_transport) {
      bool fetched = _transport->fetch_chunk(
          e.manifest_digest,
          [this](const std::string & /*digest*/, const std::vector<unsigned char> &bytes) {
            _store.put(bytes);
            _total_chunks_fetched.fetch_add(1, std::memory_order_relaxed);
          });
      if (!fetched) {
        e.status = hydration_status::pending;
        return;
      }
      if (!reader.load_manifest(e.manifest_digest, manifest)) {
        e.status = hydration_status::failed;
        _total_failed.fetch_add(1, std::memory_order_relaxed);
        notify(path, e.status);
        return;
      }
    } else {
      e.status = hydration_status::pending;
      return;
    }
  }

  // 2. Check for missing chunks and fetch them.
  auto missing = reader.missing_chunks(manifest);
  if (!missing.empty()) {
    if (_transport) {
      std::size_t fetched = _transport->fetch_chunks(
          missing, [this](const std::string & /*digest*/, const std::vector<unsigned char> &bytes) {
            _store.put(bytes);
            _total_chunks_fetched.fetch_add(1, std::memory_order_relaxed);
          });
      (void)fetched;
      // Recheck.
      missing = reader.missing_chunks(manifest);
    }
    if (!missing.empty()) {
      e.status = hydration_status::pending;
      return;
    }
  }

  // 3. Reassemble blob.
  std::vector<unsigned char> payload;
  if (!reader.get(manifest, payload)) {
    e.status = hydration_status::failed;
    _total_failed.fetch_add(1, std::memory_order_relaxed);
    notify(path, e.status);
    return;
  }

  // 4. Decode via codec registry.
  if (!e.type_name.empty() && _codecs.has(e.type_name)) {
    try {
      boost::any decoded = _codecs.decode(e.type_name, payload);
      if (e.target) {
        e.target->data(decoded);
      }
    } catch (...) {
      e.status = hydration_status::failed;
      _total_failed.fetch_add(1, std::memory_order_relaxed);
      notify(path, e.status);
      return;
    }
  }

  // 5. Mark ready.
  e.status = hydration_status::ready;
  _total_hydrated.fetch_add(1, std::memory_order_relaxed);
  notify(path, e.status);
}

void state_data_hydrator::notify(const std::string &path, hydration_status status) {
  // Must hold _mutex when called. Copy callbacks to call without lock.
  auto cbs = _callbacks;
  // Release lock would be needed if we didn't hold it for the caller.
  // Since try_hydrate holds the lock, we fire callbacks under lock
  // for simplicity. In production, a signal-queue pattern would be
  // better, but this matches the existing cvc::state callback model.
  _cv.notify_all();
  for (auto &cb : cbs) {
    if (cb)
      cb(path, status);
  }
}

} // namespace CVC_NAMESPACE
