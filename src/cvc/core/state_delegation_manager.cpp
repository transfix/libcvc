/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <chrono>
#include <cvc/core/state_delegation_manager.h>
#include <utility>

namespace cvc {

namespace {

std::uint64_t default_steady_now_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count());
}

} // namespace

state_delegation_manager::state_delegation_manager(std::string local_cluster_id, clock_fn clock)
    : _local_cluster_id(std::move(local_cluster_id)),
      _clock(clock ? std::move(clock) : &default_steady_now_ns) {}

void state_delegation_manager::set_clock(clock_fn clock) {
  std::lock_guard<std::mutex> lk(_clock_mutex);
  _clock = clock ? std::move(clock) : &default_steady_now_ns;
}

std::uint64_t state_delegation_manager::now_ns() const {
  clock_fn fn;
  {
    std::lock_guard<std::mutex> lk(_clock_mutex);
    fn = _clock;
  }
  return fn();
}

void state_delegation_manager::delegate(const std::string &path_prefix,
                                        const std::string &cluster_id, const std::string &endpoint,
                                        std::uint64_t lease_duration_ns) {
  const std::uint64_t expires_at = lease_duration_ns == 0 ? 0u : now_ns() + lease_duration_ns;
  _authority.delegate(path_prefix, cluster_id, endpoint, expires_at);
}

bool state_delegation_manager::renew(const std::string &path_prefix,
                                     std::uint64_t lease_duration_ns) {
  if (!_authority.has_exact(path_prefix)) {
    return false;
  }
  // Re-resolve to get the current cluster/endpoint, then reinsert
  // with a fresh expiry. resolve() returns the longest-prefix match
  // for the path, so use the prefix itself as the "path" so we get
  // the exact entry back.
  auto cur = _authority.resolve(path_prefix);
  if (!cur.valid) {
    return false;
  }
  const std::uint64_t expires_at = lease_duration_ns == 0 ? 0u : now_ns() + lease_duration_ns;
  _authority.delegate(path_prefix, cur.cluster_id, cur.endpoint, expires_at);
  return true;
}

bool state_delegation_manager::revoke(const std::string &path_prefix) {
  return _authority.revoke(path_prefix);
}

state_delegation_manager::route_decision
state_delegation_manager::route(const std::string &path) const {
  route_decision out;
  auto auth = _authority.resolve(path);
  if (!auth.valid) {
    out.kind = route_kind::local;
    return out;
  }
  out.cluster_id = auth.cluster_id;
  out.endpoint = auth.endpoint;
  out.expires_at_ns = auth.expires_at_ns;

  // Recover the matched prefix by re-walking the authority map's
  // snapshot for the longest prefix that covers this path.
  // (The authority map does not currently expose this, so we
  // approximate from the snapshot.)
  auto snap = _authority.snapshot();
  for (const auto &kv : snap) {
    const std::string &p = kv.first;
    if (p.empty()) {
      out.matched_prefix = p;
      break;
    }
    if (path == p ||
        (path.size() > p.size() && path.compare(0, p.size(), p) == 0 && path[p.size()] == '.')) {
      out.matched_prefix = p;
      break;
    }
  }

  // If the lease has expired, surface that explicitly. A foreign
  // cluster with an active lease is REMOTE; if the cluster matches
  // ours it is LOCAL regardless of expiry.
  const bool foreign = auth.cluster_id != _local_cluster_id;
  const bool has_expiry = auth.expires_at_ns != 0;
  const bool expired = has_expiry && now_ns() > auth.expires_at_ns;

  if (!foreign) {
    out.kind = route_kind::local;
    return out;
  }
  out.kind = expired ? route_kind::expired : route_kind::remote;
  return out;
}

} // namespace cvc
