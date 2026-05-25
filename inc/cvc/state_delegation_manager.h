/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_DELEGATION_MANAGER_H__
#define __CVC_STATE_DELEGATION_MANAGER_H__

#include <cstdint>
#include <cvc/namespace.h>
#include <cvc/state_authority_map.h>
#include <functional>
#include <mutex>
#include <string>

namespace cvc {

// ----------------
// cvc::state_delegation_manager (Phase 6)
// ----------------
// Purpose:
//   Lease-aware routing layer on top of state_authority_map.
//
//   The authority map records longest-prefix delegations
//   (path-prefix -> cluster_id, endpoint, expires_at_ns) but does
//   not interpret expiry. The delegation manager:
//
//     - exposes a `route(path)` API that classifies a path as
//       LOCAL, REMOTE (active lease to another cluster), or EXPIRED
//       (lease past `now()`);
//     - tracks lease lifecycle (delegate/renew/revoke);
//     - injects a clock so tests can advance time deterministically.
//
//   `local_cluster_id` is the cluster this shard belongs to. Any
//   delegation whose authority cluster matches `local_cluster_id`
//   is classified LOCAL even if it has an explicit entry; a path
//   with no covering entry also classifies LOCAL (root authority
//   defaults to local).
//
// Threading:
//   Thread-safe. Internal mutex protects the wrapped authority
//   map. The clock function is captured at construction; replacing
//   it via set_clock() is also thread-safe.
//
class state_delegation_manager {
public:
  enum class route_kind {
    local,   // path is owned by this cluster
    remote,  // delegated to another cluster, lease is valid
    expired, // delegated to another cluster, lease has expired
  };

  struct route_decision {
    route_kind kind = route_kind::local;
    std::string cluster_id;     // foreign cluster (empty when local)
    std::string endpoint;       // optional transport hint
    std::string matched_prefix; // empty when no delegation matched
    std::uint64_t expires_at_ns = 0;
  };

  // clock_fn must return a steady-clock-like nanosecond timestamp.
  using clock_fn = std::function<std::uint64_t()>;

  explicit state_delegation_manager(std::string local_cluster_id, clock_fn clock = nullptr);

  const std::string &local_cluster_id() const noexcept { return _local_cluster_id; }

  // Replace the underlying clock. Useful for deterministic tests.
  void set_clock(clock_fn clock);

  // Read-only access to the wrapped authority map.
  const state_authority_map &authority() const noexcept { return _authority; }
  state_authority_map &authority() noexcept { return _authority; }

  // Add or replace a delegation. `lease_duration_ns == 0` means no
  // expiry (infinite lease). Otherwise expires_at_ns = now() +
  // lease_duration_ns.
  void delegate(const std::string &path_prefix, const std::string &cluster_id,
                const std::string &endpoint = std::string(), std::uint64_t lease_duration_ns = 0);

  // Bump the lease for an existing delegation. Returns false if no
  // entry exists at `path_prefix`. `lease_duration_ns == 0` makes
  // the lease infinite.
  bool renew(const std::string &path_prefix, std::uint64_t lease_duration_ns);

  // Remove the exact delegation. Returns true if an entry existed.
  bool revoke(const std::string &path_prefix);

  // Longest-prefix lookup with lease evaluation. Always returns a
  // decision; never throws.
  route_decision route(const std::string &path) const;

  // Convenience: returns true iff route(path).kind == local.
  bool is_local(const std::string &path) const { return route(path).kind == route_kind::local; }

private:
  std::uint64_t now_ns() const;

  std::string _local_cluster_id;
  mutable std::mutex _clock_mutex;
  clock_fn _clock;
  mutable state_authority_map _authority;
};

} // namespace cvc

#endif // __CVC_STATE_DELEGATION_MANAGER_H__
