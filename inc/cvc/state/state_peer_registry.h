/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_PEER_REGISTRY_H__
#define __CVC_STATE_PEER_REGISTRY_H__

#include <cstdint>
#include <cvc/core/namespace.h>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {

// ----------------
// cvc::state_delivery_filter
// ----------------
// A per-peer DENY predicate layered on top of the subscription prefixes:
// a delivery to a peer passes only when its prefixes match AND its filter
// (if any) delivers the path. Filters are immutable once installed (swap a
// new one in with set_filter), are evaluated outside the registry lock, and
// must never call back into the registry or a transport; deliver() may run
// on any transport thread at once.
class state_delivery_filter {
public:
  virtual ~state_delivery_filter() = default;
  virtual bool deliver(const std::string &path) const noexcept = 0;
};

// ----------------
// cvc::state_peer_registry
// ----------------
// Phase 5 (Multi-Node Cluster Semantics).
//
// Tracks remote peers in the cluster topology and their declared
// path-prefix subscription set. The registry answers
// "should I deliver a mutation/message at path P to peer N?" so a
// transport can avoid all-to-all broadcast in many-node clusters.
//
// Subscription matching:
//   A peer registered with no prefixes (empty set) is treated as
//   match-all (back-compat with phases < 5). Otherwise a peer with
//   prefixes [a, b.c] receives delivery for paths "a", "a.x.y",
//   "b.c", "b.c.d" but not "b" or "z".
//
// Delivery filter:
//   set_filter() layers a state_delivery_filter over a known peer's
//   prefixes (both must pass). add_peer on an existing id keeps it;
//   remove_peer drops it. A filter the registry lets go of (replaced,
//   removed or cleared) is released outside the lock, and one that a
//   concurrent should_deliver() is still evaluating stays alive until
//   that call returns.
//
// Membership:
//   add_peer/remove_peer manage entries. note_seen() bumps a
//   liveness timestamp; callers may use it for heartbeat-driven
//   eviction, but the registry itself does not auto-expire.
//
// Threading:
//   All public methods are safe to call from multiple threads.
//
class state_peer_registry {
public:
  struct peer {
    std::string node_id;
    std::string cluster_id;
    std::string endpoint;
    std::vector<std::string> subscriptions;
    std::uint64_t last_seen_ns = 0;
    std::uint64_t mutations_delivered = 0;
    std::uint64_t messages_delivered = 0;
    std::uint64_t deliveries_filtered = 0;
    std::shared_ptr<const state_delivery_filter> filter;
  };

  // One peer's delivery rule, copied out under a single lock so a batch
  // path (a snapshot frame of many entries) does not lock per entry.
  struct delivery_view {
    bool known = false; // false: an unknown peer, which delivers everything
    std::vector<std::string> subscriptions;
    std::shared_ptr<const state_delivery_filter> filter;
    // Same answer as should_deliver() at the moment the view was taken.
    bool deliver(const std::string &path) const noexcept;
    // True when this view can deny anything (known with prefixes or a filter).
    bool filtered() const noexcept;
  };

  state_peer_registry();

  state_peer_registry(const state_peer_registry &) = delete;
  state_peer_registry &operator=(const state_peer_registry &) = delete;

  // Insert or replace a peer entry. The cluster_id and endpoint
  // overwrite prior values; the subscription set replaces prior
  // subscriptions. An existing peer's delivery filter is kept.
  void add_peer(std::string node_id, std::string cluster_id, std::string endpoint = std::string(),
                std::vector<std::string> subscriptions = {});

  // Remove a peer. Returns true if the peer existed.
  bool remove_peer(const std::string &node_id);

  // Replace the subscription set for an existing peer. Returns
  // false if the peer is unknown.
  bool set_subscriptions(const std::string &node_id, std::vector<std::string> subscriptions);

  // Returns true if the registry knows this peer.
  bool has_peer(const std::string &node_id) const;

  // Install (or, with nullptr, clear) a known peer's delivery filter.
  // Returns false if the peer is unknown.
  bool set_filter(const std::string &node_id, std::shared_ptr<const state_delivery_filter> f);

  // The peer's delivery rule as of now (known=false for an unknown peer).
  delivery_view view_of(const std::string &node_id) const;

  // Returns true iff a delivery to `node_id` for `path` would pass
  // the subscription filter and the peer's delivery filter. Unknown
  // peers return true (a transport that has not registered
  // subscriptions for an active connection should still deliver —
  // this is the back-compat default).
  bool should_deliver(const std::string &node_id, const std::string &path) const;

  // Heartbeat: update last_seen_ns to `now_ns` (caller-provided
  // steady_clock-relative timestamp).
  bool note_seen(const std::string &node_id, std::uint64_t now_ns);

  // Per-peer counter bumps for observability.
  void note_mutation_delivered(const std::string &node_id);
  void note_message_delivered(const std::string &node_id);
  void note_delivery_filtered(const std::string &node_id);

  std::size_t size() const;
  std::vector<peer> snapshot() const;
  void clear();

  // Match a single path against a set of dot-segmented prefixes.
  // Empty prefix matches any path. Otherwise prefix matches when
  // path == prefix or path starts with prefix + '.'.
  static bool prefix_matches(const std::string &prefix, const std::string &path) noexcept;

  // True if any element of `prefixes` matches `path` under
  // prefix_matches(). An empty prefix list returns true (match-all).
  static bool any_prefix_matches(const std::vector<std::string> &prefixes,
                                 const std::string &path) noexcept;

private:
  mutable std::mutex _mu;
  std::unordered_map<std::string, peer> _peers;
};

} // namespace cvc

#endif // __CVC_STATE_PEER_REGISTRY_H__
