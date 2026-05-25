/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_MESSAGE_BUS_H__
#define __CVC_STATE_MESSAGE_BUS_H__

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cvc/namespace.h>
#include <cvc/state_message.h>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

namespace cvc {

// ----------------
// cvc::state_message_bus
// ----------------
// Local subscriber registry + (origin, message_id) dedup ledger
// for cvc::state_message. The bus does NOT touch the change
// journal or vector clocks; it is a pure transient pub/sub
// surface that lives next to the state tree.
//
// Subscriber matching:
//   A subscriber registered with prefix "" matches every message.
//   A subscriber registered with prefix "p" matches messages whose
//   path is "p" or starts with "p." (dot-segment boundary).
//
// Dedup:
//   admit() admits a message at most once per (origin_node_id,
//   message_id). Subsequent admits with the same key return false
//   without invoking subscribers. The dedup ledger is bounded by
//   set_dedup_capacity(); when full the oldest entry is evicted.
//
// Backpressure:
//   admit_nonblocking() is a thin wrapper that bumps the
//   total_dropped() counter when the caller indicates the message
//   was dropped at a transport layer; this lets tests assert on
//   drop-under-pressure without coupling to a specific transport.
//
// Threading:
//   All public methods are safe to call from multiple threads.
//   subscriber callbacks fire synchronously inside admit().
//
class state_message_bus {
public:
  using subscriber_fn = std::function<void(const state_message &)>;
  using subscription_id = std::uint64_t;

  state_message_bus();

  state_message_bus(const state_message_bus &) = delete;
  state_message_bus &operator=(const state_message_bus &) = delete;

  // Register a subscriber for `path_prefix`. Returns a non-zero id
  // that can be passed to unsubscribe().
  subscription_id subscribe(std::string path_prefix, subscriber_fn cb);

  // Remove a previously-registered subscriber. Returns true if a
  // subscriber with that id existed.
  bool unsubscribe(subscription_id id);

  // Admit a message. Returns true if the (origin, id) pair was new
  // and matching subscribers were invoked, false if it was a
  // duplicate. Empty origin_node_id and message_id are treated as
  // dedup-bypass (each admit fires).
  bool admit(const state_message &m);

  // Record a transport-level drop without invoking subscribers.
  // Increments total_dropped().
  void note_dropped();

  // Configure the maximum number of (origin, id) pairs retained for
  // dedup. 0 disables eviction (unbounded). Default 8192.
  void set_dedup_capacity(std::size_t cap);
  std::size_t dedup_capacity() const noexcept;

  std::size_t subscriber_count() const;
  std::size_t dedup_size() const;

  std::uint64_t total_admitted() const noexcept { return _admitted.load(); }
  std::uint64_t total_duplicates() const noexcept { return _dups.load(); }
  std::uint64_t total_dispatched() const noexcept { return _dispatched.load(); }
  std::uint64_t total_dropped() const noexcept { return _dropped.load(); }

  // Test helper: returns true iff prefix matches path under the
  // dot-segment rule (exposed so tests can pin the matching
  // semantics).
  static bool prefix_matches(const std::string &prefix, const std::string &path) noexcept;

private:
  struct subscriber {
    subscription_id id;
    std::string prefix;
    subscriber_fn fn;
  };

  mutable std::mutex _mu;
  std::vector<subscriber> _subs;
  subscription_id _next_id = 1;

  std::deque<std::string> _seen_order;
  std::unordered_set<std::string> _seen;
  std::size_t _dedup_cap = 8192;

  std::atomic<std::uint64_t> _admitted{0};
  std::atomic<std::uint64_t> _dups{0};
  std::atomic<std::uint64_t> _dispatched{0};
  std::atomic<std::uint64_t> _dropped{0};
};

} // namespace cvc

#endif // __CVC_STATE_MESSAGE_BUS_H__
