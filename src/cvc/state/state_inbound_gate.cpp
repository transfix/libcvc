/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/state/state_inbound_gate.h>
#include <mutex>
#include <unordered_map>

namespace cvc {

namespace {

// The transports' clock: steady-clock nanoseconds (their link stamps and last_rx_ns).
std::int64_t steady_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// One node id's holder (file-local, so the map below exports nothing).
struct holding {
  const void *who = nullptr;
  const std::atomic<std::int64_t> *last_rx_ns = nullptr;
  std::atomic<bool> *displaced = nullptr;
};

} // namespace

struct state_inbound_gate::table {
  mutable std::mutex mu;
  std::unordered_map<std::string, holding> holders;
  std::int64_t takeover_ns = 6'000'000'000;
};

state_inbound_gate::state_inbound_gate() : _table(std::make_unique<table>()) {}

state_inbound_gate::~state_inbound_gate() = default;

bool state_inbound_gate::bind(const std::string &node_id, const void *who,
                              const std::atomic<std::int64_t> *last_rx_ns,
                              std::atomic<bool> *displaced) {
  if (node_id.empty() || who == nullptr)
    return false;
  const holding claim{who, last_rx_ns, displaced};
  std::lock_guard<std::mutex> lk(_table->mu);
  auto [it, fresh] = _table->holders.try_emplace(node_id, claim);
  if (fresh || it->second.who == who)
    return true;
  const holding &held = it->second;
  if (held.last_rx_ns == nullptr ||
      steady_now_ns() - held.last_rx_ns->load() <= _table->takeover_ns)
    return false;
  if (held.displaced != nullptr)
    held.displaced->store(true);
  it->second = claim;
  _takeovers.fetch_add(1, std::memory_order_relaxed);
  return true;
}

bool state_inbound_gate::release(const std::string &node_id, const void *who) {
  std::lock_guard<std::mutex> lk(_table->mu);
  auto it = _table->holders.find(node_id);
  if (it == _table->holders.end() || it->second.who != who)
    return false;
  _table->holders.erase(it);
  return true;
}

const void *state_inbound_gate::holder(const std::string &node_id) const {
  std::lock_guard<std::mutex> lk(_table->mu);
  auto it = _table->holders.find(node_id);
  return it == _table->holders.end() ? nullptr : it->second.who;
}

void state_inbound_gate::set_takeover_after(std::chrono::milliseconds silence) {
  // A day at most, so the nanosecond count cannot overflow.
  const auto capped = (std::min)(silence, std::chrono::milliseconds(std::chrono::hours(24)));
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(capped).count();
  std::lock_guard<std::mutex> lk(_table->mu);
  _table->takeover_ns = ns > 0 ? static_cast<std::int64_t>(ns) : 0;
}

std::chrono::milliseconds state_inbound_gate::takeover_after() const {
  std::lock_guard<std::mutex> lk(_table->mu);
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::nanoseconds(_table->takeover_ns));
}

bool state_inbound_gate::admit(const std::string &, const state_mutation &) const { return true; }

bool state_inbound_gate::admit(const std::string &, const state_message &) const { return true; }

} // namespace cvc
