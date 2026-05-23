/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/state_replica.h>
#include <utility>

namespace CVC_NAMESPACE {

state_replica::state_replica(std::string local_node_id) : _local_node_id(std::move(local_node_id)) {
  // Local node always counts as a peer of itself, alive.
  peer_info self;
  self.node_id = _local_node_id;
  self.last_applied_sequence = 0;
  self.alive = true;
  _peers.emplace(_local_node_id, std::move(self));
  _clock[_local_node_id] = 0;
}

void state_replica::add_peer(const std::string &node_id) {
  std::lock_guard<std::mutex> lk(_mutex);
  if (_peers.find(node_id) != _peers.end())
    return;
  peer_info p;
  p.node_id = node_id;
  p.last_applied_sequence = 0;
  p.alive = true;
  _peers.emplace(node_id, std::move(p));
  if (_clock.find(node_id) == _clock.end())
    _clock[node_id] = 0;
}

bool state_replica::remove_peer(const std::string &node_id) {
  if (node_id == _local_node_id)
    return false;
  std::lock_guard<std::mutex> lk(_mutex);
  return _peers.erase(node_id) > 0;
}

void state_replica::mark_alive(const std::string &node_id, bool alive) {
  std::lock_guard<std::mutex> lk(_mutex);
  auto it = _peers.find(node_id);
  if (it != _peers.end())
    it->second.alive = alive;
}

bool state_replica::has_peer(const std::string &node_id) const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _peers.find(node_id) != _peers.end();
}

std::vector<state_replica::peer_info> state_replica::peers() const {
  std::lock_guard<std::mutex> lk(_mutex);
  std::vector<peer_info> out;
  out.reserve(_peers.size());
  for (const auto &kv : _peers)
    out.push_back(kv.second);
  return out;
}

std::size_t state_replica::peer_count() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _peers.size();
}

std::uint64_t state_replica::set_last_applied(const std::string &node_id, std::uint64_t sequence) {
  std::lock_guard<std::mutex> lk(_mutex);
  auto it = _peers.find(node_id);
  if (it == _peers.end()) {
    peer_info p;
    p.node_id = node_id;
    p.last_applied_sequence = sequence;
    p.alive = true;
    _peers.emplace(node_id, std::move(p));
    return 0;
  }
  std::uint64_t prev = it->second.last_applied_sequence;
  if (sequence > prev)
    it->second.last_applied_sequence = sequence;
  return prev;
}

std::uint64_t state_replica::last_applied(const std::string &node_id) const {
  std::lock_guard<std::mutex> lk(_mutex);
  auto it = _peers.find(node_id);
  return (it == _peers.end()) ? 0u : it->second.last_applied_sequence;
}

void state_replica::observe_local(std::uint64_t local_sequence) {
  std::lock_guard<std::mutex> lk(_mutex);
  auto &slot = _clock[_local_node_id];
  if (local_sequence > slot)
    slot = local_sequence;
}

void state_replica::observe_remote(const std::string &node_id, std::uint64_t sequence) {
  std::lock_guard<std::mutex> lk(_mutex);
  auto &slot = _clock[node_id];
  if (sequence > slot)
    slot = sequence;
}

std::unordered_map<std::string, std::uint64_t> state_replica::clock_snapshot() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _clock;
}

std::uint64_t state_replica::clock_component(const std::string &node_id) const {
  std::lock_guard<std::mutex> lk(_mutex);
  auto it = _clock.find(node_id);
  return (it == _clock.end()) ? 0u : it->second;
}

state_replica::clock_compare
state_replica::compare_clocks(const std::unordered_map<std::string, std::uint64_t> &a,
                              const std::unordered_map<std::string, std::uint64_t> &b) {
  bool a_lt = false;
  bool b_lt = false;
  // Walk the union of node_ids.
  auto check = [&](const std::string &node, std::uint64_t av, std::uint64_t bv) {
    (void)node;
    if (av < bv)
      a_lt = true;
    else if (bv < av)
      b_lt = true;
  };
  for (const auto &kv : a) {
    auto it = b.find(kv.first);
    std::uint64_t bv = (it == b.end()) ? 0u : it->second;
    check(kv.first, kv.second, bv);
  }
  for (const auto &kv : b) {
    if (a.find(kv.first) != a.end())
      continue;
    check(kv.first, 0u, kv.second);
  }
  if (!a_lt && !b_lt)
    return clock_compare::equal;
  if (a_lt && !b_lt)
    return clock_compare::less_than;
  if (!a_lt && b_lt)
    return clock_compare::greater_than;
  return clock_compare::concurrent;
}

bool state_replica::should_replace(const state_mutation &current, const state_mutation &incoming) {
  // If both carry HLC timestamps, prefer the later one. Break ties
  // with the existing (origin_node_id, sequence) total order.
  if (current.hlc_time != 0 && incoming.hlc_time != 0) {
    if (incoming.hlc_time != current.hlc_time)
      return incoming.hlc_time > current.hlc_time;
  }
  if (incoming.origin_node_id == current.origin_node_id)
    return incoming.sequence > current.sequence;
  // Different origins: lexicographic origin then sequence.
  if (incoming.origin_node_id != current.origin_node_id)
    return incoming.origin_node_id > current.origin_node_id;
  return incoming.sequence > current.sequence;
}

bool state_replica::seen(const std::string &origin_node_id, std::uint64_t sequence, bool record) {
  std::lock_guard<std::mutex> lk(_mutex);
  auto &vec = _seen[origin_node_id];
  auto it = std::lower_bound(vec.begin(), vec.end(), sequence);
  bool present = (it != vec.end() && *it == sequence);
  if (record && !present)
    vec.insert(it, sequence);
  return present;
}

std::size_t state_replica::seen_size() const {
  std::lock_guard<std::mutex> lk(_mutex);
  std::size_t total = 0;
  for (const auto &kv : _seen)
    total += kv.second.size();
  return total;
}

void state_replica::clear_seen() {
  std::lock_guard<std::mutex> lk(_mutex);
  _seen.clear();
}

} // namespace CVC_NAMESPACE
