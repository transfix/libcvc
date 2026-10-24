/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state/state.h>
#include <cvc/state/state_peer_registry.h>
#include <stdexcept>
#include <utility>

namespace cvc {

state_peer_registry::state_peer_registry() = default;

bool state_peer_registry::prefix_matches(const std::string &prefix,
                                         const std::string &path) noexcept {
  if (prefix.empty())
    return true;
  if (path.size() < prefix.size())
    return false;
  if (path.compare(0, prefix.size(), prefix) != 0)
    return false;
  if (path.size() == prefix.size())
    return true;
  return path[prefix.size()] == '.';
}

bool state_peer_registry::any_prefix_matches(const std::vector<std::string> &prefixes,
                                             const std::string &path) noexcept {
  if (prefixes.empty())
    return true;
  for (const auto &p : prefixes) {
    if (prefix_matches(p, path))
      return true;
  }
  return false;
}

void state_peer_registry::add_peer(std::string node_id, std::string cluster_id,
                                   std::string endpoint, std::vector<std::string> subscriptions) {
  if (!state::isValidStateName(node_id))
    throw std::invalid_argument("node_id '" + node_id + "' violates C identifier rules");
  if (!cluster_id.empty() && !state::isValidStateName(cluster_id))
    throw std::invalid_argument("cluster_id '" + cluster_id + "' violates C identifier rules");
  std::lock_guard<std::mutex> lk(_mu);
  auto &p = _peers[node_id]; // an existing entry keeps its filter
  p.node_id = std::move(node_id);
  p.cluster_id = std::move(cluster_id);
  p.endpoint = std::move(endpoint);
  p.subscriptions = std::move(subscriptions);
}

bool state_peer_registry::remove_peer(const std::string &node_id) {
  std::shared_ptr<const state_delivery_filter> displaced; // released after _mu (see set_filter)
  std::lock_guard<std::mutex> lk(_mu);
  auto it = _peers.find(node_id);
  if (it == _peers.end())
    return false;
  displaced = std::move(it->second.filter);
  _peers.erase(it);
  return true;
}

bool state_peer_registry::set_subscriptions(const std::string &node_id,
                                            std::vector<std::string> subscriptions) {
  std::lock_guard<std::mutex> lk(_mu);
  auto it = _peers.find(node_id);
  if (it == _peers.end())
    return false;
  it->second.subscriptions = std::move(subscriptions);
  return true;
}

bool state_peer_registry::has_peer(const std::string &node_id) const {
  std::lock_guard<std::mutex> lk(_mu);
  return _peers.find(node_id) != _peers.end();
}

bool state_peer_registry::set_filter(const std::string &node_id,
                                     std::shared_ptr<const state_delivery_filter> f) {
  // Declared before the lock, so the filter this replaces is released after _mu is: when the
  // registry holds its last reference, its destructor (caller code) would otherwise run under the
  // lock every sender takes in should_deliver.
  std::shared_ptr<const state_delivery_filter> displaced;
  std::lock_guard<std::mutex> lk(_mu);
  auto it = _peers.find(node_id);
  if (it == _peers.end())
    return false;
  displaced = std::exchange(it->second.filter, std::move(f));
  return true;
}

state_peer_registry::delivery_view state_peer_registry::view_of(const std::string &node_id) const {
  delivery_view v;
  std::lock_guard<std::mutex> lk(_mu);
  auto it = _peers.find(node_id);
  if (it == _peers.end())
    return v;
  v.known = true;
  v.subscriptions = it->second.subscriptions;
  v.filter = it->second.filter;
  return v;
}

bool state_peer_registry::delivery_view::deliver(const std::string &path) const noexcept {
  if (!known)
    return true;
  if (!any_prefix_matches(subscriptions, path))
    return false;
  return !filter || filter->deliver(path);
}

bool state_peer_registry::delivery_view::filtered() const noexcept {
  return known && (!subscriptions.empty() || filter);
}

bool state_peer_registry::should_deliver(const std::string &node_id,
                                         const std::string &path) const {
  std::shared_ptr<const state_delivery_filter> f;
  {
    std::lock_guard<std::mutex> lk(_mu);
    auto it = _peers.find(node_id);
    if (it == _peers.end())
      return true; // unknown peer => back-compat default
    if (!any_prefix_matches(it->second.subscriptions, path))
      return false;
    f = it->second.filter;
  }
  // The filter is immutable; evaluate it outside the lock.
  return !f || f->deliver(path);
}

bool state_peer_registry::note_seen(const std::string &node_id, std::uint64_t now_ns) {
  std::lock_guard<std::mutex> lk(_mu);
  auto it = _peers.find(node_id);
  if (it == _peers.end())
    return false;
  it->second.last_seen_ns = now_ns;
  return true;
}

void state_peer_registry::note_mutation_delivered(const std::string &node_id) {
  std::lock_guard<std::mutex> lk(_mu);
  auto it = _peers.find(node_id);
  if (it != _peers.end())
    ++it->second.mutations_delivered;
}

void state_peer_registry::note_message_delivered(const std::string &node_id) {
  std::lock_guard<std::mutex> lk(_mu);
  auto it = _peers.find(node_id);
  if (it != _peers.end())
    ++it->second.messages_delivered;
}

void state_peer_registry::note_delivery_filtered(const std::string &node_id) {
  std::lock_guard<std::mutex> lk(_mu);
  auto it = _peers.find(node_id);
  if (it != _peers.end())
    ++it->second.deliveries_filtered;
}

std::size_t state_peer_registry::size() const {
  std::lock_guard<std::mutex> lk(_mu);
  return _peers.size();
}

std::vector<state_peer_registry::peer> state_peer_registry::snapshot() const {
  std::lock_guard<std::mutex> lk(_mu);
  std::vector<peer> out;
  out.reserve(_peers.size());
  for (const auto &kv : _peers)
    out.push_back(kv.second);
  return out;
}

void state_peer_registry::clear() {
  std::unordered_map<std::string, peer> dropped; // filters released after _mu (see set_filter)
  std::lock_guard<std::mutex> lk(_mu);
  _peers.swap(dropped);
}

} // namespace cvc
