/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state.h>
#include <cvc/state_peer_registry.h>
#include <stdexcept>
#include <utility>

namespace CVC_NAMESPACE {

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
  auto &p = _peers[node_id];
  p.node_id = std::move(node_id);
  p.cluster_id = std::move(cluster_id);
  p.endpoint = std::move(endpoint);
  p.subscriptions = std::move(subscriptions);
}

bool state_peer_registry::remove_peer(const std::string &node_id) {
  std::lock_guard<std::mutex> lk(_mu);
  return _peers.erase(node_id) > 0;
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

bool state_peer_registry::should_deliver(const std::string &node_id,
                                         const std::string &path) const {
  std::lock_guard<std::mutex> lk(_mu);
  auto it = _peers.find(node_id);
  if (it == _peers.end())
    return true; // unknown peer => back-compat default
  return any_prefix_matches(it->second.subscriptions, path);
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
  std::lock_guard<std::mutex> lk(_mu);
  _peers.clear();
}

} // namespace CVC_NAMESPACE
