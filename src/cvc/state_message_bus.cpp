/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cstring>
#include <cvc/state_message_bus.h>
#include <utility>

namespace CVC_NAMESPACE {

state_message_bus::state_message_bus() = default;

state_message_bus::subscription_id state_message_bus::subscribe(std::string path_prefix,
                                                                subscriber_fn cb) {
  if (!cb)
    return 0;
  std::lock_guard<std::mutex> lk(_mu);
  subscription_id id = _next_id++;
  _subs.push_back({id, std::move(path_prefix), std::move(cb)});
  return id;
}

bool state_message_bus::unsubscribe(subscription_id id) {
  std::lock_guard<std::mutex> lk(_mu);
  auto it =
      std::remove_if(_subs.begin(), _subs.end(), [id](const subscriber &s) { return s.id == id; });
  if (it == _subs.end())
    return false;
  _subs.erase(it, _subs.end());
  return true;
}

bool state_message_bus::prefix_matches(const std::string &prefix,
                                       const std::string &path) noexcept {
  if (prefix.empty())
    return true;
  if (path.size() < prefix.size())
    return false;
  if (std::memcmp(path.data(), prefix.data(), prefix.size()) != 0)
    return false;
  if (path.size() == prefix.size())
    return true;
  return path[prefix.size()] == '.';
}

bool state_message_bus::admit(const state_message &m) {
  // Dedup unless both origin and id are empty (sentinel for tests
  // that want every admit to fire).
  std::vector<subscriber_fn> matched;
  bool dedup_bypass = m.origin_node_id.empty() && m.message_id.empty();
  {
    std::lock_guard<std::mutex> lk(_mu);

    if (!dedup_bypass) {
      std::string key;
      key.reserve(m.origin_node_id.size() + 1 + m.message_id.size());
      key.append(m.origin_node_id);
      key.push_back('\0');
      key.append(m.message_id);
      auto ins = _seen.insert(key);
      if (!ins.second) {
        _dups.fetch_add(1, std::memory_order_relaxed);
        return false;
      }
      _seen_order.push_back(key);
      if (_dedup_cap != 0 && _seen_order.size() > _dedup_cap) {
        const std::string &old = _seen_order.front();
        _seen.erase(old);
        _seen_order.pop_front();
      }
    }

    matched.reserve(_subs.size());
    for (const auto &s : _subs) {
      if (prefix_matches(s.prefix, m.path))
        matched.push_back(s.fn);
    }
  }

  _admitted.fetch_add(1, std::memory_order_relaxed);
  for (auto &fn : matched) {
    try {
      fn(m);
    } catch (...) {
      // Subscriber exceptions are swallowed; the bus keeps going.
    }
    _dispatched.fetch_add(1, std::memory_order_relaxed);
  }
  return true;
}

void state_message_bus::note_dropped() { _dropped.fetch_add(1, std::memory_order_relaxed); }

void state_message_bus::set_dedup_capacity(std::size_t cap) {
  std::lock_guard<std::mutex> lk(_mu);
  _dedup_cap = cap;
  if (cap != 0) {
    while (_seen_order.size() > cap) {
      _seen.erase(_seen_order.front());
      _seen_order.pop_front();
    }
  }
}

std::size_t state_message_bus::dedup_capacity() const noexcept {
  std::lock_guard<std::mutex> lk(_mu);
  return _dedup_cap;
}

std::size_t state_message_bus::subscriber_count() const {
  std::lock_guard<std::mutex> lk(_mu);
  return _subs.size();
}

std::size_t state_message_bus::dedup_size() const {
  std::lock_guard<std::mutex> lk(_mu);
  return _seen.size();
}

} // namespace CVC_NAMESPACE
