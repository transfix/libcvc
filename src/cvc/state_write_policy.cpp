/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_write_policy.h>
#include <memory>
#include <utility>

namespace CVC_NAMESPACE {

state_write_policy::state_write_policy() = default;

std::vector<std::string> state_write_policy::split_path(const std::string &path) {
  std::vector<std::string> out;
  if (path.empty())
    return out;
  std::size_t start = 0;
  for (std::size_t i = 0; i <= path.size(); ++i) {
    if (i == path.size() || path[i] == '.') {
      if (i > start)
        out.emplace_back(path.substr(start, i - start));
      start = i + 1;
    }
  }
  return out;
}

state_write_policy::trie_node &
state_write_policy::find_or_create_node(const std::string &path_prefix) {
  trie_node *node = &_root;
  for (const auto &seg : split_path(path_prefix)) {
    auto &child = node->children[seg];
    if (!child)
      child = std::make_unique<trie_node>();
    node = child.get();
  }
  return *node;
}

void state_write_policy::allow(const std::string &path_prefix,
                               std::vector<std::string> allowed_writers) {
  std::lock_guard<std::mutex> lk(_mu);
  auto &node = find_or_create_node(path_prefix);
  if (!node.has_policy) {
    node.has_policy = true;
    ++_count;
  }
  node.allowed.clear();
  for (auto &w : allowed_writers)
    node.allowed.insert(std::move(w));
}

bool state_write_policy::revoke(const std::string &path_prefix) {
  std::lock_guard<std::mutex> lk(_mu);
  trie_node *node = &_root;
  for (const auto &seg : split_path(path_prefix)) {
    auto it = node->children.find(seg);
    if (it == node->children.end())
      return false;
    node = it->second.get();
  }
  if (!node->has_policy)
    return false;
  node->has_policy = false;
  node->allowed.clear();
  if (_count > 0)
    --_count;
  return true;
}

state_write_policy::decision
state_write_policy::authorize(const std::string &path, const std::string &origin_node_id) const {
  std::lock_guard<std::mutex> lk(_mu);
  decision result;
  const trie_node *node = &_root;
  std::string matched_prefix;
  const trie_node *best = nullptr;
  std::string best_prefix;
  if (node->has_policy) {
    best = node;
    best_prefix = matched_prefix;
  }
  std::string current;
  for (const auto &seg : split_path(path)) {
    auto it = node->children.find(seg);
    if (it == node->children.end())
      break;
    node = it->second.get();
    if (!current.empty())
      current.push_back('.');
    current += seg;
    if (node->has_policy) {
      best = node;
      best_prefix = current;
    }
  }
  if (!best) {
    result.allowed = true;
    result.had_policy = false;
    return result;
  }
  result.had_policy = true;
  result.matched_prefix = best_prefix;
  if (best->allowed.find(origin_node_id) != best->allowed.end()) {
    result.allowed = true;
  } else {
    result.allowed = false;
    result.reject_reason =
        "write policy denies node " + origin_node_id + " for prefix '" + best_prefix + "'";
  }
  return result;
}

std::size_t state_write_policy::size() const {
  std::lock_guard<std::mutex> lk(_mu);
  return _count;
}

void state_write_policy::clear() {
  std::lock_guard<std::mutex> lk(_mu);
  _root.children.clear();
  _root.has_policy = false;
  _root.allowed.clear();
  _count = 0;
}

} // namespace CVC_NAMESPACE
