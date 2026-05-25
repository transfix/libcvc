/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/core/state_authority_map.h>
#include <sstream>

namespace cvc {

std::string state_authority_map::normalize_path(const std::string &path) {
  std::string s = path;
  while (!s.empty() && s.front() == '.')
    s.erase(s.begin());
  while (!s.empty() && s.back() == '.')
    s.pop_back();
  return s;
}

std::vector<std::string> state_authority_map::split_path(const std::string &path) {
  std::vector<std::string> out;
  std::string norm = normalize_path(path);
  if (norm.empty())
    return out;
  std::string cur;
  for (char c : norm) {
    if (c == '.') {
      if (!cur.empty()) {
        out.push_back(cur);
        cur.clear();
      }
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty())
    out.push_back(cur);
  return out;
}

state_authority_map::state_authority_map() : _count(0) {}

state_authority_map::trie_node &
state_authority_map::find_or_create_node(const std::string &path_prefix) {
  trie_node *cur = &_root;
  for (const std::string &seg : split_path(path_prefix)) {
    auto it = cur->children.find(seg);
    if (it == cur->children.end()) {
      auto child = std::make_unique<trie_node>();
      trie_node *raw = child.get();
      cur->children.emplace(seg, std::move(child));
      cur = raw;
    } else {
      cur = it->second.get();
    }
  }
  return *cur;
}

state_authority_map::trie_node *state_authority_map::find_node(const std::string &path_prefix) {
  trie_node *cur = &_root;
  for (const std::string &seg : split_path(path_prefix)) {
    auto it = cur->children.find(seg);
    if (it == cur->children.end())
      return nullptr;
    cur = it->second.get();
  }
  return cur;
}

void state_authority_map::delegate(const std::string &path_prefix, const std::string &cluster_id,
                                   const std::string &endpoint, std::uint64_t expires_at_ns) {
  std::lock_guard<std::mutex> lk(_mutex);
  trie_node &node = find_or_create_node(path_prefix);
  if (!node.has_authority)
    ++_count;
  node.has_authority = true;
  node.auth.cluster_id = cluster_id;
  node.auth.endpoint = endpoint;
  node.auth.expires_at_ns = expires_at_ns;
  node.auth.valid = true;
}

bool state_authority_map::revoke(const std::string &path_prefix) {
  std::lock_guard<std::mutex> lk(_mutex);
  trie_node *n = find_node(path_prefix);
  if (n == nullptr || !n->has_authority)
    return false;
  n->has_authority = false;
  n->auth = authority{};
  --_count;
  return true;
}

state_authority_map::authority state_authority_map::resolve(const std::string &path) const {
  std::lock_guard<std::mutex> lk(_mutex);
  authority best;
  // Always check root first; then walk segments and track deepest
  // node with has_authority.
  const trie_node *cur = &_root;
  if (cur->has_authority)
    best = cur->auth;
  for (const std::string &seg : split_path(path)) {
    auto it = cur->children.find(seg);
    if (it == cur->children.end())
      break;
    cur = it->second.get();
    if (cur->has_authority)
      best = cur->auth;
  }
  return best;
}

bool state_authority_map::has_exact(const std::string &path_prefix) const {
  std::lock_guard<std::mutex> lk(_mutex);
  // Mutable lookup is safe under lock because we don't modify.
  const trie_node *cur = &_root;
  for (const std::string &seg : split_path(path_prefix)) {
    auto it = cur->children.find(seg);
    if (it == cur->children.end())
      return false;
    cur = it->second.get();
  }
  return cur->has_authority;
}

std::size_t state_authority_map::size() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _count;
}

void state_authority_map::clear() {
  std::lock_guard<std::mutex> lk(_mutex);
  _root.has_authority = false;
  _root.auth = authority{};
  _root.children.clear();
  _count = 0;
}

namespace {} // namespace

void state_authority_map::collect_snapshot(
    const state_authority_map::trie_node &node, std::string path,
    std::vector<std::pair<std::string, state_authority_map::authority>> &out) {
  if (node.has_authority)
    out.emplace_back(path, node.auth);
  for (const auto &kv : node.children) {
    std::string child_path = path.empty() ? kv.first : path + "." + kv.first;
    collect_snapshot(*kv.second, child_path, out);
  }
}

std::vector<std::pair<std::string, state_authority_map::authority>>
state_authority_map::snapshot() const {
  std::lock_guard<std::mutex> lk(_mutex);
  std::vector<std::pair<std::string, authority>> out;
  collect_snapshot(_root, std::string(), out);
  std::sort(out.begin(), out.end(), [](const auto &a, const auto &b) {
    // Most specific (longest) first; lex as tie-breaker.
    if (a.first.size() != b.first.size())
      return a.first.size() > b.first.size();
    return a.first < b.first;
  });
  return out;
}

} // namespace cvc
