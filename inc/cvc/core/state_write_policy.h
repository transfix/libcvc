/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_WRITE_POLICY_H__
#define __CVC_STATE_WRITE_POLICY_H__

#include <cstddef>
#include <cvc/namespace.h>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

namespace cvc {

// ----------------
// cvc::state_write_policy
// ----------------
// Phase 5.
//
// Per-path-prefix write-authorization table. Mirrors the trie-based
// longest-prefix lookup of state_authority_map but binds each
// prefix to the set of node_ids permitted to write that subtree.
//
// Policy semantics:
//   - If no entry covers `path`, the policy is permissive (allow).
//     Distributed-state remains usable when no policy has been
//     installed.
//   - The most specific (longest) matching prefix wins.
//   - If the matching entry has an empty allowed-writers set, all
//     writers are denied for that subtree (lockdown).
//   - allow(prefix, node_ids) replaces any prior entry at exactly
//     that prefix.
//
// Threading:
//   All public methods are safe to call from multiple threads.
//
class state_write_policy {
public:
  struct decision {
    bool allowed = true;
    bool had_policy = false;    // an entry covered this path
    std::string matched_prefix; // empty if no entry matched
    std::string reject_reason;  // populated on deny
  };

  state_write_policy();

  state_write_policy(const state_write_policy &) = delete;
  state_write_policy &operator=(const state_write_policy &) = delete;

  // Install a policy entry at `path_prefix`. The set of allowed
  // writer node_ids replaces any prior set at that exact prefix.
  // An empty set denies all writers for that subtree.
  void allow(const std::string &path_prefix, std::vector<std::string> allowed_writers);

  // Remove the exact entry at `path_prefix`. Returns true if an
  // entry existed.
  bool revoke(const std::string &path_prefix);

  // Authorize `origin_node_id` to write `path`. The lookup is
  // longest-prefix.
  decision authorize(const std::string &path, const std::string &origin_node_id) const;

  std::size_t size() const;
  void clear();

private:
  struct trie_node {
    bool has_policy = false;
    std::unordered_set<std::string> allowed;
    std::map<std::string, std::unique_ptr<trie_node>> children;
  };

  static std::vector<std::string> split_path(const std::string &path);
  trie_node &find_or_create_node(const std::string &path_prefix);

  mutable std::mutex _mu;
  trie_node _root;
  std::size_t _count = 0;
};

} // namespace cvc

#endif // __CVC_STATE_WRITE_POLICY_H__
