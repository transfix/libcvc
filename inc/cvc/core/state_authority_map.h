/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_AUTHORITY_MAP_H__
#define __CVC_STATE_AUTHORITY_MAP_H__

#include <cstdint>
#include <cvc/namespace.h>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace cvc {

// ----------------
// cvc::state_authority_map
// ----------------
// Purpose:
//   DNS-like longest-prefix delegation table for distributed-state
//   paths. Each entry binds a dot-separated path prefix to an
//   authority (cluster_id or peer endpoint) that owns writes for
//   that subtree, plus an optional lease expiry.
//
// Lookup is longest-prefix match. A path of "a.b.c.d" first checks
// "a.b.c.d", then "a.b.c", then "a.b", then "a", then root. The
// most specific entry wins.
//
// Threading:
//   Thread-safe. All public methods may be called concurrently.
//
// Lease expiry:
//   `expires_at_ns` is a steady_clock-relative timestamp (or 0 if
//   the delegation has no expiry). The map does not actively expire
//   entries; callers should ignore expired results based on their
//   own clock.
//
class state_authority_map {
public:
  struct authority {
    std::string cluster_id;
    std::string endpoint; // optional, e.g. "grpc://host:port"
    std::uint64_t expires_at_ns = 0;
    bool valid = false;
  };

  state_authority_map();

  // Add or replace a delegation for `path_prefix`. An empty prefix
  // means root.
  void delegate(const std::string &path_prefix, const std::string &cluster_id,
                const std::string &endpoint = std::string(), std::uint64_t expires_at_ns = 0);

  // Remove the exact delegation at `path_prefix`. Returns true if
  // an entry existed.
  bool revoke(const std::string &path_prefix);

  // Longest-prefix lookup. Returns the most specific delegation
  // covering `path`. The returned `authority::valid` is false if
  // no entry matches.
  authority resolve(const std::string &path) const;

  // Returns true if there is an exact-match entry at `path_prefix`.
  bool has_exact(const std::string &path_prefix) const;

  std::size_t size() const;
  void clear();

  // Snapshot of all delegations, sorted by prefix length descending
  // (most specific first). Useful for diagnostics.
  std::vector<std::pair<std::string, authority>> snapshot() const;

private:
  static std::string normalize_path(const std::string &path);
  static std::vector<std::string> split_path(const std::string &path);

  struct trie_node {
    bool has_authority = false;
    authority auth;
    std::map<std::string, std::unique_ptr<trie_node>> children;
  };

  trie_node &find_or_create_node(const std::string &path_prefix);
  trie_node *find_node(const std::string &path_prefix);
  static void collect_snapshot(const trie_node &node, std::string path,
                               std::vector<std::pair<std::string, authority>> &out);

  mutable std::mutex _mutex;
  trie_node _root;
  std::size_t _count;
};

} // namespace cvc

#endif // __CVC_STATE_AUTHORITY_MAP_H__
