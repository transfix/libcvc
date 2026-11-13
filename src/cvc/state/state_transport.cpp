/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include <algorithm>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_transport.h>
#include <unordered_set>

namespace cvc {

namespace {
bool reads(const state_delivery_filter *f, const std::string &path) {
  return f == nullptr || f->deliver(path);
}
} // namespace

void state_transport::read_scope_change(const std::vector<snapshot_entry> &tree,
                                        const state_delivery_filter *before,
                                        const state_delivery_filter *after,
                                        std::vector<std::string> &retract,
                                        std::vector<snapshot_entry> &grant) {
  retract.clear();
  grant.clear();
  // Every node above a path `after` delivers stays: the peer still needs it to hold that path.
  std::unordered_set<std::string> above;
  for (const auto &e : tree) {
    if (!reads(after, e.path))
      continue;
    for (std::size_t dot = e.path.find('.'); dot != std::string::npos;
         dot = e.path.find('.', dot + 1))
      above.insert(e.path.substr(0, dot));
  }
  std::unordered_set<std::string> gone;
  for (const auto &e : tree) {
    const bool was = reads(before, e.path), is = reads(after, e.path);
    if (is && !was)
      grant.push_back(e);
    else if (was && !is && above.count(e.path) == 0)
      gone.insert(e.path);
  }
  // A subtree's root alone: the delete takes what is below it.
  for (const auto &p : gone) {
    bool under = false;
    for (std::size_t dot = p.find('.'); dot != std::string::npos && !under;
         dot = p.find('.', dot + 1))
      under = gone.count(p.substr(0, dot)) != 0;
    if (!under)
      retract.push_back(p);
  }
  std::sort(retract.begin(), retract.end());
}

state_mutation state_transport::read_scope_retraction(state_cluster_shard &shard,
                                                      const std::string &path) {
  state_mutation m;
  m.cluster_id = shard.cluster_id();
  m.origin_node_id = shard.local_node_id() + "#scope";
  m.hlc_time = shard.clock().now().packed();
  m.sequence = m.hlc_time;
  m.path = path;
  m.op = state_mutation_op::delete_subtree;
  return m;
}

bool state_transport::read_scope_delivers(const state_delivery_filter *scope,
                                          const state_mutation &m) noexcept {
  if (scope == nullptr || scope->deliver(m.path))
    return true;
  const bool subtree =
      m.op == state_mutation_op::delete_subtree || m.op == state_mutation_op::reset_node;
  return subtree && scope->delivers_below(m.path);
}

} // namespace cvc
