/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/state.h>
#include <cvc/state_authority_map.h>
#include <cvc/state_blob_store.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_delegation_manager.h>
#include <cvc/state_distributed_admin.h>
#include <cvc/state_message_bus.h>
#include <cvc/state_peer_registry.h>
#include <cvc/state_telemetry_aggregator.h>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace cvc {

void state_distributed_admin::attach_shard(state_cluster_shard *shard) noexcept { _shard = shard; }

void state_distributed_admin::attach_peer_registry(state_peer_registry *peers) noexcept {
  _peers = peers;
}

void state_distributed_admin::attach_blob_store(state_blob_store *blobs) noexcept {
  _blobs = blobs;
}

void state_distributed_admin::attach_message_bus(state_message_bus *bus) noexcept { _bus = bus; }

void state_distributed_admin::attach_telemetry(state_telemetry_aggregator *tel) noexcept {
  _tel = tel;
}

std::string state_distributed_admin::to_text() const {
  std::string result = to_text(snapshot());
  if (_tel) {
    result += _tel->to_text();
  }
  return result;
}

state_distributed_admin::report state_distributed_admin::snapshot() const {
  report r;

  if (_shard) {
    r.shard.attached = true;
    r.shard.cluster_id = _shard->cluster_id();
    r.shard.node_id = _shard->local_node_id();
    r.shard.enforce_authority = _shard->enforce_authority();
    r.shard.enforce_write_policy = _shard->enforce_write_policy();
    r.shard.enforce_delegation = _shard->enforce_delegation();
    r.shard.resolve_conflicts = _shard->resolve_conflicts();
    r.shard.total_remote_applied = _shard->total_remote_applied();
    r.shard.total_remote_duplicates = _shard->total_remote_duplicates();
    r.shard.total_remote_rejected = _shard->total_remote_rejected();
    r.shard.total_conflicts_detected = _shard->total_conflicts_detected();
    r.shard.total_conflicts_lost = _shard->total_conflicts_lost();
    r.shard.total_delegation_routed = _shard->total_delegation_routed();
    r.shard.total_delegation_expired = _shard->total_delegation_expired();

    auto snap = _shard->delegation().authority().snapshot();
    r.delegations.reserve(snap.size());
    for (auto &kv : snap) {
      delegation_entry e;
      e.prefix = kv.first;
      e.cluster_id = kv.second.cluster_id;
      e.endpoint = kv.second.endpoint;
      e.expires_at_ns = kv.second.expires_at_ns;
      r.delegations.push_back(std::move(e));
    }
  }

  if (_peers) {
    auto plist = _peers->snapshot();
    r.peers.reserve(plist.size());
    for (auto &p : plist) {
      peer_entry e;
      e.node_id = p.node_id;
      e.cluster_id = p.cluster_id;
      e.endpoint = p.endpoint;
      e.subscriptions = p.subscriptions;
      e.last_seen_ns = p.last_seen_ns;
      e.mutations_delivered = p.mutations_delivered;
      e.messages_delivered = p.messages_delivered;
      e.deliveries_filtered = p.deliveries_filtered;
      r.peers.push_back(std::move(e));
    }
  }

  if (_bus) {
    r.bus.attached = true;
    r.bus.total_admitted = _bus->total_admitted();
    r.bus.total_duplicates = _bus->total_duplicates();
    r.bus.total_dispatched = _bus->total_dispatched();
    r.bus.total_dropped = _bus->total_dropped();
  }

  if (_blobs) {
    r.blobs.attached = true;
    r.blobs.count = _blobs->size();
    r.blobs.bytes_stored = _blobs->bytes_stored();
  }

  return r;
}

std::string state_distributed_admin::to_text(const report &r) {
  std::ostringstream os;
  os << "[shard]\n";
  if (!r.shard.attached) {
    os << "  detached\n";
  } else {
    os << "  cluster_id=" << r.shard.cluster_id << "\n"
       << "  node_id=" << r.shard.node_id << "\n"
       << "  enforce_authority=" << (r.shard.enforce_authority ? 1 : 0) << "\n"
       << "  enforce_write_policy=" << (r.shard.enforce_write_policy ? 1 : 0) << "\n"
       << "  enforce_delegation=" << (r.shard.enforce_delegation ? 1 : 0) << "\n"
       << "  resolve_conflicts=" << (r.shard.resolve_conflicts ? 1 : 0) << "\n"
       << "  remote_applied=" << r.shard.total_remote_applied << "\n"
       << "  remote_duplicates=" << r.shard.total_remote_duplicates << "\n"
       << "  remote_rejected=" << r.shard.total_remote_rejected << "\n"
       << "  conflicts_detected=" << r.shard.total_conflicts_detected << "\n"
       << "  conflicts_lost=" << r.shard.total_conflicts_lost << "\n"
       << "  delegation_routed=" << r.shard.total_delegation_routed << "\n"
       << "  delegation_expired=" << r.shard.total_delegation_expired << "\n";
  }

  os << "[delegations] count=" << r.delegations.size() << "\n";
  for (auto &d : r.delegations) {
    os << "  prefix='" << d.prefix << "' cluster=" << d.cluster_id << " endpoint=" << d.endpoint
       << " expires_at_ns=" << d.expires_at_ns << "\n";
  }

  os << "[peers] count=" << r.peers.size() << "\n";
  for (auto &p : r.peers) {
    os << "  node=" << p.node_id << " cluster=" << p.cluster_id << " endpoint=" << p.endpoint
       << " subs=" << p.subscriptions.size() << " last_seen_ns=" << p.last_seen_ns
       << " mut_delivered=" << p.mutations_delivered << " msg_delivered=" << p.messages_delivered
       << " filtered=" << p.deliveries_filtered << "\n";
  }

  os << "[bus]\n";
  if (!r.bus.attached) {
    os << "  detached\n";
  } else {
    os << "  admitted=" << r.bus.total_admitted << "\n"
       << "  duplicates=" << r.bus.total_duplicates << "\n"
       << "  dispatched=" << r.bus.total_dispatched << "\n"
       << "  dropped=" << r.bus.total_dropped << "\n";
  }

  os << "[blobs]\n";
  if (!r.blobs.attached) {
    os << "  detached\n";
  } else {
    os << "  count=" << r.blobs.count << "\n"
       << "  bytes_stored=" << r.blobs.bytes_stored << "\n";
  }

  return os.str();
}

state_distributed_admin::gc_result
state_distributed_admin::gc_blobs(const std::unordered_set<std::string> &live_digests) {
  gc_result g;
  if (!_blobs)
    return g;

  std::vector<std::string> all = _blobs->digests();
  g.scanned = all.size();
  for (const auto &d : all) {
    if (live_digests.find(d) != live_digests.end())
      continue;
    // Read size before erase so we can report bytes_freed accurately.
    std::vector<unsigned char> bytes;
    std::uint64_t sz = 0;
    if (_blobs->get(d, bytes))
      sz = static_cast<std::uint64_t>(bytes.size());
    if (_blobs->erase(d)) {
      ++g.removed;
      g.bytes_freed += sz;
    }
  }
  return g;
}

// ---------------------------------------------------------------------------
// link_cycles: Tarjan SCC over the link graph rooted at `root`.
// ---------------------------------------------------------------------------

namespace {

struct tarjan_ctx {
  std::vector<std::vector<int>> adj; // adj[u] -> successors
  std::vector<int> index_of;         // -1 == unvisited
  std::vector<int> lowlink;
  std::vector<bool> on_stack;
  std::vector<int> stack;
  int next_index = 0;
  std::vector<std::vector<int>> sccs;

  void strongconnect(int v) {
    index_of[v] = next_index;
    lowlink[v] = next_index;
    ++next_index;
    stack.push_back(v);
    on_stack[v] = true;

    for (int w : adj[v]) {
      if (index_of[w] < 0) {
        strongconnect(w);
        lowlink[v] = std::min(lowlink[v], lowlink[w]);
      } else if (on_stack[w]) {
        lowlink[v] = std::min(lowlink[v], index_of[w]);
      }
    }

    if (lowlink[v] == index_of[v]) {
      std::vector<int> comp;
      while (true) {
        int w = stack.back();
        stack.pop_back();
        on_stack[w] = false;
        comp.push_back(w);
        if (w == v)
          break;
      }
      sccs.push_back(std::move(comp));
    }
  }
};

} // namespace

state_distributed_admin::link_cycles_result state_distributed_admin::link_cycles(state &root) {
  link_cycles_result result;

  // Step 1: enumerate every node in the subtree (root + descendants).
  std::vector<std::string> all_paths = root.children();
  // children() does not include the root itself; include it so a
  // root that is itself a link is analyzed too.
  all_paths.insert(all_paths.begin(), root.fullName());

  // Step 2: collect link nodes and assign integer ids to each one's
  // absolute path. Non-link nodes are ignored: they cannot
  // participate in a cycle of links.
  std::unordered_map<std::string, int> id_of;
  std::vector<std::string> paths;       // paths[id] -> absolute path
  std::vector<std::string> targets_raw; // targets[id] -> linkTarget()
  for (const std::string &p : all_paths) {
    state *n = (p == root.fullName()) ? &root : root.findDescendant(p);
    if (n == nullptr || !n->isLink())
      continue;
    int id = static_cast<int>(paths.size());
    id_of.emplace(p, id);
    paths.push_back(p);
    targets_raw.push_back(n->linkTarget());
  }
  result.link_nodes_scanned = paths.size();

  if (paths.empty())
    return result;

  // Step 3: build adjacency. Edge l -> t exists iff `t` is the
  // normalized linkTarget of `l` AND `t` is itself a link node in
  // the same tree. linkTarget() is stored already normalized.
  // The single canonical exception is the DNS-style root marker
  // ".": it represents the root path, whose fullName() is "".
  tarjan_ctx tc;
  tc.adj.assign(paths.size(), {});
  for (std::size_t i = 0; i < paths.size(); ++i) {
    const std::string &raw = targets_raw[i];
    const std::string key = (raw == state::SEPARATOR) ? std::string() : raw;
    const auto it = id_of.find(key);
    if (it != id_of.end())
      tc.adj[i].push_back(it->second);
  }

  // Step 4: run Tarjan SCC.
  tc.index_of.assign(paths.size(), -1);
  tc.lowlink.assign(paths.size(), 0);
  tc.on_stack.assign(paths.size(), false);
  for (std::size_t i = 0; i < paths.size(); ++i) {
    if (tc.index_of[i] < 0)
      tc.strongconnect(static_cast<int>(i));
  }

  // Step 5: report SCCs that form a cycle. Singletons are cycles
  // only when they have a self-loop.
  for (auto &comp : tc.sccs) {
    bool is_cycle = comp.size() > 1;
    if (!is_cycle && comp.size() == 1) {
      int v = comp[0];
      for (int w : tc.adj[v]) {
        if (w == v) {
          is_cycle = true;
          break;
        }
      }
    }
    if (!is_cycle)
      continue;

    std::vector<std::string> cyc;
    cyc.reserve(comp.size());
    // Each link node has out-degree <= 1 (a single linkTarget),
    // so any SCC of size >= 1 with a cycle is a simple directed
    // cycle. Reconstruct traversal order starting at the
    // lexicographically smallest path for stability.
    int start = comp[0];
    for (int v : comp)
      if (paths[v] < paths[start])
        start = v;
    int cur = start;
    do {
      cyc.push_back(paths[cur]);
      // out-degree is at most 1; follow it.
      if (tc.adj[cur].empty())
        break; // defensive: should not happen for an SCC member.
      cur = tc.adj[cur][0];
    } while (cur != start && cyc.size() <= comp.size());
    result.cycles.push_back(std::move(cyc));
  }

  // Sort cycles themselves by their first element for stability
  // across runs.
  std::sort(result.cycles.begin(), result.cycles.end(),
            [](const std::vector<std::string> &a, const std::vector<std::string> &b) {
              return a.front() < b.front();
            });

  return result;
}

// ---- Slice 4c: transparent link index ----

namespace {

// Canonical form of a link target: "." (root marker) becomes "".
std::string canonical_target(const std::string &raw) {
  return (raw == state::SEPARATOR) ? std::string() : raw;
}

} // namespace

state_distributed_admin::transparent_link_index_result
state_distributed_admin::transparent_link_index(state &root) {
  transparent_link_index_result result;

  std::vector<std::string> all_paths = root.children();
  all_paths.insert(all_paths.begin(), root.fullName());

  for (const std::string &p : all_paths) {
    state *n = (p == root.fullName()) ? &root : root.findDescendant(p);
    if (n == nullptr || !n->isLink())
      continue;
    ++result.link_nodes_scanned;
    if (n->linkMode() != state::link_mode::transparent)
      continue;
    transparent_link tl;
    tl.link_path = p;
    tl.target_path = canonical_target(n->linkTarget());
    result.links.push_back(std::move(tl));
  }

  std::sort(result.links.begin(), result.links.end(),
            [](const transparent_link &a, const transparent_link &b) {
              if (a.link_path != b.link_path)
                return a.link_path < b.link_path;
              return a.target_path < b.target_path;
            });
  return result;
}

std::vector<std::string> state_distributed_admin::transparent_link_aliases(state &root,
                                                                           const std::string &path,
                                                                           std::size_t hop_budget) {
  // BFS over the transparent-link graph: starting from `path`, at
  // each hop expand the current frontier through every transparent
  // link whose target equals or is a dot-segment-aware prefix of
  // some frontier path. Stop when no new aliases appear or the hop
  // budget is exhausted. Cycles terminate naturally because already-
  // seen paths are not re-enqueued.
  auto idx = transparent_link_index(root);

  std::unordered_set<std::string> seen;
  seen.insert(path);
  std::vector<std::string> frontier = {path};
  std::vector<std::string> out;

  std::size_t hops = 0;
  while (!frontier.empty() && hops < hop_budget) {
    std::vector<std::string> next;
    for (const std::string &cur : frontier) {
      for (const auto &tl : idx.links) {
        // Don't traverse the same link twice. If cur is at or under
        // tl.link_path then we've already passed through tl in
        // producing cur (or applying tl now would just produce a
        // deeper redundant nesting under link_path). This is the
        // key termination guard for root-targeted transparent
        // links, where tl.target_path is empty and would otherwise
        // match every frontier path on every hop.
        if (cur.size() >= tl.link_path.size() &&
            cur.compare(0, tl.link_path.size(), tl.link_path) == 0 &&
            (cur.size() == tl.link_path.size() || cur[tl.link_path.size()] == '.'))
          continue;
        std::string suffix;
        if (tl.target_path.empty()) {
          // Root-targeted link: every path aliases under link_path.
          suffix = cur;
        } else if (cur == tl.target_path) {
          suffix.clear();
        } else if (cur.size() > tl.target_path.size() &&
                   cur.compare(0, tl.target_path.size(), tl.target_path) == 0 &&
                   cur[tl.target_path.size()] == '.') {
          suffix = cur.substr(tl.target_path.size() + 1);
        } else {
          continue; // no prefix match
        }
        std::string aliased = tl.link_path;
        if (!suffix.empty()) {
          if (!aliased.empty())
            aliased += '.';
          aliased += suffix;
        }
        if (aliased == path)
          continue;
        if (seen.insert(aliased).second) {
          out.push_back(aliased);
          next.push_back(std::move(aliased));
        }
      }
    }
    frontier = std::move(next);
    ++hops;
  }

  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

// -------- Phase 7: force resync --------

state_distributed_admin::resync_result
state_distributed_admin::force_resync(const std::string &peer_node_id) {
  resync_result result;
  if (!_shard || !_peers)
    return result;

  if (!_peers->has_peer(peer_node_id))
    return result;

  // Drain any pending local mutations and count them as the resync
  // payload. In a full implementation the transport would replay
  // these to the specified peer; here we measure the work that
  // would be needed. drain_local(0) = drain all.
  auto pending = _shard->drain_local(0);
  for (const auto &m : pending) {
    result.mutations_sent++;
    result.bytes_sent += m.string_value.size();
  }

  return result;
}

// -------- Phase 7: stale peer GC --------

std::vector<std::string> state_distributed_admin::gc_stale_peers(std::uint64_t now_ns,
                                                                 std::uint64_t stale_threshold_ns) {
  std::vector<std::string> removed;
  if (!_peers)
    return removed;

  auto peers = _peers->snapshot();
  for (const auto &p : peers) {
    if (p.last_seen_ns == 0)
      continue; // never seen — don't GC peers that never heartbeated
    if (now_ns > p.last_seen_ns && (now_ns - p.last_seen_ns) > stale_threshold_ns) {
      if (_peers->remove_peer(p.node_id))
        removed.push_back(p.node_id);
    }
  }
  return removed;
}

} // namespace cvc
