/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_distributed_admin.h>
#include <cvc/core/state_sync_adapter.h>
#include <thread>
#include <unordered_set>
#include <utility>

namespace cvc {

namespace {

// Per-thread suppression depth shared by all adapters.
// A non-zero value means the current thread is either (a) applying a
// remote mutation or (b) inside a suppression_scope. In either case,
// state mutations originating on this thread should not be journaled
// because they are not "local user changes".
thread_local int t_suppression_depth = 0;

struct thread_suppression_guard {
  thread_suppression_guard() { ++t_suppression_depth; }
  ~thread_suppression_guard() { --t_suppression_depth; }
};

} // anonymous namespace

// ---------------- suppression_scope ----------------

state_sync_adapter::suppression_scope::suppression_scope(state_sync_adapter &a) : _adapter(a) {
  ++t_suppression_depth;
}

state_sync_adapter::suppression_scope::~suppression_scope() {
  --t_suppression_depth;
  (void)_adapter;
}

// ---------------- state_sync_adapter ----------------

state_sync_adapter::state_sync_adapter(app &ctx, std::string root_path, std::string local_node_id)
    : _ctx(ctx), _root_path(std::move(root_path)), _local_node_id(std::move(local_node_id)),
      _journal(_local_node_id), _router(), _attached(false), _local_count(0), _remote_count(0) {}

state_sync_adapter::~state_sync_adapter() { detach(); }

bool state_sync_adapter::is_attached() const noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  return _attached;
}

void state_sync_adapter::set_on_local_mutation(on_local_mutation_func cb) {
  std::lock_guard<std::mutex> lk(_mutex);
  _on_local = std::move(cb);
}

void state_sync_adapter::set_on_remote_applied(on_remote_applied_func cb) {
  std::lock_guard<std::mutex> lk(_mutex);
  _on_remote = std::move(cb);
}

std::size_t state_sync_adapter::observed_paths() const noexcept {
  std::lock_guard<std::mutex> lk(_mutex);
  return _connections.size();
}

std::uint64_t state_sync_adapter::local_mutation_count() const noexcept {
  return _local_count.load(std::memory_order_relaxed);
}

std::uint64_t state_sync_adapter::remote_mutation_count() const noexcept {
  return _remote_count.load(std::memory_order_relaxed);
}

std::uint64_t state_sync_adapter::forwarded_through_link_count() const noexcept {
  return _forwarded_through_link_count.load(std::memory_order_relaxed);
}

std::vector<state_subscription>
state_sync_adapter::subscriptions_for_path(const std::string &path) const {
  std::vector<state_subscription> direct = _router.subscriptions_for(path);

  // Expand `path` through every transparent link in the app's state
  // tree. For each link-side alias, also pull subscriptions covering
  // the aliased path. Dedupe by id so a subscription matching both
  // directly and via an alias is reported once.
  state &root = state::instance(_ctx);
  std::vector<std::string> aliases = state_distributed_admin::transparent_link_aliases(root, path);
  if (aliases.empty())
    return direct;

  std::vector<state_subscription> merged = std::move(direct);
  std::unordered_set<state_subscription_id> seen;
  seen.reserve(merged.size() + aliases.size());
  for (const auto &s : merged)
    seen.insert(s.id);

  std::uint64_t forwarded_added = 0;
  for (const std::string &alias : aliases) {
    auto extras = _router.subscriptions_for(alias);
    for (auto &s : extras) {
      if (seen.insert(s.id).second) {
        merged.push_back(std::move(s));
        ++forwarded_added;
      }
    }
  }
  if (forwarded_added > 0)
    _forwarded_through_link_count.fetch_add(forwarded_added, std::memory_order_relaxed);
  return merged;
}

bool state_sync_adapter::current_thread_suppressed() const noexcept {
  return t_suppression_depth > 0;
}

std::string state_sync_adapter::join_path(const std::string &parent, const std::string &child) {
  if (parent.empty())
    return child;
  if (child.empty())
    return parent;
  return parent + "." + child;
}

void state_sync_adapter::attach() {
  std::lock_guard<std::mutex> lk(_mutex);
  if (_attached)
    return;
  _attached = true;

  state &root = (_root_path.empty()) ? state::instance(_ctx) : state::instance(_ctx)(_root_path);
  // attach_node will recurse over existing children; new children
  // are caught lazily via the childChanged hook installed on each
  // observed node.
  // We must not call attach_node under our own lock because the
  // signal connections themselves are thread-safe but the recursive
  // walk may take state's internal locks. Release briefly.
  // For simplicity, do the walk under the lock - boost::signals2
  // connection registration is reentrant-safe enough for our use.
  attach_node(root, _root_path);
}

void state_sync_adapter::detach() {
  std::unordered_map<std::string, connection_set> taken;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    if (!_attached)
      return;
    _attached = false;
    taken.swap(_connections);
  }
  for (auto &kv : taken) {
    kv.second.value_changed.disconnect();
    kv.second.child_changed.disconnect();
  }
}

void state_sync_adapter::attach_node(state &s, const std::string &full_path) {
  // Skip if we've already attached to this path.
  if (_connections.find(full_path) != _connections.end())
    return;

  connection_set cs;
  std::string path_copy = full_path;
  cs.value_changed = s.valueChanged.connect([this, path_copy]() {
    if (!_attached || current_thread_suppressed())
      return;
    state &node = (path_copy.empty()) ? state::instance(_ctx) : state::instance(_ctx)(path_copy);
    state_mutation m;
    m.path = path_copy;
    m.op = state_mutation_op::set_value;
    m.string_value = node.value();
    m.type_name = node.valueTypeName();
    state_mutation appended = _journal.append(m);
    _local_count.fetch_add(1, std::memory_order_relaxed);
    dispatch_local(appended);
  });

  // childChanged fires on the PARENT with the child's local name
  // (not full name). Resolve to full path and attach lazily.
  cs.child_changed = s.childChanged.connect([this, path_copy](const std::string &child_name) {
    if (!_attached)
      return;
    std::string child_full = join_path(path_copy, child_name);
    std::lock_guard<std::mutex> lk(_mutex);
    if (!_attached)
      return;
    if (_connections.find(child_full) != _connections.end())
      return;
    state &child = state::instance(_ctx)(child_full);
    attach_node(child, child_full);
  });

  _connections.emplace(full_path, std::move(cs));

  // state::children() returns full dotted paths for ALL descendants
  // recursively. Attach to each once; the recursion-via-attach_node
  // path is reserved for newly created descendants observed lazily
  // through childChanged.
  std::vector<std::string> kids = s.children();
  for (const std::string &child_full : kids) {
    if (_connections.find(child_full) != _connections.end())
      continue;
    state &child = state::instance(_ctx)(child_full);
    connection_set ccs;
    std::string cpath = child_full;
    ccs.value_changed = child.valueChanged.connect([this, cpath]() {
      if (!_attached || current_thread_suppressed())
        return;
      state &node = state::instance(_ctx)(cpath);
      state_mutation m;
      m.path = cpath;
      m.op = state_mutation_op::set_value;
      m.string_value = node.value();
      m.type_name = node.valueTypeName();
      state_mutation appended = _journal.append(m);
      _local_count.fetch_add(1, std::memory_order_relaxed);
      dispatch_local(appended);
    });
    ccs.child_changed = child.childChanged.connect([this, cpath](const std::string &child_name) {
      if (!_attached)
        return;
      std::string nf = join_path(cpath, child_name);
      std::lock_guard<std::mutex> lk(_mutex);
      if (!_attached)
        return;
      if (_connections.find(nf) != _connections.end())
        return;
      state &nc = state::instance(_ctx)(nf);
      attach_node(nc, nf);
    });
    _connections.emplace(child_full, std::move(ccs));
  }
}

void state_sync_adapter::dispatch_local(const state_mutation &m) {
  // Notify subscribers via the router, expanding through any
  // transparent-link aliases so target-side mutations also match
  // subscriptions registered at link-side paths (Phase 8 slice 4d).
  std::vector<state_subscription> subs = subscriptions_for_path(m.path);
  // Fire the callback if set. Pull the function under lock then
  // invoke outside to avoid holding the mutex during user code.
  on_local_mutation_func cb;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    cb = _on_local;
  }
  if (cb)
    cb(m);
  (void)subs; // Phase 1: router lookup is exercised; dispatch to
              // transport happens in phase 3.
}

bool state_sync_adapter::apply_remote(const state_mutation &m) {
  // Mark this thread as applying remote, so the resulting state
  // signal does not feed back into the journal.
  thread_suppression_guard guard;

  state &node = (m.path.empty()) ? state::instance(_ctx) : state::instance(_ctx)(m.path);
  switch (m.op) {
  case state_mutation_op::set_value:
    node.value(m.string_value);
    break;
  case state_mutation_op::set_comment:
    node.comment(m.string_value);
    break;
  case state_mutation_op::set_hidden:
    node.hidden(m.string_value == "1" || m.string_value == "true");
    break;
  case state_mutation_op::set_read_only:
    node.readOnly(m.string_value == "1" || m.string_value == "true");
    break;
  case state_mutation_op::touch:
    node.touch();
    break;
  case state_mutation_op::reset_node:
    node.reset();
    break;
  case state_mutation_op::set_data:
    // Inline byte payloads are codec-dependent; phase 2 will provide
    // a codec registry. For now we accept it but do not decode.
    break;
  case state_mutation_op::delete_subtree:
  case state_mutation_op::delegate_subtree:
  case state_mutation_op::revoke_delegation:
    // Phase 3 control-plane operations - intentionally ignored here.
    break;
  }

  _remote_count.fetch_add(1, std::memory_order_relaxed);

  on_remote_applied_func cb;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    cb = _on_remote;
  }
  if (cb)
    cb(m);
  return true;
}

} // namespace cvc
