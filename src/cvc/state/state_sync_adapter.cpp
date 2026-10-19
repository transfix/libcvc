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
#include <cvc/state/state.h>
#include <cvc/state/state_data_hydrator.h>
#include <cvc/state/state_distributed_admin.h>
#include <cvc/state/state_sync_adapter.h>
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
  for (auto &kv : taken)
    kv.second.disconnect_all();
}

void state_sync_adapter::attach_node(state &s, const std::string &full_path) {
  // Skip if we've already attached to this path.
  if (_connections.find(full_path) != _connections.end())
    return;

  _connections.emplace(full_path, connect_node(s, full_path));

  // state::children() returns full dotted paths for ALL descendants
  // recursively. Attach to each once; the recursion-via-attach_node
  // path is reserved for newly created descendants observed lazily
  // through childChanged.
  std::vector<std::string> kids = s.children();
  for (const std::string &child_full : kids) {
    if (_connections.find(child_full) != _connections.end())
      continue;
    state &child = state::instance(_ctx)(child_full);
    _connections.emplace(child_full, connect_node(child, child_full));
  }
}

state_sync_adapter::connection_set state_sync_adapter::connect_node(state &node,
                                                                    const std::string &full_path) {
  connection_set cs;
  std::string path_copy = full_path;

  cs.value_changed = node.valueChanged.connect([this, path_copy]() {
    if (!_attached || current_thread_suppressed())
      return;
    state &n = (path_copy.empty()) ? state::instance(_ctx) : state::instance(_ctx)(path_copy);
    state_mutation m;
    m.path = path_copy;
    m.op = state_mutation_op::set_value;
    m.string_value = n.value();
    m.type_name = n.valueTypeName();
    state_mutation appended = _journal.append(m);
    _local_count.fetch_add(1, std::memory_order_relaxed);
    dispatch_local(appended);
  });

  // data() changes replicate as a set_data mutation carrying the codec-encoded payload. Skipped
  // silently when no codec registry is set or no codec is registered for the node's data() type
  // (value() + structure still replicate). Suppressed while applying a remote mutation.
  cs.data_changed = node.dataChanged.connect([this, path_copy]() {
    if (!_attached || current_thread_suppressed() || !_codecs)
      return;
    state &n = (path_copy.empty()) ? state::instance(_ctx) : state::instance(_ctx)(path_copy);
    const std::string tn = n.dataTypeName();
    if (tn.empty())
      return;
    state_mutation m;
    m.path = path_copy;
    m.op = state_mutation_op::set_data;
    m.type_name = tn;
    try {
      m.payload = state_payload::inline_data(_codecs->encode(tn, n.data()));
    } catch (const std::exception &) {
      return; // no codec registered for this data() type, or encode failed -> not replicated
    }
    state_mutation appended = _journal.append(m);
    _local_count.fetch_add(1, std::memory_order_relaxed);
    dispatch_local(appended);
  });

  // Node metadata (comment / hidden / readOnly) replicates as dedicated mutations; apply_remote
  // mirrors each onto the peer node. No codec is involved. Suppressed while applying a remote
  // mutation so an applied change does not loop back into the journal. hidden/readOnly carry
  // "1"/"0" to match apply_remote's boolean decode.
  cs.comment_changed = node.commentChanged.connect([this, path_copy]() {
    if (!_attached || current_thread_suppressed())
      return;
    state &n = (path_copy.empty()) ? state::instance(_ctx) : state::instance(_ctx)(path_copy);
    state_mutation m;
    m.path = path_copy;
    m.op = state_mutation_op::set_comment;
    m.string_value = n.comment();
    state_mutation appended = _journal.append(m);
    _local_count.fetch_add(1, std::memory_order_relaxed);
    dispatch_local(appended);
  });

  cs.hidden_changed = node.hiddenChanged.connect([this, path_copy]() {
    if (!_attached || current_thread_suppressed())
      return;
    state &n = (path_copy.empty()) ? state::instance(_ctx) : state::instance(_ctx)(path_copy);
    state_mutation m;
    m.path = path_copy;
    m.op = state_mutation_op::set_hidden;
    m.string_value = n.hidden() ? "1" : "0";
    state_mutation appended = _journal.append(m);
    _local_count.fetch_add(1, std::memory_order_relaxed);
    dispatch_local(appended);
  });

  cs.read_only_changed = node.readOnlyChanged.connect([this, path_copy]() {
    if (!_attached || current_thread_suppressed())
      return;
    state &n = (path_copy.empty()) ? state::instance(_ctx) : state::instance(_ctx)(path_copy);
    state_mutation m;
    m.path = path_copy;
    m.op = state_mutation_op::set_read_only;
    m.string_value = n.readOnly() ? "1" : "0";
    state_mutation appended = _journal.append(m);
    _local_count.fetch_add(1, std::memory_order_relaxed);
    dispatch_local(appended);
  });

  // A property replicates as set_property: its key in type_name, its value in string_value
  // ("" removes it on the peer, as it did here).
  cs.property_changed = node.propertyChanged.connect([this, path_copy](const std::string &key) {
    if (!_attached || current_thread_suppressed())
      return;
    state &n = (path_copy.empty()) ? state::instance(_ctx) : state::instance(_ctx)(path_copy);
    state_mutation m;
    m.path = path_copy;
    m.op = state_mutation_op::set_property;
    m.type_name = key;
    m.string_value = n.property(key);
    state_mutation appended = _journal.append(m);
    _local_count.fetch_add(1, std::memory_order_relaxed);
    dispatch_local(appended);
  });

  // childChanged fires on the PARENT with the child's local name (not full name). Resolve to full
  // path and attach lazily. Takes _mutex because it mutates _connections from a signal thread.
  cs.child_changed = node.childChanged.connect([this, path_copy](const std::string &child_name) {
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

  return cs;
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
  case state_mutation_op::set_property:
    node.property(m.type_name, m.string_value);
    break;
  case state_mutation_op::touch:
    node.touch();
    break;
  case state_mutation_op::reset_node:
    node.reset();
    break;
  case state_mutation_op::set_data:
    // Decode the inline payload with the registered codec for its type and set node.data().
    // (We are inside the thread_suppression_guard above, so the resulting dataChanged does not
    // loop back into the journal.) A missing codec or a decode failure leaves data() untouched and
    // is swallowed — value() and child structure still converge.
    if (_codecs && m.payload.kind == state_payload_kind::inline_bytes && !m.type_name.empty()) {
      try {
        node.data(_codecs->decode(m.type_name, m.payload.inline_bytes));
      } catch (const std::exception &) {
        // no codec for this type on this build, or decode failed: diagnostic hook goes here.
      }
    } else if (_hydrator && m.payload.kind == state_payload_kind::blob && !m.type_name.empty()) {
      // Large payload offloaded to the blob store on the sender. Hand the blob ref to the
      // hydrator, which loads/fetches the chunk manifest, decodes via the codec registry, and
      // installs the result on this node via node.data(). When the blob store already holds the
      // chunks (shared store, or an inproc/ipc fetch that completes inline) this runs to
      // completion synchronously on THIS thread, so the install stays inside the
      // thread_suppression_guard above and does not re-enter the journal. When chunks must be
      // pulled asynchronously the hydrator returns `pending` and installs later off this thread;
      // re-checking shard conflict order on that late install is a follow-up for the async
      // (gRPC) transport path and is not yet exercised by the inproc/ipc transports.
      _hydrator->request(m.path, m.payload.blob.digest, m.type_name, &node);
    }
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
