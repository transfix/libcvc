/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A state path looked up once and held, so a frame that reads or writes the same keys each time
// walks none of their paths. The key walks again only after its tree changed shape
// (cvc::state::shapeRevision: a node added or removed, a link made, moved or cleared), so what it
// holds is always what a new walk would find.
//
//     cvc::state_key tick("physics.stats.tick");
//     tick.write(app).value(std::to_string(n));   // made when absent, as root(path)
//     if (cvc::state *s = visible.read(app))        // through links, as findThroughLinks
//       on = s->value() == "1";                     // null while the path names nothing
//
// Each lookup is the cvc::state walk it is named after, with its semantics:
//   read   findThroughLinks: through every resolvable transparent link; null when absent;
//   write  childThroughLinks: where a write lands (a writable hole's target); made when absent;
//   find   findDescendant: no links; null when absent;
//   get    operator(): no links; made when absent.
// A key holds one lookup: asked for another kind, it walks again.
//
// From an app, the root (state::instance) is looked up only when the key walks: a key is for one
// app, and lives no longer than it. A key pins what it found, the node and every node on the way
// to it, so a node it holds is never freed under it, and a write to one (which signals its
// ancestors) never meets a freed parent. A key is one caller's (not thread-safe); the tree may
// change on any thread meanwhile, and the key walks again the next time it is asked.
#ifndef CVC_STATE_STATE_KEY_H
#define CVC_STATE_STATE_KEY_H

#include <atomic>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace cvc {

class state_key {
public:
  state_key() = default;
  explicit state_key(std::string path) : _path(std::move(path)) {}

  const std::string &path() const { return _path; }
  // How many times the key has walked. While it stands, the key holds the same node as at the
  // count's last value, so a caller that keeps the count and the node's revision() can tell,
  // without reading the node, that it was not written since.
  std::uint64_t walks() const { return _walks; }
  // Another path: the next lookup walks it. The same path keeps what was found.
  void set_path(const std::string &path) {
    if (path != _path) {
      _path = path;
      forget();
    }
  }
  // Let go of what was found: the next lookup walks again.
  void forget() {
    _node.reset();
    _pins.clear();
    _tree.reset();
    _root = nullptr;
    _app = nullptr;
    _kind = kind::none;
  }

  // The lookups from `root` (any node: the path is relative to it).
  state *read(state &root) { return held(root, kind::read) ? _node.get() : walk(root, kind::read); }
  state &write(state &root) { return held(root, kind::write) ? *_node : *walk(root, kind::write); }
  state *find(state &root) { return held(root, kind::find) ? _node.get() : walk(root, kind::find); }
  state &get(state &root) { return held(root, kind::get) ? *_node : *walk(root, kind::get); }

  // The same from the app's root.
  state *read(app &ctx) { return held(ctx, kind::read) ? _node.get() : walk(ctx, kind::read); }
  state &write(app &ctx) { return held(ctx, kind::write) ? *_node : *walk(ctx, kind::write); }
  state *find(app &ctx) { return held(ctx, kind::find) ? _node.get() : walk(ctx, kind::find); }
  state &get(app &ctx) { return held(ctx, kind::get) ? *_node : *walk(ctx, kind::get); }

private:
  enum class kind : unsigned char { none, read, write, find, get };

  bool standing() const { return _tree && _tree->load(std::memory_order_acquire) == _rev; }
  // What a walk from `root` would find: the same kind, from the same root, the shape unchanged.
  bool held(const state &root, kind k) const {
    return _kind == k && _root == &root && _tree == root._shape && standing();
  }
  bool held(const app &ctx, kind k) const { return _kind == k && _app == &ctx && standing(); }

  state *walk(app &ctx, kind k) {
    state *s = walk(state::instance(ctx), k);
    _app = &ctx;
    return s;
  }
  state *walk(state &root, kind k) {
    // The shape before the walk: a change during it is seen at the next lookup, which walks again.
    std::shared_ptr<std::atomic<std::uint64_t>> tree = root._shape;
    const std::uint64_t rev = tree->load(std::memory_order_acquire);
    std::vector<state::state_ptr> pins;
    state::state_ptr node;
    switch (k) {
    case kind::read:
      node = root.findThroughLinksShared(_path, &pins);
      break;
    case kind::write:
      node = root.childThroughLinksShared(_path, false, &pins);
      break;
    case kind::find:
      node = root.findDescendantShared(_path, &pins);
      break;
    case kind::get:
      node = root.sharedChild(_path, &pins);
      break;
    case kind::none:
      break;
    }
    _pins = std::move(pins);
    _node = std::move(node);
    _tree = std::move(tree);
    _rev = rev;
    _root = &root;
    _app = nullptr;
    _kind = k;
    ++_walks;
    return _node.get();
  }

  std::string _path;
  state::state_ptr _node;                            // what the walk found (read, find: or null)
  std::vector<state::state_ptr> _pins;               // every node on the way to it
  std::shared_ptr<std::atomic<std::uint64_t>> _tree; // its tree's shape count
  std::uint64_t _rev = 0;                            // that count before the walk
  const state *_root = nullptr;                      // where it walked from
  const app *_app = nullptr;                         // whose root that was, from an app
  kind _kind = kind::none;
  std::uint64_t _walks = 0;
};

} // namespace cvc

#endif // CVC_STATE_STATE_KEY_H
