/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef CVC_STATE_STATE_KEY_H
#define CVC_STATE_STATE_KEY_H

// A state path resolved once and then kept, for a reader or writer of the same keys every frame
// (a world's telemetry, a scene's bound values). Each look up of a path walks from the app's root:
// the root through the app's data map under a named mutex, then a lock and a map search per
// segment. A state_key does that walk again only after the tree's shape changed
// (state::structureEpoch: a node made or taken out, a link changed), so while the tree holds
// still a look costs an atomic load.
//
// It pins what it resolved (the node and every hop to it, as the pinned walks do: a pin_vector,
// so the node goes before its ancestors), so it never dangles: a node taken out of the tree is let
// go at the next look, which finds the path again.
// A miss is kept too: a path with no node gets one only by a node being made or a link changing,
// and either moves the epoch. One owner's: a key is not for use from two threads at once (the
// nodes it hands out are as thread-safe as any).

#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <string>
#include <utility>
#include <vector>

namespace cvc {

class state_key {
public:
  enum class view {
    physical,      // the path's own nodes: findDescendant (find) and operator() (get)
    through_links, // seen through transparent links: findThroughLinks and childThroughLinks
  };

  state_key() = default;
  state_key(app &ctx, std::string path, view v = view::physical)
      : _ctx(&ctx), _path(std::move(path)), _view(v) {}

  // Whether this key names a path (a default-constructed one names none).
  bool bound() const { return _ctx != nullptr; }
  app *context() const { return _ctx; }
  const std::string &path() const { return _path; }
  view kind() const { return _view; }

  // The node a read of the path sees, or nullptr while it has none.
  state *find() {
    state &r = root();
    const std::uint64_t now = r.structureEpoch();
    if (_looked && _foundAt == now)
      return _found.get();
    let_go_made(now);
    _found.reset(); // the node before its ancestors
    _foundPins.clear();
    _found = _view == view::through_links ? r.findThroughLinksShared(_path, &_foundPins)
                                          : r.findDescendantShared(_root, _path, &_foundPins);
    _foundAt = now;
    _looked = true;
    return _found.get();
  }

  // The node a write of the path lands on, made when absent. Throws as childThroughLinks does
  // (a writable transparent link that cannot be resolved), and keeps nothing then.
  state &get() {
    state &r = root();
    const std::uint64_t now = r.structureEpoch(); // before the walk: one that makes nodes moves it
    if (_made && _madeAt == now)
      return *_made;
    if (_looked && _foundAt != now) { // the other half is stale too: let it go
      _looked = false;
      _found.reset();
      _foundPins.clear();
    }
    _made.reset();
    _madePins.clear();
    state::state_ptr made = _view == view::through_links
                                ? r.childThroughLinksShared(_path, false, &_madePins)
                                : r.sharedChild(_root, _path, &_madePins);
    _made = std::move(made);
    _madeAt = now;
    return *_made;
  }

  // Lets go of what it resolved; the next look walks the path again.
  void reset() {
    _looked = false;
    _found.reset();
    _made.reset();
    _foundPins.clear();
    _madePins.clear();
  }

private:
  // What get() resolved, once the tree's shape has moved since: a node taken out is not kept
  // alive by a key that only reads.
  void let_go_made(std::uint64_t now) {
    if (_made && _madeAt != now) {
      _made.reset();
      _madePins.clear();
    }
  }

  state &root() {
    if (!_root)
      _root = state::instance(*_ctx).shared_from_this();
    return *_root;
  }

  app *_ctx = nullptr;
  std::string _path;
  view _view = view::physical;
  state::state_ptr _root;
  pin_vector _foundPins, _madePins; // before the nodes, so a key goes node first
  state::state_ptr _found, _made;
  std::uint64_t _foundAt = 0, _madeAt = 0;
  bool _looked = false; // find() has looked at _foundAt (and found _found, maybe nothing)
};

} // namespace cvc

#endif
