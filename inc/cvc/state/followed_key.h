/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A state key followed for what its data channel holds (docs/roadmap/DATA_FROM_STATE_TREE.md):
// a scene node that views data the app keeps on the tree (a document's data: entry, a filter's
// output, another node's published volume) asks each tick whether the data changed, and adopts
// them when they did.
//
//     cvc::followed_key f;
//     ... each tick:
//     boost::any d;
//     if (f.changed(app.root(), "data.head.volume", d, cvc::any_pointee<const cvc::volume>))
//       adopt(d);  // empty when the key holds nothing any more
//
// The path is followed through links, so a document's data.<name> reaches the store node its
// dataset is in, and a dataset loaded again under the same name (the link moved) is a change. So
// is a write to the node: its lastMod moves. cvc::state keeps no change counter, and a clock that
// ticks in milliseconds (Windows) leaves lastMod as it was for two writes in one tick, so the
// payload's type, and the object a pointer payload holds (`pointee`), tell a change too.
#ifndef CVC_STATE_FOLLOWED_KEY_H
#define CVC_STATE_FOLLOWED_KEY_H

#include <boost/any.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <cvc/state/state.h>
#include <memory>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <utility>

namespace cvc {

// Whether `d` holds a std::shared_ptr<T>; `id` is then the object it points at.
template <class T> bool any_pointee_of(const boost::any &d, const void *&id) {
  if (const auto *p = boost::any_cast<std::shared_ptr<T>>(&d)) {
    id = p->get();
    return true;
  }
  return false;
}

// The object a payload of std::shared_ptr<T> (for any of the Ts) points at, or null.
template <class... T> const void *any_pointee(const boost::any &d) {
  const void *id = nullptr;
  (void)(any_pointee_of<T>(d, id) || ...);
  return id;
}

class followed_key {
public:
  using pointee_fn = const void *(*)(const boost::any &);

  // Whether the node at `path` (absolute, from `root`, through links) holds data other than what
  // the last call saw: another node than before (the path edited, or a link moved), or the same
  // node written since. `payload` is then what it holds (empty when nothing). An empty path forgets
  // what was seen and says false; a path that names nothing yet says false, and is a change once
  // it does.
  bool changed(cvc::state &root, const std::string &path, boost::any &payload,
               pointee_fn pointee = nullptr) {
    if (path.empty()) {
      reset();
      return false;
    }
    cvc::state *s = root.findThroughLinks(path);
    if (!s) {
      _path = path; // not there yet: followed until it is
      _node.clear();
      return false;
    }
    const std::string node = s->fullName();
    const boost::posix_time::ptime mod = s->lastMod();
    boost::any d = s->data();
    const std::type_index type(d.type());
    const void *id = pointee ? pointee(d) : nullptr;
    if (path == _path && node == _node && mod == _mod && type == _type && id == _id)
      return false;
    _path = path;
    _node = node;
    _mod = mod;
    _type = type;
    _id = id;
    payload = std::move(d);
    return true;
  }

  // Forget what was seen: the next call with a path that names a node is a change.
  void reset() { *this = followed_key{}; }

  // The path last followed, and the node it reached ("" while it reaches none).
  const std::string &path() const { return _path; }
  const std::string &node() const { return _node; }

private:
  std::string _path, _node;
  boost::posix_time::ptime _mod;
  std::type_index _type = typeid(void);
  const void *_id = nullptr;
};

} // namespace cvc

#endif // CVC_STATE_FOLLOWED_KEY_H
