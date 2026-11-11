/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License version 2.1 as published by the
  Free Software Foundation.
*/

#ifndef __CVC_FILE_HANDLER_REGISTRY_H__
#define __CVC_FILE_HANDLER_REGISTRY_H__

// cvc::file_handler_registry: one app's table of file handlers for one kind of data (volumes,
// geometry, models, images). Each cvc::app owns one per kind (app::volume_handlers(),
// geometry_handlers(), model_handlers(), image_handlers()), and its constructor fills them with
// the built-in handlers, so two apps never see each other's handlers and constructing an app
// changes no other app's table (VOLROVER-LIBRARY-PORTS-ROADMAP.md 1.3, P0a).
//
// A handler is any type with
//   id() const          -> a std::string naming it ("rawiv_io : v1.0");
//   extensions() const  -> the extensions it serves, spelled as its readers look them up
//                          (".rawiv" for volumes, geometry and models, "png" for images);
// held by boost::shared_ptr. Handlers are kept in registration order, which is the order the
// readers try them in. A handler whose id is already registered is not added again.
//
// Thread-safe: every member locks. Lookups return copies of the handler pointers, so a handler
// runs outside the lock and may be removed from the table while it runs.

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <cstddef>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace cvc {

template <class Handler> class file_handler_registry {
public:
  typedef boost::shared_ptr<Handler> ptr;
  typedef std::vector<ptr> handlers;
  typedef std::map<std::string, /* file extension */ handlers> handler_map;

  file_handler_registry() = default;
  file_handler_registry(const file_handler_registry &) = delete;
  file_handler_registry &operator=(const file_handler_registry &) = delete;

  // Add `h` after the handlers already registered. False (and nothing added) for a null handler
  // or one whose id is already registered.
  bool insert(const ptr &h) {
    if (!h)
      return false;
    const std::string id = h->id();
    std::lock_guard<std::mutex> lock(_mutex);
    for (const ptr &e : _order)
      if (e == h || e->id() == id)
        return false;
    _order.push_back(h);
    return true;
  }

  // Remove this handler object, or every handler with this id.
  void remove(const ptr &h) {
    std::lock_guard<std::mutex> lock(_mutex);
    _order.erase(std::remove(_order.begin(), _order.end(), h), _order.end());
  }
  void remove(const std::string &id) {
    std::lock_guard<std::mutex> lock(_mutex);
    _order.erase(
        std::remove_if(_order.begin(), _order.end(), [&](const ptr &e) { return e->id() == id; }),
        _order.end());
  }

  // The handlers serving `ext`, in registration order.
  handlers for_extension(const std::string &ext) const {
    handlers out;
    std::lock_guard<std::mutex> lock(_mutex);
    for (const ptr &e : _order) {
      const auto &exts = e->extensions();
      if (std::find(exts.begin(), exts.end(), ext) != exts.end())
        out.push_back(e);
    }
    return out;
  }

  // Every handler, in registration order.
  handlers all() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _order;
  }

  // The handlers by extension, each list in registration order.
  handler_map by_extension() const {
    handler_map out;
    std::lock_guard<std::mutex> lock(_mutex);
    for (const ptr &e : _order)
      for (const std::string &ext : e->extensions())
        out[ext].push_back(e);
    return out;
  }

  // Every extension some handler serves, sorted, each once.
  std::vector<std::string> extensions() const {
    std::set<std::string> s;
    std::lock_guard<std::mutex> lock(_mutex);
    for (const ptr &e : _order)
      for (const std::string &ext : e->extensions())
        s.insert(ext);
    return std::vector<std::string>(s.begin(), s.end());
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _order.size();
  }
  bool empty() const { return size() == 0; }

private:
  mutable std::mutex _mutex;
  handlers _order;
};

} // namespace cvc

#endif // __CVC_FILE_HANDLER_REGISTRY_H__
