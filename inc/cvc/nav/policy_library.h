/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// policy_library.h — nav@1's policies, loaded once per content hash.
//
// nav@1 is the navigator (docs/roadmap/ARIADNE_WORLDS.md §5): sim_world's planner over an
// entity's plant. Its params name a policy: a trained .cvcnav by URI and content hash, or none
// for coef_mlp::default_biased(). Entities naming the same policy share one loaded copy, so a
// policy per entity costs nothing extra; sim_world::set_policy takes the copy and its key, and
// runs each policy as one batch over the agents that use it.

#ifndef __CVC_NAV_POLICY_LIBRARY_H__
#define __CVC_NAV_POLICY_LIBRARY_H__

#include <cstddef>
#include <cvc/nav/coef_mlp.h>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace cvc {
namespace nav {

// A nav@1 policy, as an entity's controller params name it.
struct policy_ref {
  std::string uri;    // a .cvcnav: a path or a file:// URI; empty for the default policy
  std::string sha256; // its bytes' SHA-256, 64 hex digits; empty to take the file as it is
};

class policy_library {
public:
  struct entry {
    std::shared_ptr<const coef_mlp> model;
    std::string key; // "sha256:<hex>", or "default"; sim_world::set_policy's key
  };

  // The policy `ref` names, loaded the first time its content hash is asked for and shared after.
  // An empty ref is coef_mlp::default_biased(), key "default". With a sha256, a policy already
  // loaded under it is returned without reading the file, and a file whose bytes hash otherwise
  // is refused. Throws std::runtime_error for a file that can't be read or parsed, a hash that
  // doesn't match, or a URI that isn't a path or file://; std::invalid_argument for a sha256 that
  // isn't 64 hex digits, or a sha256 with no uri. Thread-safe.
  entry get(const policy_ref &ref);

  // How many policies are loaded, and forget them all (copies already handed out live on).
  std::size_t size() const;
  void clear();

private:
  mutable std::mutex mu_;
  std::map<std::string, std::shared_ptr<const coef_mlp>> by_key_;
};

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_POLICY_LIBRARY_H__
