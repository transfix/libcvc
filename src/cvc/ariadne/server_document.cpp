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

// The document a relay runs — see inc/cvc/ariadne/server_document.h.

#include <cvc/ariadne/loader.h> // load_file, register_ari_block, LoadResult
#include <cvc/ariadne/server_document.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace cvc {
namespace ariadne {

namespace {

const char *const kRoomTree = "conf";

// The roots a document may not publish, and why.
struct Reserved {
  const char *root;
  const char *why;
};
constexpr Reserved kReserved[] = {
    {"conf", "the relay's room tree"},
    {"ui", "where documents keep their own state"},
    {"ariadne", "the host's settings"},
    {"state_exec", "the host's settings"},
    {"room", "the document's view of the relay's room"},
};

bool under(const std::string &path, const std::string &root) {
  return path.size() >= root.size() && path.compare(0, root.size(), root) == 0 &&
         (path.size() == root.size() || path[root.size()] == '.');
}

std::string join(const std::vector<std::string> &v) {
  std::string out;
  for (const std::string &s : v)
    out += (out.empty() ? "" : ", ") + s;
  return out;
}

const Value *find_server_block(const LoadResult &lr) {
  for (const auto &[key, content] : lr.extras)
    if (key == "server")
      return &content;
  return nullptr;
}

// "" or why `block` may not be published by a document bound at `prefix`: a namespace that is the
// document's own state, holds it or lies inside it (the default prefix is under ui, refused by
// check_published_namespace already; this covers any other).
std::string check_against_prefix(const ServerBlock &block, const std::string &prefix) {
  for (const std::string &ns : block.publish)
    if (under(ns, prefix) || under(prefix, ns))
      return "server: publish: '" + ns + "' overlaps the document's own state, " + prefix;
  return std::string();
}

// The nodes under `nodes` (their children too) that have a body:.
std::size_t count_bodies(const std::vector<SceneNode> &nodes) {
  std::size_t n = 0;
  for (const SceneNode &node : nodes)
    n += (node.body.present ? 1 : 0) + count_bodies(node.children);
  return n;
}

} // namespace

std::string check_published_namespace(const std::string &ns) {
  if (ns.empty())
    return "an empty name";
  std::size_t at = 0;
  for (;;) {
    const std::size_t dot = ns.find('.', at);
    const std::string seg = ns.substr(at, dot == std::string::npos ? dot : dot - at);
    if (!cvc::state::isValidStateName(seg))
      return "'" + ns + "' is not a dotted path of identifiers";
    if (dot == std::string::npos)
      break;
    at = dot + 1;
  }
  for (const Reserved &r : kReserved)
    if (under(ns, r.root))
      return "'" + ns + "' is under " + r.root + ", " + r.why;
  return std::string();
}

std::string parse_server_block(const Value &content, ServerBlock &out,
                               std::vector<std::string> *warnings) {
  out = ServerBlock{};
  out.present = true;
  if (content.is_scalar() && content.empty())
    return std::string(); // a bare `server:`
  if (!content.is_map())
    return "server: expects a map (publish: [...])";
  for (const auto &[key, v] : content.entries) {
    if (key != "publish") {
      if (warnings)
        warnings->push_back("server: unknown key '" + key + "', ignored");
      continue;
    }
    std::vector<std::string> names;
    if (v.is_scalar()) {
      if (!v.empty())
        names.push_back(v.scalar);
    } else if (v.is_seq()) {
      for (const Value &item : v.items) {
        if (!item.is_scalar())
          return "server: publish: expects names, got a " +
                 std::string(item.is_map() ? "map" : "list");
        names.push_back(item.scalar);
      }
    } else {
      return "server: publish: expects a list of names";
    }
    for (const std::string &ns : names) {
      if (const std::string why = check_published_namespace(ns); !why.empty())
        return "server: publish: " + why;
      bool repeat = false;
      for (const std::string &have : out.publish) {
        if (have == ns) {
          repeat = true;
          break;
        }
        if (under(ns, have) || under(have, ns))
          return "server: publish: '" + ns + "' and '" + have + "' overlap";
      }
      if (!repeat)
        out.publish.push_back(ns);
    }
  }
  return std::string();
}

void register_server_block() {
  static std::once_flag once;
  std::call_once(once, [] {
    register_ari_block("server", [](const Value &content, LoadResult &out) {
      out.extras.emplace_back("server", content);
    });
  });
}

ServerDocument::ServerDocument(cvc::app &app, std::string room, std::string prefix)
    : app_(app), room_(std::move(room)), rt_(app, std::move(prefix)) {
  register_server_block();
  rt_.runtime().set_relay_document(true); // requires: [relay]
  rt_.add_load_hook([this](LoadResult &lr, const std::string &path) {
    ServerBlock b;
    if (const Value *content = find_server_block(lr)) {
      std::vector<std::string> w;
      std::string error = parse_server_block(*content, b, &w);
      if (error.empty())
        error = check_against_prefix(b, rt_.prefix());
      if (!error.empty())
        throw std::runtime_error(path + ": " + error);
      for (std::string &s : w)
        lr.warnings.push_back(std::move(s));
    }
    if (read_ && b.publish != block_.publish)
      throw std::runtime_error(path + ": its server: block publishes [" + join(b.publish) +
                               "] now, but the relay was set up for [" + join(block_.publish) +
                               "]");
    block_ = std::move(b);
    plant();
    // The scene stays here instead of in the load, which would only say it was ignored. The
    // relay publishes it into the room (room_world_from_scene, WD1); its physics waits for the
    // world runtime.
    scene_ = std::move(lr.scene);
    lr.scene = Scene{};
    const std::size_t bodies = count_bodies(scene_.nodes);
    if (scene_.physics.present || bodies > 0)
      lr.warnings.push_back(path + ": its scene's physics (" + std::to_string(bodies) +
                            " bodies) is kept but not simulated yet: the headless world runtime "
                            "(ARIADNE_WORLDS.md §14) will make it the holon's");
  });
}

ServerDocument::~ServerDocument() = default;

std::string ServerDocument::read(const std::string &path, std::vector<std::string> *warnings) {
  if (!AppRuntime::have_yaml())
    return "this libcvc was built without yaml-cpp, so it cannot read a document";
  const LoadResult lr = load_file(path);
  if (!lr.ok)
    return lr.error.empty() ? "cannot load " + path : lr.error;
  ServerBlock b;
  if (const Value *content = find_server_block(lr)) {
    std::string error = parse_server_block(*content, b, warnings);
    if (error.empty())
      error = check_against_prefix(b, rt_.prefix());
    if (!error.empty())
      return path + ": " + error;
  }
  block_ = std::move(b);
  scene_ = lr.scene; // the world, for a relay that publishes it before the document runs
  read_ = true;
  return std::string();
}

void ServerDocument::step(std::uint64_t tick, double seconds) {
  cvc::state &root = cvc::state::instance(app_);
  const std::string &prefix = rt_.prefix();
  root(prefix + ".sim.tick").value(std::to_string(tick));
  root(prefix + ".sim.time").value(seconds);
  rt_.drain();
}

void ServerDocument::scope(cvc::state_cluster_shard &shard) const {
  shard.add_publish_prefix(kRoomTree);
  for (const std::string &ns : block_.publish)
    shard.add_publish_prefix(ns);
}

void ServerDocument::plant() {
  cvc::state &root = cvc::state::instance(app_);
  const std::string &prefix = rt_.prefix();
  // Each published namespace, writable: the target is made first, since a write through a link
  // that does not resolve throws.
  for (const std::string &ns : block_.publish) {
    root(ns);
    cvc::state &hole = root(prefix + cvc::state::SEPARATOR + ns);
    hole.linkTo(ns, cvc::state::link_mode::transparent);
    hole.setLinkWritable(true);
  }
  // The room, read-only: a write under it stays in the document's own subtree.
  cvc::state &view = root(prefix + cvc::state::SEPARATOR + "room");
  view.linkTo(std::string(kRoomTree) + ".rooms." + room_, cvc::state::link_mode::transparent);
  view.setLinkWritable(false);
}

std::vector<std::string> ServerDocument::load(const std::string &path) {
  if (!started_) {
    // The app's lazily made singletons, on this thread before a pool thread needs them. Here and
    // not in the constructor, so the host's settings (arisrv --set state_exec.policy=...) are on
    // the tree before the scheduler reads them.
    app_.exec_scheduler();
    app_.exec_async_host();
    rt_.register_quit();
    started_ = true;
  }
  return rt_.load(path);
}

} // namespace ariadne
} // namespace cvc
