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

#include <algorithm>
#include <cvc/ariadne/conference/room_intrinsics.h> // room_participant_path
#include <cvc/ariadne/conference/room_write_gate.h> // may_write: a client program's writes
#include <cvc/ariadne/loader.h>                     // load_file, register_ari_block, LoadResult
#include <cvc/ariadne/server_document.h>
#include <cvc/ariadne/value.h>
#include <cvc/ariadne/world_rules.h> // register_world_block, find_world_block
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_change_journal.h> // state_mutation_op
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_exec/exceptions.h> // dsl_error
#include <cvc/state/state_exec/parser.h>     // parse_all
#include <memory>
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

namespace {
// client_programs: on|off, or a map of caps (which turns them on): "" or what is wrong.
std::string parse_client_programs(const Value &v, ClientPrograms &out,
                                  std::vector<std::string> *warnings) {
  out = ClientPrograms{};
  if (v.is_scalar()) {
    const std::string &s = v.scalar;
    if (s == "on" || s == "true" || s == "yes" || s == "1")
      out.on = true;
    else if (!(s == "off" || s == "false" || s == "no" || s == "0" || s.empty()))
      return "expects on or off, or a map of caps (steps, time, memory_mb), got '" + s + "'";
    return std::string();
  }
  if (!v.is_map())
    return "expects on or off, or a map of caps (steps, time, memory_mb)";
  out.on = true;
  for (const auto &[key, cap] : v.entries) {
    const double n = cap.as_double(-1.0);
    if (key != "steps" && key != "time" && key != "memory_mb") {
      if (warnings)
        warnings->push_back("server: client_programs: unknown key '" + key + "', ignored");
      continue;
    }
    if (!(n > 0.0))
      return key + " expects a positive number, got '" + (cap.is_scalar() ? cap.scalar : "") + "'";
    if (key == "steps")
      out.max_steps = static_cast<std::uint64_t>(n);
    else if (key == "time")
      out.max_time = n;
    else
      out.max_memory = static_cast<std::uint64_t>(n * 1024.0 * 1024.0);
  }
  return std::string();
}
} // namespace

std::string parse_server_block(const Value &content, ServerBlock &out,
                               std::vector<std::string> *warnings) {
  out = ServerBlock{};
  out.present = true;
  if (content.is_scalar() && content.empty())
    return std::string(); // a bare `server:`
  if (!content.is_map())
    return "server: expects a map (publish: [...])";
  for (const auto &[key, v] : content.entries) {
    if (key == "client_programs") {
      if (std::string why = parse_client_programs(v, out.client_programs, warnings); !why.empty())
        return "server: client_programs: " + why;
      continue;
    }
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

void register_server_block(cvc::app &app) {
  register_ari_block(app, "server", [](const Value &content, LoadResult &out) {
    out.extras.emplace_back("server", content);
  });
}

ServerDocument::ServerDocument(cvc::app &app, std::string room, std::string prefix)
    : app_(app), room_(std::move(room)), rt_(app, std::move(prefix)) {
  register_server_block(app);
  register_world_block(app);
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
  const LoadResult lr = load_file(app_, path);
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
  has_world_ = false;
  if (const Value *w = find_world_block(lr)) {
    world_ = *w;
    has_world_ = true;
  }
  blocks_.clear();
  for (const auto &[key, value] : lr.extras)
    blocks_.push_back(key);
  read_ = true;
  return std::string();
}

bool ServerDocument::has_block(const std::string &key) const {
  return std::find(blocks_.begin(), blocks_.end(), key) != blocks_.end();
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
  std::vector<std::string> warnings = rt_.load(path);
  loaded_ = true;
  return warnings;
}

namespace {
// A client program's writers (run_client_program), each held to the relay's bound-writes rule as
// the operation the rule weighs it as.
struct ClientWriter {
  const char *name;
  cvc::state_mutation_op op;
};
constexpr ClientWriter kClientWriters[] = {{"state-set", cvc::state_mutation_op::set_value},
                                           {"state-data-set", cvc::state_mutation_op::set_data},
                                           {"state-incr", cvc::state_mutation_op::set_value},
                                           {"state-cas", cvc::state_mutation_op::set_value},
                                           {"state-swap", cvc::state_mutation_op::set_value}};
// The native the program's forms are wrapped in, which keeps the last one's value.
constexpr const char *kClientResult = "__client-result";
constexpr const char *kDidNotComplete = "did not complete: ";
constexpr const char *kFailed = "failed: ";
} // namespace

void ServerDocument::run_client_program(const std::string &pid, const std::string &source,
                                        cvc::state_exec::value_t args,
                                        std::function<void(const ClientOutcome &)> done) {
  namespace se = cvc::state_exec;
  const auto refuse = [&done](const char *code, std::string message) {
    ClientOutcome o;
    o.code = code;
    o.message = std::move(message);
    if (done)
      done(o);
  };
  if (!block_.client_programs.on)
    return refuse("unavailable", "this relay runs no client programs (its document's server: "
                                 "block has no client_programs:)");
  if (!loaded_)
    return refuse("unavailable", "the relay's document has not loaded yet");
  if (pid.empty() || pid.find(cvc::state::SEPARATOR) != std::string::npos)
    return refuse("invalid", "a client program needs its sender's pid, one name");
  if (source.size() > kClientProgramMax)
    return refuse("invalid", "the program is " + std::to_string(source.size()) +
                                 " bytes, over the relay's " + std::to_string(kClientProgramMax));
  if (clients_running_.count(pid))
    return refuse("rate_limited", "this client's previous program is still running (one at a "
                                  "time)");
  // Parsed on its own first, so the wrapper below holds exactly its forms.
  try {
    (void)se::parse_all(source);
  } catch (const std::exception &e) {
    return refuse("invalid", std::string("the program does not parse: ") + e.what());
  }

  const std::string scope = conference::room_participant_path(room_, pid);
  const auto result = std::make_shared<se::value_t>();
  const auto gate = std::make_shared<conference::room_write_gate>(room_);
  LaneProcess p;
  // Its forms in one begin, whose value (the last form's) the wrapper keeps; the newline ends a
  // comment on its last line.
  p.program = std::string("(") + kClientResult + " (begin\n" + source + "\n))";
  p.name = "client " + pid;
  p.lane = ProgramLane::action;
  p.scope = scope;
  p.max_steps = block_.client_programs.max_steps;
  p.max_time = block_.client_programs.max_time;
  p.max_memory = block_.client_programs.max_memory;
  for (const char *name : read_lane_builtins())
    p.allow.emplace_back(name);
  for (const ClientWriter &w : kClientWriters)
    p.allow.emplace_back(w.name);
  p.extend = [gate, pid, scope, args = std::move(args), result](const se::environment_ptr &env,
                                                                se::intrinsics_context &) {
    env->set("args", args.is_nil() ? se::make_list() : args);
    // Each writer checks its path against the rule a client's own write meets on the transport,
    // then writes as it would.
    for (const ClientWriter &w : kClientWriters) {
      const se::value_t *v = env->lookup(w.name);
      const se::native_fn *f = v ? std::get_if<se::native_fn>(&v->v) : nullptr;
      if (!f)
        continue;
      env->set(w.name,
               se::value_t(se::native_fn([orig = *f, gate, pid, scope, name = std::string(w.name),
                                          op = w.op](se::call_args a) -> se::value_tag {
                 const std::string *path = a.empty() ? nullptr : std::get_if<std::string>(&a[0].v);
                 if (path && !gate->may_write(pid, scope + cvc::state::SEPARATOR + *path, op))
                   throw se::dsl_error("PermissionError",
                                       name + ": '" + *path +
                                           "' is not this client's to write (a key the relay "
                                           "owns, or not a plain path)",
                                       name);
                 return orig(a);
               })));
    }
    env->set(kClientResult, se::value_t(se::native_fn([result](se::call_args a) -> se::value_tag {
               *result = a.empty() ? se::value_t() : a[0];
               return se::value_t();
             })));
  };
  p.done = [this, pid, result, done](const std::string &how) {
    clients_running_.erase(pid);
    ClientOutcome o;
    const std::size_t dnc = std::char_traits<char>::length(kDidNotComplete);
    const std::size_t failed = std::char_traits<char>::length(kFailed);
    if (how.empty()) {
      o.ok = true;
      o.value = *result;
    } else if (how.compare(0, dnc, kDidNotComplete) == 0) {
      o.code = "resource_exhausted";
      o.message = how.substr(dnc);
    } else {
      o.code = "failed";
      o.message = how.compare(0, failed, kFailed) == 0 ? how.substr(failed) : how;
    }
    if (done)
      done(o);
  };
  try {
    rt_.runtime().start_process(std::move(p));
  } catch (const std::exception &e) {
    return refuse("invalid", std::string("the program does not start: ") + e.what());
  }
  clients_running_.insert(pid);
}

} // namespace ariadne
} // namespace cvc
