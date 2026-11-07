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

// ServerDocument: the document a relay runs (arisrv --doc server.ari; ARIADNE_SCRIPTING_HOSTS.md
// §6.1 and §6.5.3). A headless AppRuntime over the relay's own app, with no backend and no scene,
// for room rules, bots and checks written in the DSL, under the DSL's caps and chroot.
//
// The document binds under ui.docs.arisrv, which the relay never replicates: clients get what the
// document publishes, never the document or its own state. Its server: block names the namespaces
// it publishes:
//
//   server:
//     publish: [cyprov]       # replicated to clients; only the server writes under them
//
// - Writes. Each published namespace is a writable hole in the document's chroot (ARIADNE.md
//   §7.4): `cyprov` there is the app tree's `cyprov`, so (state-set "cyprov.status" "up") and
//   (state-set-tree "cyprov.jobs" ...) write what clients receive. A client cannot write there:
//   the relay's bound writes (room_write_gate) refuse every path outside the client's own
//   participant.
// - Reads. `room` is a read-only hole on the relay's room tree, conf.rooms.<room>.
// - What the relay sends. scope() limits the relay's shard to the room tree and the published
//   namespaces (state_cluster_shard::add_publish_prefix), for its live writes and its snapshots.
// - What reaches through a hole. Reads, state-set, state-set-tree and the atomics do; state-delete
//   and state-watch act on the physical path (ARIADNE.md §7.4). Remove published entries by
//   rewriting their parent with state-set-tree.
// - The clock. A relay calls step(tick, seconds) once per room step, the room rules' cadence. It
//   publishes the relay's world clock (the avatar holon's, 120 Hz, never backwards) to the
//   document at sim.tick and sim.time, the keys a scene's clock: publishes in a window, so its
//   programs count world ticks, not steps.
// - requires: [relay] holds in this runtime only (Runtime::set_relay_document).
// - Several, later. Each instance is one room's runtime under its own prefix, so a relay with
//   many rooms can build one each.
// - The scene. A document whose scene: has nodes or lights is the room's world: the relay
//   publishes it into conf.rooms.<room>.scene (conference::room_world_from_scene,
//   ARIADNE_WORLDS.md WD1), after read() and before load(). Its physics is not simulated yet: the
//   parsed scene is kept here (scene()) for the world runtime that will run it as the holon's one
//   physics world (ARIADNE_WORLDS.md §14), and its load says so.
#pragma once

#include <cstdint>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/scene.h>
#include <string>
#include <vector>

namespace cvc {
class app;
class state_cluster_shard;
namespace ariadne {

struct Value;

// A document's server: block.
struct ServerBlock {
  bool present = false;
  std::vector<std::string> publish; // the published namespaces, in document order, no repeats
};

// Read a server: block's content into `out`: "" or what is wrong. A key it does not know is a
// warning, pushed onto `warnings` when given. publish: takes a list of names, or one name.
std::string parse_server_block(const Value &content, ServerBlock &out,
                               std::vector<std::string> *warnings = nullptr);

// Whether `ns` may be published: "" or why not. A namespace is a dotted path of identifiers
// (cvc::state::isValidStateName), outside the relay's own roots: conf (the room tree), ui (where
// documents keep their own state, this one's too), ariadne and state_exec (the host's settings),
// and room (the name of the document's room view).
std::string check_published_namespace(const std::string &ns);

// The server: block's parser (register_ari_block), process-wide; idempotent. ServerDocument
// registers it. Without it the loader ignores a server: block with a warning.
void register_server_block();

class ServerDocument {
public:
  static constexpr const char *kPrefix = "ui.docs.arisrv";

  // A document runtime over `app` for the relay of `room`. Nothing runs, and the app's scheduler
  // is not built, until load().
  ServerDocument(cvc::app &app, std::string room, std::string prefix = kPrefix);
  ~ServerDocument();
  ServerDocument(const ServerDocument &) = delete;
  ServerDocument &operator=(const ServerDocument &) = delete;

  // Read `path`'s server: block without running anything, so the relay is set up before it
  // starts: "" or what is wrong (a document that does not parse, a bad server: block).
  std::string read(const std::string &path, std::vector<std::string> *warnings = nullptr);
  const ServerBlock &block() const { return block_; }

  // The relay's shard sends the room tree (conf) and the published namespaces only.
  void scope(cvc::state_cluster_shard &shard) const;

  // Load and run the document (its init:), with the published namespaces and the room view
  // planted in its chroot first. The first load builds the app's scheduler on this thread, as
  // pycvc.AriApp does, and gives the document (quit [code]). Returns the load's warnings; throws
  // std::runtime_error on a fatal load, and when read() saw a different server: block (the shard
  // was scoped by it).
  std::vector<std::string> load(const std::string &path);

  // One relay step at world-clock `tick` (`seconds` of world time): sim.tick and sim.time, then
  // the document's queued actions and resident programs (AppRuntime::drain).
  void step(std::uint64_t tick, double seconds);
  // (quit [code]) asks the relay to stop.
  bool should_close() const { return rt_.should_close(); }
  int exit_code() const { return rt_.exit_code(); }

  // The scene: block as read() and then each load() parsed it: nodes, bodies and physics: (see
  // the note above).
  const Scene &scene() const { return scene_; }

  AppRuntime &runtime() { return rt_; }
  const std::string &prefix() const { return rt_.prefix(); }
  const std::string &room() const { return room_; }

private:
  void plant();

  cvc::app &app_;
  std::string room_;
  AppRuntime rt_;
  ServerBlock block_;
  Scene scene_;
  bool read_ = false;    // read() fixed block_
  bool started_ = false; // load() built the app's scheduler
};

} // namespace ariadne
} // namespace cvc
