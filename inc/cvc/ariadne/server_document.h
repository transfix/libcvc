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
//   ARIADNE_WORLDS.md WD1), after read() and before load(), from the copy kept here (scene()).
//   The load runs its world headless, as any runtime without a scene adapter does (SceneWorld,
//   runtime().scene_world(): the physics world and its bodies, the world sources, the vehicles of
//   world: vehicles:), stepped on the app's world clock at each step(). That physics world is the
//   document's, apart from the holon's, which builds the avatars' statics from the published
//   objects; the entities' poses stay under the document's prefix until the relay sends them.
// - The rules. Its world: block (world_rules.h) is kept too (world()): the avatars' archetypes,
//   policy and controllers, which the relay publishes as the room's rules sections
//   (conference::write_room_rules, ARIADNE_WORLDS.md WD2), with the scene's physics: gravity.
// - Clients' programs. With `client_programs:` in its server: block, the relay runs the DSL
//   programs its clients send ((room-run ...), room_calls.h) as this document's actions, each
//   confined to its sender's own participant subtree (run_client_program). With
//   `client_guests:`, a relay with a WebAssembly engine (arisrvx; use_client_guests) runs the
//   guests they send the same way (run_client_guest).
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace cvc {
class app;
class state_cluster_shard;
namespace ariadne {

struct Value;
class ScriptActivation;
class ScriptEngine;

// Whether a relay's clients may send it programs to run (client_programs:, ServerDocument::
// run_client_program) or WebAssembly guests (client_guests:, run_client_guest), and the caps those
// run under, which only lower the action lane's (ariadne.limits.action_max_*): 0 leaves the
// lane's. A guest's linear memory is capped by max_memory too (kClientGuestMemory without one).
struct ClientPrograms {
  bool on = false;
  std::uint64_t max_steps = 0;
  double max_time = 0.0;
  std::uint64_t max_memory = 0; // bytes
};

// One client's caps for one kind of code (ServerDocument::set_client_limits;
// docs/roadmap/ARISRV_IDENTITY.md §2.6, ID4a): the same fields, `on` false refusing that client
// the kind, and 0 leaving the document's cap.
using ClientLimits = ClientPrograms;

// A document's server: block.
struct ServerBlock {
  bool present = false;
  std::vector<std::string> publish; // the published namespaces, in document order, no repeats
  ClientPrograms client_programs;
  ClientPrograms client_guests;
};

// Read a server: block's content into `out`: "" or what is wrong. A key it does not know is a
// warning, pushed onto `warnings` when given. publish: takes a list of names, or one name.
// client_programs: and client_guests: each take on/off (true/false), or a map of the caps (steps,
// time in seconds, memory_mb), which turns them on.
std::string parse_server_block(const Value &content, ServerBlock &out,
                               std::vector<std::string> *warnings = nullptr);

// Whether `ns` may be published: "" or why not. A namespace is a dotted path of identifiers
// (cvc::state::isValidStateName), outside the relay's own roots: conf (the room tree), ui (where
// documents keep their own state, this one's too), arisrv (the relay's own records, which no client
// is sent), ariadne and state_exec (the host's settings), and room (the name of the document's room
// view).
std::string check_published_namespace(const std::string &ns);

// The server: block's parser (register_ari_block) on `app`; idempotent. ServerDocument
// registers it. Without it the loader ignores a server: block with a warning.
void register_server_block(cvc::app &app);

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
  // The world: block as read() found it, or null for none (see the note above).
  const Value *world() const { return has_world_ ? &world_ : nullptr; }
  // Whether read() found a host block named `key` (one a host registered: a scripting host's
  // python:), before anything runs: arisrvx starts its script thread for one.
  bool has_block(const std::string &key) const;

  AppRuntime &runtime() { return rt_; }
  const std::string &prefix() const { return rt_.prefix(); }
  const std::string &room() const { return room_; }

  // A client's program (ARIADNE_SCRIPTING_HOSTS.md §6.3, tier 2: S3): DSL source the relay's
  // client `pid` sent ((room-run ...)), run as one of this document's actions, with
  //   - its chroot at the client's own participant subtree, conf.rooms.<room>.participants.<pid>,
  //     and each write held to the relay's bound-writes rule there (room_write_gate::may_write):
  //     a value or a data payload, never on a key the relay owns (pose, device, ...);
  //   - a default-deny environment: the read lane's builtins (read_lane_builtins), the writers
  //     state-set, state-data-set, state-incr, state-cas and state-swap, and `args`, the list it
  //     was sent with. No spawn, messages, watches, locks, sleep, files, network or quit, and
  //     none of the document's functions;
  //   - the action lane's caps (ariadne.limits.action_max_*), lowered by client_programs:.
  // One program per client at a time. `done` gets the value of its last form, or an error:
  //   unavailable          the server: block has no client_programs:, or the document has not
  //                        loaded;
  //   forbidden            this client's limits refuse it programs (set_client_limits);
  //   rate_limited         that client's previous program is still running;
  //   invalid              it does not parse, or is longer than kClientProgramMax bytes;
  //   failed               it raised (the message is its report: a write it may not make is a
  //                        PermissionError);
  //   resource_exhausted   a cap stopped it.
  // A refusal calls `done` here; otherwise the step() whose drain sees the program end does.
  struct ClientOutcome {
    bool ok = false;
    cvc::state_exec::value_t value; // ok: the value of its last form
    std::string code, message;      // else: the error
  };
  static constexpr std::size_t kClientProgramMax = 64 * 1024;
  void run_client_program(const std::string &pid, const std::string &source,
                          cvc::state_exec::value_t args,
                          std::function<void(const ClientOutcome &)> done);

  // The engine that runs clients' WebAssembly guests (the host's wasm engine: arisrvx's), or null
  // for none. The host keeps it alive while this document runs.
  void use_client_guests(std::shared_ptr<ScriptEngine> engine) { guests_ = std::move(engine); }
  bool runs_client_guests() const { return guests_ != nullptr; }
  // A client's WebAssembly guest (§6.3, §8.6; G4): a module the relay's client `pid` sent
  // ((room-run module arg ...) with the module's bytes), whose export `entry` the relay calls with
  // the list of `args`, run as that client's program is (run_client_program): the same chroot, the
  // same default-deny environment (the guest's calls and state imports reach only it, and each
  // write meets room_write_gate::may_write), the action lane's caps lowered by client_guests:, and
  // one program or guest per client at a time. Its linear memory is capped at client_guests:'
  // memory_mb (or the client's own, set_client_limits), else kClientGuestMemory, and never above
  // ariadne.limits.wasm_max_memory. It gets
  // no WASI grant (§8.6.1), no hooks, and none of its exports but `entry`. `done` gets its result,
  // or an error as for a program: unavailable (no client_guests:, no engine, not loaded),
  // forbidden (this client's limits refuse it guests), invalid (not a binary module, or over
  // kClientGuestMax bytes), rate_limited, failed (it raised, or the
  // module doesn't compile, link or export `entry`) and resource_exhausted.
  // Its program starts once its module has compiled (on the app's pool, off the relay's loop), at
  // a step(): the wait costs it none of its caps, and one that takes over kClientGuestCompileWait
  // seconds ends resource_exhausted.
  static constexpr std::size_t kClientGuestMax = 1024 * 1024;
  static constexpr std::uint64_t kClientGuestMemory = std::uint64_t(16) << 20;
  static constexpr double kClientGuestCompileWait = 10.0;
  void run_client_guest(const std::string &pid, std::string module, const std::string &entry,
                        cvc::state_exec::value_t args,
                        std::function<void(const ClientOutcome &)> done);

  // Each client's limits (docs/roadmap/ARISRV_IDENTITY.md §2.6, ID4a). Without a function, every
  // client's programs run under client_programs:' caps and its guests under client_guests:'.
  // With one, each run_client_program and run_client_guest asks it for the caps that client's
  // code of that kind runs under, given the document's as they are in its server: block, and
  // takes its answer: `on` false refuses the client (forbidden), and 0 is the document's cap.
  // Without kClientLimitsAboveDocument a cap above the document's is held to it; with it, a cap may
  // rise to the action lane's. The action lane's caps (ariadne.limits.action_max_*) hold over both,
  // and a guest's memory stays under ariadne.limits.wasm_max_memory. It is asked at each start, so
  // a change applies to the client's next program or guest, not to one running. One that throws
  // refuses the start (unavailable). The host sets it (arisrv's main(), from its client registry);
  // it runs on the thread that calls run_client_*.
  enum class ClientKind { program, guest };
  // Decision D2 (ARISRV_IDENTITY.md §4): whether a client's limits may go above its document's
  // caps (up to the action lane's), or only lower them, the document's caps being set for a
  // server's biggest clients.
  static constexpr bool kClientLimitsAboveDocument = false;
  using ClientLimitsFn = std::function<ClientLimits(const std::string &pid, ClientKind kind,
                                                    const ClientLimits &document)>;
  void set_client_limits(ClientLimitsFn limits) { limits_ = std::move(limits); }

private:
  void plant();
  // Start `program` as client `pid`'s: run_client_program's process (its chroot, environment and
  // caps), with `bind` adding to its environment.
  void start_client(const std::string &pid, std::string program, const ClientPrograms &caps,
                    cvc::state_exec::value_t args,
                    std::function<void(const cvc::state_exec::environment_ptr &)> bind,
                    std::function<void(const ClientOutcome &)> done);
  // A client's guest's program, its activation ready, under `caps`; and the held ones that have
  // become ready (or waited too long), at each step().
  void start_guest(const std::string &pid, const std::string &entry,
                   std::shared_ptr<ScriptActivation> activation, const ClientLimits &caps,
                   std::function<void(const ClientOutcome &)> done);
  void start_held_guests();
  // The caps client `pid`'s code of `kind` runs under: `document`, or set_client_limits' answer.
  // "" with `caps` set, or the refusal's code with its message in `why`.
  std::string client_caps(const std::string &pid, ClientKind kind, const ClientLimits &document,
                          ClientLimits &caps, std::string &why) const;

  cvc::app &app_;
  std::string room_;
  AppRuntime rt_;
  ServerBlock block_;
  Scene scene_;
  Value world_;
  bool has_world_ = false;
  std::vector<std::string> blocks_;       // the host blocks read() found (LoadResult::extras' keys)
  bool read_ = false;                     // read() fixed block_
  bool started_ = false;                  // load() built the app's scheduler
  bool loaded_ = false;                   // a load() returned: client programs run
  std::set<std::string> clients_running_; // the pids whose program or guest has not ended
  std::shared_ptr<ScriptEngine> guests_;  // runs clients' guests (use_client_guests)
  ClientLimitsFn limits_;                 // each client's caps (set_client_limits), or none
  // Clients' guests whose activation isn't ready yet (their module compiles), in arrival order.
  struct HeldGuest {
    std::string pid, entry;
    std::shared_ptr<ScriptActivation> activation;
    ClientLimits caps;
    std::function<void(const ClientOutcome &)> done;
    std::chrono::steady_clock::time_point since;
  };
  std::vector<HeldGuest> held_guests_;
};

} // namespace ariadne
} // namespace cvc
