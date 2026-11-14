/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A sim_world's input trace (cvc/core/input_trace.h): its starting state as a trace's header,
// the world rebuilt from one, nav's verbs as trace events, a hash of its state, and the headless
// replay `cvc-cli trace verify` runs. docs/INPUT_TRACE.md has the format and the limits.
//
// Recording, in a host that builds a kinematic world (before its first step):
//
//   cvc::nav::trace_options o;
//   o.world_id = "grid:demo";
//   o.policies[0] = cvc::nav::policy_ref{};            // the world's own policy: the default
//   cvc::input_trace_recorder rec(app, app.world_clock());
//   world.pull(app, "");                                // its knobs into the tree, so the
//   rec.start(cvc::nav::trace_header(world, o), "session.trace"); // header's settings hold them
//   ... the frame loop: world.pull(app, "") before each step, and the host's verbs through
//       cvc::nav::trace_retarget(world, &rec, id, gx, gy) and the like ...
//   rec.finish(cvc::nav::state_hash(world));
//
// and checking it, in a fresh app: replay_trace(app, cvc::input_trace::read("session.trace")).
//
// v1 limits. Only the kinematic plant: a world with a vehicle plant (Jolt or the dynamic ones),
// terrain, a material stack, per-agent vehicle columns or a multi-disc body is refused (or, with
// allow_incomplete, recorded for a host that rebuilds the world itself). Jolt is deterministic
// only within one binary, so a Jolt trace could replay only on the build that recorded it. A
// sim_world has no save and restore: a replay seeks backwards by rebuilding the world and
// replaying from tick 0. An external force, yielding, routes and intents are the host's: it
// records them as its own events, or does not use them.

#ifndef __CVC_NAV_SIM_WORLD_TRACE_H__
#define __CVC_NAV_SIM_WORLD_TRACE_H__

#include <cstdint>
#include <cvc/core/input_trace.h>
#include <cvc/nav/policy_library.h>
#include <cvc/nav/sim_world.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace nav {

// The kind a header's world section names.
inline constexpr const char *kTraceWorldKind = "cvc.nav.sim_world/v1";

// The hash of a world's state: sha256 over its tick and, for every agent in index order, its id,
// its position, goal, heading and speed in the normalized frame (the float bits), its mode and its
// reached flag, as "sha256:<hex>". Two worlds hash alike when those are byte-identical.
std::string state_hash(const sim_world &w);

struct trace_options {
  std::string world_id;    // the host's name for the world
  std::uint64_t seed = 0;  // the host's seed
  std::uint64_t phase = 0; // the world steps on ticks t with t % (dt / fixed_dt) == phase
  std::string prefix;      // the prefix the host pulls the world's settings under (sim_world::pull)
  bool pull_every_step = true; // the host calls world.pull(app, prefix) before every step
  int num_threads = 1;         // step(num_threads), for the replay
  // Each policy's file, by policy index (sim_world::policy_of): what a replay loads. Policy 0 is
  // the world's own (the constructor's model), which the world cannot name, so the host states it
  // (an empty ref is the built-in default). A later policy given a library's "sha256:<hex>" key
  // needs its uri here; one with the key "default" needs nothing. A ref with a uri and no sha256
  // gets the file's.
  std::map<int, policy_ref> policies;
  bool embed_maps = true;        // the truth and prior maps in the header, which a rebuild needs
  bool allow_incomplete = false; // describe a world v1 cannot rebuild, for a host that rebuilds it
  std::string meta = "{}";       // the header's meta: a JSON object of the host's own
};

// A trace header for `w` as it is now, before its first step. Throws std::invalid_argument for a
// world v1 cannot rebuild, naming why, unless allow_incomplete (its world section then says
// "complete": false and why). Throws std::runtime_error for a policy file that can't be read.
input_trace_header trace_header(const sim_world &w, const trace_options &o);

// Rebuild the world a header describes, its policies through `lib`. `policy_files` replaces a
// policy's file by index, with no hash check: the second build of acceptance test 6, one trace
// replayed on another policy. Throws std::runtime_error for a header that is not a complete
// cvc.nav.sim_world/v1 world, or a policy that can't be loaded or doesn't match its hash.
sim_world world_from_trace(const input_trace_header &h, policy_library &lib,
                           const std::map<int, std::string> &policy_files = {});

// nav's verbs as trace events. Each applies the verb to the world and, with a recorder, records it
// at the clock's tick. Agents are named by id (sim_world::id_of); positions are normalized. False
// (nothing recorded) for an agent the world doesn't have.
//   nav.retarget      {"agent": id, "goal": [x, y]}
//   nav.set_pose      {"agent": id, "pose": [x, y, heading, speed]}
//   nav.override      {"agent": id, "command": [steer, accel]}   for the next step only
//   nav.add_obstacle  {"rect": [r0, r1, c0, c1]}
bool trace_retarget(sim_world &w, input_trace_recorder *rec, std::uint64_t id, float gx_n,
                    float gy_n);
bool trace_set_pose(sim_world &w, input_trace_recorder *rec, std::uint64_t id, float x_n, float y_n,
                    float heading, float speed = 0.0f);
bool trace_override(sim_world &w, input_trace_recorder *rec, std::uint64_t id, float steer,
                    float accel);
void trace_add_obstacle(sim_world &w, input_trace_recorder *rec, int r0, int r1, int c0, int c1);

// Apply one of the events above to a world. False for an event that is not nav's; throws
// std::runtime_error for one of nav's whose arguments are malformed or whose agent is gone.
bool apply_trace_event(sim_world &w, const input_trace_record &e);

struct replay_options {
  int num_threads = 0;                     // step(num_threads); 0: the header's
  std::map<int, std::string> policy_files; // as world_from_trace's
  bool keep_poses = false;                 // keep every step's poses in the result
};

struct replay_result {
  bool matched = false;               // every hash the trace holds matched, the end's among them
  bool has_end = false;               // the trace has an end record
  std::uint64_t ticks = 0;            // quanta replayed
  long steps = 0;                     // world steps
  std::string hash;                   // the replay's final state hash
  std::string expected;               // the trace's (its end's), "" with no end
  std::size_t compared = 0;           // hashes compared (checks and the end)
  std::size_t agreed = 0;             // of those, matched
  bool diverged = false;              // one differed
  std::uint64_t first_divergence = 0; // the tick of the first that differed
  std::vector<std::string> notes;     // what the replay changed: a policy file replaced
  // keep_poses: after every step, its tick and every agent's x, y, heading, speed (normalized).
  std::vector<std::uint64_t> pose_ticks;
  std::vector<float> poses; // [steps][n][4]
};

// Replay a trace headlessly, on `app`'s state tree and world clock (a fresh app: the trace's
// settings are written over whatever its tree holds): rebuild the world, play the inputs at their
// ticks, step the world where it fell due (pulling its settings first when the header says the
// host did), and compare its hash with each the trace holds. Throws std::runtime_error for a
// trace it cannot replay: not a nav world, an incomplete one, or an event that isn't nav's.
replay_result replay_trace(cvc::app &app, const input_trace &t, const replay_options &o = {});

// What `cvc-cli trace verify` prints.
std::string describe(const replay_result &r);

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_SIM_WORLD_TRACE_H__
