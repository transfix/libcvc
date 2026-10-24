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

#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/conference/room_intrinsics.h> // room_participant_path, room_roster
#include <cvc/ariadne/holon.h>                      // cvc::ariadne::holon (server_pump)
#include <cvc/ariadne/holon_sync.h>
#include <cvc/core/app.h>
#include <cvc/physics/types.h> // vec3, quat
#include <cvc/state/state.h>   // cvc::state, SEPARATOR, findDescendant
#include <initializer_list>
#include <string>

namespace cvc {
namespace ariadne {

namespace {

// Round-trip-safe double text (17 significant digits exactly recovers a binary64),
// portable across every CI toolchain (no std::to_chars floating-point dependency).
std::string fmt(double d) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", d);
  return std::string(buf);
}

// A leaf path <participant>.<segs...>, joined with cvc::state::SEPARATOR (never an
// embedded literal separator, so this holds whatever SEPARATOR is).
std::string leaf(const std::string &room, const std::string &pid,
                 std::initializer_list<const char *> segs) {
  std::string s = conference::room_participant_path(room, pid);
  for (const char *seg : segs) {
    s += cvc::state::SEPARATOR;
    s += seg;
  }
  return s;
}

// touch-before-value, so a just-created node's first write is not lost to the
// distributed-state adapter's attach race (the room tree's set_replicated idiom).
void set_replicated(cvc::app &app, const std::string &path, const std::string &val) {
  cvc::state &n = app.root()(path);
  n.touch();
  n.value(val);
}

double read_double(cvc::app &app, const std::string &path) {
  cvc::state *n = app.root().findDescendant(path);
  return n ? std::strtod(n->value().c_str(), nullptr) : 0.0;
}
bool node_present(cvc::app &app, const std::string &path) {
  return app.root().findDescendant(path) != nullptr;
}

} // namespace

void write_pose(cvc::app &app, const std::string &room, const std::string &pid,
                const cvc::physics::body_state &s, std::uint64_t server_tick) {
  set_replicated(app, leaf(room, pid, {"pose", "pos", "x"}), fmt(s.position.x));
  set_replicated(app, leaf(room, pid, {"pose", "pos", "y"}), fmt(s.position.y));
  set_replicated(app, leaf(room, pid, {"pose", "pos", "z"}), fmt(s.position.z));
  set_replicated(app, leaf(room, pid, {"pose", "orient", "w"}), fmt(s.rotation.w));
  set_replicated(app, leaf(room, pid, {"pose", "orient", "x"}), fmt(s.rotation.x));
  set_replicated(app, leaf(room, pid, {"pose", "orient", "y"}), fmt(s.rotation.y));
  set_replicated(app, leaf(room, pid, {"pose", "orient", "z"}), fmt(s.rotation.z));
  set_replicated(app, leaf(room, pid, {"pose", "vel", "lin", "x"}), fmt(s.linear_velocity.x));
  set_replicated(app, leaf(room, pid, {"pose", "vel", "lin", "y"}), fmt(s.linear_velocity.y));
  set_replicated(app, leaf(room, pid, {"pose", "vel", "lin", "z"}), fmt(s.linear_velocity.z));
  set_replicated(app, leaf(room, pid, {"pose", "vel", "ang", "x"}), fmt(s.angular_velocity.x));
  set_replicated(app, leaf(room, pid, {"pose", "vel", "ang", "y"}), fmt(s.angular_velocity.y));
  set_replicated(app, leaf(room, pid, {"pose", "vel", "ang", "z"}), fmt(s.angular_velocity.z));
  set_replicated(app, leaf(room, pid, {"pose", "server_tick"}),
                 std::to_string(static_cast<unsigned long long>(server_tick)));
}

bool read_pose(cvc::app &app, const std::string &room, const std::string &pid,
               cvc::physics::body_state &out, std::uint64_t &out_server_tick) {
  if (!node_present(app, leaf(room, pid, {"pose", "pos", "x"})))
    return false; // the pose subtree has not been written/replicated yet
  out.position = {read_double(app, leaf(room, pid, {"pose", "pos", "x"})),
                  read_double(app, leaf(room, pid, {"pose", "pos", "y"})),
                  read_double(app, leaf(room, pid, {"pose", "pos", "z"}))};
  out.rotation = {read_double(app, leaf(room, pid, {"pose", "orient", "w"})),
                  read_double(app, leaf(room, pid, {"pose", "orient", "x"})),
                  read_double(app, leaf(room, pid, {"pose", "orient", "y"})),
                  read_double(app, leaf(room, pid, {"pose", "orient", "z"}))};
  out.linear_velocity = {read_double(app, leaf(room, pid, {"pose", "vel", "lin", "x"})),
                         read_double(app, leaf(room, pid, {"pose", "vel", "lin", "y"})),
                         read_double(app, leaf(room, pid, {"pose", "vel", "lin", "z"}))};
  out.angular_velocity = {read_double(app, leaf(room, pid, {"pose", "vel", "ang", "x"})),
                          read_double(app, leaf(room, pid, {"pose", "vel", "ang", "y"})),
                          read_double(app, leaf(room, pid, {"pose", "vel", "ang", "z"}))};
  out.active = true;
  cvc::state *t = app.root().findDescendant(leaf(room, pid, {"pose", "server_tick"}));
  out_server_tick =
      t ? static_cast<std::uint64_t>(std::strtoull(t->value().c_str(), nullptr, 10)) : 0;
  return true;
}

void write_intent(cvc::app &app, const std::string &room, const std::string &pid,
                  const avatar_intent &intent) {
  set_replicated(app, leaf(room, pid, {"intent", "move", "x"}), fmt(intent.move.x));
  set_replicated(app, leaf(room, pid, {"intent", "move", "y"}), fmt(intent.move.y));
  set_replicated(app, leaf(room, pid, {"intent", "move", "z"}), fmt(intent.move.z));
}

bool read_intent(cvc::app &app, const std::string &room, const std::string &pid,
                 avatar_intent &out) {
  if (!node_present(app, leaf(room, pid, {"intent", "move", "x"})))
    return false;
  out.move = {read_double(app, leaf(room, pid, {"intent", "move", "x"})),
              read_double(app, leaf(room, pid, {"intent", "move", "y"})),
              read_double(app, leaf(room, pid, {"intent", "move", "z"}))};
  return true;
}

void server_pump(holon &h, cvc::app &app, const std::string &room, double wall_dt) {
  // The present participants, straight from the replicated room tree.
  const std::vector<conference::room_participant> roster = conference::room_roster(app, room);
  // Add any newcomer, then feed each one's latest proposed intent to the holon.
  for (const conference::room_participant &p : roster) {
    if (!h.has_participant(p.pid))
      h.add_participant(p.pid);
    avatar_intent in;
    if (read_intent(app, room, p.pid, in))
      h.set_intent(p.pid, in);
  }
  // One authoritative frame.
  h.advance(wall_dt);
  // Publish each participant's authoritative pose (these are local writes on the
  // relay's shard; the caller fans them to the clients).
  for (const conference::room_participant &p : roster)
    write_pose(app, room, p.pid, h.state(p.pid), h.tick());
}

} // namespace ariadne
} // namespace cvc
