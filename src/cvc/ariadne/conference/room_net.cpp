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
#include <cvc/ariadne/conference/room_intrinsics.h> // room_participant_path
#include <cvc/ariadne/conference/room_net.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <initializer_list>
#include <string>

namespace cvc {
namespace ariadne {
namespace conference {

namespace {

std::string sub(const std::string &base, std::initializer_list<const char *> segs) {
  std::string s = base;
  for (const char *seg : segs) {
    s += cvc::state::SEPARATOR;
    s += seg;
  }
  return s;
}

// One decimal is plenty for display, and an unchanged value writes nothing new to replicate.
std::string fmt(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.1f", v);
  return std::string(buf);
}

// touch-before-value, so a just-created node's first write is not lost to the distributed-state
// adapter's attach race (the room tree's set_replicated idiom). A value that has not changed is not
// rewritten, so a steady link does not add replication traffic.
void set_replicated(cvc::app &app, const std::string &path, const std::string &val) {
  cvc::state &n = app.root()(path);
  if (n.value() == val)
    return;
  n.touch();
  n.value(val);
}

// Changed-only with ONE journaled mutation per change, for a leaf that flips often. touch() fires
// valueChanged before value() fires it again, so set_replicated's touch would first re-send the
// leaf's OLD value; it is only needed while the leaf is new or still empty (the attach race above).
// A valued leaf is already observed: its first value was set after the touch that attached it, or
// it arrived through replication (applying a remote node attaches it too).
// FORCE re-sends an unchanged value (one touch) rather than writing nothing.
void set_changed_once(cvc::app &app, const std::string &path, const std::string &val,
                      bool force = false) {
  cvc::state *n = app.root().findDescendant(path);
  const std::string cur = n ? n->value() : std::string();
  if (n && cur == val) {
    if (force && !cur.empty())
      n->touch();
    return;
  }
  if (!cur.empty()) {
    n->value(val);
    return;
  }
  cvc::state &m = app.root()(path);
  m.touch();
  m.value(val);
}

double read_double(cvc::app &app, const std::string &path, double def) {
  cvc::state *n = app.root().findDescendant(path);
  return n && !n->value().empty() ? std::strtod(n->value().c_str(), nullptr) : def;
}

void write_sample(cvc::app &app, const std::string &base, const net_sample &s) {
  set_replicated(app, sub(base, {"rtt_ms"}), fmt(s.rtt_ms));
  set_replicated(app, sub(base, {"in_kbps"}), fmt(s.in_kbps));
  set_replicated(app, sub(base, {"out_kbps"}), fmt(s.out_kbps));
}

bool read_sample(cvc::app &app, const std::string &base, net_sample &out) {
  if (!app.root().findDescendant(sub(base, {"rtt_ms"})))
    return false;
  out.rtt_ms = read_double(app, sub(base, {"rtt_ms"}), -1.0);
  out.in_kbps = read_double(app, sub(base, {"in_kbps"}), -1.0);
  out.out_kbps = read_double(app, sub(base, {"out_kbps"}), -1.0);
  return true;
}

std::string participant_net(const std::string &room, const std::string &pid) {
  return sub(room_participant_path(room, pid), {"net"});
}

std::string server_net_path(const std::string &room) {
  return sub("conf", {"rooms", room.c_str(), "server", "net"});
}

std::string participant_talking(const std::string &room, const std::string &pid) {
  return sub(room_participant_path(room, pid), {"voice", "talking"});
}

} // namespace

void write_participant_net(cvc::app &app, const std::string &room, const std::string &pid,
                           const net_sample &s) {
  write_sample(app, participant_net(room, pid), s);
}

bool read_participant_net(cvc::app &app, const std::string &room, const std::string &pid,
                          net_sample &out) {
  return read_sample(app, participant_net(room, pid), out);
}

void write_relay_net(cvc::app &app, const std::string &room, const std::string &pid,
                     const net_sample &s) {
  write_sample(app, sub(participant_net(room, pid), {"relay"}), s);
}

bool read_relay_net(cvc::app &app, const std::string &room, const std::string &pid,
                    net_sample &out) {
  return read_sample(app, sub(participant_net(room, pid), {"relay"}), out);
}

void write_server_net(cvc::app &app, const std::string &room, const server_net &s) {
  const std::string base = server_net_path(room);
  set_replicated(app, sub(base, {"in_kbps"}), fmt(s.in_kbps));
  set_replicated(app, sub(base, {"out_kbps"}), fmt(s.out_kbps));
  set_replicated(app, sub(base, {"links"}), std::to_string(s.links));
}

bool read_server_net(cvc::app &app, const std::string &room, server_net &out) {
  const std::string base = server_net_path(room);
  if (!app.root().findDescendant(sub(base, {"links"})))
    return false;
  out.in_kbps = read_double(app, sub(base, {"in_kbps"}), 0.0);
  out.out_kbps = read_double(app, sub(base, {"out_kbps"}), 0.0);
  out.links = static_cast<int>(read_double(app, sub(base, {"links"}), 0.0));
  return true;
}

void write_participant_voice(cvc::app &app, const std::string &room, const std::string &pid,
                             bool talking, bool force) {
  set_changed_once(app, participant_talking(room, pid), talking ? "1" : "0", force);
}

bool read_participant_voice(cvc::app &app, const std::string &room, const std::string &pid,
                            bool &talking) {
  cvc::state *n = app.root().findDescendant(participant_talking(room, pid));
  const std::string v = n ? n->value() : std::string();
  if (v.empty())
    return false;
  talking = v == "1";
  return true;
}

std::string format_rtt(double rtt_ms) {
  if (rtt_ms < 0.0)
    return "--";
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.0f ms", rtt_ms);
  return std::string(buf);
}

std::string format_rate(double kbps) {
  if (kbps < 0.0)
    return "--";
  char buf[32];
  if (kbps >= 1000.0)
    std::snprintf(buf, sizeof(buf), "%.1fM", kbps / 1000.0);
  else
    std::snprintf(buf, sizeof(buf), "%.0fk", kbps);
  return std::string(buf);
}

} // namespace conference
} // namespace ariadne
} // namespace cvc
