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

#include <cctype>
#include <cvc/ariadne/conference/room_intrinsics.h>
#include <cvc/ariadne/conference/room_write_gate.h>
#include <cvc/state/state.h> // isValidStateName
#include <cvc/state/state_message.h>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {
namespace conference {

namespace {

// How deep and long a client's path may be. The deepest key a client writes is
// conf.rooms.<room>.participants.<pid>.streams.<kind>.endpoint (8 segments), and the relay walks a
// participant's whole subtree every tick (the roster), so an unbounded one is a denial of service.
constexpr std::size_t kMaxDepth = 16;
constexpr std::size_t kMaxPathBytes = 512;

bool op_allowed(cvc::state_mutation_op op) {
  switch (op) {
  case cvc::state_mutation_op::set_value:
  case cvc::state_mutation_op::touch:
  case cvc::state_mutation_op::set_data:
    return true;
  default:
    return false;
  }
}

bool padded(const std::string &s) {
  return std::isspace(static_cast<unsigned char>(s.front())) ||
         std::isspace(static_cast<unsigned char>(s.back()));
}

// PATH's segments, or none when it is not canonical. The state tree trims each segment and skips
// empty ones, so "bob..pose" or "bob.pose " would land on bob.pose: every segment must be
// non-empty and unpadded, and there must be at most kMaxDepth of them.
std::vector<std::string> canonical_segments(const std::string &path) {
  std::vector<std::string> out;
  if (path.empty() || path.size() > kMaxPathBytes)
    return {};
  std::size_t start = 0;
  for (;;) {
    const std::size_t dot = path.find(cvc::state::SEPARATOR[0], start);
    std::string seg =
        path.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
    if (seg.empty() || padded(seg) || out.size() == kMaxDepth)
      return {};
    out.push_back(std::move(seg));
    if (dot == std::string::npos)
      return out;
    start = dot + 1;
  }
}

// Whether REST (the path below a participant) names a key the relay writes there: the server
// holon's `pose` (holon_sync.h), the relay's link sample `net.relay` and the participant's
// `device` tag (room_net.h), and in SFU mode each stream's `streams.<kind>.ready`
// (room_relay_tick).
bool relay_owned(const std::vector<std::string> &rest) {
  if (rest.empty())
    return false;
  if (rest[0] == "pose" || rest[0] == "device")
    return true;
  if (rest[0] == "net" && rest.size() >= 2 && rest[1] == "relay")
    return true;
  return rest[0] == "streams" && rest.size() >= 3 && rest[2] == "ready";
}

} // namespace

room_write_gate::room_write_gate(std::string room) : _room(std::move(room)) {}

bool room_write_gate::own_path(const std::string &pid, const std::string &path) const {
  // One identifier segment: a node id like "bob.pose" would otherwise own bob's pose.
  if (!cvc::state::isValidStateName(pid))
    return false;
  const std::vector<std::string> segs = canonical_segments(path);
  const std::vector<std::string> own = canonical_segments(room_participant_path(_room, pid));
  if (segs.empty() || own.empty() || segs.size() < own.size())
    return false;
  for (std::size_t i = 0; i < own.size(); ++i)
    if (segs[i] != own[i])
      return false;
  return !relay_owned(std::vector<std::string>(segs.begin() + own.size(), segs.end()));
}

bool room_write_gate::may_write(const std::string &pid, const std::string &path,
                                cvc::state_mutation_op op) const {
  return op_allowed(op) && own_path(pid, path);
}

bool room_write_gate::admit(const std::string &node_id, const cvc::state_mutation &m) const {
  return may_write(node_id, m.path, m.op);
}

bool room_write_gate::admit(const std::string &node_id, const cvc::state_message &m) const {
  return own_path(node_id, m.path);
}

} // namespace conference
} // namespace ariadne
} // namespace cvc
