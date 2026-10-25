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

#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_WRITE_GATE_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_WRITE_GATE_H__

#include <cvc/state/state_change_journal.h>
#include <cvc/state/state_inbound_gate.h>
#include <string>

namespace cvc {
namespace ariadne {
namespace conference {

// ----------------
// room_write_gate
// ----------------
// The room's write rule for a relay's clients (ARIADNE_CONFERENCE_ROADMAP §5d/§5e item 1): install
// one gate on every transport of the relay (state_transport::set_inbound_gate), which binds each
// connection to the pid of its Hello. A participant (whose pid is one identifier, no '.') may then
// write only inside its own participant subtree, conf.rooms.<room>.participants.<pid>, and there
// only:
//   - a value, a touch or a data() payload (no delete, reset, delegation, read-only or hidden
//     flag: no client uses them, and each could take a key from the relay);
//   - outside the keys the relay owns there: `pose` (the server holon's authoritative pose),
//     `net.relay` (the relay's view of the link), `device` (its device tag) and each
//     `streams.<kind>.ready` (SFU mode);
//   - on a canonical path (no empty or whitespace-padded segment, which the state tree would
//     trim or skip onto another key) at most 16 segments and 512 bytes long.
// Everything else is the relay's to write: the scene, the area of interest, the server's own
// leaves, and every other participant. A message is admitted on the same paths. What the rule
// refuses is dropped and counted (state_transport::total_inbound_rejected), and the relay's own
// writes are unaffected.
class room_write_gate final : public cvc::state_inbound_gate {
public:
  explicit room_write_gate(std::string room);

  const std::string &room() const noexcept { return _room; }

  // Whether participant `pid` may write `path` with `op` (the rule above, without the binding).
  bool may_write(const std::string &pid, const std::string &path, cvc::state_mutation_op op) const;

  bool admit(const std::string &node_id, const cvc::state_mutation &m) const override;
  bool admit(const std::string &node_id, const cvc::state_message &m) const override;
  // A participant is the source of every value it may write (its participant subtree minus the
  // relay's keys), so the relay's re-broadcast never sends those back to it.
  bool sources(const std::string &node_id, const std::string &path) const override;

private:
  bool own_path(const std::string &pid, const std::string &path) const;

  std::string _room;
};

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_WRITE_GATE_H__
