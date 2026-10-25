/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_INBOUND_GATE_H__
#define __CVC_STATE_INBOUND_GATE_H__

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cvc/core/namespace.h>
#include <memory>
#include <string>

namespace cvc {

struct state_mutation;
struct state_message;

// ----------------
// cvc::state_inbound_gate
// ----------------
// Connection-bound identity for a relay (ARIADNE_CONFERENCE_ROADMAP §5e item 1). Installed with
// state_transport::set_inbound_gate, it governs what a transport takes from the connections it
// ACCEPTED. A connection it dialed, the bridge between a relay's transports and the relay's own
// writes are not gated.
//
//   identity  A connection speaks as the node id of its first Hello, and one live connection at a
//             time holds a node id: the first to claim it, until it closes or falls silent for
//             takeover_after() (then the next claim takes it over). Install the same gate on every
//             transport of a relay, so a node id held on one cannot be claimed on another. Nothing
//             is taken from a connection before its Hello, or while another one holds its node id,
//             and its close then does not count as that peer leaving. The relay's own ids (its
//             transport's and its shards') are never a client's: such a connection never speaks.
//   origin    A mutation, message or media frame must name the connection's node id as its origin,
//             and a heartbeat must name it as its node.
//   writes    admit() then decides each mutation and message. The defaults admit everything, so a
//             plain state_inbound_gate binds identity and origin only.
//   snapshots A SnapshotResponse from an accepted connection is never applied: a relay asks its
//             clients for none.
//
// Nothing here authenticates a peer. It stops a connected peer from writing as another connected
// peer, or where admit() says it may not; a peer can still claim any node id no one holds.
//
// Threading: bind() and release() take an internal lock; admit() is called on transport reader
// threads, concurrently, and must be safe for that.
class state_inbound_gate {
public:
  state_inbound_gate();
  virtual ~state_inbound_gate();

  state_inbound_gate(const state_inbound_gate &) = delete;
  state_inbound_gate &operator=(const state_inbound_gate &) = delete;

  // Claim `node_id` for the connection `who` (any address unique to it while it lives). True when
  // `who` holds it: no one did, `who` already does, or the holder has been silent for longer than
  // takeover_after() and `who` takes it over -- so a connection whose peer went away without
  // closing it (a dropped network) does not keep the id from that peer's reconnect. A transport
  // passes the claimant's `last_rx_ns` (steady-clock ns of its last inbound frame, which a live
  // peer refreshes at least with each link-probe Pong) for later claims to read, and `displaced`,
  // which a takeover sets. Both must outlive the claim (until release). An empty node id is never
  // bound.
  bool bind(const std::string &node_id, const void *who,
            const std::atomic<std::int64_t> *last_rx_ns = nullptr,
            std::atomic<bool> *displaced = nullptr);

  // Give `node_id` up if `who` holds it (its connection closed). True when it did: only then is
  // the close that peer leaving.
  bool release(const std::string &node_id, const void *who);

  // The connection holding `node_id` now, or nullptr.
  const void *holder(const std::string &node_id) const;

  // How long a holder must be silent before another claim takes its id over (default 6 s, three
  // missed link probes at the default 2 s ping interval).
  void set_takeover_after(std::chrono::milliseconds silence);
  std::chrono::milliseconds takeover_after() const;
  std::uint64_t total_takeovers() const noexcept { return _takeovers.load(); }

  // Whether the peer `node_id` (bound, and the mutation's or message's origin) may write it.
  virtual bool admit(const std::string &node_id, const state_mutation &m) const;
  virtual bool admit(const std::string &node_id, const state_message &m) const;

private:
  // The holders, defined in the .cpp over a file-local element type (which keeps the map's
  // template code out of the cvc.dll export table).
  struct table;
  std::unique_ptr<table> _table;
  std::atomic<std::uint64_t> _takeovers{0};
};

} // namespace cvc

#endif // __CVC_STATE_INBOUND_GATE_H__
