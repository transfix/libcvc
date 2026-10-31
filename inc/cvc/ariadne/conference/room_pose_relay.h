/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_POSE_RELAY_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_POSE_RELAY_H__

#include <cstdint>
#include <memory>
#include <string>

namespace cvc {
class state_transport;
namespace ariadne {
class holon;
namespace conference {
class aoi_snapshot;

// How many room steps apart a viewer is sent pose snapshots, from its link's round trip (ms; < 0
// before one is measured): every step (60 Hz at the relay's 60 Hz room step) under 80 ms, every
// second one under 160 ms, every third beyond (or not measured yet: 30 Hz). On a short round
// trip the time between snapshots is most of what a peer is shown behind, so a fast link gets them
// as often as the room steps; a long one says the path is slower or busier, and gets fewer.
// `current` (0 = none yet) holds within 10% of a threshold, so a round trip hovering at one does
// not flip the rate.
std::uint32_t pose_send_interval_steps(double rtt_ms, std::uint32_t current = 0) noexcept;

// The relay's pose stream (pose_stream.h): every room step, each viewer that is due gets ONE
// pose_snapshot of the participants it sees, written to its own connection on whichever transport
// has it. Due: every pose_send_interval_steps(its round trip) steps. What it sees, by the area of
// interest's tier for the pair (none: all inner): an inner participant is in each of its
// snapshots that it moved for, and every one a second while still (a heartbeat); a middle or
// outer one only every `push_interval_ticks`, as the room tree pushes went. Itself is in its own
// snapshots too (its client reconciles its prediction to it). Every `keyframe_ticks` a viewer's
// snapshot restarts its stream with every participant it sees, absolute: a client that lost its
// place (a snapshot that went into a connection as it died) is back in step within that.
class room_pose_relay {
public:
  struct config {
    std::uint64_t heartbeat_ticks = 120;    // a still participant's entry, at least this often
    std::uint64_t push_interval_ticks = 24; // middle/outer pairs (aoi_pose_push_interval_ticks)
    std::uint64_t keyframe_ticks = 240;     // a restart of each viewer's stream, this often
  };
  explicit room_pose_relay(config c);
  room_pose_relay();
  ~room_pose_relay();
  room_pose_relay(const room_pose_relay &) = delete;
  room_pose_relay &operator=(const room_pose_relay &) = delete;

  // A transport whose connections carry the snapshots. Null or a repeat: ignored.
  void add_transport(cvc::state_transport *tp);

  // `pid`'s measured round trip, in ms (the transports' link reports): sets its rate.
  void set_rtt(const std::string &pid, double rtt_ms);
  // The steps between `pid`'s snapshots now (0 for a pid it has not seen).
  std::uint32_t interval_steps(const std::string &pid) const;

  // Room step number `step` (counting up by one), after server_pump: the due viewers' snapshots of
  // SERVER's participants, each tiered by `aoi` (null: all inner). A viewer is any participant of
  // SERVER; one that left is forgotten.
  void step(const holon &server, std::uint64_t step, const aoi_snapshot *aoi);

  struct stats {
    std::uint64_t snapshots = 0; // written (one per viewer's connection)
    std::uint64_t entries = 0;   // in them
    std::uint64_t unsent = 0;    // due, but no transport could write it (carried by the next)
  };
  stats take_stats();

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_POSE_RELAY_H__
