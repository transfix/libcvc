/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_POSE_SNAPSHOT_H__
#define __CVC_STATE_POSE_SNAPSHOT_H__

#include <cstdint>
#include <string>
#include <vector>

namespace cvc {

// cvc::pose_snapshot — one room step's avatar poses for ONE viewer, as the transports carry it: the
// conference relay builds one per connection (ariadne::conference::room_pose_relay) and a client
// turns it back into samples (ariadne::pose_stream_decoder). Plain and proto-free, like
// media_frame: proto/state_transport.proto's PoseSnapshot is its wire encoding, so keep the two in
// sync. Not shard state: never journaled, relayed or bridged.
//
// Units: position in millimetres, velocity in mm/s, rotation as a unit quaternion's components
// times 32767. Every number in an entry is the difference from what the connection was last sent
// for the slot; a slot's first entry carries its pid and its absolute values (ariadne/pose_stream.h
// has the rules).
struct pose_entry {
  std::uint32_t slot = 0;
  std::string pid; // on the slot's first entry only
  std::int32_t x = 0, y = 0, z = 0;
  std::int32_t vx = 0, vy = 0, vz = 0;
  std::int32_t qw = 0, qx = 0, qy = 0, qz = 0;
};

struct pose_snapshot {
  std::uint64_t server_tick = 0; // every entry is at this tick of the relay's holon
  std::uint32_t seq = 0;         // per connection, +1 each snapshot
  std::uint32_t base_seq = 0;    // the snapshot the entries are differences from
  std::vector<pose_entry> entries;
  std::vector<std::uint32_t> gone; // slots that left
};

} // namespace cvc

#endif // __CVC_STATE_POSE_SNAPSHOT_H__
