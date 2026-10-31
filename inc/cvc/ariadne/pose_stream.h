/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_POSE_STREAM_H__
#define __CVC_ARIADNE_POSE_STREAM_H__

#include <cstdint>
#include <cvc/physics/body.h>
#include <cvc/state/pose_snapshot.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

// ----------------
// The conference's pose stream: avatar poses as compact per-connection snapshots
// ----------------
// The relay sends each viewer, every room step it is due (room_pose_relay), ONE cvc::pose_snapshot
// of the participants it sees, instead of their poses as room-tree leaves (about 110 bytes for each
// changed number). An entry is the participant's pose quantized -- millimetres, mm/s, quaternion
// components times 32767 -- and taken as the difference from what this connection was last sent for
// it, so a walking avatar costs about a dozen bytes and a still one two (a heartbeat) or nothing.
//
// The encoder is the relay's end of one connection, the decoder the client's. A snapshot is a
// difference from the one before it on the same connection (base_seq), which the reliable, ordered
// transports guarantee arrives; a base_seq of 0 restarts the stream (every slot named afresh). A
// snapshot the relay could not write is not lost: the encoder only moves on at commit(), so the
// next one carries the difference from what the viewer actually has.

// A pose in the stream's units.
struct pose_q {
  std::int32_t x = 0, y = 0, z = 0;
  std::int32_t vx = 0, vy = 0, vz = 0;
  std::int32_t qw = 0, qx = 0, qy = 0, qz = 0;
  bool operator==(const pose_q &o) const {
    return x == o.x && y == o.y && z == o.z && vx == o.vx && vy == o.vy && vz == o.vz &&
           qw == o.qw && qx == o.qx && qy == o.qy && qz == o.qz;
  }
  bool operator!=(const pose_q &o) const { return !(*this == o); }
};
pose_q quantize_pose(const cvc::physics::body_state &s);
// Back to a body_state (the rotation renormalized); angular velocity is not carried (zero).
cvc::physics::body_state dequantize_pose(const pose_q &q);

// What a viewer may be sent this step about one participant.
struct pose_subject {
  std::string pid;
  cvc::physics::body_state state;
  bool due = true; // false: it stays known to the viewer, but no entry this step
};

class pose_stream_encoder {
public:
  pose_stream_encoder();
  ~pose_stream_encoder();
  pose_stream_encoder(pose_stream_encoder &&) noexcept;
  pose_stream_encoder &operator=(pose_stream_encoder &&) noexcept;

  // The snapshot at `tick` of `subjects`: an entry for each due one that is new to the connection
  // (its pid and absolute pose), has moved since it was last sent (the difference), or has not been
  // sent for `heartbeat_ticks` (an empty entry: still here, at this tick); `gone` for each
  // participant sent before that is not among `subjects`. Its seq and base_seq follow the last
  // commit. Nothing changes until commit(); a second encode() without one starts again from the
  // last commit, so an unwritten snapshot's changes are carried by the next.
  cvc::pose_snapshot encode(std::uint64_t tick, const std::vector<pose_subject> &subjects,
                            std::uint64_t heartbeat_ticks);
  // The last encode() was written: the connection now has it.
  void commit();
  // Start the stream again: the next snapshot names every slot afresh, with base_seq 0.
  void reset();
  // The tick `pid` was last committed at, 0 if never (or since it went).
  std::uint64_t last_sent_tick(const std::string &pid) const;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// A participant's pose from one snapshot.
struct pose_sample {
  std::string pid;
  cvc::physics::body_state state;
};

class pose_stream_decoder {
public:
  pose_stream_decoder();
  ~pose_stream_decoder();
  pose_stream_decoder(pose_stream_decoder &&) noexcept;
  pose_stream_decoder &operator=(pose_stream_decoder &&) noexcept;

  // Apply `snap`, appending a sample to `out` for each of its entries (at snap.server_tick).
  // False, with nothing applied, when it is not the next one (its base_seq is not the seq last
  // applied, nor 0): the stream is out of step, and waits for a restart.
  bool decode(const cvc::pose_snapshot &snap, std::vector<pose_sample> &out);
  // Snapshots refused for being out of step.
  std::uint64_t gaps() const noexcept;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_POSE_STREAM_H__
