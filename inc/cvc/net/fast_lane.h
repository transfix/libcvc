/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_NET_FAST_LANE_H__
#define __CVC_NET_FAST_LANE_H__

// cvc::fast_lane -- the conference's UNRELIABLE, UNORDERED lane (docs/roadmap/
// CONFERENCE_UNRELIABLE_TRANSPORT.md). Beside each client's reliable connection (gRPC or
// WebSocket, which keeps the room tree: roster, scene, intents, chat) a client may open one lane
// to its relay. Pose snapshots and media travel it QuakeWorld-style: a datagram that is lost is
// never resent and nothing waits behind it -- the next one supersedes it.
//
// This header is the transport-neutral part, built in every configuration: the datagram envelope
// (its first byte names the kind), the codecs for media frames and link probes, and the relay's
// view of its lanes (fast_lane_hub). fast_lane_rtc.h implements the lanes over WebRTC data
// channels; room_lane.h encodes the pose snapshots and keeps their acknowledgements.
//
// Every multi-byte field is little-endian. A datagram is one whole message: the lane never splits
// or joins them (a WebRTC data channel fragments a long message itself, and drops it whole if any
// fragment is lost).

#include <cstddef>
#include <cstdint>
#include <cvc/media/media_frame.h>
#include <functional>
#include <string>
#include <vector>

namespace cvc {

// The kind of a lane datagram: its first byte.
enum class lane_kind : std::uint8_t {
  pose = 1,     // relay -> client: a pose snapshot (room_lane.h)
  pose_ack = 2, // client -> relay: the sequence of a snapshot that arrived (room_lane.h)
  media = 3,    // either way: one media frame (encode_lane_media)
  ping = 4,     // either way: a link probe, answered at once with a pong
  pong = 5,
};

// The largest datagram a lane sends: SCTP's send buffer is never smaller than this, and a small
// buffer is what keeps a congested lane from queueing (set_fast_lane_sctp_send_buffer). A larger
// media frame is not sent on the lane: a client sends it on its connection instead, and the relay
// skips it (its rungs keep camera frames far below this: 320x240 at q60 is about 6 KB).
constexpr std::size_t kLaneMaxDatagram = 64 * 1024;

// The kind of DATA (N bytes), or 0 when it is empty.
std::uint8_t lane_kind_of(const std::uint8_t *data, std::size_t n) noexcept;

// One media frame as a lane datagram: the kind byte, the routing and timing fields of
// media_frame, then the payload. Empty when the frame would exceed kLaneMaxDatagram or a string
// field is longer than 255 bytes.
std::vector<std::uint8_t> encode_lane_media(const media_frame &f);
// The frame in DATA, false (OUT unspecified) when it is not a whole media datagram.
bool decode_lane_media(const std::uint8_t *data, std::size_t n, media_frame &out);

// A link probe: KIND (ping or pong), the prober's sequence and its steady-clock send time. A pong
// echoes the ping's two fields, so the prober measures the round trip on its own clock.
std::vector<std::uint8_t> encode_lane_probe(lane_kind kind, std::uint64_t seq,
                                            std::int64_t sent_ns);
bool decode_lane_probe(const std::uint8_t *data, std::size_t n, lane_kind &kind, std::uint64_t &seq,
                       std::int64_t &sent_ns);

// The relay's lanes, one per participant at most, keyed by the pid the participant's reliable
// connection speaks as (its offer came over that connection). Thread-safe.
class fast_lane_hub {
public:
  virtual ~fast_lane_hub() = default;
  // Whether PID has a lane open now.
  virtual bool lane_open(const std::string &pid) const = 0;
  // A count that moves whenever a lane opens or closes.
  virtual std::uint64_t lane_generation() const noexcept = 0;
  // Send one datagram to PID's lane. False when PID has no open lane, or when that lane's send
  // buffer is past its limit: the datagram is dropped there, and a newer one supersedes it.
  virtual bool send_datagram(const std::string &pid, const std::uint8_t *data, std::size_t n) = 0;
  // Datagrams of KIND that arrive on any lane, with the lane's pid. Called on a lane's reader
  // thread; must not block. Replaces the handler KIND had; {} clears it. Media is the
  // media_endpoint's (fast_lane_rtc.h), and pings are answered by the lane itself.
  using datagram_fn =
      std::function<void(const std::string &pid, const std::uint8_t *data, std::size_t n)>;
  virtual void set_on_datagram(lane_kind kind, datagram_fn fn) = 0;
};

} // namespace cvc

#endif // __CVC_NET_FAST_LANE_H__
