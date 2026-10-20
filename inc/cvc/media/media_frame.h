/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_MEDIA_FRAME_H__
#define __CVC_MEDIA_FRAME_H__

#include <cstdint>
#include <string>
#include <vector>

namespace cvc {

// cvc::media_frame — the GENERIC, transport-neutral media frame: the one internal format every
// media protocol (WebSocket, gRPC, and whatever we plug in next) maps to on the way in and back
// from on the way out. A relay's media_router fans this canonical frame between endpoints without
// knowing any wire format; each endpoint translates it to/from its own protocol.
//
// It is a plain struct, deliberately proto-free, so it can live BELOW the transports (both
// cvc::state transports include it and map their wire MediaFrame to/from it). It is NOT shard
// state: it is never journaled and never crosses the state replication / WS<->gRPC control bridge.
//
// Field semantics mirror proto/state_transport.proto's MediaFrame (that protobuf is one wire
// encoding of this struct, shared by both the WS and gRPC transports since both carry the Frame
// envelope). Keep the two in sync.
struct media_frame {
  std::string origin_node_id;        // the publishing participant's pid (route per-peer; skip self)
  std::string stream_id;             // the KIND on that peer: "camera" | "mic"
  std::string codec;                 // "mjpeg" (video) | "f32" (audio)
  std::uint32_t width = 0;           // video frame size (0 for audio)
  std::uint32_t height = 0;
  std::int64_t seq = 0;              // monotonic per (origin, stream_id)
  double pts_seconds = 0.0;          // producer presentation timestamp
  std::uint32_t sample_rate = 0;     // audio (0 for video)
  std::uint32_t channels = 0;
  std::vector<std::uint8_t> payload; // one whole frame
};

} // namespace cvc

#endif // __CVC_MEDIA_FRAME_H__
