/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_MEDIA_INTRINSICS_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_MEDIA_INTRINSICS_H__

// Conference room MEDIA verbs (roadmap §4b), namespace cvc::ariadne::conference. The MEDIA plane of
// the conference, riding on the control plane (room_intrinsics) and the GENERAL streaming substrate
// (cvc::ariadne::stream + its merged gRPC stream_bridge). Grpc-gated: these need stream_bridge, so
// the whole TU is compiled only with CVC_ENABLE_GRPC.
//
// MESH P2P (this slice): a publisher serve_port()s its stream on its own bridge and advertises the
// stream's descriptor (endpoint + token + format) under its participant subtree; a subscriber reads
// that descriptor from the (replicated) room tree, opens a local proxy sized to the format, and
// pull_from()s the publisher's endpoint. SFU-via-relay (serve/pull through the merged stream_relay,
// server-selected) is a later slice.
//
//   (room-publish ROOM {kind token})   serve the open stream `token` (from stream-open) on the
//                                       host's bridge and advertise it as my KIND stream. Returns
//                                       { ok kind token } (or { ok #f error }).
//   (room-subscribe ROOM PID KIND)      open a local proxy for participant PID's KIND stream and
//                                       pull its frames into it. Returns { ok token } — feed the
//                                       token to a `stream_view` widget (video) or
//                                       stream-play-audio.
//
// The host supplies the bridge + the endpoint peers reach it at (its transport's listen_address),
// which only the host knows — so, like register_stream_view_widget(backend, …), the media verbs
// take them at registration. TEARDOWN ORDERING: the host MUST stop its bridge before the Runtime
// tears down the documents, so no puller feeds a proxy as it closes (the bridge has no per-proxy
// stop yet — the §7 "per-stream retirement" gap, shared with room-unpublish/leave; a later slice
// closes it).

#include <memory>
#include <string>

namespace cvc {
class app;

namespace ariadne {
namespace stream {
class stream;
class stream_bridge;
} // namespace stream

namespace conference {

// Register the (room-publish)/(room-subscribe) verbs against BRIDGE (which must outlive the app's
// documents) and LOCAL_ENDPOINT (the address peers reach BRIDGE's transport at — its
// transport.listen_address() after start()). RELAY_ENDPOINT, when non-empty, switches to SFU mode:
// (room-publish) PUSHES the stream to that relay and advertises the RELAY as its source, so every
// client reaches media through the server and never peer-to-peer (NAT-safe). Empty = mesh
// (default).
void register_room_media_intrinsics(cvc::app &app, cvc::ariadne::stream::stream_bridge &bridge,
                                    const std::string &local_endpoint,
                                    const std::string &relay_endpoint = std::string());

// ── the media model as reusable free functions (the verbs wrap these; tests call them directly) ──

// Publish LOCAL into participant PID's streams/<kind> in ROOM and advertise its descriptor
// (endpoint
// + token + id + scope + format). MESH (RELAY_ENDPOINT empty): serve_port LOCAL on BRIDGE and
// advertise LOCAL_ENDPOINT. SFU (RELAY_ENDPOINT set): serve (push) LOCAL to RELAY_ENDPOINT and
// advertise RELAY_ENDPOINT, so subscribers pull the relayed copy from the server. False on bad
// args.
bool room_publish(cvc::app &app, cvc::ariadne::stream::stream_bridge &bridge,
                  const std::string &local_endpoint, const std::string &room_id,
                  const std::string &pid, const std::string &kind,
                  cvc::ariadne::stream::stream &local,
                  const std::string &relay_endpoint = std::string());

// Subscribe: read participant TARGET_PID's KIND descriptor from ROOM, open a local proxy sized to
// its advertised format (and sharing its token so the pull routes), and pull TARGET_PID's frames
// into it over BRIDGE. Returns the proxy (owned by the caller — close it BEFORE the bridge), or
// null if the descriptor is absent/malformed, the proxy cannot open, or the pull handshake fails.
std::unique_ptr<cvc::ariadne::stream::stream>
room_subscribe(cvc::app &app, cvc::ariadne::stream::stream_bridge &bridge,
               const std::string &room_id, const std::string &target_pid, const std::string &kind);

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_MEDIA_INTRINSICS_H__
