/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_LANE_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_LANE_H__

// The conference's fast lanes (cvc/net/fast_lane_rtc.h) in the room: their signaling over the
// reliable connection, the relay's side and a client's.
//
//   POSES   The pose lane carries the conference's pose snapshots (pose_stream.h,
//           room_pose_relay) as an ACKED stream: the relay's encoder for a lane viewer differs each
//           snapshot from the last one the viewer acknowledged (pose_stream_encoder::set_acked),
//           the client decodes against whichever base a snapshot names (pose_stream_decoder::
//           set_bases) and acks each one it applied. A lost snapshot is never resent as such: the
//           next one still differs from what the viewer has. The datagram is lane_kind::pose and
//           the PoseSnapshot protobuf, the same message the reliable transports carry.
//   MEDIA   The media lane carries camera and mic frames (fast_lane_server is a media_endpoint).

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/conference/room_pose_relay.h> // pose_snapshot_lane
#include <cvc/ariadne/pose_stream.h>                // pose_sample
#include <cvc/media/media_frame.h>
#include <cvc/state/pose_snapshot.h>
#include <cvc/state/state_message.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class fast_lane_client;
class fast_lane_server;
class state_cluster_shard;
class state_transport;
namespace ariadne {
namespace conference {

// ---------------- the pose lane's datagrams ----------------

// lane_kind::pose, then `snap` as the PoseSnapshot protobuf. Empty in a build without the
// transports' protobuf (CVC_ENABLE_WS and CVC_ENABLE_GRPC both off): no lane carries poses there.
std::vector<std::uint8_t> encode_lane_pose_snapshot(const cvc::pose_snapshot &snap);
bool decode_lane_pose_snapshot(const std::uint8_t *data, std::size_t n, cvc::pose_snapshot &out);

// lane_kind::pose_ack, then the seq of the snapshot the client applied.
std::vector<std::uint8_t> encode_lane_pose_ack(std::uint32_t seq);
bool decode_lane_pose_ack(const std::uint8_t *data, std::size_t n, std::uint32_t &seq);

// The bases a client's lane decoder keeps: comfortably more than the snapshots in flight over a
// round trip at 60 Hz.
constexpr std::size_t kLanePoseBases = 32;

// ---------------- signaling ----------------
// A participant has TWO lanes to its relay, each its own peer connection: the POSE lane carries
// only pose snapshots and their acks (a few kb/s, so its SCTP congestion window never binds and a
// snapshot never waits behind anything), and the MEDIA lane carries camera and mic frames. On one
// association the two would share one congestion window, and under loss the poses would queue
// behind the video at the sender (CONFERENCE_UNRELIABLE_TRANSPORT.md §7). Both share the relay's
// one UDP port (ICE UDP mux).
//
// A lane's offer and answer ride the client's reliable connection as out-of-band messages at the
// participant's own path, <participant>.lane.<role>, which a relay's bound writes let only that
// participant's connection send. The offer has ttl 0, so the relay takes it and forwards it
// nowhere; the answer goes to that participant's connections alone
// (state_transport::send_message_to).
enum class lane_role { pose, media };
const char *lane_role_name(lane_role role) noexcept; // "pose" | "media"

// The media lane's partial reliability (fast_lane_client::set_lifetime): a lost fragment of a frame
// is retransmitted while the frame is younger than this, then abandoned. The pose lane never
// retransmits.
constexpr std::chrono::milliseconds kMediaLaneLifetime{100};

constexpr const char *kLaneOfferType = "application/x-cvc-lane-offer";
constexpr const char *kLaneAnswerType = "application/x-cvc-lane-answer";

// CLIENT: PID's offer (SDP) for its ROLE lane.
cvc::state_message lane_offer_message(const std::string &room, const std::string &pid,
                                      lane_role role, const std::string &sdp);
// RELAY (RELAY_ID): its answer to PID's offer for its ROLE lane.
cvc::state_message lane_answer_message(const std::string &room, const std::string &relay_id,
                                       const std::string &pid, lane_role role,
                                       const std::string &sdp);

#if !defined(__EMSCRIPTEN__) // a relay never runs in a browser
// RELAY: answers the offers that arrive on SHARD's message bus, sending each answer on every one of
// PLANES (only the one holding the participant's connection writes): pose-lane offers with
// POSE_LANES and media-lane offers with MEDIA_LANES (the caller adds it to its media_router; null:
// media stays on the reliable connections). It is POSES' pose_snapshot_lane (install it with
// room_pose_relay::set_lane), and hands POSES each viewer's acks. A participant that leaves:
// forget() closes its lanes. Construction and forget on the main loop.
class room_lane_relay final : public pose_snapshot_lane {
public:
  room_lane_relay(cvc::state_cluster_shard &shard, std::vector<cvc::state_transport *> planes,
                  std::string room, cvc::fast_lane_server &pose_lanes,
                  cvc::fast_lane_server *media_lanes, room_pose_relay &poses);
  ~room_lane_relay() override;
  room_lane_relay(const room_lane_relay &) = delete;
  room_lane_relay &operator=(const room_lane_relay &) = delete;

  bool carries(const std::string &pid) const override;
  bool send(const std::string &pid, const cvc::pose_snapshot &snap) override;

  void forget(const std::string &pid);
  std::uint64_t offers_refused() const; // offers from a participant path that is not the sender's

private:
  struct impl;
  std::shared_ptr<impl> _impl; // shared with the bus subscriber and the ack handler
};
#endif

// CLIENT: one participant's two lanes: the offers over TP (its reliable connection to the relay),
// the answers from SHARD's message bus, the relay's pose snapshots decoded (and acknowledged) on
// the pose lane, and media both ways on the media lane. The host keeps sending media on TP while
// send_media() returns false. Natively the callbacks run on lane threads; in a browser on its event
// loop.
class room_lane_session {
public:
  room_lane_session(cvc::state_cluster_shard &shard, cvc::state_transport &tp, std::string room,
                    std::string pid);
  ~room_lane_session();
  room_lane_session(const room_lane_session &) = delete;
  room_lane_session &operator=(const room_lane_session &) = delete;

  // Offer both lanes (again, after they closed or failed), or just one. A relay without media
  // lanes (arisrv --lane-media off) never answers the media offer: media stays on the connection.
  void start();
  void start(lane_role role);
  void close();
  bool open(lane_role role) const;
  // One media frame on the media lane; false when it is not open (or full), so the caller sends it
  // on the reliable connection instead.
  bool send_media(const cvc::media_frame &f);
  // Media frames that arrive on the media lane.
  void set_on_media(std::function<void(const cvc::media_frame &f)> fn);
  // The samples of each pose snapshot the lane decoded, at its server tick, after the ack went
  // out: for the host to apply to its holon on its frame's thread (apply_pose_samples). An older
  // snapshot that arrives late gives none.
  using pose_samples_fn =
      std::function<void(std::uint64_t server_tick, const std::vector<pose_sample> &samples)>;
  void set_on_pose_samples(pose_samples_fn fn);
  // Called when a lane opens (true) or closes (false).
  void set_on_state(std::function<void(lane_role role, bool open)> fn);
  // A ping on each open lane; call it about once a second.
  void probe();

  std::uint64_t pose_gaps() const;     // snapshots whose base this client no longer had
  std::uint64_t poses_decoded() const; // snapshots decoded and acknowledged
  cvc::fast_lane_client &lane(lane_role role) noexcept;

private:
  struct impl;
  std::shared_ptr<impl> _impl;
};

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_LANE_H__
