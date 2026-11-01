/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <atomic>
#include <cstring>
#include <cvc/ariadne/conference/room_intrinsics.h> // room_participant_path
#include <cvc/ariadne/conference/room_lane.h>
#include <cvc/net/fast_lane.h>
#include <cvc/net/fast_lane_rtc.h>
#include <cvc/state/state.h> // SEPARATOR
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_transport.h>
#include <mutex>
#include <utility>

#if defined(CVC_ENABLE_WS) || defined(CVC_ENABLE_GRPC)
#include "../../state/pose_snapshot_wire.h" // pose_wire (and the generated PoseSnapshot)
#define CVC_LANE_POSE_WIRE 1
#endif

namespace cvc {
namespace ariadne {
namespace conference {

// ---------------- the pose lane's datagrams ----------------

std::vector<std::uint8_t> encode_lane_pose_snapshot(const cvc::pose_snapshot &snap) {
#if defined(CVC_LANE_POSE_WIRE)
  ::cvc::transport::v1::PoseSnapshot pb;
  cvc::pose_wire::encode(snap, &pb);
  std::vector<std::uint8_t> out(1 + pb.ByteSizeLong());
  out[0] = static_cast<std::uint8_t>(cvc::lane_kind::pose);
  if (!pb.SerializeToArray(out.data() + 1, static_cast<int>(out.size() - 1)))
    return {};
  return out;
#else
  (void)snap;
  return {};
#endif
}

bool decode_lane_pose_snapshot(const std::uint8_t *data, std::size_t n, cvc::pose_snapshot &out) {
  if (cvc::lane_kind_of(data, n) != static_cast<std::uint8_t>(cvc::lane_kind::pose))
    return false;
#if defined(CVC_LANE_POSE_WIRE)
  ::cvc::transport::v1::PoseSnapshot pb;
  if (!pb.ParseFromArray(data + 1, static_cast<int>(n - 1)))
    return false;
  out = cvc::pose_wire::decode(pb);
  return true;
#else
  (void)out;
  return false;
#endif
}

std::vector<std::uint8_t> encode_lane_pose_ack(std::uint32_t seq) {
  std::vector<std::uint8_t> out(5);
  out[0] = static_cast<std::uint8_t>(cvc::lane_kind::pose_ack);
  std::memcpy(out.data() + 1, &seq, 4); // little-endian, as every lane field
  return out;
}

bool decode_lane_pose_ack(const std::uint8_t *data, std::size_t n, std::uint32_t &seq) {
  if (n != 5 || cvc::lane_kind_of(data, n) != static_cast<std::uint8_t>(cvc::lane_kind::pose_ack))
    return false;
  std::memcpy(&seq, data + 1, 4);
  return true;
}

// ---------------- signaling ----------------

namespace {

// <participant>.lane: the prefix of both lanes' signaling paths.
std::string lane_base(const std::string &room, const std::string &pid) {
  return room_participant_path(room, pid) + cvc::state::SEPARATOR + "lane";
}

std::string lane_path(const std::string &room, const std::string &pid, lane_role role) {
  return lane_base(room, pid) + cvc::state::SEPARATOR + lane_role_name(role);
}

// The role whose path under PID's lane base PATH is, or false.
bool role_of(const std::string &room, const std::string &pid, const std::string &path,
             lane_role &out) {
  for (lane_role r : {lane_role::pose, lane_role::media})
    if (path == lane_path(room, pid, r)) {
      out = r;
      return true;
    }
  return false;
}

std::atomic<std::uint64_t> g_lane_messages{0};

cvc::state_message lane_message(const std::string &room, const std::string &origin,
                                const std::string &pid, lane_role role, bool offer,
                                const std::string &sdp) {
  cvc::state_message m;
  m.cluster_id = "conf";
  m.origin_node_id = origin;
  m.message_id = std::string(offer ? "lane-offer-" : "lane-answer-") + lane_role_name(role) + "-" +
                 pid + "-" + std::to_string(g_lane_messages.fetch_add(1) + 1);
  m.path = lane_path(room, pid, role);
  m.ttl_hops = 0;
  m.content_type = offer ? kLaneOfferType : kLaneAnswerType;
  m.string_value = sdp;
  return m;
}

} // namespace

const char *lane_role_name(lane_role role) noexcept {
  return role == lane_role::pose ? "pose" : "media";
}

cvc::state_message lane_offer_message(const std::string &room, const std::string &pid,
                                      lane_role role, const std::string &sdp) {
  return lane_message(room, pid, pid, role, true, sdp);
}

cvc::state_message lane_answer_message(const std::string &room, const std::string &relay_id,
                                       const std::string &pid, lane_role role,
                                       const std::string &sdp) {
  return lane_message(room, relay_id, pid, role, false, sdp);
}

// ---------------- the relay's side ----------------

#if !defined(__EMSCRIPTEN__)
struct room_lane_relay::impl {
  impl(cvc::state_cluster_shard &s, std::vector<cvc::state_transport *> p, std::string r,
       cvc::fast_lane_server &pl, cvc::fast_lane_server *ml)
      : shard(s), planes(std::move(p)), room(std::move(r)), pose_lanes(pl), media_lanes(ml) {}

  cvc::state_cluster_shard &shard;
  std::vector<cvc::state_transport *> planes;
  std::string room;
  cvc::fast_lane_server &pose_lanes;
  cvc::fast_lane_server *media_lanes;
  cvc::state_message_bus::subscription_id sub = 0;
  std::atomic<std::uint64_t> refused{0};
};

room_lane_relay::room_lane_relay(cvc::state_cluster_shard &shard,
                                 std::vector<cvc::state_transport *> planes, std::string room,
                                 cvc::fast_lane_server &pose_lanes,
                                 cvc::fast_lane_server *media_lanes, room_pose_relay &poses)
    : _impl(std::make_shared<impl>(shard, std::move(planes), std::move(room), pose_lanes,
                                   media_lanes)) {
  const std::weak_ptr<impl> w = _impl;
  std::string participants = room_participant_path(_impl->room, "");
  participants.pop_back(); // the bus matches whole segments: no trailing separator
  // On a transport's reader thread, synchronously inside the bus's admit().
  _impl->sub = shard.message_bus().subscribe(participants, [w](const cvc::state_message &m) {
    const auto i = w.lock();
    if (!i || m.content_type != kLaneOfferType)
      return;
    // An offer speaks for the participant whose path it is at, and only it.
    lane_role role{};
    if (m.origin_node_id.empty() || !role_of(i->room, m.origin_node_id, m.path, role)) {
      i->refused.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    cvc::fast_lane_server *lanes = role == lane_role::pose ? &i->pose_lanes : i->media_lanes;
    if (!lanes)
      return; // no media lanes on this relay: the client's media stays on its connection
    const std::string pid = m.origin_node_id;
    lanes->offer(pid, m.string_value, [w, pid, role](const std::string &answer) {
      const auto j = w.lock();
      if (!j)
        return;
      const cvc::state_message a =
          lane_answer_message(j->room, j->shard.local_node_id(), pid, role, answer);
      for (cvc::state_transport *tp : j->planes)
        tp->send_message_to(pid, a);
    });
  });
  // A viewer's acks move its acked stream's base (room_pose_relay takes them at its next step).
  pose_lanes.set_on_datagram(
      cvc::lane_kind::pose_ack,
      [&poses](const std::string &pid, const std::uint8_t *d, std::size_t n) {
        std::uint32_t seq = 0;
        if (decode_lane_pose_ack(d, n, seq))
          poses.acknowledge(pid, seq);
      });
}

room_lane_relay::~room_lane_relay() {
  _impl->pose_lanes.set_on_datagram(cvc::lane_kind::pose_ack, {});
  if (_impl->sub)
    _impl->shard.message_bus().unsubscribe(_impl->sub);
}

bool room_lane_relay::carries(const std::string &pid) const {
  return _impl->pose_lanes.lane_open(pid);
}

bool room_lane_relay::send(const std::string &pid, const cvc::pose_snapshot &snap) {
  const std::vector<std::uint8_t> d = encode_lane_pose_snapshot(snap);
  return !d.empty() && _impl->pose_lanes.send_datagram(pid, d.data(), d.size());
}

void room_lane_relay::forget(const std::string &pid) {
  _impl->pose_lanes.close(pid);
  if (_impl->media_lanes)
    _impl->media_lanes->close(pid);
}

std::uint64_t room_lane_relay::offers_refused() const { return _impl->refused.load(); }
#endif // !__EMSCRIPTEN__

// ---------------- a client's session ----------------

struct room_lane_session::impl {
  impl(cvc::state_cluster_shard &s, cvc::state_transport &t, std::string r, std::string p)
      : shard(s), tp(t), room(std::move(r)), pid(std::move(p)) {
    decoder.set_bases(kLanePoseBases);
  }

  cvc::state_cluster_shard &shard;
  cvc::state_transport &tp;
  std::string room;
  std::string pid;
  std::unique_ptr<cvc::fast_lane_client> pose, media;
  cvc::state_message_bus::subscription_id sub = 0;
  std::mutex mu;
  std::function<void(const cvc::media_frame &)> on_media;
  std::function<void(lane_role, bool)> on_state;
  pose_samples_fn on_samples;
  std::mutex decoder_mu; // snapshots decode one at a time, in arrival order
  pose_stream_decoder decoder;
  std::uint64_t decoded = 0;

  cvc::fast_lane_client &of(lane_role r) { return r == lane_role::pose ? *pose : *media; }

  void state_changed(lane_role r, bool open) {
    std::function<void(lane_role, bool)> cb;
    {
      std::lock_guard<std::mutex> lk(mu);
      cb = on_state;
    }
    if (cb)
      cb(r, open);
  }

  // A pose datagram: decode it against the base it names, ack it, then hand the samples on.
  void on_pose(const std::uint8_t *d, std::size_t n) {
    cvc::pose_snapshot snap;
    if (!decode_lane_pose_snapshot(d, n, snap))
      return;
    std::vector<pose_sample> samples;
    {
      std::lock_guard<std::mutex> lk(decoder_mu);
      if (!decoder.decode(snap, samples))
        return; // a base this client no longer has: the relay's next snapshot, or keyframe, follows
      ++decoded;
    }
    const std::vector<std::uint8_t> ack = encode_lane_pose_ack(snap.seq);
    pose->send(ack.data(), ack.size());
    if (samples.empty())
      return;
    pose_samples_fn cb;
    {
      std::lock_guard<std::mutex> lk(mu);
      cb = on_samples;
    }
    if (cb)
      cb(snap.server_tick, samples);
  }
};

room_lane_session::room_lane_session(cvc::state_cluster_shard &shard, cvc::state_transport &tp,
                                     std::string room, std::string pid)
    : _impl(std::make_shared<impl>(shard, tp, std::move(room), std::move(pid))) {
  const std::weak_ptr<impl> w = _impl;
  for (lane_role r : {lane_role::pose, lane_role::media}) {
    auto lane = std::make_unique<cvc::fast_lane_client>([w, r](const std::string &offer) {
      if (const auto i = w.lock())
        i->tp.publish_message(lane_offer_message(i->room, i->pid, r, offer));
    });
    lane->set_on_state([w, r](bool open) {
      if (const auto i = w.lock())
        i->state_changed(r, open);
    });
    (r == lane_role::pose ? _impl->pose : _impl->media) = std::move(lane);
  }
  // Poses are never retransmitted (the relay's acked stream resends what is not acknowledged). A
  // media frame may be, while it is under kMediaLaneLifetime old: at 1-3 % loss that keeps about
  // 99-92 % of camera frames instead of 92-74 % (CONFERENCE_UNRELIABLE_TRANSPORT.md §7).
  _impl->media->set_lifetime(kMediaLaneLifetime);
  _impl->pose->set_on_datagram([w](const std::uint8_t *d, std::size_t n) {
    if (const auto i = w.lock())
      i->on_pose(d, n);
  });
  _impl->media->set_on_datagram([w](const std::uint8_t *d, std::size_t n) {
    const auto i = w.lock();
    cvc::media_frame f;
    if (!i || !cvc::decode_lane_media(d, n, f) || f.origin_node_id == i->pid)
      return;
    std::function<void(const cvc::media_frame &)> cb;
    {
      std::lock_guard<std::mutex> lk(i->mu);
      cb = i->on_media;
    }
    if (cb)
      cb(f);
  });
  _impl->sub = shard.message_bus().subscribe(
      lane_base(_impl->room, _impl->pid), [w](const cvc::state_message &m) {
        const auto i = w.lock();
        lane_role r{};
        if (i && m.content_type == kLaneAnswerType && m.origin_node_id != i->pid &&
            role_of(i->room, i->pid, m.path, r))
          i->of(r).on_answer(m.string_value);
      });
}

room_lane_session::~room_lane_session() {
  if (_impl->sub)
    _impl->shard.message_bus().unsubscribe(_impl->sub);
  _impl->pose->close();
  _impl->media->close();
}

void room_lane_session::start() {
  _impl->pose->start();
  _impl->media->start();
}

void room_lane_session::start(lane_role role) { _impl->of(role).start(); }

void room_lane_session::close() {
  _impl->pose->close();
  _impl->media->close();
}

bool room_lane_session::open(lane_role role) const { return _impl->of(role).open(); }

bool room_lane_session::send_media(const cvc::media_frame &f) {
  if (!_impl->media->open())
    return false;
  const std::vector<std::uint8_t> d = cvc::encode_lane_media(f);
  return !d.empty() && _impl->media->send(d.data(), d.size());
}

void room_lane_session::set_on_media(std::function<void(const cvc::media_frame &f)> fn) {
  std::lock_guard<std::mutex> lk(_impl->mu);
  _impl->on_media = std::move(fn);
}

void room_lane_session::set_on_pose_samples(pose_samples_fn fn) {
  std::lock_guard<std::mutex> lk(_impl->mu);
  _impl->on_samples = std::move(fn);
}

void room_lane_session::set_on_state(std::function<void(lane_role role, bool open)> fn) {
  std::lock_guard<std::mutex> lk(_impl->mu);
  _impl->on_state = std::move(fn);
}

void room_lane_session::probe() {
  for (lane_role r : {lane_role::pose, lane_role::media})
    if (_impl->of(r).open())
      _impl->of(r).probe();
}

std::uint64_t room_lane_session::pose_gaps() const {
  std::lock_guard<std::mutex> lk(_impl->decoder_mu);
  return _impl->decoder.gaps();
}

std::uint64_t room_lane_session::poses_decoded() const {
  std::lock_guard<std::mutex> lk(_impl->decoder_mu);
  return _impl->decoded;
}

cvc::fast_lane_client &room_lane_session::lane(lane_role role) noexcept { return _impl->of(role); }

} // namespace conference
} // namespace ariadne
} // namespace cvc
