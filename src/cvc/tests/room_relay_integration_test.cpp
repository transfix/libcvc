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

// The room's spatial interest end to end, on loopback. A relay is assembled the way arisrv
// assembles it -- a whole-tree shard enforcing "conf", relay transports with the interest gate on
// and journal backfill off, a server holon, a media router and the room's area-of-interest driver,
// stepped every 50 ms by relay_room_step -- and a few clients the way arigl assembles them: a
// shard rooted at the room tree, a client holon pumped from it every frame, a synthetic camera and
// mic at their real rates. Each case moves the avatars through the server holon (the authority)
// and checks what every client actually received, as its shard applied it: live pose mutations
// against pushed snapshot entries, intents, frames that should never have reached it, media by
// publisher.
//
// Clients take turns over the transports this build has (WebSocket and/or gRPC), so in a build
// with both every room mixes them and its traffic crosses the relay's bridge and media router.
// The assertions are on the mechanism (no live frame for an outer peer, a fresh tick on every
// push, every sent frame accounted for); the few rates checked have wide bounds, said where they
// are, so a loaded runner does not fail them.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>                  // std::strtoull
#include <cvc/ariadne/avatar_sim.h> // avatar_intent
#include <cvc/ariadne/conference/room_aoi.h>
#include <cvc/ariadne/conference/room_aoi_relay.h>
#include <cvc/ariadne/conference/room_intrinsics.h> // room_join, room_leave, room_participant_path
#include <cvc/ariadne/conference/room_net.h>        // room_device_tag, the device leaf
#include <cvc/ariadne/conference/room_scene.h>      // seed_default_scene, seed_large_demo_scene
#include <cvc/ariadne/conference/room_write_gate.h> // bound writes
#include <cvc/ariadne/holon.h>
#include <cvc/ariadne/holon_sync.h> // server_pump, client_pump, read_pose, write_intent
#include <cvc/core/app.h>
#include <cvc/media/media_frame.h>
#include <cvc/media/media_router.h>
#include <cvc/media/media_rungs.h>
#include <cvc/physics/body.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_device_key.h>
#include <cvc/state/state_transport.h>
#if defined(CVC_ENABLE_GRPC)
#include <cvc/state/state_transport_grpc.h>
#endif
#if defined(CVC_ENABLE_WS)
#include <cvc/state/state_transport_ws.h>
#endif
#include <functional>
#include <gtest/gtest.h>
#include <initializer_list>
#include <iterator> // std::next
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace ra = cvc::ariadne::conference;
using cvc::ariadne::holon;
using cvc::ariadne::holon_role;

namespace {

constexpr const char *kRoom = "standup";
constexpr const char *kRelayId = "server";
constexpr double kRoomStepS = 0.05;         // arisrv's room step
constexpr double kFrameS = 1.0 / 30.0;      // a client frame (arigl's publisher rate)
constexpr double kCameraHz = 30.0;          // the synthetic camera
constexpr double kMicHz = 48000.0 / 1024.0; // one 1024-sample f32 mono chunk at 48 kHz
constexpr std::size_t kCameraBytes = 4096;  // small, so loopback never saturates
constexpr std::size_t kMicBytes = 1024 * 4; // 1024 f32 samples
// Media is lossless here. gRPC's stall drop is off by default, and the relay and every client turn
// off the WebSocket backlog drop. On Linux that drop sheds a frame whenever a starved receiver's
// socket holds more than 32 KB, so a loaded runner used to miss a few percent of the frames. A
// slow runner now delays frames but sheds none, and the media counts measure the gate alone.

double now_s() {
  static const auto epoch = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - epoch).count();
}

double xy_distance(const cvc::physics::vec3 &a, const cvc::physics::vec3 &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

enum class link_kind { ws, grpc };

// The transports this build has, in the order clients take them. The test is only built when it
// has at least one.
std::vector<link_kind> link_kinds() {
  std::vector<link_kind> k;
#if defined(CVC_ENABLE_WS)
  k.push_back(link_kind::ws);
#endif
#if defined(CVC_ENABLE_GRPC)
  k.push_back(link_kind::grpc);
#endif
  return k;
}

// conf.rooms.<room>.participants. -- every participant leaf is under it.
const std::string &participants_prefix() {
  static const std::string p = ra::room_participant_path(kRoom, "");
  return p;
}

// The participant (SUBJECT) and the family under it ("pose", "intent", "presence", ...) of a room
// leaf; false for a path outside the room's participants.
bool split_participant_path(const std::string &path, std::string &subject, std::string &family) {
  const std::string &p = participants_prefix();
  if (path.compare(0, p.size(), p) != 0)
    return false;
  const std::size_t a = p.size();
  const std::size_t b = path.find('.', a);
  if (b == std::string::npos)
    return false;
  subject = path.substr(a, b - a);
  const std::size_t c = path.find('.', b + 1);
  family = path.substr(b + 1, c == std::string::npos ? std::string::npos : c - b - 1);
  return true;
}

bool ends_with(const std::string &s, const std::string &suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// What one client's shard applied, counted on the transport's reader thread as it happened. A
// live mutation carries the origin node and the sequence it was journaled with there; a snapshot
// entry (the admit snapshot, a targeted push, the WebSocket re-broadcast) carries neither, so the
// two are told apart without looking at frames.
class client_tap {
public:
  void on_applied(const cvc::state_mutation &m) {
    if (m.op != cvc::state_mutation_op::set_value)
      return;
    std::string subject, family;
    if (!split_participant_path(m.path, subject, family))
      return;
    const bool is_live = !m.origin_node_id.empty();
    std::lock_guard<std::mutex> lk(_mu);
    if (family == "pose") {
      const bool tick = ends_with(m.path, ".pose.server_tick");
      if (tick) {
        std::uint64_t &hi = _max_tick[subject];
        hi = (std::max)(
            hi, static_cast<std::uint64_t>(std::strtoull(m.string_value.c_str(), nullptr, 10)));
      }
      if (is_live) {
        ++_live_pose[subject];
        if (m.origin_node_id == kRelayId)
          _live_pose_seq[subject].insert(m.sequence);
      } else if (tick) {
        ++_pushed_ticks[subject]; // every push carries its subject's tick, last
      }
    } else if (family == "intent") {
      ++_intents[subject];
    }
  }

  void on_media(const cvc::media_frame &f) {
    std::lock_guard<std::mutex> lk(_mu);
    ++_media[{f.origin_node_id, f.stream_id}];
  }

  // Live pose mutations applied for SUBJECT.
  std::uint64_t live_pose(const std::string &subject) const { return get(_live_pose, subject); }
  // Snapshot entries applied for SUBJECT's server_tick: one per push (or admit / re-broadcast).
  std::uint64_t pushed_ticks(const std::string &subject) const {
    return get(_pushed_ticks, subject);
  }
  // The newest server_tick applied for SUBJECT, live or pushed. The tree's own leaf can step back
  // (a WebSocket re-broadcast carries the relay tree's tick, which for a subject standing still is
  // its last heartbeat, older than a push), so freshness is read from here.
  std::uint64_t max_tick(const std::string &subject) const { return get(_max_tick, subject); }
  // Intent leaves applied for SUBJECT, live or from a snapshot.
  std::uint64_t intents(const std::string &subject) const { return get(_intents, subject); }
  std::uint64_t media(const std::string &publisher, const std::string &stream) const {
    std::lock_guard<std::mutex> lk(_mu);
    const auto it = _media.find({publisher, stream});
    return it == _media.end() ? 0 : it->second;
  }
  // How many of SEQUENCES (relay journal sequences) arrived live for SUBJECT.
  std::size_t live_pose_received(const std::string &subject,
                                 const std::vector<std::uint64_t> &sequences) const {
    std::lock_guard<std::mutex> lk(_mu);
    const auto it = _live_pose_seq.find(subject);
    if (it == _live_pose_seq.end())
      return 0;
    std::size_t n = 0;
    for (std::uint64_t s : sequences)
      n += it->second.count(s);
    return n;
  }

private:
  std::uint64_t get(const std::map<std::string, std::uint64_t> &m, const std::string &k) const {
    std::lock_guard<std::mutex> lk(_mu);
    const auto it = m.find(k);
    return it == m.end() ? 0 : it->second;
  }

  mutable std::mutex _mu;
  std::map<std::string, std::uint64_t> _live_pose, _pushed_ticks, _max_tick, _intents;
  std::map<std::string, std::set<std::uint64_t>> _live_pose_seq;
  std::map<std::pair<std::string, std::string>, std::uint64_t> _media;
};

struct relay_options {
  bool use_aoi = true; // the room's area-of-interest driver (arisrv --aoi)
  ra::aoi_params aoi = ra::conference_aoi_params();
  bool gate = true;         // the relay interest gate (--relay-gate)
  bool trim = true;         // trim the journal and no backfill on connect (--relay-journal trim)
  bool large_scene = false; // a 100 m plane instead of the 20 m room (where hall has room)
  bool ws_broadcast = true; // arisrv's ~1 Hz WebSocket re-broadcast of the room tree
  bool hold = true;         // hold each admission for its filter (--admission-hold-ms; AOI only)
  bool bind_writes = true;  // each client writes only its own participant (--bind-writes)
  std::string room_token;   // admit only clients that present it (--room-token-file); "" = open
};

// The relay, as arisrv builds it, stepped by the test.
class test_relay {
public:
  explicit test_relay(relay_options o) : opt(std::move(o)) {
    const std::chrono::milliseconds hold(opt.use_aoi && opt.hold ? 2000 : 0);
    if (opt.bind_writes)
      bind_gate = std::make_shared<ra::room_write_gate>(kRoom);
    shard = std::make_unique<cvc::state_cluster_shard>(app, "conf", kRelayId);
    shard->add_interest("conf");
    shard->set_enforce_interest(true);
    shard->attach();
    shard->install_as_default();
    shard->adapter().set_on_local_mutation([this](const cvc::state_mutation &m) { note_local(m); });
    shard->adapter().set_on_remote_applied([this](const cvc::state_mutation &m) {
      std::lock_guard<std::mutex> lk(_mu);
      _applied[m.origin_node_id].push_back(m.path);
    });
    // A closed connection is queued here (transport threads) and retracted on the next step.
    const auto on_departed = [this](const std::string &node_id) {
      std::lock_guard<std::mutex> lk(_mu);
      _departed.push_back(node_id);
    };
#if defined(CVC_ENABLE_GRPC)
    grpc.set_on_connection_closed(on_departed);
    grpc.set_relay_requires_interest(opt.gate);
    grpc.set_backfill_on_connect(!opt.trim);
    grpc.set_admission_hold(hold);
    grpc.set_inbound_gate(bind_gate);
    grpc.set_auth_config({opt.room_token, std::string()});
    grpc.set_device_pins(pins);
    grpc.set_media_shaper(
        [this](const cvc::media_frame &f, const std::string &v) { return shaper.shape(f, v); });
    grpc.start("127.0.0.1:0", kRelayId, "conf");
    grpc.set_relay(true);
    grpc.register_shard(shard.get());
    planes.push_back(&grpc);
    router.add_endpoint(&grpc);
#endif
#if defined(CVC_ENABLE_WS)
    ws.set_on_connection_closed(on_departed);
    ws.set_relay_requires_interest(opt.gate);
    ws.set_backfill_on_connect(!opt.trim);
    ws.set_media_backlog_drop(0); // lossless media: the note by the frame sizes above
    ws.set_admission_hold(hold);
    ws.set_inbound_gate(bind_gate);
    ws.set_auth_config({opt.room_token, std::string()});
    ws.set_device_pins(pins);
    ws.set_media_shaper(
        [this](const cvc::media_frame &f, const std::string &v) { return shaper.shape(f, v); });
    ws.start("127.0.0.1:0", kRelayId, "conf");
    ws.set_relay(true);
    ws.register_shard(shard.get());
    planes.push_back(&ws);
    router.add_endpoint(&ws);
#endif
#if defined(CVC_ENABLE_GRPC) && defined(CVC_ENABLE_WS)
    grpc.set_bridge(&ws);
    ws.set_bridge(&grpc);
#endif
    // grid 1: the large scene's plane and reference box, without the bunnies (meshes, no physics).
    if (opt.large_scene)
      ra::seed_large_demo_scene(app, kRoom, 100.0, 1);
    else
      ra::seed_default_scene(app, kRoom);
    if (opt.use_aoi) {
      aoi = std::make_unique<ra::room_aoi_relay>(app, *shard, kRoom, opt.aoi);
      for (cvc::state_transport *tp : planes)
        aoi->add_transport(tp);
#if defined(CVC_ENABLE_GRPC)
      aoi->add_media_endpoint(&grpc);
#endif
#if defined(CVC_ENABLE_WS)
      aoi->add_media_endpoint(&ws);
#endif
      ra::write_aoi_params(app, kRoom, opt.aoi);
    }
  }

  ~test_relay() {
    // arisrv's shutdown order: the driver first, while the transports still run.
    aoi.reset();
#if defined(CVC_ENABLE_GRPC) && defined(CVC_ENABLE_WS)
    grpc.set_bridge(nullptr);
    ws.set_bridge(nullptr);
#endif
#if defined(CVC_ENABLE_WS)
    ws.unregister_shard(shard.get());
    ws.stop();
#endif
#if defined(CVC_ENABLE_GRPC)
    grpc.unregister_shard(shard.get());
    grpc.stop();
#endif
    shard->adapter().set_on_local_mutation({});
    shard->adapter().set_on_remote_applied({});
    shard->detach();
  }

  test_relay(const test_relay &) = delete;
  test_relay &operator=(const test_relay &) = delete;

  // One room step, as arisrv's loop runs it: retract departures (zeroing the leaver's intent),
  // server_pump, relay_room_step, and the ~1 Hz WebSocket re-broadcast. A participant listed in
  // place_on_join is moved there on the step that adds it, before the area of interest first
  // sees it, and its pose re-written, so it appears there and never on the spawn ring.
  void step() {
    std::vector<std::string> gone;
    {
      std::lock_guard<std::mutex> lk(_mu);
      gone.swap(_departed);
    }
    for (const std::string &nid : gone) {
      ra::room_leave(app, kRoom, nid);
      cvc::ariadne::write_intent(app, kRoom, nid, cvc::ariadne::avatar_intent{});
    }
    cvc::ariadne::server_pump(server, app, kRoom);
    // The clients' joins land on transport threads, so a pump can add a newcomer that the pass
    // before it never saw: place newcomers and re-pump until a pass finds none, so the area of
    // interest below never sees one on the spawn ring.
    for (int pass = 0; pass < 8; ++pass) {
      bool moved = false;
      for (const std::string &pid : server.participants()) {
        if (!_known.insert(pid).second)
          continue;
        joined_at[pid] = now_s();
        const auto it = place_on_join.find(pid);
        if (it != place_on_join.end()) {
          place(pid, it->second.first, it->second.second);
          moved = true;
        }
      }
      if (!moved)
        break;
      cvc::ariadne::server_pump(server, app, kRoom);
    }
    for (auto it = _known.begin(); it != _known.end();)
      it = server.has_participant(*it) ? std::next(it) : _known.erase(it);
    ra::relay_room_step(*shard, planes, aoi.get(), server, now_s(), opt.trim);
    // arisrv's device tags (its ~1 s link pass): each present participant's, from the link
    // holding its pid.
    {
      std::vector<cvc::state_link_report> links;
      for (const cvc::state_transport *tp : planes)
        for (cvc::state_link_report &l : tp->link_reports())
          links.push_back(std::move(l));
      const std::vector<std::string> pids = server.participants();
      ra::write_link_devices(app, kRoom, links, std::set<std::string>(pids.begin(), pids.end()));
    }
    ++steps;
#if defined(CVC_ENABLE_WS)
    if (opt.ws_broadcast && now_s() >= _next_broadcast) {
      ws.broadcast_snapshot();
      _next_broadcast = now_s() + 1.0;
    }
#endif
  }

  // The authority moves PID: teleported to (X, Y) standing still, or walking at (VX, VY) m/s.
  void place(const std::string &pid, double x, double y) {
    cvc::physics::body_state s;
    s.position = {x, y, 0.0};
    s.active = true;
    server.override_state(pid, s);
    walk(pid, 0.0, 0.0);
  }
  void walk(const std::string &pid, double vx, double vy) {
    cvc::ariadne::avatar_intent in;
    in.move = {vx, vy, 0.0};
    double &yaw = _yaw[pid];
    if (vx != 0.0 || vy != 0.0)
      yaw = std::atan2(vy, vx);
    in.yaw = yaw; // a stop keeps the facing
    server.set_intent(pid, in);
  }

  cvc::physics::vec3 position(const std::string &pid) const { return server.state(pid).position; }
  bool tree_pose(const std::string &pid, cvc::physics::body_state &s, std::uint64_t &tick) {
    return cvc::ariadne::read_pose(app, kRoom, pid, s, tick);
  }
  ra::aoi_tier tier(const std::string &receiver, const std::string &subject) const {
    const auto snap = aoi ? aoi->snapshot() : nullptr;
    return snap ? snap->tier(receiver, subject) : ra::aoi_tier::inner;
  }

  // The relay journal's last sequence; the relay's own pose writes for PID journaled in
  // (AFTER, THROUGH]; and how many pose writes it has journaled in all.
  std::uint64_t journal_sequence() const { return shard->journal().last_sequence(); }
  std::vector<std::uint64_t> pose_sequences(const std::string &pid, std::uint64_t after,
                                            std::uint64_t through) const {
    std::lock_guard<std::mutex> lk(_mu);
    std::vector<std::uint64_t> out;
    const auto it = _pose_seq.find(pid);
    if (it != _pose_seq.end())
      for (std::uint64_t s : it->second)
        if (s > after && s <= through)
          out.push_back(s);
    return out;
  }
  std::uint64_t pose_writes() const {
    std::lock_guard<std::mutex> lk(_mu);
    return _pose_writes;
  }

  // Totals over both planes.
  std::uint64_t egress_bytes() const {
    std::uint64_t b = 0;
    for (const cvc::state_transport *tp : planes)
      for (const cvc::state_link_report &r : tp->link_reports())
        b += r.stats.bytes_out;
    return b;
  }
  std::uint64_t withheld() const {
    std::uint64_t n = 0;
#if defined(CVC_ENABLE_GRPC)
    n += grpc.total_relay_withheld();
#endif
#if defined(CVC_ENABLE_WS)
    n += ws.total_relay_withheld();
#endif
    return n;
  }
  std::uint64_t media_gated() const {
    std::uint64_t n = 0;
#if defined(CVC_ENABLE_GRPC)
    n += grpc.total_media_gated();
#endif
#if defined(CVC_ENABLE_WS)
    n += ws.total_media_gated();
#endif
    return n;
  }
  std::uint64_t backfilled() const {
    std::uint64_t n = 0;
#if defined(CVC_ENABLE_GRPC)
    n += grpc.total_backfilled();
#endif
#if defined(CVC_ENABLE_WS)
    n += ws.total_backfilled();
#endif
    return n;
  }
  // Admissions the hold delayed, and those it released without a filter (timed out).
  std::uint64_t admissions_held() const {
    std::uint64_t n = 0;
#if defined(CVC_ENABLE_GRPC)
    n += grpc.total_admissions_held();
#endif
#if defined(CVC_ENABLE_WS)
    n += ws.total_admissions_held();
#endif
    return n;
  }
  std::uint64_t admission_timeouts() const {
    std::uint64_t n = 0;
#if defined(CVC_ENABLE_GRPC)
    n += grpc.total_admission_timeouts();
#endif
#if defined(CVC_ENABLE_WS)
    n += ws.total_admission_timeouts();
#endif
    return n;
  }
  // The paths of the client mutations the relay applied, as their origin named them.
  std::vector<std::string> applied_from(const std::string &origin) const {
    std::lock_guard<std::mutex> lk(_mu);
    const auto it = _applied.find(origin);
    return it == _applied.end() ? std::vector<std::string>{} : it->second;
  }
  // Connections refused for the room token.
  // Device proofs the planes verified, and refused.
  std::uint64_t devices_verified() const {
    std::uint64_t n = 0;
    for (const cvc::state_transport *tp : planes)
      n += tp->total_devices_verified();
    return n;
  }
  std::uint64_t device_proofs_refused() const {
    std::uint64_t n = 0;
    for (const cvc::state_transport *tp : planes)
      n += tp->total_device_proofs_refused();
    return n;
  }
  std::uint64_t auth_rejected() const {
    std::uint64_t n = 0;
#if defined(CVC_ENABLE_GRPC)
    n += grpc.total_auth_rejected();
#endif
#if defined(CVC_ENABLE_WS)
    n += ws.total_auth_rejected();
#endif
    return n;
  }
  // Client frames the bound-writes gate refused.
  std::uint64_t inbound_rejected() const {
    std::uint64_t n = 0;
    for (const cvc::state_transport *tp : planes)
      n += tp->total_inbound_rejected();
    return n;
  }

  relay_options opt;
  std::shared_ptr<cvc::state_inbound_gate> bind_gate; // one for both planes (null: off)
  // Device-key pins, in memory, for both planes (arisrv keeps one always; --device-pins a file).
  std::shared_ptr<cvc::state_device_pins> pins = std::make_shared<cvc::state_device_pins>();
  // Each viewer's video rung on both planes, as arisrv shapes them (declared before the planes, so
  // it outlives every fan that calls it). A test sets rungs directly; arisrv's controller does.
  cvc::media_rung_shaper shaper;
  cvc::app app;
  std::unique_ptr<cvc::state_cluster_shard> shard;
#if defined(CVC_ENABLE_GRPC)
  cvc::state_transport_grpc grpc;
#endif
#if defined(CVC_ENABLE_WS)
  cvc::state_transport_ws ws;
#endif
  cvc::media_router router; // after the transports: it goes first, clearing their sinks
  holon server{holon_role::server, cvc::ariadne::room_holon_config()};
  std::unique_ptr<ra::room_aoi_relay> aoi;
  std::vector<cvc::state_transport *> planes;
  std::map<std::string, std::pair<double, double>> place_on_join;
  std::map<std::string, double> joined_at; // when the server holon (and so the AOI) first had it
  std::uint64_t steps = 0;

private:
  void note_local(const cvc::state_mutation &m) {
    std::string subject, family;
    if (m.op != cvc::state_mutation_op::set_value ||
        !split_participant_path(m.path, subject, family) || family != "pose")
      return;
    std::lock_guard<std::mutex> lk(_mu);
    _pose_seq[subject].push_back(m.sequence);
    ++_pose_writes;
  }

  mutable std::mutex _mu;
  std::vector<std::string> _departed;
  std::map<std::string, std::vector<std::uint64_t>> _pose_seq;
  std::map<std::string, std::vector<std::string>> _applied; // remote origin -> applied paths
  std::uint64_t _pose_writes = 0;
  std::set<std::string> _known;
  std::map<std::string, double> _yaw;
  double _next_broadcast = 0.0;
};

// A client, as arigl builds one: its shard rooted at ROOT ("conf"; "" is a client from before
// S7, which observes and sends its whole tree), interest "conf" enforced, a client holon.
class test_client {
public:
  test_client(std::string id, link_kind k, std::string root = "conf")
      : pid(std::move(id)), kind(k) {
    shard = std::make_unique<cvc::state_cluster_shard>(app, "conf", pid, std::move(root));
    shard->add_interest("conf");
    shard->set_enforce_interest(true);
    shard->attach();
    shard->adapter().set_on_remote_applied(
        [this](const cvc::state_mutation &m) { tap.on_applied(m); });
  }
  ~test_client() {
    disconnect();
    shard->adapter().set_on_remote_applied({});
    shard->detach();
  }
  test_client(const test_client &) = delete;
  test_client &operator=(const test_client &) = delete;

  bool connect(test_relay &r) {
    const auto on_media = [this](const cvc::media_frame &f) { tap.on_media(f); };
#if defined(CVC_ENABLE_GRPC)
    if (kind == link_kind::grpc) {
      _grpc = std::make_unique<cvc::state_transport_grpc>();
      _grpc->set_auth_config({std::string(), token});
      _grpc->set_device_id(device);
      _grpc->set_device_signer(cvc::state_device_signer::derive(seed, r.grpc.listen_address()));
      _grpc->start("127.0.0.1:0", pid, "conf"); // gRPC must start() even to dial
      _grpc->register_shard(shard.get());
      _grpc->set_on_media(on_media);
      tp = _grpc.get();
      return _grpc->connect_to_peer(r.grpc.listen_address(), std::chrono::milliseconds(4000));
    }
#endif
#if defined(CVC_ENABLE_WS)
    if (kind == link_kind::ws) {
      _ws = std::make_unique<cvc::state_transport_ws>();
      _ws->set_auth_config({std::string(), token});
      _ws->set_device_id(device);
      _ws->set_device_signer(cvc::state_device_signer::derive(seed, r.ws.listen_address()));
      _ws->register_shard(shard.get());
      _ws->set_on_media(on_media);
      _ws->set_media_backlog_drop(0); // its uplink is lossless too
      tp = _ws.get();
      return _ws->connect_to_peer("ws://" + r.ws.listen_address(), std::chrono::milliseconds(4000));
    }
#endif
    (void)r;
    (void)on_media;
    return false;
  }

  void disconnect() {
#if defined(CVC_ENABLE_GRPC)
    if (_grpc) {
      _grpc->stop();
      _grpc->unregister_shard(shard.get());
      _grpc.reset();
    }
#endif
#if defined(CVC_ENABLE_WS)
    if (_ws) {
      _ws->stop();
      _ws->unregister_shard(shard.get());
      _ws.reset();
    }
#endif
    tp = nullptr;
  }

  void join() {
    ra::room_join(app, kRoom, pid, pid);
    if (tp)
      tp->pump_all();
  }

  // One client frame: the holon against the replicated room, the marker echo when on, then this
  // frame's own writes sent.
  void frame() {
    cvc::ariadne::client_pump(h, app, kRoom, pid);
    if (echo)
      write_echo();
    if (tp)
      tp->pump_all();
  }

  void set_media(bool on) {
    _media = on;
    _next_cam = _next_mic = now_s();
  }
  // The camera and mic frames due by NOW (seconds), at their real rates; a client that fell
  // behind skips the backlog rather than bursting it.
  void send_due_media(double now) {
    if (!_media || !tp)
      return;
    if (now >= _next_cam) {
      send("camera", "mjpeg", kCameraBytes, cam_sent++);
      _next_cam += 1.0 / kCameraHz;
      if (_next_cam < now)
        _next_cam = now + 1.0 / kCameraHz;
    }
    if (now >= _next_mic) {
      send("mic", "f32", kMicBytes, mic_sent++);
      _next_mic += 1.0 / kMicHz;
      if (_next_mic < now)
        _next_mic = now + 1.0 / kMicHz;
    }
  }

  bool tree_pose(const std::string &q, cvc::physics::body_state &s, std::uint64_t &tick) {
    return cvc::ariadne::read_pose(app, kRoom, q, s, tick);
  }
  std::uint64_t bytes_in() const {
    std::uint64_t b = 0;
    if (tp)
      for (const cvc::state_link_report &r : tp->link_reports())
        b += r.stats.bytes_in;
    return b;
  }

  const std::string pid;
  const link_kind kind;
  cvc::app app;
  client_tap tap;
  std::unique_ptr<cvc::state_cluster_shard> shard;
  holon h{holon_role::client, cvc::ariadne::room_holon_config()};
  cvc::state_transport *tp = nullptr;
  bool echo = false; // write arigl's avatar marker matrices into ui.docs every frame
  std::string token; // the room token its connection presents ("" = none)
  std::string device; // the device id its Hello carries ("" = none)
  std::string seed;   // its device key's 32-byte seed (state_device_key.h; "" = no key)
  std::uint64_t cam_sent = 0, mic_sent = 0;

private:
  void send(const char *stream, const char *codec, std::size_t bytes, std::uint64_t seq) {
    cvc::media_frame f;
    f.origin_node_id = pid;
    f.stream_id = stream;
    f.codec = codec;
    if (std::string(stream) == "camera") {
      f.width = 640;
      f.height = 480;
    } else {
      f.sample_rate = 48000;
      f.channels = 1;
    }
    f.seq = static_cast<std::int64_t>(seq);
    f.pts_seconds = now_s();
    f.payload.assign(bytes, static_cast<std::uint8_t>(0x5a));
#if defined(CVC_ENABLE_GRPC)
    if (_grpc)
      _grpc->publish_media_frame(f);
#endif
#if defined(CVC_ENABLE_WS)
    if (_ws)
      _ws->publish_media_frame(f);
#endif
  }

  // room_view's per-frame write of each shown avatar's body and visor marker matrices into the
  // document's UI tree: a moving avatar rewrites both every frame. A shard rooted at the room
  // tree never observes it; one observing the whole tree journals and sends every write.
  void write_echo() {
    char buf[32];
    for (const std::string &q : h.participants()) {
      if (!h.pose_known(q))
        continue;
      const cvc::physics::pose p = h.display_pose(q);
      const cvc::physics::quat &r = p.rotation;
      // Column-major, as the renderer's 4x4 is: the rotation's columns, then the translation.
      double m[16] = {};
      m[0] = 1 - 2 * (r.y * r.y + r.z * r.z);
      m[1] = 2 * (r.x * r.y + r.w * r.z);
      m[2] = 2 * (r.x * r.z - r.w * r.y);
      m[4] = 2 * (r.x * r.y - r.w * r.z);
      m[5] = 1 - 2 * (r.x * r.x + r.z * r.z);
      m[6] = 2 * (r.y * r.z + r.w * r.x);
      m[8] = 2 * (r.x * r.z + r.w * r.y);
      m[9] = 2 * (r.y * r.z - r.w * r.x);
      m[10] = 1 - 2 * (r.x * r.x + r.y * r.y);
      m[12] = p.position.x;
      m[13] = p.position.y;
      m[14] = p.position.z + 0.85; // the marker's centre, half an avatar up
      m[15] = 1.0;
      for (const char *part : {"avatar_", "visor_"}) {
        std::string v;
        for (double d : m) {
          std::snprintf(buf, sizeof buf, "%.17g ", d);
          v += buf;
        }
        app.root()(std::string("ui.docs.ari.room.") + part + q + ".matrix").value(v);
      }
    }
  }

#if defined(CVC_ENABLE_GRPC)
  std::unique_ptr<cvc::state_transport_grpc> _grpc;
#endif
#if defined(CVC_ENABLE_WS)
  std::unique_ptr<cvc::state_transport_ws> _ws;
#endif
  bool _media = false;
  double _next_cam = 0.0, _next_mic = 0.0;
};

using client_list = std::vector<std::unique_ptr<test_client>>;

// Clients for PIDS, taking the build's transports in turn.
client_list make_clients(std::initializer_list<const char *> pids,
                         const std::string &root = "conf") {
  const std::vector<link_kind> kinds = link_kinds();
  client_list out;
  for (const char *p : pids)
    out.push_back(std::make_unique<test_client>(p, kinds[out.size() % kinds.size()], root));
  return out;
}
std::vector<test_client *> ptrs(const client_list &cs) {
  std::vector<test_client *> out;
  for (const auto &c : cs)
    out.push_back(c.get());
  return out;
}

// Run the room in real time for up to SECONDS: every 50 ms SCRIPT (given the seconds since the
// start) and then a room step, every 1/30 s a frame of each client, and each client's media when
// due. With UNTIL it returns true as soon as UNTIL holds (checked after each round of frames) and
// false if it never did; without, it runs the whole time. A runner that falls behind skips ahead
// instead of bursting steps.
bool run_room(test_relay &r, const std::vector<test_client *> &cs, double seconds,
              const std::function<void(double)> &script = {},
              const std::function<bool()> &until = {}) {
  const double t0 = now_s();
  double next_step = t0, next_frame = t0;
  for (;;) {
    const double t = now_s();
    if (t >= next_step) {
      if (script)
        script(t - t0);
      r.step();
      next_step += kRoomStepS;
      if (next_step < t)
        next_step = t + kRoomStepS;
    }
    if (t >= next_frame) {
      for (test_client *c : cs)
        c->frame();
      next_frame += kFrameS;
      if (next_frame < t)
        next_frame = t + kFrameS;
      if (until && until())
        return true;
    }
    for (test_client *c : cs)
      c->send_due_media(t);
    if (t - t0 >= seconds)
      return until ? until() : true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

// Every client is in the server holon and (with an AOI) in its snapshot, with the driver's filter
// on every plane's registry; and every client holds a known pose for every other one.
bool settled(test_relay &r, const std::vector<test_client *> &cs) {
  const auto snap = r.aoi ? r.aoi->snapshot() : nullptr;
  for (test_client *c : cs) {
    if (!r.server.has_participant(c->pid))
      return false;
    if (r.aoi) {
      if (!snap)
        return false;
      const auto &ps = snap->participants();
      if (std::find(ps.begin(), ps.end(), c->pid) == ps.end())
        return false;
      for (const cvc::state_transport *tp : r.planes)
        if (!tp->peers().view_of(c->pid).filter)
          return false;
    }
    for (test_client *q : cs)
      if (!c->h.has_participant(q->pid) || !c->h.pose_known(q->pid))
        return false;
  }
  return true;
}

// Connect and join every client, then run the room until it settles. A room that starts with its
// clients starts their replicas' clocks with the relay's (whose clock starts on its first step's
// server_pump), as clients up when their server starts. Started at each client's first frame
// instead, after that step -- the joins make it slow, 0.4 to 0.6 s in a Debug gRPC build -- a
// replica ran 50 to 70 ticks behind the relay: its first sample, stamped during the step, was
// within max_skew_ticks, so holon::apply_authoritative left it there until a fresh sample showed
// the whole lag and re-aligned it.
void bring_up(test_relay &r, const std::vector<test_client *> &cs) {
  for (test_client *c : cs) {
    ASSERT_TRUE(c->connect(r)) << c->pid << " could not reach the relay";
    c->join();
  }
  if (r.steps == 0)
    for (test_client *c : cs)
      c->h.advance();
  ASSERT_TRUE(run_room(r, cs, 8.0, {}, [&] { return settled(r, cs); })) << "the room never settled";
}

// A walk to and fro along one axis, AMPLITUDE either side of ANCHOR, at SPEED m/s.
struct patrol {
  double anchor = 0.0;
  double amplitude = 1.0;
  double speed = 1.5;
  bool along_y = false;
  double dir = 1.0;

  void drive(test_relay &r, const std::string &pid) {
    const cvc::physics::vec3 p = r.position(pid);
    if (((along_y ? p.y : p.x) - anchor) * dir >= amplitude)
      dir = -dir;
    r.walk(pid, along_y ? 0.0 : dir * speed, along_y ? dir * speed : 0.0);
  }
};

// ---------------------------------------------------------------------------------------------
// The default room under the conference preset: every pair is inner, so the area of interest
// must change nothing a client receives.

struct default_room_run {
  std::size_t pose_expected = 0; // other subjects' relay pose writes, summed over receivers
  std::size_t pose_missing = 0;  // ... that never arrived live
  double media = 0.0;            // the least-served (receiver, publisher, stream): got / sent
  std::uint64_t gated = 0;       // receiver deliveries the media gates denied
  ra::room_aoi_relay::stats aoi;
};

// N=4 standing on the spawn ring or at the corners (+-8, +-8: 16 to 22.6 m apart), each walking a
// metre either side of where it stands, with camera and mic on for a second.
default_room_run run_default_room(bool use_aoi, bool corners) {
  default_room_run out;
  relay_options o;
  o.use_aoi = use_aoi;
  test_relay r(o);
  const client_list own = make_clients({"A", "B", "C", "D"});
  const std::vector<test_client *> cs = ptrs(own);
  if (corners)
    r.place_on_join = {
        {"A", {8.0, 8.0}}, {"B", {-8.0, 8.0}}, {"C", {-8.0, -8.0}}, {"D", {8.0, -8.0}}};
  bring_up(r, cs);
  if (::testing::Test::HasFatalFailure())
    return out;
  std::map<std::string, patrol> walks;
  for (test_client *c : cs)
    walks[c->pid].anchor = r.position(c->pid).x;
  const auto script = [&](double) {
    for (auto &kv : walks)
      kv.second.drive(r, kv.first);
  };
  run_room(r, cs, 0.3, script);
  const std::uint64_t s0 = r.journal_sequence();
  for (test_client *c : cs)
    c->set_media(true);
  run_room(r, cs, 1.0, script);
  for (test_client *c : cs) {
    c->set_media(false);
    r.walk(c->pid, 0.0, 0.0);
  }
  const std::uint64_t s1 = r.journal_sequence();

  const auto tally = [&] {
    out.pose_expected = out.pose_missing = 0;
    out.media = 1.0;
    for (test_client *rc : cs)
      for (test_client *q : cs) {
        if (q == rc)
          continue;
        const std::vector<std::uint64_t> want = r.pose_sequences(q->pid, s0, s1);
        out.pose_expected += want.size();
        out.pose_missing += want.size() - rc->tap.live_pose_received(q->pid, want);
        for (const char *stream : {"camera", "mic"}) {
          const std::uint64_t sent = std::string(stream) == "camera" ? q->cam_sent : q->mic_sent;
          if (sent > 0)
            out.media =
                (std::min)(out.media, static_cast<double>(rc->tap.media(q->pid, stream)) / sent);
        }
      }
    return out.pose_missing == 0 && out.media >= 1.0;
  };
  // Everything the window sent lands (the room keeps running, as the relay would). A loaded runner
  // may take a while to drain it; the wait ends as soon as it has.
  run_room(r, cs, 8.0, {}, tally);
  tally();
  out.gated = r.media_gated();
  if (r.aoi)
    out.aoi = r.aoi->take_stats();
  return out;
}

} // namespace

// The 20 m room's every pair is inner under the conference preset, so live pose, full video and
// full audio reach every receiver exactly as without the area of interest: each receiver gets
// every other subject's pose mutation live and every camera and mic frame, while the gate passes
// every frame whole and the driver never pushes.
TEST(RoomRelayLoopbackTest, DefaultRoomUnchangedUnderConferencePreset) {
  for (const bool corners : {false, true}) {
    SCOPED_TRACE(corners ? "corners" : "spawn ring");
    const default_room_run off = run_default_room(false, corners);
    ASSERT_FALSE(HasFatalFailure());
    const default_room_run on = run_default_room(true, corners);
    ASSERT_FALSE(HasFatalFailure());
    std::printf("[ room ] default room (%s): pose %zu/%zu missing off, %zu/%zu on; media %.3f off, "
                "%.3f on\n",
                corners ? "corners" : "spawn", off.pose_missing, off.pose_expected, on.pose_missing,
                on.pose_expected, off.media, on.media);
    EXPECT_GT(off.pose_expected, 0u);
    EXPECT_GT(on.pose_expected, 0u);
    EXPECT_EQ(off.pose_missing, 0u);
    EXPECT_EQ(on.pose_missing, 0u) << "an inner pair's live pose is never withheld";
    EXPECT_EQ(off.media, 1.0) << "every frame lands: media is lossless here";
    EXPECT_EQ(on.media, 1.0) << "the gate passes every inner frame";
    EXPECT_EQ(on.gated, 0u) << "the gate denies no inner receiver";
    EXPECT_EQ(on.aoi.inner, 6u);
    EXPECT_EQ(on.aoi.middle, 0u);
    EXPECT_EQ(on.aoi.outer, 0u);
    EXPECT_GT(on.aoi.media_full, 0u);
    EXPECT_EQ(on.aoi.media_thumb, 0u);
    EXPECT_EQ(on.aoi.media_dropped, 0u);
    EXPECT_EQ(on.aoi.pushes, 0u) << "pairs that start inner are never pushed";
  }
}

// Hall preset: C walks 30 m from B (34 from A). B and A get no live pose frame for C, but a
// targeted push about every 24 ticks while C moves, which keeps their view of C walking; they get
// none of C's video or audio, while their own (inner) media still flows.
TEST(RoomRelayLoopbackTest, OuterPeerGetsPushesNotLiveFrames) {
  relay_options o;
  o.aoi = ra::hall_aoi_params();
  o.large_scene = true;
  test_relay r(o);
  const client_list own = make_clients({"A", "B", "C"});
  const std::vector<test_client *> cs = ptrs(own);
  test_client &A = *cs[0], &B = *cs[1], &C = *cs[2];
  r.place_on_join = {{"A", {-2.0, 10.0}}, {"B", {2.0, 10.0}}, {"C", {32.0, 10.0}}};
  ASSERT_NO_FATAL_FAILURE(bring_up(r, cs));
  ASSERT_EQ(r.tier("B", "C"), ra::aoi_tier::outer);
  ASSERT_EQ(r.tier("A", "C"), ra::aoi_tier::outer);
  ASSERT_EQ(r.tier("A", "B"), ra::aoi_tier::inner);

  const std::uint64_t live_b = B.tap.live_pose("C"), live_a = A.tap.live_pose("C");
  const std::uint64_t pushed_b = B.tap.pushed_ticks("C");
  A.set_media(true);
  C.set_media(true);
  patrol walk;
  walk.anchor = 10.0;
  walk.amplitude = 1.5;
  walk.speed = 2.0;
  walk.along_y = true;
  double max_error = 0.0;
  std::set<long long> seen; // C's y in B's tree, in millimetres
  // Two seconds, and on until C has sent ten camera frames: a client sends one a loop pass at
  // most, and a loaded Debug gRPC runner's loop managed four a second.
  constexpr double kSeconds = 2.0;
  const double t0 = now_s();
  run_room(
      r, cs, 8.0,
      [&](double) {
        cvc::physics::body_state rs, bs;
        std::uint64_t rt = 0, bt = 0;
        if (r.tree_pose("C", rs, rt) && B.tree_pose("C", bs, bt)) {
          max_error = (std::max)(max_error, xy_distance(rs.position, bs.position));
          seen.insert(std::llround(bs.position.y * 1000.0));
        }
        walk.drive(r, "C");
      },
      [&] { return now_s() - t0 >= kSeconds && C.cam_sent >= 10; });
  const double window = now_s() - t0;
  A.set_media(false);
  C.set_media(false);
  r.walk("C", 0.0, 0.0);
  run_room(r, cs, 0.3); // what is in flight lands

  EXPECT_EQ(B.tap.live_pose("C"), live_b) << "an outer peer's live pose frames are withheld";
  EXPECT_EQ(A.tap.live_pose("C"), live_a);
  // About 5 a second while C walks (one every 24 ticks). A loaded runner runs fewer room steps,
  // so the bounds are wide: 1.5 to 12 a second, over the window's wall time.
  const std::uint64_t pushes = B.tap.pushed_ticks("C") - pushed_b;
  EXPECT_GE(pushes, static_cast<std::uint64_t>(1.5 * window)) << window << " s";
  EXPECT_LE(pushes, static_cast<std::uint64_t>(12.0 * window)) << window << " s";
  EXPECT_GE(seen.size(), 3u) << "the pushes keep B's view of C moving";
  // At 2 m/s and a push every 0.2 s B's view trails by about 0.4 m.
  EXPECT_LE(max_error, 1.5);
  for (test_client *rc : {&A, &B}) {
    EXPECT_EQ(rc->tap.media("C", "camera"), 0u) << rc->pid;
    EXPECT_EQ(rc->tap.media("C", "mic"), 0u) << rc->pid;
  }
  // C did send all along.
  EXPECT_GE(C.cam_sent, 10u);
  EXPECT_GT(B.tap.media("A", "camera"), 0u) << "the inner peer's media still flows";
  EXPECT_GT(B.tap.media("A", "mic"), 0u);
}

// C stands still, outer for B (hall preset), so B holds C's pose only from pushes, while B walks
// to it. On the step the pair turns inner the relay pushes C's whole pose stamped with its CURRENT
// tick, ahead of that step's live writes: within two steps B's tree has C exactly as the relay
// does, with a tick at least that fresh, and B's replica never re-aligns its clock (a stale tick
// would make it seek, and rubber-band its own avatar). B's clock runs in step with the relay's
// (bring_up), so a heartbeat-old tick is well past max_skew_ticks from it.
TEST(RoomRelayLoopbackTest, StillOuterPeerReentryNoRealignNoFreeze) {
  relay_options o;
  o.aoi = ra::hall_aoi_params();
  o.large_scene = true;
  test_relay r(o);
  const client_list own = make_clients({"B", "C"});
  const std::vector<test_client *> cs = ptrs(own);
  test_client &B = *cs[0];
  // 18 m apart: outer (beyond middle_enter 16 m), about two seconds' walk from inner (10 m).
  r.place_on_join = {{"B", {14.0, 10.0}}, {"C", {32.0, 10.0}}};
  ASSERT_NO_FATAL_FAILURE(bring_up(r, cs));
  ASSERT_EQ(r.tier("B", "C"), ra::aoi_tier::outer);
  run_room(r, cs, 0.5);
  const std::uint64_t realigns = B.h.clock_realigns();
  const auto lag = static_cast<long long>(r.server.tick()) - static_cast<long long>(B.h.tick());

  r.walk("B", 4.0, 0.0); // the veto's cap
  bool entered = false;
  std::uint64_t entered_tick = 0, entered_step = 0, pushed_before = 0, matched_step = 0;
  std::uint64_t pushed_last = B.tap.pushed_ticks("C"); // as of before the latest step
  const auto fresh = [&] {
    if (!entered)
      return false;
    cvc::physics::body_state rs, bs;
    std::uint64_t rt = 0, bt = 0;
    if (B.tap.max_tick("C") < entered_tick || !r.tree_pose("C", rs, rt) ||
        !B.tree_pose("C", bs, bt))
      return false;
    if (std::fabs(rs.position.x - bs.position.x) > 1e-4 ||
        std::fabs(rs.position.y - bs.position.y) > 1e-4 ||
        std::fabs(rs.position.z - bs.position.z) > 1e-4)
      return false;
    matched_step = r.steps;
    return true;
  };
  ASSERT_TRUE(run_room(
      r, cs, 8.0,
      [&](double) {
        // Runs before each step: the previous step's snapshot, and the tick it pushed with. That
        // step's push may have landed already, so the count to beat is the one from before it.
        if (!entered && r.tier("B", "C") == ra::aoi_tier::inner) {
          entered = true;
          entered_tick = r.server.tick();
          entered_step = r.steps;
          pushed_before = pushed_last;
          r.walk("B", 0.0, 0.0);
        }
        pushed_last = B.tap.pushed_ticks("C");
      },
      fresh))
      << (entered ? "B never got C's fresh pose" : "B never reached C");
  // A step is 50 ms and loopback delivers in milliseconds.
  EXPECT_LE(matched_step - entered_step, 2u);
  EXPECT_GT(B.tap.pushed_ticks("C"), pushed_before) << "the entry pose came as a push";

  run_room(r, cs, 0.5); // inner now: live writes
  EXPECT_EQ(B.h.clock_realigns(), realigns)
      << "a pushed tick never re-aligns the replica (B's clock was " << lag
      << " ticks behind the relay's before the walk)";
  EXPECT_TRUE(B.h.pose_known("C"));
  cvc::physics::body_state rs;
  std::uint64_t rt = 0;
  ASSERT_TRUE(r.tree_pose("C", rs, rt));
  EXPECT_LE(xy_distance(B.h.state("C").position, rs.position), 1e-4) << "C is not frozen elsewhere";
}

// Hall preset, C 13 m from B: middle. B gets C's camera as a thumbnail -- the first frame at
// once, then about 2 a second -- and every mic chunk; A, 17 m from C and so outer, gets neither.
TEST(RoomRelayLoopbackTest, MiddleGetsThumbnailsAndAudio) {
  relay_options o;
  o.aoi = ra::hall_aoi_params();
  o.large_scene = true;
  test_relay r(o);
  const client_list own = make_clients({"A", "B", "C"});
  const std::vector<test_client *> cs = ptrs(own);
  test_client &A = *cs[0], &B = *cs[1], &C = *cs[2];
  // B-C 13 m (middle: inside middle_enter 16, beyond inner_enter 10); A-C 17 m (outer).
  r.place_on_join = {{"A", {-2.0, 10.0}}, {"B", {2.0, 10.0}}, {"C", {15.0, 10.0}}};
  ASSERT_NO_FATAL_FAILURE(bring_up(r, cs));
  ASSERT_EQ(r.tier("B", "C"), ra::aoi_tier::middle);
  ASSERT_EQ(r.tier("A", "C"), ra::aoi_tier::outer);

  // Two seconds, and on until C has sent 31 camera frames, many times what the gate passes: a
  // client sends one a loop pass at most, and a loaded Debug gRPC runner's loop managed ten a
  // second.
  constexpr double kSeconds = 2.0;
  C.set_media(true);
  const double t0 = now_s();
  run_room(r, cs, 8.0, {}, [&] { return now_s() - t0 >= kSeconds && C.cam_sent > 30; });
  const double window = now_s() - t0;
  C.set_media(false);
  // Every chunk lands (the room keeps running meanwhile).
  EXPECT_TRUE(run_room(r, cs, 8.0, {}, [&] { return B.tap.media("C", "mic") >= C.mic_sent; }));

  EXPECT_GT(C.cam_sent, 30u);
  EXPECT_EQ(B.tap.media("C", "mic"), C.mic_sent) << "a middle peer's audio is never thinned";
  // 2 +/- 1 fps over the window, plus the first frame that goes at once. The gate's timer runs on
  // wall time, so the bounds are those of the rate over the window's wall time, not of a frame
  // count.
  const std::uint64_t thumbs = B.tap.media("C", "camera");
  EXPECT_GE(thumbs, static_cast<std::uint64_t>(1.0 * window)) << window << " s";
  EXPECT_LE(thumbs, static_cast<std::uint64_t>(3.0 * window) + 1) << window << " s";
  EXPECT_EQ(A.tap.media("C", "camera"), 0u);
  EXPECT_EQ(A.tap.media("C", "mic"), 0u);
  ASSERT_TRUE(r.aoi);
  EXPECT_GE(r.aoi->take_stats().media_thumb, 2u);
}

// C joins 30 m from A and B (hall preset): the pairs start outer, and the relay pushes C's pose to
// each of them on the very step it first sees C -- not at the 5 s refresh -- so they show C at its
// spot within a step plus a round trip, never at a placeholder: until that push C has no body in
// their holons and no live pose frame ever arrives.
TEST(RoomRelayLoopbackTest, NewOuterPairPushedImmediately) {
  relay_options o;
  o.aoi = ra::hall_aoi_params();
  o.large_scene = true;
  test_relay r(o);
  client_list own = make_clients({"A", "B", "C"});
  test_client &A = *own[0], &B = *own[1], &C = *own[2];
  r.place_on_join = {{"A", {-2.0, 10.0}}, {"B", {2.0, 10.0}}, {"C", {32.0, 10.0}}};
  std::vector<test_client *> cs{&A, &B};
  ASSERT_NO_FATAL_FAILURE(bring_up(r, cs));

  ASSERT_TRUE(C.connect(r));
  C.join();
  cs.push_back(&C);
  bool ghost = false;
  double arrived = -1.0;
  ASSERT_TRUE(run_room(r, cs, 4.0, {}, [&] {
    for (test_client *rc : {&A, &B})
      if (rc->h.has_participant("C") && !rc->h.pose_known("C") && rc->h.has_body("C"))
        ghost = true;
    if (arrived < 0.0 && A.h.pose_known("C") && B.h.pose_known("C"))
      arrived = now_s();
    return arrived >= 0.0;
  }));
  ASSERT_EQ(r.joined_at.count("C"), 1u);
  EXPECT_EQ(r.tier("B", "C"), ra::aoi_tier::outer);
  EXPECT_FALSE(ghost) << "no body for C before its first pose";
  // A step plus a round trip; the refresh would be 5 s.
  EXPECT_LT(arrived - r.joined_at["C"], 1.0);
  cvc::physics::body_state rs;
  std::uint64_t rt = 0;
  ASSERT_TRUE(r.tree_pose("C", rs, rt));
  EXPECT_NEAR(rs.position.x, 32.0, 0.05);
  for (test_client *rc : {&A, &B}) {
    cvc::physics::body_state s;
    std::uint64_t t = 0;
    ASSERT_TRUE(rc->tree_pose("C", s, t)) << rc->pid;
    EXPECT_LE(xy_distance(s.position, rs.position), 1e-4) << rc->pid;
    EXPECT_GE(rc->tap.pushed_ticks("C"), 1u) << rc->pid;
    EXPECT_EQ(rc->tap.live_pose("C"), 0u) << rc->pid;
  }
}

// Default room, conference preset. A walks by proposing an intent; D is a client from before S7
// whose shard observes its whole tree, so its avatar-marker echo under ui.docs goes up to the relay
// every frame. The relay applies A's intent, but no other client ever receives it (only the
// participant's own intent is its business), and the relay's interest gate withholds D's ui.docs
// writes, so not one of them reaches another client's shard.
TEST(RoomRelayLoopbackTest, NoIntentOrUiDocsLeak) {
  test_relay r(relay_options{});
  const std::vector<link_kind> kinds = link_kinds();
  client_list own;
  own.push_back(std::make_unique<test_client>("A", kinds[0 % kinds.size()]));
  own.push_back(std::make_unique<test_client>("B", kinds[1 % kinds.size()]));
  own.push_back(std::make_unique<test_client>("D", kinds[2 % kinds.size()], ""));
  const std::vector<test_client *> cs = ptrs(own);
  test_client &A = *cs[0], &B = *cs[1], &D = *cs[2];
  ASSERT_NO_FATAL_FAILURE(bring_up(r, cs));

  const std::uint64_t filtered_a = A.shard->total_remote_filtered_out();
  const std::uint64_t filtered_b = B.shard->total_remote_filtered_out();
  const std::uint64_t refused = r.inbound_rejected();
  const std::uint64_t live_a = B.tap.live_pose("A");
  cvc::ariadne::avatar_intent walk;
  walk.move = {0.5, 0.0, 0.0};
  cvc::ariadne::write_intent(A.app, kRoom, "A", walk);
  D.echo = true;
  run_room(r, cs, 1.0);

  cvc::ariadne::avatar_intent got;
  ASSERT_TRUE(cvc::ariadne::read_intent(r.app, kRoom, "A", got)) << "the relay has A's intent";
  EXPECT_DOUBLE_EQ(got.move.x, 0.5);
  EXPECT_GT(B.tap.live_pose("A"), live_a) << "A walks, and B sees it";
  for (test_client *rc : {&B, &D}) {
    EXPECT_FALSE(cvc::ariadne::read_intent(rc->app, kRoom, "A", got)) << rc->pid;
    EXPECT_EQ(rc->tap.intents("A"), 0u) << rc->pid;
  }
  // D's echo is a write outside its participant, so bound writes refuse it before the interest
  // gate would withhold it.
  EXPECT_GT(r.inbound_rejected(), refused) << "the relay refused D's echo";
  EXPECT_EQ(B.shard->total_remote_filtered_out(), filtered_b) << "no ui.docs frame reached B";
  EXPECT_EQ(A.shard->total_remote_filtered_out(), filtered_a) << "no ui.docs frame reached A";
  EXPECT_EQ(B.app.root().findDescendant("ui.docs"), nullptr);
  EXPECT_EQ(r.app.root().findDescendant("ui.docs"), nullptr) << "ingest keeps only the room";
}

// Bound writes (arisrv --bind-writes, on by default). M writes, from its own replica and through
// its own transport like any client write, A's intent (a walk), A's authoritative pose (a
// teleport), its own authoritative pose, the scene and the area of interest; and it sends, as raw
// mutations, its own pose by paths the tree would normalise onto it ("..pose", "pose ") and A's
// intent naming A as the origin. The relay applies none of it -- every client mutation it applied
// from M lies in M's participant, outside its pose, and none from "A" touches A's intent -- and
// forwards none of it: A stands still and never receives an intent for itself. M's own walk,
// inside its participant, still works.
TEST(RoomRelayLoopbackTest, ClientWritesOnlyItsOwnParticipant) {
  test_relay r(relay_options{});
  client_list own = make_clients({"A", "M"});
  const std::vector<test_client *> cs = ptrs(own);
  test_client &A = *cs[0], &M = *cs[1];
  ASSERT_NO_FATAL_FAILURE(bring_up(r, cs));
  ASSERT_TRUE(run_room(r, cs, 5.0, nullptr, [&] {
    cvc::physics::body_state s;
    std::uint64_t t = 0;
    return M.tree_pose("A", s, t) && M.tree_pose("M", s, t);
  })) << "M holds both poses";

  const std::string a_part = ra::room_participant_path(kRoom, "A");
  const std::string m_part = ra::room_participant_path(kRoom, "M");
  const std::string a_pos = a_part + ".pose.pos.x";
  const std::string m_pos = m_part + ".pose.pos.x";
  const std::string rev = ra::room_scene_path(kRoom) + ".rev";
  const std::string aoi_on = ra::room_aoi_path(kRoom) + ".enabled";
  const auto relay_leaf = [&](const std::string &p) {
    cvc::state *n = r.app.root().findDescendant(p);
    return n ? n->value() : std::string("<absent>");
  };
  cvc::physics::body_state a0;
  std::uint64_t tick = 0;
  ASSERT_TRUE(cvc::ariadne::read_pose(r.app, kRoom, "A", a0, tick));
  const std::string rev0 = relay_leaf(rev), aoi0 = relay_leaf(aoi_on);
  ASSERT_NE(rev0, "<absent>");
  ASSERT_NE(aoi0, "<absent>");
  const std::uint64_t refused = r.inbound_rejected();

  cvc::ariadne::avatar_intent push;
  push.move = {0.9, 0.0, 0.0};
  cvc::ariadne::write_intent(M.app, kRoom, "A", push);
  M.app.root()(a_pos).value(std::string("50"));
  M.app.root()(m_pos).value(std::string("50"));
  M.app.root()(rev).value(std::string("999"));
  M.app.root()(aoi_on).value(std::string(aoi0 == "1" ? "0" : "1"));
  std::uint64_t seq = 1000000; // past anything M or A journals here, so never a duplicate
  const auto send_raw = [&](const std::string &origin, const std::string &path,
                            const std::string &v) {
    cvc::state_mutation m;
    m.cluster_id = "conf";
    m.tree_id = "default";
    m.origin_node_id = origin;
    m.sequence = ++seq;
    m.mutation_id = origin + ":" + std::to_string(m.sequence);
    m.path = path;
    m.op = cvc::state_mutation_op::set_value;
    m.type_name = "std::string";
    m.string_value = v;
    M.tp->publish(m);
  };
  send_raw("M", m_part + "..pose.pos.x", "60");
  send_raw("M", m_part + ".pose .pos.y", "60");
  send_raw("A", a_part + ".intent.move.x", "0.9");
  const std::uint64_t a_intents = A.tap.intents("A");
  run_room(r, cs, 2.0);

  for (const std::string &p : r.applied_from("M")) {
    EXPECT_TRUE(cvc::state_peer_registry::prefix_matches(m_part, p)) << "applied from M: " << p;
    EXPECT_EQ(p.find("pose"), std::string::npos) << "applied from M: " << p;
  }
  for (const std::string &p : r.applied_from("A"))
    EXPECT_EQ(p.find("intent"), std::string::npos) << "applied as A: " << p;
  cvc::ariadne::avatar_intent got;
  EXPECT_FALSE(cvc::ariadne::read_intent(r.app, kRoom, "A", got) && got.move.x != 0.0)
      << "the relay never took an intent for A";
  cvc::physics::body_state a1;
  ASSERT_TRUE(cvc::ariadne::read_pose(r.app, kRoom, "A", a1, tick));
  EXPECT_NEAR(a1.position.x, a0.position.x, 1e-3) << "A did not walk";
  EXPECT_EQ(A.tap.intents("A"), a_intents) << "nothing was forwarded to A as its intent";
  EXPECT_EQ(relay_leaf(rev), rev0) << "the scene is the relay's";
  EXPECT_EQ(relay_leaf(aoi_on), aoi0) << "so is the area of interest";
  EXPECT_GE(r.inbound_rejected() - refused, 8u)
      << "A's intent leaves, both poses, the scene, the area of interest and the raw frames";

  cvc::physics::body_state m0, m1;
  ASSERT_TRUE(cvc::ariadne::read_pose(r.app, kRoom, "M", m0, tick));
  cvc::ariadne::avatar_intent walk;
  walk.move = {0.5, 0.0, 0.0};
  cvc::ariadne::write_intent(M.app, kRoom, "M", walk);
  run_room(r, cs, 2.0);
  ASSERT_TRUE(cvc::ariadne::read_pose(r.app, kRoom, "M", m1, tick));
  EXPECT_GT(m1.position.x - m0.position.x, 0.3) << "M's own intent still moves M";
}

// A room token (arisrv --room-token-file): clients that present it join as before, and each one's
// device shows in the room as its room-scoped tag, the same for every peer, once it proved its
// device key. A client without the token, on each transport the build has, is refused before it
// is sent anything: it never sees the room.
TEST(RoomRelayLoopbackTest, RoomTokenAdmitsOnlyItsHolders) {
  relay_options o;
  o.room_token = "room-token-0123456789";
  test_relay r(o);
  client_list own = make_clients({"A", "B"});
  const std::vector<test_client *> cs = ptrs(own);
  test_client &A = *cs[0], &B = *cs[1];
  A.token = B.token = o.room_token;
  A.device = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
  B.device = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
  A.seed = std::string(32, 'a');
  B.seed = std::string(32, 'b');
  ASSERT_NO_FATAL_FAILURE(bring_up(r, cs));
  if (cvc::device_keys_available()) {
    const std::string tag_a = ra::room_device_tag(kRoom, A.device);
    const std::string tag_b = ra::room_device_tag(kRoom, B.device);
    EXPECT_TRUE(run_room(r, cs, 5.0, {}, [&] {
      return ra::read_participant_device(A.app, kRoom, "B") == tag_b &&
             ra::read_participant_device(B.app, kRoom, "A") == tag_a;
    })) << "each sees the other's device tag";
    EXPECT_EQ(ra::read_participant_device(A.app, kRoom, "A"), tag_a) << "and its own";
  }

  const std::vector<link_kind> kinds = link_kinds();
  client_list outsiders;
  for (std::size_t i = 0; i < kinds.size(); ++i) {
    outsiders.push_back(std::make_unique<test_client>("C" + std::to_string(i), kinds[i]));
    (void)outsiders.back()->connect(r); // WS fails at once; gRPC opens a channel, then is refused
  }
  EXPECT_TRUE(run_room(r, cs, 3.0, {}, [&] { return r.auth_rejected() >= kinds.size(); }));
  run_room(r, cs, 1.0);
  for (const auto &c : outsiders) {
    EXPECT_EQ(c->app.root().findDescendant(ra::room_participant_path(kRoom, "A")), nullptr)
        << c->pid << " was sent nothing of the room";
    for (const cvc::state_transport *tp : r.planes)
      for (const cvc::state_link_report &l : tp->link_reports())
        EXPECT_NE(l.remote_node_id, c->pid) << "and never admitted";
  }
}

// A viewer's video rung (media_rungs.h) shapes what the relay sends it of every camera, on each
// transport: off sends none (the mic still flows), thumb a couple of frames a second, full all of
// them. (The synthetic frames are not real JPEGs, so half and quarter would send them as is; the
// re-encoding has its own tests, MediaRungs.*.)
TEST(RoomRelayLoopbackTest, AViewersVideoRungShapesWhatItGets) {
  for (const link_kind kind : link_kinds()) {
    SCOPED_TRACE(kind == link_kind::grpc ? "grpc" : "ws");
    test_relay r(relay_options{});
    test_client A("A", kind), B("B", kind);
    const std::vector<test_client *> cs = {&A, &B};
    ASSERT_NO_FATAL_FAILURE(bring_up(r, cs));
    A.set_media(true);
    const auto camera = [&] { return B.tap.media("A", "camera"); };
    const auto mic = [&] { return B.tap.media("A", "mic"); };

    // SECONDS of the room, and on until A has sent more than 40 frames of the stream SENT counts
    // (a client sends one a loop pass at most, and a loaded Debug gRPC runner's loop managed 17 a
    // second); then on until B has every one of them, by GOT. Returns what A sent and B got.
    const auto window = [&](double seconds, const std::uint64_t &sent,
                            const std::function<std::uint64_t()> &got) {
      const std::uint64_t sent0 = sent, got0 = got();
      const double t0 = now_s();
      run_room(r, cs, 8.0, {}, [&] { return now_s() - t0 >= seconds && sent - sent0 > 40; });
      const std::uint64_t n = sent - sent0;
      run_room(r, cs, 8.0, {}, [&] { return got() - got0 >= n; });
      return std::make_pair(n, got() - got0);
    };

    r.shaper.set_rung("B", cvc::video_rung::off);
    run_room(r, cs, 0.5); // what was in flight lands
    std::uint64_t cam0 = camera();
    const auto [mic_sent, mic_got] = window(2.0, A.mic_sent, mic);
    EXPECT_EQ(camera() - cam0, 0u) << "off: no camera";
    EXPECT_GT(mic_sent, 40u);
    EXPECT_GE(mic_got, mic_sent) << "but the mic flows";

    r.shaper.set_rung("B", cvc::video_rung::thumb);
    cam0 = camera();
    run_room(r, cs, 3.0);
    const std::uint64_t thumbs = camera() - cam0;
    EXPECT_GE(thumbs, 3u) << "thumb: about 2 a second";
    EXPECT_LE(thumbs, 9u);

    r.shaper.set_rung("B", cvc::video_rung::full);
    const auto [cam_sent, cam_got] = window(2.0, A.cam_sent, camera);
    EXPECT_GT(cam_sent, 40u);
    EXPECT_GE(cam_got, cam_sent) << "full: every frame";
  }
}

// The relay's ~1 Hz WebSocket re-broadcast never sends a client its own intent back. The client is
// the source: the relay's copy is at best what the client holds, and older while the client's next
// write is in flight -- the echo that made a walking client's own avatar jump (it predicted on its
// previous intent until the next broadcast). Here the newer write is held back until after the
// broadcast, so the relay still has the older one.
TEST(RoomRelayLoopbackTest, ABroadcastNeverEchoesAClientsOwnIntent) {
#if !defined(CVC_ENABLE_WS)
  GTEST_SKIP() << "no WebSocket transport in this build";
#else
  test_relay r(relay_options{});
  test_client A("A", link_kind::ws);
  const std::vector<test_client *> cs = {&A};
  ASSERT_NO_FATAL_FAILURE(bring_up(r, cs));
  cvc::ariadne::avatar_intent first, second;
  first.move = {0.5, 0.0, 0.0};
  second.move = {0.0, 0.5, 0.0};
  cvc::ariadne::write_intent(A.app, kRoom, "A", first, 100);
  ASSERT_TRUE(run_room(r, cs, 3.0, {}, [&] {
    cvc::ariadne::avatar_intent got;
    return cvc::ariadne::read_intent(r.app, kRoom, "A", got) && got.move.x == 0.5;
  })) << "the relay has A's first intent";

  // A's second intent, not sent yet; then the relay re-broadcasts the room.
  cvc::ariadne::write_intent(A.app, kRoom, "A", second, 200);
  r.ws.broadcast_snapshot();
  std::this_thread::sleep_for(std::chrono::milliseconds(400)); // A's reader applies what came
  cvc::ariadne::avatar_intent got;
  std::uint64_t stamp = 0;
  ASSERT_TRUE(cvc::ariadne::read_intent(A.app, kRoom, "A", got, &stamp));
  EXPECT_EQ(stamp, 200u) << "A still holds its second intent, not the relay's copy of its first";
  EXPECT_DOUBLE_EQ(got.move.y, 0.5);
  EXPECT_FALSE(cvc::ariadne::read_current_intent(A.app, kRoom, "A", 300, got))
      << "and an intent older than A's last proposal is never taken as current";
#endif
}

// Device keys (state_device_key.h): a device that proves its key gets its tag, on each transport
// the build has. A second device showing the same id under another key (an impostor that learned
// the id) is refused and gets none, nor does a device with an id and no key; and the first device
// coming back with its key matches its pin and gets its tag again.
TEST(RoomRelayLoopbackTest, OnlyAProvenDeviceGetsItsTag) {
  if (!cvc::device_keys_available())
    GTEST_SKIP() << "no device keys in this build";
  const std::string dev = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
  const std::string tag = ra::room_device_tag(kRoom, dev);
  const std::vector<link_kind> kinds = link_kinds();
  for (const link_kind kind : kinds) {
    SCOPED_TRACE(kind == link_kind::grpc ? "grpc" : "ws");
    test_relay r(relay_options{});
    auto A = std::make_unique<test_client>("A", kind);
    auto I =
        std::make_unique<test_client>("I", kinds.back() == kind ? kinds.front() : kinds.back());
    auto N = std::make_unique<test_client>("N", kind);
    A->device = I->device = dev;
    A->seed = std::string(32, 'a');
    I->seed = std::string(32, 'i');
    N->device = "nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn";
    {
      const std::vector<test_client *> first = {A.get()};
      ASSERT_NO_FATAL_FAILURE(bring_up(r, first));
      EXPECT_TRUE(run_room(r, first, 5.0, {}, [&] {
        return ra::read_participant_device(A->app, kRoom, "A") == tag;
      })) << "A proved its key and shows its tag";
    }
    const std::vector<test_client *> cs = {A.get(), I.get(), N.get()};
    for (test_client *c : {I.get(), N.get()}) {
      ASSERT_TRUE(c->connect(r));
      c->join();
    }
    EXPECT_TRUE(run_room(r, cs, 5.0, {}, [&] {
      return r.device_proofs_refused() >= 1 && r.server.has_participant("I") &&
             r.server.has_participant("N");
    })) << "the impostor's key is not A's pin";
    run_room(r, cs, 1.5); // a link pass or more
    for (const char *pid : {"I", "N"})
      EXPECT_EQ(ra::read_participant_device(r.app, kRoom, pid), "") << pid << " has no tag";
    EXPECT_EQ(ra::read_participant_device(r.app, kRoom, "A"), tag) << "while A keeps its own";
    EXPECT_EQ(r.devices_verified(), 1u);

    // The same device again, as a fresh process (which cannot rejoin under its old pid: see
    // ReconnectUnderSamePidGetsZeroedIntent), proves its key against the pin.
    A->disconnect();
    run_room(r, {I.get(), N.get()}, 1.0);
    auto A2 = std::make_unique<test_client>("A", kind);
    A2->device = dev;
    A2->seed = std::string(32, 'a');
    ASSERT_TRUE(A2->connect(r));
    EXPECT_TRUE(run_room(r, {A2.get(), I.get(), N.get()}, 5.0, {}, [&] {
      return r.devices_verified() >= 2;
    })) << "the device itself matches its pin";
    EXPECT_EQ(r.device_proofs_refused(), 1u);
  }
}

// A walks (an intent it proposed), then its connection drops. The relay retracts it and zeroes
// its intent; a fresh process connecting under the same pid bootstraps the zeroed intent -- its
// own subtree is never filtered, and a departed pid is no longer a registered peer -- so it would
// not resume the walk, and B never saw A's intent at all.
//
// It does not JOIN under that pid: replication drops a write whose (origin, sequence) it has seen,
// so a process restarting under the same node id has its writes ignored until the relay restarts
// (which is why arigl gives every launch a random node-id suffix). The relaunch joins as arigl's
// would, under a fresh id, and the room settles around it after the departure.
TEST(RoomRelayLoopbackTest, ReconnectUnderSamePidGetsZeroedIntent) {
  test_relay r(relay_options{});
  client_list own = make_clients({"A", "B"});
  test_client &B = *own[1];
  std::vector<test_client *> cs = ptrs(own);
  ASSERT_NO_FATAL_FAILURE(bring_up(r, cs));

  cvc::ariadne::avatar_intent walk;
  walk.move = {0.6, 0.0, 0.0};
  cvc::ariadne::write_intent(own[0]->app, kRoom, "A", walk);
  const auto relay_intent_x = [&](double &x) {
    cvc::ariadne::avatar_intent in;
    if (!cvc::ariadne::read_intent(r.app, kRoom, "A", in))
      return false;
    x = in.move.x;
    return true;
  };
  double x = 0.0;
  ASSERT_TRUE(run_room(r, cs, 3.0, {}, [&] { return relay_intent_x(x) && x > 0.5; }));
  run_room(r, cs, 0.3);

  const link_kind kind = own[0]->kind;
  own[0].reset(); // the connection closes
  cs = {&B};
  ASSERT_TRUE(run_room(r, cs, 4.0, {}, [&] {
    return !r.server.has_participant("A") && relay_intent_x(x) && x == 0.0;
  })) << "the relay retracts A and zeroes its intent";

  // It never joins, so no filter is installed for it and the admission hold releases it only when
  // the hold (2 s) runs out, unfiltered.
  test_client again("A", kind);
  ASSERT_TRUE(again.connect(r));
  cvc::ariadne::avatar_intent got;
  ASSERT_TRUE(run_room(r, cs, 6.0, {}, [&] {
    return cvc::ariadne::read_intent(again.app, kRoom, "A", got);
  })) << "the bootstrap carries A's own intent";
  EXPECT_EQ(got.move.x, 0.0) << "zeroed, not the walk proposed before the drop";

  test_client relaunch("A_2", kind);
  ASSERT_TRUE(relaunch.connect(r));
  relaunch.join();
  cs.push_back(&relaunch);
  ASSERT_TRUE(run_room(r, cs, 8.0, {}, [&] { return settled(r, cs); }));
  run_room(r, cs, 0.5);
  EXPECT_FALSE(r.server.has_participant("A"));
  const cvc::physics::vec3 v = r.server.state("A_2").linear_velocity;
  EXPECT_LT(std::hypot(v.x, v.y), 0.05) << "the relaunch stands where it spawned";
  EXPECT_EQ(B.tap.intents("A"), 0u) << "B never received A's intent";
  EXPECT_EQ(B.tap.intents("A_2"), 0u);
}

namespace {

struct bootstrap_run {
  std::uint64_t intents = 0; // other participants' intent leaves the late joiner applied
  std::uint64_t held = 0;    // its admission: held, and released by the timeout
  std::uint64_t timeouts = 0;
  bool knows_all = false;       // it came to hold every other participant's pose
  double seconds = -1.0;        // from its join to that
  std::uint64_t steps = 0;      // ... and the room steps that took
  double outer_error = -1.0;    // its view of the outer peer C against the relay's
  std::uint64_t outer_live = 0; // live pose frames it got for C
};

// Hall preset: A and B walk (each proposing an intent) 4 m apart and C stands 30 m off; then D
// joins between A and B, so they are inner for it and C is outer. HOLD: the relay holds each
// admission for its filter, as arisrv does.
bootstrap_run run_late_bootstrap(bool hold) {
  bootstrap_run out;
  relay_options o;
  o.aoi = ra::hall_aoi_params();
  o.large_scene = true;
  o.hold = hold;
  test_relay r(o);
  client_list own = make_clients({"A", "B", "C", "D"});
  test_client &A = *own[0], &B = *own[1], &C = *own[2], &D = *own[3];
  r.place_on_join = {
      {"A", {-2.0, 10.0}}, {"B", {2.0, 10.0}}, {"C", {32.0, 10.0}}, {"D", {0.0, 12.0}}};
  std::vector<test_client *> cs{&A, &B, &C};
  bring_up(r, cs);
  if (::testing::Test::HasFatalFailure())
    return out;
  cvc::ariadne::avatar_intent walk;
  walk.move = {0.3, 0.0, 0.0};
  cvc::ariadne::write_intent(A.app, kRoom, "A", walk);
  walk.move = {-0.3, 0.0, 0.0};
  cvc::ariadne::write_intent(B.app, kRoom, "B", walk);
  cvc::ariadne::avatar_intent got;
  EXPECT_TRUE(run_room(r, cs, 3.0, {}, [&] {
    return cvc::ariadne::read_intent(r.app, kRoom, "B", got) && got.move.x < -0.2;
  })) << "the relay holds the walkers' intents";

  const std::uint64_t held = r.admissions_held(), timeouts = r.admission_timeouts();
  if (!D.connect(r)) {
    ADD_FAILURE() << "D could not reach the relay";
    return out;
  }
  D.join();
  cs.push_back(&D);
  const double t0 = now_s();
  const std::uint64_t step0 = r.steps;
  out.knows_all = run_room(r, cs, 4.0, {}, [&] {
    return D.h.pose_known("A") && D.h.pose_known("B") && D.h.pose_known("C");
  });
  out.seconds = now_s() - t0;
  out.steps = r.steps - step0;
  run_room(r, cs, 0.3);
  out.intents = D.tap.intents("A") + D.tap.intents("B") + D.tap.intents("C");
  out.held = r.admissions_held() - held;
  out.timeouts = r.admission_timeouts() - timeouts;
  cvc::physics::body_state rs, ds;
  std::uint64_t rt = 0, dt = 0;
  if (r.tree_pose("C", rs, rt) && D.tree_pose("C", ds, dt))
    out.outer_error = xy_distance(rs.position, ds.position);
  out.outer_live = D.tap.live_pose("C");
  EXPECT_EQ(r.tier("D", "C"), ra::aoi_tier::outer);
  EXPECT_EQ(r.tier("D", "A"), ra::aoi_tier::inner);
  return out;
}

} // namespace

// A late joiner's admit snapshot is filtered like its live stream. Without the admission hold it
// goes out before the relay knows who the joiner is, so it carries the walkers' intents (and the
// outer peer's pose) once; with the hold it waits a step for the joiner's filter, then carries no
// other participant's intent, and the outer peer arrives by the push its new outer pair is owed,
// in the same step.
TEST(RoomRelayLoopbackTest, LateJoinerBootstrapIsFiltered) {
  const bootstrap_run off = run_late_bootstrap(false);
  ASSERT_FALSE(HasFatalFailure());
  const bootstrap_run on = run_late_bootstrap(true);
  ASSERT_FALSE(HasFatalFailure());
  std::printf("[ room ] late joiner: others' intent leaves %llu without the hold, %llu with it; "
              "every pose in %.2f s (%llu steps), %.2f s (%llu steps)\n",
              static_cast<unsigned long long>(off.intents),
              static_cast<unsigned long long>(on.intents), off.seconds,
              static_cast<unsigned long long>(off.steps), on.seconds,
              static_cast<unsigned long long>(on.steps));
  EXPECT_GT(off.intents, 0u) << "without the hold the bootstrap carries the others' intents";
  EXPECT_EQ(on.intents, 0u) << "with it, not one";
  EXPECT_EQ(off.held, 0u);
  EXPECT_EQ(on.held, 1u);
  EXPECT_EQ(on.timeouts, 0u) << "its filter released it, not the timeout";
  // The filter, not the timeout, released it (timeouts above), and the joiner came to hold every
  // pose. How long that took is printed, not bounded: the join's trip to the relay, a step and the
  // bootstrap's trip back, ~0.1 s where this was written and 1.8 to 3.3 s on a loaded Debug gRPC
  // CI runner -- past the 2 s hold, which had not timed out.
  EXPECT_TRUE(on.knows_all);
  EXPECT_GE(on.outer_error, 0.0);
  EXPECT_LE(on.outer_error, 1e-4) << "the outer peer, from the push";
  EXPECT_EQ(on.outer_live, 0u);
}

namespace {

struct late_join_run {
  std::uint64_t pose_writes = 0;    // relay pose writes between the early and late joiners
  std::size_t journal = 0;          // the relay journal's size when the late ones came
  std::vector<std::uint64_t> early; // bytes in to a joiner before the writes, per transport
  std::vector<std::uint64_t> late;  // ... and to one after them, on the same transport
  std::uint64_t backfilled = 0;     // journal mutations the relay replayed to the late ones
};

// Connect a client that never joins the room and wait until its tree holds the room as the relay
// does (both poses, with their ticks, and the scene); false if it never does.
bool join_and_converge(test_relay &r, const std::vector<test_client *> &cs, test_client &j) {
  if (!j.connect(r))
    return false;
  const auto converged = [&] {
    for (const char *q : {"A", "B"}) {
      cvc::physics::body_state rs, js;
      std::uint64_t rt = 0, jt = 0;
      if (!r.tree_pose(q, rs, rt) || !j.tree_pose(q, js, jt) || jt != rt ||
          xy_distance(rs.position, js.position) > 1e-4)
        return false;
    }
    return ra::scene_revision(j.app, kRoom) == ra::scene_revision(r.app, kRoom);
  };
  const bool ok = run_room(r, cs, 4.0, {}, converged);
  run_room(r, cs, 0.1); // anything still in flight for the join lands
  return ok;
}

// One joiner per transport (their framing differs) connects to a room where A and B stand; A and
// B then make at least 2000 pose writes on the relay; then another joiner per transport connects.
// TRIM: the relay as arisrv runs it (journal trimmed, no backfill on connect); otherwise it keeps
// its journal and backfills each joiner with it. The joiners stay connected (a departure would add
// a retraction to the room), and none joins it.
late_join_run run_late_join(bool trim) {
  late_join_run out;
  relay_options o;
  o.trim = trim;
  o.ws_broadcast = false; // only the joins' own bytes
  o.hold = false;         // the joiners never join, so a hold would only wait out its 2 s
  test_relay r(o);
  const client_list own = make_clients({"A", "B"});
  const std::vector<test_client *> cs = ptrs(own);
  r.place_on_join = {{"A", {-3.0, 2.0}}, {"B", {3.0, 2.0}}};
  bring_up(r, cs);
  if (::testing::Test::HasFatalFailure())
    return out;
  const std::vector<link_kind> kinds = link_kinds();
  client_list joiners;
  for (std::size_t k = 0; k < kinds.size(); ++k) {
    joiners.push_back(std::make_unique<test_client>("E" + std::to_string(k), kinds[k]));
    EXPECT_TRUE(join_and_converge(r, cs, *joiners.back())) << "an early joiner never converged";
    out.early.push_back(joiners.back()->bytes_in());
  }

  // Both teleport a centimetre along a diagonal each step, stepping as fast as the relay can: two
  // changed pose leaves each per step.
  const std::uint64_t w0 = r.pose_writes();
  for (int k = 0; r.pose_writes() - w0 < 2000 && k < 20000; ++k) {
    const double d = 0.01 * (k % 100);
    r.place("A", -3.0 + d, 2.0 + d);
    r.place("B", 3.0 - d, 2.0 + d);
    r.step();
    if (k % 10 == 0)
      for (test_client *c : cs)
        c->frame();
  }
  run_room(r, cs, 0.3);
  out.pose_writes = r.pose_writes() - w0;
  out.journal = r.shard->journal().size();

  const std::uint64_t backfilled = r.backfilled();
  for (std::size_t k = 0; k < kinds.size(); ++k) {
    joiners.push_back(std::make_unique<test_client>("L" + std::to_string(k), kinds[k]));
    EXPECT_TRUE(join_and_converge(r, cs, *joiners.back())) << "a late joiner never converged";
    out.late.push_back(joiners.back()->bytes_in());
  }
  out.backfilled = r.backfilled() - backfilled;
  return out;
}

} // namespace

// After 2000 pose writes the trimmed relay journal is about empty, and a late joiner bootstraps
// from the admit snapshot alone: nothing is backfilled, and it costs what a joiner that came
// before those writes cost -- the room as it is now, not its history, which a relay that keeps its
// journal replays to every joiner.
TEST(RoomRelayLoopbackTest, LateJoinerBootstrapsFromSnapshotOnly) {
  const late_join_run keep = run_late_join(false);
  ASSERT_FALSE(HasFatalFailure());
  const late_join_run trim = run_late_join(true);
  ASSERT_FALSE(HasFatalFailure());
  const std::vector<link_kind> kinds = link_kinds();
  ASSERT_EQ(keep.early.size(), kinds.size());
  ASSERT_EQ(keep.late.size(), kinds.size());
  ASSERT_EQ(trim.early.size(), kinds.size());
  ASSERT_EQ(trim.late.size(), kinds.size());
  EXPECT_GE(trim.pose_writes, 2000u);
  EXPECT_GE(keep.pose_writes, 2000u);
  EXPECT_LE(trim.journal, 64u) << "the journal is erased as it is published";
  EXPECT_GE(keep.journal, 2000u);
  EXPECT_EQ(trim.backfilled, 0u) << "the joiners bootstrap from the admit snapshot alone";
  EXPECT_GE(keep.backfilled, 2000u * kinds.size());
  for (std::size_t k = 0; k < kinds.size(); ++k) {
    const char *name = kinds[k] == link_kind::ws ? "ws" : "grpc";
    std::printf("[ room ] %s join bytes before / after %llu pose writes: kept journal %llu / %llu; "
                "trimmed %llu / %llu\n",
                name, static_cast<unsigned long long>(trim.pose_writes),
                static_cast<unsigned long long>(keep.early[k]),
                static_cast<unsigned long long>(keep.late[k]),
                static_cast<unsigned long long>(trim.early[k]),
                static_cast<unsigned long long>(trim.late[k]));
    SCOPED_TRACE(name);
    EXPECT_GT(trim.early[k], 0u);
    // The same room either time (the poses moved, so their digits differ a little).
    EXPECT_LE(trim.late[k], trim.early[k] + trim.early[k] / 4 + 8 * 1024)
        << "join bytes do not grow with the room's history";
    // At least ~50 bytes for each of the 2000 replayed writes.
    EXPECT_GE(keep.late[k], keep.early[k] + 100u * 1000u);
  }
}

namespace {

// STEPS room steps and the client frames between them on one timeline that never skips: a step
// every kRoomStepS and a frame of each client every kFrameS, each run when due, or at once when
// the runner is late. Every run therefore has arigl's 30 frames to arisrv's 20 steps. run_room
// skips ahead instead, so a starved loop runs fewer client frames per step, which shrinks the
// echo's share of the egress. SCRIPT runs before each step.
void run_paced(test_relay &r, const std::vector<test_client *> &cs, int steps,
               const std::function<void(double)> &script) {
  const double t0 = now_s();
  int step = 0;
  long frame = 0;
  while (step < steps) {
    const double step_due = t0 + step * kRoomStepS;
    const double frame_due = t0 + static_cast<double>(frame) * kFrameS;
    const bool stepping = step_due <= frame_due;
    while (now_s() < (stepping ? step_due : frame_due))
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (stepping) {
      script(now_s() - t0);
      r.step();
      ++step;
    } else {
      for (test_client *c : cs)
        c->frame();
      ++frame;
    }
  }
}

// N=3 walking, each client echoing its avatar markers into ui.docs every frame as arigl does.
// S7: the relay as arisrv runs it and clients rooted at the room tree; otherwise the relay and
// clients from before it (no area of interest, no gate, a kept journal; whole-tree clients).
// Returns the relay's egress per second of room time: the bytes for a fixed number of steps and
// frames, however long a loaded runner takes over them. The ~1 Hz WebSocket re-broadcast is left
// out: it is the same room tree either way and would only dilute the comparison.
double run_control_plane(bool s7) {
  relay_options o;
  o.use_aoi = s7;
  o.gate = s7;
  o.trim = s7;
  o.bind_writes = s7; // the relay before S7 had no bound writes either (they too refuse the echo)
  o.ws_broadcast = false;
  test_relay r(o);
  const client_list own = make_clients({"A", "B", "C"}, s7 ? "conf" : "");
  const std::vector<test_client *> cs = ptrs(own);
  r.place_on_join = {{"A", {-4.0, 3.0}}, {"B", {4.0, 3.0}}, {"C", {0.0, -5.0}}};
  bring_up(r, cs);
  if (::testing::Test::HasFatalFailure())
    return 0.0;
  std::map<std::string, patrol> walks;
  for (test_client *c : cs) {
    walks[c->pid].anchor = r.position(c->pid).x;
    c->echo = true;
  }
  const auto script = [&](double) {
    for (auto &kv : walks)
      kv.second.drive(r, kv.first);
  };
  run_room(r, cs, 0.5, script);
  constexpr int kSteps = 30; // 1.5 s of room time
  const std::uint64_t b0 = r.egress_bytes();
  run_paced(r, cs, kSteps, script);
  // The relay forwards on its transport threads: let the window's last writes go out.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  return static_cast<double>(r.egress_bytes() - b0) / (kSteps * kRoomStepS);
}

} // namespace

// The echo is what the relay fanned for every walking avatar before S7: a client rooted at the
// room tree never sends it, and the relay's gate would withhold it from an older one. With it gone
// (and other participants' intents filtered), relay egress while N=3 walk is at most a third.
TEST(RoomRelayLoopbackTest, ControlPlaneEgressDrops) {
  const double off = run_control_plane(false);
  ASSERT_FALSE(HasFatalFailure());
  const double on = run_control_plane(true);
  ASSERT_FALSE(HasFatalFailure());
  std::printf(
      "[ room ] relay egress per room second, N=3 walking: %.1f kB before S7, %.1f with it\n",
      off / 1000.0, on / 1000.0);
  EXPECT_GT(on, 0.0);
  EXPECT_LE(on, off / 3.0);
}

// Hall preset with no downgrade dwell: every 100 ms C and D swap between beside B and 38 m away,
// so pairs flip between inner, middle and outer at 10 Hz while all four stream camera and mic and
// the WebSocket re-broadcast runs. Nothing crashes or deadlocks, and once everyone stands together
// again every receiver gets every publisher's media and every pose as the relay holds it.
TEST(RoomRelayLoopbackTest, ChurnSmoke) {
  relay_options o;
  o.aoi = ra::hall_aoi_params();
  o.aoi.downgrade_dwell_s = 0.0;
  o.large_scene = true;
  test_relay r(o);
  const client_list own = make_clients({"A", "B", "C", "D"});
  const std::vector<test_client *> cs = ptrs(own);
  r.place_on_join = {
      {"A", {-2.0, 10.0}}, {"B", {2.0, 10.0}}, {"C", {6.0, 10.0}}, {"D", {40.0, 12.0}}};
  ASSERT_NO_FATAL_FAILURE(bring_up(r, cs));
  ASSERT_TRUE(r.aoi);
  const std::uint64_t gen0 = r.aoi->snapshot()->generation();
  for (test_client *c : cs)
    c->set_media(true);
  // A swap every two room steps (100 ms of room time), counted in steps so that a loaded runner,
  // which runs fewer of them a second, still swaps every other one. Three seconds, and on until
  // the tiers have flipped ten times.
  long step = 0;
  const double t0 = now_s();
  run_room(
      r, cs, 10.0,
      [&](double) {
        const bool flip = (step++ / 2) % 2 == 1;
        r.place("C", flip ? 40.0 : 6.0, 10.0);
        r.place("D", flip ? 6.0 : 40.0, 12.0);
      },
      [&] { return now_s() - t0 >= 3.0 && r.aoi->snapshot()->generation() - gen0 >= 10; });
  EXPECT_GE(r.aoi->snapshot()->generation() - gen0, 10u) << "the tiers flipped";

  // Together again, all within 10 m of each other: inner.
  r.place("C", 5.0, 10.0);
  r.place("D", 0.0, 13.0);
  run_room(r, cs, 0.3);
  std::map<std::pair<std::string, std::string>, std::uint64_t> before;
  std::map<std::string, std::uint64_t> sent0;
  for (test_client *rc : cs) {
    sent0[rc->pid] = rc->cam_sent;
    for (test_client *q : cs)
      if (q != rc)
        before[{rc->pid, q->pid}] = rc->tap.media(q->pid, "camera");
  }
  // A second, and on until every publisher has sent more than ten camera frames (a client sends
  // one a loop pass at most, and a loaded Debug gRPC runner's loop managed eight a second); then
  // on until every receiver has every frame sent since.
  const double t1 = now_s();
  const auto sent_since = [&](test_client *q) { return q->cam_sent - sent0[q->pid]; };
  run_room(r, cs, 8.0, {}, [&] {
    if (now_s() - t1 < 1.0)
      return false;
    for (test_client *q : cs)
      if (sent_since(q) <= 10)
        return false;
    return true;
  });
  for (test_client *c : cs)
    c->set_media(false);
  const auto got_since = [&](test_client *rc, test_client *q) {
    return rc->tap.media(q->pid, "camera") - before[{rc->pid, q->pid}];
  };
  run_room(r, cs, 8.0, {}, [&] {
    for (test_client *rc : cs)
      for (test_client *q : cs)
        if (q != rc && got_since(rc, q) < sent_since(q))
          return false;
    return true;
  });
  for (test_client *rc : cs)
    for (test_client *q : cs)
      if (q != rc) {
        EXPECT_EQ(r.tier(rc->pid, q->pid), ra::aoi_tier::inner) << rc->pid << " " << q->pid;
        EXPECT_GT(sent_since(q), 10u) << q->pid;
        EXPECT_GE(got_since(rc, q), sent_since(q))
            << rc->pid << " gets " << q->pid << "'s camera at full rate again: every frame";
      }
  const auto converged = [&] {
    for (test_client *rc : cs)
      for (test_client *q : cs) {
        cvc::physics::body_state rs, ps;
        std::uint64_t rt = 0, pt = 0;
        if (!r.tree_pose(q->pid, rs, rt) || !rc->tree_pose(q->pid, ps, pt) ||
            xy_distance(rs.position, ps.position) > 1e-4)
          return false;
      }
    return true;
  };
  EXPECT_TRUE(run_room(r, cs, 3.0, {}, converged)) << "every pose converges after the churn";
}
