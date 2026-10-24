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
#include <cvc/ariadne/conference/room_scene.h>      // seed_default_scene, seed_large_demo_scene
#include <cvc/ariadne/holon.h>
#include <cvc/ariadne/holon_sync.h> // server_pump, client_pump, read_pose, write_intent
#include <cvc/core/app.h>
#include <cvc/media/media_frame.h>
#include <cvc/media/media_router.h>
#include <cvc/physics/body.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
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
};

// The relay, as arisrv builds it, stepped by the test.
class test_relay {
public:
  explicit test_relay(relay_options o) : opt(std::move(o)) {
    shard = std::make_unique<cvc::state_cluster_shard>(app, "conf", kRelayId);
    shard->add_interest("conf");
    shard->set_enforce_interest(true);
    shard->attach();
    shard->install_as_default();
    shard->adapter().set_on_local_mutation([this](const cvc::state_mutation &m) { note_local(m); });
    // A closed connection is queued here (transport threads) and retracted on the next step.
    const auto on_departed = [this](const std::string &node_id) {
      std::lock_guard<std::mutex> lk(_mu);
      _departed.push_back(node_id);
    };
#if defined(CVC_ENABLE_GRPC)
    grpc.set_on_connection_closed(on_departed);
    grpc.set_relay_requires_interest(opt.gate);
    grpc.set_backfill_on_connect(!opt.trim);
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

  relay_options opt;
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
      _ws->register_shard(shard.get());
      _ws->set_on_media(on_media);
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

// Connect and join every client, then run the room until it settles.
void bring_up(test_relay &r, const std::vector<test_client *> &cs) {
  for (test_client *c : cs) {
    ASSERT_TRUE(c->connect(r)) << c->pid << " could not reach the relay";
    c->join();
  }
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
  // Everything the window sent lands (the room keeps running, as the relay would).
  run_room(r, cs, 3.0, {}, tally);
  tally();
  out.gated = r.media_gated();
  if (r.aoi)
    out.aoi = r.aoi->take_stats();
  return out;
}

} // namespace

// The 20 m room's every pair is inner under the conference preset, so live pose, full video and
// full audio reach every receiver exactly as without the area of interest: each receiver gets
// every other subject's pose mutation live and (to within what loopback media may drop on a loaded
// runner, at the 2% the preset promises) every frame, while the gate passes every frame whole and
// the driver never pushes.
TEST(RoomRelayIntegrationTest, DefaultRoomUnchangedUnderConferencePreset) {
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
    EXPECT_GE(off.media, 0.98);
    EXPECT_GE(on.media, 0.98);
    EXPECT_NEAR(on.media, off.media, 0.02);
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
TEST(RoomRelayIntegrationTest, OuterPeerGetsPushesNotLiveFrames) {
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
  constexpr double kSeconds = 2.0;
  run_room(r, cs, kSeconds, [&](double) {
    cvc::physics::body_state rs, bs;
    std::uint64_t rt = 0, bt = 0;
    if (r.tree_pose("C", rs, rt) && B.tree_pose("C", bs, bt)) {
      max_error = (std::max)(max_error, xy_distance(rs.position, bs.position));
      seen.insert(std::llround(bs.position.y * 1000.0));
    }
    walk.drive(r, "C");
  });
  A.set_media(false);
  C.set_media(false);
  r.walk("C", 0.0, 0.0);
  run_room(r, cs, 0.3); // what is in flight lands

  EXPECT_EQ(B.tap.live_pose("C"), live_b) << "an outer peer's live pose frames are withheld";
  EXPECT_EQ(A.tap.live_pose("C"), live_a);
  // About 5 a second while C walks (one every 24 ticks). A loaded runner runs fewer room steps,
  // so the bounds are wide: 1.5 to 12 a second.
  const std::uint64_t pushes = B.tap.pushed_ticks("C") - pushed_b;
  EXPECT_GE(pushes, 3u);
  EXPECT_LE(pushes, 24u);
  EXPECT_GE(seen.size(), 3u) << "the pushes keep B's view of C moving";
  // At 2 m/s and a push every 0.2 s B's view trails by about 0.4 m.
  EXPECT_LE(max_error, 1.5);
  for (test_client *rc : {&A, &B}) {
    EXPECT_EQ(rc->tap.media("C", "camera"), 0u) << rc->pid;
    EXPECT_EQ(rc->tap.media("C", "mic"), 0u) << rc->pid;
  }
  EXPECT_GT(C.cam_sent, 20u);
  EXPECT_GT(B.tap.media("A", "camera"), 0u) << "the inner peer's media still flows";
  EXPECT_GT(B.tap.media("A", "mic"), 0u);
}

// C stands still, outer for B (hall preset), so B holds C's pose only from pushes, while B walks
// to it. On the step the pair turns inner the relay pushes C's whole pose stamped with its CURRENT
// tick, ahead of that step's live writes: within two steps B's tree has C exactly as the relay
// does, with a tick at least that fresh, and B's replica never re-aligns its clock (a stale tick
// would make it seek, and rubber-band its own avatar).
TEST(RoomRelayIntegrationTest, StillOuterPeerReentryNoRealignNoFreeze) {
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
  run_room(r, cs, 0.5); // the replicas' clocks settle on the relay's first samples
  const std::uint64_t realigns = B.h.clock_realigns();

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
  EXPECT_EQ(B.h.clock_realigns(), realigns) << "a pushed tick never re-aligns the replica";
  EXPECT_TRUE(B.h.pose_known("C"));
  cvc::physics::body_state rs;
  std::uint64_t rt = 0;
  ASSERT_TRUE(r.tree_pose("C", rs, rt));
  EXPECT_LE(xy_distance(B.h.state("C").position, rs.position), 1e-4) << "C is not frozen elsewhere";
}

// Hall preset, C 13 m from B: middle. B gets C's camera as a thumbnail -- the first frame at
// once, then about 2 a second -- and every mic chunk; A, 17 m from C and so outer, gets neither.
TEST(RoomRelayIntegrationTest, MiddleGetsThumbnailsAndAudio) {
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

  constexpr double kSeconds = 2.0;
  C.set_media(true);
  run_room(r, cs, kSeconds);
  C.set_media(false);
  // Every chunk lands (the room keeps running meanwhile).
  EXPECT_TRUE(run_room(r, cs, 3.0, {}, [&] { return B.tap.media("C", "mic") >= C.mic_sent; }));

  EXPECT_GT(C.cam_sent, 30u);
  EXPECT_EQ(B.tap.media("C", "mic"), C.mic_sent) << "a middle peer's audio is never thinned";
  // 2 +/- 1 fps over the window, plus the first frame that goes at once. The gate's timer runs on
  // wall time, so the bounds are those of the rate, not of a frame count.
  const std::uint64_t thumbs = B.tap.media("C", "camera");
  EXPECT_GE(thumbs, 2u);
  EXPECT_LE(thumbs, 7u);
  EXPECT_EQ(A.tap.media("C", "camera"), 0u);
  EXPECT_EQ(A.tap.media("C", "mic"), 0u);
  ASSERT_TRUE(r.aoi);
  EXPECT_GE(r.aoi->take_stats().media_thumb, 2u);
}

// C joins 30 m from A and B (hall preset): the pairs start outer, and the relay pushes C's pose to
// each of them on the very step it first sees C -- not at the 5 s refresh -- so they show C at its
// spot within a step plus a round trip, never at a placeholder: until that push C has no body in
// their holons and no live pose frame ever arrives.
TEST(RoomRelayIntegrationTest, NewOuterPairPushedImmediately) {
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
TEST(RoomRelayIntegrationTest, NoIntentOrUiDocsLeak) {
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
  const std::uint64_t withheld = r.withheld();
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
  EXPECT_GT(r.withheld(), withheld) << "the relay gate withheld D's echo";
  EXPECT_EQ(B.shard->total_remote_filtered_out(), filtered_b) << "no ui.docs frame reached B";
  EXPECT_EQ(A.shard->total_remote_filtered_out(), filtered_a) << "no ui.docs frame reached A";
  EXPECT_EQ(B.app.root().findDescendant("ui.docs"), nullptr);
  EXPECT_EQ(r.app.root().findDescendant("ui.docs"), nullptr) << "ingest keeps only the room";
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
TEST(RoomRelayIntegrationTest, ReconnectUnderSamePidGetsZeroedIntent) {
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

  test_client again("A", kind);
  ASSERT_TRUE(again.connect(r));
  cvc::ariadne::avatar_intent got;
  ASSERT_TRUE(run_room(r, cs, 3.0, {}, [&] {
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
TEST(RoomRelayIntegrationTest, LateJoinerBootstrapsFromSnapshotOnly) {
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

// N=3 walking, each client echoing its avatar markers into ui.docs every frame as arigl does.
// S7: the relay as arisrv runs it and clients rooted at the room tree; otherwise the relay and
// clients from before it (no area of interest, no gate, a kept journal; whole-tree clients).
// Returns the relay's egress in bytes per second. The ~1 Hz WebSocket re-broadcast is left out:
// it is the same room tree either way and would only dilute the comparison.
double run_control_plane(bool s7) {
  relay_options o;
  o.use_aoi = s7;
  o.gate = s7;
  o.trim = s7;
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
  const double t0 = now_s();
  const std::uint64_t b0 = r.egress_bytes();
  run_room(r, cs, 1.5, script);
  return static_cast<double>(r.egress_bytes() - b0) / (now_s() - t0);
}

} // namespace

// The echo is what the relay fanned for every walking avatar before S7: a client rooted at the
// room tree never sends it, and the relay's gate would withhold it from an older one. With it gone
// (and other participants' intents filtered), relay egress while N=3 walk is at most a third.
TEST(RoomRelayIntegrationTest, ControlPlaneEgressDrops) {
  const double off = run_control_plane(false);
  ASSERT_FALSE(HasFatalFailure());
  const double on = run_control_plane(true);
  ASSERT_FALSE(HasFatalFailure());
  std::printf("[ room ] relay egress, N=3 walking: %.1f kB/s before S7, %.1f kB/s with it\n",
              off / 1000.0, on / 1000.0);
  EXPECT_GT(on, 0.0);
  EXPECT_LE(on, off / 3.0);
}

// Hall preset with no downgrade dwell: every 100 ms C and D swap between beside B and 38 m away,
// so pairs flip between inner, middle and outer at 10 Hz while all four stream camera and mic and
// the WebSocket re-broadcast runs. Nothing crashes or deadlocks, and once everyone stands together
// again every receiver gets every publisher's media and every pose as the relay holds it.
TEST(RoomRelayIntegrationTest, ChurnSmoke) {
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
  run_room(r, cs, 3.0, [&](double t) {
    const bool flip = static_cast<long long>(t / 0.1) % 2 == 1;
    r.place("C", flip ? 40.0 : 6.0, 10.0);
    r.place("D", flip ? 6.0 : 40.0, 12.0);
  });
  EXPECT_GE(r.aoi->snapshot()->generation() - gen0, 10u) << "the tiers flipped";

  // Together again, all within 10 m of each other: inner.
  r.place("C", 5.0, 10.0);
  r.place("D", 0.0, 13.0);
  run_room(r, cs, 0.3);
  std::map<std::pair<std::string, std::string>, std::uint64_t> before;
  for (test_client *rc : cs)
    for (test_client *q : cs)
      if (q != rc)
        before[{rc->pid, q->pid}] = rc->tap.media(q->pid, "camera");
  run_room(r, cs, 1.0);
  for (test_client *c : cs)
    c->set_media(false);
  for (test_client *rc : cs)
    for (test_client *q : cs)
      if (q != rc) {
        EXPECT_EQ(r.tier(rc->pid, q->pid), ra::aoi_tier::inner) << rc->pid << " " << q->pid;
        const std::uint64_t was = before[{rc->pid, q->pid}];
        EXPECT_GT(rc->tap.media(q->pid, "camera"), was + 10)
            << rc->pid << " gets " << q->pid << "'s camera at full rate again";
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
