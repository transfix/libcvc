/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// arisrv_aoi_bench — a headless load bench for the relay's area of interest. One process runs a
// relay assembled as arisrv assembles it (the same switches, defaults and room step) and N
// synthetic clients assembled as arigl assembles them (a shard rooted at the room tree, a client
// holon, a camera and mic at their real rates, the avatar-marker echo into ui.docs, intents
// proposed as they walk, a net-stats leaf each second), all on loopback. After a warmup it
// measures, per receiver, what arrived by kind; per relay link, the bit rates out; the relay's
// journal and room-step time; a late joiner's bootstrap; and the probes that must hold whatever
// the traffic (clock re-alignments, avatars drawn at the origin before their pose, how far an
// outer peer's pose trails the relay's, how long a pair that turns inner waits for a fresh pose,
// how much reconcile corrected the clients' own avatars).
//
//   arisrv_aoi_bench --clients 16 --layout clusters --scene large --aoi hall --seconds 20
//   arisrv_aoi_bench --clients 4 --layout spawn --s7 off --json before.json
//
// --s7 off is the relay and clients from before S7: no area of interest, no relay gate, a kept
// journal with backfill on connect, no media stall drop, and clients that observe their whole
// tree (so the echo goes up). Byte counts of state frames are approximate (path + value + a fixed
// frame overhead); the link totals are exact.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/avatar_sim.h> // avatar_intent
#include <cvc/ariadne/conference/room_aoi.h>
#include <cvc/ariadne/conference/room_aoi_relay.h>
#include <cvc/ariadne/conference/room_intrinsics.h> // room_join, room_leave, room_roster
#include <cvc/ariadne/conference/room_net.h>        // the per-participant and relay net leaves
#include <cvc/ariadne/conference/room_scene.h>
#include <cvc/ariadne/holon.h>
#include <cvc/ariadne/holon_sync.h>
#include <cvc/core/app.h>
#include <cvc/media/media_frame.h>
#include <cvc/media/media_router.h>
#include <cvc/physics/body.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_link_stats.h>
#include <cvc/state/state_transport.h>
#if defined(CVC_ENABLE_GRPC)
#include <cvc/state/state_transport_grpc.h>
#endif
#if defined(CVC_ENABLE_WS)
#include <cvc/state/state_transport_ws.h>
#endif
#include <functional>
#include <initializer_list>
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
constexpr std::size_t kMicBytes = 1024 * 4; // 1024 f32 samples
constexpr std::size_t kFrameOverhead = 48;  // an estimate of a state frame's non-path bytes

double now_s() {
  static const auto epoch = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - epoch).count();
}

double xy_distance(const cvc::physics::vec3 &a, const cvc::physics::vec3 &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

enum class link_kind { ws, grpc };

const char *kind_name(link_kind k) { return k == link_kind::ws ? "ws" : "grpc"; }

bool kind_built(link_kind k) {
#if defined(CVC_ENABLE_WS)
  if (k == link_kind::ws)
    return true;
#endif
#if defined(CVC_ENABLE_GRPC)
  if (k == link_kind::grpc)
    return true;
#endif
  (void)k;
  return false;
}

const std::string &participants_prefix() {
  static const std::string p = ra::room_participant_path(kRoom, "");
  return p;
}

// The participant (SUBJECT) and the family under it ("pose", "intent", ...) of a room leaf;
// false for a path outside the room's participants.
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

double percentile(std::vector<double> v, double q) {
  if (v.empty())
    return 0.0;
  std::sort(v.begin(), v.end());
  const std::size_t i = static_cast<std::size_t>(q * static_cast<double>(v.size() - 1) + 0.5);
  return v[(std::min)(i, v.size() - 1)];
}

struct counter {
  std::uint64_t frames = 0;
  std::uint64_t bytes = 0;
  void add(std::uint64_t b) {
    ++frames;
    bytes += b;
  }
  counter operator-(const counter &o) const { return {frames - o.frames, bytes - o.bytes}; }
};

// What one client's shard applied and its transport delivered, counted on the reader thread. A
// live mutation carries its origin and journal sequence; a snapshot entry (admit, push, the
// WebSocket re-broadcast) carries neither.
struct receive_counts {
  counter live_pose, live_intent, live_other; // live mutations: pose leaves, intents, the rest
  counter snapshot;                           // snapshot entries
  std::uint64_t pushed_ticks = 0; // snapshot entries for a pose's server_tick (one per push)
  counter camera, mic;            // media frames, payload bytes
};

class client_tap {
public:
  void on_applied(const cvc::state_mutation &m) {
    if (m.op != cvc::state_mutation_op::set_value)
      return;
    const bool is_live = !m.origin_node_id.empty();
    const std::uint64_t bytes = m.path.size() + m.string_value.size() + kFrameOverhead;
    std::string subject, family;
    const bool participant = split_participant_path(m.path, subject, family);
    const bool tick = participant && family == "pose" && ends_with(m.path, ".pose.server_tick");
    std::lock_guard<std::mutex> lk(_mu);
    if (tick) {
      std::uint64_t &hi = _max_tick[subject];
      hi = (std::max)(
          hi, static_cast<std::uint64_t>(std::strtoull(m.string_value.c_str(), nullptr, 10)));
    }
    if (!is_live) {
      _c.snapshot.add(bytes);
      if (tick)
        ++_c.pushed_ticks;
    } else if (participant && family == "pose") {
      _c.live_pose.add(bytes);
    } else if (participant && family == "intent") {
      _c.live_intent.add(bytes);
    } else {
      _c.live_other.add(bytes);
    }
  }
  void on_media(const cvc::media_frame &f) {
    std::lock_guard<std::mutex> lk(_mu);
    (f.stream_id == "camera" ? _c.camera : _c.mic).add(f.payload.size());
  }
  receive_counts counts() const {
    std::lock_guard<std::mutex> lk(_mu);
    return _c;
  }
  std::uint64_t max_tick(const std::string &subject) const {
    std::lock_guard<std::mutex> lk(_mu);
    const auto it = _max_tick.find(subject);
    return it == _max_tick.end() ? 0 : it->second;
  }

private:
  mutable std::mutex _mu;
  receive_counts _c;
  std::map<std::string, std::uint64_t> _max_tick;
};

receive_counts operator-(const receive_counts &a, const receive_counts &b) {
  receive_counts d;
  d.live_pose = a.live_pose - b.live_pose;
  d.live_intent = a.live_intent - b.live_intent;
  d.live_other = a.live_other - b.live_other;
  d.snapshot = a.snapshot - b.snapshot;
  d.pushed_ticks = a.pushed_ticks - b.pushed_ticks;
  d.camera = a.camera - b.camera;
  d.mic = a.mic - b.mic;
  return d;
}

struct bench_config {
  int clients = 4;
  std::string layout = "spawn"; // spawn|corners|clusters|ring|walk
  std::string scene = "default";
  std::string aoi; // off|conference|hall; empty: arisrv's default for the scene
  int video_max = 0;
  double seconds = 20.0;
  double warmup = 5.0;
  std::size_t video_bytes = 4096;
  std::string transport; // ws|grpc|mixed; empty: mixed when both are built
  bool s7 = true;
  bool media = true;
  bool move = true;
  long media_stall_ms = 40;
  std::string json;
};

// The relay, as arisrv builds and steps it.
class bench_relay {
public:
  explicit bench_relay(const bench_config &cfg) {
    gate = cfg.s7;
    trim = cfg.s7;
    shard = std::make_unique<cvc::state_cluster_shard>(app, "conf", kRelayId);
    shard->add_interest("conf");
    shard->set_enforce_interest(true);
    shard->attach();
    shard->install_as_default();
    const auto on_departed = [this](const std::string &node_id) {
      std::lock_guard<std::mutex> lk(_mu);
      _departed.push_back(node_id);
    };
#if defined(CVC_ENABLE_GRPC)
    grpc.set_on_connection_closed(on_departed);
    grpc.set_relay_requires_interest(gate);
    grpc.set_backfill_on_connect(!trim);
    grpc.set_media_stall_drop(std::chrono::milliseconds(cfg.s7 ? cfg.media_stall_ms : 0));
    grpc.start("127.0.0.1:0", kRelayId, "conf");
    grpc.set_relay(true);
    grpc.register_shard(shard.get());
    planes.push_back(&grpc);
    router.add_endpoint(&grpc);
#endif
#if defined(CVC_ENABLE_WS)
    ws.set_on_connection_closed(on_departed);
    ws.set_relay_requires_interest(gate);
    ws.set_backfill_on_connect(!trim);
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
    if (cfg.scene == "large")
      ra::seed_large_demo_scene(app, kRoom);
    else
      ra::seed_default_scene(app, kRoom);
    if (cfg.s7 && cfg.aoi != "off") {
      params = cfg.aoi == "hall" ? ra::hall_aoi_params() : ra::conference_aoi_params();
      params.video_full_max = cfg.video_max;
      aoi = std::make_unique<ra::room_aoi_relay>(app, *shard, kRoom, params);
      for (cvc::state_transport *tp : planes)
        aoi->add_transport(tp);
#if defined(CVC_ENABLE_GRPC)
      aoi->add_media_endpoint(&grpc);
#endif
#if defined(CVC_ENABLE_WS)
      aoi->add_media_endpoint(&ws);
#endif
      ra::write_aoi_params(app, kRoom, params);
    }
  }

  ~bench_relay() {
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
    shard->detach();
  }

  bench_relay(const bench_relay &) = delete;
  bench_relay &operator=(const bench_relay &) = delete;

  // One room step of arisrv's loop: departures, server_pump, relay_room_step; and, on their own
  // ~1 s cadences, the WebSocket re-broadcast and the relay's net leaves. A participant listed in
  // place_on_join is moved there on the step that adds it, before the area of interest sees it.
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
    // Joins land on transport threads, so a pump can add a newcomer the pass before it never saw:
    // place newcomers and re-pump until a pass finds none.
    for (int pass = 0; pass < 8; ++pass) {
      bool moved = false;
      for (const std::string &pid : server.participants()) {
        if (!_known.insert(pid).second)
          continue;
        const auto it = place_on_join.find(pid);
        if (it != place_on_join.end()) {
          cvc::physics::body_state s;
          s.position = {it->second.first, it->second.second, 0.0};
          s.active = true;
          server.override_state(pid, s);
          moved = true;
        }
      }
      if (!moved)
        break;
      cvc::ariadne::server_pump(server, app, kRoom);
    }
    const double t0 = now_s();
    ra::relay_room_step(*shard, planes, aoi.get(), server, now_s(), trim);
    step_ms.push_back((now_s() - t0) * 1000.0);
    journal_max = (std::max)(journal_max, shard->journal().size());
    const double t = now_s();
#if defined(CVC_ENABLE_WS)
    if (t >= _next_broadcast) {
      ws.broadcast_snapshot();
      _next_broadcast = t + 1.0;
    }
#endif
    if (t >= _next_net) {
      _next_net = t + 1.0;
      write_net(t);
    }
  }

  std::vector<cvc::state_link_report> links() const {
    std::vector<cvc::state_link_report> out;
    for (const cvc::state_transport *tp : planes)
      for (cvc::state_link_report &r : tp->link_reports())
        out.push_back(std::move(r));
    return out;
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
  std::uint64_t stall_dropped() const {
#if defined(CVC_ENABLE_GRPC)
    return grpc.total_media_stall_dropped();
#else
    return 0;
#endif
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
  ra::aoi_tier tier(const std::string &receiver, const std::string &subject) const {
    const auto snap = aoi ? aoi->snapshot() : nullptr;
    return snap ? snap->tier(receiver, subject) : ra::aoi_tier::inner;
  }

  bool gate = false, trim = false;
  ra::aoi_params params;
  cvc::app app;
  std::unique_ptr<cvc::state_cluster_shard> shard;
#if defined(CVC_ENABLE_GRPC)
  cvc::state_transport_grpc grpc;
#endif
#if defined(CVC_ENABLE_WS)
  cvc::state_transport_ws ws;
#endif
  cvc::media_router router;
  holon server{holon_role::server, cvc::ariadne::room_holon_config()};
  std::unique_ptr<ra::room_aoi_relay> aoi;
  std::vector<cvc::state_transport *> planes;
  std::map<std::string, std::pair<double, double>> place_on_join;
  std::vector<double> step_ms;
  std::size_t journal_max = 0;

private:
  // arisrv's ~1 s link stats: each participant's link as the relay sees it, and the totals.
  void write_net(double t) {
    const std::vector<cvc::state_link_report> ls = links();
    const auto rates = _meter.update(ls, t);
    std::set<std::string> present;
    for (const auto &p : ra::room_roster(app, kRoom))
      present.insert(p.pid);
    ra::server_net total;
    for (const cvc::state_link_report &r : ls) {
      const auto it = rates.find(cvc::state_link_rate_meter::key_of(r));
      const double in_kbps = it == rates.end() ? 0.0 : it->second.in_bps * 8.0 / 1000.0;
      const double out_kbps = it == rates.end() ? 0.0 : it->second.out_bps * 8.0 / 1000.0;
      total.in_kbps += in_kbps;
      total.out_kbps += out_kbps;
      ++total.links;
      if (present.count(r.remote_node_id))
        ra::write_relay_net(app, kRoom, r.remote_node_id, {r.stats.rtt_ms, in_kbps, out_kbps});
    }
    ra::write_server_net(app, kRoom, total);
  }

  std::mutex _mu;
  std::vector<std::string> _departed;
  std::set<std::string> _known;
  cvc::state_link_rate_meter _meter;
  double _next_broadcast = 0.0, _next_net = 1.0;
};

// A synthetic participant, as arigl runs one.
class bench_client {
public:
  bench_client(std::string id, link_kind k, const bench_config &cfg)
      : pid(std::move(id)), kind(k), _cfg(cfg) {
    // Rooted at the room tree; the client from before S7 observed its whole tree.
    shard = std::make_unique<cvc::state_cluster_shard>(app, "conf", pid, cfg.s7 ? "conf" : "");
    shard->add_interest("conf");
    shard->set_enforce_interest(true);
    shard->attach();
    shard->adapter().set_on_remote_applied(
        [this](const cvc::state_mutation &m) { tap.on_applied(m); });
  }
  ~bench_client() {
    disconnect();
    shard->adapter().set_on_remote_applied({});
    shard->detach();
  }
  bench_client(const bench_client &) = delete;
  bench_client &operator=(const bench_client &) = delete;

  bool connect(bench_relay &r) {
    const auto on_media = [this](const cvc::media_frame &f) { tap.on_media(f); };
#if defined(CVC_ENABLE_GRPC)
    if (kind == link_kind::grpc) {
      _grpc = std::make_unique<cvc::state_transport_grpc>();
      _grpc->start("127.0.0.1:0", pid, "conf");
      _grpc->register_shard(shard.get());
      _grpc->set_on_media(on_media);
      tp = _grpc.get();
      return _grpc->connect_to_peer(r.grpc.listen_address(), std::chrono::milliseconds(8000));
    }
#endif
#if defined(CVC_ENABLE_WS)
    if (kind == link_kind::ws) {
      _ws = std::make_unique<cvc::state_transport_ws>();
      _ws->register_shard(shard.get());
      _ws->set_on_media(on_media);
      tp = _ws.get();
      return _ws->connect_to_peer("ws://" + r.ws.listen_address(), std::chrono::milliseconds(8000));
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

  // Where this participant stands and how it moves: along one axis AMPLITUDE either side of
  // (ANCHOR_X, ANCHOR_Y), or (ROAM) between pseudo-random waypoints inside +-BOUND.
  void set_motion(double ax, double ay, bool along_y, bool roam, double bound, unsigned seed) {
    _ax = ax;
    _ay = ay;
    _along_y = along_y;
    _roam = roam;
    _bound = bound;
    _rng = seed * 2654435761u + 1u;
    _target_x = ax;
    _target_y = ay;
  }

  void join() {
    ra::room_join(app, kRoom, pid, pid);
    if (tp)
      tp->pump_all();
  }

  // One client frame: walk (proposing an intent when it changes), the holon against the room,
  // probes, the avatar-marker echo, the ~1 s net leaf, then the frame's writes sent.
  void frame(bool moving, bool measuring) {
    if (moving)
      drive();
    cvc::ariadne::client_pump(h, app, kRoom, pid);
    if (h.has_participant(pid)) {
      const cvc::physics::vec3 c = h.take_correction(pid);
      if (measuring)
        corrections_m += std::sqrt(cvc::physics::dot(c, c));
    }
    if (measuring)
      for (const std::string &q : h.participants())
        if (q != pid && h.has_body(q) && !h.pose_known(q))
          ++ghost_frames; // a body held at a placeholder before the first pose
    write_echo();
    const double t = now_s();
    if (tp && t >= _next_net) {
      _next_net = t + 1.0;
      for (const cvc::state_link_report &r : tp->link_reports())
        ra::write_participant_net(app, kRoom, pid, {r.stats.rtt_ms, -1.0, -1.0});
    }
    if (tp)
      tp->pump_all();
  }

  void set_media(bool on) {
    _media = on;
    _next_cam = _next_mic = now_s();
  }
  void send_due_media(double now) {
    if (!_media || !tp)
      return;
    if (now >= _next_cam) {
      send("camera", "mjpeg", _cfg.video_bytes, cam_sent++);
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
  cvc::state_link_stats::counters link() const {
    cvc::state_link_stats::counters c;
    if (tp)
      for (const cvc::state_link_report &r : tp->link_reports()) {
        c.bytes_in += r.stats.bytes_in;
        c.bytes_out += r.stats.bytes_out;
        c.frames_in += r.stats.frames_in;
        c.frames_out += r.stats.frames_out;
      }
    return c;
  }

  const std::string pid;
  const link_kind kind;
  cvc::app app;
  client_tap tap;
  std::unique_ptr<cvc::state_cluster_shard> shard;
  holon h{holon_role::client, cvc::ariadne::room_holon_config()};
  cvc::state_transport *tp = nullptr;
  std::uint64_t cam_sent = 0, mic_sent = 0;
  double corrections_m = 0.0;     // reconcile jumps of our own avatar while measuring, summed
  std::uint64_t ghost_frames = 0; // peer-frames with a body but no pose while measuring

private:
  double rand01() {
    _rng = _rng * 1664525u + 1013904223u;
    return static_cast<double>(_rng >> 8) / static_cast<double>(1u << 24);
  }

  void drive() {
    if (!h.has_participant(pid) || !h.pose_known(pid))
      return;
    const cvc::physics::vec3 p = h.state(pid).position;
    constexpr double kPatrolSpeed = 1.5, kRoamSpeed = 2.0, kAmplitude = 1.0;
    cvc::ariadne::avatar_intent in;
    if (_roam) {
      // A new waypoint once this one is reached, or after 10 s (one behind the reference box, or
      // another avatar, may never be reached).
      const double t = now_s();
      if (std::hypot(_target_x - p.x, _target_y - p.y) < 0.5 || t >= _target_until) {
        _target_x = (2.0 * rand01() - 1.0) * _bound;
        _target_y = (2.0 * rand01() - 1.0) * _bound;
        _target_until = t + 10.0;
      }
      const double dx = _target_x - p.x, dy = _target_y - p.y, d = std::hypot(dx, dy);
      if (d > 1e-6)
        in.move = {kRoamSpeed * dx / d, kRoamSpeed * dy / d, 0.0};
    } else {
      if (((_along_y ? p.y - _ay : p.x - _ax) * _dir) >= kAmplitude)
        _dir = -_dir;
      in.move = {_along_y ? 0.0 : _dir * kPatrolSpeed, _along_y ? _dir * kPatrolSpeed : 0.0, 0.0};
    }
    in.yaw = (in.move.x != 0.0 || in.move.y != 0.0) ? std::atan2(in.move.y, in.move.x) : _last.yaw;
    // Each proposal is a replicated write: only when the walk changes (a waypoint, a turn).
    const bool same = _proposed && std::fabs(in.move.x - _last.move.x) < 0.05 &&
                      std::fabs(in.move.y - _last.move.y) < 0.05;
    if (same)
      return;
    cvc::ariadne::propose_intent(h, app, kRoom, pid, in);
    _last = in;
    _proposed = true;
  }

  void send(const char *stream, const char *codec, std::size_t bytes, std::uint64_t seq) {
    cvc::media_frame f;
    f.origin_node_id = pid;
    f.stream_id = stream;
    f.codec = codec;
    if (std::strcmp(stream, "camera") == 0) {
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

  // room_view's per-frame body and visor marker matrices for each shown avatar, into the
  // document's UI tree. A shard rooted at the room tree never observes them.
  void write_echo() {
    char buf[32];
    for (const std::string &q : h.participants()) {
      if (!h.pose_known(q))
        continue;
      const cvc::physics::pose p = h.display_pose(q);
      const cvc::physics::quat &r = p.rotation;
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
      m[14] = p.position.z + 0.85;
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

  const bench_config &_cfg;
#if defined(CVC_ENABLE_GRPC)
  std::unique_ptr<cvc::state_transport_grpc> _grpc;
#endif
#if defined(CVC_ENABLE_WS)
  std::unique_ptr<cvc::state_transport_ws> _ws;
#endif
  bool _media = false;
  double _next_cam = 0.0, _next_mic = 0.0, _next_net = 0.0;
  double _ax = 0.0, _ay = 0.0, _bound = 8.0, _dir = 1.0, _target_x = 0.0, _target_y = 0.0;
  double _target_until = 0.0;
  bool _along_y = false, _roam = false, _proposed = false;
  unsigned _rng = 1;
  cvc::ariadne::avatar_intent _last;
};

// The probes the main loop samples while measuring.
struct probes {
  std::vector<double> outer_error_m; // a non-inner subject's pose in a receiver's tree vs relay's
  std::vector<double> entry_ms;      // a pair turning inner until the receiver has a fresh pose
  std::size_t entries_unresolved = 0;
  struct pending {
    bench_client *receiver = nullptr;
    std::string subject;
    double t = 0.0;
    std::uint64_t tick = 0;
  };
  std::vector<pending> waiting;
  std::map<std::pair<std::string, std::string>, ra::aoi_tier> tiers;
  std::uint64_t generation = 0;
};

// After a room step: pairs whose tier just turned inner start waiting for a fresh pose, and every
// non-inner pair's pose error is sampled.
void probe_step(bench_relay &r, const std::vector<bench_client *> &cs, probes &pr) {
  const auto snap = r.aoi ? r.aoi->snapshot() : nullptr;
  if (!snap)
    return;
  if (snap->generation() != pr.generation) {
    pr.generation = snap->generation();
    for (bench_client *rc : cs)
      for (bench_client *q : cs) {
        if (q == rc)
          continue;
        const ra::aoi_tier t = snap->tier(rc->pid, q->pid);
        const auto key = std::make_pair(rc->pid, q->pid);
        const auto it = pr.tiers.find(key);
        if (it != pr.tiers.end() && it->second != ra::aoi_tier::inner && t == ra::aoi_tier::inner)
          pr.waiting.push_back({rc, q->pid, now_s(), r.server.tick()});
        pr.tiers[key] = t;
      }
  }
  for (bench_client *rc : cs)
    for (bench_client *q : cs) {
      if (q == rc || snap->tier(rc->pid, q->pid) == ra::aoi_tier::inner)
        continue;
      cvc::physics::body_state rs, ps;
      std::uint64_t rt = 0, pt = 0;
      if (cvc::ariadne::read_pose(r.app, kRoom, q->pid, rs, rt) && rc->tree_pose(q->pid, ps, pt))
        pr.outer_error_m.push_back(xy_distance(rs.position, ps.position));
    }
}

// After a round of frames: a waiting pair resolves once its receiver holds a pose of the subject
// at least as fresh as the tick of the step that turned it inner.
void probe_frame(probes &pr) {
  const double t = now_s();
  for (auto it = pr.waiting.begin(); it != pr.waiting.end();) {
    if (it->receiver->tap.max_tick(it->subject) >= it->tick) {
      pr.entry_ms.push_back((t - it->t) * 1000.0);
      it = pr.waiting.erase(it);
    } else {
      ++it;
    }
  }
}

// Run the room in real time for SECONDS (a room step every 50 ms, a client frame every 1/30 s,
// media when due), with the probes when MEASURING; returns early, true, once UNTIL holds.
bool run_room(bench_relay &r, const std::vector<bench_client *> &cs, double seconds, bool moving,
              bool measuring, probes *pr, const std::function<bool()> &until = {}) {
  const double t0 = now_s();
  double next_step = t0, next_frame = t0;
  for (;;) {
    const double t = now_s();
    if (t >= next_step) {
      r.step();
      if (measuring && pr)
        probe_step(r, cs, *pr);
      next_step += kRoomStepS;
      if (next_step < t)
        next_step = t + kRoomStepS;
    }
    if (t >= next_frame) {
      for (bench_client *c : cs)
        c->frame(moving, measuring);
      if (measuring && pr)
        probe_frame(*pr);
      next_frame += kFrameS;
      if (next_frame < t)
        next_frame = t + kFrameS;
      if (until && until())
        return true;
    }
    for (bench_client *c : cs)
      c->send_due_media(t);
    if (t - t0 >= seconds)
      return until ? until() : true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

bool settled(bench_relay &r, const std::vector<bench_client *> &cs) {
  const auto snap = r.aoi ? r.aoi->snapshot() : nullptr;
  for (bench_client *c : cs) {
    if (!r.server.has_participant(c->pid))
      return false;
    if (r.aoi) {
      if (!snap)
        return false;
      const auto &ps = snap->participants();
      if (std::find(ps.begin(), ps.end(), c->pid) == ps.end())
        return false;
    }
    for (bench_client *q : cs)
      if (!c->h.has_participant(q->pid) || !c->h.pose_known(q->pid))
        return false;
  }
  return true;
}

// Each participant's anchor for LAYOUT; ROAM for the walk layout.
struct placement {
  bool placed = false; // false: where the server spawns it (its 4 m ring)
  double x = 0.0, y = 0.0;
  bool along_y = false;
};

std::vector<placement> layout_of(const bench_config &cfg) {
  std::vector<placement> out(static_cast<std::size_t>(cfg.clients));
  const double pi = 3.14159265358979323846;
  for (int i = 0; i < cfg.clients; ++i) {
    placement &p = out[static_cast<std::size_t>(i)];
    p.along_y = i % 2 == 1;
    if (cfg.layout == "corners") {
      // The default room's corners (+-8, +-8), each further one a metre in toward the centre.
      const double s = (std::max)(1.0, 8.0 - static_cast<double>(i / 4));
      p.placed = true;
      p.x = (i % 4 == 0 || i % 4 == 3) ? s : -s;
      p.y = (i % 4 < 2) ? s : -s;
    } else if (cfg.layout == "clusters") {
      // Clusters of four within 2 m, centres 30 m apart on a grid around the origin.
      const int k = i / 4, cols = 2;
      const double cx = (k % cols) * 30.0 - 15.0, cy = (k / cols) * 30.0 - 15.0;
      p.placed = true;
      p.x = cx + ((i % 2) ? 0.7 : -0.7);
      p.y = cy + ((i % 4 < 2) ? 0.7 : -0.7);
    } else if (cfg.layout == "ring") {
      const double a = 2.0 * pi * i / cfg.clients;
      p.placed = true;
      p.x = 7.5 * std::cos(a);
      p.y = 7.5 * std::sin(a);
    }
  }
  return out;
}

bool check_choice(const char *flag, const std::string &v, std::initializer_list<const char *> ok) {
  std::string all;
  for (const char *o : ok) {
    if (v == o)
      return true;
    if (!all.empty())
      all += '|';
    all += o;
  }
  std::fprintf(stderr, "arisrv_aoi_bench: %s expects %s, got '%s'\n", flag, all.c_str(), v.c_str());
  return false;
}

void usage() {
  std::fprintf(stderr,
               "usage: arisrv_aoi_bench [--clients N] [--layout spawn|corners|clusters|ring|walk]\n"
               "         [--scene default|large] [--aoi off|conference|hall] [--video-max K]\n"
               "         [--seconds S] [--warmup W] [--video-bytes B] [--transport ws|grpc|mixed]\n"
               "         [--s7 on|off] [--media on|off] [--move on|off] [--media-stall-ms N]\n"
               "         [--json out.json]\n");
}

std::string json_str(const std::string &s) {
  std::string out = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\')
      out += '\\';
    out += c;
  }
  return out + "\"";
}

std::string json_counter(const counter &c, double secs) {
  char buf[160];
  std::snprintf(buf, sizeof buf,
                "{\"frames\": %llu, \"bytes\": %llu, \"fps\": %.2f, \"bps\": %.0f}",
                static_cast<unsigned long long>(c.frames), static_cast<unsigned long long>(c.bytes),
                c.frames / secs, c.bytes * 8.0 / secs);
  return buf;
}

} // namespace

int main(int argc, char **argv) {
  bench_config cfg;
  std::string s7 = "on", media = "on", move = "on";
  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    const auto next = [&]() -> const char * {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "arisrv_aoi_bench: %s needs a value\n", a);
        std::exit(2);
      }
      return argv[++i];
    };
    if (!std::strcmp(a, "--clients"))
      cfg.clients = std::atoi(next());
    else if (!std::strcmp(a, "--layout"))
      cfg.layout = next();
    else if (!std::strcmp(a, "--scene"))
      cfg.scene = next();
    else if (!std::strcmp(a, "--aoi"))
      cfg.aoi = next();
    else if (!std::strcmp(a, "--video-max"))
      cfg.video_max = (std::max)(0, std::atoi(next()));
    else if (!std::strcmp(a, "--seconds"))
      cfg.seconds = std::atof(next());
    else if (!std::strcmp(a, "--warmup"))
      cfg.warmup = std::atof(next());
    else if (!std::strcmp(a, "--video-bytes"))
      cfg.video_bytes = static_cast<std::size_t>((std::max)(1L, std::atol(next())));
    else if (!std::strcmp(a, "--transport"))
      cfg.transport = next();
    else if (!std::strcmp(a, "--s7"))
      s7 = next();
    else if (!std::strcmp(a, "--media"))
      media = next();
    else if (!std::strcmp(a, "--move"))
      move = next();
    else if (!std::strcmp(a, "--media-stall-ms"))
      cfg.media_stall_ms = (std::max)(0L, std::atol(next()));
    else if (!std::strcmp(a, "--json"))
      cfg.json = next();
    else if (!std::strcmp(a, "--help") || !std::strcmp(a, "-h")) {
      usage();
      return 0;
    } else {
      std::fprintf(stderr, "arisrv_aoi_bench: unknown argument '%s'\n", a);
      usage();
      return 2;
    }
  }
  if (cfg.aoi.empty())
    cfg.aoi = cfg.scene == "large" ? "hall" : "conference"; // arisrv's default
  if (cfg.transport.empty())
    cfg.transport = kind_built(link_kind::ws) && kind_built(link_kind::grpc) ? "mixed"
                    : kind_built(link_kind::ws)                              ? "ws"
                                                                             : "grpc";
  if (!check_choice("--layout", cfg.layout, {"spawn", "corners", "clusters", "ring", "walk"}) ||
      !check_choice("--scene", cfg.scene, {"default", "large"}) ||
      !check_choice("--aoi", cfg.aoi, {"off", "conference", "hall"}) ||
      !check_choice("--transport", cfg.transport, {"ws", "grpc", "mixed"}) ||
      !check_choice("--s7", s7, {"on", "off"}) || !check_choice("--media", media, {"on", "off"}) ||
      !check_choice("--move", move, {"on", "off"}))
    return 2;
  cfg.s7 = s7 == "on";
  cfg.media = media == "on";
  cfg.move = move == "on";
  if (!cfg.s7)
    cfg.aoi = "off";
  if (cfg.clients < 1 || !(cfg.seconds > 0.0) || cfg.warmup < 0.0) {
    std::fprintf(stderr, "arisrv_aoi_bench: --clients >= 1, --seconds > 0, --warmup >= 0\n");
    return 2;
  }
  std::vector<link_kind> kinds;
  if (cfg.transport == "ws" || cfg.transport == "mixed")
    kinds.push_back(link_kind::ws);
  if (cfg.transport == "grpc" || cfg.transport == "mixed")
    kinds.push_back(link_kind::grpc);
  for (link_kind k : kinds)
    if (!kind_built(k)) {
      std::fprintf(stderr, "arisrv_aoi_bench: this build has no %s transport\n", kind_name(k));
      return 2;
    }

  std::printf("[bench] %d client(s) | layout %s | scene %s | S7 %s | aoi %s (full-rate cameras %d)"
              " | %s | media %s (%zu B video) | move %s | warmup %.1f s, measure %.1f s\n",
              cfg.clients, cfg.layout.c_str(), cfg.scene.c_str(), cfg.s7 ? "on" : "off",
              cfg.aoi.c_str(), cfg.video_max, cfg.transport.c_str(), cfg.media ? "on" : "off",
              cfg.video_bytes, cfg.move ? "on" : "off", cfg.warmup, cfg.seconds);
  std::fflush(stdout);

  bench_relay relay(cfg);
  std::vector<std::unique_ptr<bench_client>> own;
  std::vector<bench_client *> cs;
  const std::vector<placement> places = layout_of(cfg);
  const double bound = cfg.scene == "large" ? 40.0 : 8.0;
  for (int i = 0; i < cfg.clients; ++i) {
    char id[16];
    std::snprintf(id, sizeof id, "c%02d", i);
    own.push_back(
        std::make_unique<bench_client>(id, kinds[static_cast<std::size_t>(i) % kinds.size()], cfg));
    cs.push_back(own.back().get());
    const placement &p = places[static_cast<std::size_t>(i)];
    if (p.placed)
      relay.place_on_join[id] = {p.x, p.y};
  }
  for (bench_client *c : cs) {
    if (!c->connect(relay)) {
      std::fprintf(stderr, "arisrv_aoi_bench: %s could not reach the relay\n", c->pid.c_str());
      return 1;
    }
    c->join();
  }
  if (!run_room(relay, cs, 30.0, false, false, nullptr, [&] { return settled(relay, cs); })) {
    std::fprintf(stderr, "arisrv_aoi_bench: the room never settled\n");
    return 1;
  }
  // Each one's walk around where it stands (or, for the walk layout, all over the scene).
  for (std::size_t i = 0; i < cs.size(); ++i) {
    const cvc::physics::vec3 at = relay.server.state(cs[i]->pid).position;
    cs[i]->set_motion(at.x, at.y, places[i].along_y, cfg.layout == "walk", bound,
                      static_cast<unsigned>(i + 1));
  }
  for (bench_client *c : cs)
    c->set_media(cfg.media);
  run_room(relay, cs, cfg.warmup, cfg.move, false, nullptr);

  // The measured window.
  std::map<std::string, std::pair<std::uint64_t, std::uint64_t>> link0; // key -> (in, out)
  for (const cvc::state_link_report &r : relay.links())
    link0[cvc::state_link_rate_meter::key_of(r)] = {r.stats.bytes_in, r.stats.bytes_out};
  std::vector<receive_counts> rc0;
  std::vector<cvc::state_link_stats::counters> cl0;
  std::vector<std::uint64_t> realign0;
  for (bench_client *c : cs) {
    rc0.push_back(c->tap.counts());
    cl0.push_back(c->link());
    realign0.push_back(c->h.clock_realigns());
  }
  const std::uint64_t withheld0 = relay.withheld(), gated0 = relay.media_gated();
  const std::uint64_t stalled0 = relay.stall_dropped();
  if (relay.aoi)
    relay.aoi->take_stats();
  relay.step_ms.clear();
  relay.journal_max = 0;
  probes pr;
  const double t0 = now_s();
  run_room(relay, cs, cfg.seconds, cfg.move, true, &pr);
  const double secs = now_s() - t0;
  pr.entries_unresolved = pr.waiting.size();
  ra::room_aoi_relay::stats st;
  if (relay.aoi)
    st = relay.aoi->take_stats();
  const std::size_t journal_end = relay.shard->journal().size();

  // A late joiner: connects (it never joins) and bootstraps; its bytes once its tree holds every
  // participant's pose (to within half a metre of the relay's: they keep walking) and the scene.
  for (bench_client *c : cs)
    c->set_media(false);
  const std::uint64_t backfilled0 = relay.backfilled();
  bench_client late("late", kinds[cs.size() % kinds.size()], cfg);
  const double tj = now_s();
  bool joined = late.connect(relay);
  const auto converged = [&] {
    for (bench_client *q : cs) {
      cvc::physics::body_state rs, ls;
      std::uint64_t rt = 0, lt = 0;
      if (!cvc::ariadne::read_pose(relay.app, kRoom, q->pid, rs, rt) ||
          !late.tree_pose(q->pid, ls, lt) || xy_distance(rs.position, ls.position) > 0.5)
        return false;
    }
    return ra::scene_revision(late.app, kRoom) == ra::scene_revision(relay.app, kRoom);
  };
  joined = joined && run_room(relay, cs, 10.0, cfg.move, false, nullptr, converged);
  const double join_s = now_s() - tj;
  const std::uint64_t join_bytes = late.link().bytes_in;
  const std::uint64_t join_backfilled = relay.backfilled() - backfilled0;

  // The report.
  std::uint64_t realigns = 0, ghosts = 0;
  double corrections = 0.0;
  std::string jr;
  std::printf("[bench] measured %.2f s\n", secs);
  std::printf("[bench] receiver  link  in kb/s | pose live fps  | intent | other | snapshot "
              "(pushed ticks) | camera fps | mic fps | realigns\n");
  for (std::size_t i = 0; i < cs.size(); ++i) {
    bench_client &c = *cs[i];
    const receive_counts d = c.tap.counts() - rc0[i];
    const std::uint64_t in_bytes = c.link().bytes_in - cl0[i].bytes_in;
    const std::uint64_t ra_n = c.h.clock_realigns() - realign0[i];
    realigns += ra_n;
    ghosts += c.ghost_frames;
    corrections += c.corrections_m;
    std::printf(
        "[bench]   %-6s %-4s %9.1f | %7.1f %6.1fk | %6.1f | %5.1f | %6.1f (%llu) | %10.1f | "
        "%7.1f | %llu\n",
        c.pid.c_str(), kind_name(c.kind), in_bytes * 8.0 / 1000.0 / secs, d.live_pose.frames / secs,
        d.live_pose.bytes / 1000.0 / secs, d.live_intent.frames / secs, d.live_other.frames / secs,
        d.snapshot.frames / secs, static_cast<unsigned long long>(d.pushed_ticks),
        d.camera.frames / secs, d.mic.frames / secs, static_cast<unsigned long long>(ra_n));
    char head[256];
    std::snprintf(head, sizeof head,
                  "    {\"pid\": %s, \"transport\": \"%s\", \"in_bytes\": %llu, \"in_bps\": %.0f, "
                  "\"realigns\": %llu, \"corrections_m\": %.4f, \"ghost_frames\": %llu,\n",
                  json_str(c.pid).c_str(), kind_name(c.kind),
                  static_cast<unsigned long long>(in_bytes), in_bytes * 8.0 / secs,
                  static_cast<unsigned long long>(ra_n), c.corrections_m,
                  static_cast<unsigned long long>(c.ghost_frames));
    jr += head;
    jr += "     \"live_pose\": " + json_counter(d.live_pose, secs) +
          ", \"live_intent\": " + json_counter(d.live_intent, secs) +
          ",\n     \"live_other\": " + json_counter(d.live_other, secs) +
          ", \"snapshot\": " + json_counter(d.snapshot, secs) +
          ",\n     \"pushed_ticks\": " + std::to_string(d.pushed_ticks) +
          ", \"camera\": " + json_counter(d.camera, secs) +
          ", \"mic\": " + json_counter(d.mic, secs) + "}";
    jr += i + 1 < cs.size() ? ",\n" : "\n";
  }

  double out_total = 0.0, in_total = 0.0;
  std::string jl;
  std::printf("[bench] relay links (out / in kb/s):\n");
  const std::vector<cvc::state_link_report> links = relay.links();
  for (std::size_t i = 0; i < links.size(); ++i) {
    const cvc::state_link_report &r = links[i];
    const auto it = link0.find(cvc::state_link_rate_meter::key_of(r));
    const std::uint64_t in0 = it == link0.end() ? 0 : it->second.first;
    const std::uint64_t out0 = it == link0.end() ? 0 : it->second.second;
    const double out_bps = (r.stats.bytes_out - out0) * 8.0 / secs;
    const double in_bps = (r.stats.bytes_in - in0) * 8.0 / secs;
    out_total += out_bps;
    in_total += in_bps;
    std::printf("[bench]   %-4s %-8s %10.1f / %8.1f\n", r.transport.c_str(),
                r.remote_node_id.c_str(), out_bps / 1000.0, in_bps / 1000.0);
    char buf[200];
    std::snprintf(
        buf, sizeof buf,
        "    {\"transport\": \"%s\", \"peer\": %s, \"out_bps\": %.0f, \"in_bps\": %.0f}%s\n",
        r.transport.c_str(), json_str(r.remote_node_id).c_str(), out_bps, in_bps,
        i + 1 < links.size() ? "," : "");
    jl += buf;
  }
  const double step_p50 = percentile(relay.step_ms, 0.5),
               step_p99 = percentile(relay.step_ms, 0.99);
  const double step_max =
      relay.step_ms.empty() ? 0.0 : *std::max_element(relay.step_ms.begin(), relay.step_ms.end());
  const double err_max = pr.outer_error_m.empty()
                             ? 0.0
                             : *std::max_element(pr.outer_error_m.begin(), pr.outer_error_m.end());
  std::printf("[bench] relay out %.1f kb/s, in %.1f kb/s | journal %zu (max %zu) | room step ms "
              "p50 %.3f p99 %.3f max %.3f\n",
              out_total / 1000.0, in_total / 1000.0, journal_end, relay.journal_max, step_p50,
              step_p99, step_max);
  std::printf("[bench] aoi pairs inner %zu middle %zu outer %zu | pushes %llu (%llu entries) | "
              "media full %llu thumb %llu dropped %llu | state filtered %llu | relay withheld %llu "
              "| media gated %llu, stalled %llu\n",
              st.inner, st.middle, st.outer, static_cast<unsigned long long>(st.pushes),
              static_cast<unsigned long long>(st.push_entries),
              static_cast<unsigned long long>(st.media_full),
              static_cast<unsigned long long>(st.media_thumb),
              static_cast<unsigned long long>(st.media_dropped),
              static_cast<unsigned long long>(st.state_filtered),
              static_cast<unsigned long long>(relay.withheld() - withheld0),
              static_cast<unsigned long long>(relay.media_gated() - gated0),
              static_cast<unsigned long long>(relay.stall_dropped() - stalled0));
  std::printf("[bench] late join: %s in %.2f s, %llu bytes, %llu backfilled\n",
              joined ? "converged" : "did NOT converge", join_s,
              static_cast<unsigned long long>(join_bytes),
              static_cast<unsigned long long>(join_backfilled));
  std::printf("[bench] probes: realigns %llu | origin ghost frames %llu | outer-peer error m max "
              "%.3f p95 %.3f (%zu samples) | inner entry ms p50 %.1f p95 %.1f (%zu, %zu "
              "unresolved) | reconcile corrections %.3f m\n",
              static_cast<unsigned long long>(realigns), static_cast<unsigned long long>(ghosts),
              err_max, percentile(pr.outer_error_m, 0.95), pr.outer_error_m.size(),
              percentile(pr.entry_ms, 0.5), percentile(pr.entry_ms, 0.95), pr.entry_ms.size(),
              pr.entries_unresolved, corrections);
  std::fflush(stdout);

  if (!cfg.json.empty()) {
    std::FILE *f = std::fopen(cfg.json.c_str(), "w");
    if (!f) {
      std::fprintf(stderr, "arisrv_aoi_bench: cannot write %s\n", cfg.json.c_str());
      return 1;
    }
    std::fprintf(f,
                 "{\n  \"config\": {\"clients\": %d, \"layout\": %s, \"scene\": %s, \"aoi\": %s, "
                 "\"video_max\": %d, \"seconds\": %.2f, \"warmup\": %.2f, \"video_bytes\": %zu, "
                 "\"transport\": %s, \"s7\": %s, \"media\": %s, \"move\": %s, "
                 "\"media_stall_ms\": %ld},\n",
                 cfg.clients, json_str(cfg.layout).c_str(), json_str(cfg.scene).c_str(),
                 json_str(cfg.aoi).c_str(), cfg.video_max, cfg.seconds, cfg.warmup, cfg.video_bytes,
                 json_str(cfg.transport).c_str(), cfg.s7 ? "true" : "false",
                 cfg.media ? "true" : "false", cfg.move ? "true" : "false",
                 cfg.s7 ? cfg.media_stall_ms : 0L);
    std::fprintf(f, "  \"measured_seconds\": %.3f,\n  \"receivers\": [\n%s  ],\n", secs,
                 jr.c_str());
    std::fprintf(f,
                 "  \"relay\": {\"out_bps\": %.0f, \"in_bps\": %.0f, \"journal\": %zu, "
                 "\"journal_max\": %zu,\n    \"step_ms\": {\"p50\": %.4f, \"p99\": %.4f, \"max\": "
                 "%.4f},\n",
                 out_total, in_total, journal_end, relay.journal_max, step_p50, step_p99, step_max);
    std::fprintf(f,
                 "    \"aoi\": {\"inner\": %zu, \"middle\": %zu, \"outer\": %zu, \"pushes\": %llu, "
                 "\"push_entries\": %llu, \"media_full\": %llu, \"media_thumb\": %llu, "
                 "\"media_dropped\": %llu, \"state_filtered\": %llu},\n",
                 st.inner, st.middle, st.outer, static_cast<unsigned long long>(st.pushes),
                 static_cast<unsigned long long>(st.push_entries),
                 static_cast<unsigned long long>(st.media_full),
                 static_cast<unsigned long long>(st.media_thumb),
                 static_cast<unsigned long long>(st.media_dropped),
                 static_cast<unsigned long long>(st.state_filtered));
    std::fprintf(f,
                 "    \"relay_withheld\": %llu, \"media_gated\": %llu, "
                 "\"media_stall_dropped\": %llu,\n    \"links\": [\n%s    ]},\n",
                 static_cast<unsigned long long>(relay.withheld() - withheld0),
                 static_cast<unsigned long long>(relay.media_gated() - gated0),
                 static_cast<unsigned long long>(relay.stall_dropped() - stalled0), jl.c_str());
    std::fprintf(f,
                 "  \"late_join\": {\"converged\": %s, \"seconds\": %.3f, \"bytes\": %llu, "
                 "\"backfilled\": %llu},\n",
                 joined ? "true" : "false", join_s, static_cast<unsigned long long>(join_bytes),
                 static_cast<unsigned long long>(join_backfilled));
    std::fprintf(f,
                 "  \"probes\": {\"realigns\": %llu, \"origin_ghost_frames\": %llu, "
                 "\"outer_error_m\": {\"max\": %.4f, \"p95\": %.4f, \"samples\": %zu}, "
                 "\"inner_entry_ms\": {\"p50\": %.2f, \"p95\": %.2f, \"count\": %zu, "
                 "\"unresolved\": %zu}, \"correction_sum_m\": %.4f}\n}\n",
                 static_cast<unsigned long long>(realigns), static_cast<unsigned long long>(ghosts),
                 err_max, percentile(pr.outer_error_m, 0.95), pr.outer_error_m.size(),
                 percentile(pr.entry_ms, 0.5), percentile(pr.entry_ms, 0.95), pr.entry_ms.size(),
                 pr.entries_unresolved, corrections);
    std::fclose(f);
    std::printf("[bench] wrote %s\n", cfg.json.c_str());
  }
  // Clients first (the relay retracts each as its connection closes), then the relay.
  late.disconnect();
  own.clear();
  return 0;
}
