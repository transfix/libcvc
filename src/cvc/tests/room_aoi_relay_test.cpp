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

// The relay's area-of-interest driver against transports that only record what they are asked:
// the participants' filters land in every registry and leave with the participant, a pose push
// carries the holon's current tick as its last entry and goes out before the step's live writes,
// the media gate thumbnails a middle camera, teardown takes everything back while a reader's copy
// of the gate stays safe to call, and with no driver the room step is the plain drain-and-publish
// fan-out the relay always did.

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cvc/ariadne/conference/room_aoi.h>
#include <cvc/ariadne/conference/room_aoi_relay.h>
#include <cvc/ariadne/conference/room_intrinsics.h> // room_join, room_leave, room_participant_path
#include <cvc/ariadne/holon.h>
#include <cvc/ariadne/holon_sync.h> // server_pump
#include <cvc/core/app.h>
#include <cvc/media/media_endpoint.h>
#include <cvc/media/media_frame.h>
#include <cvc/physics/body.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_transport.h>
#include <gtest/gtest.h>
#include <initializer_list>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace ra = cvc::ariadne::conference;
using cvc::ariadne::holon;
using cvc::ariadne::holon_role;
using snapshot_entry = cvc::state_transport::snapshot_entry;

namespace {

// A transport and media endpoint that does nothing but record: each publish and push goes into a
// log shared with its siblings (so a test sees the order of calls across planes), the mutations
// and pushes are kept, and the last media gate it was given is held like a real endpoint holds
// it. push_snapshot_to reports one connection written for a pid in `connected`, as the transport
// holding that participant's connection would.
class recording_transport final : public cvc::state_transport, public cvc::media_endpoint {
public:
  struct push {
    std::string node_id;
    std::vector<snapshot_entry> entries;
  };

  recording_transport(std::string name, std::vector<std::string> &log)
      : _name(std::move(name)), _log(log) {}

  const std::string &name() const { return _name; }

  std::vector<cvc::state_mutation> published;
  std::vector<push> pushes;
  std::set<std::string> connected;
  media_gate_fn gate;
  int gate_sets = 0;

  void register_shard(cvc::state_cluster_shard *s) override { _shards.push_back(s); }
  void unregister_shard(cvc::state_cluster_shard *s) override {
    for (auto it = _shards.begin(); it != _shards.end();)
      it = *it == s ? _shards.erase(it) : it + 1;
  }
  publish_stats publish(const cvc::state_mutation &m) override {
    _log.push_back(_name + " publish " + m.path);
    published.push_back(m);
    return {};
  }
  std::size_t pump_shard(cvc::state_cluster_shard &s) override {
    const std::vector<cvc::state_mutation> pending = s.drain_local();
    for (const cvc::state_mutation &m : pending)
      publish(m);
    return pending.size();
  }
  std::size_t pump_all() override {
    std::size_t n = 0;
    for (cvc::state_cluster_shard *s : _shards)
      n += pump_shard(*s);
    return n;
  }
  void flush() override {}
  std::size_t push_snapshot_to(const std::string &node_id,
                               const std::vector<snapshot_entry> &entries) override {
    _log.push_back(_name + " push " + node_id);
    pushes.push_back({node_id, entries});
    return connected.count(node_id) ? 1 : 0;
  }

  void fan_media(const cvc::media_frame &, const void *) override {}
  void set_media_sink(media_sink_fn) override {}
  const char *media_protocol() const noexcept override { return "recording"; }
  void set_media_gate(media_gate_fn g) override {
    gate = std::move(g);
    ++gate_sets;
  }

  // The last push to NODE_ID, or null.
  const push *push_to(const std::string &node_id) const {
    for (auto it = pushes.rbegin(); it != pushes.rend(); ++it)
      if (it->node_id == node_id)
        return &*it;
    return nullptr;
  }

private:
  std::string _name;
  std::vector<std::string> &_log;
  std::vector<cvc::state_cluster_shard *> _shards;
};

struct spot {
  const char *pid;
  double x;
  double y;
};

// The relay's side of a room as arisrv runs it: an app, a whole-tree shard that takes only the
// room tree, and a server holon (kinematic, so a participant left alone stands exactly still).
struct relay_room {
  cvc::app app;
  cvc::state_cluster_shard shard{app, "conf", "server"};
  holon server{holon_role::server};
  const std::string room = "r";

  relay_room() {
    shard.add_interest("conf");
    shard.set_enforce_interest(true);
    shard.attach();
  }
  ~relay_room() { shard.detach(); }

  // Advance the room TICKS holon ticks (at most 60, the holon's catch-up per advance) and write
  // the poses, as one arisrv room step does.
  void pump(int ticks = 1) {
    cvc::ariadne::server_pump(server, app, room, ticks * server.clock().fixed_dt());
  }

  // Join each participant and stand it at its spot: the first pump adds the newcomers on the
  // spawn ring, the second writes the poses the overrides put them at.
  void place(std::initializer_list<spot> where) {
    for (const spot &s : where)
      ra::room_join(app, room, s.pid);
    pump();
    for (const spot &s : where)
      move(s);
    pump();
  }

  void move(const spot &s) {
    cvc::physics::body_state b;
    b.position = {s.x, s.y, 0.0};
    ASSERT_TRUE(server.override_state(s.pid, b)) << s.pid;
  }

  // conf.rooms.r.participants.<pid>.<rest...>
  std::string leaf(const std::string &pid, std::initializer_list<const char *> rest) const {
    std::string s = ra::room_participant_path(room, pid);
    for (const char *seg : rest) {
      s += cvc::state::SEPARATOR;
      s += seg;
    }
    return s;
  }
};

cvc::media_frame frame(const char *publisher, const char *stream_id, const char *codec) {
  cvc::media_frame f;
  f.origin_node_id = publisher;
  f.stream_id = stream_id;
  f.codec = codec;
  return f;
}

std::size_t first_with(const std::vector<std::string> &log, const std::string &word) {
  for (std::size_t i = 0; i < log.size(); ++i)
    if (log[i].find(word) != std::string::npos)
      return i;
  return log.size();
}

std::size_t last_with(const std::vector<std::string> &log, const std::string &word) {
  for (std::size_t i = log.size(); i-- > 0;)
    if (log[i].find(word) != std::string::npos)
      return i;
  return log.size();
}

} // namespace

// ── registries ──────────────────────────────────────────────────────────────

// Hall preset: A and B 4 m apart (inner), C 30 m from both (outer).
TEST(RoomAoiRelayTest, FiltersInstalledOnEveryTransport) {
  relay_room rr;
  rr.place({{"A", -2.0, 0.0}, {"B", 2.0, 0.0}, {"C", 0.0, 30.0}});
  std::vector<std::string> log;
  // Declared before the driver: what it is given must outlive it.
  recording_transport grpc("grpc", log), ws("ws", log), late("late", log);
  ra::room_aoi_relay aoi(rr.app, rr.shard, rr.room, ra::hall_aoi_params());
  aoi.add_transport(&grpc);
  aoi.add_transport(&ws);
  aoi.add_media_endpoint(&grpc);
  aoi.add_media_endpoint(&ws);
  ra::relay_room_step(rr.shard, {&grpc, &ws}, &aoi, rr.server, 0.0, false);

  for (recording_transport *tp : {&grpc, &ws}) {
    SCOPED_TRACE(tp->name());
    cvc::state_peer_registry &reg = tp->peers();
    for (const char *pid : {"A", "B", "C"}) {
      const cvc::state_peer_registry::delivery_view v = reg.view_of(pid);
      EXPECT_TRUE(v.known) << pid;
      EXPECT_EQ(v.subscriptions, std::vector<std::string>{"conf"}) << pid;
      EXPECT_NE(v.filter, nullptr) << pid;
    }
    // The snapshot's rules, through the registry every send consults.
    EXPECT_TRUE(reg.should_deliver("A", rr.leaf("B", {"pose", "pos", "x"}))) << "inner";
    EXPECT_FALSE(reg.should_deliver("A", rr.leaf("C", {"pose", "pos", "x"}))) << "outer";
    EXPECT_FALSE(reg.should_deliver("A", rr.leaf("C", {"pose", "server_tick"})));
    EXPECT_TRUE(reg.should_deliver("A", rr.leaf("A", {"intent", "move", "x"}))) << "its own";
    EXPECT_FALSE(reg.should_deliver("A", rr.leaf("B", {"intent", "move", "x"})));
    EXPECT_TRUE(reg.should_deliver("A", rr.leaf("C", {"presence", "joined"}))) << "the roster";
    EXPECT_FALSE(reg.should_deliver("A", "ui.docs.x")) << "registered for the room tree only";
    EXPECT_TRUE(reg.should_deliver("peer-server", rr.leaf("C", {"pose", "pos", "x"})))
        << "a connection the driver never registered still gets everything";
    EXPECT_TRUE(static_cast<bool>(tp->gate));
  }

  // A transport added later gets the current filters at once.
  aoi.add_transport(&late);
  aoi.add_transport(&late); // a repeat is ignored
  EXPECT_NE(late.peers().view_of("C").filter, nullptr);
  EXPECT_EQ(late.peers().size(), 3u);

  const ra::room_aoi_relay::stats st = aoi.take_stats();
  EXPECT_EQ(st.inner, 1u);
  EXPECT_EQ(st.middle, 0u);
  EXPECT_EQ(st.outer, 2u);
}

TEST(RoomAoiRelayTest, DepartedPidRemovedFromRegistries) {
  relay_room rr;
  rr.place({{"A", -2.0, 0.0}, {"B", 2.0, 0.0}, {"C", 0.0, 3.0}});
  std::vector<std::string> log;
  recording_transport grpc("grpc", log), ws("ws", log);
  // Something else registered B on the gRPC side before the driver started.
  grpc.peers().add_peer("B", "conf", "grpc://b");
  ra::room_aoi_relay aoi(rr.app, rr.shard, rr.room, ra::conference_aoi_params());
  aoi.add_transport(&grpc);
  aoi.add_transport(&ws);
  const std::vector<cvc::state_transport *> planes{&grpc, &ws};
  ra::relay_room_step(rr.shard, planes, &aoi, rr.server, 0.0, false);
  for (recording_transport *tp : {&grpc, &ws})
    for (const char *pid : {"A", "B", "C"})
      ASSERT_NE(tp->peers().view_of(pid).filter, nullptr) << tp->name() << " " << pid;
  EXPECT_TRUE(grpc.peers().view_of("B").subscriptions.empty()) << "its registration is kept";

  ra::room_leave(rr.app, rr.room, "B");
  ra::room_leave(rr.app, rr.room, "C");
  rr.pump(); // the holon drops them
  ra::relay_room_step(rr.shard, planes, &aoi, rr.server, 0.05, false);

  for (recording_transport *tp : {&grpc, &ws}) {
    EXPECT_FALSE(tp->peers().has_peer("C")) << tp->name();
    EXPECT_NE(tp->peers().view_of("A").filter, nullptr) << tp->name();
  }
  EXPECT_FALSE(ws.peers().has_peer("B"));
  // The registration the driver did not make stays, without the driver's filter.
  const cvc::state_peer_registry::delivery_view b = grpc.peers().view_of("B");
  EXPECT_TRUE(b.known);
  EXPECT_EQ(b.filter, nullptr);

  // A rejoin is registered and filtered again.
  ra::room_join(rr.app, rr.room, "C");
  rr.pump();
  ra::relay_room_step(rr.shard, planes, &aoi, rr.server, 0.1, false);
  for (recording_transport *tp : {&grpc, &ws})
    EXPECT_NE(tp->peers().view_of("C").filter, nullptr) << tp->name();
}

// ── pose pushes ─────────────────────────────────────────────────────────────

// C stands still 30 m away, so write_pose leaves its leaves and server_tick alone until the
// heartbeat (about a second) and the tree's tick falls behind the holon's. The push C owes its new
// outer receivers carries the holon's tick instead, after the leaves.
TEST(RoomAoiRelayTest, PushCarriesFreshServerTickLast) {
  relay_room rr;
  rr.place({{"A", -2.0, 0.0}, {"B", 2.0, 0.0}, {"C", 0.0, 30.0}});
  rr.pump(59);
  rr.pump(59);
  const std::string c_pose = rr.leaf("C", {"pose"});
  const std::string c_tick = rr.leaf("C", {"pose", "server_tick"});
  cvc::state *tree_tick = rr.app.root().findDescendant(c_tick);
  ASSERT_NE(tree_tick, nullptr);
  const std::string tree_value = tree_tick->value();
  ASSERT_GE(rr.server.tick() - std::strtoull(tree_value.c_str(), nullptr, 10), 100u);

  std::vector<std::string> log;
  recording_transport grpc("grpc", log), ws("ws", log);
  grpc.connected = {"A", "B", "C"};
  ra::room_aoi_relay aoi(rr.app, rr.shard, rr.room, ra::hall_aoi_params());
  aoi.add_transport(&grpc);
  aoi.add_transport(&ws);
  ra::relay_room_step(rr.shard, {&grpc, &ws}, &aoi, rr.server, 0.0, true);
  const std::string now = std::to_string(static_cast<unsigned long long>(rr.server.tick()));

  const recording_transport::push *to_a = grpc.push_to("A");
  ASSERT_NE(to_a, nullptr) << "a new outer pair is pushed at once";
  ASSERT_EQ(to_a->entries.size(), 14u) << "C's 13 pose leaves and its tick, no intermediate nodes";
  EXPECT_EQ(to_a->entries.back().path, c_tick);
  EXPECT_EQ(to_a->entries.back().string_value, now);
  for (std::size_t i = 0; i + 1 < to_a->entries.size(); ++i) {
    const snapshot_entry &e = to_a->entries[i];
    EXPECT_EQ(e.path.rfind(c_pose + cvc::state::SEPARATOR, 0), 0u) << e.path;
    EXPECT_NE(e.path, c_tick);
    EXPECT_FALSE(e.string_value.empty()) << e.path;
    if (e.path == rr.leaf("C", {"pose", "pos", "y"}))
      EXPECT_DOUBLE_EQ(std::strtod(e.string_value.c_str(), nullptr), 30.0);
  }
  EXPECT_EQ(tree_tick->value(), tree_value) << "the relay's own tree keeps its tick";

  // C gets both of its new outer subjects in one frame, each one's tick after its own leaves.
  const recording_transport::push *to_c = grpc.push_to("C");
  ASSERT_NE(to_c, nullptr);
  ASSERT_EQ(to_c->entries.size(), 28u);
  EXPECT_EQ(to_c->entries[13].path, rr.leaf("A", {"pose", "server_tick"}));
  EXPECT_EQ(to_c->entries[13].string_value, now);
  EXPECT_EQ(to_c->entries[27].path, rr.leaf("B", {"pose", "server_tick"}));
  EXPECT_EQ(to_c->entries[27].string_value, now);
  // B is inner with A, so B is owed C alone.
  const recording_transport::push *to_b = grpc.push_to("B");
  ASSERT_NE(to_b, nullptr);
  EXPECT_EQ(to_b->entries.size(), 14u);
  // Every plane is asked; only the one holding the connection writes.
  EXPECT_NE(ws.push_to("A"), nullptr);
  EXPECT_EQ(grpc.pushes.size(), 3u);

  const ra::room_aoi_relay::stats st = aoi.take_stats();
  EXPECT_EQ(st.pushes, 3u);
  EXPECT_EQ(st.push_entries, 14u + 14u + 28u);
  EXPECT_EQ(aoi.take_stats().pushes, 0u) << "counters restart at each take";
}

TEST(RoomAoiRelayTest, PushesPrecedeDrainPublishes) {
  relay_room rr;
  rr.place({{"A", -2.0, 0.0}, {"B", 2.0, 0.0}, {"C", 0.0, 30.0}});
  std::vector<std::string> log;
  recording_transport grpc("grpc", log), ws("ws", log);
  grpc.connected = {"A", "B"};
  ws.connected = {"C"};
  ra::room_aoi_relay aoi(rr.app, rr.shard, rr.room, ra::hall_aoi_params());
  aoi.add_transport(&grpc);
  aoi.add_transport(&ws);
  const std::vector<cvc::state_transport *> planes{&grpc, &ws};

  // The first step owes the new outer pairs their pushes and drains the joins and poses.
  const std::size_t drained = ra::relay_room_step(rr.shard, planes, &aoi, rr.server, 0.0, false);
  EXPECT_GT(drained, 0u);
  ASSERT_LT(first_with(log, " push "), log.size());
  ASSERT_LT(first_with(log, " publish "), log.size());
  EXPECT_LT(last_with(log, " push "), first_with(log, " publish "));
  // Each drained mutation went to both planes, once each, in drain order.
  ASSERT_EQ(grpc.published.size(), drained);
  ASSERT_EQ(ws.published.size(), drained);
  for (std::size_t i = 0; i < drained; ++i)
    EXPECT_EQ(grpc.published[i].sequence, ws.published[i].sequence);

  // C walks while outer: past the push interval its move is pushed, still ahead of the live
  // writes of the same step.
  log.clear();
  rr.move({"C", 0.0, 31.0});
  rr.pump(40);
  ra::relay_room_step(rr.shard, planes, &aoi, rr.server, 0.4, false);
  ASSERT_NE(ws.push_to("A"), nullptr);
  ASSERT_LT(first_with(log, " push "), log.size());
  ASSERT_LT(first_with(log, " publish "), log.size());
  EXPECT_LT(last_with(log, " push "), first_with(log, " publish "));
}

// ── the room step without a driver ──────────────────────────────────────────

// Two identical rooms: one fanned the way the relay did before (drain once, publish each mutation
// to both planes), the other through relay_room_step with no driver and no trim. Same calls, same
// mutations, same journal; nothing registered, pushed or gated. One plane matches a transport's
// own pump_all.
TEST(RoomAoiRelayTest, NullAoiEqualsFanLocalWrites) {
  relay_room before, after;
  std::vector<std::string> log_before, log_after;
  recording_transport b1("grpc", log_before), b2("ws", log_before);
  recording_transport a1("grpc", log_after), a2("ws", log_after);
  for (int i = 0; i < 3; ++i) {
    for (relay_room *rr : {&before, &after}) {
      if (i == 0)
        rr->place({{"A", -2.0, 0.0}, {"B", 2.0, 0.0}});
      else
        rr->move({"A", -2.0, static_cast<double>(i)});
      rr->pump(6);
    }
    for (const cvc::state_mutation &m : before.shard.drain_local()) {
      b1.publish(m);
      b2.publish(m);
    }
    ra::relay_room_step(after.shard, {&a1, &a2}, nullptr, after.server, 0.0, false);
  }
  EXPECT_EQ(log_after, log_before);
  ASSERT_EQ(a1.published.size(), b1.published.size());
  ASSERT_GT(a1.published.size(), 0u);
  for (std::size_t i = 0; i < a1.published.size(); ++i) {
    const cvc::state_mutation &x = a1.published[i], &y = b1.published[i];
    EXPECT_EQ(x.path, y.path);
    EXPECT_EQ(x.op, y.op);
    EXPECT_EQ(x.string_value, y.string_value);
    EXPECT_EQ(x.type_name, y.type_name);
    EXPECT_EQ(x.sequence, y.sequence);
    EXPECT_EQ(x.origin_node_id, y.origin_node_id);
  }
  EXPECT_EQ(after.shard.journal().size(), before.shard.journal().size()) << "nothing trimmed";
  EXPECT_GT(after.shard.journal().size(), 0u);
  EXPECT_TRUE(a1.pushes.empty());
  EXPECT_TRUE(a2.pushes.empty());
  EXPECT_EQ(a1.peers().size(), 0u);
  EXPECT_EQ(a2.peers().size(), 0u);
  EXPECT_EQ(a1.gate_sets, 0);

  // One plane: the transport's own pump_all of the one shard.
  relay_room one, two;
  std::vector<std::string> log_pump, log_step;
  recording_transport p("grpc", log_pump), q("grpc", log_step);
  p.register_shard(&one.shard);
  for (relay_room *rr : {&one, &two})
    rr->place({{"A", 1.0, 1.0}});
  const std::size_t pumped = p.pump_all();
  EXPECT_EQ(ra::relay_room_step(two.shard, {&q}, nullptr, two.server, 0.0, false), pumped);
  EXPECT_EQ(log_step, log_pump);
  p.unregister_shard(&one.shard);
}

TEST(RoomAoiRelayTest, TrimJournalErasesOnlyWhatWasPublished) {
  relay_room rr;
  rr.place({{"A", -2.0, 0.0}});
  std::vector<std::string> log;
  recording_transport grpc("grpc", log);
  const std::size_t first = ra::relay_room_step(rr.shard, {&grpc}, nullptr, rr.server, 0.0, true);
  EXPECT_GT(first, 0u);
  EXPECT_EQ(rr.shard.journal().size(), 0u);
  rr.move({"A", -2.0, 1.0});
  rr.pump();
  const std::size_t second = ra::relay_room_step(rr.shard, {&grpc}, nullptr, rr.server, 0.05, true);
  EXPECT_GT(second, 0u) << "writes after a trim still drain";
  EXPECT_EQ(grpc.published.size(), first + second);
  EXPECT_EQ(rr.shard.journal().size(), 0u);
  EXPECT_GT(grpc.published.back().sequence, grpc.published.front().sequence);
}

// ── media gate ──────────────────────────────────────────────────────────────

// Hall preset: C 14 m from A and B (middle). Its camera reaches them as thumbnails, its mic in
// full; walking in turns the camera to full rate, and walking back out sends the first thumbnail
// at once rather than waiting out the interval of the last one.
TEST(RoomAoiRelayTest, MiddleCameraIsThumbnailedAndResumesAtOnce) {
  relay_room rr;
  rr.place({{"A", -2.0, 0.0}, {"B", 2.0, 0.0}, {"C", 0.0, 14.0}});
  std::vector<std::string> log;
  recording_transport grpc("grpc", log);
  ra::room_aoi_relay aoi(rr.app, rr.shard, rr.room, ra::hall_aoi_params());
  aoi.add_transport(&grpc);
  aoi.add_media_endpoint(&grpc);
  const std::vector<cvc::state_transport *> planes{&grpc};
  ra::relay_room_step(rr.shard, planes, &aoi, rr.server, 0.0, false);
  ASSERT_TRUE(static_cast<bool>(grpc.gate));
  const cvc::media_frame cam = frame("C", "camera", "mjpeg");
  const cvc::media_frame mic = frame("C", "mic", "f32");

  // A burst of 30 frames, far inside the 0.5 s thumbnail interval: the first goes, the rest wait.
  const auto t0 = std::chrono::steady_clock::now();
  int cams = 0, mics = 0, inner_cams = 0;
  for (int i = 0; i < 30; ++i) {
    cams += grpc.gate(cam, "A") ? 1 : 0;
    mics += grpc.gate(mic, "A") ? 1 : 0;
    inner_cams += grpc.gate(frame("B", "camera", "mjpeg"), "A") ? 1 : 0;
  }
  if (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(400))
    EXPECT_EQ(cams, 1);
  EXPECT_GE(cams, 1);
  EXPECT_EQ(mics, 30) << "a middle peer is heard in full";
  EXPECT_EQ(inner_cams, 30) << "an inner peer is seen in full";
  EXPECT_TRUE(grpc.gate(cam, "C")) << "oneself";

  // In (an upgrade is immediate): full rate.
  rr.move({"C", 0.0, 5.0});
  rr.pump();
  ra::relay_room_step(rr.shard, planes, &aoi, rr.server, 1.0, false);
  for (int i = 0; i < 5; ++i)
    EXPECT_TRUE(grpc.gate(cam, "A"));
  // Back out (a downgrade waits out the 1 s dwell): a thumbnail again, the first one at once.
  rr.move({"C", 0.0, 14.0});
  rr.pump();
  ra::relay_room_step(rr.shard, planes, &aoi, rr.server, 2.0, false);
  EXPECT_TRUE(grpc.gate(cam, "A")) << "still inner inside the dwell";
  ra::relay_room_step(rr.shard, planes, &aoi, rr.server, 3.5, false);
  ASSERT_EQ(aoi.snapshot()->tier("A", "C"), ra::aoi_tier::middle);
  EXPECT_TRUE(grpc.gate(cam, "A")) << "the first thumbnail after the change";
  EXPECT_FALSE(grpc.gate(cam, "A"));

  const ra::room_aoi_relay::stats st = aoi.take_stats();
  EXPECT_GE(st.media_thumb, 2u);
  EXPECT_GE(st.media_dropped, 1u);
  EXPECT_GE(st.media_full, 60u);
  EXPECT_EQ(st.middle, 2u);
  EXPECT_EQ(st.inner, 1u);
}

// enabled = false: a pass-through that registers nothing, pushes nothing and gates nothing.
TEST(RoomAoiRelayTest, DisabledParamsArePassThrough) {
  relay_room rr;
  rr.place({{"A", -2.0, 0.0}, {"B", 2.0, 0.0}, {"C", 0.0, 30.0}});
  std::vector<std::string> log;
  recording_transport grpc("grpc", log);
  grpc.connected = {"A", "B", "C"};
  ra::aoi_params off = ra::hall_aoi_params();
  off.enabled = false;
  ra::room_aoi_relay aoi(rr.app, rr.shard, rr.room, off);
  aoi.add_transport(&grpc);
  aoi.add_media_endpoint(&grpc);
  for (int i = 0; i < 3; ++i) {
    ra::relay_room_step(rr.shard, {&grpc}, &aoi, rr.server, 0.25 * i, true);
    rr.pump(30);
  }
  EXPECT_EQ(grpc.peers().size(), 0u);
  EXPECT_TRUE(grpc.pushes.empty());
  ASSERT_TRUE(static_cast<bool>(grpc.gate));
  EXPECT_TRUE(grpc.gate(frame("C", "camera", "mjpeg"), "A"));
  EXPECT_FALSE(grpc.published.empty());
}

// ── teardown ────────────────────────────────────────────────────────────────

TEST(RoomAoiRelayTest, TeardownClearsGatesAndFilters) {
  relay_room rr;
  rr.place({{"A", -2.0, 0.0}, {"B", 2.0, 0.0}, {"C", 0.0, 30.0}});
  std::vector<std::string> log;
  recording_transport grpc("grpc", log), ws("ws", log);
  grpc.peers().add_peer("B", "conf", "grpc://b");
  auto aoi = std::make_unique<ra::room_aoi_relay>(rr.app, rr.shard, rr.room, ra::hall_aoi_params());
  aoi->add_transport(&grpc);
  aoi->add_transport(&ws);
  aoi->add_media_endpoint(&grpc);
  aoi->add_media_endpoint(&ws);
  aoi->add_media_endpoint(&ws); // a repeat is ignored
  ra::relay_room_step(rr.shard, {&grpc, &ws}, aoi.get(), rr.server, 0.0, false);
  EXPECT_EQ(ws.gate_sets, 1);

  // A publisher's reader thread copied the gate just before the driver went away.
  const cvc::media_endpoint::media_gate_fn held = grpc.gate;
  ASSERT_TRUE(static_cast<bool>(held));
  const cvc::media_frame cam = frame("C", "camera", "mjpeg");
  EXPECT_FALSE(held(cam, "A")) << "C is outer to A";
  EXPECT_TRUE(held(frame("B", "camera", "mjpeg"), "A")) << "B is inner to A";

  aoi.reset();
  for (recording_transport *tp : {&grpc, &ws}) {
    SCOPED_TRACE(tp->name());
    EXPECT_FALSE(static_cast<bool>(tp->gate)) << "the gate is cleared";
    EXPECT_EQ(tp->gate_sets, 2);
    EXPECT_FALSE(tp->peers().has_peer("A"));
    EXPECT_FALSE(tp->peers().has_peer("C"));
  }
  EXPECT_FALSE(ws.peers().has_peer("B"));
  const cvc::state_peer_registry::delivery_view b = grpc.peers().view_of("B");
  EXPECT_TRUE(b.known) << "a registration the driver did not make stays";
  EXPECT_EQ(b.filter, nullptr);

  // The reader's copy still runs, and now delivers everything.
  EXPECT_TRUE(held(cam, "A"));
  EXPECT_TRUE(held(frame("A", "mic", "f32"), "C"));
}
