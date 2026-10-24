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

// The room's spatial area of interest: per-pair tiers with hysteresis, the snapshot's deny-only
// state rules and per-frame media decisions, the pose push schedule, the presets (the conference
// one must leave the default room exactly as it was), the client audio curve, and the replicated
// parameters.
//
// <windows.h> comes first, in its own preprocessor block so include sorting can never move it
// after the header it guards: its minwindef.h defines near and far, which is why the tiers are
// inner / middle / outer. The guards are the ones the library uses around <windows.h>.
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/ariadne/conference/room_aoi.h>
#include <cvc/ariadne/conference/room_intrinsics.h> // room_participant_path
#include <cvc/ariadne/conference/room_scene.h>      // default_scene_objects: the room to cover
#include <cvc/ariadne/holon.h>                      // holon_config: the peer span pushes must fit
#include <cvc/ariadne/holon_sync.h>                 // room_holon_config
#include <cvc/ariadne/scene.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_peer_registry.h>
#include <cvc/state/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <initializer_list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace ra = cvc::ariadne::conference;
using ra::aoi_media;
using ra::aoi_tier;

namespace {

const std::string kRoom = "r1";

ra::room_aoi::position at(const char *pid, double x, double y = 0.0) { return {pid, x, y}; }

// conf.rooms.r1.participants.<pid>[.<segs>...]
std::string ppath(const std::string &pid, std::initializer_list<const char *> segs = {}) {
  std::string s = ra::room_participant_path(kRoom, pid);
  for (const char *seg : segs) {
    s += cvc::state::SEPARATOR;
    s += seg;
  }
  return s;
}

// conf.rooms.r1[.<segs>...]
std::string rpath(std::initializer_list<const char *> segs) {
  std::string s =
      std::string("conf") + cvc::state::SEPARATOR + "rooms" + cvc::state::SEPARATOR + kRoom;
  for (const char *seg : segs) {
    s += cvc::state::SEPARATOR;
    s += seg;
  }
  return s;
}

using push_map = std::map<std::string, std::vector<std::string>>;
push_map as_map(const std::vector<ra::aoi_push> &pushes) {
  push_map m;
  for (const ra::aoi_push &p : pushes)
    m[p.receiver] = p.subjects;
  return m;
}

// The hall preset with a at the origin and, along +x: b inner (5 m), c middle (14 m) and d outer
// (40 m) from a; e (1 m) joins and then leaves, so it is departed. Among themselves b-c is 9 m
// (inner), b-d 35 m and c-d 26 m (outer).
void place_abcde(ra::room_aoi &aoi) {
  aoi.update({at("a", 0), at("b", 5), at("c", 14), at("d", 40), at("e", 1)}, 0, 0.0);
  aoi.update({at("a", 0), at("b", 5), at("c", 14), at("d", 40)}, 6, 0.05);
}

} // namespace

// ── tiers ───────────────────────────────────────────────────────────────────

TEST(RoomAoiTest, InitialTierUsesEnterRadii) {
  const ra::aoi_params h = ra::hall_aoi_params();
  EXPECT_EQ(ra::initial_tier(0.0, h), aoi_tier::inner);
  EXPECT_EQ(ra::initial_tier(10.0, h), aoi_tier::inner);
  EXPECT_EQ(ra::initial_tier(11.0, h), aoi_tier::middle) << "inside inner's exit, not its enter";
  EXPECT_EQ(ra::initial_tier(16.0, h), aoi_tier::middle);
  EXPECT_EQ(ra::initial_tier(17.0, h), aoi_tier::outer) << "inside middle's exit, not its enter";
  EXPECT_EQ(ra::initial_tier(1000.0, h), aoi_tier::outer);

  const ra::aoi_params c = ra::conference_aoi_params();
  EXPECT_EQ(ra::initial_tier(30.0, c), aoi_tier::inner);
  EXPECT_EQ(ra::initial_tier(31.0, c), aoi_tier::middle);
  EXPECT_EQ(ra::initial_tier(40.0, c), aoi_tier::middle);
  EXPECT_EQ(ra::initial_tier(41.0, c), aoi_tier::outer);
}

TEST(RoomAoiTest, StepTierHysteresisTable) {
  const ra::aoi_params h = ra::hall_aoi_params(); // 10/12 inner, 16/18 middle, 1 s dwell
  struct row {
    aoi_tier cur;
    double d_m, now_s, beyond_in;
    aoi_tier want;
    double beyond_out;
    const char *what;
  };
  const row rows[] = {
      {aoi_tier::inner, 11.0, 0.0, -1.0, aoi_tier::inner, -1.0, "inside the exit band"},
      {aoi_tier::inner, 12.5, 0.0, -1.0, aoi_tier::inner, 0.0, "beyond exit: the dwell starts"},
      {aoi_tier::inner, 12.5, 0.5, 0.0, aoi_tier::inner, 0.0, "still dwelling"},
      {aoi_tier::inner, 12.5, 1.0, 0.0, aoi_tier::middle, -1.0, "dwell passed: one tier down"},
      {aoi_tier::inner, 11.5, 0.6, 0.0, aoi_tier::inner, -1.0, "back inside: the dwell resets"},
      {aoi_tier::inner, 30.0, 1.0, 0.0, aoi_tier::middle, 1.0, "one tier; middle's dwell starts"},
      {aoi_tier::middle, 30.0, 1.5, 1.0, aoi_tier::middle, 1.0, "middle to outer waits"},
      {aoi_tier::middle, 30.0, 2.0, 1.0, aoi_tier::outer, -1.0, "middle to outer after its dwell"},
      {aoi_tier::middle, 17.0, 5.0, -1.0, aoi_tier::middle, -1.0, "inside middle's exit"},
      {aoi_tier::middle, 20.0, 0.0, -1.0, aoi_tier::middle, 0.0, "beyond middle's exit"},
      {aoi_tier::middle, 20.0, 0.99, 0.0, aoi_tier::middle, 0.0, "just short of the dwell"},
      {aoi_tier::middle, 20.0, 1.0, 0.0, aoi_tier::outer, -1.0, "the dwell passed"},
      {aoi_tier::middle, 9.0, 0.7, 0.5, aoi_tier::inner, -1.0, "upgrades are immediate"},
      {aoi_tier::middle, 10.0, 0.0, -1.0, aoi_tier::inner, -1.0, "the enter radius is inclusive"},
      {aoi_tier::outer, 5.0, 0.0, -1.0, aoi_tier::inner, -1.0, "outer to inner in one step"},
      {aoi_tier::outer, 15.0, 0.0, -1.0, aoi_tier::middle, -1.0, "outer to middle"},
      {aoi_tier::outer, 17.0, 0.0, -1.0, aoi_tier::outer, -1.0, "outside middle's enter"},
      {aoi_tier::outer, 1000.0, 9.0, -1.0, aoi_tier::outer, -1.0, "outer has no exit"},
  };
  for (const row &r : rows) {
    double beyond = r.beyond_in;
    EXPECT_EQ(ra::step_tier(r.cur, r.d_m, r.now_s, beyond, h), r.want) << r.what;
    EXPECT_DOUBLE_EQ(beyond, r.beyond_out) << r.what;
  }

  // A pair pacing across inner's exit never leaves inner while each excursion is shorter than
  // the dwell.
  aoi_tier t = aoi_tier::inner;
  double beyond = -1.0;
  for (int i = 0; i < 100; ++i) {
    const double now = 0.1 * i;
    t = ra::step_tier(t, (i / 5) % 2 ? 12.5 : 11.5, now, beyond, h); // 0.5 s out, 0.5 s in
    EXPECT_EQ(t, aoi_tier::inner) << "t=" << now;
  }
}

TEST(RoomAoiTest, UpdateRebuildsSnapshotOnlyOnChange) {
  ra::room_aoi aoi(kRoom, ra::hall_aoi_params());
  EXPECT_EQ(aoi.snapshot()->generation(), 0u);
  EXPECT_FALSE(aoi.update({}, 0, 0.0)) << "nobody here: nothing to decide";

  EXPECT_TRUE(aoi.update({at("a", 0), at("b", 5)}, 0, 0.0));
  const std::shared_ptr<const ra::aoi_snapshot> s1 = aoi.snapshot();
  EXPECT_EQ(s1->generation(), 1u);
  EXPECT_EQ(s1->tier("a", "b"), aoi_tier::inner);
  EXPECT_EQ(s1->participants(), (std::vector<std::string>{"a", "b"}));

  // The same positions, and a move that stays inside the tier, rebuild nothing.
  EXPECT_FALSE(aoi.update({at("a", 0), at("b", 5)}, 6, 0.05));
  EXPECT_FALSE(aoi.update({at("a", 0), at("b", 11.5)}, 12, 0.1));
  EXPECT_EQ(aoi.snapshot().get(), s1.get());

  // Beyond inner's exit nothing changes until the dwell has passed.
  EXPECT_FALSE(aoi.update({at("a", 0), at("b", 14)}, 18, 0.2));
  EXPECT_FALSE(aoi.update({at("a", 0), at("b", 14)}, 96, 0.9));
  EXPECT_EQ(aoi.snapshot().get(), s1.get());
  EXPECT_TRUE(aoi.update({at("a", 0), at("b", 14)}, 144, 1.25));
  const std::shared_ptr<const ra::aoi_snapshot> s2 = aoi.snapshot();
  EXPECT_NE(s2.get(), s1.get());
  EXPECT_EQ(s2->generation(), 2u);
  EXPECT_EQ(s2->tier("a", "b"), aoi_tier::middle);
  EXPECT_EQ(s2->tier("b", "a"), aoi_tier::middle);
  EXPECT_EQ(s1->tier("a", "b"), aoi_tier::inner) << "a published snapshot never changes";

  // A join and a leave each rebuild; the leaver is departed until it rejoins.
  EXPECT_TRUE(aoi.update({at("a", 0), at("b", 14), at("c", 1)}, 150, 1.3));
  EXPECT_EQ(aoi.snapshot()->generation(), 3u);
  EXPECT_TRUE(aoi.update({at("a", 0), at("b", 14)}, 156, 1.35));
  EXPECT_EQ(aoi.snapshot()->generation(), 4u);
  EXPECT_EQ(aoi.departed(), (std::set<std::string>{"c"}));
  EXPECT_FALSE(aoi.update({at("a", 0), at("b", 14)}, 162, 1.4));
  EXPECT_TRUE(aoi.update({at("a", 0), at("b", 14), at("c", 1)}, 168, 1.45));
  EXPECT_TRUE(aoi.departed().empty());

  // A pid listed twice counts once, at its first position.
  EXPECT_FALSE(aoi.update({at("a", 0), at("b", 14), at("c", 1), at("c", 100)}, 174, 1.5));
  EXPECT_EQ(aoi.snapshot()->tier("a", "c"), aoi_tier::inner);
}

// ── state delivery ──────────────────────────────────────────────────────────

TEST(RoomAoiTest, DeliverPathRules) {
  ra::room_aoi aoi(kRoom, ra::hall_aoi_params());
  place_abcde(aoi);
  const std::shared_ptr<const ra::aoi_snapshot> s = aoi.snapshot();
  ASSERT_EQ(s->tier("a", "b"), aoi_tier::inner);
  ASSERT_EQ(s->tier("a", "c"), aoi_tier::middle);
  ASSERT_EQ(s->tier("a", "d"), aoi_tier::outer);
  ASSERT_EQ(aoi.departed(), (std::set<std::string>{"e"}));

  const auto yes = [&](const std::string &receiver, const std::string &path) {
    EXPECT_TRUE(s->deliver_path(receiver, path)) << receiver << " <- " << path;
  };
  const auto no = [&](const std::string &receiver, const std::string &path) {
    EXPECT_FALSE(s->deliver_path(receiver, path)) << receiver << " <- " << path;
  };

  // The receiver's whole own subtree, intent included.
  yes("a", ppath("a", {"pose", "pos", "x"}));
  yes("a", ppath("a", {"pose", "server_tick"}));
  yes("a", ppath("a", {"intent", "move", "x"}));
  yes("a", ppath("a", {"net", "rtt_ms"}));

  // Pose: only an inner subject's, and then its whole subtree.
  yes("a", ppath("b", {"pose", "pos", "x"}));
  yes("a", ppath("b", {"pose", "server_tick"}));
  yes("a", ppath("b", {"pose"}));
  no("a", ppath("c", {"pose", "pos", "x"}));
  no("a", ppath("c", {"pose", "vel", "lin", "x"}));
  no("a", ppath("c", {"pose", "server_tick"}));
  no("a", ppath("c", {"pose"}));
  no("a", ppath("d", {"pose", "orient", "w"}));
  no("a", ppath("d", {"pose", "server_tick"}));
  yes("c", ppath("b", {"pose", "pos", "x"})); // b and c are 9 m apart
  no("d", ppath("a", {"pose", "pos", "x"}));  // tiers are symmetric

  // Another participant's intent, whatever its tier.
  no("a", ppath("b", {"intent", "move", "x"}));
  no("a", ppath("b", {"intent", "tick"}));
  no("a", ppath("b", {"intent"}));
  no("a", ppath("d", {"intent", "yaw"}));

  // The roster stays room-wide: presence, profile, net, voice and streams of everyone present.
  for (const char *q : {"b", "c", "d"}) {
    yes("a", ppath(q, {"presence", "joined"}));
    yes("a", ppath(q, {"profile", "name"}));
    yes("a", ppath(q, {"net", "rtt_ms"}));
    yes("a", ppath(q, {"net", "relay", "in_kbps"}));
    yes("a", ppath(q, {"voice", "talking"}));
    yes("a", ppath(q, {"streams", "camera", "token"}));
  }

  // A departed subject's ghost: pose, intent, net and voice are withheld; the rest is not.
  no("a", ppath("e", {"pose", "pos", "x"}));
  no("a", ppath("e", {"pose", "server_tick"}));
  no("a", ppath("e", {"intent", "move", "x"}));
  no("a", ppath("e", {"net", "rtt_ms"}));
  no("a", ppath("e", {"net", "relay", "rtt_ms"}));
  no("a", ppath("e", {"voice", "talking"}));
  yes("a", ppath("e", {"presence", "joined"}));
  yes("a", ppath("e", {"profile", "name"}));
  yes("a", ppath("e", {"streams", "camera", "endpoint"}));
  // ... but a departed pid reconnecting under the same id still gets its own (zeroed) intent.
  yes("e", ppath("e", {"intent", "move", "x"}));
  yes("e", ppath("e", {"pose", "pos", "x"}));

  // Room-level subtrees.
  yes("a", rpath({"scene", "rev"}));
  yes("a", rpath({"scene", "objects", "ref", "present"}));
  yes("a", rpath({"server", "net", "links"}));
  yes("a", rpath({"aoi", "inner_enter_m"}));

  // A pid this snapshot never saw delivers everything.
  yes("a", ppath("zz", {"pose", "pos", "x"}));
  yes("a", ppath("zz", {"intent", "move", "x"}));

  // Bare nodes, other rooms and malformed paths deliver.
  yes("a", ppath("d"));
  yes("a", ppath("e"));
  yes("a", rpath({"participants"}));
  yes("a", rpath({}));
  yes("a", "conf");
  yes("a", "");
  yes("a", ra::room_participant_path("r2", "d") + cvc::state::SEPARATOR + "pose");
  yes("a", ra::room_participant_path("r10", "d") + cvc::state::SEPARATOR + "pose");
  yes("a", ppath("", {"pose"}));
  yes("a", ppath("d") + cvc::state::SEPARATOR);
  yes("a", rpath({"participantsd", "pose"}));
  yes("a", ppath("d", {"posex"}));
  yes("a", ppath("d", {"poses", "x"}));
  yes("a", "x" + ppath("d", {"pose"}));
  yes("a", ppath("d", {"intentional"}));

  // A receiver the relay has not placed: tiers are unknown, so pose delivers.
  yes("zz", ppath("d", {"pose", "pos", "x"}));

  // The registry filter is the snapshot bound to one receiver.
  const ra::aoi_state_filter f(s, "a");
  EXPECT_FALSE(f.deliver(ppath("c", {"pose", "pos", "x"})));
  EXPECT_TRUE(f.deliver(ppath("b", {"pose", "pos", "x"})));
  const ra::aoi_state_filter none(nullptr, "a");
  EXPECT_TRUE(none.deliver(ppath("c", {"pose", "pos", "x"})));

  cvc::state_peer_registry reg;
  reg.add_peer("a", "conf");
  ASSERT_TRUE(reg.set_filter("a", std::make_shared<ra::aoi_state_filter>(s, "a")));
  EXPECT_FALSE(reg.should_deliver("a", ppath("c", {"pose", "pos", "x"})));
  EXPECT_TRUE(reg.should_deliver("a", ppath("b", {"pose", "pos", "x"})));
  EXPECT_TRUE(reg.should_deliver("a", ppath("c", {"profile", "name"})));

  // An empty snapshot knows no one and delivers everything.
  const ra::aoi_snapshot empty{};
  EXPECT_EQ(empty.generation(), 0u);
  EXPECT_TRUE(empty.deliver_path("a", ppath("c", {"pose", "pos", "x"})));
  EXPECT_TRUE(empty.deliver_path("a", ppath("c", {"intent", "move", "x"})));
  EXPECT_EQ(empty.media("a", "d", "camera", "mjpeg"), aoi_media::deliver);
  EXPECT_EQ(empty.tier("a", "d"), aoi_tier::inner);
}

// ── media ───────────────────────────────────────────────────────────────────

TEST(RoomAoiTest, MediaActions) {
  EXPECT_EQ(ra::conference_aoi_params().video_full_max, 0) << "the video cap is off by default";
  EXPECT_EQ(ra::hall_aoi_params().video_full_max, 0);
  {
    ra::room_aoi aoi(kRoom, ra::hall_aoi_params());
    place_abcde(aoi);
    const std::shared_ptr<const ra::aoi_snapshot> s = aoi.snapshot();
    // inner: everything.
    EXPECT_EQ(s->media("a", "b", "camera", "mjpeg"), aoi_media::deliver);
    EXPECT_EQ(s->media("a", "b", "mic", "f32"), aoi_media::deliver);
    // middle: an mjpeg camera is a thumbnail; nothing else is decimated.
    EXPECT_EQ(s->media("a", "c", "camera", "mjpeg"), aoi_media::thumbnail);
    EXPECT_EQ(s->media("a", "c", "camera", "h264"), aoi_media::deliver);
    EXPECT_EQ(s->media("a", "c", "screen", "mjpeg"), aoi_media::deliver);
    EXPECT_EQ(s->media("a", "c", "mic", "f32"), aoi_media::deliver);
    // outer: nothing.
    EXPECT_EQ(s->media("a", "d", "camera", "mjpeg"), aoi_media::drop);
    EXPECT_EQ(s->media("a", "d", "camera", "h264"), aoi_media::drop);
    EXPECT_EQ(s->media("a", "d", "mic", "f32"), aoi_media::drop);
    EXPECT_EQ(s->media("d", "a", "mic", "f32"), aoi_media::drop);
    // Unknown ends, a departed publisher and oneself: delivered.
    EXPECT_EQ(s->media("a", "zz", "camera", "mjpeg"), aoi_media::deliver);
    EXPECT_EQ(s->media("zz", "d", "camera", "mjpeg"), aoi_media::deliver);
    EXPECT_EQ(s->media("a", "e", "mic", "f32"), aoi_media::deliver);
    EXPECT_EQ(s->media("a", "a", "camera", "mjpeg"), aoi_media::deliver);
  }
  {
    // The opt-in cap: a's six nearest inner cameras at full rate, the rest thumbnails, and a
    // member gives way only to an outsider more than video_rank_margin_m (1 m) nearer.
    ra::aoi_params p = ra::hall_aoi_params();
    p.video_full_max = 6;
    ra::room_aoi aoi(kRoom, p);
    std::vector<ra::room_aoi::position> ps{at("a", 0)};
    const char *const ids[] = {"s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8"};
    for (int i = 0; i < 8; ++i)
      ps.push_back(at(ids[i], 1.0 + i)); // s<i> is i metres from a: all inner
    const auto move_s7 = [&](double x, std::uint64_t tick) {
      for (ra::room_aoi::position &q : ps)
        if (q.pid == "s7")
          q.x = x;
      aoi.update(ps, tick, 0.05 * static_cast<double>(tick) / 6.0);
      return aoi.snapshot();
    };
    std::shared_ptr<const ra::aoi_snapshot> s = move_s7(7.0, 0);
    for (const char *q : {"s1", "s2", "s3", "s4", "s5", "s6"})
      EXPECT_EQ(s->media("a", q, "camera", "mjpeg"), aoi_media::deliver) << q;
    EXPECT_EQ(s->media("a", "s7", "camera", "mjpeg"), aoi_media::thumbnail);
    EXPECT_EQ(s->media("a", "s8", "camera", "mjpeg"), aoi_media::thumbnail);
    EXPECT_EQ(s->media("a", "s7", "mic", "f32"), aoi_media::deliver) << "the mic is on/off only";
    EXPECT_EQ(s->media("a", "s7", "camera", "h264"), aoi_media::deliver) << "mjpeg-only";

    s = move_s7(5.5, 6); // 0.5 m nearer than s6: within the margin, no swap
    EXPECT_EQ(s->media("a", "s7", "camera", "mjpeg"), aoi_media::thumbnail);
    EXPECT_EQ(s->media("a", "s6", "camera", "mjpeg"), aoi_media::deliver);

    s = move_s7(4.9, 12); // 1.1 m nearer than s6: swapped in
    EXPECT_EQ(s->media("a", "s7", "camera", "mjpeg"), aoi_media::deliver);
    EXPECT_EQ(s->media("a", "s6", "camera", "mjpeg"), aoi_media::thumbnail);

    s = move_s7(6.5, 18); // now 0.5 m farther than s6: keeps its place
    EXPECT_EQ(s->media("a", "s7", "camera", "mjpeg"), aoi_media::deliver);
    EXPECT_EQ(s->media("a", "s6", "camera", "mjpeg"), aoi_media::thumbnail);

    s = move_s7(7.5, 24); // 1.5 m farther than s6: gives way
    EXPECT_EQ(s->media("a", "s7", "camera", "mjpeg"), aoi_media::thumbnail);
    EXPECT_EQ(s->media("a", "s6", "camera", "mjpeg"), aoi_media::deliver);

    ps.erase(ps.begin() + 1); // s1 leaves: the nearest outsider (s7) fills the slot
    s = move_s7(7.5, 30);
    ASSERT_EQ(aoi.departed(), (std::set<std::string>{"s1"}));
    EXPECT_EQ(s->media("a", "s7", "camera", "mjpeg"), aoi_media::deliver);
    EXPECT_EQ(s->media("a", "s8", "camera", "mjpeg"), aoi_media::thumbnail);
  }
}

// ── pose pushes ─────────────────────────────────────────────────────────────

TEST(RoomAoiTest, PosePushSchedule) {
  ra::room_aoi aoi(kRoom, ra::hall_aoi_params()); // 24-tick pushes, 600-tick refresh
  // a-c 14 m (middle), a-d 40 m and c-d 26 m (outer).
  ASSERT_TRUE(aoi.update({at("a", 0), at("c", 14), at("d", 40)}, 0, 0.0));

  // Every new non-inner pair is pushed at once, so nobody waits for the refresh.
  EXPECT_EQ(as_map(aoi.take_pose_pushes(0, {})),
            (push_map{{"a", {"c", "d"}}, {"c", {"a", "d"}}, {"d", {"a", "c"}}}));
  EXPECT_TRUE(aoi.take_pose_pushes(1, {}).empty());

  // A moving subject is pushed every 24 ticks, and only when it moved since its last push.
  EXPECT_TRUE(aoi.take_pose_pushes(10, {{"c", 5}}).empty());
  EXPECT_EQ(as_map(aoi.take_pose_pushes(24, {{"c", 5}})), (push_map{{"a", {"c"}}, {"d", {"c"}}}));
  EXPECT_TRUE(aoi.take_pose_pushes(30, {{"c", 5}}).empty());
  EXPECT_TRUE(aoi.take_pose_pushes(40, {{"c", 40}}).empty());
  EXPECT_EQ(as_map(aoi.take_pose_pushes(48, {{"c", 40}})), (push_map{{"a", {"c"}}, {"d", {"c"}}}));

  // A still subject is refreshed every 600 ticks.
  EXPECT_TRUE(aoi.take_pose_pushes(599, {{"c", 40}}).empty());
  EXPECT_EQ(as_map(aoi.take_pose_pushes(600, {{"c", 40}})),
            (push_map{{"a", {"d"}}, {"c", {"a", "d"}}, {"d", {"a"}}}));

  // Entering inner pushes both directions at once; d still gets c's motion at the cadence.
  ASSERT_TRUE(aoi.update({at("a", 0), at("c", 5), at("d", 40)}, 606, 5.05));
  ASSERT_EQ(aoi.snapshot()->tier("a", "c"), aoi_tier::inner);
  EXPECT_EQ(as_map(aoi.take_pose_pushes(606, {{"c", 606}})),
            (push_map{{"a", {"c"}}, {"c", {"a"}}, {"d", {"c"}}}));

  // An inner pair is never pushed: the live stream carries it.
  EXPECT_EQ(as_map(aoi.take_pose_pushes(700, {{"c", 700}})), (push_map{{"d", {"c"}}}));

  // A new pair that starts inner is not pushed; one that starts outer is.
  ASSERT_TRUE(aoi.update({at("a", 0), at("c", 5), at("d", 40), at("e", 1)}, 710, 6.0));
  EXPECT_EQ(as_map(aoi.take_pose_pushes(710, {{"c", 700}})),
            (push_map{{"d", {"e"}}, {"e", {"d"}}}));

  // A departed subject or receiver is never pushed.
  ASSERT_TRUE(aoi.update({at("a", 0), at("c", 5), at("e", 1)}, 720, 6.1));
  EXPECT_TRUE(aoi.take_pose_pushes(720, {{"c", 720}}).empty());
  EXPECT_TRUE(aoi.take_pose_pushes(1400, {{"c", 720}}).empty()) << "all inner: no refresh";

  // A downgrade resumes the cadence from the last live sample.
  EXPECT_FALSE(aoi.update({at("a", 0), at("c", 14), at("e", 1)}, 1406, 6.5)); // dwell starts
  ASSERT_TRUE(aoi.update({at("a", 0), at("c", 14), at("e", 1)}, 1526, 7.5));
  ASSERT_EQ(aoi.snapshot()->tier("a", "c"), aoi_tier::middle);
  ASSERT_EQ(aoi.snapshot()->tier("e", "c"), aoi_tier::middle); // 13 m
  EXPECT_EQ(as_map(aoi.take_pose_pushes(1526, {{"c", 1526}})),
            (push_map{{"a", {"c"}}, {"e", {"c"}}}));
  EXPECT_TRUE(aoi.take_pose_pushes(1527, {{"c", 1527}}).empty());
}

TEST(RoomAoiTest, PushIntervalFitsPeerSpan) {
  // A peer walks between two of its samples over at most max_peer_span_ticks. A push due every
  // interval leaves up to one relay room step late (arisrv steps the room at ~20 Hz), so the
  // interval plus that step must fit in the span or a walking peer stutters.
  for (const cvc::ariadne::holon_config &cfg :
       {cvc::ariadne::holon_config{}, cvc::ariadne::room_holon_config()}) {
    const double tick_hz = 1.0 / cfg.clock.fixed_dt;
    EXPECT_DOUBLE_EQ(tick_hz, 120.0) << "room_aoi schedules pushes in 120 Hz ticks";
    const auto step_ticks = static_cast<std::uint64_t>(std::llround(0.05 * tick_hz));
    EXPECT_EQ(step_ticks, 6u);
    for (const ra::aoi_params &p : {ra::conference_aoi_params(), ra::hall_aoi_params()}) {
      const std::uint64_t every = ra::aoi_pose_push_interval_ticks(p, tick_hz);
      EXPECT_EQ(every, static_cast<std::uint64_t>(std::llround(tick_hz / p.pose_push_hz)));
      EXPECT_EQ(every, 24u);
      EXPECT_LE(every + step_ticks, cfg.max_peer_span_ticks) << p.preset;
      EXPECT_EQ(ra::aoi_pose_refresh_ticks(p, tick_hz), 600u) << p.preset;
    }
  }
  ra::aoi_params off;
  off.pose_push_hz = 0.0;
  off.pose_refresh_s = 0.0;
  EXPECT_EQ(ra::aoi_pose_push_interval_ticks(off), 0u);
  EXPECT_EQ(ra::aoi_pose_refresh_ticks(off), 0u);
}

// ── presets and the audio curve ─────────────────────────────────────────────

TEST(RoomAoiTest, AudioGainCurve) {
  const ra::aoi_params h = ra::hall_aoi_params();
  for (double d : {0.0, 1.0, 5.0, 10.0})
    EXPECT_FLOAT_EQ(ra::aoi_audio_gain(d, h), 1.0f) << d;
  EXPECT_NEAR(ra::aoi_audio_gain(12.0, h), 10.0 / 12.0, 1e-6);
  EXPECT_NEAR(ra::aoi_audio_gain(14.0, h), 10.0 / 14.0, 1e-6) << "the fade starts at 14 m";
  EXPECT_NEAR(ra::aoi_audio_gain(15.0, h), 10.0 / 15.0 * 0.5, 1e-6);
  for (double d : {16.0, 17.0, 18.0, 30.0})
    EXPECT_FLOAT_EQ(ra::aoi_audio_gain(d, h), 0.0f) << "silent from middle_enter: " << d;
  float prev = 1.0f;
  for (int i = 0; i <= 400; ++i) {
    const float g = ra::aoi_audio_gain(0.05 * i, h);
    EXPECT_LE(g, prev) << "monotone at " << 0.05 * i;
    prev = g;
  }

  const ra::aoi_params c = ra::conference_aoi_params();
  for (int i = 0; i <= 152; ++i)
    EXPECT_FLOAT_EQ(ra::aoi_audio_gain(0.25 * i, c), 1.0f) << 0.25 * i;
  EXPECT_NEAR(ra::aoi_audio_gain(39.0, c), 0.5, 1e-6);
  EXPECT_FLOAT_EQ(ra::aoi_audio_gain(40.0, c), 0.0f);

  ra::aoi_params off = h;
  off.enabled = false;
  EXPECT_FLOAT_EQ(ra::aoi_audio_gain(30.0, off), 1.0f);
}

TEST(RoomAoiTest, ConferencePresetCoversDefaultRoom) {
  // The seeded room's ground plane is the floor everyone walks on.
  const cvc::ariadne::SceneNode *ground = nullptr;
  const std::vector<cvc::ariadne::SceneNode> objs = ra::default_scene_objects();
  for (const cvc::ariadne::SceneNode &n : objs)
    if (n.source_primitive == "plane")
      ground = &n;
  ASSERT_NE(ground, nullptr);
  const double hx = 0.5 * ground->plane_size * ground->scale[0];
  const double hy = 0.5 * ground->plane_size * ground->scale[1];
  EXPECT_DOUBLE_EQ(hx, 10.0);
  EXPECT_DOUBLE_EQ(hy, 10.0);

  // Every pair of points on a 0.5 m lattice over the plane is inner at first sight ...
  const ra::aoi_params p = ra::conference_aoi_params();
  std::vector<std::pair<double, double>> pts;
  const int nx = static_cast<int>(std::llround(2.0 * hx / 0.5));
  const int ny = static_cast<int>(std::llround(2.0 * hy / 0.5));
  for (int i = 0; i <= nx; ++i)
    for (int j = 0; j <= ny; ++j)
      pts.emplace_back(ground->position[0] - hx + 0.5 * i, ground->position[1] - hy + 0.5 * j);
  std::size_t not_inner = 0;
  for (std::size_t i = 0; i < pts.size(); ++i)
    for (std::size_t j = i + 1; j < pts.size(); ++j)
      if (ra::initial_tier(std::hypot(pts[j].first - pts[i].first, pts[j].second - pts[i].second),
                           p) != aoi_tier::inner)
        ++not_inner;
  EXPECT_EQ(not_inner, 0u);

  // ... and stays inner: the diagonal never passes inner's exit radius.
  const double diagonal = std::hypot(2.0 * hx, 2.0 * hy);
  aoi_tier t = aoi_tier::inner;
  double beyond = -1.0;
  for (int i = 0; i < 200; ++i)
    t = ra::step_tier(t, diagonal, 0.05 * i, beyond, p);
  EXPECT_EQ(t, aoi_tier::inner);

  // So avatars at the corners and the centre get everything, live, and nothing is pushed.
  ra::room_aoi aoi(kRoom, p);
  aoi.update({at("a", -hx, -hy), at("b", hx, hy), at("c", -hx, hy), at("d", hx, -hy), at("o", 0)},
             0, 0.0);
  const std::shared_ptr<const ra::aoi_snapshot> s = aoi.snapshot();
  for (const char *r : {"a", "b", "c", "d", "o"})
    for (const char *q : {"a", "b", "c", "d", "o"}) {
      EXPECT_EQ(s->tier(r, q), aoi_tier::inner) << r << " " << q;
      EXPECT_TRUE(s->deliver_path(r, ppath(q, {"pose", "pos", "x"}))) << r << " " << q;
      EXPECT_EQ(s->media(r, q, "camera", "mjpeg"), aoi_media::deliver) << r << " " << q;
      EXPECT_EQ(s->media(r, q, "mic", "f32"), aoi_media::deliver) << r << " " << q;
    }
  EXPECT_TRUE(aoi.take_pose_pushes(0, {}).empty());
  EXPECT_TRUE(aoi.take_pose_pushes(600, {{"a", 300}}).empty());
  EXPECT_FLOAT_EQ(ra::aoi_audio_gain(diagonal, p), 1.0f);
}

// ── replicated parameters ───────────────────────────────────────────────────

TEST(RoomAoiTest, ParamsRoundTrip) {
  EXPECT_EQ(ra::room_aoi_path(kRoom), rpath({"aoi"}));
  cvc::app app;
  ra::aoi_params out = ra::conference_aoi_params();
  out.preset = "untouched";
  EXPECT_FALSE(ra::read_aoi_params(app, kRoom, out));
  EXPECT_EQ(out.preset, "untouched");

  ra::aoi_params in = ra::hall_aoi_params();
  in.enabled = false;
  in.inner_enter_m = 10.1;
  in.downgrade_dwell_s = 0.75;
  in.pose_push_hz = 7.5;
  in.pose_refresh_s = 2.5;
  in.thumb_fps = 3.25;
  in.video_rank_margin_m = 1.5;
  in.audio_fade_m = 2.5;
  in.video_full_max = 6;
  ra::write_aoi_params(app, kRoom, in);
  EXPECT_NE(app.root().findDescendant(rpath({"aoi", "inner_enter_m"})), nullptr);
  EXPECT_NE(app.root().findDescendant(rpath({"aoi", "audio_rolloff"})), nullptr);

  const auto expect_same = [](const ra::aoi_params &a, const ra::aoi_params &b) {
    EXPECT_EQ(a.enabled, b.enabled);
    EXPECT_EQ(a.preset, b.preset);
    EXPECT_EQ(a.inner_enter_m, b.inner_enter_m);
    EXPECT_EQ(a.inner_exit_m, b.inner_exit_m);
    EXPECT_EQ(a.middle_enter_m, b.middle_enter_m);
    EXPECT_EQ(a.middle_exit_m, b.middle_exit_m);
    EXPECT_EQ(a.downgrade_dwell_s, b.downgrade_dwell_s);
    EXPECT_EQ(a.pose_push_hz, b.pose_push_hz);
    EXPECT_EQ(a.pose_refresh_s, b.pose_refresh_s);
    EXPECT_EQ(a.thumb_fps, b.thumb_fps);
    EXPECT_EQ(a.video_rank_margin_m, b.video_rank_margin_m);
    EXPECT_EQ(a.audio_fade_m, b.audio_fade_m);
    EXPECT_EQ(a.video_full_max, b.video_full_max);
    EXPECT_EQ(a.audio_rolloff, b.audio_rolloff);
  };
  ra::aoi_params got = ra::conference_aoi_params();
  ASSERT_TRUE(ra::read_aoi_params(app, kRoom, got));
  expect_same(got, in);

  // Rewriting the same values changes nothing; a changed field reads back.
  ra::write_aoi_params(app, kRoom, in);
  in.video_full_max = 0;
  in.enabled = true;
  ra::write_aoi_params(app, kRoom, in);
  got = ra::aoi_params{};
  ASSERT_TRUE(ra::read_aoi_params(app, kRoom, got));
  expect_same(got, in);

  // Each room has its own.
  ra::aoi_params other;
  EXPECT_FALSE(ra::read_aoi_params(app, "r2", other));

  // The default parameters are the conference preset.
  expect_same(ra::aoi_params{}, ra::conference_aoi_params());
}

TEST(RoomAoiTest, ParamsReplicateToAClient) {
  cvc::app server_app, client_app;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard ss(server_app, "conf", "server");
  cvc::state_cluster_shard cs(client_app, "conf", "client");
  ss.attach();
  cs.attach();
  tp.register_shard(&ss);
  tp.register_shard(&cs);

  ra::write_aoi_params(server_app, kRoom, ra::hall_aoi_params());
  tp.pump_all();
  ra::aoi_params got;
  ASSERT_TRUE(ra::read_aoi_params(client_app, kRoom, got));
  EXPECT_EQ(got.preset, "hall");
  EXPECT_EQ(got.inner_enter_m, 10.0);
  EXPECT_EQ(got.middle_enter_m, 16.0);
  EXPECT_TRUE(got.audio_rolloff);
  EXPECT_FLOAT_EQ(ra::aoi_audio_gain(15.0, got), ra::aoi_audio_gain(15.0, ra::hall_aoi_params()));
}

// ── the kill switch ─────────────────────────────────────────────────────────

TEST(RoomAoiTest, DisabledDeliversAll) {
  ra::aoi_params p = ra::hall_aoi_params();
  p.enabled = false;
  p.video_full_max = 1;
  ra::room_aoi aoi(kRoom, p);
  place_abcde(aoi);
  const std::shared_ptr<const ra::aoi_snapshot> s = aoi.snapshot();
  EXPECT_FALSE(s->enabled());
  for (const char *q : {"b", "c", "d", "e"}) {
    EXPECT_TRUE(s->deliver_path("a", ppath(q, {"pose", "pos", "x"}))) << q;
    EXPECT_TRUE(s->deliver_path("a", ppath(q, {"pose", "server_tick"}))) << q;
    EXPECT_TRUE(s->deliver_path("a", ppath(q, {"intent", "move", "x"}))) << q;
    EXPECT_TRUE(s->deliver_path("a", ppath(q, {"net", "rtt_ms"}))) << q;
    EXPECT_TRUE(s->deliver_path("a", ppath(q, {"voice", "talking"}))) << q;
    EXPECT_EQ(s->media("a", q, "camera", "mjpeg"), aoi_media::deliver) << q;
    EXPECT_EQ(s->media("a", q, "mic", "f32"), aoi_media::deliver) << q;
  }
  EXPECT_TRUE(ra::aoi_state_filter(s, "a").deliver(ppath("d", {"pose", "pos", "x"})));
  EXPECT_TRUE(aoi.take_pose_pushes(0, {}).empty());
  EXPECT_TRUE(aoi.take_pose_pushes(24, {{"c", 10}, {"d", 10}}).empty());
  EXPECT_TRUE(aoi.take_pose_pushes(600, {}).empty());
  EXPECT_FLOAT_EQ(ra::aoi_audio_gain(30.0, p), 1.0f);
}

// ── header hygiene ──────────────────────────────────────────────────────────

// <windows.h> was included ahead of room_aoi.h at the top of this file, so this TU compiling at
// all is the test: a tier or media name that collided with one of its macros would not.
TEST(RoomAoiTest, CompilesAfterWindowsH) {
  EXPECT_NE(aoi_tier::inner, aoi_tier::middle);
  EXPECT_NE(aoi_tier::middle, aoi_tier::outer);
  EXPECT_NE(aoi_media::deliver, aoi_media::thumbnail);
  EXPECT_NE(aoi_media::thumbnail, aoi_media::drop);
  EXPECT_EQ(ra::initial_tier((std::min)(1.0, 2.0), ra::aoi_params{}), aoi_tier::inner);
}
