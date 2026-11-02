// The conference's pose stream: the per-connection snapshot codec (pose_stream.h), the relay that
// sends each viewer its snapshots at the rate its round trip allows (room_pose_relay.h), and a
// client applying them to its holon (holon_sync.h apply_pose_snapshot).

#include <cmath>
#include <cstdint>
#include <cvc/ariadne/conference/room_pose_relay.h>
#include <cvc/ariadne/holon.h>
#include <cvc/ariadne/holon_sync.h>
#include <cvc/ariadne/pose_stream.h>
#include <cvc/state/state_transport.h>
#include <gtest/gtest.h>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

using cvc::ariadne::pose_stream_decoder;
using cvc::ariadne::pose_stream_encoder;
using cvc::ariadne::pose_subject;
using cvc::physics::body_state;
using cvc::physics::vec3;

namespace {

body_state walking(double x, double y, double vx, double vy, double yaw = 0.0) {
  body_state s;
  s.position = vec3{x, y, 0.9};
  s.linear_velocity = vec3{vx, vy, 0.0};
  s.rotation = cvc::physics::quat{std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
  return s;
}

pose_subject subject(const std::string &pid, const body_state &s, bool due = true) {
  pose_subject p;
  p.pid = pid;
  p.state = s;
  p.due = due;
  return p;
}

double dist(const vec3 &a, const vec3 &b) {
  const vec3 d = a - b;
  return std::sqrt(cvc::physics::dot(d, d));
}

const cvc::ariadne::pose_sample *find(const std::vector<cvc::ariadne::pose_sample> &v,
                                      const std::string &pid) {
  for (const auto &s : v)
    if (s.pid == pid)
      return &s;
  return nullptr;
}

} // namespace

TEST(PoseStream, QuantizedPosesRoundTripWithinHalfAMillimetre) {
  const body_state s = walking(3.14159, -2.71828, 2.5, -1.25, 1.0);
  const body_state r = cvc::ariadne::dequantize_pose(cvc::ariadne::quantize_pose(s));
  EXPECT_LE(dist(r.position, s.position), 0.0009);
  EXPECT_LE(dist(r.linear_velocity, s.linear_velocity), 0.0009);
  const auto &q = r.rotation, &p = s.rotation;
  EXPECT_GT(std::fabs(q.w * p.w + q.x * p.x + q.y * p.y + q.z * p.z), 1.0 - 1e-8);
  // A quaternion and its negation are one rotation: they quantize the same, so a sign flip is no
  // change.
  body_state flipped = s;
  flipped.rotation = {-p.w, -p.x, -p.y, -p.z};
  EXPECT_EQ(cvc::ariadne::quantize_pose(flipped), cvc::ariadne::quantize_pose(s));
}

// The first snapshot names each participant with its absolute pose; after it, only a change is
// sent, as a difference, and the decoder rebuilds the pose from it.
TEST(PoseStream, ASlotIsNamedOnceThenSentAsDifferences) {
  pose_stream_encoder enc;
  pose_stream_decoder dec;
  std::vector<cvc::ariadne::pose_sample> out;

  cvc::pose_snapshot s1 = enc.encode(
      100, {subject("a", walking(1.0, 2.0, 4.0, 0.0)), subject("b", walking(-3.0, 0.5, 0.0, 0.0))},
      120);
  enc.commit();
  EXPECT_EQ(s1.base_seq, 0u) << "a stream starts afresh";
  ASSERT_EQ(s1.entries.size(), 2u);
  EXPECT_EQ(s1.entries[0].pid, "a");
  EXPECT_EQ(s1.entries[0].x, 1000);
  EXPECT_EQ(s1.entries[0].vx, 4000);
  ASSERT_TRUE(dec.decode(s1, out));
  ASSERT_EQ(out.size(), 2u);

  // a walked 67 mm (4 m/s for 2 ticks at 120 Hz); b stood still.
  cvc::pose_snapshot s2 = enc.encode(
      102,
      {subject("a", walking(1.067, 2.0, 4.0, 0.0)), subject("b", walking(-3.0, 0.5, 0.0, 0.0))},
      120);
  enc.commit();
  EXPECT_EQ(s2.base_seq, s1.seq);
  ASSERT_EQ(s2.entries.size(), 1u) << "b did not move and is not due a heartbeat";
  const cvc::pose_entry &e = s2.entries[0];
  EXPECT_TRUE(e.pid.empty()) << "named once";
  EXPECT_EQ(e.slot, s1.entries[0].slot);
  EXPECT_EQ(e.x, 67);
  EXPECT_EQ(e.y, 0);
  EXPECT_EQ(e.vx, 0);
  EXPECT_EQ(e.qw, 0);
  out.clear();
  ASSERT_TRUE(dec.decode(s2, out));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].pid, "a");
  EXPECT_NEAR(out[0].state.position.x, 1.067, 1e-9);
  EXPECT_NEAR(out[0].state.linear_velocity.x, 4.0, 1e-9);
}

// A participant standing still is sent an empty entry once per heartbeat: still there, at this
// tick.
TEST(PoseStream, AStillParticipantGetsAHeartbeat) {
  pose_stream_encoder enc;
  const body_state still = walking(0.0, 0.0, 0.0, 0.0);
  enc.encode(10, {subject("s", still)}, 120);
  enc.commit();
  EXPECT_TRUE(enc.encode(100, {subject("s", still)}, 120).entries.empty());
  const cvc::pose_snapshot hb = enc.encode(130, {subject("s", still)}, 120);
  ASSERT_EQ(hb.entries.size(), 1u);
  EXPECT_TRUE(hb.entries[0].pid.empty());
  EXPECT_EQ(hb.entries[0].x, 0);
  EXPECT_EQ(enc.last_sent_tick("s"), 10u) << "not committed yet";
  enc.commit();
  EXPECT_EQ(enc.last_sent_tick("s"), 130u);
}

// A snapshot that was never written changes nothing: the next one carries its difference too.
TEST(PoseStream, AnUnwrittenSnapshotIsCarriedByTheNext) {
  pose_stream_encoder enc;
  pose_stream_decoder dec;
  std::vector<cvc::ariadne::pose_sample> out;
  ASSERT_TRUE(dec.decode(enc.encode(0, {subject("a", walking(0.0, 0.0, 1.0, 0.0))}, 120), out));
  enc.commit();
  const cvc::pose_snapshot lost = enc.encode(2, {subject("a", walking(0.5, 0.0, 1.0, 0.0))}, 120);
  const cvc::pose_snapshot next = enc.encode(4, {subject("a", walking(0.8, 0.0, 1.0, 0.0))}, 120);
  EXPECT_EQ(next.seq, lost.seq) << "the same place in the stream";
  ASSERT_EQ(next.entries.size(), 1u);
  EXPECT_EQ(next.entries[0].x, 800);
  enc.commit();
  out.clear();
  ASSERT_TRUE(dec.decode(next, out));
  EXPECT_NEAR(out[0].state.position.x, 0.8, 1e-9);
}

// One that leaves is sent as gone; a snapshot out of step is refused until the stream restarts.
TEST(PoseStream, GoneSlotsAndOutOfStepSnapshots) {
  pose_stream_encoder enc;
  pose_stream_decoder dec;
  std::vector<cvc::ariadne::pose_sample> out;
  const cvc::pose_snapshot s1 =
      enc.encode(1, {subject("a", walking(1, 0, 0, 0)), subject("b", walking(2, 0, 0, 0))}, 120);
  enc.commit();
  ASSERT_TRUE(dec.decode(s1, out));
  const cvc::pose_snapshot s2 = enc.encode(3, {subject("a", walking(1, 0, 0, 0))}, 120);
  enc.commit();
  ASSERT_EQ(s2.gone.size(), 1u);
  EXPECT_EQ(s2.gone[0], s1.entries[1].slot);
  ASSERT_TRUE(dec.decode(s2, out));
  EXPECT_EQ(enc.last_sent_tick("b"), 0u);

  // A snapshot the client never got (s3), then the one after it: out of step.
  enc.encode(5, {subject("a", walking(1.2, 0, 0, 0))}, 120);
  enc.commit();
  const cvc::pose_snapshot s4 = enc.encode(7, {subject("a", walking(1.4, 0, 0, 0))}, 120);
  enc.commit();
  out.clear();
  EXPECT_FALSE(dec.decode(s4, out));
  EXPECT_TRUE(out.empty());
  EXPECT_EQ(dec.gaps(), 1u);
  // A restart names everyone again, absolute, and puts the client back in step.
  enc.reset();
  const cvc::pose_snapshot s5 = enc.encode(9, {subject("a", walking(1.6, 0, 0, 0))}, 120);
  enc.commit();
  EXPECT_EQ(s5.base_seq, 0u);
  EXPECT_EQ(s5.entries[0].pid, "a");
  ASSERT_TRUE(dec.decode(s5, out));
  EXPECT_NEAR(out[0].state.position.x, 1.6, 1e-9);
}

// A participant not due this step stays known (no gone), and gets no entry.
TEST(PoseStream, ANotDueParticipantStaysKnown) {
  pose_stream_encoder enc;
  enc.encode(1, {subject("a", walking(1, 0, 0, 0)), subject("far", walking(30, 0, 1, 0))}, 120);
  enc.commit();
  const cvc::pose_snapshot s = enc.encode(
      3, {subject("a", walking(1, 0, 0, 0)), subject("far", walking(31, 0, 1, 0), false)}, 120);
  EXPECT_TRUE(s.entries.empty());
  EXPECT_TRUE(s.gone.empty());
}

TEST(PoseStream, TheRateFollowsTheRoundTrip) {
  using cvc::ariadne::conference::pose_send_interval_steps;
  EXPECT_EQ(pose_send_interval_steps(-1.0), 2u) << "not measured yet: 30 Hz";
  EXPECT_EQ(pose_send_interval_steps(5.0), 1u) << "a LAN: every step, 60 Hz";
  EXPECT_EQ(pose_send_interval_steps(41.0), 1u) << "prod today";
  EXPECT_EQ(pose_send_interval_steps(120.0), 2u);
  EXPECT_EQ(pose_send_interval_steps(250.0), 3u);
  // Near a threshold the current rate holds.
  EXPECT_EQ(pose_send_interval_steps(84.0, 1), 1u);
  EXPECT_EQ(pose_send_interval_steps(84.0, 2), 2u);
  EXPECT_EQ(pose_send_interval_steps(75.0, 2), 2u);
  EXPECT_EQ(pose_send_interval_steps(70.0, 2), 1u);
  EXPECT_EQ(pose_send_interval_steps(170.0, 2), 2u);
  EXPECT_EQ(pose_send_interval_steps(180.0, 2), 3u);
  EXPECT_EQ(pose_send_interval_steps(150.0, 3), 3u);
  EXPECT_EQ(pose_send_interval_steps(140.0, 3), 2u);
}

namespace {
// A transport that records the pose snapshots written to its connected peers.
class capture_transport : public cvc::state_transport {
public:
  std::set<std::string> connected;
  std::vector<std::pair<std::string, cvc::pose_snapshot>> sent;
  void register_shard(cvc::state_cluster_shard *) override {}
  void unregister_shard(cvc::state_cluster_shard *) override {}
  publish_stats publish(const cvc::state_mutation &) override { return {}; }
  std::size_t pump_shard(cvc::state_cluster_shard &) override { return 0; }
  std::size_t pump_all() override { return 0; }
  void flush() override {}
  std::size_t push_pose_snapshot_to(const std::string &id, const cvc::pose_snapshot &s) override {
    if (!connected.count(id))
      return 0;
    sent.emplace_back(id, s);
    return 1;
  }
};
} // namespace

// Each viewer gets its snapshots at its own rate, and its client rebuilds every participant's
// pose from them; a viewer the relay cannot reach yet gets its stream from the start once it can.
TEST(PoseStream, TheRelaySendsEachViewerItsStreamAtItsRate) {
  using cvc::ariadne::holon;
  using cvc::ariadne::holon_role;
  holon server(holon_role::server);
  server.add_participant("fast", false, walking(0, 0, 0, 0));
  server.add_participant("slow", false, walking(2, 0, 0, 0));
  server.add_participant("late", false, walking(4, 0, 0, 0));
  server.set_intent("fast", cvc::ariadne::avatar_intent{vec3{3.0, 0.0, 0.0}});
  capture_transport tp;
  tp.connected = {"fast", "slow"}; // "late" has not connected yet
  cvc::ariadne::conference::room_pose_relay relay;
  relay.add_transport(&tp);
  relay.set_rtt("fast", 20.0);
  relay.set_rtt("slow", 250.0);
  const double dt = server.clock().fixed_dt();
  std::map<std::string, pose_stream_decoder> clients;
  std::map<std::string, int> got;
  for (std::uint64_t step = 0; step < 24; ++step) {
    if (step == 12)
      tp.connected.insert("late");
    server.advance(2 * dt);
    tp.sent.clear();
    relay.step(server, step, nullptr);
    for (const auto &[viewer, snap] : tp.sent) {
      ++got[viewer];
      std::vector<cvc::ariadne::pose_sample> out;
      ASSERT_TRUE(clients[viewer].decode(snap, out)) << viewer << " step " << step;
      // Whatever it is sent matches the server to the millimetre.
      for (const auto &s : out)
        EXPECT_LE(dist(s.state.position, server.state(s.pid).position), 0.001)
            << viewer << " sees " << s.pid << " at step " << step;
      if (viewer == "fast")
        EXPECT_NE(find(out, "fast"), nullptr) << "its own moving avatar, every snapshot";
    }
  }
  EXPECT_EQ(relay.interval_steps("fast"), 1u);
  EXPECT_EQ(relay.interval_steps("slow"), 3u);
  EXPECT_EQ(got["fast"], 24) << "60 Hz: every step (someone is always moving)";
  EXPECT_EQ(got["slow"], 8) << "20 Hz";
  EXPECT_GT(got["late"], 0) << "from when it connected";
  const auto st = relay.take_stats();
  EXPECT_GT(st.unsent, 0u) << "late was due before it connected";
}

// A client applies its snapshots through apply_pose_snapshot: its own avatar's sample and its
// peers' reach the holon; a participant it has not registered yet is skipped.
TEST(PoseStream, AClientAppliesItsSnapshotsToItsHolon) {
  using cvc::ariadne::holon;
  using cvc::ariadne::holon_role;
  holon client(holon_role::client);
  client.add_participant("me", true, walking(0, 0, 0, 0));
  client.add_participant("peer", false, walking(5, 0, 0, 0), /*pose_known=*/false);
  pose_stream_encoder enc;
  pose_stream_decoder dec;
  const cvc::pose_snapshot s =
      enc.encode(50,
                 {subject("me", walking(0.1, 0, 1, 0)), subject("peer", walking(5.2, 0, 1, 0)),
                  subject("stranger", walking(9, 9, 0, 0))},
                 120);
  EXPECT_EQ(cvc::ariadne::apply_pose_snapshot(client, dec, s), 2u);
  EXPECT_TRUE(client.pose_known("peer"));
  EXPECT_NEAR(client.state("peer").position.x, 5.2, 1e-9);
}
