/*
  Copyright 2026 The University of Texas at Austin
  Tests for the distributed_state_session convenience API.
*/

#include <chrono>
#include <cstdint>
#include <cstring>
#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_codec.h>
#include <cvc/core/app.h>
#include <cvc/state/distributed_state_session.h>
#include <cvc/state/state.h>
#include <cvc/state/state_blob_store.h>
#include <cvc/state/state_compression_registry.h>
#include <cvc/state/state_data_hydrator.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

using namespace cvc;

// ---- basic lifecycle ----

TEST(DistributedStateSession, JoinAndStop) {
  app a;
  distributed_state_config cfg;
  cfg.cluster_id = "test_cluster";
  cfg.node_id = "node1";
  cfg.pump_interval_ms = 0; // no pump thread

  auto session = distributed_state_session::join(a, cfg);
  ASSERT_NE(session, nullptr);
  EXPECT_TRUE(session->is_running());
  EXPECT_EQ(session->cluster_id(), "test_cluster");
  EXPECT_EQ(session->node_id(), "node1");

  session->stop();
  EXPECT_FALSE(session->is_running());
}

TEST(DistributedStateSession, DestructorCallsStop) {
  app a;
  distributed_state_config cfg;
  cfg.cluster_id = "c";
  cfg.node_id = "n";
  cfg.pump_interval_ms = 0;

  { auto session = distributed_state_session::join(a, cfg); }
  // If destructor didn't call stop(), this would hang or crash.
}

TEST(DistributedStateSession, DoubleStopIsSafe) {
  app a;
  distributed_state_config cfg;
  cfg.cluster_id = "c";
  cfg.node_id = "n";
  cfg.pump_interval_ms = 0;

  auto session = distributed_state_session::join(a, cfg);
  session->stop();
  session->stop(); // no-op
  EXPECT_FALSE(session->is_running());
}

// ---- component access ----

TEST(DistributedStateSession, ComponentAccessors) {
  app a;
  distributed_state_config cfg;
  cfg.cluster_id = "cluster1";
  cfg.node_id = "nodeA";
  cfg.pump_interval_ms = 0;

  auto session = distributed_state_session::join(a, cfg);
  EXPECT_EQ(&session->shard().codecs(), &session->shard().codecs());
  EXPECT_EQ(session->shard().cluster_id(), "cluster1");
  EXPECT_EQ(session->shard().local_node_id(), "nodeA");
  session->stop();
}

// ---- two-node inproc replication ----

TEST(DistributedStateSession, TwoNodeInprocReplication) {
  app a1, a2;

  // For inproc replication, we need a shared transport.
  state_transport_inproc shared_transport;

  state_cluster_shard s1(a1, "cluster", "node1");
  state_cluster_shard s2(a2, "cluster", "node2");

  s1.attach();
  s2.attach();

  shared_transport.register_shard(&s1);
  shared_transport.register_shard(&s2);

  // First value() on a new child is consumed by creation, so set twice.
  state::instance(a1)("test.value").value(std::string("initial"));
  state::instance(a1)("test.value").value(std::string("hello_from_node1"));

  // Pump to propagate.
  shared_transport.pump_all();

  // Read on node 2.
  EXPECT_EQ(state::instance(a2)("test.value").value(), "hello_from_node1");

  shared_transport.unregister_shard(&s1);
  shared_transport.unregister_shard(&s2);
  s1.detach();
  s2.detach();
}

namespace {
// A stand-in for a stream's format_desc — the kind of struct that lives in a node's data()
// (boost::any) and must cross a process boundary for a remote consumer to discover a stream.
struct TestDesc {
  int w = 0, h = 0;
  double fps = 0.0;
};
} // namespace

// data() replication (Phase 10, PR1): a node's data() payload, encoded via the shard codec
// registry, replicates to a peer — not just value() + child structure. This is the control-plane
// prerequisite for cross-node streaming (the /streams/<id> descriptor is stored in data()).
TEST(DistributedStateSession, TwoNodeInprocDataReplication) {
  app a1, a2;
  a1.registerDataType<TestDesc>("test_stream_desc");
  a2.registerDataType<TestDesc>("test_stream_desc");

  state_transport_inproc shared_transport;
  state_cluster_shard s1(a1, "cluster", "node1");
  state_cluster_shard s2(a2, "cluster", "node2");

  // A trivial POD codec for TestDesc on BOTH shards' registries, keyed by the data type name.
  const auto enc = [](const boost::any &v) -> std::vector<unsigned char> {
    const TestDesc d = boost::any_cast<TestDesc>(v);
    std::vector<unsigned char> b(sizeof(TestDesc));
    std::memcpy(b.data(), &d, sizeof(TestDesc));
    return b;
  };
  const auto dec = [](const std::vector<unsigned char> &b) -> boost::any {
    TestDesc d{};
    if (b.size() == sizeof(TestDesc))
      std::memcpy(&d, b.data(), sizeof(TestDesc));
    return boost::any(d);
  };
  s1.codecs().register_codec("test_stream_desc", enc, dec);
  s2.codecs().register_codec("test_stream_desc", enc, dec);

  s1.attach();
  s2.attach();
  shared_transport.register_shard(&s1);
  shared_transport.register_shard(&s2);

  // Set a data() payload on node 1 (the descriptor pattern). Create the node first so its
  // dataChanged is connected before we write.
  state &node1 = state::instance(a1)("streams.cam0");
  node1.value(std::string("probe1"));
  node1.value(std::string("probe2"));
  node1.data(boost::any(TestDesc{1920, 1080, 59.94}));
  node1.data(boost::any(TestDesc{1920, 1080, 59.94})); // set twice (creation-consume guard)

  shared_transport.pump_all();

  // Diagnostics: did the node + its value replicate at all? (isolates attach vs data path)
  EXPECT_EQ(state::instance(a2)("streams.cam0").value(), "probe2") << "value did not replicate";

  // Node 2 must now see the SAME descriptor in data() — not an empty any, not just the node.
  const boost::any got = state::instance(a2)("streams.cam0").data();
  ASSERT_FALSE(got.empty());
  const TestDesc d = boost::any_cast<TestDesc>(got);
  EXPECT_EQ(d.w, 1920);
  EXPECT_EQ(d.h, 1080);
  EXPECT_DOUBLE_EQ(d.fps, 59.94);

  // A node whose data() type has NO codec must not crash the path — value()/structure still flow.
  state::instance(a1)("plain.value").value(std::string("v1"));
  state::instance(a1)("plain.value").value(std::string("v2"));
  shared_transport.pump_all();
  EXPECT_EQ(state::instance(a2)("plain.value").value(), "v2");

  shared_transport.unregister_shard(&s1);
  shared_transport.unregister_shard(&s2);
  s1.detach();
  s2.detach();
}

namespace {
// A larger descriptor: a small header plus a variable-length lookup table. Encoded, it exceeds
// the inline threshold, so it exercises the blob-offload + hydrator path rather than inline bytes.
struct TestLut {
  int w = 0, h = 0;
  std::vector<unsigned char> lut;
};

std::vector<unsigned char> encode_test_lut(const boost::any &v) {
  const TestLut d = boost::any_cast<TestLut>(v);
  std::vector<unsigned char> b;
  const auto push_u32 = [&b](std::uint32_t x) {
    for (int i = 0; i < 4; ++i)
      b.push_back(static_cast<unsigned char>((x >> (i * 8)) & 0xff));
  };
  push_u32(static_cast<std::uint32_t>(d.w));
  push_u32(static_cast<std::uint32_t>(d.h));
  push_u32(static_cast<std::uint32_t>(d.lut.size()));
  b.insert(b.end(), d.lut.begin(), d.lut.end());
  return b;
}

boost::any decode_test_lut(const std::vector<unsigned char> &b) {
  TestLut d{};
  if (b.size() < 12)
    return boost::any(d);
  const auto rd_u32 = [&b](std::size_t off) -> std::uint32_t {
    std::uint32_t x = 0;
    for (int i = 0; i < 4; ++i)
      x |= static_cast<std::uint32_t>(b[off + i]) << (i * 8);
    return x;
  };
  d.w = static_cast<int>(rd_u32(0));
  d.h = static_cast<int>(rd_u32(4));
  const std::uint32_t n = rd_u32(8);
  if (b.size() >= 12u + n)
    d.lut.assign(b.begin() + 12, b.begin() + 12 + n);
  return boost::any(d);
}
} // namespace

// data() blob offload + hydration (Phase 10, PR2): a data() payload larger than the shard's inline
// threshold is offloaded to the blob store as a chunked-blob manifest, replicates as a blob_ref,
// and is reassembled + decoded + installed on the peer by the state_data_hydrator — the receive
// side of the large-payload path (stream frame descriptors, LUTs, volume bricks).
TEST(DistributedStateSession, TwoNodeInprocDataBlobReplication) {
  app a1, a2;
  a1.registerDataType<TestLut>("test_lut");
  a2.registerDataType<TestLut>("test_lut");

  // Isolated per-node blob stores. The transport serves chunk fetches from node 1's store (the
  // producer), modelling a consumer that pulls blob chunks it does not yet hold.
  memory_state_blob_store store1, store2;
  state_transport_inproc shared_transport;
  shared_transport.set_blob_store(&store1);

  state_cluster_shard s1(a1, "cluster", "node1");
  state_cluster_shard s2(a2, "cluster", "node2");
  s1.set_blob_store(&store1);
  s2.set_blob_store(&store2);
  s1.set_max_inline_payload_bytes(256);
  s2.set_max_inline_payload_bytes(256);

  s1.codecs().register_codec("test_lut", encode_test_lut, decode_test_lut);
  s2.codecs().register_codec("test_lut", encode_test_lut, decode_test_lut);

  // Node 2's hydrator resolves inbound blob-ref data() payloads from its own store, fetching any
  // missing chunks over the transport.
  state_data_hydrator hydrator2(store2, s2.codecs(), &state_compression_registry::shared());
  hydrator2.set_transport(&shared_transport);
  s2.set_data_hydrator(&hydrator2);

  s1.attach();
  s2.attach();
  shared_transport.register_shard(&s1);
  shared_transport.register_shard(&s2);

  // Build a payload that exceeds the 256-byte inline threshold once codec-encoded (12 + 4096).
  TestLut desc;
  desc.w = 1920;
  desc.h = 1080;
  desc.lut.resize(4096);
  for (std::size_t i = 0; i < desc.lut.size(); ++i)
    desc.lut[i] = static_cast<unsigned char>(i % 251);

  state &node1 = state::instance(a1)("streams.cam0.lut");
  node1.value(std::string("probe1"));
  node1.value(std::string("probe2"));
  node1.data(boost::any(desc));
  node1.data(boost::any(desc)); // set twice (creation-consume guard)

  shared_transport.pump_all();

  // The payload must have been offloaded to node 1's blob store (chunk manifest + chunk blobs),
  // not sent inline.
  EXPECT_GT(store1.size(), 0u) << "large data() payload was not offloaded to the blob store";

  // Node 2 took the blob path (hydrator), not an inline decode, and the blob hydrated to ready.
  EXPECT_EQ(hydrator2.status("streams.cam0.lut"), state_data_hydrator::hydration_status::ready)
      << "blob payload did not hydrate on the peer";
  EXPECT_GT(hydrator2.total_hydrated(), 0u);

  // Node 2 must see the SAME descriptor in data(), byte-identical LUT included.
  const boost::any got = state::instance(a2)("streams.cam0.lut").data();
  ASSERT_FALSE(got.empty());
  const TestLut d = boost::any_cast<TestLut>(got);
  EXPECT_EQ(d.w, 1920);
  EXPECT_EQ(d.h, 1080);
  ASSERT_EQ(d.lut.size(), desc.lut.size());
  EXPECT_EQ(d.lut, desc.lut) << "hydrated LUT must be byte-identical to the source payload";

  shared_transport.unregister_shard(&s1);
  shared_transport.unregister_shard(&s2);
  s1.detach();
  s2.detach();
}

// Node metadata replication (Phase 10, PR3): a node's comment / hidden / readOnly flags replicate
// to a peer as their own mutations, not just value() + data() + child structure. apply_remote
// already mirrors these ops; this exercises the matching local emit side added in PR3.
TEST(DistributedStateSession, TwoNodeInprocMetadataReplication) {
  app a1, a2;

  state_transport_inproc shared_transport;
  state_cluster_shard s1(a1, "cluster", "node1");
  state_cluster_shard s2(a2, "cluster", "node2");
  s1.attach();
  s2.attach();
  shared_transport.register_shard(&s1);
  shared_transport.register_shard(&s2);

  // Establish the node first (the first value on a new node is consumed by creation) so the later
  // metadata changes fire on an already-replicated node.
  state &node1 = state::instance(a1)("ui.panel");
  node1.value(std::string("v1"));
  node1.value(std::string("v2"));
  shared_transport.pump_all();
  ASSERT_EQ(state::instance(a2)("ui.panel").value(), "v2") << "node did not replicate";

  // Mutate each piece of metadata; each replicates via its own set_comment/set_hidden/
  // set_read_only mutation.
  node1.comment(std::string("a helpful tooltip"));
  node1.hidden(true);
  node1.readOnly(true);
  shared_transport.pump_all();

  EXPECT_EQ(state::instance(a2)("ui.panel").comment(), "a helpful tooltip");
  EXPECT_TRUE(state::instance(a2)("ui.panel").hidden());
  EXPECT_TRUE(state::instance(a2)("ui.panel").readOnly());

  // Properties replicate as set_property, and so does removing one.
  node1.property("min", "0").property("max", "10");
  shared_transport.pump_all();
  EXPECT_EQ(state::instance(a2)("ui.panel").property("min"), "0");
  EXPECT_EQ(state::instance(a2)("ui.panel").property("max"), "10");
  node1.property("min", "");
  shared_transport.pump_all();
  EXPECT_FALSE(state::instance(a2)("ui.panel").has_property("min"));
  EXPECT_EQ(state::instance(a2)("ui.panel").property("max"), "10");

  // Clearing the flags replicates too (proves the boolean round-trips both ways).
  node1.hidden(false);
  node1.readOnly(false);
  shared_transport.pump_all();
  EXPECT_FALSE(state::instance(a2)("ui.panel").hidden());
  EXPECT_FALSE(state::instance(a2)("ui.panel").readOnly());

  shared_transport.unregister_shard(&s1);
  shared_transport.unregister_shard(&s2);
  s1.detach();
  s2.detach();
}

// Snapshot carries data() + metadata (Phase 10, PR4): a late joiner reconstructs each node's
// value, metadata (comment/hidden/readOnly) AND data() from the snapshot, not just value(). Large
// data() offloads to a blob in the snapshot exactly like the live path and hydrates on the joiner.
// Exercises shard::snapshot (producer) + shard::ingest_snapshot_entry (the dedup-safe join apply)
// directly — the two ends of the join path, without a transport round-trip.
TEST(DistributedStateSession, TwoNodeSnapshotDataMetadataReassembly) {
  app a1, a2;
  a1.registerDataType<TestLut>("test_lut");
  a2.registerDataType<TestLut>("test_lut");

  memory_state_blob_store store; // shared content-addressed namespace (producer + joiner hydrator)
  state_cluster_shard s1(a1, "cluster", "node1");
  state_cluster_shard s2(a2, "cluster", "node2");
  s1.set_blob_store(&store);
  s2.set_blob_store(&store);
  s1.set_max_inline_payload_bytes(256);
  s2.set_max_inline_payload_bytes(256);
  s1.codecs().register_codec("test_lut", encode_test_lut, decode_test_lut);
  s2.codecs().register_codec("test_lut", encode_test_lut, decode_test_lut);

  // The joiner hydrates blob-ref data() payloads out of the shared store.
  state_data_hydrator hydrator2(store, s2.codecs(), &state_compression_registry::shared());
  s2.set_data_hydrator(&hydrator2);

  s1.attach();
  s2.attach();

  // Build node 1's tree: a camera node (value + metadata + a LARGE data() that offloads to a blob)
  // and a child meta node (a SMALL data() that stays inline).
  TestLut big;
  big.w = 1920;
  big.h = 1080;
  big.lut.resize(4096);
  for (std::size_t i = 0; i < big.lut.size(); ++i)
    big.lut[i] = static_cast<unsigned char>(i % 251);
  state &cam = state::instance(a1)("streams.cam0");
  cam.value(std::string("v1"));
  cam.value(std::string("live"));
  cam.comment(std::string("primary camera"));
  cam.hidden(true);
  cam.property("unit", "fps").property("max", "120");
  cam.data(boost::any(big));

  // NB: not named `small` — the Windows SDK defines `#define small char`, which turns a
  // `TestLut small;` declaration into `TestLut char;` under MSVC (see windows-macro collisions).
  TestLut tiny;
  tiny.w = 2;
  tiny.h = 2;
  tiny.lut = {1, 2, 3, 4};
  state &meta = state::instance(a1)("streams.cam0.meta");
  meta.value(std::string("m1"));
  meta.value(std::string("m2"));
  meta.data(boost::any(tiny));

  // Snapshot node 1's tree and reassemble each entry onto node 2 (the join path, sans transport).
  const std::vector<state_cluster_shard::snapshot_entry> snap = s1.snapshot(std::string());
  ASSERT_FALSE(snap.empty());
  for (const auto &e : snap)
    s2.ingest_snapshot_entry(e);

  // The large payload must have been offloaded to the blob store during snapshot.
  EXPECT_GT(store.size(), 0u) << "large data() was not offloaded to a blob in the snapshot";

  // Node 2 mirrors value + metadata.
  EXPECT_EQ(state::instance(a2)("streams.cam0").value(), "live");
  EXPECT_EQ(state::instance(a2)("streams.cam0").comment(), "primary camera");
  EXPECT_TRUE(state::instance(a2)("streams.cam0").hidden());
  EXPECT_EQ(state::instance(a2)("streams.cam0").property("unit"), "fps");
  EXPECT_EQ(state::instance(a2)("streams.cam0").property("max"), "120");

  // Node 2 mirrors the large (blob-hydrated) data(), byte-identical.
  const boost::any got_big = state::instance(a2)("streams.cam0").data();
  ASSERT_FALSE(got_big.empty()) << "blob data() did not reconstruct on the joiner";
  const TestLut rb = boost::any_cast<TestLut>(got_big);
  EXPECT_EQ(rb.w, 1920);
  EXPECT_EQ(rb.h, 1080);
  EXPECT_EQ(rb.lut, big.lut);

  // Node 2 mirrors the small (inline) data().
  const boost::any got_small = state::instance(a2)("streams.cam0.meta").data();
  ASSERT_FALSE(got_small.empty()) << "inline data() did not reconstruct on the joiner";
  const TestLut rs = boost::any_cast<TestLut>(got_small);
  EXPECT_EQ(rs.lut, tiny.lut);

  s1.detach();
  s2.detach();
}

// Cross-node stream DISCOVERY (Phase 10, PR5 — the control-plane payoff): a real stream opened on
// one node publishes its format_desc into the /streams/<id> descriptor node's data(); with the
// stream codec registered on both shards, that descriptor replicates so a remote node discovers the
// stream (lifecycle + full format) without any data-plane transport.
TEST(DistributedStateSession, TwoNodeStreamDescriptorDiscovery) {
  namespace cs = cvc::ariadne::stream;
  app a1, a2;

  state_transport_inproc shared_transport;
  state_cluster_shard s1(a1, "cluster", "node1");
  state_cluster_shard s2(a2, "cluster", "node2");
  cs::register_stream_codecs(s1.codecs());
  cs::register_stream_codecs(s2.codecs());
  s1.attach();
  s2.attach();
  shared_transport.register_shard(&s1);
  shared_transport.register_shard(&s2);

  // Open a real stream on node 1 — stream::open writes the /streams/cam0 descriptor (format_desc in
  // data(), "live" in value()) and registers the stable descriptor type name on a1.
  cs::stream_params p;
  p.id = "cam0";
  p.format.kind = cs::frame_kind::video_raw;
  p.format.codec = "rgba8";
  p.format.w = 1920;
  p.format.h = 1080;
  p.format.stride = 1920 * 4;
  p.format.bytes = static_cast<std::size_t>(1920) * 1080 * 4;
  std::unique_ptr<cs::stream> strm = cs::stream::open(a1, p);
  ASSERT_NE(strm, nullptr);

  shared_transport.pump_all();

  // Node 2 discovers the stream: lifecycle in value(), the full format_desc in data().
  EXPECT_EQ(state::instance(a2)("streams.cam0").value(), "live")
      << "stream lifecycle did not replicate";
  const boost::any got = state::instance(a2)("streams.cam0").data();
  ASSERT_FALSE(got.empty()) << "stream descriptor did not replicate to the peer";
  const cs::format_desc fd = boost::any_cast<cs::format_desc>(got);
  EXPECT_EQ(static_cast<int>(fd.kind), static_cast<int>(cs::frame_kind::video_raw));
  EXPECT_EQ(fd.codec, "rgba8");
  EXPECT_EQ(fd.w, 1920);
  EXPECT_EQ(fd.h, 1080);
  EXPECT_EQ(fd.stride, 1920 * 4);
  EXPECT_EQ(fd.bytes, static_cast<std::size_t>(1920) * 1080 * 4);

  strm.reset(); // close the stream
  shared_transport.unregister_shard(&s1);
  shared_transport.unregister_shard(&s2);
  s1.detach();
  s2.detach();
}

// ---- pump thread ----

TEST(DistributedStateSession, PumpThreadRuns) {
  app a;
  distributed_state_config cfg;
  cfg.cluster_id = "c";
  cfg.node_id = "n";
  cfg.pump_interval_ms = 5;

  auto session = distributed_state_session::join(a, cfg);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_GT(session->pump_cycles(), 0u);
  session->stop();
}

// ---- sync_path ----

TEST(DistributedStateSession, SyncPath) {
  app a;
  distributed_state_config cfg;
  cfg.cluster_id = "c";
  cfg.node_id = "n";
  cfg.pump_interval_ms = 0;
  cfg.enforce_interest = true;

  auto session = distributed_state_session::join(a, cfg);
  // Before adding interest, path should not be of interest.
  EXPECT_FALSE(session->shard().path_is_of_interest("new.path"));

  session->sync_path("new.path", sync_mode::read_write);
  EXPECT_TRUE(session->shard().path_is_of_interest("new.path"));
  session->stop();
}

// ---- config mounts ----

TEST(DistributedStateSession, MountsApplied) {
  app a;
  distributed_state_config cfg;
  cfg.cluster_id = "c";
  cfg.node_id = "n";
  cfg.pump_interval_ms = 0;
  cfg.enforce_interest = true;
  cfg.mounts = {{"scene", sync_mode::read_write}, {"data", sync_mode::read_only}};

  auto session = distributed_state_session::join(a, cfg);
  EXPECT_TRUE(session->shard().path_is_of_interest("scene"));
  EXPECT_TRUE(session->shard().path_is_of_interest("scene.geometry"));
  EXPECT_TRUE(session->shard().path_is_of_interest("data"));
  EXPECT_TRUE(session->shard().path_is_of_interest("data.volume"));
  session->stop();
}

// ---- delegation ----

TEST(DistributedStateSession, DelegateAndUndelegate) {
  app a;
  distributed_state_config cfg;
  cfg.cluster_id = "local";
  cfg.node_id = "n";
  cfg.pump_interval_ms = 0;

  auto session = distributed_state_session::join(a, cfg);

  delegation_target tgt;
  tgt.cluster_id = "remote";
  tgt.endpoint = "remote:50051";
  session->delegate("remote.data", tgt);

  // The delegation should be visible in the shard's delegation manager.
  auto decision = session->shard().route_path("remote.data.volume");
  EXPECT_EQ(decision.cluster_id, "remote");

  session->undelegate("remote.data");
  session->stop();
}

// ---- policy configuration ----

TEST(DistributedStateSession, PolicyFlags) {
  app a;
  distributed_state_config cfg;
  cfg.cluster_id = "c";
  cfg.node_id = "n";
  cfg.pump_interval_ms = 0;
  cfg.enforce_authority = true;
  cfg.enforce_write_policy = true;
  cfg.enforce_delegation = true;
  cfg.resolve_conflicts = true;
  cfg.enforce_interest = true;

  auto session = distributed_state_session::join(a, cfg);
  EXPECT_TRUE(session->shard().enforce_authority());
  EXPECT_TRUE(session->shard().enforce_write_policy());
  EXPECT_TRUE(session->shard().enforce_delegation());
  EXPECT_TRUE(session->shard().resolve_conflicts());
  EXPECT_TRUE(session->shard().enforce_interest());
  session->stop();
}

// ---- admin facade ----

TEST(DistributedStateSession, AdminAttached) {
  app a;
  distributed_state_config cfg;
  cfg.cluster_id = "c";
  cfg.node_id = "n";
  cfg.pump_interval_ms = 0;

  auto session = distributed_state_session::join(a, cfg);
  // Admin should be accessible; verify it doesn't crash.
  auto &admin = session->admin();
  (void)admin;
  session->stop();
}

// ---- IPC transport selection ----

#ifndef _WIN32
TEST(DistributedStateSession, IpcTransportCreation) {
  app a;
  distributed_state_config cfg;
  cfg.cluster_id = "c";
  cfg.node_id = "n";
  cfg.transport = transport_kind::ipc;
  cfg.listen_address = "/tmp/cvc_test_session_" + std::to_string(getpid()) + ".sock";
  cfg.pump_interval_ms = 0;

  auto session = distributed_state_session::join(a, cfg);
  EXPECT_TRUE(session->is_running());
  session->stop();

  // Clean up socket.
  ::unlink(cfg.listen_address.c_str());
}
#endif // !_WIN32
