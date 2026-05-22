/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/app.h>
#include <cvc/state_distributed_admin.h>
#include <cvc/state_message.h>
#include <cvc/state_message_bus.h>
#include <cvc/state_node_telemetry.h>
#include <cvc/state_telemetry_aggregator.h>
#include <gtest/gtest.h>
#include <thread>

using namespace cvc;

// =====================================================================
// state_telemetry_aggregator
// =====================================================================

TEST(TelemetryAggregatorTest, IngestAndSummarize) {
  state_telemetry_aggregator agg("alpha");

  telemetry_snapshot s1;
  s1.node_id = "node-1";
  s1.cluster_id = "alpha";
  s1.timestamp_ns = 1'000'000'000;
  s1.mutations_published = 100;
  s1.mutations_applied = 90;
  s1.bytes_sent = 5000;

  telemetry_snapshot s2;
  s2.node_id = "node-2";
  s2.cluster_id = "alpha";
  s2.timestamp_ns = 1'000'000'000;
  s2.mutations_published = 200;
  s2.mutations_applied = 180;
  s2.bytes_sent = 8000;

  agg.ingest(s1);
  agg.ingest(s2);

  EXPECT_EQ(agg.peer_count(), 2u);

  auto sum = agg.summarize();
  EXPECT_EQ(sum.node_count, 2u);
  EXPECT_EQ(sum.total_mutations_published, 300u);
  EXPECT_EQ(sum.total_mutations_applied, 270u);
  EXPECT_EQ(sum.total_bytes_sent, 13000u);
}

TEST(TelemetryAggregatorTest, StaleDetection) {
  // Use a very short stale threshold for testing.
  state_telemetry_aggregator agg("alpha", 50'000'000ULL); // 50 ms

  telemetry_snapshot s;
  s.node_id = "old-node";
  s.cluster_id = "alpha";
  // Set timestamp far in the past.
  s.timestamp_ns = 1;
  agg.ingest(s);

  EXPECT_EQ(agg.stale_count(), 1u);

  auto sum = agg.summarize();
  EXPECT_EQ(sum.stale_count, 1u);
}

TEST(TelemetryAggregatorTest, RemovePeer) {
  state_telemetry_aggregator agg("alpha");

  telemetry_snapshot s;
  s.node_id = "node-1";
  s.cluster_id = "alpha";
  s.timestamp_ns = 1'000'000'000;
  agg.ingest(s);

  EXPECT_EQ(agg.peer_count(), 1u);
  EXPECT_TRUE(agg.remove_peer("node-1"));
  EXPECT_EQ(agg.peer_count(), 0u);
  EXPECT_FALSE(agg.remove_peer("nonexistent"));
}

TEST(TelemetryAggregatorTest, Clear) {
  state_telemetry_aggregator agg("alpha");

  telemetry_snapshot s;
  s.node_id = "node-1";
  s.cluster_id = "alpha";
  s.timestamp_ns = 1'000'000'000;
  agg.ingest(s);

  agg.clear();
  EXPECT_EQ(agg.peer_count(), 0u);
}

TEST(TelemetryAggregatorTest, PeerSnapshots) {
  state_telemetry_aggregator agg("alpha");

  telemetry_snapshot s;
  s.node_id = "node-1";
  s.cluster_id = "alpha";
  s.timestamp_ns = 1'000'000'000;
  s.mutations_published = 42;
  agg.ingest(s);

  auto peers = agg.peer_snapshots();
  EXPECT_EQ(peers.size(), 1u);
  EXPECT_EQ(peers["node-1"].mutations_published, 42u);
}

TEST(TelemetryAggregatorTest, AttachBusAndReceiveMessage) {
  state_message_bus bus;
  state_telemetry_aggregator agg("alpha");
  EXPECT_TRUE(agg.attach_bus(bus));

  // Publish a telemetry message on the bus.
  telemetry_snapshot snap;
  snap.node_id = "node-1";
  snap.cluster_id = "alpha";
  snap.timestamp_ns = 1'000'000'000;
  snap.mutations_published = 77;

  std::string json = state_node_telemetry::serialize_json(snap);
  auto msg = state_message::make_text("__telemetry.alpha.node-1", json,
                                      state_node_telemetry::MIME_TELEMETRY);
  msg.origin_node_id = "node-1";
  msg.message_id = "tel-1";
  bus.admit(msg);

  EXPECT_EQ(agg.peer_count(), 1u);
  auto peers = agg.peer_snapshots();
  EXPECT_EQ(peers["node-1"].mutations_published, 77u);

  agg.detach_bus();
}

TEST(TelemetryAggregatorTest, ToText) {
  state_telemetry_aggregator agg("alpha");

  telemetry_snapshot s;
  s.node_id = "node-1";
  s.cluster_id = "alpha";
  s.timestamp_ns = 1'000'000'000;
  s.mutations_published = 100;
  agg.ingest(s);

  std::string text = agg.to_text();
  EXPECT_NE(text.find("[telemetry]"), std::string::npos);
  EXPECT_NE(text.find("cluster_id: alpha"), std::string::npos);
  EXPECT_NE(text.find("nodes: 1"), std::string::npos);
  EXPECT_NE(text.find("[peer node-1]"), std::string::npos);
}

TEST(TelemetryAggregatorTest, MaxLatencyAcrossNodes) {
  state_telemetry_aggregator agg("alpha");

  telemetry_snapshot s1;
  s1.node_id = "node-1";
  s1.cluster_id = "alpha";
  s1.timestamp_ns = 1'000'000'000;
  s1.enqueue_p99 = 5000;
  s1.delivery_p99 = 10000;

  telemetry_snapshot s2;
  s2.node_id = "node-2";
  s2.cluster_id = "alpha";
  s2.timestamp_ns = 1'000'000'000;
  s2.enqueue_p99 = 8000;
  s2.delivery_p99 = 3000;

  agg.ingest(s1);
  agg.ingest(s2);

  auto sum = agg.summarize();
  EXPECT_EQ(sum.max_enqueue_p99, 8000u);
  EXPECT_EQ(sum.max_delivery_p99, 10000u);
}

// =====================================================================
// Routing feedback
// =====================================================================

TEST(RoutingFeedbackTest, IsolateHighLatencyPeer) {
  state_telemetry_aggregator agg("alpha");

  telemetry_snapshot s;
  s.node_id = "slow-peer";
  s.cluster_id = "alpha";
  s.timestamp_ns = 1'000'000'000;
  s.delivery_p99 = 500'000'000; // 500 ms

  agg.ingest(s);

  routing_feedback_policy policy;
  policy.latency_p99_threshold_ns = 100'000'000; // 100 ms threshold

  auto result = agg.evaluate_routing_feedback(policy);
  EXPECT_EQ(result.isolate.size(), 1u);
  EXPECT_EQ(result.isolate[0], "slow-peer");
  EXPECT_EQ(result.release.size(), 0u);
}

TEST(RoutingFeedbackTest, ReleaseHealthyPeer) {
  state_telemetry_aggregator agg("alpha");

  telemetry_snapshot s;
  s.node_id = "fast-peer";
  s.cluster_id = "alpha";
  s.timestamp_ns = 1'000'000'000;
  s.delivery_p99 = 1000; // 1 µs

  agg.ingest(s);

  routing_feedback_policy policy;
  policy.latency_p99_threshold_ns = 100'000'000;

  auto result = agg.evaluate_routing_feedback(policy);
  EXPECT_EQ(result.isolate.size(), 0u);
  EXPECT_EQ(result.release.size(), 1u);
  EXPECT_EQ(result.release[0], "fast-peer");
}

TEST(RoutingFeedbackTest, IsolateOnDrops) {
  state_telemetry_aggregator agg("alpha");

  telemetry_snapshot s;
  s.node_id = "dropping-peer";
  s.cluster_id = "alpha";
  s.timestamp_ns = 1'000'000'000;
  s.outbox_drops = 500;

  agg.ingest(s);

  routing_feedback_policy policy;
  policy.outbox_drop_threshold = 100;

  auto result = agg.evaluate_routing_feedback(policy);
  EXPECT_EQ(result.isolate.size(), 1u);
  EXPECT_EQ(result.isolate[0], "dropping-peer");
}

TEST(RoutingFeedbackTest, DisabledPolicyReleasesAll) {
  state_telemetry_aggregator agg("alpha");

  telemetry_snapshot s;
  s.node_id = "any-peer";
  s.cluster_id = "alpha";
  s.timestamp_ns = 1'000'000'000;
  s.delivery_p99 = 999'999'999;
  s.outbox_drops = 999999;

  agg.ingest(s);

  routing_feedback_policy policy; // all thresholds 0 = disabled

  auto result = agg.evaluate_routing_feedback(policy);
  EXPECT_EQ(result.isolate.size(), 0u);
  EXPECT_EQ(result.release.size(), 1u);
}

// =====================================================================
// Integration: state_distributed_admin with telemetry
// =====================================================================

TEST(AdminTelemetryTest, ToTextIncludesTelemetry) {
  state_telemetry_aggregator agg("alpha");

  telemetry_snapshot s;
  s.node_id = "node-1";
  s.cluster_id = "alpha";
  s.timestamp_ns = 1'000'000'000;
  s.mutations_published = 42;
  agg.ingest(s);

  state_distributed_admin admin;
  admin.attach_telemetry(&agg);

  std::string text = admin.to_text();
  EXPECT_NE(text.find("[telemetry]"), std::string::npos);
  EXPECT_NE(text.find("mutations_published: 42"), std::string::npos);
}
