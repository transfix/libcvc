/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <chrono>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_message_bus.h>
#include <cvc/core/state_node_telemetry.h>
#include <cvc/core/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <thread>

using namespace cvc;

// =====================================================================
// ewma
// =====================================================================

TEST(EwmaTest, InitializesToFirstSample) {
  ewma e(1'000'000'000ULL); // 1 s half-life
  e.update(42.0, 1'000'000'000ULL);
  EXPECT_DOUBLE_EQ(e.value(), 42.0);
}

TEST(EwmaTest, DecaysOverTime) {
  ewma e(1'000'000'000ULL);
  e.update(100.0, 1'000'000'000ULL);
  // After one half-life, a new sample of 0 should move the EWMA
  // halfway: ~50.
  e.update(0.0, 2'000'000'000ULL);
  double v = e.value();
  EXPECT_GT(v, 45.0);
  EXPECT_LT(v, 55.0);
}

TEST(EwmaTest, ConvergesAfterManySamples) {
  ewma e(100'000'000ULL); // 100 ms half-life
  for (int i = 0; i < 100; ++i) {
    e.update(10.0, static_cast<std::uint64_t>(i + 1) * 100'000'000ULL);
  }
  EXPECT_NEAR(e.value(), 10.0, 0.1);
}

TEST(EwmaTest, ResetClearsState) {
  ewma e;
  e.update(100.0, 1'000'000'000ULL);
  e.reset();
  EXPECT_DOUBLE_EQ(e.value(), 0.0);
  e.update(5.0, 2'000'000'000ULL);
  EXPECT_DOUBLE_EQ(e.value(), 5.0);
}

// =====================================================================
// latency_histogram
// =====================================================================

TEST(LatencyHistogramTest, RecordAndCount) {
  latency_histogram h;
  h.record(500);   // < 1 µs bucket
  h.record(2000);  // ~ 2 µs bucket
  h.record(50000); // ~ 32 µs bucket
  EXPECT_EQ(h.count(), 3u);
}

TEST(LatencyHistogramTest, PercentilesMonotonic) {
  latency_histogram h;
  // Fill with a spread of values.
  for (int i = 0; i < 1000; ++i) {
    h.record(static_cast<std::uint64_t>(i) * 1000);
  }
  EXPECT_LE(h.p50(), h.p90());
  EXPECT_LE(h.p90(), h.p99());
}

TEST(LatencyHistogramTest, EmptyHistogramReturnsZero) {
  latency_histogram h;
  EXPECT_EQ(h.p50(), 0u);
  EXPECT_EQ(h.p99(), 0u);
  EXPECT_EQ(h.count(), 0u);
}

TEST(LatencyHistogramTest, SingleValueInFirstBucket) {
  latency_histogram h;
  h.record(500); // < 1024 → bucket 0
  EXPECT_EQ(h.count(), 1u);
  EXPECT_EQ(h.p50(), 1024u); // upper boundary of bucket 0
}

TEST(LatencyHistogramTest, ResetClearsAll) {
  latency_histogram h;
  for (int i = 0; i < 100; ++i)
    h.record(static_cast<std::uint64_t>(i) * 10000);
  EXPECT_GT(h.count(), 0u);
  h.reset();
  EXPECT_EQ(h.count(), 0u);
}

// =====================================================================
// telemetry_snapshot serialization
// =====================================================================

TEST(TelemetrySnapshotTest, JsonRoundTrip) {
  telemetry_snapshot orig;
  orig.node_id = "node1";
  orig.cluster_id = "alpha";
  orig.timestamp_ns = 123456789;
  orig.mutations_published = 100;
  orig.mutations_applied = 95;
  orig.mutations_duplicates = 3;
  orig.mutations_rejected = 2;
  orig.mutations_conflicts = 1;
  orig.messages_admitted = 50;
  orig.messages_dispatched = 48;
  orig.messages_dropped = 2;
  orig.bytes_sent = 10000;
  orig.bytes_received = 9500;
  orig.peer_count = 5;
  orig.slow_peer_count = 1;
  orig.mutation_publish_rate = 42.5;
  orig.enqueue_p99 = 8192;
  orig.delivery_p99 = 16384;

  std::string json = state_node_telemetry::serialize_json(orig);
  EXPECT_FALSE(json.empty());
  EXPECT_NE(json.find("node1"), std::string::npos);

  telemetry_snapshot parsed;
  EXPECT_TRUE(state_node_telemetry::deserialize_json(json, parsed));
  EXPECT_EQ(parsed.node_id, "node1");
  EXPECT_EQ(parsed.cluster_id, "alpha");
  EXPECT_EQ(parsed.timestamp_ns, 123456789u);
  EXPECT_EQ(parsed.mutations_published, 100u);
  EXPECT_EQ(parsed.mutations_applied, 95u);
  EXPECT_EQ(parsed.peer_count, 5u);
  EXPECT_EQ(parsed.slow_peer_count, 1u);
  EXPECT_NEAR(parsed.mutation_publish_rate, 42.5, 0.01);
  EXPECT_EQ(parsed.enqueue_p99, 8192u);
}

TEST(TelemetrySnapshotTest, DeserializeRejectsEmpty) {
  telemetry_snapshot snap;
  EXPECT_FALSE(state_node_telemetry::deserialize_json("", snap));
  EXPECT_FALSE(state_node_telemetry::deserialize_json("not json", snap));
}

// =====================================================================
// state_node_telemetry
// =====================================================================

TEST(StateNodeTelemetryTest, SampleWithShard) {
  app a;
  state_cluster_shard shard(a, "alpha", "node1");
  shard.attach();

  state_node_telemetry tel(a, "node1", "alpha");
  tel.attach_shard(&shard);

  tel.sample();
  auto snap = tel.snapshot();
  EXPECT_EQ(snap.node_id, "node1");
  EXPECT_EQ(snap.cluster_id, "alpha");
  EXPECT_GT(snap.timestamp_ns, 0u);
}

TEST(StateNodeTelemetryTest, SampleWithTransport) {
  app a;
  state_cluster_shard shard(a, "alpha", "node1");
  shard.attach();

  state_transport_inproc transport;
  transport.register_shard(&shard);

  state_node_telemetry tel(a, "node1", "alpha");
  tel.attach_shard(&shard);
  tel.attach_transport(&transport);

  tel.sample();
  auto snap = tel.snapshot();
  EXPECT_EQ(snap.peer_count, 0u); // 1 shard but no peers registered
}

TEST(StateNodeTelemetryTest, SampleWithBus) {
  app a;
  state_message_bus bus;

  state_node_telemetry tel(a, "node1", "alpha");
  tel.attach_message_bus(&bus);

  // Admit a message to bump the counter.
  auto msg = state_message::make_text("test.path", "hello");
  msg.origin_node_id = "remote";
  msg.message_id = "m1";
  bus.admit(msg);

  tel.sample();
  auto snap = tel.snapshot();
  EXPECT_EQ(snap.messages_admitted, 1u);
}

TEST(StateNodeTelemetryTest, RecordLatency) {
  app a;
  state_node_telemetry tel(a, "node1", "alpha");

  for (int i = 0; i < 100; ++i) {
    tel.record_enqueue_latency(5000);   // 5 µs
    tel.record_delivery_latency(20000); // 20 µs
  }

  tel.sample();
  auto snap = tel.snapshot();
  EXPECT_GT(snap.enqueue_p50, 0u);
  EXPECT_GT(snap.delivery_p50, 0u);
}

TEST(StateNodeTelemetryTest, RateComputation) {
  app a;
  state_cluster_shard shard(a, "alpha", "node1");
  shard.attach();

  state_transport_inproc transport;
  transport.register_shard(&shard);

  state_node_telemetry tel(a, "node1", "alpha");
  tel.attach_shard(&shard);
  tel.attach_transport(&transport);

  // First sample establishes baseline.
  tel.sample();
  auto snap1 = tel.snapshot();
  EXPECT_DOUBLE_EQ(snap1.mutation_publish_rate, 0.0);

  // Wait a tiny bit and sample again — rates should remain 0
  // since no mutations were published.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  tel.sample();
  auto snap2 = tel.snapshot();
  EXPECT_NEAR(snap2.mutation_publish_rate, 0.0, 1.0);
}

TEST(StateNodeTelemetryTest, PublishSnapshot) {
  app a;
  state_message_bus bus;

  state_node_telemetry tel(a, "node1", "alpha");
  tel.attach_message_bus(&bus);

  // Subscribe to catch the telemetry message.
  bool received = false;
  std::string received_json;
  bus.subscribe("__telemetry", [&](const state_message &msg) {
    received = true;
    received_json = msg.string_value;
  });

  tel.sample();
  EXPECT_TRUE(tel.publish_snapshot());
  EXPECT_TRUE(received);
  EXPECT_FALSE(received_json.empty());

  // Parse the received JSON.
  telemetry_snapshot parsed;
  EXPECT_TRUE(state_node_telemetry::deserialize_json(received_json, parsed));
  EXPECT_EQ(parsed.node_id, "node1");
  EXPECT_EQ(parsed.cluster_id, "alpha");
}

TEST(StateNodeTelemetryTest, PublishWithoutBusReturnsFalse) {
  app a;
  state_node_telemetry tel(a, "node1", "alpha");
  tel.sample();
  EXPECT_FALSE(tel.publish_snapshot());
}
