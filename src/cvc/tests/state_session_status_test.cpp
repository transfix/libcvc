/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <chrono>
#include <cvc/core/app.h>
#include <cvc/core/distributed_state_session.h>
#include <cvc/core/state.h>
#include <cvc/core/state_distributed_metrics.h>
#include <gtest/gtest.h>
#include <thread>

using cvc::app;
using cvc::distributed_state_config;
using cvc::distributed_state_session;
using cvc::replica_status;
using cvc::transport_kind;

class StateSessionStatusTest : public ::testing::Test {
protected:
  app ctx;

  std::shared_ptr<distributed_state_session> make_session() {
    distributed_state_config config;
    config.cluster_id = "test_cluster";
    config.node_id = "node_1";
    config.transport = transport_kind::inproc;
    config.pump_interval_ms = 5;
    return distributed_state_session::join(ctx, config);
  }
};

TEST_F(StateSessionStatusTest, StatusReportsRunning) {
  auto session = make_session();
  auto s = session->status();
  EXPECT_TRUE(s.running);
  session->stop();
  auto s2 = session->status();
  EXPECT_FALSE(s2.running);
}

TEST_F(StateSessionStatusTest, StatusReportsPumpCycles) {
  auto session = make_session();
  // Wait for at least one pump cycle.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  auto s = session->status();
  EXPECT_GT(s.pump_cycles, 0u);
  session->stop();
}

TEST_F(StateSessionStatusTest, StatusReportsLocalSequence) {
  auto session = make_session();
  // First set creates the node (lost per adapter doc), second set
  // generates a journaled mutation.
  cvc::state::instance(ctx)("test.key").value(std::string("v1"));
  cvc::state::instance(ctx)("test.key").value(std::string("v2"));
  // Force a pump cycle to ensure drain_local has run.
  session->transport().pump_all();
  auto s = session->status();
  EXPECT_GT(s.local_sequence, 0u);
  session->stop();
}

TEST_F(StateSessionStatusTest, SessionStopIsIdempotent) {
  auto session = make_session();
  session->stop();
  session->stop(); // should not crash
  EXPECT_FALSE(session->is_running());
}

TEST_F(StateSessionStatusTest, WaitForDataTimeoutReturns) {
  auto session = make_session();
  // Wait for a non-existent path — should return after timeout.
  auto result = session->wait_for_data("nonexistent.path", std::chrono::milliseconds(50));
  // We just verify it returns without hanging. The status value
  // depends on whether the hydrator has seen this path.
  (void)result;
  session->stop();
}

TEST_F(StateSessionStatusTest, ClusterAndNodeIdAccessors) {
  auto session = make_session();
  EXPECT_EQ(session->cluster_id(), "test_cluster");
  EXPECT_EQ(session->node_id(), "node_1");
  session->stop();
}

TEST_F(StateSessionStatusTest, ShardAndTransportAccessible) {
  auto session = make_session();
  // These should not throw or crash.
  auto &shard = session->shard();
  EXPECT_EQ(shard.cluster_id(), "test_cluster");
  auto &transport = session->transport();
  (void)transport;
  session->stop();
}

TEST_F(StateSessionStatusTest, ConflictAutoPublish) {
  // Create a session with resolve_conflicts enabled.
  distributed_state_config config;
  config.cluster_id = "test_cluster";
  config.node_id = "node_A";
  config.transport = transport_kind::inproc;
  config.pump_interval_ms = 1; // fast pump for test
  config.resolve_conflicts = true;
  auto session = distributed_state_session::join(ctx, config);

  // Inject two conflicting mutations. Inject winner (higher
  // lexicographic node_id) first, then the loser. The ring buffer
  // records the loser arrival.
  cvc::state_mutation m1;
  m1.cluster_id = "test_cluster";
  m1.origin_node_id = "node_Z";
  m1.sequence = 10;
  m1.path = "conflict.key";
  m1.string_value = "winner";
  m1.type_name = "std::string";
  m1.op = cvc::state_mutation_op::set_value;
  session->shard().ingest_remote(m1);

  cvc::state_mutation m2;
  m2.cluster_id = "test_cluster";
  m2.origin_node_id = "node_B";
  m2.sequence = 10;
  m2.path = "conflict.key";
  m2.string_value = "loser";
  m2.type_name = "std::string";
  m2.op = cvc::state_mutation_op::set_value;
  session->shard().ingest_remote(m2);

  EXPECT_GT(session->shard().total_conflicts_detected(), 0u);

  // Wait for the pump's periodic conflict publish (every 100 pump cycles).
  //
  // Poll the published value itself, NOT the pump_cycles proxy. The old wait
  // was `pump_cycles >= 110` within 50 x 20ms, on the premise recorded in its
  // comment: "At 1ms interval, 100 cycles ~= 100ms". That premise is wrong.
  // pump_interval_ms is a sleep FLOOR, not a rate, and Windows' ~15.6ms
  // scheduler tick stretches sleep_for(1ms) to a full tick -- so 100 cycles
  // takes ~1.55s there, and sleep_for(20ms) costs two ticks, capping the loop
  // at ~1.56s. The two deadlines land within one tick of each other, so the
  // loop would exhaust, the value would be read one tick early, and the test
  // failed on an auto-vivified empty node roughly one Windows run in ten.
  // Linux exits here in ~130ms and macOS in ~1.0s, which is why only the
  // windows-2022 lane ever saw it.
  //
  // Polling the value is correct on any timer granularity: it leaves as soon
  // as the publish lands, so the common case is no slower than before.
  const std::string prefix = "__system.distributed.test_cluster.conflicts.recent.0.path";
  std::string val;
  for (int i = 0; i < 400; ++i) {
    val = cvc::state::instance(ctx)(prefix).value();
    if (!val.empty())
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Fatal, and it reports the pump counter: a future timeout should say
  // "never published, pump_cycles=N" rather than the misleading
  // `"" != "conflict.key"` that sent this one to the wrong suspect.
  ASSERT_FALSE(val.empty()) << "conflict metrics never auto-published; pump_cycles="
                            << session->status().pump_cycles;
  EXPECT_EQ(val, "conflict.key");

  session->stop();
}
