/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <chrono>
#include <cvc/app.h>
#include <cvc/distributed_state_session.h>
#include <cvc/state.h>
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
  auto result = session->wait_for_data("nonexistent.path",
                                        std::chrono::milliseconds(50));
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
