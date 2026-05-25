/*
  Copyright 2026 The University of Texas at Austin
  Tests for the distributed_state_session convenience API.
*/

#include <chrono>
#include <cvc/app.h>
#include <cvc/distributed_state_session.h>
#include <cvc/state.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>

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
