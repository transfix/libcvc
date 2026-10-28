/*
  state_exec_coordinator_test.cpp — Phase 6 integration tests
  Steps 28-33: coordinator election, submission, migration,
  observation, admin controls, and end-to-end scenarios.
*/

#include <chrono>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exec_coordinator.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/resource_policy.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/state_value_codec.h>
#include <cvc/state/state_message.h>
#include <cvc/state/state_message_bus.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::state_exec;
using cvc::state_message;
using cvc::state_message_bus;

// ===========================================================================
// Fixture: single-node coordinator
// ===========================================================================

class CoordinatorTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  state_message_bus bus;
  scheduler sched;
  exec_coordinator coord;

  void SetUp() override {
    coord.set_node_id("node-A");
    coord.set_cluster_id("cluster-1");
    coord.attach_scheduler(&sched);
    coord.attach_message_bus(&bus);
  }

  void TearDown() override {
    if (coord.is_running())
      coord.stop();
  }
};

// ===========================================================================
// Step 28: Election, heartbeat, handoff
// ===========================================================================

TEST_F(CoordinatorTest, StartRequiresBus) {
  exec_coordinator c;
  c.set_node_id("x");
  c.set_cluster_id("c");
  c.attach_scheduler(&sched);
  // No bus attached
  EXPECT_THROW(c.start(), std::runtime_error);
}

TEST_F(CoordinatorTest, StartRequiresScheduler) {
  exec_coordinator c;
  c.set_node_id("x");
  c.set_cluster_id("c");
  c.attach_message_bus(&bus);
  // No scheduler attached
  EXPECT_THROW(c.start(), std::runtime_error);
}

TEST_F(CoordinatorTest, StartStopLifecycle) {
  EXPECT_FALSE(coord.is_running());
  coord.start();
  EXPECT_TRUE(coord.is_running());
  coord.stop();
  EXPECT_FALSE(coord.is_running());
}

TEST_F(CoordinatorTest, DoubleStartNoop) {
  coord.start();
  coord.start(); // should not throw
  EXPECT_TRUE(coord.is_running());
}

TEST_F(CoordinatorTest, DoubleStopNoop) {
  coord.start();
  coord.stop();
  coord.stop(); // should not throw
  EXPECT_FALSE(coord.is_running());
}

TEST_F(CoordinatorTest, SingleNodeBecomesLeader) {
  // With no other nodes, election timeout should make us leader
  exec_coordinator::config cfg;
  cfg.election_timeout = std::chrono::milliseconds(50);
  coord.set_config(cfg);
  coord.start();

  // Simulate election timeout via heartbeat handler
  // The election starts on start(). Send ourselves a heartbeat
  // after the timeout to trigger the victory check.
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  coord.emit_heartbeat();

  // Now check — election timeout should have triggered victory
  // (but since we're the only node and no alive response came)
  // We need to trigger the heartbeat handler which checks the timeout
  auto hb = state_message::make_text("__state_exec.cluster-1.heartbeat",
                                     "{\"node_id\":\"dummy\",\"is_leader\":false,\"stats\":\"{}\"}",
                                     MIME_EXEC_HEARTBEAT);
  hb.cluster_id = "cluster-1";
  hb.origin_node_id = "dummy";
  coord.on_message(hb);

  EXPECT_TRUE(coord.is_leader());
  EXPECT_EQ(coord.leader_node_id(), "node-A");
}

TEST_F(CoordinatorTest, ElectionVictoryMessage) {
  coord.start();

  // Receive a victory message from another node
  auto msg = state_message::make_text("__state_exec.cluster-1.election",
                                      "{\"type\":\"election-victory\",\"node_id\":\"node-B\"}",
                                      MIME_EXEC_ELECTION);
  msg.cluster_id = "cluster-1";
  msg.origin_node_id = "node-B";

  coord.on_message(msg);

  EXPECT_FALSE(coord.is_leader());
  EXPECT_EQ(coord.leader_node_id(), "node-B");
}

TEST_F(CoordinatorTest, ElectionBullyHigherIdWins) {
  coord.start();

  // Send election-start from node-Z (higher ID than node-A)
  auto msg = state_message::make_text(
      "__state_exec.cluster-1.election",
      "{\"type\":\"election-start\",\"node_id\":\"node-Z\",\"priority\":0,\"timestamp\":0}",
      MIME_EXEC_ELECTION);
  msg.cluster_id = "cluster-1";
  msg.origin_node_id = "node-Z";

  // node-A should NOT send alive since node-Z has higher ID
  // (bully protocol: higher wins)
  coord.on_message(msg);
  // node-A should stop its election
  EXPECT_FALSE(coord.is_leader());
}

TEST_F(CoordinatorTest, ElectionBullyLowerIdLoses) {
  coord.start();

  // node-A should beat a node with lower ID
  // First, capture messages sent by coord
  std::vector<state_message> captured;
  bus.subscribe("", [&](const state_message &m) { captured.push_back(m); });

  auto msg = state_message::make_text(
      "__state_exec.cluster-1.election",
      "{\"type\":\"election-start\",\"node_id\":\"node-0\",\"priority\":0,\"timestamp\":0}",
      MIME_EXEC_ELECTION);
  msg.cluster_id = "cluster-1";
  msg.origin_node_id = "node-0";

  coord.on_message(msg);

  // node-A should have sent an alive message since A > 0
  bool found_alive = false;
  for (const auto &c : captured) {
    if (c.string_value.find("election-alive") != std::string::npos) {
      found_alive = true;
      break;
    }
  }
  EXPECT_TRUE(found_alive);
}

TEST_F(CoordinatorTest, ElectionPriorityOverridesId) {
  exec_coordinator::config cfg;
  cfg.election_priority = 10;
  coord.set_config(cfg);
  coord.start();

  // node-Z has higher ID but lower priority
  auto msg = state_message::make_text(
      "__state_exec.cluster-1.election",
      "{\"type\":\"election-start\",\"node_id\":\"node-Z\",\"priority\":5,\"timestamp\":0}",
      MIME_EXEC_ELECTION);
  msg.cluster_id = "cluster-1";
  msg.origin_node_id = "node-Z";

  std::vector<state_message> captured;
  bus.subscribe("", [&](const state_message &m) { captured.push_back(m); });

  coord.on_message(msg);

  // node-A (priority 10) should beat node-Z (priority 5)
  bool found_alive = false;
  for (const auto &c : captured) {
    if (c.string_value.find("election-alive") != std::string::npos) {
      found_alive = true;
      break;
    }
  }
  EXPECT_TRUE(found_alive);
}

TEST_F(CoordinatorTest, Handoff) {
  coord.start();
  // Make ourselves leader first
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);
  EXPECT_TRUE(coord.is_leader());

  // Handoff to node-B
  EXPECT_TRUE(coord.admin_handoff("node-B"));
  EXPECT_FALSE(coord.is_leader());
  EXPECT_EQ(coord.leader_node_id(), "node-B");
}

TEST_F(CoordinatorTest, HandoffRequiresLeader) {
  coord.start();
  EXPECT_FALSE(coord.admin_handoff("node-B"));
}

TEST_F(CoordinatorTest, MembershipDeadTriggersReelection) {
  coord.start();

  // Set leader to node-B
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-B\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);
  EXPECT_EQ(coord.leader_node_id(), "node-B");

  // Simulate node-B dying
  coord.on_membership_event(2, "node-B"); // 2 = dead

  // Election should have been started
  auto s = coord.stats();
  EXPECT_GE(s.elections_initiated, 1u);
}

TEST_F(CoordinatorTest, HeartbeatEmitAndReceive) {
  coord.start();

  // Emit heartbeat
  coord.emit_heartbeat();
  auto s = coord.stats();
  EXPECT_EQ(s.heartbeats_sent, 1u);

  // Receive heartbeat from another node
  auto hb =
      state_message::make_text("__state_exec.cluster-1.heartbeat",
                               "{\"node_id\":\"node-B\",\"is_leader\":false,\"stats\":\"{\\\"total_"
                               "processes\\\":3,\\\"running\\\":2,\\\"ready\\\":1,\\\"paused\\\":0,"
                               "\\\"terminated\\\":0,\\\"killed\\\":0,\\\"total_steps\\\":100}\"}",
                               MIME_EXEC_HEARTBEAT);
  hb.cluster_id = "cluster-1";
  hb.origin_node_id = "node-B";
  coord.on_message(hb);

  s = coord.stats();
  EXPECT_EQ(s.heartbeats_received, 1u);
}

TEST_F(CoordinatorTest, ElectionStats) {
  coord.start();
  coord.request_election();

  auto s = coord.stats();
  // start() triggers one election, request_election() triggers another
  EXPECT_GE(s.elections_initiated, 2u);
}

// ===========================================================================
// Step 29: Process submission
// ===========================================================================

TEST_F(CoordinatorTest, SubmitLocalAsLeader) {
  coord.start();
  // Become leader
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);
  EXPECT_TRUE(coord.is_leader());

  auto result = coord.submit("(+ 1 2)");
  EXPECT_TRUE(result.accepted);
  EXPECT_GE(result.pid, 1);
  EXPECT_EQ(result.node_id, "node-A");
}

TEST_F(CoordinatorTest, SubmitWithPolicyValidation) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  resource_policy policy;
  policy.max_time_max = 30.0;
  policy.max_time_default = 10.0;
  policy.enforce = resource_policy::mode::clamp;
  coord.set_resource_policy(policy);

  execute_options opts;
  opts.max_time = 999.0; // exceeds max
  auto result = coord.submit("(+ 1 2)", opts);
  EXPECT_TRUE(result.accepted);
  // Time was clamped to max_time_max by resource_policy
}

TEST_F(CoordinatorTest, SubmitProcessCountLimit) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  resource_policy policy;
  policy.max_processes = 2;
  coord.set_resource_policy(policy);

  auto r1 = coord.submit("(+ 1 2)");
  EXPECT_TRUE(r1.accepted);

  auto r2 = coord.submit("(+ 3 4)");
  EXPECT_TRUE(r2.accepted);

  auto r3 = coord.submit("(+ 5 6)");
  EXPECT_FALSE(r3.accepted);
  EXPECT_EQ(r3.error, "max_processes limit reached");
}

TEST_F(CoordinatorTest, SubmitForwardWhenNotLeader) {
  coord.start();
  // Set leader to node-B
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-B\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);
  EXPECT_FALSE(coord.is_leader());

  auto result = coord.submit("(+ 1 2)");
  // Should be forwarded (accepted but no local pid)
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.pid, -1);
  EXPECT_EQ(result.node_id, "node-B");
}

TEST_F(CoordinatorTest, SubmitNoScheduler) {
  exec_coordinator c;
  c.set_node_id("x");
  c.set_cluster_id("c");
  auto result = c.submit("(+ 1 2)");
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.error, "no scheduler attached");
}

TEST_F(CoordinatorTest, SubmitViaMessage) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  // Simulate a remote submission message
  auto msg = state_message::make_text(
      "__state_exec.cluster-1.submit",
      "{\"action\":\"submit\",\"script\":\"(+ 10 20)\",\"name\":\"remote-proc\","
      "\"priority\":5,\"uid\":\"user1\",\"gid\":\"group1\","
      "\"max_steps\":1000,\"max_time\":30.0,\"max_memory\":0,"
      "\"max_messages\":0,\"reply_node\":\"node-C\"}",
      MIME_EXEC_SUBMIT);
  msg.cluster_id = "cluster-1";
  msg.origin_node_id = "node-C";

  coord.on_message(msg);

  auto s = coord.stats();
  EXPECT_EQ(s.submissions_received, 1u);
  EXPECT_EQ(s.submissions_accepted, 1u);

  // Verify process was created
  EXPECT_EQ(sched.process_count(), 1);
}

TEST_F(CoordinatorTest, SubmitStats) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  coord.submit("(+ 1 2)");
  coord.submit("(+ 3 4)");

  auto s = coord.stats();
  EXPECT_EQ(s.submissions_received, 2u);
  EXPECT_EQ(s.submissions_accepted, 2u);
}

TEST_F(CoordinatorTest, SubmitExpr) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  // Direct expression submission
  auto expr = value_t(std::make_shared<std::vector<value_t>>(
      std::vector<value_t>{value_t(std::string("+")), value_t(int64_t(10)), value_t(int64_t(20))}));
  auto result = coord.submit(expr);
  EXPECT_TRUE(result.accepted);
  EXPECT_GE(result.pid, 1);
}

// ===========================================================================
// Step 30: Process migration
// ===========================================================================

TEST_F(CoordinatorTest, MigrateToSelfFails) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  coord.submit("(+ 1 2)");
  auto result = coord.migrate(1, "node-A");
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error, "cannot migrate to self");
}

TEST_F(CoordinatorTest, MigrateNonexistentProcess) {
  coord.start();
  auto result = coord.migrate(999, "node-B");
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error, "failed to pause process");
}

TEST_F(CoordinatorTest, MigrateNoScheduler) {
  exec_coordinator c;
  c.set_node_id("x");
  c.set_cluster_id("c");
  auto result = c.migrate(1, "node-B");
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error, "no scheduler");
}

TEST_F(CoordinatorTest, MigrateRunningProcess) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  // Submit and partially run a process
  auto sr = coord.submit("(begin (+ 1 2) (+ 3 4) (+ 5 6))");
  EXPECT_TRUE(sr.accepted);
  int pid = sr.pid;

  // Capture migration message
  std::vector<state_message> captured;
  bus.subscribe("__state_exec.cluster-1.migrate",
                [&](const state_message &m) { captured.push_back(m); });

  auto result = coord.migrate(pid, "node-B");
  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.target_node, "node-B");

  auto s = coord.stats();
  EXPECT_EQ(s.migrations_initiated, 1u);
}

TEST_F(CoordinatorTest, MigrateReceiveOnTarget) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  // Simulate receiving a migration request
  auto msg = state_message::make_text(
      "__state_exec.cluster-1.migrate.node-A",
      "{\"action\":\"migrate-request\",\"source_node\":\"node-B\","
      "\"source_pid\":42,"
      "\"process_data\":\"{\\\"name\\\":\\\"migrated-proc\\\","
      "\\\"priority\\\":3,\\\"uid\\\":\\\"user1\\\",\\\"gid\\\":\\\"g1\\\","
      "\\\"max_memory\\\":1024,\\\"max_time\\\":60.0,\\\"max_messages\\\":100}\"}",
      MIME_EXEC_MIGRATE);
  msg.cluster_id = "cluster-1";
  msg.origin_node_id = "node-B";

  coord.on_message(msg);

  // A new process should have been created
  EXPECT_GE(sched.process_count(), 1);

  auto s = coord.stats();
  EXPECT_EQ(s.migrations_completed, 1u);
}

TEST_F(CoordinatorTest, MigrateAckReceived) {
  coord.start();

  // Simulate receiving a migration ack
  auto msg = state_message::make_text("__state_exec.cluster-1.migrate.node-A",
                                      "{\"action\":\"migrate-ack\",\"source_pid\":1,\"new_pid\":5,"
                                      "\"target_node\":\"node-B\",\"success\":true}",
                                      MIME_EXEC_MIGRATE);
  msg.cluster_id = "cluster-1";
  msg.origin_node_id = "node-B";

  coord.on_message(msg);

  auto s = coord.stats();
  EXPECT_EQ(s.migrations_completed, 1u);
}

// ===========================================================================
// Step 31: Cross-cluster observation
// ===========================================================================

TEST_F(CoordinatorTest, PsAllLocalOnly) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  coord.submit("(+ 1 2)");
  coord.submit("(+ 3 4)");

  auto procs = coord.ps_all();
  EXPECT_EQ(procs.size(), 2u);
  for (const auto &p : procs) {
    EXPECT_EQ(p.node_id, "node-A");
  }
}

TEST_F(CoordinatorTest, PsAllWithRemote) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  coord.submit("(+ 1 2)");

  // Receive status broadcast from remote node
  auto status = state_message::make_text(
      "__state_exec.cluster-1.status",
      "{\"node_id\":\"node-B\","
      "\"processes\":\"[{\\\"pid\\\":1,\\\"name\\\":\\\"remote-proc\\\","
      "\\\"status\\\":\\\"running\\\",\\\"priority\\\":0,\\\"uid\\\":\\\"\\\","
      "\\\"gid\\\":\\\"\\\",\\\"step_count\\\":50,\\\"elapsed_time\\\":1.5,"
      "\\\"current_memory\\\":0,\\\"peak_memory\\\":0,\\\"max_memory\\\":0,"
      "\\\"max_time\\\":0,\\\"message_count\\\":0,\\\"max_messages\\\":0,"
      "\\\"parent_pid\\\":-1}]\","
      "\"stats\":\"{\\\"total_processes\\\":1,\\\"running\\\":1,\\\"ready\\\":0,"
      "\\\"paused\\\":0,\\\"terminated\\\":0,\\\"killed\\\":0,\\\"total_steps\\\":50}\"}",
      MIME_EXEC_STATUS);
  status.cluster_id = "cluster-1";
  status.origin_node_id = "node-B";
  coord.on_message(status);

  auto procs = coord.ps_all();
  EXPECT_EQ(procs.size(), 2u);

  bool found_local = false, found_remote = false;
  for (const auto &p : procs) {
    if (p.node_id == "node-A")
      found_local = true;
    if (p.node_id == "node-B")
      found_remote = true;
  }
  EXPECT_TRUE(found_local);
  EXPECT_TRUE(found_remote);
}

TEST_F(CoordinatorTest, ClusterStatsLocalOnly) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  coord.submit("(+ 1 2)");

  auto cs = coord.cluster_stats();
  EXPECT_EQ(cs.total_processes, 1);
  EXPECT_TRUE(cs.per_node.empty()); // no remote nodes
}

TEST_F(CoordinatorTest, ClusterStatsWithRemote) {
  coord.start();

  // Receive heartbeat from remote
  auto hb = state_message::make_text("__state_exec.cluster-1.heartbeat",
                                     "{\"node_id\":\"node-B\",\"is_leader\":false,"
                                     "\"stats\":\"{\\\"total_processes\\\":5,\\\"running\\\":3,"
                                     "\\\"ready\\\":2,\\\"paused\\\":0,\\\"terminated\\\":0,"
                                     "\\\"killed\\\":0,\\\"total_steps\\\":200}\"}",
                                     MIME_EXEC_HEARTBEAT);
  hb.cluster_id = "cluster-1";
  hb.origin_node_id = "node-B";
  coord.on_message(hb);

  auto cs = coord.cluster_stats();
  EXPECT_EQ(cs.per_node.size(), 1u);
  EXPECT_EQ(cs.per_node.at("node-B").total_processes, 5);
  EXPECT_EQ(cs.per_node.at("node-B").running, 3);
  EXPECT_EQ(cs.total_processes, 5); // local has 0, remote has 5
  EXPECT_EQ(cs.total_running, 3);
}

TEST_F(CoordinatorTest, StatusBroadcast) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  coord.submit("(+ 1 2)");
  coord.emit_status_broadcast();

  auto s = coord.stats();
  EXPECT_EQ(s.status_broadcasts, 1u);
}

// ===========================================================================
// Step 32: Admin controls
// ===========================================================================

TEST_F(CoordinatorTest, AdminPauseLocal) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  auto sr = coord.submit("(begin (+ 1 2) (+ 3 4))");
  int pid = sr.pid;

  EXPECT_TRUE(coord.admin_pause(pid));

  auto pi = sched.get_process_info(pid);
  ASSERT_TRUE(pi.has_value());
  EXPECT_EQ(pi->status, process_status::paused);
}

TEST_F(CoordinatorTest, AdminResumeLocal) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  auto sr = coord.submit("(begin (+ 1 2) (+ 3 4))");
  int pid = sr.pid;

  EXPECT_TRUE(coord.admin_pause(pid));
  EXPECT_TRUE(coord.admin_resume(pid));

  auto pi = sched.get_process_info(pid);
  ASSERT_TRUE(pi.has_value());
  EXPECT_EQ(pi->status, process_status::ready);
}

TEST_F(CoordinatorTest, AdminKillLocal) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  auto sr = coord.submit("(begin (+ 1 2) (+ 3 4))");
  int pid = sr.pid;

  EXPECT_TRUE(coord.admin_kill(pid));

  auto pi = sched.get_process_info(pid);
  ASSERT_TRUE(pi.has_value());
  EXPECT_EQ(pi->status, process_status::killed);
}

TEST_F(CoordinatorTest, AdminControlRemote) {
  coord.start();

  std::vector<state_message> captured;
  bus.subscribe("__state_exec.cluster-1.control",
                [&](const state_message &m) { captured.push_back(m); });

  // Send pause to remote node
  EXPECT_TRUE(coord.admin_pause(42, "node-B"));

  // Should have sent a control message
  bool found = false;
  for (const auto &m : captured) {
    if (m.string_value.find("\"pause\"") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);
}

TEST_F(CoordinatorTest, AdminSetPolicy) {
  coord.start();

  resource_policy policy;
  policy.max_processes = 10;
  policy.max_time_max = 60.0;
  policy.enforce = resource_policy::mode::strict;

  EXPECT_TRUE(coord.admin_set_policy(policy));

  auto s = coord.stats();
  EXPECT_EQ(s.admin_commands, 1u);
}

TEST_F(CoordinatorTest, ReceiveControlMessage) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  auto sr = coord.submit("(begin (+ 1 2) (+ 3 4))");
  int pid = sr.pid;

  // Simulate receiving a remote pause command
  auto msg = state_message::make_text("__state_exec.cluster-1.control.node-A",
                                      "{\"command\":\"pause\",\"pid\":" + std::to_string(pid) +
                                          ",\"from\":\"node-B\"}",
                                      MIME_EXEC_CONTROL);
  msg.cluster_id = "cluster-1";
  msg.origin_node_id = "node-B";
  coord.on_message(msg);

  auto pi = sched.get_process_info(pid);
  ASSERT_TRUE(pi.has_value());
  EXPECT_EQ(pi->status, process_status::paused);
}

TEST_F(CoordinatorTest, ReceivePolicyMessage) {
  coord.start();

  auto msg =
      state_message::make_text("__state_exec.cluster-1.policy",
                               "{\"from\":\"node-B\","
                               "\"policy\":\"{\\\"max_processes\\\":20,\\\"max_time_max\\\":120.0,"
                               "\\\"enforce\\\":0}\"}",
                               MIME_EXEC_POLICY);
  msg.cluster_id = "cluster-1";
  msg.origin_node_id = "node-B";
  coord.on_message(msg);

  // Policy was updated — verify by trying to submit with the new limits
  // (we can't directly read the policy, but it's applied on next submit)
}

TEST_F(CoordinatorTest, ReceiveKillCommand) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  auto sr = coord.submit("(begin (+ 1 2) (+ 3 4))");

  auto msg = state_message::make_text("__state_exec.cluster-1.control.node-A",
                                      "{\"command\":\"kill\",\"pid\":" + std::to_string(sr.pid) +
                                          ",\"from\":\"node-B\"}",
                                      MIME_EXEC_CONTROL);
  msg.cluster_id = "cluster-1";
  msg.origin_node_id = "node-B";
  coord.on_message(msg);

  auto pi = sched.get_process_info(sr.pid);
  ASSERT_TRUE(pi.has_value());
  EXPECT_EQ(pi->status, process_status::killed);
}

TEST_F(CoordinatorTest, AdminCommandStats) {
  coord.start();

  coord.admin_pause(1);
  coord.admin_resume(1);
  coord.admin_kill(1);

  auto s = coord.stats();
  EXPECT_EQ(s.admin_commands, 3u);
}

// ===========================================================================
// Step 33: End-to-end integration scenarios
// ===========================================================================

TEST_F(CoordinatorTest, E2E_SubmitRunAndObserve) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  // Submit multiple processes
  auto r1 = coord.submit("(+ 1 2)");
  auto r2 = coord.submit("(+ 3 4)");
  auto r3 = coord.submit("(+ 5 6)");
  EXPECT_TRUE(r1.accepted);
  EXPECT_TRUE(r2.accepted);
  EXPECT_TRUE(r3.accepted);

  // Run all processes to completion
  sched.run();

  // Observe cluster state
  auto procs = coord.ps_all();
  EXPECT_EQ(procs.size(), 3u);

  auto cs = coord.cluster_stats();
  EXPECT_EQ(cs.total_processes, 3);

  // Check results
  auto result1 = sched.get_result(r1.pid);
  ASSERT_TRUE(result1.has_value());
  EXPECT_EQ(std::get<int64_t>(result1->v), 3);
}

TEST_F(CoordinatorTest, E2E_SubmitWithResourceLimits) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  // Set strict policy
  resource_policy policy;
  policy.max_steps_max = 100;
  policy.max_steps_default = 50;
  policy.enforce = resource_policy::mode::clamp;
  coord.set_resource_policy(policy);

  execute_options opts;
  opts.max_steps = 999; // will be clamped to 100

  auto r = coord.submit("(+ 1 2)", opts);
  EXPECT_TRUE(r.accepted);

  // Run and verify it completed (simple expr, well within 100 steps)
  sched.run();
  auto result = sched.get_result(r.pid);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<int64_t>(result->v), 3);
}

TEST_F(CoordinatorTest, E2E_ForkAndObserve) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  auto r = coord.submit("(+ 10 20)");
  EXPECT_TRUE(r.accepted);

  // Fork the process
  int child_pid = sched.fork(r.pid);
  EXPECT_GT(child_pid, 0);

  auto procs = coord.ps_all();
  EXPECT_EQ(procs.size(), 2u);
}

TEST_F(CoordinatorTest, E2E_PauseResumeKill) {
  coord.start();
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord.on_message(victory);

  auto r = coord.submit("(begin (+ 1 2) (+ 3 4) (+ 5 6))");
  int pid = r.pid;

  // Pause
  EXPECT_TRUE(coord.admin_pause(pid));
  auto pi = sched.get_process_info(pid);
  EXPECT_EQ(pi->status, process_status::paused);

  // Resume
  EXPECT_TRUE(coord.admin_resume(pid));
  pi = sched.get_process_info(pid);
  EXPECT_EQ(pi->status, process_status::ready);

  // Kill
  EXPECT_TRUE(coord.admin_kill(pid));
  pi = sched.get_process_info(pid);
  EXPECT_EQ(pi->status, process_status::killed);
}

TEST_F(CoordinatorTest, E2E_MultiNodeHeartbeatAggregation) {
  coord.start();

  // Simulate heartbeats from 3 remote nodes
  for (int i = 1; i <= 3; ++i) {
    std::string nid = "node-" + std::to_string(i);
    std::ostringstream stats_json;
    stats_json << "{\\\"total_processes\\\":" << i * 2 << ",\\\"running\\\":" << i
               << ",\\\"ready\\\":" << i << ",\\\"paused\\\":0,\\\"terminated\\\":0"
               << ",\\\"killed\\\":0,\\\"total_steps\\\":" << i * 100 << "}";

    auto hb = state_message::make_text("__state_exec.cluster-1.heartbeat",
                                       "{\"node_id\":\"" + nid +
                                           "\",\"is_leader\":false,"
                                           "\"stats\":\"" +
                                           stats_json.str() + "\"}",
                                       MIME_EXEC_HEARTBEAT);
    hb.cluster_id = "cluster-1";
    hb.origin_node_id = nid;
    coord.on_message(hb);
  }

  auto cs = coord.cluster_stats();
  EXPECT_EQ(cs.per_node.size(), 3u);
  // Total: 2+4+6 = 12 processes, 1+2+3 = 6 running
  EXPECT_EQ(cs.total_processes, 12);
  EXPECT_EQ(cs.total_running, 6);
}

TEST_F(CoordinatorTest, E2E_DeadNodeCleanup) {
  coord.start();

  // Add remote node via heartbeat
  auto hb = state_message::make_text("__state_exec.cluster-1.heartbeat",
                                     "{\"node_id\":\"node-B\",\"is_leader\":false,"
                                     "\"stats\":\"{\\\"total_processes\\\":5}\"}",
                                     MIME_EXEC_HEARTBEAT);
  hb.cluster_id = "cluster-1";
  hb.origin_node_id = "node-B";
  coord.on_message(hb);

  auto cs = coord.cluster_stats();
  EXPECT_EQ(cs.per_node.size(), 1u);

  // Simulate node-B dying
  coord.on_membership_event(3, "node-B"); // 3 = evicted

  cs = coord.cluster_stats();
  EXPECT_EQ(cs.per_node.size(), 0u);
}

// ===========================================================================
// Serialization round-trip tests
// ===========================================================================

TEST_F(CoordinatorTest, SerializeSchedulerStats) {
  scheduler_stats s;
  s.total_processes = 10;
  s.running = 3;
  s.ready = 2;
  s.paused = 1;
  s.terminated = 3;
  s.killed = 1;
  s.total_steps = 5000;

  auto json = exec_coordinator::serialize_scheduler_stats(s);
  scheduler_stats s2{};
  EXPECT_TRUE(exec_coordinator::deserialize_scheduler_stats(json, s2));
  EXPECT_EQ(s2.total_processes, 10);
  EXPECT_EQ(s2.running, 3);
  EXPECT_EQ(s2.ready, 2);
  EXPECT_EQ(s2.paused, 1);
  EXPECT_EQ(s2.terminated, 3);
  EXPECT_EQ(s2.killed, 1);
  EXPECT_EQ(s2.total_steps, 5000u);
}

TEST_F(CoordinatorTest, SerializeProcessInfo) {
  process_info pi{};
  pi.pid = 42;
  pi.name = "test-proc";
  pi.status = process_status::running;
  pi.priority = 5;
  pi.uid = "user1";
  pi.gid = "group1";
  pi.step_count = 100;
  pi.elapsed_time = 2.5;
  pi.current_memory = 1024;
  pi.peak_memory = 2048;
  pi.max_memory = 4096;
  pi.max_time = 60.0;
  pi.message_count = 10;
  pi.max_messages = 100;
  pi.parent_pid = -1;

  auto json = exec_coordinator::serialize_process_info(pi);
  process_info pi2{};
  EXPECT_TRUE(exec_coordinator::deserialize_process_info(json, pi2));
  EXPECT_EQ(pi2.pid, 42);
  EXPECT_EQ(pi2.name, "test-proc");
  EXPECT_EQ(pi2.status, process_status::running);
  EXPECT_EQ(pi2.priority, 5);
  EXPECT_EQ(pi2.uid, "user1");
  EXPECT_EQ(pi2.gid, "group1");
  EXPECT_EQ(pi2.step_count, 100u);
  EXPECT_DOUBLE_EQ(pi2.elapsed_time, 2.5);
  EXPECT_EQ(pi2.current_memory, 1024u);
  EXPECT_EQ(pi2.peak_memory, 2048u);
  EXPECT_EQ(pi2.max_memory, 4096u);
  EXPECT_DOUBLE_EQ(pi2.max_time, 60.0);
  EXPECT_EQ(pi2.message_count, 10u);
  EXPECT_EQ(pi2.max_messages, 100u);
  EXPECT_EQ(pi2.parent_pid, -1);
}

TEST_F(CoordinatorTest, SerializeProcessList) {
  std::vector<process_info> procs;
  process_info p1{};
  p1.pid = 1;
  p1.name = "proc-1";
  p1.status = process_status::ready;
  procs.push_back(p1);

  process_info p2{};
  p2.pid = 2;
  p2.name = "proc-2";
  p2.status = process_status::running;
  procs.push_back(p2);

  auto json = exec_coordinator::serialize_process_list(procs);
  std::vector<process_info> procs2;
  EXPECT_TRUE(exec_coordinator::deserialize_process_list(json, procs2));
  EXPECT_EQ(procs2.size(), 2u);
  EXPECT_EQ(procs2[0].pid, 1);
  EXPECT_EQ(procs2[0].name, "proc-1");
  EXPECT_EQ(procs2[1].pid, 2);
  EXPECT_EQ(procs2[1].name, "proc-2");
}

TEST_F(CoordinatorTest, SerializeResourcePolicy) {
  resource_policy p;
  p.max_time_min = 1.0;
  p.max_time_max = 60.0;
  p.max_time_default = 10.0;
  p.max_memory_max = 1024 * 1024;
  p.max_processes = 50;
  p.enforce = resource_policy::mode::strict;

  auto json = exec_coordinator::serialize_resource_policy(p);
  resource_policy p2;
  EXPECT_TRUE(exec_coordinator::deserialize_resource_policy(json, p2));
  EXPECT_DOUBLE_EQ(p2.max_time_min, 1.0);
  EXPECT_DOUBLE_EQ(p2.max_time_max, 60.0);
  EXPECT_DOUBLE_EQ(p2.max_time_default, 10.0);
  EXPECT_EQ(p2.max_memory_max, 1024u * 1024u);
  EXPECT_EQ(p2.max_processes, 50);
  EXPECT_EQ(p2.enforce, resource_policy::mode::strict);
}

TEST_F(CoordinatorTest, SerializeEmptyList) {
  std::vector<process_info> empty;
  auto json = exec_coordinator::serialize_process_list(empty);
  EXPECT_EQ(json, "[]");

  std::vector<process_info> out;
  EXPECT_TRUE(exec_coordinator::deserialize_process_list(json, out));
  EXPECT_TRUE(out.empty());
}

TEST_F(CoordinatorTest, DeserializeInvalidJson) {
  scheduler_stats s{};
  EXPECT_FALSE(exec_coordinator::deserialize_scheduler_stats("", s));

  process_info pi{};
  EXPECT_FALSE(exec_coordinator::deserialize_process_info("", pi));

  std::vector<process_info> procs;
  EXPECT_FALSE(exec_coordinator::deserialize_process_list("", procs));
  EXPECT_FALSE(exec_coordinator::deserialize_process_list("not-json", procs));

  resource_policy p;
  EXPECT_FALSE(exec_coordinator::deserialize_resource_policy("", p));
}

// ===========================================================================
// Ignoring own messages
// ===========================================================================

TEST_F(CoordinatorTest, IgnoresOwnElection) {
  coord.start();

  auto msg = state_message::make_text(
      "__state_exec.cluster-1.election",
      "{\"type\":\"election-start\",\"node_id\":\"node-A\",\"priority\":0,\"timestamp\":0}",
      MIME_EXEC_ELECTION);
  msg.cluster_id = "cluster-1";
  msg.origin_node_id = "node-A";

  // Should not change leader
  coord.on_message(msg);
}

TEST_F(CoordinatorTest, IgnoresOwnHeartbeat) {
  coord.start();

  auto hb = state_message::make_text("__state_exec.cluster-1.heartbeat",
                                     "{\"node_id\":\"node-A\",\"is_leader\":true,\"stats\":\"{}\"}",
                                     MIME_EXEC_HEARTBEAT);
  hb.cluster_id = "cluster-1";
  hb.origin_node_id = "node-A";
  coord.on_message(hb);

  // Should not add self to remote stats
  auto cs = coord.cluster_stats();
  EXPECT_TRUE(cs.per_node.empty());
}

TEST_F(CoordinatorTest, IgnoresMessageWhenStopped) {
  // Don't start the coordinator
  auto msg = state_message::make_text("__state_exec.cluster-1.election",
                                      "{\"type\":\"election-victory\",\"node_id\":\"node-B\"}",
                                      MIME_EXEC_ELECTION);

  coord.on_message(msg);
  // Should not crash, and leader should not be set
  EXPECT_TRUE(coord.leader_node_id().empty());
}

// ===========================================================================
// Two-coordinator simulation
// ===========================================================================

class TwoCoordinatorTest : public ::testing::Test {
protected:
  cvc::app app1, app2;
  state_message_bus shared_bus;
  scheduler sched1, sched2;
  exec_coordinator coord1, coord2;

  void SetUp() override {
    coord1.set_node_id("node-A");
    coord1.set_cluster_id("cluster-1");
    coord1.attach_scheduler(&sched1);
    coord1.attach_message_bus(&shared_bus);

    coord2.set_node_id("node-B");
    coord2.set_cluster_id("cluster-1");
    coord2.attach_scheduler(&sched2);
    coord2.attach_message_bus(&shared_bus);
  }

  void TearDown() override {
    if (coord1.is_running())
      coord1.stop();
    if (coord2.is_running())
      coord2.stop();
  }
};

TEST_F(TwoCoordinatorTest, ElectionHigherIdWins) {
  exec_coordinator::config cfg;
  cfg.election_timeout = std::chrono::milliseconds(50);
  coord1.set_config(cfg);
  coord2.set_config(cfg);

  coord1.start();
  coord2.start();

  // Simulate node-B sending election victory (B > A in string order)
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-B\"}",
                                          MIME_EXEC_ELECTION);
  victory.cluster_id = "cluster-1";
  victory.origin_node_id = "node-B";

  coord1.on_message(victory);
  coord2.on_message(victory);

  EXPECT_FALSE(coord1.is_leader());
  EXPECT_TRUE(coord2.is_leader());
  EXPECT_EQ(coord1.leader_node_id(), "node-B");
  EXPECT_EQ(coord2.leader_node_id(), "node-B");
}

TEST_F(TwoCoordinatorTest, CrossNodeControlCommands) {
  coord1.start();
  coord2.start();

  // Make node-A leader
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord1.on_message(victory);
  coord2.on_message(victory);

  // Submit on node-A
  auto r = coord1.submit("(+ 1 2)");
  EXPECT_TRUE(r.accepted);

  // Node-B sends kill command to node-A via message
  auto ctrl = state_message::make_text("__state_exec.cluster-1.control.node-A",
                                       "{\"command\":\"kill\",\"pid\":" + std::to_string(r.pid) +
                                           ",\"from\":\"node-B\"}",
                                       MIME_EXEC_CONTROL);
  ctrl.cluster_id = "cluster-1";
  ctrl.origin_node_id = "node-B";
  coord1.on_message(ctrl);

  auto pi = sched1.get_process_info(r.pid);
  ASSERT_TRUE(pi.has_value());
  EXPECT_EQ(pi->status, process_status::killed);
}

TEST_F(TwoCoordinatorTest, HandoffBetweenNodes) {
  coord1.start();
  coord2.start();

  // Make node-A leader
  auto victory = state_message::make_text("__state_exec.cluster-1.election",
                                          "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
                                          MIME_EXEC_ELECTION);
  coord1.on_message(victory);
  coord2.on_message(victory);
  EXPECT_TRUE(coord1.is_leader());
  EXPECT_FALSE(coord2.is_leader());

  // Handoff from A to B
  EXPECT_TRUE(coord1.admin_handoff("node-B"));

  // Node-A receives the handoff victory message via bus
  // (in real system, the bus distributes it)
  auto handoff_victory = state_message::make_text(
      "__state_exec.cluster-1.election", "{\"type\":\"election-victory\",\"node_id\":\"node-B\"}",
      MIME_EXEC_ELECTION);
  coord2.on_message(handoff_victory);

  EXPECT_FALSE(coord1.is_leader());
  EXPECT_TRUE(coord2.is_leader());
}

// ===========================================================================
// Priorities (docs/STATE_EXEC_LOCKS.md 7.2, 7.4; locks plan L2.6)
// ===========================================================================

class Priority : public CoordinatorTest {
protected:
  void lead() {
    coord.start();
    auto victory = state_message::make_text(
        "__state_exec.cluster-1.election", "{\"type\":\"election-victory\",\"node_id\":\"node-A\"}",
        MIME_EXEC_ELECTION);
    coord.on_message(victory);
  }

  // A process of many steps (builtins only: the coordinator's processes get no environment).
  int submit_long(execute_options opts = {}) {
    const submit_result r =
        coord.submit("(begin (set i 0) (while (< i 100000) (set i (+ i 1))) i)", opts);
    EXPECT_TRUE(r.accepted) << r.error;
    return r.pid;
  }

  int base_of(int pid) { return sched.get_priority(pid).value().base; }
  int floor_of(int pid) { return sched.get_priority(pid).value().floor; }

  int pid_named(const std::string &name) {
    for (const process_info &pi : sched.list_processes())
      if (pi.name == name)
        return pi.pid;
    return -1;
  }

  // A migration request from node-B, as migrate() sends it: process_data is the metadata JSON.
  void receive_migration(const std::string &name, int priority, int floor) {
    const std::string data = "{\\\"name\\\":\\\"" + name +
                             "\\\",\\\"priority\\\":" + std::to_string(priority) +
                             ",\\\"priority_floor\\\":" + std::to_string(floor) + "}";
    auto msg = state_message::make_text(
        "__state_exec.cluster-1.migrate.node-A",
        "{\"action\":\"migrate-request\",\"source_node\":\"node-B\",\"source_pid\":42,"
        "\"process_data\":\"" +
            data + "\"}",
        MIME_EXEC_MIGRATE);
    msg.cluster_id = "cluster-1";
    msg.origin_node_id = "node-B";
    coord.on_message(msg);
  }
};

// PR-8, the coordinator's control: set_priority from a control message (a bus thread) or from
// admin_set_priority is posted to the scheduler (post_control) and applied between its steps, as
// the host's change (unchecked); remotely it is a control message.
TEST_F(Priority, PR08_CoordinatorSetPriority) {
  lead();
  const int pid = submit_long();
  ASSERT_GE(pid, 0);
  sched.step();

  // From another thread, as a bus thread delivers it: posted, applied at the next step.
  std::thread bus_thread([&] {
    auto msg =
        state_message::make_text("__state_exec.cluster-1.control.node-A",
                                 "{\"command\":\"set_priority\",\"pid\":" + std::to_string(pid) +
                                     ",\"priority\":7,\"from\":\"node-B\"}",
                                 MIME_EXEC_CONTROL);
    msg.cluster_id = "cluster-1";
    msg.origin_node_id = "node-B";
    coord.on_message(msg);
  });
  bus_thread.join();
  EXPECT_EQ(base_of(pid), 0); // not on the bus thread
  sched.step();
  EXPECT_EQ(base_of(pid), 7);
  EXPECT_EQ(sched.get_process_info(pid)->priority, 7);
  EXPECT_EQ(floor_of(pid), 0); // a host's change is unchecked, and leaves the floor

  // A control without a priority is ignored.
  auto bare = state_message::make_text(
      "__state_exec.cluster-1.control.node-A",
      "{\"command\":\"set_priority\",\"pid\":" + std::to_string(pid) + ",\"from\":\"node-B\"}",
      MIME_EXEC_CONTROL);
  bare.cluster_id = "cluster-1";
  bare.origin_node_id = "node-B";
  coord.on_message(bare);
  sched.step();
  EXPECT_EQ(base_of(pid), 7);

  // admin_set_priority on this node (empty or its own id): the same post.
  EXPECT_TRUE(coord.admin_set_priority(pid, "", 3));
  EXPECT_EQ(base_of(pid), 7);
  sched.step();
  EXPECT_EQ(base_of(pid), 3);
  EXPECT_TRUE(coord.admin_set_priority(pid, "node-A", -2));
  sched.step();
  EXPECT_EQ(base_of(pid), -2);
  // An unknown pid is found, and ignored, when the scheduler applies it.
  EXPECT_TRUE(coord.admin_set_priority(9999, "", 1));
  EXPECT_EQ(sched.step(), 1);

  // Another node: a control message naming the command, the pid and the priority.
  std::vector<state_message> captured;
  bus.subscribe("__state_exec.cluster-1.control",
                [&](const state_message &m) { captured.push_back(m); });
  EXPECT_TRUE(coord.admin_set_priority(pid, "node-B", 5));
  ASSERT_EQ(captured.size(), 1u);
  EXPECT_EQ(captured[0].path, "__state_exec.cluster-1.control.node-B");
  EXPECT_NE(captured[0].string_value.find("\"command\":\"set_priority\""), std::string::npos);
  EXPECT_NE(captured[0].string_value.find("\"priority\":5"), std::string::npos);
  EXPECT_EQ(coord.stats().admin_commands, 4u);

  // No scheduler: nothing to post to.
  exec_coordinator lone;
  EXPECT_FALSE(lone.admin_set_priority(pid, "", 1));
}

// PR-8 across two nodes: node-A's admin_set_priority reaches node-B's scheduler over the bus.
TEST_F(TwoCoordinatorTest, PR08_SetPriorityOnAnotherNode) {
  coord1.start();
  coord2.start();
  const int pid =
      sched2.execute(std::string("(begin (set i 0) (while (< i 1000) (set i (+ i 1))))"));
  EXPECT_TRUE(coord1.admin_set_priority(pid, "node-B", 6));
  EXPECT_EQ(sched2.get_priority(pid)->base, 0);
  sched2.step();
  EXPECT_EQ(sched2.get_priority(pid)->base, 6);
}

// PR-11: the floor goes with the base in the migration metadata, and the target applies its own
// resource_policy::priority_floor over it: a target with floor 3 that receives base 5 floor 0
// keeps base 5, at floor 3; base 1 is clamped to 3, and counted (the count is the record:
// state_exec has no log of its own, gap G-5). Local submits, forwarded submits and the policy's
// broadcast carry the floor too.
TEST_F(Priority, PR11_FloorInMetadata) {
  lead();

  // The source writes the floor beside the base.
  execute_options src_opts;
  src_opts.name = "src";
  src_opts.priority = 4;
  src_opts.priority_floor = 2;
  const int src = submit_long(src_opts);
  EXPECT_EQ(floor_of(src), 2);
  std::vector<state_message> sent;
  bus.subscribe("__state_exec.cluster-1.migrate.node-B",
                [&](const state_message &m) { sent.push_back(m); });
  ASSERT_TRUE(coord.migrate(src, "node-B").success);
  ASSERT_EQ(sent.size(), 1u);
  EXPECT_NE(sent[0].string_value.find("\\\"priority\\\":4"), std::string::npos)
      << sent[0].string_value;
  EXPECT_NE(sent[0].string_value.find("\\\"priority_floor\\\":2"), std::string::npos)
      << sent[0].string_value;

  // The target, with a floor of 3.
  resource_policy policy;
  policy.priority_floor = 3;
  coord.set_resource_policy(policy);
  receive_migration("m5", 5, 0);
  const int m5 = pid_named("m5");
  ASSERT_GE(m5, 0);
  EXPECT_EQ(base_of(m5), 5);
  EXPECT_EQ(floor_of(m5), 3);
  EXPECT_EQ(coord.stats().priority_clamped, 0u);
  receive_migration("m1", 1, 0);
  const int m1 = pid_named("m1");
  ASSERT_GE(m1, 0);
  EXPECT_EQ(base_of(m1), 3);
  EXPECT_EQ(floor_of(m1), 3);
  EXPECT_EQ(coord.stats().priority_clamped, 1u);
  // A stricter floor the process came with stays.
  receive_migration("m6", 6, 4);
  EXPECT_EQ(floor_of(pid_named("m6")), 4);
  EXPECT_EQ(coord.stats().priority_clamped, 1u);

  // A local submit is held to the policy's floor the same way.
  execute_options better;
  better.priority = -5;
  const int clamped = submit_long(better);
  EXPECT_EQ(base_of(clamped), 3);
  EXPECT_EQ(floor_of(clamped), 3);
  EXPECT_EQ(coord.stats().priority_clamped, 2u);
  const int plain = submit_long();
  EXPECT_EQ(base_of(plain), 3);
  EXPECT_EQ(coord.stats().priority_clamped, 3u);

  // A forwarded submit carries the submitter's floor: node-A forwards once node-B leads.
  auto other = state_message::make_text("__state_exec.cluster-1.election",
                                        "{\"type\":\"election-victory\",\"node_id\":\"node-B\"}",
                                        MIME_EXEC_ELECTION);
  coord.on_message(other);
  std::vector<state_message> forwarded;
  bus.subscribe("__state_exec.cluster-1.submit",
                [&](const state_message &m) { forwarded.push_back(m); });
  execute_options fwd;
  fwd.priority = 2;
  fwd.priority_floor = 1;
  coord.submit("1", fwd);
  ASSERT_FALSE(forwarded.empty());
  EXPECT_NE(forwarded[0].string_value.find("\"priority_floor\":1"), std::string::npos);

  // The policy's floor crosses the bus with the rest of it; a policy without one has none.
  resource_policy back;
  ASSERT_TRUE(exec_coordinator::deserialize_resource_policy(
      exec_coordinator::serialize_resource_policy(policy), back));
  EXPECT_EQ(back.priority_floor, std::optional<int>(3));
  resource_policy none;
  EXPECT_EQ(exec_coordinator::serialize_resource_policy(none).find("priority_floor"),
            std::string::npos);
  resource_policy none_back;
  ASSERT_TRUE(exec_coordinator::deserialize_resource_policy(
      exec_coordinator::serialize_resource_policy(none), none_back));
  EXPECT_FALSE(none_back.priority_floor.has_value());
}
